`timescale 1ns/1ps
`default_nettype none
// FSM encodings of rnf_dcache (in a package so a testbench can name them)
package rnf_dcache_pkg;

  typedef enum logic [4:0] {
    R_IDLE,       // MSHR free
    R_LOOKUP,     // tags of the set are valid: hit / miss / victim
    R_LOAD,       // load data is valid
    R_TAG_RD,     // read the tags (first lookup, or again after a wait: the set may have changed)
    R_REQ,        // send the request flit
    R_WAIT,       // wait for the first response
    R_RETRY,      // RetryAck seen: wait for the PCrdGrant, then resend
    R_FILL_TAG,   // CompData is there: read the tags to pick the way to fill
    R_FILL_SEL,
    R_FILL,       // take the four CompData beats
    R_FILL_DONE,  // write tag + state
    R_WB_TAG,     // DBID is there: snapshot the line being written back
    R_WB_CMP,
    R_WB_RD,
    R_WDAT,       // send write data
    R_WCOMP,      // wait for Comp of a write
    R_ACK,        // send CompAck
    R_RESP,       // answer the core
    R_DEP         // wait until every older MSHR on the same set has finished
  } rstate_e;

  typedef enum logic [2:0] { T_READ, T_EVICT, T_WB, T_NCRD, T_NCWR } txn_e;

  typedef enum logic [2:0] {
    S_IDLE,
    S_WANT,       // wants the RAM port
    S_CMP,        // tags valid: decide, update the state
    S_DATA,       // copy the line into s_q.buf
    S_SEND_DAT,   // SnpRespData
    S_SEND_RSP    // SnpResp
  } sstate_e;

endpackage : rnf_dcache_pkg

// =============================================================================
// rnf_dcache - CHI RN-F data cache with NUM_MSHR miss status holding registers
//
//  Core side : rnf_core_if. Requests go into a 2-entry FIFO (ready = a flop),
//              answers come from one output register (rsp_q), by idx, in any
//              order. No request address reaches a RAM in its accept cycle and
//              no RAM output reaches the core in the cycle it is read (timing).
//  CHI side  : REQ / RSP / DAT transmit, SNP / RSP / DAT receive.
//
//  Cache     : SETS x WAYS lines of 64 bytes. Tags + CHI line states (I / SC /
//              UC / UD / SD) in flops; data in one dual-port RAM per way.
//              Replacement uses an Invalid way first, otherwise one round-robin
//              pointer per set.
//
//  Hit pipeline (port A, see "hit pipeline"): one request issued per cycle
//              from the FIFO; tags + the request's beat of every way are read,
//              P2 compares and answers a hit into rsp_q (3 cycles after
//              accept), a store hit writes its bytes (UC -> UD) on port A.
//              Everything else is handed to an MSHR.
//
//  MSHRs     : misses, upgrades (store on a Shared line), non-cacheable
//              accesses and requests to a set with an open MSHR. An MSHR runs
//              the per-request FSM (r-states): lookup, victim Evict /
//              WriteBackFull, the read, RetryAck / PCrdGrant, fill, re-lookup,
//              CompAck, response (through rsp_q when P2 does not use it).
//              The MSHRs share
//                * port B of the tags / data (with the snoop FSM),
//                * the REQ, RSP (CompAck) and DAT (write data) TX channels,
//                * the PCrdGrant counters.
//              The TxnID of an MSHR's transaction is {sequence, MSHR index},
//              so responses on RSP / DAT are routed by TxnID.
//
//  Ordering  : per set, requests are answered in order: the pipeline never
//              issues next to an older request to the same set, and a request
//              to a set with an open MSHR becomes an MSHR waiting in R_DEP until
//              the older ones finish (two MSHRs never race for a set's victim).
//              Requests to other sets overtake freely (hit under miss, miss
//              under miss). Non-cacheable requests follow the same rule.
//
//  RAM port B: one agent at a time. An agent that read the tags / data keeps
//              the port while it acts on what it read (r-states LOOKUP,
//              FILL_SEL, WB_CMP, WB_RD; snoop S_CMP, S_DATA). Otherwise the
//              snoop FSM goes first (but not on a set the pipeline holds), then
//              the MSHRs in rotating order. The pipeline does not issue on the
//              set the snoop FSM is working on.
//
//  Requests  : load miss            ReadShared
//              store miss / store on a Shared line   ReadUnique
//              replace a clean line Evict
//              replace a dirty line WriteBackFull
//              non-cacheable        ReadNoSnp / WriteNoSnpPtl
//              RetryAck is followed by a resend once the PCrdGrant is there.
//
//  Snoops    : served by their own FSM at any time. Policy: a snoop that
//              allows it leaves a Shared Clean copy and passes dirty data on
//              (SnpRespData_SC_PD); SnpOnce keeps the state; SnpUnique /
//              SnpCleanInvalid / SnpMakeInvalid go to I.
// =============================================================================
module rnf_dcache
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
  import rnf_pkg::*;
  import rnf_dcache_pkg::*;
#(
  parameter node_id_e NODEID         = 7'h01,
  parameter node_id_e HNID           = 7'h40,
  parameter int       AddrSize       = 39,
  parameter int       MaxAccessWidth = 64,
  parameter int       ReqIdxWidth    = 4,
  parameter int       SETS           = 16,
  parameter int       WAYS           = 4,
  parameter int       RX_FIFO_DEPTH  = 4,
  parameter int       TX_CREDIT_MAX  = 15,
  parameter int       NUM_MSHR       = 4
) (
  input wire clk,
  input wire resetn,

  rnf_core_if.cache core,

  chi_req.tx req_tx,
  chi_snp.rx snp_rx,
  chi_rsp.rx rsp_rx,
  chi_rsp.tx rsp_tx,
  chi_dat.rx dat_rx,
  chi_dat.tx dat_tx
);

  localparam int SET_W  = $clog2(SETS);
  localparam int WAY_W  = $clog2(WAYS);
  localparam int TAG_W  = AddrSize - 6 - SET_W;
  localparam int REQ_W  = $bits(local_req_flit_t);
  localparam int RSP_W  = $bits(local_rsp_flit_t);
  localparam int DAT_W  = $bits(local_dat_flit_t);
  localparam int SNP_W  = $bits(local_snp_flit_t);
  localparam int MW     = (NUM_MSHR > 1) ? $clog2(NUM_MSHR) : 1;   // MSHR index bits

  initial begin
    if (AddrSize > TSHR_ADDR_WIDTH)
      $fatal(1, "rnf_dcache: AddrSize (%0d) is wider than the CHI address (%0d)", AddrSize, TSHR_ADDR_WIDTH);
    if (MaxAccessWidth > 128 || (MaxAccessWidth % 8) != 0)
      $fatal(1, "rnf_dcache: MaxAccessWidth (%0d) must be a multiple of 8 and at most 128", MaxAccessWidth);
    if (RX_FIFO_DEPTH < 2 || RX_FIFO_DEPTH > 15)
      $fatal(1, "rnf_dcache: RX_FIFO_DEPTH (%0d) must be 2..15 (one link credit per entry)", RX_FIFO_DEPTH);
    if ((1 << SET_W) != SETS || (1 << WAY_W) != WAYS || SETS < 2 || WAYS < 2)
      $fatal(1, "rnf_dcache: SETS (%0d) and WAYS (%0d) must be powers of two, at least 2", SETS, WAYS);
    if (NUM_MSHR < 1 || NUM_MSHR > 16)
      $fatal(1, "rnf_dcache: NUM_MSHR (%0d) must be 1..16", NUM_MSHR);
  end

  // ------------------------------------------------------------------ link layer
  logic            req_v, req_rdy;
  local_req_flit_t req_f;
  logic            rsp_v, rsp_rdy;
  local_rsp_flit_t rsp_f;
  logic            dat_v, dat_rdy;
  local_dat_flit_t dat_f;

  logic             req_flitv, req_flitpend;
  logic [REQ_W-1:0] req_flit;
  logic             rsp_flitv, rsp_flitpend;
  logic [RSP_W-1:0] rsp_flit;
  logic             dat_flitv, dat_flitpend;
  logic [DAT_W-1:0] dat_flit;

  rnf_tx_port #(.W(REQ_W), .MaxCredits(TX_CREDIT_MAX)) u_req_tx (
    .clk(clk), .resetn(resetn), .valid_i(req_v), .flit_i(req_f), .ready_o(req_rdy),
    .lcrdv_i(req_tx.lcrdv), .flitv_o(req_flitv), .flitpend_o(req_flitpend), .flit_o(req_flit));
  rnf_tx_port #(.W(RSP_W), .MaxCredits(TX_CREDIT_MAX)) u_rsp_tx (
    .clk(clk), .resetn(resetn), .valid_i(rsp_v), .flit_i(rsp_f), .ready_o(rsp_rdy),
    .lcrdv_i(rsp_tx.lcrdv), .flitv_o(rsp_flitv), .flitpend_o(rsp_flitpend), .flit_o(rsp_flit));
  rnf_tx_port #(.W(DAT_W), .MaxCredits(TX_CREDIT_MAX)) u_dat_tx (
    .clk(clk), .resetn(resetn), .valid_i(dat_v), .flit_i(dat_f), .ready_o(dat_rdy),
    .lcrdv_i(dat_tx.lcrdv), .flitv_o(dat_flitv), .flitpend_o(dat_flitpend), .flit_o(dat_flit));

  assign req_tx.flitv    = req_flitv    ? Y : N;
  assign req_tx.flitpend = req_flitpend ? Y : N;
  assign req_tx.flit     = req_flit;
  assign rsp_tx.flitv    = rsp_flitv    ? Y : N;
  assign rsp_tx.flitpend = rsp_flitpend ? Y : N;
  assign rsp_tx.flit     = rsp_flit;
  assign dat_tx.flitv    = dat_flitv    ? Y : N;
  assign dat_tx.flitpend = dat_flitpend ? Y : N;
  assign dat_tx.flit     = dat_flit;

  logic             rxr_v, rxr_pop;
  logic [RSP_W-1:0] rxr_bits;
  local_rsp_flit_t  rxr;
  logic             rxd_v, rxd_pop;
  logic [DAT_W-1:0] rxd_bits;
  local_dat_flit_t  rxd;
  logic             rxs_v, rxs_pop;
  logic [SNP_W-1:0] rxs_bits;
  local_snp_flit_t  rxs;

  rnf_rx_fifo #(.W(RSP_W), .DEPTH(RX_FIFO_DEPTH)) u_rsp_rx (
    .clk(clk), .resetn(resetn), .flitv_i(rsp_rx.flitv == Y), .flit_i(rsp_rx.flit),
    .lcrdv_o(rsp_rx.lcrdv), .valid_o(rxr_v), .flit_o(rxr_bits), .pop_i(rxr_pop));
  rnf_rx_fifo #(.W(DAT_W), .DEPTH(RX_FIFO_DEPTH)) u_dat_rx (
    .clk(clk), .resetn(resetn), .flitv_i(dat_rx.flitv == Y), .flit_i(dat_rx.flit),
    .lcrdv_o(dat_rx.lcrdv), .valid_o(rxd_v), .flit_o(rxd_bits), .pop_i(rxd_pop));
  rnf_rx_fifo #(.W(SNP_W), .DEPTH(RX_FIFO_DEPTH)) u_snp_rx (
    .clk(clk), .resetn(resetn), .flitv_i(snp_rx.flitv == Y), .flit_i(snp_rx.flit),
    .lcrdv_o(snp_rx.lcrdv), .valid_o(rxs_v), .flit_o(rxs_bits), .pop_i(rxs_pop));

  assign rxr = local_rsp_flit_t'(rxr_bits);
  assign rxd = local_dat_flit_t'(rxd_bits);
  assign rxs = local_snp_flit_t'(rxs_bits);

  // ------------------------------------------------------------------ RAMs
  // the MSHRs (r_*) and the snoop FSM (s_*) drive the one port; the port
  // arbiter below makes sure only one agent uses it in a cycle
  logic                 r_tag_re, r_tag_we, s_tag_re, s_tag_we;
  logic [SET_W-1:0]     r_tag_set, s_tag_set;
  logic [WAY_W-1:0]     r_tag_way, s_tag_way;
  logic [TAG_W-1:0]     r_tag_tag, s_tag_tag;
  cache_state_e         r_tag_st,  s_tag_st;

  logic                 r_dat_re, r_dat_we, s_dat_re;
  logic [SET_W-1:0]     r_dat_set, s_dat_set;
  logic [WAY_W-1:0]     r_dat_way, s_dat_way;
  logic [1:0]           r_dat_beat, s_dat_beat;
  logic [15:0]          r_dat_be;
  logic [127:0]         r_dat_wdata;

  wire s_port = s_tag_re || s_tag_we || s_dat_re;

  wire                  tag_re  = s_port ? s_tag_re  : r_tag_re;
  wire                  tag_we  = s_port ? s_tag_we  : r_tag_we;
  wire [SET_W-1:0]      tag_set = s_port ? s_tag_set : r_tag_set;
  wire [WAY_W-1:0]      tag_way = s_port ? s_tag_way : r_tag_way;
  wire [TAG_W-1:0]      tag_tag = s_port ? s_tag_tag : r_tag_tag;
  cache_state_e         tag_st;
  assign tag_st = s_port ? s_tag_st : r_tag_st;

  logic [WAYS-1:0][TAG_W-1:0] rd_tag;
  cache_state_e [WAYS-1:0]    rd_st;

  // port A of the tags / data: the hit pipeline (see "hit pipeline" below)
  logic                       pa_tag_re;
  logic [SET_W-1:0]           pa_set;
  logic [WAYS-1:0][TAG_W-1:0] pa_tag;          // P2: the tags of the P1 set
  cache_state_e [WAYS-1:0]    pa_st;
  logic                       pa_st_we;        // P2: store hit on UC -> UD
  logic [SET_W-1:0]           pa_st_set;
  logic [WAY_W-1:0]           pa_st_way;
  logic                       pa_dat_re;       // P1: the request's beat of every way
  logic [1:0]                 pa_beat;
  logic [WAYS-1:0]            pa_dat_we;       // P2: store hit write (hit way)
  logic [SET_W-1:0]           pa_wset;
  logic [1:0]                 pa_wbeat;
  logic [15:0]                pa_be;
  logic [127:0]               pa_wdata;
  logic [WAYS-1:0][127:0]     pa_rdata;        // P2: the beat of every way

  rnf_tag_ram #(.SETS(SETS), .WAYS(WAYS), .TAG_W(TAG_W)) u_tag_ram (
    .clk(clk), .resetn(resetn), .set_i(tag_set), .re_i(tag_re), .we_i(tag_we),
    .way_i(tag_way), .tag_i(tag_tag), .state_i(tag_st), .tag_o(rd_tag), .state_o(rd_st),
    .set_a_i(pa_set), .re_a_i(pa_tag_re), .tag_a_o(pa_tag), .state_a_o(pa_st),
    .we_a_i(pa_st_we), .wset_a_i(pa_st_set), .way_a_i(pa_st_way), .state_a_i(CS_UD));

  // data: one dual-port RAM per way. Port B (MSHRs, snoop FSM) addresses one
  // way; its read data is the way read last (same view as the old single RAM)
  wire                         ram_re   = s_port ? s_dat_re : r_dat_re;
  wire                         ram_we   = s_port ? 1'b0     : r_dat_we;
  wire [WAY_W-1:0]             ram_way  = s_port ? s_dat_way : r_dat_way;
  wire [SET_W+2-1:0]           ram_addr = s_port ? {s_dat_set, s_dat_beat}
                                                 : {r_dat_set, r_dat_beat};
  logic [WAYS-1:0][127:0]      ram_rdata_w;
  logic [WAY_W-1:0]            ram_rway_q;     // way of the last port B read
  logic [127:0]                ram_rdata;
  assign ram_rdata = ram_rdata_w[ram_rway_q];

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn)     ram_rway_q <= '0;
    else if (ram_re) ram_rway_q <= ram_way;
  end

  for (genvar w = 0; w < WAYS; w++) begin : gen_way_ram
    rnf_data_ram #(.DEPTH(SETS * 4)) u_data_ram (
      .clk       (clk),
      .a_addr_i  (pa_dat_we[w] ? {pa_wset, pa_wbeat} : {pa_set, pa_beat}),
      .a_re_i    (pa_dat_re),
      .a_we_i    (pa_dat_we[w]),
      .a_be_i    (pa_be),
      .a_wdata_i (pa_wdata),
      .a_rdata_o (pa_rdata[w]),
      .b_addr_i  (ram_addr),
      .b_re_i    (ram_re && ram_way == WAY_W'(w)),
      .b_we_i    (ram_we && ram_way == WAY_W'(w)),
      .b_be_i    (r_dat_be),
      .b_wdata_i (r_dat_wdata),
      .b_rdata_o (ram_rdata_w[w]));
  end

  // ------------------------------------------------------------------ MSHR state
  typedef logic [NUM_MSHR-1:0] mvec_t;

  typedef struct packed {
    rstate_e                    state;
    txn_e                       txn;
    // core request
    mem_op_e                    op;
    logic [AddrSize-1:0]        addr;
    width_e                     width;
    logic [MaxAccessWidth-1:0]  data;
    logic [ReqIdxWidth-1:0]     idx;
    mem_unit_e                  src;
    logic                       cach;        // cacheable
    mvec_t                      dep;         // older MSHRs on the same set still open
    logic                       first;       // first lookup of this request (perf counters)
    // line being filled / replaced
    logic [WAY_W-1:0]           way;
    logic [TAG_W-1:0]           vtag;        // tag of the line being evicted / written back
    logic                       keep;        // fill: the line is still here, keep its data
    cache_state_e               old_st;
    logic [3:0]                 seen;        // CompData beats taken
    logic [1:0]                 cnt;
    // transaction
    logic [11:0]                txnid;
    logic                       allow_retry;
    logic [3:0]                 pcrd_type;
    logic                       got_retry, got_comp, got_dbid;
    logic [11:0]                wdbid;       // DBID of a write
    logic                       werr;
    logic                       rd_act;      // CompData of this TxnID is expected
    logic [2:0]                 cresp;       // CompData: Resp, DBID, HomeNID, error
    logic [11:0]                cdbid;
    node_id_e                   chome;
    logic                       cerr;
    logic                       ack_pend;    // CompAck still to send
    // result
    logic                       acc_done;    // the core access is done
    logic                       err;
    logic [MaxAccessWidth-1:0]  rdata;
    cache_state_e               rst;
    // write-back snapshot
    logic [2:0]                 wb_resp;
    logic                       wb_vld;
    logic [511:0]               wb_buf;
  } rreg_t;

  rreg_t m_q [NUM_MSHR];
  rreg_t m_d [NUM_MSHR];

  logic [WAY_W-1:0] rr_q [SETS];             // round-robin victim pointer per set
  logic             rr_inc;
  logic [SET_W-1:0] rr_set;
  logic [3:0]       pcrd_q [16];             // PCrdGrants not used yet, per PCrdType
  logic             pcrd_inc, pcrd_dec;
  logic [3:0]       pcrd_dec_type;
  logic [MW-1:0]    rot_q;                   // rotating MSHR priority (port, TX channels, core response)

  // ------------------------------------------------------------------ snoop FSM state
  typedef struct packed {
    sstate_e              state;
    logic [11:0]          txnid;
    node_id_e             srcid;
    snp_opcode_e          opcode;
    logic [AddrSize-1:0]  addr;
    logic                 rts;
    logic [WAY_W-1:0]     way;
    logic [2:0]           resp;
    logic [1:0]           cnt;
    logic [511:0]         buf_;
  } sreg_t;

  sreg_t s_q, s_d;

  // ------------------------------------------------------------------ helpers
  function automatic logic [15:0] be_of(width_e w, logic [3:0] off);
    logic [15:0] m;
    unique case (w)
      WIDTH_1: m = 16'h0001;
      WIDTH_2: m = 16'h0003;
      WIDTH_4: m = 16'h000F;
      WIDTH_8: m = 16'h00FF;
      default: m = 16'hFFFF;
    endcase
    return m << off;
  endfunction

  // bytes [off +: width] of a beat, right-aligned and zero-extended
  function automatic logic [MaxAccessWidth-1:0] load_of(logic [127:0] beat, width_e w, logic [3:0] off);
    logic [127:0] sh, m;
    sh = beat >> {off, 3'b000};
    unique case (w)
      WIDTH_1: m = 128'hFF;
      WIDTH_2: m = 128'hFFFF;
      WIDTH_4: m = 128'hFFFF_FFFF;
      WIDTH_8: m = 128'hFFFF_FFFF_FFFF_FFFF;
      default: m = '1;
    endcase
    sh = sh & m;
    return sh[MaxAccessWidth-1:0];
  endfunction

  function automatic logic [2:0] st2resp(cache_state_e s);   // CopyBackWrData Resp
    unique case (s)
      CS_UD:   return RESP_UD_PD;
      CS_SD:   return RESP_SD_PD;
      CS_UC:   return RESP_UC;
      CS_SC:   return RESP_SC;
      default: return RESP_I;
    endcase
  endfunction

  function automatic snp_opcode_e snp_base(snp_opcode_e op);  // forwarding snoops are answered as plain ones
    unique case (op)
      SNP_FWD_SHARED:           return SNP_SHARED;
      SNP_FWD_CLEAN:            return SNP_CLEAN;
      SNP_FWD_ONCE:             return SNP_ONCE;
      SNP_FWD_NOT_SHARED_DIRTY: return SNP_NOT_SHARED_DIRTY;
      SNP_FWD_UNIQUE:           return SNP_UNIQUE;
      default:                  return op;
    endcase
  endfunction

  function automatic logic [SET_W-1:0] set_of(logic [AddrSize-1:0] a);
    return a[6 +: SET_W];
  endfunction

  function automatic logic [TAG_W-1:0] tag_of(logic [AddrSize-1:0] a);
    return a[AddrSize-1 -: TAG_W];
  endfunction

  // lookup of a tag on the tag RAM output (valid in the cycle after a tag read)
  typedef struct packed {
    logic             hit;
    logic [WAY_W-1:0] hway;
    cache_state_e     hst;
    logic             inv;
    logic [WAY_W-1:0] iway;
  } lk_t;

  function automatic lk_t lookup(logic [TAG_W-1:0] t);
    lk_t r;
    r = '0;
    r.hst = CS_I;
    for (int w = WAYS - 1; w >= 0; w--) begin
      if (rd_st[w] != CS_I && rd_tag[w] == t) begin
        r.hit = 1'b1; r.hway = WAY_W'(w); r.hst = rd_st[w];
      end
      if (rd_st[w] == CS_I) begin
        r.inv = 1'b1; r.iway = WAY_W'(w);
      end
    end
    return r;
  endfunction

  // state a filled line gets
  function automatic cache_state_e fill_state(rreg_t e);
    cache_state_e s;
    unique case (e.cresp)
      RESP_UC:    s = CS_UC;
      RESP_UD_PD: s = CS_UD;
      RESP_SD_PD: s = CS_SD;
      default:    s = CS_SC;
    endcase
    // the line stayed here Shared Dirty: its own data is kept and it is still dirty
    if (e.keep && e.old_st == CS_SD) s = (e.op == MEM_OP_STORE) ? CS_UD : CS_SD;
    return s;
  endfunction

  // start a CHI transaction for an MSHR (TxnID = {sequence, MSHR index})
  function automatic rreg_t start_txn(rreg_t e, txn_e kind, int i);
    rreg_t n;
    n             = e;
    n.state       = R_REQ;
    n.txn         = kind;
    if (NUM_MSHR == 1) n.txnid = e.txnid + 12'd1;
    else               n.txnid = {e.txnid[11:MW] + 1'b1, MW'(i)};
    n.allow_retry = 1'b1;
    n.pcrd_type   = '0;
    n.got_retry   = 1'b0;
    n.got_comp    = 1'b0;
    n.got_dbid    = 1'b0;
    n.werr        = 1'b0;
    n.cerr        = 1'b0;
    n.rd_act      = (kind == T_READ) || (kind == T_NCRD);
    return n;
  endfunction

  function automatic local_req_flit_t mk_req(rreg_t e);
    local_req_flit_t f;
    f            = '0;
    f.tgtid      = HNID;
    f.srcid      = NODEID;
    f.txnid      = e.txnid;
    f.allowretry = e.allow_retry ? Y : N;
    f.pcrdttype  = e.pcrd_type;
    f.size       = WIDTH_64;
    f.memattr    = MEMATTR_NORMAL_WB;
    f.addr       = TSHR_ADDR_WIDTH'({tag_of(e.addr), set_of(e.addr), 6'b0});
    unique case (e.txn)
      T_READ: begin
        f.opcode     = (e.op == MEM_OP_STORE) ? REQ_READ_UNIQUE : REQ_READ_SHARED;
        f.expcompack = 1'b1;
      end
      T_EVICT: begin
        f.opcode = REQ_EVICT;
        f.addr   = TSHR_ADDR_WIDTH'({e.vtag, set_of(e.addr), 6'b0});
      end
      T_WB: begin
        f.opcode = REQ_WRITE_BACK_FULL;
        f.addr   = TSHR_ADDR_WIDTH'({e.vtag, set_of(e.addr), 6'b0});
      end
      T_NCRD: begin
        f.opcode  = REQ_READ_NO_SNOOP;
        f.addr    = TSHR_ADDR_WIDTH'(e.addr);
        f.size    = e.width;
        f.memattr = MEMATTR_NORMAL_NC;
      end
      default: begin
        f.opcode  = REQ_WRITE_NO_SNOOP_PTL;
        f.addr    = TSHR_ADDR_WIDTH'(e.addr);
        f.size    = e.width;
        f.memattr = MEMATTR_NORMAL_NC;
      end
    endcase
    return f;
  endfunction

  function automatic local_dat_flit_t mk_wdat(rreg_t e);
    local_dat_flit_t f;
    f         = '0;
    f.tgtid   = HNID;
    f.srcid   = NODEID;
    f.txnid   = e.wdbid;
    f.dbid    = e.wdbid;
    f.homenid = HNID;
    if (e.txn == T_WB) begin
      f.opcode = DAT_COPY_BACK_WR_DATA;
      f.resp   = e.wb_resp;
      f.dataid = e.cnt;
      f.be     = e.wb_vld ? '1 : '0;
      f.data   = e.wb_vld ? e.wb_buf[e.cnt*128 +: 128] : '0;
    end else begin
      f.opcode = DAT_NON_COPY_BACK_WR_DATA;
      f.resp   = RESP_I;
      f.dataid = e.addr[5:4];
      f.be     = be_of(e.width, e.addr[3:0]);
      f.data   = 128'(e.data) << {e.addr[3:0], 3'b000};
    end
    return f;
  endfunction

  function automatic local_rsp_flit_t mk_ack(rreg_t e);
    local_rsp_flit_t f;
    f        = '0;
    f.tgtid  = e.chome;
    f.srcid  = NODEID;
    f.txnid  = e.cdbid;
    f.opcode = RSP_COMP_ACK;
    return f;
  endfunction

  function automatic logic [MW-1:0] rot_idx(logic [MW-1:0] base, int k);
    int v;
    v = (int'(base) + k) % NUM_MSHR;
    return MW'(v);
  endfunction

  // ------------------------------------------------------------------ snoop lookup
  wire [SET_W-1:0] sn_set = s_q.addr[6 +: SET_W];
  wire [TAG_W-1:0] sn_tag = s_q.addr[AddrSize-1 -: TAG_W];

  logic             s_hit;
  logic [WAY_W-1:0] s_hit_way;
  cache_state_e     s_hit_st;
  always_comb begin
    s_hit = 1'b0; s_hit_way = '0; s_hit_st = CS_I;
    for (int w = WAYS - 1; w >= 0; w--)
      if (rd_st[w] != CS_I && rd_tag[w] == sn_tag) begin
        s_hit = 1'b1; s_hit_way = WAY_W'(w); s_hit_st = rd_st[w];
      end
  end

  // ------------------------------------------------------------------ hit pipeline
  // (timing) Every core request goes through a 2-entry FIFO, so no request
  // address reaches a RAM in its accept cycle, and every response leaves
  // through one output register (rsp_q), so no RAM output reaches the core
  // in the same cycle. core.ready comes from the FIFO count (a flop).
  //
  //   accept : request -> FIFO
  //   issue  : the FIFO head reads its set's tags and its beat of every way
  //            on port A (registered); the request moves into P2
  //   P2     : compare, way select, lane extract. Then one of:
  //              hit             -> rsp_q (store: write the bytes on port A,
  //                                 UC -> UD on the tag state)
  //              miss / upgrade  -> a free MSHR, from R_TAG_RD: it does the
  //                                 old flow on port B (lookup again, victim,
  //                                 CHI read, fill, CompAck, answer)
  //              non-cacheable   -> a free MSHR, starts ReadNoSnp / WriteNoSnpPtl
  //              set has an open MSHR -> a free MSHR in R_DEP (per-set order)
  //   rsp_q  : the core sees the answer 3 cycles after accept for a hit;
  //            MSHR answers use rsp_q when P2 does not.
  // One request can be issued per cycle. An issue waits when:
  //   - P2 holds a request to the same set (order, and P2 may write it)
  //   - P2 holds a store (port A is written in P2)
  //   - the snoop FSM is working on the head's set (S_WANT .. S_DATA)
  //   - fewer MSHRs are free than P2 might need (P2 never waits for one)
  // and a snoop does not start on a set the pipeline holds (see the arbiter).
  typedef struct packed {
    mem_op_e                    op;
    logic [AddrSize-1:0]        addr;
    width_e                     width;
    logic [MaxAccessWidth-1:0]  data;
    logic [ReqIdxWidth-1:0]     idx;
    mem_unit_e                  src;
    logic                       cach;
  } preq_t;

  preq_t        fq_mem [2];
  logic         fq_rd_q, fq_wr_q;
  logic [1:0]   fq_cnt_q;
  preq_t        head;
  logic         head_v, accept_w, issue_w;
  preq_t        p2_q;
  logic         p2_v_q;

  assign head     = fq_mem[fq_rd_q];
  assign head_v   = (fq_cnt_q != 2'd0);
  assign accept_w = (core.req.valid == Y) && (fq_cnt_q != 2'd2);

  // MSHRs: how many are free now (registered states)
  logic [$clog2(NUM_MSHR+1)-1:0] free_cnt;
  always_comb begin
    free_cnt = '0;
    for (int k = 0; k < NUM_MSHR; k++) free_cnt += (m_q[k].state == R_IDLE);
  end

  logic snp_on_head_set;
  assign snp_on_head_set = (s_q.state == S_WANT || s_q.state == S_CMP || s_q.state == S_DATA) &&
                           (sn_set == set_of(head.addr));

  assign issue_w = head_v &&
                   !(p2_v_q && (set_of(p2_q.addr) == set_of(head.addr) || p2_q.op == MEM_OP_STORE)) &&
                   !snp_on_head_set &&
                   (free_cnt > ($bits(free_cnt))'(p2_v_q));

  logic pipe_on_snp_set;
  assign pipe_on_snp_set = (p2_v_q  && set_of(p2_q.addr) == sn_set) ||
                           (issue_w && set_of(head.addr) == sn_set);

  // port A reads in the issue cycle (address from the FIFO, a flop)
  assign pa_tag_re = issue_w;
  assign pa_dat_re = issue_w && head.cach;
  assign pa_set    = set_of(head.addr);
  assign pa_beat   = head.addr[5:4];

  // ---- P2 decision
  lk_t               p2_lk;
  logic              p2_store, p2_dep, p2_hit;
  logic              p2_rsp_v;            // P2 answers the core (uses rsp_q this cycle)
  logic              p2_alloc;            // P2 hands the request to a free MSHR
  logic [MaxAccessWidth-1:0] p2_rdata;
  mvec_t             p2_dep_vec;

  always_comb begin
    p2_store = (p2_q.op == MEM_OP_STORE);
    p2_lk    = '0;
    p2_lk.hst = CS_I;
    for (int w = WAYS - 1; w >= 0; w--) begin
      if (pa_st[w] != CS_I && pa_tag[w] == tag_of(p2_q.addr)) begin
        p2_lk.hit = 1'b1; p2_lk.hway = WAY_W'(w); p2_lk.hst = pa_st[w];
      end
    end
    for (int k = 0; k < NUM_MSHR; k++)
      p2_dep_vec[k] = (m_q[k].state != R_IDLE) && (set_of(m_q[k].addr) == set_of(p2_q.addr));
    p2_dep   = (p2_dep_vec != '0);
    p2_hit   = p2_v_q && p2_q.cach && !p2_dep && p2_lk.hit &&
               (!p2_store || p2_lk.hst == CS_UC || p2_lk.hst == CS_UD);
    p2_rsp_v = p2_hit;
    p2_alloc = p2_v_q && !p2_hit;
    p2_rdata = load_of(pa_rdata[p2_lk.hway], p2_q.width, p2_q.addr[3:0]);

    // store hit: the bytes on port A, UC -> UD
    pa_dat_we = '0;
    pa_wset   = set_of(p2_q.addr);
    pa_wbeat  = p2_q.addr[5:4];
    pa_be     = be_of(p2_q.width, p2_q.addr[3:0]);
    pa_wdata  = 128'(p2_q.data) << {p2_q.addr[3:0], 3'b000};
    pa_st_we  = 1'b0;
    pa_st_set = set_of(p2_q.addr);
    pa_st_way = p2_lk.hway;
    if (p2_hit && p2_store) begin
      pa_dat_we[p2_lk.hway] = 1'b1;
      pa_st_we              = (p2_lk.hst == CS_UC);
    end
  end

  // ---- response register (the only thing the core sees)
  typedef struct packed {
    logic                       valid;
    mem_op_e                    op;
    logic [MaxAccessWidth-1:0]  data;
    logic [ReqIdxWidth-1:0]     idx;
    mem_unit_e                  src;
    logic                       err;
    cache_state_e               st;
  } prsp_t;
  prsp_t rsp_q, rsp_d;

  // ------------------------------------------------------------------ RAM port arbiter
  // CompData at the head of the DAT FIFO, per MSHR
  mvec_t rxd_mine;
  always_comb
    for (int i = 0; i < NUM_MSHR; i++)
      rxd_mine[i] = rxd_v && m_q[i].rd_act && (rxd.txnid == m_q[i].txnid) && (rxd.opcode == DAT_COMP_DATA);

  function automatic logic is_atomic(rstate_e s);
    return (s == R_LOOKUP) || (s == R_FILL_SEL) || (s == R_WB_CMP) || (s == R_WB_RD);
  endfunction

  function automatic logic wants_port(rreg_t e, logic mine);
    return (e.state == R_TAG_RD) || (e.state == R_FILL_TAG) || (e.state == R_WB_TAG) ||
           (e.state == R_FILL_DONE) || (e.state == R_FILL && mine && !e.keep);
  endfunction

  // Write port (the tag-state and data arrays are LUTRAM: one write port each).
  // Port B writes either in the act cycle after a grant (LOOKUP, WB_CMP, snoop
  // S_CMP) or right away (FILL beat, FILL_DONE tag). A store hit writes in P2.
  // They never meet: no port B sequence starts in the cycle a store issues
  // (its act cycle is that store's P2), and no immediate port B write happens
  // while a store is in P2. Both conditions come from flops.
  logic st_issue, p2_store_v;
  assign st_issue   = issue_w && (head.op == MEM_OP_STORE);
  assign p2_store_v = p2_v_q && (p2_q.op == MEM_OP_STORE);

  function automatic logic wport_ok(rstate_e s);
    if (st_issue   && (s == R_TAG_RD || s == R_WB_TAG))   return 1'b0;
    if (p2_store_v && (s == R_FILL   || s == R_FILL_DONE)) return 1'b0;
    return 1'b1;
  endfunction

  mvec_t e_port;          // MSHR i owns the port this cycle
  logic  s_grant;         // snoop FSM owns the port this cycle
  logic  a_grant;         // a new request may do its first tag read in its accept cycle
  logic  port_lock;

  always_comb begin
    logic found;
    logic [MW-1:0] i;
    e_port    = '0;
    s_grant   = 1'b0;
    a_grant   = 1'b0;
    port_lock = 1'b0;
    found     = 1'b0;
    i         = '0;
    // an agent acting on what it read keeps the port
    if (s_q.state == S_CMP || s_q.state == S_DATA) begin
      s_grant   = 1'b1;
      port_lock = 1'b1;
    end
    for (int k = 0; k < NUM_MSHR; k++)
      if (is_atomic(m_q[k].state)) begin
        e_port[k] = 1'b1;
        port_lock = 1'b1;
      end
    if (!port_lock) begin
      // a snoop only starts on a set the hit pipeline is not working on (the
      // pipeline read that set's tags / data and may still write it in P2),
      // and not when its act cycle would meet a store in P2 (write port)
      if (s_q.state == S_WANT && !pipe_on_snp_set && !st_issue) begin
        s_grant = 1'b1;
      end else begin
        for (int k = 0; k < NUM_MSHR; k++) begin
          i = rot_idx(rot_q, k);
          if (!found && wants_port(m_q[i], rxd_mine[i]) && wport_ok(m_q[i].state)) begin
            e_port[i] = 1'b1;
            found     = 1'b1;
          end
        end
        a_grant = !found;
      end
    end
  end

  // ------------------------------------------------------------------ perf / debug
  logic perf_hit_m, perf_miss, perf_evict, perf_snp, perf_snp_hit, perf_snp_data;
  logic [1:0] perf_hit;                      // hits this cycle: an MSHR lookup + the hit pipeline (P2)
  assign perf_hit = 2'(perf_hit_m) + 2'(p2_hit);
  logic [4:0] perf_wb, perf_retry;           // can happen in several MSHRs in one cycle

  // the MSHR doing a lookup this cycle and what it found (for testbench coverage)
  logic             own_v;
  logic [MW-1:0]    own_i;
  lk_t              own_lk;
  logic [WAY_W-1:0] own_rr_way;
  always_comb begin
    own_v = 1'b0; own_i = '0;
    for (int k = 0; k < NUM_MSHR; k++)
      if (e_port[k] && is_atomic(m_q[k].state)) begin own_v = 1'b1; own_i = MW'(k); end
    own_lk     = lookup(tag_of(m_q[own_i].addr));
    own_rr_way = rr_q[set_of(m_q[own_i].addr)];
  end

  // ------------------------------------------------------------------ MSHR FSMs
  logic            r_ack_v;                 // CompAck wants the RSP channel
  logic            r_dat_v;                 // write data wants the DAT channel
  local_req_flit_t req_c;
  local_rsp_flit_t ack_c;
  local_dat_flit_t wdat_c;
  logic            s_rsp_v, s_dat_v;
  local_rsp_flit_t srsp_c;
  local_dat_flit_t sdat_c;

  logic                       c_ready, c_rsp_v, c_rsp_err;
  logic [MaxAccessWidth-1:0]  c_rsp_data;
  cache_state_e               c_rsp_st;
  mem_op_e                    c_rsp_op;
  logic [ReqIdxWidth-1:0]     c_rsp_idx;
  mem_unit_e                  c_rsp_src;

  mvec_t active, freed;

  always_comb begin
    logic             req_taken, dat_taken, ack_taken, rsp_taken, pcrd_taken;
    logic [MW-1:0]    i;
    rreg_t            e, n;
    lk_t              lk;
    logic [SET_W-1:0] q_set;
    logic [TAG_W-1:0] q_tag;
    logic [1:0]       q_beat;
    logic [3:0]       q_off;
    logic             q_store;
    logic [WAY_W-1:0] rrw;
    logic             free_found;
    logic [MW-1:0]    free_i;
    mvec_t            dep_now;

    for (int k = 0; k < NUM_MSHR; k++) m_d[k] = m_q[k];
    rr_inc        = 1'b0;
    rr_set        = '0;
    pcrd_inc      = 1'b0;
    pcrd_dec      = 1'b0;
    pcrd_dec_type = '0;

    c_ready    = 1'b0;
    c_rsp_v    = 1'b0;
    c_rsp_data = '0;
    c_rsp_err  = 1'b0;
    c_rsp_st   = CS_I;
    c_rsp_op   = MEM_OP_LOAD;
    c_rsp_idx  = '0;
    c_rsp_src  = MEM_UNIT_LOAD;

    r_tag_re    = 1'b0;
    r_tag_we    = 1'b0;
    r_tag_set   = '0;
    r_tag_way   = '0;
    r_tag_tag   = '0;
    r_tag_st    = CS_I;
    r_dat_re    = 1'b0;
    r_dat_we    = 1'b0;
    r_dat_set   = '0;
    r_dat_way   = '0;
    r_dat_beat  = '0;
    r_dat_be    = '1;
    r_dat_wdata = rxd.data;

    req_v   = 1'b0;
    r_ack_v = 1'b0;
    r_dat_v = 1'b0;
    req_c   = '0;
    ack_c   = '0;
    wdat_c  = '0;
    rxd_pop = rxd_v && !(|rxd_mine);           // a DAT flit nobody waits for is dropped

    perf_hit_m = 1'b0; perf_miss = 1'b0; perf_evict = 1'b0; perf_wb = '0; perf_retry = '0;

    // ---- responses on the RSP channel are taken as they arrive
    rxr_pop = rxr_v;
    if (rxr_v && rxr.opcode == RSP_PCR_GRANT_ACK) pcrd_inc = 1'b1;

    req_taken  = 1'b0;
    dat_taken  = s_dat_v;                       // the snoop FSM goes first on RSP / DAT
    ack_taken  = s_rsp_v;
    rsp_taken  = p2_rsp_v;                   // P2 (a hit) owns rsp_q this cycle
    pcrd_taken = 1'b0;

    for (int k = 0; k < NUM_MSHR; k++) begin
      i       = rot_idx(rot_q, k);
      e       = m_q[i];
      n       = m_q[i];
      q_set   = set_of(e.addr);
      q_tag   = tag_of(e.addr);
      q_beat  = e.addr[5:4];
      q_off   = e.addr[3:0];
      q_store = (e.op == MEM_OP_STORE);
      lk      = lookup(q_tag);
      rrw     = rr_q[q_set];

      if (rxr_v && rxr.opcode != RSP_PCR_GRANT_ACK && e.state != R_IDLE && rxr.txnid == e.txnid) begin
        unique case (rxr.opcode)
          RSP_RETRY_ACK: begin
            n.got_retry = 1'b1;
            n.pcrd_type = rxr.pcrdttype;
          end
          RSP_COMP: begin
            n.got_comp = 1'b1;
            if (rxr.resperr[1]) n.werr = 1'b1;
          end
          RSP_COMP_DBID_RESP: begin
            n.got_comp = 1'b1;
            n.got_dbid = 1'b1;
            n.wdbid    = rxr.dbid;
            if (rxr.resperr[1]) n.werr = 1'b1;
          end
          RSP_DBID_RESP: begin
            n.got_dbid = 1'b1;
            n.wdbid    = rxr.dbid;
          end
          default: ;
        endcase
      end

      unique case (e.state)
        R_IDLE: ;

        // ---------------------------------------------------------- ordering
        R_DEP: begin
          if (e.dep == '0) begin
            if (e.cach) n.state = R_TAG_RD;
            else        n = start_txn(n, q_store ? T_NCWR : T_NCRD, int'(i));
          end
        end

        // ---------------------------------------------------------- hit / miss
        R_LOOKUP: begin
          n.first = 1'b0;
          if (lk.hit && (!q_store || lk.hst == CS_UC || lk.hst == CS_UD)) begin
            perf_hit_m = e.first;
            n.acc_done = 1'b1;
            r_dat_set  = q_set;
            r_dat_way  = lk.hway;
            r_dat_beat = q_beat;
            if (!q_store) begin
              r_dat_re = 1'b1;
              n.rst    = lk.hst;
              n.state  = R_LOAD;
            end else begin
              r_dat_we    = 1'b1;
              r_dat_be    = be_of(e.width, q_off);
              r_dat_wdata = 128'(e.data) << {q_off, 3'b000};
              n.rst       = CS_UD;
              if (lk.hst == CS_UC) begin
                r_tag_we  = 1'b1;
                r_tag_set = q_set;
                r_tag_way = lk.hway;
                r_tag_tag = q_tag;
                r_tag_st  = CS_UD;
              end
              n.state = e.ack_pend ? R_ACK : R_RESP;
            end
          end else begin
            perf_miss = e.first;
            if (e.ack_pend) begin
              n.state = R_ACK;                   // the filled line is not usable: ack, then start over
            end else if (lk.hit || lk.inv) begin
              n = start_txn(n, T_READ, int'(i)); // store on a Shared line, or a free way
            end else begin
              n.way  = rrw;                      // replace a valid line first
              n.vtag = rd_tag[rrw];
              rr_inc = 1'b1;
              rr_set = q_set;
              if (rd_st[rrw] == CS_SC || rd_st[rrw] == CS_UC) begin
                r_tag_we   = 1'b1;               // a clean line is dropped right away
                r_tag_set  = q_set;
                r_tag_way  = rrw;
                r_tag_tag  = rd_tag[rrw];
                r_tag_st   = CS_I;
                perf_evict = 1'b1;
                n = start_txn(n, T_EVICT, int'(i));
              end else begin
                n = start_txn(n, T_WB, int'(i)); // a dirty line stays until its data is sent
              end
            end
          end
        end

        R_LOAD: begin
          n.rdata = load_of(ram_rdata, e.width, q_off);
          if (e.ack_pend) begin
            n.state = R_ACK;
          end else if (!rsp_taken) begin
            rsp_taken  = 1'b1;
            c_rsp_v    = 1'b1;
            c_rsp_data = load_of(ram_rdata, e.width, q_off);
            c_rsp_err  = e.err;
            c_rsp_st   = e.rst;
            c_rsp_op   = e.op;
            c_rsp_idx  = e.idx;
            c_rsp_src  = e.src;
            n.state    = R_IDLE;
          end else begin
            n.state = R_RESP;                    // another MSHR answers the core this cycle
          end
        end

        R_TAG_RD: begin
          if (e_port[i]) begin
            r_tag_re  = 1'b1;
            r_tag_set = q_set;
            n.state   = R_LOOKUP;
          end
        end

        // ---------------------------------------------------------- request / first response
        R_REQ: begin
          if (!req_taken) begin
            req_taken = 1'b1;
            req_v     = 1'b1;
            req_c     = mk_req(e);
            if (req_rdy) n.state = R_WAIT;
          end
        end

        R_WAIT: begin
          if (e.got_retry) begin
            n.got_retry = 1'b0;
            perf_retry  = perf_retry + 5'd1;
            n.state     = R_RETRY;
          end else begin
            unique case (e.txn)
              T_READ:  if (rxd_mine[i]) n.state = R_FILL_TAG;
              T_NCRD: begin
                if (rxd_mine[i]) begin
                  rxd_pop    = 1'b1;
                  n.rdata    = load_of(rxd.data, e.width, q_off);
                  n.err      = rxd.resperr[1];
                  n.acc_done = 1'b1;
                  n.rd_act   = 1'b0;
                  n.state    = R_RESP;
                end
              end
              T_EVICT: if (e.got_comp) n.state = R_TAG_RD;
              T_WB:    if (e.got_dbid) n.state = R_WB_TAG;
              default: begin
                if (e.got_dbid) begin
                  n.cnt   = '0;
                  n.state = R_WDAT;
                end
              end
            endcase
          end
        end

        R_RETRY: begin
          if (!pcrd_taken && pcrd_q[e.pcrd_type] != '0) begin
            pcrd_taken    = 1'b1;
            pcrd_dec      = 1'b1;
            pcrd_dec_type = e.pcrd_type;
            n.allow_retry = 1'b0;
            n.state       = R_REQ;
          end
        end

        // ---------------------------------------------------------- line fill
        R_FILL_TAG: begin
          if (e_port[i]) begin
            r_tag_re  = 1'b1;
            r_tag_set = q_set;
            n.state   = R_FILL_SEL;
          end
        end

        R_FILL_SEL: begin
          n.keep   = lk.hit;
          n.old_st = lk.hst;
          n.way    = lk.hit ? lk.hway : (lk.inv ? lk.iway : rrw);
          n.seen   = '0;
          n.state  = R_FILL;
        end

        R_FILL: begin
          if (rxd_mine[i] && (e.keep || e_port[i])) begin
            rxd_pop = 1'b1;
            if (!e.keep) begin
              r_dat_we    = 1'b1;
              r_dat_set   = q_set;
              r_dat_way   = e.way;
              r_dat_beat  = rxd.dataid;
              r_dat_be    = '1;
              r_dat_wdata = rxd.data;
            end
            if (e.seen == '0) begin
              n.cresp = rxd.resp;
              n.cdbid = rxd.dbid;
              n.chome = rxd.homenid;
            end
            if (rxd.resperr[1]) n.cerr = 1'b1;
            n.seen = e.seen | (4'b0001 << rxd.dataid);
            if (n.seen == 4'hF) begin
              n.rd_act = 1'b0;
              n.state  = R_FILL_DONE;
            end
          end
        end

        R_FILL_DONE: begin
          if (e_port[i]) begin
            n.ack_pend = 1'b1;
            if (e.cerr) begin                    // a line that came with an error is not kept
              n.err      = 1'b1;
              n.acc_done = 1'b1;
              n.state    = R_ACK;
            end else begin
              r_tag_we  = 1'b1;
              r_tag_set = q_set;
              r_tag_way = e.way;
              r_tag_tag = q_tag;
              r_tag_st  = fill_state(e);
              n.state   = R_TAG_RD;              // do the core access before the CompAck
            end
          end
        end

        // ---------------------------------------------------------- write-back
        R_WB_TAG: begin
          if (e_port[i]) begin
            r_tag_re  = 1'b1;
            r_tag_set = q_set;
            n.state   = R_WB_CMP;
          end
        end

        R_WB_CMP: begin
          n.cnt = '0;
          if (rd_st[e.way] != CS_I && rd_tag[e.way] == e.vtag) begin
            n.wb_resp  = st2resp(rd_st[e.way]);
            n.wb_vld   = 1'b1;
            r_tag_we   = 1'b1;
            r_tag_set  = q_set;
            r_tag_way  = e.way;
            r_tag_tag  = e.vtag;
            r_tag_st   = CS_I;
            r_dat_re   = 1'b1;
            r_dat_set  = q_set;
            r_dat_way  = e.way;
            r_dat_beat = 2'd0;
            n.state    = R_WB_RD;
          end else begin                         // a snoop took the line: nothing left to write
            n.wb_resp = RESP_I;
            n.wb_vld  = 1'b0;
            n.state   = R_WDAT;
          end
        end

        R_WB_RD: begin
          n.wb_buf[e.cnt*128 +: 128] = ram_rdata;
          if (e.cnt != 2'd3) begin
            r_dat_re   = 1'b1;
            r_dat_set  = q_set;
            r_dat_way  = e.way;
            r_dat_beat = e.cnt + 2'd1;
            n.cnt      = e.cnt + 2'd1;
          end else begin
            n.cnt   = '0;
            n.state = R_WDAT;
          end
        end

        R_WDAT: begin
          if (!dat_taken) begin
            dat_taken = 1'b1;
            r_dat_v   = 1'b1;
            wdat_c    = mk_wdat(e);
            if (dat_rdy) begin
              if (e.txn != T_WB || e.cnt == 2'd3) n.state = R_WCOMP;
              else                                n.cnt   = e.cnt + 2'd1;
            end
          end
        end

        R_WCOMP: begin
          if (e.got_comp) begin
            if (e.txn == T_WB) begin
              perf_wb = perf_wb + 5'd1;
              n.state = R_TAG_RD;
            end else begin
              n.acc_done = 1'b1;
              n.err      = e.werr;
              n.state    = R_RESP;
            end
          end
        end

        // ---------------------------------------------------------- finish
        R_ACK: begin
          if (!ack_taken) begin
            ack_taken = 1'b1;
            r_ack_v   = 1'b1;
            ack_c     = mk_ack(e);
            if (rsp_rdy) begin
              n.ack_pend = 1'b0;
              n.state    = e.acc_done ? R_RESP : R_TAG_RD;
            end
          end
        end

        R_RESP: begin
          if (!rsp_taken) begin
            rsp_taken  = 1'b1;
            c_rsp_v    = 1'b1;
            c_rsp_data = e.rdata;
            c_rsp_err  = e.err;
            c_rsp_st   = e.rst;
            c_rsp_op   = e.op;
            c_rsp_idx  = e.idx;
            c_rsp_src  = e.src;
            n.state    = R_IDLE;
          end
        end

        default: n.state = R_IDLE;
      endcase

      m_d[i] = n;
    end

    // ---- MSHRs that finished this cycle release the requests waiting on them
    for (int k = 0; k < NUM_MSHR; k++) begin
      active[k] = (m_q[k].state != R_IDLE);
      freed[k]  = active[k] && (m_d[k].state == R_IDLE);
    end
    for (int k = 0; k < NUM_MSHR; k++) m_d[k].dep = m_d[k].dep & ~freed;

    // ---- a request P2 does not answer goes into a free MSHR (one is always
    // free: the issue rule reserves it)
    free_found = 1'b0;
    free_i     = '0;
    for (int k = NUM_MSHR - 1; k >= 0; k--)
      if (!active[k]) begin free_found = 1'b1; free_i = MW'(k); end
    c_ready = free_found;

    dep_now = p2_dep_vec;                   // older MSHRs on the same set (registered)

    if (p2_alloc && free_found) begin
      n          = m_q[free_i];
      n.op       = p2_q.op;
      n.addr     = p2_q.addr;
      n.width    = p2_q.width;
      n.data     = p2_q.data;
      n.idx      = p2_q.idx;
      n.src      = p2_q.src;
      n.cach     = p2_q.cach;
      n.dep      = dep_now & ~freed;
      n.first    = 1'b1;
      n.acc_done = 1'b0;
      n.err      = 1'b0;
      n.rdata    = '0;
      n.rst      = CS_I;
      n.ack_pend = 1'b0;
      n.rd_act   = 1'b0;
      if (dep_now != '0) begin
        n.state = R_DEP;                    // per-set order: after the older MSHRs
      end else if (!n.cach) begin
        n = start_txn(n, (p2_q.op == MEM_OP_STORE) ? T_NCWR : T_NCRD, int'(free_i));
      end else begin
        n.state = R_TAG_RD;                 // miss / upgrade: the MSHR flow on port B
      end
      m_d[free_i] = n;
    end

    // ---- the response register: P2 first, else the MSHR that answered
    rsp_d = '0;
    if (p2_rsp_v) begin
      rsp_d.valid = 1'b1;
      rsp_d.op    = p2_q.op;
      rsp_d.data  = p2_store ? '0 : p2_rdata;
      rsp_d.idx   = p2_q.idx;
      rsp_d.src   = p2_q.src;
      rsp_d.err   = 1'b0;
      rsp_d.st    = p2_store ? CS_UD : p2_lk.hst;
    end else if (c_rsp_v) begin
      rsp_d.valid = 1'b1;
      rsp_d.op    = c_rsp_op;
      rsp_d.data  = c_rsp_data;
      rsp_d.idx   = c_rsp_idx;
      rsp_d.src   = c_rsp_src;
      rsp_d.err   = c_rsp_err;
      rsp_d.st    = c_rsp_st;
    end
  end

  // ------------------------------------------------------------------ snoop FSM
  always_comb begin
    s_d       = s_q;
    rxs_pop   = 1'b0;
    s_tag_re  = 1'b0;
    s_tag_we  = 1'b0;
    s_tag_set = sn_set;
    s_tag_way = s_hit_way;
    s_tag_tag = sn_tag;
    s_tag_st  = CS_I;
    s_dat_re  = 1'b0;
    s_dat_set = sn_set;
    s_dat_way = s_q.way;
    s_dat_beat = 2'd0;
    s_rsp_v   = 1'b0;
    s_dat_v   = 1'b0;
    perf_snp = 1'b0; perf_snp_hit = 1'b0; perf_snp_data = 1'b0;

    srsp_c        = '0;
    srsp_c.tgtid  = s_q.srcid;
    srsp_c.srcid  = NODEID;
    srsp_c.txnid  = s_q.txnid;
    srsp_c.opcode = RSP_SNP_RESP;
    srsp_c.resp   = s_q.resp;

    sdat_c         = '0;
    sdat_c.tgtid   = s_q.srcid;
    sdat_c.srcid   = NODEID;
    sdat_c.txnid   = s_q.txnid;
    sdat_c.homenid = HNID;
    sdat_c.opcode  = DAT_SNP_RESP_DATA;
    sdat_c.resp    = s_q.resp;
    sdat_c.dataid  = s_q.cnt;
    sdat_c.be      = '1;
    sdat_c.data    = s_q.buf_[s_q.cnt*128 +: 128];

    unique case (s_q.state)
      S_IDLE: begin
        if (rxs_v) begin
          rxs_pop    = 1'b1;
          perf_snp   = 1'b1;
          s_d.txnid  = rxs.txnid;
          s_d.srcid  = rxs.srcid;
          s_d.opcode = snp_base(rxs.opcode);
          s_d.addr   = rxs.addr[AddrSize-1:0];
          s_d.rts    = rxs.rettosrc;
          s_d.state  = S_WANT;
        end
      end

      S_WANT: begin
        if (s_grant) begin
          s_tag_re  = 1'b1;
          s_d.state = S_CMP;
        end
      end

      S_CMP: begin
        logic         send_data;
        cache_state_e nst;
        logic         dirty;
        send_data = 1'b0;
        nst       = s_hit_st;
        dirty     = (s_hit_st == CS_UD) || (s_hit_st == CS_SD);
        s_d.resp  = RESP_I;
        s_d.way   = s_hit_way;
        s_d.cnt   = '0;
        if (s_hit) begin
          perf_snp_hit = 1'b1;
          unique case (s_q.opcode)
            SNP_SHARED, SNP_CLEAN, SNP_NOT_SHARED_DIRTY, SNP_CLEAN_SHARED: begin
              nst       = CS_SC;
              s_d.resp  = dirty ? RESP_SC_PD : RESP_SC;
              send_data = dirty || (s_q.rts && s_q.opcode != SNP_CLEAN_SHARED);
            end
            SNP_ONCE: begin
              unique case (s_hit_st)
                CS_SC:   s_d.resp = RESP_SC;
                CS_UC:   s_d.resp = RESP_UC;
                CS_UD:   s_d.resp = RESP_UD;
                default: s_d.resp = RESP_SD;
              endcase
              send_data = dirty || s_q.rts;
            end
            SNP_MAKE_INVALID: begin
              nst = CS_I;
            end
            SNP_CLEAN_INVALID: begin
              nst       = CS_I;
              s_d.resp  = dirty ? RESP_I_PD : RESP_I;
              send_data = dirty;
            end
            default: begin                      // SnpUnique and anything not known
              nst       = CS_I;
              s_d.resp  = dirty ? RESP_I_PD : RESP_I;
              send_data = dirty || s_q.rts;
            end
          endcase
          if (nst != s_hit_st) begin
            s_tag_we = 1'b1;
            s_tag_st = nst;
          end
        end
        if (send_data) begin
          perf_snp_data = 1'b1;
          s_dat_re      = 1'b1;
          s_dat_way     = s_hit_way;
          s_d.state     = S_DATA;
        end else begin
          s_d.state = S_SEND_RSP;
        end
      end

      S_DATA: begin
        s_d.buf_[s_q.cnt*128 +: 128] = ram_rdata;
        if (s_q.cnt != 2'd3) begin
          s_dat_re   = 1'b1;
          s_dat_beat = s_q.cnt + 2'd1;
          s_d.cnt    = s_q.cnt + 2'd1;
        end else begin
          s_d.cnt   = '0;
          s_d.state = S_SEND_DAT;
        end
      end

      S_SEND_DAT: begin
        s_dat_v = 1'b1;
        if (dat_rdy) begin
          if (s_q.cnt == 2'd3) s_d.state = S_IDLE;
          else                 s_d.cnt   = s_q.cnt + 2'd1;
        end
      end

      S_SEND_RSP: begin
        s_rsp_v = 1'b1;
        if (rsp_rdy) s_d.state = S_IDLE;
      end

      default: s_d.state = S_IDLE;
    endcase
  end

  // ------------------------------------------------------------------ TX muxes: the snoop FSM goes first
  assign req_f = req_c;
  assign rsp_v = s_rsp_v || r_ack_v;
  assign rsp_f = s_rsp_v ? srsp_c : ack_c;
  assign dat_v = s_dat_v || r_dat_v;
  assign dat_f = s_dat_v ? sdat_c : wdat_c;

  // ------------------------------------------------------------------ core response
  always_comb begin
    // ready from the FIFO count, the answer from rsp_q: both flops
    core.ready         = (fq_cnt_q != 2'd2) ? Y : N;
    core.rsp           = '0;
    core.rsp.valid     = rsp_q.valid ? Y : N;
    core.rsp.op        = rsp_q.op;
    core.rsp.data      = rsp_q.data;
    core.rsp.idx       = rsp_q.idx;
    core.rsp.src       = rsp_q.src;
    core.rsp.err       = rsp_q.err ? Y : N;
    core.rsp.chr_state = rsp_q.st;
  end

  // ------------------------------------------------------------------ registers
  logic [31:0] perf_hits_r, perf_misses_r, perf_evictions_r, perf_writebacks_r, perf_retries_r;
  logic [31:0] perf_snoops_r, perf_snoop_hits_r, perf_snoop_data_r;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int k = 0; k < NUM_MSHR; k++) begin
        m_q[k]        <= '0;
        m_q[k].state  <= R_IDLE;
        m_q[k].old_st <= CS_I;
        m_q[k].rst    <= CS_I;
      end
      s_q       <= '0;
      s_q.state <= S_IDLE;
      rot_q     <= '0;
      fq_rd_q   <= 1'b0;
      fq_wr_q   <= 1'b0;
      fq_cnt_q  <= 2'd0;
      p2_v_q    <= 1'b0;
      p2_q      <= '0;
      rsp_q     <= '0;
      for (int s = 0; s < SETS; s++) rr_q[s] <= '0;
      for (int k = 0; k < 16; k++)   pcrd_q[k] <= '0;
      perf_hits_r       <= '0;
      perf_misses_r     <= '0;
      perf_evictions_r  <= '0;
      perf_writebacks_r <= '0;
      perf_retries_r    <= '0;
      perf_snoops_r     <= '0;
      perf_snoop_hits_r <= '0;
      perf_snoop_data_r <= '0;
    end else begin
      for (int k = 0; k < NUM_MSHR; k++) m_q[k] <= m_d[k];
      s_q   <= s_d;
      rot_q <= rot_idx(rot_q, 1);

      // hit pipeline: request FIFO -> P2 -> response register
      if (accept_w) begin
        fq_mem[fq_wr_q] <= '{op: core.req.op, addr: core.req.addr, width: core.req.width,
                               data: core.req.data, idx: core.req.idx, src: core.req.src,
                               cach: (core.req.cacheable == Y)};
        fq_wr_q <= ~fq_wr_q;
      end
      if (issue_w) fq_rd_q <= ~fq_rd_q;
      fq_cnt_q <= fq_cnt_q + 2'(accept_w) - 2'(issue_w);
      p2_v_q   <= issue_w;
      if (issue_w) p2_q <= head;
      rsp_q    <= rsp_d;
      if (rr_inc) rr_q[rr_set] <= rr_q[rr_set] + 1'b1;

      if (pcrd_inc && pcrd_dec && rxr.pcrdttype == pcrd_dec_type) begin
        // granted and used in the same cycle
      end else begin
        if (pcrd_inc && pcrd_q[rxr.pcrdttype] != '1) pcrd_q[rxr.pcrdttype] <= pcrd_q[rxr.pcrdttype] + 1'b1;
        if (pcrd_dec)                                pcrd_q[pcrd_dec_type] <= pcrd_q[pcrd_dec_type] - 1'b1;
      end

      perf_hits_r       <= perf_hits_r + 32'(perf_hit);
      if (perf_miss)     perf_misses_r     <= perf_misses_r + 1;
      if (perf_evict)    perf_evictions_r  <= perf_evictions_r + 1;
      perf_writebacks_r <= perf_writebacks_r + 32'(perf_wb);
      perf_retries_r    <= perf_retries_r + 32'(perf_retry);
      if (perf_snp)      perf_snoops_r     <= perf_snoops_r + 1;
      if (perf_snp_hit)  perf_snoop_hits_r <= perf_snoop_hits_r + 1;
      if (perf_snp_data) perf_snoop_data_r <= perf_snoop_data_r + 1;
    end
  end

  // ------------------------------------------------------------------ checks (simulation only)
  // synthesis translate_off
  always @(posedge clk) begin
    if (resetn) begin
      if ($countones(e_port) > 1)
        $error("rnf_dcache: %0d MSHRs own the RAM port in one cycle (%b)", $countones(e_port), e_port);
      if (s_grant && e_port != '0)
        $error("rnf_dcache: snoop FSM and MSHR(s) %b own the RAM port in one cycle", e_port);
      if (p2_alloc && !c_ready)
        $error("rnf_dcache: P2 has a request for an MSHR but none is free (the issue rule should prevent this)");
      if (pa_dat_re && pa_dat_we != '0)
        $error("rnf_dcache: port A read and store-hit write in one cycle");
    end
  end
  // synthesis translate_on

endmodule
`default_nettype wire
