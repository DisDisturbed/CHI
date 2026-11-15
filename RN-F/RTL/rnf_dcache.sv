`timescale 1ns/1ps
`default_nettype none
// FSM encodings of rnf_dcache (in a package so a testbench can name them)
package rnf_dcache_pkg;

  typedef enum logic [4:0] {
    R_IDLE,
    R_LOOKUP,     // tags of the set are valid: hit / miss / victim
    R_LOAD,       // load data is valid
    R_TAG_RD,     // read the tags again (after a wait the set may have changed)
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
    R_RESP        // answer the core
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
// rnf_dcache - CHI RN-F data cache (functional baseline)
//
//  Core side : rnf_core_if, one request at a time.
//  CHI side  : REQ / RSP / DAT transmit, SNP / RSP / DAT receive.
//
//  Cache     : SETS x WAYS lines of 64 bytes. The tag entry holds the CHI line
//              state (I / SC / UC / UD / SD). Replacement uses an Invalid way
//              first, otherwise one round-robin pointer per set.
//
//  Requests  : load miss            ReadShared
//              store miss / store on a Shared line   ReadUnique
//              replace a clean line Evict
//              replace a dirty line WriteBackFull
//              non-cacheable        ReadNoSnp / WriteNoSnpPtl
//              RetryAck is followed by a resend once the PCrdGrant is there.
//
//  Snoops    : served by their own FSM, also while a request of this cache is
//              outstanding. Both FSMs share the single RAM port: the snoop FSM
//              has priority, the request FSM only finishes the short read ->
//              act sequence it is in (r_atomic) and then waits.
//              Policy: a snoop that allows it leaves a Shared Clean copy and
//              passes dirty data on (SnpRespData_SC_PD); SnpOnce keeps the
//              state; SnpUnique / SnpCleanInvalid / SnpMakeInvalid go to I.
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
  parameter int       TX_CREDIT_MAX  = 15
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

  initial begin
    if (AddrSize > TSHR_ADDR_WIDTH)
      $fatal(1, "rnf_dcache: AddrSize (%0d) is wider than the CHI address (%0d)", AddrSize, TSHR_ADDR_WIDTH);
    if (MaxAccessWidth > 128 || (MaxAccessWidth % 8) != 0)
      $fatal(1, "rnf_dcache: MaxAccessWidth (%0d) must be a multiple of 8 and at most 128", MaxAccessWidth);
    if (RX_FIFO_DEPTH < 2 || RX_FIFO_DEPTH > 15)
      $fatal(1, "rnf_dcache: RX_FIFO_DEPTH (%0d) must be 2..15 (one link credit per entry)", RX_FIFO_DEPTH);
    if ((1 << SET_W) != SETS || (1 << WAY_W) != WAYS || SETS < 2 || WAYS < 2)
      $fatal(1, "rnf_dcache: SETS (%0d) and WAYS (%0d) must be powers of two, at least 2", SETS, WAYS);
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
    .clk(clk), .resetn(resetn), .flitv_i(rsp_rx.flitv == Y), .flit_i(rsp_rx.flit), .lcrdv_o(rsp_rx.lcrdv),
    .valid_o(rxr_v), .flit_o(rxr_bits), .pop_i(rxr_pop));
  rnf_rx_fifo #(.W(DAT_W), .DEPTH(RX_FIFO_DEPTH)) u_dat_rx (
    .clk(clk), .resetn(resetn), .flitv_i(dat_rx.flitv == Y), .flit_i(dat_rx.flit), .lcrdv_o(dat_rx.lcrdv),
    .valid_o(rxd_v), .flit_o(rxd_bits), .pop_i(rxd_pop));
  rnf_rx_fifo #(.W(SNP_W), .DEPTH(RX_FIFO_DEPTH)) u_snp_rx (
    .clk(clk), .resetn(resetn), .flitv_i(snp_rx.flitv == Y), .flit_i(snp_rx.flit), .lcrdv_o(snp_rx.lcrdv),
    .valid_o(rxs_v), .flit_o(rxs_bits), .pop_i(rxs_pop));

  assign rxr = local_rsp_flit_t'(rxr_bits);
  assign rxd = local_dat_flit_t'(rxd_bits);
  assign rxs = local_snp_flit_t'(rxs_bits);

  // ------------------------------------------------------------------ RAMs
  // request FSM (r_*) and snoop FSM (s_*) drive the one port; they never do so
  // in the same cycle (see r_atomic / s_blk)
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

  rnf_tag_ram #(.SETS(SETS), .WAYS(WAYS), .TAG_W(TAG_W)) u_tag_ram (
    .clk(clk), .resetn(resetn), .set_i(tag_set), .re_i(tag_re), .we_i(tag_we),
    .way_i(tag_way), .tag_i(tag_tag), .state_i(tag_st), .tag_o(rd_tag), .state_o(rd_st));

  wire                         ram_re   = s_port ? s_dat_re : r_dat_re;
  wire                         ram_we   = s_port ? 1'b0     : r_dat_we;
  wire [SET_W+WAY_W+2-1:0]     ram_addr = s_port ? {s_dat_set, s_dat_way, s_dat_beat}
                                                 : {r_dat_set, r_dat_way, r_dat_beat};
  logic [127:0]                ram_rdata;

  rnf_data_ram #(.DEPTH(SETS * WAYS * 4)) u_data_ram (
    .clk(clk), .addr_i(ram_addr), .re_i(ram_re), .we_i(ram_we), .be_i(r_dat_be),
    .wdata_i(r_dat_wdata), .rdata_o(ram_rdata));

  // ------------------------------------------------------------------ request FSM state
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

  rreg_t r_q, r_d;

  logic [WAY_W-1:0] rr_q [SETS];             // round-robin victim pointer per set
  logic             rr_inc;
  logic [3:0]       pcrd_q [16];             // PCrdGrants not used yet, per PCrdType
  logic             pcrd_inc, pcrd_dec;

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

  wire s_blk    = (s_q.state == S_WANT) || (s_q.state == S_CMP) || (s_q.state == S_DATA);
  wire r_atomic = (r_q.state == R_LOOKUP) || (r_q.state == R_FILL_SEL) ||
                  (r_q.state == R_WB_CMP) || (r_q.state == R_WB_RD);

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

  wire [SET_W-1:0] q_set  = r_q.addr[6 +: SET_W];
  wire [TAG_W-1:0] q_tag  = r_q.addr[AddrSize-1 -: TAG_W];
  wire [1:0]       q_beat = r_q.addr[5:4];
  wire [3:0]       q_off  = r_q.addr[3:0];
  wire [15:0]      q_be   = be_of(r_q.width, q_off);
  wire [127:0]     q_wbeat = 128'(r_q.data) << {q_off, 3'b000};
  wire             q_store = (r_q.op == MEM_OP_STORE);

  wire [SET_W-1:0] sn_set = s_q.addr[6 +: SET_W];
  wire [TAG_W-1:0] sn_tag = s_q.addr[AddrSize-1 -: TAG_W];

  // lookups on the tag RAM output
  logic             r_hit, r_inv, s_hit;
  logic [WAY_W-1:0] r_hit_way, r_inv_way, s_hit_way;
  cache_state_e     r_hit_st, s_hit_st;

  always_comb begin
    r_hit = 1'b0; r_hit_way = '0; r_hit_st = CS_I; r_inv = 1'b0; r_inv_way = '0;
    s_hit = 1'b0; s_hit_way = '0; s_hit_st = CS_I;
    for (int w = WAYS - 1; w >= 0; w--) begin
      if (rd_st[w] != CS_I && rd_tag[w] == q_tag) begin
        r_hit = 1'b1; r_hit_way = WAY_W'(w); r_hit_st = rd_st[w];
      end
      if (rd_st[w] == CS_I) begin
        r_inv = 1'b1; r_inv_way = WAY_W'(w);
      end
      if (rd_st[w] != CS_I && rd_tag[w] == sn_tag) begin
        s_hit = 1'b1; s_hit_way = WAY_W'(w); s_hit_st = rd_st[w];
      end
    end
  end

  wire [WAY_W-1:0] rr_way = rr_q[q_set];

  // CompData of the read in flight / anything else on the DAT channel
  wire rxd_mine = rxd_v && r_q.rd_act && (rxd.txnid == r_q.txnid) && (rxd.opcode == DAT_COMP_DATA);

  // state a filled line gets
  cache_state_e fill_st;
  always_comb begin
    unique case (r_q.cresp)
      RESP_UC:    fill_st = CS_UC;
      RESP_UD_PD: fill_st = CS_UD;
      RESP_SD_PD: fill_st = CS_SD;
      default:    fill_st = CS_SC;
    endcase
    // the line stayed here Shared Dirty: its own data is kept and it is still dirty
    if (r_q.keep && r_q.old_st == CS_SD) fill_st = q_store ? CS_UD : CS_SD;
  end

  // ------------------------------------------------------------------ perf counters
  logic perf_hit, perf_miss, perf_evict, perf_wb, perf_retry, perf_snp, perf_snp_hit, perf_snp_data;

  // ------------------------------------------------------------------ request FSM
  logic            start_txn;
  txn_e            start_kind;
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

  always_comb begin
    r_d        = r_q;
    start_txn  = 1'b0;
    start_kind = T_READ;
    rr_inc     = 1'b0;
    pcrd_inc   = 1'b0;
    pcrd_dec   = 1'b0;

    c_ready    = 1'b0;
    c_rsp_v    = 1'b0;
    c_rsp_data = r_q.rdata;
    c_rsp_err  = r_q.err;
    c_rsp_st   = r_q.rst;

    r_tag_re   = 1'b0;
    r_tag_we   = 1'b0;
    r_tag_set  = q_set;
    r_tag_way  = r_q.way;
    r_tag_tag  = q_tag;
    r_tag_st   = CS_I;
    r_dat_re   = 1'b0;
    r_dat_we   = 1'b0;
    r_dat_set  = q_set;
    r_dat_way  = r_q.way;
    r_dat_beat = q_beat;
    r_dat_be   = '1;
    r_dat_wdata = rxd.data;

    req_v      = 1'b0;
    r_ack_v    = 1'b0;
    r_dat_v    = 1'b0;
    rxd_pop    = rxd_v && !rxd_mine;        // a DAT flit nobody waits for is dropped

    perf_hit = 1'b0; perf_miss = 1'b0; perf_evict = 1'b0; perf_wb = 1'b0; perf_retry = 1'b0;

    // ---- responses on the RSP channel are taken as they arrive
    rxr_pop = rxr_v;
    if (rxr_v) begin
      if (rxr.opcode == RSP_PCR_GRANT_ACK) begin
        pcrd_inc = 1'b1;
      end else if (rxr.txnid == r_q.txnid) begin
        unique case (rxr.opcode)
          RSP_RETRY_ACK: begin
            r_d.got_retry = 1'b1;
            r_d.pcrd_type = rxr.pcrdttype;
          end
          RSP_COMP: begin
            r_d.got_comp = 1'b1;
            if (rxr.resperr[1]) r_d.werr = 1'b1;
          end
          RSP_COMP_DBID_RESP: begin
            r_d.got_comp = 1'b1;
            r_d.got_dbid = 1'b1;
            r_d.wdbid    = rxr.dbid;
            if (rxr.resperr[1]) r_d.werr = 1'b1;
          end
          RSP_DBID_RESP: begin
            r_d.got_dbid = 1'b1;
            r_d.wdbid    = rxr.dbid;
          end
          default: ;
        endcase
      end
    end

    // ---- request flit
    req_c            = '0;
    req_c.tgtid      = HNID;
    req_c.srcid      = NODEID;
    req_c.txnid      = r_q.txnid;
    req_c.allowretry = r_q.allow_retry ? Y : N;
    req_c.pcrdttype  = r_q.pcrd_type;
    req_c.size       = WIDTH_64;
    req_c.memattr    = MEMATTR_NORMAL_WB;
    req_c.addr       = TSHR_ADDR_WIDTH'({q_tag, q_set, 6'b0});
    unique case (r_q.txn)
      T_READ: begin
        req_c.opcode     = q_store ? REQ_READ_UNIQUE : REQ_READ_SHARED;
        req_c.expcompack = 1'b1;
      end
      T_EVICT: begin
        req_c.opcode = REQ_EVICT;
        req_c.addr   = TSHR_ADDR_WIDTH'({r_q.vtag, q_set, 6'b0});
      end
      T_WB: begin
        req_c.opcode = REQ_WRITE_BACK_FULL;
        req_c.addr   = TSHR_ADDR_WIDTH'({r_q.vtag, q_set, 6'b0});
      end
      T_NCRD: begin
        req_c.opcode  = REQ_READ_NO_SNOOP;
        req_c.addr    = TSHR_ADDR_WIDTH'(r_q.addr);
        req_c.size    = r_q.width;
        req_c.memattr = MEMATTR_NORMAL_NC;
      end
      default: begin
        req_c.opcode  = REQ_WRITE_NO_SNOOP_PTL;
        req_c.addr    = TSHR_ADDR_WIDTH'(r_q.addr);
        req_c.size    = r_q.width;
        req_c.memattr = MEMATTR_NORMAL_NC;
      end
    endcase

    // ---- CompAck flit
    ack_c        = '0;
    ack_c.tgtid  = r_q.chome;
    ack_c.srcid  = NODEID;
    ack_c.txnid  = r_q.cdbid;
    ack_c.opcode = RSP_COMP_ACK;

    // ---- write data flit
    wdat_c         = '0;
    wdat_c.tgtid   = HNID;
    wdat_c.srcid   = NODEID;
    wdat_c.txnid   = r_q.wdbid;
    wdat_c.dbid    = r_q.wdbid;
    wdat_c.homenid = HNID;
    if (r_q.txn == T_WB) begin
      wdat_c.opcode = DAT_COPY_BACK_WR_DATA;
      wdat_c.resp   = r_q.wb_resp;
      wdat_c.dataid = r_q.cnt;
      wdat_c.be     = r_q.wb_vld ? '1 : '0;
      wdat_c.data   = r_q.wb_vld ? r_q.wb_buf[r_q.cnt*128 +: 128] : '0;
    end else begin
      wdat_c.opcode = DAT_NON_COPY_BACK_WR_DATA;
      wdat_c.resp   = RESP_I;
      wdat_c.dataid = q_beat;
      wdat_c.be     = q_be;
      wdat_c.data   = q_wbeat;
    end

    unique case (r_q.state)
      // ------------------------------------------------------------ accept
      R_IDLE: begin
        c_ready = !s_blk;
        if (core.req.valid == Y && c_ready) begin
          r_d.op       = core.req.op;
          r_d.addr     = core.req.addr;
          r_d.width    = core.req.width;
          r_d.data     = core.req.data;
          r_d.idx      = core.req.idx;
          r_d.src      = core.req.src;
          r_d.first    = 1'b1;
          r_d.acc_done = 1'b0;
          r_d.err      = 1'b0;
          r_d.rdata    = '0;
          r_d.rst      = CS_I;
          if (core.req.cacheable == Y) begin
            r_tag_re  = 1'b1;
            r_tag_set = core.req.addr[6 +: SET_W];
            r_d.state = R_LOOKUP;
          end else begin
            start_txn  = 1'b1;
            start_kind = (core.req.op == MEM_OP_STORE) ? T_NCWR : T_NCRD;
          end
        end
      end

      // ------------------------------------------------------------ hit / miss
      R_LOOKUP: begin
        r_d.first = 1'b0;
        if (r_hit && (!q_store || r_hit_st == CS_UC || r_hit_st == CS_UD)) begin
          perf_hit     = r_q.first;
          r_d.acc_done = 1'b1;
          r_dat_way    = r_hit_way;
          if (!q_store) begin
            r_dat_re  = 1'b1;
            r_d.rst   = r_hit_st;
            r_d.state = R_LOAD;
          end else begin
            r_dat_we    = 1'b1;
            r_dat_be    = q_be;
            r_dat_wdata = q_wbeat;
            r_d.rst     = CS_UD;
            if (r_hit_st == CS_UC) begin
              r_tag_we  = 1'b1;
              r_tag_way = r_hit_way;
              r_tag_st  = CS_UD;
            end
            r_d.state = r_q.ack_pend ? R_ACK : R_RESP;
          end
        end else begin
          perf_miss = r_q.first;
          if (r_q.ack_pend) begin
            r_d.state = R_ACK;                 // the filled line is not usable: ack, then start over
          end else if (r_hit || r_inv) begin
            start_txn  = 1'b1;                 // store on a Shared line, or a free way
            start_kind = T_READ;
          end else begin
            r_d.way  = rr_way;                 // replace a valid line first
            r_d.vtag = rd_tag[rr_way];
            rr_inc   = 1'b1;
            start_txn = 1'b1;
            if (rd_st[rr_way] == CS_SC || rd_st[rr_way] == CS_UC) begin
              r_tag_we   = 1'b1;               // a clean line is dropped right away
              r_tag_way  = rr_way;
              r_tag_tag  = rd_tag[rr_way];
              r_tag_st   = CS_I;
              start_kind = T_EVICT;
              perf_evict = 1'b1;
            end else begin
              start_kind = T_WB;               // a dirty line stays until its data is sent
            end
          end
        end
      end

      R_LOAD: begin
        r_d.rdata = load_of(ram_rdata, r_q.width, q_off);
        if (r_q.ack_pend) begin
          r_d.state = R_ACK;
        end else begin
          c_rsp_v    = 1'b1;
          c_rsp_data = load_of(ram_rdata, r_q.width, q_off);
          r_d.state  = R_IDLE;
        end
      end

      R_TAG_RD: begin
        if (!s_blk) begin
          r_tag_re  = 1'b1;
          r_d.state = R_LOOKUP;
        end
      end

      // ------------------------------------------------------------ request / first response
      R_REQ: begin
        req_v = 1'b1;
        if (req_rdy) r_d.state = R_WAIT;
      end

      R_WAIT: begin
        if (r_q.got_retry) begin
          r_d.got_retry = 1'b0;
          perf_retry    = 1'b1;
          r_d.state     = R_RETRY;
        end else begin
          unique case (r_q.txn)
            T_READ:  if (rxd_mine) r_d.state = R_FILL_TAG;
            T_NCRD: begin
              if (rxd_mine) begin
                rxd_pop      = 1'b1;
                r_d.rdata    = load_of(rxd.data, r_q.width, q_off);
                r_d.err      = rxd.resperr[1];
                r_d.acc_done = 1'b1;
                r_d.rd_act   = 1'b0;
                r_d.state    = R_RESP;
              end
            end
            T_EVICT: if (r_q.got_comp) r_d.state = R_TAG_RD;
            T_WB:    if (r_q.got_dbid) r_d.state = R_WB_TAG;
            default: begin
              if (r_q.got_dbid) begin
                r_d.cnt   = '0;
                r_d.state = R_WDAT;
              end
            end
          endcase
        end
      end

      R_RETRY: begin
        if (pcrd_q[r_q.pcrd_type] != '0) begin
          pcrd_dec        = 1'b1;
          r_d.allow_retry = 1'b0;
          r_d.state       = R_REQ;
        end
      end

      // ------------------------------------------------------------ line fill
      R_FILL_TAG: begin
        if (!s_blk) begin
          r_tag_re  = 1'b1;
          r_d.state = R_FILL_SEL;
        end
      end

      R_FILL_SEL: begin
        r_d.keep   = r_hit;
        r_d.old_st = r_hit_st;
        r_d.way    = r_hit ? r_hit_way : (r_inv ? r_inv_way : rr_way);
        r_d.seen   = '0;
        r_d.state  = R_FILL;
      end

      R_FILL: begin
        if (rxd_mine && (r_q.keep || !s_blk)) begin
          rxd_pop = 1'b1;
          if (!r_q.keep) begin
            r_dat_we    = 1'b1;
            r_dat_beat  = rxd.dataid;
            r_dat_wdata = rxd.data;
          end
          if (r_q.seen == '0) begin
            r_d.cresp = rxd.resp;
            r_d.cdbid = rxd.dbid;
            r_d.chome = rxd.homenid;
          end
          if (rxd.resperr[1]) r_d.cerr = 1'b1;
          r_d.seen = r_q.seen | (4'b0001 << rxd.dataid);
          if (r_d.seen == 4'hF) begin
            r_d.rd_act = 1'b0;
            r_d.state  = R_FILL_DONE;
          end
        end
      end

      R_FILL_DONE: begin
        if (!s_blk) begin
          r_d.ack_pend = 1'b1;
          if (r_q.cerr) begin                  // a line that came with an error is not kept
            r_d.err      = 1'b1;
            r_d.acc_done = 1'b1;
            r_d.state    = R_ACK;
          end else begin
            r_tag_we  = 1'b1;
            r_tag_st  = fill_st;
            r_d.state = R_TAG_RD;              // do the core access before the CompAck
          end
        end
      end

      // ------------------------------------------------------------ write-back
      R_WB_TAG: begin
        if (!s_blk) begin
          r_tag_re  = 1'b1;
          r_d.state = R_WB_CMP;
        end
      end

      R_WB_CMP: begin
        r_d.cnt = '0;
        if (rd_st[r_q.way] != CS_I && rd_tag[r_q.way] == r_q.vtag) begin
          r_d.wb_resp = st2resp(rd_st[r_q.way]);
          r_d.wb_vld  = 1'b1;
          r_tag_we    = 1'b1;
          r_tag_tag   = r_q.vtag;
          r_tag_st    = CS_I;
          r_dat_re    = 1'b1;
          r_dat_beat  = 2'd0;
          r_d.state   = R_WB_RD;
        end else begin                         // a snoop took the line: nothing left to write
          r_d.wb_resp = RESP_I;
          r_d.wb_vld  = 1'b0;
          r_d.state   = R_WDAT;
        end
      end

      R_WB_RD: begin
        r_d.wb_buf[r_q.cnt*128 +: 128] = ram_rdata;
        if (r_q.cnt != 2'd3) begin
          r_dat_re   = 1'b1;
          r_dat_beat = r_q.cnt + 2'd1;
          r_d.cnt    = r_q.cnt + 2'd1;
        end else begin
          r_d.cnt   = '0;
          r_d.state = R_WDAT;
        end
      end

      R_WDAT: begin
        r_dat_v = 1'b1;
        if (dat_rdy && !s_dat_v) begin
          if (r_q.txn != T_WB || r_q.cnt == 2'd3) r_d.state = R_WCOMP;
          else                                    r_d.cnt   = r_q.cnt + 2'd1;
        end
      end

      R_WCOMP: begin
        if (r_q.got_comp) begin
          if (r_q.txn == T_WB) begin
            perf_wb   = 1'b1;
            r_d.state = R_TAG_RD;
          end else begin
            r_d.acc_done = 1'b1;
            r_d.err      = r_q.werr;
            r_d.state    = R_RESP;
          end
        end
      end

      // ------------------------------------------------------------ finish
      R_ACK: begin
        r_ack_v = 1'b1;
        if (rsp_rdy && !s_rsp_v) begin
          r_d.ack_pend = 1'b0;
          r_d.state    = r_q.acc_done ? R_RESP : R_TAG_RD;
        end
      end

      R_RESP: begin
        c_rsp_v   = 1'b1;
        r_d.state = R_IDLE;
      end

      default: r_d.state = R_IDLE;
    endcase

    if (start_txn) begin
      r_d.state       = R_REQ;
      r_d.txn         = start_kind;
      r_d.txnid       = r_q.txnid + 12'd1;
      r_d.allow_retry = 1'b1;
      r_d.pcrd_type   = '0;
      r_d.got_retry   = 1'b0;
      r_d.got_comp    = 1'b0;
      r_d.got_dbid    = 1'b0;
      r_d.werr        = 1'b0;
      r_d.cerr        = 1'b0;
      r_d.rd_act      = (start_kind == T_READ) || (start_kind == T_NCRD);
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
        if (!r_atomic) begin
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
    core.ready         = c_ready ? Y : N;
    core.rsp           = '0;
    core.rsp.valid     = c_rsp_v ? Y : N;
    core.rsp.op        = r_q.op;
    core.rsp.data      = c_rsp_data;
    core.rsp.idx       = r_q.idx;
    core.rsp.src       = r_q.src;
    core.rsp.err       = c_rsp_err ? Y : N;
    core.rsp.chr_state = c_rsp_st;
  end

  // ------------------------------------------------------------------ registers
  logic [31:0] perf_hits_r, perf_misses_r, perf_evictions_r, perf_writebacks_r, perf_retries_r;
  logic [31:0] perf_snoops_r, perf_snoop_hits_r, perf_snoop_data_r;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      r_q       <= '0;
      r_q.state <= R_IDLE;
      r_q.old_st <= CS_I;
      r_q.rst   <= CS_I;
      s_q       <= '0;
      s_q.state <= S_IDLE;
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
      r_q <= r_d;
      s_q <= s_d;
      if (rr_inc) rr_q[q_set] <= rr_q[q_set] + 1'b1;

      if (pcrd_inc && pcrd_dec && rxr.pcrdttype == r_q.pcrd_type) begin
        // granted and used in the same cycle
      end else begin
        if (pcrd_inc && pcrd_q[rxr.pcrdttype] != '1) pcrd_q[rxr.pcrdttype] <= pcrd_q[rxr.pcrdttype] + 1'b1;
        if (pcrd_dec)                                pcrd_q[r_q.pcrd_type] <= pcrd_q[r_q.pcrd_type] - 1'b1;
      end

      if (perf_hit)      perf_hits_r       <= perf_hits_r + 1;
      if (perf_miss)     perf_misses_r     <= perf_misses_r + 1;
      if (perf_evict)    perf_evictions_r  <= perf_evictions_r + 1;
      if (perf_wb)       perf_writebacks_r <= perf_writebacks_r + 1;
      if (perf_retry)    perf_retries_r    <= perf_retries_r + 1;
      if (perf_snp)      perf_snoops_r     <= perf_snoops_r + 1;
      if (perf_snp_hit)  perf_snoop_hits_r <= perf_snoop_hits_r + 1;
      if (perf_snp_data) perf_snoop_data_r <= perf_snoop_data_r + 1;
    end
  end

endmodule
`default_nettype wire
