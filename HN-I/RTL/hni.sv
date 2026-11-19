`timescale 1ns/1ps
// =============================================================================
// hni - CHI HN-I: home node for device / I/O space, a CHI -> AXI4 adapter.
//
//  CHI side (one link: REQ RX, RSP TX, DAT RX, DAT TX; requesters told apart
//  by SrcID, as behind an XP):
//    ReadNoSnp            -> AXI read  -> CompData (Resp I), 1..4 beats as the
//                                         size needs (<=16 B: one beat, DataID
//                                         from addr[5:4])
//    WriteNoSnpPtl / Full -> DBIDResp (DBID = tracker) -> NonCopyBackWrData
//                            -> AXI write -> Comp after the AXI B response
//    Sizes 1..64 B, aligned to their size. No CompAck is expected (ExpCompAck
//    must be 0). Errors: RRESP / BRESP SLVERR -> RespErr DERR, DECERR -> NDERR.
//
//  AXI4 master, 32-bit data:
//    <= 4 B: one narrow beat (AxSIZE = the request size);
//    >  4 B: INCR burst of size/4 words (up to 16 beats = one 64 B line).
//    AxID = {SrcID[SRC_IDW-1:0], tracker}: unique per transaction in flight,
//    so read data may come back interleaved / in any order; a slave can tell
//    the requester from the upper ID bits (e.g. the SoC info page: HART_ID).
//    AxCACHE = 0 (Device non-bufferable), AxPROT = 0.
//
//  Ordering (Device memory): a request goes to AXI only once every earlier
//  request from the same SrcID has completed (its CompData / Comp sent). So
//  each requester's accesses happen in program order, side effects (a UART RX
//  pop, a PLIC claim) exactly once and in order; different requesters overlap.
//
//  Flow control: no RetryAck. A request is taken from the REQ FIFO only when a
//  tracker is free; REQ link credits are given only while the FIFO has room,
//  so a full HN-I simply holds the requester's REQ link credits.
//
//  NUM_TRK trackers; one AR and one AW/W burst are issued at a time (AR and
//  AW/W in parallel), any number of reads / writes wait for R / B.
// =============================================================================
module hni
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter node_id_e NODEID      = 7'h50,
  parameter int       AddrWidth   = 39,
  parameter int       DataWidth   = 128,
  parameter int       NUM_TRK     = 4,
  parameter int       REQ_DEPTH   = 4,       // request FIFO entries (REQ credits)
  parameter int       TxCreditMax = 15,
  parameter int       SRC_IDW     = 4,       // SrcID bits in the AXI ID
  parameter int       AXI_AW      = 32,
  localparam int      TW          = (NUM_TRK <= 1) ? 1 : $clog2(NUM_TRK),
  localparam int      IDW         = SRC_IDW + TW
) (
  input  wire clk,
  input  wire resetn,

  chi_req.rx req_rx,
  chi_rsp.tx rsp_tx,
  chi_dat.rx dat_rx,
  chi_dat.tx dat_tx,

  // AXI4 master (32-bit data)
  output logic [IDW-1:0]    m_axi_awid,
  output logic [AXI_AW-1:0] m_axi_awaddr,
  output logic [7:0]        m_axi_awlen,
  output logic [2:0]        m_axi_awsize,
  output logic [1:0]        m_axi_awburst,
  output logic [3:0]        m_axi_awcache,
  output logic [2:0]        m_axi_awprot,
  output logic              m_axi_awvalid,
  input  wire               m_axi_awready,
  output logic [31:0]       m_axi_wdata,
  output logic [3:0]        m_axi_wstrb,
  output logic              m_axi_wlast,
  output logic              m_axi_wvalid,
  input  wire               m_axi_wready,
  input  wire  [IDW-1:0]    m_axi_bid,
  input  wire  [1:0]        m_axi_bresp,
  input  wire               m_axi_bvalid,
  output logic              m_axi_bready,
  output logic [IDW-1:0]    m_axi_arid,
  output logic [AXI_AW-1:0] m_axi_araddr,
  output logic [7:0]        m_axi_arlen,
  output logic [2:0]        m_axi_arsize,
  output logic [1:0]        m_axi_arburst,
  output logic [3:0]        m_axi_arcache,
  output logic [2:0]        m_axi_arprot,
  output logic              m_axi_arvalid,
  input  wire               m_axi_arready,
  input  wire  [IDW-1:0]    m_axi_rid,
  input  wire  [31:0]       m_axi_rdata,
  input  wire  [1:0]        m_axi_rresp,
  input  wire               m_axi_rlast,
  input  wire               m_axi_rvalid,
  output logic              m_axi_rready
);
  localparam int QW = (REQ_DEPTH <= 1) ? 1 : $clog2(REQ_DEPTH);
  localparam int CW = $clog2(REQ_DEPTH + 16);

  initial begin
    if (DataWidth != 128)
      $fatal(1, "hni: DataWidth must be 128 (CHI beat = 4 AXI words)");
    if (NUM_TRK < 2 || (1 << TW) != NUM_TRK)
      $fatal(1, "hni: NUM_TRK (%0d) must be a power of two, at least 2", NUM_TRK);
  end

  // ------------------------------------------------------------ helpers
  function automatic logic [2:0] beats_of(width_e s);      // CHI DAT beats
    unique case (s)
      WIDTH_64: return 3'd4;
      WIDTH_32: return 3'd2;
      default:  return 3'd1;
    endcase
  endfunction

  function automatic logic [1:0] first_id(logic [5:0] a, width_e s);
    unique case (s)
      WIDTH_64: return 2'd0;
      WIDTH_32: return {a[5], 1'b0};
      default:  return a[5:4];
    endcase
  endfunction

  function automatic logic [6:0] bytes_of(width_e s);
    return 7'd1 << s;
  endfunction

  // AXI length (beats - 1) and size for a request
  function automatic logic [7:0] ax_len(width_e s);
    return (bytes_of(s) <= 7'd4) ? 8'd0 : 8'((bytes_of(s) >> 2) - 1);
  endfunction
  function automatic logic [2:0] ax_size(width_e s);
    return (bytes_of(s) <= 7'd4) ? 3'(s) : 3'd2;
  endfunction

  // AXI RESP -> CHI RespErr (SLVERR -> DERR, DECERR -> NDERR, OKAY / EXOKAY -> OK)
  function automatic logic [1:0] resperr_of(logic [1:0] r);
    return r[1] ? r : 2'b00;
  endfunction

  // ------------------------------------------------------------ TX link credits
  logic rsp_send, dat_send, rsp_have_cr, dat_have_cr;
  credit_cntr #(.MaxCredits(TxCreditMax)) u_rsp_cr (
    .clk(clk), .resetn(resetn), .incr(rsp_tx.lcrdv), .decr(rsp_send), .have_credit(rsp_have_cr));
  credit_cntr #(.MaxCredits(TxCreditMax)) u_dat_cr (
    .clk(clk), .resetn(resetn), .incr(dat_tx.lcrdv), .decr(dat_send), .have_credit(dat_have_cr));

  // write data is always taken in the cycle it arrives (a tracker waits for it)
  rx_credit_gen #(.MaxCredits(4)) u_wd_cr (
    .clk(clk), .resetn(resetn), .flitv(dat_rx.flitv == Y), .lcrdv(dat_rx.lcrdv));

  // ------------------------------------------------------------ request FIFO
  local_req_flit_t q [REQ_DEPTH];
  logic [QW-1:0]   q_wr, q_rd;
  logic [CW-1:0]   q_cnt, q_cnt_d, cr_out, cr_out_d;
  logic            push, pop;
  local_req_flit_t head;

  assign push     = (req_rx.flitv == Y);
  assign head     = q[q_rd];
  assign q_cnt_d  = q_cnt + CW'(push) - CW'(pop);
  assign cr_out_d = cr_out + CW'(req_rx.lcrdv) - CW'(push);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      q_wr <= '0; q_rd <= '0; q_cnt <= '0; cr_out <= '0;
      req_rx.lcrdv <= 1'b0;
    end else begin
      if (push) begin
        q[q_wr] <= local_req_flit_t'(req_rx.flit);
        q_wr    <= (q_wr == QW'(REQ_DEPTH - 1)) ? '0 : q_wr + 1'b1;
      end
      if (pop) q_rd <= (q_rd == QW'(REQ_DEPTH - 1)) ? '0 : q_rd + 1'b1;
      q_cnt        <= q_cnt_d;
      cr_out       <= cr_out_d;
      req_rx.lcrdv <= (32'(q_cnt_d) + 32'(cr_out_d) < REQ_DEPTH) && (32'(cr_out_d) < TxCreditMax);
    end
  end

  // ------------------------------------------------------------ trackers
  typedef enum logic [3:0] {
    T_FREE,
    T_AR,       // read: waiting for its turn (same-source order) and the AR port
    T_R,        // read: AR given to the AR port / sent, collecting R beats
    T_CDAT,     // read: CompData beats to send
    T_DBID,     // write: DBIDResp to send
    T_WDATA,    // write: waiting for the write data beats
    T_AW,       // write: waiting for its turn and the AW/W port
    T_B,        // write: AW/W given to the port / sent, waiting for B
    T_COMP      // write: Comp to send
  } tstate_e;

  typedef struct packed {
    tstate_e              st;
    node_id_e             srcid;
    logic [11:0]          txnid;
    logic [AddrWidth-1:0] addr;
    width_e               size;
    logic [NUM_TRK-1:0]   older;      // earlier requests of the same SrcID still open
    logic [2:0]           left;       // CHI beats still to send / receive
    logic [1:0]           dataid;     // next CompData DataID
    logic [3:0]           rbeat;      // AXI R beats received
    logic [1:0]           resperr;
    logic [511:0]         buf_;       // the line: read data / write data
    logic [63:0]          be;         // write byte enables
  } trk_t;

  trk_t t_q [NUM_TRK], t_d [NUM_TRK];

  logic [NUM_TRK-1:0] busy;
  always_comb for (int k = 0; k < NUM_TRK; k++) busy[k] = (t_q[k].st != T_FREE);

  // allocation: the FIFO head into the lowest free tracker
  logic          alloc;
  logic [TW-1:0] alloc_i;
  always_comb begin
    alloc   = 1'b0;
    alloc_i = '0;
    for (int k = NUM_TRK - 1; k >= 0; k--)
      if (!busy[k]) begin alloc = (q_cnt != '0); alloc_i = TW'(k); end
  end
  assign pop = alloc;

  // ---- AR port: one read burst at a time, payload held until the handshake
  logic          ar_v_q;
  logic [TW-1:0] ar_t_q;
  logic          ar_pick;
  logic [TW-1:0] ar_pick_i;
  always_comb begin
    ar_pick = 1'b0; ar_pick_i = '0;
    for (int k = NUM_TRK - 1; k >= 0; k--)
      if (t_q[k].st == T_AR && t_q[k].older == '0) begin ar_pick = !ar_v_q; ar_pick_i = TW'(k); end
  end

  // ---- AW/W port: one write burst at a time
  logic          aw_v_q, w_act_q;
  logic [TW-1:0] w_t_q;
  logic [3:0]    w_k_q;               // W beat number
  logic          w_pick;
  logic [TW-1:0] w_pick_i;
  always_comb begin
    w_pick = 1'b0; w_pick_i = '0;
    for (int k = NUM_TRK - 1; k >= 0; k--)
      if (t_q[k].st == T_AW && t_q[k].older == '0) begin w_pick = !aw_v_q && !w_act_q; w_pick_i = TW'(k); end
  end

  // ---- RSP TX: DBIDResp / Comp; DAT TX: CompData
  logic          rsp_any, dat_any;
  logic [TW-1:0] rsp_i, dat_i;
  always_comb begin
    rsp_any = 1'b0; rsp_i = '0; dat_any = 1'b0; dat_i = '0;
    for (int k = NUM_TRK - 1; k >= 0; k--) begin
      if (t_q[k].st == T_DBID || t_q[k].st == T_COMP) begin rsp_any = 1'b1; rsp_i = TW'(k); end
      if (t_q[k].st == T_CDAT)                        begin dat_any = 1'b1; dat_i = TW'(k); end
    end
  end
  assign rsp_send = rsp_any && rsp_have_cr;
  assign dat_send = dat_any && dat_have_cr;

  // ---- next state of every tracker
  local_dat_flit_t wd;
  assign wd = local_dat_flit_t'(dat_rx.flit);

  logic [TW-1:0] r_t, b_t, wd_t;
  assign r_t  = m_axi_rid[TW-1:0];
  assign b_t  = m_axi_bid[TW-1:0];
  assign wd_t = wd.txnid[TW-1:0];             // write data comes with TxnID = our DBID

  always_comb begin
    for (int k = 0; k < NUM_TRK; k++) begin
      t_d[k] = t_q[k];
      t_d[k].older = t_q[k].older & busy;      // a freed tracker no longer holds anyone back

      unique case (t_q[k].st)
        T_AR:    if (ar_pick && ar_pick_i == TW'(k)) t_d[k].st = T_R;
        T_R:     if (m_axi_rvalid && r_t == TW'(k)) begin
                   // the word goes to its place in the line (a narrow beat sits on its lanes)
                   t_d[k].buf_[{t_q[k].addr[5:2] + t_q[k].rbeat, 5'b0} +: 32] = m_axi_rdata;
                   t_d[k].rbeat   = t_q[k].rbeat + 1'b1;
                   t_d[k].resperr = t_q[k].resperr | resperr_of(m_axi_rresp);
                   if (m_axi_rlast) begin
                     t_d[k].st     = T_CDAT;
                     t_d[k].left   = beats_of(t_q[k].size);
                     t_d[k].dataid = first_id(t_q[k].addr[5:0], t_q[k].size);
                   end
                 end
        T_CDAT:  if (dat_send && dat_i == TW'(k)) begin
                   t_d[k].left   = t_q[k].left - 1'b1;
                   t_d[k].dataid = t_q[k].dataid + 1'b1;
                   if (t_q[k].left == 3'd1) t_d[k].st = T_FREE;
                 end
        T_DBID:  if (rsp_send && rsp_i == TW'(k)) t_d[k].st = T_WDATA;
        T_WDATA: if ((dat_rx.flitv == Y) && wd_t == TW'(k)) begin
                   t_d[k].buf_[wd.dataid*128 +: 128] = wd.data;
                   t_d[k].be  [wd.dataid*16  +: 16]  = wd.be;
                   t_d[k].left = t_q[k].left - 1'b1;
                   if (t_q[k].left == 3'd1) t_d[k].st = T_AW;
                 end
        T_AW:    if (w_pick && w_pick_i == TW'(k)) t_d[k].st = T_B;
        T_B:     if (m_axi_bvalid && b_t == TW'(k)) begin
                   t_d[k].resperr = resperr_of(m_axi_bresp);
                   t_d[k].st      = T_COMP;
                 end
        T_COMP:  if (rsp_send && rsp_i == TW'(k)) t_d[k].st = T_FREE;
        default: ;
      endcase

      if (alloc && alloc_i == TW'(k)) begin
        t_d[k]         = '0;
        t_d[k].srcid   = head.srcid;
        t_d[k].txnid   = head.txnid;
        t_d[k].addr    = AddrWidth'(head.addr);
        t_d[k].size    = head.size;
        for (int j = 0; j < NUM_TRK; j++)
          t_d[k].older[j] = busy[j] && (t_q[j].srcid == head.srcid);
        if (head.opcode == REQ_READ_NO_SNOOP) begin
          t_d[k].st = T_AR;
        end else begin
          t_d[k].st   = T_DBID;
          t_d[k].left = beats_of(head.size);
        end
      end
    end
  end

  // ---- AR / AW / W ports
  logic [5:0] w_off;                           // byte offset in the line of this W beat
  logic [3:0] w_lanes;                         // lanes a narrow beat may use
  always_comb begin
    m_axi_arid    = {t_q[ar_t_q].srcid[SRC_IDW-1:0], ar_t_q};
    m_axi_araddr  = AXI_AW'(t_q[ar_t_q].addr);
    m_axi_arlen   = ax_len(t_q[ar_t_q].size);
    m_axi_arsize  = ax_size(t_q[ar_t_q].size);
    m_axi_arburst = 2'b01;
    m_axi_arcache = 4'b0000;
    m_axi_arprot  = 3'b000;
    m_axi_arvalid = ar_v_q;
    m_axi_rready  = 1'b1;

    m_axi_awid    = {t_q[w_t_q].srcid[SRC_IDW-1:0], w_t_q};
    m_axi_awaddr  = AXI_AW'(t_q[w_t_q].addr);
    m_axi_awlen   = ax_len(t_q[w_t_q].size);
    m_axi_awsize  = ax_size(t_q[w_t_q].size);
    m_axi_awburst = 2'b01;
    m_axi_awcache = 4'b0000;
    m_axi_awprot  = 3'b000;
    m_axi_awvalid = aw_v_q;

    w_off   = {t_q[w_t_q].addr[5:2] + w_k_q, 2'b00};
    unique case (t_q[w_t_q].size)
      WIDTH_1: w_lanes = 4'b0001 << t_q[w_t_q].addr[1:0];
      WIDTH_2: w_lanes = 4'b0011 << {t_q[w_t_q].addr[1], 1'b0};
      default: w_lanes = 4'b1111;
    endcase
    m_axi_wdata  = t_q[w_t_q].buf_[{w_off, 3'b000} +: 32];
    m_axi_wstrb  = t_q[w_t_q].be[w_off +: 4] & w_lanes;
    m_axi_wlast  = (8'(w_k_q) == ax_len(t_q[w_t_q].size));
    m_axi_wvalid = w_act_q;
    m_axi_bready = 1'b1;
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int k = 0; k < NUM_TRK; k++) begin
        t_q[k]    <= '0;
        t_q[k].st <= T_FREE;
      end
      ar_v_q <= 1'b0; ar_t_q <= '0;
      aw_v_q <= 1'b0; w_act_q <= 1'b0; w_t_q <= '0; w_k_q <= '0;
    end else begin
      for (int k = 0; k < NUM_TRK; k++) t_q[k] <= t_d[k];
      // AR: take a read, hold it until the handshake
      if (ar_pick) begin ar_v_q <= 1'b1; ar_t_q <= ar_pick_i; end
      else if (ar_v_q && m_axi_arready) ar_v_q <= 1'b0;
      // AW/W: take a write; AW and the W beats go out side by side
      if (w_pick) begin
        aw_v_q <= 1'b1; w_act_q <= 1'b1; w_t_q <= w_pick_i; w_k_q <= '0;
      end else begin
        if (aw_v_q && m_axi_awready) aw_v_q <= 1'b0;
        if (w_act_q && m_axi_wready) begin
          w_k_q <= w_k_q + 1'b1;
          if (m_axi_wlast) w_act_q <= 1'b0;
        end
      end
    end
  end

  // ------------------------------------------------------------ TX flits
  local_dat_flit_t dat_f;
  local_rsp_flit_t rsp_f;
  always_comb begin
    dat_f         = '0;
    dat_f.tgtid   = t_q[dat_i].srcid;
    dat_f.srcid   = NODEID;
    dat_f.txnid   = t_q[dat_i].txnid;
    dat_f.homenid = NODEID;
    dat_f.opcode  = DAT_COMP_DATA;
    dat_f.resp    = RESP_I;
    dat_f.resperr = t_q[dat_i].resperr;
    dat_f.dbid    = 12'(dat_i);
    dat_f.dataid  = t_q[dat_i].dataid;
    dat_f.be      = '1;
    dat_f.data    = t_q[dat_i].buf_[t_q[dat_i].dataid*128 +: 128];

    rsp_f         = '0;
    rsp_f.tgtid   = t_q[rsp_i].srcid;
    rsp_f.srcid   = NODEID;
    rsp_f.txnid   = t_q[rsp_i].txnid;
    rsp_f.dbid    = 12'(rsp_i);
    if (t_q[rsp_i].st == T_COMP) begin
      rsp_f.opcode  = RSP_COMP;
      rsp_f.resperr = t_q[rsp_i].resperr;
    end else begin
      rsp_f.opcode  = RSP_DBID_RESP;
    end
  end

  assign dat_tx.flitv    = dat_send ? Y : N;
  assign dat_tx.flitpend = dat_send ? Y : N;
  assign dat_tx.flit     = dat_f;
  assign rsp_tx.flitv    = rsp_send ? Y : N;
  assign rsp_tx.flitpend = rsp_send ? Y : N;
  assign rsp_tx.flit     = rsp_f;

  // ------------------------------------------------------------ sim checks
`ifndef SYNTHESIS
  always @(posedge clk) begin
    if (resetn) begin
      if (alloc && head.opcode != REQ_READ_NO_SNOOP &&
          head.opcode != REQ_WRITE_NO_SNOOP_PTL && head.opcode != REQ_WRITE_NO_SNOOP_FULL)
        $error("hni: opcode %s (txnid=%0d from %0d) is not ReadNoSnp / WriteNoSnp*", head.opcode.name(), head.txnid, head.srcid);
      if (alloc && head.expcompack)
        $error("hni: %s with ExpCompAck from %0d (device accesses take no CompAck)", head.opcode.name(), head.srcid);
      if (alloc && (head.addr[5:0] & 6'(bytes_of(head.size) - 1)) != '0)
        $error("hni: %s addr=%0h not aligned to its size %0d B", head.opcode.name(), head.addr, bytes_of(head.size));
      if ((dat_rx.flitv == Y) && t_q[wd_t].st != T_WDATA)
        $error("hni: write data with TxnID %0d: tracker %0d is not waiting for data", wd.txnid, wd_t);
      if ((dat_rx.flitv == Y) && wd.opcode != DAT_NON_COPY_BACK_WR_DATA)
        $error("hni: write data opcode %s, expected NonCopyBackWrData", wd.opcode.name());
      if (m_axi_rvalid && t_q[r_t].st != T_R)
        $error("hni: R beat for ID %0h, tracker %0d is not reading", m_axi_rid, r_t);
      if (m_axi_bvalid && t_q[b_t].st != T_B)
        $error("hni: B for ID %0h, tracker %0d is not writing", m_axi_bid, b_t);
    end
  end
`endif

endmodule
