// snf - simple SN-F (memory subordinate) in RTL.
//
// Serves the HN-F's ReadNoSnp / WriteNoSnpFull / WriteNoSnpPtl from a flat
// memory array of 2**MEM_AW bytes, indexed by the low MEM_AW address bits. The
// system address map must make sure only the SN-F's own region reaches it
// (in WASP_SoC the bridge packs every memory window into one region).
//
// Deliberately simple:
//   * requests are taken into an in-order FIFO (REQ_DEPTH entries; a REQ credit
//     is only granted while the FIFO has room for it)
//   * one read FSM and one write FSM: one read and one write can be in flight at
//     once. A request waits while the other FSM works on the same cache line.
//   * fixed LATENCY cycles before every request is served
//   * read:  CompData beats, as many as the request size needs (64 B: 4 beats)
//   * write: DBIDResp (DBID always 0: only one write is in flight), the data
//            beats are merged into the line with their byte enables, then Comp.
//            Comp is sent only after the array was written, so a read that the
//            HN-F orders after the Comp always sees the new data.
//   * never sends RetryAck (every request is accepted), never reports an error
//
// Sim-only checks (not under SYNTHESIS): address outside the SN-F region,
// illegal opcode, write data that nothing waits for.
module snf
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter chi_pkg::node_id_e    NODEID         = 7'h00,
  parameter int                   AddrWidth      = 39,
  parameter int                   DataWidth      = 128,
  parameter int                   CacheLineWidth = 512,
  parameter int                   MEM_AW         = 20,      // memory bytes = 2**MEM_AW
  parameter logic [AddrWidth-1:0] MEM_BASE       = '0,      // region base, only for the sim check
  parameter int                   LATENCY        = 0,       // cycles before a request is served
  parameter int                   REQ_DEPTH      = 4,       // request FIFO entries (REQ credits)
  parameter int                   TxCreditMax    = 15
) (
  input  wire clk,
  input  wire resetn,
  chi_req.rx req_rx,
  chi_rsp.tx rsp_tx,
  chi_dat.rx dat_rx,
  chi_dat.tx dat_tx
);

  localparam int NB    = CacheLineWidth / DataWidth;   // beats per line
  localparam int LINES = 2 ** (MEM_AW - 6);
  localparam int LIW   = MEM_AW - 6;                   // line index width
  localparam int QW    = (REQ_DEPTH <= 1) ? 1 : $clog2(REQ_DEPTH);
  localparam int CW    = $clog2(REQ_DEPTH + 16);
  localparam int LW    = (LATENCY <= 1) ? 1 : $clog2(LATENCY + 1);

  initial begin
    if (DataWidth != 128 || CacheLineWidth != 512)
      $fatal(1, "snf: only DataWidth=128 and CacheLineWidth=512 are supported");
    if (MEM_AW < 7)
      $fatal(1, "snf: MEM_AW (%0d) must hold at least 2 lines", MEM_AW);
  end

  // ------------------------------------------------------------ memory
  logic [CacheLineWidth-1:0] mem [LINES];

  function automatic logic [LIW-1:0] line_idx(logic [AddrWidth-1:0] a);
    return a[MEM_AW-1:6];
  endfunction

  function automatic logic same_line(logic [AddrWidth-1:0] a, logic [AddrWidth-1:0] b);
    return a[AddrWidth-1:6] == b[AddrWidth-1:6];
  endfunction

  // beats and first DataID for a request size (as CHI: <=16 B one beat)
  function automatic logic [2:0] beats_of(width_e s);
    unique case (s)
      WIDTH_64: return 3'd4;
      WIDTH_32: return 3'd2;
      default:  return 3'd1;
    endcase
  endfunction

  function automatic logic [1:0] first_id(logic [AddrWidth-1:0] a, width_e s);
    unique case (s)
      WIDTH_64: return 2'd0;
      WIDTH_32: return {a[5], 1'b0};
      default:  return a[5:4];
    endcase
  endfunction

  // ------------------------------------------------------------ TX link credits
  logic rsp_send, dat_send, rsp_have_cr, dat_have_cr;

  credit_cntr #(.MaxCredits(TxCreditMax)) u_rsp_cr (
    .clk(clk), .resetn(resetn), .incr(rsp_tx.lcrdv), .decr(rsp_send), .have_credit(rsp_have_cr));
  credit_cntr #(.MaxCredits(TxCreditMax)) u_dat_cr (
    .clk(clk), .resetn(resetn), .incr(dat_tx.lcrdv), .decr(dat_send), .have_credit(dat_have_cr));

  // write data: always taken the cycle it arrives, so a fixed credit pool of one line
  rx_credit_gen #(.MaxCredits(NB)) u_wd_cr (
    .clk(clk), .resetn(resetn), .flitv(dat_rx.flitv == Y), .lcrdv(dat_rx.lcrdv));

  // ------------------------------------------------------------ request FIFO
  local_req_flit_t       q      [REQ_DEPTH];
  logic [QW-1:0]         q_wr, q_rd;
  logic [CW-1:0]         q_cnt, q_cnt_d;
  logic [CW-1:0]         cr_out, cr_out_d;   // REQ credits granted, not yet used
  logic                  push, pop;
  local_req_flit_t       head;
  logic                  head_rd, head_wr;

  assign push    = (req_rx.flitv == Y);
  assign head    = q[q_rd];
  assign head_rd = (head.opcode == REQ_READ_NO_SNOOP);
  assign head_wr = (head.opcode == REQ_WRITE_NO_SNOOP_FULL) || (head.opcode == REQ_WRITE_NO_SNOOP_PTL);

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
      // a new credit only if every credit out there still finds a free entry
      req_rx.lcrdv <= (32'(q_cnt_d) + 32'(cr_out_d) < REQ_DEPTH) && (32'(cr_out_d) < TxCreditMax);
    end
  end

  // ------------------------------------------------------------ read FSM
  typedef enum logic [1:0] { R_IDLE, R_WAIT, R_LOAD, R_SEND } r_state_e;
  r_state_e                  r_st;
  logic [11:0]               r_txnid;
  node_id_e                  r_srcid;
  logic [AddrWidth-1:0]      r_addr;
  width_e                    r_size;
  logic [LW-1:0]             r_cnt;
  logic [1:0]                r_beat;
  logic [2:0]                r_left;
  logic [CacheLineWidth-1:0] r_buf;

  // ------------------------------------------------------------ write FSM
  typedef enum logic [2:0] { W_IDLE, W_WAIT, W_DBID, W_DATA, W_WRITE, W_COMP } w_state_e;
  w_state_e                  w_st;
  logic [11:0]               w_txnid;
  node_id_e                  w_srcid;
  logic [AddrWidth-1:0]      w_addr;
  logic [LW-1:0]             w_cnt;
  logic [2:0]                w_left;
  logic [CacheLineWidth-1:0] w_buf;
  logic [CacheLineWidth/8-1:0] w_be;

  // dispatch from the FIFO head, in order
  logic go_rd, go_wr, go_bad;
  assign go_rd  = (q_cnt != '0) && head_rd && (r_st == R_IDLE) && !((w_st != W_IDLE) && same_line(head.addr, w_addr));
  assign go_wr  = (q_cnt != '0) && head_wr && (w_st == W_IDLE) && !((r_st != R_IDLE) && same_line(head.addr, r_addr));
  assign go_bad = (q_cnt != '0) && !head_rd && !head_wr;     // dropped (sim error below)
  assign pop    = go_rd || go_wr || go_bad;

  assign dat_send = (r_st == R_SEND) && dat_have_cr;
  assign rsp_send = ((w_st == W_DBID) || (w_st == W_COMP)) && rsp_have_cr;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      r_st <= R_IDLE; r_cnt <= '0; r_beat <= '0; r_left <= '0;
      r_txnid <= '0; r_srcid <= node_id_e'(0); r_addr <= '0; r_size <= WIDTH_64;
    end else begin
      unique case (r_st)
        R_IDLE: if (go_rd) begin
          r_txnid <= head.txnid;
          r_srcid <= head.srcid;
          r_addr  <= AddrWidth'(head.addr);
          r_size  <= head.size;
          r_cnt   <= LW'(LATENCY);
          r_st    <= R_WAIT;
        end
        R_WAIT: if (r_cnt == '0) r_st <= R_LOAD; else r_cnt <= r_cnt - 1'b1;
        R_LOAD: begin
          r_buf  <= mem[line_idx(r_addr)];
          r_beat <= first_id(r_addr, r_size);
          r_left <= beats_of(r_size);
          r_st   <= R_SEND;
        end
        R_SEND: if (dat_send) begin
          r_beat <= r_beat + 1'b1;
          r_left <= r_left - 1'b1;
          if (r_left == 3'd1) r_st <= R_IDLE;
        end
        default: r_st <= R_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      w_st <= W_IDLE; w_cnt <= '0; w_left <= '0; w_be <= '0;
      w_txnid <= '0; w_srcid <= node_id_e'(0); w_addr <= '0;
    end else begin
      unique case (w_st)
        W_IDLE: if (go_wr) begin
          w_txnid <= head.txnid;
          w_srcid <= head.srcid;
          w_addr  <= AddrWidth'(head.addr);
          w_left  <= beats_of(head.size);
          w_be    <= '0;
          w_cnt   <= LW'(LATENCY);
          w_st    <= W_WAIT;
        end
        W_WAIT:  if (w_cnt == '0) w_st <= W_DBID; else w_cnt <= w_cnt - 1'b1;
        W_DBID:  if (rsp_send) w_st <= W_DATA;
        W_DATA:  if (dat_rx.flitv == Y) begin
          w_buf[dat_rx.flit.dataid*DataWidth +: DataWidth]     <= dat_rx.flit.data;
          w_be [dat_rx.flit.dataid*(DataWidth/8) +: DataWidth/8] <= dat_rx.flit.be;
          w_left <= w_left - 1'b1;
          if (w_left == 3'd1) w_st <= W_WRITE;
        end
        W_WRITE: w_st <= W_COMP;
        W_COMP:  if (rsp_send) w_st <= W_IDLE;
        default: w_st <= W_IDLE;
      endcase
    end
  end

  // byte-masked line write (one write port; the read FSM has its own read port)
  logic w_commit;                       // the line is written this cycle
  assign w_commit = (w_st == W_WRITE);

  always_ff @(posedge clk) begin
    if (w_commit)
      for (int b = 0; b < CacheLineWidth / 8; b++)
        if (w_be[b]) mem[line_idx(w_addr)][b*8 +: 8] <= w_buf[b*8 +: 8];
  end

  // ------------------------------------------------------------ TX flits
  local_dat_flit_t dat_f;
  local_rsp_flit_t rsp_f;

  always_comb begin
    dat_f         = '0;
    dat_f.tgtid   = r_srcid;
    dat_f.srcid   = NODEID;
    dat_f.txnid   = r_txnid;
    dat_f.homenid = NODEID;
    dat_f.opcode  = DAT_COMP_DATA;
    dat_f.resp    = 3'b000;                       // I: memory keeps no state
    dat_f.dataid  = r_beat;
    dat_f.be      = '1;
    dat_f.data    = r_buf[r_beat*DataWidth +: DataWidth];

    rsp_f         = '0;
    rsp_f.tgtid   = w_srcid;
    rsp_f.srcid   = NODEID;
    rsp_f.txnid   = w_txnid;
    rsp_f.opcode  = (w_st == W_COMP) ? RSP_COMP : RSP_DBID_RESP;
    rsp_f.dbid    = 12'h000;
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
      if (push && (req_rx.flit.addr[AddrWidth-1:MEM_AW] != MEM_BASE[AddrWidth-1:MEM_AW]))
        $error("snf: addr %0h is outside the SN-F region %0h + 2**%0d bytes (it aliases onto line %0h)",
               req_rx.flit.addr, MEM_BASE, MEM_AW, req_rx.flit.addr[MEM_AW-1:6]);
      if (go_bad)
        $error("snf: opcode %s is not ReadNoSnp/WriteNoSnp* (txnid=%0d), dropped", head.opcode.name(), head.txnid);
      if ((dat_rx.flitv == Y) && (w_st != W_DATA))
        $error("snf: write data (txnid=%0d) arrived while no write waits for data", dat_rx.flit.txnid);
      if ((dat_rx.flitv == Y) && (w_st == W_DATA) && (dat_rx.flit.opcode != DAT_NON_COPY_BACK_WR_DATA))
        $error("snf: write data opcode %s, expected NonCopyBackWrData", dat_rx.flit.opcode.name());
    end
  end
`endif

endmodule
