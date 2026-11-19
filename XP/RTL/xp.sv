// =============================================================================
// XP - crosspoint between the RN-Fs and the home nodes (HN-F, HN-I)
//
// Star topology: every RN-F and every home node has one CHI link to the XP;
// the SN-F stays on its own direct link to the HN-F. Flits are routed by
// TgtID (node ID tables RN_NID / HN_NID), so the home nodes see one link each
// and tell requesters apart by SrcID.
//
//   RN-F side (same names / directions as the old per-port HN-F):
//     rn_req    RN-F -> XP     REQ   -> the home node named by TgtID
//     rn_rsp_rx RN-F -> XP     RSP   (CompAck, SnpResp)
//     rn_dat_rx RN-F -> XP     DAT   (write data, SnpRespData)
//     rn_snp    XP -> RN-F     SNP
//     rn_rsp_tx XP -> RN-F     RSP   (Comp, DBIDResp, RetryAck, ...)
//     rn_dat_tx XP -> RN-F     DAT   (CompData)
//   home side:
//     hn_req    XP -> home     REQ
//     hn_rsp_tx XP -> home     RSP   (HN-F only: CompAck, SnpResp)
//     hn_dat_tx XP -> home     DAT
//     hn_snp    home -> XP     SNP   (HN-F only)
//     hn_rsp_rx home -> XP     RSP
//     hn_dat_rx home -> XP     DAT
//   HN_COH[h] = 1: a home with snoops (HN-F); 0: an HN-I (no SNP, no RSP in).
//   The unused links of an HN-I are left alone (lcrdv 0, flitv N).
//
// Every direction of every channel is one xp_switch:
//   * an input queue per sender (DEPTH entries). The XP gives the sender one
//     link credit per free entry, so a flit is never refused.
//   * an output port per receiver: link credits from the receiver, round-robin
//     over the senders that offer it a flit, one flit per cycle. A sender
//     offers its queue's head; with an empty queue (BYPASS = 1) it offers the
//     flit arriving this cycle, which then leaves the XP in the same cycle when
//     it wins (zero-cycle pass-through) and is queued only when it loses or
//     the output has no credit. BYPASS = 0: every flit is queued first (one
//     cycle per hop, no combinational path through the XP).
//   Different senders to one receiver interleave; flits of one sender to one
//     receiver stay in order (one FIFO, one output).
//   No deadlock from head-of-line blocking: every CHI receiver here sinks its
//   RSP / DAT / SNP without waiting on another channel, and a REQ waits only
//   for a home tracker that frees without any REQ moving.
// =============================================================================

module xp_switch
  import common_pkg::*;
#(
  parameter string NAME       = "xp",
  parameter type   flit_t     = logic,
  parameter int    NI         = 2,                 // senders
  parameter int    NO         = 1,                 // receivers
  parameter int    DEPTH      = 2,                 // input queue entries (= link credits per sender)
  parameter int    MaxCredits = 15,                // output link credits the XP can hold
  parameter bit    BYPASS     = 1'b1,              // an empty queue's arriving flit may pass in the same cycle
  parameter logic [chi_pkg::MAX_NODES-1:0] IN_EN  = '1,   // input i exists
  parameter logic [chi_pkg::MAX_NODES-1:0] OUT_EN = '1,   // output o exists
  parameter chi_pkg::nid_list_t           OUT_NID = chi_pkg::NID_SEQ1,  // node ID behind output o
  localparam int OW = (NO <= 1) ? 1 : $clog2(NO),
  localparam int IW = (NI <= 1) ? 1 : $clog2(NI),
  localparam int PW = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  input  wire        clk,
  input  wire        resetn,

  input  yn_status_e in_flitv     [NI],
  input  flit_t      in_flit      [NI],
  output logic       in_lcrdv     [NI],

  output yn_status_e out_flitv    [NO],
  output yn_status_e out_flitpend [NO],
  output flit_t      out_flit     [NO],
  input  logic       out_lcrdv    [NO]
);

  // observability (testbench credit checks): credits handed to sender i and not
  // used yet, flits queued from sender i, credits held for receiver o
  logic [7:0]    in_cr  [NI];
  logic [7:0]    in_cnt [NI];
  logic [7:0]    out_cr [NO];

  logic          head_v    [NI];
  flit_t         head_flit [NI];
  logic [OW-1:0] head_dst  [NI];
  logic          pop       [NI];       // the queue's head left
  logic          byp       [NI];       // the arriving flit left without being queued

  // what sender i offers the outputs this cycle: its queue's head, or (empty
  // queue, BYPASS) the flit arriving now
  logic          cand_v    [NI];
  flit_t         cand_flit [NI];
  logic [OW-1:0] cand_dst  [NI];
  logic          taken     [NI];       // an output took sender i's offer

  logic          gnt       [NO];
  logic [IW-1:0] gnt_in    [NO];

  // ---------------------------------------------------------------- input queues
  for (genvar i = 0; i < NI; i++) begin : gen_in
    if (IN_EN[i]) begin : q
      flit_t         mem [DEPTH];
      logic [OW-1:0] dst [DEPTH];
      logic [PW-1:0] wp_q, rp_q;
      logic [7:0]    cnt_q, cr_q, cnt_d, cr_d;
      logic          lcrdv_q;
      logic          arrive, push, route_ok;
      logic [OW-1:0] route;

      // destination of the arriving flit, kept with it in the queue
      always_comb begin
        route    = '0;
        route_ok = 1'b0;
        for (int o = 0; o < NO; o++) begin
          if (!route_ok && OUT_EN[o] && (in_flit[i].tgtid == OUT_NID[o])) begin
            route    = OW'(o);
            route_ok = 1'b1;
          end
        end
      end

      assign arrive = (in_flitv[i] == Y);
      // queued unless it passed straight through; a flit nobody can take is
      // dropped (its credit comes back)
      assign push   = arrive && route_ok && !byp[i];

      always_comb begin
        cr_d  = cr_q  + 8'(lcrdv_q) - 8'(arrive);
        cnt_d = cnt_q + 8'(push)    - 8'(pop[i]);
      end

      always_ff @(posedge clk) begin
        if (push) begin
          mem[wp_q] <= in_flit[i];
          dst[wp_q] <= route;
        end
      end

      always_ff @(posedge clk or negedge resetn) begin
        if (!resetn) begin
          wp_q    <= '0;
          rp_q    <= '0;
          cnt_q   <= '0;
          cr_q    <= '0;
          lcrdv_q <= 1'b0;
        end else begin
          if (push)   wp_q <= (wp_q == PW'(DEPTH - 1)) ? '0 : wp_q + 1'b1;
          if (pop[i]) rp_q <= (rp_q == PW'(DEPTH - 1)) ? '0 : rp_q + 1'b1;
          cnt_q   <= cnt_d;
          cr_q    <= cr_d;
          lcrdv_q <= (int'(cr_d) + int'(cnt_d)) < DEPTH;   // one credit per free entry
        end
      end

      assign in_lcrdv[i]  = lcrdv_q;
      assign head_v[i]    = (cnt_q != '0);
      assign head_flit[i] = mem[rp_q];
      assign head_dst[i]  = dst[rp_q];
      assign in_cr[i]     = cr_q;
      assign in_cnt[i]    = cnt_q;

      // the head first (order per sender); the arriving flit only past an empty queue
      assign cand_v[i]    = head_v[i] || (BYPASS && arrive && route_ok);
      assign cand_flit[i] = head_v[i] ? head_flit[i] : in_flit[i];
      assign cand_dst[i]  = head_v[i] ? head_dst[i]  : route;

`ifndef SYNTHESIS
      always @(posedge clk) begin
        if (resetn) begin
          if (arrive && (cr_q == '0))
            $error("[%0t] %s: input %0d sent a flit without a link credit", $time, NAME, i);
          if (arrive && !route_ok)
            $error("[%0t] %s: input %0d flit TgtID %0h has no route, dropped", $time, NAME, i, in_flit[i].tgtid);
        end
      end
`endif
    end else begin : q
      assign in_lcrdv[i]  = 1'b0;
      assign head_v[i]    = 1'b0;
      assign head_flit[i] = '0;
      assign head_dst[i]  = '0;
      assign in_cr[i]     = '0;
      assign in_cnt[i]    = '0;
      assign cand_v[i]    = 1'b0;
      assign cand_flit[i] = '0;
      assign cand_dst[i]  = '0;
    end
  end

  // ---------------------------------------------------------------- output ports
  for (genvar o = 0; o < NO; o++) begin : gen_out
    if (OUT_EN[o]) begin : a
      logic [7:0]    cr_q;
      logic [IW-1:0] rr_q, win;
      logic          any;

      always_comb begin
        any = 1'b0;
        win = '0;
        for (int k = 0; k < NI; k++) begin
          automatic int i = (int'(rr_q) + k) % NI;
          if (!any && (cr_q != '0) && cand_v[i] && (cand_dst[i] == OW'(o))) begin
            any = 1'b1;
            win = IW'(i);
          end
        end
      end

      always_ff @(posedge clk or negedge resetn) begin
        if (!resetn) begin
          cr_q <= '0;
          rr_q <= '0;
        end else begin
          // a credit beyond MaxCredits would be a receiver bug; it is not kept
          cr_q <= cr_q + 8'(out_lcrdv[o] && (cr_q < 8'(MaxCredits))) - 8'(any);
          if (any) rr_q <= (win == IW'(NI - 1)) ? '0 : win + 1'b1;
        end
      end

      assign out_flitv[o]    = any ? Y : N;
      assign out_flitpend[o] = any ? Y : N;
      assign out_flit[o]     = any ? cand_flit[win] : '0;
      assign gnt[o]          = any;
      assign gnt_in[o]       = win;
      assign out_cr[o]       = cr_q;
    end else begin : a
      assign out_flitv[o]    = N;
      assign out_flitpend[o] = N;
      assign out_flit[o]     = '0;
      assign gnt[o]          = 1'b0;
      assign gnt_in[o]       = '0;
      assign out_cr[o]       = '0;
    end
  end

  // an offer has one destination, so at most one output takes it
  always_comb begin
    for (int i = 0; i < NI; i++) taken[i] = 1'b0;
    for (int o = 0; o < NO; o++) if (gnt[o]) taken[gnt_in[o]] = 1'b1;
    for (int i = 0; i < NI; i++) begin
      pop[i] = taken[i] &&  head_v[i];
      byp[i] = taken[i] && !head_v[i];
    end
  end

endmodule


module xp
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter int        NRN         = 2,          // RN-Fs
  parameter int        NHN         = 1,          // home nodes (HN-F, HN-I)
  parameter nid_list_t RN_NID      = NID_SEQ1,   // node ID of RN-F port i
  parameter nid_list_t HN_NID      = {{(MAX_NODES - 1){7'h00}}, 7'h40},  // node ID of home port h
  parameter logic [MAX_NODES-1:0] HN_COH = '1,   // home h snoops (HN-F); 0 = HN-I
  parameter int        REQ_DEPTH   = 2,          // input queue entries per sender, per channel
  parameter int        RSP_DEPTH   = 2,
  parameter int        SNP_DEPTH   = 2,
  parameter int        DAT_DEPTH   = 4,          // 4: a 64 B line (4 beats) streams at one beat per cycle
  parameter bit        BYPASS      = 1'b1,       // zero-cycle pass-through of a flit into an empty queue
  parameter int        TxCreditMax = 15
) (
  input  wire clk,
  input  wire resetn,

  chi_req.rx rn_req    [NRN],
  chi_snp.tx rn_snp    [NRN],
  chi_rsp.rx rn_rsp_rx [NRN],
  chi_rsp.tx rn_rsp_tx [NRN],
  chi_dat.rx rn_dat_rx [NRN],
  chi_dat.tx rn_dat_tx [NRN],

  chi_req.tx hn_req    [NHN],
  chi_snp.rx hn_snp    [NHN],
  chi_rsp.tx hn_rsp_tx [NHN],
  chi_rsp.rx hn_rsp_rx [NHN],
  chi_dat.tx hn_dat_tx [NHN],
  chi_dat.rx hn_dat_rx [NHN]
);

  initial begin
    if (NRN < 1 || NRN > MAX_NODES || NHN < 1 || NHN > MAX_NODES)
      $fatal(1, "xp: NRN (%0d) and NHN (%0d) must be 1..%0d", NRN, NHN, MAX_NODES);
    for (int a = 0; a < NRN + NHN; a++)
      for (int b = a + 1; b < NRN + NHN; b++) begin
        automatic node_id_e na = (a < NRN) ? RN_NID[a] : HN_NID[a - NRN];
        automatic node_id_e nb = (b < NRN) ? RN_NID[b] : HN_NID[b - NRN];
        if (na == nb) $fatal(1, "xp: node ID %0h is used by two ports (%0d, %0d)", na, a, b);
      end
  end

  // ---------------------------------------------------------------- flat views
  yn_status_e      rq_iv [NRN]; local_req_flit_t rq_if [NRN]; logic rq_il [NRN];
  yn_status_e      rq_ov [NHN]; yn_status_e rq_op [NHN]; local_req_flit_t rq_of [NHN]; logic rq_ol [NHN];

  yn_status_e      sn_iv [NHN]; local_snp_flit_t sn_if [NHN]; logic sn_il [NHN];
  yn_status_e      sn_ov [NRN]; yn_status_e sn_op [NRN]; local_snp_flit_t sn_of [NRN]; logic sn_ol [NRN];

  yn_status_e      ru_iv [NRN]; local_rsp_flit_t ru_if [NRN]; logic ru_il [NRN];
  yn_status_e      ru_ov [NHN]; yn_status_e ru_op [NHN]; local_rsp_flit_t ru_of [NHN]; logic ru_ol [NHN];
  yn_status_e      rd_iv [NHN]; local_rsp_flit_t rd_if [NHN]; logic rd_il [NHN];
  yn_status_e      rd_ov [NRN]; yn_status_e rd_op [NRN]; local_rsp_flit_t rd_of [NRN]; logic rd_ol [NRN];

  yn_status_e      du_iv [NRN]; local_dat_flit_t du_if [NRN]; logic du_il [NRN];
  yn_status_e      du_ov [NHN]; yn_status_e du_op [NHN]; local_dat_flit_t du_of [NHN]; logic du_ol [NHN];
  yn_status_e      dd_iv [NHN]; local_dat_flit_t dd_if [NHN]; logic dd_il [NHN];
  yn_status_e      dd_ov [NRN]; yn_status_e dd_op [NRN]; local_dat_flit_t dd_of [NRN]; logic dd_ol [NRN];

  for (genvar i = 0; i < NRN; i++) begin : gen_rn
    assign rq_iv[i] = rn_req[i].flitv;     assign rq_if[i] = local_req_flit_t'(rn_req[i].flit);    assign rn_req[i].lcrdv    = rq_il[i];
    assign ru_iv[i] = rn_rsp_rx[i].flitv;  assign ru_if[i] = local_rsp_flit_t'(rn_rsp_rx[i].flit); assign rn_rsp_rx[i].lcrdv = ru_il[i];
    assign du_iv[i] = rn_dat_rx[i].flitv;  assign du_if[i] = local_dat_flit_t'(rn_dat_rx[i].flit); assign rn_dat_rx[i].lcrdv = du_il[i];

    assign rn_snp[i].flitv    = sn_ov[i];  assign rn_snp[i].flitpend    = sn_op[i];  assign rn_snp[i].flit    = sn_of[i];  assign sn_ol[i] = rn_snp[i].lcrdv;
    assign rn_rsp_tx[i].flitv = rd_ov[i];  assign rn_rsp_tx[i].flitpend = rd_op[i];  assign rn_rsp_tx[i].flit = rd_of[i];  assign rd_ol[i] = rn_rsp_tx[i].lcrdv;
    assign rn_dat_tx[i].flitv = dd_ov[i];  assign rn_dat_tx[i].flitpend = dd_op[i];  assign rn_dat_tx[i].flit = dd_of[i];  assign dd_ol[i] = rn_dat_tx[i].lcrdv;
  end

  for (genvar h = 0; h < NHN; h++) begin : gen_hn
    assign hn_req[h].flitv    = rq_ov[h];  assign hn_req[h].flitpend    = rq_op[h];  assign hn_req[h].flit    = rq_of[h];  assign rq_ol[h] = hn_req[h].lcrdv;
    assign hn_dat_tx[h].flitv = du_ov[h];  assign hn_dat_tx[h].flitpend = du_op[h];  assign hn_dat_tx[h].flit = du_of[h];  assign du_ol[h] = hn_dat_tx[h].lcrdv;
    assign rd_iv[h] = hn_rsp_rx[h].flitv;  assign rd_if[h] = local_rsp_flit_t'(hn_rsp_rx[h].flit); assign hn_rsp_rx[h].lcrdv = rd_il[h];
    assign dd_iv[h] = hn_dat_rx[h].flitv;  assign dd_if[h] = local_dat_flit_t'(hn_dat_rx[h].flit); assign hn_dat_rx[h].lcrdv = dd_il[h];
    if (HN_COH[h]) begin : coh
      assign sn_iv[h] = hn_snp[h].flitv;   assign sn_if[h] = local_snp_flit_t'(hn_snp[h].flit);    assign hn_snp[h].lcrdv    = sn_il[h];
      assign hn_rsp_tx[h].flitv = ru_ov[h];  assign hn_rsp_tx[h].flitpend = ru_op[h];  assign hn_rsp_tx[h].flit = ru_of[h];  assign ru_ol[h] = hn_rsp_tx[h].lcrdv;
    end else begin : io
      // HN-I: no snoops out, no RSP in
      assign sn_iv[h] = N;                 assign sn_if[h] = '0;                                   assign hn_snp[h].lcrdv    = 1'b0;
      assign hn_rsp_tx[h].flitv = N;       assign hn_rsp_tx[h].flitpend = N;       assign hn_rsp_tx[h].flit = '0;          assign ru_ol[h] = 1'b0;
    end
  end

  // ---------------------------------------------------------------- switches
  // RN-F -> home
  xp_switch #(.NAME("xp REQ"), .flit_t(local_req_flit_t), .NI(NRN), .NO(NHN), .DEPTH(REQ_DEPTH),
              .MaxCredits(TxCreditMax), .BYPASS(BYPASS), .OUT_NID(HN_NID)) u_req (
    .clk(clk), .resetn(resetn),
    .in_flitv(rq_iv), .in_flit(rq_if), .in_lcrdv(rq_il),
    .out_flitv(rq_ov), .out_flitpend(rq_op), .out_flit(rq_of), .out_lcrdv(rq_ol));

  xp_switch #(.NAME("xp RSP up"), .flit_t(local_rsp_flit_t), .NI(NRN), .NO(NHN), .DEPTH(RSP_DEPTH),
              .MaxCredits(TxCreditMax), .BYPASS(BYPASS), .OUT_EN(HN_COH), .OUT_NID(HN_NID)) u_rsp_up (
    .clk(clk), .resetn(resetn),
    .in_flitv(ru_iv), .in_flit(ru_if), .in_lcrdv(ru_il),
    .out_flitv(ru_ov), .out_flitpend(ru_op), .out_flit(ru_of), .out_lcrdv(ru_ol));

  xp_switch #(.NAME("xp DAT up"), .flit_t(local_dat_flit_t), .NI(NRN), .NO(NHN), .DEPTH(DAT_DEPTH),
              .MaxCredits(TxCreditMax), .BYPASS(BYPASS), .OUT_NID(HN_NID)) u_dat_up (
    .clk(clk), .resetn(resetn),
    .in_flitv(du_iv), .in_flit(du_if), .in_lcrdv(du_il),
    .out_flitv(du_ov), .out_flitpend(du_op), .out_flit(du_of), .out_lcrdv(du_ol));

  // home -> RN-F
  xp_switch #(.NAME("xp SNP"), .flit_t(local_snp_flit_t), .NI(NHN), .NO(NRN), .DEPTH(SNP_DEPTH),
              .MaxCredits(TxCreditMax), .BYPASS(BYPASS), .IN_EN(HN_COH), .OUT_NID(RN_NID)) u_snp (
    .clk(clk), .resetn(resetn),
    .in_flitv(sn_iv), .in_flit(sn_if), .in_lcrdv(sn_il),
    .out_flitv(sn_ov), .out_flitpend(sn_op), .out_flit(sn_of), .out_lcrdv(sn_ol));

  xp_switch #(.NAME("xp RSP down"), .flit_t(local_rsp_flit_t), .NI(NHN), .NO(NRN), .DEPTH(RSP_DEPTH),
              .MaxCredits(TxCreditMax), .BYPASS(BYPASS), .OUT_NID(RN_NID)) u_rsp_dn (
    .clk(clk), .resetn(resetn),
    .in_flitv(rd_iv), .in_flit(rd_if), .in_lcrdv(rd_il),
    .out_flitv(rd_ov), .out_flitpend(rd_op), .out_flit(rd_of), .out_lcrdv(rd_ol));

  xp_switch #(.NAME("xp DAT down"), .flit_t(local_dat_flit_t), .NI(NHN), .NO(NRN), .DEPTH(DAT_DEPTH),
              .MaxCredits(TxCreditMax), .BYPASS(BYPASS), .OUT_NID(RN_NID)) u_dat_dn (
    .clk(clk), .resetn(resetn),
    .in_flitv(dd_iv), .in_flit(dd_if), .in_lcrdv(dd_il),
    .out_flitv(dd_ov), .out_flitpend(dd_op), .out_flit(dd_of), .out_lcrdv(dd_ol));

endmodule
