
// HN_F - basic functional baseline
// for exactly NUM_RNF==2 - see the elaboration check below), no retry
// handling on a lost request-arbitration round. This is enough plumbing
// to run the existing scenario-style tests against multiple concurrent
// TSHR trackers.
module HN_F
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter int NUM_TRACKERS   = 4,
  parameter int NUM_RNF        = 2,
  localparam int RNF_IDX_W     = (NUM_RNF <= 1) ? 1 : $clog2(NUM_RNF),
  parameter int AddrWidth      = 39,
  parameter int DataWidth      = 128,
  parameter int CacheLineWidth = 512,
  parameter int TxCreditMax    = 15
) (
  input  wire clk,
  input  wire resetn,

  chi_req.rx rnf_req    [NUM_RNF], // RX: RN-F
  chi_snp.tx rnf_snp    [NUM_RNF], // TX: HN-F
  chi_rsp.rx rnf_rsp_rx [NUM_RNF], // RX: RN-F
  chi_rsp.tx rnf_rsp_tx [NUM_RNF], // TX: HN-F
  chi_dat.rx rnf_dat_rx [NUM_RNF], // RX: RN-F
  chi_dat.tx rnf_dat_tx [NUM_RNF], // TX: HN-F

  //assumed one SN-F or my brain is going to explode
  chi_req.tx snf_req,              // TX: HN-F
  chi_rsp.rx snf_rsp,              // RX: SN-F
  chi_dat.tx snf_dat_tx,           // TX: HN-F
  chi_dat.rx snf_dat_rx            // RX: SN-F
);

  initial begin
    if (NUM_RNF != 2) begin
      $fatal(1, "HN_F basic baseline only derives a snoop target for NUM_RNF==2 (no real snoop filter yet)");
    end
    if (TxCreditMax < 15) begin
      $fatal(1, "HN_F TxCreditMax (%0d) must be at least 15, the CHI maximum a receiver may grant, or credits are lost", TxCreditMax);
    end
    if (NUM_RNF > NUM_TRACKERS) begin
      $fatal(1, "HN_F NUM_RNF (%0d) must be < NUM_TRACKERS (%0d) or some requests will starve", NUM_RNF, NUM_TRACKERS);
    end
  end

  logic        tracker_ready [NUM_TRACKERS];
  logic        tracker_valid [NUM_TRACKERS]; // allocated to a live txn right now

  logic [11:0] trk_txnid     [NUM_TRACKERS];
  node_id_e    trk_srcid     [NUM_TRACKERS];
  logic [AddrWidth-1:0] trk_addr [NUM_TRACKERS];
  logic        trk_hazard    [NUM_TRACKERS];
  logic        trk_older_q   [NUM_TRACKERS][NUM_TRACKERS];

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) tracker_valid[t] = !tracker_ready[t];
  end

  // ---- REQ Channel (RX) ----
  common_pkg::yn_status_e         trk_req_flitv      [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_req_flitpend   [NUM_TRACKERS];
  tshr_flit_pkg::local_req_flit_t trk_req_flit       [NUM_TRACKERS];

  // ---- SNP Channel (TX) ----
  common_pkg::yn_status_e         trk_snp_flitv      [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_snp_flitpend   [NUM_TRACKERS];
  tshr_flit_pkg::local_snp_flit_t trk_snp_flit       [NUM_TRACKERS];
  logic                           trk_snp_gnt        [NUM_TRACKERS];

  // ---- RSP Channel (TX) ----
  common_pkg::yn_status_e         trk_rsp_tx_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_rsp_tx_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_rsp_flit_t trk_rsp_tx_flit    [NUM_TRACKERS];
  logic                           trk_rsp_tx_gnt     [NUM_TRACKERS];

  // ---- RSP Channel (RX from Requester) ----
  common_pkg::yn_status_e         trk_rsp_rx_req_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_rsp_rx_req_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_rsp_flit_t trk_rsp_rx_req_flit    [NUM_TRACKERS];

  // ---- RSP Channel (RX from Snoop Target) ----
  common_pkg::yn_status_e         trk_rsp_rx_snp_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_rsp_rx_snp_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_rsp_flit_t trk_rsp_rx_snp_flit    [NUM_TRACKERS];

  // ---- DAT Channel (TX) ----
  common_pkg::yn_status_e         trk_dat_tx_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_dat_tx_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_dat_flit_t trk_dat_tx_flit    [NUM_TRACKERS];
  logic                           trk_dat_tx_gnt     [NUM_TRACKERS];

  // ---- DAT Channel (RX from Requester) ----
  common_pkg::yn_status_e         trk_dat_rx_req_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_dat_rx_req_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_dat_flit_t trk_dat_rx_req_flit    [NUM_TRACKERS];

  // ---- DAT Channel (RX from Snoop Target) ----
  common_pkg::yn_status_e         trk_dat_rx_snp_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_dat_rx_snp_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_dat_flit_t trk_dat_rx_snp_flit    [NUM_TRACKERS];

  // ---- SN-F REQ Channel (TX) ----
  common_pkg::yn_status_e         trk_sn_req_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_sn_req_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_req_flit_t trk_sn_req_flit    [NUM_TRACKERS];
  logic                           trk_sn_req_gnt     [NUM_TRACKERS];

  // ---- SN-F RSP Channel (RX) ----
  common_pkg::yn_status_e         trk_sn_rsp_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_sn_rsp_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_rsp_flit_t trk_sn_rsp_flit    [NUM_TRACKERS];

  // ---- SN-F DAT Channel (TX) ----
  common_pkg::yn_status_e         trk_sn_dat_tx_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_sn_dat_tx_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_dat_flit_t trk_sn_dat_tx_flit    [NUM_TRACKERS];
  logic                           trk_sn_dat_tx_gnt     [NUM_TRACKERS];

  // ---- SN-F DAT Channel (RX) ----
  common_pkg::yn_status_e         trk_sn_dat_rx_flitv   [NUM_TRACKERS];
  common_pkg::yn_status_e         trk_sn_dat_rx_flitpend[NUM_TRACKERS];
  tshr_flit_pkg::local_dat_flit_t trk_sn_dat_rx_flit    [NUM_TRACKERS];

  logic [RNF_IDX_W-1:0] req_port_idx_q [NUM_TRACKERS]; // which RN-F is the requester
  logic [RNF_IDX_W-1:0] snp_port_idx   [NUM_TRACKERS]; // which RN-F is the snoop target (

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      snp_port_idx[t] = ~req_port_idx_q[t];
    end
  end

  common_pkg::yn_status_e         rnf_req_flitv_flat   [NUM_RNF];
  common_pkg::yn_status_e         rnf_req_flitpend_flat[NUM_RNF];
  tshr_flit_pkg::local_req_flit_t rnf_req_flit_flat    [NUM_RNF];
  logic                           rnf_req_lcrdv_flat   [NUM_RNF];

  logic [RNF_IDX_W-1:0]           alloc_port_idx [NUM_TRACKERS];
  logic                           alloc_drop;

  generate
    for (genvar r = 0; r < NUM_RNF; r++) begin : gen_req_bridge
      assign rnf_req_flitv_flat[r]    = rnf_req[r].flitv;
      assign rnf_req_flitpend_flat[r] = rnf_req[r].flitpend;
      assign rnf_req_flit_flat[r]     = local_req_flit_t'(rnf_req[r].flit);
      assign rnf_req[r].lcrdv         = rnf_req_lcrdv_flat[r];
    end
  endgenerate

  req_rx_arbiter #(
    .NUM_RNF      (NUM_RNF),
    .NUM_TRACKERS (NUM_TRACKERS)
  ) u_req_rx_arb (
    .clk                  (clk),
    .resetn               (resetn),
    .rnf_req_flitv        (rnf_req_flitv_flat),
    .rnf_req_lcrdv        (rnf_req_lcrdv_flat),
    .tracker_ready        (tracker_ready)
  );

  Allocator #(
    .NUM_TRACKERS(NUM_TRACKERS),
    .NUM_RNF     (NUM_RNF)
  ) u_allocator (
    .inbound_req_v_i    ( rnf_req_flitv_flat ),
    .inbound_req_flit_i ( rnf_req_flit_flat ),

    .tracker_ready_i    ( tracker_ready ),

    .alloc_v_o          ( trk_req_flitv ),
    .alloc_flit_o       ( trk_req_flit ),
    .alloc_port_o       ( alloc_port_idx ),
    .alloc_drop_o       ( alloc_drop )
  );

  always_ff @(posedge clk) begin
    if (resetn && alloc_drop) begin
      $error("[%0t] HN_F: a request arrived with no free tracker - REQ credit accounting is broken", $time);
    end
  end

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) trk_req_flitpend[t] = N;
  end

  // latch which physical RN-F port originated the request, the instant a
  // tracker is actually granted one.
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int t = 0; t < NUM_TRACKERS; t++) req_port_idx_q[t] <= '0;
    end else begin
      for (int t = 0; t < NUM_TRACKERS; t++) begin
        if (trk_req_flitv[t] == Y) req_port_idx_q[t] <= alloc_port_idx[t];
      end
    end
  end

  localparam int LineOffW = $clog2(CacheLineWidth / 8);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int a = 0; a < NUM_TRACKERS; a++)
        for (int b = 0; b < NUM_TRACKERS; b++) trk_older_q[a][b] <= 1'b0;
    end else begin
      for (int a = 0; a < NUM_TRACKERS; a++) begin
        for (int b = 0; b < NUM_TRACKERS; b++) begin
          if (trk_req_flitv[a] == Y) begin
            trk_older_q[a][b] <= (a != b) &&
                                 (tracker_valid[b] ||
                                  ((trk_req_flitv[b] == Y) && (alloc_port_idx[b] < alloc_port_idx[a])));
          end else if (tracker_ready[b]) begin
            trk_older_q[a][b] <= 1'b0;
          end
        end
      end
    end
  end

  always_comb begin
    for (int a = 0; a < NUM_TRACKERS; a++) begin
      trk_hazard[a] = 1'b0;
      for (int b = 0; b < NUM_TRACKERS; b++) begin
        if (trk_older_q[a][b] && tracker_valid[b] &&
            (trk_addr[a][AddrWidth-1:LineOffW] == trk_addr[b][AddrWidth-1:LineOffW])) begin
          trk_hazard[a] = 1'b1;
        end
      end
    end
  end

  // P-credit pool for SN-F retry: PCrdGrant carries no TxnID, so it is
  // banked here per PCrdType and handed to one waiting tracker.
  logic                 trk_pcrd_req  [NUM_TRACKERS];
  logic [3:0]           trk_pcrd_type [NUM_TRACKERS];
  logic                 trk_pcrd_gnt  [NUM_TRACKERS];
  logic [4:0]           pcrd_cnt_q    [16];
  logic [4:0]           pcrd_avail    [16];
  logic                 pcrd_arrive;
  logic [3:0]           pcrd_arrive_type;

  always_comb begin
    pcrd_arrive      = (snf_rsp.flitv == Y) && (snf_rsp.flit.opcode == RSP_PCR_GRANT_ACK);
    pcrd_arrive_type = snf_rsp.flit.pcrdttype;
    for (int k = 0; k < 16; k++) pcrd_avail[k] = pcrd_cnt_q[k];
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      trk_pcrd_gnt[t] = 1'b0;
      if (trk_pcrd_req[t] && (pcrd_avail[trk_pcrd_type[t]] != '0)) begin
        trk_pcrd_gnt[t]                = 1'b1;
        pcrd_avail[trk_pcrd_type[t]]   = pcrd_avail[trk_pcrd_type[t]] - 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int k = 0; k < 16; k++) pcrd_cnt_q[k] <= '0;
    end else begin
      for (int k = 0; k < 16; k++) begin
        pcrd_cnt_q[k] <= pcrd_avail[k] + 5'((pcrd_arrive && (pcrd_arrive_type == 4'(k))) ? 1 : 0);
      end
    end
  end

  genvar i;
  generate
    for (i = 0; i < NUM_TRACKERS; i = i + 1) begin : gen_tshr

      TSHR #(
        .AddrWidth      (AddrWidth),
        .DataWidth      (DataWidth),
        .CacheLineWidth (CacheLineWidth),
        .TRK_ID         (i),
        .HNFID          (7'h40),
        .SNFID          (7'h00)
      ) u_tshr_inst (
        .clk                 (clk),
        .resetn              (resetn),
        .ready_o             (tracker_ready[i]),

        // Exported nametags for the crossbar routers
        .TxnID_o             (trk_txnid[i]),
        .SrcID_o             (trk_srcid[i]),
        .Addr_o              (trk_addr[i]),
        .hazard_i            (trk_hazard[i]),

        // ---- REQ from Requester (rx side) ----
        .req_flitv           (trk_req_flitv[i]),
        .req_flitpend        (trk_req_flitpend[i]),
        .req_flit            (trk_req_flit[i]), // driven by Allocator

        // ---- SNP to Target (tx side) ----
        .snp_flitv           (trk_snp_flitv[i]),
        .snp_flitpend        (trk_snp_flitpend[i]),
        .snp_flit            (trk_snp_flit[i]),
        .snp_gnt             (trk_snp_gnt[i]),

        // ---- RSP to Requester (tx side) ----
        .rsp_tx_flitv        (trk_rsp_tx_flitv[i]),
        .rsp_tx_flitpend     (trk_rsp_tx_flitpend[i]),
        .rsp_tx_flit         (trk_rsp_tx_flit[i]),
        .rsp_tx_gnt          (trk_rsp_tx_gnt[i]),

        // ---- RSP from Requester (rx side) ----
        .rsp_rx_req_flitv    (trk_rsp_rx_req_flitv[i]),
        .rsp_rx_req_flitpend (trk_rsp_rx_req_flitpend[i]),
        .rsp_rx_req_flit     (trk_rsp_rx_req_flit[i]),

        // ---- RSP from Target (rx side) ----
        .rsp_rx_snp_flitv    (trk_rsp_rx_snp_flitv[i]),
        .rsp_rx_snp_flitpend (trk_rsp_rx_snp_flitpend[i]),
        .rsp_rx_snp_flit     (trk_rsp_rx_snp_flit[i]),

        // ---- DAT to Requester (tx side) ----
        .dat_tx_flitv        (trk_dat_tx_flitv[i]),
        .dat_tx_flitpend     (trk_dat_tx_flitpend[i]),
        .dat_tx_flit         (trk_dat_tx_flit[i]),
        .dat_tx_gnt          (trk_dat_tx_gnt[i]),

        // ---- DAT from Requester (rx side) ----
        .dat_rx_req_flitv    (trk_dat_rx_req_flitv[i]),
        .dat_rx_req_flitpend (trk_dat_rx_req_flitpend[i]),
        .dat_rx_req_flit     (trk_dat_rx_req_flit[i]),

        // ---- DAT from Target (rx side) ----
        .dat_rx_snp_flitv    (trk_dat_rx_snp_flitv[i]),
        .dat_rx_snp_flitpend (trk_dat_rx_snp_flitpend[i]),
        .dat_rx_snp_flit     (trk_dat_rx_snp_flit[i]),

        // ---- SN-F side ----
        .sn_req_flitv        (trk_sn_req_flitv[i]),
        .sn_req_flitpend     (trk_sn_req_flitpend[i]),
        .sn_req_flit         (trk_sn_req_flit[i]),
        .sn_req_gnt          (trk_sn_req_gnt[i]),

        .sn_rsp_flitv        (trk_sn_rsp_flitv[i]),
        .sn_rsp_flitpend     (trk_sn_rsp_flitpend[i]),
        .sn_rsp_flit         (trk_sn_rsp_flit[i]),

        .sn_dat_tx_flitv     (trk_sn_dat_tx_flitv[i]),
        .sn_dat_tx_flitpend  (trk_sn_dat_tx_flitpend[i]),
        .sn_dat_tx_flit      (trk_sn_dat_tx_flit[i]),
        .sn_dat_tx_gnt       (trk_sn_dat_tx_gnt[i]),

        .sn_dat_rx_flitv     (trk_sn_dat_rx_flitv[i]),
        .sn_dat_rx_flitpend  (trk_sn_dat_rx_flitpend[i]),
        .sn_dat_rx_flit      (trk_sn_dat_rx_flit[i]),

        .pcrd_req_o          (trk_pcrd_req[i]),
        .pcrd_type_o         (trk_pcrd_type[i]),
        .pcrd_gnt_i          (trk_pcrd_gnt[i])
      );

    end
  endgenerate
  common_pkg::yn_status_e snp_masked_flitv [NUM_RNF][NUM_TRACKERS];
  logic                   snp_targets_me   [NUM_RNF][NUM_TRACKERS];
  logic                   snp_port_gnt     [NUM_RNF][NUM_TRACKERS];

  common_pkg::yn_status_e rnf_snp_flitv_flat   [NUM_RNF];
  common_pkg::yn_status_e rnf_snp_flitpend_flat[NUM_RNF];
  local_snp_flit_t        rnf_snp_flit_flat    [NUM_RNF];

  generate
    for (genvar p = 0; p < NUM_RNF; p++) begin : gen_snp_port

      for (genvar t = 0; t < NUM_TRACKERS; t++) begin : gen_snp_mask
        assign snp_targets_me[p][t]   = tracker_valid[t] && (snp_port_idx[t] == p);
        assign snp_masked_flitv[p][t] = snp_targets_me[p][t] ? trk_snp_flitv[t] : N;
      end

      tx_chan_arbiter #(
        .NUM_TRACKERS (NUM_TRACKERS),
        .flit_t       (local_snp_flit_t),
        .MaxCredits   (TxCreditMax)
      ) u_snp_arb (
        .clk            (clk),
        .resetn         (resetn),
        .trk_flitv      (snp_masked_flitv[p]),
        .trk_flitpend   (trk_snp_flitpend),
        .trk_flit       (trk_snp_flit),
        .trk_targets_me (snp_targets_me[p]),
        .trk_gnt        (snp_port_gnt[p]),
        .phy_flitv      (rnf_snp_flitv_flat[p]),
        .phy_flitpend   (rnf_snp_flitpend_flat[p]),
        .phy_flit       (rnf_snp_flit_flat[p]),
        .phy_lcrdv      (rnf_snp[p].lcrdv)
      );

      assign rnf_snp[p].flitv    = rnf_snp_flitv_flat[p];
      assign rnf_snp[p].flitpend = rnf_snp_flitpend_flat[p];
//      assign rnf_snp[p].flit     = chi_snp::snp_flit_t'(rnf_snp_flit_flat[p]);
      assign rnf_snp[p].flit     = rnf_snp_flit_flat[p];
    end
  endgenerate

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      trk_snp_gnt[t] = 1'b0;
      for (int p = 0; p < NUM_RNF; p++) trk_snp_gnt[t] |= snp_port_gnt[p][t];
    end
  end

  // =====================================================================
  // RSP-TX and DAT-TX to the requester: same arbitration pattern, masked
  // by req_port_idx_q instead of snp_port_idx.
  // =====================================================================
  common_pkg::yn_status_e rsp_tx_masked_flitv [NUM_RNF][NUM_TRACKERS];
  logic                   rsp_tx_targets_me   [NUM_RNF][NUM_TRACKERS];
  logic                   rsp_tx_port_gnt     [NUM_RNF][NUM_TRACKERS];

  common_pkg::yn_status_e rnf_rsp_tx_flitv_flat   [NUM_RNF];
  common_pkg::yn_status_e rnf_rsp_tx_flitpend_flat[NUM_RNF];
  local_rsp_flit_t        rnf_rsp_tx_flit_flat    [NUM_RNF];

  common_pkg::yn_status_e dat_tx_masked_flitv [NUM_RNF][NUM_TRACKERS];
  logic                   dat_tx_targets_me   [NUM_RNF][NUM_TRACKERS];
  logic                   dat_tx_port_gnt     [NUM_RNF][NUM_TRACKERS];

  common_pkg::yn_status_e rnf_dat_tx_flitv_flat   [NUM_RNF];
  common_pkg::yn_status_e rnf_dat_tx_flitpend_flat[NUM_RNF];
  local_dat_flit_t        rnf_dat_tx_flit_flat    [NUM_RNF];

  generate
    for (genvar p = 0; p < NUM_RNF; p++) begin : gen_reqtgt_port

      for (genvar t = 0; t < NUM_TRACKERS; t++) begin : gen_reqtgt_mask
        assign rsp_tx_targets_me[p][t]   = tracker_valid[t] && (req_port_idx_q[t] == p);
        assign rsp_tx_masked_flitv[p][t] = rsp_tx_targets_me[p][t] ? trk_rsp_tx_flitv[t] : N;

        assign dat_tx_targets_me[p][t]   = tracker_valid[t] && (req_port_idx_q[t] == p);
        assign dat_tx_masked_flitv[p][t] = dat_tx_targets_me[p][t] ? trk_dat_tx_flitv[t] : N;
      end

      tx_chan_arbiter #(
        .NUM_TRACKERS (NUM_TRACKERS),
        .flit_t       (local_rsp_flit_t),
        .MaxCredits   (TxCreditMax)
      ) u_rsp_tx_arb (
        .clk            (clk),
        .resetn         (resetn),
        .trk_flitv      (rsp_tx_masked_flitv[p]),
        .trk_flitpend   (trk_rsp_tx_flitpend),
        .trk_flit       (trk_rsp_tx_flit),
        .trk_targets_me (rsp_tx_targets_me[p]),
        .trk_gnt        (rsp_tx_port_gnt[p]),
        .phy_flitv      (rnf_rsp_tx_flitv_flat[p]),
        .phy_flitpend   (rnf_rsp_tx_flitpend_flat[p]),
        .phy_flit       (rnf_rsp_tx_flit_flat[p]),
        .phy_lcrdv      (rnf_rsp_tx[p].lcrdv)
      );

      assign rnf_rsp_tx[p].flitv    = rnf_rsp_tx_flitv_flat[p];
      assign rnf_rsp_tx[p].flitpend = rnf_rsp_tx_flitpend_flat[p];
//      assign rnf_rsp_tx[p].flit     = chi_rsp::rsp_flit_t'(rnf_rsp_tx_flit_flat[p]);
      assign rnf_rsp_tx[p].flit     = rnf_rsp_tx_flit_flat[p];
      tx_chan_arbiter #(
        .NUM_TRACKERS (NUM_TRACKERS),
        .flit_t       (local_dat_flit_t),
        .MaxCredits   (TxCreditMax)
      ) u_dat_tx_arb (
        .clk            (clk),
        .resetn         (resetn),
        .trk_flitv      (dat_tx_masked_flitv[p]),
        .trk_flitpend   (trk_dat_tx_flitpend),
        .trk_flit       (trk_dat_tx_flit),
        .trk_targets_me (dat_tx_targets_me[p]),
        .trk_gnt        (dat_tx_port_gnt[p]),
        .phy_flitv      (rnf_dat_tx_flitv_flat[p]),
        .phy_flitpend   (rnf_dat_tx_flitpend_flat[p]),
        .phy_flit       (rnf_dat_tx_flit_flat[p]),
        .phy_lcrdv      (rnf_dat_tx[p].lcrdv)
      );

      assign rnf_dat_tx[p].flitv    = rnf_dat_tx_flitv_flat[p];
      assign rnf_dat_tx[p].flitpend = rnf_dat_tx_flitpend_flat[p];
//      assign rnf_dat_tx[p].flit     = chi_dat::dat_flit_t'(rnf_dat_tx_flit_flat[p]);
      assign rnf_dat_tx[p].flit     = rnf_dat_tx_flit_flat[p];
    end
  endgenerate

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      trk_rsp_tx_gnt[t] = 1'b0;
      trk_dat_tx_gnt[t] = 1'b0;
      for (int p = 0; p < NUM_RNF; p++) begin
        trk_rsp_tx_gnt[t] |= rsp_tx_port_gnt[p][t];
        trk_dat_tx_gnt[t] |= dat_tx_port_gnt[p][t];
      end
    end
  end

  // =====================================================================
  // SN-F TX path (single physical port each): every live tracker is a
  // candidate, no per-port masking needed since there's only one port.
  // =====================================================================
  logic sn_targets_me [NUM_TRACKERS];
  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) sn_targets_me[t] = tracker_valid[t];
  end

  common_pkg::yn_status_e sn_req_flitv_flat, sn_req_flitpend_flat;
  local_req_flit_t        sn_req_flit_flat;

  tx_chan_arbiter #(
    .NUM_TRACKERS (NUM_TRACKERS),
    .flit_t       (local_req_flit_t),
    .MaxCredits   (TxCreditMax)
  ) u_sn_req_arb (
    .clk            (clk),
    .resetn         (resetn),
    .trk_flitv      (trk_sn_req_flitv),
    .trk_flitpend   (trk_sn_req_flitpend),
    .trk_flit       (trk_sn_req_flit),
    .trk_targets_me (sn_targets_me),
    .trk_gnt        (trk_sn_req_gnt),
    .phy_flitv      (sn_req_flitv_flat),
    .phy_flitpend   (sn_req_flitpend_flat),
    .phy_flit       (sn_req_flit_flat),
    .phy_lcrdv      (snf_req.lcrdv)
  );

  assign snf_req.flitv    = sn_req_flitv_flat;
  assign snf_req.flitpend = sn_req_flitpend_flat;
//  assign snf_req.flit     = chi_req::req_flit_t'(sn_req_flit_flat);
   assign snf_req.flit     = sn_req_flit_flat;
  common_pkg::yn_status_e sn_dat_tx_flitv_flat, sn_dat_tx_flitpend_flat;
  local_dat_flit_t        sn_dat_tx_flit_flat;

  tx_chan_arbiter #(
    .NUM_TRACKERS (NUM_TRACKERS),
    .flit_t       (local_dat_flit_t),
    .MaxCredits   (TxCreditMax)
  ) u_sn_dat_tx_arb (
    .clk            (clk),
    .resetn         (resetn),
    .trk_flitv      (trk_sn_dat_tx_flitv),
    .trk_flitpend   (trk_sn_dat_tx_flitpend),
    .trk_flit       (trk_sn_dat_tx_flit),
    .trk_targets_me (sn_targets_me),
    .trk_gnt        (trk_sn_dat_tx_gnt),
    .phy_flitv      (sn_dat_tx_flitv_flat),
    .phy_flitpend   (sn_dat_tx_flitpend_flat),
    .phy_flit       (sn_dat_tx_flit_flat),
    .phy_lcrdv      (snf_dat_tx.lcrdv)
  );

  assign snf_dat_tx.flitv    = sn_dat_tx_flitv_flat;
  assign snf_dat_tx.flitpend = sn_dat_tx_flitpend_flat;
//  assign snf_dat_tx.flit     = chi_dat::dat_flit_t'(sn_dat_tx_flit_flat);
    assign snf_dat_tx.flit     = sn_dat_tx_flit_flat;

  common_pkg::yn_status_e rnf_rsp_rx_flitv_flat   [NUM_RNF];
  common_pkg::yn_status_e rnf_rsp_rx_flitpend_flat[NUM_RNF];
  local_rsp_flit_t        rnf_rsp_rx_flit_flat    [NUM_RNF];

  generate
    for (genvar p = 0; p < NUM_RNF; p++) begin : gen_rsp_rx_bridge
      assign rnf_rsp_rx_flitv_flat[p]    = rnf_rsp_rx[p].flitv;
      assign rnf_rsp_rx_flitpend_flat[p] = rnf_rsp_rx[p].flitpend;
      assign rnf_rsp_rx_flit_flat[p]     = local_rsp_flit_t'(rnf_rsp_rx[p].flit);

      rx_credit_gen u_rsp_rx_cred (
        .clk    (clk),
        .resetn (resetn),
        .flitv  (rnf_rsp_rx[p].flitv == Y),
        .lcrdv  (rnf_rsp_rx[p].lcrdv)
      );
    end
  endgenerate

  rx_port_select #(
    .NUM_TRACKERS (NUM_TRACKERS),
    .NUM_PORTS    (NUM_RNF),
    .flit_t       (local_rsp_flit_t)
  ) u_rsp_rx_req_sel (
    .phy_flitv         (rnf_rsp_rx_flitv_flat),
    .phy_flitpend      (rnf_rsp_rx_flitpend_flat),
    .phy_flit          (rnf_rsp_rx_flit_flat),
    .trk_target_idx    (req_port_idx_q),
    .trk_target_valid  (tracker_valid),
    .trk_flitv         (trk_rsp_rx_req_flitv),
    .trk_flitpend      (trk_rsp_rx_req_flitpend),
    .trk_flit          (trk_rsp_rx_req_flit)
  );

  rx_port_select #(
    .NUM_TRACKERS (NUM_TRACKERS),
    .NUM_PORTS    (NUM_RNF),
    .flit_t       (local_rsp_flit_t)
  ) u_rsp_rx_snp_sel (
    .phy_flitv         (rnf_rsp_rx_flitv_flat),
    .phy_flitpend      (rnf_rsp_rx_flitpend_flat),
    .phy_flit          (rnf_rsp_rx_flit_flat),
    .trk_target_idx    (snp_port_idx),
    .trk_target_valid  (tracker_valid),
    .trk_flitv         (trk_rsp_rx_snp_flitv),
    .trk_flitpend      (trk_rsp_rx_snp_flitpend),
    .trk_flit          (trk_rsp_rx_snp_flit)
  );

  common_pkg::yn_status_e rnf_dat_rx_flitv_flat   [NUM_RNF];
  common_pkg::yn_status_e rnf_dat_rx_flitpend_flat[NUM_RNF];
  local_dat_flit_t        rnf_dat_rx_flit_flat    [NUM_RNF];

  generate
    for (genvar p = 0; p < NUM_RNF; p++) begin : gen_dat_rx_bridge
      assign rnf_dat_rx_flitv_flat[p]    = rnf_dat_rx[p].flitv;
      assign rnf_dat_rx_flitpend_flat[p] = rnf_dat_rx[p].flitpend;
      assign rnf_dat_rx_flit_flat[p]     = local_dat_flit_t'(rnf_dat_rx[p].flit);

      rx_credit_gen u_dat_rx_cred (
        .clk    (clk),
        .resetn (resetn),
        .flitv  (rnf_dat_rx[p].flitv == Y),
        .lcrdv  (rnf_dat_rx[p].lcrdv)
      );
    end
  endgenerate

  rx_port_select #(
    .NUM_TRACKERS (NUM_TRACKERS),
    .NUM_PORTS    (NUM_RNF),
    .flit_t       (local_dat_flit_t)
  ) u_dat_rx_req_sel (
    .phy_flitv         (rnf_dat_rx_flitv_flat),
    .phy_flitpend      (rnf_dat_rx_flitpend_flat),
    .phy_flit          (rnf_dat_rx_flit_flat),
    .trk_target_idx    (req_port_idx_q),
    .trk_target_valid  (tracker_valid),
    .trk_flitv         (trk_dat_rx_req_flitv),
    .trk_flitpend      (trk_dat_rx_req_flitpend),
    .trk_flit          (trk_dat_rx_req_flit)
  );

  rx_port_select #(
    .NUM_TRACKERS (NUM_TRACKERS),
    .NUM_PORTS    (NUM_RNF),
    .flit_t       (local_dat_flit_t)
  ) u_dat_rx_snp_sel (
    .phy_flitv         (rnf_dat_rx_flitv_flat),
    .phy_flitpend      (rnf_dat_rx_flitpend_flat),
    .phy_flit          (rnf_dat_rx_flit_flat),
    .trk_target_idx    (snp_port_idx),
    .trk_target_valid  (tracker_valid),
    .trk_flitv         (trk_dat_rx_snp_flitv),
    .trk_flitpend      (trk_dat_rx_snp_flitpend),
    .trk_flit          (trk_dat_rx_snp_flit)
  );


  common_pkg::yn_status_e snf_rsp_flitv_flat, snf_rsp_flitpend_flat;
  local_rsp_flit_t        snf_rsp_flit_flat;
  assign snf_rsp_flitv_flat    = snf_rsp.flitv;
  assign snf_rsp_flitpend_flat = snf_rsp.flitpend;
  assign snf_rsp_flit_flat     = local_rsp_flit_t'(snf_rsp.flit);

  rx_credit_gen u_snf_rsp_cred (
    .clk    (clk),
    .resetn (resetn),
    .flitv  (snf_rsp.flitv == Y),
    .lcrdv  (snf_rsp.lcrdv)
  );

  common_pkg::yn_status_e snf_dat_rx_flitv_flat, snf_dat_rx_flitpend_flat;
  local_dat_flit_t        snf_dat_rx_flit_flat;
  assign snf_dat_rx_flitv_flat    = snf_dat_rx.flitv;
  assign snf_dat_rx_flitpend_flat = snf_dat_rx.flitpend;
  assign snf_dat_rx_flit_flat     = local_dat_flit_t'(snf_dat_rx.flit);

  rx_credit_gen u_snf_dat_rx_cred (
    .clk    (clk),
    .resetn (resetn),
    .flitv  (snf_dat_rx.flitv == Y),
    .lcrdv  (snf_dat_rx.lcrdv)
  );

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      trk_sn_rsp_flitv[t]    = snf_rsp_flitv_flat;
      trk_sn_rsp_flitpend[t] = snf_rsp_flitpend_flat;
      trk_sn_rsp_flit[t]     = snf_rsp_flit_flat;

      trk_sn_dat_rx_flitv[t]    = snf_dat_rx_flitv_flat;
      trk_sn_dat_rx_flitpend[t] = snf_dat_rx_flitpend_flat;
      trk_sn_dat_rx_flit[t]     = snf_dat_rx_flit_flat;
    end
  end

endmodule