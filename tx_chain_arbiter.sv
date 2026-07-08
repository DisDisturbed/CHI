// ---------------------------------------------------------------------
// tx_chan_arbiter
//
// Arbitrates NUM_TRACKERS trackers wanting to drive ONE physical output
// channel (single flitv/flitpend/flit/lcrdv port) down to exactly one
// winner per cycle for the actual send, AND routes that one physical
// lcrdv pulse to exactly one tracker's own credit input per cycle -
// never to more than one at once, which is what keeps each tracker's
// internal credit_cntr from being double-credited off a single physical
// pulse.
//
// IMPORTANT: send-side and credit-side arbitration are two INDEPENDENT
// round robins, not the same one:
//   - send_rr picks a winner only among trackers with flitv==Y right now.
//   - cred_rr picks a winner among trackers merely *assigned* to this
//     port (trk_targets_me==1), regardless of whether they're sending
//     this exact cycle.
// If credit were only ever handed to a tracker already asserting
// flitv, a tracker with zero banked credit could never assert flitv (its
// own FSM gates flitv on already having credit) - it would never look
// like it "wants" credit, so it would never get any: permanent deadlock.
// Decoupling the two round robins is what avoids that.
//
// Fed by HN_F: the caller must pre-mask trk_flitv[t] to N for any
// tracker whose target port isn't this physical port, and must compute
// trk_targets_me[t] the same way (target port == this port AND tracker
// currently allocated). Only flitv needs masking - flitpend/flit are
// only ever read from whichever index wins, which masking already
// guarantees is a genuine target of this port.
// ---------------------------------------------------------------------
module tx_chan_arbiter
  import common_pkg::*;
#(
  parameter int  NUM_TRACKERS = 4,
  parameter type flit_t       = logic,
  localparam int IDXW         = (NUM_TRACKERS <= 1) ? 1 : $clog2(NUM_TRACKERS)
) (
  input  wire clk,
  input  wire resetn,

  // per-tracker candidates (flitv pre-masked by caller - see header note)
  input  common_pkg::yn_status_e trk_flitv       [NUM_TRACKERS],
  input  common_pkg::yn_status_e trk_flitpend    [NUM_TRACKERS],
  input  flit_t                  trk_flit        [NUM_TRACKERS],
  input  logic                   trk_targets_me  [NUM_TRACKERS],
  output logic                   trk_lcrdv       [NUM_TRACKERS],

  // the single physical output port
  output common_pkg::yn_status_e phy_flitv,
  output common_pkg::yn_status_e phy_flitpend,
  output flit_t                  phy_flit,
  input  logic                   phy_lcrdv
);

  // ---- send-side: round robin among trk_flitv==Y ----
  logic [IDXW-1:0] send_rr_q, send_rr_d;
  logic            send_any;
  logic [IDXW-1:0] send_win;

  always_comb begin
    send_any = 1'b0;
    send_win = '0;
    for (int k = 0; k < NUM_TRACKERS; k++) begin
      automatic int t = (send_rr_q + k) % NUM_TRACKERS;
      if (!send_any && (trk_flitv[t] == Y)) begin
        send_any = 1'b1;
        send_win = t[IDXW-1:0];
      end
    end
    phy_flitv    = send_any ? trk_flitv[send_win]    : N;
    phy_flitpend = send_any ? trk_flitpend[send_win] : N;
    phy_flit     = send_any ? trk_flit[send_win]     : '0;
  end

  always_comb begin
    send_rr_d = send_rr_q;
    if (send_any) begin
      send_rr_d = (send_win == IDXW'(NUM_TRACKERS-1)) ? '0 : (send_win + 1'b1);
    end
  end

  // ---- credit-side: independent round robin among trk_targets_me==1,
  // regardless of whether they're sending this cycle - lets a tracker
  // bank credit ahead of actually needing to send.
  logic [IDXW-1:0] cred_rr_q, cred_rr_d;
  logic            cred_any;
  logic [IDXW-1:0] cred_win;

  always_comb begin
    cred_any = 1'b0;
    cred_win = '0;
    for (int k = 0; k < NUM_TRACKERS; k++) begin
      automatic int t = (cred_rr_q + k) % NUM_TRACKERS;
      if (!cred_any && trk_targets_me[t]) begin
        cred_any = 1'b1;
        cred_win = t[IDXW-1:0];
      end
    end
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      trk_lcrdv[t] = (cred_any && (t == cred_win)) ? phy_lcrdv : 1'b0;
    end
  end

  always_comb begin
    cred_rr_d = cred_rr_q;
    if (phy_lcrdv && cred_any) begin
      cred_rr_d = (cred_win == IDXW'(NUM_TRACKERS-1)) ? '0 : (cred_win + 1'b1);
    end
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      send_rr_q <= '0;
      cred_rr_q <= '0;
    end else begin
      send_rr_q <= send_rr_d;
      cred_rr_q <= cred_rr_d;
    end
  end

endmodule