module tx_chan_arbiter
  import common_pkg::*;
#(
  parameter int  NUM_TRACKERS = 4,
  parameter type flit_t       = logic,
  parameter int  MaxCredits   = 15,
  localparam int IDXW         = (NUM_TRACKERS <= 1) ? 1 : $clog2(NUM_TRACKERS)
) (
  input  wire clk,
  input  wire resetn,

  input  common_pkg::yn_status_e trk_flitv       [NUM_TRACKERS],
  input  common_pkg::yn_status_e trk_flitpend    [NUM_TRACKERS],
  input  flit_t                  trk_flit        [NUM_TRACKERS],
  input  logic                   trk_targets_me  [NUM_TRACKERS],
  output logic                   trk_gnt         [NUM_TRACKERS],

  output common_pkg::yn_status_e phy_flitv,
  output common_pkg::yn_status_e phy_flitpend,
  output flit_t                  phy_flit,
  input  logic                   phy_lcrdv
);

  logic port_credit_have;

  credit_cntr #(.MaxCredits(MaxCredits)) u_port_cred (
    .clk         (clk),
    .resetn      (resetn),
    .incr        (phy_lcrdv),
    .decr        (phy_flitv == Y),
    .have_credit (port_credit_have)
  );

  logic [IDXW-1:0] send_rr_q, send_rr_d;
  logic            send_any;
  logic [IDXW-1:0] send_win;

  always_comb begin
    send_any = 1'b0;
    send_win = '0;
    for (int k = 0; k < NUM_TRACKERS; k++) begin
      automatic int t = (send_rr_q + k) % NUM_TRACKERS;
      if (port_credit_have && !send_any && trk_targets_me[t] && (trk_flitv[t] == Y)) begin
        send_any = 1'b1;
        send_win = t[IDXW-1:0];
      end
    end
    phy_flitv    = send_any ? trk_flitv[send_win]    : N;
    phy_flitpend = send_any ? trk_flitpend[send_win] : N;
    phy_flit     = send_any ? trk_flit[send_win]     : '0;
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      trk_gnt[t] = send_any && (t == int'(send_win));
    end
  end

  always_comb begin
    send_rr_d = send_rr_q;
    if (send_any) begin
      send_rr_d = (send_win == IDXW'(NUM_TRACKERS-1)) ? '0 : (send_win + 1'b1);
    end
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      send_rr_q <= '0;
    end else begin
      send_rr_q <= send_rr_d;
    end
  end

endmodule
