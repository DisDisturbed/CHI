
module rx_port_select
  import common_pkg::*;
#(
  parameter int  NUM_TRACKERS = 4,
  parameter int  NUM_PORTS    = 2,
  parameter type flit_t       = logic,
  localparam int PORT_IDXW    = (NUM_PORTS <= 1) ? 1 : $clog2(NUM_PORTS)
) (

  input  common_pkg::yn_status_e phy_flitv    [NUM_PORTS],
  input  common_pkg::yn_status_e phy_flitpend [NUM_PORTS],
  input  flit_t                  phy_flit     [NUM_PORTS],
  output logic                   phy_lcrdv    [NUM_PORTS],


  input  logic [PORT_IDXW-1:0]   trk_target_idx   [NUM_TRACKERS],
  input  logic                   trk_target_valid [NUM_TRACKERS],


  output common_pkg::yn_status_e trk_flitv    [NUM_TRACKERS],
  output common_pkg::yn_status_e trk_flitpend [NUM_TRACKERS],
  output flit_t                  trk_flit     [NUM_TRACKERS]
);

  always_comb begin
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      if (trk_target_valid[t]) begin
        trk_flitv[t]    = phy_flitv   [trk_target_idx[t]];
        trk_flitpend[t] = phy_flitpend[trk_target_idx[t]];
        trk_flit[t]     = phy_flit    [trk_target_idx[t]];
      end else begin
        trk_flitv[t]    = N;
        trk_flitpend[t] = N;
        trk_flit[t]     = '0;
      end
    end
  end

  always_comb begin
    for (int p = 0; p < NUM_PORTS; p++) begin
      phy_lcrdv[p] = 1'b0;
      for (int t = 0; t < NUM_TRACKERS; t++) begin
        if (trk_target_valid[t] && (trk_target_idx[t] == p)) phy_lcrdv[p] = 1'b1;
      end
    end
  end

endmodule