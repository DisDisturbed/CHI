module Allocator
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter int NUM_TRACKERS = 4,
  parameter int NUM_RNF      = 2,
  parameter int PORT_IDXW    = (NUM_RNF <= 1) ? 1 : $clog2(NUM_RNF)
) (
  input  common_pkg::yn_status_e inbound_req_v_i    [NUM_RNF],
  input  local_req_flit_t        inbound_req_flit_i [NUM_RNF],

  input  logic                   tracker_ready_i [NUM_TRACKERS],

  output common_pkg::yn_status_e alloc_v_o    [NUM_TRACKERS],
  output local_req_flit_t        alloc_flit_o [NUM_TRACKERS],
  output logic [PORT_IDXW-1:0]   alloc_port_o [NUM_TRACKERS],
  output logic                   alloc_drop_o
);

  logic taken [NUM_TRACKERS];
  logic placed;

  always_comb begin
    alloc_drop_o = 1'b0;
    for (int i = 0; i < NUM_TRACKERS; i++) begin
      alloc_v_o[i]    = N;
      alloc_flit_o[i] = '0;
      alloc_port_o[i] = '0;
      taken[i]        = 1'b0;
    end

    for (int p = 0; p < NUM_RNF; p++) begin
      placed = 1'b0;
      if (inbound_req_v_i[p] == Y) begin
        for (int i = 0; i < NUM_TRACKERS; i++) begin
          if (!placed && tracker_ready_i[i] && !taken[i]) begin
            taken[i]        = 1'b1;
            placed          = 1'b1;
            alloc_v_o[i]    = Y;
            alloc_flit_o[i] = inbound_req_flit_i[p];
            alloc_port_o[i] = PORT_IDXW'(p);
          end
        end
        if (!placed) alloc_drop_o = 1'b1;
      end
    end
  end

endmodule
