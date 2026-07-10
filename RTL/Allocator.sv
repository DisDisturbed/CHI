module Allocator
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter int NUM_TRACKERS = 4
) (
  input  common_pkg::yn_status_e inbound_req_v_i,
  input  local_req_flit_t        inbound_req_flit_i,
  output logic                   inbound_req_rdy_o,

  // Changed to ready, expecting 1 when free
  input  logic                   tracker_ready_i [NUM_TRACKERS],
  
  // Changed to yn_status_e to match TSHR input type
  output common_pkg::yn_status_e alloc_v_o [NUM_TRACKERS],
  output local_req_flit_t        alloc_flit_o 
);

  always_comb begin
    inbound_req_rdy_o = 1'b0;
    alloc_flit_o      = inbound_req_flit_i; 
    
    for (int i = 0; i < NUM_TRACKERS; i++) begin
      alloc_v_o[i] = N; 
    end

    for (int i = 0; i < NUM_TRACKERS; i++) begin
      if (tracker_ready_i[i] == 1'b1) begin // FIXED: 1 means free
        inbound_req_rdy_o = 1'b1; 
        
        if (inbound_req_v_i == Y) begin
          alloc_v_o[i] = Y; // FIXED: Output Y/N enum
        end
        
        break; 
      end
    end
  end

endmodule