// ---------------------------------------------------------------------
// req_rx_arbiter
//
// Merges NUM_RNF physical request ports down into the single
// inbound_req_v/inbound_req_flit stream the Allocator expects, with
// round-robin fairness across RN-F ports. Also reports which physical
// port the granted request came from (inbound_req_port_idx) - HN_F needs
// this to remember, per allocated tracker, which RN-F is "the requester"
// for later response routing and snoop-target derivation.
//
// KNOWN LIMITATION (matches the original single-tracker TSHR's behavior,
// not new here): only one request is forwarded per cycle. A losing RN-F's
// request for that cycle is not queued or retried - it simply has to
// keep presenting flitv until it wins a later round. lcrdv is broadcast
// to every RN-F port unconditionally ("you may try"); whether it's
// actually consumed depends on the Allocator having a free tracker.
// ---------------------------------------------------------------------
module req_rx_arbiter
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter int NUM_RNF   = 2,
  parameter int PORT_IDXW = (NUM_RNF <= 1) ? 1 : $clog2(NUM_RNF)
) (
  input  wire clk,
  input  wire resetn,

  // physical RN-F request ports (already bridged to flat signals by HN_F)
  input  common_pkg::yn_status_e rnf_req_flitv    [NUM_RNF],
  input  common_pkg::yn_status_e rnf_req_flitpend [NUM_RNF],
  input  local_req_flit_t        rnf_req_flit     [NUM_RNF],
  output logic                   rnf_req_lcrdv    [NUM_RNF],

  // single merged stream toward the Allocator
  output common_pkg::yn_status_e inbound_req_v,
  output local_req_flit_t        inbound_req_flit,
  output logic [PORT_IDXW-1:0]   inbound_req_port_idx,

  // fed back from HN_F/Allocator: was the merged request actually
  // consumed (granted to a tracker) this cycle? Only then do we rotate
  // the round-robin pointer.
  input  logic                   inbound_req_taken
);

  logic [PORT_IDXW-1:0] rr_ptr_q, rr_ptr_d;

  always_comb begin
    inbound_req_v        = N;
    inbound_req_flit     = '0;
    inbound_req_port_idx = '0;

    // walk starting at rr_ptr_q, wrapping around; first flitv==Y found
    // (in rotation order) wins.
    for (int k = 0; k < NUM_RNF; k++) begin
      automatic int idx = (rr_ptr_q + k) % NUM_RNF;
      if ((inbound_req_v == N) && (rnf_req_flitv[idx] == Y)) begin
        inbound_req_v        = Y;
        inbound_req_flit     = rnf_req_flit[idx];
        inbound_req_port_idx = idx[PORT_IDXW-1:0];
      end
    end
  end

  // every RN-F is always invited to present a request; the round robin
  // above breaks ties on which one actually gets forwarded this cycle.
  always_comb begin
    for (int p = 0; p < NUM_RNF; p++) rnf_req_lcrdv[p] = 1'b1;
  end

  always_comb begin
    rr_ptr_d = rr_ptr_q;
    if (inbound_req_taken) begin
      rr_ptr_d = (inbound_req_port_idx == PORT_IDXW'(NUM_RNF-1)) ? '0 : (inbound_req_port_idx + 1'b1);
    end
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) rr_ptr_q <= '0;
    else         rr_ptr_q <= rr_ptr_d;
  end

endmodule