module req_rx_arbiter
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
#(
  parameter int NUM_RNF          = 2,
  parameter int NUM_TRACKERS     = 4,
  parameter int CreditsPerPort   = 1,
  parameter int PORT_IDXW        = (NUM_RNF <= 1) ? 1 : $clog2(NUM_RNF)
) (
  input  wire clk,
  input  wire resetn,

  input  common_pkg::yn_status_e rnf_req_flitv    [NUM_RNF],
  output logic                   rnf_req_lcrdv    [NUM_RNF],

  input  logic                   tracker_ready    [NUM_TRACKERS]
);

  localparam int CNTW = $clog2(NUM_TRACKERS + NUM_RNF * CreditsPerPort + 1);

  logic [CNTW-1:0]      port_cr_q [NUM_RNF];
  logic [PORT_IDXW-1:0] rr_ptr_q, rr_ptr_d;

  logic [CNTW-1:0] free_cnt;
  logic [CNTW-1:0] promised_cnt;
  logic [CNTW-1:0] avail;
  logic            any_grant;

  always_comb begin
    free_cnt     = '0;
    promised_cnt = '0;
    for (int t = 0; t < NUM_TRACKERS; t++) free_cnt     += CNTW'(tracker_ready[t]);
    for (int p = 0; p < NUM_RNF; p++)      promised_cnt += port_cr_q[p];
    avail = (free_cnt > promised_cnt) ? (free_cnt - promised_cnt) : '0;

    any_grant = 1'b0;
    for (int p = 0; p < NUM_RNF; p++) rnf_req_lcrdv[p] = 1'b0;
    for (int k = 0; k < NUM_RNF; k++) begin
      automatic int p = (rr_ptr_q + k) % NUM_RNF;
      if ((avail != '0) && (port_cr_q[p] < CNTW'(CreditsPerPort))) begin
        rnf_req_lcrdv[p] = 1'b1;
        avail            = avail - 1'b1;
        any_grant        = 1'b1;
      end
    end

    rr_ptr_d = rr_ptr_q;
    if (any_grant) rr_ptr_d = (rr_ptr_q == PORT_IDXW'(NUM_RNF-1)) ? '0 : (rr_ptr_q + 1'b1);
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      rr_ptr_q <= '0;
      for (int p = 0; p < NUM_RNF; p++) port_cr_q[p] <= '0;
    end else begin
      rr_ptr_q <= rr_ptr_d;
      for (int p = 0; p < NUM_RNF; p++) begin
        port_cr_q[p] <= port_cr_q[p] + CNTW'(rnf_req_lcrdv[p])
                        - CNTW'((rnf_req_flitv[p] == Y) && (port_cr_q[p] != '0));
      end
    end
  end

endmodule
