module rx_credit_gen #(
  parameter int MaxCredits = 15
) (
  input  wire  clk,
  input  wire  resetn,
  input  logic flitv,
  output logic lcrdv
);

  localparam int CW = $clog2(MaxCredits + 2);

  logic [CW-1:0] outstanding_q, outstanding_d;

  always_comb begin
    outstanding_d = outstanding_q + CW'(lcrdv) - CW'(flitv && (outstanding_q != '0));
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      outstanding_q <= '0;
      lcrdv         <= 1'b0;
    end else begin
      outstanding_q <= outstanding_d;
      lcrdv         <= (outstanding_d < CW'(MaxCredits));
    end
  end

endmodule
