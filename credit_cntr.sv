

module credit_cntr #(
  parameter int MaxCredits = 2
) (
  input  wire  clk,
  input  wire  resetn,
  input  logic incr,
  input  logic decr,
  output logic have_credit
);

  localparam int CW = $clog2(MaxCredits + 1);

  logic [CW-1:0] credit_q;

  assign have_credit = (credit_q != '0);

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      credit_q <= '0;
    end else begin
      unique case ({incr, decr})
        2'b10:   if (credit_q < CW'(MaxCredits)) credit_q <= credit_q + 1'b1; // accept
        2'b01:   credit_q <= credit_q - 1'b1;                                  // spend
        default: credit_q <= credit_q; // nothing
      endcase
    end
  end

endmodule