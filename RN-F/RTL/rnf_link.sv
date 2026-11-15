`timescale 1ns/1ps
// CHI link layer blocks of the RN-F.

// Receive side of one channel: a FIFO with one slot per link credit. A credit
// is handed out for every free slot and comes back when the entry is popped.
module rnf_rx_fifo #(
  parameter int W     = 32,
  parameter int DEPTH = 4
) (
  input  wire          clk,
  input  wire          resetn,
  // link
  input  logic         flitv_i,
  input  logic [W-1:0] flit_i,
  output logic         lcrdv_o,
  // dcache
  output logic         valid_o,
  output logic [W-1:0] flit_o,
  input  logic         pop_i
);

  localparam int PW = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
  localparam int CW = $clog2(DEPTH + 1);

  logic [W-1:0]  mem [DEPTH];
  logic [PW-1:0] wr_q, rd_q;
  logic [CW-1:0] cnt_q;      // entries in the FIFO
  logic [CW-1:0] slots_q;    // entries + credits handed out and not used yet

  wire push = flitv_i && (cnt_q != CW'(DEPTH));
  wire pop  = pop_i && valid_o;

  assign valid_o = (cnt_q != '0);
  assign flit_o  = mem[rd_q];

  wire [CW-1:0] slots_d = slots_q + CW'(lcrdv_o) - CW'(pop);

  always_ff @(posedge clk) begin
    if (push) mem[wr_q] <= flit_i;
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      wr_q    <= '0;
      rd_q    <= '0;
      cnt_q   <= '0;
      slots_q <= '0;
      lcrdv_o <= 1'b0;
    end else begin
      if (push) wr_q <= (wr_q == PW'(DEPTH - 1)) ? '0 : wr_q + 1'b1;
      if (pop)  rd_q <= (rd_q == PW'(DEPTH - 1)) ? '0 : rd_q + 1'b1;
      cnt_q   <= cnt_q + CW'(push) - CW'(pop);
      slots_q <= slots_d;
      lcrdv_o <= (slots_d < CW'(DEPTH));
    end
  end

endmodule

// Transmit side of one channel: counts the credits the receiver granted and
// sends one registered flit per credit. FLITPEND is asserted the cycle before
// and during FLITV.
module rnf_tx_port #(
  parameter int W          = 32,
  parameter int MaxCredits = 15
) (
  input  wire          clk,
  input  wire          resetn,
  // dcache
  input  logic         valid_i,
  input  logic [W-1:0] flit_i,
  output logic         ready_o,
  // link
  input  logic         lcrdv_i,
  output logic         flitv_o,
  output logic         flitpend_o,
  output logic [W-1:0] flit_o
);

  localparam int CW = $clog2(MaxCredits + 1);

  logic [CW-1:0] credit_q;

  assign ready_o = (credit_q != '0);
  wire   send    = valid_i && ready_o;

  assign flitpend_o = send || flitv_o;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      credit_q <= '0;
      flitv_o  <= 1'b0;
      flit_o   <= '0;
    end else begin
      unique case ({lcrdv_i, send})
        2'b10:   if (credit_q < CW'(MaxCredits)) credit_q <= credit_q + 1'b1;
        2'b01:   credit_q <= credit_q - 1'b1;
        default: ;
      endcase
      flitv_o <= send;
      if (send) flit_o <= flit_i;
    end
  end

endmodule
