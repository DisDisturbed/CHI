`timescale 1ns/1ps
// rnf_data_ram - one way of the RN-F dcache data: DEPTH beats of 128 bit.
//
// Distributed-RAM (LUTRAM) style, so the dcache needs no block RAM:
//   one write port (byte enables), two registered read ports.
//   port A: the hit pipeline (P1 read of the request's beat, P2 store-hit write)
//   port B: the MSHRs and the snoop FSM (fills, write-back / snoop reads, ...)
// Both ports may read in the same cycle. Only one of them may write in a cycle:
// the dcache arranges that (see "write port" in rnf_dcache); a_we wins if both.
module rnf_data_ram #(
  parameter int DEPTH = 64
) (
  input  wire                      clk,
  // port A
  input  logic [$clog2(DEPTH)-1:0] a_addr_i,
  input  logic                     a_re_i,
  input  logic                     a_we_i,
  input  logic [15:0]              a_be_i,
  input  logic [127:0]             a_wdata_i,
  output logic [127:0]             a_rdata_o,
  // port B
  input  logic [$clog2(DEPTH)-1:0] b_addr_i,
  input  logic                     b_re_i,
  input  logic                     b_we_i,
  input  logic [15:0]              b_be_i,
  input  logic [127:0]             b_wdata_i,
  output logic [127:0]             b_rdata_o
);

  (* ram_style = "distributed" *) logic [127:0] mem [DEPTH];

  // the one write port
  wire                     we    = a_we_i | b_we_i;
  wire [$clog2(DEPTH)-1:0] waddr = a_we_i ? a_addr_i  : b_addr_i;
  wire [15:0]              wbe   = a_we_i ? a_be_i    : b_be_i;
  wire [127:0]             wdata = a_we_i ? a_wdata_i : b_wdata_i;

  always_ff @(posedge clk) begin
    if (we)
      for (int b = 0; b < 16; b++)
        if (wbe[b]) mem[waddr][b*8 +: 8] <= wdata[b*8 +: 8];
  end

  // two read ports, registered
  always_ff @(posedge clk) begin
    if (a_re_i) a_rdata_o <= mem[a_addr_i];
    if (b_re_i) b_rdata_o <= mem[b_addr_i];
  end

  // synthesis translate_off
  always @(posedge clk)
    if (a_we_i && b_we_i) $error("rnf_data_ram: both ports write in one cycle");
  // synthesis translate_on

endmodule
