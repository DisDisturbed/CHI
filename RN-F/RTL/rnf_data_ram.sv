`timescale 1ns/1ps
// rnf_data_ram - one way of the RN-F dcache data: DEPTH beats of 128 bit,
// true dual port (BRAM style), registered reads, byte-enable writes.
//   port A: the hit pipeline (P1 read of the request's beat, P2 store-hit write)
//   port B: the MSHRs and the snoop FSM (fills, write-back / snoop reads, ...)
// The dcache never accesses the same beat on both ports in one cycle.
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

  logic [127:0] mem [DEPTH];

  always_ff @(posedge clk) begin
    if (a_we_i)
      for (int b = 0; b < 16; b++)
        if (a_be_i[b]) mem[a_addr_i][b*8 +: 8] <= a_wdata_i[b*8 +: 8];
    if (a_re_i) a_rdata_o <= mem[a_addr_i];
  end

  always_ff @(posedge clk) begin
    if (b_we_i)
      for (int b = 0; b < 16; b++)
        if (b_be_i[b]) mem[b_addr_i][b*8 +: 8] <= b_wdata_i[b*8 +: 8];
    if (b_re_i) b_rdata_o <= mem[b_addr_i];
  end

endmodule
