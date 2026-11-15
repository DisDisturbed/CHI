`timescale 1ns/1ps
// Data store of the RN-F dcache: 128-bit words (one CHI data beat) with byte
// write enables, addressed by {set, way, beat}. Single port, synchronous read:
// the word read is held until the next read.
module rnf_data_ram #(
  parameter int DEPTH = 256
) (
  input  wire                      clk,
  input  logic [$clog2(DEPTH)-1:0] addr_i,
  input  logic                     re_i,
  input  logic                     we_i,
  input  logic [15:0]              be_i,
  input  logic [127:0]             wdata_i,
  output logic [127:0]             rdata_o
);

  logic [127:0] mem [DEPTH];

  always_ff @(posedge clk) begin
    if (we_i)
      for (int b = 0; b < 16; b++)
        if (be_i[b]) mem[addr_i][b*8 +: 8] <= wdata_i[b*8 +: 8];
    if (re_i) rdata_o <= mem[addr_i];
  end

endmodule
