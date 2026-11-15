`timescale 1ns/1ps
// Tag store of the RN-F dcache: one entry per set holding every way's tag and
// line state. Single port: a cycle is either a read of a whole set (result on
// the next cycle, held until the next read) or a write of one way.
// The tags have no reset; the line states reset to I.
module rnf_tag_ram #(
  parameter int SETS  = 16,
  parameter int WAYS  = 4,
  parameter int TAG_W = 29
) (
  input  wire                           clk,
  input  wire                           resetn,
  input  logic [$clog2(SETS)-1:0]       set_i,
  input  logic                          re_i,
  input  logic                          we_i,
  input  logic [$clog2(WAYS)-1:0]       way_i,
  input  logic [TAG_W-1:0]              tag_i,
  input  chi_pkg::cache_state_e         state_i,
  output logic [WAYS-1:0][TAG_W-1:0]    tag_o,
  output chi_pkg::cache_state_e [WAYS-1:0] state_o
);
  import chi_pkg::*;

  logic [WAYS-1:0][TAG_W-1:0] tag_mem   [SETS];
  cache_state_e [WAYS-1:0]    state_mem [SETS];

  always_ff @(posedge clk) begin
    if (we_i) tag_mem[set_i][way_i] <= tag_i;
    if (re_i) tag_o <= tag_mem[set_i];
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int s = 0; s < SETS; s++)
        for (int w = 0; w < WAYS; w++) state_mem[s][w] <= CS_I;
      for (int w = 0; w < WAYS; w++) state_o[w] <= CS_I;
    end else begin
      if (we_i) state_mem[set_i][way_i] <= state_i;
      if (re_i) state_o <= state_mem[set_i];
    end
  end

endmodule
