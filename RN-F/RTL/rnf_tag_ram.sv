`timescale 1ns/1ps
// rnf_tag_ram - tags + CHI line states of the RN-F dcache, in flops
//
// Two registered read ports and two write ports (flops, so ports are free):
//   port B (set_i / re_i / we_i ...): the MSHRs and the snoop FSM (one agent
//          per cycle, see the RAM port arbiter in rnf_dcache)
//   port A (set_a_i / re_a_i / we_a_i): the hit pipeline. Reads the whole set
//          (registered, P1 -> P2); writes only the state of one way (a store
//          hit on UC -> UD). The dcache never lets A and B write the same set
//          in one cycle.
module rnf_tag_ram #(
  parameter int SETS  = 16,
  parameter int WAYS  = 4,
  parameter int TAG_W = 29
) (
  input  wire                           clk,
  input  wire                           resetn,
  // port B
  input  logic [$clog2(SETS)-1:0]       set_i,
  input  logic                          re_i,
  input  logic                          we_i,
  input  logic [$clog2(WAYS)-1:0]       way_i,
  input  logic [TAG_W-1:0]              tag_i,
  input  chi_pkg::cache_state_e         state_i,
  output logic [WAYS-1:0][TAG_W-1:0]    tag_o,
  output chi_pkg::cache_state_e [WAYS-1:0] state_o,
  // port A
  input  logic [$clog2(SETS)-1:0]       set_a_i,
  input  logic                          re_a_i,
  output logic [WAYS-1:0][TAG_W-1:0]    tag_a_o,
  output chi_pkg::cache_state_e [WAYS-1:0] state_a_o,
  input  logic                          we_a_i,
  input  logic [$clog2(SETS)-1:0]       wset_a_i,
  input  logic [$clog2(WAYS)-1:0]       way_a_i,
  input  chi_pkg::cache_state_e         state_a_i
);
  import chi_pkg::*;

  logic [WAYS-1:0][TAG_W-1:0] tag_mem   [SETS];
  cache_state_e [WAYS-1:0]    state_mem [SETS];

  always_ff @(posedge clk) begin
    if (we_i)   tag_mem[set_i][way_i] <= tag_i;
    if (re_i)   tag_o   <= tag_mem[set_i];
    if (re_a_i) tag_a_o <= tag_mem[set_a_i];
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int s = 0; s < SETS; s++)
        for (int w = 0; w < WAYS; w++) state_mem[s][w] <= CS_I;
      for (int w = 0; w < WAYS; w++) begin
        state_o[w]   <= CS_I;
        state_a_o[w] <= CS_I;
      end
    end else begin
      if (we_i)   state_mem[set_i][way_i]     <= state_i;
      if (we_a_i) state_mem[wset_a_i][way_a_i] <= state_a_i;
      if (re_i)   state_o   <= state_mem[set_i];
      if (re_a_i) state_a_o <= state_mem[set_a_i];
    end
  end

endmodule
