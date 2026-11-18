`timescale 1ns/1ps
// rnf_tag_ram - tags + CHI line states of the RN-F dcache
//
// Distributed-RAM (LUTRAM) style: one tag array and one state array PER WAY
// (generate), each indexed by set only, so synthesis maps them to LUTRAM:
//   tags   : written only from port B (fills / evictions)
//   states : ONE write port shared by port B and port A (the dcache never lets
//            both write in one cycle; A wins if it happens)
//   valid  : one flop per (set, way), cleared at reset (LUTRAM has no reset):
//            an entry reads CS_I until it is written with a state other than I
// Two registered read ports, each returns the whole set:
//   port B (set_i / re_i / we_i ...): the MSHRs and the snoop FSM
//   port A (set_a_i / re_a_i / we_a_i): the hit pipeline; its write only
//          changes the state of one way (a store hit on UC -> UD)
//   tag_af_o / state_af_o: port A's read without the register (the set of
//          set_a_i in this cycle), for the dcache's fast load path. A write in
//          the same cycle is not seen.
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
  output logic [WAYS-1:0][TAG_W-1:0]    tag_af_o,    // unregistered port A read
  output chi_pkg::cache_state_e [WAYS-1:0] state_af_o,
  input  logic                          we_a_i,
  input  logic [$clog2(SETS)-1:0]       wset_a_i,
  input  logic [$clog2(WAYS)-1:0]       way_a_i,
  input  chi_pkg::cache_state_e         state_a_i
);
  import chi_pkg::*;

  localparam int SW = $bits(cache_state_e);
  localparam int SB = $clog2(SETS);

  // the state write port: A (store hit) or B
  wire          st_we  = we_a_i | we_i;
  wire [SB-1:0] st_set = we_a_i ? wset_a_i : set_i;
  wire [$clog2(WAYS)-1:0] st_way = we_a_i ? way_a_i : way_i;
  wire [SW-1:0] st_val = we_a_i ? SW'(state_a_i) : SW'(state_i);

  logic [SETS-1:0][WAYS-1:0] valid_q;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) valid_q <= '0;
    else if (st_we) valid_q[st_set][st_way] <= (cache_state_e'(st_val) != CS_I);
  end

  for (genvar w = 0; w < WAYS; w++) begin : gen_way
    (* ram_style = "distributed" *) logic [TAG_W-1:0] tag_mem [SETS];
    (* ram_style = "distributed" *) logic [SW-1:0]    st_mem  [SETS];

    always_ff @(posedge clk) begin
      if (we_i && way_i == w)   tag_mem[set_i]  <= tag_i;
      if (st_we && st_way == w) st_mem[st_set]  <= st_val;
    end

    // port A unregistered (fast load path)
    assign tag_af_o[w]   = tag_mem[set_a_i];
    assign state_af_o[w] = valid_q[set_a_i][w] ? cache_state_e'(st_mem[set_a_i]) : CS_I;

    // registered reads
    always_ff @(posedge clk) begin
      if (re_i)   tag_o[w]   <= tag_mem[set_i];
      if (re_a_i) tag_a_o[w] <= tag_mem[set_a_i];
    end

    always_ff @(posedge clk or negedge resetn) begin
      if (!resetn) begin
        state_o[w]   <= CS_I;
        state_a_o[w] <= CS_I;
      end else begin
        if (re_i)   state_o[w]   <= valid_q[set_i][w]   ? cache_state_e'(st_mem[set_i])   : CS_I;
        if (re_a_i) state_a_o[w] <= valid_q[set_a_i][w] ? cache_state_e'(st_mem[set_a_i]) : CS_I;
      end
    end
  end

  // synthesis translate_off
  always @(posedge clk)
    if (resetn && we_a_i && we_i) $error("rnf_tag_ram: both ports write a state in one cycle");
  // synthesis translate_on

endmodule
