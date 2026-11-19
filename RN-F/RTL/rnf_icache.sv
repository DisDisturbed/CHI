`timescale 1ns/1ps
// =============================================================================
// rnf_icache - instruction cache of an RN-F node (rnf_node), non-coherent.
//
//  Core side : rnf_core_if (loads of one 128-bit chunk). One request per cycle
//              while no demand miss is open (ready from a flop).
//  Hit       : C0 (the accept cycle) reads the set's tags, valid bits and the
//              chunk of every way without a register (LUTRAM), compares and
//              registers the answer: rsp.valid in C1 (1 cycle).
//  Miss      : C0 only registers the miss (timing: the request -> compare ->
//              victim / fill allocation chain is cut there); in C1 a fill
//              buffer takes the line (victim way, valid bit cleared) and sends a
//              ReadOnce through rnf_dcache's I-side port (the node's one CHI
//              link, its TxnIDs, RetryAck / PCrdGrant handling). The demand
//              chunk is answered from the DAT beat that carries it; the line
//              becomes valid after the 4th beat.
//              Next-line prefetch: while the demand miss is open the next
//              line's set is probed (the core cannot look up then) and, if it
//              is missing, filled by the second fill buffer.
//  Coherence : none. ReadOnce gives a snapshot (the HN-F snoops a D-cache only
//              if one holds the line, e.g. code just written) and leaves no
//              snoop-filter entry, so this cache is never snooped. inv_i
//              (FENCE.I, when busy_o is low) invalidates every line, so code
//              written before a FENCE.I is fetched new.
//  Arrays    : per way LUTRAM tags [SETS] and data [SETS*4] x 128 bit, valid
//              bits and one round-robin victim pointer per set in flops.
// =============================================================================
module rnf_icache
  import chi_pkg::*;
  import common_pkg::*;
  import rnf_pkg::*;
#(
  parameter int AddrSize       = 39,
  parameter int MaxAccessWidth = 128,
  parameter int ReqIdxWidth    = 4,
  parameter int SETS           = 32,
  parameter int WAYS           = 2
) (
  input wire clk,
  input wire resetn,

  rnf_core_if.cache core,

  input  wire                 inv_i,           // FENCE.I: invalidate everything
  output logic                busy_o,          // a fill is open

  // to rnf_dcache's I-side port
  output logic                is_req_v_o,
  output logic                is_req_id_o,
  output logic [AddrSize-1:0] is_req_addr_o,
  input  wire                 is_req_ack_i,
  input  wire                 is_dat_v_i,
  input  wire                 is_dat_id_i,
  input  wire  [1:0]          is_dat_beat_i,
  input  wire  [127:0]        is_dat_i,
  input  wire                 is_dat_err_i
);
  localparam int SET_W = $clog2(SETS);
  localparam int WAY_W = (WAYS > 1) ? $clog2(WAYS) : 1;
  localparam int TAG_W = AddrSize - 6 - SET_W;
  localparam int LN_W  = AddrSize - 6;                 // line address

  initial begin
    if ((1 << SET_W) != SETS || SETS < 2 || WAYS < 2 || (1 << $clog2(WAYS)) != WAYS)
      $fatal(1, "rnf_icache: SETS (%0d) and WAYS (%0d) must be powers of two, at least 2", SETS, WAYS);
    if (MaxAccessWidth != 128)
      $fatal(1, "rnf_icache: MaxAccessWidth must be 128 (one DAT beat per fetch chunk)");
  end

  function automatic logic [SET_W-1:0] set_of(logic [LN_W-1:0] ln); return ln[SET_W-1:0]; endfunction
  function automatic logic [TAG_W-1:0] tag_of(logic [LN_W-1:0] ln); return ln[LN_W-1:SET_W]; endfunction

  // ------------------------------------------------------------------ arrays
  logic [SETS-1:0][WAYS-1:0] valid_q;
  logic [WAY_W-1:0]          rr_q [SETS];

  logic [SET_W-1:0]          rd_set;               // the read port (C0 lookup or prefetch probe)
  logic [1:0]                rd_beat;
  logic [WAYS-1:0][TAG_W-1:0] rd_tag;
  logic [WAYS-1:0][127:0]    rd_data;

  logic                      wr_v;                 // the write port (a fill beat)
  logic [WAY_W-1:0]          wr_way;
  logic [SET_W-1:0]          wr_set;
  logic [1:0]                wr_beat;
  logic                      wr_tag_v;             // first beat of a fill also writes the tag
  logic [TAG_W-1:0]          wr_tag;

  for (genvar w = 0; w < WAYS; w++) begin : gen_way
    (* ram_style = "distributed" *) logic [TAG_W-1:0] tag_mem  [SETS];
    (* ram_style = "distributed" *) logic [127:0]     data_mem [SETS * 4];
    always_ff @(posedge clk) begin
      if (wr_v && wr_way == WAY_W'(w)) begin
        data_mem[{wr_set, wr_beat}] <= is_dat_i;
        if (wr_tag_v) tag_mem[wr_set] <= wr_tag;
      end
    end
    assign rd_tag[w]  = tag_mem[rd_set];
    assign rd_data[w] = data_mem[{rd_set, rd_beat}];
  end

  // ------------------------------------------------------------------ fills
  typedef struct packed {
    logic             v;
    logic             req_pend;       // still to hand to rnf_dcache
    logic [LN_W-1:0]  line;
    logic [WAY_W-1:0] way;
    logic [3:0]       seen;
    logic             err;
    logic             kill;           // FENCE.I while open: do not validate
  } fill_t;
  fill_t f_q [2], f_d [2];

  // the open demand miss
  logic                    dem_v_q;
  logic                    dem_id_q;   // the fill that brings its line
  logic [1:0]              dem_beat_q;
  logic [ReqIdxWidth-1:0]  dem_idx_q;
  // a demand miss seen in C0 whose fill starts in C1
  logic                    miss_q;
  logic [LN_W-1:0]         miss_line_q;
  // next-line prefetch probe
  logic                    pf_v_q;
  logic [LN_W-1:0]         pf_line_q;

  // answer register
  logic                      rsp_v_q, rsp_err_q;
  logic [127:0]              rsp_data_q;
  logic [ReqIdxWidth-1:0]    rsp_idx_q;

  assign busy_o = f_q[0].v | f_q[1].v;

  // ------------------------------------------------------------------ C0 lookup
  logic             accept;
  logic [LN_W-1:0]  c_line;
  logic             c_hit;
  logic [WAY_W-1:0] c_way;
  logic             c_in_fill, c_fill_id;
  logic             c_now;                   // answered in C0: a hit, or its beat of an open fill is here
  logic [127:0]     c_data;

  // no new request while a demand miss is open, or while both fill buffers
  // are busy (a miss could not start a fill)
  logic rdy;
  assign rdy    = !dem_v_q && !(f_q[0].v && f_q[1].v);
  assign accept = (core.req.valid == Y) && rdy;
  assign c_line = core.req.addr[AddrSize-1:6];

  always_comb begin
    // the port: the core's request, or the prefetch probe while a demand is open
    rd_set  = dem_v_q ? set_of(pf_line_q) : set_of(c_line);
    rd_beat = core.req.addr[5:4];

    c_hit = 1'b0; c_way = '0;
    for (int w = WAYS - 1; w >= 0; w--)
      if (valid_q[rd_set][w] && rd_tag[w] == tag_of(dem_v_q ? pf_line_q : c_line)) begin
        c_hit = 1'b1; c_way = WAY_W'(w);
      end
    c_in_fill = 1'b0; c_fill_id = 1'b0;
    for (int k = 0; k < 2; k++)
      if (f_q[k].v && f_q[k].line == (dem_v_q ? pf_line_q : c_line)) begin
        c_in_fill = 1'b1; c_fill_id = 1'(k);
      end
    // a line still being filled: its beat may already be in the RAM, or arrive now
    c_now  = c_hit;
    c_data = rd_data[c_way];
    if (!c_hit && c_in_fill) begin
      if (f_q[c_fill_id].seen[rd_beat]) begin
        c_now = 1'b1; c_data = rd_data[f_q[c_fill_id].way];
      end else if (is_dat_v_i && is_dat_id_i == c_fill_id && is_dat_beat_i == rd_beat) begin
        c_now = 1'b1; c_data = is_dat_i;
      end
    end
  end

  // a victim way in a set: not one an open fill writes, an invalid one first,
  // else the round-robin pointer (moved past fills)
  function automatic logic [WAY_W-1:0] victim(logic [SET_W-1:0] s);
    logic [WAYS-1:0] busy;
    logic [WAY_W-1:0] v;
    busy = '0;
    for (int k = 0; k < 2; k++)
      if (f_q[k].v && set_of(f_q[k].line) == s) busy[f_q[k].way] = 1'b1;
    for (int w = WAYS - 1; w >= 0; w--)
      if (!valid_q[s][w] && !busy[w]) return WAY_W'(w);
    v = rr_q[s];
    for (int i = 0; i < WAYS; i++)
      if (busy[v]) v = v + 1'b1;
    return v;
  endfunction

  // ------------------------------------------------------------------ next state
  logic             start, start_id;          // a new fill this cycle
  logic [LN_W-1:0]  start_line;
  logic [WAY_W-1:0] start_way;
  logic             free0;

  always_comb begin
    for (int k = 0; k < 2; k++) f_d[k] = f_q[k];
    start = 1'b0; start_id = 1'b0; start_line = '0; start_way = '0;
    free0 = !f_q[0].v;

    // a demand miss registered last cycle (no open fill brings its line), or a
    // prefetch probe that misses (never in the same cycle: the probe follows
    // the demand's fill start)
    if (miss_q) begin
      start = 1'b1; start_line = miss_line_q;
    end else if (dem_v_q && pf_v_q && !c_hit && !c_in_fill && !(f_q[0].v && f_q[1].v)) begin
      start = 1'b1; start_line = pf_line_q;
    end
    start_id  = free0 ? 1'b0 : 1'b1;
    start_way = victim(set_of(start_line));
    if (start) begin
      f_d[start_id]          = '0;
      f_d[start_id].v        = 1'b1;
      f_d[start_id].req_pend = 1'b1;
      f_d[start_id].line     = start_line;
      f_d[start_id].way      = start_way;
    end

    // the request goes to rnf_dcache (one at a time, fill 0 first)
    is_req_v_o    = (f_q[0].v && f_q[0].req_pend) || (f_q[1].v && f_q[1].req_pend);
    is_req_id_o   = (f_q[0].v && f_q[0].req_pend) ? 1'b0 : 1'b1;
    is_req_addr_o = {f_q[is_req_id_o].line, 6'b0};
    if (is_req_ack_i) f_d[is_req_id_o].req_pend = 1'b0;

    // DAT beats
    wr_v = 1'b0; wr_way = f_q[is_dat_id_i].way; wr_set = set_of(f_q[is_dat_id_i].line);
    wr_beat = is_dat_beat_i; wr_tag_v = 1'b0; wr_tag = tag_of(f_q[is_dat_id_i].line);
    if (is_dat_v_i) begin
      wr_v     = 1'b1;
      wr_tag_v = (f_q[is_dat_id_i].seen == '0);
      f_d[is_dat_id_i].seen = f_q[is_dat_id_i].seen | (4'b0001 << is_dat_beat_i);
      if (is_dat_err_i) f_d[is_dat_id_i].err = 1'b1;
      if ((f_q[is_dat_id_i].seen | (4'b0001 << is_dat_beat_i)) == 4'hF) f_d[is_dat_id_i].v = 1'b0;
    end
    if (inv_i) for (int k = 0; k < 2; k++) f_d[k].kill = 1'b1;
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      valid_q    <= '0;
      for (int s = 0; s < SETS; s++) rr_q[s] <= '0;
      f_q[0]     <= '0;
      f_q[1]     <= '0;
      dem_v_q    <= 1'b0;
      dem_id_q   <= 1'b0;
      dem_beat_q <= '0;
      dem_idx_q  <= '0;
      miss_q     <= 1'b0;
      miss_line_q <= '0;
      pf_v_q     <= 1'b0;
      pf_line_q  <= '0;
      rsp_v_q    <= 1'b0;
      rsp_err_q  <= 1'b0;
      rsp_data_q <= '0;
      rsp_idx_q  <= '0;
    end else begin
      f_q[0] <= f_d[0];
      f_q[1] <= f_d[1];

      // a new fill: its way is invalid until the line is complete
      if (start) begin
        valid_q[set_of(start_line)][start_way] <= 1'b0;
        rr_q[set_of(start_line)] <= start_way + 1'b1;
      end
      // the last beat: the line is valid (unless an error or a FENCE.I)
      if (is_dat_v_i && (f_q[is_dat_id_i].seen | (4'b0001 << is_dat_beat_i)) == 4'hF &&
          !f_q[is_dat_id_i].err && !is_dat_err_i && !f_q[is_dat_id_i].kill && !inv_i)
        valid_q[set_of(f_q[is_dat_id_i].line)][f_q[is_dat_id_i].way] <= 1'b1;
      if (inv_i) valid_q <= '0;

      // answers: C0 (a hit, or the chunk of an open fill), or the demand chunk
      // from its DAT beat later
      rsp_v_q <= 1'b0;
      if (accept && c_now) begin
        rsp_v_q    <= 1'b1;
        rsp_err_q  <= (!c_hit && f_q[c_fill_id].err);
        rsp_data_q <= c_data;
        rsp_idx_q  <= core.req.idx;
      end
      if (dem_v_q && pf_v_q) pf_v_q <= 1'b0;       // the probe is one cycle
      if (accept && !c_now) begin
        dem_v_q    <= 1'b1;
        dem_beat_q <= core.req.addr[5:4];
        dem_idx_q  <= core.req.idx;
        if (c_in_fill) begin                       // an open fill brings it: probe next
          dem_id_q   <= c_fill_id;
          pf_v_q     <= 1'b1;
          pf_line_q  <= c_line + 1'b1;
        end else begin                             // a new fill, started in C1
          miss_q      <= 1'b1;
          miss_line_q <= c_line;
        end
      end
      // C1 of a miss: its fill starts now (start_id), the probe follows
      if (miss_q) begin
        miss_q    <= 1'b0;
        dem_id_q  <= start_id;
        pf_v_q    <= 1'b1;
        pf_line_q <= miss_line_q + 1'b1;
      end
      // the open demand: its beat was not there when it was accepted, so it comes later
      if (dem_v_q && !miss_q && is_dat_v_i && is_dat_id_i == dem_id_q && is_dat_beat_i == dem_beat_q) begin
        dem_v_q    <= 1'b0;
        rsp_v_q    <= 1'b1;
        rsp_err_q  <= is_dat_err_i;
        rsp_data_q <= is_dat_i;
        rsp_idx_q  <= dem_idx_q;
      end
    end
  end

  // synthesis translate_off
  always @(posedge clk)
    if (resetn && dem_v_q && !miss_q && !f_q[dem_id_q].v)
      $error("[%0t] rnf_icache: the open demand's fill %0d is gone before its beat came", $time, dem_id_q);
  // synthesis translate_on

  always_comb begin
    core.ready         = rdy ? Y : N;
    core.rsp           = '0;
    core.rsp.valid     = rsp_v_q ? Y : N;
    core.rsp.op        = MEM_OP_LOAD;
    core.rsp.data      = MaxAccessWidth'(rsp_data_q);
    core.rsp.idx       = rsp_idx_q;
    core.rsp.src       = MEM_UNIT_OTHER;
    core.rsp.err       = rsp_err_q ? Y : N;
    core.rsp.chr_state = CS_I;
    core.fast          = '0;
  end

endmodule
