`timescale 1ns/1ps
// =============================================================================
// rnf_fetch_driver - drives the I-cache port of the RN-F node (rnf_icache)
// like an in-order fetch unit, next to rnf_core_driver on the data port.
//
//  * Code: CODE_LINES lines at CODE_BASE that no agent ever writes, so every
//    fetched 128-bit chunk must be exactly the memory's initial value
//    (chi_tb_pkg::default_beat). CODE_LINES is 3x the I-cache's lines, so
//    lines are evicted and fetched again.
//  * Stream: a run of sequential chunks (crossing lines) from a random start,
//    then a jump; one request at a time (the core keeps one fetch in flight).
//  * FENCE.I: now and then, once the I-cache has no fill open (busy_i low),
//    inv_o for one cycle.
//  * Reset (resetn low) aborts the run; the driver starts again after it.
// Counts are read by the top for the report; failures go to the scoreboard.
// =============================================================================
module rnf_fetch_driver #(
  parameter int     SEED        = 1,
  parameter int     AddrSize    = 39,
  parameter int     ReqIdxWidth = 4,
  parameter int     CODE_LINES  = 192,
  parameter longint CODE_BASE   = 64'h20_0000_0000,
  parameter int     RUN_MAX     = 48,       // chunks per sequential run
  parameter int     INV_PCT     = 3,        // runs that start with a FENCE.I
  parameter int     TIMEOUT     = 20000
) (
  input  wire  clk,
  input  wire  resetn,
  input  wire  enable,
  rnf_core_if.core icore,
  output logic inv_o,
  input  wire  busy_i
);
  import chi_pkg::*;
  import common_pkg::*;
  import rnf_pkg::*;
  import chi_tb_pkg::*;

  tb_rng       rng;
  int unsigned cyc = 0;
  int unsigned n_fetch = 0, n_1cyc = 0, n_miss_lat = 0, n_inv = 0, n_bad = 0, n_runs = 0, n_err = 0;
  longint      lat_sum = 0;
  int unsigned lat_max = 0;

  always @(posedge clk) cyc <= cyc + 1;

  initial begin
    rng = new(SEED);
    icore.req   = '0;
    icore.req.valid = N;
    icore.la    = '0;
    icore.la.valid = N;
    inv_o       = 1'b0;
  end

  function automatic addr_t chunk_addr(int unsigned c);   // chunk c of the code region
    return addr_t'(CODE_BASE) + addr_t'((c % (CODE_LINES * 4)) * 16);
  endfunction

  task automatic fetch_one(input addr_t a, output bit ok);
    int unsigned t0, t_acc;
    logic [127:0] exp;
    ok = 0;
    exp = default_beat(line_of(a), int'(a[5:4]));
    @(negedge clk);
    icore.req.valid     = Y;
    icore.req.op        = MEM_OP_LOAD;
    icore.req.addr      = AddrSize'(a);
    icore.req.width     = WIDTH_16;
    icore.req.idx       = '0;
    icore.req.src       = MEM_UNIT_OTHER;
    icore.req.cacheable = Y;
    t0 = cyc;
    // accepted at the edge where ready is high
    forever begin
      @(posedge clk);
      if (!resetn) begin icore.req.valid = N; return; end
      if (icore.ready == Y) break;
      if (cyc - t0 > TIMEOUT) begin sb.err("TIMEOUT", $sformatf("FETCH: addr=%0h never accepted", a)); icore.req.valid = N; return; end
    end
    t_acc = cyc;
    @(negedge clk);
    icore.req.valid = N;
    // the answer
    if (icore.rsp.valid != Y)
      forever begin
        @(posedge clk);
        if (!resetn) return;
        if (cyc - t_acc > TIMEOUT) begin sb.err("TIMEOUT", $sformatf("FETCH: addr=%0h accepted, never answered", a)); return; end
        @(negedge clk);
        if (icore.rsp.valid == Y) break;
      end
    n_fetch++;
    if (cyc - t_acc <= 1) n_1cyc++; else begin n_miss_lat++; lat_sum += cyc - t_acc; end
    if (cyc - t_acc > lat_max) lat_max = cyc - t_acc;
    if (icore.rsp.err == Y) n_err++;                         // an injected read error (the line is not kept)
    else if (icore.rsp.data[127:0] !== exp) begin
      n_bad++;
      sb.err("READ_MISMATCH", $sformatf("FETCH: addr=%0h got %032h, memory holds %032h", a, icore.rsp.data[127:0], exp));
    end
    ok = 1;
  endtask

  initial begin
    bit ok;
    @(posedge clk);
    forever begin
      if (!resetn || !enable) begin @(posedge clk); continue; end
      n_runs++;
      if (rng.range(0, 99) < INV_PCT) begin                  // FENCE.I before this run
        while (busy_i && resetn) @(posedge clk);
        @(negedge clk); inv_o = 1'b1;
        @(negedge clk); inv_o = 1'b0;
        n_inv++;
      end
      begin
        int unsigned c0, len;
        c0  = rng.range(0, CODE_LINES * 4 - 1);
        len = rng.range(1, RUN_MAX);
        for (int unsigned k = 0; k < len; k++) begin
          fetch_one(chunk_addr(c0 + k), ok);
          if (!ok || !resetn || !enable) break;
        end
      end
      repeat (rng.range(0, 3)) @(posedge clk);
    end
  end

endmodule
