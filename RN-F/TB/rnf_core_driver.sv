`timescale 1ns/1ps
// =============================================================================
// rnf_core_driver - drives the core side of rnf_dcache like a core would:
// one load / store at a time, random address, width and data.
//
// It is an agent of the chi_tb_pkg environment, next to the behavioural
// rnf_model on the other RN-F port:
//   * commands come from the top through sb.cmd / sb.cmd_seq
//       CMD_RANDOM  sb.cmd_arg * OPS_MULT accesses
//       CMD_FLUSH   only joins the flush barrier (the dcache has no flush; the
//                   other RN-F cleans every line with CleanInvalid)
//   * a line is only accessed while sb.try_begin() allows it
//   * a store is an in-flight write from request to response (sb.wr_begin /
//     sb.wr_end); a load must return one of the values that were legal between
//     request and response (sb.rd_begin / sb.rd_end)
// Pins change 1 ns after the clock edge, like every other model.
// =============================================================================
module rnf_core_driver #(
  parameter string NAME           = "RNF0",
  parameter int    IDX            = 0,
  parameter int    SEED           = 1,
  parameter int    AddrSize       = 39,
  parameter int    MaxAccessWidth = 64,
  parameter int    ReqIdxWidth    = 4,
  parameter int    OPS_MULT       = 4,       // accesses per op of the behavioural RN-F
  parameter int    NC_PCT         = 10,      // non-cacheable accesses
  parameter int    GAP_MAX        = 6,       // idle cycles between accesses
  parameter int    OP_TIMEOUT     = 20000
) (
  input wire clk,
  input wire resetn,
  rnf_core_if.core core
);
  import chi_pkg::*;
  import common_pkg::*;
  import rnf_pkg::*;
  import chi_tb_pkg::*;

  tb_rng       rng;
  int unsigned cyc = 0;
  int unsigned idx_cnt = 0;
  int unsigned n_load = 0, n_store = 0, n_nc = 0, n_busy = 0;

  initial rng = new(SEED);

  always @(posedge clk) cyc <= cyc + 1;

  // Pin driver. The tasks set d_* and fire the event; this block applies them
  // 1 ns after the edge (a "<= #1" inside a task called from an initial block
  // would run as a blocking assignment in the Verilator flow).
  yn_status_e                d_valid = N, d_cacheable = N;
  mem_op_e                   d_op    = MEM_OP_LOAD;
  logic [AddrSize-1:0]       d_addr  = '0;
  width_e                    d_width = WIDTH_1;
  logic [MaxAccessWidth-1:0] d_data  = '0;
  logic [ReqIdxWidth-1:0]    d_idx   = '0;
  mem_unit_e                 d_src   = MEM_UNIT_LOAD;
  event                      drive_ev;

  always @(drive_ev) begin
    core.req.valid     <= #1 d_valid;
    core.req.kill      <= #1 N;
    core.req.op        <= #1 d_op;
    core.req.addr      <= #1 d_addr;
    core.req.width     <= #1 d_width;
    core.req.data      <= #1 d_data;
    core.req.idx       <= #1 d_idx;
    core.req.src       <= #1 d_src;
    core.req.cacheable <= #1 d_cacheable;
    core.req.chr_state <= #1 CS_I;
  end

  task automatic drive_idle();
    d_valid = N;
    -> drive_ev;
  endtask

  // one access; ran = 0 when the line was not free
  task automatic one_access(input int unsigned ep, output bit ran);
    addr_t       la, a;
    bit          store, nc, accepted, got, to;
    lockkind_e   lk;
    width_e      w;
    int          nbytes, off, k;
    int unsigned hn, t0;
    bit [63:0]   val, lmask;
    line_t       ld;
    bemask_t     lm;
    wr_pend      wp;
    rd_rec       rr;
    line_t       gotl;
    logic [ReqIdxWidth-1:0] idx;
    mem_unit_e   src;
    logic [MaxAccessWidth-1:0] rdata;
    bit          rerr;

    ran = 0;
    nc  = rng.chance(NC_PCT);
    if (nc) la = sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)];
    else begin
      hn = (sb.hot_n < sb.pool.size()) ? sb.hot_n : sb.pool.size();
      if (rng.chance(sb.hot_pct)) la = sb.pool[rng.range(0, hn - 1)];
      else                        la = sb.pool[rng.range(0, sb.pool.size() - 1)];
    end
    la    = line_of(la);
    store = rng.chance(50);
    lk    = store ? K_WRITE : K_READ;
    if (!sb.try_begin(la, IDX, lk, 0)) begin
      n_busy++;
      return;
    end
    ran = 1;

    w      = width_e'(rng.range(0, 3));
    nbytes = size_bytes(w);
    off    = rng.range(0, 63) & ~(nbytes - 1);
    a      = la + addr_t'(off);
    lmask  = (nbytes == 8) ? '1 : ((64'd1 << (nbytes * 8)) - 1);
    val    = rng.r64() & lmask;
    lm     = span_mask(a, w);
    ld     = line_t'(val) << (off * 8);
    idx    = ReqIdxWidth'(idx_cnt);
    idx_cnt++;
    src    = store ? MEM_UNIT_STORE : MEM_UNIT_LOAD;

    if (store) begin wp = sb.wr_begin(la, ld, lm, NAME); n_store++; end
    else       begin rr = sb.rd_begin(la, NAME);         n_load++;  end
    if (nc) n_nc++;

    d_valid     = Y;
    d_op        = store ? MEM_OP_STORE : MEM_OP_LOAD;
    d_addr      = AddrSize'(a);
    d_width     = w;
    d_data      = MaxAccessWidth'(val);
    d_idx       = idx;
    d_src       = src;
    d_cacheable = nc ? N : Y;
    -> drive_ev;

    // the request is on the pins at the next edge; the edge that sees ready accepts it
    t0 = cyc; accepted = 0; to = 0;
    while (!accepted && !to && ep == sb.epoch) begin
      @(posedge clk);
      if (core.ready == Y) accepted = 1;
      else if ((cyc - t0) > OP_TIMEOUT) to = 1;
    end
    drive_idle();

    got = 0; rdata = '0; rerr = 0;
    while (accepted && !got && !to && ep == sb.epoch) begin
      @(posedge clk);
      if (core.rsp.valid == Y) begin
        got   = 1;
        rdata = core.rsp.data;
        rerr  = (core.rsp.err == Y);
        if (core.rsp.idx !== idx || core.rsp.src !== src || core.rsp.op !== (store ? MEM_OP_STORE : MEM_OP_LOAD))
          sb.err("PROTOCOL", $sformatf("%s: response does not echo the request (idx %0d/%0d src %0d/%0d op %0d)", NAME,
                                       core.rsp.idx, idx, core.rsp.src, src, core.rsp.op));
      end else if ((cyc - t0) > OP_TIMEOUT) to = 1;
    end

    if (ep != sb.epoch) return;                 // reset: the scoreboard dropped everything

    if (to) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("%s: %s%s addr=%0h size=%0dB %s", NAME, nc ? "non-cacheable " : "", store ? "store" : "load", a, nbytes,
                                  accepted ? "got no response" : "was never accepted"));
      if (store) sb.wr_abort(la, wp);
      else       sb.rd_end(rr, '0, 1, "timeout");
      return;                                   // the line stays locked on purpose, like in rnf_model
    end

    if (rerr) begin
      sb.n_err_seen++;
      if (!sb.is_err_line(la)) sb.err("SPURIOUS_RESPERR", $sformatf("%s: %s addr=%0h returned err but no error was injected", NAME, store ? "store" : "load", a));
    end
    if (store) begin
      if (rerr) sb.wr_abort(la, wp); else sb.wr_end(la, wp, NAME);
    end else begin
      // the loaded bytes must match one of the legal line values
      gotl = '0; k = -1;
      if (!rr.skip)
        foreach (rr.allowed[i])
          if (k < 0 && ((rr.allowed[i] >> (off * 8)) & line_t'(lmask)) === line_t'(rdata)) k = i;
      if (!rr.skip && rr.allowed.size() != 0)
        gotl = merge_line(rr.allowed[(k < 0) ? 0 : k], line_t'(rdata) << (off * 8), lm);
      sb.rd_end(rr, gotl, rerr, $sformatf("%sload addr=%0h size=%0dB got=%0h", nc ? "non-cacheable " : "", a, nbytes, rdata));
    end
    sb.end_op(la, IDX, lk);
  endtask

  task automatic random_accesses(input int unsigned n, input int unsigned ep);
    int unsigned done;
    bit ran;
    done = 0;
    while (done < n && ep == sb.epoch) begin
      one_access(ep, ran);
      if (ran) done++;
      repeat (rng.range(ran ? 0 : 1, GAP_MAX)) @(posedge clk);
    end
  endtask

  // same barrier as rnf_model::sync
  task automatic barrier(input int n, input int unsigned ep);
    int unsigned g;
    g = sb.bar_gen;
    sb.bar_cnt++;
    if (sb.bar_cnt >= n) begin sb.bar_cnt = 0; sb.bar_gen++; end
    while (sb.bar_gen == g && ep == sb.epoch) @(posedge clk);
  endtask

  initial begin
    int unsigned last, ep;
    drive_idle();
    while (sb == null) @(posedge clk);
    last = 0;
    forever begin
      while (sb.cmd_seq == last) @(posedge clk);
      last = sb.cmd_seq;
      ep   = sb.epoch;
      case (sb.cmd)
        CMD_RANDOM: random_accesses(sb.cmd_arg * OPS_MULT, ep);
        CMD_FLUSH:  barrier(sb.num_rnf, ep);
        default: ;
      endcase
      if (ep != sb.epoch) drive_idle();
      sb.agents_done++;
    end
  end

endmodule
