`timescale 1ns/1ps
// =============================================================================
// rnf_core_driver - drives the core side of rnf_dcache like a core would:
// one load / store at a time.
//
// It is an agent of the chi_tb_pkg environment, next to the behavioural
// rnf_model on the other RN-F port. Commands come from the top through
// sb.cmd / sb.cmd_seq:
//   CMD_RANDOM  sb.cmd_arg * OPS_MULT accesses, in random patterns: single
//               accesses with idle gaps, back-to-back runs, runs on one line,
//               store-then-load of the same bytes, walks over one set
//   CMD_REMOTE  directed scenario sb.cmd_arg (rnf_tb_pkg::RS_*). The driver
//               does its own accesses and orders the other RN-F's ops through
//               sb.rop_q, so the two can be lined up against each other
//   CMD_FLUSH   only joins the flush barrier (the dcache has no flush)
//
// Lines:
//   sb.pool     shared with the other RN-F. A store is an in-flight write from
//               request to response (sb.wr_begin / sb.wr_end); a load must
//               return one of the values that were legal in between
//   sb.nc_pool  non-cacheable, also shared
//   priv        only this driver touches them. Besides the scoreboard, the
//               driver keeps its own byte-exact copy (pline / pkn): a load must
//               return exactly the bytes stored last
//
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
  parameter int    PRIV_PCT       = 25,      // accesses to private lines
  parameter int    NUM_PRIV       = 12,      // private lines
  parameter int    GAP_MAX        = 6,       // idle cycles between single accesses
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
  import rnf_tb_pkg::*;

  tb_rng       rng;
  int unsigned cyc = 0;
  int unsigned idx_cnt = 0;
  int unsigned n_load = 0, n_store = 0, n_nc = 0, n_busy = 0, n_priv = 0, n_priv_chk = 0;

  // private lines and their reference copy
  addr_t       priv   [$];
  line_t       pline  [addr_t];
  bemask_t     pkn    [addr_t];          // bytes of pline that are known
  int unsigned ref_epoch = 0;
  bit          st_pend = 0;              // a store to a private line is in flight
  addr_t       st_pend_la;

  addr_t       sl [$];                   // lines of one set (set_lines)
  bit          scn_abort;

  initial begin
    rng = new(SEED);
    // same three sets as the shared pool, so private and shared lines replace each other
    for (int i = 0; i < NUM_PRIV; i++) begin
      addr_t a;
      a = (addr_t'(16'h0040 + i) << 16) | (addr_t'(int'(i % 3)) << 6);
      priv.push_back(a);
      pline[a] = default_line(a);
      pkn[a]   = '1;
    end
  end

  always @(posedge clk) cyc <= cyc + 1;

  function automatic bit is_priv(addr_t la);
    return pline.exists(la);
  endfunction

  // after a reset the cache is empty: what is in memory is the value
  function automatic void priv_sync();
    if (ref_epoch != sb.epoch) begin
      foreach (priv[i]) begin
        pline[priv[i]] = sb.snf_line(priv[i]);
        pkn[priv[i]]   = '1;
      end
      if (st_pend) pkn[st_pend_la] = '0;
      st_pend   = 0;
      ref_epoch = sb.epoch;
    end
  endfunction

  // ---------------------------------------------------------------- pin driver
  // The tasks set d_* and fire the event; this block applies them 1 ns after
  // the edge (a "<= #1" inside a task called from an initial block would run
  // as a blocking assignment in the Verilator flow).
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

  // ---------------------------------------------------------------- one access
  // w_sel / off_sel < 0: random width / offset. use_val = 0: random store data.
  // ran = 0 when the line was not free (nothing was driven).
  task automatic access(input addr_t la_in, input bit store, input bit nc,
                        input int w_sel, input int off_sel, input bit use_val, input bit [63:0] val_in,
                        input int unsigned ep, output bit ran);
    addr_t       la, a;
    bit          accepted, got, to, prv;
    lockkind_e   lk;
    width_e      w;
    int          nbytes, off, k;
    int unsigned t0;
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
    if (ep != sb.epoch) return;
    priv_sync();
    la  = line_of(la_in);
    prv = is_priv(la);
    lk  = store ? K_WRITE : K_READ;
    if (!sb.try_begin(la, IDX, lk, 0)) begin
      n_busy++;
      return;
    end
    ran = 1;

    w      = width_e'((w_sel < 0) ? rng.range(0, 3) : w_sel);
    nbytes = size_bytes(w);
    off    = ((off_sel < 0) ? rng.range(0, 63) : off_sel) & ~(nbytes - 1);
    a      = la + addr_t'(off);
    lmask  = (nbytes == 8) ? '1 : ((64'd1 << (nbytes * 8)) - 1);
    val    = (use_val ? val_in : rng.r64()) & lmask;
    lm     = span_mask(a, w);
    ld     = line_t'(val) << (off * 8);
    idx    = ReqIdxWidth'(idx_cnt);
    idx_cnt++;
    src    = store ? MEM_UNIT_STORE : MEM_UNIT_LOAD;

    if (store) begin wp = sb.wr_begin(la, ld, lm, NAME); n_store++; end
    else       begin rr = sb.rd_begin(la, NAME);         n_load++;  end
    if (nc)  n_nc++;
    if (prv) n_priv++;
    if (prv && store) begin st_pend = 1; st_pend_la = la; end

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
      if (prv && store) begin pkn[la] = pkn[la] & ~lm; st_pend = 0; end
      scn_abort = 1;
      return;                                   // the line stays locked on purpose, like in rnf_model
    end

    if (rerr) begin
      sb.n_err_seen++;
      if (!sb.is_err_line(la)) sb.err("SPURIOUS_RESPERR", $sformatf("%s: %s addr=%0h returned err but no error was injected", NAME, store ? "store" : "load", a));
    end
    if (store) begin
      if (rerr) sb.wr_abort(la, wp); else sb.wr_end(la, wp, NAME);
      if (prv) begin
        st_pend = 0;
        if (rerr) pkn[la] = pkn[la] & ~lm;
        else begin pline[la] = merge_line(pline[la], ld, lm); pkn[la] = pkn[la] | lm; end
      end
    end else begin
      // the loaded bytes must match one of the legal line values
      gotl = '0; k = -1;
      if (!rr.skip)
        foreach (rr.allowed[i])
          if (k < 0 && ((rr.allowed[i] >> (off * 8)) & line_t'(lmask)) === line_t'(rdata)) k = i;
      if (!rr.skip && rr.allowed.size() != 0)
        gotl = merge_line(rr.allowed[(k < 0) ? 0 : k], line_t'(rdata) << (off * 8), lm);
      sb.rd_end(rr, gotl, rerr, $sformatf("%sload addr=%0h size=%0dB got=%0h", nc ? "non-cacheable " : "", a, nbytes, rdata));
      // private line: exactly the bytes stored last
      if (prv && !rerr && (pkn[la] & lm) == lm) begin
        bit [63:0] exp;
        line_t     sh;
        sh  = pline[la] >> (off * 8);
        exp = sh[63:0] & lmask;
        n_priv_chk++;
        if (exp !== 64'(rdata)) begin
          sb.n_rd_mis++;
          sb.err("READ_MISMATCH", $sformatf("%s: private line load addr=%0h size=%0dB got=%0h, the bytes stored last are %0h", NAME, a, nbytes, rdata, exp));
        end
      end
    end
    sb.end_op(la, IDX, lk);
  endtask

  // random access to line la
  task automatic acc(input addr_t la, input bit store, input int unsigned ep);
    bit ran;
    access(la, store, sb.is_nc(la), -1, -1, 0, '0, ep, ran);
  endtask

  function automatic addr_t pick_line();
    int unsigned hn;
    if (rng.chance(NC_PCT))   return sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)];
    if (rng.chance(PRIV_PCT)) return priv[rng.range(0, priv.size() - 1)];
    hn = (sb.hot_n < sb.pool.size()) ? sb.hot_n : sb.pool.size();
    if (rng.chance(sb.hot_pct)) return sb.pool[rng.range(0, hn - 1)];
    return sb.pool[rng.range(0, sb.pool.size() - 1)];
  endfunction

  // sl = the shared (and private) lines that fall into the same set as `ref_a`
  function automatic void set_lines(addr_t ref_a, bit with_priv);
    sl.delete();
    foreach (sb.pool[i]) if (sb.pool[i][15:6] == ref_a[15:6]) sl.push_back(line_of(sb.pool[i]));
    if (with_priv) foreach (priv[i]) if (priv[i][15:6] == ref_a[15:6]) sl.push_back(priv[i]);
  endfunction

  // ---------------------------------------------------------------- random patterns
  task automatic random_accesses(input int unsigned n, input int unsigned ep);
    int unsigned done, len, pat;
    bit ran;
    addr_t la;
    done = 0;
    while (done < n && ep == sb.epoch) begin
      pat = rng.range(0, 99);
      if (pat < 50) begin                                   // single access, idle gap
        la = pick_line();
        access(la, rng.chance(50), sb.is_nc(la), -1, -1, 0, '0, ep, ran);
        if (ran) done++;
        repeat (rng.range(ran ? 0 : 1, GAP_MAX)) @(posedge clk);
      end else if (pat < 65) begin                          // back-to-back, any line
        len = rng.range(4, 16);
        for (int unsigned i = 0; i < len && ep == sb.epoch; i++) begin
          la = pick_line();
          access(la, rng.chance(50), sb.is_nc(la), -1, -1, 0, '0, ep, ran);
          if (ran) done++; else @(posedge clk);
        end
      end else if (pat < 80) begin                          // back-to-back on one line
        la  = pick_line();
        len = rng.range(8, 24);
        for (int unsigned i = 0; i < len && ep == sb.epoch; i++) begin
          access(la, rng.chance(60), sb.is_nc(la), -1, -1, 0, '0, ep, ran);
          if (ran) done++; else @(posedge clk);
        end
      end else if (pat < 90) begin                          // store, then load the same bytes
        len = rng.range(2, 6);
        for (int unsigned i = 0; i < len && ep == sb.epoch; i++) begin
          int w, off;
          la  = pick_line();
          w   = rng.range(0, 3);
          off = rng.range(0, 63);
          access(la, 1, sb.is_nc(la), w, off, 0, '0, ep, ran);
          if (ran) done++; else @(posedge clk);
          access(la, 0, sb.is_nc(la), w, off, 0, '0, ep, ran);
          if (ran) done++; else @(posedge clk);
        end
      end else begin                                        // walk over one set
        set_lines(sb.pool[rng.range(0, sb.pool.size() - 1)], 1);
        len = rng.range(1, 3);
        for (int unsigned r = 0; r < len; r++)
          for (int i = 0; i < sl.size() && ep == sb.epoch; i++) begin
            access(sl[i], rng.chance(50), 0, -1, -1, 0, '0, ep, ran);
            if (ran) done++; else @(posedge clk);
          end
      end
    end
  endtask

  // ---------------------------------------------------------------- directed scenarios
  function automatic bit go(int unsigned ep);
    return (ep == sb.epoch) && !scn_abort;
  endfunction

  function automatic int unsigned rop_open();
    return sb.rop_issued - sb.rop_done;
  endfunction

  // wait until the other RN-F has finished every op ordered so far
  task automatic rop_wait(input int unsigned ep);
    int unsigned t, target;
    target = sb.rop_issued;
    t = 0;
    while (sb.rop_done < target && ep == sb.epoch && t < 2 * OP_TIMEOUT) begin @(posedge clk); t++; end
    if (ep == sb.epoch && sb.rop_done < target) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("%s: the other RN-F did not finish the ops ordered for this scenario (%0d of %0d done)", NAME, sb.rop_done, target));
      scn_abort = 1;
    end
  endtask

  function automatic opkind_e pick_rd();                    // an op of the other RN-F that needs the line's data
    case (rng.range(0, 5))
      0:       return OP_RS;
      1:       return OP_RU;
      2:       return OP_RO;
      3:       return OP_RC;
      4:       return OP_RNSD;
      default: return OP_WUF;
    endcase
  endfunction

  task automatic run_scenario(input int id, input int unsigned ep);
    addr_t a;
    int    n;
    bit    ran;
    opkind_e km [12];
    scn_abort = 0;
    case (id)
      RS_PINGPONG: begin
        a = sb.pool[0];
        for (int it = 0; it < 12 && go(ep); it++) begin
          acc(a, 1, ep);
          sb.rop_push(OP_RU, a, 1, 0); rop_wait(ep);
          sb.rop_push(OP_ST, a, 1, 0); rop_wait(ep);
          acc(a, 0, ep);
          sb.rop_push(OP_RS, a, 1, 0); rop_wait(ep);
          acc(a, 1, ep);
          sb.rop_push(OP_RO, a, 1, 0); rop_wait(ep);
          acc(a, 0, ep);
        end
      end

      RS_SNOOP_MATRIX: begin
        a = sb.pool[1];
        km[0] = OP_RS;  km[1] = OP_RC;  km[2] = OP_RO; km[3] = OP_RNSD; km[4]  = OP_RU; km[5]  = OP_WUF;
        km[6] = OP_WUP; km[7] = OP_CS;  km[8] = OP_CI; km[9] = OP_MI;   km[10] = OP_CU; km[11] = OP_MU;
        for (int rep = 0; rep < 2; rep++)
          for (int st = 0; st < 2; st++)
            for (int o = 0; o < 12 && go(ep); o++) begin
              if (st == 0) begin                            // dcache Shared Clean
                sb.rop_push(OP_CI, a, 1, 0); rop_wait(ep);
                acc(a, 0, ep);
              end else begin                                // dcache Unique Dirty
                acc(a, 1, ep);
              end
              sb.rop_push(km[o], a, 1, 0); rop_wait(ep);
            end
      end

      RS_WB_SNOOP_RACE, RS_EVICT_SNOOP_RACE: begin
        // Every line of one set but one is touched, private lines first, so the
        // ways end up holding shared lines. The last line then needs a way while
        // the other RN-F goes for the shared lines: one of those is the line
        // being replaced. Needs more lines in the set than the dcache has ways.
        addr_t pq [$], sq [$];
        bit    wr;
        wr = (id == RS_WB_SNOOP_RACE);
        set_lines(sb.pool[2], 1);
        foreach (sl[i]) if (is_priv(sl[i])) pq.push_back(sl[i]); else sq.push_back(sl[i]);
        n = sq.size();
        if (!wr) begin                                      // start from clean lines
          for (int j = 0; j < n; j++) sb.rop_push(OP_CS, sq[j], 1, 0);
          rop_wait(ep);
        end
        for (int it = 0; it < 48 && go(ep); it++) begin
          for (int j = 0; j < pq.size() && go(ep); j++) acc(pq[j], wr, ep);
          for (int j = 0; j < n - 1 && go(ep); j++)     acc(sq[(it + j) % n], wr, ep);
          // odd rounds take the lines away right at the request, even rounds are spread out
          for (int j = 0; j < n - 1; j++) begin
            if (it[0]) sb.rop_push(rng.chance(50) ? OP_RU : OP_WUF, sq[(it + j) % n], 1, rng.range(0, 5));
            else       sb.rop_push(pick_rd(), sq[(it + j) % n], 1, rng.range(0, 16));
          end
          acc(sq[(it + n - 1) % n], wr && rng.chance(50), ep);
          rop_wait(ep);
        end
      end

      RS_UPGRADE_RACE: begin
        a = sb.pool[3];
        for (int it = 0; it < 30 && go(ep); it++) begin
          sb.rop_push(OP_CI, a, 1, 0); rop_wait(ep);
          acc(a, 0, ep);                                    // dcache Shared
          sb.rop_push(OP_RS, a, 1, 0); rop_wait(ep);        // both Shared
          case (rng.range(0, 4))
            0:       sb.rop_push(OP_RU,  a, 0, rng.range(0, 8));
            1:       sb.rop_push(OP_CU,  a, 0, rng.range(0, 8));
            2:       sb.rop_push(OP_MU,  a, 0, rng.range(0, 8));
            3:       sb.rop_push(OP_WUF, a, 1, rng.range(0, 6));
            default: sb.rop_push(OP_WUP, a, 1, rng.range(0, 6));
          endcase
          acc(a, 1, ep);                                    // store on the Shared line
          rop_wait(ep);
        end
      end

      RS_SET_THRASH: begin
        set_lines(sb.pool[0], 1);
        n = sl.size();
        for (int i = 0; i < 160 && go(ep); i++) begin
          if ((i % 8) == 0 && rop_open() < 3)
            sb.rop_push(pick_rd(), sb.pool[rng.range(0, sb.pool.size() - 1)], 1, rng.range(0, 6));
          access(rng.chance(70) ? sl[i % n] : sl[rng.range(0, n - 1)], rng.chance(60), 0, -1, -1, 0, '0, ep, ran);
        end
        rop_wait(ep);
      end

      RS_SNOOP_IN_WAIT: begin
        for (int i = 0; i < 200 && go(ep); i++) begin
          if (rop_open() < 3 && rng.chance(60))
            sb.rop_push(pick_rd(), sb.pool[rng.range(0, sb.pool.size() - 1)], 1, rng.range(0, 10));
          a = rng.chance(PRIV_PCT) ? priv[rng.range(0, priv.size() - 1)] : sb.pool[rng.range(0, sb.pool.size() - 1)];
          acc(a, rng.chance(50), ep);
        end
        rop_wait(ep);
      end

      RS_NC_MIX: begin
        for (int i = 0; i < 80 && go(ep); i++) begin
          a = sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)];
          if (rop_open() < 2) sb.rop_push(rng.chance(50) ? OP_NRD : OP_NWR, a, 0, rng.range(0, 6));
          access(a, rng.chance(50), 1, -1, -1, 0, '0, ep, ran);
        end
        rop_wait(ep);
      end

      RS_PRIV_BURST: begin
        for (int it = 0; it < 40 && go(ep); it++) begin
          int w, off;
          a   = priv[rng.range(0, priv.size() - 1)];
          w   = rng.range(0, 3);
          off = rng.range(0, 63);
          access(a, 1, 0, w, off, 0, '0, ep, ran);          // store, load the same bytes
          access(a, 0, 0, w, off, 0, '0, ep, ran);
          for (int i = 0; i < 6 && go(ep); i++) access(a, rng.chance(50), 0, -1, -1, 0, '0, ep, ran);
          if ((it % 4) == 3) acc(sb.pool[rng.range(0, sb.pool.size() - 1)], rng.chance(50), ep);
        end
      end

      RS_CLEAN_PRIV: begin
        foreach (priv[i]) sb.rop_push(OP_CI, priv[i], 1, 0);
        rop_wait(ep);
      end

      default: ;
    endcase
    sb.rop_stop = 1;
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
      priv_sync();
      case (sb.cmd)
        CMD_RANDOM: random_accesses(sb.cmd_arg * OPS_MULT, ep);
        CMD_REMOTE: run_scenario(sb.cmd_arg, ep);
        CMD_FLUSH:  barrier(sb.num_rnf, ep);
        default: ;
      endcase
      if (ep != sb.epoch) drive_idle();
      sb.agents_done++;
    end
  end

endmodule
