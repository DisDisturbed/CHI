`timescale 1ns/1ps
// =============================================================================
// rnf_core_driver - drives the core side of rnf_dcache like a pipelined,
// out-of-order core would.
//
// Core model
//   * Up to k_out_lim requests in flight (at most 2**ReqIdxWidth), each with
//     its own idx. A new request is put on the pins as soon as the previous
//     one is accepted (no idle cycle in a burst), whether or not the earlier
//     ones have been answered. Responses are matched by idx and may come back
//     in any order.
//   * The only thing the core holds back is a real byte hazard: a request is
//     not issued while an in-flight request to the same line touches one of
//     its bytes and one of the two is a store (RAW / WAR / WAW). Requests to
//     the same line on different bytes go out together: that is the sub-word
//     merge stress.
//   * Accesses are 1, 2, 4 or 8 bytes, always aligned to their size (an
//     unaligned access is a trap in the core and never reaches the dcache).
//   * How many requests the dcache actually takes at once is up to the
//     dcache: it is measured (coverage "core: ..."), the driver never relies
//     on it.
//
// It is an agent of the chi_tb_pkg environment, next to the behavioural
// rnf_model on the other RN-F port. Commands come from the top through
// sb.cmd / sb.cmd_seq:
//   CMD_RANDOM  sb.cmd_arg * OPS_MULT accesses in random patterns (see
//               random_accesses)
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
//   priv        only this driver writes them. Besides the scoreboard, the
//               driver keeps its own byte-exact copy (pline / pkn): a load must
//               return exactly the bytes stored last. It also remembers which
//               store wrote each byte (pwr), so a load that returns bytes of
//               several sub-word stores is seen as a merge check.
//
// All sampling of the core pins is in one monitor block (rsp, accept,
// timeouts); it fires mon_ev when done, and every wait in the driver is on
// mon_ev, so a request can follow a response or an accept with no gap.
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
  parameter int    OP_TIMEOUT     = 20000,
  parameter int    MAX_OUT        = (1 << ReqIdxWidth)   // requests the core keeps in flight at most
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

  localparam int NWID = $clog2(MaxAccessWidth / 8) + 1;    // widths 1..MaxAccessWidth/8 bytes

  // one core access, from issue to response
  class core_op;
    addr_t       la, a;
    bit          store, nc, prv;
    width_e      w;
    int          nbytes, off, idx;
    bit [63:0]   val, lmask;
    line_t       ld;
    bemask_t     lm;
    wr_pend      wp;
    rd_rec       rr;
    mem_unit_e   src;
    int unsigned ep, t_drv, t_acc, seq, sid;
    bit          accepted, done, dead;
    bit          exact;            // private load: bytes known, exp is what must come back
    bit [63:0]   exp;
    int          n_wr;             // private load: distinct stores that wrote the loaded bytes
    bit          has_orig;         // private load: some loaded bytes were never stored
    bit          wr_conc;          // private load: some loaded bytes come from a store that was in flight with another one
    bit          conc;             // store: issued while another store to the line was in flight
  endclass

  tb_rng       rng;
  int unsigned cyc = 0;
  int unsigned idx_cnt = 0;
  int unsigned acc_seq = 0, st_id = 0;
  int unsigned n_load = 0, n_store = 0, n_nc = 0, n_busy = 0, n_priv = 0, n_priv_chk = 0;
  int unsigned n_hz_stall = 0, n_full_stall = 0, n_ooo = 0, n_err_rsp = 0, n_merge_chk = 0;
  int unsigned max_drv_out = 0, max_dut_out = 0;
  int unsigned last_rsp_cyc = 0, last_acc_cyc = 0;
  bit          seen_rsp = 0, seen_acc = 0;

  // ---- per-round knobs (the top sets them, reset_knobs() restores the defaults)
  int unsigned k_out_lim   = MAX_OUT;      // requests in flight at most (1 = blocking core)
  int unsigned k_store_pct = 50;
  int unsigned k_nc_pct    = NC_PCT;
  int unsigned k_priv_pct  = PRIV_PCT;
  int unsigned k_gap_max   = GAP_MAX;
  int unsigned k_b2b_pct   = 70;           // a burst goes out with no gap between requests
  int unsigned k_w_wt [NWID];              // weight of each access width

  // in flight, by idx (also the ones not accepted yet)
  core_op      infl [int];
  event        mon_ev;

  // private lines and their reference copy
  addr_t       priv   [$];
  line_t       pline  [addr_t];
  bemask_t     pkn    [addr_t];          // bytes of pline that are known
  int unsigned pwr    [addr_t];          // per byte address: id of the store that wrote it, 0 = never stored
  bit          pcon   [addr_t];          // per byte address: that store was in flight together with another store to the line
  int unsigned ref_epoch = 0;

  // lane coverage of cacheable accesses: [store][log2 width] bit p = position p (offset / width) was accessed
  bit [63:0]   pos_cov [2][NWID];

  addr_t       sl [$];                   // lines of one set (set_lines)
  int          pc_off [$], pc_w [$];     // pieces of a sub-word pattern (make_pieces)
  int          perm [$];
  bit          scn_abort;

  function automatic void reset_knobs();
    k_out_lim = MAX_OUT; k_store_pct = 50; k_nc_pct = NC_PCT; k_priv_pct = PRIV_PCT;
    k_gap_max = GAP_MAX; k_b2b_pct = 70;
    foreach (k_w_wt[i]) k_w_wt[i] = 10;
  endfunction

  initial begin
    rng = new(SEED);
    reset_knobs();
    foreach (pos_cov[s, w]) pos_cov[s][w] = '0;
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

  // (a ?: of two string literals is a padded bit vector in Verilator's %s: go through a string)
  function automatic string s_op(bit st);
    if (st) return "store";
    return "load";
  endfunction

  function automatic string s_nc(bit nc);
    if (nc) return "nc";
    return "cached";
  endfunction

  function automatic string s_start(int st);
    case (st)
      0:       return "not held";
      1:       return "Shared";
      default: return "Unique Dirty";
    endcase
  endfunction

  function automatic string nb(int unsigned n);
    if (n == 0) return "0";
    if (n == 1) return "1";
    if (n < 4)  return "2-3";
    return "4+";
  endfunction

  // after a reset the cache is empty: what is in memory is the value. A store
  // that was in flight may or may not have reached memory.
  function automatic void priv_sync();
    if (ref_epoch != sb.epoch) begin
      foreach (priv[i]) begin
        pline[priv[i]] = sb.snf_line(priv[i]);
        pkn[priv[i]]   = '1;
      end
      foreach (infl[k]) if (infl[k].prv && infl[k].store) pkn[infl[k].la] = pkn[infl[k].la] & ~infl[k].lm;
      infl.delete();
      pwr.delete();
      pcon.delete();
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

  // one clock, after the monitor has looked at the edge
  task automatic tick();
    @(mon_ev);
  endtask

  // ---------------------------------------------------------------- bookkeeping
  function automatic int unsigned n_live();
    int unsigned n;
    n = 0;
    foreach (infl[k]) if (!infl[k].dead) n++;
    return n;
  endfunction

  function automatic int unsigned n_dut_out();
    int unsigned n;
    n = 0;
    foreach (infl[k]) if (infl[k].accepted && !infl[k].done && !infl[k].dead) n++;
    return n;
  endfunction

  // 0: none, 1: RAW, 2: WAR, 3: WAW
  function automatic int hazard(addr_t la, bemask_t lm, bit store);
    foreach (infl[k]) begin
      core_op o;
      o = infl[k];
      if (!o.done && o.la == la && (o.lm & lm) != '0) begin
        if (o.store && store) return 3;
        if (o.store)          return 1;
        if (store)            return 2;
      end
    end
    return 0;
  endfunction

  function automatic bit store_in_flight(addr_t la, bit dut_side);
    foreach (infl[k])
      if (infl[k].store && infl[k].la == la && !infl[k].done && !infl[k].dead && (!dut_side || infl[k].accepted)) return 1;
    return 0;
  endfunction

  function automatic int free_idx();
    for (int i = 0; i < MAX_OUT; i++) begin
      int c;
      c = int'((idx_cnt + i) % MAX_OUT);
      if (!infl.exists(c)) begin idx_cnt = c + 1; return c; end
    end
    return -1;
  endfunction

  function automatic width_e pick_width();
    int unsigned tot, r;
    tot = 0;
    foreach (k_w_wt[i]) tot += k_w_wt[i];
    if (tot == 0) return width_e'(rng.range(0, NWID - 1));
    r = rng.range(0, tot - 1);
    foreach (k_w_wt[i]) begin
      if (r < k_w_wt[i]) return width_e'(i);
      r -= k_w_wt[i];
    end
    return WIDTH_1;
  endfunction

  // ---------------------------------------------------------------- monitor
  function automatic void finish_op(core_op o, logic [MaxAccessWidth-1:0] rdata, bit rerr, chr_state_e cst);
    lockkind_e lk;
    line_t     gotl;
    int        k;
    string     kind;
    lk   = o.store ? K_WRITE : K_READ;
    kind = $sformatf("%s %0dB %s", s_nc(o.nc), o.nbytes, s_op(o.store));
    sb.cov_hit({"core: ", kind});
    if (!o.nc) pos_cov[o.store][int'(o.w)][o.off / o.nbytes] = 1;

    if (rerr) begin
      sb.n_err_seen++;
      n_err_rsp++;
      sb.cov_hit($sformatf("core: rsp.err on %s %s", s_nc(o.nc), s_op(o.store)));
      if (!sb.is_err_line(o.la)) sb.err("SPURIOUS_RESPERR", $sformatf("%s: %s addr=%0h returned err but no error was injected", NAME, s_op(o.store), o.a));
    end else if (sb.is_err_line(o.la)) begin
      sb.cov_hit($sformatf("core: no rsp.err on an error line (%s)", s_op(o.store)));
    end
    // contract: chr_state is I for non-cacheable accesses
    if (o.nc && cst != CS_I)
      sb.err("PROTOCOL", $sformatf("%s: non-cacheable %s addr=%0h answered with chr_state=%s, expected CS_I", NAME, s_op(o.store), o.a, cst.name()));
    if (!o.nc && !rerr) sb.cov_hit($sformatf("core: cached %s answered with chr_state %s", s_op(o.store), cst.name()));

    if (o.store) begin
      if (rerr) sb.wr_abort(o.la, o.wp); else sb.wr_end(o.la, o.wp, NAME);
      if (o.prv) begin
        if (rerr) pkn[o.la] = pkn[o.la] & ~o.lm;
        else begin
          pline[o.la] = merge_line(pline[o.la], o.ld, o.lm);
          pkn[o.la]   = pkn[o.la] | o.lm;
          for (int i = 0; i < o.nbytes; i++) begin
            pwr[o.a + addr_t'(i)]  = o.sid;
            pcon[o.a + addr_t'(i)] = o.conc;
          end
        end
      end
    end else begin
      // the loaded bytes must match one of the legal line values
      gotl = '0; k = -1;
      if (!o.rr.skip)
        foreach (o.rr.allowed[i])
          if (k < 0 && ((o.rr.allowed[i] >> (o.off * 8)) & line_t'(o.lmask)) === line_t'(rdata)) k = i;
      if (!o.rr.skip && o.rr.allowed.size() != 0)
        gotl = merge_line(o.rr.allowed[(k < 0) ? 0 : k], line_t'(rdata) << (o.off * 8), o.lm);
      sb.rd_end(o.rr, gotl, rerr, $sformatf("%sload addr=%0h size=%0dB got=%0h", o.nc ? "non-cacheable " : "", o.a, o.nbytes, rdata));
      // private line: exactly the bytes stored last
      if (o.prv && o.exact && !rerr) begin
        n_priv_chk++;
        if (o.exp !== 64'(rdata)) begin
          sb.n_rd_mis++;
          sb.err("READ_MISMATCH", $sformatf("%s: private line load addr=%0h size=%0dB got=%0h, the bytes stored last are %0h", NAME, o.a, o.nbytes, rdata, o.exp));
        end else begin
          if (o.n_wr >= 2) begin
            n_merge_chk++;
            sb.cov_hit("merge: load returned the bytes of 2+ sub-word stores (exact)");
            if (o.wr_conc) sb.cov_hit("merge: load returned the bytes of sub-word stores that were in flight together (exact)");
          end
          if (o.n_wr >= 1 && o.has_orig) sb.cov_hit("merge: load returned stored bytes next to bytes never stored (exact)");
        end
      end
    end
    sb.end_op(o.la, IDX, lk);
  endfunction

  function automatic void on_rsp();
    int     k;
    core_op o;
    k = int'(core.rsp.idx);
    if (!infl.exists(k) || !infl[k].accepted || infl[k].done) begin
      sb.err("PROTOCOL", $sformatf("%s: response with idx=%0d, but no accepted request with that idx is waiting for one", NAME, k));
      return;
    end
    o = infl[k];
    if (core.rsp.src !== o.src || core.rsp.op !== (o.store ? MEM_OP_STORE : MEM_OP_LOAD))
      sb.err("PROTOCOL", $sformatf("%s: response idx=%0d does not echo its request (src %0d/%0d op %0d/%0d)", NAME, k,
                                   core.rsp.src, o.src, core.rsp.op, o.store ? MEM_OP_STORE : MEM_OP_LOAD));
    // out of order: an older accepted request is still waiting
    foreach (infl[j]) if (infl[j].accepted && !infl[j].done && !infl[j].dead && infl[j].seq < o.seq) begin
      n_ooo++;
      sb.cov_hit("core: response overtook an older request");
      if (!o.store && infl[j].store) sb.cov_hit("core: load response overtook an older store");
      if (o.la != infl[j].la)        sb.cov_hit("core: response overtook an older request to another line");
      else                           sb.cov_hit("core: response overtook an older request to the same line");
      break;
    end
    o.done = 1;
    if (!o.dead) finish_op(o, core.rsp.data, core.rsp.err == Y, core.rsp.chr_state);
    infl.delete(k);
    last_rsp_cyc = cyc; seen_rsp = 1;
  endfunction

  function automatic void on_accept();
    int     k;
    int unsigned n;
    core_op o;
    k = int'(core.req.idx);
    if (!infl.exists(k) || infl[k].accepted) return;           // not ours (cannot happen) or held one more edge
    o = infl[k];
    n = n_dut_out();
    if (o.store && store_in_flight(o.la, 1)) sb.cov_hit("core: two stores to one line outstanding at the dcache");
    o.accepted = 1; o.t_acc = cyc; o.seq = acc_seq++;
    sb.cov_hit($sformatf("core: request accepted with %s other request(s) outstanding", nb(n)));
    if (n + 1 > max_dut_out) max_dut_out = n + 1;
    if (seen_rsp && last_rsp_cyc == cyc)          sb.cov_hit("core: request accepted in the cycle of a response");
    if (seen_rsp && last_rsp_cyc + 1 == cyc)      sb.cov_hit("core: request accepted the cycle after a response");
    if (seen_acc && last_acc_cyc + 1 == cyc)      sb.cov_hit("core: requests accepted in back-to-back cycles");
    if ((cyc - o.t_drv) >= 8)                     sb.cov_hit("drv: request waited 8+ cycles for ready");
    last_acc_cyc = cyc; seen_acc = 1;
  endfunction

  function automatic void on_timeout(core_op o);
    sb.n_timeout++;
    sb.err("TIMEOUT", $sformatf("%s: %s%s idx=%0d addr=%0h size=%0dB got no response within %0d cycles", NAME, o.nc ? "non-cacheable " : "",
                                s_op(o.store), o.idx, o.a, o.nbytes, OP_TIMEOUT));
    if (o.store) sb.wr_abort(o.la, o.wp);
    else         sb.rd_end(o.rr, '0, 1, "timeout");
    if (o.prv && o.store) pkn[o.la] = pkn[o.la] & ~o.lm;
    o.dead = 1;                                 // idx stays taken, the line stays locked, like in rnf_model
    scn_abort = 1;
  endfunction

  always @(posedge clk) begin
    if (sb != null) begin
      priv_sync();
      if (resetn) begin
        if (core.rsp.valid == Y) on_rsp();
        if (core.req.valid == Y && core.ready == Y) on_accept();
        foreach (infl[k]) if (infl[k].accepted && !infl[k].done && !infl[k].dead && (cyc - infl[k].t_acc) > OP_TIMEOUT) on_timeout(infl[k]);
      end
    end
    -> mon_ev;
  end

  // ---------------------------------------------------------------- one access
  // Puts one access on the pins and returns once the dcache has accepted it;
  // the response is handled by the monitor. o = null when nothing was issued.
  // w_sel / off_sel < 0: random width / offset. use_val = 0: random store data.
  task automatic issue(input addr_t la_in, input bit store, input bit nc,
                       input int w_sel, input int off_sel, input bit use_val, input bit [63:0] val_in,
                       input int unsigned ep, output core_op o);
    addr_t       la, a;
    lockkind_e   lk;
    width_e      w;
    int          nbytes, off, idx, hz;
    int unsigned t0, ids [$];
    bit          st_hz, st_full;
    bit [63:0]   lmask;
    bemask_t     lm;

    o = null;
    if (ep != sb.epoch || scn_abort) return;
    la     = line_of(la_in);
    w      = (w_sel < 0) ? pick_width() : width_e'(w_sel);
    nbytes = size_bytes(w);
    off    = ((off_sel < 0) ? rng.range(0, 63) : off_sel) & ~(nbytes - 1) & 63;
    a      = la + addr_t'(off);
    lm     = span_mask(a, w);
    lk     = store ? K_WRITE : K_READ;

    // a free slot and no byte hazard with what is in flight
    t0 = cyc; st_hz = 0; st_full = 0;
    forever begin
      if (ep != sb.epoch || scn_abort) return;
      hz = hazard(la, lm, store);
      if (n_live() >= k_out_lim || infl.size() >= MAX_OUT) begin
        if (!st_full) begin n_full_stall++; st_full = 1; end
      end else if (hz != 0) begin
        if (!st_hz) begin
          n_hz_stall++; st_hz = 1;
          sb.cov_hit((hz == 1) ? "drv: load held back by a store to its bytes (RAW)" :
                     (hz == 2) ? "drv: store held back by a load of its bytes (WAR)" :
                                 "drv: store held back by a store to its bytes (WAW)");
        end
      end else break;
      if ((cyc - t0) > OP_TIMEOUT) begin
        sb.n_timeout++;
        sb.err("TIMEOUT", $sformatf("%s: %s addr=%0h could not be issued for %0d cycles (%0d in flight)", NAME, s_op(store), a, OP_TIMEOUT, infl.size()));
        scn_abort = 1;
        return;
      end
      tick();
    end
    if (!sb.try_begin(la, IDX, lk, 1)) begin
      n_busy++;
      return;
    end
    idx = free_idx();

    o = new();
    o.la = la; o.a = a; o.store = store; o.nc = nc; o.prv = is_priv(la);
    o.w = w; o.nbytes = nbytes; o.off = off; o.idx = idx; o.lm = lm; o.ep = ep;
    lmask   = (nbytes == 8) ? '1 : ((64'd1 << (nbytes * 8)) - 1);
    o.lmask = lmask;
    o.val   = (use_val ? val_in : rng.r64()) & lmask;
    o.ld    = line_t'(o.val) << (off * 8);
    o.src   = store ? MEM_UNIT_STORE : MEM_UNIT_LOAD;

    if (store) begin
      o.conc = store_in_flight(la, 0);
      if (o.conc) sb.cov_hit("drv: sub-word stores to one line issued while another is in flight");
      o.sid = ++st_id;
      o.wp  = sb.wr_begin(la, o.ld, lm, NAME);
      n_store++;
    end else begin
      o.rr = sb.rd_begin(la, NAME);
      n_load++;
      if (o.prv && (pkn[la] & lm) == lm) begin
        // nothing in flight writes these bytes (hazard rule): this is what must come back
        line_t sh;
        sh       = pline[la] >> (off * 8);
        o.exact  = 1;
        o.exp    = sh[63:0] & lmask;
        o.n_wr   = 0; o.has_orig = 0; o.wr_conc = 0;
        for (int i = 0; i < nbytes; i++) begin
          addr_t ba;
          int unsigned id;
          bit dup;
          ba = a + addr_t'(i);
          id = pwr.exists(ba) ? pwr[ba] : 0;
          if (id == 0) o.has_orig = 1;
          else begin
            dup = 0;
            foreach (ids[j]) if (ids[j] == id) dup = 1;
            if (!dup) begin ids.push_back(id); o.n_wr++; end
            if (pcon.exists(ba) && pcon[ba]) o.wr_conc = 1;
          end
        end
      end
    end
    if (nc)     n_nc++;
    if (o.prv)  n_priv++;
    sb.cov_hit($sformatf("drv: request presented with %s other request(s) in flight", nb(n_live())));
    infl[idx] = o;
    if (n_live() > max_drv_out) max_drv_out = n_live();

    d_valid     = Y;
    d_op        = store ? MEM_OP_STORE : MEM_OP_LOAD;
    d_addr      = AddrSize'(a);
    d_width     = w;
    d_data      = MaxAccessWidth'(o.val);
    d_idx       = ReqIdxWidth'(idx);
    d_src       = o.src;
    d_cacheable = nc ? N : Y;
    -> drive_ev;
    o.t_drv = cyc;

    // on the pins from the next edge on; the monitor marks the edge that accepts it
    while (!o.accepted && ep == sb.epoch) begin
      tick();
      if (!o.accepted && (cyc - o.t_drv) > OP_TIMEOUT) begin
        sb.n_timeout++;
        sb.err("TIMEOUT", $sformatf("%s: %s%s idx=%0d addr=%0h size=%0dB was never accepted", NAME, nc ? "non-cacheable " : "",
                                    s_op(store), idx, a, nbytes));
        if (store) sb.wr_abort(la, o.wp);
        else       sb.rd_end(o.rr, '0, 1, "never accepted");
        if (o.prv && store) pkn[la] = pkn[la] & ~lm;
        infl.delete(idx);
        drive_idle();
        scn_abort = 1;
        o = null;
        return;                                 // the line stays locked on purpose, like in rnf_model
      end
    end
    drive_idle();                               // a request issued right after this one overrides it
  endtask

  task automatic wait_op(input core_op o, input int unsigned ep);
    if (o == null) return;
    while (!o.done && !o.dead && ep == sb.epoch) tick();
  endtask

  // every request answered (or timed out)
  task automatic drain(input int unsigned ep);
    while (n_live() != 0 && ep == sb.epoch) tick();
  endtask

  // blocking access (the request is the only thing the core does until its response)
  task automatic access(input addr_t la, input bit store, input bit nc,
                        input int w_sel, input int off_sel, input bit use_val, input bit [63:0] val_in,
                        input int unsigned ep, output bit ran);
    core_op o;
    issue(la, store, nc, w_sel, off_sel, use_val, val_in, ep, o);
    ran = (o != null);
    wait_op(o, ep);
  endtask

  task automatic acc(input addr_t la, input bit store, input int unsigned ep);
    bit ran;
    access(la, store, sb.is_nc(la), -1, -1, 0, '0, ep, ran);
  endtask

  // non-blocking random access, with an optional gap afterwards
  task automatic go_acc(input addr_t la, input bit store, input int unsigned ep, input bit gap);
    core_op o;
    issue(la, store, sb.is_nc(la), -1, -1, 0, '0, ep, o);
    if (o == null) tick();
    if (gap) repeat (rng.range(0, k_gap_max)) tick();
  endtask

  function automatic addr_t pick_line();
    int unsigned hn;
    if (rng.chance(k_nc_pct))   return sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)];
    if (rng.chance(k_priv_pct)) return priv[rng.range(0, priv.size() - 1)];
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

  // perm = 0..n-1 in random order
  function automatic void make_perm(int n);
    perm.delete();
    for (int i = 0; i < n; i++) perm.push_back(i);
    for (int i = n - 1; i > 0; i--) begin
      int j, t;
      j = int'(rng.range(0, i));
      t = perm[i]; perm[i] = perm[j]; perm[j] = t;
    end
  endfunction

  // pc_off / pc_w = bytes [base, base + len) cut into aligned pieces of at
  // most 2**maxw bytes, sizes drawn at random, in random order
  function automatic void make_pieces(int base, int len, int maxw);
    int pos, mw, wl;
    int to [$], tw [$];
    pos = base;
    while (pos < base + len) begin
      mw = 0;
      while (mw < maxw && mw + 1 < NWID && (pos % (1 << (mw + 1))) == 0 && pos + (1 << (mw + 1)) <= base + len) mw++;
      wl = int'(rng.range(0, mw));
      to.push_back(pos); tw.push_back(wl);
      pos += (1 << wl);
    end
    make_perm(to.size());
    pc_off.delete(); pc_w.delete();
    foreach (perm[i]) begin pc_off.push_back(to[perm[i]]); pc_w.push_back(tw[perm[i]]); end
  endfunction

  // pieces scattered over the whole line: n disjoint random sub-words
  function automatic void make_scatter(int n);
    bemask_t used;
    int      off, wl, tries;
    used = '0;
    pc_off.delete(); pc_w.delete();
    tries = 0;
    while (pc_off.size() < n && tries < 200) begin
      bemask_t m;
      wl  = int'(rng.range(0, NWID - 1));
      off = int'(rng.range(0, 63)) & ~((1 << wl) - 1);
      m   = '0;
      for (int i = 0; i < (1 << wl); i++) m[off + i] = 1'b1;
      if ((m & used) == '0) begin used |= m; pc_off.push_back(off); pc_w.push_back(wl); end
      tries++;
    end
  endfunction

  // the pieces of pc_off / pc_w as stores (or loads), back to back, all in flight together
  task automatic issue_pieces(input addr_t la, input bit store, input bit nc, input int unsigned ep);
    core_op o;
    for (int i = 0; i < pc_off.size() && go(ep); i++) begin
      issue(la, store, nc, pc_w[i], pc_off[i], 0, '0, ep, o);
      if (o == null) tick();
    end
  endtask

  // loads that read back [base, base + len): one wide load per 8 bytes, then
  // random narrower ones; they wait (RAW) for the stores and follow them at once
  task automatic read_back(input addr_t la, input bit nc, input int base, input int len, input int unsigned ep);
    core_op o;
    for (int p = base & ~7; p < base + len && go(ep); p += 8) begin
      issue(la, 0, nc, NWID - 1, p, 0, '0, ep, o);
      if (o == null) tick();
    end
    for (int i = 0; i < 4 && go(ep); i++) begin
      issue(la, 0, nc, -1, base + int'(rng.range(0, len - 1)), 0, '0, ep, o);
      if (o == null) tick();
    end
  endtask

  // ---------------------------------------------------------------- random patterns
  task automatic random_accesses(input int unsigned n, input int unsigned ep);
    int unsigned done0, len, pat;
    bit ran;
    addr_t la;
    core_op o;
    done0 = n_load + n_store;
    while ((n_load + n_store - done0) < n && go(ep)) begin
      pat = rng.range(0, 99);
      if (pat < 20) begin                                   // single blocking access, idle gap
        la = pick_line();
        access(la, rng.chance(k_store_pct), sb.is_nc(la), -1, -1, 0, '0, ep, ran);
        repeat (rng.range(ran ? 0 : 1, k_gap_max)) tick();
      end else if (pat < 36) begin                          // pipelined burst, any line
        bit gap;
        len = rng.range(4, 16);
        gap = !rng.chance(k_b2b_pct);
        for (int unsigned i = 0; i < len && go(ep); i++) go_acc(pick_line(), rng.chance(k_store_pct), ep, gap);
      end else if (pat < 48) begin                          // pipelined burst on one line (hazards and merges)
        la  = pick_line();
        len = rng.range(6, 24);
        for (int unsigned i = 0; i < len && go(ep); i++) go_acc(la, rng.chance(k_store_pct), ep, 0);
      end else if (pat < 58) begin                          // store, then load the same bytes (RAW right behind it)
        len = rng.range(2, 6);
        for (int unsigned i = 0; i < len && go(ep); i++) begin
          int w, off;
          la  = pick_line();
          w   = int'(pick_width());
          off = rng.range(0, 63);
          issue(la, 1, sb.is_nc(la), w, off, 0, '0, ep, o); if (o == null) tick();
          issue(la, 0, sb.is_nc(la), w, off, 0, '0, ep, o); if (o == null) tick();
        end
      end else if (pat < 68) begin                          // walk over one set, pipelined
        set_lines(sb.pool[rng.range(0, sb.pool.size() - 1)], 1);
        len = rng.range(1, 3);
        for (int unsigned r = 0; r < len; r++) begin
          make_perm(sl.size());
          for (int i = 0; i < sl.size() && go(ep); i++) go_acc(sl[perm[i]], rng.chance(k_store_pct), ep, 0);
        end
      end else if (pat < 80) begin                          // sub-word merge: build bytes from pieces, read them back
        int base, blen;
        bit nc;
        la   = pick_line();
        nc   = sb.is_nc(la);
        if (rng.chance(30)) begin
          make_scatter(rng.range(3, 10));
          issue_pieces(la, 1, nc, ep);
          read_back(la, nc, 0, 64, ep);
        end else begin
          blen = 8 << rng.range(0, 2);                      // 8, 16 or 32 bytes
          base = int'(rng.range(0, 63)) & ~(blen - 1);
          make_pieces(base, blen, NWID - 1);
          issue_pieces(la, 1, nc, ep);
          read_back(la, nc, base, blen, ep);
        end
      end else if (pat < 92) begin                          // misses first, hits behind them
        int nm;
        nm = rng.range(1, 3);
        for (int i = 0; i < nm && go(ep); i++) go_acc(sb.pool[rng.range(0, sb.pool.size() - 1)], rng.chance(k_store_pct), ep, 0);
        if (rng.chance(40)) go_acc(sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)], rng.chance(50), ep, 0);
        la  = priv[rng.range(0, priv.size() - 1)];
        len = rng.range(2, 8);
        for (int unsigned i = 0; i < len && go(ep); i++) go_acc(la, rng.chance(k_store_pct), ep, 0);
      end else begin                                        // the core waits for everything, then idles
        drain(ep);
        repeat (rng.range(0, 3 * k_gap_max)) tick();
      end
    end
    drain(ep);
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
    while (sb.rop_done < target && ep == sb.epoch && t < 2 * OP_TIMEOUT) begin tick(); t++; end
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

  // bring line la into a start state in the dcache: 0 not held, 1 Shared
  // (the other RN-F holds it too), 2 Unique Dirty
  task automatic prep_line(input addr_t la, input int st, input int unsigned ep);
    bit ran;
    drain(ep);
    sb.rop_push(OP_CI, la, 1, 0); rop_wait(ep);
    case (st)
      1: begin
        access(la, 0, 0, -1, -1, 0, '0, ep, ran);
        sb.rop_push(OP_RS, la, 1, 0); rop_wait(ep);
      end
      2: access(la, 1, 0, -1, -1, 0, '0, ep, ran);
      default: ;
    endcase
  endtask

  task automatic run_scenario(input int id, input int unsigned ep);
    addr_t a;
    int    n;
    bit    ran;
    core_op o;
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
        // the first 240 rounds sweep every remote op over remote delays 0..15,
        // three times over, so the snoop lands at each point of the dcache
        // upgrade (before and after its request, while it waits for CompData;
        // that last window is only a cycle or two wide); the rest are random
        for (int it = 0; it < 270 && go(ep); it++) begin
          int unsigned rk, rd;
          rk = (it < 240) ? (it % 5) : rng.range(0, 4);
          rd = (it < 240) ? ((it / 5) % 16) : rng.range(0, 15);
          sb.rop_push(OP_CI, a, 1, 0); rop_wait(ep);
          acc(a, 0, ep);                                    // dcache Shared
          sb.rop_push(OP_RS, a, 1, 0); rop_wait(ep);        // both Shared
          case (rk)
            0:       sb.rop_push(OP_RU,  a, 0, rd);
            1:       sb.rop_push(OP_CU,  a, 0, rd);
            2:       sb.rop_push(OP_MU,  a, 0, rd);
            3:       sb.rop_push(OP_WUF, a, 1, rd);
            default: sb.rop_push(OP_WUP, a, 1, rd);
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
          w   = rng.range(0, NWID - 1);
          off = rng.range(0, 63);
          access(a, 1, 0, w, off, 0, '0, ep, ran);          // store, load the same bytes
          access(a, 0, 0, w, off, 0, '0, ep, ran);
          for (int i = 0; i < 6 && go(ep); i++) access(a, rng.chance(50), 0, -1, -1, 0, '0, ep, ran);
          if ((it % 4) == 3) acc(sb.pool[rng.range(0, sb.pool.size() - 1)], rng.chance(50), ep);
        end
      end

      // ---- sub-word merging -------------------------------------------------
      RS_SUBWORD_MERGE: begin
        // An 8/16/32-byte window (or scattered sub-words) is built from
        // smaller stores that are all in flight together, starting from a
        // line the dcache does not hold (merge into the fill), holds Shared
        // (merge into the upgrade) or holds Unique Dirty (merge into the hit),
        // then read back with wide and narrow loads. Even rounds use private
        // lines (exact check), odd rounds shared lines, where the other RN-F
        // may have written the whole line first (its bytes must survive).
        for (int it = 0; it < 36 && go(ep); it++) begin
          int st, base, blen;
          bit sh;
          sh = it[0];
          a  = sh ? sb.pool[rng.range(0, sb.pool.size() - 1)] : priv[rng.range(0, priv.size() - 1)];
          st = (it / 2) % 3;
          prep_line(a, st, ep);
          if (sh && st == 0 && rng.chance(60)) begin        // the other RN-F owns the line dirty
            sb.rop_push(rng.chance(50) ? OP_ST : OP_WUF, a, 1, 0); rop_wait(ep);
            sb.cov_hit("merge: sub-word stores onto a line the other RN-F holds dirty");
          end
          case (rng.range(0, 3))
            0:       begin base = int'(rng.range(0, 7)) * 8; blen = 8; make_pieces(base, blen, 0); end   // 8 single bytes
            1:       begin blen = 8 << rng.range(0, 2); base = int'(rng.range(0, 63)) & ~(blen - 1); make_pieces(base, blen, NWID - 1); end
            2:       begin make_scatter(rng.range(4, 12)); base = 0; blen = 64; end
            default: begin base = int'(rng.range(0, 7)) * 8; blen = 8; make_pieces(base, blen, 1); end   // bytes and halfwords
          endcase
          sb.cov_hit($sformatf("merge: sub-word stores onto a line that starts %s", s_start(st)));
          issue_pieces(a, 1, 0, ep);
          // optionally the other RN-F snoops the line while the stores are in flight
          if (sh && rng.chance(40)) sb.rop_push(rng.chance(50) ? OP_RS : OP_RU, a, 1, rng.range(0, 6));
          read_back(a, 0, base, blen, ep);
          drain(ep);
          rop_wait(ep);
        end
      end

      RS_SUBWORD_LANES: begin
        // every aligned position of every width: stored (all in flight
        // together), then read back at every width, on private lines
        for (int wl = 0; wl < NWID && go(ep); wl++) begin
          int np;
          a  = priv[(wl * 5 + 1) % priv.size()];
          np = 64 >> wl;
          if (wl[0]) prep_line(a, 0, ep);                   // half of the passes start from a missing line
          make_perm(np);
          for (int i = 0; i < np && go(ep); i++) begin issue(a, 1, 0, wl, perm[i] << wl, 0, '0, ep, o); if (o == null) tick(); end
          for (int rl = 0; rl < NWID && go(ep); rl++) begin
            make_perm(64 >> rl);
            for (int i = 0; i < (64 >> rl) && go(ep); i++) begin issue(a, 0, 0, rl, perm[i] << rl, 0, '0, ep, o); if (o == null) tick(); end
          end
          drain(ep);
        end
        // the same on one shared line, checked by the scoreboard
        a = sb.pool[4 % sb.pool.size()];
        prep_line(a, 1, ep);
        for (int wl = 0; wl < NWID && go(ep); wl++) begin
          make_perm(64 >> wl);
          for (int i = 0; i < (64 >> wl) && go(ep); i++) begin issue(a, 1, 0, wl, perm[i] << wl, 0, '0, ep, o); if (o == null) tick(); end
          make_perm(64 >> (NWID - 1));
          for (int i = 0; i < (64 >> (NWID - 1)) && go(ep); i++) begin issue(a, 0, 0, NWID - 1, perm[i] << (NWID - 1), 0, '0, ep, o); if (o == null) tick(); end
        end
        drain(ep);
      end

      RS_NC_SUBWORD: begin
        // non-cacheable sub-word stores to disjoint bytes of one line, all in
        // flight together, read back; the other RN-F now and then writes or
        // reads the same line non-cacheably in between
        for (int it = 0; it < 30 && go(ep); it++) begin
          int base, blen;
          a = sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)];
          if (rng.chance(40)) begin make_scatter(rng.range(3, 8)); base = 0; blen = 64; end
          else begin blen = 8 << rng.range(0, 1); base = int'(rng.range(0, 63)) & ~(blen - 1); make_pieces(base, blen, NWID - 1); end
          if (rng.chance(30) && rop_open() < 2) sb.rop_push(rng.chance(50) ? OP_NRD : OP_NWR, a, 0, rng.range(0, 8));
          issue_pieces(a, 1, 1, ep);
          read_back(a, 1, base, blen, ep);
          drain(ep);
        end
        rop_wait(ep);
      end

      // ---- pipelined / out-of-order stress ----------------------------------
      RS_OOO_BURST: begin
        // Long-latency requests first (misses on lines the other RN-F just
        // cleaned out, non-cacheable accesses to slow memory), short ones
        // behind them (hits on warm private lines), all back to back with the
        // full in-flight window. A dcache that can, answers the hits first.
        addr_t warm [$];
        for (int i = 0; i < 4; i++) warm.push_back(priv[(i * 3) % priv.size()]);
        foreach (warm[i]) acc(warm[i], 1, ep);
        for (int it = 0; it < 40 && go(ep); it++) begin
          int nm;
          addr_t cl [$];
          // misses: clean two shared lines out of the dcache first
          for (int j = 0; j < 2; j++) begin
            addr_t c;
            c = sb.pool[rng.range(0, sb.pool.size() - 1)];
            cl.push_back(c);
            sb.rop_push(OP_CI, c, 1, 0);
          end
          rop_wait(ep);
          nm = rng.range(1, 3);
          for (int j = 0; j < nm && go(ep); j++) go_acc(cl[j % cl.size()], rng.chance(50), ep, 0);
          if (rng.chance(60)) go_acc(sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)], rng.chance(50), ep, 0);
          if (rng.chance(30)) sb.rop_push(pick_rd(), cl[0], 1, rng.range(0, 10));   // the miss line is snooped meanwhile
          n = rng.range(4, MAX_OUT);
          for (int j = 0; j < n && go(ep); j++) go_acc(warm[rng.range(0, warm.size() - 1)], rng.chance(k_store_pct), ep, 0);
          drain(ep);
          rop_wait(ep);
        end
      end

      RS_B2B_SAME_LINE: begin
        // back-to-back loads and stores to one line, disjoint and overlapping
        // bytes mixed (merges and RAW / WAR / WAW chains), from every start
        // state, while the other RN-F takes the line away in the middle
        for (int it = 0; it < 36 && go(ep); it++) begin
          a = it[0] ? sb.pool[rng.range(0, sb.pool.size() - 1)] : priv[rng.range(0, priv.size() - 1)];
          prep_line(a, it % 3, ep);
          if (rng.chance(60)) begin
            case (rng.range(0, 3))
              0:       sb.rop_push(OP_RU, a, 1, rng.range(0, 12));
              1:       sb.rop_push(OP_RS, a, 1, rng.range(0, 12));
              2:       sb.rop_push(OP_CI, a, 1, rng.range(0, 12));
              default: sb.rop_push(it[0] ? OP_ST : OP_RO, a, 1, rng.range(0, 12));   // the other RN-F never writes a private line
            endcase
          end
          n = rng.range(6, 20);
          for (int j = 0; j < n && go(ep); j++) go_acc(a, rng.chance(55), ep, 0);
          drain(ep);
          rop_wait(ep);
        end
      end

      RS_B2B_SET_CONFLICT: begin
        // every line of one set (more than the ways) back to back, all in
        // flight: misses that need a victim while other misses of the same
        // set are still open; the other RN-F reads some of them meanwhile
        for (int it = 0; it < 24 && go(ep); it++) begin
          addr_t a2;
          set_lines(sb.pool[rng.range(0, sb.pool.size() - 1)], 1);
          make_perm(sl.size());
          // the other RN-F only goes for the shared lines of the set (pick_rd may write)
          for (int j = 0; j < 2; j++) begin
            a2 = sl[rng.range(0, sl.size() - 1)];
            if (!is_priv(a2) && rop_open() < 3) sb.rop_push(pick_rd(), a2, 1, rng.range(0, 8));
          end
          for (int j = 0; j < sl.size() && go(ep); j++) go_acc(sl[perm[j]], rng.chance(it[0] ? 80 : 30), ep, 0);
          drain(ep);
          rop_wait(ep);
        end
      end

      RS_HAZARD_CHAIN: begin
        // chains on the same bytes, with independent accesses to other lines
        // between them: store, load (RAW), store over the loaded bytes (WAR),
        // wider store over both (WAW), wide load. The core holds each one back
        // until the one it depends on is answered and issues it right then.
        for (int it = 0; it < 40 && go(ep); it++) begin
          int w, off, ww;
          a   = rng.chance(70) ? priv[rng.range(0, priv.size() - 1)] : sb.pool[rng.range(0, sb.pool.size() - 1)];
          w   = rng.range(0, NWID - 2);
          ww  = w + 1;
          off = int'(rng.range(0, 63)) & ~((1 << ww) - 1);
          issue(a, 1, 0, w, off, 0, '0, ep, o);                       if (o == null) tick();
          go_acc(pick_line(), rng.chance(50), ep, 0);
          issue(a, 0, 0, w, off, 0, '0, ep, o);                       if (o == null) tick();
          go_acc(pick_line(), rng.chance(50), ep, 0);
          issue(a, 1, 0, w, off, 0, '0, ep, o);                       if (o == null) tick();
          issue(a, 1, 0, ww, off, 0, '0, ep, o);                      if (o == null) tick();
          issue(a, 0, 0, NWID - 1, off & ~7, 0, '0, ep, o);           if (o == null) tick();
          if ((it % 8) == 7) drain(ep);
        end
        drain(ep);
      end

      RS_NC_ERR: begin
        // the SN-F answers one non-cacheable line with an error (the top marks
        // it): every access to it should come back with err, accesses to the
        // other lines in the same bursts must not
        addr_t el;
        el = sb.nc_pool[sb.nc_pool.size() - 1];
        for (int it = 0; it < 30 && go(ep); it++) begin
          n = rng.range(2, 8);
          for (int j = 0; j < n && go(ep); j++) begin
            case (rng.range(0, 3))
              0, 1:    go_acc(el, rng.chance(50), ep, 0);
              2:       go_acc(sb.nc_pool[rng.range(0, sb.nc_pool.size() - 2)], rng.chance(50), ep, 0);
              default: go_acc(priv[rng.range(0, priv.size() - 1)], rng.chance(50), ep, 0);
            endcase
          end
          drain(ep);
        end
        drain(ep);
        sb.err_line.delete();
        if (ep == sb.epoch) sb.resync_line(el);             // nobody caches it: memory holds the value again
      end

      RS_CLEAN_PRIV: begin
        foreach (priv[i]) sb.rop_push(OP_CI, priv[i], 1, 0);
        rop_wait(ep);
      end

      default: ;
    endcase
    drain(ep);
    sb.rop_stop = 1;
  endtask

  // same barrier as rnf_model::sync
  task automatic barrier(input int n, input int unsigned ep);
    int unsigned g;
    g = sb.bar_gen;
    sb.bar_cnt++;
    if (sb.bar_cnt >= n) begin sb.bar_cnt = 0; sb.bar_gen++; end
    while (sb.bar_gen == g && ep == sb.epoch) tick();
  endtask

  // ---------------------------------------------------------------- lane coverage report
  // positions of cacheable accesses never reached, "" when every one was
  function automatic string lane_gaps();
    string s;
    s = "";
    for (int st = 0; st < 2; st++)
      for (int wl = 0; wl < NWID; wl++) begin
        int np, miss;
        np = 64 >> wl; miss = 0;
        for (int p = 0; p < np; p++) if (!pos_cov[st][wl][p]) miss++;
        if (miss != 0) s = {s, $sformatf(" %s %0dB: %0d of %0d positions", s_op(st), 1 << wl, miss, np)};
      end
    return s;
  endfunction

  initial begin
    int unsigned last, ep;
    drive_idle();
    while (sb == null) @(posedge clk);
    last = 0;
    forever begin
      while (sb.cmd_seq == last) tick();
      last = sb.cmd_seq;
      ep   = sb.epoch;
      scn_abort = 0;
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
