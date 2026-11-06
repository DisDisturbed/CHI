`timescale 1ns/1ps
// =============================================================================
// chi_tb_pkg - shared infrastructure for the HN-F random/directed testbench
//
//  * own PRNG (xorshift32): identical sequence on Vivado/Verilator/Xcelium for
//    the same SEED (no $urandom / randomize() / shuffle() anywhere in the TB)
//  * logging with red timestamps for errors
//  * scoreboard: architectural value model (with concurrency-aware read
//    checking), per-RN cache-state mirror for SWMR checks, SN-F memory image
// =============================================================================
package chi_tb_pkg;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;

  localparam int NUM_BEATS = 4;            // 512-bit line / 128-bit data
  localparam int ADDR_W    = 39;
  typedef logic [ADDR_W-1:0] addr_t;
  typedef logic [511:0]      line_t;
  typedef logic [63:0]       bemask_t;

  typedef enum int { ST_I, ST_SC, ST_UC, ST_UD, ST_SD } cstate_e;
  typedef enum int { K_READ, K_WRITE, K_EXCL }          lockkind_e;

  // operation kinds (index also selects the weight in sb.w[])
  typedef enum int { OP_RS, OP_RC, OP_RU, OP_RO, OP_WUF, OP_WUP, OP_WB, OP_WC,
                     OP_CS, OP_CI, OP_MI, OP_ST, OP_SE, OP_NRD, OP_NWR } opkind_e;
  localparam int NUM_OPS = 15;

  // top -> agent commands
  localparam int CMD_NONE = 0, CMD_SCEN = 1, CMD_RANDOM = 2, CMD_FLUSH = 3;

  // directed scenarios
  localparam int SC_PRODCONS = 1,  SC_UNIQUE_RACE = 2, SC_WB_SNOOP_RACE = 3,
                 SC_WU_SHARERS = 4, SC_PTL_MERGE = 5,  SC_TXNID_REUSE = 6,
                 SC_CREDIT_STARVE = 7, SC_RETRY_FLOOD = 8, SC_SNF_SLOW = 9,
                 SC_SNF_ERR = 10,  SC_SILENT_EVICT = 11, SC_MAKEINV = 12,
                 SC_CMO_FLUSH = 13, SC_NONCOH = 14, SC_STATE_MATRIX = 15,
                 SC_DATA_INTLV = 16, SC_HOT_LINES = 17;
  localparam int SC_LAST = 17;

  // Resp[2:0] / RespErr[1:0] encodings - short TB aliases of the RTL package
  // definitions (chi_pkg::RESP_* / RESPERR_*), so TB and RTL share one source
  localparam logic [2:0] R_I = RESP_I, R_SC = RESP_SC, R_UC = RESP_UC, R_SD = RESP_SD,
                         R_I_PD = RESP_I_PD, R_SC_PD = RESP_SC_PD, R_UD_PD = RESP_UD_PD, R_SD_PD = RESP_SD_PD;
  localparam logic [1:0] ERR_OK = RESPERR_OK, ERR_EXOK = RESPERR_EXOK, ERR_DERR = RESPERR_DERR, ERR_NDERR = RESPERR_NDERR;

  // ---------------------------------------------------------------------------
  // logging. log_mode bit0: ANSI red timestamp, bit1: "ERROR:" line prefix
  // (Vivado's Tcl console colours these lines red), bit2: also call $error
  // ---------------------------------------------------------------------------
  int unsigned log_mode    = 3;
  int unsigned log_verbose = 1;
  string       c_red       = "";
  string       c_rst       = "";

  function automatic void log_init(int unsigned mode, int unsigned verbose);
    log_mode    = mode;
    log_verbose = verbose;
    if (mode[0]) begin
      c_red = $sformatf("%c[1;31m", 8'd27);
      c_rst = $sformatf("%c[0m",    8'd27);
    end
  endfunction

  function automatic void tb_info(string msg);
    if (log_verbose != 0) $display("[%0t] %s", $time, msg);
  endfunction

  function automatic void tb_err(string cat, string msg);
    string ts;
    ts = $sformatf("%s[%0t]%s", c_red, $time, c_rst);
    if (log_mode[1]) $display("ERROR: %s %s: %s", ts, cat, msg);
    else             $display("%s %s: %s", ts, cat, msg);
    if (log_mode[2]) $error("%s: %s", cat, msg);
  endfunction

  // ---------------------------------------------------------------------------
  // portable PRNG
  // ---------------------------------------------------------------------------
  class tb_rng;
    bit [31:0] s;
    function new(int unsigned seed);
      s = seed ^ 32'h9E3779B9;
      s = s * 32'h85EBCA6B + 32'h7F4A7C15;
      if (s == 0) s = 32'h1;
      repeat (4) void'(next());
    endfunction
    function bit [31:0] next();
      s = s ^ (s << 13);
      s = s ^ (s >> 17);
      s = s ^ (s << 5);
      return s;
    endfunction
    function int unsigned range(int unsigned lo, int unsigned hi);
      bit [31:0] r;
      r = next();
      if (hi <= lo) return lo;
      return lo + (r % (hi - lo + 1));
    endfunction
    function bit chance(int unsigned pct);
      bit [31:0] r;
      r = next();
      return (r % 100) < pct;
    endfunction
    function bit [63:0] r64();
      bit [31:0] a, b;
      a = next(); b = next();
      return {a, b};
    endfunction
    function logic [127:0] r128();
      bit [63:0] a, b;
      a = r64(); b = r64();
      return {a, b};
    endfunction
    function line_t rline();
      logic [127:0] a, b, c, d;
      a = r128(); b = r128(); c = r128(); d = r128();
      return {a, b, c, d};
    endfunction
  endclass

  // ---------------------------------------------------------------------------
  // helpers
  // ---------------------------------------------------------------------------
  function automatic int size_bytes(width_e s);
    case (s)
      WIDTH_1:  return 1;
      WIDTH_2:  return 2;
      WIDTH_4:  return 4;
      WIDTH_8:  return 8;
      WIDTH_16: return 16;
      WIDTH_32: return 32;
      default:  return 64;
    endcase
  endfunction

  function automatic int num_beats_for(width_e s);
    int n;
    n = size_bytes(s);
    return (n <= 16) ? 1 : (n / 16);
  endfunction

  function automatic bit [1:0] first_dataid(addr_t a, width_e s);
    int n;
    n = size_bytes(s);
    if (n <= 16)      return a[5:4];
    else if (n == 32) return {a[5], 1'b0};
    else              return 2'b00;
  endfunction

  function automatic addr_t line_of(addr_t a);
    return {a[ADDR_W-1:6], 6'b0};
  endfunction

  function automatic line_t merge_line(line_t base, line_t nd, bemask_t m);
    line_t r;
    r = base;
    for (int i = 0; i < 64; i++) if (m[i]) r[i*8 +: 8] = nd[i*8 +: 8];
    return r;
  endfunction

  function automatic bemask_t span_mask(addr_t a, width_e s);
    bemask_t m;
    int n, off;
    n = size_bytes(s);
    off = a[5:0];
    m = '0;
    for (int i = 0; i < n; i++) if ((off + i) < 64) m[off + i] = 1'b1;
    return m;
  endfunction

  // same "never written" pattern as the original scoreboard
  function automatic logic [127:0] default_beat(addr_t addr, int beat);
    logic [6:0] b7;
    b7 = beat[6:0];
    return {addr, 25'h0, b7} ^ 128'hA5A5_A5A5_A5A5_A5A5_A5A5_A5A5_A5A5_A5A5;
  endfunction

  function automatic line_t default_line(addr_t addr);
    line_t l;
    for (int b = 0; b < NUM_BEATS; b++) l[b*128 +: 128] = default_beat(addr, b);
    return l;
  endfunction

  // ---------------------------------------------------------------------------
  // small containers
  // ---------------------------------------------------------------------------
  // one in-flight RN transaction (responses are demuxed into the queues)
  class rn_txn;
    logic [11:0]      txnid;
    req_opcode_e      opcode;
    addr_t            addr;
    width_e           size;
    int unsigned      t0;
    int unsigned      retries;
    bit               allowretry;
    logic [3:0]       pcrdttype;
    logic [3:0]       memattr;
    local_rsp_flit_t  rsp_q [$];
    local_dat_flit_t  dat_q [$];
  endclass

  // tiny mutex (events instead of semaphore for simulator portability)
  // NOTE: the unlock method must not be called release() - that is a
  // reserved SystemVerilog keyword (force/release) and fails to compile.
  class tx_lock;
    bit   busy;
    event ev;
    function new(); busy = 0; endfunction
    task acquire();
      while (busy) @(ev);
      busy = 1;
    endtask
    function void unlock();
      busy = 0;
      -> ev;
    endfunction
  endclass

  class line_info;
    addr_t   a;
    line_t   cur;        // last committed architectural value
    bit      known;      // 0: value indeterminate (MakeInvalid / error / reset)
    line_t   pend;       // value of the write currently in flight
    bit      has_pend;
    bit      pend_unk;
    bit      wbusy;
    bit      excl;
    int      nbusy;
    cstate_e st   [];
    bit      busy [];
    function new(addr_t a_, int nrnf);
      a = a_; cur = default_line(a_); known = 1; has_pend = 0; pend_unk = 0;
      wbusy = 0; excl = 0; nbusy = 0; pend = '0;
      st = new[nrnf]; busy = new[nrnf];
      foreach (st[i]) begin st[i] = ST_I; busy[i] = 0; end
    endfunction
  endclass

  class rd_rec;
    addr_t a;
    string who;
    line_t allowed [$];
    bit    skip;
  endclass

  // ---------------------------------------------------------------------------
  // scoreboard + global TB configuration
  // ---------------------------------------------------------------------------
  class chi_scoreboard;
    // ---- static config ----
    int num_rnf, num_snf, ilv_lsb;
    bit lock_all;       // legacy behaviour: one op per line system-wide
    bit chk_ids;        // check TgtID/SrcID on HN->RN/SN traffic
    bit use_memattr;    // drive MemAttr on RN requests (coherent: 4'b1101, non-coherent: 4'b0001)
    // ---- runtime knobs (top randomises these per round) ----
    int unsigned rx_cr_cap, rx_cr_pct;       // credits this agent grants to HN
    int unsigned snp_lat_max, bubble_pct, op_timeout, max_out, ack_lat_max;
    bit          dat_interleave, dat_shuffle, compack_readonce, compack_write, wb_race;
    int unsigned snf_lat_max, snf_slow_pct, snf_err_pct, snf_retry_pct;
    int unsigned hot_n, hot_pct, scn_ops;
    int          txid_mode;      // 0 partitioned per RN, 1 overlap 0..15, 2 overlap 0..FFF, 3 overlap 0..1
    int unsigned w [NUM_OPS];
    addr_t       pool [$];
    addr_t       nc_pool [$];
    bit          err_line [addr_t];
    // ---- control ----
    int unsigned epoch;
    int          cmd, cmd_arg;
    int unsigned cmd_seq;
    int          agents_done;
    int          bar_cnt;
    int unsigned bar_gen;
    tb_rng       trng;
    // ---- model state ----
    line_info lines [addr_t];
    rd_rec    open_rd [$];
    line_t    snf_mem [addr_t];
    string    infl [string];
    int unsigned cat_cnt [string];
    // ---- stats ----
    int unsigned n_req, n_rd_chk, n_rd_unchk, n_rd_mis, n_wr, n_store, n_swmr,
                 n_proto, n_timeout, n_unexp, n_stale, n_mem_mis, n_retry, n_pcrd,
                 n_snp, n_snp_data, n_err_inj, n_err_seen, n_reset, n_misroute, n_ops_done;
    int unsigned n_op [NUM_OPS];
    localparam int MAX_PRINT_PER_CAT = 25;

    function new(int nrnf, int nsnf, int ilv, bit lock_all_);
      num_rnf = nrnf; num_snf = nsnf; ilv_lsb = ilv; lock_all = lock_all_;
      chk_ids = 1; use_memattr = 1;
      rx_cr_cap = 15; rx_cr_pct = 100; snp_lat_max = 6; bubble_pct = 0;
      op_timeout = 2000; max_out = 4; ack_lat_max = 3;
      dat_interleave = 0; dat_shuffle = 0; compack_readonce = 0; compack_write = 0; wb_race = 1;
      snf_lat_max = 4; snf_slow_pct = 0; snf_err_pct = 0; snf_retry_pct = 0;
      hot_n = 4; hot_pct = 50; scn_ops = 40; txid_mode = 0;
      epoch = 0; cmd = 0; cmd_arg = 0; cmd_seq = 0; agents_done = 0; bar_cnt = 0; bar_gen = 0;
      n_req = 0; n_rd_chk = 0; n_rd_unchk = 0; n_rd_mis = 0; n_wr = 0; n_store = 0; n_swmr = 0;
      n_proto = 0; n_timeout = 0; n_unexp = 0; n_stale = 0; n_mem_mis = 0; n_retry = 0; n_pcrd = 0;
      n_snp = 0; n_snp_data = 0; n_err_inj = 0; n_err_seen = 0; n_reset = 0; n_misroute = 0; n_ops_done = 0;
      for (int i = 0; i < NUM_OPS; i++) begin n_op[i] = 0; w[i] = 10; end
      default_weights();
    endfunction

    function void default_weights();
      w[OP_RS]=14; w[OP_RC]=6; w[OP_RU]=14; w[OP_RO]=6; w[OP_WUF]=7; w[OP_WUP]=8;
      w[OP_WB]=8;  w[OP_WC]=3; w[OP_CS]=2;  w[OP_CI]=2; w[OP_MI]=1; w[OP_ST]=14;
      w[OP_SE]=5;  w[OP_NRD]=4; w[OP_NWR]=4;
    endfunction

    // ---- central error funnel (limits print flood, keeps counters) ----
    function void err(string cat, string msg);
      int unsigned c;
      c = cat_cnt.exists(cat) ? cat_cnt[cat] : 0;
      cat_cnt[cat] = c + 1;
      if (c < MAX_PRINT_PER_CAT)       tb_err(cat, msg);
      else if (c == MAX_PRINT_PER_CAT) tb_err(cat, "(further messages of this category suppressed, still counted)");
      case (cat)
        "PROTOCOL", "LINK_CREDIT", "TXNID_REUSE", "ILLEGAL_OPCODE", "SPURIOUS_RESPERR": n_proto++;
        default: ;
      endcase
    endfunction

    // ---- address map ----
    function int snf_of(addr_t a);
      addr_t sh;
      sh = a >> ilv_lsb;
      return int'(sh % num_snf);
    endfunction

    function line_info get_line(addr_t a);
      addr_t la;
      la = line_of(a);
      if (!lines.exists(la)) lines[la] = new(la, num_rnf);
      return lines[la];
    endfunction

    // ---- per-line operation admission ----
    function bit try_begin(addr_t a, int rn, lockkind_e k);
      line_info L;
      lockkind_e kk;
      L = get_line(a);
      kk = lock_all ? K_EXCL : k;
      if (L.excl || L.busy[rn]) return 0;
      if (kk == K_EXCL && L.nbusy > 0) return 0;
      if (kk == K_WRITE && L.wbusy) return 0;
      L.busy[rn] = 1; L.nbusy++;
      if (kk == K_EXCL)  L.excl  = 1;
      if (kk == K_WRITE) L.wbusy = 1;
      return 1;
    endfunction

    function void end_op(addr_t a, int rn, lockkind_e k);
      line_info L;
      lockkind_e kk;
      L = get_line(a);
      kk = lock_all ? K_EXCL : k;
      L.busy[rn] = 0; if (L.nbusy > 0) L.nbusy--;
      if (kk == K_EXCL)  L.excl  = 0;
      if (kk == K_WRITE) L.wbusy = 0;
    endfunction

    // ---- architectural value tracking ----
    function rd_rec rd_begin(addr_t a, string who);
      line_info L;
      rd_rec r;
      L = get_line(a);
      r = new();
      r.a = L.a; r.who = who; r.skip = !L.known;
      if (L.known) r.allowed.push_back(L.cur);
      if (L.has_pend && !L.pend_unk) r.allowed.push_back(L.pend);
      open_rd.push_back(r);
      return r;
    endfunction

    function void rd_end(rd_rec r, line_t got, bit data_err, string ctx);
      bit ok;
      foreach (open_rd[i]) if (open_rd[i] == r) begin open_rd.delete(i); break; end
      n_rd_chk++;
      if (data_err || r.skip || r.allowed.size() == 0) begin n_rd_unchk++; return; end
      ok = 0;
      foreach (r.allowed[i]) if (r.allowed[i] === got) ok = 1;
      if (!ok) begin
        n_rd_mis++;
        err("READ_MISMATCH", $sformatf("%s %s addr=%0h: data matches none of %0d legal values", r.who, ctx, r.a, r.allowed.size()));
        for (int b = 0; b < NUM_BEATS; b++)
          if (got[b*128 +: 128] !== r.allowed[0][b*128 +: 128])
            tb_err("READ_MISMATCH", $sformatf("   beat%0d got=%032h expected(oldest legal)=%032h", b, got[b*128 +: 128], r.allowed[0][b*128 +: 128]));
      end
    endfunction

    function void wr_begin(addr_t a, line_t data, bemask_t m, string who);
      line_info L;
      line_t nv;
      L = get_line(a);
      nv = (m == '1) ? data : merge_line(L.cur, data, m);
      L.has_pend = 1;
      L.pend_unk = (!L.known && m != '1);
      L.pend     = nv;
      if (!L.pend_unk) foreach (open_rd[i]) if (open_rd[i].a == L.a && !open_rd[i].skip) open_rd[i].allowed.push_back(nv);
    endfunction

    function void wr_end(addr_t a, string who);
      line_info L;
      L = get_line(a);
      if (L.has_pend) begin
        if (!L.pend_unk) begin L.cur = L.pend; L.known = 1; end
        L.has_pend = 0; L.pend_unk = 0;
      end
      n_wr++;
    endfunction

    function void wr_abort(addr_t a);
      line_info L;
      L = get_line(a);
      L.has_pend = 0; L.pend_unk = 0; L.known = 0;
    endfunction

    function void local_store(addr_t a, int rn, line_t data, bemask_t m, string who);
      line_info L;
      line_t nv;
      L = get_line(a);
      nv = merge_line(L.cur, data, m);
      if (L.known) begin
        L.cur = nv;
        foreach (open_rd[i]) if (open_rd[i].a == L.a && !open_rd[i].skip) open_rd[i].allowed.push_back(nv);
      end
      n_store++;
    endfunction

    function void mark_unknown(addr_t a);
      line_info L;
      L = get_line(a);
      L.known = 0;
    endfunction

    function bit is_err_line(addr_t a);
      return err_line.exists(line_of(a)) && err_line[line_of(a)];
    endfunction

    // data held in a cache must be (a legal) current value, otherwise the copy is stale
    function void check_cached(addr_t a, line_t d, string who, string ctx);
      line_info L;
      bit ok;
      L = get_line(a);
      if (!L.known) return;
      ok = (d === L.cur) || (L.has_pend && !L.pend_unk && d === L.pend);
      if (!ok) begin
        n_stale++;
        err("STALE_DATA", $sformatf("%s %s addr=%0h: cached copy matches neither committed nor in-flight value (beat0 got=%032h exp=%032h)",
                                    who, ctx, L.a, d[127:0], L.cur[127:0]));
      end
    endfunction

    // ---- coherence-state mirror / SWMR ----
    function void set_state(int rn, addr_t a, cstate_e s);
      line_info L;
      L = get_line(a);
      L.st[rn] = s;
    endfunction

    function string st_str(line_info L);
      string s;
      s = "";
      foreach (L.st[i]) s = {s, $sformatf(" RN%0d=%s", i, L.st[i].name())};
      return s;
    endfunction

    function void check_swmr(addr_t a, string who, string ctx);
      line_info L;
      int nuniq, nvalid, nsd;
      L = get_line(a);
      nuniq = 0; nvalid = 0; nsd = 0;
      foreach (L.st[i]) begin
        if (L.st[i] != ST_I) nvalid++;
        if (L.st[i] == ST_UC || L.st[i] == ST_UD) nuniq++;
        if (L.st[i] == ST_SD) nsd++;
      end
      if (nuniq > 0 && nvalid > 1) begin
        n_swmr++;
        err("SWMR", $sformatf("%s (%s) addr=%0h: Unique copy coexists with other valid copies:%s", who, ctx, L.a, st_str(L)));
      end else if (nsd > 1) begin
        n_swmr++;
        err("SWMR", $sformatf("%s (%s) addr=%0h: more than one SD owner:%s", who, ctx, L.a, st_str(L)));
      end
    endfunction

    function void check_invalid_except(addr_t a, int rn, string who, string ctx);
      line_info L;
      L = get_line(a);
      foreach (L.st[i]) if (i != rn && L.st[i] != ST_I) begin
        n_swmr++;
        err("SWMR", $sformatf("%s (%s) addr=%0h: RN%0d still holds %s after transaction completed:%s", who, ctx, L.a, i, L.st[i].name(), st_str(L)));
      end
    endfunction

    function void check_no_dirty(addr_t a, string who, string ctx);
      line_info L;
      L = get_line(a);
      foreach (L.st[i]) if (L.st[i] == ST_UD || L.st[i] == ST_SD) begin
        n_swmr++;
        err("SWMR", $sformatf("%s (%s) addr=%0h: RN%0d still dirty (%s) after clean operation completed", who, ctx, L.a, i, L.st[i].name()));
      end
    endfunction

    // ---- SN-F memory image ----
    function line_t snf_line(addr_t a);
      addr_t la;
      la = line_of(a);
      if (snf_mem.exists(la)) return snf_mem[la];
      return default_line(la);
    endfunction

    function void snf_write(addr_t a, line_t d, bemask_t m);
      addr_t la;
      la = line_of(a);
      snf_mem[la] = merge_line(snf_line(la), d, m);
    endfunction

    function void mem_check_line(addr_t a, string who, string ctx);
      line_info L;
      line_t    m;
      L = get_line(a);
      if (!L.known) return;
      m = snf_line(L.a);              // part-select on a function call result is not legal SV
      if (m !== L.cur) begin
        n_mem_mis++;
        err("MEM_MISMATCH", $sformatf("%s (%s) addr=%0h: memory does not hold the architectural value (mem beat0=%032h exp=%032h)",
                                      who, ctx, L.a, m[127:0], L.cur[127:0]));
      end
    endfunction

    function void final_memory_check();
      foreach (lines[a]) mem_check_line(a, "FINAL", "end-of-test flush");
    endfunction

    // ---- in-flight tracking for diagnosable timeouts ----
    function void note_inflight(string who, logic [11:0] txnid, addr_t a, req_opcode_e op);
      infl[$sformatf("%s:%0d", who, txnid)] = $sformatf("addr=%0h opcode=%s txnid=%0d", a, op.name(), txnid);
    endfunction

    function void clear_inflight(string who, logic [11:0] txnid);
      string k;
      k = $sformatf("%s:%0d", who, txnid);
      if (infl.exists(k)) infl.delete(k);
    endfunction

    function void report_timeout(string who, logic [11:0] txnid, string msg);
      string k;
      k = $sformatf("%s:%0d", who, txnid);
      n_timeout++;
      if (infl.exists(k)) err("TIMEOUT", $sformatf("%s: %s (%s)", who, msg, infl[k]));
      else                err("TIMEOUT", $sformatf("%s: %s (no in-flight info, txnid=%0d)", who, msg, txnid));
    endfunction

    // ---- reset handling: dirty cache data is lost, memory image survives ----
    function void on_reset();
      n_reset++;
      open_rd.delete();
      foreach (lines[a]) begin
        line_info L;
        L = lines[a];
        foreach (L.st[i]) begin L.st[i] = ST_I; L.busy[i] = 0; end
        L.nbusy = 0; L.excl = 0;
        L.known    = !L.wbusy && !L.has_pend;
        L.cur      = snf_line(a);
        L.has_pend = 0; L.pend_unk = 0; L.wbusy = 0;
      end
      bar_cnt = 0; bar_gen++;
      infl.delete();
    endfunction

    // ---- reporting ----
    function int unsigned total_failures();
      return n_rd_mis + n_swmr + n_proto + n_timeout + n_unexp + n_stale + n_mem_mis + n_misroute;
    endfunction

    function void final_report(int seed);
      $display("\n================= SCOREBOARD SUMMARY (seed=%0d) =================", seed);
      $display("  requests issued     : %0d (retries %0d, PCrdGrants %0d)", n_req, n_retry, n_pcrd);
      $display("  ops by kind         : RS=%0d RC=%0d RU=%0d RO=%0d WUF=%0d WUP=%0d WB=%0d WC=%0d CS=%0d CI=%0d MI=%0d ST=%0d SE=%0d NRD=%0d NWR=%0d",
               n_op[0], n_op[1], n_op[2], n_op[3], n_op[4], n_op[5], n_op[6], n_op[7], n_op[8], n_op[9], n_op[10], n_op[11], n_op[12], n_op[13], n_op[14]);
      $display("  reads checked       : %0d (%0d unchecked: unknown value / RespErr)", n_rd_chk, n_rd_unchk);
      $display("  writes committed    : %0d, silent stores %0d", n_wr, n_store);
      $display("  snoops answered     : %0d", n_snp);
      $display("  SN-F errors injected: %0d (seen at RN %0d)", n_err_inj, n_err_seen);
      $display("  resets applied      : %0d", n_reset);
      $display("  ---- failures ----");
      $display("  read mismatches     : %0d", n_rd_mis);
      $display("  stale cached data   : %0d", n_stale);
      $display("  SWMR violations     : %0d", n_swmr);
      $display("  protocol errors     : %0d", n_proto);
      $display("  unexpected responses: %0d", n_unexp);
      $display("  SN-F misroutes      : %0d", n_misroute);
      $display("  memory mismatches   : %0d", n_mem_mis);
      $display("  timeouts            : %0d", n_timeout);
      $display("========================================================\n");
    endfunction
  endclass

  // single global handle, created by the top-level testbench
  chi_scoreboard sb;

endpackage