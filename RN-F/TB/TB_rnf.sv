`timescale 1ns/1ps
// =============================================================================
// Tb_RNF - RTL RN-F dcache and one behavioural RN-F on the same HN-F
//
//   RN-F 0 : rnf_dcache (RTL), core side driven by rnf_core_driver
//   RN-F 1 : rnf_model  (behavioural), random CHI traffic on the same lines
//   HN-F   : RTL,  SN-F : snf_model
//
//  The two RN-Fs snoop each other through the HN-F. Checking is done by the
//  chi_tb_pkg scoreboard (load data, SWMR on the mirrored line states, memory
//  image at the end); the dcache's line states are mirrored from its tag
//  writes.
//
//  Sequence: NUM_ROUNDS random rounds (knobs re-rolled per round), optional
//  MID_RESETS rounds that are reset in the middle of traffic, then the flush:
//  the behavioural RN-F writes its lines back and sends CleanInvalid for every
//  line, which pulls the dcache's dirty lines out through snoops. After that
//  the memory image must hold every value and the dcache must be empty.
// =============================================================================
module Tb_RNF;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
  import chi_tb_pkg::*;

  parameter int SEED                   = 1;
  parameter int LOG_MODE               = 3;
  parameter int VERBOSE                = 1;
  parameter int NUM_TRACKERS           = 4;
  parameter int NUM_ROUNDS             = 8;
  parameter int OPS_PER_ROUND          = 50;       // behavioural RN-F ops; the core does CORE_OPS_MULT times as many accesses
  parameter int CORE_OPS_MULT          = 4;
  parameter int NC_PCT                 = 10;       // core accesses that are non-cacheable
  parameter bit RANDOMIZE_KNOBS        = 1;
  parameter int MID_RESETS             = 0;
  parameter bit ISOLATE_FAILURES       = 1;
  parameter int SETS                   = 16;
  parameter int WAYS                   = 4;
  parameter int RX_FIFO_DEPTH          = 4;
  parameter int CMD_TIMEOUT_CYCLES     = 400000;
  parameter int OVERALL_TIMEOUT_CYCLES = 8000000;

  localparam int       NRNF  = 2;
  localparam node_id_e HN_ID = 7'h40;
  localparam node_id_e SN_ID = 7'h00;
  localparam int       AW    = 39;
  localparam int       DW    = 64;
  localparam int       IW    = 4;

  logic clk    = 0;
  logic resetn = 0;
  always #5 clk = ~clk;

  // ---------------------------------------------------------------- interfaces
  chi_req rnf_req_if    [NRNF]();
  chi_snp rnf_snp_if    [NRNF]();
  chi_rsp rnf_rsp_rx_if [NRNF]();
  chi_rsp rnf_rsp_tx_if [NRNF]();
  chi_dat rnf_dat_rx_if [NRNF]();
  chi_dat rnf_dat_tx_if [NRNF]();

  chi_req snf_req_if();
  chi_rsp snf_rsp_if();
  chi_dat snf_dat_tx_if(), snf_dat_rx_if();

  rnf_core_if #(.AddrSize(AW), .MaxAccessWidth(DW), .ReqIdxWidth(IW)) core_if();

  // ---------------------------------------------------------------- HN-F
  HN_F #(
    .NUM_TRACKERS   (NUM_TRACKERS),
    .NUM_RNF        (NRNF),
    .AddrWidth      (39),
    .DataWidth      (128),
    .CacheLineWidth (512),
    .TxCreditMax    (15)
  ) hnf (
    .clk        (clk),
    .resetn     (resetn),
    .rnf_req    (rnf_req_if),
    .rnf_snp    (rnf_snp_if),
    .rnf_rsp_rx (rnf_rsp_rx_if),
    .rnf_rsp_tx (rnf_rsp_tx_if),
    .rnf_dat_rx (rnf_dat_rx_if),
    .rnf_dat_tx (rnf_dat_tx_if),
    .snf_req    (snf_req_if),
    .snf_rsp    (snf_rsp_if),
    .snf_dat_tx (snf_dat_tx_if),
    .snf_dat_rx (snf_dat_rx_if)
  );

  // ---------------------------------------------------------------- RN-F 0: RTL dcache + core driver
  rnf_dcache #(
    .NODEID(7'h01), .HNID(HN_ID), .AddrSize(AW), .MaxAccessWidth(DW), .ReqIdxWidth(IW),
    .SETS(SETS), .WAYS(WAYS), .RX_FIFO_DEPTH(RX_FIFO_DEPTH)
  ) dut (
    .clk(clk), .resetn(resetn),
    .core(core_if),
    .req_tx(rnf_req_if[0]), .snp_rx(rnf_snp_if[0]),
    .rsp_rx(rnf_rsp_tx_if[0]), .rsp_tx(rnf_rsp_rx_if[0]),
    .dat_rx(rnf_dat_tx_if[0]), .dat_tx(rnf_dat_rx_if[0])
  );

  rnf_core_driver #(
    .NAME("RNF0"), .IDX(0), .SEED(SEED * 7 + 1), .AddrSize(AW), .MaxAccessWidth(DW), .ReqIdxWidth(IW),
    .OPS_MULT(CORE_OPS_MULT), .NC_PCT(NC_PCT)
  ) drv (
    .clk(clk), .resetn(resetn), .core(core_if)
  );

  // ---------------------------------------------------------------- RN-F 1: behavioural, SN-F
  rnf_model #(.NODEID(7'h02), .HNID(HN_ID), .NAME("RNF1"), .IDX(1), .SEED(SEED * 7 + 2)) rnf1 (
    .clk(clk), .resetn(resetn),
    .req_tx(rnf_req_if[1]), .snp_rx(rnf_snp_if[1]),
    .rsp_rx(rnf_rsp_tx_if[1]), .rsp_tx(rnf_rsp_rx_if[1]),
    .dat_rx(rnf_dat_tx_if[1]), .dat_tx(rnf_dat_rx_if[1])
  );

  snf_model #(.NAME("SNF"), .IDX(0), .NODEID(SN_ID), .SEED(SEED * 7 + 3)) snf (
    .clk(clk), .resetn(resetn),
    .req_rx(snf_req_if),
    .rsp_tx(snf_rsp_if),
    .dat_rx(snf_dat_tx_if),
    .dat_tx(snf_dat_rx_if)
  );

  // ---------------------------------------------------------------- knobs
  function automatic void base_knobs();
    sb.rx_cr_cap = 15;  sb.rx_cr_pct = 100; sb.snp_lat_max = 6; sb.bubble_pct = 0;
    sb.op_timeout = 2000; sb.max_out = 4; sb.ack_lat_max = 3;
    sb.dat_interleave = 0; sb.dat_shuffle = 0; sb.compack_readonce = 0; sb.compack_write = 0; sb.wb_race = 1;
    sb.snf_lat_max = 4; sb.snf_slow_pct = 0; sb.snf_err_pct = 0; sb.snf_retry_pct = 0;
    sb.snf_comp_first_pct = 0; sb.snf_early_comp_pct = 0; sb.snf_pcrd_early_pct = 0; sb.snf_pcrd_types = 4;
    sb.snp_opt_data_pct = 0; sb.zero_be_pct = 0; sb.nc_multi = 0; sb.unaligned_pct = 0;
    sb.hot_n = 4; sb.hot_pct = 50; sb.scn_ops = 40; sb.txid_mode = 0;
    sb.partial_states = 1;
    sb.err_line.delete();
    sb.default_weights();
  endfunction

  function automatic void randomize_knobs();
    tb_rng r;
    base_knobs();
    if (!RANDOMIZE_KNOBS) return;
    r = sb.trng;
    sb.rx_cr_cap      = r.range(1, 15);
    sb.rx_cr_pct      = r.chance(70) ? 100 : r.range(20, 90);
    sb.snp_lat_max    = r.range(0, 10);
    sb.bubble_pct     = r.chance(60) ? 0 : r.range(5, 30);
    sb.max_out        = r.range(1, 4);
    sb.ack_lat_max    = r.chance(70) ? r.range(0, 5) : r.range(6, 30);
    sb.dat_interleave = r.chance(30);
    sb.dat_shuffle    = r.chance(30);
    sb.snf_lat_max    = r.range(0, 8);
    sb.snf_slow_pct   = r.chance(75) ? 0 : r.range(5, 25);
    sb.snf_retry_pct  = r.chance(65) ? 0 : r.range(5, 40);
    sb.snf_pcrd_types = r.range(1, 16);
    sb.hot_n          = r.range(1, 4);
    sb.hot_pct        = r.range(20, 90);
    sb.compack_readonce = r.chance(40);
    sb.compack_write    = r.chance(40);
    if (r.chance(40)) sb.snf_comp_first_pct = r.range(5, 40);
    if (r.chance(40)) sb.snf_early_comp_pct = r.range(5, 60);
    if (r.chance(50)) sb.snf_pcrd_early_pct = r.range(10, 70);
    if (r.chance(40)) sb.snp_opt_data_pct   = r.range(10, 60);
  endfunction

  function automatic string knob_str();
    return $sformatf("cr_cap=%0d cr_pct=%0d snp_lat=%0d bubble=%0d max_out=%0d ack_lat=%0d intlv=%0b shuffle=%0b snf_lat=%0d snf_slow=%0d hot=%0d/%0d%% retry=%0d%%",
                     sb.rx_cr_cap, sb.rx_cr_pct, sb.snp_lat_max, sb.bubble_pct, sb.max_out, sb.ack_lat_max, sb.dat_interleave,
                     sb.dat_shuffle, sb.snf_lat_max, sb.snf_slow_pct, sb.hot_n, sb.hot_pct, sb.snf_retry_pct);
  endfunction

  // ---------------------------------------------------------------- reset / commands
  task automatic apply_reset(input string why);
    int unsigned t;
    tb_info($sformatf("TOP: reset (%s)", why));
    resetn = 0;
    sb.epoch++;
    sb.on_reset();
    repeat (10) @(posedge clk);
    resetn = 1;
    repeat (5) @(posedge clk);
    t = 0;
    while (sb.agents_done < NRNF && t < 5000) begin @(posedge clk); t++; end
  endtask

  task automatic start_cmd(input int c, input int arg, input string what);
    tb_info($sformatf("TOP: start %s", what));
    sb.agents_done = 0;
    sb.cmd         = c;
    sb.cmd_arg     = arg;
    sb.cmd_seq++;
  endtask

  task automatic issue_cmd(input int c, input int arg, input string what);
    int unsigned t, f0;
    f0 = sb.total_failures();
    start_cmd(c, arg, what);
    t = 0;
    while (sb.agents_done < NRNF && t < CMD_TIMEOUT_CYCLES) begin @(posedge clk); t++; end
    if (sb.agents_done < NRNF) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("TOP: %s did not finish within %0d cycles (%0d/%0d agents done) - resetting",
                                  what, CMD_TIMEOUT_CYCLES, sb.agents_done, NRNF));
      apply_reset("command timeout");
    end else begin
      tb_info($sformatf("TOP: done  %s (%0d cycles, failures so far %0d)", what, t, sb.total_failures()));
      if (ISOLATE_FAILURES && sb.total_failures() != f0 && c != CMD_FLUSH) apply_reset($sformatf("isolate the failure of %s", what));
    end
  endtask

  task automatic issue_cmd_reset_mid(input int arg, input string what);
    int unsigned t, d;
    start_cmd(CMD_RANDOM, arg, what);
    d = sb.trng.range(20, 600);
    t = 0;
    while (sb.agents_done < NRNF && t < d) begin @(posedge clk); t++; end
    apply_reset($sformatf("mid-traffic reset after %0d cycles", t));
    if (sb.agents_done < NRNF) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("TOP: %s - agents did not unwind after the reset", what));
    end
  endtask

  // ---------------------------------------------------------------- dcache monitors
  function automatic cstate_e to_cstate(cache_state_e s);
    case (s)
      CS_SC:   return ST_SC;
      CS_UC:   return ST_UC;
      CS_UD:   return ST_UD;
      CS_SD:   return ST_SD;
      default: return ST_I;
    endcase
  endfunction

  // link credits of the dcache's three TX channels: granted by the HN-F / used by the dcache
  int req_cr = 0, rsp_cr = 0, dat_cr = 0;
  // what the dcache was seen doing, kept across resets
  int unsigned c_hit = 0, c_miss = 0, c_evict = 0, c_wb = 0, c_snp_hit = 0, c_snp_data = 0;

  always @(posedge clk) begin
    if (!resetn) begin
      req_cr = 0; rsp_cr = 0; dat_cr = 0;
    end else if (sb != null) begin
      // mirror of the dcache line states, from its tag writes
      if (dut.tag_we) begin
        addr_t a;
        a = addr_t'({dut.tag_tag, dut.tag_set, 6'b0});
        sb.set_state(0, a, to_cstate(dut.tag_st));
        sb.check_swmr(a, "RNF0", {"dcache line -> ", dut.tag_st.name()});
      end
      c_hit      += dut.perf_hit;      c_miss     += dut.perf_miss;
      c_evict    += dut.perf_evict;    c_wb       += dut.perf_wb;
      c_snp_hit  += dut.perf_snp_hit;  c_snp_data += dut.perf_snp_data;
      if (dut.s_port && (dut.r_tag_re || dut.r_tag_we || dut.r_dat_re || dut.r_dat_we))
        sb.err("DUT_STATE", "dcache: request FSM and snoop FSM use the RAM port in the same cycle");

      if (rnf_req_if[0].flitv == Y) begin
        if (req_cr <= 0) sb.err("LINK_CREDIT", "RNF0: dcache sent a REQ flit without a credit");
        req_cr--;
        sb.n_req++;
      end
      if (rnf_rsp_rx_if[0].flitv == Y) begin
        if (rsp_cr <= 0) sb.err("LINK_CREDIT", "RNF0: dcache sent an RSP flit without a credit");
        rsp_cr--;
      end
      if (rnf_dat_rx_if[0].flitv == Y) begin
        if (dat_cr <= 0) sb.err("LINK_CREDIT", "RNF0: dcache sent a DAT flit without a credit");
        dat_cr--;
      end
      if (rnf_req_if[0].lcrdv    === 1'b1) req_cr++;
      if (rnf_rsp_rx_if[0].lcrdv === 1'b1) rsp_cr++;
      if (rnf_dat_rx_if[0].lcrdv === 1'b1) dat_cr++;

      if (rnf_req_if[0].flitv    == Y && rnf_req_if[0].flitpend    != Y) sb.err("FLITPEND", "RNF0 REQ: FLITV=1 with FLITPEND=0");
      if (rnf_rsp_rx_if[0].flitv == Y && rnf_rsp_rx_if[0].flitpend != Y) sb.err("FLITPEND", "RNF0 RSP: FLITV=1 with FLITPEND=0");
      if (rnf_dat_rx_if[0].flitv == Y && rnf_dat_rx_if[0].flitpend != Y) sb.err("FLITPEND", "RNF0 DAT: FLITV=1 with FLITPEND=0");
    end
  end

  // after the flush the dcache must be idle and hold no line
  function automatic void check_dcache_empty();
    int n;
    n = 0;
    for (int s = 0; s < SETS; s++)
      for (int w = 0; w < WAYS; w++)
        if (dut.u_tag_ram.state_mem[s][w] != CS_I) n++;
    if (n != 0)
      sb.err("DUT_STATE", $sformatf("end of test: dcache still holds %0d valid line(s) after every line got CleanInvalid", n));
    if (int'(dut.r_q.state) != 0 || int'(dut.s_q.state) != 0)      // 0 = R_IDLE / S_IDLE
      sb.err("DUT_STATE", $sformatf("end of test: dcache is not idle (request FSM state %0d, snoop FSM state %0d)", dut.r_q.state, dut.s_q.state));
  endfunction

  function automatic void need(string what, int unsigned n);
    if (n == 0)
      sb.err("COVERAGE", $sformatf("never reached: \"%s\" - the run did not test this, so a pass proves nothing about it", what));
  endfunction

  function automatic void report_dcache();
    $display("  ---- dcache (RNF0) ----");
    $display("  core accesses       : %0d loads, %0d stores (%0d non-cacheable), %0d picks of a busy line", drv.n_load, drv.n_store, drv.n_nc, drv.n_busy);
    $display("  hits / misses       : %0d / %0d", dut.perf_hits_r, dut.perf_misses_r);
    $display("  Evict / WriteBack   : %0d / %0d", dut.perf_evictions_r, dut.perf_writebacks_r);
    $display("  snoops              : %0d (%0d hit a line, %0d answered with data)", dut.perf_snoops_r, dut.perf_snoop_hits_r, dut.perf_snoop_data_r);
    $display("  RetryAck            : %0d", dut.perf_retries_r);
  endfunction

  function automatic void print_categories();
    int unsigned cc [string];
    cc = sb.cat_cnt;
    if (cc.size() == 0) return;
    $display("  ---- error categories ----");
    foreach (cc[c]) $display("  %s: %0d", c, cc[c]);
  endfunction

  // ---------------------------------------------------------------- main sequence
  initial begin
    chi_tb_pkg::chi_scoreboard s;

    log_init(LOG_MODE, VERBOSE);
    s = new(NRNF, 1, 6, 0);
    s.trng = new(SEED ^ 32'h5EED_0001);
    // coherent pool: 16 lines spread over 3 sets of the dcache, so every set
    // has more lines than ways; pool[0..3] are the hot lines
    for (int i = 1; i <= 16; i++) s.pool.push_back((addr_t'(i) << 16) | (addr_t'(int'(i % 3)) << 6));
    for (int i = 0; i < 4; i++)   s.nc_pool.push_back(addr_t'(39'h20_0000) + (addr_t'(i) << 16));
    s.flush_ci_rn = 1;       // the behavioural RN-F cleans every line at the end
    sb = s;
    base_knobs();

    $display("=== Tb_RNF: SEED=%0d NUM_TRACKERS=%0d SETS=%0d WAYS=%0d rounds=%0d x %0d ops (core x%0d) mid_resets=%0d ===",
             SEED, NUM_TRACKERS, SETS, WAYS, NUM_ROUNDS, OPS_PER_ROUND, CORE_OPS_MULT, MID_RESETS);

    resetn = 0;
    repeat (10) @(posedge clk);
    resetn = 1;
    repeat (5) @(posedge clk);

    for (int rd = 0; rd < NUM_ROUNDS; rd++) begin
      randomize_knobs();
      tb_info($sformatf("TOP: round %0d knobs: %s", rd, knob_str()));
      issue_cmd(CMD_RANDOM, OPS_PER_ROUND, $sformatf("random round %0d", rd));
    end

    if (MID_RESETS > 0) begin
      for (int rd = 0; rd < MID_RESETS; rd++) begin
        randomize_knobs();
        issue_cmd_reset_mid(OPS_PER_ROUND, $sformatf("reset round %0d", rd));
      end
      randomize_knobs();
      issue_cmd(CMD_RANDOM, OPS_PER_ROUND, "random round after the mid-traffic resets");
    end

    base_knobs();
    issue_cmd(CMD_FLUSH, 0, "end-of-test flush");
    repeat (500) @(posedge clk);
    sb.final_memory_check();
    check_dcache_empty();

    need("dcache: load / store hit",          c_hit);
    need("dcache: miss",                      c_miss);
    need("dcache: Evict",                     c_evict);
    need("dcache: WriteBackFull",             c_wb);
    need("dcache: snoop hit",                 c_snp_hit);
    need("dcache: snoop answered with data",  c_snp_data);

    sb.final_report(SEED);
    report_dcache();
    print_categories();
    if (sb.total_failures() == 0)
      $display("*** RN-F REGRESSION: ALL CHECKS PASSED (seed=%0d) ***", SEED);
    else
      $display("*** RN-F REGRESSION: %0d FAILURES (seed=%0d) - rerun with this SEED to reproduce ***", sb.total_failures(), SEED);
    $finish;
  end

  initial begin
    repeat (OVERALL_TIMEOUT_CYCLES) @(posedge clk);
    tb_err("TIMEOUT", $sformatf("TOP: regression did not finish within %0d cycles", OVERALL_TIMEOUT_CYCLES));
    if (sb != null) begin
      sb.final_report(SEED);
      report_dcache();
      print_categories();
    end
    $display("*** RN-F REGRESSION: GLOBAL TIMEOUT (seed=%0d) ***", SEED);
    $finish;
  end

endmodule
