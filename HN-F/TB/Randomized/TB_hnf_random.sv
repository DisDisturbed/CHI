`timescale 1ns/1ps
// =============================================================================
// Tb_HNF_Random - top level for the chi_tb_pkg based HN-F environment
//
//  * creates the global scoreboard / config object chi_tb_pkg::sb that the
//    rnf_model / snf_model agents wait for before starting their threads
//  * drives the agents through the command mailbox in sb
//    (sb.cmd / sb.cmd_arg / sb.cmd_seq, agents count up sb.agents_done):
//      1. every directed scenario SC_* (apply_profile() sets the knobs the
//         "knob-driven" scenarios rely on)
//      2. NUM_ROUNDS constrained-random rounds, knobs re-randomised per round
//      3. MID_RESETS rounds that are reset in the middle of traffic, followed
//         by one clean round that must pass after them
//      4. end-of-test flush (write back / evict, then CleanInvalid every line)
//         followed by the final memory-image check and the DUT end-state
//         checks (trackers idle, P-credit pool empty, link credits conserved)
//      5. coverage check: every stress the run is meant to reach must have
//         been reached, otherwise the run fails with COVERAGE
//  * a command that does not finish is reported as a TIMEOUT and recovered
//    with a reset (sb.epoch++ / sb.on_reset() / resetn pulse)
//
// Reproducibility: everything random comes from chi_tb_pkg::tb_rng seeded
// from SEED, so the same SEED gives the same run on any simulator.
//
// Feature switches (all 1 by default). Each one enables a spec-legal
// behaviour of the RN-F / SN-F models; set one to 0 to see whether a failure
// depends on it. A switched-off feature is left out of the coverage check.
//   SNF_COMP_FIRST  SN-F sends a separate Comp before DBIDResp
//   SNF_COMP_EARLY  SN-F sends a separate Comp after DBIDResp, before the write data
//   SNF_EARLY_COMP  0 turns both of the above off
//   SNF_PCRD_EARLY  SN-F sends PCrdGrant before RetryAck; PCrdType 0..15
//   COMPACK_OPT     ExpCompAck=1 on ReadOnce / ReadNoSnp / WriteUnique / WriteNoSnp
//   SNP_OPT_DATA    clean line answers SnpUnique (RetToSrc=0) with data
//   ZERO_BE         partial writes with every byte enable 0
//   NC_MULTI        one RN-F has several ReadNoSnp / WriteNoSnp to one line in flight
//   UNALIGNED       ReadNoSnp / WriteNoSnpPtl / WriteUniquePtl with an address not aligned to Size
//   CLEAN_UNIQUE    CleanUnique requests
//   MAKE_UNIQUE     MakeUnique requests
//   EVICT           Evict requests
//   READ_NSD        ReadNotSharedDirty requests
//   PARTIAL_STATES  UCE / UDP cache states (WriteBackPtl, SnpRespDataPtl)
// =============================================================================
module Tb_HNF_Random;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
  import chi_tb_pkg::*;

  // ---------------------------------------------------------------- knobs
  parameter int SEED                   = 1;
  parameter int LOG_MODE               = 3;        // see chi_tb_pkg::log_init
  parameter int VERBOSE                = 1;
  parameter int NUM_TRACKERS           = 4;
  parameter bit LOCK_ALL               = 0;        // 1: one op per line system-wide (hides every same-line race)
  parameter bit RUN_SCENARIOS          = 1;
  parameter int FIRST_SCENARIO         = 1;        // SC_PRODCONS
  parameter int LAST_SCENARIO          = SC_LAST;
  parameter int NUM_ROUNDS             = 8;
  parameter int OPS_PER_ROUND          = 50;       // per RN-F
  parameter bit RANDOMIZE_KNOBS        = 1;        // re-roll latencies / credits / interleaving per round
  parameter bit TXNID_COLLISION_STRESS = 1;        // allow overlapping TxnID ranges across RN-Fs in random rounds
  parameter bit RANDOM_RESETS          = 0;        // occasionally reset the system between random rounds
  parameter int MID_RESETS             = 4;        // rounds reset in the middle of traffic
  parameter bit SNF_EARLY_COMP         = 1;
  parameter bit SNF_COMP_FIRST         = 1;
  parameter bit SNF_COMP_EARLY         = 1;
  localparam bit COMP_FIRST            = SNF_EARLY_COMP && SNF_COMP_FIRST;
  localparam bit COMP_EARLY            = SNF_EARLY_COMP && SNF_COMP_EARLY;
  parameter bit SNF_PCRD_EARLY         = 1;
  parameter bit COMPACK_OPT            = 1;
  parameter bit SNP_OPT_DATA           = 1;
  parameter bit ZERO_BE                = 1;
  parameter bit NC_MULTI               = 1;
  parameter bit UNALIGNED              = 1;
  parameter bit CLEAN_UNIQUE           = 1;        // RN-F sends CleanUnique (SC/SD -> UC/UD, I -> UCE)
  parameter bit MAKE_UNIQUE            = 1;        // RN-F sends MakeUnique (+ full-line store)
  parameter bit EVICT                  = 1;        // RN-F sends Evict for clean lines (besides silent eviction)
  parameter bit READ_NSD               = 1;        // RN-F sends ReadNotSharedDirty
  parameter bit PARTIAL_STATES         = 1;        // RN-F uses UCE / UDP: partial stores, WriteBackPtl, SnpRespDataPtl
  parameter bit CHECK_COVERAGE         = 1;
  parameter bit ISOLATE_FAILURES       = 1;        // reset after a command that failed, so failures do not cascade
  parameter int CMD_TIMEOUT_CYCLES     = 200000;   // one command (scenario / round / flush)
  parameter int OVERALL_TIMEOUT_CYCLES = 8000000;

  localparam int       NRNF  = 2;
  localparam node_id_e HN_ID = 7'h40;              // TSHR HNFID
  localparam node_id_e SN_ID = 7'h00;              // TSHR SNFID

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

  // ---------------------------------------------------------------- DUT
  HN_F #(
    .NUM_TRACKERS   (NUM_TRACKERS),
    .NUM_RNF        (NRNF),
    .AddrWidth      (39),
    .DataWidth      (128),
    .CacheLineWidth (512),
    .TxCreditMax    (15)
  ) dut (
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

  // ---------------------------------------------------------------- agents
  rnf_model #(.NODEID(7'h01), .HNID(HN_ID), .NAME("RNF0"), .IDX(0), .SEED(SEED * 7 + 1)) rnf0 (
    .clk(clk), .resetn(resetn),
    .req_tx(rnf_req_if[0]), .snp_rx(rnf_snp_if[0]),
    .rsp_rx(rnf_rsp_tx_if[0]), .rsp_tx(rnf_rsp_rx_if[0]),
    .dat_rx(rnf_dat_tx_if[0]), .dat_tx(rnf_dat_rx_if[0])
  );

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

  // ---------------------------------------------------------------- knob profiles
  function automatic void base_knobs();
    sb.rx_cr_cap = 15;  sb.rx_cr_pct = 100; sb.snp_lat_max = 6; sb.bubble_pct = 0;
    sb.op_timeout = 2000; sb.max_out = 4; sb.ack_lat_max = 3;
    sb.dat_interleave = 0; sb.dat_shuffle = 0; sb.compack_readonce = 0; sb.compack_write = 0; sb.wb_race = 1;
    sb.snf_lat_max = 4; sb.snf_slow_pct = 0; sb.snf_err_pct = 0; sb.snf_retry_pct = 0;
    sb.snf_comp_first_pct = 0; sb.snf_early_comp_pct = 0; sb.snf_pcrd_early_pct = 0; sb.snf_pcrd_types = 4;
    sb.snp_opt_data_pct = 0; sb.zero_be_pct = 0; sb.nc_multi = 0; sb.unaligned_pct = 0;
    sb.hot_n = 4; sb.hot_pct = 50; sb.scn_ops = 40; sb.txid_mode = 0;
    sb.err_line.delete();
    sb.default_weights();
    apply_feature_switches();
  endfunction

  // request types switched off never get picked (weight 0)
  function automatic void apply_feature_switches();
    sb.partial_states = PARTIAL_STATES;
    if (!CLEAN_UNIQUE) sb.w[OP_CU]   = 0;
    if (!MAKE_UNIQUE)  sb.w[OP_MU]   = 0;
    if (!EVICT)        sb.w[OP_EV]   = 0;
    if (!READ_NSD)     sb.w[OP_RNSD] = 0;
  endfunction

  // knobs for the directed scenarios; the "knob-driven" ones (TXNID_REUSE,
  // CREDIT_STARVE, RETRY_FLOOD, SNF_SLOW, DATA_INTLV, HOT_LINES, EARLY_COMP,
  // PCRD_EARLY, COMPACK_LATE, ZERO_BE) are plain random traffic inside
  // rnf_model::run_scenario with the knobs set here
  function automatic void apply_profile(int id);
    base_knobs();
    case (id)
      SC_TXNID_REUSE:   sb.txid_mode = 3;                                   // both RN-Fs share TxnIDs 0..1
      SC_CREDIT_STARVE: begin sb.rx_cr_cap = 1; sb.rx_cr_pct = 25; end
      SC_RETRY_FLOOD:   begin sb.snf_retry_pct = 50; sb.snf_pcrd_types = 16; end
      SC_SNF_SLOW:      begin sb.snf_slow_pct = 30; sb.snf_lat_max = 12; end
      SC_SNF_ERR:       sb.err_line[line_of(sb.pool[2])] = 1;               // SN-F answers this line with DERR
      SC_DATA_INTLV:    begin sb.dat_interleave = 1; sb.dat_shuffle = 1; sb.bubble_pct = 25; end
      SC_HOT_LINES:     begin sb.hot_n = 1; sb.hot_pct = 90; sb.max_out = 4; end
      SC_WU_SHARERS:    sb.snp_opt_data_pct = SNP_OPT_DATA ? 70 : 0;       // the sharer may return its clean copy
      SC_NONCOH:        sb.unaligned_pct = UNALIGNED ? 50 : 0;
      SC_PTL_MERGE:     sb.unaligned_pct = UNALIGNED ? 50 : 0;              // unaligned WriteUniquePtl onto a dirty line
      SC_WC_SNOOP_RACE: sb.snp_lat_max = 3;
      SC_SAME_CYCLE, SC_WW_RACE, SC_DIRTY_CHAIN: begin
        sb.snp_lat_max = 3;
        sb.snp_opt_data_pct = SNP_OPT_DATA ? 40 : 0;
        sb.zero_be_pct      = ZERO_BE ? 10 : 0;
      end
      SC_NC_PILEUP:     begin sb.nc_multi = NC_MULTI; sb.zero_be_pct = ZERO_BE ? 10 : 0; sb.unaligned_pct = UNALIGNED ? 30 : 0; end
      SC_EARLY_COMP: begin                                                  // every SN-F write answered with a separate, early Comp
        sb.snf_comp_first_pct = COMP_FIRST ? 40 : 0;
        sb.snf_early_comp_pct = COMP_EARLY ? 70 : 0;
        sb.hot_n = 2; sb.hot_pct = 80;
        sb.w[OP_WUF] = 14; sb.w[OP_WUP] = 14; sb.w[OP_WB] = 14; sb.w[OP_NWR] = 12; sb.w[OP_ST] = 18;
      end
      SC_PCRD_EARLY: begin                                                  // every SN-F request retried once
        sb.snf_retry_pct      = 100;
        sb.snf_pcrd_early_pct = SNF_PCRD_EARLY ? 60 : 0;
        sb.snf_pcrd_types     = SNF_PCRD_EARLY ? 16 : 1;
        sb.max_out = 4; sb.hot_n = 4; sb.hot_pct = 30;
      end
      SC_COMPACK_LATE: begin                                                // CompAck on everything, very late
        sb.compack_readonce = COMPACK_OPT;
        sb.compack_write    = COMPACK_OPT;
        sb.ack_lat_max      = 25;
        sb.hot_n = 1; sb.hot_pct = 85;
        sb.w[OP_RO] = 12; sb.w[OP_NRD] = 8; sb.w[OP_ST] = 18;
      end
      SC_ZERO_BE: begin
        sb.zero_be_pct = ZERO_BE ? 50 : 0;
        sb.w[OP_WUP] = 20; sb.w[OP_NWR] = 14;
      end
      default: ;
    endcase
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
    if (TXNID_COLLISION_STRESS && r.chance(60)) sb.txid_mode = int'(r.range(1, 3));
    if (COMPACK_OPT)    begin sb.compack_readonce = r.chance(40); sb.compack_write = r.chance(40); end
    if (COMP_FIRST && r.chance(40)) sb.snf_comp_first_pct = r.range(5, 40);
    if (COMP_EARLY && r.chance(40)) sb.snf_early_comp_pct = r.range(5, 60);
    if (SNF_PCRD_EARLY && r.chance(50)) sb.snf_pcrd_early_pct = r.range(10, 70);
    if (SNP_OPT_DATA   && r.chance(40)) sb.snp_opt_data_pct = r.range(10, 60);
    if (ZERO_BE        && r.chance(30)) sb.zero_be_pct = r.range(5, 30);
    if (NC_MULTI)      sb.nc_multi = r.chance(40);
    if (UNALIGNED && r.chance(40)) sb.unaligned_pct = r.range(10, 60);
  endfunction

  function automatic string knob_str();
    return $sformatf({"cr_cap=%0d cr_pct=%0d snp_lat=%0d bubble=%0d max_out=%0d ack_lat=%0d intlv=%0b shuffle=%0b snf_lat=%0d snf_slow=%0d hot=%0d/%0d%% txid_mode=%0d",
                      " | retry=%0d%% pcrd_types=%0d pcrd_early=%0d%% comp_first=%0d%% comp_early=%0d%% compack ro/wr=%0b/%0b snp_opt_data=%0d%% zero_be=%0d%% nc_multi=%0b unaligned=%0d%%"},
                     sb.rx_cr_cap, sb.rx_cr_pct, sb.snp_lat_max, sb.bubble_pct, sb.max_out, sb.ack_lat_max, sb.dat_interleave,
                     sb.dat_shuffle, sb.snf_lat_max, sb.snf_slow_pct, sb.hot_n, sb.hot_pct, sb.txid_mode,
                     sb.snf_retry_pct, sb.snf_pcrd_types, sb.snf_pcrd_early_pct, sb.snf_comp_first_pct, sb.snf_early_comp_pct,
                     sb.compack_readonce, sb.compack_write, sb.snp_opt_data_pct, sb.zero_be_pct, sb.nc_multi, sb.unaligned_pct);
  endfunction

  // ---------------------------------------------------------------- reset / commands
  task automatic wait_agents_idle(input int unsigned max_cycles);
    int unsigned t;
    t = 0;
    while (sb.agents_done < NRNF && t < max_cycles) begin @(posedge clk); t++; end
  endtask

  task automatic apply_reset(input string why);
    tb_info($sformatf("TOP: reset (%s)", why));
    resetn = 0;
    sb.epoch++;            // agents drop all in-flight work at their next clock edge
    sb.on_reset();         // scoreboard: caches lost, memory image kept
    repeat (10) @(posedge clk);
    resetn = 1;
    repeat (5) @(posedge clk);
    wait_agents_idle(5000); // let the aborted command threads unwind before the next command
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
      // a failed command can leave trackers stuck and lines locked; reset so
      // the next command is judged on its own and not on this one's wreckage
      if (ISOLATE_FAILURES && sb.total_failures() != f0 && c != CMD_FLUSH) apply_reset($sformatf("isolate the failure of %s", what));
    end
  endtask

  // a random round that is cut by a reset while traffic is in flight
  task automatic issue_cmd_reset_mid(input int arg, input string what);
    int unsigned t, d;
    start_cmd(CMD_RANDOM, arg, what);
    d = sb.trng.range(20, 600);
    t = 0;
    while (sb.agents_done < NRNF && t < d) begin @(posedge clk); t++; end
    if (sb.agents_done < NRNF) sb.cov_hit("top: reset with traffic in flight");
    apply_reset($sformatf("mid-traffic reset after %0d cycles", t));
    if (sb.agents_done < NRNF) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("TOP: %s - agents did not unwind after the reset", what));
    end
    check_dut_idle("right after a mid-traffic reset");
  endtask

  // ---------------------------------------------------------------- DUT monitors
  always @(posedge clk) begin
    if (resetn && sb != null) begin
      bool_cov();
      if (dut.alloc_drop)
        sb.err("DUT_STATE", "HN-F Allocator dropped a request: it gave out a REQ credit with no free tracker");
      chk_flitpend();
    end
  end

  // Strict CHI link layer rule on every TX interface of the HN-F: a cycle
  // with FLITV asserted must also have FLITPEND asserted. Catches a TX path
  // that uses FLITPEND as "more beats follow" and drops it on the last beat.
  function automatic void chk_flitpend_one(string name, yn_status_e flitv, yn_status_e flitpend);
    if (flitv == Y && flitpend != Y)
      sb.err("FLITPEND", $sformatf("HN-F %s: FLITV=1 with FLITPEND=0 (strict link layer: FLITV -> FLITPEND)", name));
  endfunction

  function automatic void chk_flitpend();
    chk_flitpend_one("SNP    -> RNF0", rnf_snp_if[0].flitv,    rnf_snp_if[0].flitpend);
    chk_flitpend_one("SNP    -> RNF1", rnf_snp_if[1].flitv,    rnf_snp_if[1].flitpend);
    chk_flitpend_one("RSP    -> RNF0", rnf_rsp_tx_if[0].flitv, rnf_rsp_tx_if[0].flitpend);
    chk_flitpend_one("RSP    -> RNF1", rnf_rsp_tx_if[1].flitv, rnf_rsp_tx_if[1].flitpend);
    chk_flitpend_one("DAT    -> RNF0", rnf_dat_tx_if[0].flitv, rnf_dat_tx_if[0].flitpend);
    chk_flitpend_one("DAT    -> RNF1", rnf_dat_tx_if[1].flitv, rnf_dat_tx_if[1].flitpend);
    chk_flitpend_one("REQ    -> SNF",  snf_req_if.flitv,       snf_req_if.flitpend);
    chk_flitpend_one("WDAT   -> SNF",  snf_dat_tx_if.flitv,    snf_dat_tx_if.flitpend);
  endfunction

  function automatic void bool_cov();
    bit all_busy, any_retry_wait, any_pool;
    if (rnf_req_if[0].flitv == Y && rnf_req_if[1].flitv == Y) begin
      sb.cov_hit("hnf: both RN-F REQ on the same cycle");
      if (line_of(rnf_req_if[0].flit.addr) == line_of(rnf_req_if[1].flit.addr))
        sb.cov_hit("hnf: both RN-F REQ on the same cycle, same line");
    end
    all_busy = 1; any_retry_wait = 0; any_pool = 0;
    for (int t = 0; t < NUM_TRACKERS; t++) begin
      if (dut.tracker_ready[t]) all_busy = 0;
      if (dut.trk_pcrd_req[t])  any_retry_wait = 1;
    end
    for (int k = 0; k < 16; k++) if (dut.pcrd_cnt_q[k] != 0) any_pool = 1;
    if (all_busy) sb.cov_hit("hnf: all trackers busy");
    if (all_busy && rnf_req_if[0].flitv != Y && rnf_req_if[1].flitv != Y) sb.cov_hit("hnf: all trackers busy (cycles)");
    if (any_pool && !any_retry_wait) sb.cov_hit("hnf: P-credit banked with no tracker waiting");
  endfunction

  // ---------------------------------------------------------------- DUT end-state checks
  function automatic void check_dut_idle(string when);
    bit all_idle;
    all_idle = 1;
    for (int t = 0; t < NUM_TRACKERS; t++)
      if (!dut.tracker_ready[t]) begin
        all_idle = 0;
        sb.err("DUT_STATE", $sformatf("%s: tracker %0d is not idle (txnid=%0d srcid=%0h addr=%0h)", when, t,
                                      dut.trk_txnid[t], dut.trk_srcid[t], dut.trk_addr[t]));
      end
    for (int k = 0; k < 16; k++)
      if (dut.pcrd_cnt_q[k] != 0)
        sb.err("DUT_STATE", $sformatf("%s: HN-F P-credit pool still holds %0d credit(s) of PCrdType %0d that no tracker used", when, dut.pcrd_cnt_q[k], k));
    if (all_idle) for (int a = 0; a < NUM_TRACKERS; a++)
      for (int b = 0; b < NUM_TRACKERS; b++)
        if (dut.trk_older_q[a][b])
          sb.err("DUT_STATE", $sformatf("%s: hazard order bit trk_older_q[%0d][%0d] still set with every tracker idle", when, a, b));
  endfunction

  function automatic void cr_eq(string what, int hn, int partner);
    if (hn != partner)
      sb.err("CREDIT_LEAK", $sformatf("%s: HN-F counts %0d link credits, the link partner counts %0d", what, hn, partner));
  endfunction

  // every credit counter in the HN-F must agree with the agent on the other end
  function automatic void check_credits();
    // HN-F TX ports: credits the RN-F / SN-F granted and HN-F has not spent
    cr_eq("SNP  -> RNF0", int'(dut.gen_snp_port[0].u_snp_arb.u_port_cred.credit_q),       rnf0.snp_gr - rnf0.snp_rc);
    cr_eq("SNP  -> RNF1", int'(dut.gen_snp_port[1].u_snp_arb.u_port_cred.credit_q),       rnf1.snp_gr - rnf1.snp_rc);
    cr_eq("RSP  -> RNF0", int'(dut.gen_reqtgt_port[0].u_rsp_tx_arb.u_port_cred.credit_q), rnf0.rsp_gr - rnf0.rsp_rc);
    cr_eq("RSP  -> RNF1", int'(dut.gen_reqtgt_port[1].u_rsp_tx_arb.u_port_cred.credit_q), rnf1.rsp_gr - rnf1.rsp_rc);
    cr_eq("DAT  -> RNF0", int'(dut.gen_reqtgt_port[0].u_dat_tx_arb.u_port_cred.credit_q), rnf0.dat_gr - rnf0.dat_rc);
    cr_eq("DAT  -> RNF1", int'(dut.gen_reqtgt_port[1].u_dat_tx_arb.u_port_cred.credit_q), rnf1.dat_gr - rnf1.dat_rc);
    cr_eq("REQ  -> SNF ", int'(dut.u_sn_req_arb.u_port_cred.credit_q),                    snf.req_gr - snf.req_rc);
    cr_eq("DAT  -> SNF ", int'(dut.u_sn_dat_tx_arb.u_port_cred.credit_q),                 snf.wd_gr - snf.wd_rc);
    // HN-F RX ports: credits HN-F granted and the partner has not spent
    cr_eq("REQ  <- RNF0", int'(dut.u_req_rx_arb.port_cr_q[0]),                            rnf0.req_cr_in - rnf0.req_cr_out);
    cr_eq("REQ  <- RNF1", int'(dut.u_req_rx_arb.port_cr_q[1]),                            rnf1.req_cr_in - rnf1.req_cr_out);
    cr_eq("RSP  <- RNF0", int'(dut.gen_rsp_rx_bridge[0].u_rsp_rx_cred.outstanding_q),     rnf0.rsp_cr_in - rnf0.rsp_cr_out);
    cr_eq("RSP  <- RNF1", int'(dut.gen_rsp_rx_bridge[1].u_rsp_rx_cred.outstanding_q),     rnf1.rsp_cr_in - rnf1.rsp_cr_out);
    cr_eq("DAT  <- RNF0", int'(dut.gen_dat_rx_bridge[0].u_dat_rx_cred.outstanding_q),     rnf0.dat_cr_in - rnf0.dat_cr_out);
    cr_eq("DAT  <- RNF1", int'(dut.gen_dat_rx_bridge[1].u_dat_rx_cred.outstanding_q),     rnf1.dat_cr_in - rnf1.dat_cr_out);
    cr_eq("RSP  <- SNF ", int'(dut.u_snf_rsp_cred.outstanding_q),                         snf.rsp_cr_in - snf.rsp_cr_out);
    cr_eq("DAT  <- SNF ", int'(dut.u_snf_dat_rx_cred.outstanding_q),                      snf.dat_cr_in - snf.dat_cr_out);
  endfunction

  function automatic void check_end_state();
    check_dut_idle("end of test");
    for (int k = 0; k < 16; k++)
      if (sb.pcrd_out[k] != 0)
        sb.err("PCRD", $sformatf("end of test: SN-F granted %0d P-credit(s) of PCrdType %0d that were never used by a resend", sb.pcrd_out[k], k));
    check_credits();
  endfunction

  // ---------------------------------------------------------------- coverage
  function automatic void need(bit en, string k);
    if (en && sb.cov_get(k) == 0)
      sb.err("COVERAGE", $sformatf("never reached: \"%s\" - the run did not test this, so a pass proves nothing about it", k));
  endfunction

  function automatic void check_coverage();
    bit full;
    full = RUN_SCENARIOS && FIRST_SCENARIO <= 1 && LAST_SCENARIO >= SC_LAST;
    if (!CHECK_COVERAGE || !full) begin
      tb_info("TOP: coverage check skipped (needs RUN_SCENARIOS=1 and the full scenario range)");
      return;
    end
    need(1,                 "tshr: enter ST_SNOOP_WAIT");
    need(1,                 "tshr: enter ST_SN_RETRY_WAIT");
    need(1,                 "tshr: enter ST_SN_COMP_WAIT");
    need(1,                 "tshr: enter ST_DIRTY_WB_DATA_SEND");
    need(1,                 "tshr: enter ST_COMPACK_WAIT");
    need(1,                 "tshr: dirty write-back after TX_READ");
    need(1,                 "tshr: dirty write-back after TX_WRITE_UNIQUE");
    need(1,                 "tshr: dirty write-back after TX_CMO");
    need(1,                 "hnf: all trackers busy");
    need(1,                 "snf: RetryAck");
    need(!LOCK_ALL && NUM_TRACKERS > 1, "tshr: enter ST_HAZARD_WAIT");
    need(!LOCK_ALL && NUM_TRACKERS > 1, "hnf: both RN-F REQ on the same cycle, same line");
    need(UNALIGNED,         "rn: unaligned ReadNoSnp");
    need(UNALIGNED,         "rn: unaligned WriteNoSnpPtl");
    need(UNALIGNED,         "rn: unaligned WriteUniquePtl");
    need(!LOCK_ALL,         "rn: WriteCleanFull lost its line to a snoop");
    need(!LOCK_ALL,         "sb: writes in flight on one line >=2");
    need(COMPACK_OPT,       "rn: CompAck on ReadOnce/ReadNoSnp");
    need(COMPACK_OPT,       "rn: CompAck on a write");
    need(COMP_FIRST,        "snf: Comp before DBIDResp");
    need(COMP_EARLY,        "snf: Comp after DBIDResp, before write data");
    need(SNF_PCRD_EARLY,    "snf: PCrdGrant before RetryAck");
    need(SNF_PCRD_EARLY,    "hnf: P-credit banked with no tracker waiting");
    need(1,                 "snf: PCrdType != 0");
    need(SNP_OPT_DATA,      "rn: clean SnpRespData_I on SnpUnique RetToSrc=0");
    need(ZERO_BE,           "rn: partial write with BE=0");
    need(MID_RESETS > 0,    "top: reset with traffic in flight");
    need(CLEAN_UNIQUE,      "rn: CleanUnique SC->UC");
    need(CLEAN_UNIQUE,      "rn: CleanUnique SD->UD");
    need(MAKE_UNIQUE,       "rn: MakeUnique");
    need(EVICT,             "rn: Evict");
    need(READ_NSD,          "rn: ReadNotSharedDirty");
    need(CLEAN_UNIQUE && PARTIAL_STATES, "rn: CleanUnique ->UCE");
    need(CLEAN_UNIQUE && PARTIAL_STATES, "rn: store UCE->UDP");
    need(CLEAN_UNIQUE && PARTIAL_STATES, "rn: WriteBackPtl");
    need(CLEAN_UNIQUE && PARTIAL_STATES, "rn: SnpRespDataPtl");
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
    s = new(NRNF, 1, 6, LOCK_ALL);
    s.trng = new(SEED ^ 32'h5EED_0001);
    // coherent pool: line-aligned, small on purpose to force collisions;
    // pool[0..3] double as the hot lines / directed-scenario addresses
    for (int i = 1; i <= 16; i++) s.pool.push_back(addr_t'(i) << 16);
    // non-coherent (ReadNoSnp / WriteNoSnp) lines, kept apart from the pool
    for (int i = 0; i < 4; i++)   s.nc_pool.push_back(addr_t'(39'h20_0000) + (addr_t'(i) << 16));
    sb = s;                // agents start their threads once this is non-null
    base_knobs();

    $display("=== Tb_HNF_Random: SEED=%0d NUM_TRACKERS=%0d LOCK_ALL=%0b scenarios=%0b(%0d..%0d) rounds=%0d x %0d ops/RN mid_resets=%0d ===",
             SEED, NUM_TRACKERS, LOCK_ALL, RUN_SCENARIOS, FIRST_SCENARIO, LAST_SCENARIO, NUM_ROUNDS, OPS_PER_ROUND, MID_RESETS);
    $display("=== features: comp_first=%0b comp_early=%0b pcrd_early=%0b compack_opt=%0b snp_opt_data=%0b zero_be=%0b nc_multi=%0b unaligned=%0b txid_collide=%0b ===",
             COMP_FIRST, COMP_EARLY, SNF_PCRD_EARLY, COMPACK_OPT, SNP_OPT_DATA, ZERO_BE, NC_MULTI, UNALIGNED, TXNID_COLLISION_STRESS);
    $display("=== requests: clean_unique=%0b make_unique=%0b evict=%0b read_nsd=%0b partial_states=%0b ===",
             CLEAN_UNIQUE, MAKE_UNIQUE, EVICT, READ_NSD, PARTIAL_STATES);

    resetn = 0;
    repeat (10) @(posedge clk);
    resetn = 1;
    repeat (5) @(posedge clk);

    // 1. directed scenarios
    if (RUN_SCENARIOS) begin
      for (int id = FIRST_SCENARIO; id <= LAST_SCENARIO; id++) begin
        apply_profile(id);
        issue_cmd(CMD_SCEN, id, $sformatf("scenario %0d", id));
      end
      base_knobs();
    end

    // 2. constrained-random rounds
    for (int rd = 0; rd < NUM_ROUNDS; rd++) begin
      randomize_knobs();
      tb_info($sformatf("TOP: round %0d knobs: %s", rd, knob_str()));
      issue_cmd(CMD_RANDOM, OPS_PER_ROUND, $sformatf("random round %0d", rd));
      if (RANDOM_RESETS && sb.trng.chance(20)) apply_reset("random mid-test reset");
    end

    // 3. resets in the middle of traffic, then one round that must run clean
    if (MID_RESETS > 0) begin
      for (int rd = 0; rd < MID_RESETS; rd++) begin
        randomize_knobs();
        tb_info($sformatf("TOP: reset round %0d knobs: %s", rd, knob_str()));
        issue_cmd_reset_mid(OPS_PER_ROUND, $sformatf("reset round %0d", rd));
      end
      randomize_knobs();
      tb_info($sformatf("TOP: post-reset round knobs: %s", knob_str()));
      issue_cmd(CMD_RANDOM, OPS_PER_ROUND, "random round after the mid-traffic resets");
    end

    // 4. flush + final memory check + DUT end state
    base_knobs();
    issue_cmd(CMD_FLUSH, 0, "end-of-test flush");
    repeat (500) @(posedge clk);   // let trailing SN-F write-backs land and RX credits refill
    sb.final_memory_check();
    check_end_state();

    // 5. coverage
    check_coverage();

    sb.final_report(SEED);
    print_categories();
    if (sb.total_failures() == 0)
      $display("*** HN-F REGRESSION: ALL CHECKS PASSED (seed=%0d) ***", SEED);
    else
      $display("*** HN-F REGRESSION: %0d FAILURES (seed=%0d) - rerun with this SEED to reproduce ***", sb.total_failures(), SEED);
    $finish;
  end

  // ---------------------------------------------------------------- global watchdog
  initial begin
    repeat (OVERALL_TIMEOUT_CYCLES) @(posedge clk);
    tb_err("TIMEOUT", $sformatf("TOP: regression did not finish within %0d cycles", OVERALL_TIMEOUT_CYCLES));
    if (sb != null) begin
      sb.final_report(SEED);
      print_categories();
    end
    $display("*** HN-F REGRESSION: GLOBAL TIMEOUT (seed=%0d) ***", SEED);
    $finish;
  end

endmodule
