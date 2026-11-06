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
//      3. end-of-test flush (write back / evict, then CleanInvalid every line)
//         followed by the final memory-image check
//  * a command that does not finish is reported as a TIMEOUT and recovered
//    with a reset (sb.epoch++ / sb.on_reset() / resetn pulse)
//
// Reproducibility: everything random comes from chi_tb_pkg::tb_rng seeded
// from SEED, so the same SEED gives the same run on any simulator.
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
  parameter bit LOCK_ALL               = 1;        // one op per line system-wide (HN_F has no address-hazard detection yet)
  parameter bit RUN_SCENARIOS          = 1;
  parameter int FIRST_SCENARIO         = 1;        // SC_PRODCONS
  parameter int LAST_SCENARIO          = SC_LAST;
  parameter int NUM_ROUNDS             = 8;
  parameter int OPS_PER_ROUND          = 50;       // per RN-F
  parameter bit RANDOMIZE_KNOBS        = 1;        // re-roll latencies / credits / interleaving per round
  parameter bit TXNID_COLLISION_STRESS = 0;        // allow overlapping TxnID ranges across RN-Fs in random rounds
  parameter bit RANDOM_RESETS          = 0;        // occasionally reset the system between random rounds
  parameter int CMD_TIMEOUT_CYCLES     = 200000;   // one command (scenario / round / flush)
  parameter int OVERALL_TIMEOUT_CYCLES = 5000000;

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
    .NUM_TRACKERS   (4),
    .NUM_RNF        (NRNF),
    .AddrWidth      (39),
    .DataWidth      (128),
    .CacheLineWidth (512),
    .TxCreditMax    (2)
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
    sb.hot_n = 4; sb.hot_pct = 50; sb.scn_ops = 40; sb.txid_mode = 0;
    sb.err_line.delete();
    sb.default_weights();
  endfunction

  // knobs for the directed scenarios; the "knob-driven" ones (TXNID_REUSE,
  // CREDIT_STARVE, RETRY_FLOOD, SNF_SLOW, DATA_INTLV, HOT_LINES) are plain
  // random traffic inside rnf_model::run_scenario with the knobs set here
  function automatic void apply_profile(int id);
    base_knobs();
    case (id)
      SC_TXNID_REUSE:   sb.txid_mode = 3;                                   // both RN-Fs share TxnIDs 0..1
      SC_CREDIT_STARVE: begin sb.rx_cr_cap = 1; sb.rx_cr_pct = 25; end
      SC_RETRY_FLOOD:   sb.snf_retry_pct = 50;
      SC_SNF_SLOW:      begin sb.snf_slow_pct = 30; sb.snf_lat_max = 12; end
      SC_SNF_ERR:       sb.err_line[line_of(sb.pool[2])] = 1;               // SN-F answers this line with DERR
      SC_DATA_INTLV:    begin sb.dat_interleave = 1; sb.dat_shuffle = 1; sb.bubble_pct = 25; end
      SC_HOT_LINES:     begin sb.hot_n = 1; sb.hot_pct = 90; sb.max_out = 4; end
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
    sb.ack_lat_max    = r.range(0, 5);
    sb.dat_interleave = r.chance(30);
    sb.dat_shuffle    = r.chance(30);
    sb.snf_lat_max    = r.range(0, 8);
    sb.snf_slow_pct   = r.chance(75) ? 0 : r.range(5, 25);
    sb.hot_n          = r.range(1, 4);
    sb.hot_pct        = r.range(20, 90);
    if (TXNID_COLLISION_STRESS && r.chance(50)) sb.txid_mode = int'(r.range(1, 2));
  endfunction

  function automatic string knob_str();
    return $sformatf("cr_cap=%0d cr_pct=%0d snp_lat=%0d bubble=%0d max_out=%0d intlv=%0b shuffle=%0b snf_lat=%0d snf_slow=%0d hot=%0d/%0d%% txid_mode=%0d",
                     sb.rx_cr_cap, sb.rx_cr_pct, sb.snp_lat_max, sb.bubble_pct, sb.max_out, sb.dat_interleave,
                     sb.dat_shuffle, sb.snf_lat_max, sb.snf_slow_pct, sb.hot_n, sb.hot_pct, sb.txid_mode);
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

  task automatic issue_cmd(input int c, input int arg, input string what);
    int unsigned t;
    tb_info($sformatf("TOP: start %s", what));
    sb.agents_done = 0;
    sb.cmd         = c;
    sb.cmd_arg     = arg;
    sb.cmd_seq++;
    t = 0;
    while (sb.agents_done < NRNF && t < CMD_TIMEOUT_CYCLES) begin @(posedge clk); t++; end
    if (sb.agents_done < NRNF) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("TOP: %s did not finish within %0d cycles (%0d/%0d agents done) - resetting",
                                  what, CMD_TIMEOUT_CYCLES, sb.agents_done, NRNF));
      apply_reset("command timeout");
    end else begin
      tb_info($sformatf("TOP: done  %s (%0d cycles, failures so far %0d)", what, t, sb.total_failures()));
    end
  endtask

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

    $display("=== Tb_HNF_Random: SEED=%0d LOCK_ALL=%0b scenarios=%0b(%0d..%0d) rounds=%0d x %0d ops/RN ===",
             SEED, LOCK_ALL, RUN_SCENARIOS, FIRST_SCENARIO, LAST_SCENARIO, NUM_ROUNDS, OPS_PER_ROUND);

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

    // 3. flush + final memory check
    base_knobs();
    issue_cmd(CMD_FLUSH, 0, "end-of-test flush");
    repeat (500) @(posedge clk);   // let trailing SN-F write-backs land
    sb.final_memory_check();

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
    $finish;
  end

endmodule
