`timescale 1ns/1ps


module Tb_HNF_Random;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;

  localparam int NUM_BEATS = 4; // CacheLineWidth(512)/DataWidth(128)

  // ---- knobs ----
  parameter int SEED                          = 1;
  parameter int NUM_OPS_PER_AGENT              = 200;   // stop condition
  parameter int PER_OP_WATCHDOG_CYCLES         = 150;   // one op's own timeout
  parameter int OVERALL_TIMEOUT_CYCLES         = 200000;
  parameter bit ENABLE_TXNID_COLLISION_STRESS  = 0;      // see header note

  logic clk = 0;
  logic resetn = 0;
  always #5 clk = ~clk;

  chi_scoreboard#(NUM_BEATS) sb;

  chi_req rnf_req_if    [2]();
  chi_snp rnf_snp_if    [2]();
  chi_rsp rnf_rsp_rx_if [2]();
  chi_rsp rnf_rsp_tx_if [2]();
  chi_dat rnf_dat_rx_if [2]();
  chi_dat rnf_dat_tx_if [2]();

  chi_req snf_req_if();
  chi_rsp snf_rsp_if();
  chi_dat snf_dat_tx_if(), snf_dat_rx_if();

  HN_F #(
    .NUM_TRACKERS   (4),
    .NUM_RNF        (2),
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

  rnf_model #(.NODEID(7'h01), .NAME("RNF0"), .NUM_BEATS(NUM_BEATS)) rnf0 (
    .clk(clk), .resetn(resetn),
    .req_tx(rnf_req_if[0]), .snp_rx(rnf_snp_if[0]),
    .rsp_rx(rnf_rsp_tx_if[0]), .rsp_tx(rnf_rsp_rx_if[0]),
    .dat_rx(rnf_dat_tx_if[0]), .dat_tx(rnf_dat_rx_if[0])
  );

  rnf_model #(.NODEID(7'h02), .NAME("RNF1"), .NUM_BEATS(NUM_BEATS)) rnf1 (
    .clk(clk), .resetn(resetn),
    .req_tx(rnf_req_if[1]), .snp_rx(rnf_snp_if[1]),
    .rsp_rx(rnf_rsp_tx_if[1]), .rsp_tx(rnf_rsp_rx_if[1]),
    .dat_rx(rnf_dat_tx_if[1]), .dat_tx(rnf_dat_rx_if[1])
  );

  snf_model #(.NAME("SNF"), .NUM_BEATS(NUM_BEATS)) snf (
    .clk(clk), .resetn(resetn),
    .req_rx(snf_req_if),
    .rsp_tx(snf_rsp_if),
    .dat_rx(snf_dat_tx_if),
    .dat_tx(snf_dat_rx_if)
  );

  // ---- shared address pool ----
  // Small on purpose: forces reuse/collisions across both RN-Fs (which
  // is exactly what stresses the crossbar) rather than every op landing
  // on its own untouched line. Line-aligned (bottom bits zero) since
  // that's what TSHR/HN_F assume throughout.
  logic [38:0] addr_pool[$] = '{
    39'h1_0000, 39'h2_0000, 39'h3_0000, 39'h4_0000, 39'h5_0000, 39'h6_0000,
    39'h7_0000, 39'h8_0000, 39'h9_0000, 39'hA_0000, 39'hB_0000, 39'hC_0000,
    39'hD_0000, 39'hE_0000, 39'hF_0000, 39'h10_0000
  };

  int unsigned rnf0_ops_done = 0;
  int unsigned rnf1_ops_done = 0;

  // Wraps a single issue_random_op call with its own watchdog so one
  // wedged transaction can't stall the whole random run - it gets
  // flagged and the loop moves on to try a different (unlocked) address
  // next iteration. See chi_scoreboard for why the address stays locked
  // forever after a genuine timeout rather than being force-freed.
  //
  // NOTE: issue_random_op is a task (it has to be - it contains
  // time-consuming calls), so its "did this call actually do anything"
  // result comes back via the `issued` output argument, not via an
  // assignment from the call itself (`x = some_task(...)` is only legal
  // for functions in SystemVerilog). `issued` is declared here and just
  // gets discarded if the watchdog branch wins the race instead - we
  // don't need to inspect it, only issue_random_op's own internals do.
  task automatic run_one_watched(string who);
    bit issued;
    fork
      begin
        if (who == "RNF0") rnf0.issue_random_op(sb, addr_pool, issued);
        else                rnf1.issue_random_op(sb, addr_pool, issued);
      end
      begin
        int wd;
        for (wd = 0; wd < PER_OP_WATCHDOG_CYCLES; wd++) @(posedge clk);
        sb.report_timeout(who, $sformatf("a random op did not complete within %0d cycles", PER_OP_WATCHDOG_CYCLES));
      end
    join_any
    disable fork;
  endtask

  initial begin
    $srandom(SEED);
    $display("=== Tb_HNF_Random: SEED=%0d NUM_OPS_PER_AGENT=%0d TXNID_COLLISION_STRESS=%0b ===",
              SEED, NUM_OPS_PER_AGENT, ENABLE_TXNID_COLLISION_STRESS);

    resetn = 0;
    repeat (5) @(posedge clk);
    resetn = 1;
    repeat (2) @(posedge clk);

    sb = new();

    if (ENABLE_TXNID_COLLISION_STRESS) begin
      // deliberately overlapping ranges - see header note. Expect this
      // to surface the known cross-tracker TxnID-matching gap if it's
      // still open (protocol errors / mismatched data / timeouts on the
      // SN-F-facing legs specifically).
      rnf0.configure_txnid_range(12'h000, 12'h00F);
      rnf1.configure_txnid_range(12'h000, 12'h00F);
    end else begin
      rnf0.configure_txnid_range(12'h000, 12'h7FF);
      rnf1.configure_txnid_range(12'h800, 12'h7FF);
    end

    fork
      rnf0.snoop_responder();
      rnf1.snoop_responder();
      snf.autonomous_responder(sb);
    join_none

    fork
      begin : rnf0_driver
        while (rnf0_ops_done < NUM_OPS_PER_AGENT) begin
          run_one_watched("RNF0");
          rnf0_ops_done++;
          repeat ($urandom_range(0, 3)) @(posedge clk);
        end
      end
      begin : rnf1_driver
        while (rnf1_ops_done < NUM_OPS_PER_AGENT) begin
          run_one_watched("RNF1");
          rnf1_ops_done++;
          repeat ($urandom_range(0, 3)) @(posedge clk);
        end
      end
    join

    // let any still-draining background writebacks (e.g. dirty-hit
    // followups triggered by the last few ops) settle before reporting
    repeat (200) @(posedge clk);

    sb.final_report();
    if (sb.num_read_mismatches == 0 && sb.num_protocol_errors == 0 && sb.num_timeouts == 0) begin
      $display("*** RANDOM REGRESSION: ALL CHECKS PASSED (seed=%0d) ***", SEED);
    end else begin
      $display("*** RANDOM REGRESSION: FAILURES DETECTED (seed=%0d) - rerun with this SEED to reproduce ***", SEED);
    end
    $finish;
  end

  initial begin
    int wd;
    for (wd = 0; wd < OVERALL_TIMEOUT_CYCLES; wd++) @(posedge clk);
    $display("TIMEOUT - random regression did not finish within %0d cycles", OVERALL_TIMEOUT_CYCLES);
    if (sb != null) sb.final_report();
    $finish;
  end

endmodule