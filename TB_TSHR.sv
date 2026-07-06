`timescale 1ns/1ps

// ============================= TODO ========================
// check hit RN-F1 response on things other than unique
// check RN-F0 and RN-F1 data combine on modified state on RN-F1
// check REQ_READ_UNIQUE on readunique and readshared snoops
//
// ---------------------------------------------------------------------
// EXTENDED with additional scenarios (E, F, G1, G2, H) that specifically
// target the three TODOs above. These are EXPECTED TO FAIL against the
// current TSHR implementation - that's the point. Each scenario is run
// under a watchdog so a hang in one scenario doesn't take down the rest
// of the regression, and the DUT is reset between every scenario so a
// stuck FSM state doesn't corrupt the next test.
// ---------------------------------------------------------------------

module rnf_model  #(
  parameter chi_pkg::node_id_e  NODEID = 7'h00,
  parameter string              NAME  = "RNF"
) (
  input  wire clk,
  input  wire resetn,
  chi_req.tx req_tx,
  chi_snp.rx snp_rx,
  chi_rsp.rx rsp_rx,
  chi_rsp.tx rsp_tx,
  chi_dat.rx dat_rx,
  chi_dat.tx dat_tx
);
  import chi_pkg::*;
  import common_pkg::*;

  // Unlimited receive capacity - always grant credit on every RX port.
  assign snp_rx.lcrdv = 1'b1;
  assign rsp_rx.lcrdv = 1'b1;
  assign dat_rx.lcrdv = 1'b1;

  // ---- REQ: send a request, waiting for TSHR's (state-gated) credit ----
  task automatic send_req(input logic [11:0] txnid, input req_opcode_e opcode,
                           input logic [38:0] addr, input width_e size);
    while (req_tx.lcrdv !== 1'b1) @(posedge clk);
    req_tx.flit            = '0;
    req_tx.flit.qos        = 4'h0;
    req_tx.flit.srcid      = NODEID;
    req_tx.flit.txnid      = txnid;
    req_tx.flit.opcode     = opcode;
    req_tx.flit.size       = size;
    req_tx.flit.addr       = addr;
    req_tx.flit.allowretry = Y;
    req_tx.flitv           = Y;
    req_tx.flitpend        = Y;
    $display("[%0t] %s: REQ  txnid=%0d opcode=%s addr=%0h", $time, NAME, txnid, opcode.name(), addr);
    @(posedge clk);
    req_tx.flitv    = N;
    req_tx.flitpend = N;
  endtask

  // ---- RSP: send SnpResp / CompAck (no data, no credit wait needed) ----
  task automatic send_rsp(input logic [11:0] txnid, input  chi_pkg::rsp_opcode_e opcode);
    rsp_tx.flit        = '0;
    rsp_tx.flit.srcid  = NODEID;
    rsp_tx.flit.txnid  = txnid;
    rsp_tx.flit.opcode = opcode;
    rsp_tx.flitv       = Y;
    rsp_tx.flitpend    = Y;
    $display("[%0t] %s: RSP  txnid=%0d opcode=%s", $time, NAME, txnid, opcode.name());
    @(posedge clk);
    rsp_tx.flitv    = N;
    rsp_tx.flitpend = N;
  endtask

  // ---- DAT: send snoop-response-data or write data ----
  task automatic send_dat(input logic [11:0] txnid, input dat_opcode_e opcode,
                           input logic [127:0] data, input logic [15:0] be);
    dat_tx.flit        = '0;
    dat_tx.flit.srcid  = NODEID;
    dat_tx.flit.txnid  = txnid;
    dat_tx.flit.opcode = opcode;
    dat_tx.flit.data   = data;
    dat_tx.flit.be     = be;
    dat_tx.flitv       = Y;
    dat_tx.flitpend    = Y;
    $display("[%0t] %s: DAT  txnid=%0d opcode=%s data=%0h", $time, NAME, txnid, opcode.name(), data);
    @(posedge clk);
    dat_tx.flitv    = N;
    dat_tx.flitpend = N;
  endtask

  // ---- wait for an incoming snoop ----
  task automatic wait_snp(output logic [11:0] o_txnid, output snp_opcode_e o_opcode, output logic o_rettosrc);
    while (!(snp_rx.flitv == Y)) @(posedge clk);
    o_txnid    = snp_rx.flit.txnid;
    o_opcode   = snp_rx.flit.opcode;
    o_rettosrc = snp_rx.flit.rettosrc;
    $display("[%0t] %s: SNP  txnid=%0d opcode=%s rettosrc=%0b", $time, NAME, o_txnid, o_opcode.name(), o_rettosrc);
  endtask

  // ---- wait for an incoming RSP (DBIDResp / CompDBIDResp / Comp) ----
  task automatic wait_rsp(output logic [11:0] o_txnid, output rsp_opcode_e o_opcode);
    while (!(rsp_rx.flitv == Y)) @(posedge clk);
    o_txnid  = rsp_rx.flit.txnid;
    o_opcode = rsp_rx.flit.opcode;
    $display("[%0t] %s: got RSP txnid=%0d opcode=%s", $time, NAME, o_txnid, o_opcode.name());
  endtask

  // ---- wait for an incoming DAT (CompData) ----
  task automatic wait_dat(output logic [11:0] o_txnid, output dat_opcode_e o_opcode,
                           output logic [127:0] o_data, output logic [15:0] o_be);
    while (!(dat_rx.flitv == Y)) @(posedge clk);
    o_txnid  = dat_rx.flit.txnid;
    o_opcode = dat_rx.flit.opcode;
    o_data   = dat_rx.flit.data;
    o_be     = dat_rx.flit.be;
    $display("[%0t] %s: got DAT txnid=%0d opcode=%s data=%0h", $time, NAME, o_txnid, o_opcode.name(), o_data);
  endtask

endmodule


module snf_model #(
  parameter string NAME = "SNF"
) (
  input  wire clk,
  input  wire resetn,
  chi_req.rx req_rx,
  chi_rsp.tx rsp_tx,
  chi_dat.rx dat_rx,
  chi_dat.tx dat_tx
);
  import chi_pkg::*;
  import common_pkg::*;

  // Unlimited receive capacity on the request/write-data channels.
  assign req_rx.lcrdv = 1'b1;
  assign dat_rx.lcrdv = 1'b1;

  task automatic wait_req(output logic [11:0] o_txnid, output req_opcode_e o_opcode,
                           output logic [38:0] o_addr, output width_e o_size);
    while (!(req_rx.flitv == Y)) @(posedge clk);
    o_txnid  = req_rx.flit.txnid;
    o_opcode = req_rx.flit.opcode;
    o_addr   = req_rx.flit.addr;
    o_size   = req_rx.flit.size;
    $display("[%0t] %s: got REQ txnid=%0d opcode=%s addr=%0h", $time, NAME, o_txnid, o_opcode.name(), o_addr);
  endtask

  task automatic send_rsp(input logic [11:0] txnid, input rsp_opcode_e opcode, input logic [11:0] dbid);
    rsp_tx.flit        = '0;
    rsp_tx.flit.txnid  = txnid;
    rsp_tx.flit.opcode = opcode;
    rsp_tx.flit.dbid   = dbid;
    rsp_tx.flitv       = Y;
    rsp_tx.flitpend    = Y;
    $display("[%0t] %s: RSP  txnid=%0d opcode=%s dbid=%0d", $time, NAME, txnid, opcode.name(), dbid);
    @(posedge clk);
    rsp_tx.flitv    = N;
    rsp_tx.flitpend = N;
  endtask

  task automatic send_dat(input logic [11:0] txnid, input dat_opcode_e opcode, input logic [127:0] data);
    dat_tx.flit        = '0;
    dat_tx.flit.txnid  = txnid;
    dat_tx.flit.opcode = opcode;
    dat_tx.flit.data   = data;
    dat_tx.flitv       = Y;
    dat_tx.flitpend    = Y;
    $display("[%0t] %s: DAT  txnid=%0d opcode=%s data=%0h", $time, NAME, txnid, opcode.name(), data);
    @(posedge clk);
    dat_tx.flitv    = N;
    dat_tx.flitpend = N;
  endtask

  task automatic wait_dat(output logic [11:0] o_txnid, output dat_opcode_e o_opcode,
                           output logic [127:0] o_data, output logic [15:0] o_be, output logic [11:0] o_dbid);
    while (!(dat_rx.flitv == Y)) @(posedge clk);
    o_txnid  = dat_rx.flit.txnid;
    o_opcode = dat_rx.flit.opcode;
    o_data   = dat_rx.flit.data;
    o_be     = dat_rx.flit.be;
    o_dbid   = dat_rx.flit.dbid;
    $display("[%0t] %s: got DAT txnid=%0d opcode=%s data=%0h dbid=%0d", $time, NAME, o_txnid, o_opcode.name(), o_data, o_dbid);
  endtask

endmodule


// ---------------------------------------------------------------------
// Watchdog-wrapped scenario runner. Runs CALL; if it doesn't finish
// within LIMIT cycles, flags a failure and moves on. Either way the DUT
// is force-reset afterwards so a stuck FSM state can't wreck the next
// scenario.
// ---------------------------------------------------------------------
`define RUN_SCN(CALL, LIMIT) \
  fork \
    begin \
      CALL ; \
    end \
    begin \
      repeat (LIMIT) @(posedge clk); \
      $display("[%0t] WATCHDOG: %s did not complete within %0d cycles - treating as FAIL", $time, `"CALL`", LIMIT); \
      errors++; \
    end \
  join_any \
  disable fork; \
  reset_dut();


module Tb_TSHR;
  import chi_pkg::*;
  import common_pkg::*;

  logic clk = 0;
  logic resetn = 0;
  always #5 clk = ~clk;

  int errors = 0;

  // RN-F0 / RN-F1 <-> TSHR
  chi_req req_arr[2]();
  chi_snp snp_arr[2]();
  chi_rsp rsp_tx_arr[2]();
  chi_rsp rsp_rx_arr[2]();
  chi_dat dat_tx_arr[2]();
  chi_dat dat_rx_arr[2]();

  // TSHR <-> SN-F
  chi_req sn_req();
  chi_rsp sn_rsp();
  chi_dat sn_dat_wr(), sn_dat_rd();

  TSHR #(
    .AddrWidth(39), .DataWidth(128),
    .HNFID(7'h40), .SNFID(7'h00)
  ) dut (
    .clk(clk), .resetn(resetn),
    .req_rx(req_arr),
    .snp_tx(snp_arr),
    .rsp_tx(rsp_tx_arr),
    .rsp_rx(rsp_rx_arr),
    .dat_tx(dat_tx_arr),
    .dat_rx(dat_rx_arr),
    .sn_req_tx(sn_req),
    .sn_rsp_rx(sn_rsp),
    .sn_dat_tx(sn_dat_wr),
    .sn_dat_rx(sn_dat_rd)
  );

  rnf_model #(.NODEID(7'h01), .NAME("RNF0")) rnf0 (
    .clk(clk), .resetn(resetn),
    .req_tx(req_arr[0]), .snp_rx(snp_arr[0]),
    .rsp_rx(rsp_tx_arr[0]), .rsp_tx(rsp_rx_arr[0]),
    .dat_rx(dat_tx_arr[0]), .dat_tx(dat_rx_arr[0])
  );

  rnf_model #(.NODEID(7'h02), .NAME("RNF1")) rnf1 (
    .clk(clk), .resetn(resetn),
    .req_tx(req_arr[1]), .snp_rx(snp_arr[1]),
    .rsp_rx(rsp_tx_arr[1]), .rsp_tx(rsp_rx_arr[1]),
    .dat_rx(dat_tx_arr[1]), .dat_tx(dat_rx_arr[1])
  );

  snf_model #(.NAME("SNF")) snf (
    .clk(clk), .resetn(resetn),
    .req_rx(sn_req),
    .rsp_tx(sn_rsp),
    .dat_rx(sn_dat_wr),
    .dat_tx(sn_dat_rd)
  );

  // small helpers just to satisfy task output-arg signatures above where
  // the value isn't actually checked by the caller
  function automatic logic [38:0] addr_dummy(); return '0; endfunction
  function automatic width_e size_dummy(); return WIDTH_1; endfunction

  // Force the DUT back to a known idle state between scenarios. Needed
  // because several scenarios below are expected to hang the FSM in a
  // wait-state given the current (incomplete) TSHR implementation - we
  // don't want one hung scenario to prevent the rest of the suite from
  // running.
  task automatic reset_dut();
    resetn = 0;
    repeat (3) @(posedge clk);
    resetn = 1;
    repeat (2) @(posedge clk);
  endtask

  // ---------------------------------------------------------------------
  // Scenario A: WriteUniquePtl from RNF0, RNF1 has the line (invalidate),
  //             write pushed to memory. (write-with-snoop / separate-resp
  //             flow: DBIDResp then, later, a separate Comp)
  // ---------------------------------------------------------------------
  task automatic scenario_write_unique_ptl();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0] snp_txnid;
    logic        rts;
    logic [127:0] data;
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario A: WriteUniquePtl (RNF0 -> snoop RNF1 -> SN) =====");
    txnid = 12'h0A1;
    fork
      rnf0.send_req(txnid, REQ_WRITE_UNIQUE_PTL, 39'h1000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_UNIQUE || rts !== 1'b0) begin
          $display("FAIL A: expected SnpUnique/rettosrc=0, got %s/%0b", rsp_op.name(), rts);
          errors++;
        end
        rnf1.send_rsp(txnid, RSP_SNP_RESP);
      end
    join

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_DBID_RESP) begin $display("FAIL A: expected DBIDResp, got %s", rsp_op.name()); errors++; end

    rnf0.send_dat(txnid, DAT_DATA_FLIT, 128'hDEAD_BEEF_0000_0000_0000_0000_CAFE_F00D, 16'hFFFF);

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_PTL) begin $display("FAIL A: expected WriteNoSnpPtl, got %s", req_op.name()); errors++; end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h5A5);

    snf.wait_dat(txnid, dat_op, data, be, dbid);
    if (dat_op !== DAT_NON_COPY_BACK_WR_DATA) begin $display("FAIL A: expected NonCopyBackWrData, got %s", dat_op.name()); errors++; end
    if (dbid !== 12'h5A5) begin $display("FAIL A: dbid not threaded through, got %0d", dbid); errors++; end
    if (data !== 128'hDEAD_BEEF_0000_0000_0000_0000_CAFE_F00D) begin $display("FAIL A: write data mismatch"); errors++; end

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_COMP) begin $display("FAIL A: expected Comp, got %s", rsp_op.name()); errors++; end
    rnf0.send_rsp(txnid, RSP_COMP_ACK);

    repeat (3) @(posedge clk);
    $display("Scenario A done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario B: ReadUnique from RNF0, RNF1 has the line (cache hit, CLEAN)
  //             and returns data directly on the snoop response - no SN access.
  // ---------------------------------------------------------------------
  task automatic scenario_read_unique_hit();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0] snp_txnid;
    logic        rts;
    logic [127:0] data;
    logic [15:0]  be;

    // NOTE: uses REQ_READ_SHARED (not REQ_READ_UNIQUE) on purpose. This
    // scenario is only about clean cache-hit forwarding, not about which
    // snoop opcode a given read maps to (that's Scenario G's job) - using
    // ReadShared keeps this test's SnpShared assertion valid regardless
    // of how the ReadUnique->SnpUnique mapping fix in G2 lands.
    $display("\n===== Scenario B: ReadShared, RNF1 cache HIT (clean) =====");
    txnid = 12'h0B2;
    fork
      rnf0.send_req(txnid, REQ_READ_SHARED, 39'h2000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
          $display("FAIL B: expected SnpShared/rettosrc=1, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf1.send_dat(txnid, DAT_SNP_RESP_DATA, 128'h1111_2222_3333_4444_5555_6666_7777_8888, 16'hFFFF);
      end
    join

    rnf0.wait_dat(txnid, dat_op, data, be);
    if (dat_op !== DAT_COMP_DATA) begin $display("FAIL B: expected CompData, got %s", dat_op.name()); errors++; end
    if (data !== 128'h1111_2222_3333_4444_5555_6666_7777_8888) begin $display("FAIL B: data mismatch"); errors++; end

    rnf0.send_rsp(txnid, RSP_COMP_ACK);
    repeat (3) @(posedge clk);
    $display("Scenario B done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario C: ReadUnique from RNF1, RNF0 misses (doesn't have the line)
  //             -> TSHR falls back to reading from SN-F.
  // ---------------------------------------------------------------------
  task automatic scenario_read_unique_miss();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0] snp_txnid;
    logic        rts;
    logic [127:0] data;
    logic [15:0]  be;
    logic [11:0]  dbid;

    // NOTE: uses REQ_READ_SHARED for the same reason as Scenario B - this
    // is testing the cache-miss fallback to SN-F, not the ReadUnique
    // snoop-opcode mapping (Scenario G covers that).
    $display("\n===== Scenario C: ReadShared, RNF0 cache MISS -> read from SN =====");
    txnid = 12'h0C3;
    fork
      rnf1.send_req(txnid, REQ_READ_SHARED, 39'h3000, WIDTH_16);
      begin
        rnf0.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
          $display("FAIL C: expected SnpShared/rettosrc=1, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        // RNF0 doesn't have the line - ack only, no data.
        rnf0.send_rsp(txnid, RSP_SNP_RESP);
      end
    join

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_READ_NO_SNOOP) begin $display("FAIL C: expected ReadNoSnoop, got %s", req_op.name()); errors++; end
    snf.send_dat(txnid, DAT_COMP_DATA, 128'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_0000_1111);

    rnf1.wait_dat(txnid, dat_op, data, be);
    if (dat_op !== DAT_COMP_DATA) begin $display("FAIL C: expected CompData, got %s", dat_op.name()); errors++; end
    if (data !== 128'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_0000_1111) begin $display("FAIL C: data mismatch"); errors++; end

    rnf1.send_rsp(txnid, RSP_COMP_ACK);
    repeat (3) @(posedge clk);
    $display("Scenario C done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario D: WriteBackFull from RNF0 - no snoop at all, combined
  //             CompDBIDResp, and no final CompAck phase.
  // ---------------------------------------------------------------------
  task automatic scenario_writeback_full();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [127:0] data;
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario D: WriteBackFull (no snoop, no CompAck) =====");
    txnid = 12'h0D4;
    rnf0.send_req(txnid, REQ_WRITE_BACK_FULL, 39'h4000, WIDTH_16);

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_COMP_DBID_RESP) begin $display("FAIL D: expected CompDBIDResp, got %s", rsp_op.name()); errors++; end

    rnf0.send_dat(txnid, DAT_DATA_FLIT, 128'h0BAD_F00D_0BAD_F00D_0BAD_F00D_0BAD_F00D, 16'hFFFF);

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_FULL) begin $display("FAIL D: expected WriteNoSnpFull, got %s", req_op.name()); errors++; end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h123);

    snf.wait_dat(txnid, dat_op, data, be, dbid);
    if (dat_op !== DAT_COPY_BACK_WR_DATA) begin $display("FAIL D: expected CopyBackWrData, got %s", dat_op.name()); errors++; end

    // No Comp / CompAck expected for writeback - just let a few cycles
    // pass and confirm TSHR returns to idle (ready for the next request).
    repeat (5) @(posedge clk);
    $display("Scenario D done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario E (NEW): WriteUniquePtl from RNF0 hits a MODIFIED line in
  //   RNF1. Agreed semantics: NO combining - RNF1's dirty data is simply
  //   written back to SN-F first (as its own complete WriteNoSnoopFull /
  //   CopyBackWrData transaction), and only afterwards does the TSHR
  //   proceed to accept and forward RNF0's new (partial) write data as a
  //   second, independent WriteNoSnoopPtl / NonCopyBackWrData transaction
  //   that overwrites it. So SN-F should see TWO separate write
  //   transactions, in this order, and RNF0 should only see its
  //   DBIDResp/Comp after the RNF1 writeback has fully drained.
  //
  //   The current TSHR's ST_SNOOP_WAIT write-branch only ever looks at
  //   the RSP channel for the write flow (it never reads the DAT channel
  //   RNF1 used here), so it will never even notice the dirty data ->
  //   the FSM is expected to hang at that first step (watchdog flags it).
  // ---------------------------------------------------------------------
  task automatic scenario_write_hits_modified();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0] snp_txnid;
    logic        rts;
    logic [127:0] data;
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario E: WriteUniquePtl hits MODIFIED line in RNF1 (writeback-then-overwrite, no combine) =====");
    txnid = 12'h0E5;
    fork
      rnf0.send_req(txnid, REQ_WRITE_UNIQUE_PTL, 39'h5000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_UNIQUE) begin
          $display("FAIL E: expected SnpUnique on write-hits-modified, got %s", snp_op.name());
          errors++;
        end
        // RNF1's line is MODIFIED/dirty: it must hand back the dirty data
        // when invalidating, not just ack the invalidate.
        rnf1.send_dat(txnid, DAT_SNP_RESP_DATA, 128'hFEED_FACE_0000_0000_0000_0000_D00D_1234, 16'hFFFF);
      end
    join

    // Leg 1: TSHR should push RNF1's dirty data down to SN-F on its own,
    // as a full-line copy-back write, BEFORE touching RNF0's request.
    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_FULL) begin
      $display("FAIL E: expected WriteNoSnpFull for RNF1's dirty writeback, got %s", req_op.name());
      errors++;
    end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h700);

    snf.wait_dat(txnid, dat_op, data, be, dbid);
    if (dat_op !== DAT_COPY_BACK_WR_DATA) begin
      $display("FAIL E: expected CopyBackWrData for RNF1's dirty writeback, got %s", dat_op.name());
      errors++;
    end
    if (data !== 128'hFEED_FACE_0000_0000_0000_0000_D00D_1234) begin
      $display("FAIL E: RNF1 writeback data mismatch, got %0h", data);
      errors++;
    end
    if (dbid !== 12'h700) begin $display("FAIL E: writeback dbid not threaded through, got %0d", dbid); errors++; end

    // Leg 2: only now should RNF0's own write proceed.
    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_DBID_RESP) begin
      $display("FAIL E: expected DBIDResp for RNF0's write after RNF1's writeback drained, got %s", rsp_op.name());
      errors++;
    end

    rnf0.send_dat(txnid, DAT_DATA_FLIT, 128'hDEAD_DEAD_0000_0000_0000_0000_CAFE_CAFE, 16'hFFFF);

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_PTL) begin
      $display("FAIL E: expected WriteNoSnpPtl for RNF0's overwrite, got %s", req_op.name());
      errors++;
    end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h701);

    snf.wait_dat(txnid, dat_op, data, be, dbid);
    if (dat_op !== DAT_NON_COPY_BACK_WR_DATA) begin
      $display("FAIL E: expected NonCopyBackWrData for RNF0's overwrite, got %s", dat_op.name());
      errors++;
    end
    if (data !== 128'hDEAD_DEAD_0000_0000_0000_0000_CAFE_CAFE) begin
      $display("FAIL E: RNF0 overwrite data mismatch, got %0h", data);
      errors++;
    end

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_COMP) begin $display("FAIL E: expected final Comp to RNF0, got %s", rsp_op.name()); errors++; end
    rnf0.send_rsp(txnid, RSP_COMP_ACK);

    repeat (3) @(posedge clk);
    $display("Scenario E done (exposes missing modified-line writeback-then-overwrite sequencing on write-hit).");
  endtask

  // ---------------------------------------------------------------------
  // Scenario F (NEW): ReadUnique from RNF0 hits a MODIFIED line in RNF1.
  //   RNF1 returns the dirty data on the snoop response (as in scenario
  //   B), which the TSHR correctly forwards to RNF0 as CompData. BUT,
  //   because the line was dirty/modified, SN-F's copy of memory is now
  //   stale - a correct HN-F must push that dirty data down to SN-F as
  //   well. Current TSHR goes straight to ST_COMPACK_WAIT/ST_DONE with
  //   no SN-F write at all, so the wait below is expected to time out.
  // ---------------------------------------------------------------------
  task automatic scenario_read_hit_modified();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0] snp_txnid;
    logic        rts;
    logic [127:0] data;
    logic [15:0]  be;

    $display("\n===== Scenario F: ReadUnique hits MODIFIED line in RNF1 (memory writeback expected) =====");
    txnid = 12'h0F6;
    fork
      rnf0.send_req(txnid, REQ_READ_UNIQUE, 39'h6000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        // RNF1's line is dirty - it returns data on the snoop response.
        rnf1.send_dat(txnid, DAT_SNP_RESP_DATA, 128'hB16B_00B5_0000_0000_0000_0000_DEAD_10CC, 16'hFFFF);
      end
    join

    rnf0.wait_dat(txnid, dat_op, data, be);
    if (dat_op !== DAT_COMP_DATA) begin $display("FAIL F: expected CompData, got %s", dat_op.name()); errors++; end
    if (data !== 128'hB16B_00B5_0000_0000_0000_0000_DEAD_10CC) begin $display("FAIL F: data mismatch"); errors++; end
    rnf0.send_rsp(txnid, RSP_COMP_ACK);

    // MESI, no Owned state: once the dirty line is shared out, memory
    // must be brought up to date. The dirty data must make it back to
    // SN-F as its own full write-back transaction. Current TSHR has no
    // path for this at all - expect the watchdog branch below to fire.
    fork
      begin
        logic [11:0] dbid;
        snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
        if (req_op !== REQ_WRITE_NO_SNOOP_FULL) begin
          $display("FAIL F: expected WriteNoSnpFull for the memory writeback, got %s", req_op.name());
          errors++;
        end
        snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h7F0);

        snf.wait_dat(txnid, dat_op, data, be, dbid);
        if (dat_op !== DAT_COPY_BACK_WR_DATA) begin
          $display("FAIL F: expected CopyBackWrData for the memory writeback, got %s", dat_op.name());
          errors++;
        end
        if (data !== 128'hB16B_00B5_0000_0000_0000_0000_DEAD_10CC) begin
          $display("FAIL F: writeback data mismatch, got %0h", data);
          errors++;
        end
      end
      begin
        repeat (60) @(posedge clk);
        $display("FAIL F: SN-F never received a writeback of the dirty data returned on the read-hit-modified snoop");
        errors++;
      end
    join_any
    disable fork;

    repeat (3) @(posedge clk);
    $display("Scenario F done (exposes missing dirty-line writeback-on-read-hit logic - MESI requires memory be updated here).");
  endtask

  // ---------------------------------------------------------------------
  // Scenario G1 (NEW): ReadShared from RNF0 -> expect SnpShared/rettosrc=1
  //   sent to RNF1. This is the "control" case: with the current
  //   implementation, all TX_READ opcodes are treated identically, so
  //   this should actually PASS.
  // ---------------------------------------------------------------------
  task automatic scenario_read_shared_snoop_map();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    logic [11:0] snp_txnid;
    logic        rts;

    $display("\n===== Scenario G1: ReadShared -> expect SnpShared/rettosrc=1 =====");
    txnid = 12'h0071;
    fork
      rnf0.send_req(txnid, REQ_READ_SHARED, 39'h7000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
          $display("FAIL G1: expected SnpShared/rettosrc=1 for ReadShared, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf1.send_rsp(txnid, RSP_SNP_RESP); // RNF1 doesn't have the line
      end
    join

    // G1 only checks the snoop-opcode mapping for ReadShared; it
    // deliberately does not chase the rest of the transaction (that would
    // require standing up an SN-F responder for the read-miss path, which
    // is unrelated to what this scenario is checking). reset_dut() in the
    // RUN_SCN wrapper takes care of returning the FSM to idle afterward.
    $display("Scenario G1 done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario G2 (NEW): ReadUnique from RNF0 -> should map to SnpUnique
  //   (rettosrc=1) so any other sharer is forced to invalidate while
  //   still handing back its data, per the ReadUnique semantics called
  //   out in the TODOs. The current TSHR always sends SnpShared for any
  //   TX_READ opcode (it doesn't distinguish REQ_READ_UNIQUE from
  //   REQ_READ_SHARED/REQ_READ_CLEAN/REQ_READ_ONCE), so this is expected
  //   to FAIL.
  // ---------------------------------------------------------------------
  task automatic scenario_read_unique_snoop_map();
    logic [11:0] txnid;
    snp_opcode_e  snp_op;
    logic [11:0] snp_txnid;
    logic        rts;

    $display("\n===== Scenario G2: ReadUnique -> expect SnpUnique/rettosrc=1 (NOT SnpShared) =====");
    txnid = 12'h0072;
    fork
      rnf0.send_req(txnid, REQ_READ_UNIQUE, 39'h8000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_UNIQUE || rts !== 1'b1) begin
          $display("FAIL G2: expected SnpUnique/rettosrc=1 for ReadUnique, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf1.send_rsp(txnid, RSP_SNP_RESP);
      end
    join

    $display("Scenario G2 done (exposes ReadUnique vs ReadShared snoop-opcode mapping bug).");
  endtask

  // ---------------------------------------------------------------------
  // Scenario H (NEW): RNF0 and RNF1 issue requests in the very same
  //   cycle. req_rx[*].lcrdv is only asserted while state_q==ST_IDLE, and
  //   each rnf_model.send_req only holds flitv high for a single cycle
  //   with no retry logic on the requester side. So whichever request
  //   loses arbitration this cycle simply vanishes - it is not queued,
  //   retried, or NACKed. This scenario documents/exposes that gap: we
  //   expect exactly one of the two txns to be serviced, and want the
  //   watchdog to fire if somehow neither one gets through.
  // ---------------------------------------------------------------------
  task automatic scenario_simultaneous_requests();
    logic [11:0] txnid0, txnid1;
    snp_opcode_e  snp_op;
    logic [11:0]  snp_txnid;
    logic         rts;

    $display("\n===== Scenario H: simultaneous RNF0+RNF1 requests (arbitration / silent-drop check) =====");
    txnid0 = 12'h0AA0;
    txnid1 = 12'h0BB0;

    fork
      rnf0.send_req(txnid0, REQ_READ_ONCE, 39'h9000, WIDTH_16);
      rnf1.send_req(txnid1, REQ_READ_ONCE, 39'hA000, WIDTH_16);
    join

    // Only one of the two requests can actually be latched (whichever
    // wins req_gnt_idx_q priority this cycle); service whichever snoop
    // shows up, and flag a failure if neither ever does.
    fork
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        rnf1.send_rsp(snp_txnid, RSP_SNP_RESP);
      end
      begin
        rnf0.wait_snp(snp_txnid, snp_op, rts);
        rnf0.send_rsp(snp_txnid, RSP_SNP_RESP);
      end
      begin
        repeat (30) @(posedge clk);
        $display("FAIL H: neither RNF0 nor RNF1 ever received a snoop - both simultaneous requests appear lost");
        errors++;
      end
    join_any
    disable fork;

    repeat (20) @(posedge clk);
    $display("Scenario H done - manually confirm exactly one of txnid0/txnid1 was serviced, and decide whether silently dropping the loser (no retry/queue) is acceptable.");
  endtask

  initial begin
    resetn = 0;
    repeat (5) @(posedge clk);
    resetn = 1;
    repeat (2) @(posedge clk);

    `RUN_SCN(scenario_write_unique_ptl(), 200)
    `RUN_SCN(scenario_read_unique_hit(), 200)
    `RUN_SCN(scenario_read_unique_miss(), 200)
    `RUN_SCN(scenario_writeback_full(), 200)
    `RUN_SCN(scenario_write_hits_modified(), 200)
    `RUN_SCN(scenario_read_hit_modified(), 250)
    `RUN_SCN(scenario_read_shared_snoop_map(), 200)
    `RUN_SCN(scenario_read_unique_snoop_map(), 200)
    `RUN_SCN(scenario_simultaneous_requests(), 200)

    if (errors == 0) $display("\n*** ALL SCENARIOS PASSED ***");
    else              $display("\n*** %0d FAILURE(S) ***", errors);

    $finish;
  end

  // safety timeout - overall regression bound (each scenario also has
  // its own per-scenario watchdog above)
  initial begin
    #300000;
    $display("TIMEOUT - overall regression hung");
    $finish;
  end

endmodule