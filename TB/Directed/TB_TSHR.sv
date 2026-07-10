`timescale 1ns/1ps

// ============================= TODO ========================
// check hit RN-F1 response on things other than unique
// check RN-F0 and RN-F1 data combine on modified state on RN-F1
// check REQ_READ_UNIQUE on readunique and readshared snoops
//
// ---------------------------------------------------------------------
// UPDATED for the current beat-based TSHR + HN_F architecture:
//   - TSHR now moves a full cache line (NumBeats = CacheLineWidth /
//     DataWidth, 4 by default) per DAT transfer instead of one flit.
//     rnf_model/snf_model's send_dat/wait_dat now loop over beats,
//     using flitpend to know when the burst ends.
//   - The DUT is now HN_F (owns the real interfaces, routes to
//     NUM_TRACKERS TSHR instances internally via the arbiter/router
//     modules) instead of a single raw TSHR.
//   - Added Scenario I: two independent transactions genuinely
//     concurrent across two different trackers, including RN-F0 being
//     simultaneously "requester" of its own txn and "snoop target" of
//     RN-F1's txn at the same time - exactly the case the whole
//     multi-tracker rebuild exists to handle correctly.
// ---------------------------------------------------------------------

module rnf_model #(
  parameter chi_pkg::node_id_e NODEID    = 7'h00,
  parameter string              NAME     = "RNF",
  parameter int                 NUM_BEATS = 4 // must match HN_F's CacheLineWidth/DataWidth
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

  // ---- REQ: send a request, waiting for HN_F's (state-gated) credit ----
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
  task automatic send_rsp(input logic [11:0] txnid, input chi_pkg::rsp_opcode_e opcode);
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

  // ---- DAT: send a full NUM_BEATS-beat burst (snoop-response-data or
  // write data). flitpend stays Y until the last beat, matching how
  // TSHR itself signals multi-beat transfers.
  task automatic send_dat(input logic [11:0] txnid, input dat_opcode_e opcode,
                           input logic [127:0] data_beats [NUM_BEATS], input logic [15:0] be);
    for (int b = 0; b < NUM_BEATS; b++) begin
      dat_tx.flit        = '0;
      dat_tx.flit.srcid  = NODEID;
      dat_tx.flit.txnid  = txnid;
      dat_tx.flit.opcode = opcode;
      dat_tx.flit.dataid = b[1:0];
      dat_tx.flit.data   = data_beats[b];
      dat_tx.flit.be     = be;
      dat_tx.flitv       = Y;
      dat_tx.flitpend    = (b == NUM_BEATS-1) ? N : Y;
      $display("[%0t] %s: DAT  txnid=%0d opcode=%s beat=%0d/%0d data=%0h",
                $time, NAME, txnid, opcode.name(), b, NUM_BEATS-1, data_beats[b]);
      @(posedge clk);
      dat_tx.flitv    = N;
      dat_tx.flitpend = N;
    end
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

  // ---- wait for an incoming DAT (CompData), collecting all NUM_BEATS
  // beats before returning.
  task automatic wait_dat(output logic [11:0] o_txnid, output dat_opcode_e o_opcode,
                           output logic [127:0] o_data_beats [NUM_BEATS], output logic [15:0] o_be);
    for (int b = 0; b < NUM_BEATS; b++) begin
      while (!(dat_rx.flitv == Y)) @(posedge clk);
      o_txnid         = dat_rx.flit.txnid;
      o_opcode        = dat_rx.flit.opcode;
      o_data_beats[b] = dat_rx.flit.data;
      o_be            = dat_rx.flit.be;
      $display("[%0t] %s: got DAT txnid=%0d opcode=%s beat=%0d/%0d data=%0h",
                $time, NAME, o_txnid, o_opcode.name(), b, NUM_BEATS-1, o_data_beats[b]);
      @(posedge clk);
    end
  endtask

endmodule


module snf_model #(
  parameter string NAME      = "SNF",
  parameter int    NUM_BEATS = 4
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

  // ---- DAT: send a full NUM_BEATS-beat burst back to HN_F (e.g. a read
  // response). No dbid on this direction - matches sn_dat_rx's flit shape.
  task automatic send_dat(input logic [11:0] txnid, input dat_opcode_e opcode,
                           input logic [127:0] data_beats [NUM_BEATS]);
    for (int b = 0; b < NUM_BEATS; b++) begin
      dat_tx.flit        = '0;
      dat_tx.flit.txnid  = txnid;
      dat_tx.flit.opcode = opcode;
      dat_tx.flit.dataid = b[1:0];
      dat_tx.flit.data   = data_beats[b];
      dat_tx.flitv       = Y;
      dat_tx.flitpend    = (b == NUM_BEATS-1) ? N : Y;
      $display("[%0t] %s: DAT  txnid=%0d opcode=%s beat=%0d/%0d data=%0h",
                $time, NAME, txnid, opcode.name(), b, NUM_BEATS-1, data_beats[b]);
      @(posedge clk);
      dat_tx.flitv    = N;
      dat_tx.flitpend = N;
    end
  endtask

  // ---- wait for an incoming DAT (write data), collecting all
  // NUM_BEATS beats before returning. dbid is the same across every
  // beat of a given burst (HN_F threads it through unchanged), so it's
  // just captured on the first beat and returned once.
  task automatic wait_dat(output logic [11:0] o_txnid, output dat_opcode_e o_opcode,
                           output logic [127:0] o_data_beats [NUM_BEATS], output logic [15:0] o_be,
                           output logic [11:0] o_dbid);
    for (int b = 0; b < NUM_BEATS; b++) begin
      while (!(dat_rx.flitv == Y)) @(posedge clk);
      o_txnid         = dat_rx.flit.txnid;
      o_opcode        = dat_rx.flit.opcode;
      o_data_beats[b] = dat_rx.flit.data;
      o_be            = dat_rx.flit.be;
      o_dbid          = dat_rx.flit.dbid;
      $display("[%0t] %s: got DAT txnid=%0d opcode=%s beat=%0d/%0d data=%0h dbid=%0d",
                $time, NAME, o_txnid, o_opcode.name(), b, NUM_BEATS-1, o_data_beats[b], o_dbid);
      @(posedge clk);
    end
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


module Tb_HNF;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;

  localparam int NUM_BEATS = 4; // CacheLineWidth(512)/DataWidth(128)

  logic clk = 0;
  logic resetn = 0;
  always #5 clk = ~clk;

  int errors = 0;

  // RN-F0 / RN-F1 <-> HN_F
  chi_req rnf_req_if    [2]();
  chi_snp rnf_snp_if    [2]();
  chi_rsp rnf_rsp_rx_if [2](); // HN_F RX from RN-F (SnpResp/CompAck)
  chi_rsp rnf_rsp_tx_if [2](); // HN_F TX to RN-F   (DBIDResp/Comp)
  chi_dat rnf_dat_rx_if [2](); // HN_F RX from RN-F (write-data/snoop-resp-data)
  chi_dat rnf_dat_tx_if [2](); // HN_F TX to RN-F   (CompData)

  // HN_F <-> SN-F
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

  // small helpers just to satisfy task output-arg signatures above where
  // the value isn't actually checked by the caller
  function automatic logic [38:0] addr_dummy(); return '0; endfunction
  function automatic width_e size_dummy(); return WIDTH_1; endfunction

  // Force the DUT back to a known idle state between scenarios.
  task automatic reset_dut();
    resetn = 0;
    repeat (3) @(posedge clk);
    resetn = 1;
    repeat (2) @(posedge clk);
  endtask

  // helper: fill all NUM_BEATS entries of a beat array with the same value
  function automatic void fill_beats(output logic [127:0] beats [NUM_BEATS], input logic [127:0] val);
    for (int b = 0; b < NUM_BEATS; b++) beats[b] = val;
  endfunction

  // ---------------------------------------------------------------------
  // Scenario A: WriteUniquePtl from RNF0, RNF1 has the line (invalidate),
  //             write pushed to memory. 4 distinct beats to prove beat
  //             ordering/dataid threading, not just a single repeated value.
  // ---------------------------------------------------------------------
  task automatic scenario_write_unique_ptl();
    logic [11:0]  txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0]  snp_txnid;
    logic         rts;
    logic [127:0] wdata [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario A: WriteUniquePtl (RNF0 -> snoop RNF1 -> SN), 4-beat data =====");
    txnid = 12'h0A1;
    wdata[0] = 128'hDEAD_BEEF_0000_0000_0000_0000_CAFE_F00D;
    wdata[1] = 128'h1111_1111_2222_2222_3333_3333_4444_4444;
    wdata[2] = 128'h5555_5555_6666_6666_7777_7777_8888_8888;
    wdata[3] = 128'h9999_9999_AAAA_AAAA_BBBB_BBBB_CCCC_CCCC;

    fork
      rnf0.send_req(txnid, REQ_WRITE_UNIQUE_PTL, 39'h1000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_UNIQUE || rts !== 1'b0) begin
          $display("FAIL A: expected SnpUnique/rettosrc=0, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf1.send_rsp(txnid, RSP_SNP_RESP);
      end
    join

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_DBID_RESP) begin $display("FAIL A: expected DBIDResp, got %s", rsp_op.name()); errors++; end

    rnf0.send_dat(txnid, DAT_DATA_FLIT, wdata, 16'hFFFF);

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_PTL) begin $display("FAIL A: expected WriteNoSnpPtl, got %s", req_op.name()); errors++; end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h5A5);

    snf.wait_dat(txnid, dat_op, got_data, be, dbid);
    if (dat_op !== DAT_NON_COPY_BACK_WR_DATA) begin $display("FAIL A: expected NonCopyBackWrData, got %s", dat_op.name()); errors++; end
    if (dbid !== 12'h5A5) begin $display("FAIL A: dbid not threaded through, got %0d", dbid); errors++; end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== wdata[b]) begin
        $display("FAIL A: beat %0d write data mismatch, expected %0h got %0h", b, wdata[b], got_data[b]);
        errors++;
      end
    end

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_COMP) begin $display("FAIL A: expected Comp, got %s", rsp_op.name()); errors++; end
    rnf0.send_rsp(txnid, RSP_COMP_ACK);

    repeat (3) @(posedge clk);
    $display("Scenario A done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario B: ReadShared from RNF0, RNF1 has the line (cache hit, CLEAN)
  //             and returns data directly on the snoop response - no SN access.
  //             (NOTE: uses REQ_READ_SHARED, not REQ_READ_UNIQUE - see
  //             the comment in Scenario G for why.)
  // ---------------------------------------------------------------------
  task automatic scenario_read_unique_hit();
    logic [11:0]  txnid;
    snp_opcode_e  snp_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0]  snp_txnid;
    logic         rts;
    logic [127:0] hitdata [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;

    $display("\n===== Scenario B: ReadShared, RNF1 cache HIT (clean) =====");
    txnid = 12'h0B2;
    fill_beats(hitdata, 128'h1111_2222_3333_4444_5555_6666_7777_8888);

    fork
      rnf0.send_req(txnid, REQ_READ_SHARED, 39'h2000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
          $display("FAIL B: expected SnpShared/rettosrc=1, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf1.send_dat(txnid, DAT_SNP_RESP_DATA, hitdata, 16'hFFFF);
      end
    join

    rnf0.wait_dat(txnid, dat_op, got_data, be);
    if (dat_op !== DAT_COMP_DATA) begin $display("FAIL B: expected CompData, got %s", dat_op.name()); errors++; end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== hitdata[b]) begin $display("FAIL B: beat %0d data mismatch", b); errors++; end
    end

    rnf0.send_rsp(txnid, RSP_COMP_ACK);
    repeat (3) @(posedge clk);
    $display("Scenario B done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario C: ReadShared from RNF1, RNF0 misses (doesn't have the line)
  //             -> HN_F falls back to reading from SN-F.
  // ---------------------------------------------------------------------
  task automatic scenario_read_unique_miss();
    logic [11:0]  txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    logic [11:0]  snp_txnid;
    logic         rts;
    logic [127:0] missdata [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;

    $display("\n===== Scenario C: ReadShared, RNF0 cache MISS -> read from SN =====");
    txnid = 12'h0C3;
    fill_beats(missdata, 128'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_0000_1111);

    fork
      rnf1.send_req(txnid, REQ_READ_SHARED, 39'h3000, WIDTH_16);
      begin
        rnf0.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
          $display("FAIL C: expected SnpShared/rettosrc=1, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf0.send_rsp(txnid, RSP_SNP_RESP);
      end
    join

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_READ_NO_SNOOP) begin $display("FAIL C: expected ReadNoSnoop, got %s", req_op.name()); errors++; end
    snf.send_dat(txnid, DAT_COMP_DATA, missdata);

    rnf1.wait_dat(txnid, dat_op, got_data, be);
    if (dat_op !== DAT_COMP_DATA) begin $display("FAIL C: expected CompData, got %s", dat_op.name()); errors++; end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== missdata[b]) begin $display("FAIL C: beat %0d data mismatch", b); errors++; end
    end

    rnf1.send_rsp(txnid, RSP_COMP_ACK);
    repeat (3) @(posedge clk);
    $display("Scenario C done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario D: WriteBackFull from RNF0 - no snoop at all, combined
  //             CompDBIDResp, and no final CompAck phase.
  // ---------------------------------------------------------------------
  task automatic scenario_writeback_full();
    logic [11:0]  txnid;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [127:0] wbdata [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario D: WriteBackFull (no snoop, no CompAck) =====");
    txnid = 12'h0D4;
    fill_beats(wbdata, 128'h0BAD_F00D_0BAD_F00D_0BAD_F00D_0BAD_F00D);

    rnf0.send_req(txnid, REQ_WRITE_BACK_FULL, 39'h4000, WIDTH_16);

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_COMP_DBID_RESP) begin $display("FAIL D: expected CompDBIDResp, got %s", rsp_op.name()); errors++; end

    rnf0.send_dat(txnid, DAT_DATA_FLIT, wbdata, 16'hFFFF);

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_FULL) begin $display("FAIL D: expected WriteNoSnpFull, got %s", req_op.name()); errors++; end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h123);

    snf.wait_dat(txnid, dat_op, got_data, be, dbid);
    if (dat_op !== DAT_COPY_BACK_WR_DATA) begin $display("FAIL D: expected CopyBackWrData, got %s", dat_op.name()); errors++; end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== wbdata[b]) begin $display("FAIL D: beat %0d data mismatch", b); errors++; end
    end

    repeat (5) @(posedge clk);
    $display("Scenario D done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario E: WriteUniquePtl from RNF0 hits a MODIFIED line in RNF1.
  //   Writeback-then-overwrite, no combine: SN-F should see TWO
  //   independent full-burst write transactions in order.
  // ---------------------------------------------------------------------
  task automatic scenario_write_hits_modified();
    logic [11:0]  txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    rsp_opcode_e  rsp_op;
    logic [11:0]  snp_txnid;
    logic         rts;
    logic [127:0] dirty_data [NUM_BEATS];
    logic [127:0] overwrite_data [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario E: WriteUniquePtl hits MODIFIED line in RNF1 (writeback-then-overwrite, no combine) =====");
    txnid = 12'h0E5;
    fill_beats(dirty_data, 128'hFEED_FACE_0000_0000_0000_0000_D00D_1234);
    fill_beats(overwrite_data, 128'hDEAD_DEAD_0000_0000_0000_0000_CAFE_CAFE);

    fork
      rnf0.send_req(txnid, REQ_WRITE_UNIQUE_PTL, 39'h5000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_UNIQUE) begin
          $display("FAIL E: expected SnpUnique on write-hits-modified, got %s", snp_op.name());
          errors++;
        end
        rnf1.send_dat(txnid, DAT_SNP_RESP_DATA, dirty_data, 16'hFFFF);
      end
    join

    // Leg 1: RNF1's dirty data pushed to SN-F first.
    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_FULL) begin
      $display("FAIL E: expected WriteNoSnpFull for RNF1's dirty writeback, got %s", req_op.name());
      errors++;
    end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h700);

    snf.wait_dat(txnid, dat_op, got_data, be, dbid);
    if (dat_op !== DAT_COPY_BACK_WR_DATA) begin
      $display("FAIL E: expected CopyBackWrData for RNF1's dirty writeback, got %s", dat_op.name());
      errors++;
    end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== dirty_data[b]) begin $display("FAIL E: writeback beat %0d mismatch", b); errors++; end
    end
    if (dbid !== 12'h700) begin $display("FAIL E: writeback dbid not threaded through, got %0d", dbid); errors++; end

    // Leg 2: only now does RNF0's own write proceed.
    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_DBID_RESP) begin
      $display("FAIL E: expected DBIDResp for RNF0's write after RNF1's writeback drained, got %s", rsp_op.name());
      errors++;
    end

    rnf0.send_dat(txnid, DAT_DATA_FLIT, overwrite_data, 16'hFFFF);

    snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
    if (req_op !== REQ_WRITE_NO_SNOOP_PTL) begin
      $display("FAIL E: expected WriteNoSnpPtl for RNF0's overwrite, got %s", req_op.name());
      errors++;
    end
    snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h701);

    snf.wait_dat(txnid, dat_op, got_data, be, dbid);
    if (dat_op !== DAT_NON_COPY_BACK_WR_DATA) begin
      $display("FAIL E: expected NonCopyBackWrData for RNF0's overwrite, got %s", dat_op.name());
      errors++;
    end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== overwrite_data[b]) begin $display("FAIL E: overwrite beat %0d mismatch", b); errors++; end
    end

    rnf0.wait_rsp(txnid, rsp_op);
    if (rsp_op !== RSP_COMP) begin $display("FAIL E: expected final Comp to RNF0, got %s", rsp_op.name()); errors++; end
    rnf0.send_rsp(txnid, RSP_COMP_ACK);

    repeat (3) @(posedge clk);
    $display("Scenario E done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario F: ReadUnique from RNF0 hits a MODIFIED line in RNF1. MESI,
  //   no Owned state: the dirty data forwarded to RNF0 must also make it
  //   back to SN-F as its own full write-back transaction.
  // ---------------------------------------------------------------------
  task automatic scenario_read_hit_modified();
    logic [11:0]  txnid;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    logic [11:0]  snp_txnid;
    logic         rts;
    logic [127:0] dirty_data [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;
    logic [11:0]  dbid;

    $display("\n===== Scenario F: ReadUnique hits MODIFIED line in RNF1 (memory writeback expected) =====");
    txnid = 12'h0F6;
    fill_beats(dirty_data, 128'hB16B_00B5_0000_0000_0000_0000_DEAD_10CC);

    fork
      rnf0.send_req(txnid, REQ_READ_UNIQUE, 39'h6000, WIDTH_16);
      begin
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        rnf1.send_dat(txnid, DAT_SNP_RESP_DATA, dirty_data, 16'hFFFF);
      end
    join

    rnf0.wait_dat(txnid, dat_op, got_data, be);
    if (dat_op !== DAT_COMP_DATA) begin $display("FAIL F: expected CompData, got %s", dat_op.name()); errors++; end
    for (int b = 0; b < NUM_BEATS; b++) begin
      if (got_data[b] !== dirty_data[b]) begin $display("FAIL F: CompData beat %0d mismatch", b); errors++; end
    end
    rnf0.send_rsp(txnid, RSP_COMP_ACK);

    fork
      begin
        snf.wait_req(txnid, req_op, addr_dummy(), size_dummy());
        if (req_op !== REQ_WRITE_NO_SNOOP_FULL) begin
          $display("FAIL F: expected WriteNoSnpFull for the memory writeback, got %s", req_op.name());
          errors++;
        end
        snf.send_rsp(txnid, RSP_COMP_DBID_RESP, 12'h7F0);

        snf.wait_dat(txnid, dat_op, got_data, be, dbid);
        if (dat_op !== DAT_COPY_BACK_WR_DATA) begin
          $display("FAIL F: expected CopyBackWrData for the memory writeback, got %s", dat_op.name());
          errors++;
        end
        for (int b = 0; b < NUM_BEATS; b++) begin
          if (got_data[b] !== dirty_data[b]) begin $display("FAIL F: writeback beat %0d mismatch", b); errors++; end
        end
      end
      begin
        repeat (80) @(posedge clk);
        $display("FAIL F: SN-F never received a writeback of the dirty data returned on the read-hit-modified snoop");
        errors++;
      end
    join_any
    disable fork;

    repeat (3) @(posedge clk);
    $display("Scenario F done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario G1: ReadShared -> expect SnpShared/rettosrc=1 (control case).
  // ---------------------------------------------------------------------
  task automatic scenario_read_shared_snoop_map();
    logic [11:0] txnid;
    snp_opcode_e snp_op;
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
        rnf1.send_rsp(txnid, RSP_SNP_RESP);
      end
    join

    $display("Scenario G1 done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario G2: ReadUnique -> expect SnpUnique/rettosrc=1 (NOT SnpShared).
  // ---------------------------------------------------------------------
  task automatic scenario_read_unique_snoop_map();
    logic [11:0] txnid;
    snp_opcode_e snp_op;
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

    $display("Scenario G2 done.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario H: RNF0 and RNF1 issue requests in the very same cycle.
  //   Known limitation preserved: req_rx_arbiter only forwards one
  //   request per cycle and rnf_model.send_req doesn't retry, so the
  //   loser simply has to be re-sent later - not queued automatically.
  // ---------------------------------------------------------------------
  task automatic scenario_simultaneous_requests();
    logic [11:0] txnid0, txnid1;
    snp_opcode_e snp_op;
    logic [11:0] snp_txnid;
    logic        rts;

    $display("\n===== Scenario H: simultaneous RNF0+RNF1 requests (arbitration / silent-drop check) =====");
    txnid0 = 12'h0AA0;
    txnid1 = 12'h0BB0;

    fork
      rnf0.send_req(txnid0, REQ_READ_ONCE, 39'h9000, WIDTH_16);
      rnf1.send_req(txnid1, REQ_READ_ONCE, 39'hA000, WIDTH_16);
    join

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
    $display("Scenario H done - manually confirm exactly one of txnid0/txnid1 was serviced.");
  endtask

  // ---------------------------------------------------------------------
  // Scenario I (NEW): two genuinely concurrent, independent transactions
  //   across two different trackers. This is the actual point of the
  //   whole multi-tracker/arbiter rebuild, so it gets its own dedicated
  //   test rather than relying on Scenario H's simultaneous-cycle case
  //   (which only stresses admission, not sustained concurrency).
  //
  //   Sequence:
  //     1. RNF0 fires a ReadShared miss (txnid_rnf0, addr 0xB000).
  //     2. RNF1 acks the resulting snoop as a miss.
  //     3. WHILE RNF0's tracker is still waiting on SN-F, RNF1
  //        independently fires its OWN unrelated ReadShared miss
  //        (txnid_rnf1, addr 0xC000). RNF0 must now simultaneously be
  //        "requester" of its own still-outstanding txn AND "snoop
  //        target" of RNF1's brand new txn - on two different
  //        trackers at once.
  //     4. SN-F services both requests, in whatever order the SN-F
  //        arbiter presents them (matched by txnid, not assumed order).
  //     5. Both RNF0 and RNF1 get their own correct (unswapped) data
  //        back and close out independently.
  // ---------------------------------------------------------------------
task automatic scenario_concurrent_independent_trackers();
    logic [11:0]  txnid_rnf0, txnid_rnf1;
    snp_opcode_e  snp_op;
    req_opcode_e  req_op;
    dat_opcode_e  dat_op;
    logic [11:0]  snp_txnid;
    logic         rts;
    logic [127:0] rnf0_expect [NUM_BEATS];
    logic [127:0] rnf1_expect [NUM_BEATS];
    logic [127:0] got_data [NUM_BEATS];
    logic [15:0]  be;
    logic [11:0]  this_txnid;
    logic [38:0]  this_addr;
    logic [11:0]  got_txnid;
    int           served;

    $display("\n===== Scenario I: two concurrent independent transactions (multi-tracker stress) =====");
    txnid_rnf0 = 12'h0900;
    txnid_rnf1 = 12'h0A00;
    fill_beats(rnf0_expect, 128'hB000_0000_0000_0000_0000_0000_0000_0001);
    fill_beats(rnf1_expect, 128'hC000_0000_0000_0000_0000_0000_0000_0002);

    fork
      begin
        rnf0.send_req(txnid_rnf0, REQ_READ_SHARED, 39'hB000, WIDTH_16);
        $display("rnf0 send at time %0t", $time);
        
        rnf1.wait_snp(snp_txnid, snp_op, rts);
        if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
          $display("FAIL I: expected SnpShared/rettosrc=1 for RNF0's txn, got %s/%0b", snp_op.name(), rts);
          errors++;
        end
        rnf1.send_rsp(snp_txnid, RSP_SNP_RESP); 

        fork
          rnf1.send_req(txnid_rnf1, REQ_READ_SHARED, 39'hC000, WIDTH_16);
          begin
            rnf0.wait_snp(snp_txnid, snp_op, rts);
            if (snp_op !== SNP_SHARED || rts !== 1'b1) begin
              $display("FAIL I: expected SnpShared/rettosrc=1 for RNF1's txn, got %s/%0b", snp_op.name(), rts);
              errors++;
            end
            rnf0.send_rsp(snp_txnid, RSP_SNP_RESP);
          end
        join

        fork
          begin
            rnf0.wait_dat(got_txnid, dat_op, got_data, be);
            if (dat_op !== DAT_COMP_DATA) begin $display("FAIL I: RNF0 expected CompData, got %s", dat_op.name()); errors++; end
            for (int b = 0; b < NUM_BEATS; b++) begin
              if (got_data[b] !== rnf0_expect[b]) begin $display("FAIL I: RNF0 beat %0d data mismatch", b); errors++; end
            end
            rnf0.send_rsp(got_txnid, RSP_COMP_ACK);
          end
          begin
            rnf1.wait_dat(got_txnid, dat_op, got_data, be);
            if (dat_op !== DAT_COMP_DATA) begin $display("FAIL I: RNF1 expected CompData, got %s", dat_op.name()); errors++; end
            for (int b = 0; b < NUM_BEATS; b++) begin
              if (got_data[b] !== rnf1_expect[b]) begin $display("FAIL I: RNF1 beat %0d data mismatch", b); errors++; end
            end
            // FIX: Echo the DBID provided in the CompData packet
            rnf1.send_rsp(got_txnid, RSP_COMP_ACK);
          end
        join
      end

      // =====================================================================
      // THREAD 2: SN-F Listener (Awake from cycle 0)
      // =====================================================================
      begin
        served = 0;
        while (served < 2) begin
          snf.wait_req(this_txnid, req_op, this_addr, size_dummy());
          if (req_op !== REQ_READ_NO_SNOOP) begin
            $display("FAIL I: expected ReadNoSnoop, got %s", req_op.name());
            errors++;
          end
          
          // WARNING: If your HN-F is correctly sending Tracker IDs (e.g., 0x00 and 0x01) 
          // to the SN-F instead of 0x0900/0x0A00, you need to check this_addr to determine 
          // which data to send back, rather than this_txnid.
          if (this_addr == 39'hB000) begin
            snf.send_dat(this_txnid, DAT_COMP_DATA, rnf0_expect);
          end else if (this_addr == 39'hC000) begin
            snf.send_dat(this_txnid, DAT_COMP_DATA, rnf1_expect);
          end else begin
            $display("FAIL I: SN-F saw an unexpected Address %h", this_addr);
            errors++;
          end
          
          served = served + 1;
        end
      end
    join

    repeat (5) @(posedge clk);
    $display("Scenario I done - both concurrent transactions completed independently and correctly.");
  endtask

  initial begin
    resetn = 0;
    repeat (5) @(posedge clk);
    resetn = 1;
    repeat (2) @(posedge clk);

    `RUN_SCN(scenario_write_unique_ptl(), 250)
    `RUN_SCN(scenario_read_unique_hit(), 250)
    `RUN_SCN(scenario_read_unique_miss(), 250)
    `RUN_SCN(scenario_writeback_full(), 250)
    `RUN_SCN(scenario_write_hits_modified(), 300)
    `RUN_SCN(scenario_read_hit_modified(), 350)
   `RUN_SCN(scenario_read_shared_snoop_map(), 250)
    `RUN_SCN(scenario_read_unique_snoop_map(), 250)
   `RUN_SCN(scenario_simultaneous_requests(), 250)
  

  
    $display("fail scenario started here",$time);
    `RUN_SCN(scenario_concurrent_independent_trackers(), 400)

    if (errors == 0) $display("\n*** ALL SCENARIOS PASSED ***");
    else              $display("\n*** %0d FAILURE(S) ***", errors);

    $finish;
  end

  // safety timeout - overall regression bound (each scenario also has
  // its own per-scenario watchdog above)
  initial begin
    #400000;
    $display("TIMEOUT - overall regression hung");
    $finish;
  end

endmodule