`timescale 1ns/1ps
//=============================================================================
// Simple behavioral CHI bus-functional models for TSHR verification.
//
// rnf_model  - stands in for RN-F0 / RN-F1. Issues requests, answers
//              snoops, sends write data, waits for/acks completions.
// snf_model  - stands in for the SN-F (memory). Answers requests forwarded
//              by TSHR with CompDBIDResp/data (writes) or CompData (reads).
//
// Credit handling is deliberately simplified: every RX-side port in these
// models grants credit permanently (lcrdv tied high, unlimited receive
// capacity), matching how TSHR itself treats rsp_rx/dat_rx/sn_rsp_rx/
// sn_dat_rx. The one place a model must actually wait for credit is
// rnf_model.send_req(), because TSHR's req_rx[i].lcrdv is state-gated
// (only asserted while idle and while that port holds arbitration
// priority) rather than free-running.
//=============================================================================

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
  // Scenario B: ReadUnique from RNF0, RNF1 has the line (cache hit) and
  //             returns data directly on the snoop response - no SN access.
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

    $display("\n===== Scenario B: ReadUnique, RNF1 cache HIT =====");
    txnid = 12'h0B2;
    fork
      rnf0.send_req(txnid, REQ_READ_UNIQUE, 39'h2000, WIDTH_16);
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

    $display("\n===== Scenario C: ReadUnique, RNF0 cache MISS -> read from SN =====");
    txnid = 12'h0C3;
    fork
      rnf1.send_req(txnid, REQ_READ_UNIQUE, 39'h3000, WIDTH_16);
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

  // small helpers just to satisfy task output-arg signatures above where
  // the value isn't actually checked by the caller
  function automatic logic [38:0] addr_dummy(); return '0; endfunction
  function automatic width_e size_dummy(); return WIDTH_1; endfunction

  initial begin
    resetn = 0;
    repeat (5) @(posedge clk);
    resetn = 1;
    repeat (2) @(posedge clk);

    scenario_write_unique_ptl();
    scenario_read_unique_hit();
    scenario_read_unique_miss();
    scenario_writeback_full();

    if (errors == 0) $display("\n*** ALL SCENARIOS PASSED ***");
    else              $display("\n*** %0d FAILURE(S) ***", errors);

    $finish;
  end

  // safety timeout
  initial begin
    #100000;
    $display("TIMEOUT - a scenario hung");
    $finish;
  end

endmodule