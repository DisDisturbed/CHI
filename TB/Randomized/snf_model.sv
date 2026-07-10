

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


  bit tx_bus_busy = 0;
  event tx_bus_free_event;

  task automatic lock_tx_bus();
    while (tx_bus_busy) @(tx_bus_free_event);
    tx_bus_busy = 1;
  endtask

  task automatic unlock_tx_bus();
    tx_bus_busy = 0;
    -> tx_bus_free_event;
  endtask

  logic [11:0] dbid_ctr = '0;

  function automatic logic [11:0] next_dbid();
    logic [11:0] d;
    d = dbid_ctr;
    dbid_ctr = dbid_ctr + 12'h1;
    return d;
  endfunction

  // one persistent handler per outstanding write, filtering the shared
  // dat_rx stream by its own assigned dbid so concurrent write bursts
  // from different trackers can't cross-wire even if their beats
  // interleave on the wire.
  task automatic handle_write(chi_scoreboard#(NUM_BEATS) sb, logic [11:0] txnid, logic [38:0] addr, logic [11:0] my_dbid);
    logic [127:0] beats [NUM_BEATS];
    int b;
 
    lock_tx_bus();
    send_rsp(txnid, RSP_COMP_DBID_RESP, my_dbid);
    unlock_tx_bus();

    b = 0;
    while (b < NUM_BEATS) begin
      while (!(dat_rx.flitv == Y && dat_rx.flit.dbid == my_dbid)) @(posedge clk);
      beats[b] = dat_rx.flit.data;
      $display("[%0t] %s: got DAT(auto) dbid=%0d beat=%0d/%0d data=%0h",
                $time, NAME, my_dbid, b, NUM_BEATS-1, beats[b]);
      b++;
      @(posedge clk);
    end

    sb.update_write(addr, beats, {NAME, "(via memory writeback)"});
  endtask

  task automatic handle_read(chi_scoreboard#(NUM_BEATS) sb, logic [11:0] txnid, logic [38:0] addr);
    logic [127:0] beats [NUM_BEATS];
    int delay;

    for (int b = 0; b < NUM_BEATS; b++) beats[b] = sb.read_golden(addr, b);

    // small random service latency so concurrent requests genuinely
    // interleave/complete out of order rather than trivially in FIFO
    // order - stresses the crossbar's per-txnid matching harder.
    delay = $urandom_range(0, 4);
    repeat (delay) @(posedge clk);

    lock_tx_bus();
    send_dat(txnid, DAT_COMP_DATA, beats);
    unlock_tx_bus();
  endtask

  task automatic autonomous_responder(chi_scoreboard#(NUM_BEATS) sb);
    logic [11:0]  r_txnid;
    req_opcode_e  r_opcode;
    logic [38:0]  r_addr;
    logic [11:0]  assigned_dbid;

    forever begin
      while (!(req_rx.flitv == Y)) @(posedge clk);
      r_txnid  = req_rx.flit.txnid;
      r_opcode = req_rx.flit.opcode;
      r_addr   = req_rx.flit.addr;
      $display("[%0t] %s: got REQ(auto) txnid=%0d opcode=%s addr=%0h", $time, NAME, r_txnid, r_opcode.name(), r_addr);

      if (r_opcode == REQ_READ_NO_SNOOP) begin
        fork
          automatic logic [11:0] cap_txnid = r_txnid;
          automatic logic [38:0] cap_addr  = r_addr;
          handle_read(sb, cap_txnid, cap_addr);
        join_none
      end else if (r_opcode == REQ_WRITE_NO_SNOOP_FULL || r_opcode == REQ_WRITE_NO_SNOOP_PTL) begin
        assigned_dbid = next_dbid();
        fork
          automatic logic [11:0] cap_txnid = r_txnid;
          automatic logic [38:0] cap_addr  = r_addr;
          automatic logic [11:0] cap_dbid  = assigned_dbid;
          handle_write(sb, cap_txnid, cap_addr, cap_dbid);
        join_none
      end else begin
        sb.report_protocol_error($sformatf("%s: unexpected REQ opcode %s from HN_F", NAME, r_opcode.name()));
      end

      @(posedge clk);
    end
  endtask

endmodule