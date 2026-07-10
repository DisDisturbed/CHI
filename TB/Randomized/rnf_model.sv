

module rnf_model #(
  parameter chi_pkg::node_id_e NODEID    = 7'h00,
  parameter string              NAME     = "RNF",
  parameter int                 NUM_BEATS = 4
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

  assign snp_rx.lcrdv = 1'b1;
  assign rsp_rx.lcrdv = 1'b1;
  assign dat_rx.lcrdv = 1'b1;

  // =====================================================================
  // Original directed-scenario tasks - unchanged.
  // =====================================================================

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

  task automatic wait_snp(output logic [11:0] o_txnid, output snp_opcode_e o_opcode, output logic o_rettosrc);
    while (!(snp_rx.flitv == Y)) @(posedge clk);
    o_txnid    = snp_rx.flit.txnid;
    o_opcode   = snp_rx.flit.opcode;
    o_rettosrc = snp_rx.flit.rettosrc;
    $display("[%0t] %s: SNP  txnid=%0d opcode=%s rettosrc=%0b", $time, NAME, o_txnid, o_opcode.name(), o_rettosrc);
  endtask

  task automatic wait_rsp(output logic [11:0] o_txnid, output rsp_opcode_e o_opcode);
    while (!(rsp_rx.flitv == Y)) @(posedge clk);
    o_txnid  = rsp_rx.flit.txnid;
    o_opcode = rsp_rx.flit.opcode;
    $display("[%0t] %s: got RSP txnid=%0d opcode=%s", $time, NAME, o_txnid, o_opcode.name());
  endtask

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

  // =====================================================================
  // NEW: local coherence state + randomized issue path
  // =====================================================================

  typedef enum { L_INVALID, L_SHARED, L_UNIQUE } line_state_e;

  line_state_e local_state [logic [38:0]];
  logic [127:0] local_data [logic [38:0]][NUM_BEATS];

  function automatic line_state_e get_state(logic [38:0] addr);
    if (!local_state.exists(addr)) return L_INVALID;
    return local_state[addr];
  endfunction

  // running txnid counter for this agent - kept in a private range per
  // agent (see configure_txnid_range) so that, by default, two agents
  // never pick the same TxnID value. The top-level TB can override this
  // by giving both agents overlapping ranges when it specifically wants
  // to stress the known cross-tracker TxnID-collision gap.
  logic [11:0] txnid_base = '0;
  logic [11:0] txnid_span = 12'hFFF;
  logic [11:0] txnid_ctr  = '0;

  function automatic void configure_txnid_range(logic [11:0] base, logic [11:0] span);
    txnid_base = base;
    txnid_span = span;
    txnid_ctr  = '0;
  endfunction

  function automatic logic [11:0] next_txnid();
    logic [11:0] t;
    t = txnid_base + (txnid_ctr % (txnid_span + 12'h1));
    txnid_ctr = txnid_ctr + 12'h1;
    return t;
  endfunction

  // ---- always-on background snoop responder ----
  // Started once (forever loop) by the testbench after reset. Answers
  // every incoming snoop from this agent's own local_state, matching
  // what TSHR actually expects (see file header). Runs independently of
  // whatever issue_random_op() calls are in flight for THIS agent's own
  // outstanding requests - a real core answers snoops regardless of
  // what it's doing itself.
  task automatic snoop_responder();
    logic [11:0]  s_txnid;
    snp_opcode_e  s_opcode;
    logic         s_rettosrc;
    logic [38:0]  s_addr;
    line_state_e  st;
    logic [127:0] beats [NUM_BEATS];

    forever begin
      while (!(snp_rx.flitv == Y)) @(posedge clk);
      s_txnid    = snp_rx.flit.txnid;
      s_opcode   = snp_rx.flit.opcode;
      s_rettosrc = snp_rx.flit.rettosrc;
      s_addr     = snp_rx.flit.addr;
      $display("[%0t] %s: SNP(auto) txnid=%0d opcode=%s addr=%0h rettosrc=%0b",
                $time, NAME, s_txnid, s_opcode.name(), s_addr, s_rettosrc);

      st = get_state(s_addr);
      if (st == L_INVALID) begin
        send_rsp(s_txnid, RSP_SNP_RESP);
      end else begin
        for (int b = 0; b < NUM_BEATS; b++) beats[b] = local_data[s_addr][b];
        send_dat(s_txnid, DAT_SNP_RESP_DATA, beats, 16'hFFFF);
        // TSHR treats ANY DAT_SNP_RESP_DATA as dirty and will push a
        // writeback afterward regardless of Shared/Unique - see header.
      end

      // update local state for the line based on what kind of snoop this was
      if (s_opcode == SNP_UNIQUE) begin
        local_state[s_addr] = L_INVALID;
      end else if (s_opcode == SNP_SHARED) begin
        if (st != L_INVALID) local_state[s_addr] = L_SHARED; // downgrade, keep data
      end
    end
  endtask

  // ---- randomized single-operation issue ----
  // sb: scoreboard handle (shared across all agents)
  // addr_pool: queue of candidate line-aligned addresses to pick from
  // success (output): 1 if an operation was actually issued, 0 if it
  // skipped this call (e.g. every address in the pool was locked right
  // now) - the caller's loop should just try again next call in that
  // case. No `return` statements anywhere below - the "nothing free,
  // bail out" path is a plain if-wrapper instead of an early return.
  task automatic issue_random_op(chi_scoreboard#(NUM_BEATS) sb, logic [38:0] addr_pool[$],
                                  output bit success);
    logic [38:0]  addr;
    int           pick_order[$];
    bit           locked;
    line_state_e  st;
    int           opc_roll;
    req_opcode_e  opcode;
    logic [11:0]  txnid;
    logic [127:0] wdata [NUM_BEATS];
    logic [127:0] rdata [NUM_BEATS];
    dat_opcode_e  got_dat_op;
    rsp_opcode_e  got_rsp_op;
    logic [11:0]  got_txnid;
    logic [15:0]  be;

    success = 0;

    // shuffle a candidate order over the pool so we don't always probe
    // addresses in the same order when several are locked
    pick_order = {};
    for (int i = 0; i < addr_pool.size(); i++) pick_order.push_back(i);
    pick_order.shuffle();

    locked = 0;
    foreach (pick_order[i]) begin
      addr = addr_pool[pick_order[i]];
      if (sb.try_lock_addr(addr)) begin
        locked = 1;
        break; // just exits the foreach loop, not the task
      end
    end

    // everything is only attempted if we actually got a lock; if not,
    // we just fall through to the end of the task with success left at
    // 0 - no early return needed.
    if (locked) begin
      st = get_state(addr);

      // weight opcode choice by local state: prefer reads when we don't
      // already have exclusive/dirty access, occasionally write, and only
      // ever WriteBackFull (pure evict, no snoop) when we're already
      // L_UNIQUE (matches the project's own MESI comment: only a dirty
      // owner can silently write back).
      opc_roll = $urandom_range(0, 99);
      if (st == L_UNIQUE && opc_roll < 20) begin
        opcode = REQ_WRITE_BACK_FULL;
      end else if (opc_roll < 55) begin
        opcode = (opc_roll < 30) ? REQ_READ_SHARED : REQ_READ_UNIQUE;
      end else begin
        opcode = REQ_WRITE_UNIQUE_PTL;
      end

      sb.lock_req_gate();
      txnid = next_txnid();
      sb.num_requests_issued++;
      send_req(txnid, opcode, addr, WIDTH_16);
      sb.unlock_req_gate();
      sb.note_inflight(NAME, addr, opcode, txnid);

      unique case (opcode)
        REQ_READ_SHARED, REQ_READ_UNIQUE: begin
          wait_dat(got_txnid, got_dat_op, rdata, be);
          if (got_txnid !== txnid) sb.report_protocol_error($sformatf("%s: CompData txnid mismatch, expected %0d got %0d", NAME, txnid, got_txnid));
          if (got_dat_op !== DAT_COMP_DATA) sb.report_protocol_error($sformatf("%s: expected CompData, got %s", NAME, got_dat_op.name()));
          sb.check_read(addr, rdata, NAME);
          for (int b = 0; b < NUM_BEATS; b++) local_data[addr][b] = rdata[b];
          local_state[addr] = (opcode == REQ_READ_UNIQUE) ? L_UNIQUE : L_SHARED;
          send_rsp(txnid, RSP_COMP_ACK);
        end

        REQ_WRITE_UNIQUE_PTL: begin
          wait_rsp(got_txnid, got_rsp_op);
          if (got_txnid !== txnid) sb.report_protocol_error($sformatf("%s: DBIDResp txnid mismatch, expected %0d got %0d", NAME, txnid, got_txnid));
          if (got_rsp_op !== RSP_DBID_RESP) sb.report_protocol_error($sformatf("%s: expected DBIDResp, got %s", NAME, got_rsp_op.name()));
          for (int b = 0; b < NUM_BEATS; b++) wdata[b] = {$urandom(), $urandom(), $urandom(), $urandom()} ^ {addr, 25'h0, b[6:0]};
          send_dat(txnid, DAT_DATA_FLIT, wdata, 16'hFFFF);
          sb.update_write(addr, wdata, NAME);
          for (int b = 0; b < NUM_BEATS; b++) local_data[addr][b] = wdata[b];
          local_state[addr] = L_UNIQUE;
          wait_rsp(got_txnid, got_rsp_op);
          if (got_rsp_op !== RSP_COMP) sb.report_protocol_error($sformatf("%s: expected final Comp, got %s", NAME, got_rsp_op.name()));
          send_rsp(txnid, RSP_COMP_ACK);
        end

        REQ_WRITE_BACK_FULL: begin
          wait_rsp(got_txnid, got_rsp_op);
          if (got_rsp_op !== RSP_COMP_DBID_RESP) sb.report_protocol_error($sformatf("%s: expected CompDBIDResp, got %s", NAME, got_rsp_op.name()));
          for (int b = 0; b < NUM_BEATS; b++) wdata[b] = local_data[addr][b]; // evicting exactly what we hold
          send_dat(txnid, DAT_DATA_FLIT, wdata, 16'hFFFF);
          sb.update_write(addr, wdata, NAME);
          local_state[addr] = L_INVALID; // gave the line up
          // WriteBackFull has no Comp/CompAck phase
        end

        default: ;
      endcase

      sb.unlock_addr(addr);
      sb.clear_inflight(NAME);
      success = 1;
    end
  endtask

endmodule