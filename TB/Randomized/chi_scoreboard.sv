// ---------------------------------------------------------------------
// chi_scoreboard (patched: in-flight tracking for diagnosable timeouts)
//
// Same golden-memory + address-lock + req-gate mutex as before. Added:
// note_inflight()/clear_inflight() let an agent record "here's what I'm
// currently waiting on" right after issuing a request, so that if the
// watchdog in Tb_HNF_Random fires, report_timeout can print WHICH
// address/opcode/txnid actually got stuck instead of a bare "RNF0 timed
// out" with no clue what it was doing. Without this, every timeout looks
// identical and there's no way to tell if it's the same bug repeating or
// several different ones.
// ---------------------------------------------------------------------

class chi_scoreboard #(parameter int NUM_BEATS = 4);

  // ---- golden memory: last write wins, keyed by line-aligned address ----
  logic [127:0] golden_mem [logic [38:0]][NUM_BEATS];
  bit           golden_seen [logic [38:0]]; // has this address ever been written?

  // ---- address busy lock ----
  bit addr_busy [logic [38:0]];

  // ---- per-agent in-flight tracking (for diagnosable timeouts) ----
  logic [38:0]  inflight_addr   [string];
  chi_pkg::req_opcode_e  inflight_opcode [string];
  logic [11:0]  inflight_txnid  [string];
  bit           inflight_valid  [string];

  function automatic void note_inflight(string who, logic [38:0] addr, chi_pkg::req_opcode_e opcode, logic [11:0] txnid);
    inflight_addr[who]   = addr;
    inflight_opcode[who] = opcode;
    inflight_txnid[who]  = txnid;
    inflight_valid[who]  = 1;
  endfunction

  function automatic void clear_inflight(string who);
    inflight_valid[who] = 0;
  endfunction

  // ---- stats ----
  int unsigned num_requests_issued   = 0;
  int unsigned num_reads_checked     = 0;
  int unsigned num_read_mismatches   = 0;
  int unsigned num_writes_observed   = 0;
  int unsigned num_protocol_errors   = 0;
  int unsigned num_timeouts          = 0;

  // deterministic "unwritten" pattern so reads-before-any-write are still
  // checkable instead of just skipped - makes SN-F's own memory-init
  // pattern part of the contract the whole environment agrees on.
  function automatic logic [127:0] default_pattern(logic [38:0] addr, int beat);
    return {addr, 25'h0, beat[6:0]} ^ 128'hA5A5_A5A5_A5A5_A5A5_A5A5_A5A5_A5A5_A5A5;
  endfunction

  function automatic logic [127:0] read_golden(logic [38:0] addr, int beat);
    if (!golden_seen.exists(addr)) begin
      golden_mem[addr][0] = default_pattern(addr, 0);
      golden_mem[addr][1] = default_pattern(addr, 1);
      golden_mem[addr][2] = default_pattern(addr, 2);
      golden_mem[addr][3] = default_pattern(addr, 3);
      golden_seen[addr] = 1;
    end
    return golden_mem[addr][beat];
  endfunction

  function automatic void update_write(logic [38:0] addr, logic [127:0] beats [NUM_BEATS], string who);
    golden_mem[addr][0] = beats[0];
    golden_mem[addr][1] = beats[1];
    golden_mem[addr][2] = beats[2];
    golden_mem[addr][3] = beats[3];
    golden_seen[addr] = 1;
    num_writes_observed++;
    $display("[%0t] SCOREBOARD: %s wrote addr=%0h beat0=%0h", $time, who, addr, beats[0]);
  endfunction

  function automatic void check_read(logic [38:0] addr, logic [127:0] got_beats [NUM_BEATS], string who);
    bit mismatch;
    mismatch = 0;
    num_reads_checked++;
    for (int b = 0; b < NUM_BEATS; b++) begin
      logic [127:0] expected;
      expected = read_golden(addr, b);
      if (got_beats[b] !== expected) begin
        $display("[%0t] SCOREBOARD FAIL: %s read addr=%0h beat=%0d got=%0h expected=%0h",
                  $time, who, addr, b, got_beats[b], expected);
        mismatch = 1;
      end
    end
    if (mismatch) num_read_mismatches++;
  endfunction

  function automatic void report_protocol_error(string msg);
    $display("[%0t] SCOREBOARD PROTOCOL ERROR: %s", $time, msg);
    num_protocol_errors++;
  endfunction

  // now prints whatever note_inflight last recorded for `who`, if any -
  // this is the actual fix for "every timeout looks the same".
  function automatic void report_timeout(string who, string msg);
    if (inflight_valid.exists(who) && inflight_valid[who]) begin
      $display("[%0t] SCOREBOARD TIMEOUT: %s (addr=%0h opcode=%s txnid=%0d)",
                $time, msg, inflight_addr[who], inflight_opcode[who].name(), inflight_txnid[who]);
    end else begin
      $display("[%0t] SCOREBOARD TIMEOUT: %s (no in-flight info recorded)", $time, msg);
    end
    num_timeouts++;
  endfunction

  function automatic bit try_lock_addr(logic [38:0] addr);
    if (addr_busy.exists(addr) && addr_busy[addr]) return 0;
    addr_busy[addr] = 1;
    return 1;
  endfunction

  function automatic void unlock_addr(logic [38:0] addr);
    addr_busy[addr] = 0;
  endfunction

  function automatic void final_report();
    $display("\n================= SCOREBOARD SUMMARY =================");
    $display("  requests issued     : %0d", num_requests_issued);
    $display("  reads checked       : %0d", num_reads_checked);
    $display("  read mismatches     : %0d", num_read_mismatches);
    $display("  writes observed     : %0d", num_writes_observed);
    $display("  protocol errors     : %0d", num_protocol_errors);
    $display("  timeouts            : %0d", num_timeouts);
    $display("========================================================\n");
  endfunction

  bit req_gate_busy = 0;

  task automatic lock_req_gate();
    while (req_gate_busy) @(req_gate_free_event);
    req_gate_busy = 1;
  endtask

  task automatic unlock_req_gate();
    req_gate_busy = 0;
    -> req_gate_free_event;
  endtask

  event req_gate_free_event;

endclass