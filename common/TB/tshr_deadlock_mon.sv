// Deadlock monitor for every TSHR instance, attached with bind (see the
// bottom of this file), so neither the RTL nor the testbench tops change.
//
// A tracker counts as stuck when it stays in the same non-IDLE state with the
// same beat count for DEADLOCK_CYCLES cycles. Then it prints, once:
//   DEADLOCK: <instance> stuck in <state> for N cycles: <what that state waits for>
//             header (txnid/srcid/opcode/addr/tx_type) and which TX grants/requests are up
// When the tracker moves again it prints RELEASED (it was only slow), and a
// reset prints CLEARED BY RESET (the random TB resets after a timeout).
// At the end of the simulation every tracker that is still stuck is listed.
//
//   +deadlock_cycles=N    threshold in cycles (default 100)
//   +no_deadlock_mon      turn the monitor off
module tshr_deadlock_mon
  import chi_pkg::*;
#(
  parameter type state_t   = logic [4:0],
  parameter type tx_type_t = logic [1:0]
) (
  input logic        clk,
  input logic        resetn,
  input state_t      state_q,
  input tx_type_t    tx_type_q,
  input logic [7:0]  beat_cnt,
  input logic [11:0] txnid_q,
  input node_id_e    srcid_q,
  input req_opcode_e opcode_q,
  input logic [63:0] addr_q,
  input logic        expcompack_q,
  input logic        pcrd_req,
  input logic [3:0]  pcrd_type
);

  int unsigned limit;
  bit          enabled;
  int unsigned same_cycles;
  bit          reported;
  state_t      last_state;
  logic [7:0]  last_beat;
  time         stuck_since;

  initial begin
    enabled = !$test$plusargs("no_deadlock_mon");
    if (!$value$plusargs("deadlock_cycles=%d", limit)) limit = 100;
  end

  function automatic string credits();
    return $sformatf("expcompack=%0b pcrd_req=%0b pcrd_type=%0d", expcompack_q, pcrd_req, pcrd_type);
  endfunction

  function automatic string header();
    return $sformatf("txnid=%0d srcid=%0h opcode=%s addr=%0h tx_type=%s beat=%0d",
                     txnid_q, srcid_q, opcode_q.name(), addr_q, tx_type_q.name(), beat_cnt);
  endfunction

  // What the current state needs before it can move on.
  function automatic string waiting_for();
    string s = state_q.name();
    case (s)
      "ST_SNOOP_SEND":         return "waits for a grant on the SNP port (port credit or arbitration)";
      "ST_DBIDRESP_SEND",
      "ST_COMP_SEND":          return "waits for a grant on the requester's RSP port";
      "ST_RDATA_SEND":         return "waits for a grant on the requester's DAT port";
      "ST_SN_REQ_SEND",
      "ST_DIRTY_WB_REQ_SEND":  return "waits for a grant on the SN-F REQ port";
      "ST_SN_DATA_SEND",
      "ST_DIRTY_WB_DATA_SEND": return "waits for a grant on the SN-F DAT port";
      "ST_SN_COMP_WAIT":       return "waits for Comp from SN-F after the write data";
      "ST_HAZARD_WAIT":        return "waits for an older tracker on the same cache line to finish";
      "ST_SN_RETRY_WAIT":      return "waits for a PCrdGrant of this type from SN-F (P-credit pool)";
      "ST_SNOOP_WAIT":         return "waits for SnpResp / SnpRespData from the snoop target with this txnid";
      "ST_WDATA_WAIT":         return "waits for write data from the requester with this txnid+srcid";
      "ST_SN_RSP_WAIT":        return (tx_type_q.name() == "TX_READ")
                                      ? "waits for CompData from SN-F with this txnid"
                                      : "waits for CompDBIDResp from SN-F with this txnid";
      "ST_DIRTY_WB_RSP_WAIT":  return "waits for CompDBIDResp from SN-F with this txnid";
      "ST_COMPACK_WAIT":       return "waits for CompAck from the requester with this txnid+srcid";
      default:                 return "a state that should never wait";
    endcase
  endfunction

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      if (enabled && reported)
        $display("[%0t] DEADLOCK CLEARED BY RESET: %m was stuck in %s since %0t",
                 $time, last_state.name(), stuck_since);
      same_cycles <= 0;
      reported    <= 0;
      last_state  <= state_q;
      last_beat   <= beat_cnt;
    end else if (enabled) begin
      last_state <= state_q;
      last_beat  <= beat_cnt;
      if (state_q != last_state || beat_cnt != last_beat || state_q.name() == "ST_IDLE") begin
        if (reported)
          $display("[%0t] DEADLOCK RELEASED: %m left %s after %0d cycles (only slow, not deadlocked)",
                   $time, last_state.name(), same_cycles);
        same_cycles <= 0;
        reported    <= 0;
      end else begin
        same_cycles <= same_cycles + 1;
        if (same_cycles == 0) stuck_since <= $time;
        if (!reported && same_cycles + 1 >= limit) begin
          reported <= 1;
          $display("[%0t] DEADLOCK: %m stuck in %s for %0d cycles: %s",
                   $time, state_q.name(), same_cycles + 1, waiting_for());
          $display("           %s", header());
          $display("           %s", credits());
        end
      end
    end
  end

  final begin
    if (enabled && reported)
      $display("DEADLOCK AT END OF SIM: %m still stuck in %s since %0t: %s | %s | %s",
               state_q.name(), stuck_since, waiting_for(), header(), credits());
  end

endmodule

// One monitor per TSHR instance. Port expressions are resolved inside TSHR,
// so they can name its internal registers and credit counters.
bind TSHR tshr_deadlock_mon #(
  .state_t   (type(state_q)),
  .tx_type_t (type(tx_type_q))
) u_deadlock_mon (
  .clk        (clk),
  .resetn     (resetn),
  .state_q    (state_q),
  .tx_type_q  (tx_type_q),
  .beat_cnt   (8'(beat_cnt_q)),
  .txnid_q    (txnid_q),
  .srcid_q    (srcid_q),
  .opcode_q   (opcode_q),
  .addr_q     (64'(addr_q)),
  .expcompack_q (expcompack_q),
  .pcrd_req   (pcrd_req_o),
  .pcrd_type  (pcrd_type_o)
);
