// Coverage monitor for every TSHR instance of the random testbench, attached
// with bind like tshr_deadlock_mon. It counts, in chi_tb_pkg::sb.cov:
//   "tshr: enter <state>"                     every entry into a non-IDLE state
//   "tshr: dirty write-back after <tx_type>"  which flow started a dirty write-back
// The top-level testbench turns missing entries into COVERAGE failures, so a
// passing run proves the stress really reached these states.
module tshr_cov
  import chi_tb_pkg::*;
#(
  parameter type state_t   = logic [4:0],
  parameter type tx_type_t = logic [2:0]
) (
  input logic     clk,
  input logic     resetn,
  input state_t   state_q,
  input tx_type_t tx_type_q
);

  state_t last_state;

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      last_state <= state_q;
    end else begin
      last_state <= state_q;
      if (sb != null && state_q != last_state && state_q.name() != "ST_IDLE") begin
        sb.cov_hit({"tshr: enter ", state_q.name()});
        if (state_q.name() == "ST_DIRTY_WB_REQ_SEND" && last_state.name() != "ST_SN_RETRY_WAIT")
          sb.cov_hit({"tshr: dirty write-back after ", tx_type_q.name()});
      end
    end
  end

endmodule

bind TSHR tshr_cov #(
  .state_t   (type(state_q)),
  .tx_type_t (type(tx_type_q))
) u_tshr_cov (
  .clk       (clk),
  .resetn    (resetn),
  .state_q   (state_q),
  .tx_type_q (tx_type_q)
);
