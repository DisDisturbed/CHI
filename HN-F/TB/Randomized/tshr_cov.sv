// Coverage monitor for every TSHR instance of the random testbench, attached
// with bind like tshr_deadlock_mon. It counts, in chi_tb_pkg::sb.cov:
//   "tshr: enter <state>"                     every entry into a non-IDLE state
//   "tshr: dirty write-back after <tx_type>"  which flow started a dirty write-back
//   "sf: ..."                                 snoop filter: how many RN-Fs one
//                                             transaction snooped, back-invalidations
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
  input tx_type_t tx_type_q,
  input logic     bi_q,          // back-invalidating a snoop filter victim
  input logic     snp_gnt        // a snoop left this tracker
);

  state_t last_state;
  int     n_snp;                 // snoops of the request itself (not the back-invalidation)
  bit     did_bi;

  // snoop phase timing (keys "perf: ..."): from the first cycle in a snoop
  // state to the first cycle after it, with the number of snoops it sent.
  // Summed into sb.cov so the final report shows cycles and phase counts.
  function automatic bit in_snoop(state_t s);
    return s.name() == "ST_SNOOP_PICK" || s.name() == "ST_SNOOP_SEND" ||
           s.name() == "ST_SNOOP_WAIT" || s.name() == "ST_SNOOP_PAR";
  endfunction
  function automatic void perf_add(string k, int n);
    sb.cov[k] = sb.cov_get(k) + n;
  endfunction
  bit ph_on;
  int ph_cyc, ph_snp;
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      ph_on <= 0; ph_cyc <= 0; ph_snp <= 0;
    end else if (sb != null) begin
      if (in_snoop(state_q)) begin
        ph_on  <= 1;
        ph_cyc <= ph_on ? ph_cyc + 1 : 1;
        ph_snp <= (ph_on ? ph_snp : 0) + int'(snp_gnt);
      end else if (ph_on) begin
        ph_on <= 0;
        if (ph_snp >= 3) begin
          perf_add("perf: snoop phases with 3+ snoops", 1);
          perf_add("perf: snoop phase cycles, 3+ snoops", ph_cyc);
        end else if (ph_snp == 2) begin
          perf_add("perf: snoop phases with 2 snoops", 1);
          perf_add("perf: snoop phase cycles, 2 snoops", ph_cyc);
        end else if (ph_snp == 1) begin
          perf_add("perf: snoop phases with 1 snoop", 1);
          perf_add("perf: snoop phase cycles, 1 snoop", ph_cyc);
        end
      end
    end
  end

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      last_state <= state_q;
      n_snp      <= 0;
      did_bi     <= 0;
    end else begin
      last_state <= state_q;
      if (snp_gnt && !bi_q) n_snp <= n_snp + 1;
      if (bi_q) did_bi <= 1;
      if (sb != null && state_q != last_state && state_q.name() != "ST_IDLE") begin
        sb.cov_hit({"tshr: enter ", state_q.name()});
        if (state_q.name() == "ST_DIRTY_WB_REQ_SEND" && last_state.name() != "ST_SN_RETRY_WAIT")
          sb.cov_hit(bi_q ? "sf: back-invalidation wrote dirty data back"
                          : {"tshr: dirty write-back after ", tx_type_q.name()});
        if ((state_q.name() == "ST_SNOOP_PICK" || state_q.name() == "ST_SNOOP_PAR") && last_state.name() == "ST_SF_LOOKUP" && bi_q)
          sb.cov_hit("sf: back-invalidation");
      end
      // end of a transaction: how many RN-Fs did it snoop
      if (sb != null && state_q.name() == "ST_DONE") begin
        if (tx_type_q.name() == "TX_READ" || tx_type_q.name() == "TX_WRITE_UNIQUE" || tx_type_q.name() == "TX_CMO") begin
          if (n_snp == 0) sb.cov_hit("sf: coherent request needed no snoop");
          if (n_snp == 2) sb.cov_hit("sf: one request snooped 2 RN-Fs");
          if (n_snp >= 3) sb.cov_hit("sf: one request snooped 3+ RN-Fs");
          if (did_bi) sb.cov_hit("sf: back-invalidation, then the request itself");
        end
      end
      if (state_q.name() == "ST_IDLE") begin
        n_snp  <= 0;
        did_bi <= 0;
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
  .tx_type_q (tx_type_q),
  .bi_q      (bi_q),
  .snp_gnt   (snp_gnt)
);
