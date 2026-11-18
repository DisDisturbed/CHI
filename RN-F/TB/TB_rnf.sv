`timescale 1ns/1ps
// =============================================================================
// Tb_RNF - RTL RN-F dcache and one behavioural RN-F on the same HN-F
//
//   RN-F 0 : rnf_dcache (RTL), core side driven by rnf_core_driver
//   RN-F 1 : rnf_model  (behavioural), random CHI traffic on the same lines
//   HN-F   : RTL,  SN-F : snf_model
//
//  The two RN-Fs snoop each other through the HN-F.
//
//  Checking:
//   * chi_tb_pkg scoreboard: load data, SWMR on the mirrored line states (the
//     dcache's are mirrored from its tag writes), memory image at the end
//   * rnf_core_driver: byte-exact copy of the lines only the dcache touches
//   * rnf_chi_checker: protocol checker on the dcache's CHI ports
//   * coverage of what the dcache went through (cov_dcache, white-box) and of
//     the core side and the bus (driver / checker, black-box); with the full
//     scenario range every cell in check_coverage() must have been reached
//
//  The core driver is a pipelined out-of-order core (up to CORE_MAX_OUT
//  requests in flight, responses by idx in any order, held back only by byte
//  hazards). REQUIRE_CORE_OOO=1 also requires the dcache to have accepted
//  several requests at once and answered them out of order.
//
//  Sequence:
//   1. directed scenarios rnf_tb_pkg::RS_* (the core driver runs them and
//      orders the behavioural RN-F's ops)
//   2. NUM_ROUNDS random rounds, knobs re-rolled per round
//   3. optional MID_RESETS rounds that are reset in the middle of traffic
//   4. flush: the behavioural RN-F sends CleanInvalid for every private line,
//      writes its own lines back and sends CleanInvalid for every shared
//      line, which pulls the dcache's dirty lines out through snoops. After
//      that the memory image must hold every value and the dcache must be
//      empty and idle.
// =============================================================================
module Tb_RNF;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
  import chi_tb_pkg::*;
  import rnf_dcache_pkg::*;
  import rnf_tb_pkg::*;

  parameter int SEED                   = 1;
  parameter int LOG_MODE               = 3;
  parameter int VERBOSE                = 1;
  parameter int NUM_TRACKERS           = 4;
  parameter bit RUN_SCENARIOS          = 1;
  parameter int FIRST_SCENARIO         = 1;        // rnf_tb_pkg::RS_PINGPONG
  parameter int LAST_SCENARIO          = RS_LAST;
  parameter bit CHECK_COVERAGE         = 1;
  parameter int NUM_ROUNDS             = 8;
  parameter int OPS_PER_ROUND          = 50;       // behavioural RN-F ops; the core does CORE_OPS_MULT times as many accesses
  parameter int CORE_OPS_MULT          = 4;
  parameter int NC_PCT                 = 10;       // core accesses that are non-cacheable
  parameter int PRIV_PCT               = 25;       // core accesses to lines only the dcache touches
  parameter bit RANDOMIZE_KNOBS        = 1;
  parameter int MID_RESETS             = 0;
  parameter bit ISOLATE_FAILURES       = 1;
  parameter int SETS                   = 16;
  parameter int WAYS                   = 4;
  parameter int RX_FIFO_DEPTH          = 4;
  parameter int CORE_MAX_OUT           = 16;       // requests the core keeps in flight at most (<= 2**IW)
  parameter int NUM_MSHR               = 4;        // dcache MSHRs; must be <= NUM_TRACKERS (HN-F)
  parameter bit REQUIRE_CORE_OOO       = (NUM_MSHR > 1); // also require the dcache to have taken several requests at once and answered out of order
  parameter bit FAST_LD                = 1;        // dcache fast load path; the driver sends lookaheads before cacheable loads
  parameter int LA_PCT                 = 50;       // cacheable loads that get a lookahead first
  parameter bit ICACHE                 = 1;        // split I-cache (rnf_icache on the dcache's I-side port) + fetch driver
  parameter int CMD_TIMEOUT_CYCLES     = 400000;
  parameter int OVERALL_TIMEOUT_CYCLES = 16000000;

  localparam int       NRNF  = 2;
  localparam node_id_e HN_ID = 7'h40;
  localparam node_id_e SN_ID = 7'h00;
  localparam int       AW    = 39;
  localparam int       DW    = 64;
  localparam int       IW    = 4;

  logic clk    = 0;
  logic resetn = 0;
  always #5 clk = ~clk;

  // ---------------------------------------------------------------- interfaces
  chi_req rnf_req_if    [NRNF]();
  chi_snp rnf_snp_if    [NRNF]();
  chi_rsp rnf_rsp_rx_if [NRNF]();
  chi_rsp rnf_rsp_tx_if [NRNF]();
  chi_dat rnf_dat_rx_if [NRNF]();
  chi_dat rnf_dat_tx_if [NRNF]();

  chi_req snf_req_if();
  chi_rsp snf_rsp_if();
  chi_dat snf_dat_tx_if(), snf_dat_rx_if();

  rnf_core_if #(.AddrSize(AW), .MaxAccessWidth(DW), .ReqIdxWidth(IW)) core_if();
  rnf_core_if #(.AddrSize(AW), .MaxAccessWidth(128), .ReqIdxWidth(IW)) icore_if();   // I-cache port
  logic                ic_inv, ic_busy, fetch_en;
  logic                is_req_v, is_req_id, is_req_ack, is_dat_v, is_dat_id, is_dat_err;
  logic [AW-1:0]       is_req_addr;
  logic [1:0]          is_dat_beat;
  logic [127:0]        is_dat;

  // ---------------------------------------------------------------- HN-F
  HN_F #(
    .NUM_TRACKERS   (NUM_TRACKERS),
    .NUM_RNF        (NRNF),
    .AddrWidth      (39),
    .DataWidth      (128),
    .CacheLineWidth (512),
    .TxCreditMax    (15)
  ) hnf (
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

  // ---------------------------------------------------------------- RN-F 0: RTL dcache + core driver
  rnf_dcache #(
    .NODEID(7'h01), .HNID(HN_ID), .AddrSize(AW), .MaxAccessWidth(DW), .ReqIdxWidth(IW),
    .SETS(SETS), .WAYS(WAYS), .RX_FIFO_DEPTH(RX_FIFO_DEPTH), .NUM_MSHR(NUM_MSHR),
    .FAST_LD(FAST_LD), .ISIDE(ICACHE)
  ) dut (
    .clk(clk), .resetn(resetn),
    .core(core_if),
    .is_req_v_i(is_req_v), .is_req_id_i(is_req_id), .is_req_addr_i(is_req_addr), .is_req_ack_o(is_req_ack),
    .is_dat_v_o(is_dat_v), .is_dat_id_o(is_dat_id), .is_dat_beat_o(is_dat_beat), .is_dat_o(is_dat),
    .is_dat_err_o(is_dat_err),
    .req_tx(rnf_req_if[0]), .snp_rx(rnf_snp_if[0]),
    .rsp_rx(rnf_rsp_tx_if[0]), .rsp_tx(rnf_rsp_rx_if[0]),
    .dat_rx(rnf_dat_tx_if[0]), .dat_tx(rnf_dat_rx_if[0])
  );

  // ---------------------------------------------------------------- RN-F 0: I-cache + fetch driver (ICACHE)
  // (the same wiring as rnf_node: the dcache's I-side port carries its fills)
  rnf_icache #(.AddrSize(AW), .MaxAccessWidth(128), .ReqIdxWidth(IW), .SETS(32), .WAYS(2)) idut (
    .clk(clk), .resetn(resetn), .core(icore_if), .inv_i(ic_inv), .busy_o(ic_busy),
    .is_req_v_o(is_req_v), .is_req_id_o(is_req_id), .is_req_addr_o(is_req_addr), .is_req_ack_i(is_req_ack),
    .is_dat_v_i(is_dat_v), .is_dat_id_i(is_dat_id), .is_dat_beat_i(is_dat_beat), .is_dat_i(is_dat),
    .is_dat_err_i(is_dat_err)
  );

  rnf_fetch_driver #(.SEED(SEED * 7 + 5), .AddrSize(AW), .ReqIdxWidth(IW)) fdrv (
    .clk(clk), .resetn(resetn), .enable(fetch_en && ICACHE), .icore(icore_if), .inv_o(ic_inv), .busy_i(ic_busy)
  );
  initial fetch_en = 1'b1;

  rnf_core_driver #(
    .NAME("RNF0"), .IDX(0), .SEED(SEED * 7 + 1), .AddrSize(AW), .MaxAccessWidth(DW), .ReqIdxWidth(IW),
    .OPS_MULT(CORE_OPS_MULT), .NC_PCT(NC_PCT), .PRIV_PCT(PRIV_PCT), .MAX_OUT(CORE_MAX_OUT),
    .LA_PCT(FAST_LD ? LA_PCT : 0)
  ) drv (
    .clk(clk), .resetn(resetn), .core(core_if)
  );

  rnf_chi_checker #(.NODEID(7'h01), .HNID(HN_ID), .NAME("RNF0")) chk (
    .clk(clk), .resetn(resetn), .core(core_if),
    .req(rnf_req_if[0]), .snp(rnf_snp_if[0]),
    .rsp_rx(rnf_rsp_tx_if[0]), .rsp_tx(rnf_rsp_rx_if[0]),
    .dat_rx(rnf_dat_tx_if[0]), .dat_tx(rnf_dat_rx_if[0])
  );

  // ---------------------------------------------------------------- RN-F 1: behavioural, SN-F
  rnf_model #(.NODEID(7'h02), .HNID(HN_ID), .NAME("RNF1"), .IDX(1), .SEED(SEED * 7 + 2)) rnf1 (
    .clk(clk), .resetn(resetn),
    .req_tx(rnf_req_if[1]), .snp_rx(rnf_snp_if[1]),
    .rsp_rx(rnf_rsp_tx_if[1]), .rsp_tx(rnf_rsp_rx_if[1]),
    .dat_rx(rnf_dat_tx_if[1]), .dat_tx(rnf_dat_rx_if[1])
  );

  snf_model #(.NAME("SNF"), .IDX(0), .NODEID(SN_ID), .SEED(SEED * 7 + 3)) snf (
    .clk(clk), .resetn(resetn),
    .req_rx(snf_req_if),
    .rsp_tx(snf_rsp_if),
    .dat_rx(snf_dat_tx_if),
    .dat_tx(snf_dat_rx_if)
  );

  // ---------------------------------------------------------------- knobs
  function automatic void base_knobs();
    sb.rx_cr_cap = 15;  sb.rx_cr_pct = 100; sb.snp_lat_max = 6; sb.bubble_pct = 0;
    sb.op_timeout = 2000; sb.max_out = 4; sb.ack_lat_max = 3;
    sb.dat_interleave = 0; sb.dat_shuffle = 0; sb.compack_readonce = 0; sb.compack_write = 0; sb.wb_race = 1;
    sb.snf_lat_max = 4; sb.snf_slow_pct = 0; sb.snf_err_pct = 0; sb.snf_retry_pct = 0;
    sb.snf_comp_first_pct = 0; sb.snf_early_comp_pct = 0; sb.snf_pcrd_early_pct = 0; sb.snf_pcrd_types = 4;
    sb.snp_opt_data_pct = 0; sb.zero_be_pct = 0; sb.nc_multi = 0; sb.unaligned_pct = 0;
    sb.hot_n = 4; sb.hot_pct = 50; sb.scn_ops = 40; sb.txid_mode = 0;
    sb.partial_states = 1;
    sb.err_line.delete();
    sb.default_weights();
    drv.reset_knobs();
  endfunction

  // knobs for the directed scenarios
  function automatic void apply_profile(int id);
    base_knobs();
    case (id)
      RS_SNOOP_IN_WAIT: begin sb.snf_slow_pct = 40; sb.snf_lat_max = 12; end   // the dcache waits long for its own requests
      RS_WB_SNOOP_RACE, RS_EVICT_SNOOP_RACE, RS_UPGRADE_RACE: sb.snf_lat_max = 8;
      RS_OOO_BURST: begin sb.snf_slow_pct = 60; sb.snf_lat_max = 16; end       // misses and non-cacheable accesses take long
      RS_B2B_SET_CONFLICT, RS_B2B_SAME_LINE: begin sb.snf_lat_max = 10; sb.dat_shuffle = 1; end
      RS_SUBWORD_MERGE: begin sb.dat_interleave = 1; sb.snp_lat_max = 10; end
      RS_NC_SUBWORD: sb.snf_lat_max = 10;
      RS_NC_ERR: sb.err_line[line_of(sb.nc_pool[sb.nc_pool.size() - 1])] = 1;  // the SN-F answers this line with an error
      default: ;
    endcase
  endfunction

  function automatic void randomize_knobs();
    tb_rng r;
    base_knobs();
    if (!RANDOMIZE_KNOBS) return;
    r = sb.trng;
    sb.rx_cr_cap      = r.range(1, 15);
    sb.rx_cr_pct      = r.chance(70) ? 100 : r.range(20, 90);
    sb.snp_lat_max    = r.range(0, 10);
    sb.bubble_pct     = r.chance(60) ? 0 : r.range(5, 30);
    sb.max_out        = r.range(1, 4);
    sb.ack_lat_max    = r.chance(70) ? r.range(0, 5) : r.range(6, 30);
    sb.dat_interleave = r.chance(30);
    sb.dat_shuffle    = r.chance(30);
    sb.snf_lat_max    = r.range(0, 8);
    sb.snf_slow_pct   = r.chance(75) ? 0 : r.range(5, 25);
    sb.snf_retry_pct  = r.chance(65) ? 0 : r.range(5, 40);
    sb.snf_pcrd_types = r.range(1, 16);
    sb.hot_n          = r.range(1, 4);
    sb.hot_pct        = r.range(20, 90);
    sb.compack_readonce = r.chance(40);
    sb.compack_write    = r.chance(40);
    if (r.chance(40)) sb.snf_comp_first_pct = r.range(5, 40);
    if (r.chance(40)) sb.snf_early_comp_pct = r.range(5, 60);
    if (r.chance(50)) sb.snf_pcrd_early_pct = r.range(10, 70);
    if (r.chance(40)) sb.snp_opt_data_pct   = r.range(10, 60);
    // the behavioural RN-F's own request mix (legal CHI it may send next to the dcache)
    if (r.chance(30)) sb.txid_mode     = int'(r.range(1, 3));
    if (r.chance(30)) sb.zero_be_pct   = r.range(5, 30);
    sb.nc_multi = r.chance(40);
    if (r.chance(40)) sb.unaligned_pct = r.range(10, 60);
    sb.partial_states = r.chance(80);
    // the core: in-flight window, op mix, widths, gaps
    case (r.range(0, 9))
      0:       drv.k_out_lim = 1;                               // blocking core
      1, 2:    drv.k_out_lim = r.range(2, 4);
      default: drv.k_out_lim = r.range(4, CORE_MAX_OUT);
    endcase
    drv.k_store_pct = r.range(20, 80);
    drv.k_nc_pct    = r.chance(20) ? 0 : r.range(5, 30);
    drv.k_priv_pct  = r.range(10, 50);
    drv.k_gap_max   = r.range(0, 10);
    drv.k_b2b_pct   = r.range(30, 100);
    foreach (drv.k_w_wt[i]) drv.k_w_wt[i] = r.range(0, 10);
    if (r.chance(25)) drv.k_w_wt[0] = 40;                       // mostly single bytes
    if (r.chance(15)) drv.k_w_wt[$size(drv.k_w_wt) - 1] = 40;  // mostly full dwords
  endfunction

  function automatic string knob_str();
    return $sformatf("cr_cap=%0d cr_pct=%0d snp_lat=%0d bubble=%0d max_out=%0d ack_lat=%0d intlv=%0b shuffle=%0b snf_lat=%0d snf_slow=%0d hot=%0d/%0d%% retry=%0d%% txid=%0d zbe=%0d ncm=%0b unal=%0d ptl=%0b | core: out=%0d st=%0d%% nc=%0d%% priv=%0d%% gap=%0d b2b=%0d%% w=%0d/%0d/%0d/%0d",
                     sb.rx_cr_cap, sb.rx_cr_pct, sb.snp_lat_max, sb.bubble_pct, sb.max_out, sb.ack_lat_max, sb.dat_interleave,
                     sb.dat_shuffle, sb.snf_lat_max, sb.snf_slow_pct, sb.hot_n, sb.hot_pct, sb.snf_retry_pct,
                     sb.txid_mode, sb.zero_be_pct, sb.nc_multi, sb.unaligned_pct, sb.partial_states,
                     drv.k_out_lim, drv.k_store_pct, drv.k_nc_pct, drv.k_priv_pct, drv.k_gap_max, drv.k_b2b_pct,
                     drv.k_w_wt[0], drv.k_w_wt[1], drv.k_w_wt[2], drv.k_w_wt[3]);
  endfunction

  // ---------------------------------------------------------------- reset / commands
  task automatic apply_reset(input string why);
    int unsigned t;
    tb_info($sformatf("TOP: reset (%s)", why));
    resetn = 0;
    sb.epoch++;
    sb.on_reset();
    repeat (10) @(posedge clk);
    resetn = 1;
    repeat (5) @(posedge clk);
    t = 0;
    while (sb.agents_done < NRNF && t < 5000) begin @(posedge clk); t++; end
  endtask

  task automatic start_cmd(input int c, input int arg, input string what);
    tb_info($sformatf("TOP: start %s", what));
    sb.agents_done = 0;
    sb.rop_stop    = 0;
    sb.cmd         = c;
    sb.cmd_arg     = arg;
    sb.cmd_seq++;
  endtask

  task automatic issue_cmd(input int c, input int arg, input string what);
    int unsigned t, f0;
    f0 = sb.total_failures();
    start_cmd(c, arg, what);
    t = 0;
    while (sb.agents_done < NRNF && t < CMD_TIMEOUT_CYCLES) begin @(posedge clk); t++; end
    if (sb.agents_done < NRNF) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("TOP: %s did not finish within %0d cycles (%0d/%0d agents done) - resetting",
                                  what, CMD_TIMEOUT_CYCLES, sb.agents_done, NRNF));
      apply_reset("command timeout");
    end else begin
      tb_info($sformatf("TOP: done  %s (%0d cycles, failures so far %0d)", what, t, sb.total_failures()));
      if (ISOLATE_FAILURES && sb.total_failures() != f0 && c != CMD_FLUSH) apply_reset($sformatf("isolate the failure of %s", what));
    end
  endtask

  task automatic issue_cmd_reset_mid(input int arg, input string what);
    int unsigned t, d;
    start_cmd(CMD_RANDOM, arg, what);
    d = sb.trng.range(20, 600);
    t = 0;
    while (sb.agents_done < NRNF && t < d) begin @(posedge clk); t++; end
    apply_reset($sformatf("mid-traffic reset after %0d cycles", t));
    if (sb.agents_done < NRNF) begin
      sb.n_timeout++;
      sb.err("TIMEOUT", $sformatf("TOP: %s - agents did not unwind after the reset", what));
    end
  endtask

  // ---------------------------------------------------------------- dcache monitors
  function automatic cstate_e to_cstate(cache_state_e s);
    case (s)
      CS_SC:   return ST_SC;
      CS_UC:   return ST_UC;
      CS_UD:   return ST_UD;
      CS_SD:   return ST_SD;
      default: return ST_I;
    endcase
  endfunction

  // what the dcache was seen doing, kept across resets
  int unsigned c_hit = 0, c_miss = 0, c_evict = 0, c_wb = 0, c_snp_hit = 0, c_snp_data = 0;

  always @(posedge clk) begin
    if (resetn && sb != null) begin
      // mirror of the dcache line states, from its tag writes
      if (dut.tag_we) begin
        addr_t a;
        a = addr_t'({dut.tag_tag, dut.tag_set, 6'b0});
        sb.set_state(0, a, to_cstate(dut.tag_st));
        sb.check_swmr(a, "RNF0", {"dcache line -> ", dut.tag_st.name()});
      end
      // port A: a store hit in the pipeline turns UC into UD
      if (dut.pa_st_we) begin
        addr_t a;
        a = addr_t'({dut.pa_tag[dut.pa_st_way], dut.pa_st_set, 6'b0});
        sb.set_state(0, a, ST_UD);
        sb.check_swmr(a, "RNF0", "dcache line -> CS_UD (store hit, pipeline)");
      end
      c_hit      += dut.perf_hit;      c_miss     += dut.perf_miss;
      c_evict    += dut.perf_evict;    c_wb       += dut.perf_wb;
      c_snp_hit  += dut.perf_snp_hit;  c_snp_data += dut.perf_snp_data;
      if (dut.s_port && (dut.r_tag_re || dut.r_tag_we || dut.r_dat_re || dut.r_dat_we))
        sb.err("DUT_STATE", "dcache: request FSM and snoop FSM use the RAM port in the same cycle");
      if (rnf_req_if[0].flitv == Y) sb.n_req++;
      cov_dcache();
    end
  end

  // ---------------------------------------------------------------- dcache coverage
  // keys "dc ...": what the dcache went through, sampled inside it
  // every MSHR runs the same per-request FSM; the "request FSM" coverage
  // below is collected over all of them
  sstate_e cv_ss;
  bit      cv_upg [NUM_MSHR];   // the request in this MSHR is a store on a Shared line
  bit      cv_rsp_prev = 0;
  initial foreach (cv_upg[k]) cv_upg[k] = 0;

  function automatic string req_state_str(rstate_e rs, txn_e rt);
    case (rs)
      R_REQ, R_WAIT, R_WDAT, R_WCOMP: return $sformatf("%s %s", rs.name(), rt.name());
      default:                        return rs.name();
    endcase
  endfunction

  function automatic void cov_dcache();
    cache_state_e st;
    snp_opcode_e  so;
    rstate_e      rs;
    txn_e         rt;
    bit           store;
    string        opn;
    int           nopen, nmiss;
    addr_t        a;
    logic [5:0]   sset;
    cv_ss = sstate_e'(dut.s_q.state);

    // every snoop: type x state of the line, and what the MSHRs were doing
    if (cv_ss == S_CMP) begin
      st = dut.s_hit ? dut.s_hit_st : CS_I;
      so = dut.s_q.opcode;
      sb.cov_hit($sformatf("dc snoop: %s on %s", so.name(), st.name()));
      for (int k = 0; k < NUM_MSHR; k++) begin
        rs = rstate_e'(dut.m_q[k].state);
        rt = txn_e'(dut.m_q[k].txn);
        a  = addr_t'(dut.m_q[k].addr);
        if (rs == R_IDLE) continue;
        sb.cov_hit($sformatf("dc snoop while the request FSM is in %s", req_state_str(rs, rt)));
        if (dut.sn_set == dut.m_q[k].addr[6 +: $clog2(SETS)]) begin
          if (rt == T_READ && dut.sn_tag == dut.m_q[k].addr[AW-1:6+$clog2(SETS)])
            sb.cov_hit("dc snoop: on the line of the read in flight");
          if (rt == T_WB && dut.sn_tag == dut.m_q[k].vtag)    sb.cov_hit("dc snoop: on the line of the WriteBackFull in flight");
          if (rt == T_EVICT && dut.sn_tag == dut.m_q[k].vtag) sb.cov_hit("dc snoop: on the line of the Evict in flight");
        end
      end
    end
    if (dut.u_snp_rx.cnt_q >= 2) sb.cov_hit("dc snoop: two or more snoops queued");

    // the hit pipeline (P2): every hit it answers, op x the line's state
    if (dut.p2_hit) begin
      st = dut.p2_lk.hst;
      if (dut.p2_q.op == rnf_pkg::MEM_OP_STORE) opn = "store"; else opn = "load";
      sb.cov_hit($sformatf("dc core: %s hit on %s", opn, st.name()));
      sb.cov_hit("dc pipe: hit answered by the pipeline");
    end
    if (dut.p2_v_q && !dut.p2_hit && dut.p2_dep) sb.cov_hit("dc pipe: request waited for an MSHR on its set");
    if (dut.p2_v_q && dut.issue_w) sb.cov_hit("dc pipe: back-to-back issue");

    // the MSHR holding the RAM port: op x what its lookup found
    if (dut.own_v) begin
      int k;
      k     = int'(dut.own_i);
      rs    = rstate_e'(dut.m_q[k].state);
      store = (dut.m_q[k].op == rnf_pkg::MEM_OP_STORE);
      if (store) opn = "store"; else opn = "load";
      if (rs == R_LOOKUP && dut.m_q[k].first) begin
        if (dut.own_lk.hit) begin
          st = dut.own_lk.hst;
          sb.cov_hit($sformatf("dc core: %s hit on %s", opn, st.name()));
        end else if (dut.own_lk.inv) begin
          sb.cov_hit($sformatf("dc core: %s miss, free way", opn));
        end else begin
          st = dut.rd_st[dut.own_rr_way];
          sb.cov_hit($sformatf("dc core: miss replaces a %s line", st.name()));
        end
        cv_upg[k] = dut.own_lk.hit && store && (dut.own_lk.hst == CS_SC || dut.own_lk.hst == CS_SD);
      end
      if (rs == R_FILL_SEL) begin
        sb.cov_hit(dut.own_lk.hit ? "dc fill: line still here, own data kept" : "dc fill: data taken from CompData");
        if (cv_upg[k]) sb.cov_hit(dut.own_lk.hit ? "dc upgrade: Shared copy kept until CompData" : "dc upgrade: Shared copy lost to a snoop before CompData");
        cv_upg[k] = 0;
      end
      if (rs == R_WB_CMP) begin
        st = dut.rd_st[dut.m_q[k].way];
        if (st != CS_I && dut.rd_tag[dut.m_q[k].way] == dut.m_q[k].vtag) sb.cov_hit($sformatf("dc write-back: line sent as %s", st.name()));
        else                                                             sb.cov_hit("dc write-back: line lost to a snoop");
      end
    end

    // MSHR occupancy
    nopen = 0; nmiss = 0;
    for (int k = 0; k < NUM_MSHR; k++) begin
      rs = rstate_e'(dut.m_q[k].state);
      if (rs != R_IDLE) nopen++;
      if (rs inside {R_REQ, R_WAIT, R_RETRY, R_FILL_TAG, R_FILL_SEL, R_FILL, R_WDAT, R_WCOMP}) nmiss++;
      if (rs == R_DEP) sb.cov_hit("dc mshr: request waited for an older MSHR on its set");
    end
    if (nopen >= 2)        sb.cov_hit("dc mshr: 2+ MSHRs open");
    if (nopen == NUM_MSHR) sb.cov_hit("dc mshr: every MSHR open");
    if (nmiss >= 2)        sb.cov_hit("dc mshr: 2+ CHI transactions in flight");
    if (nmiss >= 1 && dut.perf_hit) sb.cov_hit("dc mshr: hit under an open miss");

    if (core_if.req.valid == Y && core_if.ready == Y && cv_rsp_prev) sb.cov_hit("dc core: request accepted the cycle after a response");

    // fast load path
    if (dut.fl_go) begin
      if (dut.fl_hit)                         sb.cov_hit("dc fast: lookahead hit");
      if (!dut.fl_way_hit)                    sb.cov_hit("dc fast: lookahead missed");
      if (dut.fl_way_hit && dut.fl_blk)       sb.cov_hit("dc fast: hit refused, the set is busy");
      if (dut.head_v)                         sb.cov_hit("dc fast: lookahead took port A from the FIFO");
      if (dut.fl_hit && nmiss >= 1)           sb.cov_hit("dc fast: hit under an open miss");
    end
    if ((core_if.la.valid == Y) && FAST_LD && !dut.fl_go) sb.cov_hit("dc fast: lookahead not tried (P2 store on port A)");

    // split I-cache
    if (ICACHE) begin
      if (idut.accept && idut.c_hit)                       sb.cov_hit("ic: hit (1 cycle)");
      if (idut.accept && !idut.c_now && !idut.c_in_fill)   sb.cov_hit("ic: miss, new fill");
      if (idut.accept && !idut.c_hit && idut.c_in_fill)    sb.cov_hit("ic: request on a line still being filled");
      if (idut.accept && !idut.c_hit && idut.c_in_fill && idut.c_now) sb.cov_hit("ic: chunk of a line being filled answered at once");
      if (idut.start && idut.dem_v_q)                      sb.cov_hit("ic: next-line prefetch");
      if (idut.inv_i)                                      sb.cov_hit("ic: FENCE.I");
      if (idut.inv_i && (idut.valid_q != '0))              sb.cov_hit("ic: FENCE.I with valid lines");
      if (int'(dut.i_q[0].st) == 6 || int'(dut.i_q[1].st) == 6) sb.cov_hit("ic: fill retried (RetryAck)");
      if (dut.i_pcmp_v && dut.i_lk.hit)                    sb.cov_hit("ic: fill served by the node's own D-cache");
      if (dut.i_pcmp_v && !dut.i_lk.hit)                   sb.cov_hit("ic: probe missed, ReadOnce sent");
      if (dut.i_pdel && dut.rxd_v && dut.rxd.txnid[11])    sb.cov_hit("ic: ReadOnce beat waited for a probe beat");
      if (idut.is_dat_v_i && idut.is_dat_err_i)            sb.cov_hit("ic: fill beat with an error");
      if (dut.i_req_win && (dut.want_req != '0))           sb.cov_hit("ic: fill request went before an MSHR's");
      if (idut.f_q[0].v && idut.f_q[1].v)                  sb.cov_hit("ic: two fills open");
    end
    cv_rsp_prev = (core_if.rsp.valid == Y);
  endfunction

  // after the flush the dcache must be idle and hold no line
  function automatic void check_dcache_empty();
    int n;
    n = 0;
    for (int s = 0; s < SETS; s++)
      for (int w = 0; w < WAYS; w++)
        if (dut.u_tag_ram.valid_q[s][w]) n++;          // valid = state other than I
    if (n != 0)
      sb.err("DUT_STATE", $sformatf("end of test: dcache still holds %0d valid line(s) after every line got CleanInvalid", n));
    for (int k = 0; k < NUM_MSHR; k++)
      if (int'(dut.m_q[k].state) != 0)                               // 0 = R_IDLE
        sb.err("DUT_STATE", $sformatf("end of test: dcache MSHR %0d is not idle (state %0d)", k, dut.m_q[k].state));
    if (int'(dut.s_q.state) != 0)                                    // 0 = S_IDLE
      sb.err("DUT_STATE", $sformatf("end of test: dcache snoop FSM is not idle (state %0d)", dut.s_q.state));
    if (dut.fq_cnt_q != 0 || dut.p2_v_q)
      sb.err("DUT_STATE", $sformatf("end of test: dcache hit pipeline is not empty (%0d in the FIFO, P2 %0b)", dut.fq_cnt_q, dut.p2_v_q));
  endfunction

  function automatic void need(string k);
    if (sb.cov_get(k) == 0)
      sb.err("COVERAGE", $sformatf("never reached: \"%s\" - the run did not test this, so a pass proves nothing about it", k));
  endfunction

  function automatic void need_n(string what, int unsigned n);
    if (n == 0)
      sb.err("COVERAGE", $sformatf("never reached: \"%s\" - the run did not test this, so a pass proves nothing about it", what));
  endfunction

  // Only what this environment can reach is required: the HN-F here grants
  // SC to ReadShared and UC to ReadUnique, so the dcache's lines are SC or UD
  // when a snoop finds them (UC only between fill and store, SD never).
  function automatic void check_coverage();
    string sn_ops [8] = '{"SNP_SHARED", "SNP_CLEAN", "SNP_ONCE", "SNP_NOT_SHARED_DIRTY", "SNP_UNIQUE",
                          "SNP_CLEAN_SHARED", "SNP_CLEAN_INVALID", "SNP_MAKE_INVALID"};
    need_n("dcache: load / store hit",          c_hit);
    need_n("dcache: miss",                      c_miss);
    need_n("dcache: Evict",                     c_evict);
    need_n("dcache: WriteBackFull",             c_wb);
    need_n("dcache: snoop hit",                 c_snp_hit);
    need_n("dcache: snoop answered with data",  c_snp_data);
    if (!CHECK_COVERAGE || !(RUN_SCENARIOS && FIRST_SCENARIO <= 1 && LAST_SCENARIO >= RS_LAST)) begin
      tb_info("TOP: coverage matrix not checked (needs RUN_SCENARIOS=1 and the full scenario range)");
      return;
    end
    foreach (sn_ops[i]) begin
      need($sformatf("dc snoop: %s on CS_SC", sn_ops[i]));
      need($sformatf("dc snoop: %s on CS_UD", sn_ops[i]));
    end
    need("dc snoop: SNP_SHARED on CS_I");
    need("dc snoop: SNP_UNIQUE on CS_I");
    need("dc core: load hit on CS_SC");
    need("dc core: load hit on CS_UD");
    need("dc core: store hit on CS_SC");
    need("dc core: store hit on CS_UD");
    need("dc core: load miss, free way");
    need("dc core: store miss, free way");
    need("dc core: miss replaces a CS_SC line");
    need("dc core: miss replaces a CS_UD line");
    need("dc core: request accepted the cycle after a response");
    need("dc fill: data taken from CompData");
    need("dc fill: line still here, own data kept");
    need("dc upgrade: Shared copy kept until CompData");
    need("dc upgrade: Shared copy lost to a snoop before CompData");
    need("dc write-back: line sent as CS_UD");
    need("dc write-back: line lost to a snoop");
    need("dc snoop while the request FSM is in R_WAIT T_READ");
    need("dc snoop while the request FSM is in R_WAIT T_WB");
    need("dc snoop: on the line of the read in flight");
    need("dc snoop: on the line of the WriteBackFull in flight");
    need("chk: snoop answered while a request is outstanding");
    need("chk: snoop answered for the line of the outstanding request");
    check_core_coverage();
  endfunction

  // black-box coverage of the core side (driver and checker, pins only)
  function automatic void check_core_coverage();
    string lg;
    string st_names [3] = '{"not held", "Shared Clean", "Unique"};
    // every width, cacheable and not, load and store
    foreach (st_names[i]) begin
      need($sformatf("bb core: load on a line %s",  st_names[i]));
      need($sformatf("bb core: store on a line %s", st_names[i]));
      need($sformatf("bb core: sub-word store (<8B) on a line %s", st_names[i]));
    end
    for (int b = 1; b <= DW / 8; b *= 2)
      foreach (sn_kind[i]) need($sformatf("core: %s %0dB %s", sn_kind[i], b, sn_op[i]));
    lg = drv.lane_gaps();
    if (lg != "") sb.err("COVERAGE", {"never reached: cacheable access lanes -", lg});
    // hit / miss / victims, seen on the bus
    need("bb core: load completed without a line read (hit)");
    need("bb core: store completed without a line read (hit)");
    need("bb core: load completed after a line read (miss)");
    need("bb core: store completed after a line read (miss)");
    need("bb core: miss open while a victim was written back (WriteBackFull)");
    need("bb core: miss open while a victim was evicted (Evict)");
    need("bb core: load whose line was snooped while it was open");
    need("bb core: store whose line was snooped while it was open");
    // the core: pipelining and hazards (what the driver did)
    need("drv: request presented with 1 other request(s) in flight");
    need("drv: load held back by a store to its bytes (RAW)");
    need("drv: store held back by a load of its bytes (WAR)");
    need("drv: store held back by a store to its bytes (WAW)");
    need("drv: sub-word stores to one line issued while another is in flight");
    need("drv: request waited 8+ cycles for ready");
    need("core: request accepted the cycle after a response");
    need("dc pipe: hit answered by the pipeline");
    need("core: rsp.err on nc load");
    // sub-word merging, checked byte-exact
    need("merge: load returned the bytes of 2+ sub-word stores (exact)");
    need("merge: load returned the bytes of sub-word stores that were in flight together (exact)");
    need("merge: load returned stored bytes next to bytes never stored (exact)");
    need("merge: sub-word stores onto a line that starts not held");
    need("merge: sub-word stores onto a line that starts Shared");
    need("merge: sub-word stores onto a line that starts Unique Dirty");
    need("merge: sub-word stores onto a line the other RN-F holds dirty");
    // what the dcache did with the requests in flight: only when asked for
    if (REQUIRE_CORE_OOO) begin
      need("drv: request presented with 2-3 other request(s) in flight");
      need("drv: request presented with 4+ other request(s) in flight");
      need("core: request accepted with 1 other request(s) outstanding");
      need("core: request accepted with 2-3 other request(s) outstanding");
      need("core: requests accepted in back-to-back cycles");
      need("core: two stores to one line outstanding at the dcache");
      need("core: response overtook an older request");
      need("core: response overtook an older request to another line");
      need("core: load response overtook an older store");
      need("bb core: two requests to one line open at once");
      need("dc mshr: 2+ MSHRs open");
      need("dc mshr: every MSHR open");
      need("dc mshr: 2+ CHI transactions in flight");
      need("dc mshr: hit under an open miss");
      need("dc mshr: request waited for an older MSHR on its set");
      need("dc pipe: back-to-back issue");
      need("dc pipe: request waited for an MSHR on its set");
    end
    if (ICACHE) begin
      need("ic: hit (1 cycle)");
      need("ic: miss, new fill");
      need("ic: request on a line still being filled");
      need("ic: next-line prefetch");
      need("ic: FENCE.I with valid lines");
      need("ic: fill request went before an MSHR's");
      need("ic: two fills open");
      need("ic: probe missed, ReadOnce sent");
    end
    if (FAST_LD) begin
      need("dc fast: lookahead hit");
      need("dc fast: lookahead missed");
      need("dc fast: hit refused, the set is busy");
      need("dc fast: lookahead took port A from the FIFO");
      need("dc fast: lookahead not tried (P2 store on port A)");
      need("core: fast load answered");
      need("core: fast load missed, sent on req");
    end
  endfunction

  string sn_kind [4] = '{"cached", "cached", "nc", "nc"};
  string sn_op   [4] = '{"load", "store", "load", "store"};

  function automatic void report_dcache();
    $display("  ---- dcache (RNF0) ----");
    $display("  core accesses       : %0d loads, %0d stores (%0d non-cacheable, %0d private), %0d picks of a busy line", drv.n_load, drv.n_store, drv.n_nc, drv.n_priv, drv.n_busy);
    $display("  private exact checks: %0d (%0d of them on bytes merged from 2+ stores)", drv.n_priv_chk, drv.n_merge_chk);
    $display("  in flight (core)    : up to %0d issued, up to %0d accepted by the dcache at once; %0d responses out of order", drv.max_drv_out, drv.max_dut_out, drv.n_ooo);
    $display("  core stalls         : %0d on a byte hazard, %0d on a full window; %0d err responses", drv.n_hz_stall, drv.n_full_stall, drv.n_err_rsp);
    $display("  hits / misses       : %0d / %0d (%0d of the hits on the fast load path)", dut.perf_hits_r, dut.perf_misses_r, dut.perf_fast_r);
    $display("  Evict / WriteBack   : %0d / %0d", dut.perf_evictions_r, dut.perf_writebacks_r);
    $display("  snoops              : %0d (%0d hit a line, %0d answered with data)", dut.perf_snoops_r, dut.perf_snoop_hits_r, dut.perf_snoop_data_r);
    $display("  RetryAck            : %0d", dut.perf_retries_r);
    if (ICACHE)
      $display("  I-cache fetches     : %0d (%0d answered in 1 cycle, %0d later: avg %0.1f, max %0d cycles), %0d FENCE.I, %0d errors, %0d mismatches",
               fdrv.n_fetch, fdrv.n_1cyc, fdrv.n_miss_lat, (fdrv.n_miss_lat != 0) ? real'(fdrv.lat_sum) / fdrv.n_miss_lat : 0.0,
               fdrv.lat_max, fdrv.n_inv, fdrv.n_err, fdrv.n_bad);
  endfunction

  function automatic void print_categories();
    int unsigned cc [string];
    cc = sb.cat_cnt;
    if (cc.size() == 0) return;
    $display("  ---- error categories ----");
    foreach (cc[c]) $display("  %s: %0d", c, cc[c]);
  endfunction

  // ---------------------------------------------------------------- main sequence
  initial begin
    chi_tb_pkg::chi_scoreboard s;

    if (NUM_TRACKERS < NUM_MSHR)
      $fatal(1, "Tb_RNF: NUM_TRACKERS (%0d) < NUM_MSHR (%0d): the HN-F must be able to track every request the dcache can have open",
             NUM_TRACKERS, NUM_MSHR);

    log_init(LOG_MODE, VERBOSE);
    s = new(NRNF, 1, 6, 0);
    s.trng = new(SEED ^ 32'h5EED_0001);
    // coherent pool: 16 lines spread over 3 sets of the dcache, so every set
    // has more lines than ways; pool[0..3] are the hot lines
    for (int i = 1; i <= 16; i++) s.pool.push_back((addr_t'(i) << 16) | (addr_t'(int'(i % 3)) << 6));
    for (int i = 0; i < 4; i++)   s.nc_pool.push_back(addr_t'(39'h20_0000) + (addr_t'(i) << 16));
    s.flush_ci_rn = 1;       // the behavioural RN-F cleans every line at the end
    sb = s;
    base_knobs();

    $display("=== Tb_RNF: SEED=%0d NUM_TRACKERS=%0d NUM_MSHR=%0d SETS=%0d WAYS=%0d scenarios=%0b(%0d..%0d) rounds=%0d x %0d ops (core x%0d) mid_resets=%0d ===",
             SEED, NUM_TRACKERS, NUM_MSHR, SETS, WAYS, RUN_SCENARIOS, FIRST_SCENARIO, LAST_SCENARIO, NUM_ROUNDS, OPS_PER_ROUND, CORE_OPS_MULT, MID_RESETS);

    resetn = 0;
    repeat (10) @(posedge clk);
    resetn = 1;
    repeat (5) @(posedge clk);

    // 1. directed scenarios
    if (RUN_SCENARIOS) begin
      for (int id = FIRST_SCENARIO; id <= LAST_SCENARIO; id++) begin
        apply_profile(id);
        issue_cmd(CMD_REMOTE, id, $sformatf("scenario %0d %s", id, rs_name(id)));
      end
      base_knobs();
    end

    // 2. random rounds
    for (int rd = 0; rd < NUM_ROUNDS; rd++) begin
      randomize_knobs();
      tb_info($sformatf("TOP: round %0d knobs: %s", rd, knob_str()));
      issue_cmd(CMD_RANDOM, OPS_PER_ROUND, $sformatf("random round %0d", rd));
    end

    if (MID_RESETS > 0) begin
      for (int rd = 0; rd < MID_RESETS; rd++) begin
        randomize_knobs();
        issue_cmd_reset_mid(OPS_PER_ROUND, $sformatf("reset round %0d", rd));
      end
      randomize_knobs();
      issue_cmd(CMD_RANDOM, OPS_PER_ROUND, "random round after the mid-traffic resets");
    end

    // 4. flush, final checks (fetching stops first: no fill open at the end)
    fetch_en = 1'b0;
    base_knobs();
    issue_cmd(CMD_REMOTE, RS_CLEAN_PRIV, "CleanInvalid on the private lines");
    issue_cmd(CMD_FLUSH, 0, "end-of-test flush");
    repeat (500) @(posedge clk);
    sb.final_memory_check();
    check_dcache_empty();
    if (chk.open_txn != 0 || chk.open_snp != 0)
      sb.err("DUT_STATE", $sformatf("end of test: %0d request(s) and %0d snoop(s) of the dcache are still open on the bus", chk.open_txn, chk.open_snp));
    if (chk.open_core != 0)
      sb.err("DUT_STATE", $sformatf("end of test: %0d core request(s) were accepted and never answered", chk.open_core));
    check_coverage();

    sb.final_report(SEED);
    report_dcache();
    print_categories();
    if (sb.total_failures() == 0)
      $display("*** RN-F REGRESSION: ALL CHECKS PASSED (seed=%0d) ***", SEED);
    else
      $display("*** RN-F REGRESSION: %0d FAILURES (seed=%0d) - rerun with this SEED to reproduce ***", sb.total_failures(), SEED);
    $finish;
  end

  initial begin
    repeat (OVERALL_TIMEOUT_CYCLES) @(posedge clk);
    tb_err("TIMEOUT", $sformatf("TOP: regression did not finish within %0d cycles", OVERALL_TIMEOUT_CYCLES));
    if (sb != null) begin
      sb.final_report(SEED);
      report_dcache();
      print_categories();
    end
    $display("*** RN-F REGRESSION: GLOBAL TIMEOUT (seed=%0d) ***", SEED);
    $finish;
  end

endmodule
