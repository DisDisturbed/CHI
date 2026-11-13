`timescale 1ns/1ps
// =============================================================================
// rnf_model - spec-strict, multi-outstanding RN-F agent
//
//  Threads (all started from one initial block, UVM-agent style):
//    rx monitor   (always block)  link credits, response/data demux per TxnID, snoop capture
//    snoop engine (forever)       one forked responder per snoop, random latency
//    cmd loop     (forever)       executes top-level commands: directed scenario /
//                                 constrained-random ops / end-of-test flush
//    workers      (fork/join_none) up to sb.max_out concurrent operations
//
//  Local cache model: I / SC / UC / UD / SD with data. Snoop responses follow the
//  legal response tables (B4.5); silent clean evictions, silent stores
//  (UC->UD), write-back vs snoop crossing, RetryAck/PCrdGrant, credit starvation
//  and data-beat shuffling/interleaving are all modelled.
// =============================================================================
module rnf_model #(
  parameter chi_pkg::node_id_e NODEID = 7'h01,
  parameter chi_pkg::node_id_e HNID   = 7'h00,
  parameter string             NAME   = "RNF",
  parameter int                IDX    = 0,
  parameter int                SEED   = 1
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
  import tshr_flit_pkg::*;
  import chi_tb_pkg::*;

  // ---------------------------------------------------------------- state
  tb_rng rng, crng;
  cstate_e lstate   [addr_t];
  line_t   ldata    [addr_t];
  bit      evicting [addr_t];
  bit      wb_lost  [addr_t];
  rn_txn   tbl      [logic [11:0]];
  bit      leaked   [logic [11:0]];
  local_snp_flit_t snp_q [$];
  bit      snp_busy [logic [11:0]];
  logic [3:0] pcr_q [$];
  int      ack_wait [addr_t];          // lines with CompData received and CompAck not sent yet
  bit      dbid_inuse [logic [11:0]];  // DBIDs of my writes whose data is not sent yet

  int unsigned cyc = 0;
  int unsigned my_epoch = 0;
  int unsigned outstanding = 0, launched = 0;

  int req_cr_in = 0, req_cr_out = 0, rsp_cr_in = 0, rsp_cr_out = 0, dat_cr_in = 0, dat_cr_out = 0;
  int snp_gr = 0, snp_rc = 0, rsp_gr = 0, rsp_rc = 0, dat_gr = 0, dat_rc = 0;
  tx_lock req_lock, rsp_lock, dat_lock;

  initial begin
    rng = new(SEED); crng = new(SEED ^ 32'h00C0FFEE);
    req_lock = new(); rsp_lock = new(); dat_lock = new();
  end

  function automatic cstate_e get_st(addr_t a);
    if (!lstate.exists(a)) return ST_I;
    return lstate[a];
  endfunction

  function automatic void set_st(addr_t a, cstate_e s);
    lstate[a] = s;
    sb.set_state(IDX, a, s);
    sb.check_swmr(a, NAME, s.name());
  endfunction

  function automatic void reset_state();
    lstate.delete(); ldata.delete(); evicting.delete(); wb_lost.delete();
    tbl.delete(); leaked.delete(); snp_q.delete(); snp_busy.delete(); pcr_q.delete();
    ack_wait.delete(); dbid_inuse.delete();
    req_cr_out = 0; rsp_cr_out = 0; dat_cr_out = 0;
    outstanding = 0; launched = 0;
    req_lock = new(); rsp_lock = new(); dat_lock = new();
  endfunction

  // ---------------------------------------------------------------- RX handlers (no time)
  function automatic void rx_rsp_handle(local_rsp_flit_t f);
    if (f.opcode == RSP_PCR_GRANT_ACK) begin
      pcr_q.push_back(f.pcrdttype);
      sb.n_pcrd++;
    end else begin
      if (sb.chk_ids && (f.tgtid !== NODEID || f.srcid !== HNID))
        sb.err("PROTOCOL", $sformatf("%s: RSP %s txnid=%0d has TgtID=%0h SrcID=%0h (expected %0h/%0h)", NAME, f.opcode.name(), f.txnid, f.tgtid, f.srcid, NODEID, HNID));
      if (tbl.exists(f.txnid)) tbl[f.txnid].rsp_q.push_back(f);
      else begin
        sb.n_unexp++;
        sb.err("UNEXPECTED_RSP", $sformatf("%s: RSP %s txnid=%0d does not match any outstanding request", NAME, f.opcode.name(), f.txnid));
      end
    end
  endfunction

  function automatic void rx_dat_handle(local_dat_flit_t f);
    if (sb.chk_ids && (f.tgtid !== NODEID || f.srcid !== HNID))
      sb.err("PROTOCOL", $sformatf("%s: DAT %s txnid=%0d has TgtID=%0h SrcID=%0h (expected %0h/%0h)", NAME, f.opcode.name(), f.txnid, f.tgtid, f.srcid, NODEID, HNID));
    if (sb.chk_ids && f.opcode == DAT_COMP_DATA && f.homenid !== HNID)
      sb.err("PROTOCOL", $sformatf("%s: CompData txnid=%0d has HomeNID=%0h (expected %0h): CompAck would go to the wrong node", NAME, f.txnid, f.homenid, HNID));
    if (tbl.exists(f.txnid)) tbl[f.txnid].dat_q.push_back(f);
    else begin
      sb.n_unexp++;
      sb.err("UNEXPECTED_RSP", $sformatf("%s: DAT %s txnid=%0d dataid=%0d does not match any outstanding request", NAME, f.opcode.name(), f.txnid, f.dataid));
    end
  endfunction

  function automatic void rx_snp_handle(local_snp_flit_t f);
    addr_t la;
    la = line_of(f.addr);
    if (sb.chk_ids && f.srcid !== HNID)
      sb.err("PROTOCOL", $sformatf("%s: snoop %s txnid=%0d has SrcID=%0h (expected %0h): the response would go to the wrong node", NAME, f.opcode.name(), f.txnid, f.srcid, HNID));
    if (ack_wait.exists(la) && ack_wait[la] > 0)
      sb.err("SNOOP_ORDER", $sformatf("%s: snoop %s addr=%0h arrived after CompData and before my CompAck for that line (HN-F must hold snoops until CompAck)", NAME, f.opcode.name(), la));
    if (snp_busy.exists(f.txnid) && snp_busy[f.txnid])
      sb.err("TXNID_REUSE", $sformatf("%s: HN-F reused snoop TxnID %0d while the previous snoop is still unanswered", NAME, f.txnid));
    snp_busy[f.txnid] = 1;
    snp_q.push_back(f);
  endfunction

  // ---------------------------------------------------------------- link layer + monitor
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (sb != null && my_epoch != sb.epoch) begin
      reset_state();
      my_epoch = sb.epoch;
      req_tx.flitv <= N; req_tx.flitpend <= N; rsp_tx.flitv <= N; rsp_tx.flitpend <= N;
      dat_tx.flitv <= N; dat_tx.flitpend <= N;
    end
    if (!resetn || sb == null) begin
      req_cr_in <= 0; rsp_cr_in <= 0; dat_cr_in <= 0;
      snp_rx.lcrdv <= 1'b0; rsp_rx.lcrdv <= 1'b0; dat_rx.lcrdv <= 1'b0;
      snp_gr = 0; snp_rc = 0; rsp_gr = 0; rsp_rc = 0; dat_gr = 0; dat_rc = 0;
    end else begin
      // credits we receive from the HN-F for our TX channels
      if (req_tx.lcrdv === 1'b1) req_cr_in <= req_cr_in + 1;
      if (rsp_tx.lcrdv === 1'b1) rsp_cr_in <= rsp_cr_in + 1;
      if (dat_tx.lcrdv === 1'b1) dat_cr_in <= dat_cr_in + 1;

      // credits we hand out for our RX channels (random back-pressure) + flit capture
      if (snp_rx.lcrdv === 1'b1) snp_gr++;
      if (snp_rx.flitv == Y) begin
        if (snp_rc >= snp_gr) sb.err("LINK_CREDIT", $sformatf("%s: HN-F sent a SNP flit without a credit", NAME));
        snp_rc++;
        rx_snp_handle(local_snp_flit_t'(snp_rx.flit));
      end
      snp_rx.lcrdv <= ((snp_gr - snp_rc) < int'(sb.rx_cr_cap)) && crng.chance(sb.rx_cr_pct);

      if (rsp_rx.lcrdv === 1'b1) rsp_gr++;
      if (rsp_rx.flitv == Y) begin
        if (rsp_rc >= rsp_gr) sb.err("LINK_CREDIT", $sformatf("%s: HN-F sent a RSP flit without a credit", NAME));
        rsp_rc++;
        rx_rsp_handle(local_rsp_flit_t'(rsp_rx.flit));
      end
      rsp_rx.lcrdv <= ((rsp_gr - rsp_rc) < int'(sb.rx_cr_cap)) && crng.chance(sb.rx_cr_pct);

      if (dat_rx.lcrdv === 1'b1) dat_gr++;
      if (dat_rx.flitv == Y) begin
        if (dat_rc >= dat_gr) sb.err("LINK_CREDIT", $sformatf("%s: HN-F sent a DAT flit without a credit", NAME));
        dat_rc++;
        rx_dat_handle(local_dat_flit_t'(dat_rx.flit));
      end
      dat_rx.lcrdv <= ((dat_gr - dat_rc) < int'(sb.rx_cr_cap)) && crng.chance(sb.rx_cr_pct);
    end
  end

  // ---------------------------------------------------------------- TX pin drivers
  // The TX tasks below end up inside tasks marked no_inline_task, which keeps
  // elaboration in the Verilator flow from blowing up. That tool does not
  // schedule logic sensitive to signals written from such tasks, so the tasks
  // must not write the interfaces directly: they set req/rsp/dat_d_* and fire
  // the event, and these module-level blocks apply it with the same NBA timing.
  local_req_flit_t req_d_flit;
  local_rsp_flit_t rsp_d_flit;
  local_dat_flit_t dat_d_flit;
  yn_status_e req_d_v, req_d_pend, rsp_d_v, rsp_d_pend, dat_d_v, dat_d_pend;
  event req_drive, rsp_drive, dat_drive;

  always @(req_drive) begin req_tx.flit <= #1 req_d_flit; req_tx.flitv <= #1 req_d_v; req_tx.flitpend <= #1 req_d_pend; end
  always @(rsp_drive) begin rsp_tx.flit <= #1 rsp_d_flit; rsp_tx.flitv <= #1 rsp_d_v; rsp_tx.flitpend <= #1 rsp_d_pend; end
  always @(dat_drive) begin dat_tx.flit <= #1 dat_d_flit; dat_tx.flitv <= #1 dat_d_v; dat_tx.flitpend <= #1 dat_d_pend; end

  task automatic send_req_flit(input local_req_flit_t f, input int unsigned ep, output bit sent);
    int unsigned t0;
    tx_lock lk;
    sent = 0;
    if (sb.bubble_pct != 0 && rng.chance(sb.bubble_pct)) repeat (rng.range(1, 4)) @(posedge clk);
    lk = req_lock;
    lk.acquire();
    t0 = cyc;
    while ((req_cr_in - req_cr_out) <= 0 && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
    if (ep == sb.epoch && (req_cr_in - req_cr_out) > 0) begin
      req_cr_out++;
      req_d_flit = f;
      req_d_v    = Y;
      req_d_pend = Y;
      -> req_drive;
      sb.n_req++;
      sent = 1;
      @(posedge clk);
      req_d_v    = N;
      req_d_pend = N;
      -> req_drive;
    end else if (ep == sb.epoch) begin
      sb.err("LINK_CREDIT", $sformatf("%s: HN-F never granted a REQ credit (cannot send %s txnid=%0d)", NAME, f.opcode.name(), f.txnid));
    end
    lk.unlock();
  endtask

  task automatic send_rsp_flit(input local_rsp_flit_t f, input int unsigned ep);
    int unsigned t0;
    tx_lock lk;
    lk = rsp_lock;
    lk.acquire();
    t0 = cyc;
    while ((rsp_cr_in - rsp_cr_out) <= 0 && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
    if (ep == sb.epoch && (rsp_cr_in - rsp_cr_out) > 0) begin
      rsp_cr_out++;
      rsp_d_flit = f;
      rsp_d_v    = Y;
      rsp_d_pend = Y;
      -> rsp_drive;
      @(posedge clk);
      rsp_d_v    = N;
      rsp_d_pend = N;
      -> rsp_drive;
    end else if (ep == sb.epoch) begin
      sb.err("LINK_CREDIT", $sformatf("%s: HN-F never granted an RSP credit (cannot send %s txnid=%0d)", NAME, f.opcode.name(), f.txnid));
    end
    lk.unlock();
  endtask

  task automatic send_dat_beats(input logic [11:0] txnid, input node_id_e tgt, input dat_opcode_e op,
                                input logic [2:0] resp, input logic [11:0] dbid, input line_t data,
                                input bemask_t mask, input addr_t a, input width_e sz, input int unsigned ep);
    local_dat_flit_t f;
    int n, ord [4];
    int unsigned did, t0, j, tmp;
    tx_lock lk;
    n = num_beats_for(sz);
    for (int i = 0; i < 4; i++) ord[i] = i;
    if (sb.dat_shuffle) begin
      for (int i = n - 1; i > 0; i--) begin
        j = rng.range(0, i);
        tmp = ord[i]; ord[i] = ord[j]; ord[j] = tmp;
      end
    end
    lk = dat_lock;
    if (!sb.dat_interleave) lk.acquire();
    for (int i = 0; i < n; i++) begin
      did = first_dataid(a, sz) + ord[i];
      f = '0;
      f.tgtid = tgt; f.srcid = NODEID; f.txnid = txnid; f.homenid = HNID; f.opcode = op;
      f.resp = resp; f.dbid = dbid; f.dataid = did[1:0];
      f.data = data[did*128 +: 128]; f.be = mask[did*16 +: 16];
      if (sb.dat_interleave) lk.acquire();
      t0 = cyc;
      while ((dat_cr_in - dat_cr_out) <= 0 && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
      if (ep == sb.epoch && (dat_cr_in - dat_cr_out) > 0) begin
        dat_cr_out++;
        dat_d_flit = f;
        dat_d_v    = Y;
        dat_d_pend = (i == n - 1) ? N : Y;
        -> dat_drive;
        @(posedge clk);
        dat_d_v    = N;
        dat_d_pend = N;
        -> dat_drive;
      end else if (ep == sb.epoch) begin
        sb.err("LINK_CREDIT", $sformatf("%s: HN-F never granted a DAT credit (cannot send beat %0d txnid=%0d)", NAME, i, txnid));
      end
      if (sb.dat_interleave) lk.unlock();
      if (sb.bubble_pct != 0 && rng.chance(sb.bubble_pct)) repeat (rng.range(1, 3)) @(posedge clk);
    end
    if (!sb.dat_interleave) lk.unlock();
  endtask

  function automatic local_rsp_flit_t mk_rsp(node_id_e tgt, logic [11:0] txnid, rsp_opcode_e op, logic [2:0] resp);
    local_rsp_flit_t f;
    f = '0;
    f.tgtid = tgt; f.srcid = NODEID; f.txnid = txnid; f.opcode = op; f.resp = resp;
    return f;
  endfunction

  // ExpCompAck: same rule run_read / run_write use to decide whether a CompAck is sent
  function automatic bit exp_compack(req_opcode_e op);
    case (op)
      REQ_READ_SHARED, REQ_READ_CLEAN, REQ_READ_UNIQUE:      return 1'b1;
      REQ_READ_ONCE, REQ_READ_NO_SNOOP:                      return sb.compack_readonce;
      REQ_WRITE_UNIQUE_FULL, REQ_WRITE_UNIQUE_PTL,
      REQ_WRITE_NO_SNOOP_FULL, REQ_WRITE_NO_SNOOP_PTL:       return sb.compack_write;
      REQ_WRITE_BACK_FULL, REQ_WRITE_CLEAN_FULL:             return 1'b0;   // CopyBack writes never use CompAck
      default:                                               return 1'b0;
    endcase
  endfunction

  function automatic local_req_flit_t mk_req(rn_txn t);
    local_req_flit_t f;
    f = '0;
    f.tgtid = HNID; f.srcid = NODEID; f.txnid = t.txnid; f.opcode = t.opcode; f.size = t.size;
    f.addr = t.addr; f.allowretry = t.allowretry ? Y : N; f.pcrdttype = t.pcrdttype;
    f.memattr = t.memattr;
    f.expcompack = t.expack;
    return f;
  endfunction

  // ---------------------------------------------------------------- TxnID management
  function automatic void txid_range(output logic [11:0] base, output int unsigned span);
    case (sb.txid_mode)
      1:       begin base = 12'h000; span = 15;   end
      2:       begin base = 12'h000; span = 4095; end
      3:       begin base = 12'h000; span = 1;    end
      default: begin span = (4096 / sb.num_rnf) - 1; base = 12'(IDX * (span + 1)); end
    endcase
  endfunction

  function automatic bit new_txn(req_opcode_e op, addr_t a, width_e sz, bit coherent, output rn_txn t);
    logic [11:0] base, cand;
    int unsigned span;
    bit found;
    t = null;
    found = 0;
    txid_range(base, span);
    for (int k = 0; k < 24 && !found; k++) begin
      cand = base + 12'(rng.range(0, span));
      if (!tbl.exists(cand) && !leaked.exists(cand)) found = 1;
    end
    for (int k = 0; k <= int'(span) && !found; k++) begin
      cand = base + 12'(k);
      if (!tbl.exists(cand) && !leaked.exists(cand)) found = 1;
    end
    if (found) begin
      t = new();
      t.txnid = cand; t.opcode = op; t.addr = a; t.size = sz; t.t0 = cyc; t.retries = 0;
      t.allowretry = 1; t.pcrdttype = 4'h0; t.memattr = sb.use_memattr ? (coherent ? MEMATTR_NORMAL_WB : MEMATTR_NORMAL_NC) : 4'b0000;
      t.expack = exp_compack(op);
    end
    return found;
  endfunction

  function automatic void txn_open(rn_txn t);
    tbl[t.txnid] = t;
    t.t0 = cyc;
    sb.note_inflight(NAME, t.txnid, t.addr, t.opcode);
  endfunction

  function automatic void txn_close(rn_txn t, int unsigned ep, bit timed_out);
    if (ep == sb.epoch) begin
      if (timed_out) leaked[t.txnid] = 1;   // late responses must not look "unexpected"; ID is never reused
      else begin
        if (t.rsp_q.size() != 0 || t.dat_q.size() != 0)
          sb.err("PROTOCOL", $sformatf("%s: txnid=%0d completed but %0d RSP / %0d DAT flits were left unconsumed", NAME, t.txnid, t.rsp_q.size(), t.dat_q.size()));
        tbl.delete(t.txnid);
        sb.clear_inflight(NAME, t.txnid);
      end
    end
  endfunction

  // ---------------------------------------------------------------- generic waits
  task automatic wait_any(input rn_txn t, input int unsigned ep, output bit alive);
    while (t.rsp_q.size() == 0 && t.dat_q.size() == 0 && ep == sb.epoch && (cyc - t.t0) < sb.op_timeout) @(posedge clk);
    alive = (t.rsp_q.size() != 0 || t.dat_q.size() != 0) && ep == sb.epoch;
  endtask

  function automatic int find_pcr(logic [3:0] ty);
    foreach (pcr_q[i]) if (pcr_q[i] == ty) return i;
    return -1;
  endfunction

  // RetryAck -> wait for matching PCrdGrant -> resend with AllowRetry=0
  task automatic do_retry(input rn_txn t, input int unsigned ep, input local_rsp_flit_t ra, output bit ok);
    int idx;
    bit sent;
    ok = 0;
    sb.n_retry++;
    if (!t.allowretry) begin
      sb.err("PROTOCOL", $sformatf("%s: RetryAck for txnid=%0d although the request had AllowRetry=0", NAME, t.txnid));
    end else begin
      t.pcrdttype = ra.pcrdttype;
      t.t0 = cyc;
      idx = find_pcr(ra.pcrdttype);
      while (idx < 0 && ep == sb.epoch && (cyc - t.t0) < sb.op_timeout) begin
        @(posedge clk);
        idx = find_pcr(ra.pcrdttype);
      end
      if (idx >= 0) begin
        pcr_q.delete(idx);
        t.allowretry = 0; t.retries++; t.t0 = cyc;
        send_req_flit(mk_req(t), ep, sent);
        ok = sent;
      end
    end
  endtask

  // next non-retry response of a transaction (retries handled transparently)
  task automatic next_rsp(input rn_txn t, input int unsigned ep, output local_rsp_flit_t f, output bit ok);
    bit done, rok;
    ok = 0; done = 0; f = '0;
    while (!done) begin
      while (t.rsp_q.size() == 0 && ep == sb.epoch && (cyc - t.t0) < sb.op_timeout) @(posedge clk);
      if (t.rsp_q.size() == 0) done = 1;
      else begin
        f = t.rsp_q.pop_front();
        if (f.opcode == RSP_RETRY_ACK) begin
          do_retry(t, ep, f, rok);
          if (!rok) done = 1;
        end else begin
          ok = 1; done = 1;
        end
      end
    end
  endtask

  function automatic void fail_txn(rn_txn t, int unsigned ep, string msg);
    if (ep == sb.epoch) begin
      sb.report_timeout(NAME, t.txnid, msg);
      txn_close(t, ep, 1);
    end
  endfunction

  // ---------------------------------------------------------------- read family
  function automatic bit read_resp_ok(req_opcode_e op, logic [2:0] r, output cstate_e st);
    st = ST_I;
    case (op)
      REQ_READ_SHARED: begin
        if (r == R_SC)    begin st = ST_SC; return 1; end
        if (r == R_UC)    begin st = ST_UC; return 1; end
        if (r == R_UD_PD) begin st = ST_UD; return 1; end
        if (r == R_SD_PD) begin st = ST_SD; return 1; end
        return 0;
      end
      REQ_READ_CLEAN: begin
        if (r == R_SC) begin st = ST_SC; return 1; end
        if (r == R_UC) begin st = ST_UC; return 1; end
        return 0;
      end
      REQ_READ_UNIQUE: begin
        if (r == R_UC)    begin st = ST_UC; return 1; end
        if (r == R_UD_PD) begin st = ST_UD; return 1; end
        return 0;
      end
      default: return (r == R_I || r == R_UC);   // ReadOnce / ReadNoSnp: line is not kept coherent
    endcase
  endfunction

  task automatic run_read(input rn_txn t, input int unsigned ep, output bit ok); /*verilator no_inline_task*/
    rd_rec rr;
    local_dat_flit_t df;
    local_rsp_flit_t rf;
    line_t got;
    int nb, first, cnt;
    bit [3:0] expect_mask, seen;
    bit sent, abort, alive, retry_ok, rerr_seen, resp_set, coherent, legal, ack;
    logic [2:0] resp;
    logic [11:0] cdbid;
    cstate_e nst;
    addr_t la;
    ok = 0; abort = 0; cnt = 0; seen = '0; rerr_seen = 0; resp_set = 0; resp = R_I; got = '0;
    la = line_of(t.addr);
    nb = num_beats_for(t.size); first = first_dataid(t.addr, t.size);
    expect_mask = '0;
    for (int i = 0; i < nb; i++) expect_mask[first + i] = 1'b1;
    coherent = (t.opcode != REQ_READ_NO_SNOOP);
    rr = sb.rd_begin(t.addr, NAME);
    txn_open(t);
    send_req_flit(mk_req(t), ep, sent);
    abort = !sent;
    while (cnt < nb && !abort) begin
      wait_any(t, ep, alive);
      if (!alive) abort = 1;
      else begin
        if (t.rsp_q.size() > 0) begin
          rf = t.rsp_q.pop_front();
          if (rf.opcode == RSP_RETRY_ACK) begin
            do_retry(t, ep, rf, retry_ok);
            if (!retry_ok) abort = 1;
          end else begin
            sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d received unexpected RSP %s while waiting for CompData", NAME, t.opcode.name(), t.txnid, rf.opcode.name()));
          end
        end
        if (t.dat_q.size() > 0 && !abort) begin
          df = t.dat_q.pop_front();
          if (df.opcode != DAT_COMP_DATA)
            sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d expected CompData, got %s", NAME, t.opcode.name(), t.txnid, df.opcode.name()));
          else if (!expect_mask[df.dataid])
            sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d CompData dataid=%0d outside expected window (size=%0dB addr=%0h)", NAME, t.opcode.name(), t.txnid, df.dataid, size_bytes(t.size), t.addr));
          else if (seen[df.dataid])
            sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d duplicate CompData beat dataid=%0d", NAME, t.opcode.name(), t.txnid, df.dataid));
          else begin
            seen[df.dataid] = 1;
            got[df.dataid*128 +: 128] = df.data;
            if (!resp_set) begin resp = df.resp; cdbid = df.dbid; resp_set = 1; end
            else begin
              if (df.resp !== resp)
                sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d CompData beats carry different Resp (%0b vs %0b)", NAME, t.opcode.name(), t.txnid, resp, df.resp));
              if (df.dbid !== cdbid)
                sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d CompData beats carry different DBID (%0d vs %0d)", NAME, t.opcode.name(), t.txnid, cdbid, df.dbid));
            end
            if (df.resperr != ERR_OK) rerr_seen = 1;
            cnt++;
          end
        end
      end
    end
    if (abort) begin
      sb.rd_end(rr, got, 1, "aborted");
      fail_txn(t, ep, $sformatf("%s did not complete (%0d/%0d beats received)", t.opcode.name(), cnt, nb));
    end else begin
      if (rerr_seen) begin
        sb.n_err_seen++;
        if (!sb.is_err_line(t.addr)) sb.err("SPURIOUS_RESPERR", $sformatf("%s: %s txnid=%0d addr=%0h returned RespErr but no error was injected", NAME, t.opcode.name(), t.txnid, t.addr));
      end
      legal = read_resp_ok(t.opcode, resp, nst);
      if (!legal) sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d illegal CompData Resp=%0b for this opcode", NAME, t.opcode.name(), t.txnid, resp));
      sb.rd_end(rr, got, rerr_seen, $sformatf("%s txnid=%0d", t.opcode.name(), t.txnid), expect_mask);
      if (coherent && legal && !rerr_seen) begin
        ldata[la] = got;
        set_st(la, nst);
      end
      ack = t.expack;
      if (ack && ep == sb.epoch) begin
        ack_wait[la] = ack_wait.exists(la) ? ack_wait[la] + 1 : 1;
        if (sb.ack_lat_max > 8) sb.cov_hit("rn: CompAck delayed > 8 cycles possible");
        repeat (rng.range(0, sb.ack_lat_max)) @(posedge clk);
        if (t.opcode == REQ_READ_ONCE || t.opcode == REQ_READ_NO_SNOOP) sb.cov_hit("rn: CompAck on ReadOnce/ReadNoSnp");
        // CompAck TxnID = the DBID of CompData (IHI0050 B2.6)
        send_rsp_flit(mk_rsp(HNID, cdbid, RSP_COMP_ACK, resp), ep);
        if (ep == sb.epoch && ack_wait.exists(la)) begin
          ack_wait[la]--;
          if (ack_wait[la] <= 0) ack_wait.delete(la);
        end
      end
      txn_close(t, ep, 0);
      ok = 1;
    end
  endtask

  // ---------------------------------------------------------------- write family
  function automatic logic [2:0] cb_resp(cstate_e s);
    case (s)
      ST_UD:   return R_UD_PD;
      ST_SD:   return R_SD_PD;
      ST_UC:   return R_UC;
      ST_SC:   return R_SC;
      default: return R_I;
    endcase
  endfunction

  task automatic run_write(input rn_txn t, input line_t data, input bemask_t mask, input int unsigned ep, output bit ok); /*verilator no_inline_task*/
    local_rsp_flit_t rf;
    bit have_dbid, have_comp, data_sent, abort, sent, alive, is_cb, is_coh, got_err;
    logic [11:0] dbid;
    logic [2:0] resp;
    cstate_e st0;
    line_t sdata;
    bemask_t smask;
    dat_opcode_e dop;
    addr_t la;
    wr_pend wp;
    ok = 0; have_dbid = 0; have_comp = 0; data_sent = 0; abort = 0; got_err = 0; dbid = '0;
    la = line_of(t.addr);
    is_cb  = (t.opcode == REQ_WRITE_BACK_FULL || t.opcode == REQ_WRITE_CLEAN_FULL);
    is_coh = (t.opcode != REQ_WRITE_NO_SNOOP_FULL && t.opcode != REQ_WRITE_NO_SNOOP_PTL);
    wp = null;
    if (!is_cb) wp = sb.wr_begin(t.addr, data, mask, NAME);
    else begin evicting[la] = 1; wb_lost[la] = 0; end
    txn_open(t);
    send_req_flit(mk_req(t), ep, sent);
    abort = !sent;
    while (!(have_comp && data_sent) && !abort) begin
      next_rsp(t, ep, rf, alive);
      if (!alive) abort = 1;
      else begin
        if (rf.resperr != ERR_OK) got_err = 1;
        case (rf.opcode)
          RSP_COMP_DBID_RESP: begin
            if (have_dbid || have_comp) sb.err("PROTOCOL", $sformatf("%s: duplicate Comp/DBID for txnid=%0d", NAME, t.txnid));
            have_dbid = 1; have_comp = 1; dbid = rf.dbid;
          end
          RSP_DBID_RESP: begin
            if (have_dbid) sb.err("PROTOCOL", $sformatf("%s: duplicate DBIDResp for txnid=%0d", NAME, t.txnid));
            have_dbid = 1; dbid = rf.dbid;
          end
          RSP_COMP: begin
            if (have_comp) sb.err("PROTOCOL", $sformatf("%s: duplicate Comp for txnid=%0d", NAME, t.txnid));
            if (!have_dbid) sb.cov_hit("rn: Comp before DBIDResp");
            have_comp = 1;
          end
          default: sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d received unexpected RSP %s", NAME, t.opcode.name(), t.txnid, rf.opcode.name()));
        endcase
        if (have_dbid && !data_sent) begin
          if (dbid_inuse.exists(dbid))
            sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d got DBID %0d, which another of my writes is still using", NAME, t.opcode.name(), t.txnid, dbid));
          dbid_inuse[dbid] = 1;
          if (is_cb) begin
            if (wb_lost[la]) begin
              resp = R_I; smask = '0; sdata = rng.rline();      // line was taken by a snoop: nothing left to write
              if (t.opcode == REQ_WRITE_CLEAN_FULL) sb.cov_hit("rn: WriteCleanFull lost its line to a snoop");
            end else begin
              st0 = get_st(la); resp = cb_resp(st0); smask = '1;
              sdata = ldata.exists(la) ? ldata[la] : rng.rline();
              sb.check_cached(la, sdata, NAME, {t.opcode.name(), " write-back data"});
            end
            dop = DAT_COPY_BACK_WR_DATA;
          end else begin
            resp = R_I; sdata = data; smask = mask; dop = DAT_NON_COPY_BACK_WR_DATA;
          end
          send_dat_beats(dbid, HNID, dop, resp, dbid, sdata, smask, t.addr, t.size, ep);
          if (ep == sb.epoch) dbid_inuse.delete(dbid);
          data_sent = 1;
          if (is_cb && ep == sb.epoch) begin
            if (!wb_lost[la]) begin
              if (t.opcode == REQ_WRITE_BACK_FULL) set_st(la, ST_I);
              else if (get_st(la) == ST_UD) set_st(la, ST_UC);
              else if (get_st(la) == ST_SD) set_st(la, ST_SC);
            end
            evicting[la] = 0;
          end
        end
      end
    end
    if (abort) begin
      if (ep == sb.epoch && !is_cb) sb.wr_abort(t.addr, wp);
      if (ep == sb.epoch && is_cb) evicting[la] = 0;
      fail_txn(t, ep, $sformatf("%s did not complete (dbid=%0b comp=%0b data_sent=%0b)", t.opcode.name(), have_dbid, have_comp, data_sent));
    end else begin
      if (got_err) begin
        sb.n_err_seen++;
        if (!sb.is_err_line(t.addr)) sb.err("SPURIOUS_RESPERR", $sformatf("%s: %s txnid=%0d addr=%0h completed with RespErr but no error was injected", NAME, t.opcode.name(), t.txnid, t.addr));
      end
      if (!is_cb) begin
        if (got_err) sb.wr_abort(t.addr, wp); else sb.wr_end(t.addr, wp, NAME);
        if (is_coh) sb.check_invalid_except(la, IDX, NAME, {t.opcode.name(), " Comp"});
      end
      if (t.expack && ep == sb.epoch) begin
        sb.cov_hit("rn: CompAck on a write");
        send_rsp_flit(mk_rsp(HNID, t.txnid, RSP_COMP_ACK, R_I), ep);
      end
      txn_close(t, ep, 0);
      ok = 1;
    end
  endtask

  // ---------------------------------------------------------------- dataless (CMO) family
  task automatic run_cmo(input rn_txn t, input int unsigned ep, output bit ok); /*verilator no_inline_task*/
    local_rsp_flit_t rf;
    bit sent, alive;
    addr_t la;
    ok = 0;
    la = line_of(t.addr);
    if (t.opcode == REQ_MAKE_INVALID) sb.mark_unknown(la);
    txn_open(t);
    send_req_flit(mk_req(t), ep, sent);
    alive = 0;
    if (sent) next_rsp(t, ep, rf, alive);
    if (!alive) fail_txn(t, ep, $sformatf("%s did not complete", t.opcode.name()));
    else begin
      if (rf.opcode != RSP_COMP) sb.err("PROTOCOL", $sformatf("%s: %s txnid=%0d expected Comp, got %s", NAME, t.opcode.name(), t.txnid, rf.opcode.name()));
      if (rf.resperr != ERR_OK && !sb.is_err_line(t.addr)) sb.err("SPURIOUS_RESPERR", $sformatf("%s: %s txnid=%0d completed with RespErr", NAME, t.opcode.name(), t.txnid));
      case (t.opcode)
        REQ_CLEAN_SHARED: begin
          sb.check_no_dirty(la, NAME, "CleanShared Comp");
          sb.mem_check_line(la, NAME, "CleanShared Comp");
        end
        REQ_CLEAN_INVALID: begin
          sb.check_invalid_except(la, -1, NAME, "CleanInvalid Comp");
          sb.mem_check_line(la, NAME, "CleanInvalid Comp");
        end
        REQ_MAKE_INVALID: sb.check_invalid_except(la, IDX, NAME, "MakeInvalid Comp");
        default: ;
      endcase
      txn_close(t, ep, 0);
      ok = 1;
    end
  endtask

  // ---------------------------------------------------------------- operations (shared by random + directed)
  function automatic bit [NUM_OPS-1:0] allowed_ops(cstate_e st);
    bit [NUM_OPS-1:0] a;
    a = '0;
    a[OP_RS]  = (st == ST_I);  a[OP_RC] = (st == ST_I);  a[OP_RO] = (st == ST_I);
    a[OP_RU]  = (st == ST_I || st == ST_SC);
    a[OP_WUF] = (st == ST_I);  a[OP_WUP] = (st == ST_I);
    a[OP_WB]  = (st != ST_I);  a[OP_WC] = (st == ST_UD || st == ST_SD);
    a[OP_CS]  = (st == ST_I);  a[OP_CI] = (st == ST_I);  a[OP_MI] = (st == ST_I);
    a[OP_ST]  = (st == ST_UC || st == ST_UD);
    a[OP_SE]  = (st == ST_SC || st == ST_UC);
    a[OP_NRD] = 1'b1;          a[OP_NWR] = 1'b1;
    return a;
  endfunction

  function automatic lockkind_e lock_kind(opkind_e k);
    case (k)
      OP_WUF, OP_WUP, OP_ST, OP_NWR: return K_WRITE;
      OP_CS, OP_CI, OP_MI:           return K_EXCL;
      default:                       return K_READ;
    endcase
  endfunction

  function automatic bemask_t rand_mask();
    bemask_t m;
    m = rng.r64();
    if (m == '0) m = 64'h1;
    return m;
  endfunction

  task automatic do_op(input opkind_e k, input addr_t a_in, input int unsigned ep, output bit ran); /*verilator no_inline_task*/
    addr_t a, wa;
    cstate_e st;
    lockkind_e lk;
    rn_txn t;
    bit ok, got_t, partial;
    line_t d, nd;
    bemask_t m;
    width_e sz;
    int off, nbytes;
    req_opcode_e opc;
    bit [NUM_OPS-1:0] al;
    ran = 0;
    got_t = 1;
    a  = line_of(a_in);
    st = get_st(a);
    lk = lock_kind(k);
    al = allowed_ops(st);                   // bit-select on a function call result is not legal SV
    if (al[k] && sb.try_begin(a, IDX, lk, (k == OP_NRD || k == OP_NWR) && sb.nc_multi)) begin
      ok = 1;
      sb.n_op[k]++;
      ran = 1;
      case (k)
        OP_RS, OP_RC, OP_RU, OP_RO, OP_NRD: begin
          opc = (k == OP_RS) ? REQ_READ_SHARED : (k == OP_RC) ? REQ_READ_CLEAN :
                (k == OP_RU) ? REQ_READ_UNIQUE : (k == OP_RO) ? REQ_READ_ONCE : REQ_READ_NO_SNOOP;
          sz = WIDTH_64; wa = a;
          if (k == OP_NRD) begin
            sz = width_e'(rng.range(0, 6)); nbytes = size_bytes(sz); wa = a + addr_t'(rng.range(0, 63) & ~(nbytes - 1));
            if (nbytes > 1 && sb.unaligned_pct != 0 && rng.chance(sb.unaligned_pct)) begin
              wa = wa + addr_t'(rng.range(1, nbytes - 1));
              sb.cov_hit("rn: unaligned ReadNoSnp");
            end
          end
          got_t = new_txn(opc, wa, sz, (k != OP_NRD), t);
          if (got_t) run_read(t, ep, ok); else ok = 0;
        end
        OP_WUF, OP_WUP, OP_NWR: begin
          partial = (k == OP_WUP) || (k == OP_NWR && rng.chance(50));
          sz = WIDTH_64; wa = a;
          if (partial) begin sz = width_e'(rng.range(0, 6)); nbytes = size_bytes(sz); wa = a + addr_t'(rng.range(0, 63) & ~(nbytes - 1)); end
          if (k == OP_NWR) opc = partial ? REQ_WRITE_NO_SNOOP_PTL : REQ_WRITE_NO_SNOOP_FULL;
          else             opc = partial ? REQ_WRITE_UNIQUE_PTL   : REQ_WRITE_UNIQUE_FULL;
          m = '1;
          if (partial) begin
            m = span_mask(wa, sz) & rng.r64();
            if (m == '0) m = span_mask(wa, sz);
            if (sb.zero_be_pct != 0 && rng.chance(sb.zero_be_pct)) begin m = '0; sb.cov_hit("rn: partial write with BE=0"); end
          end
          d = rng.rline();
          // the byte enables are set for the Size-aligned window; the request
          // address itself may point anywhere inside that window
          if ((k == OP_NWR || k == OP_WUP) && partial && nbytes > 1 && sb.unaligned_pct != 0 && rng.chance(sb.unaligned_pct)) begin
            wa = wa + addr_t'(rng.range(1, nbytes - 1));
            sb.cov_hit((k == OP_NWR) ? "rn: unaligned WriteNoSnpPtl" : "rn: unaligned WriteUniquePtl");
          end
          got_t = new_txn(opc, wa, sz, (k != OP_NWR), t);
          if (got_t) run_write(t, d, m, ep, ok); else ok = 0;
        end
        OP_WB, OP_WC: begin
          opc = (k == OP_WB) ? REQ_WRITE_BACK_FULL : REQ_WRITE_CLEAN_FULL;
          got_t = new_txn(opc, a, WIDTH_64, 1, t);
          if (got_t) run_write(t, '0, '1, ep, ok); else ok = 0;
        end
        OP_CS, OP_CI: begin
          got_t = new_txn((k == OP_CS) ? REQ_CLEAN_SHARED : REQ_CLEAN_INVALID, a, WIDTH_64, 1, t);
          if (got_t) run_cmo(t, ep, ok); else ok = 0;
        end
        OP_MI: begin
          got_t = new_txn(REQ_MAKE_INVALID, a, WIDTH_64, 1, t);
          if (got_t) run_cmo(t, ep, ok); else ok = 0;
          if (ok) begin                                  // data is gone: refill with a full write
            got_t = new_txn(REQ_WRITE_UNIQUE_FULL, a, WIDTH_64, 1, t);
            if (got_t) run_write(t, rng.rline(), '1, ep, ok); else ok = 0;
          end
        end
        OP_ST: begin                                      // silent store: UC/UD -> UD, no bus traffic
          m = rand_mask(); d = rng.rline();
          ldata[a] = merge_line(ldata[a], d, m);
          sb.local_store(a, IDX, d, m, NAME);
          set_st(a, ST_UD);
        end
        OP_SE: begin                                      // silent clean eviction: HN-F directory goes stale
          set_st(a, ST_I);
        end
        default: ;
      endcase
      // on timeout the line stays locked on purpose; an op that never started (no free TxnID) releases it
      if (ep == sb.epoch && (ok || !got_t)) sb.end_op(a, IDX, lk);
    end
  endtask

  task automatic make_invalid_local(input addr_t a, input int unsigned ep);
    bit ran;
    case (get_st(line_of(a)))
      ST_SC, ST_UC: do_op(OP_SE, a, ep, ran);
      ST_UD, ST_SD: do_op(OP_WB, a, ep, ran);
      default: ;
    endcase
  endtask

  // directed helper: bring the line into a state from which `k` is legal, then run it
  task automatic do_op_f(input opkind_e k, input addr_t a, input int unsigned ep); /*verilator no_inline_task*/
    bit ran;
    cstate_e st;
    st = get_st(line_of(a));
    case (k)
      OP_RS, OP_RC, OP_RO, OP_WUF, OP_WUP, OP_CS, OP_CI, OP_MI: make_invalid_local(a, ep);
      OP_RU: if (st != ST_I && st != ST_SC) make_invalid_local(a, ep);
      OP_ST: if (st != ST_UC && st != ST_UD) begin make_invalid_local(a, ep); do_op(OP_RU, a, ep, ran); end
      default: ;
    endcase
    do_op(k, a, ep, ran);
  endtask

  // ---------------------------------------------------------------- random driver
  task automatic random_op_once(input int unsigned ep);
    addr_t a;
    opkind_e k;
    bit [NUM_OPS-1:0] al;
    int unsigned tot, roll, acc, hn;
    bit ran, picked;
    hn = (sb.hot_n < sb.pool.size()) ? sb.hot_n : sb.pool.size();
    if (rng.chance(sb.hot_pct)) a = sb.pool[rng.range(0, hn - 1)];
    else                        a = sb.pool[rng.range(0, sb.pool.size() - 1)];
    al = allowed_ops(get_st(line_of(a)));
    tot = 0;
    for (int i = 0; i < NUM_OPS; i++) if (al[i]) tot += sb.w[i];
    if (tot != 0) begin
      roll = rng.range(0, tot - 1); acc = 0; picked = 0; k = OP_RS;
      for (int i = 0; i < NUM_OPS; i++) begin
        if (al[i] && !picked) begin
          acc += sb.w[i];
          if (roll < acc) begin k = opkind_e'(i); picked = 1; end
        end
      end
      if (k == OP_NRD || k == OP_NWR) a = sb.nc_pool[rng.range(0, sb.nc_pool.size() - 1)];
      do_op(k, a, ep, ran);
    end
  endtask

  task automatic random_ops(input int unsigned n, input int unsigned ep);
    launched = 0; outstanding = 0;
    while (launched < n && ep == sb.epoch) begin
      if (outstanding < sb.max_out) begin
        launched++; outstanding++;
        fork
          begin
            automatic int unsigned my_ep = ep;
            random_op_once(my_ep);
            if (my_ep == sb.epoch) outstanding--;
          end
        join_none
      end
      repeat (rng.range(1, 4)) @(posedge clk);
    end
    while (outstanding > 0 && ep == sb.epoch) @(posedge clk);
  endtask

  // ---------------------------------------------------------------- snoop engine
  function automatic bit is_fwd(snp_opcode_e op);
    return (op >= SNP_FWD_SHARED && op <= SNP_FWD_MAKE_INVALID);
  endfunction

  function automatic snp_opcode_e strip_fwd(snp_opcode_e op);
    case (op)
      SNP_FWD_SHARED:           return SNP_SHARED;
      SNP_FWD_CLEAN:            return SNP_CLEAN;
      SNP_FWD_ONCE:             return SNP_ONCE;
      SNP_FWD_NOT_SHARED_DIRTY: return SNP_NOT_SHARED_DIRTY;
      SNP_FWD_UNIQUE:           return SNP_UNIQUE;
      SNP_FWD_CLEAN_SHARED:     return SNP_CLEAN_SHARED;
      SNP_FWD_CLEAN_INVALID:    return SNP_CLEAN_INVALID;
      default:                  return SNP_MAKE_INVALID;
    endcase
  endfunction

  // legal snoop response selection (B4.5 tables); `rts` = RetToSrc
  function automatic void snoop_decide(input snp_opcode_e op, input cstate_e st, input bit rts, input bit dngsd,
                                       output cstate_e nst, output bit send_data, output logic [2:0] resp);
    int unsigned r;
    nst = st; send_data = 0; resp = R_I;
    r = rng.range(0, 99);
    case (op)
      SNP_SHARED, SNP_CLEAN, SNP_NOT_SHARED_DIRTY: begin
        case (st)
          ST_SC: begin
            if (r < 75) begin nst = ST_SC; resp = R_SC; end else begin nst = ST_I; resp = R_I; end
            send_data = rts;
          end
          ST_UC: begin
            if (r < 70) begin nst = ST_SC; resp = R_SC; end else begin nst = ST_I; resp = R_I; end
            send_data = rts;
          end
          ST_UD, ST_SD: begin
            send_data = 1;
            if (op == SNP_SHARED && !dngsd && r < 40) begin nst = ST_SD;  resp = R_SD;    end
            else if (r < 75)                          begin nst = ST_SC;  resp = R_SC_PD; end
            else                                      begin nst = ST_I;   resp = R_I_PD;  end
          end
          default: ;
        endcase
      end
      SNP_ONCE: begin
        case (st)
          ST_SC: begin resp = R_SC; send_data = rts; end
          ST_UC: begin resp = R_UC; send_data = rts; end
          ST_UD: begin
            send_data = 1;
            if (r < 75) begin resp = R_UC; end else begin nst = ST_I; resp = R_I_PD; end
          end
          ST_SD: begin send_data = 1; resp = R_SD; end
          default: ;
        endcase
      end
      SNP_UNIQUE, SNP_CLEAN_INVALID: begin
        nst = ST_I;
        case (st)
          ST_SC, ST_UC: begin
            resp = R_I; send_data = rts;
            // a clean line may be returned on SnpUnique even with RetToSrc=0 (SnpRespData_I)
            if (!rts && op == SNP_UNIQUE && sb.snp_opt_data_pct != 0 && rng.chance(sb.snp_opt_data_pct)) begin
              send_data = 1; sb.cov_hit("rn: clean SnpRespData_I on SnpUnique RetToSrc=0");
            end
          end
          ST_UD, ST_SD: begin resp = R_I_PD; send_data = 1; end
          default: ;
        endcase
      end
      SNP_MAKE_INVALID: begin nst = ST_I; resp = R_I; send_data = 0; end   // dirty data discarded by definition
      SNP_CLEAN_SHARED: begin
        case (st)
          ST_SC: begin resp = R_SC; end
          ST_UC: begin resp = R_UC; end
          ST_UD: begin
            send_data = 1;
            if (r < 40)      begin nst = ST_UC; resp = R_UD_PD; end
            else if (r < 80) begin nst = ST_SC; resp = R_SC_PD; end
            else             begin nst = ST_I;  resp = R_I_PD;  end
          end
          ST_SD: begin
            send_data = 1;
            if (r < 60) begin nst = ST_SC; resp = R_SC_PD; end else begin nst = ST_I; resp = R_I_PD; end
          end
          default: ;
        endcase
      end
      default: ;
    endcase
  endfunction

  task automatic handle_snoop(input local_snp_flit_t f, input int unsigned ep);
    snp_opcode_e op;
    addr_t la;
    cstate_e st, nst;
    bit send_data;
    logic [2:0] resp;
    line_t d;
    op = f.opcode;
    la = line_of(f.addr);
    repeat (rng.range(0, sb.snp_lat_max)) @(posedge clk);
    if (ep == sb.epoch) begin
      sb.n_snp++;
      if (op == SNP_DVM_OP)
        sb.err("PROTOCOL", $sformatf("%s: HN-F sent SnpDVMOp (HN-F must never snoop DVM) txnid=%0d", NAME, f.txnid));
      if (is_fwd(op)) begin
        sb.err("PROTOCOL", $sformatf("%s: forwarding snoop %s received - TB has no Fwded response encodings in chi_pkg, answering as non-forwarding snoop", NAME, op.name()));
        op = strip_fwd(op);
      end
      st = get_st(la);
      snoop_decide(op, st, f.rettosrc, f.donotgotosd, nst, send_data, resp);
      d = ldata.exists(la) ? ldata[la] : '0;
      if (send_data && st != ST_I) sb.check_cached(la, d, NAME, {f.opcode.name(), " snoop data"});
      if (nst != st) begin
        if (nst == ST_I && evicting.exists(la) && evicting[la]) wb_lost[la] = 1;
        set_st(la, nst);
      end
      if (send_data) begin
        sb.n_snp_data++;
        send_dat_beats(f.txnid, f.srcid, DAT_SNP_RESP_DATA, resp, '0, d, '1, la, WIDTH_64, ep);
      end else begin
        send_rsp_flit(mk_rsp(f.srcid, f.txnid, RSP_SNP_RESP, resp), ep);
      end
      if (ep == sb.epoch) snp_busy.delete(f.txnid);
    end
  endtask

  task automatic snoop_engine();
    local_snp_flit_t f;
    forever begin
      while (snp_q.size() == 0) @(posedge clk);
      f = snp_q.pop_front();
      fork
        begin
          automatic local_snp_flit_t f2 = f;
          automatic int unsigned my_ep = my_epoch;
          handle_snoop(f2, my_ep);
        end
      join_none
      @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------- directed scenarios
  task automatic sync(input int n, input int unsigned ep);
    int unsigned g;
    g = sb.bar_gen;
    sb.bar_cnt++;
    if (sb.bar_cnt >= n) begin sb.bar_cnt = 0; sb.bar_gen++; end
    while (sb.bar_gen == g && ep == sb.epoch) @(posedge clk);
  endtask

  task automatic skew(input int unsigned mx);
    repeat (rng.range(0, mx)) @(posedge clk);
  endtask

  // sync() lets the last RN-F through up to one cycle before the others;
  // one more edge puts every RN-F on the same cycle
  task automatic sync_aligned(input int n, input int unsigned ep);
    sync(n, ep);
    @(posedge clk);
  endtask

  // a random coherent op that is legal from `st` and may overlap with the other RN-F
  function automatic opkind_e pick_op(cstate_e st);
    opkind_e c [$];
    bit [NUM_OPS-1:0] al;
    al = allowed_ops(st);
    if (al[OP_RS])  c.push_back(OP_RS);
    if (al[OP_RC])  c.push_back(OP_RC);
    if (al[OP_RU])  c.push_back(OP_RU);
    if (al[OP_RO])  c.push_back(OP_RO);
    if (al[OP_WUF]) c.push_back(OP_WUF);
    if (al[OP_WUP]) c.push_back(OP_WUP);
    if (al[OP_WB])  c.push_back(OP_WB);
    if (al[OP_WC])  c.push_back(OP_WC);
    if (al[OP_ST])  c.push_back(OP_ST);
    if (c.size() == 0) return OP_RS;
    return c[rng.range(0, c.size() - 1)];
  endfunction

  function automatic opkind_e nc_op();
    return rng.chance(50) ? OP_NWR : OP_NRD;
  endfunction

  task automatic run_scenario(input int id, input int unsigned ep);
    int nact;
    addr_t a0, a1, a2, a3, n0, n1;
    opkind_e ko[9];
    bit ran;
    nact = (sb.num_rnf >= 2) ? 2 : 1;
    a0 = sb.pool[0]; a1 = sb.pool[1]; a2 = sb.pool[2]; a3 = sb.pool[3];
    n0 = sb.nc_pool[0]; n1 = sb.nc_pool[1];
    if (IDX < nact) begin
      case (id)
        SC_PRODCONS: begin
          for (int it = 0; it < 6; it++) begin
            if (IDX == 0) begin do_op_f(OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); do_op_f(OP_ST, a0, ep); end
            sync(nact, ep);
            if (IDX == nact - 1) do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
            if (IDX == nact - 1) begin do_op_f(OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); end
            sync(nact, ep);
            if (IDX == 0) do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
          end
        end
        SC_UNIQUE_RACE: begin
          ko[0] = OP_RU; ko[1] = OP_RS; ko[2] = OP_RC; ko[3] = OP_WUF;
          for (int it = 0; it < 14; it++) begin
            sync(nact, ep);
            skew(3);
            do_op_f(ko[rng.range(0, 3)], a0, ep);
            sync(nact, ep);
            if (rng.chance(60)) do_op_f(OP_ST, a0, ep);
            sync(nact, ep);
          end
        end
        SC_WB_SNOOP_RACE: begin
          for (int it = 0; it < 12; it++) begin
            if (IDX == 0) begin do_op_f(OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); end
            sync(nact, ep);
            if (IDX == 0) do_op_f(OP_WB, a0, ep);
            else begin skew(8); do_op_f(rng.chance(50) ? OP_RU : OP_RS, a0, ep); end
            sync(nact, ep);
          end
        end
        SC_WU_SHARERS: begin
          for (int it = 0; it < 6; it++) begin
            do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
            if (IDX == 0) do_op_f(it[0] ? OP_WUF : OP_WUP, a0, ep);
            sync(nact, ep);
            if (IDX == nact - 1) do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
          end
        end
        SC_PTL_MERGE: begin
          for (int it = 0; it < 8; it++) begin
            if (IDX == 0) begin do_op_f(OP_RU, a1, ep); do_op_f(OP_ST, a1, ep); end
            sync(nact, ep);
            if (IDX == nact - 1) do_op_f(OP_WUP, a1, ep);
            sync(nact, ep);
            do_op_f(OP_RS, a1, ep);
            sync(nact, ep);
          end
        end
        SC_SNF_ERR: begin
          for (int it = 0; it < 4; it++) begin
            if (IDX == 0) do_op_f(OP_RS, a2, ep);
            sync(nact, ep);
            if (IDX == nact - 1) do_op_f(OP_WUF, a2, ep);
            sync(nact, ep);
            if (IDX == 0) do_op_f(OP_RU, a2, ep);
            sync(nact, ep);
            do_op_f(OP_SE, a2, ep);
            sync(nact, ep);
          end
        end
        SC_SILENT_EVICT: begin
          for (int it = 0; it < 8; it++) begin
            if (IDX == 0) begin do_op_f(it[0] ? OP_RU : OP_RC, a0, ep); do_op(OP_SE, a0, ep, ran); end
            sync(nact, ep);
            if (IDX == nact - 1) begin do_op_f(it[1] ? OP_RS : OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); end
            sync(nact, ep);
          end
        end
        SC_MAKEINV: begin
          for (int it = 0; it < 4; it++) begin
            if (IDX == 0) do_op_f(OP_MI, a0, ep);
            sync(nact, ep);
            if (IDX == nact - 1) begin do_op_f(OP_RS, a0, ep); do_op_f(OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); end
            sync(nact, ep);
            if (IDX == 0) do_op_f(OP_MI, a0, ep);
            sync(nact, ep);
            if (IDX == nact - 1) do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
          end
        end
        SC_CMO_FLUSH: begin
          for (int it = 0; it < 4; it++) begin
            if (IDX == 0) begin do_op_f(OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); do_op_f(OP_RU, a1, ep); do_op_f(OP_ST, a1, ep); end
            sync(nact, ep);
            if (IDX == nact - 1) begin do_op_f(OP_CI, a0, ep); do_op_f(OP_CS, a1, ep); end
            sync(nact, ep);
            if (IDX == 0) begin do_op_f(OP_RS, a0, ep); do_op_f(OP_RS, a1, ep); end
            sync(nact, ep);
          end
        end
        SC_NONCOH: begin
          for (int it = 0; it < 8; it++) begin
            if (IDX == 0) do_op(OP_NWR, n0, ep, ran);
            sync(nact, ep);
            if (IDX == nact - 1) begin do_op(OP_NRD, n0, ep, ran); do_op(OP_NWR, n1, ep, ran); end
            sync(nact, ep);
            if (IDX == 0) do_op(OP_NRD, n1, ep, ran);
            sync(nact, ep);
          end
        end
        SC_STATE_MATRIX: begin
          ko[0] = OP_RS; ko[1] = OP_RC; ko[2] = OP_RU; ko[3] = OP_RO; ko[4] = OP_WUF;
          ko[5] = OP_WUP; ko[6] = OP_CS; ko[7] = OP_CI; ko[8] = OP_MI;
          for (int s = 0; s < 4; s++) begin
            for (int o = 0; o < 9; o++) begin
              make_invalid_local(a3, ep);
              sync(nact, ep);
              if (s == 1) begin
                if (IDX == nact - 1) do_op_f(OP_RS, a3, ep);
                sync(nact, ep);
                if (IDX == 0) do_op_f(OP_RS, a3, ep);
                sync(nact, ep);
                if (IDX == nact - 1 && nact > 1) make_invalid_local(a3, ep);
              end else begin
                if (IDX == 0) begin
                  if (s == 2) do_op_f(OP_RU, a3, ep);
                  if (s == 3) begin do_op_f(OP_RU, a3, ep); do_op_f(OP_ST, a3, ep); end
                end
              end
              sync(nact, ep);
              if (IDX == nact - 1) do_op_f(ko[o], a3, ep);
              sync(nact, ep);
            end
          end
        end
        // both RN-Fs send a request for the same line on the same clock edge
        SC_SAME_CYCLE: begin
          for (int it = 0; it < 48; it++) begin
            addr_t ax;
            ax = (it % 3 == 2) ? a1 : a0;
            make_invalid_local(ax, ep);
            sync(nact, ep);
            if (IDX == it % nact && rng.chance(60)) begin
              do_op_f(OP_RU, ax, ep);
              if (rng.chance(60)) do_op_f(OP_ST, ax, ep);
            end
            sync_aligned(nact, ep);
            do_op(pick_op(get_st(ax)), ax, ep, ran);
            sync(nact, ep);
          end
        end
        // both RN-Fs write the same line at the same time, then one reads it
        SC_WW_RACE: begin
          for (int it = 0; it < 32; it++) begin
            make_invalid_local(a1, ep);
            sync_aligned(nact, ep);
            skew(it % 3);
            do_op(rng.chance(50) ? OP_WUF : OP_WUP, a1, ep, ran);
            sync(nact, ep);
            if (IDX == it % nact) do_op_f(rng.chance(50) ? OP_RS : OP_RU, a1, ep);
            sync(nact, ep);
          end
        end
        // three overlapping ReadNoSnp / WriteNoSnp per RN-F on ONE line: more
        // requests than trackers, all hazard-chained behind each other
        SC_NC_PILEUP: begin
          for (int it = 0; it < 16; it++) begin
            addr_t nx;
            nx = it[0] ? n1 : n0;
            sync_aligned(nact, ep);
            fork
              begin bit r0; do_op(nc_op(), nx, ep, r0); end
              begin bit r1; skew(2); do_op(nc_op(), nx, ep, r1); end
              begin bit r2; skew(5); do_op(nc_op(), nx, ep, r2); end
            join
            sync(nact, ep);
          end
        end
        // dirty data moves by snoop while the other RN-F immediately follows
        // up on the same line (HN-F dirty write-back still in flight)
        SC_DIRTY_CHAIN: begin
          for (int it = 0; it < 20; it++) begin
            make_invalid_local(a2, ep);
            sync(nact, ep);
            if (IDX == 0) begin do_op_f(OP_RU, a2, ep); do_op_f(OP_ST, a2, ep); end
            sync_aligned(nact, ep);
            if (IDX == nact - 1) begin
              case (rng.range(0, 3))
                0: do_op(OP_RS, a2, ep, ran);
                1: do_op(OP_RC, a2, ep, ran);
                2: do_op(OP_RO, a2, ep, ran);
                default: do_op(OP_RU, a2, ep, ran);
              endcase
              do_op(pick_op(get_st(a2)), a2, ep, ran);
            end else begin
              skew(6);
              do_op(pick_op(get_st(a2)), a2, ep, ran);
            end
            sync(nact, ep);
            if (IDX == it % nact) begin
              make_invalid_local(a2, ep);
              do_op(rng.chance(50) ? OP_CS : OP_CI, a2, ep, ran);
            end
            sync(nact, ep);
          end
        end
        // WriteCleanFull of a dirty line races a snoop from the other RN-F's
        // request for that line. The writer alternates; the line is checked
        // by a read from each side afterwards.
        SC_WC_SNOOP_RACE: begin
          ko[0] = OP_RU; ko[1] = OP_RS; ko[2] = OP_RC; ko[3] = OP_RO; ko[4] = OP_WUF; ko[5] = OP_WUP; ko[6] = OP_CI;
          for (int it = 0; it < 24; it++) begin
            int wr;
            wr = it % nact;                               // this iteration's WriteClean sender
            if (IDX == wr) begin do_op_f(OP_RU, a0, ep); do_op_f(OP_ST, a0, ep); end
            else make_invalid_local(a0, ep);
            sync_aligned(nact, ep);
            if (IDX == wr) begin
              skew(it % 4);
              do_op(OP_WC, a0, ep, ran);                  // UD -> UC, unless the snoop got there first
              if (rng.chance(50)) do_op(pick_op(get_st(a0)), a0, ep, ran);
            end else begin
              skew(6);
              do_op(ko[rng.range(0, 6)], a0, ep, ran);
            end
            sync(nact, ep);
            if (IDX != wr) do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
            if (IDX == wr) do_op_f(OP_RS, a0, ep);
            sync(nact, ep);
          end
        end
        default: random_ops(sb.scn_ops, ep);   // knob-driven scenarios (see top-level apply_profile)
      endcase
    end
  endtask

  // end-of-test flush: everything evicted/written back, then RN0 CleanInvalid on every line
  task automatic flush_all(input int unsigned ep);
    bit ran;
    foreach (sb.pool[i]) begin
      case (get_st(line_of(sb.pool[i])))
        ST_UD, ST_SD:  do_op(OP_WB, sb.pool[i], ep, ran);
        ST_SC, ST_UC:  do_op(OP_SE, sb.pool[i], ep, ran);
        default: ;
      endcase
    end
    sync(sb.num_rnf, ep);
    if (IDX == 0) foreach (sb.pool[i]) do_op(OP_CI, sb.pool[i], ep, ran);
  endtask

  // ---------------------------------------------------------------- command loop / start-up
  task automatic cmd_loop();
    int unsigned last, ep;
    last = 0;
    forever begin
      while (sb.cmd_seq == last) @(posedge clk);
      last = sb.cmd_seq;
      ep = my_epoch;
      case (sb.cmd)
        CMD_SCEN:   run_scenario(sb.cmd_arg, ep);
        CMD_RANDOM: random_ops(sb.cmd_arg, ep);
        CMD_FLUSH:  flush_all(ep);
        default: ;
      endcase
      sb.agents_done++;
    end
  endtask

  initial begin
    while (sb == null) @(posedge clk);
    fork
      snoop_engine();
      cmd_loop();
    join_none
  end

endmodule