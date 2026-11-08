`timescale 1ns/1ps
// =============================================================================
// snf_model - spec-strict SN-F agent.
//  * credit-based link layer both directions, random credit back-pressure
//  * requests accepted into a queue, each served by its own thread: random,
//    occasionally very long latency -> out-of-order completion
//  * data beats optionally shuffled / interleaved with other transactions
//  * writes: CompDBIDResp or DBIDResp(+Comp after data), randomised unique DBIDs
//  * RetryAck / PCrdGrant, RespErr injection
//  * checks: misrouted address, illegal opcode, alignment/size, TxnID reuse
//    while in flight, duplicate/missing/early write data, wrong write-data opcode
// =============================================================================
module snf_model #(
  parameter string             NAME   = "SNF",
  parameter int                IDX    = 0,
  parameter chi_pkg::node_id_e NODEID = 7'h40,
  parameter int                SEED   = 1
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
  import tshr_flit_pkg::*;
  import chi_tb_pkg::*;

  // ---------------- state ----------------
  tb_rng rng, crng;
  tx_lock rsp_lock, dat_lock;
  int unsigned cyc = 0;
  int unsigned my_epoch = 0;

  int rsp_cr_in = 0, rsp_cr_out = 0, dat_cr_in = 0, dat_cr_out = 0;
  int req_gr = 0, req_rc = 0, wd_gr = 0, wd_rc = 0;

  local_req_flit_t req_q [$];
  bit              inflight  [logic [11:0]];   // HN TxnIDs being served
  bit              retried   [logic [11:0]];   // waiting for HN to resend
  bit              resent    [logic [11:0]];
  local_req_flit_t retried_rq [logic [11:0]];  // the request that got RetryAck, to compare the resend with

  // write context, keyed by granted DBID
  class wr_ctx;
    logic [11:0] txnid;
    addr_t       addr;
    width_e      size;
    line_t       data;
    bemask_t     mask;
    int          nbeats, got;
    bit [3:0]    seen;
    bit          done, bad;
    bit          dbid_sent;
  endclass
  wr_ctx wr_by_dbid [logic [11:0]];
  bit    dbid_used  [logic [11:0]];

  initial begin
    rng = new(SEED); crng = new(SEED ^ 32'h00C0FFEE);
    rsp_lock = new(); dat_lock = new();
  end

  function automatic bit alloc_dbid(output logic [11:0] id);
    logic [11:0] c;
    for (int k = 0; k < 4096; k++) begin
      c = 12'(rng.range(0, 4095));
      if (!dbid_used.exists(c)) begin id = c; dbid_used[c] = 1; return 1; end
    end
    id = '0;
    return 0;
  endfunction

  function automatic void reset_state();
    req_q.delete(); inflight.delete(); retried.delete(); resent.delete(); retried_rq.delete();
    wr_by_dbid.delete(); dbid_used.delete();
    rsp_cr_out = 0; dat_cr_out = 0;
    rsp_lock = new(); dat_lock = new();
  endfunction

  // ---------------- request checks / enqueue ----------------
  function automatic void rx_req_handle(local_req_flit_t f);
    bit legal;
    legal = (f.opcode == REQ_READ_NO_SNOOP) || (f.opcode == REQ_WRITE_NO_SNOOP_FULL) || (f.opcode == REQ_WRITE_NO_SNOOP_PTL);
    if (!legal) begin
      sb.err("ILLEGAL_OPCODE", $sformatf("%s: HN-F sent opcode %s to SN-F (only ReadNoSnp/WriteNoSnp* allowed) txnid=%0d addr=%0h", NAME, f.opcode.name(), f.txnid, f.addr));
    end
    if (sb.snf_of(f.addr) != IDX) begin
      sb.n_misroute++;
      sb.err("MISROUTE", $sformatf("%s: addr=%0h belongs to SN-F%0d (interleave bit %0d) but was sent to SN-F%0d", NAME, f.addr, sb.snf_of(f.addr), sb.ilv_lsb, IDX));
    end
    if (f.opcode == REQ_READ_NO_SNOOP || f.opcode == REQ_WRITE_NO_SNOOP_FULL) begin
      if (f.opcode == REQ_WRITE_NO_SNOOP_FULL && f.size != WIDTH_64)
        sb.err("PROTOCOL", $sformatf("%s: WriteNoSnpFull with size!=64B txnid=%0d", NAME, f.txnid));
    end
    // unaligned addresses are legal for ReadNoSnp / WriteNoSnpPtl to Normal
    // memory; the TB only sends them to the non-coherent lines
    if ((f.addr & addr_t'(size_bytes(f.size) - 1)) != 0 && !(sb.is_nc(f.addr) && f.opcode != REQ_WRITE_NO_SNOOP_FULL))
      sb.err("PROTOCOL", $sformatf("%s: unaligned request addr=%0h size=%0dB txnid=%0d", NAME, f.addr, size_bytes(f.size), f.txnid));
    if (sb.chk_ids && f.tgtid !== NODEID)
      sb.err("PROTOCOL", $sformatf("%s: request TgtID=%0h, expected %0h txnid=%0d", NAME, f.tgtid, NODEID, f.txnid));
    if (inflight.exists(f.txnid) && inflight[f.txnid])
      sb.err("TXNID_REUSE", $sformatf("%s: HN-F reused TxnID %0d while previous request still in flight", NAME, f.txnid));
    if (f.allowretry == N) begin
      // a request without AllowRetry must spend a P-credit this SN-F granted
      if (sb.pcrd_out[f.pcrdttype] <= 0)
        sb.err("PCRD", $sformatf("%s: %s txnid=%0d sent with AllowRetry=0 and PCrdType=%0d, but no unused PCrdGrant of that type exists", NAME, f.opcode.name(), f.txnid, f.pcrdttype));
      else
        sb.pcrd_out[f.pcrdttype]--;
      if (!retried_rq.exists(f.txnid))
        sb.err("PCRD", $sformatf("%s: %s txnid=%0d sent with AllowRetry=0 but this TxnID was never retried", NAME, f.opcode.name(), f.txnid));
      else if (retried_rq[f.txnid].opcode != f.opcode || retried_rq[f.txnid].addr != f.addr || retried_rq[f.txnid].size != f.size)
        sb.err("PCRD", $sformatf("%s: resent txnid=%0d is %s addr=%0h size=%0d, but the retried request was %s addr=%0h size=%0d", NAME, f.txnid,
                                 f.opcode.name(), f.addr, f.size, retried_rq[f.txnid].opcode.name(), retried_rq[f.txnid].addr, retried_rq[f.txnid].size));
      if (retried_rq.exists(f.txnid)) retried_rq.delete(f.txnid);
    end
    if (retried.exists(f.txnid) && f.allowretry == N) begin
      resent[f.txnid] = 1;
    end
    req_q.push_back(f);
  endfunction

  function automatic void rx_dat_handle(local_dat_flit_t f);
    wr_ctx c;
    logic [11:0] key;
    bit found;
    found = 0;
    if (wr_by_dbid.exists(f.txnid)) begin key = f.txnid; found = 1; end
    else if (wr_by_dbid.exists(f.dbid)) begin
      key = f.dbid; found = 1;
      sb.err("PROTOCOL", $sformatf("%s: write data carries DBID %0d in the DBID field but TxnID=%0d (spec: TxnID must equal the granted DBID)", NAME, f.dbid, f.txnid));
    end
    if (!found) begin
      sb.n_unexp++;
      sb.err("UNEXPECTED_RSP", $sformatf("%s: write data with TxnID=%0d dbid=%0d matches no granted DBID (opcode=%s)", NAME, f.txnid, f.dbid, f.opcode.name()));
    end else begin
      c = wr_by_dbid[key];
      if (f.opcode != DAT_NON_COPY_BACK_WR_DATA)
        sb.err("PROTOCOL", $sformatf("%s: write data opcode %s, expected NonCopyBackWrData (dbid=%0d)", NAME, f.opcode.name(), key));
      if (!c.dbid_sent)
        sb.err("PROTOCOL", $sformatf("%s: write data for DBID %0d arrived before the DBID response was sent", NAME, key));
      if (c.seen[f.dataid]) begin
        sb.err("PROTOCOL", $sformatf("%s: duplicate write data beat dataid=%0d dbid=%0d", NAME, f.dataid, key));
      end else begin
        c.seen[f.dataid] = 1;
        c.data[f.dataid*128 +: 128] = f.data;
        c.mask[f.dataid*16 +: 16]   = f.be;
        c.got++;
        if (f.dataid < first_dataid(c.addr, c.size) || f.dataid >= first_dataid(c.addr, c.size) + c.nbeats)
          sb.err("PROTOCOL", $sformatf("%s: write data beat dataid=%0d outside the %0d-beat window of dbid=%0d", NAME, f.dataid, c.nbeats, key));
      end
      if (c.got >= c.nbeats) c.done = 1;
    end
  endfunction

  // ---------------- link layer + monitors ----------------
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (sb != null && my_epoch != sb.epoch) begin
      reset_state();
      my_epoch = sb.epoch;
      rsp_tx.flitv <= N; rsp_tx.flitpend <= N; dat_tx.flitv <= N; dat_tx.flitpend <= N;
    end
    if (!resetn || sb == null) begin
      rsp_cr_in <= 0; dat_cr_in <= 0;
      req_rx.lcrdv <= 1'b0; dat_rx.lcrdv <= 1'b0;
      req_gr = 0; req_rc = 0; wd_gr = 0; wd_rc = 0;
    end else begin
      if (rsp_tx.lcrdv === 1'b1) rsp_cr_in <= rsp_cr_in + 1;
      if (dat_tx.lcrdv === 1'b1) dat_cr_in <= dat_cr_in + 1;

      if (req_rx.lcrdv === 1'b1) req_gr++;
      if (req_rx.flitv == Y) begin
        if (req_rc >= req_gr) sb.err("LINK_CREDIT", $sformatf("%s: HN-F sent a request flit without a credit", NAME));
        req_rc++;
        rx_req_handle(local_req_flit_t'(req_rx.flit));
      end
      req_rx.lcrdv <= ((req_gr - req_rc) < int'(sb.rx_cr_cap)) && crng.chance(sb.rx_cr_pct);

      if (dat_rx.lcrdv === 1'b1) wd_gr++;
      if (dat_rx.flitv == Y) begin
        if (wd_rc >= wd_gr) sb.err("LINK_CREDIT", $sformatf("%s: HN-F sent a data flit without a credit", NAME));
        wd_rc++;
        rx_dat_handle(local_dat_flit_t'(dat_rx.flit));
      end
      dat_rx.lcrdv <= ((wd_gr - wd_rc) < int'(sb.rx_cr_cap)) && crng.chance(sb.rx_cr_pct);
    end
  end

  // ---------------- TX primitives ----------------
  task automatic send_rsp_flit(input local_rsp_flit_t f, input int unsigned ep);
    int unsigned t0;
    rsp_lock.acquire();
    t0 = cyc;
    while ((rsp_cr_in - rsp_cr_out) <= 0 && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
    if (ep == sb.epoch && (rsp_cr_in - rsp_cr_out) > 0) begin
      rsp_cr_out++;
      rsp_tx.flit     <= #1 f;
      rsp_tx.flitv    <= #1 Y;
      rsp_tx.flitpend <= #1 Y;
      @(posedge clk);
      rsp_tx.flitv    <= #1 N;
      rsp_tx.flitpend <= #1 N;
    end else if (ep == sb.epoch) begin
      sb.err("LINK_CREDIT", $sformatf("%s: HN-F never granted an RSP credit (cannot send %s txnid=%0d)", NAME, f.opcode.name(), f.txnid));
    end
    rsp_lock.unlock();
  endtask

  task automatic send_dat_beats(input logic [11:0] txnid, input node_id_e tgt, input logic [2:0] resp,
                                input logic [1:0] rerr, input line_t data, input addr_t a, input width_e sz,
                                input int unsigned ep);
    local_dat_flit_t f;
    int n, ord [4];
    int unsigned did, t0, j, tmp;
    n = num_beats_for(sz);
    for (int i = 0; i < 4; i++) ord[i] = i;
    if (sb.dat_shuffle) begin
      for (int i = n - 1; i > 0; i--) begin
        j = rng.range(0, i);
        tmp = ord[i]; ord[i] = ord[j]; ord[j] = tmp;
      end
    end
    if (!sb.dat_interleave) dat_lock.acquire();
    for (int i = 0; i < n; i++) begin
      did = first_dataid(a, sz) + ord[i];
      f = '0;
      f.tgtid = tgt; f.srcid = NODEID; f.txnid = txnid; f.homenid = NODEID;
      f.opcode = DAT_COMP_DATA; f.resp = resp; f.resperr = rerr;
      f.dataid = did[1:0]; f.data = data[did*128 +: 128]; f.be = 16'hFFFF;
      if (sb.dat_interleave) dat_lock.acquire();
      t0 = cyc;
      while ((dat_cr_in - dat_cr_out) <= 0 && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
      if (ep == sb.epoch && (dat_cr_in - dat_cr_out) > 0) begin
        dat_cr_out++;
        dat_tx.flit     <= #1 f;
        dat_tx.flitv    <= #1 Y;
        dat_tx.flitpend <= #1 (i == n - 1) ? N : Y;
        @(posedge clk);
        dat_tx.flitv    <= #1 N;
        dat_tx.flitpend <= #1 N;
      end
      if (sb.dat_interleave) dat_lock.unlock();
      if (sb.bubble_pct != 0 && rng.chance(sb.bubble_pct)) repeat (rng.range(1, 3)) @(posedge clk);
    end
    if (!sb.dat_interleave) dat_lock.unlock();
  endtask

  function automatic local_rsp_flit_t mk_rsp(local_req_flit_t rq, rsp_opcode_e op, logic [11:0] dbid, logic [1:0] rerr);
    local_rsp_flit_t f;
    f = '0;
    f.tgtid = rq.srcid; f.srcid = NODEID; f.txnid = rq.txnid; f.opcode = op;
    f.dbid = dbid; f.resperr = rerr;
    return f;
  endfunction

  function automatic int unsigned lat();
    int unsigned l;
    l = rng.range(0, sb.snf_lat_max);
    if (sb.snf_slow_pct != 0 && rng.chance(sb.snf_slow_pct)) l = l + 8 * sb.snf_lat_max + 20;
    return l;
  endfunction

  // ---------------- per-request handlers ----------------
  task automatic handle_read(local_req_flit_t rq, int unsigned ep);
    bit err;
    line_t d;
    err = sb.is_err_line(rq.addr) || (sb.snf_err_pct != 0 && rng.chance(sb.snf_err_pct));
    repeat (lat()) @(posedge clk);
    if (ep == sb.epoch) begin
      d = sb.snf_line(rq.addr);
      if (err) begin sb.n_err_inj++; sb.mark_unknown(rq.addr); end
      send_dat_beats(rq.txnid, rq.srcid, R_I, err ? ERR_DERR : ERR_OK, d, rq.addr, rq.size, ep);
    end
  endtask

  task automatic handle_write(local_req_flit_t rq, int unsigned ep);
    wr_ctx c;
    logic [11:0] dbid;
    bit combined, err, ok, comp_sent;
    int unsigned t0;
    local_rsp_flit_t f;
    err = sb.is_err_line(rq.addr) || (sb.snf_err_pct != 0 && rng.chance(sb.snf_err_pct));
    ok = alloc_dbid(dbid);
    if (!ok) begin
      sb.err("PROTOCOL", $sformatf("%s: out of DBIDs", NAME));
    end else begin
      c = new();
      c.txnid = rq.txnid; c.addr = rq.addr; c.size = rq.size; c.nbeats = num_beats_for(rq.size);
      c.data = '0; c.mask = '0; c.got = 0; c.seen = '0; c.done = 0; c.dbid_sent = 0;
      wr_by_dbid[dbid] = c;
      repeat (lat()) @(posedge clk);
      combined  = rng.chance(60);
      comp_sent = 0;
      // separate Comp may come before DBIDResp, or right after it before any
      // write data: both are legal, Comp and DBIDResp are not ordered
      if (!combined && sb.snf_comp_first_pct != 0 && rng.chance(sb.snf_comp_first_pct)) begin
        f = mk_rsp(rq, RSP_COMP, 12'h0, err ? ERR_DERR : ERR_OK);
        if (ep == sb.epoch) send_rsp_flit(f, ep);
        comp_sent = 1;
        sb.cov_hit("snf: Comp before DBIDResp");
      end
      c.dbid_sent = 1;
      f = mk_rsp(rq, combined ? RSP_COMP_DBID_RESP : RSP_DBID_RESP, dbid, (combined && err) ? ERR_DERR : ERR_OK);
      if (ep == sb.epoch) send_rsp_flit(f, ep);
      if (!combined && !comp_sent && sb.snf_early_comp_pct != 0 && rng.chance(sb.snf_early_comp_pct)) begin
        f = mk_rsp(rq, RSP_COMP, 12'h0, err ? ERR_DERR : ERR_OK);
        if (ep == sb.epoch) send_rsp_flit(f, ep);
        comp_sent = 1;
        sb.cov_hit("snf: Comp after DBIDResp, before write data");
      end
      t0 = cyc;
      while (!c.done && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
      if (ep == sb.epoch) begin
        if (!c.done) begin
          sb.report_timeout(NAME, rq.txnid, $sformatf("write data for dbid=%0d incomplete (%0d/%0d beats)", dbid, c.got, c.nbeats));
        end else begin
          if (rq.opcode == REQ_WRITE_NO_SNOOP_FULL && c.mask != '1)
            sb.err("PROTOCOL", $sformatf("%s: WriteNoSnpFull txnid=%0d addr=%0h arrived with byte enables %016h (must be all ones)", NAME, rq.txnid, rq.addr, c.mask));
          if (err) begin sb.n_err_inj++; sb.mark_unknown(rq.addr); end
          else     sb.snf_write(rq.addr, c.data, c.mask);
          if (!combined && !comp_sent) begin
            repeat (rng.range(0, 3)) @(posedge clk);
            f = mk_rsp(rq, RSP_COMP, 12'h0, err ? ERR_DERR : ERR_OK);
            send_rsp_flit(f, ep);
          end
          wr_by_dbid.delete(dbid);
          dbid_used.delete(dbid);
        end
      end
    end
  endtask

  task automatic retry_flow(local_req_flit_t rq, int unsigned ep);
    local_rsp_flit_t f;
    logic [3:0] pc;
    int unsigned t0;
    local_rsp_flit_t g;
    bit early;
    pc = 4'(rng.range(0, (sb.snf_pcrd_types > 16 ? 16 : (sb.snf_pcrd_types == 0 ? 1 : sb.snf_pcrd_types)) - 1));
    if (pc != 0) sb.cov_hit("snf: PCrdType != 0");
    retried[rq.txnid] = 1;
    retried_rq[rq.txnid] = rq;
    f = mk_rsp(rq, RSP_RETRY_ACK, 12'h0, ERR_OK);
    f.pcrdttype = pc;
    g = '0;
    g.tgtid = rq.srcid; g.srcid = NODEID; g.opcode = RSP_PCR_GRANT_ACK; g.pcrdttype = pc;
    // PCrdGrant may overtake its RetryAck (it carries no TxnID)
    early = sb.snf_pcrd_early_pct != 0 && rng.chance(sb.snf_pcrd_early_pct);
    sb.cov_hit("snf: RetryAck");
    if (early) begin
      sb.cov_hit("snf: PCrdGrant before RetryAck");
      if (ep == sb.epoch) sb.pcrd_out[pc]++;
      send_rsp_flit(g, ep);
      repeat (rng.range(0, 6)) @(posedge clk);
      send_rsp_flit(f, ep);
    end else begin
      send_rsp_flit(f, ep);
      repeat (rng.range(2, 40)) @(posedge clk);
      if (ep == sb.epoch) sb.pcrd_out[pc]++;
      send_rsp_flit(g, ep);
    end
    t0 = cyc;
    while (!(resent.exists(rq.txnid) && resent[rq.txnid]) && ep == sb.epoch && (cyc - t0) < sb.op_timeout) @(posedge clk);
    if (ep == sb.epoch && !(resent.exists(rq.txnid) && resent[rq.txnid]))
      sb.report_timeout(NAME, rq.txnid, "HN-F never resent the request after PCrdGrant");
    if (ep == sb.epoch) begin retried.delete(rq.txnid); resent.delete(rq.txnid); end
  endtask

  task automatic serve(local_req_flit_t rq, int unsigned ep);
    if (rq.allowretry == Y && sb.snf_retry_pct != 0 && !retried.exists(rq.txnid) && rng.chance(sb.snf_retry_pct)) begin
      retry_flow(rq, ep);
    end else begin
      inflight[rq.txnid] = 1;
      if (rq.opcode == REQ_READ_NO_SNOOP) handle_read(rq, ep);
      else if (rq.opcode == REQ_WRITE_NO_SNOOP_FULL || rq.opcode == REQ_WRITE_NO_SNOOP_PTL) handle_write(rq, ep);
      if (ep == sb.epoch) inflight.delete(rq.txnid);
    end
  endtask

  initial begin
    local_req_flit_t rq;
    while (sb == null) @(posedge clk);
    forever begin
      while (req_q.size() == 0) @(posedge clk);
      rq = req_q.pop_front();
      fork
        begin
          automatic local_req_flit_t r2 = rq;
          automatic int unsigned ep = my_epoch;
          serve(r2, ep);
        end
      join_none
      @(posedge clk);
    end
  end

endmodule