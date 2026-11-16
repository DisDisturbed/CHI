`timescale 1ns/1ps
// =============================================================================
// rnf_chi_checker - passive protocol checker on the CHI ports of one RN-F
// (the RTL dcache) and on its core interface.
//
// It looks only at the pins. From the traffic it keeps its own view of every
// line the RN-F holds (I / SC / U = UC or UD / SD, plus "surely dirty") and
// checks against it:
//
//  requests   fields per opcode, TxnID not in use, AllowRetry, legal start
//             state (ReadShared only without a copy, ReadUnique not on a
//             Unique line, Evict / WriteBackFull only for a line it holds,
//             Evict never for a dirty line)
//  reads      CompAck exactly once, after the last CompData beat, with
//             TxnID = DBID and TgtID = HomeNID; none for other requests
//  writes     write data only after the DBID, TxnID = DBID, right opcode,
//             DataIDs once each, one Resp on all beats, byte enables; the
//             CopyBackWrData Resp fits the line's state; the data is a legal
//             value (scoreboard)
//  snoops     answered exactly once, either SnpResp or four SnpRespData beats,
//             to the right node and TxnID, within SNP_RSP_MAX cycles; the
//             response is legal for the snoop type and for the state the line
//             had; dirty data is never dropped; returned data is a legal value
//  core       requests are tracked by idx (any number in flight, answered in
//             any order): a response must answer an accepted request with
//             that idx, an idx is not reused before its response, a cacheable
//             store may only complete on a line held Unique
//  link       no flit without a credit, FLITV only with FLITPEND
//
// Coverage (keys "bb core: ..."), from the pins only: core op x the line's
// state on the bus at accept, hit / miss (a line read for that line between
// accept and response), victims written back / evicted while a core miss
// was open, snoops on the line of an open core request.
//
// Failures go to the scoreboard as PROTOCOL / LINK_CREDIT / FLITPEND.
// open_txn / open_snp / open_core are what is still in flight (0 at the end of a test).
// =============================================================================
module rnf_chi_checker #(
  parameter chi_pkg::node_id_e NODEID      = 7'h01,
  parameter chi_pkg::node_id_e HNID        = 7'h40,
  parameter string             NAME        = "RNF0",
  parameter int                SNP_RSP_MAX = 200
) (
  input wire  clk,
  input wire  resetn,
  rnf_core_if core,
  chi_req     req,       // RN-F -> HN-F
  chi_snp     snp,       // HN-F -> RN-F
  chi_rsp     rsp_rx,    // HN-F -> RN-F
  chi_rsp     rsp_tx,    // RN-F -> HN-F
  chi_dat     dat_rx,    // HN-F -> RN-F
  chi_dat     dat_tx     // RN-F -> HN-F
);
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;
  import rnf_pkg::*;
  import chi_tb_pkg::*;

  typedef enum int { L_I, L_SC, L_U, L_SD } lst_e;

  class txn_c;
    local_req_flit_t f;
    addr_t       la;
    bit          is_rd, is_wr, is_nc;
    // read
    bit [3:0]    seen, need;
    logic [2:0]  cresp;
    logic [11:0] cdbid;
    node_id_e    chome;
    bit          cerr, await_ack;
    // write
    bit          got_dbid, got_comp, data_done;
    bit          amb;            // a snoop of this line was answered while the write-back had its DBID
    rd_rec       rr;             // values the line may hold from the DBID on
    logic [11:0] dbid;
    bit [3:0]    wseen;
    logic [2:0]  wresp;
    line_t       wdata;
    bemask_t     wbe;
  endclass

  class snp_c;
    local_snp_flit_t f;
    addr_t       la;
    int unsigned t0;
    bit [3:0]    seen;
    logic [2:0]  resp;
    line_t       data;
    bit          wb_crossed;     // a write-back / Evict of this line went out while the snoop was open
    rd_rec       rr;             // values the line may hold while the snoop is open
    bit          late;
  endclass

  // one core request, from accept to response
  class creq_c;
    bit          store, cach;
    addr_t       la;
    int          nbytes;
    lst_e        st0;
    bit          saw_rd, saw_wb, saw_ev, saw_snp;
  endclass

  lst_e  lst    [addr_t];
  bit    ldirty [addr_t];        // the line is dirty for sure
  bit    held   [addr_t];        // filled and not given up yet (Evict / WriteBack allowed)
  int    ack_wait [addr_t];      // CompData received, CompAck not sent yet
  txn_c  tx      [logic [11:0]];
  txn_c  by_dbid [logic [11:0]];
  snp_c  sn      [logic [11:0]];

  int unsigned cyc = 0;
  int          open_txn = 0, open_snp = 0, open_core = 0;
  int          req_cr = 0, rsp_cr = 0, dat_cr = 0;

  // core requests accepted and not answered yet, by idx
  creq_c creq [int];

  function automatic string s_op(bit st);
    if (st) return "store";
    return "load";
  endfunction

  function automatic string lst_str(lst_e s);
    case (s)
      L_SC:    return "Shared Clean";
      L_U:     return "Unique";
      L_SD:    return "Shared Dirty";
      default: return "not held";
    endcase
  endfunction

  function automatic lst_e st_of(addr_t la);
    return lst.exists(la) ? lst[la] : L_I;
  endfunction

  function automatic bit dirty_of(addr_t la);
    return ldirty.exists(la) && ldirty[la];
  endfunction

  function automatic void perr(string msg);
    sb.err("PROTOCOL", {NAME, " checker: ", msg});
  endfunction

  function automatic bit snp_open_on(addr_t la);
    foreach (sn[k]) if (sn[k].la == la) return 1;
    return 0;
  endfunction

  // a WriteBackFull of this line already has its DBID: the line may have left the cache
  function automatic bit wb_started(addr_t la);
    foreach (tx[k]) if (tx[k].f.opcode == REQ_WRITE_BACK_FULL && tx[k].la == la && tx[k].got_dbid) return 1;
    return 0;
  endfunction

  function automatic void txn_done(txn_c t);
    if (t.got_dbid && by_dbid.exists(t.dbid) && by_dbid[t.dbid] == t) by_dbid.delete(t.dbid);
    if (tx.exists(t.f.txnid) && tx[t.f.txnid] == t) tx.delete(t.f.txnid);
  endfunction

  function automatic void wr_maybe_done(txn_c t);
    if (t.data_done && t.got_comp) txn_done(t);
  endfunction

  // ---------------------------------------------------------------- request
  function automatic void on_req(local_req_flit_t f);
    txn_c  t;
    addr_t la;
    lst_e  st;
    bit    coh;
    la = line_of(f.addr);
    st = st_of(la);
    if (f.srcid !== NODEID || f.tgtid !== HNID)
      perr($sformatf("%s txnid=%0d has SrcID=%0h TgtID=%0h (expected %0h/%0h)", f.opcode.name(), f.txnid, f.srcid, f.tgtid, NODEID, HNID));
    if (tx.exists(f.txnid))
      perr($sformatf("%s reuses TxnID %0d while %s with that TxnID is still in flight", f.opcode.name(), f.txnid, tx[f.txnid].f.opcode.name()));
    if (f.allowretry != Y)
      perr($sformatf("%s txnid=%0d sent with AllowRetry=0 although it never got a RetryAck", f.opcode.name(), f.txnid));
    coh = 1;
    case (f.opcode)
      REQ_READ_SHARED: begin
        if (st != L_I && !snp_open_on(la)) perr($sformatf("ReadShared addr=%0h for a line it already holds (%s)", la, st.name()));
        if (!f.expcompack) perr("ReadShared without ExpCompAck");
      end
      REQ_READ_UNIQUE: begin
        if (st == L_U && !snp_open_on(la)) perr($sformatf("ReadUnique addr=%0h for a line it already holds Unique", la));
        if (!f.expcompack) perr("ReadUnique without ExpCompAck");
      end
      REQ_EVICT: begin
        if (!(held.exists(la) && held[la])) perr($sformatf("Evict addr=%0h for a line it never got", la));
        if (st != L_I && dirty_of(la) && !snp_open_on(la)) perr($sformatf("Evict addr=%0h for a dirty line: the data is lost", la));
        if (f.expcompack) perr("Evict with ExpCompAck");
        foreach (sn[k]) if (sn[k].la == la) sn[k].wb_crossed = 1;   // its response may still be on the way
        lst[la] = L_I; ldirty[la] = 0; held[la] = 0;
      end
      REQ_WRITE_BACK_FULL: begin
        if (!(held.exists(la) && held[la])) perr($sformatf("WriteBackFull addr=%0h for a line it never got", la));
        if (f.expcompack) perr("WriteBackFull with ExpCompAck");
      end
      REQ_READ_NO_SNOOP, REQ_WRITE_NO_SNOOP_PTL: begin
        coh = 0;
        if (f.expcompack) perr($sformatf("%s with ExpCompAck", f.opcode.name()));
        if (size_bytes(f.size) > 16) perr($sformatf("%s size=%0dB: more than one beat", f.opcode.name(), size_bytes(f.size)));
        if ((f.addr & addr_t'(size_bytes(f.size) - 1)) != '0) perr($sformatf("%s addr=%0h is not aligned to size=%0dB", f.opcode.name(), f.addr, size_bytes(f.size)));
      end
      default: perr($sformatf("request %s (opcode %0h) is not one this RN-F is meant to send", f.opcode.name(), f.opcode));
    endcase
    if (coh) begin
      if (f.size != WIDTH_64)           perr($sformatf("%s size=%0dB, a line request must be 64 bytes", f.opcode.name(), size_bytes(f.size)));
      if (f.memattr != MEMATTR_NORMAL_WB) perr($sformatf("%s MemAttr=%04b (expected %04b)", f.opcode.name(), f.memattr, MEMATTR_NORMAL_WB));
    end else if (f.memattr != MEMATTR_NORMAL_NC) begin
      perr($sformatf("%s MemAttr=%04b (expected %04b)", f.opcode.name(), f.memattr, MEMATTR_NORMAL_NC));
    end
    // what the open core requests caused
    foreach (creq[k]) if (creq[k].cach) begin
      if ((f.opcode == REQ_READ_SHARED || f.opcode == REQ_READ_UNIQUE) && creq[k].la == la) creq[k].saw_rd = 1;
      if (f.opcode == REQ_WRITE_BACK_FULL && creq[k].la != la) creq[k].saw_wb = 1;
      if (f.opcode == REQ_EVICT           && creq[k].la != la) creq[k].saw_ev = 1;
    end
    t = new();
    t.f = f; t.la = la;
    t.is_rd = (f.opcode == REQ_READ_SHARED || f.opcode == REQ_READ_UNIQUE || f.opcode == REQ_READ_NO_SNOOP);
    t.is_wr = (f.opcode == REQ_WRITE_BACK_FULL || f.opcode == REQ_WRITE_NO_SNOOP_PTL);
    t.is_nc = !coh;
    t.need  = coh ? 4'hF : (4'b0001 << f.addr[5:4]);
    tx[f.txnid] = t;
  endfunction

  // ---------------------------------------------------------------- HN-F -> RN-F
  function automatic void on_rsp_rx(local_rsp_flit_t f);
    txn_c t;
    if (f.opcode == RSP_PCR_GRANT_ACK) return;
    if (!tx.exists(f.txnid)) return;          // the HN-F side is checked by the other agents
    t = tx[f.txnid];
    case (f.opcode)
      RSP_COMP: begin
        t.got_comp = 1;
        if (t.f.opcode == REQ_EVICT) txn_done(t);
        else if (t.is_wr)            wr_maybe_done(t);
      end
      RSP_COMP_DBID_RESP, RSP_DBID_RESP: begin
        if (f.opcode == RSP_COMP_DBID_RESP) t.got_comp = 1;
        t.got_dbid = 1; t.dbid = f.dbid;
        by_dbid[f.dbid] = t;
        if (t.f.opcode == REQ_WRITE_BACK_FULL && t.rr == null) t.rr = sb.rd_begin(t.la, NAME);
      end
      default: ;
    endcase
  endfunction

  function automatic void on_dat_rx(local_dat_flit_t f);
    txn_c t;
    if (f.opcode != DAT_COMP_DATA || !tx.exists(f.txnid)) return;
    t = tx[f.txnid];
    if (!t.is_rd) return;
    if (t.seen == '0) begin
      t.cresp = f.resp; t.cdbid = f.dbid; t.chome = f.homenid;
      if (!t.is_nc) ack_wait[t.la] = ack_wait.exists(t.la) ? ack_wait[t.la] + 1 : 1;
    end
    if (f.resperr[1]) t.cerr = 1;
    t.seen[f.dataid] = 1;
    if ((t.seen & t.need) == t.need) begin
      if (t.f.expcompack) t.await_ack = 1;
      else                txn_done(t);
    end
  endfunction

  function automatic void on_snp(local_snp_flit_t f);
    snp_c  s;
    addr_t la;
    la = line_of(f.addr);
    if (sn.exists(f.txnid)) begin
      perr($sformatf("snoop TxnID %0d is used again by the HN-F before the first one was answered", f.txnid));
      return;
    end
    if (ack_wait.exists(la) && ack_wait[la] > 0)
      sb.err("SNOOP_ORDER", $sformatf("%s checker: snoop %s addr=%0h arrived after CompData and before the CompAck for that line", NAME, f.opcode.name(), la));
    foreach (creq[k]) if (creq[k].la == la) creq[k].saw_snp = 1;
    s = new();
    s.f = f; s.la = la; s.t0 = cyc;
    s.rr = sb.rd_begin(la, NAME);
    sn[f.txnid] = s;
  endfunction

  // ---------------------------------------------------------------- RN-F -> HN-F
  function automatic void on_ack(local_rsp_flit_t f);
    txn_c t;
    t = null;
    foreach (tx[k]) if (tx[k].await_ack && tx[k].cdbid == f.txnid) t = tx[k];
    if (t == null) begin
      perr($sformatf("CompAck txnid=%0d does not belong to a read whose CompData is complete", f.txnid));
      return;
    end
    if (f.tgtid !== t.chome || f.srcid !== NODEID)
      perr($sformatf("CompAck txnid=%0d has TgtID=%0h SrcID=%0h (expected HomeNID %0h / %0h)", f.txnid, f.tgtid, f.srcid, t.chome, NODEID));
    if (ack_wait.exists(t.la)) begin
      ack_wait[t.la]--;
      if (ack_wait[t.la] <= 0) ack_wait.delete(t.la);
    end
    if (!t.cerr) begin                           // the line is held from here on
      held[t.la] = 1;
      if (t.f.opcode == REQ_READ_UNIQUE) begin
        if (t.cresp != RESP_UC && t.cresp != RESP_UD_PD) perr($sformatf("HN-F answered ReadUnique with Resp=%03b", t.cresp));
        // a Shared Dirty copy that was kept stays dirty
        ldirty[t.la] = (t.cresp == RESP_UD_PD) || (st_of(t.la) == L_SD);
        lst[t.la]    = L_U;
      end else begin
        case (t.cresp)
          RESP_UC:    begin lst[t.la] = L_U;  ldirty[t.la] = 0; end
          RESP_UD_PD: begin lst[t.la] = L_U;  ldirty[t.la] = 1; end
          RESP_SD_PD: begin lst[t.la] = L_SD; ldirty[t.la] = 1; end
          default:    begin lst[t.la] = L_SC; ldirty[t.la] = 0; end
        endcase
      end
    end
    txn_done(t);
  endfunction

  // legal for the snoop type and for the state the line had?
  function automatic void eval_snoop(snp_c s, logic [2:0] resp, bit has_data);
    snp_opcode_e op;
    lst_e        st;
    bit          skip, dirty;
    string       what;
    op    = s.f.opcode;
    st    = st_of(s.la);
    dirty = dirty_of(s.la);
    skip  = s.wb_crossed || wb_started(s.la);
    what  = $sformatf("%s addr=%0h answered %s Resp=%03b, line was %s%s", op.name(), s.la, has_data ? "SnpRespData" : "SnpResp", resp,
                      st.name(), dirty ? " (dirty)" : "");
    if ((cyc - s.t0) > SNP_RSP_MAX && !s.late)
      perr($sformatf("%s addr=%0h was answered after %0d cycles (limit %0d)", op.name(), s.la, cyc - s.t0, SNP_RSP_MAX));

    // by snoop type
    case (op)
      SNP_UNIQUE, SNP_CLEAN_INVALID:
        if (resp[1:0] != 2'b00) perr({what, ": the line must end Invalid"});
      SNP_MAKE_INVALID:
        if (resp != RESP_I || has_data) perr({what, ": must be SnpResp_I"});
      SNP_SHARED, SNP_CLEAN, SNP_NOT_SHARED_DIRTY: begin
        if (resp[1:0] == 2'b10) perr({what, ": the line must not stay Unique"});
        if (resp[1:0] == 2'b11 && (op != SNP_SHARED || s.f.donotgotosd)) perr({what, ": the line must not stay Shared Dirty"});
      end
      SNP_CLEAN_SHARED:
        if (resp[1:0] == 2'b11) perr({what, ": the line must not stay dirty"});
      SNP_ONCE: ;
      default: perr($sformatf("snoop %s is not one the HN-F is meant to send", op.name()));
    endcase

    // by the state the line had
    if (!skip) begin
      case (st)
        L_I:  if (resp != RESP_I || has_data) perr({what, ": it has no copy, only SnpResp_I is legal"});
        L_SC: if (resp[2] || resp[1:0] == 2'b10 || resp[1:0] == 2'b11) perr({what, ": a Shared Clean line can only end I or SC, without PassDirty"});
        default: begin
          if (resp[2] && !has_data) perr({what, ": PassDirty without data"});
          if ((dirty || st == L_SD) && op != SNP_MAKE_INVALID) begin
            if (!has_data) perr({what, ": the dirty data was not returned"});
            else if (!resp[2] && (resp[1:0] == 2'b00 || resp[1:0] == 2'b01)) perr({what, ": dirty data given up without PassDirty"});
          end
          if (st == L_SD && resp[1:0] == 2'b10) perr({what, ": a Shared line cannot end Unique"});
        end
      endcase
    end
    // returned data: a value the line could hold while the snoop was open
    if (has_data && st != L_I && !skip) sb.rd_end(s.rr, s.data, 0, {op.name(), " snoop data (checker)"});
    else                                sb.rd_end(s.rr, '0, 1, "snoop without data");

    // new view of the line
    if (st != L_I) begin
      case (resp[1:0])
        2'b00:   begin lst[s.la] = L_I;  ldirty[s.la] = 0; end
        2'b01:   begin lst[s.la] = L_SC; ldirty[s.la] = 0; end
        2'b11:   begin lst[s.la] = L_SD; end
        default: begin lst[s.la] = L_U;  if (resp[2]) ldirty[s.la] = 0; end
      endcase
    end

    // answered while the write-back was under way: before or after the line was copied out is not visible here
    foreach (tx[k]) if (tx[k].f.opcode == REQ_WRITE_BACK_FULL && tx[k].la == s.la && tx[k].got_dbid) tx[k].amb = 1;

    if (tx.size() != 0) begin
      sb.cov_hit("chk: snoop answered while a request is outstanding");
      foreach (tx[k]) if (tx[k].la == s.la) sb.cov_hit("chk: snoop answered for the line of the outstanding request");
    end
  endfunction

  function automatic void on_snp_resp(local_rsp_flit_t f);
    snp_c s;
    if (!sn.exists(f.txnid)) begin
      perr($sformatf("SnpResp txnid=%0d does not answer any open snoop", f.txnid));
      return;
    end
    s = sn[f.txnid];
    if (s.seen != '0) perr($sformatf("snoop txnid=%0d answered with SnpRespData beats and with SnpResp", f.txnid));
    if (f.tgtid !== s.f.srcid || f.srcid !== NODEID)
      perr($sformatf("SnpResp txnid=%0d has TgtID=%0h SrcID=%0h (expected %0h/%0h)", f.txnid, f.tgtid, f.srcid, s.f.srcid, NODEID));
    eval_snoop(s, f.resp, 0);
    sn.delete(f.txnid);
  endfunction

  function automatic void on_snp_data(local_dat_flit_t f);
    snp_c s;
    if (!sn.exists(f.txnid)) begin
      perr($sformatf("SnpRespData txnid=%0d dataid=%0d does not answer any open snoop", f.txnid, f.dataid));
      return;
    end
    s = sn[f.txnid];
    if (f.tgtid !== s.f.srcid || f.srcid !== NODEID)
      perr($sformatf("SnpRespData txnid=%0d has TgtID=%0h SrcID=%0h (expected %0h/%0h)", f.txnid, f.tgtid, f.srcid, s.f.srcid, NODEID));
    if (s.seen[f.dataid]) perr($sformatf("SnpRespData txnid=%0d: DataID %0d sent twice", f.txnid, f.dataid));
    if (s.seen != '0 && f.resp !== s.resp) perr($sformatf("SnpRespData txnid=%0d: beats carry different Resp (%03b / %03b)", f.txnid, s.resp, f.resp));
    if (f.be !== '1) perr($sformatf("SnpRespData txnid=%0d dataid=%0d: BE=%04h, a full line is expected", f.txnid, f.dataid, f.be));
    s.resp = f.resp;
    s.seen[f.dataid] = 1;
    s.data[f.dataid*128 +: 128] = f.data;
    if (s.seen == 4'hF) begin
      eval_snoop(s, s.resp, 1);
      sn.delete(f.txnid);
    end
  endfunction

  function automatic void on_wr_data(local_dat_flit_t f);
    txn_c t;
    bit   cb;
    lst_e st;
    if (!by_dbid.exists(f.txnid)) begin
      perr($sformatf("%s txnid=%0d does not match the DBID of any write that got one", f.opcode.name(), f.txnid));
      return;
    end
    t  = by_dbid[f.txnid];
    cb = (t.f.opcode == REQ_WRITE_BACK_FULL);
    if (t.data_done) perr($sformatf("%s: more write data after the last beat (dbid=%0d)", t.f.opcode.name(), f.txnid));
    if ((f.opcode == DAT_COPY_BACK_WR_DATA) != cb)
      perr($sformatf("%s answered with %s", t.f.opcode.name(), f.opcode.name()));
    if (f.tgtid !== HNID || f.srcid !== NODEID)
      perr($sformatf("%s dbid=%0d has TgtID=%0h SrcID=%0h (expected %0h/%0h)", f.opcode.name(), f.txnid, f.tgtid, f.srcid, HNID, NODEID));
    if (t.wseen[f.dataid]) perr($sformatf("%s dbid=%0d: DataID %0d sent twice", f.opcode.name(), f.txnid, f.dataid));
    if (t.wseen != '0 && f.resp !== t.wresp) perr($sformatf("%s dbid=%0d: beats carry different Resp", f.opcode.name(), f.txnid));
    t.wresp = f.resp;
    t.wseen[f.dataid] = 1;
    t.wdata[f.dataid*128 +: 128] = f.data;
    t.wbe[f.dataid*16 +: 16]     = f.be;
    if (!cb) begin
      // one beat, byte enables inside the Size window
      bemask_t span;
      span = span_mask(t.f.addr, t.f.size);
      if (f.dataid != t.f.addr[5:4]) perr($sformatf("WriteNoSnp data: DataID %0d, the address is in beat %0d", f.dataid, t.f.addr[5:4]));
      if (f.resp != RESP_I)           perr($sformatf("WriteNoSnp data: Resp=%03b", f.resp));
      if ((t.wbe & ~span) != '0)      perr($sformatf("WriteNoSnp data: byte enables %016h outside the %0d-byte window at %0h", t.wbe, size_bytes(t.f.size), t.f.addr));
      t.data_done = 1;
    end else if (t.wseen == 4'hF) begin
      t.data_done = 1;
      st = st_of(t.la);
      if (f.resp == RESP_I) begin
        if (t.wbe != '0) perr("CopyBackWrData_I with byte enables set");
        sb.rd_end(t.rr, '0, 1, "write-back without data");
      end else begin
        if (t.wbe != '1) perr($sformatf("CopyBackWrData Resp=%03b without every byte enable", f.resp));
        sb.rd_end(t.rr, t.wdata, 0, "WriteBackFull data (checker)");
      end
      // the Resp must be the state the line had, unless a snoop of this line is still open
      if (!snp_open_on(t.la) && !t.amb) begin
        case (st)
          L_I:  if (f.resp != RESP_I)                         perr($sformatf("WriteBackFull addr=%0h: Resp=%03b but the line was given up to a snoop", t.la, f.resp));
          L_SC: if (f.resp != RESP_SC)                        perr($sformatf("WriteBackFull addr=%0h: Resp=%03b for a Shared Clean line", t.la, f.resp));
          L_SD: if (f.resp != RESP_SD_PD)                     perr($sformatf("WriteBackFull addr=%0h: Resp=%03b for a Shared Dirty line", t.la, f.resp));
          default: begin
            if (f.resp != RESP_UC && f.resp != RESP_UD_PD)    perr($sformatf("WriteBackFull addr=%0h: Resp=%03b for a Unique line", t.la, f.resp));
            if (dirty_of(t.la) && f.resp != RESP_UD_PD)       perr($sformatf("WriteBackFull addr=%0h: Resp=%03b for a dirty line", t.la, f.resp));
          end
        endcase
      end
      foreach (sn[k]) if (sn[k].la == t.la) sn[k].wb_crossed = 1;
      lst[t.la] = L_I; ldirty[t.la] = 0; held[t.la] = 0;
    end
    wr_maybe_done(t);
  endfunction

  // ---------------------------------------------------------------- sampling
  always @(posedge clk) begin
    cyc <= cyc + 1;
    if (!resetn) begin
      lst.delete(); ldirty.delete(); held.delete(); ack_wait.delete();
      tx.delete(); by_dbid.delete(); sn.delete();
      req_cr = 0; rsp_cr = 0; dat_cr = 0;
      creq.delete();
    end else if (sb != null) begin
      // link layer of the three RN-F TX channels
      if (req.flitv == Y) begin
        if (req_cr <= 0) sb.err("LINK_CREDIT", {NAME, " checker: REQ flit without a credit"});
        req_cr--;
        if (req.flitpend != Y) sb.err("FLITPEND", {NAME, " checker: REQ FLITV=1 with FLITPEND=0"});
      end
      if (rsp_tx.flitv == Y) begin
        if (rsp_cr <= 0) sb.err("LINK_CREDIT", {NAME, " checker: RSP flit without a credit"});
        rsp_cr--;
        if (rsp_tx.flitpend != Y) sb.err("FLITPEND", {NAME, " checker: RSP FLITV=1 with FLITPEND=0"});
      end
      if (dat_tx.flitv == Y) begin
        if (dat_cr <= 0) sb.err("LINK_CREDIT", {NAME, " checker: DAT flit without a credit"});
        dat_cr--;
        if (dat_tx.flitpend != Y) sb.err("FLITPEND", {NAME, " checker: DAT FLITV=1 with FLITPEND=0"});
      end
      if (req.lcrdv    === 1'b1) req_cr++;
      if (rsp_tx.lcrdv === 1'b1) rsp_cr++;
      if (dat_tx.lcrdv === 1'b1) dat_cr++;
      if (req_cr > 15 || rsp_cr > 15 || dat_cr > 15)
        sb.err("LINK_CREDIT", {NAME, " checker: the HN-F granted more than 15 credits on a channel"});

      // HN-F -> RN-F first, then what the RN-F sent, then the core
      if (rsp_rx.flitv == Y) on_rsp_rx(local_rsp_flit_t'(rsp_rx.flit));
      if (dat_rx.flitv == Y) on_dat_rx(local_dat_flit_t'(dat_rx.flit));
      if (snp.flitv    == Y) on_snp(local_snp_flit_t'(snp.flit));

      if (req.flitv == Y) on_req(local_req_flit_t'(req.flit));
      if (rsp_tx.flitv == Y) begin
        local_rsp_flit_t f;
        f = local_rsp_flit_t'(rsp_tx.flit);
        case (f.opcode)
          RSP_COMP_ACK: on_ack(f);
          RSP_SNP_RESP: on_snp_resp(f);
          default:      perr($sformatf("RSP opcode %s is not one an RN-F sends", f.opcode.name()));
        endcase
      end
      if (dat_tx.flitv == Y) begin
        local_dat_flit_t f;
        f = local_dat_flit_t'(dat_tx.flit);
        case (f.opcode)
          DAT_SNP_RESP_DATA:                                on_snp_data(f);
          DAT_COPY_BACK_WR_DATA, DAT_NON_COPY_BACK_WR_DATA: on_wr_data(f);
          default: perr($sformatf("DAT opcode %s is not one this RN-F is meant to send", f.opcode.name()));
        endcase
      end

      // core: responses by idx; a cacheable store completes on a Unique line, and makes it dirty
      if (core.rsp.valid == Y) begin
        int    k;
        creq_c c;
        string opn;
        k = int'(core.rsp.idx);
        if (!creq.exists(k)) perr($sformatf("core response idx=%0d, but no accepted request with that idx is open", k));
        else begin
          c   = creq[k];
          opn = c.store ? "store" : "load";
          if (c.store && c.cach && core.rsp.err != Y && !snp_open_on(c.la)) begin
            if (st_of(c.la) != L_U)
              perr($sformatf("store to addr=%0h completed, but the line is %s on the bus, not Unique", c.la, st_of(c.la).name()));
            else ldirty[c.la] = 1;
          end
          if (c.cach && core.rsp.err != Y) begin
            if (c.saw_rd) sb.cov_hit($sformatf("bb core: %s completed after a line read (miss)", opn));
            else          sb.cov_hit($sformatf("bb core: %s completed without a line read (hit)", opn));
            if (c.saw_rd && c.saw_wb) sb.cov_hit("bb core: miss open while a victim was written back (WriteBackFull)");
            if (c.saw_rd && c.saw_ev) sb.cov_hit("bb core: miss open while a victim was evicted (Evict)");
            if (c.saw_snp)            sb.cov_hit($sformatf("bb core: %s whose line was snooped while it was open", opn));
          end
          creq.delete(k);
        end
      end
      if (core.req.valid == Y && core.ready == Y) begin
        int    k;
        creq_c c;
        k = int'(core.req.idx);
        if (creq.exists(k)) perr($sformatf("core request accepted with idx=%0d while the request with that idx has no response yet", k));
        c = new();
        c.store  = (core.req.op == MEM_OP_STORE);
        c.cach   = (core.req.cacheable == Y);
        c.la     = line_of(addr_t'(core.req.addr));
        c.nbytes = size_bytes(core.req.width);
        c.st0    = st_of(c.la);
        if (c.cach) sb.cov_hit($sformatf("bb core: %s on a line %s", s_op(c.store), lst_str(c.st0)));
        if (c.cach && c.store && c.nbytes < 8) sb.cov_hit($sformatf("bb core: sub-word store (<8B) on a line %s", lst_str(c.st0)));
        foreach (creq[j]) if (creq[j].la == c.la) begin
          sb.cov_hit("bb core: two requests to one line open at once");
          break;
        end
        creq[k] = c;
      end

      // a snoop nobody answers
      foreach (sn[k]) if (!sn[k].late && (cyc - sn[k].t0) > SNP_RSP_MAX) begin
        sn[k].late = 1;
        perr($sformatf("snoop %s addr=%0h txnid=%0d not answered within %0d cycles", sn[k].f.opcode.name(), sn[k].la, k, SNP_RSP_MAX));
      end

      open_txn = tx.size();
      open_snp = sn.size();
      open_core = creq.size();
    end
  end

endmodule
