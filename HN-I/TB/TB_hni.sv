`timescale 1ns/1ps
// =============================================================================
// TB_hni - randomized testbench of the HN-I (HN-I/RTL/hni.sv).
//
//  Requesters: NSRC sources (SrcID 1..NSRC) on the HN-I's one CHI link, as they
//  would arrive through an XP. Each issues OPS random accesses, up to OUT_MAX
//  outstanding: ReadNoSnp / WriteNoSnpPtl (any byte enables, some bytes off) /
//  WriteNoSnpFull, sizes 1..64 B aligned to their size, in its own 4 KB window
//  (so a read has exactly one right answer), and ~10% to two error windows
//  (SLVERR, DECERR).
//  Link layer: credits both ways; the TB grants the DUT's TX credits at random
//  so its RSP / DAT channels stall now and then.
//  AXI slave: byte memory, random AR/AW/W ready, random R / B latency, read
//  beats of different IDs interleaved, B responses in any order.
//
//  Checked: read data (every requested byte), CompData fields / DataIDs / beat
//  count / RespErr, DBIDResp -> write data -> Comp, Comp only after the AXI B of
//  that write, per-source completion in issue order (Device order), link
//  credits (no flit without a credit), AXI rules (INCR, size <= 4 B, aligned,
//  no 4 KB crossing, WLAST on the last beat, WSTRB only on the lanes of a
//  narrow beat), DUT idle at the end. Coverage points must be reached.
//
//  Prints "*** HN-I REGRESSION: ALL CHECKS PASSED (seed=N) ***" (sim/Makefile).
// =============================================================================
module TB_hni;
  import chi_pkg::*;
  import common_pkg::*;
  import tshr_flit_pkg::*;

  parameter int SEED      = 1;
  parameter int NSRC      = 4;
  parameter int OPS       = 1500;      // per source
  parameter int OUT_MAX   = 2;         // outstanding per source
  parameter int NUM_TRK   = 4;
  parameter int REQ_DEPTH = 4;
  parameter int STALL_PCT = 30;        // AXI ready / TB credit throttling
  parameter int LAT_MAX   = 12;        // AXI R / B latency
  parameter int TIMEOUT   = 2_000_000;

  localparam node_id_e HNI = 7'h50;
  localparam int TW  = $clog2(NUM_TRK);
  localparam int IDW = 4 + TW;

  // ------------------------------------------------------------ clock / reset / random
  logic clk = 0, resetn = 0;
  always #5 clk = ~clk;
  int unsigned cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  int unsigned rs;
  function automatic int unsigned rnd();
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
  endfunction
  function automatic int unsigned rr(int unsigned lo, int unsigned hi);   // [lo, hi]
    return lo + (rnd() % (hi - lo + 1));
  endfunction
  function automatic bit chance(int unsigned pct); return (rnd() % 100) < pct; endfunction

  int n_err = 0;
  function automatic void err(string m);
    n_err++;
    if (n_err <= 20) $display("ERROR [%0d] %s", cyc, m);
  endfunction

  int unsigned cov [string];
  function automatic void hit(string k); cov[k] = cov.exists(k) ? cov[k] + 1 : 1; endfunction

  // ------------------------------------------------------------ memories
  // initial value of a byte: a function of its address (AXI slave and reference agree)
  function automatic byte unsigned init_b(logic [31:0] a);
    return byte'(a[7:0] ^ a[15:8] ^ a[23:16] ^ 8'h5A);
  endfunction
  byte unsigned smem [logic [31:0]];          // AXI slave memory
  byte unsigned rmem [logic [31:0]];          // reference (program order per source)
  function automatic byte unsigned sget(logic [31:0] a); return smem.exists(a) ? smem[a] : init_b(a); endfunction
  function automatic byte unsigned rget(logic [31:0] a); return rmem.exists(a) ? rmem[a] : init_b(a); endfunction

  // windows: source s at 0x1000_0000 + s*0x1000; errors at 0xE000_0000 (SLVERR), 0xF000_0000 (DECERR)
  function automatic logic [1:0] region_resp(logic [31:0] a);
    if (a[31:28] == 4'hE) return 2'b10;
    if (a[31:28] == 4'hF) return 2'b11;
    return 2'b00;
  endfunction

  // ------------------------------------------------------------ DUT
  chi_req req_if();
  chi_rsp rsp_if();
  chi_dat wd_if();     // TB -> DUT write data
  chi_dat rd_if();     // DUT -> TB CompData

  logic [IDW-1:0] awid, bid, arid, rid;
  logic [31:0]    awaddr, araddr, wdata, rdata;
  logic [7:0]     awlen, arlen;
  logic [2:0]     awsize, arsize, awprot, arprot;
  logic [1:0]     awburst, arburst, bresp, rresp;
  logic [3:0]     awcache, arcache, wstrb;
  logic           awvalid, awready, wvalid, wready, wlast, bvalid, bready;
  logic           arvalid, arready, rvalid, rready, rlast;

  hni #(.NODEID(HNI), .NUM_TRK(NUM_TRK), .REQ_DEPTH(REQ_DEPTH), .SRC_IDW(4)) dut (
    .clk(clk), .resetn(resetn),
    .req_rx(req_if), .rsp_tx(rsp_if), .dat_rx(wd_if), .dat_tx(rd_if),
    .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
    .m_axi_awburst(awburst), .m_axi_awcache(awcache), .m_axi_awprot(awprot),
    .m_axi_awvalid(awvalid), .m_axi_awready(awready),
    .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
    .m_axi_bid(bid), .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
    .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
    .m_axi_arburst(arburst), .m_axi_arcache(arcache), .m_axi_arprot(arprot),
    .m_axi_arvalid(arvalid), .m_axi_arready(arready),
    .m_axi_rid(rid), .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast),
    .m_axi_rvalid(rvalid), .m_axi_rready(rready)
  );

  // ------------------------------------------------------------ operations
  typedef struct {
    bit           wr, full, done, dbid_seen;
    int           src, seq;
    logic [31:0]  addr;
    width_e       size;
    logic [511:0] line;       // read: expected line image; write: data
    logic [63:0]  mask;       // read: bytes to check; write: byte enables
    logic [1:0]   experr;
    int           beats, seen;
    logic [3:0]   ids_seen;
    int           dbid;
    bit           b_done;     // the AXI B of this write was given
  } op_t;
  op_t ops [int];                              // by TxnID
  int  outstanding [NSRC + 1];
  int  next_done   [NSRC + 1];                 // next seq that must complete
  int  finished = 0;

  function automatic int beats_of(width_e s);
    return (s == WIDTH_64) ? 4 : (s == WIDTH_32) ? 2 : 1;
  endfunction
  function automatic int first_id(logic [31:0] a, width_e s);
    return (s == WIDTH_64) ? 0 : (s == WIDTH_32) ? {a[5], 1'b0} : int'(a[5:4]);
  endfunction

  // ------------------------------------------------------------ TB -> DUT channels
  local_req_flit_t req_q [$];
  local_dat_flit_t wd_q  [$];
  int req_cr = 0, wd_cr = 0;                   // credits the DUT gave us
  int rsp_cr = 0, rd_cr = 0;                   // credits we gave the DUT, not used yet

  initial begin
    req_if.flitv = N; req_if.flitpend = N; req_if.flit = '0;
    wd_if.flitv  = N; wd_if.flitpend  = N; wd_if.flit  = '0;
    rsp_if.lcrdv = 1'b0; rd_if.lcrdv = 1'b0;
  end

  always @(posedge clk) if (resetn) begin
    if (req_if.lcrdv) req_cr++;
    if (wd_if.lcrdv)  wd_cr++;
  end

  always @(negedge clk) begin
    req_if.flitv = N; req_if.flitpend = N;
    wd_if.flitv  = N; wd_if.flitpend  = N;
    if (resetn) begin
      if (req_q.size() != 0) begin
        if (req_cr > 0 && !chance(10)) begin
          req_if.flit = req_q.pop_front(); req_if.flitv = Y; req_if.flitpend = Y; req_cr--;
        end else if (req_cr == 0) hit("chi: request waited for a REQ credit (HN-I full)");
      end
      if (wd_q.size() != 0 && wd_cr > 0 && !chance(10)) begin
        wd_if.flit = wd_q.pop_front(); wd_if.flitv = Y; wd_if.flitpend = Y; wd_cr--;
      end
      // the DUT's TX credits: at most 15 out, granted at random
      rsp_if.lcrdv = (rsp_cr < 15) && !chance(STALL_PCT);
      rd_if.lcrdv  = (rd_cr  < 15) && !chance(STALL_PCT);
    end else begin
      rsp_if.lcrdv = 1'b0; rd_if.lcrdv = 1'b0;
    end
  end

  always @(posedge clk) if (resetn) begin
    if (rsp_if.lcrdv) rsp_cr++;
    if (rd_if.lcrdv)  rd_cr++;
    if (rsp_if.flitv == Y) begin if (rsp_cr == 0) err("RSP flit without a credit"); rsp_cr--; end
    if (rd_if.flitv == Y)  begin if (rd_cr  == 0) err("DAT flit without a credit"); rd_cr--;  end
  end

  // ------------------------------------------------------------ sources
  function automatic void complete(int key);
    op_t o = ops[key];
    if (o.seq != next_done[o.src])
      err($sformatf("src %0d: op #%0d completed before #%0d (Device order)", o.src, o.seq, next_done[o.src]));
    next_done[o.src] = o.seq + 1;
    outstanding[o.src]--;
    ops.delete(key);
  endfunction

  for (genvar gs = 1; gs <= NSRC; gs++) begin : gen_src
    initial begin
      int unsigned seq = 0;
      @(posedge resetn);
      repeat (5) @(posedge clk);
      for (int i = 0; i < OPS; i++) begin
        op_t o;
        local_req_flit_t f;
        int nb, key;
        while (outstanding[gs] >= OUT_MAX) @(posedge clk);
        repeat (rr(0, 3)) @(posedge clk);
        o = '{size: WIDTH_1, default: '0};
        o.src = gs; o.seq = seq;
        o.wr  = chance(50);
        o.size = width_e'(chance(60) ? rr(0, 2) : rr(3, 6));      // mostly 1..4 B (registers)
        nb = 1 << o.size;
        o.addr = (chance(90) ? (32'h1000_0000 + gs * 32'h1000) :
                  chance(50) ? 32'hE000_0000 : 32'hF000_0000) + (rr(0, 4095) & ~(nb - 1));
        o.experr = region_resp(o.addr);
        o.beats  = beats_of(o.size);
        o.full   = o.wr && (o.size == WIDTH_64) && chance(50);
        if (o.wr) begin
          for (int b = 0; b < nb; b++) begin
            int off;
            off = o.addr[5:0] + b;
            o.line[off*8 +: 8] = byte'(rnd());
            o.mask[off]        = o.full ? 1'b1 : chance(85);
            if (o.mask[off] && o.experr == 2'b00) rmem[o.addr + b] = o.line[off*8 +: 8];
          end
          if (!o.full && o.mask == '0) hit("op: partial write with no byte enabled");
        end else begin
          for (int b = 0; b < nb; b++) begin
            int off;
            off = o.addr[5:0] + b;
            o.line[off*8 +: 8] = rget(o.addr + b);
            o.mask[off]        = 1'b1;
          end
        end
        key = {4'(gs), seq[7:0]};
        if (ops.exists(key)) err($sformatf("TB: TxnID %0h still in use", key));
        ops[key] = o;
        outstanding[gs]++;
        seq++;
        begin
          string opn, ern;                     // (string variables: a ?: of literals pads them)
          if (!o.wr)       opn = "ReadNoSnp";
          else if (o.full) opn = "WriteNoSnpFull";
          else             opn = "WriteNoSnpPtl";
          if (o.experr == 2'b10)      ern = " SLVERR";
          else if (o.experr == 2'b11) ern = " DECERR";
          else                        ern = "";
          hit($sformatf("op: %s %0d B%s", opn, nb, ern));
        end
        if (outstanding[gs] == OUT_MAX) hit("op: a source with 2 accesses outstanding");
        f = '0;
        f.tgtid  = HNI; f.srcid = node_id_e'(gs); f.txnid = 12'(key);
        f.opcode = o.wr ? (o.full ? REQ_WRITE_NO_SNOOP_FULL : REQ_WRITE_NO_SNOOP_PTL) : REQ_READ_NO_SNOOP;
        f.size   = o.size; f.addr = TSHR_ADDR_WIDTH'(o.addr);
        f.allowretry = Y; f.memattr = mem_attr_e'(4'b0010);   // Device
        f.expcompack = 1'b0;
        req_q.push_back(f);
      end
      while (outstanding[gs] != 0) @(posedge clk);
      finished++;
    end
  end

  // ------------------------------------------------------------ DUT -> TB: RSP and CompData
  always @(posedge clk) if (resetn) begin
    if (rsp_if.flitv == Y) begin
      local_rsp_flit_t r;
      int key;
      op_t o;
      r   = local_rsp_flit_t'(rsp_if.flit);
      key = int'(r.txnid);
      if (!ops.exists(key)) err($sformatf("RSP %s for unknown TxnID %0h", r.opcode.name(), r.txnid));
      else begin
        o = ops[key];
        if (r.srcid != HNI || r.tgtid != node_id_e'(o.src)) err($sformatf("RSP TxnID %0h: SrcID %0h TgtID %0h", key, r.srcid, r.tgtid));
        if (!o.wr) err($sformatf("RSP %s for a read (TxnID %0h)", r.opcode.name(), key));
        else if (r.opcode == RSP_DBID_RESP) begin
          if (o.dbid_seen) err($sformatf("second DBIDResp for TxnID %0h", key));
          o.dbid_seen = 1; o.dbid = int'(r.dbid);
          for (int k = 0; k < o.beats; k++) begin
            local_dat_flit_t d;
            int id;
            d  = '0;
            id = first_id(o.addr, o.size) + k;
            d.tgtid = HNI; d.srcid = node_id_e'(o.src); d.txnid = r.dbid; d.homenid = HNI;
            d.opcode = DAT_NON_COPY_BACK_WR_DATA; d.dataid = 2'(id);
            d.data = o.line[id*128 +: 128]; d.be = o.mask[id*16 +: 16];
            wd_q.push_back(d);
          end
          ops[key] = o;
        end else if (r.opcode == RSP_COMP) begin
          if (!o.dbid_seen) err($sformatf("Comp before DBIDResp, TxnID %0h", key));
          if (!o.b_done)    err($sformatf("Comp for TxnID %0h before the AXI B of its write", key));
          if (r.resperr != o.experr) err($sformatf("write TxnID %0h addr %0h: RespErr %0b, expected %0b", key, o.addr, r.resperr, o.experr));
          complete(key);
        end else err($sformatf("RSP opcode %s not expected (TxnID %0h)", r.opcode.name(), key));
      end
    end

    if (rd_if.flitv == Y) begin
      local_dat_flit_t d;
      int key, id;
      op_t o;
      d   = local_dat_flit_t'(rd_if.flit);
      key = int'(d.txnid);
      if (!ops.exists(key)) err($sformatf("CompData for unknown TxnID %0h", d.txnid));
      else begin
        o  = ops[key];
        id = int'(d.dataid);
        if (o.wr || d.opcode != DAT_COMP_DATA) err($sformatf("DAT %s for TxnID %0h (a %s)", d.opcode.name(), key, o.wr ? "write" : "read"));
        if (d.srcid != HNI || d.homenid != HNI || d.tgtid != node_id_e'(o.src) || d.resp != RESP_I)
          err($sformatf("CompData TxnID %0h: SrcID %0h HomeNID %0h TgtID %0h Resp %0b", key, d.srcid, d.homenid, d.tgtid, d.resp));
        if (id < first_id(o.addr, o.size) || id >= first_id(o.addr, o.size) + o.beats || o.ids_seen[id])
          err($sformatf("CompData TxnID %0h: DataID %0d not expected", key, id));
        o.ids_seen[id] = 1;
        if (d.resperr != o.experr) err($sformatf("read TxnID %0h addr %0h: RespErr %0b, expected %0b", key, o.addr, d.resperr, o.experr));
        if (o.experr == 2'b00)
          for (int b = 0; b < 16; b++)
            if (o.mask[id*16 + b] && d.data[b*8 +: 8] !== o.line[(id*16 + b)*8 +: 8])
              err($sformatf("read TxnID %0h addr %0h byte %0d: got %02h, expected %02h", key, o.addr, id*16 + b - o.addr[5:0],
                            d.data[b*8 +: 8], o.line[(id*16 + b)*8 +: 8]));
        o.seen++;
        ops[key] = o;
        if (o.seen == o.beats) complete(key);
      end
    end
  end

  // ------------------------------------------------------------ AXI slave
  typedef struct { logic [IDW-1:0] id; logic [31:0] addr; int len; int size; int beat; int ready_at; } burst_t;
  burst_t rq [$];                  // reads waiting / sending
  burst_t aq [$];                  // writes: AW taken, W beats in order
  typedef struct { logic [IDW-1:0] id; logic [1:0] resp; int ready_at; } bresp_t;
  bresp_t bq [$];
  int r_cur = -1;                  // read burst whose beat is on R now
  bit r_hs = 0, b_hs = 0;          // the R beat / B on the bus was taken at the last edge
  logic [IDW-1:0] last_rid; bit last_rid_open = 0;

  function automatic void ax_check(string ch, logic [31:0] a, logic [7:0] len, logic [2:0] size, logic [1:0] burst);
    int bytes = (int'(len) + 1) << size;
    if (burst != 2'b01) err($sformatf("%s burst %0b, expected INCR", ch, burst));
    if (size > 3'd2)    err($sformatf("%s size %0d: more than 4 B on a 32-bit bus", ch, size));
    if ((a & ((32'd1 << size) - 1)) != 0) err($sformatf("%s addr %0h not aligned to size %0d", ch, a, size));
    if (bytes > 64)     err($sformatf("%s burst of %0d B: more than one 64 B line", ch, bytes));
    if ((a >> 12) != ((a + bytes - 1) >> 12)) err($sformatf("%s burst %0h + %0d crosses 4 KB", ch, a, bytes));
  endfunction

  initial begin
    arready = 0; awready = 0; wready = 0; rvalid = 0; bvalid = 0; rlast = 0;
    rid = '0; rdata = '0; rresp = '0; bid = '0; bresp = '0;
  end

  always @(negedge clk) if (resetn) begin
    arready = !chance(STALL_PCT);
    awready = !chance(STALL_PCT);
    wready  = (aq.size() != 0) && !chance(STALL_PCT);
    // R: a beat of a random ready burst (different IDs interleave); hold while not taken
    if (!rvalid || r_hs) begin
      rvalid = 0;
      r_hs   = 0;
      begin
        int c[$];
        c.delete();                                    // (static in an always block)
        foreach (rq[i]) if (rq[i].ready_at <= int'(cyc)) c.push_back(i);
        if (c.size() != 0 && !chance(STALL_PCT)) begin
          logic [31:0] wa;
          r_cur = c[rnd() % c.size()];
          wa = (rq[r_cur].addr & ~32'h3) + 32'(rq[r_cur].beat * (1 << rq[r_cur].size)) ;
          wa = wa & ~32'h3;
          rid   = rq[r_cur].id;
          rdata = {sget(wa + 3), sget(wa + 2), sget(wa + 1), sget(wa)};
          rresp = region_resp(rq[r_cur].addr);
          rlast = (rq[r_cur].beat == rq[r_cur].len);
          rvalid = 1;
          if (last_rid_open && rid != last_rid) hit("axi: read beats of two IDs interleaved");
        end
      end
    end
    // B: a random ready response
    if (!bvalid || b_hs) begin
      bvalid = 0;
      b_hs   = 0;
      begin
        int c[$];
        int j;
        c.delete();
        foreach (bq[i]) if (bq[i].ready_at <= int'(cyc)) c.push_back(i);
        if (c.size() != 0 && !chance(STALL_PCT)) begin
          j = c[rnd() % c.size()];
          if (j != 0) hit("axi: B responses out of order");
          bid = bq[j].id; bresp = bq[j].resp; bvalid = 1;
          bq.delete(j);
        end
      end
    end
  end

  always @(posedge clk) if (resetn) begin
    if ($test$plusargs("trace") && cyc < 80) begin
      if (arvalid) $display("[%0d] AR v=%0b r=%0b id=%0h addr=%0h len=%0d", cyc, arvalid, arready, arid, araddr, arlen);
      if (rvalid)  $display("[%0d] R  id=%0h last=%0b rq=%0d", cyc, rid, rlast, rq.size());
      if (req_if.flitv == Y) $display("[%0d] REQ %p", cyc, local_req_flit_t'(req_if.flit));
    end
    if (arvalid && arready) begin
      ax_check("AR", araddr, arlen, arsize, arburst);
      rq.push_back('{id: arid, addr: araddr, len: int'(arlen), size: int'(arsize), beat: 0, ready_at: int'(cyc) + rr(1, LAT_MAX)});
      if (arsize < 3'd2) hit("axi: narrow read");
      if (arlen == 8'd15) hit("axi: 16-beat read burst");
    end
    if (rvalid && rready) begin
      last_rid = rid; last_rid_open = !rlast;
      if (rlast) rq.delete(r_cur); else rq[r_cur].beat++;
      r_hs = 1;             // the negedge picks the next beat
    end
    if (awvalid && awready) begin
      ax_check("AW", awaddr, awlen, awsize, awburst);
      aq.push_back('{id: awid, addr: awaddr, len: int'(awlen), size: int'(awsize), beat: 0, ready_at: 0});
      if (awsize < 3'd2) hit("axi: narrow write");
      if (awlen == 8'd15) hit("axi: 16-beat write burst");
    end
    if (wvalid && wready) begin
      if (aq.size() == 0) err("W beat with no AW");
      else begin
        logic [31:0] wa;
        logic [3:0]  lanes;
        wa = (aq[0].addr & ~32'h3) + 32'(aq[0].beat * (1 << aq[0].size));
        wa = wa & ~32'h3;
        lanes = (aq[0].size == 0) ? (4'b0001 << aq[0].addr[1:0]) :
                (aq[0].size == 1) ? (4'b0011 << {aq[0].addr[1], 1'b0}) : 4'b1111;
        if ((wstrb & ~lanes) != 0) err($sformatf("W strobe %04b outside the lanes %04b of a %0d B beat", wstrb, lanes, 1 << aq[0].size));
        if (wlast != (aq[0].beat == aq[0].len)) err($sformatf("WLAST %0b on beat %0d of %0d", wlast, aq[0].beat, aq[0].len + 1));
        if (region_resp(aq[0].addr) == 2'b00)
          for (int b = 0; b < 4; b++) if (wstrb[b]) smem[wa + b] = wdata[b*8 +: 8];
        if (aq[0].beat == aq[0].len) begin
          bq.push_back('{id: aq[0].id, resp: region_resp(aq[0].addr), ready_at: int'(cyc) + rr(1, LAT_MAX)});
          aq.delete(0);
        end else aq[0].beat++;
      end
    end
    if (bvalid && bready) begin
      // the write whose AXI ID this is: {src, tracker}, its DBID = tracker
      int src, trk;
      bit found;
      src = int'(bid[IDW-1:TW]); trk = int'(bid[TW-1:0]); found = 0;
      foreach (ops[k]) if (!found && ops[k].wr && ops[k].src == src && ops[k].dbid_seen && ops[k].dbid == trk && !ops[k].b_done) begin
        ops[k].b_done = 1; found = 1;
      end
      if (!found) err($sformatf("AXI B for ID %0h: no write of source %0d with DBID %0d waits", bid, src, trk));
      b_hs = 1;
    end
    // HN-I occupancy
    if (dut.busy == '1) hit("hni: every tracker busy");
  end

  // ------------------------------------------------------------ run
  initial begin
    rs = 32'h9E3779B9 ^ (SEED * 32'h85EBCA6B);
    if (rs == 0) rs = 1;
    for (int s = 0; s <= NSRC; s++) begin outstanding[s] = 0; next_done[s] = 0; end
    repeat (10) @(posedge clk);
    resetn = 1;
    fork
      begin
        wait (finished == NSRC);
        repeat (200) @(posedge clk);
      end
      begin
        repeat (TIMEOUT) @(posedge clk);
        err($sformatf("TIMEOUT after %0d cycles: %0d sources finished, %0d operations open", TIMEOUT, finished, ops.size()));
      end
    join_any
    // end of test
    if (dut.busy != '0) err($sformatf("end: HN-I trackers still busy (%b)", dut.busy));
    if (rq.size() || aq.size() || bq.size()) err("end: AXI transactions still open");
    if (req_q.size() || wd_q.size()) err("end: TB flits never sent");
    foreach (rmem[a]) if (smem.exists(a) ? smem[a] != rmem[a] : init_b(a) != rmem[a]) begin
      err($sformatf("end: memory byte %0h is %02h, expected %02h", a, sget(a), rmem[a])); break;
    end
    begin
      string need [$] = '{"chi: request waited for a REQ credit (HN-I full)", "axi: read beats of two IDs interleaved",
                          "axi: B responses out of order", "axi: narrow read", "axi: narrow write",
                          "axi: 16-beat read burst", "axi: 16-beat write burst", "hni: every tracker busy",
                          "op: a source with 2 accesses outstanding", "op: ReadNoSnp 4 B SLVERR", "op: WriteNoSnpPtl 4 B DECERR",
                          "op: WriteNoSnpFull 64 B", "op: ReadNoSnp 64 B"};
      foreach (need[i]) if (!cov.exists(need[i])) err($sformatf("COVERAGE: never reached: \"%s\"", need[i]));
    end
    $display("---- HN-I TB (seed %0d): %0d sources x %0d accesses, finished at cycle %0d", SEED, NSRC, OPS, cyc);
    foreach (cov[k]) $display("  %-52s %0d", k, cov[k]);
    if (n_err == 0) $display("*** HN-I REGRESSION: ALL CHECKS PASSED (seed=%0d) ***", SEED);
    else            $display("*** HN-I REGRESSION: %0d FAILURES (seed=%0d) - rerun with this SEED to reproduce ***", n_err, SEED);
    $finish;
  end

endmodule
