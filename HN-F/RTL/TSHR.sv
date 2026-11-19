
// TSHR - transaction snoop handling register
//
// Role in the HN-F node
// this is the simplest possible or lets say "dumbest" tracker module i could think of
// it only knows one transaction at time so HN-F node should be handle it
// it has no idea about the interfaceses or RN-F port counts
// it sees one requester and one snoop target at a time snf is not a snoop target it is just a dumb slave
// top module should instantiate this module multiple times to track multiple outstanding requests at one
// HN-F also need to track which TSHR is free to give request to them
// HN-F maps the requester's SrcID to its RN-F index (req_port_i); snoops carry
// TgtID = RNF_NID[target index] so the XP can route them
// HN-F also need address hazard detection to detect which tracker is safe to give
//
// Snoop filter (snoop_filter.sv, owned by the HN-F)
//   ST_SF_LOOKUP reads the entry of this line once (presence bit per RN-F).
//   Snoops go only to RN-Fs whose bit is set, never to the requester
//   (snp_port_o names the target of the snoop being issued):
//   * reads that need one copy (ReadShared / ReadClean / ReadOnce /
//     ReadNotSharedDirty): one RN-F at a time (ST_SNOOP_PICK -> ST_SNOOP_SEND
//     -> ST_SNOOP_WAIT), stopping at the first data
//   * invalidating snoops (ReadUnique, WriteUnique*, CMOs, back-invalidation):
//     every target is issued back-to-back and the responses are collected in
//     any order, from all ports at once (ST_SNOOP_PAR). The HN-F hands every
//     tracker every port's SnpResp / SnpRespData; it keeps those with HN_TXNID.
//   Each SnpResp* with final state I clears that bit in sfpres_q; ST_DONE writes
//   the final vector back (plus the requester bit: set when it now caches the
//   line, cleared on WriteBack / Evict).
//   A lookup that needs a new entry may take over a victim entry: the tracker
//   then back-invalidates the victim line first (bi_q, SnpCleanInvalid to every
//   present RN-F, dirty data written to the SN-F) and only then starts its own
//   transaction. bi_o / bi_addr_o let the HN-F hazard logic hold younger
//   requests to the victim line meanwhile.
//
// NONE OF THE ROUTING AND ARBITRATION LIVES INSIDE THIS FILE.


module TSHR

 import chi_pkg::*;
 import common_pkg::*;
 import tshr_flit_pkg::*;
 #(
  parameter int AddrWidth      = 39,
  parameter int DataWidth      = 128,
  // total data moved per transaction as bits .
  // must be an integer multiple of DataWidth. NumBeats = CacheLineWidth 
  parameter int CacheLineWidth = 512,
  parameter int TRK_ID         = 0,
  parameter chi_pkg::node_id_e HNFID = 7'h40,
  parameter chi_pkg::node_id_e SNFID = 7'h00,
  parameter int NUM_RNF        = 2,
  parameter chi_pkg::nid_list_t RNF_NID = chi_pkg::NID_SEQ1,  // node ID of RN-F index i (snoop TgtID)
  parameter int SF_WAYW        = 1,
  parameter bit SF_BROADCAST   = 1'b0,   // 1: snoop every other RN-F, ignore the filter's presence bits
  localparam int RNF_IDX_W     = (NUM_RNF <= 1) ? 1 : $clog2(NUM_RNF)
) (
  input  wire clk,
  input  wire resetn,
  output logic ready_o,

  // ---- RN-F ports: which one is the requester, which one gets the current snoop
  input  logic [RNF_IDX_W-1:0] req_port_i,
  output logic [RNF_IDX_W-1:0] snp_port_o,

  // ---- back-invalidation of a snoop filter victim (for the HN-F hazard logic)
  output logic                 bi_o,
  output logic [AddrWidth-1:0] bi_addr_o,

  // ---- snoop filter lookup (ST_SF_LOOKUP) and final write (ST_DONE)
  output logic                 sf_lk_req_o,
  output logic                 sf_lk_alloc_o,
  input  logic                 sf_lk_gnt_i,
  input  logic                 sf_lk_entry_i,
  input  logic [SF_WAYW-1:0]   sf_lk_way_i,
  input  logic [NUM_RNF-1:0]   sf_lk_pres_i,
  input  logic                 sf_lk_victim_i,
  input  logic [AddrWidth-1:0] sf_lk_victim_addr_i,
  input  logic [NUM_RNF-1:0]   sf_lk_victim_pres_i,
  output logic                 sf_wr_o,
  output logic [SF_WAYW-1:0]   sf_wr_way_o,
  output logic [NUM_RNF-1:0]   sf_wr_pres_o,



  output logic [11:0] TxnID_o,
  output node_id_e SrcID_o,
  output logic [AddrWidth-1:0] Addr_o,
  input  logic                 hazard_i,


  // ---- REQ from the requester (rx side): the original request ----
  input  common_pkg::yn_status_e req_flitv,
  input  common_pkg::yn_status_e req_flitpend,
  input  local_req_flit_t        req_flit,

  // ---- SNP to the snoop target (tx side): the snoop this tracker issues
  output common_pkg::yn_status_e snp_flitv,
  output common_pkg::yn_status_e snp_flitpend,
  output local_snp_flit_t        snp_flit,
  input  logic                   snp_gnt,

  // ---- RSP to the requester (tx side): DBIDResp / CompDBIDResp / Comp
  output common_pkg::yn_status_e rsp_tx_flitv,
  output common_pkg::yn_status_e rsp_tx_flitpend,
  output local_rsp_flit_t        rsp_tx_flit,
  input  logic                   rsp_tx_gnt,

  // ---- RSP from the requester (rx side): CompAck
  input  common_pkg::yn_status_e rsp_rx_req_flitv,
  input  common_pkg::yn_status_e rsp_rx_req_flitpend,
  input  local_rsp_flit_t        rsp_rx_req_flit,

  // ---- RSP from the snoop targets (rx side): SnpResp, one entry per RN-F port
  //      (every port is seen; a flit is ours when it carries HN_TXNID)
  input  common_pkg::yn_status_e rsp_rx_snp_flitv    [NUM_RNF],
  input  common_pkg::yn_status_e rsp_rx_snp_flitpend [NUM_RNF],
  input  local_rsp_flit_t        rsp_rx_snp_flit     [NUM_RNF],

  // ---- DAT to the requester (tx side): CompData proxy forward
  output common_pkg::yn_status_e dat_tx_flitv,
  output common_pkg::yn_status_e dat_tx_flitpend,
  output local_dat_flit_t        dat_tx_flit,
  input  logic                   dat_tx_gnt,

  // ---- DAT from the requester (rx side): write-data
  input  common_pkg::yn_status_e dat_rx_req_flitv,
  input  common_pkg::yn_status_e dat_rx_req_flitpend,
  input  local_dat_flit_t        dat_rx_req_flit,

  // ---- DAT from the snoop targets (rx side): snoop-resp-data, one entry per RN-F port
  input  common_pkg::yn_status_e dat_rx_snp_flitv    [NUM_RNF],
  input  common_pkg::yn_status_e dat_rx_snp_flitpend [NUM_RNF],
  input  local_dat_flit_t        dat_rx_snp_flit     [NUM_RNF],

  // ---- SN-F side (already scalar, unchanged) ----
  output common_pkg::yn_status_e sn_req_flitv,
  output common_pkg::yn_status_e sn_req_flitpend,
  output local_req_flit_t        sn_req_flit,
  input  logic                   sn_req_gnt,

  input  common_pkg::yn_status_e sn_rsp_flitv,
  input  common_pkg::yn_status_e sn_rsp_flitpend,
  input  local_rsp_flit_t        sn_rsp_flit,

  output common_pkg::yn_status_e sn_dat_tx_flitv,
  output common_pkg::yn_status_e sn_dat_tx_flitpend,
  output local_dat_flit_t        sn_dat_tx_flit,
  input  logic                   sn_dat_tx_gnt,

  input  common_pkg::yn_status_e sn_dat_rx_flitv,
  input  common_pkg::yn_status_e sn_dat_rx_flitpend,
  input  local_dat_flit_t        sn_dat_rx_flit,

  output logic                   pcrd_req_o,
  output logic [3:0]             pcrd_type_o,
  input  logic                   pcrd_gnt_i
);

  localparam int NumBeats  = CacheLineWidth / DataWidth;
  localparam int BeatCntW  = (NumBeats <= 1) ? 1 : $clog2(NumBeats);
  localparam logic [BeatCntW-1:0] LAST_BEAT = BeatCntW'(NumBeats-1);
  localparam int BeatBytes = DataWidth / 8;
  localparam int BeatOffW  = $clog2(BeatBytes);
  localparam int LineOffW  = $clog2(CacheLineWidth / 8);
  localparam int DidShift  = $clog2(DataWidth / 128);
  localparam logic [11:0] HN_TXNID = 12'(TRK_ID);

  initial begin // simple beat tracker it is not parameterized  but this will give error if someone change it late on
    if (CacheLineWidth % DataWidth != 0) begin
      $fatal(1, "TSHR cacheline widht (%0d) must be an integer multiple of DataWidth (%0d)",
             CacheLineWidth, DataWidth);
    end
    if (NumBeats > 4) begin
      $fatal(1, "TSHR beat number (%0d) exceeds 4 - DataID is only 2 bits ",
             NumBeats);
    end
    if (DataWidth < 128) begin
      $fatal(1, "TSHR DataWidth (%0d) must be at least 128 - DataID counts 128-bit chunks", DataWidth);
    end
  end

  typedef enum logic [4:0] {
    ST_IDLE,
    ST_ALLOC,
    ST_SNOOP_SEND,
    ST_SNOOP_WAIT,
    ST_DBIDRESP_SEND,
    ST_WDATA_WAIT,
    ST_SN_REQ_SEND,
    ST_SN_RSP_WAIT,
    ST_SN_DATA_SEND,
    ST_RDATA_SEND,
    ST_COMP_SEND,
    ST_COMPACK_WAIT,
    ST_DONE,
    ST_READNO_SNOOP,
    ST_DIRTY_WB_REQ_SEND,
    ST_DIRTY_WB_RSP_WAIT,
    ST_DIRTY_WB_DATA_SEND,
    ST_SN_COMP_WAIT,
    ST_SN_RETRY_WAIT,
    ST_HAZARD_WAIT,
    ST_SF_LOOKUP,
    ST_SNOOP_PICK,
    ST_SNOOP_PAR      // invalidating snoops: all targets issued back-to-back, responses collected in any order
  } state_e;

  state_e state_q, state_d;

  logic [AddrWidth-1:0]  addr_q, addr_d;
  node_id_e              srcid_q, srcid_d;
  logic [11:0]           txnid_q, txnid_d;
  req_opcode_e            opcode_q, opcode_d;
  width_e                 size_q, size_d;
  logic [3:0]             qos_q, qos_d;
  logic                   expcompack_q, expcompack_d;

  // beat data storage
  logic [DataWidth-1:0]     data_q [NumBeats], data_d [NumBeats];
  logic [(DataWidth/8)-1:0] be_q   [NumBeats], be_d   [NumBeats];
  logic [BeatCntW-1:0]      beat_cnt_q, beat_cnt_d;

  logic [11:0]              sn_dbid_q, sn_dbid_d;

  logic [1:0]               resperr_q, resperr_d;
  logic [2:0]               wdata_resp_q, wdata_resp_d;
  logic                     sn_comp_seen_q, sn_comp_seen_d;
  logic                     compack_seen_q, compack_seen_d;
  logic                     wb_chain_q, wb_chain_d;
  logic                     ptl_q, ptl_d;  // snoop data came as SnpRespDataPtl
  common_pkg::yn_status_e   allowretry_q, allowretry_d;
  logic [3:0]               pcrdtype_q, pcrdtype_d;
  state_e                   retry_ret_q, retry_ret_d;

  // snoop filter / snoop loop
  logic                     sf_entry_q, sf_entry_d;   // this tracker owns an SF entry for its line
  logic [SF_WAYW-1:0]       sf_way_q,   sf_way_d;
  logic [NUM_RNF-1:0]       sfpres_q,   sfpres_d;     // presence vector of this line, updated by the snoop responses
  logic [NUM_RNF-1:0]       snp_mask_q, snp_mask_d;   // RN-Fs still to be snooped
  logic [RNF_IDX_W-1:0]     snp_tgt_q,  snp_tgt_d;    // RN-F of the snoop in flight
  logic                     got_data_q, got_data_d;   // a snoop returned data for this transaction
  logic                     bi_q,       bi_d;         // back-invalidating the SF victim
  logic [AddrWidth-1:0]     bi_addr_q,  bi_addr_d;
  // parallel snoops (ST_SNOOP_PAR): snp_mask_q = still to issue, snp_tgt_q =
  // the next one to issue (lowest bit of snp_mask_q)
  logic [NUM_RNF-1:0]       pend_q,     pend_d;       // issued, response not complete yet
  logic [BeatCntW-1:0]      pbeat_q [NUM_RNF], pbeat_d [NUM_RNF];  // SnpRespData beats seen per port
  logic                     dsrc_v_q,   dsrc_v_d;     // a port's data is being kept
  logic [RNF_IDX_W-1:0]     dsrc_q,     dsrc_d;       // ...which one (its beats go to data_q)
  logic                     rts_used_q, rts_used_d;   // RetToSrc already asked of one snoop
  logic [NUM_RNF-1:0]       req_bit;

  assign req_bit = NUM_RNF'(1) << req_port_i;

  logic [BeatCntW:0]        req_beats;
  logic [BeatCntW-1:0]      first_beat;
  logic [BeatCntW-1:0]      tx_beat;
  logic                     req_last_beat;
  logic                     line_last_beat;

  always_comb begin
    if ((8'd1 << size_q) <= 8'(BeatBytes)) req_beats = (BeatCntW+1)'(1);
    else                                   req_beats = (BeatCntW+1)'((8'd1 << size_q) / 8'(BeatBytes));
    first_beat     = BeatCntW'(addr_q[LineOffW-1:0] >> BeatOffW) & ~BeatCntW'(req_beats - 1'b1);
    tx_beat        = first_beat + beat_cnt_q;
    req_last_beat  = ({1'b0, beat_cnt_q} == (req_beats - 1'b1));
    line_last_beat = (beat_cnt_q == LAST_BEAT);
  end

     // simple decoding block to understand where to go
     // this simplify the if else blocks and makes us only check for tx_type
     // it is latched on the alloc state
    typedef enum logic [2:0] {
        TX_READ,
        TX_WRITE_UNIQUE,
        TX_WRITE_BACK,
        TX_READ_NOSNP,
        TX_WRITE_NOSNP,
        TX_CMO
      } tx_type_e;

      tx_type_e tx_type_q, tx_type_d;

    logic            rx_rsp_flitv_snp;
    local_rsp_flit_t rx_rsp_flit_snp;

    logic            rx_rsp_flitv_req;
    local_rsp_flit_t rx_rsp_flit_req;

    logic            rx_dat_flitv_snp;
    local_dat_flit_t rx_dat_flit_snp;

    logic            rx_dat_flitv_req;
    local_dat_flit_t rx_dat_flit_req;

   // simplest possible fsm that i can think of
   // the basic sequence of the fsm is the following
   // req flit comes to the HN-F node from RN-F0 It holds the addr and other useful things inside the tracker
   // after it accepts the flit it goes to RN-F1 for snoop requests and waits for their response
   // depending on the RN-F1's response and depending on read or write, it either goes to ST_DBIDRESP_SEND or ST_RDATA_SEND

  // i was going to include everything in here but like  i have second monitor now so i just open specs at this point

   assign ready_o = (state_q == ST_IDLE);
   logic dirty_hit_q, dirty_hit_d;
  //  default state you dont like latches are you? so dont hurt the feelings of this state
  function automatic void fsm_defaults();
    state_d       = state_q;
    tx_type_d     = tx_type_q;
    addr_d        = addr_q;
    srcid_d       = srcid_q;
    txnid_d       = txnid_q;
    opcode_d      = opcode_q;
    size_d        = size_q;
    qos_d         = qos_q;
    expcompack_d  = expcompack_q;
    for (int i = 0; i < NumBeats; i++) begin
      data_d[i] = data_q[i];
      be_d[i]   = be_q[i];
    end
    beat_cnt_d     = beat_cnt_q;
    sn_dbid_d      = sn_dbid_q;
    dirty_hit_d    = dirty_hit_q;
    resperr_d      = resperr_q;
    wdata_resp_d   = wdata_resp_q;
    sn_comp_seen_d = sn_comp_seen_q;
    compack_seen_d = compack_seen_q;
    wb_chain_d     = wb_chain_q;
    ptl_d          = ptl_q;
    allowretry_d   = allowretry_q;
    pcrdtype_d     = pcrdtype_q;
    retry_ret_d    = retry_ret_q;
    sf_entry_d     = sf_entry_q;
    sf_way_d       = sf_way_q;
    sfpres_d       = sfpres_q;
    snp_mask_d     = snp_mask_q;
    snp_tgt_d      = snp_tgt_q;
    got_data_d     = got_data_q;
    bi_d           = bi_q;
    bi_addr_d      = bi_addr_q;
    pend_d         = pend_q;
    for (int p = 0; p < NUM_RNF; p++) pbeat_d[p] = pbeat_q[p];
    dsrc_v_d       = dsrc_v_q;
    dsrc_d         = dsrc_q;
    rts_used_d     = rts_used_q;

    sf_lk_req_o    = 1'b0;
    sf_lk_alloc_o  = 1'b0;
    sf_wr_o        = 1'b0;
    sf_wr_way_o    = sf_way_q;
    sf_wr_pres_o   = sfpres_q;

    // you lazy interfaceses just sleep for now
    snp_flitv    = N; snp_flitpend    = N; snp_flit    = '0;
    rsp_tx_flitv = N; rsp_tx_flitpend = N; rsp_tx_flit = '0;
    dat_tx_flitv = N; dat_tx_flitpend = N; dat_tx_flit = '0;

    sn_req_flitv       = N;
    sn_req_flitpend    = N;
    sn_req_flit        = '0;
    sn_dat_tx_flitv    = N;
    sn_dat_tx_flitpend = N;
    sn_dat_tx_flit     = '0;

    pcrd_req_o  = 1'b0;
    pcrd_type_o = pcrdtype_q;
  endfunction

  // requester CompAck comes in while waiting on it in ST_COMPACK_WAIT the snoop target SnpResp comes in while we are 
   // waiting on it in ST_SNOOP_WAIT each own fixed port so this is just a pick
  function automatic void fsm_select_inputs();
    // serial snoop (ST_SNOOP_WAIT): the port of the snoop in flight, as before
    rx_rsp_flitv_snp = (rsp_rx_snp_flitv[snp_tgt_q] == Y);
    rx_rsp_flit_snp  = rsp_rx_snp_flit[snp_tgt_q];

    rx_rsp_flitv_req = (rsp_rx_req_flitv == Y);
    rx_rsp_flit_req  = rsp_rx_req_flit;

    rx_dat_flitv_snp = (dat_rx_snp_flitv[snp_tgt_q] == Y);
    rx_dat_flit_snp  = dat_rx_snp_flit[snp_tgt_q];

    rx_dat_flitv_req = (dat_rx_req_flitv == Y);
    rx_dat_flit_req  = dat_rx_req_flit;
  endfunction

  function automatic void fsm_watch_compack();
    if ((state_q != ST_IDLE) && (state_q != ST_ALLOC) && rx_rsp_flitv_req &&
        (rx_rsp_flit_req.opcode == RSP_COMP_ACK) &&
        (rx_rsp_flit_req.txnid == txnid_q) && (rx_rsp_flit_req.srcid == srcid_q)) begin
      compack_seen_d = 1'b1;
    end
  endfunction

  // the chief engineer at amba thought this is a good idea so here we go
  // specs doesnt order snfs separate comp against the dbidresp or the write data 
  // it might come before dbidresp between dbidresp and data or after the data 
  // so it should be caught every state not only in the comp_wait 
  // it even gets worst dont get comfortable 
  // when sn request is send or resend yea it can resend it so comp can only belong to the write 
  // that is why it is outstanding

  // so every function is some combination of where is comp gotten
  function automatic void fsm_watch_sn_comp();
    if ((sn_rsp_flitv == Y) && (sn_rsp_flit.opcode == RSP_COMP) && (sn_rsp_flit.txnid == HN_TXNID)) begin
      sn_comp_seen_d = 1'b1;
      resperr_d      = resperr_q | sn_rsp_flit.resperr;
    end
  endfunction

  // the requester will hold the line afterwards, so the snoop filter needs an entry for it
  function automatic logic sf_alloc_op();
    return (tx_type_q == TX_READ && opcode_q != REQ_READ_ONCE) ||
           opcode_q == REQ_CLEAN_UNIQUE || opcode_q == REQ_MAKE_UNIQUE;
  endfunction

  // the snoop filter part is done (lookup, maybe back-invalidation): start the
  // request itself. WriteBack / WriteClean / Evict need no snoop; everything
  // else snoops every RN-F the filter lists, except the requester.
  function automatic void start_main(logic [NUM_RNF-1:0] pres);
    got_data_d = 1'b0;
    if (tx_type_q == TX_WRITE_BACK) begin
      state_d = ST_DBIDRESP_SEND;
    end else if (opcode_q == REQ_EVICT) begin
      state_d = ST_COMP_SEND;
    end else begin
      start_snoops((SF_BROADCAST ? {NUM_RNF{1'b1}} : pres) & ~req_bit, 1'b0);
    end
  endfunction

  // Snoops that invalidate (or clean) every copy need every sharer anyway, so
  // they go out back-to-back and are collected in any order (ST_SNOOP_PAR):
  // ReadUnique, WriteUnique*, CleanShared / CleanInvalid / MakeInvalid /
  // CleanUnique / MakeUnique, and the back-invalidation of an SF victim.
  // Reads that need one copy keep the serial walk (ST_SNOOP_PICK), which stops
  // at the first data instead of downgrading every sharer.
  function automatic logic par_op(logic bi);
    return bi || tx_type_q == TX_CMO || tx_type_q == TX_WRITE_UNIQUE ||
           (tx_type_q == TX_READ && opcode_q == REQ_READ_UNIQUE);
  endfunction

  function automatic logic [RNF_IDX_W-1:0] lowest(logic [NUM_RNF-1:0] m);
    logic [RNF_IDX_W-1:0] r;
    r = '0;
    for (int p = NUM_RNF - 1; p >= 0; p--) if (m[p]) r = RNF_IDX_W'(p);
    return r;
  endfunction

  function automatic void start_snoops(logic [NUM_RNF-1:0] mask, logic bi);
    snp_mask_d = mask;
    if (par_op(bi)) begin
      snp_tgt_d  = lowest(mask);
      pend_d     = '0;
      for (int p = 0; p < NUM_RNF; p++) pbeat_d[p] = '0;
      dsrc_v_d   = 1'b0;
      rts_used_d = 1'b0;
      state_d    = ST_SNOOP_PAR;
    end else begin
      state_d    = ST_SNOOP_PICK;
    end
  endfunction

  // back-invalidation finished (victim line gone from every RN-F, dirty data in memory)
  function automatic void bi_done();
    bi_d        = 1'b0;
    dirty_hit_d = 1'b0;
    ptl_d       = 1'b0;
    wb_chain_d  = 1'b0;
    beat_cnt_d  = '0;
    resperr_d   = '0;     // a write error on the victim line is not this request's error
    for (int i = 0; i < NumBeats; i++) begin
      data_d[i] = '0;
      be_d[i]   = '0;
    end
    start_main(sfpres_q);
  endfunction

  function automatic void goto_after_dirty_wb();
    wb_chain_d  = 1'b0;
    dirty_hit_d = 1'b0;
    ptl_d       = 1'b0;
    if (bi_q) begin
      bi_done();
    end else if (tx_type_q == TX_READ && ptl_q) begin
      // the partial snoop is in the memory so get that little boy
      state_d = ST_SN_REQ_SEND;
    end else if (tx_type_q == TX_WRITE_UNIQUE) begin
      state_d = ST_DBIDRESP_SEND;
    end else if (tx_type_q == TX_CMO) begin
      state_d = ST_COMP_SEND;
    end else begin
      state_d = ST_DONE;
    end
  endfunction
  
  function automatic void goto_after_sn_write();
    if (tx_type_q == TX_WRITE_BACK) begin
      state_d = expcompack_q ? ST_COMPACK_WAIT : ST_DONE;
    end else begin
      state_d = ST_COMP_SEND;
    end
  endfunction
 // a imaginary resting spot it doenst exist they said
  function automatic void st_idle();
    dirty_hit_d    = 0; //you stupid fuck
    beat_cnt_d     = '0;
    resperr_d      = '0;
    wdata_resp_d   = '0;
    sn_comp_seen_d = 1'b0;
    compack_seen_d = 1'b0;
    wb_chain_d     = 1'b0;
    ptl_d          = 1'b0;
    allowretry_d   = Y;
    pcrdtype_d     = '0;
    sf_entry_d     = 1'b0;
    sfpres_d       = '0;
    snp_mask_d     = '0;
    got_data_d     = 1'b0;
    bi_d           = 1'b0;
    if (req_flitv == Y) begin
      addr_d       = req_flit.addr;
      srcid_d      = req_flit.srcid;
      txnid_d      = req_flit.txnid;
      opcode_d     = req_flit.opcode;
      size_d       = req_flit.size;
      qos_d        = req_flit.qos;
      expcompack_d = req_flit.expcompack;
      state_d      = ST_ALLOC;
    end
  endfunction
   // well it is just waiting until hazard cleared 
  function automatic void st_hazard_wait();
    if (!hazard_i) begin
      state_d = ST_ALLOC;
    end
  endfunction
  //lets go guys we have a transaction to route so we are allocating one spot for it
  function automatic void st_alloc(); 
    if (hazard_i) begin // HAZARD EXIST freak the fuck out and panic sell everything right now
      state_d = ST_HAZARD_WAIT;
    end else if (opcode_q == REQ_WRITE_BACK_FULL || opcode_q == REQ_WRITE_CLEAN_FULL || opcode_q == REQ_WRITE_BACK_PTL) begin
      tx_type_d = TX_WRITE_BACK;
      state_d   = ST_SF_LOOKUP;     // no snoop, but the SF entry is updated at the end
    end else if (opcode_q == REQ_WRITE_UNIQUE_PTL || opcode_q == REQ_WRITE_UNIQUE_FULL) begin
      tx_type_d = TX_WRITE_UNIQUE;
      state_d   = ST_SF_LOOKUP;
    end else if (opcode_q == REQ_WRITE_NO_SNOOP_PTL || opcode_q == REQ_WRITE_NO_SNOOP_FULL) begin
      tx_type_d = TX_WRITE_NOSNP;
      state_d   = ST_DBIDRESP_SEND; // non-coherent: the snoop filter is not involved
    end else if (opcode_q == REQ_READ_NO_SNOOP) begin
      tx_type_d = TX_READ_NOSNP;
      state_d   = ST_SN_REQ_SEND;
    end else if (opcode_q == REQ_CLEAN_SHARED || opcode_q == REQ_CLEAN_INVALID || opcode_q == REQ_MAKE_INVALID || opcode_q == REQ_CLEAN_UNIQUE || opcode_q == REQ_MAKE_UNIQUE) begin
      tx_type_d = TX_CMO;
      state_d   = ST_SF_LOOKUP;
    end
    else if (opcode_q == REQ_EVICT) begin
       tx_type_d = TX_CMO; state_d = ST_SF_LOOKUP;
    end else begin
      tx_type_d = TX_READ;
      state_d   = ST_SF_LOOKUP;
      if (opcode_q !=  REQ_READ_SHARED && opcode_q != REQ_READ_CLEAN && opcode_q != REQ_READ_ONCE && opcode_q != REQ_READ_NOT_SHARED_DIRTY && opcode_q != REQ_READ_UNIQUE) begin
      $display("might be a problem: opcode %0d is not handled in TSHR", opcode_q);
      end
    end
  endfunction

  // one snoop filter lookup. A victim with RN-Fs present is back-invalidated
  // before the request itself starts.
  function automatic void st_sf_lookup();
    sf_lk_req_o   = 1'b1;
    sf_lk_alloc_o = sf_alloc_op();
    if (sf_lk_gnt_i) begin
      sf_entry_d = sf_lk_entry_i;
      sf_way_d   = sf_lk_way_i;
      sfpres_d   = sf_lk_pres_i;
      if (sf_lk_victim_i && (sf_lk_victim_pres_i != '0)) begin
        bi_d       = 1'b1;
        bi_addr_d  = sf_lk_victim_addr_i;
        start_snoops(sf_lk_victim_pres_i, 1'b1);
      end else begin
        sfpres_d = sf_lk_pres_i;
        start_main(sf_lk_pres_i);
      end
    end
  endfunction

  // next snoop target = lowest RN-F still in snp_mask_q. None left: decide
  // from what the snoops returned (same decisions as the old single snoop)
  function automatic void st_snoop_pick();
    if (snp_mask_q != '0) begin
      for (int r = NUM_RNF - 1; r >= 0; r--)
        if (snp_mask_q[r]) snp_tgt_d = RNF_IDX_W'(r);
      state_d = ST_SNOOP_SEND;
    end else if (bi_q) begin
      if (dirty_hit_q || ptl_q) state_d = ST_DIRTY_WB_REQ_SEND;   // victim data to memory
      else                      bi_done();
    end else if (tx_type_q == TX_READ) begin
      if (!got_data_q)  state_d = ST_SN_REQ_SEND;                 // nobody had it: memory
      else if (ptl_q)   state_d = ST_DIRTY_WB_REQ_SEND;           // partial data: merge in memory, read back
      else              state_d = ST_RDATA_SEND;
    end else if (dirty_hit_q && (opcode_q != REQ_MAKE_INVALID)) begin
      state_d = ST_DIRTY_WB_REQ_SEND;
    end else if (tx_type_q == TX_CMO) begin
      dirty_hit_d = 1'b0;
      state_d     = ST_COMP_SEND;
    end else begin
      dirty_hit_d = 1'b0;
      state_d     = ST_DBIDRESP_SEND;
    end
  endfunction

  // sending a required snoop to the destination depending of the credit
  //  not depends combinationally on snp_lcrdv due to credit counters
  function automatic void st_snoop_send();
    fill_snp_flit(!got_data_q);
    if (snp_gnt) begin
      state_d = ST_SNOOP_WAIT;
    end
  endfunction

  // the snoop for this transaction; rts = RetToSrc for a read (one copy is
  // enough: after the first data, later snoops of a ReadUnique only invalidate)
  function automatic void fill_snp_flit(logic rts);
    snp_flitv          = Y;
    snp_flitpend       = Y;
    snp_flit.qos       = qos_q;
    snp_flit.tgtid     = RNF_NID[snp_tgt_q];
    snp_flit.srcid     = HNFID;
    snp_flit.txnid     = HN_TXNID;
    snp_flit.fwdnid    = '0;
    snp_flit.fwdtxnid  = '0;
    snp_flit.addr      = bi_q ? bi_addr_q : addr_q;

    if (bi_q) begin
      // back-invalidation: the line leaves every cache, dirty data comes back
      snp_flit.opcode   = SNP_CLEAN_INVALID;
      snp_flit.rettosrc = 1'b0;
    end else if (tx_type_q == TX_READ) begin
      snp_flit.rettosrc = rts;

      if (opcode_q == REQ_READ_UNIQUE) begin
        snp_flit.opcode = SNP_UNIQUE;
      end else if (opcode_q == REQ_READ_CLEAN) begin
        snp_flit.opcode = SNP_CLEAN;
      end else if (opcode_q == REQ_READ_ONCE) begin
        snp_flit.opcode = SNP_ONCE;
      end else if (opcode_q == REQ_READ_NOT_SHARED_DIRTY) begin
        snp_flit.opcode = SNP_NOT_SHARED_DIRTY; // snoopee must not keep SD
      end else begin
        snp_flit.opcode = SNP_SHARED;
      end

    end else if (tx_type_q == TX_CMO) begin
      snp_flit.rettosrc = 1'b0;
      if (opcode_q == REQ_CLEAN_SHARED) begin
        snp_flit.opcode = SNP_CLEAN_SHARED;
      end else if (opcode_q == REQ_CLEAN_INVALID || opcode_q == REQ_CLEAN_UNIQUE) begin
        // CleanUnique SnpCleanInvalid not SnpMakeInvalid The requester may be
        // SC while the other RN-F is SD that dirty data must reach memory 
        snp_flit.opcode = SNP_CLEAN_INVALID;
      end else begin
        snp_flit.opcode = SNP_MAKE_INVALID; // MakeInvalid and MakeUnique
      end
    end else begin
      snp_flit.opcode   = SNP_UNIQUE;
      snp_flit.rettosrc = 1'b0;
    end
  endfunction

  // waiting for the response of the snoop it means that response are waiting from the destination
  // the HN-F routes the RX channels of RN-F snp_port_o (= snp_tgt_q) here; it does
  // not change while the snoop is in flight
  // A finished response (SnpResp, or the last beat of SnpRespData*) clears the
  // target from snp_mask_q; final state I clears its presence bit too. The
  // decision what to do next is taken in ST_SNOOP_PICK when no target is left:
  //   SnpRespDataPtl -> partial bytes: write them to memory, read the line back
  //   dirty data on a write / CMO -> write back first (not for MakeInvalid)
  function automatic void st_snoop_wait();
    logic [BeatCntW-1:0] idx;
    logic                done_one, fin_i, got;
    idx      = BeatCntW'(rx_dat_flit_snp.dataid >> DidShift);
    done_one = 1'b0;
    fin_i    = 1'b0;
    got      = got_data_q;
    if ((rx_dat_flitv_snp == Y) && (rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA ||rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA_PTL ) && (rx_dat_flit_snp.txnid == HN_TXNID)) begin
      data_d[idx] = rx_dat_flit_snp.data;
      be_d[idx]   = rx_dat_flit_snp.be;
      if (rx_dat_flit_snp.resp[2]) begin
        dirty_hit_d = 1'b1;
      end
      if (rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA_PTL) begin
        ptl_d = 1'b1;
      end
      if (!line_last_beat) begin
        // more beats of this dirty line still coming
        beat_cnt_d = beat_cnt_q + 1'b1;
      end else begin
        beat_cnt_d = '0;
        done_one   = 1'b1;
        got        = 1'b1;
        fin_i      = (rx_dat_flit_snp.resp[1:0] == 2'b00);
      end
    end
    else if ((rx_rsp_flitv_snp == Y) && (rx_rsp_flit_snp.opcode == RSP_SNP_RESP) && (rx_rsp_flit_snp.txnid == HN_TXNID)) begin // also correct for snpresp_i
      done_one = 1'b1;
      fin_i    = (rx_rsp_flit_snp.resp[1:0] == 2'b00);
    end
    if (done_one) begin
      got_data_d            = got;
      snp_mask_d[snp_tgt_q] = 1'b0;
      if (!bi_q && fin_i) sfpres_d[snp_tgt_q] = 1'b0;
      // a read that does not take ownership needs only one copy
      if (!bi_q && tx_type_q == TX_READ && opcode_q != REQ_READ_UNIQUE && got) snp_mask_d = '0;
      state_d = ST_SNOOP_PICK;
    end
  endfunction

  // parallel snoops (invalidating ones, see par_op)
  //   issue  : one snoop per grant to snp_tgt_q (the lowest RN-F still in
  //            snp_mask_q), back-to-back; the target moves on in the grant
  //            cycle, so snp_port_o names the next one right away
  //   collect: from every port in the same cycle (the HN-F does not buffer RX):
  //            SnpResp          -> that port is done
  //            SnpRespData(Ptl) -> beats counted per port. The first port that
  //                                sends data is the one whose beats go to
  //                                data_q (every copy holds the current value);
  //                                a dirty (PassDirty) response from any port
  //                                sets dirty_hit
  //            final state I clears the port's presence bit (not for a
  //            back-invalidation, which frees the whole victim entry)
  //   done   : nothing left to issue or to wait for -> ST_SNOOP_PICK with an
  //            empty mask, which takes the same decisions as after the serial walk
  // RetToSrc goes to the first snoop of a ReadUnique only (as the serial walk).
  function automatic void st_snoop_par();
    logic [NUM_RNF-1:0]   issue_m, pend_m;
    logic                 got, own_v;
    logic [RNF_IDX_W-1:0] own;
    logic [BeatCntW-1:0]  idx;
    issue_m = snp_mask_q;
    pend_m  = pend_q;
    got     = got_data_q;
    own_v   = dsrc_v_q;
    own     = dsrc_q;

    if (issue_m != '0) begin
      fill_snp_flit(!rts_used_q);
      if (snp_gnt) begin
        issue_m[snp_tgt_q] = 1'b0;
        pend_m[snp_tgt_q]  = 1'b1;
        if (tx_type_q == TX_READ) rts_used_d = 1'b1;
        snp_tgt_d = lowest(issue_m);
      end
    end

    for (int p = 0; p < NUM_RNF; p++) begin
      if (pend_q[p]) begin                // a response never comes in its grant cycle
        if ((dat_rx_snp_flitv[p] == Y) && (dat_rx_snp_flit[p].txnid == HN_TXNID) &&
            (dat_rx_snp_flit[p].opcode == DAT_SNP_RESP_DATA || dat_rx_snp_flit[p].opcode == DAT_SNP_RESP_DATA_PTL)) begin
          idx = BeatCntW'(dat_rx_snp_flit[p].dataid >> DidShift);
          if (!own_v) begin
            own_v = 1'b1;
            own   = RNF_IDX_W'(p);
          end
          if (own == RNF_IDX_W'(p)) begin
            data_d[idx] = dat_rx_snp_flit[p].data;
            be_d[idx]   = dat_rx_snp_flit[p].be;
            if (dat_rx_snp_flit[p].opcode == DAT_SNP_RESP_DATA_PTL) ptl_d = 1'b1;
          end
          if (dat_rx_snp_flit[p].resp[2]) dirty_hit_d = 1'b1;
          if (pbeat_q[p] != LAST_BEAT) begin
            pbeat_d[p] = pbeat_q[p] + 1'b1;
          end else begin
            pbeat_d[p] = '0;
            pend_m[p]  = 1'b0;
            if (own == RNF_IDX_W'(p)) got = 1'b1;
            if (!bi_q && dat_rx_snp_flit[p].resp[1:0] == 2'b00) sfpres_d[p] = 1'b0;
          end
        end else if ((rsp_rx_snp_flitv[p] == Y) && (rsp_rx_snp_flit[p].txnid == HN_TXNID) &&
                     (rsp_rx_snp_flit[p].opcode == RSP_SNP_RESP)) begin
          pend_m[p] = 1'b0;
          if (!bi_q && rsp_rx_snp_flit[p].resp[1:0] == 2'b00) sfpres_d[p] = 1'b0;
        end
      end
    end

    snp_mask_d = issue_m;
    pend_d     = pend_m;
    got_data_d = got;
    dsrc_v_d   = own_v;
    dsrc_d     = own;
    if (issue_m == '0 && pend_m == '0) state_d = ST_SNOOP_PICK;
  endfunction

  function automatic void st_dirty_wb_req_send();
    sn_req_flitv           = Y;
    sn_req_flitpend        = Y;
    sn_req_flit.qos        = qos_q;
    sn_req_flit.tgtid      = SNFID;
    sn_req_flit.srcid      = HNFID;
    sn_req_flit.txnid      = HN_TXNID;
    sn_req_flit.size       = WIDTH_64;
    sn_req_flit.addr       = bi_q ? {bi_addr_q[AddrWidth-1:LineOffW], LineOffW'(0)}
                                  : {addr_q[AddrWidth-1:LineOffW], LineOffW'(0)};
    sn_req_flit.allowretry = allowretry_q;
    sn_req_flit.pcrdttype  = (allowretry_q == Y) ? 4'h0 : pcrdtype_q;
    sn_req_flit.memattr    = '0;

    // SnpRespDataPtl: only the bytes with BE set are valid a Full write all BE must be 1 is not allowed
    sn_req_flit.opcode     = ptl_q ? REQ_WRITE_NO_SNOOP_PTL : REQ_WRITE_NO_SNOOP_FULL;

    wb_chain_d     = 1'b1;
    sn_comp_seen_d = 1'b0; 
    if (sn_req_gnt) begin
      state_d = ST_DIRTY_WB_RSP_WAIT;
    end
  endfunction

  function automatic void st_dirty_wb_rsp_wait();
    if ((sn_rsp_flitv == Y) && (sn_rsp_flit.txnid == HN_TXNID)) begin
      if (sn_rsp_flit.opcode == RSP_COMP_DBID_RESP) begin
        sn_dbid_d      = sn_rsp_flit.dbid;
        sn_comp_seen_d = 1'b1;
        resperr_d      = resperr_q | sn_rsp_flit.resperr;
        state_d        = ST_DIRTY_WB_DATA_SEND;
      end else if (sn_rsp_flit.opcode == RSP_DBID_RESP) begin
        sn_dbid_d      = sn_rsp_flit.dbid; // keep sn_comp_seen: Comp may have come first
        state_d        = ST_DIRTY_WB_DATA_SEND;
      end else if (sn_rsp_flit.opcode == RSP_RETRY_ACK) begin
        pcrdtype_d  = sn_rsp_flit.pcrdttype;
        retry_ret_d = ST_DIRTY_WB_REQ_SEND;
        state_d     = ST_SN_RETRY_WAIT;
      end
    end
  endfunction

  function automatic void st_dirty_wb_data_send();
    sn_dat_tx_flitv        = Y;
    sn_dat_tx_flitpend     = Y;
    sn_dat_tx_flit.qos     = qos_q;
    sn_dat_tx_flit.tgtid   = SNFID;
    sn_dat_tx_flit.srcid   = HNFID;
    sn_dat_tx_flit.txnid   = sn_dbid_q;
    sn_dat_tx_flit.dbid    = sn_dbid_q;
    sn_dat_tx_flit.dataid  = 2'(beat_cnt_q) << DidShift;
    sn_dat_tx_flit.be       = ptl_q ? be_q[beat_cnt_q] : '1;
    sn_dat_tx_flit.data    = data_q[beat_cnt_q];
    sn_dat_tx_flit.opcode  = DAT_NON_COPY_BACK_WR_DATA;

    if (sn_dat_tx_gnt) begin
      if (beat_cnt_q == LAST_BEAT) begin
        beat_cnt_d   = '0;
        allowretry_d = Y;
        if (sn_comp_seen_q || sn_comp_seen_d) begin
          goto_after_dirty_wb();
        end else begin
          state_d = ST_SN_COMP_WAIT;
        end
      end else begin
        beat_cnt_d = beat_cnt_q + 1'b1;
      end
    end
  endfunction

  function automatic void st_dbidresp_send();
    rsp_tx_flitv       = Y;
    rsp_tx_flitpend    = Y;
    rsp_tx_flit.qos    = qos_q;
    rsp_tx_flit.tgtid  = srcid_q;
    rsp_tx_flit.srcid  = HNFID;
    rsp_tx_flit.txnid  = txnid_q;
    rsp_tx_flit.dbid   = txnid_q;

    if (tx_type_q == TX_WRITE_BACK) begin
      rsp_tx_flit.opcode = RSP_COMP_DBID_RESP;
    end else begin
      rsp_tx_flit.opcode = RSP_DBID_RESP;
    end
    if (rsp_tx_gnt) begin
      state_d = ST_WDATA_WAIT;
    end
  endfunction

  function automatic void st_wdata_wait();
    logic [BeatCntW-1:0] idx;
    idx = BeatCntW'(rx_dat_flit_req.dataid >> DidShift);
    // srcid check added: txnid alone is NOT a unique transaction key
    // across different requesters CHI only guarantees TxnID uniqueness  one requester own outstanding set so if
    // two different RN-Fs happen to pick the same TxnID, matching on
    // txnid alone could let a write-data beat meant for a different
    // requester transaction get accepted here srcid_q was latched
    // from the original request so requiring it to match fixes the issue
    if ((rx_dat_flitv_req == Y) && (rx_dat_flit_req.txnid == txnid_q) && (rx_dat_flit_req.srcid == srcid_q) &&
        ((rx_dat_flit_req.opcode == DAT_NON_COPY_BACK_WR_DATA) || (rx_dat_flit_req.opcode == DAT_COPY_BACK_WR_DATA))) begin
      data_d[idx]  = rx_dat_flit_req.data;
      be_d[idx]    = rx_dat_flit_req.be;
      wdata_resp_d = rx_dat_flit_req.resp;
      if (!req_last_beat) begin
        beat_cnt_d = beat_cnt_q + 1'b1;
      end else begin
        beat_cnt_d = '0;
        if ((tx_type_q == TX_WRITE_BACK) && (rx_dat_flit_req.resp == chi_pkg::RESP_I)) begin
          state_d = expcompack_q ? ST_COMPACK_WAIT : ST_DONE;
        end else begin
          state_d = ST_SN_REQ_SEND;
        end
      end
    end
  endfunction

  function automatic void st_sn_req_send();
    // send req to the slave node
    sn_req_flitv        = Y;
    sn_req_flitpend     = Y;
    sn_req_flit.qos     = qos_q;
    sn_req_flit.tgtid   = SNFID;
    sn_req_flit.srcid   = HNFID;
    sn_req_flit.txnid   = HN_TXNID;
    sn_req_flit.size    = size_q;
    sn_req_flit.addr    = addr_q;
    sn_req_flit.allowretry = allowretry_q;
    sn_req_flit.pcrdttype  = (allowretry_q == Y) ? 4'h0 : pcrdtype_q;
    sn_req_flit.memattr    = '0;

    sn_comp_seen_d = 1'b0; // new SN request: no Comp for it can exist yet
    if (tx_type_q == TX_READ || tx_type_q == TX_READ_NOSNP) begin
      sn_req_flit.opcode = REQ_READ_NO_SNOOP;
    end else if (tx_type_q == TX_WRITE_BACK) begin
      if(opcode_q == REQ_WRITE_BACK_PTL) begin //priority might be wrong i am not sure
        sn_req_flit.opcode = REQ_WRITE_NO_SNOOP_PTL;
      end else begin 
      sn_req_flit.opcode = REQ_WRITE_NO_SNOOP_FULL;
      end
    end else if (tx_type_q == TX_WRITE_NOSNP) begin
      sn_req_flit.opcode = opcode_q;
    end else begin
      sn_req_flit.opcode = REQ_WRITE_NO_SNOOP_PTL;
    end
    if (sn_req_gnt) begin
      state_d = ST_SN_RSP_WAIT;
    end
  endfunction

  function automatic void st_sn_rsp_wait();
    logic [BeatCntW-1:0] idx;
    idx = BeatCntW'(sn_dat_rx_flit.dataid >> DidShift);
    if ((sn_rsp_flitv == Y) && (sn_rsp_flit.opcode == RSP_RETRY_ACK) && (sn_rsp_flit.txnid == HN_TXNID)) begin
      pcrdtype_d  = sn_rsp_flit.pcrdttype;
      retry_ret_d = ST_SN_REQ_SEND;
      state_d     = ST_SN_RETRY_WAIT;
    end else if (tx_type_q == TX_READ || tx_type_q == TX_READ_NOSNP) begin
      // wait for memory response on the data (SN-F is a single fixed
      // target we ourselves issued the txnid to, so txnid-only
      // matching is unambiguous here).
      if ((sn_dat_rx_flitv == Y) && (sn_dat_rx_flit.opcode == DAT_COMP_DATA) && (sn_dat_rx_flit.txnid == HN_TXNID)) begin
        data_d[idx] = sn_dat_rx_flit.data;
        be_d[idx]   = sn_dat_rx_flit.be;
        resperr_d   = resperr_q | sn_dat_rx_flit.resperr;
        if (!req_last_beat) begin
          beat_cnt_d = beat_cnt_q + 1'b1;
        end else begin
          beat_cnt_d   = '0;
          allowretry_d = Y;
          state_d      = ST_RDATA_SEND;
        end
      end
    end else begin
      //wait for memory write ack on RSP channel
      if ((sn_rsp_flitv == Y) && (sn_rsp_flit.txnid == HN_TXNID)) begin
        if (sn_rsp_flit.opcode == RSP_COMP_DBID_RESP) begin
          sn_dbid_d      = sn_rsp_flit.dbid; // Capture memory's DBID
          sn_comp_seen_d = 1'b1;
          resperr_d      = resperr_q | sn_rsp_flit.resperr;
          state_d        = ST_SN_DATA_SEND;
        end else if (sn_rsp_flit.opcode == RSP_DBID_RESP) begin
          sn_dbid_d      = sn_rsp_flit.dbid; // keep sn_comp_seen: Comp may have come first
          state_d        = ST_SN_DATA_SEND;
        end
      end
    end
  endfunction
  //st_sn_data_send write data to SN-F carried TxnID = txnid_q. CHI needs TxnID = the DBID SN-F granted Now sn_dbid_q the dirty-writeback state already did this
  function automatic void st_sn_data_send();
    sn_dat_tx_flitv        = Y;
    sn_dat_tx_flitpend     = Y; //FLITPEND with every FLITV
    sn_dat_tx_flit.qos     = qos_q;
    sn_dat_tx_flit.tgtid   = SNFID;
    sn_dat_tx_flit.srcid   = HNFID;
    sn_dat_tx_flit.txnid   = sn_dbid_q;
    sn_dat_tx_flit.dbid    = sn_dbid_q;
    sn_dat_tx_flit.dataid  = 2'(tx_beat) << DidShift;
    sn_dat_tx_flit.be      = be_q[tx_beat];
    sn_dat_tx_flit.data    = data_q[tx_beat];
    sn_dat_tx_flit.opcode  = DAT_NON_COPY_BACK_WR_DATA;

    if (sn_dat_tx_gnt) begin
      if (req_last_beat) begin
        beat_cnt_d   = '0;
        allowretry_d = Y;
        if (sn_comp_seen_q || sn_comp_seen_d) begin
          goto_after_sn_write();
        end else begin
          state_d = ST_SN_COMP_WAIT;
        end
      end else begin
        beat_cnt_d = beat_cnt_q + 1'b1;
      end
    end
  endfunction

  function automatic void st_sn_comp_wait();
    //  comp latched by fsm_watch_sn_comp()
    if (sn_comp_seen_q || sn_comp_seen_d) begin
      if (wb_chain_q) begin
        goto_after_dirty_wb();
      end else begin
        goto_after_sn_write();
      end
    end
  endfunction
  // RetryAck -> ST_SN_RETRY_WAIT -> get a P-credit of that PCrdType from HN-F's pool -> resend the same
  //  request with AllowRetry=N and that PCrdType  it loops back into the retry_ret_q  for simplicity
  function automatic void st_sn_retry_wait();
    pcrd_req_o  = 1'b1;
    pcrd_type_o = pcrdtype_q;
    if (pcrd_gnt_i) begin
      allowretry_d = N;
      state_d      = retry_ret_q;
    end
  endfunction

  function automatic void st_rdata_send();
    dat_tx_flitv        = Y;
    dat_tx_flitpend     = Y; 
    dat_tx_flit.qos     = qos_q;
    dat_tx_flit.tgtid   = srcid_q;
    dat_tx_flit.srcid   = HNFID;
    dat_tx_flit.txnid   = txnid_q;
    dat_tx_flit.homenid = HNFID;
    dat_tx_flit.dbid    = txnid_q;
    dat_tx_flit.opcode  = DAT_COMP_DATA;
    dat_tx_flit.dataid  = 2'(tx_beat) << DidShift;
    dat_tx_flit.be      = be_q[tx_beat];
    dat_tx_flit.data    = data_q[tx_beat];
    dat_tx_flit.resperr = resperr_q;
    if(opcode_q == REQ_READ_UNIQUE) begin
        dat_tx_flit.resp = chi_pkg::RESP_UC;
    end else if (opcode_q == REQ_READ_ONCE || opcode_q == REQ_READ_NO_SNOOP) begin
        dat_tx_flit.resp = chi_pkg::RESP_I;
    end else begin
        dat_tx_flit.resp = chi_pkg::RESP_SC;
    end
    if (dat_tx_gnt) begin
      if (req_last_beat) begin
        beat_cnt_d = '0;
        if (expcompack_q) begin
          state_d = ST_COMPACK_WAIT;
        end else if (dirty_hit_q) begin
          state_d = ST_DIRTY_WB_REQ_SEND;
        end else begin
          state_d = ST_DONE;
        end
      end else begin
        beat_cnt_d = beat_cnt_q + 1'b1;
      end
    end
  endfunction

  function automatic void st_comp_send();
    rsp_tx_flitv        = Y;
    rsp_tx_flitpend     = Y;
    rsp_tx_flit.qos     = qos_q;
    rsp_tx_flit.tgtid   = srcid_q;
    rsp_tx_flit.srcid   = HNFID;
    rsp_tx_flit.txnid   = txnid_q;
    rsp_tx_flit.dbid    = txnid_q;
    rsp_tx_flit.resperr = resperr_q;
    rsp_tx_flit.opcode  = RSP_COMP;
    rsp_tx_flit.resp = (opcode_q == REQ_CLEAN_UNIQUE || opcode_q == REQ_MAKE_UNIQUE) ? chi_pkg::RESP_UC : chi_pkg::RESP_I;
    if (rsp_tx_gnt) begin
      state_d = expcompack_q ? ST_COMPACK_WAIT : ST_DONE;
    end
  endfunction

  function automatic void st_compack_wait();
    if (compack_seen_q || compack_seen_d) begin

      if (dirty_hit_q == 1'b1) begin
        // served the core now go push the dirty data to memory
        state_d = ST_DIRTY_WB_REQ_SEND;
      end else begin
        // normal read no writeback
        state_d = ST_DONE;
      end

    end
  endfunction

  // write the final presence vector of this line back to the snoop filter.
  // Requester bit: set when it caches the line now, cleared when it gave it up
  // (WriteBack / Evict). Everything else keeps its bit (a stale bit is safe).
  function automatic void st_done();
    logic [NUM_RNF-1:0] p;
    p = sfpres_q;
    if (sf_alloc_op()) p = p | req_bit;
    if (opcode_q == REQ_EVICT || opcode_q == REQ_WRITE_BACK_FULL || opcode_q == REQ_WRITE_BACK_PTL)
      p = p & ~req_bit;
    sf_wr_o      = sf_entry_q;
    sf_wr_way_o  = sf_way_q;
    sf_wr_pres_o = p;
    state_d = ST_IDLE;
  endfunction

  always_comb begin
    fsm_defaults();
    fsm_select_inputs();
    fsm_watch_compack();
    fsm_watch_sn_comp();

    case (state_q)
      ST_IDLE:               st_idle();
      ST_ALLOC:              st_alloc();
      ST_HAZARD_WAIT:        st_hazard_wait();
      ST_SF_LOOKUP:          st_sf_lookup();
      ST_SNOOP_PICK:         st_snoop_pick();
      ST_SNOOP_SEND:         st_snoop_send();
      ST_SNOOP_WAIT:         st_snoop_wait();
      ST_SNOOP_PAR:          st_snoop_par();
      ST_DIRTY_WB_REQ_SEND:  st_dirty_wb_req_send();
      ST_DIRTY_WB_RSP_WAIT:  st_dirty_wb_rsp_wait();
      ST_DIRTY_WB_DATA_SEND: st_dirty_wb_data_send();
      ST_DBIDRESP_SEND:      st_dbidresp_send();
      ST_WDATA_WAIT:         st_wdata_wait();
      ST_SN_REQ_SEND:        st_sn_req_send();
      ST_SN_RSP_WAIT:        st_sn_rsp_wait();
      ST_SN_DATA_SEND:       st_sn_data_send();
      ST_SN_COMP_WAIT:       st_sn_comp_wait();
      ST_SN_RETRY_WAIT:      st_sn_retry_wait();
      ST_RDATA_SEND:         st_rdata_send();
      ST_COMP_SEND:          st_comp_send();
      ST_COMPACK_WAIT:       st_compack_wait();
      ST_DONE:               st_done();
      default:               state_d = ST_IDLE;
    endcase
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      state_q       <= ST_IDLE;

      tx_type_q     <= TX_READ;
      addr_q        <= '0;
      srcid_q       <= '0;
      txnid_q       <= '0;
      opcode_q      <= req_opcode_e'(8'h00);
      size_q        <= WIDTH_1;
      qos_q         <= '0;
      expcompack_q  <= 1'b0;
      for (int i = 0; i < NumBeats; i++) begin
        data_q[i] <= '0;
        be_q[i]   <= '0;
      end
      beat_cnt_q     <= '0;
      sn_dbid_q      <= '0;
      dirty_hit_q    <= 0;
      resperr_q      <= '0;
      wdata_resp_q   <= '0;
      sn_comp_seen_q <= 1'b0;
      compack_seen_q <= 1'b0;
      wb_chain_q     <= 1'b0;
      ptl_q          <= 1'b0;
      allowretry_q   <= Y;
      pcrdtype_q     <= '0;
      retry_ret_q    <= ST_IDLE;
      sf_entry_q     <= 1'b0;
      sf_way_q       <= '0;
      sfpres_q       <= '0;
      snp_mask_q     <= '0;
      snp_tgt_q      <= '0;
      got_data_q     <= 1'b0;
      bi_q           <= 1'b0;
      bi_addr_q      <= '0;
      pend_q         <= '0;
      for (int p = 0; p < NUM_RNF; p++) pbeat_q[p] <= '0;
      dsrc_v_q       <= 1'b0;
      dsrc_q         <= '0;
      rts_used_q     <= 1'b0;
    end else begin
      sf_entry_q     <= sf_entry_d;
      sf_way_q       <= sf_way_d;
      sfpres_q       <= sfpres_d;
      snp_mask_q     <= snp_mask_d;
      snp_tgt_q      <= snp_tgt_d;
      got_data_q     <= got_data_d;
      bi_q           <= bi_d;
      bi_addr_q      <= bi_addr_d;
      pend_q         <= pend_d;
      for (int p = 0; p < NUM_RNF; p++) pbeat_q[p] <= pbeat_d[p];
      dsrc_v_q       <= dsrc_v_d;
      dsrc_q         <= dsrc_d;
      rts_used_q     <= rts_used_d;
      state_q       <= state_d;
      if (tx_type_d == TX_READ || tx_type_d == TX_WRITE_UNIQUE || tx_type_d == TX_WRITE_BACK ||
          tx_type_d == TX_READ_NOSNP || tx_type_d == TX_WRITE_NOSNP || tx_type_d == TX_CMO) begin
         tx_type_q <= tx_type_d;
      end else begin
         $display("[%0t] TSHR Module: unsupported tx_type  %s", $time, tx_type_d.name());
         
         tx_type_q <= tx_type_q; //  dont send garbage
      end
      dirty_hit_q   <= dirty_hit_d;
      addr_q        <= addr_d;
      srcid_q       <= srcid_d;
      txnid_q       <= txnid_d;
      opcode_q      <= opcode_d;
      size_q        <= size_d;
      qos_q         <= qos_d;
      expcompack_q  <= expcompack_d;
      for (int i = 0; i < NumBeats; i++) begin
        data_q[i] <= data_d[i];
        be_q[i]   <= be_d[i];
      end
      beat_cnt_q     <= beat_cnt_d;
      sn_dbid_q      <= sn_dbid_d;
      resperr_q      <= resperr_d;
      wdata_resp_q   <= wdata_resp_d;
      sn_comp_seen_q <= sn_comp_seen_d;
      compack_seen_q <= compack_seen_d;
      wb_chain_q     <= wb_chain_d;
      ptl_q          <= ptl_d;
      allowretry_q   <= allowretry_d;
      pcrdtype_q     <= pcrdtype_d;
      retry_ret_q    <= retry_ret_d;
    end
  end
  assign TxnID_o = txnid_q;
  assign SrcID_o = srcid_q;
  assign Addr_o  = addr_q;
  assign snp_port_o = snp_tgt_q;
  assign bi_o       = bi_q;
  assign bi_addr_o  = bi_addr_q;

endmodule
