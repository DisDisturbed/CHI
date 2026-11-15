
// TSHR - transaction snoop handling register
//
// Role in the HN-F node
// this is the simplest possible or lets say "dumbest" tracker module i could think of
// it only knows one transaction at time so HN-F node should be handle it
// it has no idea about the interfaceses or RN-F port counts
// it only see 2 things one requester and one snoop target (snoop target can be SN-F dont forget this)
// top module should instantiate this module multiple times to track multiple outstanding requests at one
// HN-F also need to track which TSHR is free to give request to them
// HN-F also need to know which port RN-F plugs
// HN-F also need its own snoop filter
// HN-F also need address hazard detection to detect which tracker is safe to give
//
// NONE OF THAT ROUTING AND ARBITRATION LIVES INSIDE THIS FILE.


module TSHR

 import chi_pkg::*;
 import common_pkg::*;
 import tshr_flit_pkg::*;
 #(
  parameter int AddrWidth      = 39,
  parameter int DataWidth      = 128,
  // total data moved per transaction as bits .
  // must be an integer multiple of DataWidth. NumBeats = CacheLineWidth /
  parameter int CacheLineWidth = 512,
  parameter int TRK_ID         = 0,
  parameter chi_pkg::node_id_e HNFID = 7'h40,
  parameter chi_pkg::node_id_e SNFID = 7'h00
) (
  input  wire clk,
  input  wire resetn,
  output logic ready_o,



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

  // ---- RSP from the snoop target (rx side): SnpResp
  input  common_pkg::yn_status_e rsp_rx_snp_flitv,
  input  common_pkg::yn_status_e rsp_rx_snp_flitpend,
  input  local_rsp_flit_t        rsp_rx_snp_flit,

  // ---- DAT to the requester (tx side): CompData proxy forward
  output common_pkg::yn_status_e dat_tx_flitv,
  output common_pkg::yn_status_e dat_tx_flitpend,
  output local_dat_flit_t        dat_tx_flit,
  input  logic                   dat_tx_gnt,

  // ---- DAT from the requester (rx side): write-data
  input  common_pkg::yn_status_e dat_rx_req_flitv,
  input  common_pkg::yn_status_e dat_rx_req_flitpend,
  input  local_dat_flit_t        dat_rx_req_flit,

  // ---- DAT from the snoop target (rx side): snoop-resp-data
  input  common_pkg::yn_status_e dat_rx_snp_flitv,
  input  common_pkg::yn_status_e dat_rx_snp_flitpend,
  input  local_dat_flit_t        dat_rx_snp_flit,

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
    ST_HAZARD_WAIT
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
   // the basic sequence of the fsm is the following:
   // data flit comes to the HN-F node from RN-F0. It holds the addr and other useful things inside the tracker.
   // after it accepts the flit, it goes to RN-F1 for snoop requests and waits for their response.
   // depending on the RN-F1's response (and if it's a read or write), it either goes to ST_DBIDRESP_SEND or ST_RDATA_SEND.

   // path from ST_DBIDRESP_SEND (The Write Flow - WriteUniquePtl):
   // 1. ST_DBIDRESP_SEND: Send a DBIDResp back to the original requester (RN-F0), telling it "I am ready for your write data."
   // 2. ST_WDATA_WAIT: Wait for the requester to send the actual write data on the DAT channel (now: NumBeats beats).
   // 3. ST_SN_REQ_SEND: Forward a request (WriteNoSnpPtl) down to the memory (SN-F).
   // 4. ST_SN_RSP_WAIT: Wait for the memory to reply with CompDBIDResp, meaning memory is ready for the data.
   // 5. ST_SN_DATA_SEND: Push the captured write data down to the memory (now: NumBeats beats).
   // 6. ST_COMP_SEND: Send a final Completion (Comp) to the original requester.
   // 7. ST_COMPACK_WAIT: Wait for the requester to acknowledge the completion (CompAck).
   // 8. ST_DONE: Clean up and return to IDLE.


   // path from ST_RDATA_SEND (The Read Flow - ReadUnique):
   // 1. ST_RDATA_SEND: We already got the requested data from snooping RN-F1. Forward this data directly to the original requester (RN-F0) as CompData (now: NumBeats beats).
   // 2. ( skipped Memory access steps for this flow).
   // 3. ST_COMPACK_WAIT: Wait for the requester to acknowledge receipt of the data (CompAck).
   // 4. ST_DONE: Clean up and return to IDLE.

   assign ready_o = (state_q == ST_IDLE);
   logic dirty_hit_q, dirty_hit_d;

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

    // default all output ports to inactive
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

  // requester's CompAck comes in while we're waiting on it in
  // ST_COMPACK_WAIT; the snoop target's SnpResp comes in while we're
  // waiting on it in ST_SNOOP_WAIT. Each is now its own fixed port, so
  // this is just a plain state-based pick, no index needed.
  function automatic void fsm_select_inputs();
    rx_rsp_flitv_snp = (rsp_rx_snp_flitv == Y);
    rx_rsp_flit_snp  = rsp_rx_snp_flit;

    rx_rsp_flitv_req = (rsp_rx_req_flitv == Y);
    rx_rsp_flit_req  = rsp_rx_req_flit;

    rx_dat_flitv_snp = (dat_rx_snp_flitv == Y);
    rx_dat_flit_snp  = dat_rx_snp_flit;

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

  // CHI does not order the SN-F's separate Comp against DBIDResp or the
  // write data: it may come before DBIDResp, between DBIDResp and the data,
  // or after the data. So it is caught here in every state, not only in
  // ST_SN_COMP_WAIT. HN_TXNID is ours alone, and sn_comp_seen is cleared
  // when the SN request is (re)sent, so a Comp can only belong to the write
  // that is outstanding now.
  function automatic void fsm_watch_sn_comp();
    if ((sn_rsp_flitv == Y) && (sn_rsp_flit.opcode == RSP_COMP) && (sn_rsp_flit.txnid == HN_TXNID)) begin
      sn_comp_seen_d = 1'b1;
      resperr_d      = resperr_q | sn_rsp_flit.resperr;
    end
  endfunction

  function automatic void goto_after_dirty_wb();
    wb_chain_d  = 1'b0;
    dirty_hit_d = 1'b0;
    ptl_d       = 1'b0;
    if (tx_type_q == TX_READ && ptl_q) begin
      // partial snoop data is in memory now: read the whole line from there
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

  function automatic void st_idle();
    dirty_hit_d    = 0;
    beat_cnt_d     = '0;
    resperr_d      = '0;
    wdata_resp_d   = '0;
    sn_comp_seen_d = 1'b0;
    compack_seen_d = 1'b0;
    wb_chain_d     = 1'b0;
    ptl_d          = 1'b0;
    allowretry_d   = Y;
    pcrdtype_d     = '0;
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

  function automatic void st_hazard_wait();
    if (!hazard_i) begin
      state_d = ST_ALLOC;
    end
  endfunction

  function automatic void st_alloc(); // allocate a place in the TSHR
    if (hazard_i) begin
      state_d = ST_HAZARD_WAIT;
    end else if (opcode_q == REQ_WRITE_BACK_FULL || opcode_q == REQ_WRITE_CLEAN_FULL || opcode_q == REQ_WRITE_BACK_PTL) begin
      tx_type_d = TX_WRITE_BACK;
      state_d   = ST_DBIDRESP_SEND; // Skip snoop
    end else if (opcode_q == REQ_WRITE_UNIQUE_PTL || opcode_q == REQ_WRITE_UNIQUE_FULL) begin
      tx_type_d = TX_WRITE_UNIQUE;
      state_d   = ST_SNOOP_SEND;
    end else if (opcode_q == REQ_WRITE_NO_SNOOP_PTL || opcode_q == REQ_WRITE_NO_SNOOP_FULL) begin
      tx_type_d = TX_WRITE_NOSNP;
      state_d   = ST_DBIDRESP_SEND;
    end else if (opcode_q == REQ_READ_NO_SNOOP) begin
      tx_type_d = TX_READ_NOSNP;
      state_d   = ST_SN_REQ_SEND;
    end else if (opcode_q == REQ_CLEAN_SHARED || opcode_q == REQ_CLEAN_INVALID || opcode_q == REQ_MAKE_INVALID || opcode_q == REQ_CLEAN_UNIQUE || opcode_q == REQ_MAKE_UNIQUE) begin
      tx_type_d = TX_CMO;
      state_d   = ST_SNOOP_SEND;
    end
    else if (opcode_q == REQ_EVICT) begin
       tx_type_d = TX_CMO; state_d = ST_COMP_SEND; 
    end else begin
      tx_type_d = TX_READ;
      state_d   = ST_SNOOP_SEND;
      if (opcode_q !=  REQ_READ_SHARED && opcode_q != REQ_READ_CLEAN && opcode_q != REQ_READ_ONCE && opcode_q != REQ_READ_NOT_SHARED_DIRTY && opcode_q != REQ_READ_UNIQUE) begin
      $display("might be a problem: opcode %0d is not handled in TSHR", opcode_q);
      end
    end
  endfunction

  // sending a required snoop to the destination depending of the credit
  //  not depends combinationally on snp_lcrdv.
  function automatic void st_snoop_send();
    snp_flitv          = Y;
    snp_flitpend       = Y;
    snp_flit.qos       = qos_q;
    snp_flit.srcid     = HNFID;
    snp_flit.txnid     = HN_TXNID;
    snp_flit.fwdnid    = '0;
    snp_flit.fwdtxnid  = '0;
    snp_flit.addr      = addr_q;

    if (tx_type_q == TX_READ) begin
      snp_flit.rettosrc = 1'b1;

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
        // CleanUnique: SnpCleanInvalid, not SnpMakeInvalid. The requester may be
        // SC while the other RN-F is SD; that dirty data must reach memory.
        snp_flit.opcode = SNP_CLEAN_INVALID;
      end else begin
        snp_flit.opcode = SNP_MAKE_INVALID; // MakeInvalid / MakeUnique
      end
    end else begin
      snp_flit.opcode   = SNP_UNIQUE;
      snp_flit.rettosrc = 1'b0;
    end

    if (snp_gnt) begin
      state_d = ST_SNOOP_WAIT;
    end
  endfunction

  // waiting for the response of the snoop it means that response are waiting from the destination
  // snoop-target identity is implicitly guaranteed by the crossbar's
  // fixed physical wiring for the lifetime
  function automatic void st_snoop_wait();
    logic [BeatCntW-1:0] idx;
    logic                dirty;
    idx   = BeatCntW'(rx_dat_flit_snp.dataid >> DidShift);
    dirty = dirty_hit_q;
    if ((rx_dat_flitv_snp == Y) && (rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA ||rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA_PTL ) && (rx_dat_flit_snp.txnid == HN_TXNID)) begin
      data_d[idx] = rx_dat_flit_snp.data;
      be_d[idx]   = rx_dat_flit_snp.be;
      if (rx_dat_flit_snp.resp[2]) begin
        dirty       = 1'b1;
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
        if (tx_type_q == TX_READ && (ptl_q || ptl_d)) begin
          // SnpRespDataPtl holds only some bytes of the line, so it cannot be
          // forwarded as CompData. Write those bytes to memory (WriteNoSnpPtl),
          // then read the merged line back (see goto_after_dirty_wb).
          // SnpRespDataPtl_UD (SnpOnce, owner stays UDP) has no PD bit but
          // takes the same path: memory gets bytes the owner still holds dirty.
          state_d = ST_DIRTY_WB_REQ_SEND;
        end else if (tx_type_q == TX_READ) begin
          state_d = ST_RDATA_SEND;
        end else if (dirty && (opcode_q != REQ_MAKE_INVALID)) begin
          state_d = ST_DIRTY_WB_REQ_SEND;
        end else if (tx_type_q == TX_CMO) begin
          dirty_hit_d = 1'b0;
          state_d     = ST_COMP_SEND;
        end else begin
          dirty_hit_d = 1'b0;
          state_d     = ST_DBIDRESP_SEND;
        end
      end
    end
    else if ((rx_rsp_flitv_snp == Y) && (rx_rsp_flit_snp.opcode == RSP_SNP_RESP) && (rx_rsp_flit_snp.txnid == HN_TXNID)) begin // also correct for snpresp_i
      if (tx_type_q == TX_READ) begin
        // read miss: target doesn't have the line
        state_d = ST_SN_REQ_SEND;
      end else if (tx_type_q == TX_CMO) begin
        state_d = ST_COMP_SEND;
      
      end else begin
        // wait for invalidate ack
        state_d = ST_DBIDRESP_SEND;
      end
    end
  endfunction

  function automatic void st_dirty_wb_req_send();
    sn_req_flitv           = Y;
    sn_req_flitpend        = Y;
    sn_req_flit.qos        = qos_q;
    sn_req_flit.tgtid      = SNFID;
    sn_req_flit.srcid      = HNFID;
    sn_req_flit.txnid      = HN_TXNID;
    sn_req_flit.size       = WIDTH_64;
    sn_req_flit.addr       = {addr_q[AddrWidth-1:LineOffW], LineOffW'(0)};
    sn_req_flit.allowretry = allowretry_q;
    sn_req_flit.pcrdttype  = (allowretry_q == Y) ? 4'h0 : pcrdtype_q;
    sn_req_flit.memattr    = '0;

    // SnpRespDataPtl: only the bytes with BE set are valid, so a Full write
    // (all BE must be 1) is not allowed
    sn_req_flit.opcode     = ptl_q ? REQ_WRITE_NO_SNOOP_PTL : REQ_WRITE_NO_SNOOP_FULL;

    wb_chain_d     = 1'b1;
    sn_comp_seen_d = 1'b0; // new SN write: no Comp for it can exist yet
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
    // across different requesters (CHI only guarantees TxnID
    // uniqueness *within* one requester's own outstanding set), so if
    // two different RN-Fs happen to pick the same TxnID, matching on
    // txnid alone could let a write-data beat meant for a different
    // requester's transaction get accepted here. srcid_q was latched
    // from the original request, so requiring it to match fixes the issue
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

  function automatic void st_sn_data_send();
    sn_dat_tx_flitv        = Y;
    sn_dat_tx_flitpend     = Y; // link layer: FLITPEND with every FLITV, last beat included
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
    // the Comp itself is latched by fsm_watch_sn_comp()
    if (sn_comp_seen_q || sn_comp_seen_d) begin
      if (wb_chain_q) begin
        goto_after_dirty_wb();
      end else begin
        goto_after_sn_write();
      end
    end
  endfunction

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
    dat_tx_flitpend     = Y; // link layer: FLITPEND with every FLITV, last beat included
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
        // served the core, now go push the dirty data to memory
        state_d = ST_DIRTY_WB_REQ_SEND;
      end else begin
        // normal read, no writeback needed
        state_d = ST_DONE;
      end

    end
  endfunction

  function automatic void st_done();
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
      ST_SNOOP_SEND:         st_snoop_send();
      ST_SNOOP_WAIT:         st_snoop_wait();
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
    end else begin
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

endmodule
