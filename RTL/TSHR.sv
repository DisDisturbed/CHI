
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
  // depth of the registered credit bank on each TX channel. 2 gives a
  // little pipelining headroom over the historical single-credit
  // behavior; raise it if the surrounding fabric can usefully bank more.
  parameter int TxCreditMax    = 2,
  parameter chi_pkg::node_id_e HNFID = 7'h40,
  parameter chi_pkg::node_id_e SNFID = 7'h00
) (
  input  wire clk,
  input  wire resetn,
  output logic ready_o,
 
 
 
  output logic [11:0] TxnID_o,
  output node_id_e SrcID_o, 

  
  // ---- REQ from the requester (rx side): the original request ----
  input  common_pkg::yn_status_e req_flitv,
  input  common_pkg::yn_status_e req_flitpend,
  input  local_req_flit_t        req_flit,
  output logic                   req_lcrdv,

  // ---- SNP to the snoop target (tx side): the snoop this tracker issues
  output common_pkg::yn_status_e snp_flitv,
  output common_pkg::yn_status_e snp_flitpend,
  output local_snp_flit_t        snp_flit,
  input  logic                   snp_lcrdv,

  // ---- RSP to the requester (tx side): DBIDResp / CompDBIDResp / Comp
  output common_pkg::yn_status_e rsp_tx_flitv,
  output common_pkg::yn_status_e rsp_tx_flitpend,
  output local_rsp_flit_t        rsp_tx_flit,
  input  logic                   rsp_tx_lcrdv,

  // ---- RSP from the requester (rx side): CompAck
  input  common_pkg::yn_status_e rsp_rx_req_flitv,
  input  common_pkg::yn_status_e rsp_rx_req_flitpend,
  input  local_rsp_flit_t        rsp_rx_req_flit,
  output logic                   rsp_rx_req_lcrdv,

  // ---- RSP from the snoop target (rx side): SnpResp
  input  common_pkg::yn_status_e rsp_rx_snp_flitv,
  input  common_pkg::yn_status_e rsp_rx_snp_flitpend,
  input  local_rsp_flit_t        rsp_rx_snp_flit,
  output logic                   rsp_rx_snp_lcrdv,

  // ---- DAT to the requester (tx side): CompData proxy forward
  output common_pkg::yn_status_e dat_tx_flitv,
  output common_pkg::yn_status_e dat_tx_flitpend,
  output local_dat_flit_t        dat_tx_flit,
  input  logic                   dat_tx_lcrdv,

  // ---- DAT from the requester (rx side): write-data
  input  common_pkg::yn_status_e dat_rx_req_flitv,
  input  common_pkg::yn_status_e dat_rx_req_flitpend,
  input  local_dat_flit_t        dat_rx_req_flit,
  output logic                   dat_rx_req_lcrdv,

  // ---- DAT from the snoop target (rx side): snoop-resp-data
  input  common_pkg::yn_status_e dat_rx_snp_flitv,
  input  common_pkg::yn_status_e dat_rx_snp_flitpend,
  input  local_dat_flit_t        dat_rx_snp_flit,
  output logic                   dat_rx_snp_lcrdv,

  // ---- SN-F side (already scalar, unchanged) ----
  output common_pkg::yn_status_e sn_req_flitv,
  output common_pkg::yn_status_e sn_req_flitpend,
  output local_req_flit_t        sn_req_flit,
  input  logic                   sn_req_lcrdv,

  input  common_pkg::yn_status_e sn_rsp_flitv,
  input  common_pkg::yn_status_e sn_rsp_flitpend,
  input  local_rsp_flit_t        sn_rsp_flit,
  output logic                   sn_rsp_lcrdv,

  output common_pkg::yn_status_e sn_dat_tx_flitv,
  output common_pkg::yn_status_e sn_dat_tx_flitpend,
  output local_dat_flit_t        sn_dat_tx_flit,
  input  logic                   sn_dat_tx_lcrdv,

  input  common_pkg::yn_status_e sn_dat_rx_flitv,
  input  common_pkg::yn_status_e sn_dat_rx_flitpend,
  input  local_dat_flit_t        sn_dat_rx_flit,
  output logic                   sn_dat_rx_lcrdv
);
 
  localparam int NumBeats  = CacheLineWidth / DataWidth;
  localparam int BeatCntW  = (NumBeats <= 1) ? 1 : $clog2(NumBeats);
  localparam logic [BeatCntW-1:0] LAST_BEAT = BeatCntW'(NumBeats-1);

  initial begin // simple beat tracker it is not parameterized  but this will give error if someone change it late on
    if (CacheLineWidth % DataWidth != 0) begin
      $fatal(1, "TSHR cacheline widht (%0d) must be an integer multiple of DataWidth (%0d)",
             CacheLineWidth, DataWidth);
    end
    if (NumBeats > 4) begin
      $fatal(1, "TSHR beat number (%0d) exceeds 4 - DataID is only 2 bits ",
             NumBeats);
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
    ST_DIRTY_WB_DATA_SEND
  } state_e;

  state_e state_q, state_d;

  logic [AddrWidth-1:0]  addr_q, addr_d;
  node_id_e              srcid_q, srcid_d;
  logic [11:0]           txnid_q, txnid_d;
  req_opcode_e            opcode_q, opcode_d;
  width_e                 size_q, size_d;
  logic [3:0]             qos_q, qos_d;

  // beat data storage
  logic [DataWidth-1:0]     data_q [NumBeats], data_d [NumBeats];
  logic [(DataWidth/8)-1:0] be_q   [NumBeats], be_d   [NumBeats];
  logic [BeatCntW-1:0]      beat_cnt_q, beat_cnt_d;

  logic [11:0]              sn_dbid_q, sn_dbid_d;

  // RX credit grants
  // these stay unconditional: this tracker never needs to backpressure a
  // beat once it has committed to receiving one it just latches straight
  // into data_d[] and be_d[] combinationally, so there's nothing to gate.
  // sizing/pacing of what's allowed to arrive is the external Credit
  assign req_lcrdv        = (state_q == ST_IDLE);
  assign rsp_rx_req_lcrdv = 1'b1;
  assign rsp_rx_snp_lcrdv = 1'b1;
  assign dat_rx_req_lcrdv = 1'b1;
  assign dat_rx_snp_lcrdv = 1'b1;
  assign sn_rsp_lcrdv     = 1'b1;
  assign sn_dat_rx_lcrdv  = 1'b1;

  logic snp_flitv_b, rsp_tx_flitv_b, dat_tx_flitv_b, sn_req_flitv_b, sn_dat_tx_flitv_b;
  logic snp_credit_have, rsp_tx_credit_have, dat_tx_credit_have,
        sn_req_credit_have, sn_dat_tx_credit_have;

  assign snp_flitv_b       = (snp_flitv       == Y);
  assign rsp_tx_flitv_b    = (rsp_tx_flitv    == Y);
  assign dat_tx_flitv_b    = (dat_tx_flitv    == Y);
  assign sn_req_flitv_b    = (sn_req_flitv    == Y);
  assign sn_dat_tx_flitv_b = (sn_dat_tx_flitv == Y);
  
  
  /// this modules exist due to the possible combinational loop 
  // on chi link credit must be tranmitters previous banked credit only 
  // if these modules doesnt register the credit any arbiter that above this module 
  // gives lcrdv in the same cycle then flitv depends on lcrdv 
  // any reasonable arbiter must need to grant lcrdv based on the flitv which is the current requester
  // then it will create a combinational loop 
  // this modules breaks that loop with having credit counters for every channel 
  // soo transmitter van gate the flitv without reading this cycles lcrdv without combinational loop
  //
  // incr = uper module pulsed lcrdv this cycle   -> accept one credit for later use
  // decr = we are spending one credit right now
  // it is just simple counter to track if we have credit or not 
  credit_cntr #(.MaxCredits(TxCreditMax)) u_snp_cred (
    .clk(clk),
    .resetn(resetn),
    .incr(snp_lcrdv), 
    .decr(snp_flitv_b),
    .have_credit(snp_credit_have)
  );
  credit_cntr #(.MaxCredits(TxCreditMax)) u_rsp_tx_cred (
    .clk(clk),
     .resetn(resetn),
    .incr(rsp_tx_lcrdv),
    .decr(rsp_tx_flitv_b),
    .have_credit(rsp_tx_credit_have)
  );
  credit_cntr #(.MaxCredits(TxCreditMax)) u_dat_tx_cred (
    .clk(clk),
    .resetn(resetn),
    .incr(dat_tx_lcrdv),
    .decr(dat_tx_flitv_b),
     .have_credit(dat_tx_credit_have)
  );
  credit_cntr #(.MaxCredits(TxCreditMax)) u_sn_req_cred (
    .clk(clk),
    .resetn(resetn),
    .incr(sn_req_lcrdv),
    .decr(sn_req_flitv_b),
    .have_credit(sn_req_credit_have)
  );
  credit_cntr #(.MaxCredits(TxCreditMax)) u_sn_dat_tx_cred (
    .clk(clk), 
   .resetn(resetn),
   .incr(sn_dat_tx_lcrdv), 
   .decr(sn_dat_tx_flitv_b), 
   .have_credit(sn_dat_tx_credit_have)
  );

     // simple decoding block to understand where to go
     // this simplify the if else blocks and makes us only check for tx_type
     // it is latched on the alloc state
    typedef enum logic [1:0] {
        TX_READ,
        TX_WRITE_UNIQUE,
        TX_WRITE_BACK
      } tx_type_e;
      
      tx_type_e tx_type_q, tx_type_d;

    logic            rx_rsp_flitv;
    local_rsp_flit_t rx_rsp_flit;
    
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
  always_comb begin
    state_d       = state_q;
    tx_type_d = tx_type_q; 
    addr_d        = addr_q;
    srcid_d       = srcid_q;
    txnid_d       = txnid_q;
    opcode_d      = opcode_q;
    size_d        = size_q;
    qos_d         = qos_q;
    for (int i = 0; i < NumBeats; i++) begin
      data_d[i] = data_q[i];
      be_d[i]   = be_q[i];
    end
    beat_cnt_d    = beat_cnt_q;
    sn_dbid_d     = sn_dbid_q;
    dirty_hit_d   = dirty_hit_q;

    // default all output ports to inactive
    snp_flitv    = N; snp_flitpend    = N; snp_flit    = '0;
    rsp_tx_flitv = N; rsp_tx_flitpend = N; rsp_tx_flit = '0;
    dat_tx_flitv = N; dat_tx_flitpend = N; dat_tx_flit = '0;

    sn_req_flitv    = N;
    sn_req_flitpend = N;
    sn_req_flit     = '0;
    sn_dat_tx_flitv    = N;
    sn_dat_tx_flitpend = N;
    sn_dat_tx_flit     = '0;

    // requester's CompAck comes in while we're waiting on it in
    // ST_COMPACK_WAIT; the snoop target's SnpResp comes in while we're
    // waiting on it in ST_SNOOP_WAIT. Each is now its own fixed port, so
    // this is just a plain state-based pick, no index needed.
    rx_rsp_flitv = (state_q == ST_SNOOP_WAIT) ? (rsp_rx_snp_flitv == Y) : (rsp_rx_req_flitv == Y);
    rx_rsp_flit  = (state_q == ST_SNOOP_WAIT) ? rsp_rx_snp_flit          : rsp_rx_req_flit;

    rx_dat_flitv_snp = (dat_rx_snp_flitv == Y);
    rx_dat_flit_snp  = dat_rx_snp_flit;

    rx_dat_flitv_req = (dat_rx_req_flitv == Y);
    rx_dat_flit_req  = dat_rx_req_flit;

    case (state_q)
      ST_IDLE: begin
        dirty_hit_d = 0;
        beat_cnt_d  = '0;
        if (req_flitv == Y) begin
          addr_d   = req_flit.addr;
          srcid_d  = req_flit.srcid;
          txnid_d  = req_flit.txnid;
          opcode_d = req_flit.opcode;
          size_d   = req_flit.size;
          qos_d    = req_flit.qos;
          state_d  = ST_ALLOC;
        end
      end

      ST_ALLOC: begin
        if (opcode_q == REQ_WRITE_BACK_FULL || opcode_q == REQ_WRITE_CLEAN_FULL) begin
          tx_type_d = TX_WRITE_BACK;
          state_d   = ST_DBIDRESP_SEND; // Skip snoop
        end else if (opcode_q == REQ_WRITE_UNIQUE_PTL || opcode_q == REQ_WRITE_UNIQUE_FULL) begin
          tx_type_d = TX_WRITE_UNIQUE;
          state_d   = ST_SNOOP_SEND;
        end else begin
          tx_type_d = TX_READ;
          state_d   = ST_SNOOP_SEND;
        end
      end

      ST_SNOOP_SEND: begin

        //  not depends combinationally on snp_lcrdv.
        if (snp_credit_have) begin
          snp_flitv          = Y;
          snp_flitpend       = Y;
          snp_flit.qos       = qos_q;
          snp_flit.srcid     = HNFID;
          snp_flit.txnid     = txnid_q;
          snp_flit.fwdnid    = '0;
          snp_flit.fwdtxnid  = '0;
          snp_flit.addr      = addr_q;
          
          if (tx_type_q == TX_READ) begin
            snp_flit.rettosrc = 1'b1; 
            
            if (opcode_q == REQ_READ_UNIQUE) begin
              snp_flit.opcode = SNP_UNIQUE;
            end else begin
              snp_flit.opcode = SNP_SHARED; 
            end
            
          end else begin
            snp_flit.opcode   = SNP_UNIQUE;
            snp_flit.rettosrc = 1'b0; 
          end
          
          state_d = ST_SNOOP_WAIT;
        end
      end

      ST_SNOOP_WAIT: begin
        // snoop-target identity is implicitly guaranteed by the crossbar's
        // fixed physical wiring for the lifetime 
        if (tx_type_q == TX_READ) begin
          // read hit: target has the (possibly dirty) line
          if ((rx_dat_flitv_snp == Y) && (rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA) && (rx_dat_flit_snp.txnid == txnid_q)) begin
            data_d[beat_cnt_q] = rx_dat_flit_snp.data;
            be_d[beat_cnt_q]   = rx_dat_flit_snp.be;
            dirty_hit_d = 1;
            if (dat_rx_snp_flitpend == Y) begin
              // more beats of this dirty line still coming
              beat_cnt_d = beat_cnt_q + 1'b1;
            end else begin
              beat_cnt_d = '0;
              state_d    = ST_RDATA_SEND;
            end
          end
          // read miss: target doesn't have the line
          else if ((rx_rsp_flitv == Y) && (rx_rsp_flit.opcode == RSP_SNP_RESP) && (rx_rsp_flit.txnid == txnid_q)) begin
            state_d = ST_SN_REQ_SEND;
          end
        end else begin
          // wait for invalidate ack
          if ((rx_rsp_flitv == Y) && (rx_rsp_flit.opcode == RSP_SNP_RESP) && (rx_rsp_flit.txnid == txnid_q)) begin
            state_d = ST_DBIDRESP_SEND;
          end
          else if ((rx_dat_flitv_snp == Y) && (rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA) && (rx_dat_flit_snp.txnid == txnid_q)) begin
            data_d[beat_cnt_q] = rx_dat_flit_snp.data;
            be_d[beat_cnt_q]   = rx_dat_flit_snp.be;
            if (dat_rx_snp_flitpend == Y) begin
              beat_cnt_d = beat_cnt_q + 1'b1;
            end else begin
              beat_cnt_d = '0;
              state_d    = ST_DIRTY_WB_REQ_SEND;
            end
          end
        end
      end

      ST_DIRTY_WB_REQ_SEND: begin
        if (sn_req_credit_have) begin
          sn_req_flitv           = Y;
          sn_req_flitpend        = Y;
          sn_req_flit.qos        = qos_q;
          sn_req_flit.tgtid      = SNFID;
          sn_req_flit.srcid      = HNFID;
          sn_req_flit.txnid      = txnid_q;
          sn_req_flit.size       = size_q;
          sn_req_flit.addr       = addr_q;
          sn_req_flit.allowretry = Y;
          sn_req_flit.pcrdttype  = '0;
          sn_req_flit.memattr    = '0;
         
          sn_req_flit.opcode     = REQ_WRITE_NO_SNOOP_FULL; 
          
          state_d = ST_DIRTY_WB_RSP_WAIT;
        end
      end

      ST_DIRTY_WB_RSP_WAIT: begin
        if ((sn_rsp_flitv == Y) && 
            (sn_rsp_flit.opcode == RSP_COMP_DBID_RESP) && 
            (sn_rsp_flit.txnid == txnid_q)) begin
          
          sn_dbid_d = sn_rsp_flit.dbid; 
          state_d   = ST_DIRTY_WB_DATA_SEND;
        end
      end

      ST_DIRTY_WB_DATA_SEND: begin
        if (sn_dat_tx_credit_have) begin
          sn_dat_tx_flitv        = Y;
          sn_dat_tx_flitpend     = (beat_cnt_q != LAST_BEAT) ? Y : N;
          sn_dat_tx_flit.qos     = qos_q;
          sn_dat_tx_flit.tgtid   = SNFID;
          sn_dat_tx_flit.srcid   = HNFID;
          sn_dat_tx_flit.txnid   = txnid_q;
          sn_dat_tx_flit.dbid    = sn_dbid_q;
          sn_dat_tx_flit.dataid  = beat_cnt_q[1:0];
          sn_dat_tx_flit.be      = {(DataWidth/8){1'b1}};
          sn_dat_tx_flit.data    = data_q[beat_cnt_q];
          sn_dat_tx_flit.opcode  = DAT_COPY_BACK_WR_DATA;

          if (beat_cnt_q == LAST_BEAT) begin
            beat_cnt_d = '0;
            state_d = (tx_type_q == TX_READ) ? ST_DONE : ST_DBIDRESP_SEND;
          end else begin
            beat_cnt_d = beat_cnt_q + 1'b1;
          end
        end
      end
      
      ST_DBIDRESP_SEND: begin
        if (rsp_tx_credit_have) begin
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
          state_d = ST_WDATA_WAIT;
        end
      end

      ST_WDATA_WAIT: begin
        // srcid check added: txnid alone is NOT a unique transaction key
        // across different requesters (CHI only guarantees TxnID
        // uniqueness *within* one requester's own outstanding set), so if
        // two different RN-Fs happen to pick the same TxnID, matching on
        // txnid alone could let a write-data beat meant for a different
        // requester's transaction get accepted here. srcid_q was latched
        // from the original request, so requiring it to match fixes the issue
        if ((rx_dat_flitv_req == Y) && (rx_dat_flit_req.txnid == txnid_q) && (rx_dat_flit_req.srcid == srcid_q)) begin
          data_d[beat_cnt_q] = rx_dat_flit_req.data;
          be_d[beat_cnt_q]   = rx_dat_flit_req.be;
          if (dat_rx_req_flitpend == Y) begin
            beat_cnt_d = beat_cnt_q + 1'b1;
          end else begin
            beat_cnt_d = '0;
            state_d    = ST_SN_REQ_SEND;
          end
        end
      end

      ST_SN_REQ_SEND: begin
        // send req to the slave node 
        if (sn_req_credit_have) begin
          sn_req_flitv        = Y;
          sn_req_flitpend     = Y;
          sn_req_flit.qos     = qos_q;
          sn_req_flit.tgtid   = SNFID;
          sn_req_flit.srcid   = HNFID;
          sn_req_flit.txnid   = txnid_q;
          sn_req_flit.size    = size_q;
          sn_req_flit.addr    = addr_q;
          sn_req_flit.allowretry = Y;
          sn_req_flit.pcrdttype  = '0;
          sn_req_flit.memattr    = '0;

          if (tx_type_q == TX_READ) begin
            sn_req_flit.opcode = REQ_READ_NO_SNOOP;
          end else if (tx_type_q == TX_WRITE_BACK) begin
            sn_req_flit.opcode = REQ_WRITE_NO_SNOOP_FULL;
          end else begin
            sn_req_flit.opcode = REQ_WRITE_NO_SNOOP_PTL;
          end
          state_d = ST_SN_RSP_WAIT;
        end
      end

      ST_SN_RSP_WAIT: begin
        if (tx_type_q == TX_READ) begin
          // wait for memory response on the data (SN-F is a single fixed
          // target we ourselves issued the txnid to, so txnid-only
          // matching is unambiguous here).
          if ((sn_dat_rx_flitv == Y) && (sn_dat_rx_flit.opcode == DAT_COMP_DATA) && (sn_dat_rx_flit.txnid == txnid_q)) begin
            data_d[beat_cnt_q] = sn_dat_rx_flit.data;
            be_d[beat_cnt_q]   = sn_dat_rx_flit.be;
            if (sn_dat_rx_flitpend == Y) begin
              beat_cnt_d = beat_cnt_q + 1'b1;
            end else begin
              beat_cnt_d = '0;
              state_d    = ST_RDATA_SEND;
            end
          end
        end else begin
          //wait for memory write ack on RSP channel
          if ((sn_rsp_flitv == Y) && (sn_rsp_flit.opcode == RSP_COMP_DBID_RESP) && (sn_rsp_flit.txnid == txnid_q)) begin
            sn_dbid_d = sn_rsp_flit.dbid; // Capture memory's DBID
            state_d   = ST_SN_DATA_SEND;
          end
        end
      end

      ST_SN_DATA_SEND: begin
        if (sn_dat_tx_credit_have) begin
          sn_dat_tx_flitv        = Y;
          sn_dat_tx_flitpend     = (beat_cnt_q != LAST_BEAT) ? Y : N;
          sn_dat_tx_flit.qos     = qos_q;
          sn_dat_tx_flit.tgtid   = SNFID;
          sn_dat_tx_flit.srcid   = HNFID;
          sn_dat_tx_flit.txnid   = txnid_q;
          sn_dat_tx_flit.dbid    = sn_dbid_q;
          sn_dat_tx_flit.dataid  = beat_cnt_q[1:0];
          sn_dat_tx_flit.be      = be_q[beat_cnt_q];
          sn_dat_tx_flit.data    = data_q[beat_cnt_q];

          if (tx_type_q == TX_WRITE_BACK) begin
            sn_dat_tx_flit.opcode = DAT_COPY_BACK_WR_DATA;
          end else begin
            sn_dat_tx_flit.opcode = DAT_NON_COPY_BACK_WR_DATA;
          end

          if (beat_cnt_q == LAST_BEAT) begin
            beat_cnt_d = '0;
            state_d = (tx_type_q == TX_WRITE_BACK) ? ST_DONE : ST_COMP_SEND;
          end else begin
            beat_cnt_d = beat_cnt_q + 1'b1;
          end
        end
      end

      ST_RDATA_SEND: begin
        if (dat_tx_credit_have) begin
          dat_tx_flitv       = Y;
          dat_tx_flitpend    = (beat_cnt_q != LAST_BEAT) ? Y : N;
          dat_tx_flit.qos    = qos_q;
          dat_tx_flit.tgtid  = srcid_q;
          dat_tx_flit.srcid  = HNFID;
          dat_tx_flit.txnid  = txnid_q;
          dat_tx_flit.opcode = DAT_COMP_DATA;
          dat_tx_flit.dataid = beat_cnt_q[1:0];
          dat_tx_flit.be     = be_q[beat_cnt_q];
          dat_tx_flit.data   = data_q[beat_cnt_q];

          if (beat_cnt_q == LAST_BEAT) begin
            beat_cnt_d = '0;
            state_d = ST_COMPACK_WAIT;
          end else begin
            beat_cnt_d = beat_cnt_q + 1'b1;
          end
        end
      end

      ST_COMP_SEND: begin
        if (rsp_tx_credit_have) begin
          rsp_tx_flitv       = Y;
          rsp_tx_flitpend    = Y;
          rsp_tx_flit.qos    = qos_q;
          rsp_tx_flit.tgtid  = srcid_q;
          rsp_tx_flit.srcid  = HNFID;
          rsp_tx_flit.txnid  = txnid_q;
          rsp_tx_flit.opcode = RSP_COMP;
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMPACK_WAIT: begin
        
        // ERROR IN HERE PROBABLY ON THE RX ROUTING ON CONCURRENT INDEPENENT TRANSACTIONS TSHR1 DOESNT GET THE REQUIRED SIGNALS
        // RX_RSP_FLITV NEVER COMES 
        if ((rx_rsp_flitv == Y) && (rx_rsp_flit.opcode == RSP_COMP_ACK) && (rx_rsp_flit.txnid == txnid_q) && (rx_rsp_flit.srcid == srcid_q)) begin
          
          if (dirty_hit_q == 1'b1) begin
            // served the core, now go push the dirty data to memory
            state_d = ST_DIRTY_WB_REQ_SEND;
          end else begin
            // normal read, no writeback needed
            state_d = ST_DONE;
          end
          
        end
      end

      ST_DONE: begin
        state_d = ST_IDLE;
      end

      default: begin
        state_d = ST_IDLE;
      end
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
      for (int i = 0; i < NumBeats; i++) begin
        data_q[i] <= '0;
        be_q[i]   <= '0;
      end
      beat_cnt_q    <= '0;
      sn_dbid_q     <= '0;
      dirty_hit_q    <= 0;
    end else begin
      state_q       <= state_d;
      if (tx_type_d == TX_READ || tx_type_d == TX_WRITE_UNIQUE || tx_type_d == TX_WRITE_BACK) begin
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
      for (int i = 0; i < NumBeats; i++) begin
        data_q[i] <= data_d[i];
        be_q[i]   <= be_d[i];
      end
      beat_cnt_q    <= beat_cnt_d;
      sn_dbid_q     <= sn_dbid_d;
    end 
  end
  assign TxnID_o = txnid_q;
  assign SrcID_o = srcid_q;
  
endmodule