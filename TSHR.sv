


// basic sequence of the mesi protocol
// both RN-F0 and RN-F1 modules fetch the same line that line is shared
// now RN-F0 wants to change a data in its cache it send a snoop signal to the Bus and ask for invaliate
// RN-F1 snoops the bus(send cleanunique or makeunique) and claim the request give a fetching persmion 
// if RN-F0 has to change the state of the line of its cache to modified
// if RN-F1 has to change the state of the line of its cahce to invalid
// it is not permited to write or read from the invalid line
// exlusive lines can be written or read freely 
// shared line can be read freely but cannot be written without turning to modified

// if read miss happens it loads from the memory(or other caches) if other cores have the data it is shared if only memory has it it is exclusive 
// if writeback happens it is free of snoop to push memory cuz it is already modified


//define macro for requester this is needed for configureable RN-F interface counts 
// it is basicly couple of muxes workaround for dynamic slicing of interface arrays
// however credit system will also need a change 
`define DEMUX_TX(IDX, PROXY_V, PROXY_PEND, PROXY_FLIT, IF_ARRAY) \
  IF_ARRAY[0].flitv    = (IDX == 1'b0) ? PROXY_V : 1'b0; \
  IF_ARRAY[0].flitpend = (IDX == 1'b0) ? PROXY_PEND : 1'b0; \
  IF_ARRAY[0].flit     = (IDX == 1'b0) ? PROXY_FLIT : '0; \
  IF_ARRAY[1].flitv    = (IDX == 1'b1) ? PROXY_V : 1'b0; \
  IF_ARRAY[1].flitpend = (IDX == 1'b1) ? PROXY_PEND : 1'b0; \
  IF_ARRAY[1].flit     = (IDX == 1'b1) ? PROXY_FLIT : '0;





// the things that this module has 
// fully blocking, non-pipelined, one transaction system-wide at a time HN-F module 
// this TSHR can be called a HN-F module because it's just what a pool-size-1 HN-F is.
// the specs of this module 
//request admission from either RN-F, arbitrated fairly (change between every request) 
//snoop filter (only checks for the requester is trying to snoop itself)
//snoop generator 
//data collection/forwarding for snoop-response data, write data, read data 
//SN-F request/response/data sequencing 
//completion + CompAck handling 
//per-channel credit tracking 

// the things that this module doesnt have are the following
//tracker pool
//real persistent directory/snoop-filter state (this only increase efficiency by skipping snoops you already know the answer)
//address hazard detection (it is basicly a edge case where 2 TSHR allocate same address at the same time window  it is impossible when there is only one TSHR) 
//txnid uniqueness/remapping across RN-Fs (txnID is always correct system wide)
// the upper comments is what real system HN-F should look like


module TSHR // transaction snoop handling register
 import chi_pkg::*;
 import common_pkg::*;  #( 

  parameter int AddrWidth = 39,
  parameter int DataWidth = 128,
  parameter chi_pkg::node_id_e HNFID = 7'h40,
  parameter chi_pkg::node_id_e SNFID = 7'h00
) (
  input  wire clk,
  input  wire resetn,
  output logic busy_o,
  chi_req.rx req_rx [2],
  chi_snp.tx snp_tx [2],
  chi_rsp.tx rsp_tx [2],
  chi_rsp.rx rsp_rx [2],
  chi_dat.tx dat_tx [2],
  chi_dat.rx dat_rx [2],

  chi_req.tx sn_req_tx,
  chi_rsp.rx sn_rsp_rx,
  chi_dat.tx sn_dat_tx,
  chi_dat.rx sn_dat_rx
);

  
  
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
    ST_READNO_SNOOP
    
  } state_e;

  state_e state_q, state_d;

  logic                  req_idx_q, req_idx_d;
  logic                  snp_idx_q, snp_idx_d;
  logic                  is_write_q, is_write_d;


  logic                  req_gnt_idx_q, req_gnt_idx_d;

  logic [AddrWidth-1:0]  addr_q, addr_d;
  node_id_e              srcid_q, srcid_d;
  logic [11:0]           txnid_q, txnid_d;
  req_opcode_e            opcode_q, opcode_d;
  width_e                 size_q, size_d;
  logic [3:0]             qos_q, qos_d;

  logic [DataWidth-1:0]    data_q, data_d;
  logic [(DataWidth/8)-1:0] be_q, be_d;
  logic [11:0]              sn_dbid_q, sn_dbid_d;

  logic [3:0] snp_credit_q [2];
  logic [3:0] rsp_credit_q [2];
  logic [3:0] dat_credit_q [2];
  logic [3:0] sn_req_credit_q;
  logic [3:0] sn_dat_credit_q;

  assign req_rx[0].lcrdv = (state_q == ST_IDLE);
  assign req_rx[1].lcrdv = (state_q == ST_IDLE);
  assign rsp_rx[0].lcrdv = 1'b1;
  assign rsp_rx[1].lcrdv = 1'b1;
  assign dat_rx[0].lcrdv = 1'b1;
  assign dat_rx[1].lcrdv = 1'b1;
  assign sn_rsp_rx.lcrdv = 1'b1;
   
  // fuck you vivado why cant you access interface arrays dynmaicly
  // generate Block oly for interface;
  genvar i;
  generate 
    for(i = 0; i < 2 ; i = i + 1) begin : gen_credit_base
      always_ff @(posedge clk or negedge resetn) begin
        if (!resetn) begin
          snp_credit_q[i] <= 4'd0;
          rsp_credit_q[i] <= 4'd0;
          dat_credit_q[i] <= 4'd0;
        end else begin
          unique case ({snp_tx[i].lcrdv, (snp_tx[i].flitv == Y)})
            2'b10:   snp_credit_q[i] <= snp_credit_q[i] + 4'd1;
            2'b01:   snp_credit_q[i] <= snp_credit_q[i] - 4'd1;
            default: snp_credit_q[i] <= snp_credit_q[i];
          endcase
          
          unique case ({rsp_tx[i].lcrdv, (rsp_tx[i].flitv == Y)})
            2'b10:   rsp_credit_q[i] <= rsp_credit_q[i] + 4'd1;
            2'b01:   rsp_credit_q[i] <= rsp_credit_q[i] - 4'd1;
            default: rsp_credit_q[i] <= rsp_credit_q[i];
          endcase
          
          unique case ({dat_tx[i].lcrdv, (dat_tx[i].flitv == Y)})
            2'b10:   dat_credit_q[i] <= dat_credit_q[i] + 4'd1;
            2'b01:   dat_credit_q[i] <= dat_credit_q[i] - 4'd1;
            default: dat_credit_q[i] <= dat_credit_q[i];
          endcase
        end
      end
    end
  endgenerate
  // non array element block;
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      sn_req_credit_q <= 4'd0;
      sn_dat_credit_q <= 4'd0;
    end else begin
      unique case ({sn_req_tx.lcrdv, (sn_req_tx.flitv == Y)})
        2'b10:   sn_req_credit_q <= sn_req_credit_q + 4'd1;
        2'b01:   sn_req_credit_q <= sn_req_credit_q - 4'd1;
        default: sn_req_credit_q <= sn_req_credit_q;
      endcase
      
      unique case ({sn_dat_tx.lcrdv, (sn_dat_tx.flitv == Y)})
        2'b10:   sn_dat_credit_q <= sn_dat_credit_q + 4'd1;
        2'b01:   sn_dat_credit_q <= sn_dat_credit_q - 4'd1;
        default: sn_dat_credit_q <= sn_dat_credit_q;
      endcase
    end
  end

  // ==========================================================================
  // synthax error module
//  always_ff @(posedge clk or negedge resetn) begin
//    if (!resetn) begin
//      for (int i = 0; i < 2; i++) begin
//        snp_credit_q[i] <= 4'd0;
//        rsp_credit_q[i] <= 4'd0;
//        dat_credit_q[i] <= 4'd0;
//      end
//      sn_req_credit_q <= 4'd0;
//      sn_dat_credit_q <= 4'd0;
//    end else begin
//      for (int i = 0; i < 2; i++) begin
//        unique case ({snp_tx[i].lcrdv, (snp_tx[i].flitv == Y)})
//          2'b10:   snp_credit_q[i] <= snp_credit_q[i] + 4'd1;
//          2'b01:   snp_credit_q[i] <= snp_credit_q[i] - 4'd1;
//          default: snp_credit_q[i] <= snp_credit_q[i];
//        endcase
//        unique case ({rsp_tx[i].lcrdv, (rsp_tx[i].flitv == Y)})
//          2'b10:   rsp_credit_q[i] <= rsp_credit_q[i] + 4'd1;
//          2'b01:   rsp_credit_q[i] <= rsp_credit_q[i] - 4'd1;
//          default: rsp_credit_q[i] <= rsp_credit_q[i];
//        endcase
//        unique case ({dat_tx[i].lcrdv, (dat_tx[i].flitv == Y)})
//          2'b10:   dat_credit_q[i] <= dat_credit_q[i] + 4'd1;
//          2'b01:   dat_credit_q[i] <= dat_credit_q[i] - 4'd1;
//          default: dat_credit_q[i] <= dat_credit_q[i];
//        endcase
//      end
//      unique case ({sn_req_tx.lcrdv, (sn_req_tx.flitv == Y)})
//        2'b10:   sn_req_credit_q <= sn_req_credit_q + 4'd1;
//        2'b01:   sn_req_credit_q <= sn_req_credit_q - 4'd1;
//        default: sn_req_credit_q <= sn_req_credit_q;
//      endcase
//      unique case ({sn_dat_tx.lcrdv, (sn_dat_tx.flitv == Y)})
//        2'b10:   sn_dat_credit_q <= sn_dat_credit_q + 4'd1;
//        2'b01:   sn_dat_credit_q <= sn_dat_credit_q - 4'd1;
//        default: sn_dat_credit_q <= sn_dat_credit_q;
//      endcase
//    end
//  end
     // simple decoding block to understand where to go 
     // this simplify the if else blocks and makes us only check for tx_type
     // it is latched on the alloc state
    typedef enum logic [1:0] {
        TX_READ,
        TX_WRITE_UNIQUE,
        TX_WRITE_BACK
      } tx_type_e;
      
      tx_type_e tx_type_q, tx_type_d;
  // WORKAROUND: Local struct definitions 
  typedef struct packed {
    logic [3:0]           qos;
    node_id_e             srcid;
    logic [11:0]          txnid;
    node_id_e             fwdnid;
    logic [11:0]          fwdtxnid;
    snp_opcode_e          opcode;
    logic [AddrWidth-1:0] addr;
    logic                 ns;
    logic                 donotgotosd;
    logic                 rettosrc;
    logic                 tracetag;
  } local_snp_flit_t;

  typedef struct packed {
    logic [3:0]           qos;
    node_id_e             tgtid;
    node_id_e             srcid;
    logic [11:0]          txnid;
    rsp_opcode_e          opcode;
    logic [1:0]           resperr;
    logic [2:0]           resp;
    logic [2:0]           fwdstate;
    logic [2:0]           cbusy;
    logic [11:0]          dbid;
    logic [3:0]           pcrdttype;
    logic [1:0]           tagop;
    logic                 tracetag;
  } local_rsp_flit_t;

  typedef struct packed {
    logic [3:0]                 qos;
    node_id_e                   tgtid;
    node_id_e                   srcid;
    logic [11:0]                txnid;
    node_id_e                   homenid;
    dat_opcode_e                opcode;
    logic [1:0]                 resperr;
    logic [2:0]                 resp;
    logic [2:0]                 fwdstate;
    logic [2:0]                 cbusy;
    logic [11:0]                dbid;
    logic [1:0]                 ccid;
    logic [1:0]                 dataid;
    logic [1:0]                 tagop;
    logic [(DataWidth/32)-1:0]  tag;
    logic [(DataWidth/128)-1:0] tu;
    logic                       tracetag;
    logic [(DataWidth/8)-1:0]   be;
    logic [DataWidth-1:0]       data;
  } local_dat_flit_t;
    
    // workaround for assigning the local workspace
    logic            proxy_snp_flitv, proxy_snp_flitpend;
    local_snp_flit_t proxy_snp_flit; 
    
    logic            proxy_rsp_flitv, proxy_rsp_flitpend;
    local_rsp_flit_t proxy_rsp_flit;
    
    logic            proxy_dat_flitv, proxy_dat_flitpend;
    local_dat_flit_t proxy_dat_flit;

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
   // 2. ST_WDATA_WAIT: Wait for the requester to send the actual write data on the DAT channel.
   // 3. ST_SN_REQ_SEND: Forward a request (WriteNoSnpPtl) down to the memory (SN-F).
   // 4. ST_SN_RSP_WAIT: Wait for the memory to reply with CompDBIDResp, meaning memory is ready for the data.
   // 5. ST_SN_DATA_SEND: Push the captured write data down to the memory.
   // 6. ST_COMP_SEND: Send a final Completion (Comp) to the original requester.
   // 7. ST_COMPACK_WAIT: Wait for the requester to acknowledge the completion (CompAck).
   // 8. ST_DONE: Clean up and return to IDLE.
   
   
   // path from ST_RDATA_SEND (The Read Flow - ReadUnique):
   // 1. ST_RDATA_SEND: We already got the requested data from snooping RN-F1. Forward this data directly to the original requester (RN-F0) as CompData.
   // 2. ( skipped Memory access steps for this flow).
   // 3. ST_COMPACK_WAIT: Wait for the requester to acknowledge receipt of the data (CompAck).
   // 4. ST_DONE: Clean up and return to IDLE.
   
   
   // ======================= TODO ==============================
   // implement cache hit on RN-F1 - done
   // implement cache miss everywhere (standart memory read however RN-F1 gets nothing) -done
   // implement standart memory write (RN-F1 gets validated after RN-F0 request) -done
   // implement cache write back on the RN-F0 no snoop required just push data to the main memory since it is already modified -done
   assign busy_o = (state_q != ST_IDLE); 
  always_comb begin
    state_d       = state_q;
    req_idx_d     = req_idx_q;
    snp_idx_d     = snp_idx_q;
    is_write_d    = is_write_q;
    req_gnt_idx_d = req_gnt_idx_q;
    addr_d        = addr_q;
    srcid_d       = srcid_q;
    txnid_d       = txnid_q;
    opcode_d      = opcode_q;
    size_d        = size_q;
    qos_d         = qos_q;
    data_d        = data_q;
    be_d          = be_q;
    sn_dbid_d     = sn_dbid_q;

    proxy_snp_flitv    = 1'b0;
    proxy_snp_flitpend = 1'b0;
    proxy_snp_flit     = '0;
    proxy_rsp_flitv    = 1'b0;
    proxy_rsp_flitpend = 1'b0;
    proxy_rsp_flit     = '0;
    proxy_dat_flitv    = 1'b0;
    proxy_dat_flitpend = 1'b0;
    proxy_dat_flit     = '0;

    sn_req_tx.flitv    = N;
    sn_req_tx.flitpend = N;
    sn_req_tx.flit     = '0;
    sn_dat_tx.flitv    = N;
    sn_dat_tx.flitpend = N;
    sn_dat_tx.flit     = '0;
    //payload assignments 
    rx_rsp_flit  = (state_q == ST_SNOOP_WAIT) ? 
                 local_rsp_flit_t'((snp_idx_q == 1'b0) ? rsp_rx[0].flit : rsp_rx[1].flit) :
                 local_rsp_flit_t'((req_idx_q == 1'b0) ? rsp_rx[0].flit : rsp_rx[1].flit);

    rx_dat_flit_snp  = local_dat_flit_t'((snp_idx_q == 1'b0) ? dat_rx[0].flit  : dat_rx[1].flit);
    
    rx_dat_flit_req  = local_dat_flit_t'((req_idx_q == 1'b0) ? dat_rx[0].flit  : dat_rx[1].flit);
    
    rx_rsp_flitv = (state_q == ST_SNOOP_WAIT) ?
               ((snp_idx_q == 1'b0) ? (rsp_rx[0].flitv == Y) : (rsp_rx[1].flitv == Y)) :
               ((req_idx_q == 1'b0) ? (rsp_rx[0].flitv == Y) : (rsp_rx[1].flitv == Y));

    rx_dat_flitv_snp = (snp_idx_q == 1'b0) ? (dat_rx[0].flitv == Y) : (dat_rx[1].flitv == Y);
    
    rx_dat_flitv_req = (req_idx_q == 1'b0) ? (dat_rx[0].flitv == Y) : (dat_rx[1].flitv == Y);

    case (state_q)
      ST_IDLE: begin
          if (req_gnt_idx_q == 1'b0) begin
            if (req_rx[0].flitv == Y) begin
              req_idx_d = 1'b0;
              addr_d    = req_rx[0].flit.addr;
              srcid_d   = req_rx[0].flit.srcid;
              txnid_d   = req_rx[0].flit.txnid;
              opcode_d  = req_rx[0].flit.opcode;
              size_d    = req_rx[0].flit.size;
              qos_d     = req_rx[0].flit.qos;
              req_gnt_idx_d = 1'b1;      // rotate priority after granting 
              state_d   = ST_ALLOC;
            end else if (req_rx[1].flitv == Y) begin
              req_idx_d = 1'b1;
              addr_d    = req_rx[1].flit.addr;
              srcid_d   = req_rx[1].flit.srcid;
              txnid_d   = req_rx[1].flit.txnid;
              opcode_d  = req_rx[1].flit.opcode;
              size_d    = req_rx[1].flit.size;
              qos_d     = req_rx[1].flit.qos;
              state_d   = ST_ALLOC;
            end
          end else begin // req_gnt_idx_q == 1'b1;
            if (req_rx[1].flitv == Y) begin
              req_idx_d = 1'b1;
              addr_d    = req_rx[1].flit.addr;
              srcid_d   = req_rx[1].flit.srcid;
              txnid_d   = req_rx[1].flit.txnid;
              opcode_d  = req_rx[1].flit.opcode;
              size_d    = req_rx[1].flit.size;
              qos_d     = req_rx[1].flit.qos;
              req_gnt_idx_d = 1'b0;
              state_d   = ST_ALLOC;
            end else if (req_rx[0].flitv == Y) begin
              req_idx_d = 1'b0;
              addr_d    = req_rx[0].flit.addr;
              srcid_d   = req_rx[0].flit.srcid;
              txnid_d   = req_rx[0].flit.txnid;
              opcode_d  = req_rx[0].flit.opcode;
              size_d    = req_rx[0].flit.size;
              qos_d     = req_rx[0].flit.qos;
              state_d   = ST_ALLOC;
            end
          end
        end

      ST_ALLOC: begin
        snp_idx_d  = ~req_idx_q;
        if (opcode_q == REQ_WRITE_BACK_FULL || opcode_q == REQ_WRITE_CLEAN_FULL) begin
          tx_type_d = TX_WRITE_BACK;
          is_write_d = 1;
          state_d   = ST_DBIDRESP_SEND; //skip snoop
        end else if (opcode_q == REQ_WRITE_UNIQUE_PTL || opcode_q == REQ_WRITE_UNIQUE_FULL) begin
          tx_type_d = TX_WRITE_UNIQUE;
          is_write_d = 1;
          state_d   = ST_SNOOP_SEND;
        end else begin
          tx_type_d = TX_READ;
          is_write_d = 0;
          state_d   = ST_SNOOP_SEND;
        end
      end

      ST_SNOOP_SEND: begin
        if (snp_credit_q[snp_idx_q] != 4'd0) begin
          proxy_snp_flitv          = Y;
          proxy_snp_flitpend       = Y;
          proxy_snp_flit.qos       = qos_q;
          proxy_snp_flit.srcid     = HNFID;
          proxy_snp_flit.txnid     = txnid_q;
          proxy_snp_flit.fwdnid    = '0;
          proxy_snp_flit.fwdtxnid  = '0;
          proxy_snp_flit.opcode    = SNP_UNIQUE;
          proxy_snp_flit.addr      = addr_q;
          proxy_snp_flit.rettosrc  = is_write_q ? 1'b0 : 1'b1;
               if (tx_type_q == TX_READ) begin
                 proxy_snp_flit.opcode   = SNP_SHARED;
                 proxy_snp_flit.rettosrc = 1'b1; // other RN-F must return data
               end else begin
                 proxy_snp_flit.opcode   = SNP_UNIQUE;
                 proxy_snp_flit.rettosrc = 1'b0; // other RN-F just invalidates
               end
          state_d = ST_SNOOP_WAIT;
        end
      end
        ST_SNOOP_WAIT: begin
        if (tx_type_q == TX_READ) begin
          // read hit other RN-F has the line 
          if ((rx_dat_flitv_snp == Y) && 
          (rx_dat_flit_snp.opcode == DAT_SNP_RESP_DATA) &&
           (rx_dat_flit_snp.txnid == txnid_q)) begin
            data_d  = rx_dat_flit_snp.data;
            be_d    = rx_dat_flit_snp.be;
            state_d = ST_RDATA_SEND;
          end
          //  read miss other RN-F doesnt have the line
          else if ((rx_rsp_flitv == Y) && 
          (rx_rsp_flit.opcode == RSP_SNP_RESP) && 
          (rx_rsp_flit.txnid == txnid_q)) begin
            state_d = ST_SN_REQ_SEND;
          end
        end else begin
          // wait for invalid ack)
          if ((rx_rsp_flitv == Y) &&
           (rx_rsp_flit.opcode == RSP_SNP_RESP) && 
           (rx_rsp_flit.txnid == txnid_q)) begin
            state_d = ST_DBIDRESP_SEND;
          end
        end
      end
      ST_DBIDRESP_SEND: begin
        if (rsp_credit_q[req_idx_q] != 4'd0) begin
          proxy_rsp_flitv       = Y;
          proxy_rsp_flitpend    = Y;
          proxy_rsp_flit.qos    = qos_q;
          proxy_rsp_flit.tgtid  = srcid_q;
          proxy_rsp_flit.srcid  = HNFID;
          proxy_rsp_flit.txnid  = txnid_q;
          proxy_rsp_flit.dbid   = txnid_q;
          
          if (tx_type_q == TX_WRITE_BACK) begin
            proxy_rsp_flit.opcode = RSP_COMP_DBID_RESP;
          end else begin
            proxy_rsp_flit.opcode = RSP_DBID_RESP;
          end
          state_d = ST_WDATA_WAIT;
        end
      end

      ST_WDATA_WAIT: begin 
      // wait state for RN-F response after DBIDresp given to RN-F it can wait here any arbitary time but if peer sends request it just vanishes
      // in the future this will probably breaks ======= DONT FORGET
        if ((rx_dat_flitv_req == Y) && (rx_dat_flit_req.txnid == txnid_q)) begin
          data_d  = rx_dat_flit_req.data;
          be_d    = rx_dat_flit_req.be;
          state_d = ST_SN_REQ_SEND;
        end
      end

      ST_SN_REQ_SEND: begin
        // send req to the slave node 
        if (sn_req_credit_q != 4'd0) begin
          sn_req_tx.flitv        = Y;
          sn_req_tx.flitpend     = Y;
          sn_req_tx.flit.qos     = qos_q;
          sn_req_tx.flit.tgtid   = SNFID;
          sn_req_tx.flit.srcid   = HNFID;
          sn_req_tx.flit.txnid   = txnid_q;
          sn_req_tx.flit.size    = size_q;
          sn_req_tx.flit.addr    = addr_q;
          sn_req_tx.flit.allowretry = Y;
          sn_req_tx.flit.pcrdttype  = '0;
          sn_req_tx.flit.memattr    = '0;

          if (tx_type_q == TX_READ) begin
            sn_req_tx.flit.opcode = REQ_READ_NO_SNOOP;
          end else if (tx_type_q == TX_WRITE_BACK) begin
            sn_req_tx.flit.opcode = REQ_WRITE_NO_SNOOP_FULL;
          end else begin
            sn_req_tx.flit.opcode = REQ_WRITE_NO_SNOOP_PTL;
          end
          state_d = ST_SN_RSP_WAIT;
        end
      end

      ST_SN_RSP_WAIT: begin
        if (tx_type_q == TX_READ) begin
            // wait for memory respose on the data
          if ((sn_dat_rx.flitv == Y) && (sn_dat_rx.flit.opcode == DAT_COMP_DATA) && (sn_dat_rx.flit.txnid == txnid_q)) begin
            data_d  = sn_dat_rx.flit.data;
            be_d    = sn_dat_rx.flit.be;
            state_d = ST_RDATA_SEND;
          end
        end else begin
          //wait for memory write ack on RSP channel
          if ((sn_rsp_rx.flitv == Y) && (sn_rsp_rx.flit.opcode == RSP_COMP_DBID_RESP) && (sn_rsp_rx.flit.txnid == txnid_q)) begin
            sn_dbid_d = sn_rsp_rx.flit.dbid; // capture memory DBID
            state_d   = ST_SN_DATA_SEND;
          end
        end
      end

      ST_SN_DATA_SEND: begin
        if (sn_dat_credit_q != 4'd0) begin
          sn_dat_tx.flitv        = Y;
          sn_dat_tx.flitpend     = Y;
          sn_dat_tx.flit.qos     = qos_q;
          sn_dat_tx.flit.tgtid   = SNFID;
          sn_dat_tx.flit.srcid   = HNFID;
          sn_dat_tx.flit.txnid   = txnid_q; // not sure about this specs says different things
          sn_dat_tx.flit.dbid    = sn_dbid_q; 
          sn_dat_tx.flit.be      = be_q;
          sn_dat_tx.flit.data    = data_q;
          
          if (tx_type_q == TX_WRITE_BACK) begin
            sn_dat_tx.flit.opcode = DAT_COPY_BACK_WR_DATA;
            state_d = ST_DONE; // already modified no snoop
          end else begin
            sn_dat_tx.flit.opcode = DAT_NON_COPY_BACK_WR_DATA;
            state_d = ST_COMP_SEND;
          end
        end
      end

      ST_RDATA_SEND: begin 
        if (dat_credit_q[req_idx_q] != 4'd0) begin
          proxy_dat_flitv       = Y;
          proxy_dat_flitpend    = Y;
          proxy_dat_flit.qos    = qos_q;
          proxy_dat_flit.tgtid  = srcid_q;
          proxy_dat_flit.srcid  = HNFID;
          proxy_dat_flit.txnid  = txnid_q;
          proxy_dat_flit.opcode = DAT_COMP_DATA;
          proxy_dat_flit.be     = be_q;
          proxy_dat_flit.data   = data_q;
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMP_SEND: begin // completed send
        if (rsp_credit_q[req_idx_q] != 4'd0) begin
          proxy_rsp_flitv       = Y;
          proxy_rsp_flitpend    = Y;
          proxy_rsp_flit.qos    = qos_q;
          proxy_rsp_flit.tgtid  = srcid_q;
          proxy_rsp_flit.srcid  = HNFID;
          proxy_rsp_flit.txnid  = txnid_q;
          proxy_rsp_flit.opcode = RSP_COMP;
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMPACK_WAIT: begin // wait for ack
        if ((rx_rsp_flitv == Y) && (rx_rsp_flit.opcode == RSP_COMP_ACK) && (rx_rsp_flit.txnid == txnid_q)) begin
          state_d = ST_DONE;
        end
      end

      ST_DONE: begin
        state_d = ST_IDLE;
      end

      default: begin
        state_d = ST_IDLE;
      end
    endcase

        `DEMUX_TX (snp_idx_q, proxy_snp_flitv, proxy_snp_flitpend, proxy_snp_flit, snp_tx)
        `DEMUX_TX (req_idx_q, proxy_rsp_flitv, proxy_rsp_flitpend, proxy_rsp_flit, rsp_tx)
        `DEMUX_TX (req_idx_q, proxy_dat_flitv, proxy_dat_flitpend, proxy_dat_flit, dat_tx)
  end
   // just simple state transitions
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      state_q       <= ST_IDLE;
      tx_type_q     <= TX_READ;
      req_idx_q     <= 1'b0;
      snp_idx_q     <= 1'b0;
      req_gnt_idx_q <= 1'b0;
      addr_q        <= '0;
      srcid_q       <= '0;
      txnid_q       <= '0;
      opcode_q      <= req_opcode_e'(8'h00);
      size_q        <= WIDTH_1;
      qos_q         <= '0;
      data_q        <= '0;
      be_q          <= '0;
      sn_dbid_q     <= '0;
    end else begin
      state_q       <= state_d;
      tx_type_q     <= tx_type_d;
      req_idx_q     <= req_idx_d;
      snp_idx_q     <= snp_idx_d;
      req_gnt_idx_q <= req_gnt_idx_d;
      addr_q        <= addr_d;
      srcid_q       <= srcid_d;
      txnid_q       <= txnid_d;
      opcode_q      <= opcode_d;
      size_q        <= size_d;
      qos_q         <= qos_d;
      data_q        <= data_d;
      be_q          <= be_d;
      sn_dbid_q     <= sn_dbid_d;
    end
  end

endmodule