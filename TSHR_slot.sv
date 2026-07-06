module TSHR_Slot
 import chi_pkg::*;
 import common_pkg::*;  #( 
  parameter int AddrWidth = 39,
  parameter int DataWidth = 128,
  parameter chi_pkg::node_id_e HNFID = 7'h40,
  parameter chi_pkg::node_id_e SNFID = 7'h00
) (
  input  wire clk,
  input  wire resetn,
  
  // -----------------------------------------
  // 1. Allocation Interface (From Central Router)
  // -----------------------------------------
  output logic            alloc_rdy_o, 
  input  logic            alloc_v_i,
  input  logic            alloc_req_idx_i, // 0 = RNF0, 1 = RNF1
  input  local_req_flit_t alloc_req_flit_i, 

  // -----------------------------------------
  // 2. Incoming Flits (Routed to this specific TSHR)
  // -----------------------------------------
  input  logic            rx_rsp_v_i,
  input  local_rsp_flit_t rx_rsp_flit_i,
  
  input  logic            rx_dat_v_i,
  input  local_dat_flit_t rx_dat_flit_i,
  
  input  logic            rx_sn_rsp_v_i,
  input  local_rsp_flit_t rx_sn_rsp_flit_i,
  
  input  logic            rx_sn_dat_v_i,
  input  local_dat_flit_t rx_sn_dat_flit_i,

  // -----------------------------------------
  // 3. Outgoing Flits (Req/Gnt to Central Crossbar)
  // -----------------------------------------
  output logic            tx_snp_req_o,
  input  logic            tx_snp_gnt_i,
  output local_snp_flit_t tx_snp_flit_o,

  output logic            tx_rsp_req_o,
  input  logic            tx_rsp_gnt_i,
  output local_rsp_flit_t tx_rsp_flit_o,

  output logic            tx_dat_req_o,
  input  logic            tx_dat_gnt_i,
  output local_dat_flit_t tx_dat_flit_o,

  output logic            tx_sn_req_req_o,
  input  logic            tx_sn_req_gnt_i,
  output local_req_flit_t tx_sn_req_flit_o,
  
  output logic            tx_sn_dat_req_o,
  input  logic            tx_sn_dat_gnt_i,
  output local_dat_flit_t tx_sn_dat_flit_o
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
    ST_DIRTY_WB_REQ_SEND,
    ST_DIRTY_WB_RSP_WAIT,
    ST_DIRTY_WB_DATA_SEND
  } state_e;

  state_e state_q, state_d;

  typedef enum logic [1:0] {
    TX_READ,
    TX_WRITE_UNIQUE,
    TX_WRITE_BACK
  } tx_type_e;
      
  tx_type_e tx_type_q, tx_type_d;

  logic                   req_idx_q, req_idx_d;
  logic                   snp_idx_q, snp_idx_d;
  logic                   is_write_q, is_write_d;
  logic [AddrWidth-1:0]   addr_q, addr_d;
  node_id_e               srcid_q, srcid_d;
  logic [11:0]            txnid_q, txnid_d;
  req_opcode_e            opcode_q, opcode_d;
  width_e                 size_q, size_d;
  logic [3:0]             qos_q, qos_d;

  logic [DataWidth-1:0]     data_q, data_d;
  logic [(DataWidth/8)-1:0] be_q, be_d;
  logic [11:0]              sn_dbid_q, sn_dbid_d;

  // New state registers for dirty writebacks
  logic [DataWidth-1:0]     dirty_data_q, dirty_data_d;
  logic                     dirty_hit_q, dirty_hit_d;

  assign alloc_rdy_o = (state_q == ST_IDLE);

  always_comb begin
    // --------------------------------------------------------
    // Default Assignments (Prevents Latches)
    // --------------------------------------------------------
    state_d      = state_q;
    tx_type_d    = tx_type_q;
    req_idx_d    = req_idx_q;
    snp_idx_d    = snp_idx_q;
    is_write_d   = is_write_q;
    addr_d       = addr_q;
    srcid_d      = srcid_q;
    txnid_d      = txnid_q;
    opcode_d     = opcode_q;
    size_d       = size_q;
    qos_d        = qos_q;
    data_d       = data_q;
    be_d         = be_q;
    sn_dbid_d    = sn_dbid_q;
    dirty_data_d = dirty_data_q;
    dirty_hit_d  = dirty_hit_q;

    // Default Output Flits to 0
    tx_snp_req_o     = 1'b0;
    tx_snp_flit_o    = '0;
    tx_rsp_req_o     = 1'b0;
    tx_rsp_flit_o    = '0;
    tx_dat_req_o     = 1'b0;
    tx_dat_flit_o    = '0;
    tx_sn_req_req_o  = 1'b0;
    tx_sn_req_flit_o = '0;
    tx_sn_dat_req_o  = 1'b0;
    tx_sn_dat_flit_o = '0;

    // --------------------------------------------------------
    // State Machine
    // --------------------------------------------------------
    case (state_q)
      ST_IDLE: begin
        dirty_hit_d = 1'b0; // Reset dirty flag
        if (alloc_v_i == 1'b1) begin
          req_idx_d = alloc_req_idx_i;
          addr_d    = alloc_req_flit_i.addr;
          srcid_d   = alloc_req_flit_i.srcid;
          txnid_d   = alloc_req_flit_i.txnid;
          opcode_d  = alloc_req_flit_i.opcode;
          size_d    = alloc_req_flit_i.size;
          qos_d     = alloc_req_flit_i.qos;
          state_d   = ST_ALLOC;
        end
      end

      ST_ALLOC: begin
        snp_idx_d = ~req_idx_q;
        if (opcode_q == REQ_WRITE_BACK_FULL || opcode_q == REQ_WRITE_CLEAN_FULL) begin
          tx_type_d  = TX_WRITE_BACK;
          is_write_d = 1'b1;
          state_d    = ST_DBIDRESP_SEND; 
        end else if (opcode_q == REQ_WRITE_UNIQUE_PTL || opcode_q == REQ_WRITE_UNIQUE_FULL) begin
          tx_type_d  = TX_WRITE_UNIQUE;
          is_write_d = 1'b1;
          state_d    = ST_SNOOP_SEND;
        end else begin
          tx_type_d  = TX_READ;
          is_write_d = 1'b0;
          state_d    = ST_SNOOP_SEND;
        end
      end

      ST_SNOOP_SEND: begin
        tx_snp_req_o            = 1'b1;
        tx_snp_flit_o.qos       = qos_q;
        tx_snp_flit_o.srcid     = HNFID;
        tx_snp_flit_o.txnid     = txnid_q;
        tx_snp_flit_o.fwdnid    = '0;
        tx_snp_flit_o.fwdtxnid  = '0;
        tx_snp_flit_o.addr      = addr_q;
        
        if (tx_type_q == TX_READ) begin
          tx_snp_flit_o.rettosrc = 1'b1; 
          // Scenario G2 Fix
          if (opcode_q == REQ_READ_UNIQUE) begin
            tx_snp_flit_o.opcode = SNP_UNIQUE; 
          end else begin
            tx_snp_flit_o.opcode = SNP_SHARED; 
          end
        end else begin
          tx_snp_flit_o.opcode   = SNP_UNIQUE;
          tx_snp_flit_o.rettosrc = 1'b0; 
        end
        
        if (tx_snp_gnt_i == 1'b1) begin
          state_d = ST_SNOOP_WAIT;
        end
      end

      ST_SNOOP_WAIT: begin 
        if (tx_type_q == TX_READ) begin
          // Scenario F Fix (Read Hit Modified)
          if ((rx_dat_v_i == 1'b1) && (rx_dat_flit_i.opcode == DAT_SNP_RESP_DATA) && (rx_dat_flit_i.txnid == txnid_q)) begin
            data_d       = rx_dat_flit_i.data;
            be_d         = rx_dat_flit_i.be;
            dirty_data_d = rx_dat_flit_i.data; 
            dirty_hit_d  = 1'b1; 
            state_d      = ST_RDATA_SEND;
          end
          // Read miss
          else if ((rx_rsp_v_i == 1'b1) && (rx_rsp_flit_i.opcode == RSP_SNP_RESP) && (rx_rsp_flit_i.txnid == txnid_q)) begin
            state_d = ST_SN_REQ_SEND;
          end
        end else begin
          // Write Hit Clean / Miss
          if ((rx_rsp_v_i == 1'b1) && (rx_rsp_flit_i.opcode == RSP_SNP_RESP) && (rx_rsp_flit_i.txnid == txnid_q)) begin
            state_d = ST_DBIDRESP_SEND;
          end
          // Scenario E Fix (Write Hit Modified)
          else if ((rx_dat_v_i == 1'b1) && (rx_dat_flit_i.opcode == DAT_SNP_RESP_DATA) && (rx_dat_flit_i.txnid == txnid_q)) begin 
            dirty_data_d = rx_dat_flit_i.data; 
            state_d      = ST_DIRTY_WB_REQ_SEND; 
          end
        end
      end
      
      ST_DIRTY_WB_REQ_SEND: begin
        tx_sn_req_req_o            = 1'b1;
        tx_sn_req_flit_o.qos       = qos_q;
        tx_sn_req_flit_o.tgtid     = SNFID;
        tx_sn_req_flit_o.srcid     = HNFID;
        tx_sn_req_flit_o.txnid     = txnid_q;
        tx_sn_req_flit_o.size      = size_q;
        tx_sn_req_flit_o.addr      = addr_q;
        tx_sn_req_flit_o.allowretry = Y;
        tx_sn_req_flit_o.pcrdttype = '0;
        tx_sn_req_flit_o.memattr   = '0;
        tx_sn_req_flit_o.opcode    = REQ_WRITE_NO_SNOOP_FULL; 
        
        if (tx_sn_req_gnt_i == 1'b1) begin
          state_d = ST_DIRTY_WB_RSP_WAIT;
        end
      end

      ST_DIRTY_WB_RSP_WAIT: begin
        if ((rx_sn_rsp_v_i == 1'b1) && 
            (rx_sn_rsp_flit_i.opcode == RSP_COMP_DBID_RESP) && 
            (rx_sn_rsp_flit_i.txnid == txnid_q)) begin
          sn_dbid_d = rx_sn_rsp_flit_i.dbid; 
          state_d   = ST_DIRTY_WB_DATA_SEND;
        end
      end

      ST_DIRTY_WB_DATA_SEND: begin
        tx_sn_dat_req_o           = 1'b1;
        tx_sn_dat_flit_o.qos      = qos_q;
        tx_sn_dat_flit_o.tgtid    = SNFID;
        tx_sn_dat_flit_o.srcid    = HNFID;
        tx_sn_dat_flit_o.txnid    = txnid_q; 
        tx_sn_dat_flit_o.dbid     = sn_dbid_q; 
        tx_sn_dat_flit_o.be       = {(DataWidth/8){1'b1}}; 
        tx_sn_dat_flit_o.data     = dirty_data_q;          
        tx_sn_dat_flit_o.opcode   = DAT_COPY_BACK_WR_DATA;
        
        if (tx_sn_dat_gnt_i == 1'b1) begin
          if (tx_type_q == TX_READ) begin
            state_d = ST_DONE; 
          end else begin
            state_d = ST_DBIDRESP_SEND; 
          end
        end
      end
      
      ST_DBIDRESP_SEND: begin
        tx_rsp_req_o           = 1'b1;
        tx_rsp_flit_o.qos      = qos_q;
        tx_rsp_flit_o.tgtid    = srcid_q;
        tx_rsp_flit_o.srcid    = HNFID;
        tx_rsp_flit_o.txnid    = txnid_q;
        tx_rsp_flit_o.dbid     = txnid_q;
        
        if (tx_type_q == TX_WRITE_BACK) begin
          tx_rsp_flit_o.opcode = RSP_COMP_DBID_RESP;
        end else begin
          tx_rsp_flit_o.opcode = RSP_DBID_RESP;
        end
        
        if (tx_rsp_gnt_i == 1'b1) begin
          state_d = ST_WDATA_WAIT;
        end
      end

      ST_WDATA_WAIT: begin 
        if ((rx_dat_v_i == 1'b1) && (rx_dat_flit_i.txnid == txnid_q)) begin
          data_d  = rx_dat_flit_i.data;
          be_d    = rx_dat_flit_i.be;
          state_d = ST_SN_REQ_SEND;
        end
      end

      ST_SN_REQ_SEND: begin
        tx_sn_req_req_o            = 1'b1;
        tx_sn_req_flit_o.qos       = qos_q;
        tx_sn_req_flit_o.tgtid     = SNFID;
        tx_sn_req_flit_o.srcid     = HNFID;
        tx_sn_req_flit_o.txnid     = txnid_q;
        tx_sn_req_flit_o.size      = size_q;
        tx_sn_req_flit_o.addr      = addr_q;
        tx_sn_req_flit_o.allowretry = Y;
        tx_sn_req_flit_o.pcrdttype = '0;
        tx_sn_req_flit_o.memattr   = '0;

        if (tx_type_q == TX_READ) begin
          tx_sn_req_flit_o.opcode = REQ_READ_NO_SNOOP;
        end else if (tx_type_q == TX_WRITE_BACK) begin
          tx_sn_req_flit_o.opcode = REQ_WRITE_NO_SNOOP_FULL;
        end else begin
          tx_sn_req_flit_o.opcode = REQ_WRITE_NO_SNOOP_PTL;
        end
        
        if (tx_sn_req_gnt_i == 1'b1) begin
          state_d = ST_SN_RSP_WAIT;
        end
      end

      ST_SN_RSP_WAIT: begin
        if (tx_type_q == TX_READ) begin
          if ((rx_sn_dat_v_i == 1'b1) && (rx_sn_dat_flit_i.opcode == DAT_COMP_DATA) && (rx_sn_dat_flit_i.txnid == txnid_q)) begin
            data_d  = rx_sn_dat_flit_i.data;
            be_d    = rx_sn_dat_flit_i.be;
            state_d = ST_RDATA_SEND;
          end
        end else begin
          if ((rx_sn_rsp_v_i == 1'b1) && (rx_sn_rsp_flit_i.opcode == RSP_COMP_DBID_RESP) && (rx_sn_rsp_flit_i.txnid == txnid_q)) begin
            sn_dbid_d = rx_sn_rsp_flit_i.dbid; 
            state_d   = ST_SN_DATA_SEND;
          end
        end
      end

      ST_SN_DATA_SEND: begin
        tx_sn_dat_req_o           = 1'b1;
        tx_sn_dat_flit_o.qos      = qos_q;
        tx_sn_dat_flit_o.tgtid    = SNFID;
        tx_sn_dat_flit_o.srcid    = HNFID;
        tx_sn_dat_flit_o.txnid    = txnid_q; 
        tx_sn_dat_flit_o.dbid     = sn_dbid_q; 
        tx_sn_dat_flit_o.be       = be_q;
        tx_sn_dat_flit_o.data     = data_q;
        
        if (tx_type_q == TX_WRITE_BACK) begin
          tx_sn_dat_flit_o.opcode = DAT_COPY_BACK_WR_DATA;
        end else begin
          tx_sn_dat_flit_o.opcode = DAT_NON_COPY_BACK_WR_DATA;
        end
        
        if (tx_sn_dat_gnt_i == 1'b1) begin
           if (tx_type_q == TX_WRITE_BACK) begin
             state_d = ST_DONE;
           end else begin
             state_d = ST_COMP_SEND;
           end
        end
      end

      ST_RDATA_SEND: begin
        tx_dat_req_o           = 1'b1;
        tx_dat_flit_o.qos      = qos_q;
        tx_dat_flit_o.tgtid    = srcid_q;
        tx_dat_flit_o.srcid    = HNFID;
        tx_dat_flit_o.txnid    = txnid_q;
        tx_dat_flit_o.opcode   = DAT_COMP_DATA;
        tx_dat_flit_o.be       = be_q;
        tx_dat_flit_o.data     = data_q;
        
        if (tx_dat_gnt_i == 1'b1) begin
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMP_SEND: begin
        tx_rsp_req_o           = 1'b1;
        tx_rsp_flit_o.qos      = qos_q;
        tx_rsp_flit_o.tgtid    = srcid_q;
        tx_rsp_flit_o.srcid    = HNFID;
        tx_rsp_flit_o.txnid    = txnid_q;
        tx_rsp_flit_o.opcode   = RSP_COMP;
        
        if (tx_rsp_gnt_i == 1'b1) begin
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMPACK_WAIT: begin
        if ((rx_rsp_v_i == 1'b1) && (rx_rsp_flit_i.opcode == RSP_COMP_ACK) && (rx_rsp_flit_i.txnid == txnid_q)) begin
          if (dirty_hit_q == 1'b1) begin
            state_d = ST_DIRTY_WB_REQ_SEND;
          end else begin
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
      state_q      <= ST_IDLE;
      tx_type_q    <= TX_READ;
      req_idx_q    <= 1'b0;
      snp_idx_q    <= 1'b0;
      addr_q       <= '0;
      srcid_q      <= '0;
      txnid_q      <= '0;
      opcode_q     <= req_opcode_e'(8'h00);
      size_q       <= WIDTH_1;
      qos_q        <= '0;
      data_q       <= '0;
      be_q         <= '0;
      sn_dbid_q    <= '0;
      dirty_hit_q  <= 1'b0;
      dirty_data_q <= '0;
    end else begin
      state_q      <= state_d;
      tx_type_q    <= tx_type_d; 
      req_idx_q    <= req_idx_d;
      snp_idx_q    <= snp_idx_d;
      addr_q       <= addr_d;
      srcid_q      <= srcid_d;
      txnid_q      <= txnid_d;
      opcode_q     <= opcode_d;
      size_q       <= size_d;
      qos_q        <= qos_d;
      data_q       <= data_d;
      be_q         <= be_d;
      sn_dbid_q    <= sn_dbid_d;
      dirty_hit_q  <= dirty_hit_d;
      dirty_data_q <= dirty_data_d;
    end 
  end

endmodule