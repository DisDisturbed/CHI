module hnf_single_tracker (
    input logic clk,
    input logic rst_n,

    // RN-F0 Interfaces (Requester)
    chi_req.rx  rnf0_req,
    chi_rsp.tx  rnf0_rsp,
    chi_dat.rx  rnf0_dat_rx,
    chi_dat.tx  rnf0_dat_tx,

    // RN-F1 Interfaces (Snoopee)
    chi_snp.tx  rnf1_snp,
    chi_rsp.rx  rnf1_rsp,
    chi_dat.rx  rnf1_dat,

    // SN-F Interfaces (Memory)
    chi_req.tx  snf_req,
    chi_dat.tx  snf_dat,
    chi_rsp.rx  snf_rsp
);

  import chi_pkg::*;

  // Tracker State Enums
  typedef enum logic [3:0] {
    IDLE,
    
    // Flow 1 States (Write)
    F1_SEND_SNOOP,
    F1_WAIT_SNOOP_RSP,
    F1_SEND_DBID_RESP,
    F1_SEND_COMP,
    F1_WAIT_WRITE_DATA,
    F1_UPDATE_MEMORY_REQ,
    F1_UPDATE_MEMORY_DAT,
    F1_WAIT_MEMORY_ACK,

    // Flow 2 States (Read)
    F2_SEND_SNOOP,
    F2_WAIT_SNOOP_DATA,
    F2_FWD_DATA_TO_RNF0
  } tracker_state_e;

  tracker_state_e state, next_state;

  // Internal storage for the transaction
  logic [11:0] saved_txnid;
  logic [38:0] saved_addr;
  logic [127:0] saved_data; // Buffer for snooped or write data

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= IDLE;
    else        state <= next_state;
  end

  always_comb begin
    // Default assignments
    next_state = state;
    
    // Zero out all transmit valid signals by default
    rnf0_rsp.flitv = common_pkg::N;
    rnf0_dat_tx.flitv = common_pkg::N;
    rnf1_snp.flitv = common_pkg::N;
    snf_req.flitv = common_pkg::N;
    snf_dat.flitv = common_pkg::N;

    case (state)
      IDLE: begin
        if (rnf0_req.flitv == common_pkg::Y) begin
          // Decode Request Type
          if (rnf0_req.flit.opcode == REQ_WRITE_UNIQUE_FULL) begin
             next_state = F1_SEND_SNOOP; // Start Flow 1
          end 
          else if (rnf0_req.flit.opcode == REQ_READ_SHARED) begin
             next_state = F2_SEND_SNOOP; // Start Flow 2
          end
        end
      end

      // =================================================================
      // FLOW 1: Write with Snoop, Separate Responses, Memory Update
      // =================================================================
      F1_SEND_SNOOP: begin
        rnf1_snp.flitv = common_pkg::Y;
        rnf1_snp.flit.opcode = SNP_UNIQUE; // Invalidate RN-F1
        if (rnf1_snp.lcrdv) next_state = F1_WAIT_SNOOP_RSP;
      end

      F1_WAIT_SNOOP_RSP: begin
        // Wait for RN-F1 to say "Invalidated" (Assuming 8'h01 is added to your pkg for SNP_RESP)
        if (rnf1_rsp.flitv == common_pkg::Y && rnf1_rsp.flit.opcode == 8'h01) begin
          next_state = F1_SEND_DBID_RESP;
        end
      end

      F1_SEND_DBID_RESP: begin
        // Separate Response 1: Send DBID
        rnf0_rsp.flitv = common_pkg::Y;
        rnf0_rsp.flit.opcode = 8'h03; // RSP_DBID_RESP (Send me your data)
        rnf0_rsp.flit.dbid = 12'h001; // Example Tracker ID
        if (rnf0_rsp.lcrdv) next_state = F1_SEND_COMP;
      end

      F1_SEND_COMP: begin
        // Separate Response 2: Send Completion
        rnf0_rsp.flitv = common_pkg::Y;
        rnf0_rsp.flit.opcode = RSP_COMP; // Coherence complete
        if (rnf0_rsp.lcrdv) next_state = F1_WAIT_WRITE_DATA;
      end

      F1_WAIT_WRITE_DATA: begin
        // Wait for RN-F0 to send the payload
        if (rnf0_dat_rx.flitv == common_pkg::Y && 
           (rnf0_dat_rx.flit.opcode == DAT_NON_COPY_BACK_WR_DATA || rnf0_dat_rx.flit.opcode == DAT_COPY_BACK_WR_DATA)) begin
          next_state = F1_UPDATE_MEMORY_REQ;
        end
      end

      F1_UPDATE_MEMORY_REQ: begin
        // Send write command to SN-F
        snf_req.flitv = common_pkg::Y;
        snf_req.flit.opcode = REQ_WRITE_UNIQUE_FULL;
        if (snf_req.lcrdv) next_state = F1_UPDATE_MEMORY_DAT;
      end

      F1_UPDATE_MEMORY_DAT: begin
        // Send actual write data to SN-F
        snf_dat.flitv = common_pkg::Y;
        snf_dat.flit.opcode = DAT_NON_COPY_BACK_WR_DATA;
        if (snf_dat.lcrdv) next_state = F1_WAIT_MEMORY_ACK;
      end

      F1_WAIT_MEMORY_ACK: begin
        // Wait for memory controller to say it finished the write
        if (snf_rsp.flitv == common_pkg::Y) begin
          next_state = IDLE; // Flow 1 Complete
        end
      end

      // =================================================================
      // FLOW 2: Read with Snoop Partial Data, NO Memory Update
      // =================================================================
      F2_SEND_SNOOP: begin
        rnf1_snp.flitv = common_pkg::Y;
        rnf1_snp.flit.opcode = SNP_SHARED; // Ask RN-F1 for data
        if (rnf1_snp.lcrdv) next_state = F2_WAIT_SNOOP_DATA;
      end

      F2_WAIT_SNOOP_DATA: begin
        // Wait for RN-F1 to send partial data on DAT channel (Assuming 8'h05 is added for SNP_RESP_DATA_PTL)
        if (rnf1_dat.flitv == common_pkg::Y && rnf1_dat.flit.opcode == 8'h05) begin
          next_state = F2_FWD_DATA_TO_RNF0;
        end
      end

      F2_FWD_DATA_TO_RNF0: begin
        // Forward the exact snooped data to RN-F0
        rnf0_dat_tx.flitv = common_pkg::Y;
        rnf0_dat_tx.flit.opcode = DAT_COMP_DATA;
        
        // NO MEMORY UPDATE TO SN-F HAPPENS HERE. 
        // Once this data flit is accepted by RN-F0, we are done.
        if (rnf0_dat_tx.lcrdv) next_state = IDLE; // Flow 2 Complete
      end

      default: next_state = IDLE;
    endcase
  end

endmodule