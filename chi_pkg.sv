


package chi_pkg;
   
  //---------------------------------------------------------- -----------
  // Node identifier
  //---------------------------------------------------------------------
  // Plain ID type, NOT an enum -- CHI node IDs are arbitrary numeric tags
  // (e.g. HNFID = 7'h40), so an enum type would reject that assignment
  // without an explicit member for every legal ID.
  typedef logic [6:0] node_id_e;

  //---------------------------------------------------------------------
  // REQ channel MemAttr field
  //---------------------------------------------------------------------
  // {Device, EWA, Allocate, Cacheable} bundled as a flat vector for now.
  // Kept as its own typedef (rather than inlining logic[3:0] at every use
  // site) so it can be split into a packed struct later without touching
  // every module that just does `memattr = '0`.
  typedef logic [3:0] mem_attr_e;

  //---------------------------------------------------------------------
  // REQ Channel Opcodes
  //---------------------------------------------------------------------
  typedef enum logic [7:0] {
        REQ_READ_SHARED         = 8'h01,
        REQ_READ_CLEAN          = 8'h02,
        REQ_READ_ONCE           = 8'h03,
        REQ_READ_NO_SNOOP       = 8'h04,
        REQ_READ_UNIQUE         = 8'h07,
        REQ_CLEAN_SHARED        = 8'h08,
        REQ_CLEAN_INVALID       = 8'h09,
        REQ_MAKE_INVALID        = 8'h0D,
        REQ_WRITE_BACK_FULL     = 8'h18,
        REQ_WRITE_CLEAN_FULL    = 8'h19,
        REQ_WRITE_UNIQUE_FULL   = 8'h1A,
        REQ_WRITE_UNIQUE_PTL    = 8'h1B,
        REQ_WRITE_NO_SNOOP_FULL = 8'h1C,
        REQ_WRITE_NO_SNOOP_PTL  = 8'h1D,
        REQ_ATOMIC_STORE        = 8'h20,
        REQ_ATOMIC_LOAD         = 8'h21,
        REQ_ATOMIC_SWAP         = 8'h22,
        REQ_ATOMIC_COMPARE      = 8'h23,
        REQ_DVM_OP              = 8'h24
    } req_opcode_e;

  //---------------------------------------------------------------------
  // RSP Channel Opcodes
  //---------------------------------------------------------------------
  // NOTE: the original package had RSP_COMP_DATA / RSP_DATA_SEPDATA on
  // this channel with the *same* encoding as their DAT-channel
  // counterparts below. CompData/DataSepData carry a data payload and
  // only ever appear on the DAT channel in CHI - they were removed from
  // here. In their place, SnpResp / DBIDResp / CompDBIDResp were added:
  // these are real RSP-channel opcodes that were missing entirely, even
  // though the write-with-separate-response flow depends on them.
  typedef enum logic [7:0] {
        RSP_SNP_RESP        = 8'h01,
        RSP_DBID_RESP       = 8'h02,
        RSP_COMP_DBID_RESP  = 8'h03,
        RSP_COMP_ACK        = 8'h14,
        RSP_READ_RECEIPT    = 8'h15,
        RSP_COMP            = 8'h16,
        RSP_RETRY_ACK       = 8'h17,
        RSP_PCR_GRANT_ACK   = 8'h18
    } rsp_opcode_e;

  //---------------------------------------------------------------------
  // DAT Channel Opcodes
  //---------------------------------------------------------------------
  // SNP_RESP_DATA added - needed by the read-with-snoop-data flow and
  // previously missing.
  typedef enum logic [7:0] {
        DAT_DATA_FLIT             = 8'h00,
        DAT_SNP_RESP_DATA         = 8'h01,
        DAT_COMP_DATA             = 8'h04,
        DAT_DATA_SEP_DATA         = 8'h05,
        DAT_NON_COPY_BACK_WR_DATA = 8'h06,
        DAT_COPY_BACK_WR_DATA     = 8'h07
    } dat_opcode_e;

  //---------------------------------------------------------------------
  // SNP Channel Opcodes
  //---------------------------------------------------------------------
  // Left intentionally over-complete (Clean/Once/NotSharedDirty/DVM,
  // DCT forwarding variants, etc.) even though the current TSHR only
  // drives SNP_UNIQUE - kept for future scalability as requested.
  typedef enum logic [7:0] {
        SNP_SHARED               = 8'h20,
        SNP_CLEAN                = 8'h21,
        SNP_ONCE                 = 8'h22,
        SNP_NOT_SHARED_DIRTY     = 8'h23,
        SNP_UNIQUE               = 8'h24,
        SNP_CLEAN_SHARED         = 8'h25,
        SNP_CLEAN_INVALID        = 8'h26,
        SNP_MAKE_INVALID         = 8'h27,
        SNP_DVM_OP               = 8'h28,
        // Forward Snoop Opcodes for DCT (Direct Cache Transfer)
        SNP_FWD_SHARED           = 8'h30,
        SNP_FWD_CLEAN            = 8'h31,
        SNP_FWD_ONCE             = 8'h32,
        SNP_FWD_NOT_SHARED_DIRTY = 8'h33,
        SNP_FWD_UNIQUE           = 8'h34,
        SNP_FWD_CLEAN_SHARED     = 8'h35,
        SNP_FWD_CLEAN_INVALID    = 8'h36,
        SNP_FWD_MAKE_INVALID     = 8'h37
    } snp_opcode_e;
    
endpackage : chi_pkg