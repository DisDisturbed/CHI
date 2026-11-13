`timescale 1ns/1ps
package common_pkg;

  typedef enum logic {
    N = 1'b0, // guess: CHI Y/N status encoding
    Y = 1'b1  // guess: CHI Y/N status encoding
  } yn_status_e;

  typedef enum logic [2:0] {
    WIDTH_1  = 3'b000, // guess: CHI Size field, 1 byte
    WIDTH_2  = 3'b001, // guess: CHI Size field, 2 bytes
    WIDTH_4  = 3'b010, // guess: CHI Size field, 4 bytes
    WIDTH_8  = 3'b011, // guess: CHI Size field, 8 bytes
    WIDTH_16 = 3'b100, // guess: CHI Size field, 16 bytes
    WIDTH_32 = 3'b101, // guess: CHI Size field, 32 bytes
    WIDTH_64 = 3'b110  // guess: CHI Size field, 64 bytes
  } width_e;
   

endpackage




package chi_pkg;
   
  typedef logic [6:0] node_id_e;

  typedef logic [3:0] mem_attr_e;

  // ---------------------------------------------------------------------------
  // Opcode values are the CHI ones (IHI0050 Issue E, opcode tables in B13).
  // The fields are kept 8 bits wide here; in the spec they are REQ 7, RSP 5,
  // DAT 4 and SNP 5 bits, and every value below fits those widths.
  // Always use these names, never the numbers.
  // ---------------------------------------------------------------------------
  typedef enum logic [7:0] {
        REQ_READ_SHARED         = 8'h01,
        REQ_READ_CLEAN          = 8'h02,
        REQ_READ_ONCE           = 8'h03,
        REQ_READ_NO_SNOOP       = 8'h04,
        REQ_READ_UNIQUE         = 8'h07,
        REQ_CLEAN_SHARED        = 8'h08,
        REQ_CLEAN_INVALID       = 8'h09,
        REQ_MAKE_INVALID        = 8'h0A,
        REQ_CLEAN_UNIQUE        = 8'h0B,
        REQ_MAKE_UNIQUE         = 8'h0C,
        REQ_EVICT               = 8'h0D,
        REQ_DVM_OP              = 8'h14,
        REQ_WRITE_CLEAN_FULL    = 8'h17,
        REQ_WRITE_UNIQUE_PTL    = 8'h18,
        REQ_WRITE_UNIQUE_FULL   = 8'h19,
        REQ_WRITE_BACK_PTL      = 8'h1A,
        REQ_WRITE_BACK_FULL     = 8'h1B,
        REQ_WRITE_NO_SNOOP_PTL  = 8'h1C,
        REQ_WRITE_NO_SNOOP_FULL = 8'h1D,
        // Atomics: the low 3 bits of AtomicStore / AtomicLoad select the
        // operation (ADD, CLR, EOR, SET, SMAX, SMIN, UMAX, UMIN); these are
        // the first (ADD) code of each group
        REQ_ATOMIC_STORE        = 8'h28,
        REQ_ATOMIC_LOAD         = 8'h30,
        REQ_ATOMIC_SWAP         = 8'h38,
        REQ_ATOMIC_COMPARE      = 8'h39,
        REQ_READ_NOT_SHARED_DIRTY = 8'h26
        // not modelled: WriteEvictFull (0x15), WriteCleanPtl, stash, ...
    } req_opcode_e;


  typedef enum logic [7:0] {
        RSP_SNP_RESP        = 8'h01,
        RSP_COMP_ACK        = 8'h02,
        RSP_RETRY_ACK       = 8'h03,
        RSP_COMP            = 8'h04,
        RSP_COMP_DBID_RESP  = 8'h05,
        RSP_DBID_RESP       = 8'h06,
        RSP_PCR_GRANT_ACK   = 8'h07,   // PCrdGrant
        RSP_READ_RECEIPT    = 8'h08
    } rsp_opcode_e;


  typedef enum logic [7:0] {
        DAT_LCRD_RETURN           = 8'h00,   // DataLCrdReturn
        DAT_SNP_RESP_DATA         = 8'h01,
        DAT_COPY_BACK_WR_DATA     = 8'h02,
        DAT_NON_COPY_BACK_WR_DATA = 8'h03,
        DAT_COMP_DATA             = 8'h04,
        DAT_SNP_RESP_DATA_PTL     = 8'h05,   // snoop data from a UDP (partial dirty) line
        DAT_DATA_SEP_RESP         = 8'h0B
    } dat_opcode_e;

  typedef enum logic [7:0] {
        SNP_SHARED               = 8'h01,
        SNP_CLEAN                = 8'h02,
        SNP_ONCE                 = 8'h03,
        SNP_NOT_SHARED_DIRTY     = 8'h04,
        SNP_UNIQUE               = 8'h07,
        SNP_CLEAN_SHARED         = 8'h08,
        SNP_CLEAN_INVALID        = 8'h09,
        SNP_MAKE_INVALID         = 8'h0A,
        SNP_DVM_OP               = 8'h0D,
        // Forwarding snoops for DCT (Direct Cache Transfer). NEVER TESTED.
        // The spec has no forwarding form of CleanShared / CleanInvalid / MakeInvalid.
        SNP_FWD_SHARED           = 8'h11,
        SNP_FWD_CLEAN            = 8'h12,
        SNP_FWD_ONCE             = 8'h13,
        SNP_FWD_NOT_SHARED_DIRTY = 8'h14,
        SNP_FWD_UNIQUE           = 8'h17
    } snp_opcode_e;

  // ---------------------------------------------------------------------------
  // Resp[2:0] field encodings (IHI0050 B4.5). The meaning of an encoding
  // depends on the message it is carried on, so several names share a value:
  //   CompData / Comp       : I, SC, UC, UD_PD, SD_PD
  //     ReadNotSharedDirty  : SC, UC or UD_PD (never SD_PD)
  //     Comp to CleanUnique / MakeUnique: UC
  //     Comp to Evict       : I
  //   SnpResp               : I, SC, UC (=UD on SnpResp), SD
  //                           a UCE line always answers I
  //   SnpRespData           : I, SC, UC, SD, I_PD, SC_PD, UD_PD, SD_PD
  //   SnpRespDataPtl        : I_PD or UD only (sent from a UDP line)
  //   CopyBackWrData        : I, SC, UC, UD_PD, SD_PD
  //                           WriteBackPtl from UDP sends UD_PD
  //   NonCopyBackWrData etc.: always I (3'b000)
  // The flit structs keep resp as plain logic [2:0]; assign these directly.
  // ---------------------------------------------------------------------------
  localparam logic [2:0] RESP_I     = 3'b000;
  localparam logic [2:0] RESP_SC    = 3'b001;
  localparam logic [2:0] RESP_UC    = 3'b010;
  localparam logic [2:0] RESP_UD    = 3'b010;  // SnpResp / SnpRespDataPtl (same code as UC)
  localparam logic [2:0] RESP_SD    = 3'b011;
  localparam logic [2:0] RESP_I_PD  = 3'b100;
  localparam logic [2:0] RESP_SC_PD = 3'b101;
  localparam logic [2:0] RESP_UC_PD = 3'b110;  // SnpRespData only
  localparam logic [2:0] RESP_UD_PD = 3'b110;  // CompData / CopyBackWrData / SnpRespData
  localparam logic [2:0] RESP_SD_PD = 3'b111;

  // RespErr[1:0] field encodings (IHI0050 B9.4)
  localparam logic [1:0] RESPERR_OK    = 2'b00; // Normal Okay
  localparam logic [1:0] RESPERR_EXOK  = 2'b01; // Exclusive Okay
  localparam logic [1:0] RESPERR_DERR  = 2'b10; // Data Error
  localparam logic [1:0] RESPERR_NDERR = 2'b11; // Non-data Error

  // cache line states as tracked by an RN-F (and by an HN-F snoop filter /
  // directory). Not a flit field - use it for the snoop filter you still need.
  typedef enum logic [2:0] {
    CS_I  = 3'd0, // Invalid
    CS_SC = 3'd1, // Shared Clean
    CS_UC = 3'd2, // Unique Clean
    CS_UD  = 3'd3, // Unique Dirty
    CS_SD  = 3'd4, // Shared Dirty
    CS_UCE = 3'd5, // Unique Clean Empty: unique, no valid data (after CleanUnique from I)
    CS_UDP = 3'd6  // Unique Dirty Partial: unique, only some bytes valid, all of those dirty
  } cache_state_e;
  // For an HN-F snoop filter UCE and UDP count as unique, like UC/UD: tracking
  // I / shared / unique per RN-F is enough to pick the right snoops.

  // MemAttr[3:0] bit positions: {Allocate, Cacheable, Device, EWA}
  localparam int MEMATTR_EWA_BIT       = 0;
  localparam int MEMATTR_DEVICE_BIT    = 1;
  localparam int MEMATTR_CACHEABLE_BIT = 2;
  localparam int MEMATTR_ALLOCATE_BIT  = 3;
  localparam mem_attr_e MEMATTR_NORMAL_WB = 4'b1101; // Allocate, Cacheable, EWA - what the TB RN-Fs send on coherent requests
  localparam mem_attr_e MEMATTR_NORMAL_NC = 4'b0001; // non-cacheable, EWA       - what the TB RN-Fs send on ReadNoSnp / WriteNoSnp

endpackage : chi_pkg

package tshr_flit_pkg;
  import chi_pkg::*;

  localparam int TSHR_ADDR_WIDTH = 39;
  localparam int TSHR_DATA_WIDTH = 128;

  typedef struct packed {
    logic [3:0]                  qos;
    node_id_e                    tgtid;
    node_id_e                    srcid;
    logic [11:0]                 txnid;
    req_opcode_e                 opcode;
    common_pkg::width_e          size;
    logic [TSHR_ADDR_WIDTH-1:0]  addr;
    common_pkg::yn_status_e      allowretry;
    logic [3:0]                  pcrdttype;
    mem_attr_e                   memattr;
    logic                        expcompack;
  } local_req_flit_t;

  typedef struct packed {
    logic [3:0]                  qos;
    node_id_e                    srcid;
    logic [11:0]                 txnid;
    node_id_e                    fwdnid;
    logic [11:0]                 fwdtxnid;
    snp_opcode_e                 opcode;
    logic [TSHR_ADDR_WIDTH-1:0]  addr;
    logic                        ns;
    logic                        donotgotosd;
    logic                        rettosrc;
    logic                        tracetag;
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
    logic [3:0]                          qos;
    node_id_e                            tgtid;
    node_id_e                            srcid;
    logic [11:0]                         txnid;
    node_id_e                            homenid;
    dat_opcode_e                         opcode;
    logic [1:0]                          resperr;
    logic [2:0]                          resp;
    logic [2:0]                          fwdstate;
    logic [2:0]                          cbusy;
    logic [11:0]                         dbid;
    logic [1:0]                          ccid;
    logic [1:0]                          dataid;
    logic [1:0]                          tagop;
    logic [(TSHR_DATA_WIDTH/32)-1:0]     tag;
    logic [(TSHR_DATA_WIDTH/128)-1:0]    tu;
    logic                                tracetag;
    logic [(TSHR_DATA_WIDTH/8)-1:0]      be;
    logic [TSHR_DATA_WIDTH-1:0]          data;
  } local_dat_flit_t;

endpackage : tshr_flit_pkg

interface chi_req #(
    parameter AddrWidth = 39,
    parameter DataWidth = 128
);

  typedef struct packed {
    logic [3:0]             qos;
    chi_pkg::node_id_e      tgtid;
    chi_pkg::node_id_e      srcid;
    logic [11:0]            txnid;
    chi_pkg::req_opcode_e       opcode;
    common_pkg::width_e     size;
    logic [AddrWidth-1:0]   addr;
    common_pkg::yn_status_e allowretry;
    logic [3:0]             pcrdttype;
    chi_pkg::mem_attr_e     memattr;
    logic                   expcompack;
    //chi_pkg::snp_attr_e     snp_attr_e;
  } req_flit_t;

  common_pkg::yn_status_e flitv;
  common_pkg::yn_status_e flitpend;
  req_flit_t              flit;
  logic                   lcrdv;

  modport tx(output flitv, output flitpend, output flit, input lcrdv);
  modport rx(input flitv, input flitpend, input flit, output lcrdv);

endinterface

interface chi_rsp #(
    parameter AddrWidth = 39,
    parameter DataWidth = 128
);

  typedef struct packed {
    logic [3:0]        qos;
    chi_pkg::node_id_e tgtid;
    chi_pkg::node_id_e srcid;
    logic [11:0]       txnid;
    chi_pkg::rsp_opcode_e  opcode;
    logic [1:0]        resperr;
    logic [2:0]        resp;
    logic [2:0]        fwdstate;
    logic [2:0]        cbusy;
    logic [11:0]       dbid;
    logic [3:0]        pcrdttype;
    logic [1:0]        tagop;
    logic              tracetag;
  } rsp_flit_t;

  common_pkg::yn_status_e flitv;
  common_pkg::yn_status_e flitpend;
  rsp_flit_t              flit;
  logic                   lcrdv;

  modport tx(output flitv, output flitpend, output flit, input lcrdv);
  modport rx(input flitv, input flitpend, input flit, output lcrdv);

endinterface


interface chi_dat #(
    parameter AddrWidth = 39,
    parameter DataWidth = 128
);

  typedef struct packed {
    logic [3:0]                 qos;
    chi_pkg::node_id_e          tgtid;
    chi_pkg::node_id_e          srcid;
    logic [11:0]                txnid;
    chi_pkg::node_id_e          homenid;
    chi_pkg::dat_opcode_e           opcode;
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
  } dat_flit_t;

  common_pkg::yn_status_e flitv;
  common_pkg::yn_status_e flitpend;
  dat_flit_t              flit;
  logic                   lcrdv;

  modport tx(output flitv, output flitpend, output flit, input lcrdv);
  modport rx(input flitv, input flitpend, input flit, output lcrdv);

endinterface


interface chi_snp #(
    parameter AddrWidth = 39,
    parameter DataWidth = 128
);

  typedef struct packed {
    logic [3:0]           qos;
    chi_pkg::node_id_e    srcid;
    logic [11:0]          txnid;
    chi_pkg::node_id_e    fwdnid;
    logic [11:0]          fwdtxnid;
    chi_pkg::snp_opcode_e     opcode;
    logic [AddrWidth-1:0] addr;
    logic                 ns;
    logic                 donotgotosd;
    logic                 rettosrc;
    logic                 tracetag;
  } snp_flit_t;

  common_pkg::yn_status_e flitv;
  common_pkg::yn_status_e flitpend;
  snp_flit_t              flit;
  logic                   lcrdv;

  modport tx(output flitv, output flitpend, output flit, input lcrdv);
  modport rx(input flitv, input flitpend, input flit, output lcrdv);

endinterface