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
        REQ_DVM_OP              = 8'h24 // NO EVICTION IS INCLUDED CUZ I AM TIRED
    } req_opcode_e;


  typedef enum logic [7:0] {
        RSP_SNP_RESP        = 8'h01,
        RSP_DBID_RESP       = 8'h02,
        RSP_COMP_DBID_RESP  = 8'h03,
        RSP_COMP_ACK        = 8'h14,
        RSP_READ_RECEIPT    = 8'h15,
        RSP_COMP            = 8'h16,
        RSP_RETRY_ACK       = 8'h17, // THIS HAS NOT LOGIC BEHIND IT JUST INCLUDED IN PKG
        RSP_PCR_GRANT_ACK   = 8'h18  // THIS HAS NOT LOGIC BEHIND IT JUST INCLUDED IN PKG
    } rsp_opcode_e;


  typedef enum logic [7:0] {
        DAT_DATA_FLIT             = 8'h00,
        DAT_SNP_RESP_DATA         = 8'h01,
        DAT_COMP_DATA             = 8'h04,
        DAT_DATA_SEP_DATA         = 8'h05,
        DAT_NON_COPY_BACK_WR_DATA = 8'h06,
        DAT_COPY_BACK_WR_DATA     = 8'h07
    } dat_opcode_e;

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
        // NEVER TESTED
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