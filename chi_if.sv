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