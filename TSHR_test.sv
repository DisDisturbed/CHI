`define DEMUX_TX_UNUSED 1
// (kept the old macro name reserved/unused as a marker - this version does
// its own explicit per-tracker arbitration + muxing instead of the single
// direct-drive DEMUX_TX macro from the original single-FSM TSHR.)

module TSHR // transaction snoop handling register - MULTI-TRACKER version
 import chi_pkg::*;
 import common_pkg::*; #(

  parameter int NUM_TRACKERS = 4,          // number of concurrent outstanding transactions supported
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

  localparam int NT     = NUM_TRACKERS;
  localparam int TIDX_W = (NT <= 1) ? 1 : $clog2(NT);

  // Per-transaction state machine. This is exactly the same sequence as
  // the single-FSM TSHR (including the DIRTY_WB writeback path and the
  // ReadUnique->SnpUnique mapping fix), just instantiated NT times.
  typedef enum logic [4:0] {
    T_IDLE,
    T_SNOOP_SEND,
    T_SNOOP_WAIT,
    T_DBIDRESP_SEND,
    T_WDATA_WAIT,
    T_SN_REQ_SEND,
    T_SN_RSP_WAIT,
    T_SN_DATA_SEND,
    T_RDATA_SEND,
    T_COMP_SEND,
    T_COMPACK_WAIT,
    T_DONE,
    T_DIRTY_WB_REQ_SEND,
    T_DIRTY_WB_RSP_WAIT,
    T_DIRTY_WB_DATA_SEND
  } tstate_e;

  typedef enum logic [1:0] {
    TX_READ,
    TX_WRITE_UNIQUE,
    TX_WRITE_BACK
  } tx_type_e;

  // -------- local struct defs (same field layout as the original TSHR) --
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

  // -------- per-tracker storage --------
  tstate_e              tr_state_q    [NT], tr_state_d    [NT];
  logic                 tr_req_idx_q  [NT], tr_req_idx_d  [NT];
  logic                 tr_snp_idx_q  [NT], tr_snp_idx_d  [NT];
  logic                 tr_is_write_q [NT], tr_is_write_d [NT];
  tx_type_e             tr_tx_type_q  [NT], tr_tx_type_d  [NT];
  logic [AddrWidth-1:0] tr_addr_q     [NT], tr_addr_d     [NT];
  node_id_e             tr_srcid_q    [NT], tr_srcid_d    [NT];
  logic [11:0]          tr_txnid_q    [NT], tr_txnid_d    [NT];
  req_opcode_e          tr_opcode_q   [NT], tr_opcode_d   [NT];
  width_e               tr_size_q     [NT], tr_size_d     [NT];
  logic [3:0]           tr_qos_q      [NT], tr_qos_d      [NT];
  logic [DataWidth-1:0] tr_data_q     [NT], tr_data_d     [NT];
  logic [(DataWidth/8)-1:0] tr_be_q   [NT], tr_be_d       [NT];
  logic [11:0]          tr_sn_dbid_q  [NT], tr_sn_dbid_d  [NT];
  logic                 tr_dirty_hit_q[NT], tr_dirty_hit_d[NT];

  // -------- shared credit counters (aggregate, same role as before) -----
  logic [3:0] snp_credit_q [2];
  logic [3:0] rsp_credit_q [2];
  logic [3:0] dat_credit_q [2];
  logic [3:0] sn_req_credit_q;
  logic [3:0] sn_dat_credit_q;

  // -------- free-tracker detection / RX credit --------
  logic any_free;
  always_comb begin
    any_free = 1'b0;
    for (int t = 0; t < NT; t++) if (tr_state_q[t] == T_IDLE) any_free = 1'b1;
  end
  assign req_rx[0].lcrdv = any_free;
  assign req_rx[1].lcrdv = any_free;
  assign rsp_rx[0].lcrdv = 1'b1;
  assign rsp_rx[1].lcrdv = 1'b1;
  assign dat_rx[0].lcrdv = 1'b1;
  assign dat_rx[1].lcrdv = 1'b1;
  assign sn_rsp_rx.lcrdv = 1'b1;

  assign busy_o = !any_free;

  // -------- credit counters, driven by the final arbitrated tx signals --
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int p = 0; p < 2; p++) begin
        snp_credit_q[p] <= 4'd0;
        rsp_credit_q[p] <= 4'd0;
        dat_credit_q[p] <= 4'd0;
      end
    end else begin
      for (int p = 0; p < 2; p++) begin
        unique case ({snp_tx[p].lcrdv, (snp_tx[p].flitv == Y)})
          2'b10:   if (snp_credit_q[p] != 4'hF) snp_credit_q[p] <= snp_credit_q[p] + 1;
          2'b01:   snp_credit_q[p] <= snp_credit_q[p] - 4'd1;
          default: snp_credit_q[p] <= snp_credit_q[p];
        endcase
        unique case ({rsp_tx[p].lcrdv, (rsp_tx[p].flitv == Y)})
          2'b10:   if (rsp_credit_q[p] != 4'hF) rsp_credit_q[p] <= rsp_credit_q[p] + 1;
          2'b01:   rsp_credit_q[p] <= rsp_credit_q[p] - 4'd1;
          default: rsp_credit_q[p] <= rsp_credit_q[p];
        endcase
        unique case ({dat_tx[p].lcrdv, (dat_tx[p].flitv == Y)})
          2'b10:   if (dat_credit_q[p] != 4'hF) dat_credit_q[p] <= dat_credit_q[p] + 1;
          2'b01:   dat_credit_q[p] <= dat_credit_q[p] - 4'd1;
          default: dat_credit_q[p] <= dat_credit_q[p];
        endcase
      end
    end
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      sn_req_credit_q <= 4'd0;
      sn_dat_credit_q <= 4'd0;
    end else begin
      unique case ({sn_req_tx.lcrdv, (sn_req_tx.flitv == Y)})
        2'b10:   if (sn_req_credit_q != 4'hF) sn_req_credit_q <= sn_req_credit_q + 4'd1;
        2'b01:   sn_req_credit_q <= sn_req_credit_q - 4'd1;
        default: sn_req_credit_q <= sn_req_credit_q;
      endcase
      unique case ({sn_dat_tx.lcrdv, (sn_dat_tx.flitv == Y)})
        2'b10:   if (sn_dat_credit_q != 4'hF) sn_dat_credit_q <= sn_dat_credit_q + 4'd1;
        2'b01:   sn_dat_credit_q <= sn_dat_credit_q - 4'd1;
        default: sn_dat_credit_q <= sn_dat_credit_q;
      endcase
    end
  end

  // -------- per-tracker "wants" for shared/arbitrated output ports ------
  logic            tr_want_snp   [NT];
  local_snp_flit_t tr_snp_flit   [NT];

  logic            tr_want_rsp   [NT];  // DBIDResp / CompDBIDResp / Comp - all on rsp_tx[req_idx]
  local_rsp_flit_t tr_rsp_flit   [NT];

  logic            tr_want_dat   [NT];  // CompData proxy on dat_tx[req_idx]
  local_dat_flit_t tr_dat_flit   [NT];

  logic            tr_want_snreq [NT];  // sn_req_tx (single shared port)
  req_opcode_e     tr_snreq_op   [NT];

  logic            tr_want_sndat [NT];  // sn_dat_tx (single shared port)
  dat_opcode_e     tr_sndat_op   [NT];
  logic [(DataWidth/8)-1:0] tr_sndat_be [NT];

  logic            snp_grant   [NT];
  logic            rsp_grant   [NT];
  logic            dat_grant   [NT];
  logic            snreq_grant [NT];
  logic            sndat_grant [NT];

  // -------- allocation: at most one new transaction admitted per cycle --
  // Same round-robin policy between req_rx[0]/req_rx[1] as the original
  // single-FSM TSHR (req_gnt_idx toggling), just now targeting whichever
  // tracker slot happens to be free instead of the one global ST_IDLE.
  logic                 alloc_fire;
  logic [TIDX_W-1:0]    alloc_idx;
  logic                 alloc_port;
  logic [AddrWidth-1:0] alloc_addr;
  node_id_e             alloc_srcid;
  logic [11:0]          alloc_txnid;
  req_opcode_e          alloc_opcode;
  width_e               alloc_size;
  logic [3:0]           alloc_qos;

  logic req_gnt_idx_q, req_gnt_idx_d;

  always_comb begin
    alloc_fire    = 1'b0;
    alloc_idx     = '0;
    alloc_port    = 1'b0;
    alloc_addr    = '0;
    alloc_srcid   = '0;
    alloc_txnid   = '0;
    alloc_opcode  = req_opcode_e'(8'h00);
    alloc_size    = WIDTH_1;
    alloc_qos     = '0;
    req_gnt_idx_d = req_gnt_idx_q;

    for (int t = 0; t < NT; t++) begin
      if (!alloc_fire && tr_state_q[t] == T_IDLE) begin
        if (req_gnt_idx_q == 1'b0) begin
          if (req_rx[0].flitv == Y) begin
            alloc_fire  = 1'b1; alloc_idx = t[TIDX_W-1:0]; alloc_port = 1'b0;
            alloc_addr  = req_rx[0].flit.addr; alloc_srcid = req_rx[0].flit.srcid;
            alloc_txnid = req_rx[0].flit.txnid; alloc_opcode = req_rx[0].flit.opcode;
            alloc_size  = req_rx[0].flit.size;  alloc_qos    = req_rx[0].flit.qos;
            req_gnt_idx_d = 1'b1;
          end else if (req_rx[1].flitv == Y) begin
            alloc_fire  = 1'b1; alloc_idx = t[TIDX_W-1:0]; alloc_port = 1'b1;
            alloc_addr  = req_rx[1].flit.addr; alloc_srcid = req_rx[1].flit.srcid;
            alloc_txnid = req_rx[1].flit.txnid; alloc_opcode = req_rx[1].flit.opcode;
            alloc_size  = req_rx[1].flit.size;  alloc_qos    = req_rx[1].flit.qos;
          end
        end else begin
          if (req_rx[1].flitv == Y) begin
            alloc_fire  = 1'b1; alloc_idx = t[TIDX_W-1:0]; alloc_port = 1'b1;
            alloc_addr  = req_rx[1].flit.addr; alloc_srcid = req_rx[1].flit.srcid;
            alloc_txnid = req_rx[1].flit.txnid; alloc_opcode = req_rx[1].flit.opcode;
            alloc_size  = req_rx[1].flit.size;  alloc_qos    = req_rx[1].flit.qos;
            req_gnt_idx_d = 1'b0;
          end else if (req_rx[0].flitv == Y) begin
            alloc_fire  = 1'b1; alloc_idx = t[TIDX_W-1:0]; alloc_port = 1'b0;
            alloc_addr  = req_rx[0].flit.addr; alloc_srcid = req_rx[0].flit.srcid;
            alloc_txnid = req_rx[0].flit.txnid; alloc_opcode = req_rx[0].flit.opcode;
            alloc_size  = req_rx[0].flit.size;  alloc_qos    = req_rx[0].flit.qos;
          end
        end
      end
    end
  end

  // -------- per-tracker FSM --------
  genvar gt;
  generate
    for (gt = 0; gt < NT; gt++) begin : gen_tracker

      // this tracker's view into the rx channels (matched by txnid later)
      local_rsp_flit_t rx_rsp;
      logic            rx_rsp_v;
      local_dat_flit_t rx_dat_snp;
      logic            rx_dat_snp_v;
      local_dat_flit_t rx_dat_req;
      logic            rx_dat_req_v;

      always_comb begin
        rx_rsp   = (tr_state_q[gt] == T_SNOOP_WAIT) ?
                     local_rsp_flit_t'((tr_snp_idx_q[gt]==1'b0) ? rsp_rx[0].flit : rsp_rx[1].flit) :
                     local_rsp_flit_t'((tr_req_idx_q[gt]==1'b0) ? rsp_rx[0].flit : rsp_rx[1].flit);
        rx_rsp_v = (tr_state_q[gt] == T_SNOOP_WAIT) ?
                     ((tr_snp_idx_q[gt]==1'b0) ? (rsp_rx[0].flitv==Y) : (rsp_rx[1].flitv==Y)) :
                     ((tr_req_idx_q[gt]==1'b0) ? (rsp_rx[0].flitv==Y) : (rsp_rx[1].flitv==Y));

        rx_dat_snp   = local_dat_flit_t'((tr_snp_idx_q[gt]==1'b0) ? dat_rx[0].flit : dat_rx[1].flit);
        rx_dat_snp_v = (tr_snp_idx_q[gt]==1'b0) ? (dat_rx[0].flitv==Y) : (dat_rx[1].flitv==Y);

        rx_dat_req   = local_dat_flit_t'((tr_req_idx_q[gt]==1'b0) ? dat_rx[0].flit : dat_rx[1].flit);
        rx_dat_req_v = (tr_req_idx_q[gt]==1'b0) ? (dat_rx[0].flitv==Y) : (dat_rx[1].flitv==Y);
      end

      always_comb begin
        // defaults: hold everything
        tr_state_d[gt]      = tr_state_q[gt];
        tr_req_idx_d[gt]    = tr_req_idx_q[gt];
        tr_snp_idx_d[gt]    = tr_snp_idx_q[gt];
        tr_is_write_d[gt]   = tr_is_write_q[gt];
        tr_tx_type_d[gt]    = tr_tx_type_q[gt];
        tr_addr_d[gt]       = tr_addr_q[gt];
        tr_srcid_d[gt]      = tr_srcid_q[gt];
        tr_txnid_d[gt]      = tr_txnid_q[gt];
        tr_opcode_d[gt]     = tr_opcode_q[gt];
        tr_size_d[gt]       = tr_size_q[gt];
        tr_qos_d[gt]        = tr_qos_q[gt];
        tr_data_d[gt]       = tr_data_q[gt];
        tr_be_d[gt]         = tr_be_q[gt];
        tr_sn_dbid_d[gt]    = tr_sn_dbid_q[gt];
        tr_dirty_hit_d[gt]  = tr_dirty_hit_q[gt];

        tr_want_snp[gt]   = 1'b0; tr_snp_flit[gt] = '0;
        tr_want_rsp[gt]   = 1'b0; tr_rsp_flit[gt] = '0;
        tr_want_dat[gt]   = 1'b0; tr_dat_flit[gt] = '0;
        tr_want_snreq[gt] = 1'b0; tr_snreq_op[gt] = req_opcode_e'(8'h00);
        tr_want_sndat[gt] = 1'b0; tr_sndat_op[gt] = dat_opcode_e'(8'h00); tr_sndat_be[gt] = '0;

        case (tr_state_q[gt])

          T_IDLE: begin
            // nothing on its own - allocation (below) fills a free tracker
          end

          T_SNOOP_SEND: begin
            tr_want_snp[gt]          = 1'b1;
            tr_snp_flit[gt].qos      = tr_qos_q[gt];
            tr_snp_flit[gt].srcid    = HNFID;
            tr_snp_flit[gt].txnid    = tr_txnid_q[gt];
            tr_snp_flit[gt].fwdnid   = '0;
            tr_snp_flit[gt].fwdtxnid = '0;
            tr_snp_flit[gt].addr     = tr_addr_q[gt];
            if (tr_tx_type_q[gt] == TX_READ) begin
              tr_snp_flit[gt].rettosrc = 1'b1;
              tr_snp_flit[gt].opcode   = (tr_opcode_q[gt] == REQ_READ_UNIQUE) ? SNP_UNIQUE : SNP_SHARED;
            end else begin
              tr_snp_flit[gt].opcode   = SNP_UNIQUE;
              tr_snp_flit[gt].rettosrc = 1'b0;
            end
            if (snp_grant[gt]) tr_state_d[gt] = T_SNOOP_WAIT;
          end

          T_SNOOP_WAIT: begin
            if (tr_tx_type_q[gt] == TX_READ) begin
              if (rx_dat_snp_v && (rx_dat_snp.opcode == DAT_SNP_RESP_DATA) && (rx_dat_snp.txnid == tr_txnid_q[gt])) begin
                tr_data_d[gt]      = rx_dat_snp.data;
                tr_be_d[gt]        = rx_dat_snp.be;
                tr_dirty_hit_d[gt] = 1'b1;
                tr_state_d[gt]     = T_RDATA_SEND;
              end else if (rx_rsp_v && (rx_rsp.opcode == RSP_SNP_RESP) && (rx_rsp.txnid == tr_txnid_q[gt])) begin
                tr_state_d[gt] = T_SN_REQ_SEND;
              end
            end else begin
              if (rx_rsp_v && (rx_rsp.opcode == RSP_SNP_RESP) && (rx_rsp.txnid == tr_txnid_q[gt])) begin
                tr_state_d[gt] = T_DBIDRESP_SEND;
              end else if (rx_dat_snp_v && (rx_dat_snp.opcode == DAT_SNP_RESP_DATA) && (rx_dat_snp.txnid == tr_txnid_q[gt])) begin
                tr_data_d[gt]  = rx_dat_snp.data;
                tr_be_d[gt]    = rx_dat_snp.be;
                tr_state_d[gt] = T_DIRTY_WB_REQ_SEND;
              end
            end
          end

          T_DIRTY_WB_REQ_SEND: begin
            tr_want_snreq[gt] = 1'b1;
            tr_snreq_op[gt]   = REQ_WRITE_NO_SNOOP_FULL;
            if (snreq_grant[gt]) tr_state_d[gt] = T_DIRTY_WB_RSP_WAIT;
          end

          T_DIRTY_WB_RSP_WAIT: begin
            if ((sn_rsp_rx.flitv == Y) && (sn_rsp_rx.flit.opcode == RSP_COMP_DBID_RESP) && (sn_rsp_rx.flit.txnid == tr_txnid_q[gt])) begin
              tr_sn_dbid_d[gt] = sn_rsp_rx.flit.dbid;
              tr_state_d[gt]   = T_DIRTY_WB_DATA_SEND;
            end
          end

          T_DIRTY_WB_DATA_SEND: begin
            tr_want_sndat[gt] = 1'b1;
            tr_sndat_op[gt]   = DAT_COPY_BACK_WR_DATA;
            tr_sndat_be[gt]   = {(DataWidth/8){1'b1}};
            if (sndat_grant[gt]) begin
              tr_state_d[gt] = (tr_tx_type_q[gt] == TX_READ) ? T_DONE : T_DBIDRESP_SEND;
            end
          end

          T_DBIDRESP_SEND: begin
            tr_want_rsp[gt]        = 1'b1;
            tr_rsp_flit[gt].qos    = tr_qos_q[gt];
            tr_rsp_flit[gt].tgtid  = tr_srcid_q[gt];
            tr_rsp_flit[gt].srcid  = HNFID;
            tr_rsp_flit[gt].txnid  = tr_txnid_q[gt];
            tr_rsp_flit[gt].dbid   = tr_txnid_q[gt];
            tr_rsp_flit[gt].opcode = (tr_tx_type_q[gt] == TX_WRITE_BACK) ? RSP_COMP_DBID_RESP : RSP_DBID_RESP;
            if (rsp_grant[gt]) tr_state_d[gt] = T_WDATA_WAIT;
          end

          T_WDATA_WAIT: begin
            if (rx_dat_req_v && (rx_dat_req.txnid == tr_txnid_q[gt])) begin
              tr_data_d[gt]  = rx_dat_req.data;
              tr_be_d[gt]    = rx_dat_req.be;
              tr_state_d[gt] = T_SN_REQ_SEND;
            end
          end

          T_SN_REQ_SEND: begin
            tr_want_snreq[gt] = 1'b1;
            if (tr_tx_type_q[gt] == TX_READ)            tr_snreq_op[gt] = REQ_READ_NO_SNOOP;
            else if (tr_tx_type_q[gt] == TX_WRITE_BACK) tr_snreq_op[gt] = REQ_WRITE_NO_SNOOP_FULL;
            else                                         tr_snreq_op[gt] = REQ_WRITE_NO_SNOOP_PTL;
            if (snreq_grant[gt]) tr_state_d[gt] = T_SN_RSP_WAIT;
          end

          T_SN_RSP_WAIT: begin
            if (tr_tx_type_q[gt] == TX_READ) begin
              if ((sn_dat_rx.flitv == Y) && (sn_dat_rx.flit.opcode == DAT_COMP_DATA) && (sn_dat_rx.flit.txnid == tr_txnid_q[gt])) begin
                tr_data_d[gt]  = sn_dat_rx.flit.data;
                tr_be_d[gt]    = sn_dat_rx.flit.be;
                tr_state_d[gt] = T_RDATA_SEND;
              end
            end else begin
              if ((sn_rsp_rx.flitv == Y) && (sn_rsp_rx.flit.opcode == RSP_COMP_DBID_RESP) && (sn_rsp_rx.flit.txnid == tr_txnid_q[gt])) begin
                tr_sn_dbid_d[gt] = sn_rsp_rx.flit.dbid;
                tr_state_d[gt]   = T_SN_DATA_SEND;
              end
            end
          end

          T_SN_DATA_SEND: begin
            tr_want_sndat[gt] = 1'b1;
            tr_sndat_op[gt]   = (tr_tx_type_q[gt] == TX_WRITE_BACK) ? DAT_COPY_BACK_WR_DATA : DAT_NON_COPY_BACK_WR_DATA;
            tr_sndat_be[gt]   = tr_be_q[gt];
            if (sndat_grant[gt]) begin
              tr_state_d[gt] = (tr_tx_type_q[gt] == TX_WRITE_BACK) ? T_DONE : T_COMP_SEND;
            end
          end

          T_RDATA_SEND: begin
            tr_want_dat[gt]        = 1'b1;
            tr_dat_flit[gt].qos    = tr_qos_q[gt];
            tr_dat_flit[gt].tgtid  = tr_srcid_q[gt];
            tr_dat_flit[gt].srcid  = HNFID;
            tr_dat_flit[gt].txnid  = tr_txnid_q[gt];
            tr_dat_flit[gt].opcode = DAT_COMP_DATA;
            tr_dat_flit[gt].be     = tr_be_q[gt];
            tr_dat_flit[gt].data   = tr_data_q[gt];
            if (dat_grant[gt]) tr_state_d[gt] = T_COMPACK_WAIT;
          end

          T_COMP_SEND: begin
            tr_want_rsp[gt]        = 1'b1;
            tr_rsp_flit[gt].qos    = tr_qos_q[gt];
            tr_rsp_flit[gt].tgtid  = tr_srcid_q[gt];
            tr_rsp_flit[gt].srcid  = HNFID;
            tr_rsp_flit[gt].txnid  = tr_txnid_q[gt];
            tr_rsp_flit[gt].opcode = RSP_COMP;
            if (rsp_grant[gt]) tr_state_d[gt] = T_COMPACK_WAIT;
          end

          T_COMPACK_WAIT: begin
            if (rx_rsp_v && (rx_rsp.opcode == RSP_COMP_ACK) && (rx_rsp.txnid == tr_txnid_q[gt])) begin
              tr_state_d[gt] = tr_dirty_hit_q[gt] ? T_DIRTY_WB_REQ_SEND : T_DONE;
            end
          end

          T_DONE: begin
            tr_state_d[gt] = T_IDLE;
          end

          default: tr_state_d[gt] = T_IDLE;
        endcase

        // allocation of a free tracker overrides its (do-nothing) T_IDLE
        // default above - placed last so it always wins for that slot.
        if (alloc_fire && (alloc_idx == gt[TIDX_W-1:0])) begin
          tr_req_idx_d[gt]   = alloc_port;
          tr_snp_idx_d[gt]   = ~alloc_port;
          tr_addr_d[gt]      = alloc_addr;
          tr_srcid_d[gt]     = alloc_srcid;
          tr_txnid_d[gt]     = alloc_txnid;
          tr_opcode_d[gt]    = alloc_opcode;
          tr_size_d[gt]      = alloc_size;
          tr_qos_d[gt]       = alloc_qos;
          tr_dirty_hit_d[gt] = 1'b0;
          if (alloc_opcode == REQ_WRITE_BACK_FULL || alloc_opcode == REQ_WRITE_CLEAN_FULL) begin
            tr_tx_type_d[gt]  = TX_WRITE_BACK;
            tr_is_write_d[gt] = 1'b1;
            tr_state_d[gt]    = T_DBIDRESP_SEND;
          end else if (alloc_opcode == REQ_WRITE_UNIQUE_PTL || alloc_opcode == REQ_WRITE_UNIQUE_FULL) begin
            tr_tx_type_d[gt]  = TX_WRITE_UNIQUE;
            tr_is_write_d[gt] = 1'b1;
            tr_state_d[gt]    = T_SNOOP_SEND;
          end else begin
            tr_tx_type_d[gt]  = TX_READ;
            tr_is_write_d[gt] = 1'b0;
            tr_state_d[gt]    = T_SNOOP_SEND;
          end
        end
      end
    end
  endgenerate

  // -------- arbitration: lowest tracker index wins, gated by credit ------
  always_comb begin
    for (int t = 0; t < NT; t++) snp_grant[t] = 1'b0;
    for (int p = 0; p < 2; p++) begin : arb_snp
      logic granted;
      granted = 1'b0;
      for (int t = 0; t < NT; t++) begin
        if (!granted && tr_want_snp[t] && (tr_snp_idx_q[t] == p[0]) && (snp_credit_q[p] != 4'd0)) begin
          snp_grant[t] = 1'b1;
          granted = 1'b1;
        end
      end
    end
  end

  always_comb begin
    for (int t = 0; t < NT; t++) rsp_grant[t] = 1'b0;
    for (int p = 0; p < 2; p++) begin : arb_rsp
      logic granted;
      granted = 1'b0;
      for (int t = 0; t < NT; t++) begin
        if (!granted && tr_want_rsp[t] && (tr_req_idx_q[t] == p[0]) && (rsp_credit_q[p] != 4'd0)) begin
          rsp_grant[t] = 1'b1;
          granted = 1'b1;
        end
      end
    end
  end

  always_comb begin
    for (int t = 0; t < NT; t++) dat_grant[t] = 1'b0;
    for (int p = 0; p < 2; p++) begin : arb_dat
      logic granted;
      granted = 1'b0;
      for (int t = 0; t < NT; t++) begin
        if (!granted && tr_want_dat[t] && (tr_req_idx_q[t] == p[0]) && (dat_credit_q[p] != 4'd0)) begin
          dat_grant[t] = 1'b1;
          granted = 1'b1;
        end
      end
    end
  end

  always_comb begin
    for (int t = 0; t < NT; t++) snreq_grant[t] = 1'b0;
    begin : arb_snreq
      logic granted;
      granted = 1'b0;
      for (int t = 0; t < NT; t++) begin
        if (!granted && tr_want_snreq[t] && (sn_req_credit_q != 4'd0)) begin
          snreq_grant[t] = 1'b1;
          granted = 1'b1;
        end
      end
    end
  end

  always_comb begin
    for (int t = 0; t < NT; t++) sndat_grant[t] = 1'b0;
    begin : arb_sndat
      logic granted;
      granted = 1'b0;
      for (int t = 0; t < NT; t++) begin
        if (!granted && tr_want_sndat[t] && (sn_dat_credit_q != 4'd0)) begin
          sndat_grant[t] = 1'b1;
          granted = 1'b1;
        end
      end
    end
  end

  // -------- final output muxing (drive the physical interfaces) --------
  always_comb begin
    for (int p = 0; p < 2; p++) begin
      snp_tx[p].flitv    = N;
      snp_tx[p].flitpend = N;
      snp_tx[p].flit     = '0;
    end
    for (int t = 0; t < NT; t++) begin
      if (snp_grant[t]) begin
        snp_tx[tr_snp_idx_q[t]].flitv    = Y;
        snp_tx[tr_snp_idx_q[t]].flitpend = Y;
        snp_tx[tr_snp_idx_q[t]].flit     = tr_snp_flit[t];
      end
    end
  end

  always_comb begin
    for (int p = 0; p < 2; p++) begin
      rsp_tx[p].flitv    = N;
      rsp_tx[p].flitpend = N;
      rsp_tx[p].flit     = '0;
    end
    for (int t = 0; t < NT; t++) begin
      if (rsp_grant[t]) begin
        rsp_tx[tr_req_idx_q[t]].flitv    = Y;
        rsp_tx[tr_req_idx_q[t]].flitpend = Y;
        rsp_tx[tr_req_idx_q[t]].flit     = tr_rsp_flit[t];
      end
    end
  end

  always_comb begin
    for (int p = 0; p < 2; p++) begin
      dat_tx[p].flitv    = N;
      dat_tx[p].flitpend = N;
      dat_tx[p].flit     = '0;
    end
    for (int t = 0; t < NT; t++) begin
      if (dat_grant[t]) begin
        dat_tx[tr_req_idx_q[t]].flitv    = Y;
        dat_tx[tr_req_idx_q[t]].flitpend = Y;
        dat_tx[tr_req_idx_q[t]].flit     = tr_dat_flit[t];
      end
    end
  end

  always_comb begin
    sn_req_tx.flitv    = N;
    sn_req_tx.flitpend = N;
    sn_req_tx.flit     = '0;
    for (int t = 0; t < NT; t++) begin
      if (snreq_grant[t]) begin
        sn_req_tx.flitv           = Y;
        sn_req_tx.flitpend        = Y;
        sn_req_tx.flit.qos        = tr_qos_q[t];
        sn_req_tx.flit.tgtid      = SNFID;
        sn_req_tx.flit.srcid      = HNFID;
        sn_req_tx.flit.txnid      = tr_txnid_q[t];
        sn_req_tx.flit.size       = tr_size_q[t];
        sn_req_tx.flit.addr       = tr_addr_q[t];
        sn_req_tx.flit.allowretry = Y;
        sn_req_tx.flit.pcrdttype  = '0;
        sn_req_tx.flit.memattr    = '0;
        sn_req_tx.flit.opcode     = tr_snreq_op[t];
      end
    end
  end

  always_comb begin
    sn_dat_tx.flitv    = N;
    sn_dat_tx.flitpend = N;
    sn_dat_tx.flit     = '0;
    for (int t = 0; t < NT; t++) begin
      if (sndat_grant[t]) begin
        sn_dat_tx.flitv       = Y;
        sn_dat_tx.flitpend    = Y;
        sn_dat_tx.flit.qos    = tr_qos_q[t];
        sn_dat_tx.flit.tgtid  = SNFID;
        sn_dat_tx.flit.srcid  = HNFID;
        sn_dat_tx.flit.txnid  = tr_txnid_q[t];
        sn_dat_tx.flit.dbid   = tr_sn_dbid_q[t];
        sn_dat_tx.flit.be     = tr_sndat_be[t];
        sn_dat_tx.flit.data   = tr_data_q[t];
        sn_dat_tx.flit.opcode = tr_sndat_op[t];
      end
    end
  end

  // -------- sequential state update --------
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      req_gnt_idx_q <= 1'b0;
      for (int t = 0; t < NT; t++) begin
        tr_state_q[t]      <= T_IDLE;
        tr_req_idx_q[t]    <= 1'b0;
        tr_snp_idx_q[t]    <= 1'b0;
        tr_is_write_q[t]   <= 1'b0;
        tr_tx_type_q[t]    <= TX_READ;
        tr_addr_q[t]       <= '0;
        tr_srcid_q[t]      <= '0;
        tr_txnid_q[t]      <= '0;
        tr_opcode_q[t]     <= req_opcode_e'(8'h00);
        tr_size_q[t]       <= WIDTH_1;
        tr_qos_q[t]        <= '0;
        tr_data_q[t]       <= '0;
        tr_be_q[t]         <= '0;
        tr_sn_dbid_q[t]    <= '0;
        tr_dirty_hit_q[t]  <= 1'b0;
      end
    end else begin
      req_gnt_idx_q <= req_gnt_idx_d;
      for (int t = 0; t < NT; t++) begin
        tr_state_q[t]      <= tr_state_d[t];
        tr_req_idx_q[t]    <= tr_req_idx_d[t];
        tr_snp_idx_q[t]    <= tr_snp_idx_d[t];
        tr_is_write_q[t]   <= tr_is_write_d[t];
        tr_tx_type_q[t]    <= tr_tx_type_d[t];
        tr_addr_q[t]       <= tr_addr_d[t];
        tr_srcid_q[t]      <= tr_srcid_d[t];
        tr_txnid_q[t]      <= tr_txnid_d[t];
        tr_opcode_q[t]     <= tr_opcode_d[t];
        tr_size_q[t]       <= tr_size_d[t];
        tr_qos_q[t]        <= tr_qos_d[t];
        tr_data_q[t]       <= tr_data_d[t];
        tr_be_q[t]         <= tr_be_d[t];
        tr_sn_dbid_q[t]    <= tr_sn_dbid_d[t];
        tr_dirty_hit_q[t]  <= tr_dirty_hit_d[t];
      end
    end
  end

endmodule