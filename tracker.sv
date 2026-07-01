module chi_hnf_tracker #(
  parameter int AddrWidth = 39,
  parameter int DataWidth = 128,
  parameter chi_pkg::node_id_e HNFID = 7'h40,
  parameter chi_pkg::node_id_e SNFID = 7'h00
) (
  input  logic clk,
  input  logic resetn,

  chi_req.rx req_rx [2],
  chi_snp.tx snp_tx [2],
  chi_rsp.tx rsp_tx [2],
  chi_rsp.rx rsp_rx [2],
  chi_dat.tx dat_tx [2],
  chi_dat.rx dat_rx [2],

  chi_req.tx sn_req_tx,
  chi_rsp.rx sn_rsp_rx,
  chi_dat.tx sn_dat_tx
);

  import chi_pkg::*;
  import common_pkg::*;

  typedef enum logic [3:0] {
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
    ST_DONE
  } state_e;

  state_e state_q, state_d;

  logic                  req_idx_q, req_idx_d;
  logic                  snp_idx_q, snp_idx_d;
  logic                  is_write_q, is_write_d;

  logic [AddrWidth-1:0]  addr_q, addr_d;
  node_id_e              srcid_q, srcid_d;
  logic [11:0]           txnid_q, txnid_d;
  opcode_e                opcode_q, opcode_d;
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

  int i;

  assign req_rx[0].lcrdv = (state_q == ST_IDLE);
  assign req_rx[1].lcrdv = (state_q == ST_IDLE);
  assign rsp_rx[0].lcrdv = 1'b1;
  assign rsp_rx[1].lcrdv = 1'b1;
  assign dat_rx[0].lcrdv = 1'b1;
  assign dat_rx[1].lcrdv = 1'b1;
  assign sn_rsp_rx.lcrdv = 1'b1;

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (i = 0; i < 2; i++) begin
        snp_credit_q[i] <= 4'd0;
        rsp_credit_q[i] <= 4'd0;
        dat_credit_q[i] <= 4'd0;
      end
      sn_req_credit_q <= 4'd0;
      sn_dat_credit_q <= 4'd0;
    end else begin
      for (i = 0; i < 2; i++) begin
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

  always_comb begin
    state_d    = state_q;
    req_idx_d  = req_idx_q;
    snp_idx_d  = snp_idx_q;
    is_write_d = is_write_q;
    addr_d     = addr_q;
    srcid_d    = srcid_q;
    txnid_d    = txnid_q;
    opcode_d   = opcode_q;
    size_d     = size_q;
    qos_d      = qos_q;
    data_d     = data_q;
    be_d       = be_q;
    sn_dbid_d  = sn_dbid_q;

    for (i = 0; i < 2; i++) begin
      snp_tx[i].flitv    = N;
      snp_tx[i].flitpend = N;
      snp_tx[i].flit     = '0;
      rsp_tx[i].flitv    = N;
      rsp_tx[i].flitpend = N;
      rsp_tx[i].flit     = '0;
      dat_tx[i].flitv    = N;
      dat_tx[i].flitpend = N;
      dat_tx[i].flit     = '0;
    end
    sn_req_tx.flitv    = N;
    sn_req_tx.flitpend = N;
    sn_req_tx.flit     = '0;
    sn_dat_tx.flitv    = N;
    sn_dat_tx.flitpend = N;
    sn_dat_tx.flit     = '0;

    case (state_q)
      ST_IDLE: begin
        if (req_rx[0].flitv == Y) begin
          req_idx_d = 1'b0;
          addr_d    = req_rx[0].flit.addr;
          srcid_d   = req_rx[0].flit.srcid;
          txnid_d   = req_rx[0].flit.txnid;
          opcode_d  = req_rx[0].flit.opcode;
          size_d    = req_rx[0].flit.size;
          qos_d     = req_rx[0].flit.qos;
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
      end

      ST_ALLOC: begin
        snp_idx_d  = ~req_idx_q;
        is_write_d = (opcode_q == REQ_WRITEUNIQUEFULL);
        state_d    = ST_SNOOP_SEND;
      end

      ST_SNOOP_SEND: begin
        if (snp_credit_q[snp_idx_q] != 4'd0) begin
          snp_tx[snp_idx_q].flitv         = Y;
          snp_tx[snp_idx_q].flitpend      = Y;
          snp_tx[snp_idx_q].flit.qos      = qos_q;
          snp_tx[snp_idx_q].flit.srcid    = HNFID;
          snp_tx[snp_idx_q].flit.txnid    = txnid_q;
          snp_tx[snp_idx_q].flit.fwdnid   = '0;
          snp_tx[snp_idx_q].flit.fwdtxnid = '0;
          snp_tx[snp_idx_q].flit.opcode   = is_write_q ? SNP_SNPUNIQUE : SNP_SNPSHARED;
          snp_tx[snp_idx_q].flit.addr     = addr_q;
          snp_tx[snp_idx_q].flit.rettosrc = 1'b0;
          state_d = ST_SNOOP_WAIT;
        end
      end

      ST_SNOOP_WAIT: begin
        if (is_write_q) begin
          if ((rsp_rx[snp_idx_q].flitv == Y) &&
              (rsp_rx[snp_idx_q].flit.opcode == RSP_SNPRESP) &&
              (rsp_rx[snp_idx_q].flit.txnid == txnid_q)) begin
            state_d = ST_DBIDRESP_SEND;
          end
        end else begin
          if ((dat_rx[snp_idx_q].flitv == Y) &&
              (dat_rx[snp_idx_q].flit.opcode == DAT_SNPRESPDATA) &&
              (dat_rx[snp_idx_q].flit.txnid == txnid_q)) begin
            data_d  = dat_rx[snp_idx_q].flit.data;
            be_d    = dat_rx[snp_idx_q].flit.be;
            state_d = ST_RDATA_SEND;
          end
        end
      end

      ST_DBIDRESP_SEND: begin
        if (rsp_credit_q[req_idx_q] != 4'd0) begin
          rsp_tx[req_idx_q].flitv       = Y;
          rsp_tx[req_idx_q].flitpend    = Y;
          rsp_tx[req_idx_q].flit.qos    = qos_q;
          rsp_tx[req_idx_q].flit.tgtid  = srcid_q;
          rsp_tx[req_idx_q].flit.srcid  = HNFID;
          rsp_tx[req_idx_q].flit.txnid  = txnid_q;
          rsp_tx[req_idx_q].flit.opcode = RSP_DBIDRESP;
          rsp_tx[req_idx_q].flit.dbid   = txnid_q;
          state_d = ST_WDATA_WAIT;
        end
      end

      ST_WDATA_WAIT: begin
        if ((dat_rx[req_idx_q].flitv == Y) && (dat_rx[req_idx_q].flit.txnid == txnid_q)) begin
          data_d  = dat_rx[req_idx_q].flit.data;
          be_d    = dat_rx[req_idx_q].flit.be;
          state_d = ST_SN_REQ_SEND;
        end
      end

      ST_SN_REQ_SEND: begin
        if (sn_req_credit_q != 4'd0) begin
          sn_req_tx.flitv           = Y;
          sn_req_tx.flitpend        = Y;
          sn_req_tx.flit.qos        = qos_q;
          sn_req_tx.flit.tgtid      = SNFID;
          sn_req_tx.flit.srcid      = HNFID;
          sn_req_tx.flit.txnid      = txnid_q;
          sn_req_tx.flit.opcode     = REQ_WRITENOSNPFULL;
          sn_req_tx.flit.size       = size_q;
          sn_req_tx.flit.addr       = addr_q;
          sn_req_tx.flit.allowretry = common_pkg::Y;
          sn_req_tx.flit.pcrdttype  = '0;
          sn_req_tx.flit.memattr    = '0;
          state_d = ST_SN_RSP_WAIT;
        end
      end

      ST_SN_RSP_WAIT: begin
        if ((sn_rsp_rx.flitv == Y) &&
            (sn_rsp_rx.flit.opcode == RSP_COMPDBIDRESP) &&
            (sn_rsp_rx.flit.txnid == txnid_q)) begin
          sn_dbid_d = sn_rsp_rx.flit.dbid;
          state_d   = ST_SN_DATA_SEND;
        end
      end

      ST_SN_DATA_SEND: begin
        if (sn_dat_credit_q != 4'd0) begin
          sn_dat_tx.flitv       = Y;
          sn_dat_tx.flitpend    = Y;
          sn_dat_tx.flit.qos    = qos_q;
          sn_dat_tx.flit.tgtid  = SNFID;
          sn_dat_tx.flit.srcid  = HNFID;
          sn_dat_tx.flit.txnid  = txnid_q;
          sn_dat_tx.flit.opcode = DAT_NONCOPYBACKWRDATA;
          sn_dat_tx.flit.dbid   = sn_dbid_q;
          sn_dat_tx.flit.be     = be_q;
          sn_dat_tx.flit.data   = data_q;
          state_d = ST_COMP_SEND;
        end
      end

      ST_RDATA_SEND: begin
        if (dat_credit_q[req_idx_q] != 4'd0) begin
          dat_tx[req_idx_q].flitv       = Y;
          dat_tx[req_idx_q].flitpend    = Y;
          dat_tx[req_idx_q].flit.qos    = qos_q;
          dat_tx[req_idx_q].flit.tgtid  = srcid_q;
          dat_tx[req_idx_q].flit.srcid  = HNFID;
          dat_tx[req_idx_q].flit.txnid  = txnid_q;
          dat_tx[req_idx_q].flit.opcode = DAT_COMPDATA;
          dat_tx[req_idx_q].flit.be     = be_q;
          dat_tx[req_idx_q].flit.data   = data_q;
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMP_SEND: begin
        if (rsp_credit_q[req_idx_q] != 4'd0) begin
          rsp_tx[req_idx_q].flitv       = Y;
          rsp_tx[req_idx_q].flitpend    = Y;
          rsp_tx[req_idx_q].flit.qos    = qos_q;
          rsp_tx[req_idx_q].flit.tgtid  = srcid_q;
          rsp_tx[req_idx_q].flit.srcid  = HNFID;
          rsp_tx[req_idx_q].flit.txnid  = txnid_q;
          rsp_tx[req_idx_q].flit.opcode = RSP_COMP;
          state_d = ST_COMPACK_WAIT;
        end
      end

      ST_COMPACK_WAIT: begin
        if ((rsp_rx[req_idx_q].flitv == Y) &&
            (rsp_rx[req_idx_q].flit.opcode == RSP_COMPACK) &&
            (rsp_rx[req_idx_q].flit.txnid == txnid_q)) begin
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
  end

  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      state_q    <= ST_IDLE;
      req_idx_q  <= 1'b0;
      snp_idx_q  <= 1'b0;
      is_write_q <= 1'b0;
      addr_q     <= '0;
      srcid_q    <= '0;
      txnid_q    <= '0;
      opcode_q   <= '0;
      size_q     <= WIDTH_1;
      qos_q      <= '0;
      data_q     <= '0;
      be_q       <= '0;
      sn_dbid_q  <= '0;
    end else begin
      state_q    <= state_d;
      req_idx_q  <= req_idx_d;
      snp_idx_q  <= snp_idx_d;
      is_write_q <= is_write_d;
      addr_q     <= addr_d;
      srcid_q    <= srcid_d;
      txnid_q    <= txnid_d;
      opcode_q   <= opcode_d;
      size_q     <= size_d;
      qos_q      <= qos_d;
      data_q     <= data_d;
      be_q       <= be_d;
      sn_dbid_q  <= sn_dbid_d;
    end
  end

endmodule
