`timescale 1ns/1ps
// =============================================================================
// rnf_node - one CHI RN-F node: rnf_dcache (+ rnf_icache when ICACHE = 1).
//
//  ICACHE = 0: the D-cache alone (instruction fetches go through it as data,
//              the requester's choice: icore is not used, ready = N).
//  ICACHE = 1: split caches. rnf_icache serves icore (1-cycle hits) and fills
//              its lines with ReadOnce through rnf_dcache's I-side port, so
//              the node keeps one CHI link (REQ / RSP / DAT, one set of link
//              credits) and one node ID. The I-cache is not coherent: ic_inv_i
//              (FENCE.I, when ic_busy_o is low) invalidates it.
// =============================================================================
module rnf_node
  import chi_pkg::*;
  import common_pkg::*;
  import rnf_pkg::*;
#(
  parameter node_id_e NODEID         = 7'h01,
  parameter node_id_e HNID           = 7'h40,
  parameter int       AddrSize       = 39,
  parameter int       MaxAccessWidth = 64,
  parameter int       ReqIdxWidth    = 4,
  parameter int       SETS           = 16,
  parameter int       WAYS           = 4,
  parameter int       RX_FIFO_DEPTH  = 4,
  parameter int       TX_CREDIT_MAX  = 15,
  parameter int       NUM_MSHR       = 4,
  parameter bit       FAST_LD        = 0,
  parameter bit       ICACHE         = 0,
  parameter int       I_SETS         = 32,
  parameter int       I_WAYS         = 2,
  // system address map: the device range goes to the HN-I (rnf_dcache)
  parameter node_id_e            HNIID    = 7'h50,
  parameter logic [AddrSize-1:0] DEV_BASE = '0,
  parameter logic [AddrSize-1:0] DEV_SIZE = '0
) (
  input wire clk,
  input wire resetn,

  rnf_core_if.cache core,           // data (and fetches when ICACHE = 0)
  rnf_core_if.cache icore,          // fetches (ICACHE = 1), MaxAccessWidth 128
  input  wire       ic_inv_i,
  output logic      ic_busy_o,

  chi_req.tx req_tx,
  chi_snp.rx snp_rx,
  chi_rsp.rx rsp_rx,
  chi_rsp.tx rsp_tx,
  chi_dat.rx dat_rx,
  chi_dat.tx dat_tx
);
  logic                is_req_v, is_req_id, is_req_ack;
  logic [AddrSize-1:0] is_req_addr;
  logic                is_dat_v, is_dat_id, is_dat_err;
  logic [1:0]          is_dat_beat;
  logic [127:0]        is_dat;

  rnf_dcache #(
    .NODEID(NODEID), .HNID(HNID), .AddrSize(AddrSize), .MaxAccessWidth(MaxAccessWidth),
    .ReqIdxWidth(ReqIdxWidth), .SETS(SETS), .WAYS(WAYS), .RX_FIFO_DEPTH(RX_FIFO_DEPTH),
    .TX_CREDIT_MAX(TX_CREDIT_MAX), .NUM_MSHR(NUM_MSHR), .FAST_LD(FAST_LD), .ISIDE(ICACHE),
    .HNIID(HNIID), .DEV_BASE(DEV_BASE), .DEV_SIZE(DEV_SIZE)
  ) u_dc (
    .clk(clk), .resetn(resetn), .core(core),
    .req_tx(req_tx), .snp_rx(snp_rx), .rsp_rx(rsp_rx), .rsp_tx(rsp_tx), .dat_rx(dat_rx), .dat_tx(dat_tx),
    .is_req_v_i(is_req_v), .is_req_id_i(is_req_id), .is_req_addr_i(is_req_addr), .is_req_ack_o(is_req_ack),
    .is_dat_v_o(is_dat_v), .is_dat_id_o(is_dat_id), .is_dat_beat_o(is_dat_beat), .is_dat_o(is_dat),
    .is_dat_err_o(is_dat_err)
  );

  if (ICACHE) begin : gen_ic
    rnf_icache #(
      .AddrSize(AddrSize), .MaxAccessWidth(128), .ReqIdxWidth(ReqIdxWidth),   // icore: 128-bit chunks
      .SETS(I_SETS), .WAYS(I_WAYS)
    ) u_ic (
      .clk(clk), .resetn(resetn), .core(icore), .inv_i(ic_inv_i), .busy_o(ic_busy_o),
      .is_req_v_o(is_req_v), .is_req_id_o(is_req_id), .is_req_addr_o(is_req_addr), .is_req_ack_i(is_req_ack),
      .is_dat_v_i(is_dat_v), .is_dat_id_i(is_dat_id), .is_dat_beat_i(is_dat_beat), .is_dat_i(is_dat),
      .is_dat_err_i(is_dat_err)
    );
  end else begin : gen_no_ic
    assign is_req_v    = 1'b0;
    assign is_req_id   = 1'b0;
    assign is_req_addr = '0;
    assign ic_busy_o   = 1'b0;
    always_comb begin
      icore.ready = N;
      icore.rsp   = '0;
      icore.fast  = '0;
    end
  end

endmodule
