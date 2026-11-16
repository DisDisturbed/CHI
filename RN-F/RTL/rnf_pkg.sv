`timescale 1ns/1ps
// Core-side definitions of the RN-F dcache. This is a first draft of the
// core <-> dcache contract: change the enums / widths here to match the core.
package rnf_pkg;

  // what the core asks for
  typedef enum logic [1:0] {
    MEM_OP_LOAD  = 2'd0,
    MEM_OP_STORE = 2'd1
  } mem_op_e;

  // which unit of the core issued the request; echoed back with the response
  typedef enum logic [1:0] {
    MEM_UNIT_LOAD  = 2'd0,
    MEM_UNIT_STORE = 2'd1,
    MEM_UNIT_PTW   = 2'd2,
    MEM_UNIT_OTHER = 2'd3
  } mem_unit_e;

  // cache line state: same encoding as the CHI line states
  typedef chi_pkg::cache_state_e chr_state_e;

endpackage : rnf_pkg

// -----------------------------------------------------------------------------
// Core <-> dcache interface.
//
// Request: the core holds req (valid = Y) until it samples ready = Y on a
// clock edge; that edge accepts the request. ready = Y while the dcache has
// a free MSHR (NUM_MSHR requests can be open at once). Responses carry the
// request's idx and come back in any order; requests to the same set are
// answered in the order they were accepted. idx must not be reused while its
// request is open.
//   kill       present in the struct, not looked at by the dcache yet
//   addr       must be aligned to width; an access never crosses 16 bytes
//   width      at most MaxAccessWidth/8 bytes
//   data       store data, right-aligned
//   idx, src   echoed back in the response
//   cacheable  Y: through the cache (coherent). N: single ReadNoSnp /
//              WriteNoSnpPtl on the bus, the cache is not looked at
//   chr_state  not looked at by the dcache yet
//
// Response: rsp.valid = Y for one cycle.
//   data       load data, right-aligned and zero-extended
//   err        the CHI transaction came back with an error response
//   chr_state  state of the line after the access (I for non-cacheable)
// -----------------------------------------------------------------------------
interface rnf_core_if #(
  parameter int AddrSize       = 39,
  parameter int MaxAccessWidth = 64,
  parameter int ReqIdxWidth    = 4
);

  typedef struct packed {
    common_pkg::yn_status_e    valid;
    common_pkg::yn_status_e    kill;
    rnf_pkg::mem_op_e          op;
    logic [AddrSize-1:0]       addr;
    common_pkg::width_e        width;
    logic [MaxAccessWidth-1:0] data;
    logic [ReqIdxWidth-1:0]    idx;
    rnf_pkg::mem_unit_e        src;
    common_pkg::yn_status_e    cacheable;
    rnf_pkg::chr_state_e       chr_state;
  } req_t;

  typedef struct packed {
    common_pkg::yn_status_e    valid;
    rnf_pkg::mem_op_e          op;
    logic [MaxAccessWidth-1:0] data;
    logic [ReqIdxWidth-1:0]    idx;
    rnf_pkg::mem_unit_e        src;
    common_pkg::yn_status_e    err;
    rnf_pkg::chr_state_e       chr_state;
  } rsp_t;

  req_t                   req;
  rsp_t                   rsp;
  common_pkg::yn_status_e ready;

  modport core  (output req, input  rsp, input  ready);
  modport cache (input  req, output rsp, output ready);

endinterface
