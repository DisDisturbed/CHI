`timescale 1ns/1ps
// Directed scenarios of the RN-F dcache testbench. The core driver runs them
// (rnf_core_driver::run_scenario); the behavioural RN-F does the ops the
// driver orders through sb.rop_q (CMD_REMOTE).
package rnf_tb_pkg;

  localparam int RS_PINGPONG         = 1;  // one line moves back and forth between the two RN-Fs
  localparam int RS_SNOOP_MATRIX     = 2;  // every snoop type onto a Shared Clean and a Unique Dirty line
  localparam int RS_WB_SNOOP_RACE    = 3;  // replacing a dirty line while the other RN-F asks for the lines of that set
  localparam int RS_EVICT_SNOOP_RACE = 4;  // same with clean lines (Evict)
  localparam int RS_UPGRADE_RACE     = 5;  // store on a Shared line while the other RN-F goes for the same line
  localparam int RS_SET_THRASH       = 6;  // back-to-back walk over more lines than ways of one set
  localparam int RS_SNOOP_IN_WAIT    = 7;  // slow memory: snoops arrive while the dcache waits for its own request
  localparam int RS_NC_MIX           = 8;  // non-cacheable accesses of both RN-Fs on the same lines
  localparam int RS_PRIV_BURST       = 9;  // store / load bursts on the private lines, exact check
  localparam int RS_LAST             = 9;

  localparam int RS_CLEAN_PRIV       = 100; // end of test: CleanInvalid on every private line

  function automatic string rs_name(int id);
    case (id)
      RS_PINGPONG:         return "PINGPONG";
      RS_SNOOP_MATRIX:     return "SNOOP_MATRIX";
      RS_WB_SNOOP_RACE:    return "WB_SNOOP_RACE";
      RS_EVICT_SNOOP_RACE: return "EVICT_SNOOP_RACE";
      RS_UPGRADE_RACE:     return "UPGRADE_RACE";
      RS_SET_THRASH:       return "SET_THRASH";
      RS_SNOOP_IN_WAIT:    return "SNOOP_IN_WAIT";
      RS_NC_MIX:           return "NC_MIX";
      RS_PRIV_BURST:       return "PRIV_BURST";
      RS_CLEAN_PRIV:       return "CLEAN_PRIV";
      default:             return $sformatf("scenario %0d", id);
    endcase
  endfunction

endpackage
