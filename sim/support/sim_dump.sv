// Turns on FST dumping without touching the testbenches: this module is bound
// into whichever top is being simulated (SIM_TOP is set by the Makefile).
//   +wave=<file>   where to write the waveform (default wave.fst)
//   +no_wave       do not dump at all (fastest)
module sim_dump;
  string wave_file;
  initial begin
    if (!$test$plusargs("no_wave")) begin
      if (!$value$plusargs("wave=%s", wave_file)) wave_file = "wave.fst";
      $dumpfile(wave_file);
      $dumpvars;
    end
  end
endmodule

bind `SIM_TOP sim_dump u_sim_dump();
