// Directed HN-F scenarios (top: Tb_HNF). TB_TSHR.sv has its own rnf_model and
// snf_model, so it cannot be compiled together with TB/Randomized.
-f filelists/rtl.f
../TB/Directed/TB_TSHR.sv
