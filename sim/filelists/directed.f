// Directed HN-F scenarios (top: Tb_HNF). TB_TSHR.sv has its own rnf_model and
// snf_model, so it cannot be compiled together with common/TB.
-f filelists/rtl.f
../HN-F/TB/Directed/TB_TSHR.sv
// TSHR deadlock monitor, bound into every TSHR instance.
../common/TB/tshr_deadlock_mon.sv
