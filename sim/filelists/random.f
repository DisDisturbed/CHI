// Randomized HN-F environment (top: Tb_HNF_Random).
// HN-F/TB/Randomized/chi_scoreboard.sv is left out on purpose: it is not used, the
// scoreboard in use is the class inside common/TB/chi_tb_pkg.sv.
-f filelists/rtl.f
../common/TB/chi_tb_pkg.sv
../common/TB/rnf_model.sv
../common/TB/snf_model.sv
../HN-F/TB/Randomized/TB_hnf_random.sv
../HN-F/TB/Randomized/tshr_cov.sv
// TSHR deadlock monitor, bound into every TSHR instance.
../common/TB/tshr_deadlock_mon.sv
