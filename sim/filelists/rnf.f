// RN-F dcache environment (top: Tb_RNF): the RTL RN-F (driven like a core) and
// one behavioural rnf_model on the HN-F RTL, with the behavioural SN-F.
-f filelists/rtl.f
../RN-F/RTL/rnf_pkg.sv
../RN-F/RTL/rnf_link.sv
../RN-F/RTL/rnf_tag_ram.sv
../RN-F/RTL/rnf_data_ram.sv
../RN-F/RTL/rnf_dcache.sv
../RN-F/RTL/rnf_icache.sv
../RN-F/RTL/rnf_node.sv
../common/TB/chi_tb_pkg.sv
../common/TB/rnf_model.sv
../common/TB/snf_model.sv
../RN-F/TB/rnf_tb_pkg.sv
../RN-F/TB/rnf_core_driver.sv
../RN-F/TB/rnf_fetch_driver.sv
../RN-F/TB/rnf_chi_checker.sv
../RN-F/TB/TB_rnf.sv
// TSHR deadlock monitor, bound into every TSHR instance.
../common/TB/tshr_deadlock_mon.sv
