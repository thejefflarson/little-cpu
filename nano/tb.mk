# nanocpu's cxxrtl harness. NANO_CFLAGS is the one place its -march/-mabi is stated;
# test/march_test.sh's exception list names this line by its exact count.
NANO_CFLAGS := -march=rv32emc -mabi=ilp32e

NANO_RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM \
                            RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32

NANO_SIM_RTL_SRCS := nano/nano.v nano/tb/nano_memory.v soc/compare/dhry_monitor.v
NANO_SIM_TB_SRCS  := nano/tb/nano_testbench.v

nano/tb/nano_rtl.cc: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) $^; hierarchy -top nano_testbench; write_cxxrtl $@'

nano-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" $< -o $@

.PHONY: nano-test
nano-test: nano-sim
	@./nano/asm/run_nano_tests.sh ./nano-sim nano/asm nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-exec-probe
nano-exec-probe:
	@./nano/tb/nano_exec_probe.sh

.PHONY: nano-exec-test
nano-exec-test: nano-exec-probe
	@./nano/tb/nano_exec_run.sh nano/nano.v

NANO_DHRY_RUNS   ?= 200
NANO_DHRY_CYCLES ?= 4000000

.PHONY: nano-dhrystone
nano-dhrystone: nano-sim
	@./nano/bench/run_dhrystone.sh ./nano-sim $(NANO_DHRY_RUNS) $(NANO_DHRY_CYCLES) '$(NANO_CFLAGS)'

NANO_COREMARK_ITERATIONS ?= 5
NANO_COREMARK_CYCLES     ?= 20000000

.PHONY: nano-coremark
nano-coremark: nano-sim
	@./nano/bench/run_coremark.sh ./nano-sim $(NANO_COREMARK_ITERATIONS) $(NANO_COREMARK_CYCLES) '$(NANO_CFLAGS)'
