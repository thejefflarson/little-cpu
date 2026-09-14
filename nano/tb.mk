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

nano/tb/nano_icarus.vvp: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: nano-x-probe
nano-x-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/tb/nano_x_probe.sh '$(NANO_CFLAGS)' '$(NANO_SIM_RTL_SRCS)' '$(NANO_RISCV_FORMAL_MACROS)'

.PHONY: nano-test
nano-test: nano-sim nano/tb/nano_icarus.vvp nano-x-probe
	@./nano/tb/nano_dual_leg_test.sh ./nano-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-startup-test
nano-startup-test: nano-sim
	@./nano/bench/run_startup_test.sh ./nano-sim '$(NANO_CFLAGS)'

nano/tb/nano_latch_rtl.cc: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_LATCH_RF $^; hierarchy -top nano_testbench; write_cxxrtl $@'

nano-latch-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_latch_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_HEADER='"nano_latch_rtl.cc"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" $< -o $@

nano/tb/nano_icarus_latch.vvp: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS -DNANO_LATCH_RF $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: nano-latch-test
nano-latch-test: nano-latch-sim nano/tb/nano_icarus_latch.vvp
	@NANO_VVP_IMAGE="$(CURDIR)/nano/tb/nano_icarus_latch.vvp" \
	  ./nano/tb/nano_dual_leg_test.sh ./nano-latch-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-latch-startup-test
nano-latch-startup-test: nano-latch-sim
	@./nano/bench/run_startup_test.sh ./nano-latch-sim '$(NANO_CFLAGS)'

.PHONY: nano-exec-probe
nano-exec-probe:
	@./nano/tb/nano_exec_probe.sh

.PHONY: nano-exec-test
nano-exec-test: nano-exec-probe
	@./nano/tb/nano_exec_run.sh nano/nano.v

.PHONY: nano-littlecpu-test
nano-littlecpu-test: nano-sim
	@./nano/asm/run_nano_tests.sh ./nano-sim test/asm nano/asm/LITTLECPU_EXPECTED_FAIL \
	  nano/asm/LITTLECPU_FLOOR '$(NANO_CFLAGS)' nano/asm/nano.lds 10000

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
