# nanocpu's cxxrtl harness. NANO_CFLAGS is the one place its -march/-mabi is stated;
# test/march_test.sh's exception list names this line by its exact count.
NANO_CFLAGS := -march=rv32ec_zicsr -mabi=ilp32e

NANO_RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM \
                            RISCV_FORMAL_MEM_FAULT RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 \
                            RISCV_FORMAL_ILEN=32

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

.PHONY: nano-meip-floor-probe
nano-meip-floor-probe: nano-sim
	@./nano/tb/nano_meip_floor_probe.sh ./nano-sim '$(NANO_CFLAGS)'

.PHONY: nano-test
nano-test: nano-sim nano/tb/nano_icarus.vvp nano-x-probe nano-meip-floor-probe
	@./nano/tb/nano_dual_leg_test.sh ./nano-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-startup-test
nano-startup-test: nano-sim
	@./nano/bench/run_startup_test.sh ./nano-sim '$(NANO_CFLAGS)'

nano/tb/nano_latch_rtl.cc: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_LATCH_RF $^; hierarchy -top nano_testbench; write_cxxrtl $@'

nano-latch-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_latch_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"nano_latch_rtl.cc"' \
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

nano/tb/nano_oneport_rtl.cc: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_ONE_PORT_RF $^; hierarchy -top nano_testbench; write_cxxrtl $@'

nano-oneport-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_oneport_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"nano_oneport_rtl.cc"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" $< -o $@

nano/tb/nano_icarus_oneport.vvp: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS -DNANO_ONE_PORT_RF $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: nano-oneport-test
nano-oneport-test: nano-oneport-sim nano/tb/nano_icarus_oneport.vvp
	@NANO_VVP_IMAGE="$(CURDIR)/nano/tb/nano_icarus_oneport.vvp" \
	  ./nano/tb/nano_dual_leg_test.sh ./nano-oneport-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-oneport-startup-test
nano-oneport-startup-test: nano-oneport-sim
	@./nano/bench/run_startup_test.sh ./nano-oneport-sim '$(NANO_CFLAGS)'

nano/tb/nano_oneport_latch_rtl.cc: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_ONE_PORT_RF -D NANO_LATCH_RF $^; hierarchy -top nano_testbench; write_cxxrtl $@'

nano-oneport-latch-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_oneport_latch_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"nano_oneport_latch_rtl.cc"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" $< -o $@

nano/tb/nano_icarus_oneport_latch.vvp: rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS -DNANO_ONE_PORT_RF -DNANO_LATCH_RF $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: nano-oneport-latch-test
nano-oneport-latch-test: nano-oneport-latch-sim nano/tb/nano_icarus_oneport_latch.vvp
	@NANO_VVP_IMAGE="$(CURDIR)/nano/tb/nano_icarus_oneport_latch.vvp" \
	  ./nano/tb/nano_dual_leg_test.sh ./nano-oneport-latch-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-oneport-latch-startup-test
nano-oneport-latch-startup-test: nano-oneport-latch-sim
	@./nano/bench/run_startup_test.sh ./nano-oneport-latch-sim '$(NANO_CFLAGS)'

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

NANO_QSPI_SIM_RTL_SRCS := nano/nano.v nano/tb/nano_qspi_memory.v soc/compare/dhry_monitor.v
NANO_QSPI_PREFETCH_DEPTH ?= 0
NANO_QSPI_LOOP_KIND      ?= 0
NANO_QSPI_LOOP_WINDOW    ?= 0
NANO_QSPI_PREAMBLE_CYCLES ?= 24
NANO_QSPI_PSRAM_LOAD_CYCLES ?= 44
NANO_QSPI_PSRAM_STORE_CYCLES ?= 33
# A distinct tag per config, so a sweep of several builds never reads a stale binary.
NANO_QSPI_TAG ?= default
NANO_QSPI_RTL := nano/tb/nano_qspi_rtl.$(NANO_QSPI_TAG).cc
NANO_QSPI_OUT := nano-qspi-sim.$(NANO_QSPI_TAG)

.PHONY: $(NANO_QSPI_RTL)
$(NANO_QSPI_RTL): rvfi_macros.vh $(NANO_QSPI_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_QSPI_TIMING -D NANO_QSPI_PREFETCH_DEPTH=$(NANO_QSPI_PREFETCH_DEPTH) -D NANO_QSPI_LOOP_KIND=$(NANO_QSPI_LOOP_KIND) -D NANO_QSPI_LOOP_WINDOW=$(NANO_QSPI_LOOP_WINDOW) -D NANO_QSPI_PREAMBLE_CYCLES=$(NANO_QSPI_PREAMBLE_CYCLES) -D NANO_QSPI_PSRAM_LOAD_CYCLES=$(NANO_QSPI_PSRAM_LOAD_CYCLES) -D NANO_QSPI_PSRAM_STORE_CYCLES=$(NANO_QSPI_PSRAM_STORE_CYCLES) $^; hierarchy -top nano_testbench; write_cxxrtl $@'

.PHONY: nano-qspi-sim
nano-qspi-sim: nano/tb/nano_cxxrtl.cc $(NANO_QSPI_RTL)
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"$(notdir $(NANO_QSPI_RTL))"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" -I nano/tb $< -o $(NANO_QSPI_OUT)

.PHONY: nano-qspi-timing
nano-qspi-timing: nano-sim
	@./nano/bench/run_qspi_timing.sh '$(NANO_CFLAGS)'

.PHONY: nano-qspi-loop-probe
nano-qspi-loop-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/bench/run_qspi_loop_buffer_probe.sh '$(NANO_CFLAGS)'

.PHONY: nano-qspi-loop-test
nano-qspi-loop-test: nano-qspi-loop-probe
	@./nano/bench/run_qspi_loop_buffer_test.sh '$(NANO_CFLAGS)'

# nano.v -> nano_qspi_ctrl -> a flash model and a PSRAM model, speaking sck/cs_n/sio
# rather than the abstract bus nano_qspi_memory.v times. On `make test`'s path.
NANO_QSPI_PINS_RTL_SRCS := nano/nano.v nano/qspi.v nano/tb/nano_qspi_flash_model.v \
                           nano/tb/nano_qspi_psram_model.v soc/compare/dhry_monitor.v

nano/tb/nano_qspi_pins_rtl.cc: rvfi_macros.vh $(NANO_QSPI_PINS_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_QSPI_PINS $^; hierarchy -top nano_testbench; write_cxxrtl $@'

nano-qspi-pins-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_qspi_pins_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"nano_qspi_pins_rtl.cc"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" -I nano/tb $< -o $@

nano/tb/nano_icarus_qspi_pins.vvp: rvfi_macros.vh $(NANO_QSPI_PINS_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS -DNANO_QSPI_PINS $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: nano-qspi-pins-probe
nano-qspi-pins-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/tb/nano_qspi_pins_probe.sh '$(NANO_CFLAGS)' '$(NANO_QSPI_PINS_RTL_SRCS)' '$(NANO_RISCV_FORMAL_MACROS)'

# Proves the two-leg agreement check below actually catches a structurally blind cxxrtl leg.
.PHONY: nano-qspi-derived-clock-probe
nano-qspi-derived-clock-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/tb/nano_qspi_derived_clock_probe.sh '$(NANO_CFLAGS)' '$(NANO_QSPI_PINS_RTL_SRCS)' '$(NANO_RISCV_FORMAL_MACROS)'

.PHONY: nano-qspi-pins-test
nano-qspi-pins-test: nano-qspi-pins-sim nano/tb/nano_icarus_qspi_pins.vvp nano-qspi-pins-probe \
                      nano-qspi-derived-clock-probe
	@NANO_VVP_IMAGE="$(CURDIR)/nano/tb/nano_icarus_qspi_pins.vvp" \
	  ./nano/tb/nano_dual_leg_test.sh ./nano-qspi-pins-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-qspi-pins-dhrystone
nano-qspi-pins-dhrystone: nano-qspi-pins-sim
	@./nano/bench/run_dhrystone.sh ./nano-qspi-pins-sim $(NANO_DHRY_RUNS) $(NANO_DHRY_CYCLES) '$(NANO_CFLAGS)'

.PHONY: nano-qspi-pins-coremark
nano-qspi-pins-coremark: nano-qspi-pins-sim
	@./nano/bench/run_coremark.sh ./nano-qspi-pins-sim $(NANO_COREMARK_ITERATIONS) $(NANO_COREMARK_CYCLES) '$(NANO_CFLAGS)'

# The minimal reproduction, no nano.v; the models' own grader.
NANO_QSPI_RESUME_SRCS := nano/qspi.v nano/tb/nano_qspi_flash_model.v \
                          nano/tb/nano_qspi_psram_model.v nano/tb/nano_qspi_resume_tb.v

nano/tb/nano_qspi_resume.vvp: $(NANO_QSPI_RESUME_SRCS)
	iverilog -g2012 -o $@ $(NANO_QSPI_RESUME_SRCS)

.PHONY: nano-qspi-resume-probe
nano-qspi-resume-probe:
	@./nano/tb/nano_qspi_resume_probe.sh

.PHONY: nano-qspi-resume-test
nano-qspi-resume-test: nano/tb/nano_qspi_resume.vvp nano-qspi-resume-probe
	@out=$$(vvp nano/tb/nano_qspi_resume.vvp); echo "$$out"; printf '%s\n' "$$out" | grep -q '^PASS$$'

# nano-qspi-resume-test's reproduction, plus one clk of injected round-trip latency. On `make test`'s path.
nano/tb/nano_qspi_latency.vvp: $(NANO_QSPI_RESUME_SRCS)
	iverilog -g2012 -DQSPI_RESUME_TB_DELAY_CYCLES=1 -o $@ $(NANO_QSPI_RESUME_SRCS)

.PHONY: nano-qspi-latency-probe
nano-qspi-latency-probe:
	@./nano/tb/nano_qspi_latency_probe.sh

.PHONY: nano-qspi-latency-test
nano-qspi-latency-test: nano/tb/nano_qspi_latency.vvp nano-qspi-latency-probe
	@out=$$(vvp nano/tb/nano_qspi_latency.vvp); echo "$$out"; printf '%s\n' "$$out" | grep -q '^PASS$$'
