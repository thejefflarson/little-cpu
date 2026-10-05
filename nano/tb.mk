# nanocpu's cxxrtl harness. NANO_CFLAGS is the one place its -march/-mabi is stated; test/march_test.sh's exception list names this line by its exact count.
NANO_CFLAGS := -march=rv32ec_zicsr -mabi=ilp32e

NANO_RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM \
                            RISCV_FORMAL_MEM_FAULT RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 \
                            RISCV_FORMAL_ILEN=32

include nano/stamp.mk

NANO_SIM_RTL_SRCS := nano/nano.v nano/tb/nano_memory.v soc/compare/dhry_monitor.v
NANO_SIM_TB_SRCS  := nano/tb/nano_testbench.v
NANO_SIM_IN       := rvfi_macros.vh $(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
$(eval $(call nano_stamp,NANO_SIM_STAMP,$(NANO_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) nano/tb/nano_cxxrtl.cc,$(NANO_RISCV_FORMAL_MACROS)))

nano/tb/nano_rtl.cc: $(NANO_SIM_IN) $(NANO_SIM_STAMP)
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) $(NANO_SIM_IN); hierarchy -top nano_testbench; write_cxxrtl $@'

nano-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_rtl.cc $(NANO_SIM_STAMP)
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" $< -o $@

nano/tb/nano_icarus.vvp: $(NANO_SIM_IN) $(NANO_SIM_STAMP)
	iverilog -I./rtl/ -DICARUS $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $(NANO_SIM_IN)

.PHONY: nano-x-probe
nano-x-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/tb/nano_x_probe.sh '$(NANO_CFLAGS)' '$(NANO_SIM_RTL_SRCS)' '$(NANO_RISCV_FORMAL_MACROS)'

.PHONY: nano-vcd-probe
nano-vcd-probe: nano/tb/nano_icarus.vvp
	@./nano/tb/nano_vcd_probe.sh ./nano/tb/nano_sim_icarus.sh

.PHONY: nano-rf-model-test
nano-rf-model-test:
	@./nano/tb/nano_rf_model_probe.sh

.PHONY: nano-rf-timing-probe
nano-rf-timing-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/tb/nano_rf_timing_probe.sh '$(NANO_CFLAGS)' '$(NANO_SIM_RTL_SRCS)' '$(NANO_RISCV_FORMAL_MACROS)'

.PHONY: nano-meip-floor-probe
nano-meip-floor-probe: nano-sim
	@./nano/tb/nano_meip_floor_probe.sh ./nano-sim '$(NANO_CFLAGS)'

.PHONY: nano-test
nano-test: nano-sim nano/tb/nano_icarus.vvp nano-x-probe nano-meip-floor-probe \
          nano-vcd-probe nano-rf-timing-probe nano-rf-model-test
	@./nano/tb/nano_dual_leg_test.sh ./nano-sim ./nano/tb/nano_sim_icarus.sh nano/asm \
	  nano/asm/EXPECTED_FAIL nano/asm/OBSERVED_FLOOR '$(NANO_CFLAGS)'

.PHONY: nano-startup-test
nano-startup-test: nano-sim
	@./nano/bench/run_startup_test.sh ./nano-sim '$(NANO_CFLAGS)'

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
NANO_COREMARK_CYCLES     ?= 30000000

NANO_QSPI_PINS_DHRY_CYCLES         ?= 8000000
NANO_QSPI_PINS_COREMARK_ITERATIONS ?= 1
NANO_QSPI_PINS_COREMARK_CYCLES     ?= 30000000
NANO_QSPI_PINS_MEMORY := nano/qspi.v against the pin-level flash and PSRAM models (nano/tb/nano_qspi_*_model.v)

.PHONY: nano-coremark
nano-coremark: nano-sim
	@./nano/bench/run_coremark.sh ./nano-sim $(NANO_COREMARK_ITERATIONS) $(NANO_COREMARK_CYCLES) '$(NANO_CFLAGS)'

NANO_QSPI_SIM_RTL_SRCS := nano/nano.v nano/tb/nano_qspi_memory.v soc/compare/dhry_monitor.v
NANO_QSPI_PREFETCH_DEPTH ?= 0
NANO_QSPI_LOOP_KIND      ?= 0
NANO_QSPI_LOOP_WINDOW    ?= 0
NANO_QSPI_PREAMBLE_CYCLES ?= 24
NANO_QSPI_PARCEL_CYCLES ?= 8
NANO_QSPI_PSRAM_LOAD_CYCLES ?= 44
NANO_QSPI_PSRAM_STORE_CYCLES ?= 33
# A distinct tag per config, so a sweep of several builds never reads a stale binary.
NANO_QSPI_TAG ?= default
NANO_QSPI_RTL := nano/tb/nano_qspi_rtl.$(NANO_QSPI_TAG).cc
NANO_QSPI_OUT := nano-qspi-sim.$(NANO_QSPI_TAG)

.PHONY: $(NANO_QSPI_RTL)
$(NANO_QSPI_RTL): rvfi_macros.vh $(NANO_QSPI_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_QSPI_TIMING -D NANO_QSPI_PREFETCH_DEPTH=$(NANO_QSPI_PREFETCH_DEPTH) -D NANO_QSPI_LOOP_KIND=$(NANO_QSPI_LOOP_KIND) -D NANO_QSPI_LOOP_WINDOW=$(NANO_QSPI_LOOP_WINDOW) -D NANO_QSPI_PREAMBLE_CYCLES=$(NANO_QSPI_PREAMBLE_CYCLES) -D NANO_QSPI_PARCEL_CYCLES=$(NANO_QSPI_PARCEL_CYCLES) -D NANO_QSPI_PSRAM_LOAD_CYCLES=$(NANO_QSPI_PSRAM_LOAD_CYCLES) -D NANO_QSPI_PSRAM_STORE_CYCLES=$(NANO_QSPI_PSRAM_STORE_CYCLES) $^; hierarchy -top nano_testbench; write_cxxrtl "$@"'

.PHONY: nano-qspi-sim
nano-qspi-sim: nano/tb/nano_cxxrtl.cc $(NANO_QSPI_RTL)
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"$(notdir $(NANO_QSPI_RTL))"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" -I nano/tb $< -o $(NANO_QSPI_OUT)

.PHONY: nano-qspi-timing
nano-qspi-timing: nano-sim
	@./nano/bench/run_qspi_timing.sh '$(NANO_CFLAGS)'

.PHONY: nano-qspi-control-test
nano-qspi-control-test: nano-sim
	@./nano/bench/run_qspi_timing.sh '$(NANO_CFLAGS)' --control-only

.PHONY: nano-qspi-loop-probe
nano-qspi-loop-probe: rvfi_macros.vh test/monitor.sim.v
	@./nano/bench/run_qspi_loop_buffer_probe.sh '$(NANO_CFLAGS)'

.PHONY: nano-qspi-loop-test
nano-qspi-loop-test: nano-qspi-loop-probe
	@./nano/bench/run_qspi_loop_buffer_test.sh '$(NANO_CFLAGS)'

# nano.v -> nano_qspi_ctrl -> a flash model and a PSRAM model, speaking sck/cs_n/sio rather than the abstract bus nano_qspi_memory.v times. On `make test`'s path.
NANO_QSPI_PINS_RTL_SRCS := nano/nano.v nano/qspi.v nano/tb/nano_qspi_flash_model.v \
                           nano/tb/nano_qspi_psram_model.v soc/compare/dhry_monitor.v

NANO_QSPI_PINS_IN    := rvfi_macros.vh $(NANO_QSPI_PINS_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v
$(eval $(call nano_stamp,NANO_QSPI_PINS_STAMP,$(NANO_QSPI_PINS_RTL_SRCS) $(NANO_SIM_TB_SRCS) nano/tb/nano_cxxrtl.cc,$(NANO_RISCV_FORMAL_MACROS) NANO_QSPI_PINS))

nano/tb/nano_qspi_pins_rtl.cc: $(NANO_QSPI_PINS_IN) $(NANO_QSPI_PINS_STAMP)
	yosys -p 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) -D NANO_QSPI_PINS $(NANO_QSPI_PINS_IN); hierarchy -top nano_testbench; write_cxxrtl $@'

nano-qspi-pins-sim: nano/tb/nano_cxxrtl.cc nano/tb/nano_qspi_pins_rtl.cc $(NANO_QSPI_PINS_STAMP)
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror -DNANO_RTL_INCLUDE='"nano_qspi_pins_rtl.cc"' \
	  -isystem "$$(yosys-config --datdir)/include/backends/cxxrtl/runtime" -I nano/tb $< -o $@

nano/tb/nano_icarus_qspi_pins.vvp: $(NANO_QSPI_PINS_IN) $(NANO_QSPI_PINS_STAMP)
	iverilog -I./rtl/ -DICARUS -DNANO_QSPI_PINS $(addprefix -D,$(NANO_RISCV_FORMAL_MACROS)) -g2012 -o $@ $(NANO_QSPI_PINS_IN)

.PHONY: nano-stale-build-test
nano-stale-build-test:
	@./nano/tb/nano_stale_build_probe.sh

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
	@NANO_BENCH_MEMORY='$(NANO_QSPI_PINS_MEMORY)' \
	  ./nano/bench/run_dhrystone.sh ./nano-qspi-pins-sim $(NANO_DHRY_RUNS) $(NANO_QSPI_PINS_DHRY_CYCLES) '$(NANO_CFLAGS)'

.PHONY: nano-qspi-pins-coremark
nano-qspi-pins-coremark: nano-qspi-pins-sim
	@NANO_BENCH_MEMORY='$(NANO_QSPI_PINS_MEMORY)' \
	  ./nano/bench/run_coremark.sh ./nano-qspi-pins-sim $(NANO_QSPI_PINS_COREMARK_ITERATIONS) $(NANO_QSPI_PINS_COREMARK_CYCLES) '$(NANO_CFLAGS)'

# The minimal reproduction, no nano.v; the models' own grader.
NANO_QSPI_RESUME_SRCS := nano/qspi.v nano/tb/nano_qspi_flash_model.v \
                          nano/tb/nano_qspi_psram_model.v nano/tb/nano_qspi_resume_tb.v

$(eval $(call nano_stamp,NANO_QSPI_RESUME_STAMP,$(NANO_QSPI_RESUME_SRCS),))

nano/tb/nano_qspi_resume.vvp: $(NANO_QSPI_RESUME_SRCS) $(NANO_QSPI_RESUME_STAMP)
	iverilog -g2012 -o $@ $(NANO_QSPI_RESUME_SRCS)

.PHONY: nano-qspi-resume-probe
nano-qspi-resume-probe:
	@./nano/tb/nano_qspi_resume_probe.sh

.PHONY: nano-qspi-resume-test
nano-qspi-resume-test: nano/tb/nano_qspi_resume.vvp nano-qspi-resume-probe
	@out=$$(vvp nano/tb/nano_qspi_resume.vvp); echo "$$out"; printf '%s\n' "$$out" | grep -q '^PASS$$'

.PHONY: nano-uio-oe-probe
nano-uio-oe-probe:
	@./nano/tb/nano_uio_oe_probe.sh '$(NANO_CFLAGS)'

.PHONY: nano-tt-test
nano-tt-test: nano-uio-oe-probe
	@./nano/tb/run_nano_tt_test.sh '$(NANO_CFLAGS)'

# Runs a hardened netlist through the pins-only test. Off `make test`'s path, like nano-area.
.PHONY: nano-gl-gate-probe
nano-gl-gate-probe: nano-sky130-verilog-setup
	@./nano/tb/nano_gl_gate_probe.sh '$(NANO_SKY130_VERILOG_DIR)'

.PHONY: nano-gl-census-probe
nano-gl-census-probe:
	@./nano/gl_census_probe.sh

.PHONY: nano-gl-test
nano-gl-test: nano-sky130-verilog-setup nano-gl-gate-probe nano-gl-census-probe
	@if [ -z "$(NETLIST)" ]; then \
	  echo "usage: make nano-gl-test NETLIST=<path to a hardened .nl.v netlist>" >&2; \
	  exit 2; \
	fi
	@./nano/tb/run_nano_gl_test.sh '$(NANO_CFLAGS)' '$(NETLIST)' '$(NANO_SKY130_VERILOG_DIR)'

# nano-qspi-resume-test's reproduction, plus one clk of injected round-trip latency. On `make test`'s path.
$(eval $(call nano_stamp,NANO_QSPI_LATENCY_STAMP,$(NANO_QSPI_RESUME_SRCS),QSPI_RESUME_TB_DELAY_CYCLES=1))

nano/tb/nano_qspi_latency.vvp: $(NANO_QSPI_RESUME_SRCS) $(NANO_QSPI_LATENCY_STAMP)
	iverilog -g2012 -DQSPI_RESUME_TB_DELAY_CYCLES=1 -o $@ $(NANO_QSPI_RESUME_SRCS)

.PHONY: nano-qspi-latency-probe
nano-qspi-latency-probe:
	@./nano/tb/nano_qspi_latency_probe.sh

.PHONY: nano-qspi-latency-test
nano-qspi-latency-test: nano/tb/nano_qspi_latency.vvp nano-qspi-latency-probe
	@out=$$(vvp nano/tb/nano_qspi_latency.vvp); echo "$$out"; printf '%s\n' "$$out" | grep -q '^PASS$$'

.PHONY: nano-qspi-window-test
nano-qspi-window-test:
	@./nano/tb/nano_qspi_window_probe.sh

# The three yosys elaborations behind nano-sim, nano-qspi-sim and nano-qspi-pins-sim, through `check`. `make elaborate-strict` runs it, and nano_elaborate_strict.sh fails on any warning but yosys's deep-recursion notice.
NANO_ELAB_YOSYS = ./nano/tb/nano_elaborate_strict.sh $(BUILD)/$(3).log 'read_verilog -sv $(addprefix -D ,$(NANO_RISCV_FORMAL_MACROS)) $(1) $(2); hierarchy -top nano_testbench; proc; opt_clean; check; write_cxxrtl $(BUILD)/$(3).cc'

.PHONY: nano-elaborate-strict
nano-elaborate-strict: $(NANO_SIM_IN) $(NANO_QSPI_SIM_RTL_SRCS) $(NANO_QSPI_PINS_IN) | $(BUILD)
	$(call NANO_ELAB_YOSYS,,$(NANO_SIM_IN),nano-elaborate-strict)
	$(call NANO_ELAB_YOSYS,-D NANO_QSPI_TIMING,rvfi_macros.vh $(NANO_QSPI_SIM_RTL_SRCS) $(NANO_SIM_TB_SRCS) test/monitor.sim.v,nano-elaborate-strict-qspi)
	$(call NANO_ELAB_YOSYS,-D NANO_QSPI_PINS,$(NANO_QSPI_PINS_IN),nano-elaborate-strict-pins)
