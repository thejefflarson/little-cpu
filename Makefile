include formal/pin.mk

# A recipe that dies partway through must not leave a truncated target behind
# looking newer than its prerequisites — several targets here are shell
# redirections, which create the output file before the generator has written
# a byte of it.
.DELETE_ON_ERROR:

# Macro names that turn RVFI on (ADR-0006), shared by both sim legs below so
# they can't drift apart on which -D flags mean "RVFI is live" -- the whole
# point is that both legs check the same thing. One list of names;
# each consumer applies its own flag syntax (iverilog: -DNAME, yosys
# `read_verilog`: -D NAME) via $(addprefix ...) since no single literal string
# satisfies both.
RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32

rvfi_macros.vh: $(RISCV_FORMAL_DIR)/checks/rvfi_macros.py
	python3 $^ > $@

testbench.vvp: rtl/structs.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS $(addprefix -D,$(RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

# ADR-0007: iverilog is the waveform leg, not cxxrtl (`./sim` now requires
# --rom/--ram/--cycles and was never a VCD demo — see the `sim:` rule
# above). testbench.vvp already `$dumpfile`s/`$dumpvars`s under `ifdef
# ICARUS` (test/testbench.v) and already carries test/monitor.sim.v, so this
# waveform is also a self-checking per-retire run, not just a raw trace. It
# runs the fixed increment-loop program baked into test/testbench.v's
# `initial rom[...]` block for 200 cycles -- there's no --rom flag on this
# leg, unlike ./sim's.
.PHONY: waves
waves: waves.vcd
waves.vcd: testbench.vvp
	vvp $<
	mv testbench.vcd $@

# -O2 -DNDEBUG: the sim binary is the primary runner (ADR-0007) and needs to
# run 46 tests well under a minute; -g -O0 (the old flags) is 5-10x slower
# for no benefit here. -isystem (not -I) for the vendored cxxrtl runtime
# headers so -Wall -Wextra -Werror only fails on warnings in code this repo
# owns, not on the third-party runtime's.
sim: test/cxxrtl.cc test/rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

# ---------------------------------------------------------------------------
# Sail co-simulation (docs/adr/0031) -- a SPIKE, deliberately opt-in.
#
# Nothing below is reachable from `make test`, `make test-units` or CI. The
# whole point of keeping it off the default path is that `make test` must
# still work on a machine with no Sail installed, and a time-boxed experiment
# must not quietly become a merge gate. Run it by hand:
#
#     make sail-setup     # once: fetch the pinned sail-riscv release
#     make cosim          # build the architectural-state tracer
#     make cosim-run      # run it on one program (PROG=add.S by default)
# ---------------------------------------------------------------------------

# The sail-riscv release this spike was run against. Pinned for the same
# reason formal/pin.mk pins riscv-formal: an oracle that moves under you is
# not an oracle. Unlike that pin this one is NOT an enforced supply-chain
# control -- no code is executed out of the tarball at build time, and
# `make test` does not depend on it -- so it is a plain variable, overridable
# for a bisect.
SAIL_RISCV_VERSION ?= 0.13.1
SAIL_RISCV_DIR     := tools/sail

# Prebuilt, first-party release binaries from github.com/riscv/sail-riscv.
# Building the model from source needs opam + OCaml + the Sail compiler
# (there is no `brew install sail-riscv`; homebrew's `sail` formula is an
# unrelated WordPress deploy tool). The upstream release ships a macOS arm64
# and two Linux tarballs, which covers every machine this repo is developed
# and CI'd on, so the source build is not worth its cost here.
.PHONY: sail-setup
sail-setup: $(SAIL_RISCV_DIR)/bin/sail_riscv_sim
$(SAIL_RISCV_DIR)/bin/sail_riscv_sim:
	@case "$$(uname -s)/$$(uname -m)" in \
	  Darwin/arm64)  asset=sail-riscv-Mac-arm64 ;; \
	  Linux/x86_64)  asset=sail-riscv-Linux-x86_64 ;; \
	  Linux/aarch64) asset=sail-riscv-Linux-aarch64 ;; \
	  *) echo "no prebuilt sail-riscv for $$(uname -s)/$$(uname -m);" >&2; \
	     echo "build it from https://github.com/riscv/sail-riscv and set" >&2; \
	     echo "SAIL_RISCV_SIM to the resulting sail_riscv_sim." >&2; exit 1 ;; \
	esac; \
	url=https://github.com/riscv/sail-riscv/releases/download/$(SAIL_RISCV_VERSION)/$$asset.tar.gz; \
	echo "fetching $$url"; \
	rm -rf $(SAIL_RISCV_DIR).tmp && mkdir -p $(SAIL_RISCV_DIR).tmp && \
	curl -fsSL "$$url" | tar xz -C $(SAIL_RISCV_DIR).tmp --strip-components=1 && \
	test -x $(SAIL_RISCV_DIR).tmp/bin/sail_riscv_sim && \
	rm -rf $(SAIL_RISCV_DIR) && mv $(SAIL_RISCV_DIR).tmp $(SAIL_RISCV_DIR)
	@$@ --version

# The co-simulation runner: a SECOND cxxrtl binary, not a change to `sim`.
# It reads the real `uut regfile regs` array through debug_items and reads no
# rvfi_* signal at all; see the header comment in test/cosim.cc for why that
# distinction is the entire value of this leg. Shares test/rtl.cc with `sim`,
# so it costs one extra compile and no extra elaboration.
cosim: test/cosim.cc test/rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

PROG ?= add.S
.PHONY: cosim-run
cosim-run: cosim
	./test/cosim.py $(PROG)

# test/monitor.sim.v is a build-time-only, gitignored derivative of the
# tracked test/monitor.v (ADR-0019). test/monitor.v itself stays
# pristine and tracked (CLAUDE.md invariant 7); regenerate this file, never
# hand-edit it, and never commit it. BOTH sim legs read this file, so they
# cannot drift into checking different specs -- and it is therefore
# load-bearing for correctness, not just elaboration: changing a rule below
# changes the oracle. Two rules, each anchored tightly enough that it cannot
# match anywhere it wasn't meant to. `diff test/monitor.v test/monitor.sim.v`
# is six lines; if it stops being readable at a glance, fix the generator
# upstream instead.
#
#   1. yosys's AST_AUTOWIRE elaboration bug trips on `$time` used as a bare
#      $display argument (four call sites -- iverilog handles it fine, yosys
#      does not). Strips the timestamp from the error banner; the iverilog
#      leg loses a diagnostic nicety, not a check.
#   2. ADR-0019: monitor_insn_div/monitor_insn_rem compute signed division as
#      a conditional whose other branches are unsigned, so IEEE 1800
#      sign-context propagation silently evaluates the division UNSIGNED --
#      wrong answers for negative operands only, which is exactly what
#      div.S/rem.S exercise. Wrapping the arithmetic in $signed() makes it
#      self-determined, so the enclosing conditional can no longer downgrade
#      it. Two lines change (DIV `/` and REM `%`).
test/monitor.sim.v: test/monitor.v
	sed -E -e 's/ at time %0t --------", ([A-Za-z0-9_]+), \$$time\)/ --------", \1)/' \
	       -e 's/\$$signed\((rvfi_rs1_rdata)\) ([\/%]) \$$signed\((rvfi_rs2_rdata)\);/$$signed($$signed(\1) \2 $$signed(\3));/' \
	       $< > $@

# $(RISCV_FORMAL_MACROS) turns on the rvfi_* ports and their driving logic
# throughout rtl/ and test/testbench.v's `monitor` instance (ADR-0006);
# only RISCV_FORMAL itself is actually read by rtl/ or test/testbench.v
# today, the rest ride along for consistency with testbench.vvp above.
# test/monitor.sim.v (not test/monitor.v) supplies the `monitor` module
# here; see its rule above for why.
test/rtl.cc: rtl/structs.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top testbench; write_cxxrtl $@'

# test/monitor.v is generated by riscv-formal's monitor/generate.py at
# RISCV_FORMAL_SHA (formal/pin.mk — bump the pin there, not here). generate.py
# opens ../insns/isa_<isa>.txt relative to its own CWD, so it must be run from
# riscv-formal/monitor/, not the repo root (see ADR-0007). The tracked file is
# regenerated in place; use `make monitor-check` to check freshness without
# touching it.
MONITOR_GEN = cd $(RISCV_FORMAL_DIR)/monitor && python3 generate.py -i rv32imc -c 1 -a -p monitor

# Depends on the generator and the pin, NOT on the clone directory: a directory's
# mtime bumps on any write anywhere inside it, which would make routine builds
# overwrite this tracked file (CLAUDE.md invariant 7) at unpredictable moments and
# defeat the point of monitor-check. The clone is order-only.
test/monitor.v: $(RISCV_FORMAL_DIR)/monitor/generate.py formal/pin.mk | $(RISCV_FORMAL_DIR)
	$(MONITOR_GEN) > $(CURDIR)/$@

.PHONY: monitor-check
monitor-check: $(RISCV_FORMAL_DIR)/monitor/generate.py | $(RISCV_FORMAL_DIR)
	@tmp=$$(mktemp "$${TMPDIR:-/tmp}/monitor-check.XXXXXX"); \
	trap 'rm -f "$$tmp"' EXIT; \
	($(MONITOR_GEN)) > "$$tmp" && \
	diff -u test/monitor.v "$$tmp"

# Installs the RISC-V cross compiler the test suite is assembled with
# (ADR-0007/ADR-0008). Both packages are freestanding-only (no multilib/
# newlib needed, since the tests are `-nostdlib`), and both are verified
# real, bottled/packaged builds — not a hallucinated package name.
.PHONY: setup
setup:
ifeq ($(shell uname -s),Darwin)
	brew install riscv64-elf-gcc
else
	@echo "On Linux, install the RISC-V cross compiler with:"
	@echo "  sudo apt-get install gcc-riscv64-unknown-elf"
endif

# Four small benches that landed in `eb18320` and `a4662a2` with no runner
# (rtl/executor.v, rtl/memory.v, rtl/decoder.v, rtl/regfile.v respectively —
# regfile_tb.v covers the write-through bypass and x0 semantics, the single
# most load-bearing change in the project, and was verified by hand via
# iverilog+vvp before this rule existed to run it in CI). Compiled and run
# straight through iverilog/vvp; each bench $fatal(1)s on a mismatch and
# $finish (exit 0) on success, so vvp's own exit code is the pass/fail
# signal — no output-parsing needed. A separate target from `test` (that one
# is the ADR-0007 cxxrtl regression gate over test/asm/*.S; these are
# RTL-level unit benches with their own, unrelated pass/fail mechanism) but
# `test` depends on it so one `make test` still catches everything.
.PHONY: test-units
test-units:
	@tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/test-units.XXXXXX"); \
	trap 'rm -rf "$$tmp"' EXIT; \
	set -e; \
	echo "== exec_tb =="; \
	iverilog -I./rtl/ -g2012 -o $$tmp/exec_tb.vvp rtl/structs.v rtl/executor.v test/exec_tb.v; \
	vvp $$tmp/exec_tb.vvp; \
	echo "== mem_tb =="; \
	iverilog -I./rtl/ -g2012 -o $$tmp/mem_tb.vvp rtl/memory.v test/mem_tb.v; \
	vvp $$tmp/mem_tb.vvp; \
	echo "== decoder_tb =="; \
	iverilog -I./rtl/ -g2012 -o $$tmp/decoder_tb.vvp rtl/structs.v rtl/decoder.v test/decoder_tb.v; \
	vvp $$tmp/decoder_tb.vvp; \
	echo "== regfile_tb =="; \
	iverilog -I./rtl/ -g2012 -o $$tmp/regfile_tb.vvp rtl/regfile.v test/regfile_tb.v; \
	vvp $$tmp/regfile_tb.vvp

# Assembles every test/asm/*.S (rv32im_zicsr, -nostdlib, ADR-0008's memory
# map), runs each under `sim`, and checks the pass/fail table against
# test/EXPECTED_FAIL — the sprint-1 baseline. Exits 0 only when the actual
# failure list matches that file exactly, so this is a working regression
# gate today, against the current (still-broken, see CLAUDE.md) core.
.PHONY: test
test: sim test-units
	@./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL

pll.v: timing
	icepll -m -f $@ -i 12 -o $(shell cat $^)

# rtl/imemory.v unconditionally $readmemh's this at elaboration time; it's
# gitignored and nothing generates it yet (real ROM contents are M1 work, once
# there's a test-binary-to-hex pipeline). An empty file is enough for
# synth_ice40 to elaborate — $readmemh on an empty file just leaves the ROM
# unmodified. Order-only prerequisite so it's not fed to read_verilog.
#
# This deliberately turns a hard yosys error into a successful build, so it has
# to say out loud that the resulting bitstream has an empty ROM and traps on its
# first instruction. Never silently.
rtl/rom.mem:
	touch $@
	@echo '*** WARNING: created an EMPTY $@ placeholder. Anything synthesised'
	@echo '*** from it has an uninitialised ROM and will trap on instruction 0.'
	@echo '*** Real ROM contents are M1 work (test-binary-to-hex).'

riscv.json: rtl/structs.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v rtl/imemory.v rtl/memory.v | rtl/rom.mem
	yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlesoc -json $@'

riscv.asc: riscv.json riscv.pcf
	nextpnr-ice40 --up5k --json riscv.json --pcf riscv.pcf --asc riscv.asc --pcf-allow-unconstrained --opt-timing

timing: riscv.asc
	icetime -d up5k $^ | egrep -oi '\(\d+' | egrep -o '\d+' > $@

clean:
	rm -f riscv.json
	rm -f riscv.asc
	rm -f timing
	rm -f pll.v
	rm -f waves.vcd
	rm -rf sim sim.dSYM
	rm -rf cosim cosim.dSYM
	rm -f testbench.vvp testbench.vcd
	rm -f test/rtl.cc
	rm -f test/monitor.sim.v
	rm -f rvfi_macros.vh
	@# NOT rtl/rom.mem: gitignored, untracked, and nothing regenerates real
	@# contents for it, so `clean` deleting it is unrecoverable data loss.
	@# NOT tools/sail either: it is a multi-megabyte network fetch that nothing
	@# on `make test`'s path needs, so blowing it away on every `clean` costs a
	@# download to get back something `clean` was never asked to rebuild.
	@# `rm -rf tools/sail` by hand, or bump SAIL_RISCV_VERSION, to re-fetch.

# The riscv-formal clone rule and its pin guard live in formal/pin.mk, included
# above, so the root and formal/ Makefiles cannot drift apart on it.
