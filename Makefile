include formal/pin.mk

.DELETE_ON_ERROR:

RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32

rvfi_macros.vh: $(RISCV_FORMAL_DIR)/checks/rvfi_macros.py
	python3 $^ > $@

# The RTL both sim legs elaborate, named once. Do not spell this list out
# anywhere else: ci.yml's `elaborate` job carried a fourth copy, it went stale
# the moment rtl/imemory.v and rtl/memory.v landed, and the gate spent a run
# elaborating a testbench whose memory instances resolved to nothing. That job
# calls `make elaborate-strict` below, so there is one copy to update.
SIM_RTL_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
                rtl/fetcher.v rtl/imemory.v rtl/memory.v rtl/regfile.v rtl/writeback.v \
                rtl/littlecpu.v

testbench.vvp: $(SIM_RTL_SRCS) rvfi_macros.vh test/testbench.v test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS $(addprefix -D,$(RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: waves
waves: waves.vcd
waves.vcd: testbench.vvp
	vvp $<
	mv testbench.vcd $@

sim: test/cxxrtl.cc test/rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

# Sail co-simulation is deliberately opt-in: nothing from here to `cosim-suite`
# is reachable from `make test`, `make test-units` or CI's required set, so that
# `make test` keeps working on a machine with no Sail installed (ADR-0032).
ifneq ($(filter command line environment,$(origin SAIL_RISCV_VERSION)),)
$(error SAIL_RISCV_VERSION cannot be set from the command line or the \
  environment: it pins bytes this repo executes. Change it in the Makefile, \
  together with the SHA-256 digests below it)
endif
override SAIL_RISCV_VERSION := 0.13.1

ifeq ($(shell printf '%s' '$(SAIL_RISCV_VERSION)' | grep -cE '^[0-9]+\.[0-9]+\.[0-9]+$$'),0)
$(error SAIL_RISCV_VERSION must be a three-part release version like 0.13.1, \
  not a branch, a moving tag or a range: '$(SAIL_RISCV_VERSION)')
endif

# Prebuilt binaries because building from source needs opam, OCaml and the Sail
# compiler: there is no `brew install sail-riscv`, and homebrew's `sail` formula
# is an unrelated WordPress deploy tool.
SAIL_SHA256_sail-riscv-Mac-arm64     := 53d0c6fd84edd898728e7ba01c1575e66e5f17efd098847c5273690abbbd0737
SAIL_SHA256_sail-riscv-Linux-x86_64  := ee052f64494a2f5f071afd9c2cb4aa5eaae4ba84753e4f77e442b4f83f2e9469
SAIL_SHA256_sail-riscv-Linux-aarch64 := 3cd33a323d6749aec4667e54f71d2bf8e8e6e220a4e4bafd9083440f9a7e55f0

SAIL_ASSET_Darwin_arm64   := sail-riscv-Mac-arm64
SAIL_ASSET_Linux_x86_64   := sail-riscv-Linux-x86_64
SAIL_ASSET_Linux_aarch64  := sail-riscv-Linux-aarch64

SAIL_RISCV_DIR := tools/sail
SAIL_SIM_BIN   := $(SAIL_RISCV_DIR)/bin/sail_riscv_sim
SAIL_ASSET     := $(SAIL_ASSET_$(shell uname -s)_$(shell uname -m))
SAIL_SHA256    := $(SAIL_SHA256_$(SAIL_ASSET))

SAIL_STAMP := $(SAIL_RISCV_DIR)/.sail-pin
SAIL_PIN   := $(SAIL_RISCV_VERSION) $(SAIL_ASSET) $(SAIL_SHA256)

# Scoped to the goals that actually run the binary: a stale tools/sail that
# could fail `make test` would make co-simulation a gate by the back door.
ifneq ($(filter cosim-run cosim-suite,$(MAKECMDGOALS)),)
ifneq ($(wildcard $(SAIL_SIM_BIN)),)
SAIL_PIN_ON_DISK := $(shell sed -n 1p $(SAIL_STAMP) 2>/dev/null)
ifneq ($(SAIL_PIN_ON_DISK),$(SAIL_PIN))
$(error $(SAIL_RISCV_DIR) was fetched under '$(SAIL_PIN_ON_DISK)', not the pin \
  '$(SAIL_PIN)'. Re-fetch it with: make sail-setup)
endif
endif
endif

.PHONY: sail-setup
sail-setup:
	@set -e; \
	if [ -z '$(SAIL_ASSET)' ]; then \
	  echo "no prebuilt sail-riscv for $$(uname -s)/$$(uname -m);" >&2; \
	  echo "build it from https://github.com/riscv/sail-riscv and set" >&2; \
	  echo "SAIL_RISCV_SIM to the resulting sail_riscv_sim." >&2; \
	  exit 1; \
	fi; \
	if [ -z '$(SAIL_SHA256)' ]; then \
	  echo "no SHA-256 pinned for $(SAIL_ASSET) at $(SAIL_RISCV_VERSION);" >&2; \
	  echo "add one beside the others in the Makefile. Fetching an asset this" >&2; \
	  echo "repo cannot verify is not an option this target offers." >&2; \
	  exit 1; \
	fi; \
	if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256'; \
	elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum'; \
	else \
	  echo "neither shasum nor sha256sum is on PATH; refusing to unpack a" >&2; \
	  echo "tarball this machine cannot check." >&2; \
	  exit 1; \
	fi; \
	if [ -x $(SAIL_SIM_BIN) ] && \
	   [ "$$(sed -n 1p $(SAIL_STAMP) 2>/dev/null)" = '$(SAIL_PIN)' ]; then \
	  want=$$(sed -n 2p $(SAIL_STAMP)); \
	  got=$$($$sha $(SAIL_SIM_BIN) | cut -d ' ' -f 1); \
	  if [ "$$want" != "$$got" ]; then \
	    echo "$(SAIL_SIM_BIN) is not the binary its stamp was written for:" >&2; \
	    echo "  recorded : $$want" >&2; \
	    echo "  on disk  : $$got" >&2; \
	    echo "the tree changed after it was verified. Start over with:" >&2; \
	    echo "  rm -rf $(SAIL_RISCV_DIR) && make sail-setup" >&2; \
	    exit 1; \
	  fi; \
	  echo "sail-riscv $(SAIL_RISCV_VERSION) already verified in $(SAIL_RISCV_DIR)"; \
	  exit 0; \
	fi; \
	url=https://github.com/riscv/sail-riscv/releases/download/$(SAIL_RISCV_VERSION)/$(SAIL_ASSET).tar.gz; \
	tmp=$(SAIL_RISCV_DIR).tmp; tgz=$$tmp/$(SAIL_ASSET).tar.gz; \
	rm -rf $$tmp; mkdir -p $$tmp; \
	echo "fetching $$url"; \
	curl -fsSL -o $$tgz "$$url"; \
	got=$$($$sha $$tgz | cut -d ' ' -f 1); \
	if [ "$$got" != '$(SAIL_SHA256)' ]; then \
	  echo "sail-riscv tarball SHA-256 MISMATCH -- refusing to extract:" >&2; \
	  echo "  asset    : $(SAIL_ASSET).tar.gz at $(SAIL_RISCV_VERSION)" >&2; \
	  echo "  expected : $(SAIL_SHA256)" >&2; \
	  echo "  actual   : $$got" >&2; \
	  rm -rf $$tmp; \
	  exit 1; \
	fi; \
	echo "sha256 ok: $$got"; \
	tar tzf $$tgz | awk -v top='$(SAIL_ASSET)/' ' \
	  index($$0, top) != 1 { print "member outside " top ": " $$0 > "/dev/stderr"; bad = 1 } \
	  /(^|\/)\.\.(\/|$$)/  { print "traversal in member: " $$0 > "/dev/stderr"; bad = 1 } \
	  END { exit bad ? 1 : 0 }' \
	  || { echo "refusing to extract $(SAIL_ASSET).tar.gz" >&2; rm -rf $$tmp; exit 1; }; \
	tar tvzf $$tgz | awk ' \
	  substr($$1, 1, 1) !~ /^[-d]$$/ { print "not a file or directory: " $$0 > "/dev/stderr"; bad = 1 } \
	  END { exit bad ? 1 : 0 }' \
	  || { echo "refusing to extract $(SAIL_ASSET).tar.gz" >&2; rm -rf $$tmp; exit 1; }; \
	tar xzf $$tgz -C $$tmp --strip-components=1 \
	  --no-same-owner --no-same-permissions; \
	rm -f $$tgz; \
	test -x $$tmp/bin/sail_riscv_sim; \
	printf '%s\n' '$(SAIL_PIN)' > $$tmp/.sail-pin; \
	$$sha $$tmp/bin/sail_riscv_sim | cut -d ' ' -f 1 >> $$tmp/.sail-pin; \
	rm -rf $(SAIL_RISCV_DIR); \
	mv $$tmp $(SAIL_RISCV_DIR)
	@$(SAIL_SIM_BIN) --version

cosim: test/cosim.cc test/rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

PROG ?= add.S
.PHONY: cosim-run
cosim-run: cosim
	./test/cosim.py $(PROG)

.PHONY: cosim-suite
cosim-suite: cosim
	./test/run_cosim.sh ./cosim test/asm test/COSIM_EXPECTED_FAIL test/OBSERVED_FLOOR

# Both sim legs read this derived file, so changing a sanitizer rule changes the
# oracle rather than fixing an elaboration problem (ADR-0019).
test/monitor.sim.v: test/monitor.v test/sanitize_monitor.py
	python3 test/sanitize_monitor.py $< > $@

test/rtl.cc: $(SIM_RTL_SRCS) rvfi_macros.vh test/testbench.v test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top testbench; write_cxxrtl $@'

# `check` is what reports an undriven wire; the test/rtl.cc recipe above
# elaborates one with no diagnostic and exit 0.
ELABORATE_STRICT_OUT ?= /tmp/elaborate-strict.cc

# Keep the yosys script on one physical line. A backslash-continuation inside
# the single quotes is not removed by the shell, so yosys reads the backslash
# as a command and dies with "No such command: \".
.PHONY: elaborate-strict
elaborate-strict: $(SIM_RTL_SRCS) test/testbench.v
	yosys -p 'read_verilog -sv $(SIM_RTL_SRCS) test/testbench.v; hierarchy -top testbench; proc; opt_clean; check; write_cxxrtl $(ELABORATE_STRICT_OUT)'

# The `cd` is required: generate.py opens ../insns/isa_<isa>.txt relative to its
# own CWD.
MONITOR_GEN = cd $(RISCV_FORMAL_DIR)/monitor && python3 generate.py -i rv32imc -c 1 -a -p monitor

# The clone is order-only on purpose. A directory's mtime bumps on any write
# inside it, so a normal prerequisite would have routine builds overwriting this
# tracked file (invariant 7) at unpredictable moments.
test/monitor.v: $(RISCV_FORMAL_DIR)/monitor/generate.py formal/pin.mk | $(RISCV_FORMAL_DIR)
	$(MONITOR_GEN) > $(CURDIR)/$@

.PHONY: monitor-check
monitor-check: $(RISCV_FORMAL_DIR)/monitor/generate.py | $(RISCV_FORMAL_DIR)
	@tmp=$$(mktemp "$${TMPDIR:-/tmp}/monitor-check.XXXXXX"); \
	trap 'rm -f "$$tmp"' EXIT; \
	($(MONITOR_GEN)) > "$$tmp" && \
	diff -u test/monitor.v "$$tmp"

.PHONY: setup
setup:
ifeq ($(shell uname -s),Darwin)
	brew install riscv64-elf-gcc svlint
else
	@echo "On Linux, install the RISC-V cross compiler with:"
	@echo "  sudo apt-get install gcc-riscv64-unknown-elf"
	@echo
	@echo "svlint (the structural lint gate, \`make lint\`) is not packaged by"
	@echo "apt. Get the pinned release archive with:"
	@echo "  make lint-setup"
	@echo "which fetches, SHA-256-verifies and unpacks it into $(SVLINT_DIR)/."
	@echo "\`cargo install svlint --version $(SVLINT_VERSION)\` also works, but"
	@echo "builds from an unpinned crates.io tarball -- prefer lint-setup."
endif

ifneq ($(filter command line environment,$(origin SVLINT_VERSION)),)
$(error SVLINT_VERSION cannot be set from the command line or the environment: \
  it pins bytes this repo executes. Change it in the Makefile, together with \
  the SHA-256 digests below it)
endif
override SVLINT_VERSION := 0.9.5

ifeq ($(shell printf '%s' '$(SVLINT_VERSION)' | grep -cE '^[0-9]+\.[0-9]+\.[0-9]+$$'),0)
$(error SVLINT_VERSION must be a three-part release version like 0.9.5, not a \
  branch, a moving tag or a range: '$(SVLINT_VERSION)')
endif

# Do not interpolate $(SVLINT_VERSION) into these names. Spelling it out is what
# makes bumping the version without re-pinning resolve the lookup to empty, so
# `lint-setup` refuses to fetch instead of carrying a stale digest forward.
SVLINT_SHA256_svlint-v0.9.5-x86_64-lnx  := 0bbb3850b8ef604d7ccf25c2b0d2a751154ac2e18b2a12753ae1648f237a8ceb
SVLINT_SHA256_svlint-v0.9.5-x86_64-mac  := 53838f356862b6492777347999ccf44c1b44bc78f51cb032759b9e17bd213519
SVLINT_SHA256_svlint-v0.9.5-aarch64-mac := d032be600f0ee04130e0663daa05da3cc562d3d34bbc4305d6b70cb99310c6df

SVLINT_ASSET_Linux_x86_64  := svlint-v$(SVLINT_VERSION)-x86_64-lnx
SVLINT_ASSET_Darwin_x86_64 := svlint-v$(SVLINT_VERSION)-x86_64-mac
SVLINT_ASSET_Darwin_arm64  := svlint-v$(SVLINT_VERSION)-aarch64-mac

SVLINT_DIR   := tools/svlint
SVLINT_ASSET := $(SVLINT_ASSET_$(shell uname -s)_$(shell uname -m))
SVLINT_SHA256 := $(SVLINT_SHA256_$(SVLINT_ASSET))

SVLINT ?= $(shell command -v svlint 2>/dev/null || echo ./$(SVLINT_DIR)/bin/svlint)

# `-i rtl` is required, not decorative: svlint resolves `include "structs.v"`
# through the include path only. Both passes below are load-bearing too — a
# fifth of rtl/ sits behind `ifdef RISCV_FORMAL`, and svlint's preprocessor
# drops those blocks unless the macros are defined.
SVLINT_FLAGS := -c .svlint.toml -i rtl $(if $(GITHUB_ACTIONS),--github-actions,-1)

.PHONY: lint
lint:
	@command -v $(SVLINT) >/dev/null 2>&1 || test -x $(SVLINT) || { \
	  echo "svlint not found. Install it with 'make setup' (macOS) or" >&2; \
	  echo "'make lint-setup' (pinned release archive)." >&2; exit 1; }
	@echo "== svlint: rtl/, RVFI off =="
	$(SVLINT) $(SVLINT_FLAGS) rtl/*.v
	@echo "== svlint: rtl/, RVFI on =="
	$(SVLINT) $(SVLINT_FLAGS) $(addprefix -D ,$(RISCV_FORMAL_MACROS)) rtl/*.v
	@echo "svlint: clean in both passes"

.PHONY: lint-setup
lint-setup:
	@set -e; \
	if [ -z '$(SVLINT_ASSET)' ] || [ -z '$(SVLINT_SHA256)' ]; then \
	  echo "no svlint $(SVLINT_VERSION) release pinned for" >&2; \
	  echo "$$(uname -s)/$$(uname -m). Install it with 'brew install svlint'" >&2; \
	  echo "or 'cargo install svlint --version $(SVLINT_VERSION)'. Fetching an" >&2; \
	  echo "asset this repo cannot verify is not an option this target offers." >&2; \
	  exit 1; \
	fi; \
	if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256'; \
	elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum'; \
	else \
	  echo "neither shasum nor sha256sum is on PATH; refusing to unpack an" >&2; \
	  echo "archive this machine cannot check." >&2; \
	  exit 1; \
	fi; \
	tmp=$(SVLINT_DIR).tmp; zip=$$tmp/$(SVLINT_ASSET).zip; \
	url=https://github.com/dalance/svlint/releases/download/v$(SVLINT_VERSION)/$(SVLINT_ASSET).zip; \
	rm -rf $$tmp; mkdir -p $$tmp; \
	echo "fetching $$url"; \
	curl -fsSL -o $$zip "$$url"; \
	got=$$($$sha $$zip | cut -d ' ' -f 1); \
	if [ "$$got" != '$(SVLINT_SHA256)' ]; then \
	  echo "svlint archive SHA-256 MISMATCH -- refusing to extract:" >&2; \
	  echo "  asset    : $(SVLINT_ASSET).zip" >&2; \
	  echo "  expected : $(SVLINT_SHA256)" >&2; \
	  echo "  actual   : $$got" >&2; \
	  rm -rf $$tmp; \
	  exit 1; \
	fi; \
	echo "sha256 ok: $$got"; \
	unzip -Z1 $$zip | awk ' \
	  /(^|\/)\.\.(\/|$$)/ { print "traversal in member: " $$0 > "/dev/stderr"; bad = 1 } \
	  /^\// { print "absolute member: " $$0 > "/dev/stderr"; bad = 1 } \
	  END { exit bad ? 1 : 0 }' \
	  || { echo "refusing to extract $(SVLINT_ASSET).zip" >&2; rm -rf $$tmp; exit 1; }; \
	unzip -q $$zip -d $$tmp; \
	rm -f $$zip; \
	chmod +x $$tmp/bin/svlint; \
	test -x $$tmp/bin/svlint; \
	rm -rf $(SVLINT_DIR); \
	mv $$tmp $(SVLINT_DIR)
	@./$(SVLINT_DIR)/bin/svlint --version

UNIT_BENCHES := exec_tb mem_tb imem_tb decoder_tb regfile_tb csr_tb accessor_tb monitor_tb

UNIT_BENCH_SRC_exec_tb     := rtl/structs.v rtl/executor.v
UNIT_BENCH_SRC_mem_tb      := rtl/memory.v
UNIT_BENCH_SRC_imem_tb     := rtl/imemory.v
UNIT_BENCH_SRC_decoder_tb  := rtl/structs.v rtl/decoder.v
UNIT_BENCH_SRC_regfile_tb  := rtl/regfile.v
UNIT_BENCH_SRC_csr_tb      := rtl/structs.v rtl/csrs.v
UNIT_BENCH_SRC_accessor_tb := rtl/structs.v rtl/accessor.v
UNIT_BENCH_SRC_monitor_tb  := test/monitor.sim.v

# `present` is derived inside the recipe, not from a $(wildcard) at parse time:
# make caches directory contents, and a check whose idea of what is on disk can
# be stale is a check that can be wrong in the direction that matters.
#
# `set -e` comes first here and in `test-units`, before mktemp and before the
# trap. The other order installed the trap on an unchecked $$tmp: a failed
# mktemp left it empty and every path collapsed to the filesystem root.
.PHONY: check-unit-benches
check-unit-benches:
	@set -e; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/bench-inventory.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	printf '%s\n' $(UNIT_BENCHES) | sort > "$$tmp/declared"; \
	ls test/*_tb.v 2>/dev/null | sed -e 's|^test/||' -e 's|\.v$$||' | sort > "$$tmp/present"; \
	if [ ! -s "$$tmp/present" ]; then \
	  echo "error: no test/*_tb.v benches found; $(words $(UNIT_BENCHES)) are declared." >&2; \
	  exit 1; \
	fi; \
	if ! cmp -s "$$tmp/declared" "$$tmp/present"; then \
	  echo "error: the benches make runs are not the benches in test/:" >&2; \
	  comm -13 "$$tmp/declared" "$$tmp/present" \
	    | sed -e 's|^|  in test/ but not in UNIT_BENCHES: |' >&2; \
	  comm -23 "$$tmp/declared" "$$tmp/present" \
	    | sed -e 's|^|  in UNIT_BENCHES but not in test/: |' >&2; \
	  echo "A bench in test/ that make does not run is a test nothing executes;" >&2; \
	  echo "a declared bench with no file is a run that cannot happen. Fix the" >&2; \
	  echo "UNIT_BENCHES list in the Makefile, in the same commit either way." >&2; \
	  exit 1; \
	fi
	@set -e; $(foreach b,$(UNIT_BENCHES), \
	  test -n '$(UNIT_BENCH_SRC_$(b))' || { \
	    echo "error: $(b) is in UNIT_BENCHES with no UNIT_BENCH_SRC_$(b)." >&2; \
	    echo "Declare what it compiles against; an empty list would build the" >&2; \
	    echo "bench with no design under test and pass vacuously." >&2; \
	    exit 1; }; ) true
	@echo "$(words $(UNIT_BENCHES)) unit benches, matching test/*_tb.v exactly."

.PHONY: test-units
test-units: check-unit-benches test/monitor.sim.v
	@set -e; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/test-units.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	$(foreach b,$(UNIT_BENCHES), \
	  echo "== $(b) =="; \
	  iverilog -I./rtl/ -g2012 -o $$tmp/$(b).vvp $(UNIT_BENCH_SRC_$(b)) test/$(b).v; \
	  vvp $$tmp/$(b).vvp; ) \
	true

# A prerequisite of `test` rather than a target of its own, so it runs inside
# CI's required job with no workflow change. It needs no toolchain, no Sail, no
# yosys and no sby, so it cannot narrow where `make test` runs (ADR-0053).
.PHONY: probe-gates
probe-gates:
	@./test/probe_gates.sh

.PHONY: test
test: sim test-units probe-gates
	@./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

# Area is counted in nextpnr logic cells, never yosys cell counts: a DFF whose D
# input is not its co-located LUT's output takes a whole cell on its own, and
# over a thousand of this design's cells are exactly that. Two planning
# estimates were wrong in opposite directions from counting `SB_LUT4`.
#
# Do not try to make the placement succeed. This top presents 231 `SB_IO`
# against sg48's 39, so nextpnr always errors on a pad — after printing the
# utilisation table, which is the measurement. Placing it needs a real pinout,
# which means the SoC memory system (ADR-0038).
FIT_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
            rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v

fit.json: $(FIT_SRCS)
	@echo 'yosys: synthesising littlecpu for ice40 (log: fit.synth.log)'
	@yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlecpu -json $@' \
	  > fit.synth.log 2>&1 || { tail -40 fit.synth.log; exit 1; }

# Deliberately not the measurement (3899 cells locally): edits that synthesise
# to identical hardware move it by tens of cells, and the pinned CI toolchain
# reads ~21 higher than a local one, so 4100 keeps headroom over both. Raising
# this to clear a red is how a ratchet becomes a rubber stamp.
FIT_MAX_LC := 4100

.PHONY: fit
fit: fit.json
	@nextpnr-ice40 --up5k --package sg48 --json $< --pcf-allow-unconstrained \
	  > fit.log 2>&1 || true
	@python3 soc/fit_report.py fit.log --max-lc $(FIT_MAX_LC)

# A different design from `make fit`'s, whose numbers must not be merged with
# these: this is the core plus a block-RAM ROM, an SPRAM data RAM and four pins,
# so its logic-cell count includes the ROM's depth mux, the RAM's range decode
# and the LED taps. Unlike `fit` it must place, because `icetime` reads an
# `.asc` (ADR-0054).
SOC_PROG      ?= add.S
SOC_ROM_WORDS := 2048
# Exact rather than budgeted the way FIT_MAX_LC is, because both are properties
# of the RTL rather than of placement: 2 SPRAM for the 64 KB data RAM, and 16
# EBR for the 8 KB banked ROM plus 4 for rtl/regfile.v.
SOC_EXPECT_SPRAM := 2
SOC_EXPECT_EBR   := 20

SOC_SRCS      := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v \
                 rtl/executor.v rtl/fetcher.v rtl/imemory.v rtl/memory.v \
                 rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v

# PHONY so `soc.json` resynthesises every run: the image depends on SOC_PROG,
# which make cannot see a change to, and a stale ROM would make the measurement
# describe a program nobody asked for.
.PHONY: soc-rom
soc-rom:
	@set -e; \
	for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do \
	  if command -v $$candidate >/dev/null 2>&1; then CC=$$candidate; break; fi; \
	done; \
	if [ -z "$$CC" ]; then \
	  echo "error: no RISC-V cross compiler found; see \`make setup\`." >&2; exit 1; \
	fi; \
	OBJCOPY=$${CC%gcc}objcopy; \
	command -v $$OBJCOPY >/dev/null 2>&1 || { \
	  echo "error: $$OBJCOPY not found (half-installed toolchain)." >&2; exit 1; }; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/soc-rom.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	$$CC -march=rv32imc_zicsr_zifencei -mabi=ilp32 -nostdlib -I test/asm \
	  -T test/asm/sections.lds -o "$$tmp/prog.elf" test/asm/$(SOC_PROG); \
	$$OBJCOPY -O verilog --verilog-data-width=4 -j .text "$$tmp/prog.elf" "$$tmp/rom.hex"; \
	python3 soc/rom_banks.py "$$tmp/rom.hex" soc/rom_even.hex soc/rom_odd.hex \
	  --rom-words $(SOC_ROM_WORDS)

soc.json: $(SOC_SRCS) soc-rom
	@echo 'yosys: synthesising littlesoc for ice40 (log: soc.synth.log)'
	@yosys -p 'read_verilog -sv $(SOC_SRCS); synth_ice40 -dsp -spram -top littlesoc -json $@' \
	  > soc.synth.log 2>&1 || { tail -40 soc.synth.log; exit 1; }
	@# rtl/memory.v maps to SPRAM only because its read port is no-change on a
	@# write; the read-first spelling maps the same array to 148 `SB_RAM40_4K`
	@# -- five times the part's entire block RAM -- and yosys reports that as a
	@# normal run, failing later in nextpnr with a message about BELs.
	@# rtl/imemory.v maps to block RAM only while it stays a plain synchronous
	@# array. The census below is what catches either regression; soc/cell_census.py
	@# carries the reasoning for matching by exact count rather than by name.
	@python3 soc/cell_census.py soc.synth.log SB_SPRAM256KA $(SOC_EXPECT_SPRAM) \
	  "rtl/memory.v has stopped matching the SPRAM shape -- read its header comment about the no-change read port"
	@python3 soc/cell_census.py soc.synth.log SB_RAM40_4K $(SOC_EXPECT_EBR) \
	  "rtl/imemory.v or rtl/regfile.v has stopped inferring block RAM, or the ROM size changed"

# nextpnr's exit status is deliberately not the signal: it defaults to a 12 MHz
# target and exits nonzero when the design misses it, which this one does, having
# already written the `.asc`. Honouring the status would mean no icetime report
# at all because the design is 6% slow. `.DELETE_ON_ERROR` is why the `||` is
# needed rather than merely tidy — without it make deletes that `.asc`.
#
# `SOC_SEED=<n>` places the same netlist differently. One placement is a sample:
# unmodified `main` spans 87.43-88.51 ns over four of them (ADR-0058).
SOC_SEED ?=

soc.asc: soc.json soc/littlesoc.pcf
	@echo 'nextpnr: placing and routing littlesoc on up5k/sg48 (log: soc.pnr.log)'
	@nextpnr-ice40 --up5k --package sg48 --json $< --pcf soc/littlesoc.pcf $(if $(SOC_SEED),--seed '$(SOC_SEED)') \
	  --asc $@ > soc.pnr.log 2>&1 || true
	@test -s $@ || { \
	  echo '*** make soc-timing: nextpnr produced no bitstream, so NOTHING was'; \
	  echo '*** measured. That is a failed placement, not a slow design.'; \
	  tail -30 soc.pnr.log; \
	  rm -f $@; \
	  exit 1; \
	}
	@grep -q 'ICESTORM_LC:' soc.pnr.log || { \
	  echo '*** make soc-timing: nextpnr printed no utilisation table.'; \
	  tail -30 soc.pnr.log; \
	  rm -f $@; \
	  exit 1; \
	}

# A regression floor below the measurement, not ADR-0038's declared 12 MHz: the
# design measures 11.30 MHz, and the churn axis here is ~3.6% — two logically
# identical spellings of a module NOT on the critical path give 88.51 ns and
# 91.67 ns, because placement redistributes. Closing the gap to 12 MHz is a
# design decision, not something to buy by editing this line.
SOC_MIN_MHZ := 10.0

.PHONY: soc-timing
soc-timing: soc.asc
	@sed -n '/^Info: Device utilisation:/,/^$$/s/^Info: //p' soc.pnr.log
	@grep -E "Max frequency for clock .*'clk" soc.pnr.log | tail -1 \
	  | sed -e 's/^Info: /nextpnr /' -e 's/^ERROR: /nextpnr /'
	@echo
	@echo '== icetime: the critical path, and the LOGIC/ROUTING SPLIT =='
	@icetime -d up5k -P sg48 -p soc/littlesoc.pcf -t -r soc.timing.rpt soc.asc \
	  > soc.icetime.log 2>&1 || { cat soc.icetime.log; exit 1; }
	@echo
	@echo 'Every hop, with its cell and its delay: soc.timing.rpt'
	@echo 'nextpnr placement and its own timing analysis: soc.pnr.log'
	@echo
	@echo 'READ ADR-0054 BEFORE QUOTING ANY OF THIS. It is a static estimate for'
	@echo 'one placement of one build at the worst-case corner, and it is'
	@echo 'toolchain-dependent the same way `make fit` is. ADR-0038 declares Fmax'
	@echo 'at 12 MHz as an INTENT; this measurement does not move it in either'
	@echo 'direction, and the design does not currently meet it.'
	@# The ratchet is applied by the thing that already parses the report. It
	@# was a `python3 -c` here, i.e. a SECOND parser of the same file -- and the
	@# second one was the one holding the gate.
	@python3 soc/timing_split.py soc.timing.rpt --min-mhz $(SOC_MIN_MHZ)

clean:
	rm -f fit.json fit.log fit.synth.log
	rm -f soc.json soc.asc soc.synth.log soc.pnr.log soc.timing.rpt
	rm -f soc/rom_even.hex soc/rom_odd.hex
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
	@# Bumping SAIL_RISCV_VERSION and re-running `make sail-setup` re-fetches
	@# on its own now -- the pin is recorded in tools/sail/.sail-pin and
	@# compared, so this comment is no longer the only thing making that true.
	@# `rm -rf tools/sail` still works as the blunt instrument.
	@# NOT tools/svlint either, and for the same reason: a network fetch that
	@# `clean` was never asked to rebuild. `make lint-setup` re-fetches
	@# unconditionally, so `rm -rf tools/svlint` is the blunt instrument there.

# The riscv-formal clone rule and its pin guard live in formal/pin.mk.
