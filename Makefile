include formal/pin.mk
include soc/compare/hazard3_pin.mk

.DELETE_ON_ERROR:

RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM RISCV_FORMAL_MEM_FAULT RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32

rvfi_macros.vh: $(RISCV_FORMAL_DIR)/checks/rvfi_macros.py
	python3 $^ > $@

SIM_RTL_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
                rtl/fetcher.v rtl/imemory.v rtl/memory.v rtl/regfile.v rtl/regsel.v \
                rtl/timer.v rtl/uart.v rtl/spiflash.v rtl/writeback.v rtl/littlecpu.v

SIM_TB_SRCS := test/testbench.v test/spiflash_model.v

testbench.vvp: $(SIM_RTL_SRCS) rvfi_macros.vh $(SIM_TB_SRCS) test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS $(addprefix -D,$(RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

.PHONY: waves
waves: waves.vcd
waves.vcd: testbench.vvp
	vvp $<
	mv testbench.vcd $@

# SIM_OPT is a knob for one caller: mutation-check builds eleven times and wants -O0.
SIM_OPT ?= -O2

sim: test/cxxrtl.cc test/rtl.cc
	clang++ $(SIM_OPT) -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

# Outside the checkout, because a worktree gets tracked files only and a tool installed
# inside one is invisible from every other.
TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

include nano/nano.mk

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

SAIL_SHA256_sail-riscv-Mac-arm64     := 53d0c6fd84edd898728e7ba01c1575e66e5f17efd098847c5273690abbbd0737
SAIL_SHA256_sail-riscv-Linux-x86_64  := ee052f64494a2f5f071afd9c2cb4aa5eaae4ba84753e4f77e442b4f83f2e9469
SAIL_SHA256_sail-riscv-Linux-aarch64 := 3cd33a323d6749aec4667e54f71d2bf8e8e6e220a4e4bafd9083440f9a7e55f0

SAIL_ASSET_Darwin_arm64   := sail-riscv-Mac-arm64
SAIL_ASSET_Linux_x86_64   := sail-riscv-Linux-x86_64
SAIL_ASSET_Linux_aarch64  := sail-riscv-Linux-aarch64

SAIL_RISCV_DIR := $(TOOL_CACHE)/sail
SAIL_SIM_BIN   := $(SAIL_RISCV_DIR)/bin/sail_riscv_sim
SAIL_ASSET     := $(SAIL_ASSET_$(shell uname -s)_$(shell uname -m))
SAIL_SHA256    := $(SAIL_SHA256_$(SAIL_ASSET))

SAIL_STAMP := $(SAIL_RISCV_DIR)/.sail-pin
SAIL_PIN   := $(SAIL_RISCV_VERSION) $(SAIL_ASSET) $(SAIL_SHA256)

SAIL_DOWNLOAD_DIR := $(TOOL_CACHE)/download
SAIL_TARBALL      := $(SAIL_DOWNLOAD_DIR)/$(SAIL_ASSET)-$(SAIL_RISCV_VERSION).tar.gz
SAIL_CACHE_KEY    := sail-$(SAIL_RISCV_VERSION)-$(SAIL_ASSET)-$(SAIL_SHA256)

# Scoped to the goals that run the binary -- an unscoped check would break `make test`
# for everyone on a stale local cache.
ifneq ($(filter cosim-run cosim-suite sail-reservation-probe,$(MAKECMDGOALS)),)
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
	if [ -x '$(SAIL_SIM_BIN)' ] && \
	   [ "$$(sed -n 1p '$(SAIL_STAMP)' 2>/dev/null)" = '$(SAIL_PIN)' ]; then \
	  want=$$(sed -n 2p '$(SAIL_STAMP)'); \
	  got=$$($$sha '$(SAIL_SIM_BIN)' | cut -d ' ' -f 1); \
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
	mkdir -p '$(SAIL_DOWNLOAD_DIR)'; \
	tgz='$(SAIL_TARBALL)'; \
	if [ -f "$$tgz" ]; then \
	  echo "using the tarball already in $(SAIL_DOWNLOAD_DIR)"; \
	else \
	  url=https://github.com/riscv/sail-riscv/releases/download/$(SAIL_RISCV_VERSION)/$(SAIL_ASSET).tar.gz; \
	  echo "fetching $$url"; \
	  curl -fsSL -o "$$tgz".part "$$url"; \
	  mv "$$tgz".part "$$tgz"; \
	fi; \
	got=$$($$sha "$$tgz" | cut -d ' ' -f 1); \
	if [ "$$got" != '$(SAIL_SHA256)' ]; then \
	  echo "sail-riscv tarball SHA-256 MISMATCH -- refusing to extract:" >&2; \
	  echo "  asset    : $(SAIL_ASSET).tar.gz at $(SAIL_RISCV_VERSION)" >&2; \
	  echo "  expected : $(SAIL_SHA256)" >&2; \
	  echo "  actual   : $$got" >&2; \
	  echo "  tarball  : $$tgz (removed)" >&2; \
	  rm -f "$$tgz"; \
	  exit 1; \
	fi; \
	echo "sha256 ok: $$got"; \
	tmp=$$(mktemp -d '$(SAIL_RISCV_DIR)'.XXXXXX); \
	tar tzf "$$tgz" | awk -v top='$(SAIL_ASSET)/' ' \
	  index($$0, top) != 1 { print "member outside " top ": " $$0 > "/dev/stderr"; bad = 1 } \
	  /(^|\/)\.\.(\/|$$)/  { print "traversal in member: " $$0 > "/dev/stderr"; bad = 1 } \
	  END { exit bad ? 1 : 0 }' \
	  || { echo "refusing to extract $(SAIL_ASSET).tar.gz" >&2; rm -rf $$tmp; exit 1; }; \
	tar tvzf "$$tgz" | awk ' \
	  substr($$1, 1, 1) !~ /^[-d]$$/ { print "not a file or directory: " $$0 > "/dev/stderr"; bad = 1 } \
	  END { exit bad ? 1 : 0 }' \
	  || { echo "refusing to extract $(SAIL_ASSET).tar.gz" >&2; rm -rf $$tmp; exit 1; }; \
	tar xzf "$$tgz" -C $$tmp --strip-components=1 \
	  --no-same-owner --no-same-permissions; \
	test -x $$tmp/bin/sail_riscv_sim; \
	printf '%s\n' '$(SAIL_PIN)' > $$tmp/.sail-pin; \
	$$sha $$tmp/bin/sail_riscv_sim | cut -d ' ' -f 1 >> $$tmp/.sail-pin; \
	rm -rf '$(SAIL_RISCV_DIR)'; \
	mv $$tmp '$(SAIL_RISCV_DIR)'
	@'$(SAIL_SIM_BIN)' --version

.PHONY: sail-pin
sail-pin:
	@printf 'key=%s\n' '$(SAIL_CACHE_KEY)'
	@printf 'path=%s\n' '$(SAIL_DOWNLOAD_DIR)'
	@printf 'tarball=%s\n' '$(SAIL_TARBALL)'

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

.PHONY: sail-reservation-probe
sail-reservation-probe:
	./test/sail/reservation_probe.sh $(SAIL_SIM_BIN)

# Both sim legs check every retire against this file; edit test/sanitize_monitor.py, not
# this rule, or the change is silent.
test/monitor.sim.v: test/monitor.v test/sanitize_monitor.py
	python3 test/sanitize_monitor.py $< > $@

test/rtl.cc: $(SIM_RTL_SRCS) rvfi_macros.vh $(SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top testbench; write_cxxrtl $@'

# A separate harness, not a configuration axis: none of this is on `make test`'s path.
DUAL_RTL_SRCS := $(SIM_RTL_SRCS) rtl/busarbiter.v rtl/littledual.v

test/dual_rtl.cc: $(DUAL_RTL_SRCS) rvfi_macros.vh test/dual_testbench.v test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top dual_testbench; write_cxxrtl $@'

dual-sim: test/dual_cxxrtl.cc test/dual_rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

.PHONY: dual-elaborate
dual-elaborate: $(DUAL_RTL_SRCS) rvfi_macros.vh test/dual_testbench.v test/monitor.sim.v
	iverilog -I./rtl/ $(addprefix -D,$(RISCV_FORMAL_MACROS)) -g2012 -o /dev/null $^
	@echo 'dual-elaborate: iverilog read test/dual_testbench.v'

.PHONY: dual-smoke
dual-smoke: dual-sim
	@./test/dual_smoke.sh ./dual-sim

# Dhrystone on the dual configuration, in two shapes; the two runner scripts have their
# own headers.
.PHONY: dual-dhrystone-contention
dual-dhrystone-contention: dual-sim
	@./test/dual/bench/run_contention.sh ./dual-sim $(DHRY_RUNS) $(DHRY_CYCLES) '$(DHRY_CFLAGS)'

.PHONY: dual-dhrystone-aggregate
dual-dhrystone-aggregate: dual-sim
	@./test/dual/bench/run_aggregate.sh ./dual-sim $(DHRY_RUNS) $(DHRY_CYCLES) '$(DHRY_CFLAGS)'

# Keep this yosys -p script on one line: a backslash split inside its single quotes stays
# literal, and yosys dies on it on CI's make though not on macOS's.
.PHONY: elaborate-strict
elaborate-strict: $(SIM_RTL_SRCS) $(SIM_TB_SRCS)
	yosys -p 'read_verilog -sv $(SIM_RTL_SRCS) $(SIM_TB_SRCS); hierarchy -top testbench; proc; opt_clean; check; write_cxxrtl /tmp/elaborate-strict.cc'

MONITOR_GEN = cd $(RISCV_FORMAL_DIR)/monitor && python3 generate.py -i rv32imc -c 1 -a -p monitor

# Order-only (after `|`): a normal prerequisite goes stale on any write inside that
# directory and rewrites this checked-in file into someone else's commit.
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

# Names omit $(SVLINT_VERSION) on purpose: bumping the version without adding digests
# then makes the lookup empty, and lint-setup refuses rather than fetches.
SVLINT_SHA256_svlint-v0.9.5-x86_64-lnx  := 0bbb3850b8ef604d7ccf25c2b0d2a751154ac2e18b2a12753ae1648f237a8ceb
SVLINT_SHA256_svlint-v0.9.5-x86_64-mac  := 53838f356862b6492777347999ccf44c1b44bc78f51cb032759b9e17bd213519
SVLINT_SHA256_svlint-v0.9.5-aarch64-mac := d032be600f0ee04130e0663daa05da3cc562d3d34bbc4305d6b70cb99310c6df

SVLINT_ASSET_Linux_x86_64  := svlint-v$(SVLINT_VERSION)-x86_64-lnx
SVLINT_ASSET_Darwin_x86_64 := svlint-v$(SVLINT_VERSION)-x86_64-mac
SVLINT_ASSET_Darwin_arm64  := svlint-v$(SVLINT_VERSION)-aarch64-mac

SVLINT_DIR   := $(TOOL_CACHE)/svlint
SVLINT_ASSET := $(SVLINT_ASSET_$(shell uname -s)_$(shell uname -m))
SVLINT_SHA256 := $(SVLINT_SHA256_$(SVLINT_ASSET))

SVLINT ?= $(shell command -v svlint 2>/dev/null || echo $(SVLINT_DIR)/bin/svlint)

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
	url=https://github.com/dalance/svlint/releases/download/v$(SVLINT_VERSION)/$(SVLINT_ASSET).zip; \
	mkdir -p '$(TOOL_CACHE)'; \
	tmp=$$(mktemp -d '$(SVLINT_DIR)'.XXXXXX); zip=$$tmp/$(SVLINT_ASSET).zip; \
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
	rm -rf '$(SVLINT_DIR)'; \
	mv $$tmp '$(SVLINT_DIR)'
	@'$(SVLINT_DIR)'/bin/svlint --version

UNIT_BENCHES := exec_tb mem_tb imem_tb decoder_tb regfile_tb csr_tb accessor_tb monitor_tb \
                timer_tb uart_tb spiflash_tb pin_lockout_tb miso_share_enable_tb

UNIT_BENCH_SRC_exec_tb     := rtl/structs.v rtl/executor.v
UNIT_BENCH_SRC_mem_tb      := rtl/memory.v
UNIT_BENCH_SRC_imem_tb     := rtl/imemory.v
UNIT_BENCH_SRC_decoder_tb  := rtl/structs.v rtl/decoder.v rtl/regsel.v
UNIT_BENCH_SRC_regfile_tb  := rtl/regfile.v
UNIT_BENCH_SRC_csr_tb      := rtl/structs.v rtl/csrs.v
UNIT_BENCH_SRC_accessor_tb := rtl/structs.v rtl/accessor.v
UNIT_BENCH_SRC_monitor_tb  := test/monitor.sim.v
UNIT_BENCH_SRC_timer_tb    := rtl/timer.v
UNIT_BENCH_SRC_uart_tb     := rtl/uart.v
UNIT_BENCH_SRC_spiflash_tb := rtl/spiflash.v test/spiflash_model.v
UNIT_BENCH_SRC_pin_lockout_tb := soc/pin_lockout.v
UNIT_BENCH_SRC_miso_share_enable_tb := soc/miso_share_enable.v

# `present` reads the directory in the recipe, not via $(wildcard) -- make caches that
# and a stale listing could miss a bench that is really there.
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

.PHONY: unit-bench-list
unit-bench-list:
	@printf '%s\n' $(UNIT_BENCHES)

.PHONY: $(addprefix test-unit-,$(UNIT_BENCHES))
$(addprefix test-unit-,$(UNIT_BENCHES)): test-unit-%: test/monitor.sim.v
	@set -e; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/test-unit.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	iverilog -I./rtl/ -g2012 -o $$tmp/$*.vvp $(UNIT_BENCH_SRC_$*) test/$*.v; \
	vvp $$tmp/$*.vvp

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

.PHONY: probe-gates
probe-gates:
	@./test/probe_gates.sh

.PHONY: pin-bump-test
pin-bump-test:
	@./formal/test-propose-pin-bump.sh

.PHONY: tool-cache-test
tool-cache-test:
	@./test/tool_cache_test.sh '$(SAIL_RISCV_DIR)' '$(SVLINT_DIR)' '$(SAIL_DOWNLOAD_DIR)' '$(NANO_LIBERTY_DIR)'

.PHONY: memmap-test
memmap-test:
	@./test/memmap_test.sh

# Asserts that every rtl/*.v file has a ruling on whether a mutation of it is caught by
# anything -- a named mutation, or `unpaired` and a real bench or formal task -- checked
# against `ls rtl/*.v` both ways round.
.PHONY: comment-density-test
comment-density-test:
	@python3 ./test/comment_density_test.py

.PHONY: mutation-coverage-test
mutation-coverage-test:
	@./test/mutation_coverage_test.sh

# Asserts that every ADR file has a unique number and exactly one row in
# docs/adr/README.md, both ways round.
.PHONY: adr-numbering-test
adr-numbering-test:
	@./test/adr_numbering_test.sh

# A target defined twice is last-wins with only a warning, and a `?=` default beside it
# is first-wins, so two additions of one name pair one route's body with the other's
# variables.
.PHONY: makefile-target-test
makefile-target-test:
	@./test/makefile_target_test.sh

# The cross-core comparison harness states its geometry in several places, read from this
# Makefile's own COMPARE_TOP/-T lines rather than a second hand-kept list, and this is
# what says they agree.
.PHONY: compare-geometry-test
compare-geometry-test:
	@./soc/compare/geometry_test.sh

# The two IVERILOG comparison recipes must read VexRiscv through $(VEXRISCV_V) and never
# through the riscv-formal clone -- see soc/compare/vexriscv_pin.mk for why the two
# builds are not peers.
.PHONY: vexriscv-path-test
vexriscv-path-test:
	@./soc/compare/vexriscv_path_test.sh

.PHONY: port-connect-test
port-connect-test:
	@python3 ./test/port_connect_test.py

.PHONY: retired-term-test
retired-term-test:
	@./test/retired_term_test.sh

# The ISA string is stated at seven sites and three of them build programs that use no
# atomic, so a site left behind goes on producing numbers rather than failing to
# assemble.
.PHONY: march-test
march-test:
	@./test/march_test.sh

# A `.gitignore` rule never applies to a file git already tracks, so a tracked file
# matching one is always a mistake -- a dead rule, or a commit that should not have
# happened.
.PHONY: tracked-ignored-test
tracked-ignored-test:
	@./test/tracked_ignored_test.sh

.PHONY: band-source-test
band-source-test:
	@python3 ./test/band_source_test.py

.PHONY: zkt-isolation-test
zkt-isolation-test:
	@python3 ./test/zkt_isolation_test.py

# Refuses a bare `sed -i` in test/probe_gates.sh's own fixtures (it proves nothing when
# the pattern matches nothing) and a hand-typed fixture with no fixture_anchor tying it
# to the real shape it imitates.
.PHONY: fixture-freshness-test
fixture-freshness-test:
	@python3 ./test/fixture_freshness_test.py

# Forces the elaboration checks in rtl/{imemory,memory,timer,uart,spiflash}.v and
# rtl/littlecpu.v's copy of that map to fire, in both frontends.
.PHONY: window-test
window-test:
	@./test/window_test.sh

.PHONY: imem-share-test
imem-share-test:
	@./test/imem_share_test.sh

.PHONY: abc-engine-test
abc-engine-test:
	@./formal/test-abc-engine.sh

# MUTATION_SHARD=<i>/<n> grades every nth mutation starting at i, so CI can run the
# eleven of them as several jobs.
.PHONY: mutation-check
mutation-check:
	@./test/mutation_check.sh $(if $(MUTATION_SHARD),--shard $(MUTATION_SHARD))

.PHONY: mutation-probe
mutation-probe:
	@./test/mutation_probe.sh

# The two-hart programs. Only one of them runs (`make dual-smoke`, off `test`'s path).
.PHONY: dual-build
dual-build:
	@./test/dual_build.sh test/dual test/asm test/dual/MUTATION_PAIRINGS

.PHONY: test
test: sim test-units probe-gates pin-bump-test tool-cache-test memmap-test \
      adr-numbering-test compare-geometry-test vexriscv-path-test retired-term-test port-connect-test march-test \
      band-source-test zkt-isolation-test fixture-freshness-test window-test imem-share-test \
      abc-engine-test makefile-target-test mutation-probe dual-build board-elaborate \
      tracked-ignored-test mutation-coverage-test comment-density-test
	@./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

.PHONY: cycles
cycles: sim
	@STALL_REPORT=1 ./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

# Dhrystone 2.1, the one number this core can be quoted against other cores'.
DHRY_RUNS   ?= 2000
DHRY_CYCLES ?= 4000000
DHRY_CFLAGS := -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -O2 -std=c11 \
               -ffreestanding -fno-tree-loop-distribute-patterns \
               -Wall -Wextra -Werror

.PHONY: dhrystone
dhrystone: sim
	@./test/bench/run_dhrystone.sh ./sim $(DHRY_RUNS) $(DHRY_CYCLES) '$(DHRY_CFLAGS)'

# CoreMark, SIMULATED AT 16 KB OF ROM -- double the part's 8, because it does not fit the
# smaller one.
COREMARK_ITERATIONS ?= 100
COREMARK_CYCLES     ?= 200000000
COREMARK_CFLAGS := -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -O2 -std=c11 \
                    -ffreestanding -fno-tree-loop-distribute-patterns \
                    -Wall -Wextra -Werror

ifneq ($(filter command line environment,$(origin COREMARK_PIN)),)
$(error COREMARK_PIN cannot be set from the command line or the environment: \
  it pins the bytes this target treats as ground truth. Change it in the \
  Makefile, together with test/bench/coremark/PINNED.sha256's header)
endif
override COREMARK_PIN := 1f483d5b8316753a742cbf5590caf5bd0a4e4777

ifeq ($(shell printf '%s' '$(COREMARK_PIN)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error COREMARK_PIN must be a full 40-hex commit id, not a branch or tag: '$(COREMARK_PIN)')
endif

COREMARK_VENDOR_DIR := test/bench/coremark

.PHONY: revendor-coremark
revendor-coremark:
	@set -e; \
	if command -v shasum >/dev/null 2>&1; then sha='shasum -a 256'; \
	elif command -v sha256sum >/dev/null 2>&1; then sha='sha256sum'; \
	else \
	  echo "neither shasum nor sha256sum is on PATH; refusing to verify a" >&2; \
	  echo "tree this machine cannot hash." >&2; \
	  exit 1; \
	fi; \
	files=$$(awk '!/^#/ && NF { print $$NF }' '$(COREMARK_VENDOR_DIR)/PINNED.sha256'); \
	if [ -z "$$files" ]; then \
	  echo "$(COREMARK_VENDOR_DIR)/PINNED.sha256 names no files; nothing to verify." >&2; \
	  exit 1; \
	fi; \
	tmp=$$(mktemp -d); trap 'rm -rf $$tmp' EXIT; \
	tgz=$$tmp/coremark.tar.gz; \
	url=https://codeload.github.com/eembc/coremark/tar.gz/$(COREMARK_PIN); \
	echo "fetching $$url"; \
	curl -fsSL -o "$$tgz" "$$url"; \
	prefix=coremark-$(COREMARK_PIN); \
	mismatch=0; \
	for f in $$files; do \
	  if ! tar xzf "$$tgz" -O "$$prefix/$$f" > "$$tmp/$$f" 2>/dev/null; then \
	    echo "MISSING upstream: $$f is not at $$prefix/$$f in the pinned archive" >&2; \
	    mismatch=1; continue; \
	  fi; \
	  got=$$($$sha "$$tmp/$$f" | cut -d ' ' -f 1); \
	  want=$$($$sha '$(COREMARK_VENDOR_DIR)'/$$f | cut -d ' ' -f 1); \
	  if [ "$$got" = "$$want" ]; then \
	    echo "match    : $$f"; \
	  else \
	    echo "DIFFERS  : $$f" >&2; \
	    echo "  vendored : $$want" >&2; \
	    echo "  upstream : $$got" >&2; \
	    mismatch=1; \
	  fi; \
	done; \
	if [ "$$mismatch" -ne 0 ]; then \
	  echo "$(COREMARK_VENDOR_DIR) does NOT match eembc/coremark at $(COREMARK_PIN)." >&2; \
	  echo "This is a finding, not something this target fixes: read the diff" >&2; \
	  echo "above, decide whether to accept the new bytes, and update both" >&2; \
	  echo "$(COREMARK_VENDOR_DIR)/ and its PINNED.sha256 by hand if so." >&2; \
	  exit 1; \
	fi; \
	echo "$(COREMARK_VENDOR_DIR) matches eembc/coremark at $(COREMARK_PIN) exactly."

.PHONY: coremark
coremark: sim
	@./test/bench/run_coremark.sh ./sim $(COREMARK_ITERATIONS) $(COREMARK_CYCLES) \
	  '$(COREMARK_CFLAGS)'

# Count logic cells from nextpnr, never cell counts from yosys: the two disagree in
# magnitude and in sign on the same netlist.
FIT_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
            rtl/fetcher.v rtl/regfile.v rtl/regsel.v rtl/writeback.v rtl/littlecpu.v

fit.json: $(FIT_SRCS)
	@echo 'yosys: synthesising littlecpu for ice40 (log: fit.synth.log)'
	@yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlecpu -json $@' \
	  > fit.synth.log 2>&1 || { tail -40 fit.synth.log; exit 1; }

# 4219 = 4097 + 68 + 54: the fit job's measured count, the measured churn band, and the
# widest toolchain gap measured on one tree.
FIT_MAX_LC := 4219

FIT_LAST_LC := 4097

FIT_TOOLS := yosys nextpnr-ice40

.PHONY: fit-toolchain
fit-toolchain:
	@soc/print_toolchain.sh $(FIT_TOOLS)

.PHONY: fit
fit: fit-toolchain fit.json
	@nextpnr-ice40 --up5k --package sg48 --json fit.json --pcf-allow-unconstrained \
	  > fit.log 2>&1 || true
	@python3 soc/fit_report.py fit.log --max-lc $(FIT_MAX_LC) --previous $(FIT_LAST_LC)

SOC_PROG      ?= datainit.c
SOC_ROM_WORDS := 2048
# Named once and referenced by every littlesoc synthesis (ice40, ECP5, the iCESugar-Pro
# top): chparam before hierarchy/synth_*, so littlesoc's default is what gets elaborated
# rather than a second copy of the number.
SOC_ROM_CHPARAM := $(if $(filter command line,$(origin SOC_ROM_WORDS)),chparam -set ROM_WORDS $(SOC_ROM_WORDS) littlesoc;)
# Exact rather than budgeted the way FIT_MAX_LC is, because both are properties of the
# RTL rather than of placement: 2 SPRAM for the 64 KB data RAM, and 16 EBR for the 8 KB
# banked ROM plus 4 for rtl/regfile.v.
SOC_EXPECT_SPRAM := 2
SOC_EXPECT_EBR   := 20

SOC_SRCS      := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v \
                 rtl/executor.v rtl/fetcher.v rtl/imemory.v rtl/memory.v \
                 rtl/regfile.v rtl/regsel.v rtl/timer.v rtl/uart.v rtl/spiflash.v \
                 rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v

# PHONY because SOC_PROG changes what this builds and make cannot see that from a
# timestamp.
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
	case '$(SOC_PROG)' in \
	  */*) prog='$(SOC_PROG)' ;; \
	  *)   prog='test/asm/$(SOC_PROG)' ;; \
	esac; \
	test -f "$$prog" || { echo "error: no such program: $$prog" >&2; exit 1; }; \
	case '$(SOC_PROG)' in \
	  *.c) $$CC -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -nostdlib \
	         -Os -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns \
	         -Wall -Wextra -Werror -I test/asm -T test/asm/boot.lds \
	         -o "$$tmp/prog.elf" test/crt0.S "$$prog"; \
	       sections='-j .text -j .data' ;; \
	  *)   $$CC -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -nostdlib -I test/asm \
	         -T test/asm/sections.lds -o "$$tmp/prog.elf" "$$prog"; \
	       sections='-j .text' ;; \
	esac; \
	$$OBJCOPY -O verilog --verilog-data-width=4 $$sections "$$tmp/prog.elf" "$$tmp/rom.hex"; \
	python3 soc/rom_banks.py "$$tmp/rom.hex" soc/rom_even.hex soc/rom_odd.hex \
	  --rom-words $(SOC_ROM_WORDS)

# Named once, used verbatim everywhere the netlist matters: a second copy would let the
# digest and the placement it grades describe different builds.
SOC_SYNTH := read_verilog -sv $(SOC_SRCS); \
             $(SOC_ROM_CHPARAM) \
             synth_ice40 -device u -dsp -spram -top littlesoc
SOC_PNR   := nextpnr-ice40 --up5k --package sg48 --pcf soc/littlesoc.pcf

soc.json: $(SOC_SRCS) soc-rom
	@echo 'yosys: synthesising littlesoc for ice40 (log: soc.synth.log)'
	@yosys -p '$(SOC_SYNTH) -json $@' \
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

# `|| true` matters: nextpnr's exit status is not the signal (icetime's report of the
# .asc is), and without it .DELETE_ON_ERROR deletes the .asc unread.
SOC_SEED ?=

soc.asc: soc.json soc/littlesoc.pcf
	@echo 'nextpnr: placing and routing littlesoc on up5k/sg48 (log: soc.pnr.log)'
	@$(SOC_PNR) --json $< $(if $(SOC_SEED),--seed '$(SOC_SEED)') \
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

SOC_MIN_MHZ := 12.0

SOC_TIMING_TOOLS := yosys nextpnr-ice40 icetime

.PHONY: soc-timing-toolchain
soc-timing-toolchain:
	@soc/print_toolchain.sh $(SOC_TIMING_TOOLS)

.PHONY: soc-timing
soc-timing: soc-timing-toolchain soc.asc
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
	@echo 'toolchain-dependent the same way `make fit` is. 12 MHz is a'
	@echo 'REQUIREMENT as of ADR-0066: it is the board clock, and the step below'
	@echo 'it is 6 MHz. One placement is a sample: soc/timing_sweep.sh prints the'
	@echo 'spread, and a requirement has to hold at all of them.'
	@# The ratchet is applied by the thing that already parses the report. It
	@# was a `python3 -c` here, i.e. a SECOND parser of the same file -- and the
	@# second one was the one holding the gate.
	@python3 soc/timing_split.py soc.timing.rpt --min-mhz $(SOC_MIN_MHZ)

ECP5_DEVICE  := --25k
ECP5_PACKAGE := CABGA381
ECP5_SPEED   := 6
ECP5_PART    := LFE5U-25F-6CABGA381

ECP5_TARGET_MHZ := 200.0

ECP5_CLOCK := clk

ECP5_EXPECT_DP16KD := 36
ECP5_EXPECT_LUTRAM := 32
ECP5_EXPECT_DSP    := 4

ECP5_SEED ?=

ecp5.json: $(SOC_SRCS) soc-rom
	@echo 'yosys: synthesising littlesoc for ECP5 (log: ecp5.synth.log)'
	@# Plain `synth_ecp5`, no mapper flags. abc9 is this script's default on this
	@# part, so passing `-noabc9` would be as much of a mapper change as turning
	@# abc9 on is on ice40, and a mapper change landing under a brand-new
	@# instrument would confound both.
	@yosys -p 'read_verilog -sv $(SOC_SRCS); $(SOC_ROM_CHPARAM) synth_ecp5 -top littlesoc -json $@' \
	  > ecp5.synth.log 2>&1 || { tail -40 ecp5.synth.log; exit 1; }
	@python3 soc/cell_census.py ecp5.synth.log DP16KD $(ECP5_EXPECT_DP16KD) \
	  "rtl/memory.v's no-change read port was shaped for SPRAM inference and there is no SPRAM on this part, so a spelling that stops matching block RAM falls back to LUT RAM and says nothing" \
	  --gate 'make ecp5-timing' --declared ECP5_EXPECT_DP16KD
	@python3 soc/cell_census.py ecp5.synth.log TRELLIS_DPR16X4 $(ECP5_EXPECT_LUTRAM) \
	  "rtl/regfile.v has stopped inferring distributed RAM, which is the only memory it maps to here -- zero means it fell into flops and soft muxes" \
	  --gate 'make ecp5-timing' --declared ECP5_EXPECT_LUTRAM
	@python3 soc/cell_census.py ecp5.synth.log MULT18X18D $(ECP5_EXPECT_DSP) \
	  "rtl/executor.v's multiplier has stopped inferring a DSP block; in soft logic it would be invisible in a frequency number and enormous in area" \
	  --gate 'make ecp5-timing' --declared ECP5_EXPECT_DSP
	@python3 soc/bram_reset_check.py $@ --gate 'make ecp5-timing'

ICESUGAR_DEVICE  := --25k
ICESUGAR_PACKAGE := CABGA256
ICESUGAR_SPEED   := 6
ICESUGAR_PART    := LFE5U-25F-6BG256C
ICESUGAR_MHZ     := 25
ICESUGAR_TOP     := icesugar_pro_top
ICESUGAR_SRCS    := $(SOC_SRCS) soc/board_icesugar_pro.v
ICESUGAR_PROG    ?= soc/blink.S

ICESUGAR_ROM     ?= soc-rom

icesugar.json: $(ICESUGAR_SRCS) soc/icesugar_pro.lpf
	@$(MAKE) --no-print-directory $(ICESUGAR_ROM) SOC_PROG=$(ICESUGAR_PROG)
	@echo 'yosys: synthesising $(ICESUGAR_TOP) for $(ICESUGAR_PART) (log: icesugar.synth.log)'
	@# chparam names littlesoc, not $(ICESUGAR_TOP): the parameter lives on the
	@# submodule icesugar_pro_top instantiates at its own default, so setting
	@# littlesoc's default before hierarchy is what icesugar_pro_top inherits.
	@yosys -p 'read_verilog -sv $(ICESUGAR_SRCS); $(SOC_ROM_CHPARAM) synth_ecp5 -top $(ICESUGAR_TOP) -json $@' \
	  > icesugar.synth.log 2>&1 || { tail -40 icesugar.synth.log; exit 1; }
	@python3 soc/bram_reset_check.py $@ --gate 'make icesugar-bitstream'

icesugar.config: icesugar.json
	@rm -f $@
	@echo 'nextpnr: placing $(ICESUGAR_TOP) on $(ICESUGAR_PART) at $(ICESUGAR_MHZ) MHz (log: icesugar.pnr.log)'
	@nextpnr-ecp5 $(ICESUGAR_DEVICE) --package $(ICESUGAR_PACKAGE) --speed $(ICESUGAR_SPEED) \
	  --json $< --lpf soc/icesugar_pro.lpf --freq $(ICESUGAR_MHZ) \
	  --textcfg $@ > icesugar.pnr.log 2>&1 || { tail -30 icesugar.pnr.log; exit 1; }
	@test -s $@ || { echo '*** nextpnr wrote no configuration.'; tail -30 icesugar.pnr.log; exit 1; }

icesugar.bit: icesugar.config
	@ecppack $< $@
	@test -s $@ || { echo '*** ecppack wrote no bitstream.'; exit 1; }

.PHONY: icesugar-bitstream
icesugar-bitstream: icesugar.bit
	@echo
	@echo '== $(ICESUGAR_PART): a bitstream, not a measurement =='
	@grep -E 'Max frequency for clock' icesugar.pnr.log | tail -2
	@ls -l icesugar.bit | awk '{ print "icesugar.bit  " $$5 " bytes" }'
	@echo
	@echo 'Put it on the board with `make icesugar-prog`. What the tools think'
	@echo 'the placement does is above; a board is the only thing that can'
	@echo 'disagree.'

ICESUGAR_LOADER  ?= openFPGALoader
ICESUGAR_VID     ?= 0x1d50
ICESUGAR_PID     ?= 0x602b
ICESUGAR_READ_S  ?= 30

.PHONY: icesugar-prog
icesugar-prog: icesugar.bit
	@$(ICESUGAR_LOADER) -c cmsisdap --vid $(ICESUGAR_VID) --pid $(ICESUGAR_PID) \
	  -m icesugar.bit
	@echo
	@echo 'Loaded into SRAM; the design is running. Read it with'
	@echo '`make icesugar-read`, or power-cycle the board to go back to flash.'

.PHONY: icesugar-read
icesugar-read:
	@python3 soc/board_read.py --seconds $(ICESUGAR_READ_S)

.PHONY: icesugar-dhrystone
icesugar-dhrystone:
	@rm -f icesugar.json icesugar.config icesugar.bit
	@$(MAKE) --no-print-directory dhrystone-rom
	@$(MAKE) --no-print-directory icesugar.bit ICESUGAR_ROM=noop-rom
	@$(MAKE) --no-print-directory icesugar-prog
	@python3 soc/board_read.py --seconds 60 --until 'Self-check' \
	  --out icesugar_dhrystone.txt

ICESUGAR_COREMARK_ROM_WORDS := 4096

.PHONY: icesugar-coremark
icesugar-coremark:
	@rm -f icesugar.json icesugar.config icesugar.bit
	@$(MAKE) --no-print-directory coremark-rom-ecp5 \
	  SOC_ROM_WORDS=$(ICESUGAR_COREMARK_ROM_WORDS)
	@$(MAKE) --no-print-directory icesugar.bit ICESUGAR_ROM=noop-rom \
	  SOC_ROM_WORDS=$(ICESUGAR_COREMARK_ROM_WORDS)
	@$(MAKE) --no-print-directory icesugar-prog
	@python3 soc/board_read.py --seconds 60 --until 'Self-check' \
	  --out icesugar_coremark.txt

ECP5_TOOLS := yosys nextpnr-ecp5 trellis-db

.PHONY: ecp5-timing-toolchain
ecp5-timing-toolchain:
	@soc/print_toolchain.sh $(ECP5_TOOLS)

ecp5.config: ecp5.json soc/littlesoc.lpf
	@rm -f $@ ecp5.report.json
	@echo 'nextpnr: placing and routing littlesoc on $(ECP5_PART) (log: ecp5.pnr.log)'
	@nextpnr-ecp5 $(ECP5_DEVICE) --package $(ECP5_PACKAGE) --speed $(ECP5_SPEED) \
	  --json $< --lpf soc/littlesoc.lpf --lpf-allow-unconstrained \
	  --freq $(ECP5_TARGET_MHZ) $(if $(ECP5_SEED),--seed '$(ECP5_SEED)') \
	  --textcfg $@ --report ecp5.report.json > ecp5.pnr.log 2>&1 || true
	@{ test -s $@ && test -s ecp5.report.json; } || { \
	  echo '*** make ecp5-timing: nextpnr wrote no configuration and report pair,'; \
	  echo '*** so NOTHING was measured. That is a failed run, not a slow design,'; \
	  echo '*** and it is deliberately NOT graded against whatever the last run'; \
	  echo '*** left on disk.'; \
	  tail -30 ecp5.pnr.log; \
	  rm -f $@ ecp5.report.json; \
	  exit 1; \
	}

.PHONY: ecp5-timing
ecp5-timing: ecp5-timing-toolchain ecp5.config
	@echo
	@echo '== nextpnr-ecp5: the frequency, its corner and its constraint =='
	@python3 soc/ecp5_report.py ecp5.report.json ecp5.config \
	  --clock $(ECP5_CLOCK) --part $(ECP5_PART) --constraint-mhz $(ECP5_TARGET_MHZ)
	@echo
	@echo "Placement, routing and nextpnr's own timing analysis: ecp5.pnr.log"

FTDI_CFLAGS ?= $(shell pkg-config --cflags libftdi1 2>/dev/null || echo -I/opt/homebrew/opt/libftdi/include/libftdi1)
FTDI_LIBS   ?= $(shell pkg-config --libs libftdi1 2>/dev/null || echo -L/opt/homebrew/opt/libftdi/lib -lftdi1)

ftread: soc/ftread.c
	@command -v cc >/dev/null || { echo 'error: no C compiler for the host.' >&2; exit 1; }
	cc -O2 -Wall -o $@ $< $(FTDI_CFLAGS) $(FTDI_LIBS)
	@echo 'built ./ftread -- run it as root: sudo ./ftread 115200 8000'

.PHONY: suite-board
suite-board: ftread
	@echo 'Runs the .S suite on the part, in batches. Needs root for the same'
	@echo 'reason `make prog` does. Roughly ten minutes.'
	@echo
	@sudo ./soc/run_suite_board.sh

DHRY_BOARD_CFLAGS ?= $(DHRY_CFLAGS)

.PHONY: dhrystone-rom
dhrystone-rom:
	@set -e; \
	for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do \
	  if command -v $$candidate >/dev/null 2>&1; then CC=$$candidate; break; fi; \
	done; \
	if [ -z "$$CC" ]; then echo "error: no RISC-V cross compiler; see \`make setup\`." >&2; exit 1; fi; \
	OBJCOPY=$${CC%gcc}objcopy; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/dhry-rom.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	flags='$(DHRY_BOARD_CFLAGS)'; \
	$$CC $$flags -DDHRY_UART=$(DHRY_UART_BASE) \
	  "-DDHRY_FLAGS=\"$$flags\"" \
	  -DDHRY_RUNS=$(DHRY_BOARD_RUNS) \
	  -nostdlib -I test/bench -T test/bench/bench.lds -o "$$tmp/dhry.elf" \
	  test/crt0.S test/bench/dhry_1.c test/bench/dhry_2.c test/bench/dhry_port.c; \
	$$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data \
	  "$$tmp/dhry.elf" "$$tmp/rom.hex"; \
	python3 soc/rom_banks.py "$$tmp/rom.hex" soc/rom_even.hex soc/rom_odd.hex \
	  --rom-words $(SOC_ROM_WORDS)

DHRY_UART_BASE   ?= 0x00020020

DHRY_BOARD_RUNS  ?= 20000

.PHONY: dhrystone-board
dhrystone-board:
	@rm -f board.json board.asc board.bin
	@$(MAKE) --no-print-directory board.bin BOARD_OSC=$(BOARD_OSC) BOARD_ROM=dhrystone-rom
	@echo
	@echo 'Dhrystone is in board.bin. Flash it with `make prog`, then read the'
	@echo 'report off the UART -- it prints itself, cycles and all.'

# Two names, not one target with two bodies: make's last-wins recipe and first-wins `?=`
# would pair one route's flags with the other's script. test/makefile_target_test.sh.
COREMARK_ECP5_UART_BASE  ?= $(DHRY_UART_BASE)
COREMARK_ECP5_CFLAGS     ?= $(COREMARK_CFLAGS)
COREMARK_ECP5_ITERATIONS ?= $(COREMARK_ITERATIONS)

# -O2 does not fit the part's 8 KB ROM at any port size; this is the smallest that does.
COREMARK_UP5K_CFLAGS ?= -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -Os -flto \
                          -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns \
                          -Wall -Wextra -Werror

COREMARK_UP5K_ITERATIONS ?= 800
COREMARK_HZ              ?= 12000000

.PHONY: coremark-pin-check
coremark-pin-check:
	@test/bench/coremark_pin_check.sh $(COREMARK_VENDOR_DIR)

.PHONY: coremark-rom-ecp5
coremark-rom-ecp5: coremark-pin-check
	@set -e; \
	for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do \
	  if command -v $$candidate >/dev/null 2>&1; then CC=$$candidate; break; fi; \
	done; \
	if [ -z "$$CC" ]; then echo "error: no RISC-V cross compiler; see \`make setup\`." >&2; exit 1; fi; \
	OBJCOPY=$${CC%gcc}objcopy; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/coremark-rom.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	flags='$(COREMARK_ECP5_CFLAGS)'; \
	objects=""; \
	for unit in coremark/core_list_join coremark/core_main coremark/core_matrix \
	            coremark/core_state coremark/core_util coremark_port; do \
	  name=$$(basename "$$unit"); \
	  $$CC $$flags -I test/bench -I $(COREMARK_VENDOR_DIR) \
	    -DITERATIONS=$(COREMARK_ECP5_ITERATIONS) "-DCOREMARK_FLAGS=\"$$flags\"" \
	    -DCOREMARK_UART=$(COREMARK_ECP5_UART_BASE) \
	    -c "test/bench/$$unit.c" -o "$$tmp/$$name.o"; \
	  objects="$$objects $$tmp/$$name.o"; \
	done; \
	$$CC $$flags -DCOREMARK_UART=$(COREMARK_ECP5_UART_BASE) -nostdlib \
	  -T test/bench/coremark.lds -o "$$tmp/coremark.elf" \
	  test/crt0.S $$objects; \
	$$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data \
	  "$$tmp/coremark.elf" "$$tmp/rom.hex"; \
	python3 soc/rom_banks.py "$$tmp/rom.hex" soc/rom_even.hex soc/rom_odd.hex \
	  --rom-words $(SOC_ROM_WORDS)

.PHONY: coremark-rom-up5k
coremark-rom-up5k: coremark-pin-check
	@set -e; \
	for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do \
	  if command -v $$candidate >/dev/null 2>&1; then CC=$$candidate; break; fi; \
	done; \
	if [ -z "$$CC" ]; then echo "error: no RISC-V cross compiler; see \`make setup\`." >&2; exit 1; fi; \
	OBJCOPY=$${CC%gcc}objcopy; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/coremark-rom.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	flags='$(COREMARK_UP5K_CFLAGS)'; \
	$$CC $$flags -I test/bench -I $(COREMARK_VENDOR_DIR) \
	  -DITERATIONS=$(COREMARK_UP5K_ITERATIONS) -DCOREMARK_HZ=$(COREMARK_HZ) \
	  -DCOREMARK_UART=$(DHRY_UART_BASE) "-DCOREMARK_FLAGS=\"$$flags\"" \
	  -nostdlib -T test/bench/bench.lds -o "$$tmp/coremark.elf" \
	  test/crt0.S test/bench/coremark/core_list_join.c \
	  test/bench/coremark/core_main.c test/bench/coremark/core_matrix.c \
	  test/bench/coremark/core_state.c test/bench/coremark/core_util.c \
	  test/bench/coremark_port.c; \
	$$OBJCOPY -O verilog --verilog-data-width=4 -j .text -j .data \
	  "$$tmp/coremark.elf" "$$tmp/rom.hex"; \
	python3 soc/rom_banks.py "$$tmp/rom.hex" soc/rom_even.hex soc/rom_odd.hex \
	  --rom-words $(SOC_ROM_WORDS)

.PHONY: coremark-board
coremark-board:
	@rm -f board.json board.asc board.bin
	@$(MAKE) --no-print-directory board.bin BOARD_OSC=$(BOARD_OSC) BOARD_ROM=coremark-rom-up5k
	@echo
	@echo 'CoreMark is in board.bin. Flash it with `make prog`, then read the'
	@echo 'report off the UART -- it prints itself, cycles and all.'

BOARD ?= upduino

BOARD_SRCS := $(SOC_SRCS) soc/miso_share_enable.v soc/board_upduino.v
BOARD_TOP  := upduino_top
BOARD_PCF  := soc/upduino.pcf

BOARD_OSC ?= crystal
BOARD_OSC_PARAM := $(if $(filter internal,$(BOARD_OSC)),1,0)

BOARD_ROM ?= soc-rom

.PHONY: noop-rom
noop-rom:
	@test -s soc/rom_even.hex -a -s soc/rom_odd.hex || { \
	  echo '*** BOARD_ROM=noop-rom, but soc/rom_*.hex are missing or empty.'; \
	  echo '*** Something was meant to write them before this ran.'; \
	  exit 1; \
	}

.PHONY: board-elaborate
board-elaborate: $(BOARD_SRCS) $(BOARD_ROM)
	@./soc/board_elaborate.sh yosys $(BOARD_TOP) $(BOARD_SRCS)

board.json: $(BOARD_SRCS) $(BOARD_ROM)
	@echo 'yosys: synthesising $(BOARD_TOP) for ice40 (log: board.synth.log)'
	@yosys -p 'read_verilog -sv $(BOARD_SRCS); \
	   chparam -set INTERNAL_OSC $(BOARD_OSC_PARAM) $(BOARD_TOP); \
	   synth_ice40 -device u -dsp -spram -top $(BOARD_TOP) -json $@' \
	  > board.synth.log 2>&1 || { tail -40 board.synth.log; exit 1; }
	@python3 soc/cell_census.py board.synth.log SB_SPRAM256KA $(SOC_EXPECT_SPRAM) \
	  "the board wrapper changed how rtl/memory.v maps -- the SoC underneath it is the same design"

board.asc: board.json $(BOARD_PCF)
	@echo 'nextpnr: placing $(BOARD_TOP) on up5k/sg48 (log: board.pnr.log)'
	@nextpnr-ice40 --up5k --package sg48 --pcf $(BOARD_PCF) --json $< \
	  --asc $@ > board.pnr.log 2>&1 || true
	@test -s $@ || { \
	  echo '*** make bitstream: nextpnr wrote no .asc, so there is nothing to'; \
	  echo '*** pack. That is a failed placement, not a slow design.'; \
	  tail -30 board.pnr.log; \
	  rm -f $@; \
	  exit 1; \
	}

board.bin: board.asc
	@icepack $< $@
	@echo "board.bin: $$(wc -c < $@ | tr -d ' ') bytes for $(BOARD), clock $(BOARD_OSC)"

.PHONY: bitstream
bitstream: board.bin
	@echo
	@echo "== $(BOARD): a bitstream, not a measurement =="
	@# No `-p`: icetime's pcf parser takes exactly two arguments per line and
	@# rejects the `-nowarn` this board's file needs, which nextpnr requires so
	@# that an unused clock pin under BOARD_OSC=internal is not an error. The
	@# flag only teaches icetime the IO net names, and nothing here reads them.
	@icetime -d up5k -P sg48 -t board.asc 2>&1 | tail -3
	@echo
	@echo 'This says what the TOOLS think the placement does. A board is the only'
	@echo 'thing that can disagree, and none has run this yet.'

ICEPROG_DEV  ?=
ICEPROG_SUDO ?= $(if $(filter Darwin,$(shell uname -s)),sudo,)
.PHONY: prog
prog: board.bin
	@command -v iceprog >/dev/null || { \
	  echo '*** iceprog is not on PATH. It ships with the OSS CAD Suite that'; \
	  echo '*** `make setup` caches -- put its bin/ first on PATH.'; \
	  exit 1; \
	}
	@echo 'Flashing $(BOARD). On macOS this needs root -- see the comment above.'
	$(ICEPROG_SUDO) iceprog $(if $(ICEPROG_DEV),-d '$(ICEPROG_DEV)') board.bin

DUAL_SRCS := $(DUAL_RTL_SRCS) rtl/littledualsoc.v

DUAL_EXPECT_DP16KD := 40
DUAL_EXPECT_LUTRAM := 64
DUAL_EXPECT_DSP    := 8

dual_ecp5.json: $(DUAL_SRCS) soc-rom
	@echo 'yosys: synthesising littledualsoc for ECP5 (log: dual_ecp5.synth.log)'
	@yosys -p 'read_verilog -sv $(DUAL_SRCS); synth_ecp5 -top littledualsoc -json $@' \
	  > dual_ecp5.synth.log 2>&1 || { tail -40 dual_ecp5.synth.log; exit 1; }
	@python3 soc/cell_census.py dual_ecp5.synth.log DP16KD $(DUAL_EXPECT_DP16KD) \
	  "two fetch windows are two copies of the banked ROM and one data RAM; a count that stopped doubling means the second window stopped being its own storage" \
	  --gate 'make dual-ecp5-timing' --declared DUAL_EXPECT_DP16KD
	@python3 soc/cell_census.py dual_ecp5.synth.log TRELLIS_DPR16X4 $(DUAL_EXPECT_LUTRAM) \
	  "one register file per hart as distributed RAM; zero means it fell into flops and soft muxes, half means one core did" \
	  --gate 'make dual-ecp5-timing' --declared DUAL_EXPECT_LUTRAM
	@python3 soc/cell_census.py dual_ecp5.synth.log MULT18X18D $(DUAL_EXPECT_DSP) \
	  "one multiplier per hart; in soft logic either would be invisible in a frequency number and enormous in area" \
	  --gate 'make dual-ecp5-timing' --declared DUAL_EXPECT_DSP
	@python3 soc/bram_reset_check.py $@ --gate 'make dual-ecp5-timing'

dual_ecp5.config: dual_ecp5.json soc/littlesoc.lpf
	@rm -f $@ dual_ecp5.report.json
	@echo 'nextpnr: placing and routing littledualsoc on $(ECP5_PART) (log: dual_ecp5.pnr.log)'
	@nextpnr-ecp5 $(ECP5_DEVICE) --package $(ECP5_PACKAGE) --speed $(ECP5_SPEED) \
	  --json $< --lpf soc/littlesoc.lpf --lpf-allow-unconstrained \
	  --freq $(ECP5_TARGET_MHZ) $(if $(ECP5_SEED),--seed '$(ECP5_SEED)') \
	  --textcfg $@ --report dual_ecp5.report.json > dual_ecp5.pnr.log 2>&1 || true
	@{ test -s $@ && test -s dual_ecp5.report.json; } || { \
	  echo '*** make dual-ecp5-timing: nextpnr wrote no configuration and report'; \
	  echo '*** pair, so NOTHING was measured. That is a failed run, not a slow'; \
	  echo '*** design, and it is deliberately NOT graded against whatever the'; \
	  echo '*** last run left on disk.'; \
	  tail -30 dual_ecp5.pnr.log; \
	  rm -f $@ dual_ecp5.report.json; \
	  exit 1; \
	}

.PHONY: dual-ecp5-timing
dual-ecp5-timing: ecp5-timing-toolchain dual_ecp5.config
	@echo
	@echo '== nextpnr-ecp5: the DUAL frequency, its corner and its constraint =='
	@python3 soc/ecp5_report.py dual_ecp5.report.json dual_ecp5.config \
	  --clock $(ECP5_CLOCK) --part $(ECP5_PART) --constraint-mhz $(ECP5_TARGET_MHZ)
	@echo
	@echo "Placement, routing and nextpnr's own timing analysis: dual_ecp5.pnr.log"

print-%:
	@echo '$($*)'

TOOLS ?= $(sort $(FIT_TOOLS) $(SOC_TIMING_TOOLS) $(ECP5_TOOLS))

.PHONY: print-toolchain
print-toolchain:
	@soc/print_toolchain.sh $(TOOLS)

.PHONY: doctor
doctor:
	@set -e; \
	for candidate in riscv64-elf-gcc riscv64-unknown-elf-gcc; do \
	  if command -v $$candidate >/dev/null 2>&1; then CC=$$candidate; break; fi; \
	done; \
	if [ -z "$$CC" ]; then \
	  echo "error: no RISC-V cross compiler found; see \`make setup\`." >&2; exit 1; \
	fi; \
	soc/print_toolchain.sh "$$CC" $(SOC_TIMING_TOOLS)

NETLIST_PART ?= up5k
NETLIST_OUT  ?= netlist.out

ifeq ($(NETLIST_PART),up5k)
NETLIST_ROM     := soc-rom
NETLIST_SYNTH   := $(SOC_SYNTH)
NETLIST_PNR     := $(SOC_PNR)
NETLIST_PNR_OUT := --asc
NETLIST_MUTANT  := rtl/littlesoc.v mem_addr
endif

NETLIST_ENV = NETLIST_SYNTH='$(NETLIST_SYNTH)' NETLIST_PNR='$(NETLIST_PNR)' \
              NETLIST_PNR_OUT='$(NETLIST_PNR_OUT)' NETLIST_PNR_DONE='$(NETLIST_PNR_DONE)' \
              NETLIST_MUTANT='$(NETLIST_MUTANT)' NETLIST_OUT='$(NETLIST_OUT)' \
              SOC_PROG='$(SOC_PROG)'

define netlist-part-check
test -n '$(NETLIST_SYNTH)' || { \
	  echo '*** NETLIST_PART=$(NETLIST_PART) has no synthesis flow here.'; \
	  echo '*** Parts with one: up5k. A new part needs NETLIST_ROM, NETLIST_SYNTH,'; \
	  echo '*** NETLIST_PNR, NETLIST_PNR_OUT and NETLIST_MUTANT set in the Makefile'; \
	  echo '*** block above. Nothing was digested.'; \
	  exit 2; }
endef

.PHONY: netlist-determinism
netlist-determinism: $(NETLIST_ROM)
	@$(netlist-part-check)
	@$(NETLIST_ENV) sh soc/netlist_determinism.sh

.PHONY: netlist-digest
netlist-digest: netlist-determinism
	@$(netlist-part-check)
	@echo
	@python3 soc/netlist_digest.py digest $(NETLIST_OUT)/this.canon.json \
	  --label '$(NETLIST_PART), $(SOC_PROG)'

.PHONY: netlist-diff
netlist-diff: export BASE := $(BASE)
netlist-diff: netlist-determinism
	@$(netlist-part-check)
	@test -n "$$BASE" || { \
	  echo '*** make netlist-diff: name the commit to compare against, e.g.'; \
	  echo '*** make netlist-diff BASE=origin/main.'; \
	  exit 2; }
	@$(NETLIST_ENV) sh soc/netlist_base.sh "$$BASE" $(NETLIST_OUT)/base.canon.json
	@echo
	@python3 soc/netlist_digest.py compare \
	  $(NETLIST_OUT)/base.canon.json $(NETLIST_OUT)/this.canon.json \
	  --base-label "$$BASE" --new-label 'this tree'

COMPARE_CORE  ?= littlecpu
COMPARE_SEED  ?=
COMPARE_ROM_WORDS := 1024
COMPARE_RAM_WORDS := 16384

COMPARE_PART ?= up5k

ifeq ($(COMPARE_PART),up5k)
COMPARE_PNR_FLAGS   := --up5k --package sg48
COMPARE_PCF         := soc/compare/bench_up5k.pcf
COMPARE_SYNTH_FLAGS := -device u -dsp -spram
COMPARE_ICETIME_ARG := -d up5k -P sg48
else ifeq ($(COMPARE_PART),hx8k)
COMPARE_PNR_FLAGS   := --hx8k --package ct256
COMPARE_PCF         := soc/compare/bench_hx8k.pcf
COMPARE_SYNTH_FLAGS :=
COMPARE_ICETIME_ARG := -d hx8k -P ct256
else
$(error COMPARE_PART is '$(COMPARE_PART)'; this harness knows up5k and hx8k)
endif
COMPARE_MIN_RATIO := 0.8

HAZARD3_HDL  := $(HAZARD3_DIR)/hdl
HAZARD3_SRCS := $(HAZARD3_HDL)/hazard3_core.v $(HAZARD3_HDL)/hazard3_cpu_2port.v \
                $(HAZARD3_HDL)/arith/hazard3_alu.v \
                $(HAZARD3_HDL)/arith/hazard3_branchcmp.v \
                $(HAZARD3_HDL)/arith/hazard3_mul_fast.v \
                $(HAZARD3_HDL)/arith/hazard3_muldiv_seq.v \
                $(HAZARD3_HDL)/arith/hazard3_onehot_encode.v \
                $(HAZARD3_HDL)/arith/hazard3_onehot_priority.v \
                $(HAZARD3_HDL)/arith/hazard3_onehot_priority_dynamic.v \
                $(HAZARD3_HDL)/arith/hazard3_priority_encode.v \
                $(HAZARD3_HDL)/arith/hazard3_shift_barrel.v \
                $(HAZARD3_HDL)/hazard3_csr.v $(HAZARD3_HDL)/hazard3_decode.v \
                $(HAZARD3_HDL)/hazard3_frontend.v \
                $(HAZARD3_HDL)/hazard3_instr_decompress.v \
                $(HAZARD3_HDL)/hazard3_irq_ctrl.v $(HAZARD3_HDL)/hazard3_pmp.v \
                $(HAZARD3_HDL)/hazard3_power_ctrl.v \
                $(HAZARD3_HDL)/hazard3_regfile_1w2r.v $(HAZARD3_HDL)/hazard3_triggers.v

include soc/compare/vexriscv_pin.mk

ifeq ($(COMPARE_CORE),vexriscv)
COMPARE_TOP  := bench_vexriscv
COMPARE_SRCS := soc/compare/bench_vexriscv.v rtl/memory.v
COMPARE_READ := read_verilog $(VEXRISCV_V); \
                read_verilog -sv $(COMPARE_SRCS)
COMPARE_CORE_READ := read_verilog $(VEXRISCV_V); \
                     hierarchy -top VexRiscv; delete -port VexRiscv/rvfi_*
COMPARE_CORE_TOP  := VexRiscv
COMPARE_DEPS      := $(COMPARE_SRCS) $(VEXRISCV_V) vexriscv-pin-check
COMPARE_CORE_DEPS := $(VEXRISCV_V) vexriscv-pin-check
else ifeq ($(COMPARE_CORE),hazard3)
COMPARE_TOP  := bench_hazard3
COMPARE_SRCS := $(HAZARD3_SRCS) rtl/memory.v soc/compare/bench_hazard3.v
COMPARE_READ := read_verilog -sv -I $(HAZARD3_HDL) $(COMPARE_SRCS)
COMPARE_CORE_READ := read_verilog -sv -I $(HAZARD3_HDL) $(HAZARD3_SRCS); \
                     hierarchy -top hazard3_cpu_2port
COMPARE_CORE_TOP  := hazard3_cpu_2port
COMPARE_DEPS      := $(COMPARE_SRCS) | $(HAZARD3_DIR)
COMPARE_CORE_DEPS := $(HAZARD3_SRCS) | $(HAZARD3_DIR)
else
COMPARE_TOP  := bench_littlecpu
COMPARE_SRCS := $(FIT_SRCS) rtl/imemory.v rtl/memory.v \
                soc/compare/bench_littlecpu.v
COMPARE_READ := read_verilog -sv $(COMPARE_SRCS)
COMPARE_CORE_READ := read_verilog -sv $(FIT_SRCS); \
                     hierarchy -top littlecpu; delete -port littlecpu/rvfi_*
COMPARE_CORE_TOP  := littlecpu
COMPARE_DEPS      := $(COMPARE_SRCS)
COMPARE_CORE_DEPS := $(FIT_SRCS)
endif

.PHONY: compare-rom
compare-rom: compare-geometry-test
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
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/compare-rom.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
	$$CC -march=rv32i -mabi=ilp32 -nostdlib -T soc/compare/bench.lds \
	  -o "$$tmp/bench.elf" soc/compare/bench.S; \
	$$OBJCOPY -O verilog --verilog-data-width=4 -j .text "$$tmp/bench.elf" "$$tmp/bench.hex"; \
	python3 soc/rom_banks.py "$$tmp/bench.hex" \
	  soc/compare/rom_even.hex soc/compare/rom_odd.hex --rom-words $(COMPARE_ROM_WORDS); \
	python3 soc/compare/rom_flat.py "$$tmp/bench.hex" \
	  soc/compare/rom_flat.hex --rom-words $(COMPARE_ROM_WORDS)

compare.$(COMPARE_CORE).core.log: $(COMPARE_CORE_DEPS)
	@echo 'yosys: synthesising $(COMPARE_CORE_TOP) alone for hx8k (log: $@)'
	@yosys -p '$(COMPARE_CORE_READ); synth_ice40 $(COMPARE_SYNTH_FLAGS) -top $(COMPARE_CORE_TOP); stat' \
	  > $@ 2>&1 || { tail -40 $@; exit 1; }

compare.$(COMPARE_CORE).json: compare-rom $(COMPARE_DEPS)
	@echo 'yosys: synthesising $(COMPARE_TOP) for $(COMPARE_PART) (log: compare.$(COMPARE_CORE).synth.log)'
	@# The synthesis flags come from the part table above. hx8k gets none: it has
	@# no SB_MAC16 and no SPRAM, so the multiplier is soft logic there and the
	@# 64 KB data RAM will not fit at all -- which is why up5k is the default.
	@# chparam BEFORE hierarchy, so the harness's geometry has one source -- the
	@# variables above -- rather than a second copy in each .v file's defaults.
	@yosys -p '$(COMPARE_READ); \
	  chparam -set ROM_WORDS $(COMPARE_ROM_WORDS) -set RAM_WORDS $(COMPARE_RAM_WORDS) $(COMPARE_TOP); \
	  hierarchy -top $(COMPARE_TOP); \
	  synth_ice40 $(COMPARE_SYNTH_FLAGS) -top $(COMPARE_TOP) -json $@; stat' \
	  > compare.$(COMPARE_CORE).synth.log 2>&1 \
	  || { tail -40 compare.$(COMPARE_CORE).synth.log; exit 1; }

compare.$(COMPARE_CORE).asc: compare.$(COMPARE_CORE).json $(COMPARE_PCF)
	@echo 'nextpnr: placing $(COMPARE_TOP) on $(COMPARE_PART) (log: compare.$(COMPARE_CORE).pnr.log)'
	@nextpnr-ice40 $(COMPARE_PNR_FLAGS) --json $< --pcf $(COMPARE_PCF) \
	  $(if $(COMPARE_SEED),--seed '$(COMPARE_SEED)') --asc $@ \
	  > compare.$(COMPARE_CORE).pnr.log 2>&1 || true
	@test -s $@ || { \
	  echo '*** make compare-timing: nextpnr produced no bitstream, so NOTHING'; \
	  echo '*** was measured. That is a failed placement, not a fast design.'; \
	  tail -30 compare.$(COMPARE_CORE).pnr.log; \
	  rm -f $@; \
	  exit 1; \
	}

COMPARE_SMOKE_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                      soc/compare/bench_vexriscv.v soc/compare/bench_hazard3.v \
                      soc/compare/bench_tb.v

compare.vvp: $(COMPARE_SMOKE_SRCS) compare-rom $(VEXRISCV_V) vexriscv-pin-check \
             | $(HAZARD3_DIR)
	iverilog -I./rtl/ -I$(HAZARD3_HDL) -g2012 -o $@ \
	  $(VEXRISCV_V) $(HAZARD3_SRCS) \
	  $(COMPARE_SMOKE_SRCS)

.PHONY: compare-smoke
compare-smoke: compare.vvp
	@vvp $<

COMPARE_DHRY_RUNS   ?= 400
COMPARE_DHRY_CYCLES ?= 2000000
COMPARE_DHRY_CFLAGS := -march=rv32im -mabi=ilp32 -O2 -std=c11 \
                       -ffreestanding -fno-tree-loop-distribute-patterns \
                       -Wall -Wextra -Werror

COMPARE_DHRY_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                     soc/compare/bench_vexriscv.v soc/compare/bench_hazard3.v \
                     soc/compare/dhry_monitor.v soc/compare/dhry_tb.v

compare.dhry.vvp: $(COMPARE_DHRY_SRCS) $(VEXRISCV_V) vexriscv-pin-check \
                  | $(HAZARD3_DIR)
	iverilog -I./rtl/ -I$(HAZARD3_HDL) -g2012 -o $@ \
	  $(VEXRISCV_V) $(HAZARD3_SRCS) \
	  $(COMPARE_DHRY_SRCS)

COMPARE_DHRY_SOLO_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                          soc/compare/dhry_monitor.v soc/compare/dhry_solo_tb.v

compare.dhry.solo.vvp: $(COMPARE_DHRY_SOLO_SRCS)
	iverilog -I./rtl/ -g2012 -o $@ $(COMPARE_DHRY_SOLO_SRCS)

.PHONY: compare-dhrystone
compare-dhrystone: compare.dhry.vvp compare.dhry.solo.vvp
	@$(MAKE) --no-print-directory COMPARE_CORE=littlecpu compare.littlecpu.core.log
	@$(MAKE) --no-print-directory COMPARE_CORE=vexriscv compare.vexriscv.core.log
	@$(MAKE) --no-print-directory COMPARE_CORE=hazard3 compare.hazard3.core.log
	@echo '== the three-way row: littlecpu, VexRiscv and Hazard3, all at RV32IM =='
	@./soc/compare/run_dhrystone.sh $(COMPARE_DHRY_RUNS) $(COMPARE_DHRY_CYCLES) \
	  '$(COMPARE_DHRY_CFLAGS)' hardware compare.dhry.vvp littlecpu,vexriscv,hazard3 \
	  littlecpu=compare.littlecpu.core.log vexriscv=compare.vexriscv.core.log \
	  hazard3=compare.hazard3.core.log
	@echo
	@echo '== the ISA-cost row: littlecpu alone, at its native ISA =='
	@./soc/compare/run_dhrystone.sh $(COMPARE_DHRY_RUNS) $(COMPARE_DHRY_CYCLES) \
	  '$(DHRY_CFLAGS)' hardware compare.dhry.solo.vvp littlecpu \
	  littlecpu=compare.littlecpu.core.log
	@if [ -f soc/compare/product.json ]; then \
	  echo '== the stamped cross-core product, if the stamp still matches this tree =='; \
	  python3 soc/compare/product_check.py soc/compare/product.json dhrystone \
	    --repo . --current 'cflags=$(COMPARE_DHRY_CFLAGS)' \
	    --current 'rom_words=$(COMPARE_ROM_WORDS)' \
	    --current 'ram_words=$(COMPARE_RAM_WORDS)' || true; \
	else \
	  echo 'no soc/compare/product.json yet -- `make compare-product` stamps one'; \
	fi

.PHONY: compare-product
compare-product:
	@./soc/compare/run_product.sh

COMPARE_COREMARK_ITERATIONS ?= 1
COMPARE_COREMARK_CYCLES     ?= 200000000
COMPARE_COREMARK_CFLAGS := -march=rv32im -mabi=ilp32 -O2 -std=c11 \
                           -ffreestanding -fno-tree-loop-distribute-patterns \
                           -Wall -Wextra -Werror

COMPARE_COREMARK_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                         soc/compare/bench_vexriscv.v soc/compare/bench_hazard3.v \
                         soc/compare/dhry_monitor.v soc/compare/coremark_tb.v

compare.coremark.vvp: $(COMPARE_COREMARK_SRCS) $(VEXRISCV_V) vexriscv-pin-check \
                       | $(HAZARD3_DIR)
	iverilog -I./rtl/ -I$(HAZARD3_HDL) -g2012 -o $@ \
	  $(VEXRISCV_V) $(HAZARD3_SRCS) \
	  $(COMPARE_COREMARK_SRCS)

.PHONY: compare-coremark
compare-coremark: compare.coremark.vvp
	@$(MAKE) --no-print-directory COMPARE_CORE=littlecpu compare.littlecpu.core.log
	@$(MAKE) --no-print-directory COMPARE_CORE=vexriscv compare.vexriscv.core.log
	@$(MAKE) --no-print-directory COMPARE_CORE=hazard3 compare.hazard3.core.log
	@./soc/compare/run_coremark_compare.sh $(COMPARE_COREMARK_ITERATIONS) \
	  $(COMPARE_COREMARK_CYCLES) '$(COMPARE_COREMARK_CFLAGS)' compare.coremark.vvp

.PHONY: compare-timing
compare_ecp5.$(COMPARE_CORE).json: compare-rom $(COMPARE_DEPS)
	@echo 'yosys: synthesising $(COMPARE_TOP) for ECP5 (log: compare_ecp5.$(COMPARE_CORE).synth.log)'
	@yosys -p '$(COMPARE_READ); \
	  chparam -set ROM_WORDS $(COMPARE_ROM_WORDS) -set RAM_WORDS $(COMPARE_RAM_WORDS) $(COMPARE_TOP); \
	  synth_ecp5 -top $(COMPARE_TOP) -json $@; stat' \
	  > compare_ecp5.$(COMPARE_CORE).synth.log 2>&1 \
	  || { tail -40 compare_ecp5.$(COMPARE_CORE).synth.log; exit 1; }

compare_ecp5.$(COMPARE_CORE).config: compare_ecp5.$(COMPARE_CORE).json soc/compare/bench_ecp5.lpf
	@rm -f $@ compare_ecp5.$(COMPARE_CORE).report.json
	@echo 'nextpnr: placing $(COMPARE_TOP) on $(ECP5_PART) (log: compare_ecp5.$(COMPARE_CORE).pnr.log)'
	@nextpnr-ecp5 $(ECP5_DEVICE) --package $(ECP5_PACKAGE) --speed $(ECP5_SPEED) \
	  --json $< --lpf soc/compare/bench_ecp5.lpf --lpf-allow-unconstrained \
	  --freq $(ECP5_TARGET_MHZ) $(if $(ECP5_SEED),--seed '$(ECP5_SEED)') \
	  --textcfg $@ --report compare_ecp5.$(COMPARE_CORE).report.json \
	  > compare_ecp5.$(COMPARE_CORE).pnr.log 2>&1 || true
	@{ test -s $@ && test -s compare_ecp5.$(COMPARE_CORE).report.json; } || { \
	  echo '*** make compare-ecp5-timing: nextpnr wrote no configuration and'; \
	  echo '*** report pair, so NOTHING was measured. That is a failed run, not'; \
	  echo '*** a slow design, and it is deliberately NOT graded against whatever'; \
	  echo '*** the last run left on disk.'; \
	  tail -30 compare_ecp5.$(COMPARE_CORE).pnr.log; \
	  rm -f $@ compare_ecp5.$(COMPARE_CORE).report.json; \
	  exit 1; \
	}

.PHONY: compare-ecp5-timing
compare-ecp5-timing: compare_ecp5.$(COMPARE_CORE).config
	@echo
	@echo '== nextpnr-ecp5: $(COMPARE_CORE) on $(ECP5_PART) =='
	@python3 soc/ecp5_report.py compare_ecp5.$(COMPARE_CORE).report.json \
	  compare_ecp5.$(COMPARE_CORE).config --clock clk --part $(ECP5_PART) \
	  --constraint-mhz $(ECP5_TARGET_MHZ)

compare-timing: compare.$(COMPARE_CORE).asc compare.$(COMPARE_CORE).core.log
	@sed -n '/^Info: Device utilisation:/,/^$$/s/^Info: //p' compare.$(COMPARE_CORE).pnr.log
	@echo
	@echo '== is the core still there? =='
	@python3 soc/compare/placed_vs_synth.py compare.$(COMPARE_CORE).pnr.log \
	  compare.$(COMPARE_CORE).core.log $(COMPARE_CORE) --min-ratio $(COMPARE_MIN_RATIO)
	@echo
	@echo '== icetime: the critical path, and the LOGIC/ROUTING SPLIT =='
	@icetime $(COMPARE_ICETIME_ARG) -p $(COMPARE_PCF) -t \
	  -r compare.$(COMPARE_CORE).timing.rpt compare.$(COMPARE_CORE).asc \
	  > compare.$(COMPARE_CORE).icetime.log 2>&1 \
	  || { cat compare.$(COMPARE_CORE).icetime.log; exit 1; }
	@echo
	@echo 'THIS IS NOT `make soc-timing`, AND NOT A LIKE-FOR-LIKE COMPARISON.'
	@echo 'Different part, smaller memories, no timer, and the cores implement'
	@echo 'different ISAs: RV32IMAC+Zicsr with traps here, RV32IC with no CSR'
	@echo 'file and no traps for VexRiscv, RV32IMA with no C and no counters'
	@echo 'for Hazard3. Quote the ISA and the geometry with the number. One'
	@echo 'placement is a sample: soc/compare/sweep.sh runs four each.'
	@python3 soc/timing_split.py compare.$(COMPARE_CORE).timing.rpt

clean:
	rm -f fit.json fit.log fit.synth.log
	rm -f soc.json soc.asc soc.synth.log soc.pnr.log soc.timing.rpt
	rm -f board.json board.asc board.bin board.synth.log board.pnr.log
	rm -f ecp5.json ecp5.config ecp5.report.json ecp5.synth.log ecp5.pnr.log
	rm -f dual_ecp5.json dual_ecp5.config dual_ecp5.report.json dual_ecp5.synth.log dual_ecp5.pnr.log
	rm -f test/dual_rtl.cc dual-sim
	rm -f soc/rom_even.hex soc/rom_odd.hex
	rm -f compare.*.json compare.*.asc compare.*.log compare.*.rpt compare.vvp
	rm -f compare.dhry.vvp compare.dhry.solo.vvp compare.coremark.vvp
	rm -f soc/compare/rom_even.hex soc/compare/rom_odd.hex soc/compare/rom_flat.hex
	rm -f soc/compare/dhry_even.hex soc/compare/dhry_odd.hex soc/compare/dhry_flat.hex
	rm -f soc/compare/dhry_ram.hex
	rm -f soc/compare/coremark_even.hex soc/compare/coremark_odd.hex
	rm -f soc/compare/coremark_flat.hex soc/compare/coremark_ram.hex
	rm -f waves.vcd
	rm -rf sim sim.dSYM
	rm -rf cosim cosim.dSYM
	rm -f testbench.vvp testbench.vcd
	rm -f test/rtl.cc
	rm -f test/monitor.sim.v
	rm -f rvfi_macros.vh
	@# NOT rtl/rom.mem: gitignored, untracked, and nothing regenerates real
	@# contents for it, so `clean` deleting it is unrecoverable data loss.
	@# NOT $(SAIL_RISCV_DIR) either: a multi-megabyte network fetch outside the
	@# checkout that nothing on `make test`'s path needs, so blowing it away on
	@# every `clean` costs a download to get back something `clean` was never
	@# asked to rebuild. Bumping SAIL_RISCV_VERSION and re-running
	@# `make sail-setup` re-fetches on its own -- the pin is recorded in
	@# $(SAIL_STAMP) and compared -- and `rm -rf` of the directory by hand is
	@# the blunt instrument.
	@# NOT $(SVLINT_DIR) either, and for the same reason: a network fetch that
	@# `clean` was never asked to rebuild. `make lint-setup` re-fetches
	@# unconditionally, so `rm -rf` of it by hand is the blunt instrument there.
