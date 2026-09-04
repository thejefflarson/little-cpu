include formal/pin.mk
include soc/compare/hazard3_pin.mk

.DELETE_ON_ERROR:

RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM RISCV_FORMAL_MEM_FAULT RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32

rvfi_macros.vh: $(RISCV_FORMAL_DIR)/checks/rvfi_macros.py
	python3 $^ > $@

# Both sim legs build from this list. Do not copy it anywhere else — a second
# copy goes stale and the gate then checks a different design than it says it
# does. The CI job had one; it missed rtl/imemory.v and rtl/memory.v when they
# landed, and spent a run elaborating a testbench whose memories were not there.
# That job calls `make elaborate-strict` now, so there is one list to update.
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

sim: test/cxxrtl.cc test/rtl.cc
	clang++ -O2 -DNDEBUG -std=c++17 -Wall -Wextra -Werror \
	  -isystem $$(yosys-config --datdir)/include/backends/cxxrtl/runtime $< -o $@

# Outside the checkout: a worktree gets tracked files only, so tools installed
# inside it are invisible from every other worktree.
TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

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

# Scoped to the goals that run the binary -- an unscoped check would break
# `make test` for everyone on a stale local cache.
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

# Both sim legs check every retire against this file; edit
# test/sanitize_monitor.py, not this rule, or the change is silent.
test/monitor.sim.v: test/monitor.v test/sanitize_monitor.py
	python3 test/sanitize_monitor.py $< > $@

test/rtl.cc: $(SIM_RTL_SRCS) rvfi_macros.vh $(SIM_TB_SRCS) test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top testbench; write_cxxrtl $@'

# ---- the dual configuration ------------------------------------------------
# A separate harness, not a configuration axis: two monitor instances roughly
# double a 7000-line generated module, so none of this is on `make test`'s path.
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

# Keep this yosys -p script on one line: a backslash split inside its single
# quotes stays literal, and yosys dies on it on CI's make though not on macOS's.
.PHONY: elaborate-strict
elaborate-strict: $(SIM_RTL_SRCS) $(SIM_TB_SRCS)
	yosys -p 'read_verilog -sv $(SIM_RTL_SRCS) $(SIM_TB_SRCS); hierarchy -top testbench; proc; opt_clean; check; write_cxxrtl /tmp/elaborate-strict.cc'

MONITOR_GEN = cd $(RISCV_FORMAL_DIR)/monitor && python3 generate.py -i rv32imc -c 1 -a -p monitor

# Order-only (after `|`): a normal prerequisite goes stale on any write inside
# that directory and rewrites this checked-in file into someone else's commit.
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

# Names omit $(SVLINT_VERSION) on purpose: bumping the version without adding
# digests then makes the lookup empty, and lint-setup refuses rather than fetches.
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

# `present` reads the directory in the recipe, not via $(wildcard) -- make
# caches that and a stale listing could miss a bench that is really there.
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
	@./test/tool_cache_test.sh '$(SAIL_RISCV_DIR)' '$(SVLINT_DIR)' '$(SAIL_DOWNLOAD_DIR)'

.PHONY: memmap-test
memmap-test:
	@./test/memmap_test.sh

# Asserts that every rtl/*.v file has a ruling on whether a mutation of it is
# caught by anything -- a named mutation, or `unpaired` and a real bench or
# formal task -- checked against `ls rtl/*.v` both ways round. Hangs off
# `test` like the other bash checks -- grep, sed and comm, no cross compiler,
# no simulator, no yosys -- so the answer arrives before CI rather than from
# `make mutation-check`, which is required but off this path.
.PHONY: mutation-coverage-test
mutation-coverage-test:
	@./test/mutation_coverage_test.sh

# Asserts that every ADR file has a unique number and exactly one row in
# docs/adr/README.md, both ways round. Hangs off `test` like the other bash
# checks -- ls, grep and sed, no cross compiler, no simulator, no yosys.
# Catches the failure a README-row conflict does not: two files claiming the
# same reserved number under two different filenames merge with no conflict
# at all, because only one of them needs a row.
.PHONY: adr-numbering-test
adr-numbering-test:
	@./test/adr_numbering_test.sh

# The cross-core comparison harness states its geometry in several places, read
# from this Makefile's own COMPARE_TOP/-T lines rather than a second hand-kept
# list, and this is what says they agree. Hangs off `test` like the other bash
# checks -- grep and sed only -- because the harness itself needs yosys,
# nextpnr and the pinned clone, so nothing else on this machine would notice
# it rotting.
.PHONY: compare-geometry-test
compare-geometry-test:
	@./soc/compare/geometry_test.sh

.PHONY: port-connect-test
port-connect-test:
	@python3 ./test/port_connect_test.py

.PHONY: retired-term-test
retired-term-test:
	@./test/retired_term_test.sh

# The ISA string is stated at seven sites and three of them build programs that
# use no atomic, so a site left behind goes on producing numbers rather than
# failing to assemble. Hangs off `test` like the other bash checks -- git, grep,
# sed and awk only -- because the two sites it would otherwise take a Dhrystone
# run and an SoC place-and-route to notice are exactly the silent ones.
.PHONY: march-test
march-test:
	@./test/march_test.sh

# A `.gitignore` rule never applies to a file git already tracks, so a tracked
# file matching one is always a mistake -- a dead rule, or a commit that should
# not have happened. `git ls-files | git check-ignore --stdin --no-index -v` is
# the query that finds the class; nothing ran it, and five nextpnr build
# artifacts sat tracked for as long as their own .gitignore lines did nothing.
# Hangs off `test` like the other bash checks -- git only -- for the same
# reason `retired-term-test` does: the mistake is a commit, and no review of
# the commit that adds the ignore rule can see that tracking predates it.
.PHONY: tracked-ignored-test
tracked-ignored-test:
	@./test/tracked_ignored_test.sh

.PHONY: band-source-test
band-source-test:
	@python3 ./test/band_source_test.py

.PHONY: zkt-isolation-test
zkt-isolation-test:
	@python3 ./test/zkt_isolation_test.py

# Forces the elaboration checks in rtl/{imemory,memory,timer,uart,spiflash}.v
# and rtl/littlecpu.v's copy of that map to fire, in both frontends. Hangs off
# `test` because the parameter shapes they guard are the ones the SoC and the
# benches pass, so nothing else here would notice a check that had stopped
# checking. It needs iverilog and yosys, which `sim` and `test-units` already
# require.
.PHONY: window-test
window-test:
	@./test/window_test.sh

.PHONY: imem-share-test
imem-share-test:
	@./test/imem_share_test.sh

.PHONY: abc-engine-test
abc-engine-test:
	@./formal/test-abc-engine.sh

.PHONY: mutation-check
mutation-check:
	@./test/mutation_check.sh

.PHONY: mutation-probe
mutation-probe:
	@./test/mutation_probe.sh

# The two-hart programs. Only one of them runs (`make dual-smoke`, off `test`'s
# path): this assembles and links every one and checks it against the pairing
# that claims it catches something, in both directions, so the four the runner
# does not yet grade cannot rot silently, and the pairings cannot rot with
# them. It needs the same cross compiler `make test` already needs and no
# simulator, so it runs wherever the suite runs.
.PHONY: dual-build
dual-build:
	@./test/dual_build.sh test/dual test/asm test/dual/MUTATION_PAIRINGS

.PHONY: test
test: sim test-units probe-gates pin-bump-test tool-cache-test memmap-test \
      adr-numbering-test compare-geometry-test retired-term-test port-connect-test march-test \
      band-source-test zkt-isolation-test window-test imem-share-test \
      abc-engine-test mutation-probe dual-build board-elaborate \
      tracked-ignored-test mutation-coverage-test
	@./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

.PHONY: cycles
cycles: sim
	@STALL_REPORT=1 ./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

# Dhrystone 2.1, the one number this core can be quoted against other cores'.
# Not a prerequisite of anything and not on CI. 2000 runs: test/crt0.S's
# Arr_2_Glob zeroing loop is under 3% of accounted cycles by then. Flags fixed
# for comparability with the cores in the comparison set.
DHRY_RUNS   ?= 2000
DHRY_CYCLES ?= 4000000
DHRY_CFLAGS := -march=rv32imac_zicsr_zifencei_zkt -mabi=ilp32 -O2 -std=c11 \
               -ffreestanding -fno-tree-loop-distribute-patterns \
               -Wall -Wextra -Werror

.PHONY: dhrystone
dhrystone: sim
	@./test/bench/run_dhrystone.sh ./sim $(DHRY_RUNS) $(DHRY_CYCLES) '$(DHRY_CFLAGS)'

# CoreMark, SIMULATED AT 16 KB OF ROM -- double the part's 8, because it does
# not fit the smaller one. Not a prerequisite of anything and not on CI. 100
# iterations: the CoreMark/MHz ratio is already stable to three decimals by 10.
# Flags fixed for comparability with the cores in the comparison set.
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

# Count logic cells from nextpnr, never cell counts from yosys. A flip-flop that
# cannot share a cell with the LUT feeding it takes a whole cell by itself, and
# over a thousand of this design's cells are like that. Counting `SB_LUT4`
# instead gave two planning estimates that were wrong in opposite directions.
FIT_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
            rtl/fetcher.v rtl/regfile.v rtl/regsel.v rtl/writeback.v rtl/littlecpu.v

fit.json: $(FIT_SRCS)
	@echo 'yosys: synthesising littlecpu for ice40 (log: fit.synth.log)'
	@yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlecpu -json $@' \
	  > fit.synth.log 2>&1 || { tail -40 fit.synth.log; exit 1; }

# 4219 = 4097 + 68 + 54: the fit job's measured count, the measured churn
# band, and the widest toolchain gap measured on one tree.
# If this goes red, find out what grew; raising it to pass defeats the point.
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
# Exact rather than budgeted the way FIT_MAX_LC is, because both are properties
# of the RTL rather than of placement: 2 SPRAM for the 64 KB data RAM, and 16
# EBR for the 8 KB banked ROM plus 4 for rtl/regfile.v.
SOC_EXPECT_SPRAM := 2
SOC_EXPECT_EBR   := 20

SOC_SRCS      := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v \
                 rtl/executor.v rtl/fetcher.v rtl/imemory.v rtl/memory.v \
                 rtl/regfile.v rtl/regsel.v rtl/timer.v rtl/uart.v rtl/spiflash.v \
                 rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v

# PHONY: SOC_PROG changes what this builds and make cannot see that.
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

# Named once, used verbatim everywhere the netlist matters: a second copy
# would let the digest and the placement it grades describe different builds.
SOC_SYNTH := read_verilog -sv $(SOC_SRCS); synth_ice40 -device u -dsp -spram -top littlesoc
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

# `|| true` matters: nextpnr's exit status is not the signal (icetime's report
# of the .asc is), and without it .DELETE_ON_ERROR deletes the .asc unread.
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

# 12 MHz is the board crystal's own step (the next one down is 6) -- a
# requirement, not a regression floor. When it trips, fix the design.
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

# ---- the ECP5 instrument ----------------------------------------------------
# A third instrument over a third design (same littlesoc, placed on an ECP5);
# none of its numbers merge with `make fit`'s or `make soc-timing`'s.

ECP5_DEVICE  := --25k
ECP5_PACKAGE := CABGA381
ECP5_SPEED   := 6
ECP5_PART    := LFE5U-25F-6CABGA381

# The constraint is deliberately far above this design's real Fmax: nextpnr
# stops optimising a path once the constraint is met, so pinning it at the
# board's real 25 MHz would measure the constraint, not the design.
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
	@yosys -p 'read_verilog -sv $(SOC_SRCS); synth_ecp5 -top littlesoc -json $@' \
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

ECP5_TOOLS := yosys nextpnr-ecp5 trellis-db

.PHONY: ecp5-timing-toolchain
ecp5-timing-toolchain:
	@soc/print_toolchain.sh $(ECP5_TOOLS)

# nextpnr exits 1 on every successful run here (the constraint is meant to be
# missed), so `|| true` is required. The first line deletes both outputs for
# the same reason: a run that dies before nextpnr's last line must not leave a
# stale, internally-consistent pair from the PREVIOUS run for the report reader
# to trust. Do not drop this line.
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

# THE FLAGS STRING IS PART OF THE RESULT: an unquoted parenthetical once let the
# report print a truncated -- and therefore wrong -- flags line. Quoted here and
# reused from DHRY_CFLAGS so the board and simulated numbers stay comparable.
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

# Not policed by test/memmap_test.sh -- if the UART base ever moves in the
# RTL, move it here too.
DHRY_UART_BASE   ?= 0x00020020

DHRY_BOARD_RUNS  ?= 20000

.PHONY: dhrystone-board
dhrystone-board:
	@rm -f board.json board.asc board.bin
	@$(MAKE) --no-print-directory board.bin BOARD_OSC=$(BOARD_OSC) BOARD_ROM=dhrystone-rom
	@echo
	@echo 'Dhrystone is in board.bin. Flash it with `make prog`, then read the'
	@echo 'report off the UART -- it prints itself, cycles and all.'

# ---- a bitstream, and a board to put it on ---------------------------------
# A separate flow from `soc-timing` on purpose -- different top, different
# pins -- so a frequency from here is not comparable. Do not merge the two.
BOARD ?= upduino

BOARD_SRCS := $(SOC_SRCS) soc/miso_share_enable.v soc/board_upduino.v
BOARD_TOP  := upduino_top
BOARD_PCF  := soc/upduino.pcf

BOARD_OSC ?= crystal
BOARD_OSC_PARAM := $(if $(filter internal,$(BOARD_OSC)),1,0)

BOARD_ROM ?= soc-rom

# BOARD_ROM=noop-rom means the banks are already written (by
# soc/run_suite_board.sh) and must not be rebuilt.
.PHONY: noop-rom
noop-rom:
	@test -s soc/rom_even.hex -a -s soc/rom_odd.hex || { \
	  echo '*** BOARD_ROM=noop-rom, but soc/rom_*.hex are missing or empty.'; \
	  echo '*** Something was meant to write them before this ran.'; \
	  exit 1; \
	}

# Also needs $(BOARD_ROM): rtl/imemory.v's $readmemh treats missing
# soc/rom_*.hex as a fatal yosys ERROR, which once failed this on a clean
# checkout while a working tree with leftover files still passed.
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

# Not a prerequisite of anything: flashing hardware is an outward-facing act.
# ROOT, ON macOS, NOT A STYLE CHOICE: Apple's DriverKit extension claims the
# FT232H's only interface at enumeration, so every unprivileged libftdi tool
# sees zero devices; root opens it anyway. Linux needs none.
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

# ---- the dual configuration, placed ----------------------------------------
# ECP5 only -- two fetch windows are two copies of the banked ROM -- so no
# number here merges with an up5k flow.
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

# ---- the mapped netlist's digest -------------------------------------------
# THE DIGEST REPLACES A SWEEP, NEVER A GATE. `make fit` and `make soc-timing`
# are graded against exactly what they are graded against today; what an equal
# digest buys is the sixteen placements a tied-off change would otherwise owe.
# It is sound in one direction only: equal means the placer's input moved by
# nothing but dead nets and source attributes, and different means nothing at
# all except that the seeds have to be spent.
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

# Refuses an unlisted NETLIST_PART rather than digesting nothing: two empty
# trees would otherwise compare equal, which this gate must never do by accident.
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

# BASE reaches the shell only through the environment, never as recipe text:
# git allows a quote, a semicolon and a backtick in a ref name (e.g. from
# `gh pr checkout`), which pasted into '$(BASE)' would break out and run.
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

# ---- the cross-core comparison harness -------------------------------------
#
# Places THIS core, VexRiscv and Hazard3's iCE40 build in one harness -- one
# geometry, one program, one part, one toolchain, the same seeds -- so the Fmax
# figures are one experiment instead of several. Only this core and VexRiscv are
# cycle-measurable here: Hazard3's iCE40 configuration sets CSR_COUNTER=0, so it
# has no mcycle to self-time a Dhrystone run with, and compare-dhrystone below is
# two cores, not three. Nothing here is a gate on the shipping design and nothing
# here touches rtl/: `make soc-timing` remains the SoC's measurement and this
# target's numbers are not comparable to it.
#
# `COMPARE_CORE=littlecpu` (default), `vexriscv` or `hazard3`;
# `COMPARE_SEED=<n>` picks a placement. soc/compare/sweep.sh runs littlecpu and
# vexriscv over four seeds each by default, which is a look at a distribution
# and not a verdict on one: a decision costs twelve to sixteen. This harness
# places hx8k, whose spread nobody has swept -- `soc/bands.py hx8k` is where
# that is stated, and it does not hand back up5k's figures for it.
COMPARE_CORE  ?= littlecpu
COMPARE_SEED  ?=
# 4 KB of ROM (8 SB_RAM40_4K) and 2 KB of data RAM (4 more). Shrunk from the
# shipping 8 KB / 64 KB so both designs fit one hx8k: VexRiscv takes 18 of the
# part's 32 block RAMs before either memory -- 4 for a register file the same
# size as this core's, 14 for a 1024-entry branch predictor -- and 30 is what
# its side of the harness then comes to. soc/compare/bench.lds states the same
# two sizes in its own syntax and soc/compare/geometry_test.sh compares them.
COMPARE_ROM_WORDS := 1024
COMPARE_RAM_WORDS := 512
# The placed design must be at least this fraction of what the core synthesises
# to alone. soc/compare/placed_vs_synth.py carries why, and it is the check that
# stops this flow reporting a number for a core yosys folded away.
COMPARE_MIN_RATIO := 0.8

# Hazard3's own file list (soc/compare/hazard3/hdl/hazard3.f), minus
# hazard3_cpu_2port.v: this harness uses the single-AHB-port top only.
# HAZARD3_DIR comes from soc/compare/hazard3_pin.mk.
HAZARD3_HDL  := $(HAZARD3_DIR)/hdl
HAZARD3_SRCS := $(HAZARD3_HDL)/hazard3_core.v $(HAZARD3_HDL)/hazard3_cpu_1port.v \
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

ifeq ($(COMPARE_CORE),vexriscv)
COMPARE_TOP  := bench_vexriscv
COMPARE_SRCS := soc/compare/bench_vexriscv.v rtl/memory.v
# Read as plain Verilog, out of the SHA-pinned clone, and never copied into this
# repo. Its RVFI outputs are left unconnected in the harness, where synthesis
# prunes them; on the standalone run below they are the top's own ports, and
# there `delete -port` -- formal/check-nonperturbation.py's technique -- is what
# stops 556 SB_IO no ice40 package can place.
COMPARE_READ := read_verilog $(RISCV_FORMAL_DIR)/cores/VexRiscv/VexRiscv.v; \
                read_verilog -sv $(COMPARE_SRCS)
COMPARE_CORE_READ := read_verilog $(RISCV_FORMAL_DIR)/cores/VexRiscv/VexRiscv.v; \
                     hierarchy -top VexRiscv; delete -port VexRiscv/rvfi_*
COMPARE_CORE_TOP  := VexRiscv
COMPARE_DEPS      := $(COMPARE_SRCS) | $(RISCV_FORMAL_DIR)
COMPARE_CORE_DEPS := | $(RISCV_FORMAL_DIR)
else ifeq ($(COMPARE_CORE),hazard3)
COMPARE_TOP  := bench_hazard3
COMPARE_SRCS := $(HAZARD3_SRCS) rtl/memory.v soc/compare/bench_hazard3.v
# `-I` reaches hazard3_config.vh, hazard3_config_inst.vh and the three other
# headers hdl/hazard3.f's `include .` names -- soc/compare/bench_hazard3.v is
# fpga_icebreaker.v's own configuration, read straight out of the SHA-pinned
# clone soc/compare/hazard3_pin.mk materialises, and never copied into rtl/.
COMPARE_READ := read_verilog -sv -I $(HAZARD3_HDL) $(COMPARE_SRCS)
COMPARE_CORE_READ := read_verilog -sv -I $(HAZARD3_HDL) $(HAZARD3_SRCS); \
                     hierarchy -top hazard3_cpu_1port
COMPARE_CORE_TOP  := hazard3_cpu_1port
COMPARE_DEPS      := $(COMPARE_SRCS) | $(HAZARD3_DIR)
COMPARE_CORE_DEPS := $(HAZARD3_SRCS) | $(HAZARD3_DIR)
else
COMPARE_TOP  := bench_littlecpu
# FIT_SRCS is already this repo's list of "the core and nothing else", which is
# exactly what the standalone reference synthesis wants. A second copy of it
# here would be the stale-list defect SIM_RTL_SRCS's comment describes.
COMPARE_SRCS := $(FIT_SRCS) rtl/imemory.v rtl/memory.v \
                soc/compare/bench_littlecpu.v
COMPARE_READ := read_verilog -sv $(COMPARE_SRCS)
COMPARE_CORE_READ := read_verilog -sv $(FIT_SRCS); \
                     hierarchy -top littlecpu; delete -port littlecpu/rvfi_*
COMPARE_CORE_TOP  := littlecpu
COMPARE_DEPS      := $(COMPARE_SRCS)
COMPARE_CORE_DEPS := $(FIT_SRCS)
endif

# PHONY for the same reason `soc-rom` is: the image depends on nothing make can
# see a change to, and a stale ROM would make the measurement describe a
# program nobody asked for. Both images come out of one objcopy run, so the two
# harnesses cannot come to be running different code.
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

# The core on its own, with no harness to fold against. This is the reference
# soc/compare/placed_vs_synth.py grades the placement against; it never places
# (both cores present far more SB_IO than any package has) and is not meant to.
compare.$(COMPARE_CORE).core.log: $(COMPARE_CORE_DEPS)
	@echo 'yosys: synthesising $(COMPARE_CORE_TOP) alone for hx8k (log: $@)'
	@yosys -p '$(COMPARE_CORE_READ); synth_ice40 -top $(COMPARE_CORE_TOP); stat' \
	  > $@ 2>&1 || { tail -40 $@; exit 1; }

# `compare-rom` FIRST. COMPARE_DEPS ends with an order-only `| $(RISCV_FORMAL_DIR)`
# for VexRiscv, and everything after a `|` is order-only -- so written the other
# way round the phony stopped forcing a rebuild, `--seed` reached nextpnr on a
# netlist make never regenerated, and four "placements" of that core reported
# one number to the millisecond.
compare.$(COMPARE_CORE).json: compare-rom $(COMPARE_DEPS)
	@echo 'yosys: synthesising $(COMPARE_TOP) for hx8k (log: compare.$(COMPARE_CORE).synth.log)'
	@# No `-dsp`: hx8k has no SB_MAC16, so this core's multiplier is soft logic
	@# here and `make fit`'s DSP-mapped number does not transfer.
	@# chparam BEFORE hierarchy, so the harness's geometry has one source -- the
	@# variables above -- rather than a second copy in each .v file's defaults.
	@yosys -p '$(COMPARE_READ); \
	  chparam -set ROM_WORDS $(COMPARE_ROM_WORDS) -set RAM_WORDS $(COMPARE_RAM_WORDS) $(COMPARE_TOP); \
	  hierarchy -top $(COMPARE_TOP); \
	  synth_ice40 -top $(COMPARE_TOP) -json $@; stat' \
	  > compare.$(COMPARE_CORE).synth.log 2>&1 \
	  || { tail -40 compare.$(COMPARE_CORE).synth.log; exit 1; }

# nextpnr's own status is not the signal, for the reason `soc.asc` records: it
# grades its own default clock with its own estimator, and what is graded here
# is icetime's report of the .asc it wrote.
compare.$(COMPARE_CORE).asc: compare.$(COMPARE_CORE).json soc/compare/bench_hx8k.pcf
	@echo 'nextpnr: placing $(COMPARE_TOP) on hx8k/ct256 (log: compare.$(COMPARE_CORE).pnr.log)'
	@nextpnr-ice40 --hx8k --package ct256 --json $< --pcf soc/compare/bench_hx8k.pcf \
	  $(if $(COMPARE_SEED),--seed '$(COMPARE_SEED)') --asc $@ \
	  > compare.$(COMPARE_CORE).pnr.log 2>&1 || true
	@test -s $@ || { \
	  echo '*** make compare-timing: nextpnr produced no bitstream, so NOTHING'; \
	  echo '*** was measured. That is a failed placement, not a fast design.'; \
	  tail -30 compare.$(COMPARE_CORE).pnr.log; \
	  rm -f $@; \
	  exit 1; \
	}

# All three harnesses in one simulation, running the one image, required to
# publish the same values. soc/compare/placed_vs_synth.py says a core is still
# in the netlist; this says the netlist runs. Not a prerequisite of
# `compare-timing`: it needs iverilog and the cross compiler, and a timing
# measurement of a design that does not execute is a defect this catches
# rather than one it prevents.
COMPARE_SMOKE_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                      soc/compare/bench_vexriscv.v soc/compare/bench_hazard3.v \
                      soc/compare/bench_tb.v

compare.vvp: $(COMPARE_SMOKE_SRCS) compare-rom | $(RISCV_FORMAL_DIR) $(HAZARD3_DIR)
	iverilog -I./rtl/ -I$(HAZARD3_HDL) -g2012 -o $@ \
	  $(RISCV_FORMAL_DIR)/cores/VexRiscv/VexRiscv.v $(HAZARD3_SRCS) \
	  $(COMPARE_SMOKE_SRCS)

.PHONY: compare-smoke
compare-smoke: compare.vvp
	@vvp $<

# The OTHER factor of throughput. `compare-timing` above reports each core's
# clock; this reports the cycles each takes for the same work, so a DMIPS figure
# for either side is measured here rather than quoted from a project's README.
#
# IT IS A SIMULATION, AND CANNOT BE A PLACEMENT. Dhrystone needs more memory
# than an hx8k has block RAM for -- soc/compare/dhry_fit.py prints that
# arithmetic on every run -- so the memories are enlarged for both cores
# together and the clock to multiply these cycles by comes from the smaller
# placed geometry. That is the caveat on the result, and it travels with it.
#
# COMPARE_DHRY_CFLAGS is not DHRY_CFLAGS and must not be made to match it: this
# image has to run on both cores, and their common ISA is RV32IC. No M
# extension, so multiply and divide are libgcc calls; no Zicsr, so the run is
# timed on the bus instead of by `mcycle`. `make dhrystone`'s number is a
# different workload on a different machine and the two are not comparable.
# 400 runs. The measured window is the benchmark's loop and nothing else, so the
# figure is flat in this: 100 runs and 400 differ by 0.02% on this core and 0.26%
# on VexRiscv. The count is set by how long two cores in one iverilog simulation
# take, not by what the number needs.
COMPARE_DHRY_RUNS   ?= 400
COMPARE_DHRY_CYCLES ?= 2000000
COMPARE_DHRY_CFLAGS := -march=rv32ic -mabi=ilp32 -O2 -std=c11 \
                       -ffreestanding -fno-tree-loop-distribute-patterns \
                       -Wall -Wextra -Werror

COMPARE_DHRY_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                     soc/compare/bench_vexriscv.v soc/compare/dhry_tb.v

compare.dhry.vvp: $(COMPARE_DHRY_SRCS) | $(RISCV_FORMAL_DIR)
	iverilog -I./rtl/ -g2012 -o $@ $(RISCV_FORMAL_DIR)/cores/VexRiscv/VexRiscv.v \
	  $(COMPARE_DHRY_SRCS)

# Both cores' standalone censuses, because the fit arithmetic reads how much
# block RAM each core needs before either memory out of them rather than
# carrying a copy. Recursive, because those log names are COMPARE_CORE's and
# this target needs both sides at once.
.PHONY: compare-dhrystone
compare-dhrystone: compare.dhry.vvp
	@$(MAKE) --no-print-directory COMPARE_CORE=littlecpu compare.littlecpu.core.log
	@$(MAKE) --no-print-directory COMPARE_CORE=vexriscv compare.vexriscv.core.log
	@./soc/compare/run_dhrystone.sh $(COMPARE_DHRY_RUNS) $(COMPARE_DHRY_CYCLES) \
	  '$(COMPARE_DHRY_CFLAGS)' compare.dhry.vvp
	@echo
	@if [ -f soc/compare/product.json ]; then \
	  echo '== the stamped cross-core product, if the stamp still matches this tree =='; \
	  python3 soc/compare/product_check.py soc/compare/product.json dhrystone \
	    --repo . --current 'cflags=$(COMPARE_DHRY_CFLAGS)' \
	    --current 'rom_words=$(COMPARE_ROM_WORDS)' \
	    --current 'ram_words=$(COMPARE_RAM_WORDS)' || true; \
	else \
	  echo 'no soc/compare/product.json yet -- `make compare-product` stamps one'; \
	fi

# Both factors of both cross-core pairs (Dhrystone against VexRiscv, CoreMark
# against Hazard3 once make compare-coremark exists) in one command, written
# into soc/compare/product.json -- soc/compare/run_product.sh's header has the
# reasoning. COMPARE_PRODUCT_SEEDS defaults to twelve, this file's own floor for
# a verdict rather than a look; COMPARE_PRODUCT_OUT overrides where it lands,
# for a dry run that should not touch the tracked artifact.
.PHONY: compare-product
compare-product:
	@./soc/compare/run_product.sh

.PHONY: compare-timing
compare-timing: compare.$(COMPARE_CORE).asc compare.$(COMPARE_CORE).core.log
	@sed -n '/^Info: Device utilisation:/,/^$$/s/^Info: //p' compare.$(COMPARE_CORE).pnr.log
	@echo
	@echo '== is the core still there? =='
	@python3 soc/compare/placed_vs_synth.py compare.$(COMPARE_CORE).pnr.log \
	  compare.$(COMPARE_CORE).core.log $(COMPARE_CORE) --min-ratio $(COMPARE_MIN_RATIO)
	@echo
	@echo '== icetime: the critical path, and the LOGIC/ROUTING SPLIT =='
	@icetime -d hx8k -P ct256 -p soc/compare/bench_hx8k.pcf -t \
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
	rm -f compare.dhry.vvp
	rm -f soc/compare/rom_even.hex soc/compare/rom_odd.hex soc/compare/rom_flat.hex
	rm -f soc/compare/dhry_even.hex soc/compare/dhry_odd.hex soc/compare/dhry_flat.hex
	rm -f soc/compare/dhry_ram.hex
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

# The riscv-formal clone rule and its pin guard live in formal/pin.mk.
