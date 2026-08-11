include formal/pin.mk

.DELETE_ON_ERROR:

RISCV_FORMAL_MACROS := RISCV_FORMAL RISCV_FORMAL_COMPRESSED RISCV_FORMAL_ALIGNED_MEM RISCV_FORMAL_NRET=1 RISCV_FORMAL_XLEN=32 RISCV_FORMAL_ILEN=32

rvfi_macros.vh: $(RISCV_FORMAL_DIR)/checks/rvfi_macros.py
	python3 $^ > $@

# Both sim legs build from this list. Do not copy it anywhere else — a second
# copy goes stale and the gate then checks a different design than it says it
# does. The CI job had one; it missed rtl/imemory.v and rtl/memory.v when they
# landed, and spent a run elaborating a testbench whose memories were not there.
# That job calls `make elaborate-strict` now, so there is one list to update.
SIM_RTL_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
                rtl/fetcher.v rtl/imemory.v rtl/memory.v rtl/pairtable.v rtl/regfile.v \
                rtl/regsel.v rtl/timer.v rtl/writeback.v rtl/littlecpu.v

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

# Downloaded tools are unpacked here, outside the checkout. An install inside
# it would be gitignored, and a git worktree gets tracked files only, so it
# would be invisible from every worktree -- which is how `make cosim-suite`
# came to report Sail as not installed on a machine that had it.
# XDG_CACHE_HOME moves this; test/cosim.py resolves the same path from the same
# rule, and test/tool_cache_test.sh is what says the two still agree.
TOOL_CACHE := $(if $(XDG_CACHE_HOME),$(XDG_CACHE_HOME),$(HOME)/.cache)/little-cpu

# Nothing from here down to `cosim-suite` is reachable from `make test` or
# `make test-units`. Keep it that way now that CI requires co-simulation: it is
# a job of its own that fetches Sail first, so `make test` still runs on a
# machine without Sail, and a divergence reads as co-sim rather than as a suite
# failure.
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

# Prebuilt binaries. Building from source needs opam, OCaml and the Sail
# compiler. There is no `brew install sail-riscv`; homebrew's `sail` is an
# unrelated WordPress tool.
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

# The verified tarball is kept rather than deleted, so a CI cache can hold the
# fetch without holding anything executable. A cache of the unpacked tree would
# restore a binary that met no checksum in the run that executes it; a cache of
# the tarball meets the digest below on every run, hit or miss.
SAIL_DOWNLOAD_DIR := $(TOOL_CACHE)/download
SAIL_TARBALL      := $(SAIL_DOWNLOAD_DIR)/$(SAIL_ASSET)-$(SAIL_RISCV_VERSION).tar.gz
SAIL_CACHE_KEY    := sail-$(SAIL_RISCV_VERSION)-$(SAIL_ASSET)-$(SAIL_SHA256)

# Only for the goals that run the binary. If this check could stop `make test`,
# an out-of-date tools/sail would break the merge gate for everyone.
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

# Where the fetch is cached and under what key, read out of here rather than
# spelled out in the workflow, so a cache key cannot go on naming bytes the pin
# no longer has. `name=value` lines because a workflow step appends them to
# $GITHUB_OUTPUT; test/probe_gates.sh seeds its fixture from the third.
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

# Both sim legs check every retired instruction against this file. A rule in
# test/sanitize_monitor.py therefore decides what counts as a correct result.
# Changing one changes what the tests mean. It is not an elaboration fix.
test/monitor.sim.v: test/monitor.v test/sanitize_monitor.py
	python3 test/sanitize_monitor.py $< > $@

test/rtl.cc: $(SIM_RTL_SRCS) rvfi_macros.vh test/testbench.v test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top testbench; write_cxxrtl $@'

# `check` is what reports a wire nothing drives. The test/rtl.cc recipe above
# builds one of those quietly and exits 0.
ELABORATE_STRICT_OUT ?= /tmp/elaborate-strict.cc

# Keep the yosys script on one line. A line split with a backslash inside single
# quotes stays literal, so yosys reads the backslash as a command and dies with
# "No such command: \". It works with the make on macOS and fails on the runner.
.PHONY: elaborate-strict
elaborate-strict: $(SIM_RTL_SRCS) test/testbench.v
	yosys -p 'read_verilog -sv $(SIM_RTL_SRCS) test/testbench.v; hierarchy -top testbench; proc; opt_clean; check; write_cxxrtl $(ELABORATE_STRICT_OUT)'

# The `cd` is required: generate.py opens ../insns/isa_<isa>.txt relative to its
# own CWD.
MONITOR_GEN = cd $(RISCV_FORMAL_DIR)/monitor && python3 generate.py -i rv32imc -c 1 -a -p monitor

# The clone is an order-only prerequisite, after the `|`. A directory's
# timestamp changes whenever anything is written inside it, so as a normal
# prerequisite it would go out of date on unrelated work. This file is checked
# in, so that means builds rewriting it and the diff showing up in someone
# else's commit.
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

# Do not put $(SVLINT_VERSION) in these names. Spelling the version out is what
# makes the lookup below come back empty when someone bumps the version and
# forgets the digests, so `lint-setup` refuses to download anything.
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

# `-i rtl` is required. svlint looks up `include "structs.v"` on the include path
# and nowhere else, so without it nothing parses. The two passes below both
# matter as well: about a fifth of rtl/ is inside `ifdef RISCV_FORMAL`, and
# svlint skips those lines unless the macros are defined.
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
                timer_tb pairtable_tb

UNIT_BENCH_SRC_exec_tb     := rtl/structs.v rtl/executor.v
UNIT_BENCH_SRC_mem_tb      := rtl/memory.v
UNIT_BENCH_SRC_imem_tb     := rtl/imemory.v
UNIT_BENCH_SRC_decoder_tb  := rtl/structs.v rtl/decoder.v rtl/regsel.v
UNIT_BENCH_SRC_regfile_tb  := rtl/regfile.v
UNIT_BENCH_SRC_csr_tb      := rtl/structs.v rtl/csrs.v
UNIT_BENCH_SRC_accessor_tb := rtl/structs.v rtl/accessor.v
UNIT_BENCH_SRC_monitor_tb  := test/monitor.sim.v
UNIT_BENCH_SRC_timer_tb    := rtl/timer.v
UNIT_BENCH_SRC_pairtable_tb := rtl/pairtable.v

# `present` is read from disk inside the recipe, not with $(wildcard). Make reads
# a directory once and remembers it, and a check working from a stale listing can
# miss a bench that is really there.
#
# `set -e` comes first here and in `test-units`, before mktemp and before the
# trap. The other way round, a failed mktemp left $$tmp empty, the trap was set
# on nothing, and every path below turned into a path at the root of the disk.
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

# Hangs off `test` rather than standing alone, so it runs in the job CI already
# requires and nobody has to add a step for it. It needs no cross compiler, no
# Sail, no yosys and no sby, so it does not stop `make test` running anywhere it
# ran before.
.PHONY: probe-gates
probe-gates:
	@./test/probe_gates.sh

# Hangs off `test` for the same reason `probe-gates` does: it is bash and a stub
# `gh`, so it runs anywhere, and the pin-bump path is otherwise exercised once a
# week by a workflow nobody watches.
.PHONY: pin-bump-test
pin-bump-test:
	@./formal/test-propose-pin-bump.sh

# Reads the three directories this file resolves and checks them against
# test/cosim.py's. Hangs off `test` like the other bash checks: python3 and a
# shell, no cross compiler, no Sail, no yosys.
.PHONY: tool-cache-test
tool-cache-test:
	@./test/tool_cache_test.sh '$(SAIL_RISCV_DIR)' '$(SVLINT_DIR)' '$(SAIL_DOWNLOAD_DIR)'

# Compares every file that describes the memory map against the RTL that
# implements it. Hangs off `test` like the other bash checks -- grep and sed, no
# cross compiler, no simulator, no yosys -- and it has to run BEFORE the suite is
# graded, because what it catches is the suite grading a machine that is not the
# one that places on the part.
.PHONY: memmap-test
memmap-test:
	@./test/memmap_test.sh

# The cross-core comparison harness states its geometry in six places and this
# is what says they agree. Hangs off `test` like the other bash checks -- grep
# and sed only -- because the harness itself needs yosys, nextpnr and the pinned
# clone, so nothing else on this machine would notice it rotting.
.PHONY: compare-geometry-test
compare-geometry-test:
	@./soc/compare/geometry_test.sh

# Forces the elaboration checks in rtl/imemory.v, rtl/memory.v and rtl/timer.v
# to fire, in both frontends. Hangs off `test` because the parameter shapes they
# guard are the ones the SoC and the benches pass, so nothing else here would
# notice a check that had stopped checking. It needs iverilog and yosys, which
# `sim` and `test-units` already require.
.PHONY: window-test
window-test:
	@./test/window_test.sh

# Drives formal/check-abc-engine.sh both ways against a stub yosys and a stub
# sby. Hangs off `test` like the other bash checks -- it is bash and two stubs,
# so it runs anywhere -- and it has to, because the diagnostic it covers only
# ever speaks on a machine that cannot run `make -C formal complete` at all.
.PHONY: abc-engine-test
abc-engine-test:
	@./formal/test-abc-engine.sh

.PHONY: test
test: sim test-units probe-gates pin-bump-test tool-cache-test memmap-test \
      compare-geometry-test window-test abc-engine-test
	@./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

# The same suite `make test` runs, with the runner charging every cycle to the
# decoder's stall reasons, and test/stall_report.py turning that into a table.
# Its own target rather than part of `make test`: the counting costs a
# debug_eval() per cycle on every program, and a CPI figure is a measurement to
# compare against the last one, not something to fail a merge on. What it does
# grade is its own arithmetic -- a cycle the decoder stalled for a reason this
# does not name is a reason nobody has written down, and it exits nonzero.
.PHONY: cycles
cycles: sim
	@STALL_REPORT=1 ./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

# Dhrystone 2.1, the one number this core can be quoted against other cores'.
# Not a prerequisite of anything and not on CI: there is no CPI ratchet here and
# this adds none. It reports DMIPS/MHz, the ROM image against the SoC's 8 KB,
# and the same per-reason cycle accounting `make cycles` prints -- which is the
# first read of that split on compiled code rather than on hand-written
# assembly.
#
# DHRY_CFLAGS IS THE MEASUREMENT'S OTHER HALF. The same string compiles the
# benchmark and is compiled INTO it, so the flags print beside the number and
# cannot be separated from it; test/bench/dhry_port.c will not build without
# them. Changing them changes the number -- Dhrystone is famously sensitive to
# the optimiser -- so quote both or neither, the same rule `make fit` and
# `make soc-timing` already carry about their toolchains.
#
# -O2 rather than the suite's -Os: it is what the cores in the comparison set
# publish, and the ROM budget below is what says whether this part can afford
# it. -fno-tree-loop-distribute-patterns keeps gcc from rewriting the byte loops
# in dhry_port.c into calls to the very routines they define.
# 2000 runs, not the smallest number that produces a figure. test/crt0.S zeroes
# Dhrystone's 10 KB Arr_2_Glob a word at a time before main, and that loop's
# stall mix is nothing like the benchmark's -- at 500 runs it is a tenth of the
# accounted cycles and at 2000 it is under three percent. The runner prints the
# residual so the number is checked rather than assumed.
DHRY_RUNS   ?= 2000
DHRY_CYCLES ?= 4000000
DHRY_CFLAGS := -march=rv32imc_zicsr_zifencei -mabi=ilp32 -O2 -std=c11 \
               -ffreestanding -fno-tree-loop-distribute-patterns \
               -Wall -Wextra -Werror

# The 8 KB budget is NOT here. test/bench/bench.lds gives the `rom` region that
# length, so ld enforces it and reports an overflow in bytes; the runner reads
# the number back out of that file to print the image against it.
.PHONY: dhrystone
dhrystone: sim
	@./test/bench/run_dhrystone.sh ./sim $(DHRY_RUNS) $(DHRY_CYCLES) '$(DHRY_CFLAGS)'

# Count logic cells from nextpnr, never cell counts from yosys. A flip-flop that
# cannot share a cell with the LUT feeding it takes a whole cell by itself, and
# over a thousand of this design's cells are like that. Counting `SB_LUT4`
# instead gave two planning estimates that were wrong in opposite directions.
#
# Placement always fails here, and that is fine. This top has its memories
# outside the chip, so it needs far more pins than the sg48 package has and
# nextpnr gives up on one. It prints the utilisation table before it gets that
# far, and that table is the number we want. Making it place would mean picking
# real pins, which means building the SoC memory first.
FIT_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
            rtl/fetcher.v rtl/pairtable.v rtl/regfile.v rtl/regsel.v rtl/writeback.v \
            rtl/littlecpu.v

fit.json: $(FIT_SRCS)
	@echo 'yosys: synthesising littlecpu for ice40 (log: fit.synth.log)'
	@yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlecpu -json $@' \
	  > fit.synth.log 2>&1 || { tail -40 fit.synth.log; exit 1; }

# 3625 is the `fit` job's 3543 plus a 50-cell churn band plus 32 for the
# toolchain: identical RTL moves about 50 cells on ABC re-mapping alone, and this
# tree read 3543 under CI's suite against 3575 under a local Homebrew yosys. CI
# resolves the OSS CAD Suite rather than pinning it, so the second term is what
# stops a suite bump going red on a pull request that changed no RTL.
# If this goes red, find out what grew; raising it to pass defeats the point.
FIT_MAX_LC := 3625

.PHONY: fit
fit: fit.json
	@nextpnr-ice40 --up5k --package sg48 --json $< --pcf-allow-unconstrained \
	  > fit.log 2>&1 || true
	@python3 soc/fit_report.py fit.log --max-lc $(FIT_MAX_LC)

# This builds `littlesoc`, not the `littlecpu` that `make fit` measures. It
# includes the ROM and the data RAM, so its cell count is bigger and the two
# numbers are not comparable. This one also has to place, because icetime reads
# the `.asc` file that only a finished placement writes.
# A `.c` program is linked against test/asm/boot.lds and test/crt0.S, so its
# ROM image carries `.data`'s initialiser after `.text` and the startup copies
# it into SPRAM -- which no bitstream can initialise. That is the only image
# shape this board can boot a program with globals from, so it is the default:
# an image shape nothing builds is one nobody notices breaking.
SOC_PROG      ?= datainit.c
# soc/rom_banks.py rejects an image that overruns the ROM, so this has to be
# rtl/littlesoc.v's `ROM_WORDS` and not merely near it: too small rejects a
# program that fits, too large splits one that does not into banks the
# bitstream then truncates. test/memmap_test.sh compares them.
SOC_ROM_WORDS := 2048
# Exact rather than budgeted the way FIT_MAX_LC is, because both are properties
# of the RTL rather than of placement: 2 SPRAM for the 64 KB data RAM, and 16
# EBR for the 8 KB banked ROM plus 4 for rtl/regfile.v and 1 for
# rtl/pairtable.v, whose 256 entries of 16 bits are exactly one.
SOC_EXPECT_SPRAM := 2
SOC_EXPECT_EBR   := 21

SOC_SRCS      := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v \
                 rtl/executor.v rtl/fetcher.v rtl/imemory.v rtl/memory.v \
                 rtl/pairtable.v rtl/regfile.v rtl/regsel.v rtl/timer.v \
                 rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v

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
	case '$(SOC_PROG)' in \
	  *.c) $$CC -march=rv32imc_zicsr_zifencei -mabi=ilp32 -nostdlib \
	         -Os -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns \
	         -Wall -Wextra -Werror -I test/asm -T test/asm/boot.lds \
	         -o "$$tmp/prog.elf" test/crt0.S test/asm/$(SOC_PROG); \
	       sections='-j .text -j .data' ;; \
	  *)   $$CC -march=rv32imc_zicsr_zifencei -mabi=ilp32 -nostdlib -I test/asm \
	         -T test/asm/sections.lds -o "$$tmp/prog.elf" test/asm/$(SOC_PROG); \
	       sections='-j .text' ;; \
	esac; \
	$$OBJCOPY -O verilog --verilog-data-width=4 $$sections "$$tmp/prog.elf" "$$tmp/rom.hex"; \
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

# nextpnr's exit status is deliberately not the signal: it grades its own 12 MHz
# default with its own estimator, and what this repo grades is icetime's report
# of the `.asc` nextpnr has already written. When it does fail it writes that
# file first, so honouring the status would leave nothing to measure.
# `.DELETE_ON_ERROR` is why the `||` is needed rather than merely tidy — without
# it make deletes that `.asc`.
#
# `SOC_SEED=<n>` places the same design differently. One placement is a sample,
# not a measurement — compare a few seeds before believing a change helped.
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

# 12 MHz is the board crystal, and the part's own oscillator divides 48 by 1, 2,
# 4 or 8 -- so the step below 12 is 6, and missing it costs half the clock.
# This sits at what the hardware asks for rather than under the last
# measurement, so unlike a regression floor it does not slide as the design
# moves. When it trips, fix the design: lowering the number gives up the board
# clock.
SOC_MIN_MHZ := 12.0

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
	@echo 'toolchain-dependent the same way `make fit` is. 12 MHz is a'
	@echo 'REQUIREMENT as of ADR-0066: it is the board clock, and the step below'
	@echo 'it is 6 MHz. One placement is a sample: soc/timing_sweep.sh prints the'
	@echo 'spread, and a requirement has to hold at all of them.'
	@# The ratchet is applied by the thing that already parses the report. It
	@# was a `python3 -c` here, i.e. a SECOND parser of the same file -- and the
	@# second one was the one holding the gate.
	@python3 soc/timing_split.py soc.timing.rpt --min-mhz $(SOC_MIN_MHZ)

# ---- the cross-core comparison harness -------------------------------------
#
# Places THIS core and VexRiscv in one harness -- one geometry, one program, one
# part, one toolchain, the same seeds -- so the two Fmax figures are one
# experiment instead of two. Nothing here is a gate on the shipping design and
# nothing here touches rtl/: `make soc-timing` remains the SoC's measurement and
# this target's numbers are not comparable to it.
#
# `COMPARE_CORE=littlecpu` (default) or `vexriscv`; `COMPARE_SEED=<n>` picks a
# placement. soc/compare/sweep.sh runs both cores over four seeds each, which is
# what a distribution needs.
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

# Both harnesses in one simulation, running the one image, required to publish
# the same values. soc/compare/placed_vs_synth.py says the core is still in the
# netlist; this says the netlist runs. Not a prerequisite of `compare-timing`:
# it needs iverilog and the cross compiler, and a timing measurement of a design
# that does not execute is a defect this catches rather than one it prevents.
COMPARE_SMOKE_SRCS := $(SIM_RTL_SRCS) soc/compare/bench_littlecpu.v \
                      soc/compare/bench_vexriscv.v soc/compare/bench_tb.v

compare.vvp: $(COMPARE_SMOKE_SRCS) compare-rom | $(RISCV_FORMAL_DIR)
	iverilog -I./rtl/ -g2012 -o $@ $(RISCV_FORMAL_DIR)/cores/VexRiscv/VexRiscv.v \
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
	@echo 'Different part, smaller memories, no timer, and the two cores'
	@echo 'implement different ISAs: RV32IMC+Zicsr with traps here, RV32IC with'
	@echo 'no CSR file and no traps there. Read ADR-0086 before quoting any of'
	@echo 'it. One placement is a sample: soc/compare/sweep.sh runs four each.'
	@python3 soc/timing_split.py compare.$(COMPARE_CORE).timing.rpt

clean:
	rm -f fit.json fit.log fit.synth.log
	rm -f soc.json soc.asc soc.synth.log soc.pnr.log soc.timing.rpt
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
