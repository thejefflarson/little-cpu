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

testbench.vvp: rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v test/monitor.sim.v
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
# Sail co-simulation (docs/adr/0032) -- a SPIKE, deliberately opt-in.
#
# Nothing below is reachable from `make test`, `make test-units` or CI. The
# whole point of keeping it off the default path is that `make test` must
# still work on a machine with no Sail installed, and a time-boxed experiment
# must not quietly become a merge gate. Run it by hand:
#
#     make sail-setup     # once: fetch, verify and unpack the pinned release
#     make cosim          # build the architectural-state tracer
#     make cosim-run      # run it on one program (PROG=add.S by default)
# ---------------------------------------------------------------------------

# The sail-riscv release this spike was run against, pinned the way
# formal/pin.mk pins riscv-formal and for the same reason (ADR-0013): an oracle
# that moves under you is not an oracle -- and this one is EXECUTED, by the
# `--version` probe at the end of the fetch below and by every `make cosim-run`
# after it, via test/cosim.py.
#
# A GitHub release asset is MUTABLE, so a version on its own pins a FILENAME,
# not BYTES. The SHA-256 beside each asset name below is what pins the bytes,
# and it is checked before anything is extracted, let alone run. Bumping the
# version therefore means re-pinning all three digests; that friction is the
# point, exactly as it is for RISCV_FORMAL_SHA.
#
# An attempt to set the version from outside is REJECTED rather than ignored:
# `override` alone would make `make SAIL_RISCV_VERSION=...` silently do
# nothing, which reads as a working knob. This is a supply-chain control, not
# a knob.
#
# ADR-0033 gap 4 recorded the unverified fetch as accepted-because-opt-in, with
# "give it pin.mk's treatment" as the precondition for ever putting co-sim on
# `make test` or CI. That precondition is met here. It is the only one: the
# rest of ADR-0032's integration list (the `tohost` doubleword, a
# COSIM_EXPECTED_FAIL baseline, a nightly job) is untouched, and co-simulation
# stays off every default path.
ifneq ($(filter command line environment,$(origin SAIL_RISCV_VERSION)),)
$(error SAIL_RISCV_VERSION cannot be set from the command line or the \
  environment: it pins bytes this repo executes. Change it in the Makefile, \
  together with the SHA-256 digests below it)
endif
override SAIL_RISCV_VERSION := 0.13.1

# Three-part, so a branch name, `latest`, or one of upstream's rolling
# date-and-SHA tags (`2026-07-27-9901550`) cannot be written here by accident.
# Upstream also ships two-part tags (`0.13`, `0.12`): bumping to one means
# widening this regex deliberately, in the same commit as the new digests,
# rather than discovering at fetch time that the guard was never the control.
# The digests are.
ifeq ($(shell printf '%s' '$(SAIL_RISCV_VERSION)' | grep -cE '^[0-9]+\.[0-9]+\.[0-9]+$$'),0)
$(error SAIL_RISCV_VERSION must be a three-part release version like 0.13.1, \
  not a branch, a moving tag or a range: '$(SAIL_RISCV_VERSION)')
endif

# Prebuilt, first-party release binaries from github.com/riscv/sail-riscv.
# Building the model from source needs opam + OCaml + the Sail compiler
# (there is no `brew install sail-riscv`; homebrew's `sail` formula is an
# unrelated WordPress deploy tool). The upstream release ships a macOS arm64
# and two Linux tarballs, which covers every machine this repo is developed
# and CI'd on, so the source build is not worth its cost here.
#
# Digests are `shasum -a 256` over all three 0.13.1 assets, downloaded fresh
# and cross-checked against the per-asset digest the GitHub releases API
# reports. All three are pinned deliberately: a host whose asset has no digest
# here must fail closed rather than fetch something unverifiable, so the
# recipe below refuses to run rather than skipping the check for one platform.
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

# tools/sail/.sail-pin, written only after the digest check passes. Line 1 is
# the pin the tree was fetched under; line 2 is the SHA-256 of the unpacked
# sail_riscv_sim, which is what test/cosim.py checks the binary against before
# executing it. Line 1 is what makes a version bump re-fetch instead of
# silently reusing whatever landed first -- the old rule keyed on the binary
# merely existing, so the first binary to arrive was executed forever.
SAIL_STAMP := $(SAIL_RISCV_DIR)/.sail-pin
SAIL_PIN   := $(SAIL_RISCV_VERSION) $(SAIL_ASSET) $(SAIL_SHA256)

# Fail closed, the way formal/pin.mk's HEAD check does: a tree on disk at
# anything other than the pin stops the target that would execute it, rather
# than being silently reused. `sail-setup` is exempt because it is the way out
# of this state (pin.mk exempts `clean` for the same reason) -- it re-fetches
# on a pin mismatch.
#
# Scoped to the goals that actually run the binary. A stale tools/sail must not
# break `make test`: co-simulation is opt-in (ADR-0032) and a guard on it that
# could fail the suite would have made it a gate by the back door.
ifneq ($(filter cosim-run,$(MAKECMDGOALS)),)
ifneq ($(wildcard $(SAIL_SIM_BIN)),)
SAIL_PIN_ON_DISK := $(shell sed -n 1p $(SAIL_STAMP) 2>/dev/null)
ifneq ($(SAIL_PIN_ON_DISK),$(SAIL_PIN))
$(error $(SAIL_RISCV_DIR) was fetched under '$(SAIL_PIN_ON_DISK)', not the pin \
  '$(SAIL_PIN)'. Re-fetch it with: make sail-setup)
endif
endif
endif

# Download to a file, verify the digest, audit the member paths, and only then
# extract -- the previous recipe piped curl straight into tar, so bytes from
# the network were unpacked before anything could reject them. `--version` runs
# last, after verification, which is the order the comment above claims.
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
# load-bearing for correctness, not just elaboration: changing a rule in the
# sanitizer changes the oracle.
#
# The three rules, and why each exists, live in test/sanitize_monitor.py.
# They were an inline `sed` chain here until the third one (the !spec_trap
# gate) arrived: that rule is a structural insert across a 45-line span, not
# a line substitution, and the script additionally asserts a site count per
# rule so a pin bump that changes the generator's output fails loudly instead
# of silently shipping an unapplied sanitizer. `diff test/monitor.v
# test/monitor.sim.v` is eight lines; if it stops being readable at a glance,
# fix the generator upstream instead.
test/monitor.sim.v: test/monitor.v test/sanitize_monitor.py
	python3 test/sanitize_monitor.py $< > $@

# $(RISCV_FORMAL_MACROS) turns on the rvfi_* ports and their driving logic
# throughout rtl/ and test/testbench.v's `monitor` instance (ADR-0006);
# only RISCV_FORMAL itself is actually read by rtl/ or test/testbench.v
# today, the rest ride along for consistency with testbench.vvp above.
# test/monitor.sim.v (not test/monitor.v) supplies the `monitor` module
# here; see its rule above for why.
test/rtl.cc: rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v test/monitor.sim.v
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

# ---------------------------------------------------------------------------
# Structural lint (.svlint.toml) -- the THIRD SystemVerilog frontend.
#
# svlint parses with sv-parser, which is neither yosys's nor iverilog's
# frontend. Read .svlint.toml's header for why this gate exists and why nearly
# every rule it enables reports zero findings today; read the rule comments
# there before changing any of them.
#
# rtl/ only. test/ benches use delays, `$display` and `initial` blocks these
# rules were never meant for.
# ---------------------------------------------------------------------------

# Pinned the way formal/pin.mk pins riscv-formal and the sail spike above pins
# sail-riscv, and for the same reason (ADR-0013): this fetches a 45 MB
# EXECUTABLE off the network. A GitHub release asset is MUTABLE, so a version
# on its own pins a FILENAME, not BYTES -- the SHA-256 beside each asset name
# is what pins the bytes, and it is checked before anything is extracted, let
# alone run. Bumping the version means re-pinning every digest; that friction
# is the point.
#
# `override` + an origin check so this cannot be redirected from a script or a
# CI job. A supply-chain control, not a knob.
ifneq ($(filter command line environment,$(origin SVLINT_VERSION)),)
$(error SVLINT_VERSION cannot be set from the command line or the environment: \
  it pins bytes this repo executes. Change it in the Makefile, together with \
  the SHA-256 digests below it)
endif
override SVLINT_VERSION := 0.9.5

# Three-part, so a branch name or a moving tag cannot be written here by
# accident.
ifeq ($(shell printf '%s' '$(SVLINT_VERSION)' | grep -cE '^[0-9]+\.[0-9]+\.[0-9]+$$'),0)
$(error SVLINT_VERSION must be a three-part release version like 0.9.5, not a \
  branch, a moving tag or a range: '$(SVLINT_VERSION)')
endif

# `shasum -a 256` over each 0.9.5 asset, downloaded fresh and cross-checked
# against the per-asset digest the GitHub releases API reports. Upstream ships
# no Linux aarch64 build, so that host has no line here and `lint-setup` fails
# closed on it rather than fetching something it cannot verify (use `brew` or
# `cargo install` there).
#
# The version is spelled out in each variable NAME, not interpolated. That is
# deliberate: bumping SVLINT_VERSION without re-pinning makes the lookup below
# resolve to empty, and `lint-setup` refuses to fetch. A digest that silently
# carried over to a new version would be a control that reads as protection
# while providing none (ADR-0013).
SVLINT_SHA256_svlint-v0.9.5-x86_64-lnx  := 0bbb3850b8ef604d7ccf25c2b0d2a751154ac2e18b2a12753ae1648f237a8ceb
SVLINT_SHA256_svlint-v0.9.5-x86_64-mac  := 53838f356862b6492777347999ccf44c1b44bc78f51cb032759b9e17bd213519
SVLINT_SHA256_svlint-v0.9.5-aarch64-mac := d032be600f0ee04130e0663daa05da3cc562d3d34bbc4305d6b70cb99310c6df

SVLINT_ASSET_Linux_x86_64  := svlint-v$(SVLINT_VERSION)-x86_64-lnx
SVLINT_ASSET_Darwin_x86_64 := svlint-v$(SVLINT_VERSION)-x86_64-mac
SVLINT_ASSET_Darwin_arm64  := svlint-v$(SVLINT_VERSION)-aarch64-mac

SVLINT_DIR   := tools/svlint
SVLINT_ASSET := $(SVLINT_ASSET_$(shell uname -s)_$(shell uname -m))
SVLINT_SHA256 := $(SVLINT_SHA256_$(SVLINT_ASSET))

# Prefer whatever is on PATH (brew, cargo, the distro) and fall back to the
# tree `lint-setup` unpacks, so a developer who already has svlint installed
# never pays for the download and CI needs no extra PATH plumbing.
SVLINT ?= $(shell command -v svlint 2>/dev/null || echo ./$(SVLINT_DIR)/bin/svlint)

# TWO passes, and both are load-bearing. Roughly a fifth of rtl/ sits behind
# `ifdef RISCV_FORMAL` (rvfi_* capture in structs/decoder/accessor/writeback);
# svlint's preprocessor drops those blocks unless the macros are defined, so a
# single undefined pass would leave the RVFI instrumentation unlinted --
# exactly the code ADR-0020 says must stay write-only with respect to the core
# and that nothing else structurally checks. Same macro list as both sim legs
# ($(RISCV_FORMAL_MACROS), defined at the top of this file) so the three
# cannot drift on what "RVFI is live" means.
#
# `-i rtl` is required, not decorative: svlint resolves `include "structs.v"`
# relative to the include path only, and without it every module that includes
# it fails to parse. Neither yosys nor iverilog surfaces that.
#
# `--github-actions` when GITHUB_ACTIONS is set turns findings into
# ::error file=...,line=... annotations on the PR diff; plain `-1`
# (one finding per line) otherwise.
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

# Download to a file, verify the digest, audit the member paths, and only then
# extract -- the same order `sail-setup` above uses, and for the same reason:
# bytes from the network must not be unpacked before anything can reject them.
# tools/svlint is gitignored; never commit the binary.
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

# Six small benches. Four landed in `eb18320` and `a4662a2` with no runner
# (rtl/executor.v, rtl/memory.v, rtl/decoder.v, rtl/regfile.v respectively —
# regfile_tb.v covers the write-through bypass and x0 semantics, the single
# most load-bearing change in the project, and was verified by hand via
# iverilog+vvp before this rule existed to run it in CI). csr_tb is the fifth,
# covering rtl/csrs.v's read mux, the address set its `implemented` output
# accepts (that output decides whether a Zicsr encoding is a recognised
# instruction at all), and its WARL masks — which the riscv-formal ladder
# structurally cannot check, since rvfi_csrw_check.sv has no WARL model; see
# formal/checks.cfg's [csrs] note. The sixth,
# monitor_tb, is not an RTL bench at all: it drives the sanitized monitor
# (test/monitor.sim.v) directly with hand-built RVFI retires, because that
# file is the oracle both sim legs check against and nothing else exercises
# it on inputs whose correct verdict is known independently of the core
# (ADR-0019). Compiled and run straight through iverilog/vvp; each bench
# $fatal(1)s on a mismatch and $finish (exit 0) on success, so vvp's own exit
# code is the pass/fail signal — no output-parsing needed. A separate target
# from `test` (that one is the ADR-0007 cxxrtl regression gate over
# test/asm/*.S; these are unit benches with their own, unrelated pass/fail
# mechanism) but `test` depends on it so one `make test` still catches
# everything.
#
# `set -e` comes FIRST, before mktemp and before the trap (ADR-0035). The
# other order installed the cleanup trap on an unchecked $$tmp: a failed
# mktemp left it empty, every -o path collapsed to the filesystem root, and
# the trap then removed nothing.
.PHONY: test-units
test-units: test/monitor.sim.v
	@set -e; \
	tmp=$$(mktemp -d "$${TMPDIR:-/tmp}/test-units.XXXXXX"); \
	test -n "$$tmp" -a -d "$$tmp"; \
	trap 'rm -rf "$$tmp"' EXIT; \
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
	vvp $$tmp/regfile_tb.vvp; \
	echo "== csr_tb =="; \
	iverilog -I./rtl/ -g2012 -o $$tmp/csr_tb.vvp rtl/structs.v rtl/csrs.v test/csr_tb.v; \
	vvp $$tmp/csr_tb.vvp; \
	echo "== monitor_tb =="; \
	iverilog -g2012 -o $$tmp/monitor_tb.vvp test/monitor.sim.v test/monitor_tb.v; \
	vvp $$tmp/monitor_tb.vvp

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

riscv.json: rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v rtl/imemory.v rtl/memory.v | rtl/rom.mem
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
	@# Bumping SAIL_RISCV_VERSION and re-running `make sail-setup` re-fetches
	@# on its own now -- the pin is recorded in tools/sail/.sail-pin and
	@# compared, so this comment is no longer the only thing making that true.
	@# `rm -rf tools/sail` still works as the blunt instrument.
	@# NOT tools/svlint either, and for the same reason: a network fetch that
	@# `clean` was never asked to rebuild. `make lint-setup` re-fetches
	@# unconditionally, so `rm -rf tools/svlint` is the blunt instrument there.

# The riscv-formal clone rule and its pin guard live in formal/pin.mk, included
# above, so the root and formal/ Makefiles cannot drift apart on it.
