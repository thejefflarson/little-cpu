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

# The RTL both sim legs elaborate, named ONCE. This list used to be written out
# per recipe, and .github/workflows/ci.yml's `elaborate` job carried a fourth
# hand-maintained copy under a comment asserting that it "reads the same
# sources, so the two cannot diverge on what they elaborate". They diverged the
# moment ADR-0054 put rtl/imemory.v and rtl/memory.v on the graph: the gate went
# on elaborating a testbench whose memory instances resolved to nothing, and
# reported every bit of `imem_data2` as undriven. The gate was right and its
# file list was stale. Nothing may spell this list out again -- the `elaborate`
# job calls `make elaborate-strict` (below) so there is one copy to update.
SIM_RTL_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
                rtl/fetcher.v rtl/imemory.v rtl/memory.v rtl/regfile.v rtl/writeback.v \
                rtl/littlecpu.v

testbench.vvp: $(SIM_RTL_SRCS) rvfi_macros.vh test/testbench.v test/monitor.sim.v
	iverilog -I./rtl/ -DICARUS $(addprefix -D,$(RISCV_FORMAL_MACROS)) -g2012 -o $@ $^

# ADR-0007: iverilog is the waveform leg, not cxxrtl (`./sim` now requires
# --rom/--ram/--cycles and was never a VCD demo — see the `sim:` rule
# above). testbench.vvp already `$dumpfile`s/`$dumpvars`s under `ifdef
# ICARUS` (test/testbench.v) and already carries test/monitor.sim.v, so this
# waveform is also a self-checking per-retire run, not just a raw trace. It
# runs the fixed increment-loop program test/testbench.v writes into the ROM's
# two banks from an `ifdef ICARUS` block (ADR-0054) for 200 cycles -- there's no
# --rom flag on this leg, unlike ./sim's.
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
# Sail co-simulation (docs/adr/0032, docs/adr/0039) -- deliberately opt-in.
#
# Nothing below is reachable from `make test`, `make test-units` or CI's
# required set. The whole point of keeping it off the default path is that
# `make test` must still work on a machine with no Sail installed, and an
# experiment must not quietly become a merge gate. Run it by hand:
#
#     make sail-setup     # once: fetch, verify and unpack the pinned release
#     make cosim          # build the architectural-state tracer
#     make cosim-run      # run it on one program (PROG=add.S by default)
#     make cosim-suite    # run the whole suite, graded against the baseline
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
# `make test` or CI. That precondition is met here. ADR-0039 then landed the
# `tohost` doubleword and the `test/COSIM_EXPECTED_FAIL` baseline behind
# `make cosim-suite`, and co-simulation still stays off every default path:
# `make test`, `make test-units` and CI's required set do not reach any of it.
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
ifneq ($(filter cosim-run cosim-suite,$(MAKECMDGOALS)),)
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

# The whole suite under co-simulation, graded by set equality in BOTH
# directions against test/COSIM_EXPECTED_FAIL -- the same contract `make test`
# applies to test/EXPECTED_FAIL (ADR-0014, ADR-0035). See docs/adr/0039.
#
# Deliberately NOT a prerequisite of `test`, `test-units` or anything CI
# requires (ADR-0032). It needs a Sail install that `make test` must keep
# working without, and the moment it became required it would stop being the
# opt-in experiment it was accepted as. The regfile-to-block-RAM change gates
# on it by pasting its output into the PR, not by branch protection.
#
# The fourth argument is the suite manifest -- test/OBSERVED_FLOOR, the same
# file `make test` grades retire counts against. Both legs read the same one on
# purpose: two lists of what the suite is could come to disagree, and then the
# interesting failure would be about the lists rather than about the core.
.PHONY: cosim-suite
cosim-suite: cosim
	./test/run_cosim.sh ./cosim test/asm test/COSIM_EXPECTED_FAIL test/OBSERVED_FLOOR

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
test/rtl.cc: $(SIM_RTL_SRCS) rvfi_macros.vh test/testbench.v test/monitor.sim.v
	yosys -p 'read_verilog -sv $(addprefix -D ,$(RISCV_FORMAL_MACROS)) $^; hierarchy -top testbench; write_cxxrtl $@'

# The `elaborate` gate (.github/workflows/ci.yml). The SAME SOURCES as
# test/rtl.cc above -- guaranteed by the variable rather than asserted by a
# comment -- with `proc; opt_clean; check` ahead of write_cxxrtl. `check` is
# what actually reports an undriven wire; the bare recipe above elaborates one
# with zero diagnostic output and exit 0, which is the whole reason a strict
# pipeline exists.
#
# DELIBERATELY UNLIKE test/rtl.cc, this reads no `-D RISCV_FORMAL*` and no
# test/monitor.sim.v: the gate elaborates the plain functional configuration,
# so `ifdef RISCV_FORMAL` shadow logic and the generated monitor are out of
# frame. That predates this target and is preserved, not inherited by accident.
# It is also why the gate needs no riscv-formal clone to run.
#
# It lives here rather than in the workflow so the file list cannot drift from
# the build it claims to mirror. The promotion-to-error POLICY stays in the
# workflow, where it belongs: this target elaborates and reports; the gate
# decides which warnings are fatal.
ELABORATE_STRICT_OUT ?= /tmp/elaborate-strict.cc

# One line on purpose. A backslash-continuation inside the single-quoted script
# is NOT removed by the shell -- single quotes make it literal -- so yosys reads
# the backslash as a command and dies with "No such command: \". It survived a
# local run and failed on the gate, which is the wrong way round to find out.
.PHONY: elaborate-strict
elaborate-strict: $(SIM_RTL_SRCS) test/testbench.v
	yosys -p 'read_verilog -sv $(SIM_RTL_SRCS) test/testbench.v; hierarchy -top testbench; proc; opt_clean; check; write_cxxrtl $(ELABORATE_STRICT_OUT)'

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

# Seven small benches. Four landed in `eb18320` and `a4662a2` with no runner
# (rtl/executor.v, rtl/memory.v, rtl/decoder.v, rtl/regfile.v respectively —
# regfile_tb.v covers the write-through bypass and x0 semantics, the single
# most load-bearing change in the project, and was verified by hand via
# iverilog+vvp before this rule existed to run it in CI). csr_tb is the fifth,
# covering rtl/csrs.v's read mux, the address set its `implemented` output
# accepts (that output decides whether a Zicsr encoding is a recognised
# instruction at all), and its WARL masks — which the riscv-formal ladder
# structurally cannot check, since rvfi_csrw_check.sv has no WARL model; see
# formal/checks.cfg's [csrs] note. imem_tb is the sixth (ADR-0054): it walks
# rtl/imemory.v's bank select at every word index and both parities against a
# FLAT reference array, plus the range decode at the last word and past it --
# corners the .S suite cannot reach, since real programs only produce the
# alignments they happen to produce. The seventh,
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
#
# THE BENCH LIST IS ASSERTED AGAINST THE TREE, not merely written out. Six
# iverilog+vvp invocations used to be spelled out in the recipe with nothing
# tying that list to test/*_tb.v, so a seventh bench could land, be committed,
# pass review and never run — and the gate would report six benches green
# exactly as before. That is the same hole test/OBSERVED_FLOOR closes for the
# .S suite and formal/EXPECTED_CHECKS closes for the ladder: a verdict cannot
# report on something that never ran. UNIT_BENCHES is the declaration,
# `check-unit-benches` compares it against `ls test/*_tb.v` in BOTH directions,
# and the recipe below is DRIVEN by it — so the list cannot be satisfied by
# adding a name without also giving the bench its sources.
UNIT_BENCHES := exec_tb mem_tb imem_tb decoder_tb regfile_tb csr_tb accessor_tb monitor_tb

# Per-bench RTL. monitor_tb is not an RTL bench at all: its source is the
# sanitized monitor, which is why it names a file under test/ rather than rtl/.
UNIT_BENCH_SRC_exec_tb     := rtl/structs.v rtl/executor.v
UNIT_BENCH_SRC_mem_tb      := rtl/memory.v
UNIT_BENCH_SRC_imem_tb     := rtl/imemory.v
UNIT_BENCH_SRC_decoder_tb  := rtl/structs.v rtl/decoder.v
UNIT_BENCH_SRC_regfile_tb  := rtl/regfile.v
UNIT_BENCH_SRC_csr_tb      := rtl/structs.v rtl/csrs.v
UNIT_BENCH_SRC_accessor_tb := rtl/structs.v rtl/accessor.v
UNIT_BENCH_SRC_monitor_tb  := test/monitor.sim.v

# `present` is derived from the tree inside the recipe rather than from a
# $(wildcard) at parse time: make caches directory contents, and a check whose
# idea of what is on disk can be stale is a check that can be wrong in the one
# direction that matters.
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

# THE GRADING LAYER'S OWN RED DIRECTION. Every graded comparison in the two
# suite runners, the manifest check, the co-simulation comparison, the monitor
# sanitizer and the two formal gates is forced to FAIL here, and required to
# fail for the reason it was written for. Five of this repo's recorded defects
# lived in that layer and every one was in a script (ADR-0037 §4, ADR-0040,
# ADR-0033 gap 1, and the sanitizer's rule 3); the class is the comparison
# whose failure path was never executed.
#
# It is a prerequisite of `test` rather than a target of its own on purpose.
# A gate reached by no automation reads like coverage and is not — CLAUDE.md
# records `make waves` and `make -C formal all` as exactly that — and hanging
# it off `test` puts it in CI's required job with no workflow change and no
# branch-protection change (ADR-0036 makes the latter a human action anyway).
# It needs no RISC-V toolchain, no Sail, no yosys and no sby, so it cannot
# narrow where `make test` runs.
.PHONY: probe-gates
probe-gates:
	@./test/probe_gates.sh

# Assembles every test/asm/*.S (rv32im_zicsr, -nostdlib, ADR-0008's memory
# map), runs each under `sim`, and checks the pass/fail table against
# test/EXPECTED_FAIL — the sprint-1 baseline. Exits 0 only when the actual
# failure list matches that file exactly, so this is a working regression
# gate today, against the current (still-broken, see CLAUDE.md) core.
.PHONY: test
test: sim test-units probe-gates
	@./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL test/OBSERVED_FLOOR

# ---------------------------------------------------------------------------
# `make fit` -- the repo's one and only area measurement (ADR-0038).
#
# Area is measured in *nextpnr logic cells*, never in yosys cell counts, and
# nothing on this path prints a yosys number. A logic cell holds one LUT4 AND
# one flip-flop, but a DFF whose D input is not the output of its co-located
# LUT consumes a whole cell on its own -- over a thousand of this design's
# cells are exactly that, and an `SB_LUT4` count is structurally blind to it.
# Two planning estimates were wrong in opposite directions because they counted
# LUT4s: one said 102% where the truth was 126%, the other undercounted a
# saving by more than half. Hence the synthesis log below goes to a file.
#
# The top is `littlecpu` with its memories external, not `littlesoc`: the SoC's
# memories are placeholders whose real implementation will be SPRAM and will
# not consume logic cells. This replaces the `riscv.json`/`riscv.asc`/`timing`
# targets that stood here, which synthesised `littlesoc` -- whose only outputs
# are the flash pins, so yosys deleted the entire core and place-and-route
# reported *4 LCs, 0%*. They measured an empty design and looked like a metric.
#
# WHY THIS TARGET TOLERATES A FAILED PLACEMENT, ON PURPOSE:
# `littlecpu` with memories external presents 231 `SB_IO` against sg48's 39,
# so nextpnr ALWAYS errors out -- after printing the utilisation table. Even a
# configuration using 76% of the part fails, and it fails on an `imem_data2`
# pad, not on logic cells. A top with realistic IO means a real pinout, which
# means the SoC memory system, which is out of scope here. So a `make fit` that
# required successful placement would never run at all, which is worse than
# having no metric because it would look like one (ADR-0038 decision 1a). The
# measurement is the utilisation nextpnr prints *before* it attempts placement.
#
# `icetime` IS DELIBERATELY OUT OF SCOPE, for the same reason: it needs an
# `.asc`, which needs a completed placement, which needs that real pinout. Fmax
# stays *declared* at 12 MHz and unmeasured (ADR-0038 decision 2); raising it
# means breaking invariant 1 or invariant 6 and needs its own ADR. That is also
# why the `pll.v`/`icepll` rule that fed off `timing` is gone.
#
# The contract is therefore inverted from a normal build, and the check below
# is the whole point of the target:
#
#   nextpnr ran, printed a table, then failed to place -> EXPECTED, exit 0
#   nextpnr absent, crashed, or printed no table       -> exit NONZERO
#
# A missing binary or a truncated JSON must not yield a green run with no
# number in it. nextpnr's exit status cannot tell those apart -- it is nonzero
# either way -- so the presence of the utilisation table is what decides.
#
# The whole table is printed, not just the logic-cell line: `SB_GB` is already
# at 8/8, so a global-buffer overflow is a placement failure that no logic-cell
# ratchet would ever predict.
# ---------------------------------------------------------------------------
FIT_SRCS := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v rtl/executor.v \
            rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v

fit.json: $(FIT_SRCS)
	@echo 'yosys: synthesising littlecpu for ice40 (log: fit.synth.log)'
	@yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlecpu -json $@' \
	  > fit.synth.log 2>&1 || { tail -40 fit.synth.log; exit 1; }

# THE RATCHET (ADR-0042). `make fit` was report-only while the core did not
# fit at all; it does now, and a number nothing defends drifts back.
#
# WHY THE BUDGET IS NOT THE MEASUREMENT. `littlecpu` measures 4208 logic cells
# at ADR-0052, under CI's pinned OSS CAD Suite -- which is the number to quote,
# and the reason to say which toolchain took it: the same commit measures 4187
# under a local Homebrew yosys, 21 cells apart on the synthesiser build alone.
# (ADR-0042's 4236 was likewise a local number.) The budget is looser than that,
# because the measurement is not stable to the cell: edits that synthesise to
# identical hardware -- renaming a wire, reordering two independent
# assignments, hoisting a struct field into a named signal -- move it by tens
# of cells, since ABC's result depends on the order it sees the netlist in. A
# ratchet pinned to the measurement of the day would go red on changes that
# alter nothing, and the only way to clear it would be to raise the number,
# which is how a ratchet becomes a rubber stamp. The 4236 -> 4208 drift is
# itself an instance, and 4208 vs 4187 is a second axis of the same problem:
# nobody set out to move either number.
#
# 4400 is 192 cells of headroom, about 4.6% of the measurement and more than
# three times the observed noise band. A change that trips it has grown the
# core by more than any resynthesis artifact can account for, and the right
# response is to find out why -- not to edit this line. Lowering it after a
# real reduction is always welcome; raising it needs a reason in the commit
# message.
#
# It is NOT set at the part's 5280. Fitting is the floor, not the goal: 80%
# with the SoC memory system still to come is the number worth defending.
#
# LOWERED FROM 4400 AT ADR-0054, after a reduction that is real rather than
# resynthesis drift. Lifting the PC redirects out of rtl/decoder.v's publish
# block replaced six independent `cond ? pc+imm : pc+inc` writes to `pc` with
# one `branch_taken` and one priority mux, and it measures **4187 -> 3875 logic
# cells on this branch, both under the SAME local Homebrew yosys**: 312 cells,
# six times the +/-50 churn floor, and measured as a delta rather than quoted
# across toolchains (which is the trap ADR-0052 records). Expect roughly 3896
# under CI's pinned OSS CAD Suite, on that +21-cell axis; the `fit` job is where
# the number that counts is taken.
#
# 4100 keeps ~204 cells of headroom over the local measurement, the same
# proportion 4400 kept over 4208 and still well clear of both noise axes.
FIT_MAX_LC := 4100

# The table-presence check and the ratchet both live in soc/fit_report.py, not
# here, so test/probe_gates.sh can drive them against a fixture log instead of
# a second copy of this parsing (ADR-0053; the pattern soc/timing_split.py
# already uses for `make soc-timing`'s ratchet).
.PHONY: fit
fit: fit.json
	@nextpnr-ice40 --up5k --package sg48 --json $< --pcf-allow-unconstrained \
	  > fit.log 2>&1 || true
	@python3 soc/fit_report.py fit.log --max-lc $(FIT_MAX_LC)

# ---------------------------------------------------------------------------
# `make soc-timing` -- the SoC place-and-route and the FIRST REAL TIMING NUMBER
# this project has ever had (ADR-0054).
#
# DELIBERATELY NOT `make fit`, AND IT MEASURES A DIFFERENT THING. `fit` is the
# core's area ratchet: `littlecpu` with its memories external, tolerating a
# failed placement because 231 `SB_IO` against sg48's 39 can never place
# (ADR-0038 decision 1a). This target is the whole SoC -- core plus a block-RAM
# ROM plus an SPRAM data RAM plus four pins -- and it MUST place, because
# `icetime` reads an `.asc` and there is no `.asc` without a completed
# placement. The two numbers are separate on purpose and must not be merged: the
# SoC's logic-cell count includes the ROM's depth mux, the RAM's range decode
# and the LED taps, none of which are the core.
#
# The contract is therefore the ordinary one, inverted back from `fit`'s:
#
#   nextpnr places, icetime reports a path   -> exit 0, the numbers are printed
#   anything fails                           -> exit NONZERO
#
# WHAT THE NUMBER DOES NOT DESCRIBE, since a timing figure travels further than
# its caveats: it is `icetime`'s static estimate for one placement of one build
# of one program's ROM contents, at the default (worst-case) corner, on a part
# whose routing dominates. ADR-0038 declares Fmax at 12 MHz as an INTENT and
# ADR-0054 does not move it in either direction -- reconciling a measurement
# with an intent is a decision, not a consequence.
#
# The ROM image is a real program, assembled with the same toolchain and the
# same link script `make test` uses, so this needs the RISC-V cross compiler.
# That is why it is a hand-run target and not a CI job. Override the program
# with `make soc-timing SOC_PROG=lw.S`.
# ---------------------------------------------------------------------------
SOC_PROG      ?= add.S
SOC_ROM_WORDS := 2048
# The hard-block census the design is DECLARED to produce. Both are properties
# of the RTL, not of placement, so they are exact rather than budgeted the way
# FIT_MAX_LC is: 2 SPRAM for the 64 KB data RAM, and 16 EBR for the 8 KB banked
# ROM plus 4 for rtl/regfile.v.
SOC_EXPECT_SPRAM := 2
SOC_EXPECT_EBR   := 20

SOC_SRCS      := rtl/structs.v rtl/accessor.v rtl/csrs.v rtl/decoder.v \
                 rtl/executor.v rtl/fetcher.v rtl/imemory.v rtl/memory.v \
                 rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v

# The ROM image, de-interleaved into rtl/imemory.v's two banks. soc/rom_banks.py
# refuses to truncate a program that does not fit -- a ROM ceiling breached is a
# finding about the part, not something to silently cut in half.
#
# PHONY on purpose, so `soc.json` resynthesises every run. The image depends on
# SOC_PROG, which make cannot see a change to, and a stale ROM would make the
# measurement describe a program nobody asked for. This target is hand-run and
# the whole flow is ~33 s; correctness is worth more than incrementality here.
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

# nextpnr's exit status is NOT the signal here, and tolerating it is a decision
# rather than a convenience. It defaults to a 12 MHz target on ice40 and EXITS
# NONZERO when the design misses it -- which this one does (ADR-0054: 11.70 MHz
# by nextpnr's own analysis, 11.30 MHz by icetime). The placement itself
# succeeded and the `.asc` is written before that check runs, so throwing it away
# would mean no `icetime` report at all, i.e. no measurement, because the design
# is 6% slow. The graded conditions are below: the `.asc` exists, nextpnr printed
# a utilisation table, and the frequency clears SOC_MIN_MHZ.
#
# `.DELETE_ON_ERROR` at the top of this file is why the `||` matters: without
# tolerating the status, make deletes the `.asc` nextpnr just wrote.
#
# `SOC_SEED=<n>` places the same netlist differently. One placement of one build
# is a sample, not a measurement: ADR-0057 measured the spread across four
# placements at 1-2% for this design and ADR-0058 saw 87.43 to 88.51 ns on
# unmodified `main`, which is wider than several changes worth arguing about.
# Compare distributions, not single runs.
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

# A REGRESSION RATCHET, SET BELOW THE MEASUREMENT -- NOT ADR-0038's 12 MHz.
# The design measures 11.30 MHz by icetime and does not close 12 (ADR-0054).
# Pinning this at 12 would be a gate that is red on arrival, which is a gate
# nobody keeps; pinning it at 11.30 would be red on the next edit, and that is
# not a guess.
#
# THE CHURN AXIS IS MEASURED, AND IT IS BIGGER THAN `make fit`'s. Two logically
# identical spellings of rtl/memory.v's write/read arms -- a module that is not
# on the critical path at all -- give 88.51 ns and 91.67 ns, 41 and 53 logic
# levels, on 11 logic cells' difference in the netlist. **3.6%**, from an edit
# that changes no hardware, because placement redistributes. Each figure is
# reproducible run to run (nextpnr is seeded); it is the EDIT the number is
# unstable under, exactly as ADR-0038 found for logic cells at ~1.2%.
#
# So the headroom is set at roughly four times that band, the same ratio
# FIT_MAX_LC keeps: 10.0 MHz is 11.5% under the measurement. A change that trips
# it has slowed the fetch->decode->next-PC loop by more than placement churn can
# account for.
#
# RAISING THIS AFTER A REAL IMPROVEMENT IS ALWAYS WELCOME. Lowering it needs a
# reason in the commit message, and closing the gap to 12 MHz is a DESIGN
# decision (it means shortening the loop invariant 1 puts in one cycle), not
# something to buy by editing this line.
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

# The riscv-formal clone rule and its pin guard live in formal/pin.mk, included
# above, so the root and formal/ Makefiles cannot drift apart on it.
