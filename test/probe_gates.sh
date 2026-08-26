#!/bin/bash
# Forces every graded comparison in this repo's grading scripts to FAIL, and
# asserts that each one goes red for the reason it was written for. The defect
# class is the comparison whose failure path was never executed, and this repo
# has shipped five of them.
#
# Usage: probe_gates.sh          # run every probe; exit 0 only if all pass
#
# A probe pins the exit STATUS and a fragment of the DIAGNOSTIC. Status alone is
# nearly worthless: a script that exited 1 on a mistyped fixture path would
# satisfy every exit-status probe here while demonstrating nothing. Every group
# also carries a control, because a grader degenerated into `exit 1` would
# otherwise pass the lot.
#
# Hermetic: no RISC-V toolchain, no `sim`, no Sail, no sby -- but the
# test/zkt_isolation_test.py group elaborates rtl/decoder.v with yosys, once per
# fixture, so this file needs yosys and fails loudly rather than silently on a
# machine without it. It is otherwise all fork and no work, so the wall time is
# the host's property rather than this file's.
#
# Four failure paths need the elaborated design or the pinned clone and are
# demonstrated by hand instead: test/cxxrtl.cc's exits 4 and 5, test/cosim.cc's
# divergence check, and genchecks-audit.py's three set equalities.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)

# Pinned as a literal: a probe that is deleted, or that stops being reached by
# an early `return`, would otherwise cut this file's coverage while it kept
# printing a green summary.
PROBES_EXPECTED=550

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-probe.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory under ${TMPDIR:-/tmp}." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

probes=0
failed=0
group=""

# `mktemp -d` rather than a counter because fixtures are built inside `$(...)`,
# a subshell: a counter would never advance in the parent, so every fixture
# would land on top of the last one and the probes would still report something.
new_case() {
  mktemp -d "$tmp/case.XXXXXX"
}

group_started=0

begin_group() {
  if [ -n "$group" ]; then
    printf '  (%ss)\n' "$((SECONDS - group_started))"
  fi
  group=$1
  group_started=$SECONDS
  echo
  echo "== $group"
}

# probe <label> <expected-exit> <expected-text> <shell-snippet>
probe() {
  local label=$1 want_exit=$2 want_text=$3 snippet=$4
  probes=$((probes + 1))
  local out rc
  set +e
  out=$(eval "$snippet" 2>&1)
  rc=$?
  set -e
  local why=""
  # A here-string, not a pipe. `grep -q` exits at the first match and closes the
  # pipe, so against a large output the writer dies of SIGPIPE -- which under
  # pipefail becomes the pipeline's status, making a MATCHING probe report no
  # match.
  if [ "$rc" -ne "$want_exit" ]; then
    why="exited $rc, expected $want_exit"
  elif ! grep -qF -- "$want_text" <<< "$out"; then
    why="output does not contain $want_text"
  fi
  if [ -z "$why" ]; then
    printf '  ok   %s\n' "$label"
  else
    printf '  RED  %s\n     -> %s\n' "$label" "$why" >&2
    printf '%s\n' "$out" | sed -e 's|^|        |' >&2
    failed=1
  fi
}

# Written once and shared by every fixture, as are the scratch copies of the
# scripts under test: macOS re-scans an executable the first time it is exec'd
# after being written, so a probe that created its own stub tree measured
# 1.5-3.3s of wall against 0.06s of user time.

make_toolchain_stubs() {  # $1 = bin dir
  local bin=$1
  mkdir -p "$bin"
  cat > "$bin/riscv64-elf-gcc" <<'STUB'
#!/bin/sh
# Stands in for the cross compiler. Writes a non-empty file at -o and succeeds,
# so run_tests.sh / cosim.py get past assembly without a toolchain installed.
[ -n "${STUB_CC_WARN:-}" ] && echo "stub: assembler warning" >&2
out=; prev=
for a in "$@"; do
  if [ "$prev" = "-o" ]; then out=$a; fi
  prev=$a
done
[ -n "$out" ] && echo stub-elf > "$out"
exit ${STUB_CC_EXIT:-0}
STUB
  cat > "$bin/riscv64-elf-objcopy" <<'STUB'
#!/bin/sh
# The last argument is the output image; run_tests.sh and cosim.py both call it
# that way. STUB_OBJCOPY_FAIL stands in for a full disk or a binutils too old for
# --verilog-data-width. STUB_OBJCOPY_EMPTY is the quiet one: exit 0 having
# written nothing, which still parses and makes `tohost` read zero.
for out; do :; done
if [ -n "${STUB_OBJCOPY_FAIL:-}" ]; then
  echo "stub objcopy: refusing" >&2
  exit 1
fi
if [ -n "${STUB_OBJCOPY_EMPTY:-}" ]; then
  : > "$out"
  exit 0
fi
printf '@00000000\n13 00 00 00\n' > "$out"
exit 0
STUB
  chmod +x "$bin/riscv64-elf-gcc" "$bin/riscv64-elf-objcopy"
}

make_sim_stub() {  # $1 = path
  cat > "$1" <<'STUB'
#!/bin/sh
# Stands in for ./sim (test/cxxrtl.cc). Reproduces its output contract -- one
# `RETIRES <n> SPEC-CHECKED <m>` line and one verdict line -- and its exit
# statuses, so run_tests.sh's grading of them can be probed without the
# elaborated design.
[ -z "${STUB_SIM_NOCOUNTS:-}" ] && \
  echo "RETIRES ${STUB_SIM_RETIRES:-10} SPEC-CHECKED ${STUB_SIM_SPEC:-10}"
# The `--stalls` line. STUB_SIM_UNATTR stands in for a stall reason the report
# does not name; STUB_SIM_SKEW breaks the sum without touching any column, which
# is what a renamed field would do.
stalls=
for a in "$@"; do [ "$a" = "--stalls" ] && stalls=1; done
if [ -n "$stalls" ] && [ -z "${STUB_SIM_NOSTALLS:-}" ]; then
  unattr=${STUB_SIM_UNATTR:-0}
  echo "STALLS cycles=$((20 + unattr + ${STUB_SIM_SKEW:-0})) issue=10 divider=0" \
       "atomic=0 hazard=10 serialize=0 operand=0 fetch=0 bus=0 region=0" \
       "unattributed=$unattr lsissue=4 lsedge=2 lsbypass=1"
fi
case ${STUB_SIM_EXIT:-0} in
  0) echo "PASS" ;;
  1) echo "FAIL 7" ;;
  4) echo "RVFI monitor error 101 at cycle 12" >&2 ;;
esac
exit ${STUB_SIM_EXIT:-0}
STUB
  chmod +x "$1"
}

make_cosim_py_stub() {  # $1 = path
  cat > "$1" <<'STUB'
#!/bin/sh
# Stands in for test/cosim.py as run_cosim.sh drives it: a --check-setup probe
# and a per-program run that prints the one machine-readable COSIM-STATUS line
# the suite runner contracts on.
case "$1" in
  --check-setup) echo "stub setup"; exit ${STUB_SETUP_EXIT:-0} ;;
esac
[ -z "${STUB_COSIM_SILENT:-}" ] && echo "COSIM-STATUS ${STUB_COSIM_STATUS:-AGREE}"
exit ${STUB_COSIM_EXIT:-0}
STUB
  chmod +x "$1"
}

make_sail_stub() {  # $1 = path
  cat > "$1" <<'STUB'
#!/bin/sh
# Stands in for sail_riscv_sim. Copies a fixture trace to wherever cosim.py
# asked for one and replays a fixture stdout, so cosim.py's whole comparison --
# trace parsing, the distinct-state reduction, the divergence labels and the
# HTIF verdict cross-check -- is exercised with no Sail installed.
trace=; prev=
for a in "$@"; do
  if [ "$prev" = "--trace-output" ]; then trace=$a; fi
  prev=$a
done
if [ -z "${STUB_SAIL_NOTRACE:-}" ] && [ -n "$trace" ]; then
  cat "${STUB_SAIL_TRACE:-/dev/null}" > "$trace"
fi
[ -n "${STUB_SAIL_STDOUT:-}" ] && cat "$STUB_SAIL_STDOUT"
exit 0
STUB
  chmod +x "$1"
}

make_cosim_bin_stub() {  # $1 = path
  cat > "$1" <<'STUB'
#!/bin/sh
# Stands in for ./cosim (test/cosim.cc): replays a fixture's CS record stream.
cat "${STUB_DUT_OUT:-/dev/null}"
exit ${STUB_DUT_EXIT:-0}
STUB
  chmod +x "$1"
}

make_curl_stub() {  # $1 = path
  cat > "$1" <<'STUB'
#!/bin/sh
# Stands in for curl in `make sail-setup`. Writes bytes that are NOT the pinned
# release to wherever -o points and succeeds, so the digest comparison is the
# only thing standing between a substituted asset and an executed binary.
out=; prev=
for a in "$@"; do
  if [ "$prev" = "-o" ]; then out=$a; fi
  prev=$a
done
[ -n "$out" ] && echo "not the pinned release" > "$out"
exit 0
STUB
  chmod +x "$1"
}

# Three ways a tool answers when soc/print_toolchain.sh asks it for a version.
# Stubs rather than the real toolchain: the two red ones are what a broken
# install does, and no probe here may need yosys or nextpnr to be present.
make_version_stubs() {  # $1 = bin dir
  local bin=$1
  mkdir -p "$bin"
  cat > "$bin/goodtool" <<'STUB'
#!/bin/sh
# The control: answers --version the way every tool in the real toolchain does.
echo "GoodTool 1.2.3"
STUB
  cat > "$bin/mutetool" <<'STUB'
#!/bin/sh
# Succeeds having printed nothing, which a stamp built from the first line alone
# would record as this toolchain's version.
exit 0
STUB
  cat > "$bin/brokentool" <<'STUB'
#!/bin/sh
# The case the refusal was written for, and it is live: the Homebrew
# nextpnr-ice40 on one machine here answers --version with a dynamic-linker
# error, which is a non-empty first line under a nonzero status.
echo "dyld: Symbol not found: __ZN5boost15program_options3argE" >&2
exit 1
STUB
  chmod +x "$bin/goodtool" "$bin/mutetool" "$bin/brokentool"
}

# `leg-rt` / `leg-rc` are scratch copies of the two suite runners, because each
# resolves its helper scripts relative to its own path.
mkdir -p "$tmp/bin-none" "$tmp/bin-curl" "$tmp/leg-rt" "$tmp/leg-rc" "$tmp/leg-rc-nopy"

# For the one probe that claims "no cross compiler", `bin-none` has to be the
# WHOLE path: with /usr/bin behind it, run_tests.sh finds a real
# riscv64-unknown-elf-gcc on any host that has one there, which is every CI
# runner -- that is how that probe went green here and red on CI.
#
# No python3 and no env here on purpose: a python3 that is a version-manager shim
# would shadow the real interpreter for every OTHER probe, whose PATH has this
# directory on it too.
for util in sed awk sort uniq comm basename dirname wc tr cat rm mktemp diff \
             grep find head; do
  path=$(command -v "$util") || {
    echo "error: $util is not on PATH; the probes cannot build a minimal one." >&2
    exit 1
  }
  ln -s "$path" "$tmp/bin-none/$util"
done
make_toolchain_stubs "$tmp/bin"
make_toolchain_stubs "$tmp/bin-noobjcopy"
rm "$tmp/bin-noobjcopy/riscv64-elf-objcopy"
make_sim_stub "$tmp/sim"
make_sail_stub "$tmp/sail"
make_curl_stub "$tmp/bin-curl/curl"
make_version_stubs "$tmp/bin-tools"
make_cosim_bin_stub "$tmp/dut"
cp "$HERE/run_tests.sh" "$HERE/check_suite_shape.sh" "$HERE/stall_report.py" "$tmp/leg-rt/"
cp "$HERE/run_cosim.sh" "$HERE/check_suite_shape.sh" "$tmp/leg-rc/"
make_cosim_py_stub "$tmp/leg-rc/cosim.py"
cp "$HERE/run_cosim.sh" "$HERE/check_suite_shape.sh" "$tmp/leg-rc-nopy/"
make_cosim_py_stub "$tmp/leg-rc-nopy/cosim.py"
chmod -x "$tmp/leg-rc-nopy/cosim.py"

begin_group "test/check_suite_shape.sh"

SHAPE="$HERE/check_suite_shape.sh"

# Both program shapes, because the glob and the manifest's name check each have
# to see .c as well as .S. A fixture with only .S would let either one go back
# to being .S-only and stay green.
shape_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/asm"
  : > "$d/asm/add.S"; : > "$d/asm/lw.S"; : > "$d/asm/boot.c"
  printf '# a comment\nadd.S 10 10\nlw.S 5 5\nboot.c 7 7\n' > "$d/MANIFEST"
  printf '%s' "$d"
}

d=$(shape_fixture)
probe "control: a suite matching its manifest is green" 0 "Suite shape matches" \
  "$SHAPE '$d/asm' '$d/MANIFEST'"

probe "wrong argument count is a usage error, not a silent pass" 1 "usage:" \
  "$SHAPE '$d/asm'"

probe "a missing asm directory is named" 1 "does not exist" \
  "$SHAPE '$d/nope' '$d/MANIFEST'"

probe "a mistyped manifest path cannot yield an empty expected set" 1 \
  "does not exist or is not readable" "$SHAPE '$d/asm' '$d/NOPE'"

d=$(shape_fixture); printf '# only comments\n' > "$d/MANIFEST"
probe "an empty manifest matches an empty suite and is rejected" 1 \
  "names no programs" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); printf 'add.S 10 10\nnot-a-program 5 5\n' > "$d/MANIFEST"
probe "a manifest entry that names no program is a typo, not a phantom" 1 \
  "is not a '<test>.S' or" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); printf 'add.S 10 10\nadd.S 10 10\nlw.S 5 5\n' > "$d/MANIFEST"
probe "a duplicated manifest line would make comm non-symmetric" 1 \
  "names the same program more than once" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); rm "$d/asm/add.S" "$d/asm/lw.S" "$d/asm/boot.c"
probe "an empty asm directory is red rather than a suite of size zero" 1 \
  "no programs found" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); rm "$d/asm/lw.S"
probe "a SHRUNK suite: the manifest names a program the tree does not have" 1 \
  "The suite has SHRUNK" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); rm "$d/asm/boot.c"
probe "a SHRUNK suite is caught when the missing program is the C one" 1 \
  "The suite has SHRUNK" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); : > "$d/asm/unlisted.S"
probe "the other direction: a .S that landed without a manifest line" 1 \
  "runs unmeasured" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); : > "$d/asm/unlisted.c"
probe "the other direction for a .c: the glob has to see it too" 1 \
  "runs unmeasured" "$SHAPE '$d/asm' '$d/MANIFEST'"

begin_group "test/run_tests.sh"

rt_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/asm"
  : > "$d/asm/add.S"
  : > "$d/asm/sections.lds"
  printf 'add.S 10 10\n' > "$d/FLOOR"
  : > "$d/BASELINE"
  printf '%s' "$d"
}

RT="$tmp/leg-rt/run_tests.sh"
rt() { printf "PATH='%s/bin:%s/bin-none:/usr/bin:/bin' %s %s/sim %s/asm %s/BASELINE %s/FLOOR" \
  "$tmp" "$tmp" "$RT" "$tmp" "$1" "$1" "$1"; }

d=$(rt_fixture)
probe "control: a passing suite against an empty baseline is green" 0 \
  "Failure list matches" "$(rt "$d")"

probe "wrong argument count is a usage error" 1 "usage:" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $RT $tmp/sim $d/asm"

probe "a mistyped baseline path cannot compare nothing and pass" 1 \
  "without it there is no gate" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $RT $tmp/sim $d/asm $d/NOPE $d/FLOOR"

probe "a mistyped floor path cannot make every floor lookup miss" 1 \
  "there is nothing to compare this run's observation against" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $RT $tmp/sim $d/asm $d/BASELINE $d/NOPE"

d=$(rt_fixture); : > "$d/asm/unlisted.S"
probe "the suite's shape is asserted before a single program is assembled" 1 \
  "nothing was run" "$(rt "$d")"

d=$(rt_fixture); printf 'add.S 10\n' > "$d/FLOOR"
probe "a floor line this loop cannot parse is a floor that is not enforced" 1 \
  "are not '<program> <retires> <spec-checked>'" "$(rt "$d")"

d=$(rt_fixture); printf 'add.S ten 10\n' > "$d/FLOOR"
probe "a non-numeric floor is named rather than compared arithmetically" 1 \
  "are not '<program> <retires> <spec-checked>'" "$(rt "$d")"

# The manifest check runs first, so the fixture needs the .c to exist as well.
d=$(rt_fixture); : > "$d/asm/boot.c"
printf 'add.S 10 10\nboot.c 400 400\n' > "$d/FLOOR"
probe "a C floor copied out of the measured table is rejected, not honoured" 1 \
  "gives a C program a floor above" "$(rt "$d")"

d=$(rt_fixture); printf 'add.S\n' > "$d/BASELINE"
probe "a pre-ADR-0035 one-field baseline line is rejected, not half-matched" 1 \
  "entries with no status" "$(rt "$d")"

d=$(rt_fixture)
probe "a half-installed toolchain says so once, up front" 1 \
  "no RISC-V cross compiler found" \
  "PATH='$tmp/bin-none' $RT $tmp/sim $d/asm $d/BASELINE $d/FLOOR"

d=$(rt_fixture)
probe "objcopy is probed, not assumed to exist because gcc did" 1 \
  "but not its matching" \
  "PATH='$tmp/bin-noobjcopy:$tmp/bin-none' $RT $tmp/sim $d/asm $d/BASELINE $d/FLOOR"

d=$(rt_fixture)
probe "a failing assembler is ASSEMBLE-ERROR" 1 "ASSEMBLE-ERROR" \
  "STUB_CC_EXIT=1 $(rt "$d")"

probe "assembler output on a successful build is still a defect" 1 \
  "ASSEMBLE-WARNING" "STUB_CC_WARN=1 $(rt "$d")"

probe "a failing objcopy names the region that failed" 1 "OBJCOPY-ERROR rom" \
  "STUB_OBJCOPY_FAIL=1 $(rt "$d")"

probe "the quiet one: objcopy exits 0 having written no bytes" 1 \
  "OBJCOPY-EMPTY rom" "STUB_OBJCOPY_EMPTY=1 $(rt "$d")"

probe "runner exit 1 is FAIL, carrying the test number" 1 "FAIL 7" \
  "STUB_SIM_EXIT=1 $(rt "$d")"

probe "runner exit 2 is TIMEOUT" 1 "TIMEOUT" "STUB_SIM_EXIT=2 $(rt "$d")"

probe "runner exit 4 is MONITOR-ERROR, carrying the monitor's code" 1 \
  "MONITOR-ERROR 101" "STUB_SIM_EXIT=4 $(rt "$d")"

probe "runner exit 5 is TRAP-TO-ZERO, not a timeout with no cause" 1 \
  "TRAP-TO-ZERO" "STUB_SIM_EXIT=5 $(rt "$d")"

probe "runner exit 6 is MONITOR-SILENT: the oracle never looked" 1 \
  "MONITOR-SILENT" "STUB_SIM_EXIT=6 $(rt "$d")"

probe "an unexpected runner status is RUNNER-ERROR, carrying it" 1 \
  "RUNNER-ERROR 3" "STUB_SIM_EXIT=3 $(rt "$d")"

probe "an unstartable runner is RUNNER-ERROR 127, never FAIL" 1 \
  "RUNNER-ERROR 127" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $RT $tmp/no-such-sim $d/asm $d/BASELINE $d/FLOOR"

probe "a PASS with no counts line is not a pass" 1 "NO-COUNTS" \
  "STUB_SIM_NOCOUNTS=1 $(rt "$d")"

probe "a program that went quiet is BELOW-FLOOR on retires" 1 \
  "BELOW-FLOOR retires" "STUB_SIM_RETIRES=1 $(rt "$d")"

probe "retires that stopped being spec-checked is its own status" 1 \
  "BELOW-FLOOR spec-checked" "STUB_SIM_SPEC=1 $(rt "$d")"

d=$(rt_fixture); printf 'add.S FAIL 7\n' > "$d/BASELINE"
probe "control: a baselined failure that fails exactly that way is green" 0 \
  "Failure list matches" "STUB_SIM_EXIT=1 $(rt "$d")"

probe "a baselined test that starts failing a DIFFERENT way is red" 1 \
  "does NOT match" "STUB_SIM_EXIT=2 $(rt "$d")"

probe "an unexpected PASS is red too -- the baseline is not a ceiling" 1 \
  "does NOT match" "$(rt "$d")"

# STALL_REPORT=1 is `make cycles`. It leaves the pass/fail grading alone and adds
# one of its own, so both statuses have to be able to reach the caller.
d=$(rt_fixture)
probe "control: STALL_REPORT prints the accounting and keeps the suite green" 0 \
  "cycle accounting" "STALL_REPORT=1 $(rt "$d")"

probe "a stall reason the accounting cannot name is red even with every test passing" 1 \
  "stalled for a reason this report does not name" \
  "STALL_REPORT=1 STUB_SIM_UNATTR=3 $(rt "$d")"

probe "columns that no longer add up to the cycle count are red" 1 \
  "columns sum to" "STALL_REPORT=1 STUB_SIM_SKEW=5 $(rt "$d")"

probe "a runner that reports no cycles at all cannot produce a clean table" 1 \
  "no program reported its cycles" "STALL_REPORT=1 STUB_SIM_NOSTALLS=1 $(rt "$d")"

begin_group "test/run_cosim.sh"

rc_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/asm"
  : > "$d/asm/add.S"
  printf 'add.S 10 10\n' > "$d/MANIFEST"
  : > "$d/BASELINE"
  : > "$d/cosim-bin"
  printf '%s' "$d"
}

RC="$tmp/leg-rc/run_cosim.sh"
rcs() { printf "%s %s/cosim-bin %s/asm %s/BASELINE %s/MANIFEST" \
  "$RC" "$1" "$1" "$1" "$1"; }

d=$(rc_fixture)
probe "control: a suite that agrees against an empty baseline is green" 0 \
  "Divergence list matches" "$(rcs "$d")"

probe "wrong argument count is a usage error" 1 "usage:" \
  "$RC $d/cosim-bin"

probe "a mistyped baseline path cannot compare nothing and pass" 1 \
  "without it there is no gate" \
  "$RC $d/cosim-bin $d/asm $d/NOPE $d/MANIFEST"

d=$(rc_fixture)
probe "a cosim.py that cannot be executed stops the run" 1 \
  "is missing or not executable" \
  "$tmp/leg-rc-nopy/run_cosim.sh $d/cosim-bin $d/asm $d/BASELINE $d/MANIFEST"

d=$(rc_fixture); printf 'add.S\n' > "$d/BASELINE"
probe "a one-field baseline line is rejected before the suite runs" 1 \
  "entries with no status" "$(rcs "$d")"

d=$(rc_fixture); : > "$d/asm/unlisted.S"
probe "the manifest is checked here too, and before the setup probe" 1 \
  "nothing was run" "$(rcs "$d")"

d=$(rc_fixture)
probe "a missing Sail install stops the run rather than scoring it" 1 \
  "setup is incomplete" "STUB_SETUP_EXIT=1 $(rcs "$d")"

probe "a run that printed no COSIM-STATUS is COSIM-ERROR, not a verdict" 1 \
  "COSIM-ERROR 3" "STUB_COSIM_SILENT=1 STUB_COSIM_EXIT=3 $(rcs "$d")"

probe "status AGREE with a nonzero exit is a broken harness, not agreement" 1 \
  "COSIM-ERROR status/exit mismatch 1" "STUB_COSIM_EXIT=1 $(rcs "$d")"

probe "and the reverse: a divergence reported with exit 0" 1 \
  "COSIM-ERROR status/exit mismatch 0" \
  "STUB_COSIM_STATUS='DISAGREE AT 4' $(rcs "$d")"

d=$(rc_fixture); printf 'add.S DISAGREE AT 4\n' > "$d/BASELINE"
probe "control: a baselined divergence at the baselined point is green" 0 \
  "Divergence list matches" \
  "STUB_COSIM_STATUS='DISAGREE AT 4' STUB_COSIM_EXIT=1 $(rcs "$d")"

probe "a baselined program that diverges SOMEWHERE ELSE is red" 1 \
  "does NOT match" \
  "STUB_COSIM_STATUS='DISAGREE AT 9' STUB_COSIM_EXIT=1 $(rcs "$d")"

probe "an unexpected agreement is red -- both directions, as everywhere" 1 \
  "does NOT match" "$(rcs "$d")"

begin_group "test/cosim.py"

# Only the two ends are fixtures; everything between them -- the distinct-state
# reduction, the two cursors, the divergence labels, the HTIF verdict
# cross-check -- is the real cosim.py.
cp_fixture() {
  local d; d=$(new_case)
  cat > "$d/sail.trace" <<'TRACE'
[1] [M]: 0x00000000 (0x00100093) addi x1, x0, 1
x1 <- 0x00000001
[2] [M]: 0x00000004 (0x00200113) addi x2, x0, 2
x2 <- 0x00000002
TRACE
  printf 'SUCCESS\n' > "$d/sail.out"
  cat > "$d/dut.out" <<'DUT'
CS 0 10 x1=00000001 @pc=00000004
CS 1 12 x2=00000002 @pc=00000008
CS END PASS 12
DUT
  printf '%s' "$d"
}

cps() {
  printf "PATH='%s/bin:%s/bin-none:/usr/bin:/bin' STUB_SAIL_TRACE=%s/sail.trace STUB_SAIL_STDOUT=%s/sail.out STUB_DUT_OUT=%s/dut.out %s/test/cosim.py --quiet --sail %s/sail --cosim-binary %s/dut add.S" \
    "$tmp" "$tmp" "$1" "$1" "$1" "$REPO" "$tmp" "$tmp"
}

d=$(cp_fixture)
probe "control: identical register traces and verdicts AGREE" 0 \
  "COSIM-STATUS AGREE" "$(cps "$d")"

d=$(cp_fixture)
sed -i.bak 's/x2=00000002/x2=000000ff/' "$d/dut.out"
probe "a differing value is a divergence, located by change index" 1 \
  "COSIM-STATUS DISAGREE AT 1" "$(cps "$d")"

d=$(cp_fixture)
sed -i.bak 's/^CS 1 .*$//' "$d/dut.out"
probe "a change the core never made is a length divergence" 1 \
  "COSIM-STATUS DISAGREE LENGTH" "$(cps "$d")"

d=$(cp_fixture)
cat >> "$d/dut.out" <<'DUT'
CS 2 14 x31=deadbeef @pc=0000000c
DUT
probe "an EXTRA architectural write the model never made is caught" 1 \
  "COSIM-STATUS DISAGREE LENGTH" "$(cps "$d")"

d=$(cp_fixture); printf '' > "$d/sail.out"
probe "the reference model hitting its budget is INCONCLUSIVE, not a finding" 2 \
  "COSIM-STATUS INCONCLUSIVE SAIL-LIMIT" "$(cps "$d")"

d=$(cp_fixture); sed -i.bak 's/^CS END PASS 12$/CS END TIMEOUT/' "$d/dut.out"
probe "the core hitting its cycle budget is INCONCLUSIVE too" 2 \
  "COSIM-STATUS INCONCLUSIVE CORE-TIMEOUT" "$(cps "$d")"

d=$(cp_fixture); printf 'FAILURE: 3\n' > "$d/sail.out"
sed -i.bak 's/^CS END PASS 12$/CS END FAIL 3 12/' "$d/dut.out"
probe "control: both sides failing the same test number still AGREE" 0 \
  "COSIM-STATUS AGREE" "$(cps "$d")"

d=$(cp_fixture); printf 'FAILURE: 3\n' > "$d/sail.out"
probe "identical traces with different verdicts is DISAGREE VERDICT" 1 \
  "COSIM-STATUS DISAGREE VERDICT" "$(cps "$d")"

d=$(cp_fixture); sed -i.bak 's/^CS END PASS 12$//' "$d/dut.out"
probe "a runner that printed no CS END terminator is a broken harness" 3 \
  "produced no \`CS END\` line" "$(cps "$d")"

d=$(cp_fixture); printf 'CS 9 nonsense\n' >> "$d/dut.out"
probe "an unrecognised CS line is a hard error, never guessed at" 3 \
  "unparseable cosim record line" "$(cps "$d")"

d=$(cp_fixture)
probe "a reference model that produced no trace is fatal" 3 \
  "sail produced no trace" "STUB_SAIL_NOTRACE=1 $(cps "$d")"

d=$(cp_fixture); : > "$d/sail.trace"
probe "an empty trace is fatal rather than an empty comparison" 3 \
  "sail traced no instructions" "$(cps "$d")"

d=$(cp_fixture)
probe "a failing assembler is fatal, not a divergence" 3 "failed" \
  "STUB_CC_EXIT=1 $(cps "$d")"

# NONCOMPARABLE_CSRS is the one thing in cosim.py that makes the comparison
# weaker, so both halves are probed.
d=$(cp_fixture)
cat > "$d/sail.trace" <<'TRACE'
[1] [M]: 0x00000000 (0xb00025f3) csrr x11, mcycle
x11 <- 0x00000041
TRACE
printf 'CS 0 10 x11=00000099 @pc=00000004\nCS END PASS 10\n' > "$d/dut.out"
probe "a non-comparable CSR value is exempted -- and every exemption printed" 0 \
  "NOT COMPARED BY VALUE (1" "$(cps "$d")"

d=$(cp_fixture)
cat > "$d/sail.trace" <<'TRACE'
[1] [M]: 0x00000000 (0xb00025f3) csrr x11, mcycle
x11 <- 0x00000041
TRACE
printf 'CS 0 10 x12=00000099 @pc=00000004\nCS END PASS 10\n' > "$d/dut.out"
probe "the exemption does not extend to a write of the WRONG register" 1 \
  "COSIM-STATUS DISAGREE" "$(cps "$d")"

begin_group "formal/check-baseline.sh"

CB="$REPO/formal/check-baseline.sh"

cb_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/checks/alpha" "$d/checks/beta"
  : > "$d/checks/alpha.sby"; : > "$d/checks/beta.sby"
  printf 'PASS 0 31\n' > "$d/checks/alpha/status"
  printf 'PASS 0 31\n' > "$d/checks/beta/status"
  printf 'alpha\nbeta\n' > "$d/EXPECTED_CHECKS"
  : > "$d/EXPECTED_FAIL"
  printf '%s' "$d"
}

cbs() { printf "%s %s/checks %s/EXPECTED_FAIL %s/EXPECTED_CHECKS" "$CB" "$1" "$1" "$1"; }

d=$(cb_fixture)
probe "control: a full-pass check set against an empty baseline is green" 0 \
  "Failure list matches" "$(cbs "$d")"

probe "wrong argument count is exit 2 -- the inputs are broken, not the checks" 2 \
  "usage:" "$CB $d/checks"

probe "a missing baseline file is exit 2, not an empty expected set" 2 \
  "no such file" "$CB $d/checks $d/NOPE $d/EXPECTED_CHECKS"

d=$(cb_fixture); printf 'beta FAIL\n' > "$d/EXPECTED_FAIL"; chmod 000 "$d/EXPECTED_FAIL"
probe "an UNREADABLE baseline is refused, not read as an empty one" 2 \
  "exists but is not readable" "$(cbs "$d")"
chmod 644 "$d/EXPECTED_FAIL"

d=$(cb_fixture); chmod 000 "$d/EXPECTED_CHECKS"
probe "and the same for the check-set baseline" 2 \
  "exists but is not readable" "$(cbs "$d")"
chmod 644 "$d/EXPECTED_CHECKS"

d=$(cb_fixture); rm "$d/checks/alpha.sby" "$d/checks/beta.sby"
probe "a check set that was never generated is exit 2, not zero checks passing" 2 \
  "the check set was never" "$(cbs "$d")"

d=$(cb_fixture); printf 'beta\n' > "$d/EXPECTED_FAIL"
probe "a pre-ADR-0036 one-field baseline line is named, not half-matched" 2 \
  "no status field" "$(cbs "$d")"

d=$(cb_fixture); printf 'beta BANANA\n' > "$d/EXPECTED_FAIL"
probe "a status sby cannot produce is rejected at format time" 2 \
  "is not a status sby can produce" "$(cbs "$d")"

d=$(cb_fixture); printf 'beta PASS\n' > "$d/EXPECTED_FAIL"
probe "PASS in a failure set is a comparison whose only branch is failure" 2 \
  "must never be baselined" "$(cbs "$d")"

d=$(cb_fixture); printf 'beta ERROR\n' > "$d/EXPECTED_FAIL"
probe "baselining ERROR would re-create the btorsim hole with this gate's blessing" 2 \
  "must never be baselined" "$(cbs "$d")"

d=$(cb_fixture); printf 'beta NO-STATUS\n' > "$d/EXPECTED_FAIL"
probe "NO-STATUS is a broken harness and is not baselineable either" 2 \
  "must never be baselined" "$(cbs "$d")"

d=$(cb_fixture); printf 'beta PASS 0 31\n' > "$d/EXPECTED_FAIL"
probe "sby's trailing engine numbers are not part of the status" 2 \
  "The format is exactly two" "$(cbs "$d")"

d=$(cb_fixture); printf 'FAIL 0 20\n' > "$d/checks/beta/status"
probe "a check that went red is red here" 1 "beta FAIL" "$(cbs "$d")"

d=$(cb_fixture); printf 'FAIL 0 20\n' > "$d/checks/beta/status"
printf 'beta FAIL\n' > "$d/EXPECTED_FAIL"
probe "control: a baselined red check at the baselined status is green" 0 \
  "Failure list matches" "$(cbs "$d")"

printf 'ERROR 16 2\n' > "$d/checks/beta/status"
probe "still red, but for a DIFFERENT reason: FAIL baselined, ERROR on disk" 1 \
  "beta ERROR" "$(cbs "$d")"

d=$(cb_fixture); printf 'TIMEOUT 0 20\n' > "$d/checks/beta/status"
printf 'beta TIMEOUT\n' > "$d/EXPECTED_FAIL"
probe "control: TIMEOUT is a real verdict and IS baselineable" 0 \
  "Failure list matches" "$(cbs "$d")"

d=$(cb_fixture); rm -r "$d/checks/beta"
probe "a generated check that was never scheduled resolves to NO-STATUS" 1 \
  "beta NO-STATUS" "$(cbs "$d")"

d=$(cb_fixture); : > "$d/checks/gamma.sby"; mkdir "$d/checks/gamma"
printf 'PASS 0 31\n' > "$d/checks/gamma/status"
probe "a check that APPEARED needs a line -- the shape check runs both ways" 1 \
  "Generated check set does NOT match" "$(cbs "$d")"

d=$(cb_fixture); rm "$d/checks/beta.sby"
probe "a lost [depth] line trips the shape check AND the verdict check" 1 \
  "Generated check set does NOT match" "$(cbs "$d")"

d=$(cb_fixture); printf 'WOBBLE 1 2\n' > "$d/checks/beta/status"
probe "a status sby has never written here is reported on its own" 1 \
  "Unrecognised status" "$(cbs "$d")"

begin_group "formal/check-complete-exclusions.py"

CE="$REPO/formal/check-complete-exclusions.py"

# The riscv-formal stand-in carries only what clause 5 reads. Naming `add` and
# `lw` is what makes the control meaningful: the file is non-empty and names no
# excluded mnemonic.
ce_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rf/insns"
  printf 'add\nlw\n' > "$d/rf/insns/isa_rv32imc.txt"
  cp "$REPO/formal/complete.sv" "$d/complete.sv"
  cp "$REPO/formal/COMPLETE_EXCLUSIONS" "$d/BASELINE"
  printf '%s' "$d"
}

ces() { printf "%s %s/complete.sv %s/BASELINE %s/rf" "$CE" "$1" "$1" "$1"; }

d=$(ce_fixture)
probe "control: the shipping exclusion set matches its baseline both ways" 0 \
  "COMPLETE EXCLUSION SET: PASS" "$(ces "$d")"

probe "wrong argument count is exit 2" 2 "check-complete-exclusions.py" \
  "$CE $d/complete.sv"

d=$(ce_fixture)
sed -i.bak "s|wire \[6:0\]  insn_opcode       = rvfi_insn\[6:0\];|wire [6:0]  insn_opcode       = decoded_opcode;|" "$d/complete.sv"
probe "the two definitions the predicates are built from are pinned literally" 1 \
  "expected the exact line defining" "$(ces "$d")"

d=$(ce_fixture)
sed -i.bak "s|wire exclude_misc_mem = insn_uncompressed && insn_opcode == 7'b0001111;|wire exclude_misc_mem = decoder_is_fence;|" "$d/complete.sv"
probe "a predicate keyed on a decoder flag is rejected, not parsed" 1 \
  "must not be able to excuse" "$(ces "$d")"

d=$(ce_fixture)
sed -i.bak "s|wire exclude_misc_mem = insn_uncompressed && insn_opcode == 7'b0001111;|wire exclude_fences = insn_uncompressed \&\& insn_opcode == 7'b0001111;|" "$d/complete.sv"
probe "a wire that does not carry its class's name is named" 1 \
  "must name its wire" "$(ces "$d")"

d=$(ce_fixture)
sed -i.bak "s|wire exclude_misc_mem = insn_uncompressed && insn_opcode == 7'b0001111;|wire exclude_misc_mem = insn_uncompressed \&\& insn_opcode == 7'b0000011;|" "$d/complete.sv"
probe "a predicate matching an opcode its declaration does not is named" 1 \
  "but its wire matches" "$(ces "$d")"

# `@` as the sed delimiter throughout this block: the text being replaced
# contains `||`, which closes an `s|...|...|` early.
d=$(ce_fixture)
sed -i.bak "s@wire insn_excluded = .*@wire insn_excluded = exclude_system;@" "$d/complete.sv"
probe "a declared-but-unwired exclusion over-reports the restriction" 1 \
  "must be wired in and nothing else may be" "$(ces "$d")"

d=$(ce_fixture)
sed -i.bak "s@wire insn_excluded = .*@wire insn_excluded = exclude_misc_mem || (exclude_system \&\& !rvfi_trap);@" "$d/complete.sv"
probe "insn_excluded must be a plain disjunction, not an expression" 1 \
  "must be a plain" "$(ces "$d")"

d=$(ce_fixture)
sed -i.bak "s|^  // EXCLUDE MISC-MEM 0001111 fence fence.i$|  // EXCLUDE MISC-MEM 0001111 fence fence.i lw|" "$d/complete.sv"
probe "widening a declared mnemonic list without the baseline is red" 1 \
  "declares an exclusion that" "$(ces "$d")"

d=$(ce_fixture)
printf 'STORE 0100011 sw\n' >> "$d/BASELINE"
probe "a baseline line with no predicate behind it is red too" 1 \
  "names an exclusion" "$(ces "$d")"

d=$(ce_fixture)
sed -i.bak 's|^MISC-MEM  0001111  fence fence.i$|MISC-MEM  00011 fence fence.i|' "$d/BASELINE"
probe "a baseline opcode that is not seven binary digits is named" 1 \
  "seven binary digits" "$(ces "$d")"

d=$(ce_fixture); : > "$d/rf/insns/insn_fence.v"
probe "a pin that ADDS a spec model makes the exclusion stale, and red" 1 \
  "EXISTS at the pin" "$(ces "$d")"

d=$(ce_fixture); printf 'add\nlw\nfence\n' > "$d/rf/insns/isa_rv32imc.txt"
probe "the isa list is read too, not just the insns/ directory" 1 \
  "so rvfi_isa_rv32imc drives a spec model" "$(ces "$d")"

d=$(ce_fixture); rm "$d/rf/insns/isa_rv32imc.txt"
probe "an unreadable clone makes 'no spec model' unmeasurable, and fatal" 1 \
  "cannot read" "$(ces "$d")"

# The reason lives on the comment lines under the header, so deleting them is
# the mutation.
d=$(ce_fixture)
python3 - "$d/complete.sv" <<'PY'
import sys
path = sys.argv[1]
lines = open(path).read().splitlines(True)
out, strip = [], False
for line in lines:
    if line.strip().startswith('// EXCLUDE MISC-MEM'):
        out.append(line)
        strip = True
        continue
    if strip:
        if line.lstrip().startswith('//'):
            continue
        strip = False
    out.append(line)
open(path, 'w').write(''.join(out))
PY
probe "an exclusion with no reason written under it is rejected" 1 \
  "has no reason written under it" "$(ces "$d")"

# Nothing below edits the sanitizer's constants: each probe mutates a COPY of
# the tracked test/monitor.v and requires the sanitizer to refuse it. Re-deriving
# TRAP_GATE_ENCLOSED_CODES from whatever the generator currently emits is how a
# change gets laundered into the expectation, which that list's comment forbids.
begin_group "test/sanitize_monitor.py"

SM="python3 $REPO/test/sanitize_monitor.py"

sm_fixture() {
  local d; d=$(new_case)
  cp "$REPO/test/monitor.v" "$d/monitor.v"
  printf '%s' "$d"
}

sm_mutate() {  # $1 = file, stdin = python that rewrites `text`
  python3 - "$1"
}

d=$(sm_fixture)
probe "control: the tracked monitor sanitizes, and rule 3 fires" 0 \
  "if (!ch0_spec_trap && !ch0_rvfi_mem_fault) begin" "$SM $d/monitor.v"

d=$(sm_fixture)
probe "control: rule 6 gates the trap comparison and writes its compensation" 0 \
  'ch0_handle_error(150, "refused access without a trap")' "$SM $d/monitor.v"

probe "usage: a missing argument is exit 2" 2 "usage:" "$SM"

d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace(', $time)', ')', 1)
open(p, 'w').write(t)
PY
probe "rule 1 declares a site count, so a generator change cannot ship unapplied" 1 \
  "matched 3 site(s), expected 4" "$SM $d/monitor.v"

d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace(
    '$signed(rvfi_rs1_rdata) / $signed(rvfi_rs2_rdata);',
    '(rvfi_rs1_rdata / rvfi_rs2_rdata);', 1)
open(p, 'w').write(t)
PY
probe "rule 2 likewise -- ADR-0019's DIV/REM repair cannot silently stop applying" 1 \
  "matched 1 site(s), expected 2" "$SM $d/monitor.v"

d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace(
    '          ch0_handle_error(102, "mismatch in rs1_addr");',
    '          ch0_handle_error(102, "mismatch in rs1_addr");\n'
    '          if (ch0_rvfi_trap != ch0_spec_trap)\n'
    '            ch0_handle_error(101, "mismatch in trap");', 1)
open(p, 'w').write(t)
PY
probe "layer 1: the trap comparison moved INTO the span is refused, not gated" 1 \
  "the generator now emits the" "$SM $d/monitor.v"

d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace('ch0_handle_error(107, "mismatch in mem_addr")',
                           'ch0_handle_error(109, "mismatch in mem_addr")', 1)
open(p, 'w').write(t)
PY
probe "layer 2: the enclosed handle_error multiset is pinned, not merely counted" 1 \
  "the span encloses" "$SM $d/monitor.v"

# Layers 1 and 2 both accept this one, because neither looks outside the span.
# It mutates the sanitizer rather than the monitor because rule 6 anchors on the
# trap comparison: take that out of the INPUT and rule 6's site count is what
# goes red. What is left for this layer is the output, which is where an edit to
# rule 6's own replacement would drop it.
d=$(sm_fixture)
python3 - "$REPO/test/sanitize_monitor.py" "$d/sanitize.py" <<'PY'
import sys
# The unescaped paren and the trailing semicolon are what make this the emitted
# line and not the regex that looks for it afterwards.
src = open(sys.argv[1]).read()
out = src.replace('_handle_error(101, "mismatch in trap");', '_nothing();', 1)
assert out != src, "the sanitizer no longer spells what this probe removes"
open(sys.argv[2], 'w').write(out)
PY
probe "layer 3: error 101 vanishing from the OUTPUT is caught after every rule" 1 \
  "the trap comparison survives" "python3 $d/sanitize.py $d/monitor.v"

# Rules 4 to 6 carry rvfi_mem_fault into the monitor. Each declares its own site
# count, for the reason rules 1 and 2 do: the spec model has no memory map, so a
# rule here that stopped applying would put both sim legs back to reporting
# error 101 on a core doing what this platform's map says.
d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace('  input [0:0] rvfi_mem_extamo,\n', '', 1)
open(p, 'w').write(t)
PY
probe "rule 4: the port has nowhere to go if the generator drops its anchor" 1 \
  'rule "give the monitor rvfi_mem_fault": matched 0 site(s)' "$SM $d/monitor.v"

d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace(
    '  wire ch0_rvfi_mem_extamo = rvfi_mem_extamo[0];\n', '', 1)
open(p, 'w').write(t)
PY
probe "rule 5: a port the channel never reads is caught, not shipped" 1 \
  'rule "read rvfi_mem_fault into the channel": matched 0 site(s)' "$SM $d/monitor.v"

d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace(
    'if (ch0_rvfi_trap != ch0_spec_trap) begin',
    'if (ch0_spec_trap != ch0_rvfi_trap) begin', 1)
open(p, 'w').write(t)
PY
probe "rule 6: a respelled trap comparison stops rather than gating nothing" 1 \
  'rule "gate the trap comparison on a refused access": matched 0 site(s)' "$SM $d/monitor.v"

# THE ONE THAT MATTERS: the gate is what makes a refused access unreadable by
# the spec model, and the compensation is the only thing that asks what the core
# did with the flag it excused itself with. Rule 6 writes both halves in one
# substitution, so losing one alone is an edit to the sanitizer -- which is what
# this mutates.
d=$(sm_fixture)
python3 - "$REPO/test/sanitize_monitor.py" "$d/sanitize.py" <<'PY'
import sys
src = open(sys.argv[1]).read()
out = src.replace('handle_error(150, "refused access without a trap");',
                  'handle_error(150, "refused access");', 1)
assert out != src, "the sanitizer no longer spells what this probe removes"
open(sys.argv[2], 'w').write(out)
PY
probe "the compensation going missing is caught after every rule" 1 \
  "the refused-access compensation survives" "python3 $d/sanitize.py $d/monitor.v"

begin_group "formal/genchecks-audit.py"

probe "running the generator from the wrong directory is refused, not done" 1 \
  "error: run from" "cd '$tmp' && python3 '$REPO/formal/genchecks-audit.py'"

begin_group "soc/timing_split.py"

TS="python3 $REPO/soc/timing_split.py"

# 1.50 ns is 666.67 MHz, clear of every floor these probes use.
ts_fixture() {
  local d; d=$(new_case)
  cat > "$d/report.rpt" <<'RPT'
 lut1 (LogicCell40) LC: 1.00 ns
   1.00 ns netA (start_point)
 mux1 (LocalMux) MX: 0.50 ns
   1.50 ns netB (mid_point)
              lcout -> end_point
Total path delay: 1.50 ns
Total number of logic levels: 1
RPT
  printf '%s' "$d"
}

d=$(ts_fixture)
probe "control: a report clearing the floor is green" 0 "RATCHET:" \
  "$TS $d/report.rpt --min-mhz 10.0"

d=$(ts_fixture)
probe "a report under the floor fails, because the floor is the board clock" 1 \
  "is under the" "$TS $d/report.rpt --min-mhz 9999"

d=$(ts_fixture); sed -i.bak '/^Total path delay:/d' "$d/report.rpt"
probe "no critical path in the report is a failed measurement, not a fast design" 1 \
  "does not look like an" "$TS $d/report.rpt --min-mhz 10.0"

d=$(ts_fixture)
sed -i.bak 's/^Total path delay: 1.50 ns/Total path delay: 5.00 ns/' "$d/report.rpt"
probe "a hop sum that does not reconcile blames the script, not the design" 1 \
  "summed hops come to" "$TS $d/report.rpt --min-mhz 10.0"

ts_carry_fixture() {
  local d; d=$(new_case)
  cat > "$d/report.rpt" <<'RPT'
 lut1 (LogicCell40) in0 -> lcout: 1.00 ns
   1.00 ns netA (start_point)
 cin1 (ICE_CARRY_IN_MUX) carryinitin -> carryinitout: 0.50 ns
 c1 (LogicCell40) carryin -> carryout: 0.25 ns
   1.75 ns netB (mid_point)
              lcout -> end_point
Total path delay: 1.75 ns
Total number of logic levels: 2
RPT
  printf '%s' "$d"
}

d=$(ts_carry_fixture)
probe "a carry hop is counted apart from a LUT level" 0 \
  "1 LUT/setup + 1 carry" "$TS $d/report.rpt"

d=$(ts_carry_fixture)
probe "the carry chain's own interconnect is charged to the carry hop" 0 \
  "1.00 ns per LUT level, 0.75 ns per carry hop" "$TS $d/report.rpt"

d=$(new_case)
cat > "$d/report.rpt" <<'RPT'
 cin1 (ICE_CARRY_IN_MUX) carryinitin -> carryinitout: 0.50 ns
 c1 (LogicCell40) carryin -> carryout: 0.25 ns
   0.75 ns netA (start_point)
              lcout -> end_point
Total path delay: 0.75 ns
Total number of logic levels: 1
RPT
probe "a path with no LUT level at all reports zero rather than dividing by it" 0 \
  "0.00 ns per LUT level" "$TS $d/report.rpt"

begin_group "soc/baseline_summary.py"

BS="python3 $REPO/soc/baseline_summary.py"

# The block soc/baseline_sweep.sh stamps ahead of its rows, written by hand so a
# probe costs no placement. Every refusal below breaks one field of it, and the
# control runs first: a fixture malformed in some other way would take the whole
# group green-to-red rather than pass it.
bs_sweep() {  # <file> <base> <dirty> <yosys>
  cat > "$1" <<EOF
# baseline-sweep v1
# date: 2026-08-16T00:00:00Z
# base: $2
# dirty: $3
# part: up5k
# yosys: $4 [/opt/bin/yosys]
# nextpnr-ice40: nextpnr-0.11 [/opt/bin/nextpnr-ice40]
# icetime: oss-cad-suite 20260811 sha256:0123456789abcdef [/opt/bin/icetime]
# prog: datainit.c
# rom_words: 2048
# seeds: default 1
# host: Darwin arm64 25.3.0
# reproduce: git checkout $2 && SOC_SEEDS='default 1' SOC_PROG=datainit.c soc/baseline_sweep.sh
# end-provenance
part,variant,seed,ns,mhz,lut_levels,carry_hops,logic_ns,routing_ns,lc,start,end
up5k,sweep,default,80.00,12.50,23,4,20.00,60.00,4769,rom_RDATA,next_pc
up5k,sweep,1,82.00,12.20,24,4,21.00,61.00,4769,rom_RDATA,next_pc
EOF
}

# The other part's stamp, which is a different SET of fields rather than the same
# fields with different values: no icetime, the database nextpnr places against,
# and the constraint it was handed. The four icetime columns are the `NA` literal
# soc/depth/row.py writes for a part with no icetime walk behind it.
bs_ecp5() {  # <file> <base> <dirty> <yosys>
  cat > "$1" <<EOF
# baseline-sweep v1
# date: 2026-08-16T00:00:00Z
# base: $2
# dirty: $3
# part: ecp5
# yosys: $4 [/opt/bin/yosys]
# nextpnr-ecp5: nextpnr-0.11 [/opt/bin/nextpnr-ecp5]
# trellis-db: devices.json sha256:0123456789abcdef [/opt/share/trellis/database]
# corner: LFE5U-25F-6CABGA381
# constraint_mhz: 200.0
# prog: datainit.c
# rom_words: 2048
# seeds: default 1
# host: Darwin arm64 25.3.0
# reproduce: git checkout $2 && BASELINE_PART=ecp5 soc/baseline_sweep.sh
# end-provenance
part,variant,seed,ns,mhz,lut_levels,carry_hops,logic_ns,routing_ns,lc,start,end
ecp5,sweep,default,29.72,33.65,NA,NA,NA,NA,5331,imem.rom_even,imem.rom_odd
ecp5,sweep,1,28.80,34.73,NA,NA,NA,NA,5331,imem.rom_even,imem.rom_odd
EOF
}

bs_fixture() {
  local d; d=$(new_case)
  bs_sweep "$d/before.csv" aaaaaaaaaaaa no 'Yosys 0.68'
  bs_sweep "$d/after.csv" aaaaaaaaaaaa no 'Yosys 0.68'
  sed -i.bak 's/,82.00,12.20,/,84.00,11.90,/' "$d/after.csv"
  printf '%s' "$d"
}

d=$(bs_fixture)
probe "control: a stamped sweep summarises" 0 "2 placements" "$BS $d/before.csv"

d=$(bs_fixture)
probe "control: two sweeps measured the same way are subtracted" 0 \
  "delta, second sweep against first" "$BS $d/before.csv $d/after.csv"

d=$(bs_fixture); sed -i.bak '1d' "$d/before.csv"
probe "an unstamped file is rejected rather than summarised" 1 \
  "no provenance block" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak '/^# end-provenance/d' "$d/before.csv"
probe "a block with no end is truncated, and a truncated one is not read" 1 \
  "the provenance block is truncated" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak '/^# icetime:/d' "$d/before.csv"
probe "a block short of a tool is named, not summarised around" 1 \
  "the provenance block is missing icetime" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak '/^part,/,$d' "$d/before.csv"
probe "a stamp with no table under it is a failed measurement" 1 \
  "no CSV header" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak '/^up5k,/d' "$d/before.csv"
probe "a table with no placements in it is one too" 1 \
  "no placements in it" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak 's/,80.00,/,eighty,/' "$d/before.csv"
probe "a row whose period is not a number stops the read" 1 \
  "ns column reads 'eighty'" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak 's/^# base: aaaaaaaaaaaa/# base: bbbbbbbbbbbb/' "$d/after.csv"
probe "two base commits refuse the delta rather than warning above it" 1 \
  "not measured the same way" "$BS $d/before.csv $d/after.csv"

d=$(bs_fixture); sed -i.bak 's/^# yosys: Yosys 0.68/# yosys: Yosys 0.55/' "$d/after.csv"
probe "two toolchains are named, which is the disagreement that flipped a sign" 1 \
  "yosys:" "$BS $d/before.csv $d/after.csv"

d=$(bs_fixture); sed -i.bak 's/^# dirty: no/# dirty: yes/' "$d/after.csv"
probe "a dirty tree names no base, so its sweep is not subtracted either" 1 \
  "uncommitted changes" "$BS $d/before.csv $d/after.csv"

d=$(bs_fixture); sed -i.bak 's/^# base: aaaaaaaaaaaa/# base: bbbbbbbbbbbb/' "$d/after.csv"
probe "the override prints the mismatch it was passed to get past" 0 \
  "MISMATCH" "$BS $d/before.csv $d/after.csv --allow-mismatch"

d=$(bs_fixture); sed -i.bak 's/^# base: aaaaaaaaaaaa/# base: bbbbbbbbbbbb/' "$d/after.csv"
probe "and prints the delta beside it, not instead of it" 0 \
  "delta, second sweep against first" \
  "$BS $d/before.csv $d/after.csv --allow-mismatch"

d=$(bs_fixture)
probe "three sweeps have no one difference, so they are refused" 1 \
  "takes one sweep, or two" "$BS $d/before.csv $d/after.csv $d/before.csv"

d=$(bs_fixture)
probe "a sweep file that is not there reads as missing, not as empty" 1 \
  "nothing to summarise" "$BS $d/gone.csv"

# ---- the part, which is a stamped and compared field and not just a column ----
#
# The subtraction these forbid was available for as long as `part` was a CSV
# column nothing read: two sweeps of two different fabrics, placed by two
# different engines and graded by two different classes of estimator, would
# produce a tidy percentage under a heading that says "delta".

bs_pair() {  # an up5k sweep and an ECP5 one, same tree, same everything else
  local d; d=$(new_case)
  bs_sweep "$d/up5k.csv" aaaaaaaaaaaa no 'Yosys 0.68'
  bs_ecp5 "$d/ecp5.csv" aaaaaaaaaaaa no 'Yosys 0.68'
  printf '%s' "$d"
}

d=$(bs_pair)
probe "control: an ECP5 sweep summarises against its own instrument" 0 \
  "2 placements" "$BS $d/ecp5.csv"

d=$(bs_pair)
probe "control: and names the corner and constraint it was placed against" 0 \
  "LFE5U-25F-6CABGA381" "$BS $d/ecp5.csv"

# The four icetime columns, which this part has none of. A zero here is a number
# somebody subtracts; the literal is a statement that no such number exists.
d=$(bs_pair)
probe "a part with no icetime walk reads NA rather than a fabricated zero" 0 \
  "LUT levels   : NA   carry hops: NA" "$BS $d/ecp5.csv"

d=$(bs_pair)
probe "and says why it is NA, so it is not read as a path with no logic on it" 0 \
  "no icetime walk behind it" "$BS $d/ecp5.csv"

d=$(bs_pair)
probe "the cell count names its own unit rather than borrowing the other's" 0 \
  "TRELLIS_COMB: 5331" "$BS $d/ecp5.csv"

d=$(bs_fixture); sed -i.bak '/^# part: up5k/d' "$d/before.csv"
probe "a sweep that names no part says which instrument is missing" 1 \
  "names no part" "$BS $d/before.csv"

d=$(bs_fixture); sed -i.bak 's/^# part: up5k/# part: ecp5x/' "$d/before.csv"
probe "a part this script cannot grade a stamp for is rejected, not guessed at" 1 \
  "not one this" "$BS $d/before.csv"

# Both directions of "the stamp describes a run that did not happen". The first
# is the shape a hand-edited header takes when someone changes the part line to
# make a comparison stop complaining.
d=$(bs_fixture); sed -i.bak 's/^# part: up5k/# part: ecp5/' "$d/before.csv"
probe "an ECP5 stamp carrying up5k's tools is missing its own" 1 \
  "missing nextpnr-ecp5, trellis-db" "$BS $d/before.csv"

# A complete up5k stamp with one ECP5 field added, so the `missing` check has
# nothing to say and the foreign-field check is the one under test.
d=$(bs_fixture)
sed -i.bak 's|^# icetime: \(.*\)$|# icetime: \1\
# nextpnr-ecp5: nextpnr-0.11 [/opt/bin/nextpnr-ecp5]|' "$d/before.csv"
probe "and an up5k stamp carrying an ECP5 tool is rejected on the foreign field" 1 \
  "belongs to another part's instrument" "$BS $d/before.csv"

d=$(bs_pair)
probe "two parts are refused rather than subtracted" 1 \
  "There is no difference between them" "$BS $d/up5k.csv $d/ecp5.csv"

# THE ONE THAT MATTERS. --allow-mismatch covers a tree, a toolchain, a program
# and a ROM size, every one of which can be a deliberate before-and-after. It
# must not cover this one, and the probe above passing says nothing about that.
d=$(bs_pair)
probe "and --allow-mismatch does NOT cover a cross-part subtraction" 1 \
  "does NOT cover this" "$BS $d/up5k.csv $d/ecp5.csv --allow-mismatch"

d=$(bs_pair)
probe "the refusal is the part, not the four tool mismatches it also produces" 1 \
  "placed up5k and" "$BS $d/up5k.csv $d/ecp5.csv --allow-mismatch"

begin_group "soc/baseline_sweep.sh"

probe "a part this repo does not place stops the sweep before any placement" 2 \
  "BASELINE_PART is 'xc7'" \
  "BASELINE_PART=xc7 sh $REPO/soc/baseline_sweep.sh"

# The rest of this script places the SoC, so this is the one check in it that
# runs without yosys, nextpnr or a cross compiler -- and it is the one that
# would otherwise silently place the default sixteen seeds for someone who
# asked for none.
probe "an empty seed list stops the sweep instead of placing the default" 2 \
  "SOC_SEEDS is empty" "SOC_SEEDS= sh $REPO/soc/baseline_sweep.sh"

begin_group "soc/print_toolchain.sh"

# The stamp `make fit`, `make soc-timing` and the sweep above all print. What is
# forced red here is the refusal: a tool that cannot be asked has to stop the
# run rather than be recorded as whatever it said, because a version nobody can
# reproduce reads exactly like one anybody can and the number underneath it is
# graded against a ratchet.
#
# The script is reached by its own path and its shebang rather than through an
# interpreter on PATH, because the PATH set here is the fixture: it holds the
# three stubs and the utilities the script itself runs, and nothing else.
PT="PATH='$tmp/bin-tools:$tmp/bin-none' $REPO/soc/print_toolchain.sh"

probe "control: a tool that answers is stamped with its version and its path" 0 \
  "# goodtool: GoodTool 1.2.3 [$tmp/bin-tools/goodtool]" "$PT goodtool"

probe "a tool that prints nothing is refused, not stamped blank" 1 \
  "printed no version string" "$PT mutetool"

probe "a tool that exits nonzero while printing is refused on its status" 1 \
  "could not be asked for its version" "$PT brokentool"

# Whether the good tool's line survives the refusal, not merely whether the exit
# status did: a short block spliced into a CSV header or a step summary looks
# exactly like a whole one.
pt_partial() {
  local said
  said=$(eval "$PT goodtool brokentool" 2> /dev/null) || true
  printf 'stdout=%s\n' "${said:-empty}"
}
probe "one red tool leaves no partial stamp on stdout" 0 "stdout=empty" pt_partial

probe "a tool that is not installed names itself rather than the list" 1 \
  "no nosuchtool on PATH" "$PT nosuchtool"

# The Trellis database is stamped as a pseudo-tool, so it has its own refusal:
# it is resolved from nextpnr-ecp5's install, and the fixture PATH here has no
# nextpnr-ecp5 in it at all.
probe "the Trellis database cannot be stamped without the tool it belongs to" 1 \
  "no nextpnr-ecp5 on PATH" "$PT trellis-db"

probe "and an empty TRELLIS_DB is refused rather than stamped as nothing" 1 \
  "no readable" "TRELLIS_DB='$tmp/no-such-db' PATH='$tmp/bin-tools' \
    $REPO/soc/print_toolchain.sh trellis-db"

begin_group "soc/bands.py"

# The band figures had six prose copies and no owner. What is forced red here is
# the property that replaced them: a part whose band nobody measured gets an
# answer about THAT part, never another part's numbers. A fallback would be a
# wrong answer that looks exactly like a right one, and every caller here prints
# what it gets without checking.
BD="python3 $REPO/soc/bands.py"

probe "control: a derived part states both figures and names itself" 0 \
  "up5k (make soc-timing): placement spread" "$BD up5k"

probe "control: the note a delta is read against carries the part too" 0 \
  "up5k" "$BD up5k --note"

# hx8k is the cross-core harness's part and nothing has ever been swept on it.
# It is in the table precisely so that asking gets this sentence rather than a
# KeyError somebody would 'fix' by copying up5k's row.
probe "an underived part says so rather than borrowing another part's band" 0 \
  "no other part's transfer" "$BD hx8k"

probe "a caller that needs the figures rather than the prose is refused" 1 \
  "no band has been derived for hx8k" "$BD hx8k --require"

probe "and is told that another part's does not transfer" 1 \
  "does not transfer" "$BD hx8k --require"

probe "a part this repo does not place is refused, not added by asking" 1 \
  "is not a part this repo places" "$BD xc7"

probe "--list answers for every part, derived or not" 0 \
  "hx8k: no placement spread" "$BD --list"

begin_group "test/band_source_test.py"

# A COPY OF THE SHIPPING FILES plus a `git init`, the same fixture shape
# test/march_test.sh's probes use and for the same reason: the control is then
# the real tree, and every red probe is one edit away from it.
BSRC="python3 $HERE/band_source_test.py"

bsrc_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/soc" "$d/test" "$d/docs/adr" "$d/rtl"
  cp "$REPO/CLAUDE.md" "$d/"
  cp "$REPO/soc/bands.py" "$REPO/soc/timing_sweep.sh" "$REPO/soc/baseline_summary.py" "$d/soc/"
  cp "$REPO/test/band_source_test.py" "$d/test/"
  # One real ADR, because that directory is exempt and the exemption is itself a
  # decision worth a probe: a dated record must NOT move when a later sweep
  # moves the band.
  cp "$REPO/docs/adr/0121-the-occupancy-prediction-is-registered-and-the-placement-spread-is-corrected.md" \
     "$d/docs/adr/"
  git -c init.defaultBranch=main -C "$d" init -q
  git -C "$d" add -A
  printf '%s' "$d"
}

bsrc_edit() {  # $1 = fixture, $2 = path within it, $3 = sed expression
  sed -i.bak "$3" "$1/$2"
  rm -f "$1/$2.bak"
  git -C "$1" add -A
}

d=$(bsrc_fixture)
probe "control: the shipping tree states every band figure in one place" 0 \
  "every band figure is soc/bands.py's" "$BSRC $d"

# The defect this exists for, reintroduced: a comment stating a spread.
d=$(bsrc_fixture)
bsrc_edit "$d" soc/timing_sweep.sh \
  's|^# One placement is a sample|# The placement spread is 1-2%.\n# One placement is a sample|'
probe "a prose copy of a band figure goes red where it is written" 1 \
  "soc/timing_sweep.sh" "$BSRC $d"

d=$(bsrc_fixture)
bsrc_edit "$d" soc/timing_sweep.sh \
  's|^# One placement is a sample|# The placement spread is 1-2%.\n# One placement is a sample|'
probe "and is named as a percentage beside the word that makes it a claim" 1 \
  "beside 'churn' or 'spread'" "$BSRC $d"

# The wrapped case, which is how all six of the real copies were written: the
# word on one line and the number on the next.
d=$(bsrc_fixture)
bsrc_edit "$d" soc/baseline_summary.py \
  's|^WORST, MEDIAN AND SPREAD|A wrapped churn band of\n3.9% goes here.\nWORST, MEDIAN AND SPREAD|'
probe "a copy wrapped across lines is caught, not read as two harmless ones" 1 \
  "soc/baseline_summary.py" "$BSRC $d"

# The staleness direction. The rulebook keeps its copy on purpose and it is
# graded, so a re-derived band that was not carried into it goes red.
d=$(bsrc_fixture)
bsrc_edit "$d" CLAUDE.md 's|4–9% placement spread|5–11% placement spread|'
probe "the rulebook quoting a figure the source no longer states is red" 1 \
  "CLAUDE.md states no placement spread" "$BSRC $d"

d=$(bsrc_fixture)
bsrc_edit "$d" CLAUDE.md 's|`soc/bands.py` is the one place|it is the one place|'
probe "and a rulebook that does not name the source is red too" 1 \
  "does not name soc/bands.py" "$BSRC $d"

# The exemption, asserted rather than assumed. An ADR is a measurement with a
# date on it: if this went red for one, the pressure would be to edit the record.
d=$(bsrc_fixture)
probe "a dated ADR stating an old band figure is NOT red" 0 \
  "every band figure is soc/bands.py's" "$BSRC $d"

d=$(bsrc_fixture); rm -f "$d/soc/bands.py"; git -C "$d" add -A
probe "a tree with no source file at all is red rather than vacuously green" 1 \
  "is not in" "$BSRC $d"

begin_group "test/zkt_isolation_test.py"

# The script takes a path argument directly, so a fixture is just a mutated
# COPY of the shipping rtl/decoder.v plus its two dependencies -- no git init
# needed, unlike the checks above that enumerate tracked files. regsel.v
# joins structs.v here because this script elaborates the real module now,
# not a text scan of decoder.v alone, and decoder.v instantiates it twice.
ZKT="python3 $HERE/zkt_isolation_test.py"

zkt_fixture() {
  local d; d=$(new_case)
  cp "$REPO/rtl/decoder.v" "$d/decoder.v"
  cp "$REPO/rtl/structs.v" "$d/structs.v"
  cp "$REPO/rtl/regsel.v" "$d/regsel.v"
  printf '%s' "$d"
}

# This control is also the only thing that exercises CONTROL_FIELDS: emptying
# that table makes live_rs1/live_rs2's real reads of out.rd/out.valid and
# executor_out.rd/valid show up as reachable on the SHIPPING RTL, no mutation
# needed, because out.valid's own bubble condition genuinely depends on
# region_stall and trap_pending genuinely depends on reg_rs1 (misalignment).
# A red here can mean that table went stale as easily as it can mean a real
# decoder.v regression.
d=$(zkt_fixture)
probe "control: the shipping decoder reaches region_stall only, gated" 0 \
  "reach only region_stall" "$ZKT $d/decoder.v"

# FORWARD REACHABILITY, the plain case: a register-file DATA bit routed
# straight into hazard, the way a forwarding path or a data-dependent
# early-out might be added by someone who never meant to touch Zkt's claim.
d=$(zkt_fixture)
sed -i.bak \
  's/assign hazard = hazard_rs1 || hazard_rs2 || serialize;/assign hazard = hazard_rs1 || hazard_rs2 || serialize || reg_rs1[0];/' \
  "$d/decoder.v"
probe "a reg_rs1 bit routed into hazard is red, at hazard's own site" 1 \
  "\`hazard\` is reachable" "$ZKT $d/decoder.v"

# FORWARD REACHABILITY THROUGH A REGISTER: reg_rs1 laundered through the
# publish block's own `out.rs1 <= reg_rs1` before reaching hazard. An
# RTL-text scan of continuous assigns alone cannot see this -- `out.rs1` is
# written procedurally -- and this defeated an earlier round of this check
# for exactly that reason. On the elaborated netlist a flip-flop's D input
# feeding its Q output is one more edge, not a different kind of thing.
d=$(zkt_fixture)
sed -i.bak \
  's/assign hazard = hazard_rs1 || hazard_rs2 || serialize;/assign hazard = hazard_rs1 || hazard_rs2 || serialize || out.rs1[0];/' \
  "$d/decoder.v"
probe "reg_rs1 laundered through out.rs1's own register is still red" 1 \
  "\`hazard\` is reachable" "$ZKT $d/decoder.v"

# FORWARD REACHABILITY THROUGH A COMPARATOR: branch_taken depends on
# cmp_eq/cmp_lt/cmp_ltu, which are reg_rs1/reg_rs2 through a subtraction --
# every bit of it real dataflow, computed in an always_comb block. This is
# the other shape that defeated an earlier round: `branch_taken` is never a
# continuous assign's own left-hand side, so a text scan of assigns alone
# never followed a path through it either.
d=$(zkt_fixture)
sed -i.bak \
  's/assign hazard = hazard_rs1 || hazard_rs2 || serialize;/assign hazard = hazard_rs1 || hazard_rs2 || serialize || branch_taken;/' \
  "$d/decoder.v"
probe "branch_taken carrying reg_rs1/reg_rs2 into hazard is red" 1 \
  "\`hazard\` is reachable" "$ZKT $d/decoder.v"

# FORWARD REACHABILITY, the other seed: a register-file DATA output
# STRUCT_FIELD_SEEDS never named. executor_out.rd_data is a 32-bit field of
# a decoder input port, the same shape as reg_rs1/reg_rs2, and a forwarding
# path routing it into a stall reason is exactly the change this repo keeps
# pricing and declining (ADR-0083/0092/0100).
d=$(zkt_fixture)
sed -i.bak \
  's/assign atomic_stall = out.valid && out.is_amo && !divider_stall;/assign atomic_stall = out.valid \&\& out.is_amo \&\& !divider_stall || executor_out.rd_data[0];/' \
  "$d/decoder.v"
probe "an executor_out.rd_data bit routed into a stall reason is red" 1 \
  "\`atomic_stall\` is reachable" "$ZKT $d/decoder.v"

# FINDING 1: a stall reason reading region_stall's own captured state
# directly, bypassing the ls_access gate rather than going through it. The
# RTL-text version trusted `ls_answer_valid` as a KNOWN_CLEAN_LEAF without
# verifying the claim against what it depends on; this version computes it,
# with a narrower reachability pass seeded from ls_capture/ls_answer/
# ls_answer_valid themselves rather than from reg_rs1.
d=$(zkt_fixture)
sed -i.bak \
  's/assign hazard = hazard_rs1 || hazard_rs2 || serialize;/assign hazard = hazard_rs1 || hazard_rs2 || serialize || ls_answer_valid;/' \
  "$d/decoder.v"
probe "hazard reading ls_answer_valid directly is red (finding 1)" 1 \
  "region_stall's own captured answer" "$ZKT $d/decoder.v"

# FINDING 2: the same leak, behind a decoy. An `assign` inside an un-taken
# \`generate if (0)\` claiming ls_answer_valid is a harmless constant would
# have fooled a regex that does not understand generate semantics, by
# masking the real always_ff driver underneath two textual definitions for
# one name. Elaboration never sees the dead branch at all, so the real
# driver -- and the leak above -- is exactly as reachable as it was without
# the decoy.
d=$(zkt_fixture)
python3 - "$d/decoder.v" <<'PYEOF'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace(
    'assign hazard = hazard_rs1 || hazard_rs2 || serialize;',
    'assign hazard = hazard_rs1 || hazard_rs2 || serialize || ls_answer_valid;\n'
    '  generate\n'
    '    if (0) begin : dead_gen\n'
    "      assign ls_answer_valid = 1'b0;\n"
    '    end\n'
    '  endgenerate',
    1)
open(p, 'w').write(s)
PYEOF
probe "a dead generate-if(0) decoy does not hide the same leak (finding 2)" 1 \
  "region_stall's own captured answer" "$ZKT $d/decoder.v"

# FINDING 3: a new decoder input wider than a register NUMBER, added with no
# Zkt classification at all. An RTL-text \`[N:0]\` match against a
# parameterised width can read the wrong number of bits; this script reads
# the port's MEASURED width off the elaborated netlist instead, so a plain
# 10-bit port is unmistakably wide enough to matter.
d=$(zkt_fixture)
sed -i.bak \
  's/  input  logic \[31:0\] reg_rs1,/  input  logic [31:0] reg_rs1,\n  input  logic [9:0] probe_wide_input,/' \
  "$d/decoder.v"
probe "a new wide input port with no classification is red (finding 3)" 2 \
  "no Zkt classification" "$ZKT $d/decoder.v"

# FINDING 5: CONTROL_FIELDS' own written justification is entirely a width
# argument, and nothing checked it. This is the load-bearing half: emptying
# the table is red against the SHIPPING decoder.v with no mutation needed,
# the same way the top control probe above is -- live_rs1/live_rs2's real
# reads of out.rd/out.valid, and executor_out's own two, are genuinely
# reachable once nothing blocks them.
d=$(new_case)
cp "$HERE/zkt_isolation_test.py" "$d/zkt_isolation_test.py"
python3 - "$d/zkt_isolation_test.py" <<'PYEOF'
import sys
p = sys.argv[1]
s = open(p).read()
old = ("CONTROL_FIELDS = {\n"
       "    'out': ('decoder_output', ['valid', 'rd', 'is_amo']),\n"
       "    'executor_out': ('executor_output', ['valid', 'rd']),\n"
       "}")
assert s.count(old) == 1
open(p, 'w').write(s.replace(old, 'CONTROL_FIELDS = {}'))
PYEOF
probe "CONTROL_FIELDS emptied is red against the shipping decoder (finding 5)" 1 \
  "is reachable" "python3 $d/zkt_isolation_test.py $REPO/rtl/decoder.v"

# FINDING 5, the width bound: control_field_bits asserted no width, even
# though the written justification for the whole table is entirely one --
# "rd is [4:0], the same width SEED_PORTS/NON_VALUE_PORTS draw the line at."
# Widening decoder_output's own `rd` field past 5 bits must be caught here,
# the same bound classify_inputs already enforces for input ports.
d=$(zkt_fixture)
python3 - "$d/structs.v" <<'PYEOF'
import sys
p = sys.argv[1]
s = open(p).read()
marker = '} decoder_output;'
idx = s.index(marker)
head, tail = s[:idx], s[idx:]
old = '  logic [4:0]  rd;'
assert head.count(old) == 1
open(p, 'w').write(head.replace(old, '  logic [31:0]  rd;', 1) + tail)
PYEOF
probe "decoder_output.rd widened past 5 bits is red (finding 5)" 2 \
  "wider than a register NUMBER" "$ZKT $d/decoder.v"

# THE OTHER DIRECTION: a classification whose port the netlist no longer
# has. Unlike the probes above, the RTL is the shipping one and the SCRIPT
# is the fixture -- the asymmetry finding 1 named: SEEDS/NON_VALUE_PORTS are
# checked stale in both directions, and KNOWN_CLEAN_LEAVES never was.
d=$(new_case)
cp "$HERE/zkt_isolation_test.py" "$d/zkt_isolation_test.py"
sed -i.bak \
  "s/NON_VALUE_PORTS = {/NON_VALUE_PORTS = {\n    'totally_fake_port',/" \
  "$d/zkt_isolation_test.py"
probe "a classification naming a port the netlist has never seen is red" 2 \
  "Remove the stale entry" "python3 $d/zkt_isolation_test.py $REPO/rtl/decoder.v"

# A stall-reason name with no driving cell at all -- a deleted
# \`assign hazard = ...;\` with the declaration left behind -- would make
# reachability through it vacuously true (nothing flows out of a wire
# nothing drives) rather than the missing stall reason it is.
d=$(zkt_fixture)
sed -i.bak '/assign hazard = hazard_rs1 || hazard_rs2 || serialize;/d' "$d/decoder.v"
probe "a stall reason with no driving cell stops the run" 2 \
  "hazard has no driving cell" "$ZKT $d/decoder.v"

# The anti-vacuity control: if the RTL stopped carrying reg_rs1 into
# region_stall at all, every PASS above would be a check of nothing, and this
# is what says so instead of staying green. Redirected to csr_rdata rather
# than tied to a constant, so ls_block stays a real (if irrelevant) alias
# instead of tripping the driving-cell probe above for an unrelated reason.
d=$(zkt_fixture)
sed -i.bak \
  's/assign ls_block = reg_rs1\[31:LS_BLOCK_BITS\];/assign ls_block = csr_rdata[31:LS_BLOCK_BITS];/' \
  "$d/decoder.v"
probe "a graph with no edges out of reg_rs1 is red, not a vacuous pass" 1 \
  "found no edges at all" "$ZKT $d/decoder.v"

probe "wrong argument count is exit 2" 2 "Usage:" "$ZKT $d/decoder.v extra"

probe "a decoder.v that does not exist is exit 2, not a vacuous pass" 2 \
  "cannot read" "$ZKT $d/nonexistent.v"

begin_group "soc/routing_bins.py"

# One placement's worth of fixture: an icetime report whose path leaves a block
# RAM, crosses a LUT, leaves the pc and stops; the netlist those names resolve
# through; and the stamped sweep that names the report. Written by hand so the
# group costs no placement, and small enough that the reconciliation below can
# be checked with a calculator: 1.279 + 0.649 + 1.099 = 3.027 ns of routing.
rb_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/sweep"
  cat > "$d/sweep/probe.default.timing.rpt" <<'RPT'
        ram0 (SB_RAM40_4K) [clk] -> RDATA[0]: 1.279 ns
     1.279 ns net_1 (mem.rdata[0])
        odrv_0 (Odrv4) I -> O: 0.649 ns
        lc40_0 (LogicCell40) in0 -> lcout: 1.285 ns
     3.213 ns net_2 (riscv.pc[0])
        t1 (LocalMux) I -> O: 1.099 ns
        lc40_1 (LogicCell40) in0 -> lcout: 1.285 ns
     5.597 ns net_3 (mid[0])
              lcout -> mid[0]
Total number of logic levels: 2
Total path delay: 5.60 ns (178.57 MHz)
RPT
  cat > "$d/sweep/probe.csv" <<'CSV'
# baseline-sweep v1
# date: 2026-08-16T00:00:00Z
# base: aaaaaaaaaaaa
# dirty: no
# part: up5k
# yosys: Yosys 0.68 [/opt/bin/yosys]
# nextpnr-ice40: nextpnr-0.11 [/opt/bin/nextpnr-ice40]
# icetime: oss-cad-suite 20260811 sha256:0123456789abcdef [/opt/bin/icetime]
# prog: datainit.c
# rom_words: 2048
# seeds: default
# host: Darwin arm64 25.3.0
# reproduce: git checkout aaaaaaaaaaaa && SOC_SEEDS='default' SOC_PROG=datainit.c soc/baseline_sweep.sh
# end-provenance
part,variant,seed,ns,mhz,lut_levels,carry_hops,logic_ns,routing_ns,lc,start,end
up5k,probe,default,5.60,178.57,2,0,2.57,3.03,4754,mem.rdata[0],mid[0]
CSV
  cat > "$d/soc.json" <<'JSON'
{"modules": {"littlesoc": {
  "cells": {
    "ram0": {"type": "SB_RAM40_4K", "connections": {"RDATA": [10], "RADDR": [30]}},
    "pcff": {"type": "SB_DFF", "connections": {"Q": [20], "D": [40]}},
    "lut0": {"type": "SB_LUT4", "connections": {"O": [50], "I0": [20]}}
  },
  "netnames": {
    "mem.rdata": {"bits": [10]},
    "riscv.pc": {"bits": [20]},
    "mid": {"bits": [50]},
    "ram.addr": {"bits": [30]}
  }
}}}
JSON
  printf '%s' "$d"
}

rb() { printf 'python3 %s/soc/routing_bins.py %s/sweep/probe.csv %s/soc.json' \
  "$REPO" "$1" "$1"; }

d=$(rb_fixture)
probe "control: a stamped sweep with its reports behind it is binned" 0 \
  "aggregate over 1 placement" "$(rb "$d")"

d=$(rb_fixture)
probe "control: the block RAM is named at the end of the path it sits at" 0 \
  "from ram0 (SB_RAM40_4K)" "$(rb "$d")"

d=$(rb_fixture)
probe "control: the pc hop reaches the pc bin, by the declared net's bits" 0 \
  "1.10 ns   36.3%" "$(rb "$d")"

# The defect this whole script exists for: bins that add up to less than the
# path, printed as confidently as bins that add up to all of it. Forced by
# giving a COPY of the reader a soc/timing_split.py that charges the Odrv4 hop
# to logic -- which keeps that script's own reconciliation green, so only this
# one can catch it.
d=$(rb_fixture)
mkdir -p "$d/soc/depth"
cp "$REPO/soc/routing_bins.py" "$REPO/soc/timing_split.py" \
   "$REPO/soc/baseline_summary.py" "$REPO/soc/bands.py" "$d/soc/"
cp "$REPO/soc/depth/path_stages.py" "$d/soc/depth/"
sed -i.bak 's/if kind in LOGIC_CELLS:/if kind in LOGIC_CELLS or kind == "Odrv4":/' \
  "$d/soc/timing_split.py"
probe "two walks disagreeing about what routing is refuse to print a histogram" 1 \
  "the bins come to" \
  "python3 $d/soc/routing_bins.py $d/sweep/probe.csv $d/soc.json"

d=$(rb_fixture)
cat > "$d/sweep/probe.default.timing.rpt" <<'RPT'
        lc40_0 (LogicCell40) in0 -> lcout: 1.285 ns
     1.285 ns net_1 (mid[0])
              lcout -> mid[0]
Total number of logic levels: 1
Total path delay: 1.29 ns
RPT
probe "a report with no routing hop in it is a failed read, not a wired design" 1 \
  "no routing hop was read" "$(rb "$d")"

d=$(rb_fixture); rm "$d/sweep/probe.default.timing.rpt"
probe "a row with no placement behind it stops the read, not just that seed" 1 \
  "no placement behind it" "$(rb "$d")"

# Everything this script does walks an icetime report, and one of the two parts
# has none. Refused where the answer is still readable, rather than a hundred
# lines later blaming the sweep for a report it never wrote.
# A WELL-FORMED sweep of the other part, not this one's stamp with its part line
# flipped: that shape is rejected by the shared reader first, so it would probe
# the field check rather than this one.
d=$(rb_fixture); bs_ecp5 "$d/sweep/probe.csv" aaaaaaaaaaaa no 'Yosys 0.68'
probe "a sweep of the part with no icetime is refused, not walked" 1 \
  "there is nothing here to walk" "$(rb "$d")"

d=$(rb_fixture); sed -i.bak 's/"mem.rdata"/"other.rdata"/' "$d/soc.json"
probe "a netlist from another tree is named, not binned as \`neither\`" 1 \
  "never heard of" "$(rb "$d")"

d=$(rb_fixture); sed -i.bak '/"riscv.pc"/d' "$d/soc.json"
probe "a netlist with no pc net would leave that bin empty for ever" 1 \
  "could never be reached" "$(rb "$d")"

d=$(rb_fixture); sed -i.bak '1d' "$d/sweep/probe.csv"
probe "an unstamped sweep is refused here too, by the reader it shares" 1 \
  "no provenance block" "$(rb "$d")"

begin_group "soc/cell_census.py"

CC="python3 $REPO/soc/cell_census.py"

cc_fixture() {
  local d; d=$(new_case)
  cat > "$d/soc.synth.log" <<'LOG'
     4   SB_MAC16
     2   SB_SPRAM256KA
    20   SB_RAM40_4K
LOG
  printf '%s' "$d"
}

d=$(cc_fixture)
probe "control: a count matching the declaration is green" 0 "as declared" \
  "$CC $d/soc.synth.log SB_SPRAM256KA 2 reason"

d=$(cc_fixture)
probe "the wrong count is named against the log's real one" 1 \
  "20 SB_RAM40_4K cells, expected 16" "$CC $d/soc.synth.log SB_RAM40_4K 16 reason"

d=$(cc_fixture)
probe "a cell type the log never mentions reads as zero, not a crash" 1 \
  "0 SB_RGBA_DRV cells, expected 1" "$CC $d/soc.synth.log SB_RGBA_DRV 1 reason"

# `make ecp5-timing` runs this same script over yosys's ECP5 cell table, so the
# diagnostic has to name the flow that stopped and the variable that declares
# the count. Pointing an ECP5 failure at the ice40 flow's SOC_EXPECT_* would be
# a dangling reference in the one message a reader has.
d=$(new_case)
printf '     36   DP16KD\n' > "$d/ecp5.synth.log"

probe "a census under the other gate names that gate, not the ice40 one" 1 \
  "*** make ecp5-timing: 36 DP16KD cells, expected 4" \
  "$CC $d/ecp5.synth.log DP16KD 4 reason --gate 'make ecp5-timing' --declared ECP5_EXPECT_DP16KD"

probe "and points at that flow's own declaration" 1 \
  "ECP5_EXPECT_DP16KD in the Makefile" \
  "$CC $d/ecp5.synth.log DP16KD 4 reason --gate 'make ecp5-timing' --declared ECP5_EXPECT_DP16KD"

begin_group "soc/ecp5_report.py"

ER="python3 $REPO/soc/ecp5_report.py"
ER_ARGS="--clock clk --part LFE5U-25F-6CABGA381 --constraint-mhz 200.0"

# A three-hop path of 5 + 15 + 5 = 25 ns, which is 40.00 MHz exactly, so the
# reconciliation between the walked path and the frequency nextpnr published
# from it is arithmetic a reader can check by eye. The clock is spelt the way
# nextpnr spells a promoted global, because that mangling is the whole reason
# the parser matches on `$`-separated components instead of on the port name.
ecp5_fixture() {
  local d; d=$(new_case)
  cat > "$d/ecp5.config" <<'CFG'
.device LFE5U-25F

.comment Part: LFE5U-25F-6CABGA381
CFG
  cat > "$d/ecp5.report.json" <<'JSON'
{
  "fmax": {"$glbnet$clk$TRELLIS_IO_IN": {"achieved": 40.0, "constraint": 200.0}},
  "utilization": {"DP16KD": {"available": 56, "used": 36}},
  "critical_paths": [
    {
      "from": "posedge $glbnet$clk$TRELLIS_IO_IN",
      "to": "posedge $glbnet$clk$TRELLIS_IO_IN",
      "path": [
        {"type": "clk-to-q", "delay": 5.0,
         "from": {"cell": "imem.rom_even.0.0"}, "to": {"cell": "imem.rom_even.0.0"}},
        {"type": "routing", "delay": 15.0,
         "from": {"cell": "imem.rom_even.0.0"}, "to": {"cell": "riscv.lut"}},
        {"type": "logic", "delay": 5.0,
         "from": {"cell": "riscv.lut"}, "to": {"cell": "imem.addr_ff"}}
      ]
    }
  ]
}
JSON
  printf '%s' "$d"
}

ecp5_mutate() {  # $1 = report file, stdin = python that rewrites the JSON
  python3 - "$1"
}

d=$(ecp5_fixture)
probe "control: a well-formed report publishes the frequency" 0 \
  "Fmax          : 40.00 MHz" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
probe "control: the caveats travel with the number, on every run" 0 \
  "NEVER MERGED WITH OR SUBTRACTED FROM AN UP5K ONE" \
  "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
sed -i.bak 's/LFE5U-25F-6CABGA381/LFE5U-45F-8CABGA381/' "$d/ecp5.config"
probe "a configuration for another corner is refused, not reported" 1 \
  "does not name LFE5U-25F-6CABGA381" \
  "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture); : > "$d/ecp5.config"
probe "an empty configuration means nothing was expressible on the part" 1 \
  "is empty or missing" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture); rm -f "$d/ecp5.config"
probe "a configuration nextpnr never wrote is a failed run, not a fast design" 1 \
  "is empty or missing" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture); rm -f "$d/ecp5.report.json"
probe "a report that does not exist is named, not read as zero" 1 \
  "does not exist, so NOTHING was" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture); printf 'Info: placing\n' > "$d/ecp5.report.json"
probe "a truncated report blames the run rather than parsing what is left" 1 \
  "is not JSON" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["fmax"] = {}
json.dump(r, open(p, "w"))
PY
probe "no constraint found is red, which is the shape of the recorded defects" 1 \
  "carries no fmax table" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["fmax"] = {"$glbnet$sysclk$TRELLIS_IO_IN": {"achieved": 40.0, "constraint": 200.0}}
json.dump(r, open(p, "w"))
PY
probe "a report naming some other clock does not stand in for this one" 1 \
  "no clock named 'clk'" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["fmax"]["clk"] = {"achieved": 12.0, "constraint": 200.0}
json.dump(r, open(p, "w"))
PY
probe "two clocks named clk makes the frequency a guess, so it refuses to guess" 1 \
  "clocks in the report name" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
del r["fmax"]["$glbnet$clk$TRELLIS_IO_IN"]["achieved"]
json.dump(r, open(p, "w"))
PY
probe "an fmax entry that stopped carrying both halves is refused, not read" 1 \
  "nextpnr has changed the shape of its report" \
  "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["fmax"]["$glbnet$clk$TRELLIS_IO_IN"]["constraint"] = 25.0
json.dump(r, open(p, "w"))
PY
probe "a constraint that is not the declared one is a different measurement" 1 \
  "was placed against a 25.00 MHz" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["fmax"]["$glbnet$clk$TRELLIS_IO_IN"]["achieved"] = 250.0
json.dump(r, open(p, "w"))
PY
probe "a design that MET the target has measured the target" 1 \
  "measures the CONSTRAINT and not the design" \
  "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
del r["utilization"]
json.dump(r, open(p, "w"))
PY
probe "a report with no utilisation table is missing half the measurement" 1 \
  "carries no utilisation table" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["critical_paths"][0]["to"] = "<async>"
json.dump(r, open(p, "w"))
PY
probe "a frequency with no path under it is not a measurement" 1 \
  "critical paths run from" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["critical_paths"][0]["path"][1]["type"] = "iologic"
json.dump(r, open(p, "w"))
PY
probe "a hop class nothing classifies would move the split silently" 1 \
  "is not one this" "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

d=$(ecp5_fixture)
ecp5_mutate "$d/ecp5.report.json" <<'PY'
import json, sys
p = sys.argv[1]
r = json.load(open(p))
r["critical_paths"][0]["path"][1]["delay"] = 10.0
json.dump(r, open(p, "w"))
PY
probe "a path that does not reconcile blames the script, not the design" 1 \
  "but nextpnr publishes 40.00 MHz" \
  "$ER $d/ecp5.report.json $d/ecp5.config $ER_ARGS"

# The one red direction the sixteen above cannot reach: every one of them assumes
# the two files describe THIS run. nextpnr writes both only at the very end of
# its flow, so a run that dies earlier -- killed, out of memory, unroutable, a
# database it cannot load -- would leave the previous run's pair intact, coherent
# with each other and with a tree that has since changed underneath them.
#
# Driven against the real Makefile rather than against a copy of the guard,
# because a second copy of a three-line refusal is the shape this file exists to
# catch. `-o soc-rom` is what keeps it hermetic: `ecp5.json` depends on that
# phony target, so without it make would rebuild the netlist and want yosys and a
# cross compiler for a file the stub never reads. Marking it old leaves
# `ecp5.json` up to date and `ecp5.config` genuinely out of date, which is the
# state a second run on a laptop is really in.
ecp5_stale_fixture() {  # stdin = the stub nextpnr-ecp5's body, after --version
  local d; d=$(new_case)
  mkdir -p "$d/soc" "$d/formal" "$d/bin"
  cp "$REPO/Makefile" "$d/Makefile"
  cp "$REPO/formal/pin.mk" "$d/formal/"
  cp "$REPO/soc/littlesoc.lpf" "$REPO/soc/ecp5_report.py" \
     "$REPO/soc/print_toolchain.sh" "$d/soc/"
  cp -R "$REPO/rtl" "$d/"
  # The pair a previous, COMPLETE run left behind. Both are exactly what the
  # parser accepts, which is the whole point: every refusal it has is satisfied
  # by this pair, so only the recipe can tell it is not this run's.
  cat > "$d/ecp5.config" <<'CFG'
.device LFE5U-25F

.comment Part: LFE5U-25F-6CABGA381
CFG
  cat > "$d/ecp5.report.json" <<'JSON'
{
  "fmax": {"$glbnet$clk$TRELLIS_IO_IN": {"achieved": 40.0, "constraint": 200.0}},
  "utilization": {"DP16KD": {"available": 56, "used": 36}},
  "critical_paths": [
    {
      "from": "posedge $glbnet$clk$TRELLIS_IO_IN",
      "to": "posedge $glbnet$clk$TRELLIS_IO_IN",
      "path": [
        {"type": "clk-to-q", "delay": 5.0,
         "from": {"cell": "stale.rom"}, "to": {"cell": "stale.rom"}},
        {"type": "routing", "delay": 15.0,
         "from": {"cell": "stale.rom"}, "to": {"cell": "stale.lut"}},
        {"type": "logic", "delay": 5.0,
         "from": {"cell": "stale.lut"}, "to": {"cell": "stale.ff"}}
      ]
    }
  ]
}
JSON
  cp "$d/ecp5.report.json" "$d/complete.json"
  echo 'Info: a previous run' > "$d/ecp5.pnr.log"
  # Dated rather than merely written first: what makes the recipe run is that
  # `ecp5.json` is newer than the pair, and a stamp settles that without leaning
  # on the filesystem's timestamp resolution.
  touch -t 202001010000 "$d/ecp5.config" "$d/ecp5.report.json"
  # `ecp5-timing-toolchain` asks both tools for a version and the Trellis
  # database for its device table before anything else runs, so all three have
  # to answer or a probe would go red before reaching the guard it is about.
  printf '#!/bin/sh\necho "stub yosys"\n' > "$d/bin/yosys"
  { echo '#!/bin/sh'
    echo 'case "$1" in --version|-V) echo "stub nextpnr-ecp5"; exit 0;; esac'
    cat
  } > "$d/bin/nextpnr-ecp5"
  chmod +x "$d/bin/yosys" "$d/bin/nextpnr-ecp5"
  mkdir -p "$d/trellis-db"
  echo '{"families": {}}' > "$d/trellis-db/devices.json"
  touch "$d/ecp5.json"
  printf '%s' "$d"
}

ecp5_stale_run() {  # $1 = fixture dir
  printf "cd '%s' && PATH='%s/bin':\$PATH TRELLIS_DB='%s/trellis-db' make -o soc-rom ecp5-timing" \
    "$1" "$1" "$1"
}

# Stands in for a COMPLETE run: writes both files and exits 1, which is what the
# real nextpnr does every time it misses the pinned constraint. Without this
# control, a fixture that simply failed to build would take the four below green
# for a reason that has nothing to do with the guard.
d=$(ecp5_stale_fixture <<'STUB'
cat > ecp5.config <<CFG
.device LFE5U-25F

.comment Part: LFE5U-25F-6CABGA381
CFG
cp complete.json ecp5.report.json
exit 1
STUB
)
probe "control: a nextpnr that wrote its pair is graded, exit 1 and all" 0 \
  "Fmax          : 40.00 MHz" "$(ecp5_stale_run "$d")"

d=$(ecp5_stale_fixture <<< 'exit 1')
probe "a nextpnr that died early is NOT graded on the last run's pair" 2 \
  "so NOTHING was measured" "$(ecp5_stale_run "$d")"

# `.DELETE_ON_ERROR` would remove `ecp5.config` on its own, because that one is a
# make target. `ecp5.report.json` is not, so nothing but the recipe's own `rm`
# takes it away -- and a report left behind is half a stale pair waiting for the
# next run. Graded through a configuration written here, so that the parser gets
# past its cheapest check and the missing report is what speaks.
d=$(ecp5_stale_fixture <<< 'exit 1')
probe "the REPORT goes too, which .DELETE_ON_ERROR cannot do for a non-target" 1 \
  "does not exist, so NOTHING was" \
  "$(ecp5_stale_run "$d") > /dev/null 2>&1; \
   printf '.device LFE5U-25F\n\n.comment Part: LFE5U-25F-6CABGA381\n' > '$d/good.config'; \
   python3 '$REPO/soc/ecp5_report.py' '$d/ecp5.report.json' '$d/good.config' \
     --clock clk --part LFE5U-25F-6CABGA381 --constraint-mhz 200.0"

d=$(ecp5_stale_fixture <<< ': > ecp5.config; exit 1')
probe "a configuration truncated to nothing is caught, which test -e cannot be" 2 \
  "so NOTHING was measured" "$(ecp5_stale_run "$d")"

d=$(ecp5_stale_fixture <<< 'cp complete.json ecp5.config; exit 1')
probe "a configuration written without its report is half a run, not a run" 2 \
  "so NOTHING was measured" "$(ecp5_stale_run "$d")"

begin_group "check-unit-benches"

# Driven against the real Makefile and the real test/*_tb.v tree with the
# declaration overridden on the command line: duplicating the comparison here
# would be the second-parser risk this file exists to avoid elsewhere.
MB="make -C $REPO check-unit-benches"

probe "control: the declared list matches the tree exactly" 0 \
  "unit benches, matching test/*_tb.v exactly" "$MB"

probe "a bench in test/ that make does not run is named" 2 \
  "in test/ but not in UNIT_BENCHES: monitor_tb" \
  "$MB UNIT_BENCHES='exec_tb mem_tb imem_tb decoder_tb regfile_tb csr_tb accessor_tb'"

probe "a declared bench with no file is named the other way" 2 \
  "in UNIT_BENCHES but not in test/: nope_tb" \
  "$MB UNIT_BENCHES='exec_tb mem_tb imem_tb decoder_tb regfile_tb csr_tb accessor_tb monitor_tb nope_tb'"

probe "a declared bench with no UNIT_BENCH_SRC_* would build with no design under test" 2 \
  "monitor_tb is in UNIT_BENCHES with no UNIT_BENCH_SRC_monitor_tb" \
  "$MB UNIT_BENCH_SRC_monitor_tb="

begin_group "test/stall_report.py"

SR="python3 $REPO/test/stall_report.py"

# add.S issues 10 of its 40 cycles and spends 20 waiting on the scoreboard and
# 10 fetching operands; lw.S is the other way round, so the two programs
# disagree about which reason dominates and the total has to decide.
sr_fixture() {
  local d; d=$(new_case)
  cat > "$d/counts" <<'COUNTS'
add.S cycles=40 issue=10 divider=0 atomic=0 hazard=20 serialize=0 operand=10 fetch=0 bus=0 region=0 unattributed=0 lsissue=4 lsedge=1 lsbypass=0 retires=10
lw.S cycles=40 issue=10 divider=0 atomic=0 hazard=5 serialize=0 operand=25 fetch=0 bus=0 region=0 unattributed=0 lsissue=6 lsedge=3 lsbypass=2 retires=10
COUNTS
  printf '%s' "$d"
}

d=$(sr_fixture)
probe "control: an accounting that adds up prints the table" 0 \
  "cycle accounting" "$SR $d/counts"

probe "the dominant reason is the suite's, not the first program's" 0 \
  "The largest single reason is operand" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/^add.S cycles=40/add.S cycles=41/' "$d/counts"
probe "columns that do not add up blame the report, not the core" 1 \
  "columns sum to 40, cycles is 41" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/^add.S \(.*\)unattributed=0/add.S \1unattributed=2/' "$d/counts"
sed -i.bak2 's/^add.S cycles=40/add.S cycles=42/' "$d/counts"
probe "a stall nothing in the list explains is a reason nobody wrote down" 1 \
  "2 cycles stalled for a reason this report does not name" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/ operand=10//' "$d/counts"
probe "a field the runner stopped printing is named, not counted as zero" 1 \
  "is missing operand" "$SR $d/counts"

# The locality counters are not cycles and add up to nothing, so the arithmetic
# above cannot see them at all. What can be seen is a subset larger than the set
# it is drawn from, which is what a counter incremented on the wrong event looks
# like from here.
d=$(sr_fixture)
probe "control: the locality counters are reported under the table" 0 \
  "10 of those instructions were loads or stores" "$SR $d/counts"

probe "control: each subset is printed as a share of that number" 0 \
  "4 (40.0%) with rs1 within 2 KB of a mapped-region edge" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/lsedge=3/lsedge=7/' "$d/counts"
probe "more accesses near an edge than there were accesses is red" 1 \
  "lsedge is 7 against 6 issuing loads and stores" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/lsbypass=0/lsbypass=5/' "$d/counts"
probe "the same for the bypass counter, per program rather than in total" 1 \
  "lsbypass is 5 against 4 issuing loads and stores" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/ lsissue=6//' "$d/counts"
probe "a locality counter that stopped being printed is named too" 1 \
  "is missing lsissue" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/cycles=40/cycles=lots/' "$d/counts"
probe "a count that is not a number stops rather than summing to nonsense" 1 \
  "cycles is 'lots', not a number" "$SR $d/counts"

d=$(sr_fixture); sed -i.bak 's/ issue=10/ issue/' "$d/counts"
probe "a field with no value is a malformed line" 1 "'issue' is not key=value" \
  "$SR $d/counts"

d=$(new_case); : > "$d/counts"
probe "no programs at all would report a clean 0 of 0" 1 \
  "no program reported its cycles" "$SR $d/counts"

begin_group "test/tool_cache_test.sh"

# XDG_CACHE_HOME is what both the Makefile and test/cosim.py resolve the tool
# cache from, so setting it here decides the python side of the comparison
# without installing anything. Nothing below is created on disk: the check
# compares path strings and never stats them.
TCT="$HERE/tool_cache_test.sh"
tc_cache="$tmp/cache/little-cpu"

probe "control: agreeing paths outside the checkout are green" 0 \
  "outside the checkout and agreed on" \
  "XDG_CACHE_HOME=$tmp/cache $TCT $tc_cache/sail $tc_cache/svlint $tc_cache/download"

probe "the Makefile and test/cosim.py drifting apart is red" 1 \
  "do not agree on where the Sail" \
  "XDG_CACHE_HOME=$tmp/cache $TCT $tc_cache/elsewhere $tc_cache/svlint $tc_cache/download"

probe "a Sail install back inside the checkout is red" 1 \
  "test/cosim.py installs tools inside the checkout" \
  "XDG_CACHE_HOME=$REPO/cache $TCT $REPO/cache/little-cpu/sail $tc_cache/svlint $tc_cache/download"

probe "an svlint install inside the checkout is red on its own" 1 \
  "$REPO/tools/svlint" \
  "XDG_CACHE_HOME=$tmp/cache $TCT $tc_cache/sail $REPO/tools/svlint $tc_cache/download"

# The kept release tarball is what a CI cache holds, so a download directory
# back inside the checkout would be cached under a path no worktree can read.
probe "the Sail download directory inside the checkout is red on its own" 1 \
  "$REPO/tools/download" \
  "XDG_CACHE_HOME=$tmp/cache $TCT $tc_cache/sail $tc_cache/svlint $REPO/tools/download"

probe "a relative install directory is red before it is compared" 1 \
  "names a relative tool install directory" \
  "XDG_CACHE_HOME=$tmp/cache $TCT tools/sail tools/svlint tools/download"

begin_group "make sail-setup"

# The whole reason co-simulation is allowed in the merge gate is that this
# recipe verifies the release tarball before anything comes out of it. A stub
# curl substitutes the asset, so what is forced red below is exactly the
# comparison that stands between a substituted download and an executed binary.
#
# SAIL_ASSET is fixed rather than left to `uname`, so the fixture is the same on
# every host and `make test` does not start requiring a machine upstream ships a
# tarball for. It is not `override` in the Makefile, and the three digests are
# all pinned there, so naming one of them here cannot widen what may be fetched.
SS_ASSET=SAIL_ASSET=sail-riscv-Linux-x86_64
SS="MAKEFLAGS= MFLAGS= MAKELEVEL= PATH='$tmp/bin-curl:$PATH' \
    make --no-print-directory -C '$REPO' $SS_ASSET sail-setup"

# The tarball's name comes from the Makefile, not from a second copy of the
# naming rule here -- a copy would agree with itself while the recipe wrote
# somewhere else, and the probes would then be seeding a file nothing reads.
ss_tarball() {  # $1 = case dir
  XDG_CACHE_HOME="$1/cache" make --no-print-directory -C "$REPO" $SS_ASSET \
    sail-pin | sed -n 's/^tarball=//p'
}

# Runs the recipe against a substituted download and reports what it left
# behind: whether the digest comparison spoke at all, whether anything was
# unpacked, and whether the bytes that failed it are still there for the next
# run to serve. `refused` is in there so this cannot read clean because make
# died before reaching the comparison.
ss_aftermath() {  # $1 = case dir
  local tgz log=$1/setup.log
  eval "XDG_CACHE_HOME=$1/cache $SS" > "$log" 2>&1
  tgz=$(ss_tarball "$1")
  printf 'refused=%s unpacked=%s tarball=%s\n' \
    "$(grep -qF 'refusing to extract' "$log" && echo yes || echo no)" \
    "$([ -e "$1/cache/little-cpu/sail/bin/sail_riscv_sim" ] && echo yes || echo none)" \
    "$([ -e "$tgz" ] && echo present || echo gone)"
}

d=$(new_case)
# Exit 2, not 1: the recipe's own `exit 1` reaches the probe as make's status.
probe "a download whose bytes are not the pin is refused before extraction" 2 \
  "SHA-256 MISMATCH -- refusing to extract" "XDG_CACHE_HOME=$d/cache $SS"

d=$(new_case)
probe "the refused download is neither unpacked nor kept to be served again" 0 \
  "refused=yes unpacked=none tarball=gone" "ss_aftermath $d"

d=$(new_case); tgz=$(ss_tarball "$d"); mkdir -p "$(dirname "$tgz")"
echo "not the pinned release either" > "$tgz"
probe "a tarball already in the cache is reused, and still meets the digest" 2 \
  "using the tarball already in" "XDG_CACHE_HOME=$d/cache $SS"

begin_group "test/memmap_test.sh"

# The fixture is a COPY OF THE SHIPPING FILES, not a stub tree, so the control
# below is the real repo and every red probe is one edit away from it. A
# hand-written fixture would drift from the map it is supposed to be checking,
# which is the defect this whole check exists for.
MM="$HERE/memmap_test.sh"

mm_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/test/asm" "$d/test/bench" "$d/formal"
  cp "$REPO"/rtl/memory.v "$REPO"/rtl/timer.v "$REPO"/rtl/uart.v "$REPO"/rtl/spiflash.v \
     "$REPO"/rtl/imemory.v "$REPO"/rtl/littlecpu.v "$REPO"/rtl/littlesoc.v "$d/rtl/"
  cp "$REPO"/test/testbench.v "$REPO"/test/cxxrtl.cc "$REPO"/test/cosim.cc \
     "$REPO"/test/dual_cxxrtl.cc "$d/test/"
  cp "$REPO"/test/asm/riscv_test.h "$REPO"/test/asm/sections.lds \
     "$REPO"/test/asm/boot.lds "$d/test/asm/"
  cp "$REPO"/test/bench/bench.lds "$d/test/bench/"
  cp "$REPO"/formal/traps.sv "$d/formal/"
  cp "$REPO"/Makefile "$d/"
  printf '%s' "$d"
}

d=$(mm_fixture)
probe "control: the shipping files describe one machine" 0 \
  "Memory map agreed on:" "$MM $d"

probe "a repo root that does not exist is red before anything is parsed" 1 \
  "is not a directory" "$MM $d/nowhere"

# THE ONE THAT MATTERS: the original defect, re-entered. The harness modelled a
# RAM sixteen times smaller than the SoC's and every program still fit.
d=$(mm_fixture); sed -i.bak 's/^  memory dmem (/  memory #(.RAM_WORDS(1024)) dmem (/' "$d/test/testbench.v"
probe "the harness sizing its own RAM again is red" 1 \
  "overrides \`memory\`'s parameters" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/^  timer mtimer (/  timer #(.BASE(32'h0003_0000)) mtimer (/" "$d/rtl/littlesoc.v"
probe "the SoC restating the timer base is red too" 1 \
  "rtl/littlesoc.v overrides \`timer\`'s parameters" "$MM $d"

# The UART is the newest region and the one whose baud rate an integrator would
# be most tempted to speed up for a simulation, which is the whole defect.
d=$(mm_fixture); sed -i.bak "s/^  uart tty (/  uart #(.BAUD(1_000_000)) tty (/" "$d/test/testbench.v"
probe "the harness giving the UART its own baud rate is red" 1 \
  "test/testbench.v overrides \`uart\`'s parameters" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/^  uart tty (/  nouart tty (/' "$d/rtl/littlesoc.v"
probe "a SoC with no UART at all does not pass by silence" 1 \
  "does not instantiate \`uart\` at all" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/^  spiflash flash (/  nospiflash flash (/' "$d/rtl/littlesoc.v"
probe "a SoC with no SPI master at all does not pass by silence" 1 \
  "does not instantiate \`spiflash\` at all" "$MM $d"

# Without this the check above passes vacuously on a file that lost its memory.
d=$(mm_fixture); sed -i.bak 's/^  memory dmem (/  nomemory dmem (/' "$d/test/testbench.v"
probe "a harness with no data RAM at all does not pass by silence" 1 \
  "does not instantiate \`memory\` at all" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LENGTH = 64K/LENGTH = 4K/' "$d/test/asm/sections.lds"
probe "a linker script back on the old 4 KB ram is red" 1 \
  "gives \`ram\` 4096 bytes" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/ORIGIN = 0x00010000/ORIGIN = 0x00020000/' "$d/test/asm/boot.lds"
probe "a linker script that moves ram off the decoded base is red" 1 \
  "puts \`ram\` at 0x00020000" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LENGTH = 16K/LENGTH = 8K/' "$d/test/asm/boot.lds"
probe "a suite script linking against a rom the harness has not got" 1 \
  "gives \`rom\` 8192 bytes" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LENGTH = 8K/LENGTH = 32K/' "$d/test/bench/bench.lds"
probe "the benchmark script must keep linking against the part's rom" 1 \
  "test/bench/bench.lds gives \`rom\` 32768 bytes" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/kRamBase = 0x00010000/kRamBase = 0x00011000/' "$d/test/cxxrtl.cc"
probe "the cxxrtl runner's kRamBase drifting is red" 1 \
  "test/cxxrtl.cc's kRamBase is 0x00011000" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/kRamBase = 0x00010000/kRamBase = 0x00011000/' "$d/test/cosim.cc"
probe "the co-sim runner's kRamBase drifting is red on its own" 1 \
  "test/cosim.cc's kRamBase is 0x00011000" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/MTIMER_BASE      0x00020000/MTIMER_BASE      0x00030000/' "$d/test/asm/riscv_test.h"
probe "the timer address the programs arm is checked against the timer" 1 \
  "MTIMER_BASE is 0x00030000" "$MM $d"

# A store to an unmapped address is dropped by every memory on this bus, so the
# programs would wait forever rather than fail.
d=$(mm_fixture); sed -i.bak "s/BASE = 32'h0002_0000/BASE = 32'h0004_0000/" "$d/rtl/timer.v"
probe "a gap opening between the data RAM and the timer is red" 1 \
  "the data RAM ends at 0x00020000 and the timer starts at" "$MM $d"

# The timer decodes four words at one hart and eight at two, so a base that is
# only 16-byte aligned elaborates today and stops elaborating the day the second
# hart lands. That is the failure the reserved span exists to bring forward.
d=$(mm_fixture); sed -i.bak "s/BASE = 32'h0002_0000/BASE = 32'h0002_0010/" "$d/rtl/timer.v"
probe "a timer base aligned only for one hart is red" 1 \
  "0x00020010 is off its reserved" "$MM $d"

# A device in the words the second hart's mtimecmp needs. At one hart they read
# zero from every memory on this bus, so the device would work and the overlap
# would surface only when the dual top was built.
d=$(mm_fixture)
printf "module probe_device #(\n  parameter logic [31:0] BASE = 32'h0002_0010\n) ();\nendmodule\n" \
  > "$d/rtl/probe_device.v"
probe "a peripheral inside the timer's reserved span is red" 1 \
  "rtl/probe_device.v puts its window at 0x00020010" "$MM $d"

# ...and the same device above the span is not, or the check above would be
# refusing every address rather than the reserved ones.
d=$(mm_fixture)
printf "module probe_device #(\n  parameter logic [31:0] BASE = 32'h0004_0000\n) ();\nendmodule\n" \
  > "$d/rtl/probe_device.v"
probe "control: a peripheral above the reserved span is accepted" 0 \
  "Memory map agreed on:" "$MM $d"

# The UART abuts the RESERVED span, not the decoded one -- it starts where the
# second hart's mtimecmp would end. A move in either direction is an overlap or
# a hole, and the OR that joins the read buses would report neither.
d=$(mm_fixture); sed -i.bak "s/BASE     = 32'h0002_0020/BASE     = 32'h0002_0028/" "$d/rtl/uart.v"
probe "a gap opening between the timer's reservation and the UART is red" 1 \
  "the timer reserves through 0x0002001f and the" "$MM $d"

# Its range test reads the address bits above an 8-byte window, which is only a
# membership test while the base is a multiple of 8.
d=$(mm_fixture); sed -i.bak "s/BASE     = 32'h0002_0020/BASE     = 32'h0002_0024/" "$d/rtl/uart.v"
probe "a UART base off its own window is red" 1 \
  "is not a multiple of its own" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/UART_BASE          0x00020020/UART_BASE          0x00030020/' "$d/test/asm/riscv_test.h"
probe "the address the printing program writes is checked against the UART" 1 \
  "UART_BASE is 0x00030020" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/LS_UART_BASE  = 32'h0002_0020/LS_UART_BASE  = 32'h0003_0020/" "$d/rtl/littlecpu.v"
probe "the UART moving in the core's copy alone is red" 1 \
  "LS_UART_BASE is 196640 against rtl/uart.v's 131104" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/LS_UART_BASE  = 32'h0002_0020/LS_UART_BASE  = 32'h0003_0020/" "$d/formal/traps.sv"
probe "the UART moving in the proof's copy alone is red" 1 \
  "formal/traps.sv's LS_UART_BASE is 196640" "$MM $d"

# The SPI master abuts the UART's two words the way the UART abuts the timer's
# reservation, and for the same reason: five read buses join with an OR, so a
# hole is wasted map and an overlap is two live answers at once.
d=$(mm_fixture); sed -i.bak "s/BASE = 32'h0002_0028/BASE = 32'h0002_0030/" "$d/rtl/spiflash.v"
probe "a gap opening between the UART and the SPI master is red" 1 \
  "the UART ends at 0x00020028 and the SPI master starts at" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/BASE = 32'h0002_0028/BASE = 32'h0002_002c/" "$d/rtl/spiflash.v"
probe "an SPI base off its own window is red" 1 \
  "is not a multiple of its own" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/SPI_BASE            0x00020028/SPI_BASE            0x00030028/' "$d/test/asm/riscv_test.h"
probe "the address the flash-reading program uses is checked against the master" 1 \
  "SPI_BASE is 0x00030028" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/LS_FLASH_BASE = 32'h0002_0028/LS_FLASH_BASE = 32'h0003_0028/" "$d/rtl/littlecpu.v"
probe "the SPI master moving in the core's copy alone is red" 1 \
  "LS_FLASH_BASE is 196648 against rtl/spiflash.v's 131112" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/LS_FLASH_BASE = 32'h0002_0028/LS_FLASH_BASE = 32'h0003_0028/" "$d/formal/traps.sv"
probe "the SPI master moving in the proof's copy alone is red" 1 \
  "formal/traps.sv's LS_FLASH_BASE is 196648" "$MM $d"

# MAP_TOP is the address two programs store to expecting a refusal. Left behind
# when a device lands above the topmost window, it names an address that IS
# answered and both programs fail for a reason that is not in the core.
d=$(mm_fixture); sed -i.bak 's/MAP_TOP            0x00020030/MAP_TOP            0x00020028/' "$d/test/asm/riscv_test.h"
probe "a MAP_TOP inside the topmost window is red" 1 \
  "MAP_TOP is 0x00020028" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/^SOC_ROM_WORDS := 2048/SOC_ROM_WORDS := 4096/' "$d/Makefile"
probe "the ROM image built to a different size than the ROM is red" 1 \
  "builds the SoC ROM image for 4096 words" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/localparam int ROM_WORDS = 4096;/localparam int ROM_WORDS = 1024;/' "$d/test/testbench.v"
probe "a simulated ROM smaller than the part's is red" 1 \
  "The harness is allowed to be larger" "$MM $d"

# The parse is load-bearing: a respelled declaration must stop the run rather
# than compare against an empty string.
d=$(mm_fixture); sed -i.bak "s/parameter logic \[31:0\] BASE/parameter logic [31:0] RAM_ORIGIN/" "$d/rtl/memory.v"
probe "a respelled parameter stops rather than comparing nothing" 1 \
  "no \`BASE\` parameter default found in rtl/memory.v" "$MM $d"

d=$(mm_fixture); rm "$d/test/cosim.cc"
probe "a file that moved away takes the check with it, loudly" 1 \
  "test/cosim.cc is missing" "$MM $d"

# Bash arithmetic reads a bare word as a variable name, so an unparsed size
# would otherwise compare as zero and report drift that is really a parse bug.
d=$(mm_fixture); sed -i.bak 's/LENGTH = 64K/LENGTH = LOTS/' "$d/test/asm/sections.lds"
probe "a size the parser cannot read stops rather than comparing as zero" 1 \
  "is not a size this check can read" "$MM $d"

# The core's own copy of the map. Nothing in the datapath reads it, so every one
# of these drifts is silent everywhere else: the counters keep reporting, about
# a machine no file describes.
d=$(mm_fixture); sed -i.bak "s/LS_RAM_BASE   = 32'h0001_0000/LS_RAM_BASE   = 32'h0002_0000/" "$d/rtl/littlecpu.v"
probe "the core's copy of the RAM base drifting from the RAM is red" 1 \
  "rtl/littlecpu.v's LS_RAM_BASE is 131072 against rtl/memory.v's 65536" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LS_RAM_WORDS  = 16384/LS_RAM_WORDS  = 8192/' "$d/rtl/littlecpu.v"
probe "a RAM half the size in the core's copy is red" 1 \
  "LS_RAM_WORDS is 8192 against rtl/memory.v's 16384" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/LS_TIMER_BASE = 32'h0002_0000/LS_TIMER_BASE = 32'h0003_0000/" "$d/rtl/littlecpu.v"
probe "the timer moving in the core's copy alone is red" 1 \
  "LS_TIMER_BASE is 196608 against rtl/timer.v's 131072" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LS_TEXT_WORDS = 2048/LS_TEXT_WORDS = 4096/' "$d/rtl/littlecpu.v"
probe "the default text window is the part's, not the harness's" 1 \
  "LS_TEXT_WORDS is 4096 against rtl/littlesoc.v's 2048" "$MM $d"

# The one the parameter defaults cannot catch: each integrator states its own
# ROM size twice, once to the memory and once to the core.
d=$(mm_fixture); sed -i.bak 's/littlecpu #(.LS_TEXT_WORDS(2048))/littlecpu #(.LS_TEXT_WORDS(4096))/' "$d/rtl/littlesoc.v"
probe "an integrator telling the core a text size its ROM has not got" 1 \
  "gives its \`imemory\` 2048 words of ROM and tells the core the text" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/littlecpu #(.LS_TEXT_WORDS(ROM_WORDS))/littlecpu #(.LS_TEXT_WORDS(2048))/' "$d/test/testbench.v"
probe "the harness passing a literal instead of the size it sized" 1 \
  "gives its \`imemory\` ROM_WORDS words of ROM" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/littlecpu #(.LS_TEXT_WORDS(ROM_WORDS)) uut/littlecpu uut/' "$d/test/testbench.v"
probe "an integrator that stopped stating it at all is red, not defaulted" 1 \
  "names no .ROM_WORDS or no .LS_TEXT_WORDS" "$MM $d"

# The trap proof's copy of the map. Nothing but that proof reads it, so each of
# these drifts is silent everywhere else: components_traps goes on passing,
# having excused the wrong accesses from `must_not_trap`.
d=$(mm_fixture); sed -i.bak "s/LS_RAM_BASE   = 32'h0001_0000/LS_RAM_BASE   = 32'h0002_0000/" "$d/formal/traps.sv"
probe "the proof's copy of the RAM base drifting from the RAM is red" 1 \
  "formal/traps.sv's LS_RAM_BASE is 131072 against rtl/memory.v's 65536" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LS_RAM_WORDS  = 16384/LS_RAM_WORDS  = 8192/' "$d/formal/traps.sv"
probe "a RAM half the size in the proof's copy is red" 1 \
  "LS_RAM_WORDS is 8192 against rtl/memory.v's 16384" "$MM $d"

d=$(mm_fixture); sed -i.bak "s/LS_TIMER_BASE = 32'h0002_0000/LS_TIMER_BASE = 32'h0003_0000/" "$d/formal/traps.sv"
probe "the timer moving in the proof's copy alone is red" 1 \
  "LS_TIMER_BASE is 196608 against rtl/timer.v's 131072" "$MM $d"

d=$(mm_fixture); sed -i.bak 's/LS_TEXT_WORDS = 2048/LS_TEXT_WORDS = 4096/' "$d/formal/traps.sv"
probe "the proof describes the part's text window, not the harness's" 1 \
  "LS_TEXT_WORDS is 4096 against rtl/littlesoc.v's 2048" "$MM $d"

begin_group "test/retired_term_test.sh"

# A COPY OF THE SHIPPING FILES for the same reason test/memmap_test.sh's fixture
# is one, plus a `git init` over it because that check reads git's index rather
# than the filesystem. The copy carries every allow-listed path, so the control
# below is the real allow-list and every red probe is one edit away from it.
RN="$HERE/retired_term_test.sh"

rn_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/docs/adr" "$d/docs/ideas" "$d/test" "$d/formal"
  cp "$REPO/CODE_OF_CONDUCT.md" "$d/"
  cp "$REPO/docs/THREAT_MODEL.md" "$d/docs/"
  # One real file under each history directory rather than all of them: the
  # entries covering those two are directories, and a hundred more copies per
  # fixture would buy nothing but wall time.
  cp "$REPO/docs/adr/README.md" "$d/docs/adr/"
  # The one brief carrying BOTH retired terms, so a single copy covers the
  # docs/ideas/ entry on either list.
  cp "$REPO/docs/ideas/finish-the-rewrite.md" "$d/docs/ideas/"
  cp "$REPO/test/probe_gates.sh" "$REPO/test/retired_term_test.sh" "$d/test/"
  cp "$REPO/formal/wrapper.v" "$d/formal/"
  # -c init.defaultBranch, so the branch-name advice cannot land in the middle of
  # a fixture whose stdout is the directory name.
  git -c init.defaultBranch=main -C "$d" init -q
  git -C "$d" add -A
  printf '%s' "$d"
}

d=$(rn_fixture)
probe "control: the shipping tree keeps the retired term inside its allow-list" 0 \
  "confined to its" "$RN $d"

probe "a repo root that does not exist is red before anything is scanned" 1 \
  "is not a directory" "$RN $d/nowhere"

# THE ONE THAT MATTERS: the original reintroduction, re-entered. That comment was
# written on a branch predating the sweep and merged after it, and nothing
# anywhere objected.
d=$(rn_fixture)
sed -i.bak "s/What they need is/What the ladder needs is/" "$d/formal/wrapper.v"
probe "the word coming back in a formal harness comment is red, and located" 1 \
  "formal/wrapper.v:" "$RN $d"

probe "and the diagnostic says what to write instead, not just what is wrong" 1 \
  'Write "the generated riscv-formal checks"' "$RN $d"

d=$(rn_fixture); printf 'The Ladder is green.\n' > "$d/test/notes.md"
git -C "$d" add -A
probe "a capitalised spelling is the same word and is caught too" 1 \
  "test/notes.md:" "$RN $d"

# The scan is git's index on purpose: a build artifact or an agent worktree under
# the checkout is not something a merge can bring the word back through.
d=$(rn_fixture); printf 'ladder\n' > "$d/test/scratch.log"
probe "control: an untracked file is out of scope and does not fail the build" 0 \
  "confined to its" "$RN $d"

# A directory entry has to match on the path separator, or it silently exempts
# every sibling whose name it happens to prefix.
d=$(rn_fixture); printf 'ladder\n' > "$d/docs/adrenaline.md"
git -C "$d" add -A
probe "a lookalike sibling is not covered by the directory entry above it" 1 \
  "docs/adrenaline.md:" "$RN $d"

# The other direction, which is the half a one-way grep would not have: an
# exemption that outlived the use it was written for.
d=$(rn_fixture)
sed -i.bak 's/enforcement ladder/enforcement sequence/' "$d/CODE_OF_CONDUCT.md"
probe "an allow-list entry whose site no longer has the word is red" 1 \
  "the allow-list exempts CODE_OF_CONDUCT.md" "$RN $d"

d=$(new_case)
probe "a tree git cannot list is a scan of nothing, not a green one" 1 \
  "cannot enumerate any tracked files" "$RN $d"

# THE SECOND TERM: the ISA name the core outgrew. Its list is not the first
# term's, so these probes also demonstrate that the two are graded apart.
d=$(rn_fixture)
sed -i.bak "s/What they need is/What an RV32IMC core needs is/" "$d/formal/wrapper.v"
probe "the stale ISA name coming back in a formal harness comment is red" 1 \
  "formal/wrapper.v:" "$RN $d"

probe "and the diagnostic names the ISA the core does claim" 1 \
  "Write RV32IMAC" "$RN $d"

# THE ONE THAT DECIDES WHETHER THIS TERM IS USABLE AT ALL. The lower-case
# spelling is a live argument to riscv-formal's generator, not the retired prose
# name, and a check that caught it would be red on the shipping tree forever.
d=$(rn_fixture); printf 'isa rv32imc\n' > "$d/formal/checks.cfg"
git -C "$d" add -A
probe "control: the formal flow's lower-case rv32imc is a different thing" 0 \
  "confined to its" "$RN $d"

# The other direction, on the second term's own list: docs/adr/ is exempt for
# both words, and losing one of them has to be red for that one alone.
d=$(rn_fixture); sed -i.bak 's/RV32IMC/RV32IMAC/g' "$d/docs/adr/README.md"
probe "an allow-list entry whose site lost the stale ISA name is red" 1 \
  "the allow-list exempts docs/adr/" "$RN $d"

# The table itself. These three run the FIXTURE's copy, because what they edit is
# the table inside it -- a term added with no list behind it grades against
# nothing, and a scan of nothing is the failure this whole file exists for.
RNF="test/retired_term_test.sh"

d=$(rn_fixture); sed -i.bak 's/^ladder any-case$/ladder some-case/' "$d/$RNF"
probe "a casing keyword the table does not define stops rather than guessing" 1 \
  "which is neither" "$d/$RNF $d"

d=$(rn_fixture); sed -i.bak 's/^ladder any-case$/ladders any-case/' "$d/$RNF"
probe "a term with no allow-list behind it stops rather than grading nothing" 1 \
  "no allow-list is written for the term" "$d/$RNF $d"

d=$(rn_fixture)
sed -i.bak -e 's/^ladder any-case$//' -e 's/^RV32IMC exact-case$//' "$d/$RNF"
probe "an empty term table is a scan of nothing, not a green one" 1 \
  "the term table is empty" "$d/$RNF $d"

begin_group "test/march_test.sh"

# A COPY OF THE SHIPPING FILES again, plus a `git init` over it, for the reasons
# the two fixtures above give: the control is then the real set of build sites
# and every red probe is one edit away from it. A hand-written fixture would
# drift from the flags it is supposed to be comparing, which is the defect.
MA="$HERE/march_test.sh"

ma_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/test/sail" "$d/soc/depth" "$d/soc/compare" "$d/formal" \
           "$d/docs/adr" "$d/docs/ideas"
  cp "$REPO/CLAUDE.md" "$REPO/Makefile" "$d/"
  cp "$REPO/test/run_tests.sh" "$REPO/test/cosim.py" "$REPO/test/march_test.sh" \
     "$REPO/test/dual_build.sh" "$REPO/test/probe_gates.sh" "$d/test/"
  cp "$REPO/test/sail/reservation_probe.sh" "$d/test/sail/"
  cp "$REPO/soc/depth/cycles.py" "$d/soc/depth/"
  cp "$REPO/soc/compare/run_dhrystone.sh" "$d/soc/compare/"
  cp "$REPO/formal/checks.cfg" "$d/formal/"
  # One real file under each history directory: those two entries are
  # directories, and a hundred more copies per fixture would buy only wall time.
  cp "$REPO/docs/adr/0106-the-a-extension-is-built-and-the-board-still-closes.md" "$d/docs/adr/"
  cp "$REPO/docs/ideas/the-a-extension-lands-single-hart.md" "$d/docs/ideas/"
  git -c init.defaultBranch=main -C "$d" init -q
  git -C "$d" add -A
  printf '%s' "$d"
}

# sed's backup goes away before the tree is re-indexed. This check scans every
# TRACKED file, so a leftover `Makefile.bak` would be a second copy of the
# unedited flags for it to grade -- the probes would still go red, for a reason
# that is not the one they name.
ma_edit() {  # $1 = fixture dir, $2 = path within it, $3 = sed expression
  sed -i.bak "$3" "$1/$2"
  rm -f "$1/$2.bak"
  git -C "$1" add -A
}

d=$(ma_fixture)
probe "control: the shipping tree names one ISA at every site" 0 \
  "at all 7 sites" "$MA $d"

probe "a repo root that does not exist is red before anything is scanned" 1 \
  "is not a directory" "$MA $d/nowhere"

# THE ONE THAT MATTERS: a site left behind. The `.c` arm of the suite runner
# assembles a program with no atomic in it either way.
d=$(ma_fixture)
ma_edit "$d" test/run_tests.sh '158s/rv32imac_zicsr_zifencei_zkt/rv32imc_zicsr_zifencei_zkt/'
probe "one build site left at the narrower ISA is red, and located" 1 \
  "test/run_tests.sh:158: -march=rv32imc_zicsr_zifencei_zkt" "$MA $d"

probe "...and the count that site was declared with is red too" 1 \
  "test/run_tests.sh states -march=rv32imac_zicsr_zifencei_zkt 1 time(s), not 2" "$MA $d"

# The silent one. Nothing about this changes whether anything builds.
d=$(ma_fixture)
ma_edit "$d" Makefile \
  's/^DHRY_CFLAGS := -march=rv32imac_zicsr_zifencei_zkt/DHRY_CFLAGS := -march=rv32imc_zicsr_zifencei_zkt/'
probe "the Dhrystone flags drifting from the suite's ISA is red" 1 \
  "Makefile states -march=rv32imac_zicsr_zifencei_zkt 3 time(s), not 4" "$MA $d"

probe "...and their second copy is compared whole, not just its ISA" 1 \
  "the Dhrystone flags are stated twice and they disagree" "$MA $d"

# The whole-string comparison on its own: same ISA, different optimiser.
d=$(ma_fixture)
ma_edit "$d" soc/depth/cycles.py \
  's/-mabi=ilp32 -O2 -std=c11 -ffreestanding/-mabi=ilp32 -O3 -std=c11 -ffreestanding/'
probe "two copies of the flags agreeing about -march and nothing else" 1 \
  "-O3" "$MA $d"

# The other direction, which is the half a one-way sweep would not have. This
# is the one probe that runs the FIXTURE'S copy of the script rather than the
# shipping one, because what it moves is the declaration inside it.
d=$(ma_fixture)
ma_edit "$d" test/march_test.sh 's/rv32imac_zicsr_zifencei/rv32imafc_zicsr_zifencei/'
probe "moving the declared string alone, with every site unchanged, is red" 1 \
  "CLAUDE.md states -march=rv32imafc_zicsr_zifencei_zkt 0 time(s), not 1" \
  "$d/test/march_test.sh $d"

# The two spellings that must not move with it. Widening either generates
# nothing at the pin.
d=$(ma_fixture)
ma_edit "$d" formal/checks.cfg 's/^isa rv32imc$/isa rv32imac/'
probe "the generated check set's isa line swept along with the flags is red" 1 \
  "formal/checks.cfg no longer says \`isa rv32imc\`" "$MA $d"

d=$(ma_fixture)
ma_edit "$d" Makefile 's/generate.py -i rv32imc /generate.py -i rv32imac /'
probe "the monitor generator's -i swept along with them is red as well" 1 \
  "MONITOR_GEN no longer passes \`-i rv32imc\`" "$MA $d"

# An unnamed ISA anywhere, which is what a new build site arriving looks like.
d=$(ma_fixture)
printf '#!/bin/sh\nriscv64-elf-gcc -march=rv32e -o x y.c\n' > "$d/test/newbuild.sh"
git -C "$d" add -A
probe "a new site naming an ISA nothing declared is red, and located" 1 \
  "test/newbuild.sh:2: -march=rv32e" "$MA $d"

# ...and the exception list's own both-ways direction.
d=$(ma_fixture)
ma_edit "$d" Makefile 's/-march=rv32i -mabi=ilp32/-march=rv32imac_zicsr_zifencei -mabi=ilp32/'
probe "an exception whose site stopped naming that ISA is red" 1 \
  "the exception list names \`Makefile rv32i\`" "$MA $d"

# A directory entry has to match on the path separator, or it silently exempts
# every sibling whose name it happens to prefix.
d=$(ma_fixture)
printf 'built at -march=rv32im_zicsr once.\n' > "$d/docs/adrenaline.md"
git -C "$d" add -A
probe "a lookalike sibling is not covered by the directory entry above it" 1 \
  "docs/adrenaline.md:1: -march=rv32im_zicsr" "$MA $d"

# A declaration this cannot read must stop the run rather than compare against
# an empty string, which is how a check reports green over a file it has
# stopped understanding.
d=$(ma_fixture)
ma_edit "$d" Makefile 's/^DHRY_CFLAGS :=/DHRY_CFLAGS_RENAMED :=/'
probe "a respelled flag declaration stops rather than comparing nothing" 1 \
  "no Dhrystone flag string could be read out of Makefile" "$MA $d"

d=$(new_case)
probe "a tree git cannot list is a scan of nothing, not a green one" 1 \
  "cannot enumerate any tracked files" "$MA $d"

begin_group "soc/fit_report.py"

FR="python3 $REPO/soc/fit_report.py"

fr_fixture() {
  local d; d=$(new_case)
  cat > "$d/fit.log" <<'LOG'
Warning: No PCF file specified; IO pins will be placed automatically

Info: Packing constants..
Info: Device utilisation:
Info: 	         ICESTORM_LC:    3875/   5280    73%
Info: 	        ICESTORM_RAM:       4/     30    13%
Info: 	               SB_IO:     263/     39   674%
Info: 	               SB_GB:       8/      8   100%

Info: Placed 0 cells based on constraints.
ERROR: Unable to find a placement location for cell 'imem_addr[20]$sb_io'
1 warning, 1 error
LOG
  printf '%s' "$d"
}

d=$(fr_fixture)
probe "control: a measurement within budget is green" 0 "RATCHET:" \
  "$FR $d/fit.log --max-lc 4100"

d=$(fr_fixture)
probe "over budget names the count and the budget" 1 \
  "is over the 3800-cell budget" "$FR $d/fit.log --max-lc 3800"

d=$(fr_fixture); sed -i.bak '/ICESTORM_LC:/d' "$d/fit.log"
probe "no utilisation table is a failure, not a 0% fit" 1 \
  "printed no utilisation table" "$FR $d/fit.log --max-lc 4100"

# The trend line is a diagnostic and these three probes are what keep it one: a
# second number that could redden the job would be a ratchet nobody derived, and
# it would go red for churn, which is the thing it exists to make legible.
d=$(fr_fixture)
probe "the trend against the recorded count is printed beside the verdict" 0 \
  "TREND: +75 cells against the 3800" "$FR $d/fit.log --max-lc 4100 --previous 3800"

d=$(fr_fixture)
probe "a move far outside the churn band still cannot fail the job" 0 "TREND: +875" \
  "$FR $d/fit.log --max-lc 4100 --previous 3000"

d=$(fr_fixture)
probe "a trip prints the trend too, since that is the run that needs it" 1 \
  "TREND: +75" "$FR $d/fit.log --max-lc 3800 --previous 3800"

begin_group "formal/check-interrupt-tie-off.py"

IT="python3 $REPO/formal/check-interrupt-tie-off.py"

# The riscv-formal stand-in carries only what the upstream half reads: two
# files that mention rvfi_intr and one that does not, so the control is
# non-empty in both directions.
it_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/formal" "$d/rf/checks"
  for f in wrapper.v complete.sv cover.sv dmemcheck.sv imemcheck.sv; do
    cp "$REPO/formal/$f" "$d/formal/$f"
  done
  printf 'wire intr = rvfi_intr[0];\n' > "$d/rf/checks/rvfi_pc_fwd_check.sv"
  printf 'wire intr = rvfi_intr[0];\n' > "$d/rf/checks/rvfi_insn_check.sv"
  printf 'assert (rvfi_valid);\n'      > "$d/rf/checks/rvfi_reg_check.sv"
  {
    printf 'HARNESS wrapper.v\nHARNESS complete.sv\nHARNESS cover.sv\n'
    printf 'HARNESS dmemcheck.sv\nHARNESS imemcheck.sv\n'
    printf 'UPSTREAM checks/rvfi_pc_fwd_check.sv\n'
    printf 'UPSTREAM checks/rvfi_insn_check.sv\n'
  } > "$d/BASELINE"
  printf '%s' "$d"
}

its() { printf "%s %s/formal %s/BASELINE %s/rf" "$IT" "$1" "$1" "$1"; }

d=$(it_fixture)
probe "control: the shipping harnesses tie off, both directions" 0 \
  "INTERRUPT TIE-OFF: PASS" "$(its "$d")"

probe "wrong argument count is exit 2" 2 "check-interrupt-tie-off.py" \
  "$IT $d/formal"

# The failure that matters most: a harness the baseline claims is tied off and
# is not. Its checks would be running against a machine with interrupts, at
# depths derived without them.
d=$(it_fixture); sed -i.bak "/\.irq_timer(1'b0),/d" "$d/formal/complete.sv"
probe "a declared harness that does not tie the input off is red" 1 \
  "does not connect .irq_timer" "$(its "$d")"

d=$(it_fixture); sed -i.bak '/^HARNESS cover.sv$/d' "$d/BASELINE"
probe "a harness with no line in the baseline is red" 1 \
  "does not name it" "$(its "$d")"

d=$(it_fixture); rm "$d/formal/cover.sv"
probe "a line with no harness behind it is red too" 1 \
  "which does not instantiate littlecpu" "$(its "$d")"

# The re-derivation from the pin, in both directions. This is what a baseline
# alone cannot do: a stale tie-off covers less and less while staying green.
d=$(it_fixture); printf 'if (!rvfi_intr[0]) assert(0);\n' > "$d/rf/checks/rvfi_unique_check.sv"
probe "a pin that makes another check read rvfi_intr is red" 1 \
  "may now have something to say" "$(its "$d")"

d=$(it_fixture); printf 'nothing to see here\n' > "$d/rf/checks/rvfi_insn_check.sv"
probe "a baselined upstream file that stopped mentioning it is red" 1 \
  "was not re-derived" "$(its "$d")"

d=$(it_fixture); printf 'assert (csr_mstatus_wdata == 0);\n' >> "$d/rf/checks/rvfi_reg_check.sv"
probe "a pin that models an interrupt CSR makes the tie-off an argument again" 1 \
  "now has a model of interrupt state" "$(its "$d")"

d=$(it_fixture); printf 'HARNESS\n' >> "$d/BASELINE"
probe "a malformed baseline line is named rather than skipped" 1 \
  "expected \`HARNESS <path>\`" "$(its "$d")"

d=$(it_fixture); rm -rf "$d/rf/checks"
probe "an unreadable clone makes the re-derivation impossible, and fatal" 1 \
  "is not a directory" "$(its "$d")"

begin_group "formal/check-multihart-tie-off.py"

MT="python3 $REPO/formal/check-multihart-tie-off.py"

# The real harnesses, the real port list and the real parser: this check reads
# littlecpu's ports through test/port_connect_test.py rather than through a
# second regex, so a fixture that stubbed either would be probing neither.
mt_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/formal" "$d/repo/rtl" "$d/repo/test"
  for f in wrapper.v complete.sv cover.sv dmemcheck.sv imemcheck.sv; do
    cp "$REPO/formal/$f" "$d/formal/$f"
  done
  cp "$REPO/rtl/littlecpu.v" "$d/repo/rtl/littlecpu.v"
  cp "$REPO/test/port_connect_test.py" "$d/repo/test/port_connect_test.py"
  {
    printf 'HARNESS wrapper.v\nHARNESS complete.sv\nHARNESS cover.sv\n'
    printf 'HARNESS dmemcheck.sv\nHARNESS imemcheck.sv\n'
    printf "PORT bus_wait 1'b0\nPORT snoop_write 1'b0\nPORT snoop_addr 32'b0\n"
    printf 'ELSEWHERE irq_timer INTERRUPT_TIE_OFF\n'
  } > "$d/BASELINE"
  printf '%s' "$d"
}

mts() { printf "%s %s/formal %s/BASELINE %s/repo" "$MT" "$1" "$1" "$1"; }

d=$(mt_fixture)
probe "control: the shipping harnesses tie off, both directions" 0 \
  "MULTI-HART TIE-OFF: PASS" "$(mts "$d")"

probe "wrong argument count is exit 2" 2 "check-multihart-tie-off.py" \
  "$MT $d/formal"

# The failure that matters most: a harness the baseline claims is tied off and
# is not. Its checks would be running against a machine whose bus another agent
# can take away, at depths derived where nobody can.
d=$(mt_fixture); sed -i.bak "s/\.bus_wait(1'b0)/.bus_wait(free_wait)/" "$d/formal/complete.sv"
probe "a declared harness that does not tie the input off is red" 1 \
  "connects .bus_wait(free_wait)" "$(mts "$d")"

d=$(mt_fixture); sed -i.bak '/^HARNESS cover.sv$/d' "$d/BASELINE"
probe "a harness with no line in the baseline is red" 1 \
  "does not name it" "$(mts "$d")"

d=$(mt_fixture); rm "$d/formal/cover.sv"
probe "a line with no harness behind it is red too" 1 \
  "which does not instantiate littlecpu" "$(mts "$d")"

# The re-derivation from the RTL, which is what a baseline alone cannot do: a
# port renamed leaves the line behind declaring a tie-off of nothing.
d=$(mt_fixture); printf "PORT no_such_port 1'b0\n" >> "$d/BASELINE"
probe "a declared port littlecpu does not have is red" 1 \
  "is not an input of littlecpu" "$(mts "$d")"

# ...and the direction that rots: the next tied-off input landing with the
# depths derived under it and nothing written down.
d=$(mt_fixture); sed -i.bak "/^PORT snoop_write /d" "$d/BASELINE"
probe "a tie-off at every harness that no baseline declares is red" 1 \
  "and no baseline says so" "$(mts "$d")"

d=$(mt_fixture); printf 'PORT\n' >> "$d/BASELINE"
probe "a malformed baseline line is named rather than skipped" 1 \
  "expected \`HARNESS <path>\`" "$(mts "$d")"

d=$(mt_fixture); rm "$d/repo/test/port_connect_test.py"
probe "a missing parser stops the run rather than grading with a second one" 2 \
  "port_connect_test.py is missing" "$(mts "$d")"

begin_group "soc/compare/placed_vs_synth.py"

PS="python3 $REPO/soc/compare/placed_vs_synth.py"

# The numbers are the ones this repo actually measured: 2379 placed logic cells
# against 1711 synthesised, and the 449-against-1711 that the all-NOP ROM
# produced and that nothing caught.
ps_fixture() {
  local d; d=$(new_case)
  cat > "$d/pnr.log" <<'LOG'
Info: Device utilisation:
Info: 	         ICESTORM_LC:    2379/   7680    30%
Info: 	        ICESTORM_RAM:      30/     32    93%
LOG
  cat > "$d/core.log" <<'LOG'
     1709   SB_LUT4
       18   SB_RAM40_4K
     1711   SB_LUT4
LOG
  printf '%s' "$d"
}

d=$(ps_fixture)
probe "control: a placement holding the whole core is green" 0 "RATCHET:" \
  "$PS $d/pnr.log $d/core.log vexriscv --min-ratio 0.8"

# THE ONE THAT MATTERS: the defect this gate was written for.
d=$(ps_fixture); sed -i.bak 's/2379\/   7680    30%/ 449\/   7680     5%/' "$d/pnr.log"
probe "a core yosys folded away is red, not a fast design" 1 \
  "under the 0.80x floor" "$PS $d/pnr.log $d/core.log vexriscv --min-ratio 0.8"

d=$(ps_fixture); sed -i.bak '/ICESTORM_LC/d' "$d/pnr.log"
probe "no utilisation table means nothing was placed, not that nothing was lost" 1 \
  "no ICESTORM_LC utilisation line" \
  "$PS $d/pnr.log $d/core.log vexriscv --min-ratio 0.8"

d=$(ps_fixture); sed -i.bak '/SB_LUT4/d' "$d/core.log"
probe "no standalone count leaves nothing to compare against" 1 \
  "no SB_LUT4 count" "$PS $d/pnr.log $d/core.log vexriscv --min-ratio 0.8"

# Without this the ratio is a division by zero, which would raise rather than
# report -- and a traceback is not a diagnostic.
d=$(ps_fixture); sed -i.bak 's/^     1711   SB_LUT4/        0   SB_LUT4/' "$d/core.log"
probe "a standalone synthesis of zero cells is named, not divided by" 1 \
  "no SB_LUT4 count" "$PS $d/pnr.log $d/core.log vexriscv --min-ratio 0.8"

begin_group "soc/compare/geometry_test.sh"

# A COPY OF THE SHIPPING FILES for the same reason test/memmap_test.sh's fixture
# is one: the control has to be the real harness, so every red probe below is
# one edit away from what is actually measured.
GT="$REPO/soc/compare/geometry_test.sh"

gt_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/soc/compare"
  cp "$REPO"/rtl/memory.v "$d/rtl/"
  cp "$REPO"/soc/compare/bench_littlecpu.v "$REPO"/soc/compare/bench_vexriscv.v \
     "$REPO"/soc/compare/bench.lds "$REPO"/soc/compare/bench.S "$d/soc/compare/"
  cp "$REPO"/Makefile "$d/"
  printf '%s' "$d"
}

d=$(gt_fixture)
probe "control: the shipping harness states one geometry" 0 \
  "stated the same way in all six places" "$GT $d"

probe "a repo root that does not exist is red before anything is parsed" 1 \
  "is not a directory" "$GT $d/nowhere"

d=$(gt_fixture); sed -i.bak 's/parameter integer ROM_WORDS = 1024/parameter integer ROM_WORDS = 2048/' \
  "$d/soc/compare/bench_vexriscv.v"
probe "one core's ROM growing behind the other's is red" 1 \
  "has ROM_WORDS=2048, the Makefile has COMPARE_ROM_WORDS=1024" "$GT $d"

d=$(gt_fixture); sed -i.bak 's/parameter integer RAM_WORDS = 512/parameter integer RAM_WORDS = 256/' \
  "$d/soc/compare/bench_littlecpu.v"
probe "one core's data RAM shrinking behind the other's is red" 1 \
  "has RAM_WORDS=256, the Makefile has COMPARE_RAM_WORDS=512" "$GT $d"

d=$(gt_fixture); sed -i.bak 's/LENGTH = 4K/LENGTH = 8K/' "$d/soc/compare/bench.lds"
probe "a linker script linking past the harness ROM is red" 1 \
  "rom region is 8192 bytes, the harness ROM is 4096" "$GT $d"

d=$(gt_fixture); sed -i.bak 's/LENGTH = 2K/LENGTH = 16K/' "$d/soc/compare/bench.lds"
probe "a linker script promising RAM the harness does not have is red" 1 \
  "ram region is 16384 bytes, the harness RAM is 2048" "$GT $d"

d=$(gt_fixture); sed -i.bak 's/li      t0, 0x00010000/li      t0, 0x00020000/' \
  "$d/soc/compare/bench.S"
probe "the program addressing RAM somewhere the harness has none is red" 1 \
  "addresses RAM at 0x00020000" "$GT $d"

d=$(gt_fixture); sed -i.bak 's/ORIGIN = 0x00010000/ORIGIN = 0x00020000/' \
  "$d/soc/compare/bench.lds"
probe "the linker script's RAM origin drifting from the RTL's base is red" 1 \
  "ram ORIGIN is 0x00020000" "$GT $d"

d=$(gt_fixture); rm "$d/soc/compare/bench.lds"
probe "a file that moved out from under the check is fatal, not skipped" 1 \
  "is missing" "$GT $d"

d=$(gt_fixture); sed -i.bak 's/^COMPARE_ROM_WORDS/COMPARE_ROMWORDS/' "$d/Makefile"
probe "a declaration this cannot read stops rather than comparing empty strings" 1 \
  "teach this script the new" "$GT $d"

begin_group "soc/compare/dhry_fit.py"

DF="python3 $REPO/soc/compare/dhry_fit.py"

# The measured image, the measured geometries and both cores' measured block RAM
# counts, so every red probe below is one edit away from the real run.
df_fixture() {
  local d; d=$(new_case)
  printf '     6052   SB_LUT4\n        4   SB_RAM40_4K\n' > "$d/ours.log"
  printf '     1711   SB_LUT4\n       18   SB_RAM40_4K\n' > "$d/theirs.log"
  cat > "$d/tb.v" <<'TB'
  localparam int ROM_WORDS = 2048;
  localparam int RAM_WORDS = 4096;
TB
  printf '%s' "$d"
}

df_args() {  # $1 = fixture dir
  printf -- '--rom-bytes 1528 --ram-bytes 10572 --placed-rom 4096 --placed-ram 2048 '
  printf -- '--sim-rom 8192 --sim-ram 16384 --tb %s/tb.v ' "$1"
  printf -- '--core littlecpu=%s/ours.log --core vexriscv=%s/theirs.log' "$1" "$1"
}

d=$(df_fixture)
probe "control: the measured image reports the shortfall it has" 0 \
  "DOES NOT FIT THE PLACED GEOMETRY" "$DF $(df_args "$d")"

# The point of the script: a run whose memories no ice40 in this flow can hold
# has to say so beside its numbers, every time.
d=$(df_fixture)
probe "a core that cannot hold the image is named, not left to the reader" 0 \
  "vexriscv   18 of its own + 26 for the image =  44 blocks: DOES NOT FIT" \
  "$DF $(df_args "$d")"

# One geometry, two files. A testbench simulating memories the linker script
# does not describe is two machines reported as one.
d=$(df_fixture); sed -i.bak 's/RAM_WORDS = 4096/RAM_WORDS = 2048/' "$d/tb.v"
probe "a testbench simulating a different map than the image is linked for is red" 1 \
  "the simulated geometry does not agree with itself" "$DF $(df_args "$d")"

d=$(df_fixture); sed -i.bak '/RAM_WORDS/d' "$d/tb.v"
probe "a parameter this cannot read stops rather than comparing nothing" 1 \
  "script the new spelling rather than dropping" "$DF $(df_args "$d")"

# ld refuses a .text overflow; nothing refuses a .bss past the end of RAM.
d=$(df_fixture)
probe "data past the end of the simulated RAM is red, not silent" 1 \
  "does not fit the simulated geometry" "$DF $(df_args "$d") --ram-bytes 20000"

d=$(df_fixture); sed -i.bak '/SB_RAM40_4K/d' "$d/theirs.log"
probe "a census with no block RAM line is a synthesis that did not finish" 1 \
  "no SB_RAM40_4K line" "$DF $(df_args "$d")"

d=$(df_fixture)
probe "an empty image is named rather than reported as fitting easily" 1 \
  "an empty image is not a measurement" "$DF $(df_args "$d") --rom-bytes 0"

begin_group "soc/compare/dhry_dmips.py"

DD="python3 $REPO/soc/compare/dhry_dmips.py"

# The numbers this repo measured, at 400 runs.
dd_fixture() {
  local d; d=$(new_case)
  cat > "$d/run.log" <<'LOG'
DHRY ran 431000 cycles of a 2000000 cycle limit
DHRY core=littlecpu marks=2 cycles=335229 verdict=1 writes=31474
DHRY core=vexriscv marks=2 cycles=408758 verdict=1 writes=31474
DHRY ramdiff=0 of=4096 words
LOG
  printf '%s' "$d"
}

d=$(dd_fixture)
probe "control: a good run reports both cores' figures" 0 "0.679" \
  "$DD $d/run.log --runs 400"

d=$(dd_fixture)
probe "a clock turns the per-MHz figure into an absolute one" 0 "22.10" \
  "$DD $d/run.log --runs 400 --mhz littlecpu=32.54 --mhz vexriscv=48.19"

# THE ONE THAT MATTERS: two cores that did not compute the same thing have no
# comparable cycle count between them.
d=$(dd_fixture); sed -i.bak 's/ramdiff=0/ramdiff=111/' "$d/run.log"
probe "two cores whose RAMs differ are red, not a 1.2x result" 1 \
  "data RAMs differ in 111 of 4096 words" "$DD $d/run.log --runs 400"

d=$(dd_fixture); sed -i.bak 's/of=4096/of=0/' "$d/run.log"
probe "a RAM comparison over no words is named as unable to fail" 1 \
  "could not have failed" "$DD $d/run.log --runs 400"

d=$(dd_fixture); sed -i.bak '/ramdiff/d' "$d/run.log"
probe "a run that never made the cross-core check is red" 1 \
  "no ramdiff line" "$DD $d/run.log --runs 400"

d=$(dd_fixture); sed -i.bak 's/core=vexriscv marks=2/core=vexriscv marks=1/' "$d/run.log"
probe "a core that reached the start of the loop and not the end is red" 1 \
  "published 1 marker(s), not 2" "$DD $d/run.log --runs 400"

d=$(dd_fixture); sed -i.bak 's/core=littlecpu marks=2 cycles=335229 verdict=1/core=littlecpu marks=2 cycles=335229 verdict=3/' \
  "$d/run.log"
probe "the benchmark's own FAIL verdict stops the number being quoted" 1 \
  "did not compute the published results" "$DD $d/run.log --runs 400"

d=$(dd_fixture); sed -i.bak '/core=vexriscv/d' "$d/run.log"
probe "one side alone is not a cross-core figure" 1 \
  "no result for vexriscv" "$DD $d/run.log --runs 400"

d=$(dd_fixture); sed -i.bak 's/^DHRY core=/DHRY CORE=/' "$d/run.log"
probe "a simulation this cannot parse is a run that did not happen" 1 \
  "no DHRY result lines" "$DD $d/run.log --runs 400"

d=$(dd_fixture)
probe "zero runs would divide the work by nothing" 1 "nothing was measured" \
  "$DD $d/run.log --runs 0"

d=$(dd_fixture)
probe "a clock for a core nobody graded is named rather than ignored" 1 \
  "which is not one of the cores graded" \
  "$DD $d/run.log --runs 400 --mhz picorv32=40"

d=$(dd_fixture)
probe "a placement at zero MHz is not a placement" 1 "is not placed" \
  "$DD $d/run.log --runs 400 --mhz littlecpu=0"

begin_group "test/port_connect_test.py"

# A COPY OF THE SHIPPING FILES for the same reason test/memmap_test.sh's fixture
# is one, plus a `git init` over it because that check enumerates tracked files
# rather than walking the filesystem. formal/wrapper.v is the only one of the
# five formal harnesses copied in: all five wire the core the same way, through
# `RVFI_CONN, so a second one would buy wall time and no coverage.
PC="python3 $REPO/test/port_connect_test.py"

pc_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/test" "$d/soc/compare" "$d/formal"
  cp "$REPO"/rtl/littlecpu.v "$REPO"/rtl/littlesoc.v "$d/rtl/"
  cp "$REPO"/test/testbench.v "$d/test/"
  cp "$REPO"/soc/compare/bench_littlecpu.v "$d/soc/compare/"
  cp "$REPO"/formal/wrapper.v "$d/formal/"
  git -c init.defaultBranch=main -C "$d" init -q
  git -C "$d" add -A
  printf '%s' "$d"
}

d=$(pc_fixture)
probe "control: every site names every port of littlecpu" 0 \
  "named at every one of" "$PC $d"

probe "a repo root that does not exist is red before anything is parsed" 1 \
  "is not a directory" "$PC $d/nowhere"

# THE ONE THAT MATTERS: the original defect, re-entered. This instance really did
# miss `imem_fault`, and the harness then placed 536 cells of a 6006-cell core.
d=$(pc_fixture); sed -i.bak '/\.imem_fault(imem_fault),/d' "$d/soc/compare/bench_littlecpu.v"
probe "a port the comparison harness stops naming is red, and located" 1 \
  "soc/compare/bench_littlecpu.v's \`riscv\` instance does not connect .imem_fault" "$PC $d"

probe "and the diagnostic says what it costs, not just that it is missing" 1 \
  "placed 536 cells of a 6006-cell core" "$PC $d"

# An empty connection is a floating pin spelled a second way, and yosys accepts
# both.
d=$(pc_fixture); sed -i.bak 's/\.imem_fault(imem_fault),/.imem_fault(),/' \
  "$d/soc/compare/bench_littlecpu.v"
probe "a port named with nothing in the parentheses is red too" 1 \
  "names .imem_fault() with nothing in it" "$PC $d"

# The other direction, which is the half a one-way check would not have: the
# port goes away and the harnesses keep naming it.
d=$(pc_fixture); sed -i.bak 's/^  input  logic        imem_fault,//' "$d/rtl/littlecpu.v"
probe "a connection to a port littlecpu no longer has is red" 1 \
  "connects .imem_fault, and littlecpu has no such port" "$PC $d"

d=$(pc_fixture); sed -i.bak '/\.rvfi_mem_rmask(rvfi_mem_rmask),/d' "$d/test/testbench.v"
probe "half of a macro-guarded group is red, where none of it is not" 1 \
  "connects littlecpu under RISCV_FORMAL but not .rvfi_mem_rmask" "$PC $d"

# A port declared unconditionally and connected only under a macro floats
# wherever that macro is absent, which is every build but one.
d=$(pc_fixture); sed -i.bak -e 's/^    \.irq_timer(irq_timer),$/    .irq_timer(irq_timer)/' \
  -e 's/^    \.trap(trap)$//' -e 's/^    , \.rvfi_valid/    , .trap(trap), .rvfi_valid/' \
  "$d/test/testbench.v"
probe "an unconditional port connected only inside an ifdef is red" 1 \
  "connects .trap only under RISCV_FORMAL" "$PC $d"

# `RVFI_CONN is the one macro this check cannot expand, so a harness dropping it
# has to be caught by something other than the port list.
d=$(pc_fixture); sed -i.bak -e 's/^    \.trap(trap),$/    .trap(trap)/' \
  -e '/`RVFI_CONN/d' "$d/formal/wrapper.v"
probe "a formal harness that stops wiring rvfi at all is red" 1 \
  "carries no \`RVFI_CONN" "$PC $d"

# The exception table both ways round: an omission that has been fixed leaves an
# entry behind, and an exemption kept past its reason is how the next one gets
# waved through.
d=$(pc_fixture); sed -i.bak 's/^    , \.rvfi_valid(rvfi_valid),/    , .rvfi_valid(rvfi_valid), .rvfi_mode(rvfi_mode),/' \
  "$d/test/testbench.v"
probe "an exception whose port is connected now is red" 1 \
  "EXCEPTIONS exempts .rvfi_mode at test/testbench.v" "$PC $d"

# A positional connection re-aims every port after the one that moved, so it
# stops the run rather than being graded as far as it can be read.
d=$(pc_fixture); sed -i.bak 's/^    \.clk(clk),/    clk,/' "$d/soc/compare/bench_littlecpu.v"
probe "a connection by position stops rather than being half-read" 1 \
  "connects littlecpu by something this check cannot read" "$PC $d"

# The last defect in this file class was a superfluous comma in a port list,
# which yosys accepts and only iverilog and svlint rejected.
d=$(pc_fixture); sed -i.bak 's/^    \.trap(trap)$/    .trap(trap),/' "$d/rtl/littlesoc.v"
probe "a stray comma in a connection list is named as one" 1 \
  "a stray or trailing comma" "$PC $d"

# The parse is load-bearing: a port this cannot read would go undemanded at
# every site rather than reported at one.
d=$(pc_fixture); sed -i.bak 's/^  input  logic clk,/  clk,/' "$d/rtl/littlecpu.v"
probe "a port declaration the parser cannot read stops the run" 1 \
  "cannot read as a port declaration" "$PC $d"

d=$(pc_fixture); rm "$d/rtl/littlecpu.v"
probe "the module file moving away takes the check with it, loudly" 1 \
  "rtl/littlecpu.v is missing" "$PC $d"

# Without these two the whole check passes over a tree it never read.
d=$(new_case); mkdir -p "$d/rtl"; cp "$REPO/rtl/littlecpu.v" "$d/rtl/"
git -c init.defaultBranch=main -C "$d" init -q; git -C "$d" add -A
probe "a tree that instantiates the core nowhere is not a clean one" 1 \
  "instantiates littlecpu at all" "$PC $d"

d=$(new_case); mkdir -p "$d/rtl"; cp "$REPO/rtl/littlecpu.v" "$d/rtl/"
probe "a tree git cannot list is a scan of nothing, not a green one" 1 \
  "cannot enumerate any tracked Verilog" "$PC $d"

begin_group "formal/traps-region-probe.py"

# That file is itself the demonstrated red direction for formal/traps.sv's two
# load/store region arms -- the trap and its cause -- and it needs a solver to be
# one, so it runs under `make -C formal components_traps` rather than here. What
# is probed here is its own grading: it builds two mutated cores and requires
# each to go red at one named line, and each of those comparisons has a failure
# path of its own.
#
# The stub reads the tree it is handed and answers the way sby does, so the
# control below is the real script over the real RTL with only the solver
# replaced. A stub that answered from its arguments alone would make every probe
# here a test of the stub.
TR="python3 $REPO/formal/traps-region-probe.py"

cat > "$tmp/sby-stub" <<'STUB'
#!/bin/sh
# Stands in for sby. The case being run is the name of the directory it is run
# in, and the assertion line is read out of the copy of traps.sv it was handed,
# so PASS and FAIL land where the real solver puts them.
mkdir -p probe
case $(basename "$PWD") in
  no-trap)
    line=$(grep -n 'assert(trap_entry);' src/traps.sv | cut -d: -f1)
    status=${STUB_SBY_NOTRAP:-FAIL}; line=${STUB_SBY_NOTRAP_LINE:-$line} ;;
  wrong-cause)
    line=$(grep -n 'assert(csr_rdata == prev_cause);' src/traps.sv | cut -d: -f1)
    status=${STUB_SBY_WRONG:-FAIL}; line=${STUB_SBY_WRONG_LINE:-$line} ;;
esac
: > probe/logfile.txt
if [ "$status" = FAIL ]; then
  echo "SBY [probe] engine_0.basecase: Assert failed in traps: traps.sv:$line.5-$line.36" \
    > probe/logfile.txt
fi
[ -n "${STUB_SBY_NO_STATUS:-}" ] && exit 1
if [ -n "${STUB_SBY_EMPTY_STATUS:-}" ]; then : > probe/status; exit 1; fi
echo "$status 2 0" > probe/status
STUB
chmod +x "$tmp/sby-stub"

tr_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/formal"
  cp "$REPO"/rtl/structs.v "$REPO"/rtl/fetcher.v "$REPO"/rtl/decoder.v \
     "$REPO"/rtl/regsel.v "$REPO"/rtl/csrs.v "$d/rtl/"
  cp "$REPO"/formal/traps.sv "$REPO"/formal/components.sby "$d/formal/"
  printf '%s' "$d"
}

trs() { printf "%s --repo %s --workdir %s/work --sby %s" "$TR" "$1" "$1" "$tmp/sby-stub"; }

d=$(tr_fixture)
probe "control: both arms fail, each at its own line" 0 \
  "Both load/store region arms fail for their own reason" "$(trs "$d")"

# THE TWO THAT MATTER: an arm that cannot fail is the whole reason this file
# exists, and there are two of them now that the model requires the trap as well
# as the cause.
d=$(tr_fixture)
probe "an arm that admits a fault the core never commits is red" 1 \
  "the no-trap core proves" "STUB_SBY_NOTRAP=PASS $(trs "$d")"

d=$(tr_fixture)
probe "an arm that admits the wrong cause is red" 1 \
  "the wrong-cause core proves" "STUB_SBY_WRONG=PASS $(trs "$d")"

d=$(tr_fixture)
probe "a must-trap proof going red somewhere else is not evidence" 1 \
  "which does not include line" "STUB_SBY_NOTRAP_LINE=9 $(trs "$d")"

d=$(tr_fixture)
probe "a cause proof going red somewhere else is not evidence either" 1 \
  "which does not include line" "STUB_SBY_WRONG_LINE=9 $(trs "$d")"

d=$(tr_fixture)
probe "a solver that wrote no verdict is exit 2, not a red arm" 2 \
  "wrote no status for the" "STUB_SBY_NO_STATUS=1 $(trs "$d")"

d=$(tr_fixture)
probe "an empty status file is refused rather than read as a verdict" 2 \
  "status file for the no-trap core is empty" "STUB_SBY_EMPTY_STATUS=1 $(trs "$d")"

# The four parses. Each one is what the probe pins its answer to, so a
# respelling has to stop the run rather than quietly probe nothing.
d=$(tr_fixture); sed -i.bak 's/assert(csr_rdata == prev_cause);/assert(csr_rdata == prev_cause2);/' "$d/formal/traps.sv"
probe "a respelled cause comparison stops rather than pinning nothing" 2 \
  "prev_cause);\` 0 times" "$(trs "$d")"

d=$(tr_fixture); sed -i.bak 's/assert(trap_entry);/assert(trap_entry != 1'"'"'b0);/' "$d/formal/traps.sv"
probe "a respelled must-trap assertion stops rather than pinning nothing" 2 \
  "assert(trap_entry);\` 0 times" "$(trs "$d")"

d=$(tr_fixture); sed -i.bak 's/assign load_access_fault  = (atomic_fault/assign load_access_fault = (atomic_fault/' "$d/rtl/decoder.v"
probe "a respelled fault site stops rather than building the shipping core twice" 2 \
  "no longer spells what the wrong-cause mutation replaces" "$(trs "$d")"

d=$(tr_fixture); sed -i.bak 's/assign ls_fault = ls_access \&\& ls_answer_valid/assign ls_fault = ls_access\&\& ls_answer_valid/' "$d/rtl/decoder.v"
probe "a respelled ls_fault stops: a core that still faults proves nothing" 2 \
  "no longer spells what the no-trap mutation replaces" "$(trs "$d")"

d=$(tr_fixture); rm "$d/formal/traps.sv"
probe "the model moving away takes the probe with it, loudly" 2 \
  "formal/traps.sv is missing from" "$(trs "$d")"

# The script is READ from formal/components.sby rather than copied into these
# probes, so the two ways that read can come up empty are their own red
# directions. Both are silent otherwise: a probe built against a script it
# invented proves a design the shipping task does not build, and says so with a
# green control.
d=$(tr_fixture); rm "$d/formal/components.sby"
probe "no components.sby is exit 2, not a probe against an invented script" 2 \
  "components.sby is missing" "$(trs "$d")"

d=$(tr_fixture); sed -i.bak 's/^traps:$/trapsx:/' "$d/formal/components.sby"
probe "a renamed task stops rather than probing some other design" 2 \
  "block under [script]" "$(trs "$d")"

begin_group "formal/decoder-zkt-probe.py"

# That file is itself the demonstrated red direction for rtl/decoder.v's two
# Zkt-isolation assertions -- region_stall implies ls_access, and ls_access is
# exactly the eight base load/store encodings -- and it needs a solver to be
# one, so it runs under `make -C formal components_decoder` rather than here.
# What is probed here is its own grading: it builds two mutated cores and
# requires each to go red at one named line, and each of those comparisons has
# a failure path of its own.
#
# The stub reads the tree it is handed and answers the way sby does, so the
# control below is the real script over the real RTL with only the solver
# replaced. A stub that answered from its arguments alone would make every
# probe here a test of the stub.
DZ="python3 $REPO/formal/decoder-zkt-probe.py"

cat > "$tmp/sby-dz-stub" <<'STUB'
#!/bin/sh
# Stands in for sby. The case being run is the name of the directory it is run
# in, and the assertion line is read out of the copy of decoder.v it was
# handed, so PASS and FAIL land where the real solver puts them.
mkdir -p probe
case $(basename "$PWD") in
  region-stall-ungated)
    line=$(grep -n 'assert(!region_stall || ls_access);' src/decoder.v | cut -d: -f1)
    status=${STUB_SBY_REGION:-FAIL}; line=${STUB_SBY_REGION_LINE:-$line} ;;
  ls-access-extra)
    line=$(grep -n 'assert(ls_access == (instr_lb ||' src/decoder.v | cut -d: -f1)
    status=${STUB_SBY_ENCODING:-FAIL}; line=${STUB_SBY_ENCODING_LINE:-$line} ;;
esac
: > probe/logfile.txt
if [ "$status" = FAIL ]; then
  echo "SBY [probe] engine_0.basecase: Assert failed in decoder: decoder.v:$line.5-$line.36" \
    > probe/logfile.txt
fi
[ -n "${STUB_SBY_NO_STATUS:-}" ] && exit 1
if [ -n "${STUB_SBY_EMPTY_STATUS:-}" ]; then : > probe/status; exit 1; fi
echo "$status 2 0" > probe/status
STUB
chmod +x "$tmp/sby-dz-stub"

dz_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/formal"
  cp "$REPO"/rtl/structs.v "$REPO"/rtl/decoder.v "$REPO"/rtl/regsel.v "$d/rtl/"
  cp "$REPO"/formal/components.sby "$d/formal/"
  printf '%s' "$d"
}

dzs() { printf "%s --repo %s --workdir %s/work --sby %s" "$DZ" "$1" "$1" "$tmp/sby-dz-stub"; }

d=$(dz_fixture)
probe "control: both Zkt-isolation assertions fail, each at its own line" 0 \
  "Both Zkt-isolation assertions fail for their own reason" "$(dzs "$d")"

# THE TWO THAT MATTER: an assertion that cannot fail is the whole reason this
# file exists.
d=$(dz_fixture)
probe "an ungated region_stall proving is red" 1 \
  "the region-stall-ungated core proves" "STUB_SBY_REGION=PASS $(dzs "$d")"

d=$(dz_fixture)
probe "an ls_access with an extra encoding proving is red" 1 \
  "the ls-access-extra core proves" "STUB_SBY_ENCODING=PASS $(dzs "$d")"

d=$(dz_fixture)
probe "the gate proof going red somewhere else is not evidence" 1 \
  "which does not include line" "STUB_SBY_REGION_LINE=9 $(dzs "$d")"

d=$(dz_fixture)
probe "the encoding proof going red somewhere else is not evidence either" 1 \
  "which does not include line" "STUB_SBY_ENCODING_LINE=9 $(dzs "$d")"

d=$(dz_fixture)
probe "a solver that wrote no verdict is exit 2, not a red arm" 2 \
  "wrote no status for the" "STUB_SBY_NO_STATUS=1 $(dzs "$d")"

d=$(dz_fixture)
probe "an empty status file is refused rather than read as a verdict" 2 \
  "status file for the region-stall-ungated core is empty" \
  "STUB_SBY_EMPTY_STATUS=1 $(dzs "$d")"

# The two parses. Each one is what the probe pins its answer to, so a
# respelling has to stop the run rather than quietly probe nothing.
d=$(dz_fixture)
sed -i.bak 's/assert(!region_stall || ls_access);/assert(!region_stall || ls_access == 1);/' \
  "$d/rtl/decoder.v"
probe "a respelled gate assertion stops rather than pinning nothing" 2 \
  "states \`assert(!region_stall || ls_access);\` 0 times" "$(dzs "$d")"

d=$(dz_fixture)
sed -i.bak "s/assign region_stall = ls_access && !ls_settled && !ls_answer_valid;/assign region_stall = ls_access \&\& !ls_settled\&\& !ls_answer_valid;/" \
  "$d/rtl/decoder.v"
probe "a respelled region_stall site stops rather than building the shipping core twice" 2 \
  "no longer spells what the region-stall-ungated mutation replaces" "$(dzs "$d")"

d=$(dz_fixture)
sed -i.bak "s/assign ls_access = instr_ls_load || instr_ls_store;/assign ls_access = instr_ls_load||instr_ls_store;/" \
  "$d/rtl/decoder.v"
probe "a respelled ls_access site stops rather than building the shipping core twice" 2 \
  "no longer spells what the ls-access-extra mutation replaces" "$(dzs "$d")"

d=$(dz_fixture); rm "$d/rtl/decoder.v"
probe "the RTL moving away takes the probe with it, loudly" 2 \
  "rtl/decoder.v is missing from" "$(dzs "$d")"

# The script is READ from formal/components.sby rather than copied into these
# probes, the same reasoning traps-region-probe.py's own two probes below
# apply: a probe built against an invented script proves a design the shipping
# task does not build, and says so with a green control.
d=$(dz_fixture); rm "$d/formal/components.sby"
probe "no components.sby is exit 2, not a probe against an invented script" 2 \
  "formal/components.sby is missing" "$(dzs "$d")"

d=$(dz_fixture); sed -i.bak 's/^decoder:$/decoderx:/' "$d/formal/components.sby"
probe "a renamed task stops rather than probing some other design" 2 \
  "block under [script]" "$(dzs "$d")"

begin_group "formal/traps-tval-probe.py"

# The same demand on formal/traps.sv's mtval arm, and the same division of
# labour: the script itself needs a solver, so it runs under
# `make -C formal components_traps`, and what is probed HERE is its own grading.
# Nothing else in the tree reads mtval -- no generated check names it, and both
# sim legs see only what a program loaded it into -- so that arm failing when it
# should is the whole oracle.
TT="python3 $REPO/formal/traps-tval-probe.py"

cat > "$tmp/sby-tval-stub" <<'STUB'
#!/bin/sh
# Stands in for sby, the way the region probe's stub does: the case is the name
# of the directory it runs in, and the assertion line is read out of the copy of
# traps.sv it was handed, so PASS and FAIL land where the real solver puts them.
mkdir -p probe
line=$(grep -n 'assert(csr_rdata == prev_tval);' src/traps.sv | cut -d: -f1)
case $(basename "$PWD") in
  control)    status=${STUB_TVAL_CONTROL:-PASS} ;;
  wrong-addr) status=${STUB_TVAL_ADDR:-FAIL}; line=${STUB_TVAL_ADDR_LINE:-$line} ;;
  wrong-word) status=${STUB_TVAL_WORD:-FAIL}; line=${STUB_TVAL_WORD_LINE:-$line} ;;
esac
: > probe/logfile.txt
if [ "$status" = FAIL ]; then
  echo "SBY [probe] engine_0.basecase: Assert failed in traps: traps.sv:$line.5-$line.36" \
    > probe/logfile.txt
fi
[ -n "${STUB_TVAL_NO_STATUS:-}" ] && exit 1
if [ -n "${STUB_TVAL_EMPTY_STATUS:-}" ]; then : > probe/status; exit 1; fi
echo "$status 2 0" > probe/status
STUB
chmod +x "$tmp/sby-tval-stub"

tts() { printf "%s --repo %s --workdir %s/work --sby %s" "$TT" "$1" "$1" "$tmp/sby-tval-stub"; }

d=$(tr_fixture)
probe "control: the arm proves for the shipping core and fails on both values" 0 \
  "proves for the shipping core and fails on the value" "$(tts "$d")"

d=$(tr_fixture)
probe "a model that cannot prove the shipping core makes both reds meaningless" 1 \
  "the shipping core does not prove" "STUB_TVAL_CONTROL=FAIL $(tts "$d")"

# THE TWO THAT MATTER: an arm that admits the wrong value is the whole reason
# this file exists, once per value source.
d=$(tr_fixture)
probe "an arm that admits rs1 where the effective address belongs is red" 1 \
  "the wrong-addr core proves" "STUB_TVAL_ADDR=PASS $(tts "$d")"

d=$(tr_fixture)
probe "an arm that admits a zero where the faulting word belongs is red" 1 \
  "the wrong-word core proves" "STUB_TVAL_WORD=PASS $(tts "$d")"

d=$(tr_fixture)
probe "a proof going red somewhere else is not evidence about the mtval arm" 1 \
  "not at line" "STUB_TVAL_ADDR_LINE=9 $(tts "$d")"

d=$(tr_fixture)
probe "...and the same for the second mutation, which shares that line" 1 \
  "not at line" "STUB_TVAL_WORD_LINE=9 $(tts "$d")"

d=$(tr_fixture)
probe "a solver that wrote no verdict is exit 2, not a red arm" 2 \
  "wrote no status for the" "STUB_TVAL_NO_STATUS=1 $(tts "$d")"

d=$(tr_fixture)
probe "an empty status file is refused rather than read as a verdict" 2 \
  "status file for the control core is empty" "STUB_TVAL_EMPTY_STATUS=1 $(tts "$d")"

# The three parses. Each is what the probe pins its answer to, so a respelling
# has to stop the run rather than quietly probe nothing -- and the control case
# checks BOTH mutation sites, which is what stops a half-respelled mux from
# building the shipping core three times.
d=$(tr_fixture); sed -i.bak 's/assert(csr_rdata == prev_tval);/assert(csr_rdata == prev_tval2);/' "$d/formal/traps.sv"
probe "a respelled mtval comparison stops rather than pinning nothing" 2 \
  "0 times" "$(tts "$d")"

d=$(tr_fixture); sed -i.bak "s/      data_fault:        trap_tval = mem_addr_calc;/      data_fault: trap_tval = mem_addr_calc;/" "$d/rtl/decoder.v"
probe "a respelled address arm stops rather than proving the shipping core" 2 \
  "no longer spells its mtval mux" "$(tts "$d")"

d=$(tr_fixture); sed -i.bak "s/      instr_illegal:     trap_tval = instr;/      instr_illegal: trap_tval = instr;/" "$d/rtl/decoder.v"
probe "a respelled word arm stops the same way" 2 \
  "no longer spells its mtval mux" "$(tts "$d")"

d=$(tr_fixture); rm "$d/formal/traps.sv"
probe "the model moving away takes this probe with it too" 2 \
  "formal/traps.sv is missing from" "$(tts "$d")"

# Both ways the shared read of components.sby's `traps` block can come up empty.
# This probe has a control that must PASS, so a script it invented would report
# a green one about a design the shipping task does not build.
d=$(tr_fixture); rm "$d/formal/components.sby"
probe "no components.sby is exit 2 here too, not a green control" 2 \
  "components.sby is missing" "$(tts "$d")"

d=$(tr_fixture); sed -i.bak 's/^traps:$/trapsx:/' "$d/formal/components.sby"
probe "a renamed task stops this probe rather than moving its control" 2 \
  "block under [script]" "$(tts "$d")"



begin_group "soc/netlist_digest.py"

ND="python3 $REPO/soc/netlist_digest.py"

# A yosys-shaped netlist small enough to read: one blackbox module, one top with
# two cells, one port and one named net. Written by hand rather than captured,
# because a captured one is 7 MB and every edit below has to be visible.
nd_netlist() {  # <file>
  cat > "$1" <<'JSON'
{
  "creator": "Yosys 0.68 (git sha1 abcdef0)",
  "modules": {
    "SB_LUT4": {
      "attributes": { "blackbox": "00000000000000000000000000000001" },
      "ports": { "O": { "direction": "output", "bits": [2] } },
      "cells": {},
      "netnames": {}
    },
    "littlesoc": {
      "attributes": {
        "top": "00000000000000000000000000000001",
        "src": "rtl/littlesoc.v:4.1-99.10"
      },
      "ports": { "clk": { "direction": "input", "bits": [2] } },
      "cells": {
        "lut.1": {
          "hide_name": 1, "type": "SB_LUT4",
          "parameters": { "LUT_INIT": "1010101010101010" },
          "attributes": { "src": "rtl/decoder.v:120.3-120.9", "hdlname": "decode" },
          "connections": { "I0": [2], "O": [3] }
        },
        "dff.2": {
          "hide_name": 1, "type": "SB_DFF", "parameters": {},
          "attributes": {
            "src": "rtl/decoder.v:121.3-121.9",
            "module_src": "rtl/decoder.v:4.1-1205.10"
          },
          "connections": { "C": [2], "D": [3], "Q": [4] }
        }
      },
      "netnames": {
        "clk": {
          "hide_name": 0, "bits": [2],
          "attributes": { "src": "rtl/littlesoc.v:5.1-5.9" }
        }
      }
    }
  }
}
JSON
}

nd_pair() {  # a fixture directory holding base.json and new.json, identical
  local d; d=$(new_case)
  nd_netlist "$d/base.json"
  nd_netlist "$d/new.json"
  printf '%s' "$d"
}

d=$(nd_pair)
probe "control: a netlist digests, and the digest is a sha256" 0 \
  "digest    sha256:" "$ND digest $d/base.json"

d=$(nd_pair)
probe "control: the digest states what its two verdicts mean" 0 \
  "sound in one" "$ND digest $d/base.json"

d=$(nd_pair)
probe "control: two identical netlists are equal" 0 "DIGEST-EQUAL" \
  "$ND compare $d/base.json $d/new.json"

# The comment and whitespace classes, which is every `src` and `module_src` in
# the file moving and nothing else. This is the whole reason a bare hash of the
# netlist was not enough.
d=$(nd_pair); sed -i.bak 's/\.v:\([0-9]*\)\./.v:9\1./g' "$d/new.json"
probe "every source line moving is the comment class, and is forgiven" 0 \
  "DIGEST-EQUAL" "$ND compare $d/base.json $d/new.json"

# ...and only those two. An attribute that is not a source line is a difference,
# because dropping one is forgiving one, and the placer is not obliged to agree.
d=$(nd_pair); sed -i.bak 's/"hdlname": "decode"/"hdlname": "decoder"/' "$d/new.json"
probe "an attribute that is not a source line is not forgiven" 1 \
  "DIGEST-DIFFERENT" "$ND compare $d/base.json $d/new.json"

# The dead tie-off class is `opt_clean -purge`'s to remove, upstream of this
# script. What is pinned here is that this script does NOT forgive a dead net
# left in the file: a netlist that still carries one is a netlist that differs.
d=$(nd_pair)
python3 - "$d/new.json" <<'PY'
import json, sys
design = json.load(open(sys.argv[1]))
design["modules"]["littlesoc"]["netnames"]["dead_tieoff"] = {
    "hide_name": 0, "bits": [5],
    "attributes": {"unused_bits": "0 1", "src": "rtl/decoder.v:98.3-98.9"}}
json.dump(design, open(sys.argv[1], "w"))
PY
probe "a dead net still in the file is a difference, not a forgiveness" 1 \
  "named nets: 1 -> 2" "$ND compare $d/base.json $d/new.json"

# A one-bit constant change: no module, cell count or port moves, so the
# structural summary has nothing to say and the report has to name the path.
d=$(nd_pair); sed -i.bak 's/1010101010101010/1010101010101011/' "$d/new.json"
probe "a one-bit constant is a different digest" 1 "DIGEST-DIFFERENT" \
  "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); sed -i.bak 's/1010101010101010/1010101010101011/' "$d/new.json"
probe "...and the report names the parameter, not merely 'changed'" 1 \
  "parameters.LUT_INIT: 1010101010101010 -> 1010101010101011" \
  "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); sed -i.bak 's/"dff.2"/"dff.3"/' "$d/new.json"
probe "a renamed cell is named by path" 1 "cells.dff.2: in base only" \
  "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); sed -i.bak 's/"type": "SB_DFF"/"type": "SB_LUT4"/' "$d/new.json"
probe "a cell type that moved is reported as a count, both ways" 1 \
  "SB_LUT4                       1 ->      2  (+1)" \
  "$ND compare $d/base.json $d/new.json"

d=$(nd_pair)
sed -i.bak 's/"clk": { "direction": "input", "bits": \[2\] }/"clk": { "direction": "input", "bits": [2] }, "hart_id": { "direction": "input", "bits": [7] }/' "$d/new.json"
probe "a port that appeared is named, which is what a tie-off adds" 1 \
  "port hart_id: (none) -> input [1]" "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); sed -i.bak 's/"SB_LUT4": {/"SB_MAC16": { "attributes": {}, "ports": {}, "cells": {}, "netnames": {} },\n    "SB_LUT4": {/' "$d/new.json"
probe "a module that appeared is named too" 1 "modules in this tree only: SB_MAC16" \
  "$ND compare $d/base.json $d/new.json"

# The toolchain is inside the digest, so a yosys that moved reads as different.
# That is the sound direction and the one this repo has been bitten in.
d=$(nd_pair); sed -i.bak 's/Yosys 0.68 (git sha1 abcdef0)/Yosys 0.55 (git sha1 abcdef0)/' "$d/new.json"
probe "a toolchain that moved is a different digest, and is named first" 1 \
  "toolchain: Yosys 0.68" "$ND compare $d/base.json $d/new.json"

# The four refusals. None of them may read as equal: the whole value of this
# gate is that its equal verdict is the one that skips twelve minutes of work.
d=$(nd_pair)
probe "a netlist that is not there is refused, not read as equal" 2 \
  "so there is nothing to digest" "$ND compare $d/gone.json $d/new.json"

d=$(nd_pair); : > "$d/new.json"
probe "an empty netlist is a synthesis that wrote nothing" 2 \
  "empty, which is a synthesis that wrote nothing" \
  "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); head -c 400 "$d/base.json" > "$d/new.json"
probe "a truncated netlist is refused, which is what a full disk leaves" 2 \
  "not parseable as JSON" "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); printf '{"creator": "yosys"}' > "$d/new.json"
probe "JSON that is not a netlist is refused" 2 \
  "no \`modules\` object in it" "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); sed -i.bak 's/"top": "00000000000000000000000000000001",//' "$d/new.json"
probe "a netlist with no top module names no one design" 2 \
  "0 modules are marked \`top\`" "$ND compare $d/base.json $d/new.json"

d=$(nd_pair); python3 - "$d/new.json" <<'PY'
import json, sys
design = json.load(open(sys.argv[1]))
design["modules"]["littlesoc"]["cells"] = {}
json.dump(design, open(sys.argv[1], "w"))
PY
probe "a top module with no cells in it is a failed synthesis" 2 \
  "has no cells in it" "$ND compare $d/base.json $d/new.json"

begin_group "soc/netlist_determinism.sh"

# The control that decides whether the digest means anything, run against stub
# tools: three placements of the real flow are three minutes and need yosys and
# nextpnr, and what is graded here is the four comparisons rather than the
# placer. The stubs are keyed on the paths the script itself chooses, so a
# renamed output would stop them standing in.
nl_stub_yosys() {  # $1 = bin dir, $2 = fixture dir
  cat > "$1/yosys" <<STUB
#!/bin/sh
# Writes the fixture netlist the -p script names as its output. Which fixture
# depends on the tree it is run in and on whether the script asks for the purge:
# the shipping forms are only ever compared byte for byte, the canonical ones
# are read by the real soc/netlist_digest.py.
script=; prev=
for a in "\$@"; do
  if [ "\$prev" = "-p" ]; then script=\$a; fi
  prev=\$a
done
out=\$(printf '%s' "\$script" | tr ' ' '\n' | tail -1)
case \$(pwd) in
  */mutant) tree=mutant ;;
  *)        tree=this ;;
esac
case \$script in
  *"opt_clean -purge"*)
    if [ "\$tree" = mutant ] && [ -n "\${STUB_YOSYS_CANON_MOVED:-}" ]; then
      cp "$2/canon.moved.json" "\$out"
    elif [ "\$tree" = mutant ] && [ -n "\${STUB_YOSYS_DEAD_SURVIVES:-}" ]; then
      cp "$2/canon.dead.json" "\$out"
    else
      cp "$2/canon.json" "\$out"
    fi ;;
  *)
    if [ "\$tree" != mutant ] || [ -n "\${STUB_YOSYS_NO_TRACE:-}" ]; then
      printf 'shipping netlist\n' > "\$out"
    elif [ -n "\${STUB_YOSYS_NO_DEAD:-}" ]; then
      printf 'shipping netlist, mutated\n' > "\$out"
    else
      printf 'shipping netlist, mutated, netlist_control_dead\n' > "\$out"
    fi ;;
esac
exit \${STUB_YOSYS_EXIT:-0}
STUB
  chmod +x "$1/yosys"
}

nl_stub_pnr() {  # $1 = bin dir
  cat > "$1/nextpnr-ice40" <<'STUB'
#!/bin/sh
# Stands in for the placer. The bitstream it writes is keyed on the output name
# the script asked for, so a placement that is not a function of the netlist and
# a mutant that places elsewhere are each one environment variable away.
asc=; prev=
for a in "$@"; do
  if [ "$prev" = "--asc" ]; then asc=$a; fi
  prev=$a
done
if [ -n "${STUB_PNR_EMPTY:-}" ]; then exit 1; fi
bits=placement-A
case $asc in
  *mutant.asc) if [ -n "${STUB_PNR_MUTANT_MOVED:-}" ]; then bits=placement-B; fi ;;
  *this.2.asc) if [ -n "${STUB_PNR_FLAKY:-}" ]; then bits=placement-B; fi ;;
esac
printf '%s\n' "$bits" > "$asc"
# Killed part-way through the write: a non-empty bitstream, and a log that never
# reaches the line the real placer prints after finishing one.
if [ -n "${STUB_PNR_TRUNCATED:-}" ]; then exit 137; fi
echo "Info: Program finished normally."
STUB
  chmod +x "$1/nextpnr-ice40"
}

nl_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/bin" "$d/repo/rtl" "$d/repo/soc" "$d/fix"
  cp "$REPO/soc/netlist_determinism.sh" "$REPO/soc/netlist_digest.py" "$d/repo/soc/"
  # Two lines of the shape the injection reaches for: a trailing `endmodule` and
  # the signal the dead wire reads.
  cat > "$d/repo/rtl/decoder.v" <<'RTL'
module decoder (input logic [31:0] reg_rs1, output logic [31:0] out);
  assign out = reg_rs1;
endmodule
RTL
  nd_netlist "$d/fix/canon.json"
  nd_netlist "$d/fix/canon.moved.json"
  sed -i.bak 's/1010101010101010/1010101010101011/' "$d/fix/canon.moved.json"
  # A canonical form the purge did NOT clean the dead net out of.
  nd_netlist "$d/fix/canon.dead.json"
  sed -i.bak 's/"hdlname": "decode"/"hdlname": "netlist_control_dead"/' \
    "$d/fix/canon.dead.json"
  nl_stub_yosys "$d/bin" "$d/fix"
  nl_stub_pnr "$d/bin"
  printf '%s' "$d"
}

nl_run() {  # <fixture dir> [what follows the stubs on PATH]
  printf '%s' "PATH=$1/bin:${2:-\$PATH} \
    NETLIST_SYNTH='read_verilog -sv rtl/decoder.v; synth_ice40 -top littlesoc' \
    NETLIST_PNR='nextpnr-ice40 --up5k' NETLIST_PNR_OUT=--asc \
    NETLIST_MUTANT='rtl/decoder.v reg_rs1' \
    sh $1/repo/soc/netlist_determinism.sh"
}

d=$(nl_fixture)
probe "control: a deterministic placer and a forgiven mutant pass" 0 \
  "netlist-determinism: PASS" "$(nl_run "$d")"

d=$(nl_fixture)
probe "one netlist placed twice giving two bitstreams voids the gate" 1 \
  "Placement is not a function of the netlist" \
  "STUB_PNR_FLAKY=1 $(nl_run "$d")"

d=$(nl_fixture)
probe "a mutant the digest calls equal placing elsewhere voids it too" 1 \
  "placed to different bitstreams" "STUB_PNR_MUTANT_MOVED=1 $(nl_run "$d")"

# The vacuity check. Without it the control passes hardest when it is testing
# nothing, which is the shape of every defect this file exists for.
d=$(nl_fixture)
probe "a mutant that left no trace demonstrates nothing, and says so" 1 \
  "nothing was injected" "STUB_YOSYS_NO_TRACE=1 $(nl_run "$d")"

d=$(nl_fixture)
probe "a canonical form that stopped forgiving the class is red" 1 \
  "no longer forgives a comment" "STUB_YOSYS_CANON_MOVED=1 $(nl_run "$d")"

d=$(nl_fixture)
probe "a placer that wrote no bitstream placed nothing, which is not a pass" 1 \
  "wrote no bitstream" "STUB_PNR_EMPTY=1 $(nl_run "$d")"

# THE ONE A NON-EMPTY FILE HIDES: a placer killed mid-write leaves a partial
# bitstream, and a deterministic one killed three times leaves three partial
# files that compare equal. Size alone calls that a placement.
d=$(nl_fixture)
probe "a bitstream the placer never finished writing is not a placement" 1 \
  "never finished" "STUB_PNR_TRUNCATED=1 $(nl_run "$d")"

# The vacuity checks the comment class cannot stand in for. The mutant carries
# three edits and the comment alone makes its netlist differ, so "something
# moved" says nothing about the dead net -- which is the class the purge is what
# forgives.
d=$(nl_fixture)
probe "a dead tie-off that never reached the netlist exercises no class" 1 \
  "left no trace in the mapped netlist" "STUB_YOSYS_NO_DEAD=1 $(nl_run "$d")"

d=$(nl_fixture)
probe "...and one the purge did not remove means the digest is equal by luck" 1 \
  "survives" "STUB_YOSYS_DEAD_SURVIVES=1 $(nl_run "$d")"

d=$(nl_fixture)
probe "a synthesis that failed is not a passed control either" 1 \
  "could not synthesise" "STUB_YOSYS_EXIT=1 $(nl_run "$d")"

# These two name the WHOLE path, for the reason the `bin-none` note above gives:
# with the caller's PATH behind the stubs, deleting one finds the host's real
# yosys or nextpnr and the probe demonstrates nothing. Both refusals fire before
# anything is synthesised, so /usr/bin and /bin are all either one needs.
d=$(nl_fixture); rm "$d/bin/yosys"
probe "no yosys on PATH is refused rather than skipped" 2 "no yosys on PATH" \
  "$(nl_run "$d" /usr/bin:/bin)"

d=$(nl_fixture); rm "$d/bin/nextpnr-ice40"
probe "no placer on PATH is refused the same way" 2 \
  "no nextpnr-ice40 on PATH" "$(nl_run "$d" /usr/bin:/bin)"

d=$(nl_fixture); sed -i.bak 's/reg_rs1/reg_rs9/g' "$d/repo/rtl/decoder.v"
probe "an injection site that moved stops the control, loudly" 1 \
  "the mutant could not be built" "$(nl_run "$d")"

# The quiet way that site rots: the signal is still in the FILE, in a module the
# tie-off does not land in. Implicitly declared one bit wide, its part-selects
# fold to a constant and the assign is optimised away -- so the check has to be
# scoped to the module being spliced into, not to the file.
d=$(nl_fixture)
cat > "$d/repo/rtl/decoder.v" <<'RTL'
module regsel (input logic [31:0] reg_rs1, output logic [31:0] picked);
  assign picked = reg_rs1;
endmodule
module decoder (input logic [31:0] word, output logic [31:0] out);
  assign out = word;
endmodule
RTL
probe "a signal in scope only in an earlier module stops it too" 1 \
  "is not in the module the tie-off splices into" "$(nl_run "$d")"

d=$(nl_fixture); rm "$d/repo/rtl/decoder.v"
probe "...and so does the file it injects into going away" 1 \
  "is not in this tree" "$(nl_run "$d")"

d=$(nl_fixture)
probe "an empty part table synthesises nothing, so it is refused" 2 \
  "NETLIST_SYNTH is not set" \
  "PATH=$d/bin:\$PATH sh $d/repo/soc/netlist_determinism.sh"

begin_group "soc/netlist_base.sh"

# The other tree's half of `make netlist-diff`. `git archive` and a stub yosys,
# so the whole extraction runs without a placement or a cross compiler.
nb_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/bin" "$d/repo/rtl" "$d/repo/soc"
  cp "$REPO/soc/netlist_base.sh" "$d/repo/soc/"
  printf 'module decoder ();\nendmodule\n' > "$d/repo/rtl/decoder.v"
  # A Makefile with the two targets this script asks another tree for, and
  # nothing else: soc-rom, which builds the image that gets synthesised, and
  # print-%, which is how the base tree is asked to name its own synth script.
  cat > "$d/repo/Makefile" <<'MK'
soc-rom:
	@:
print-%:
	@echo '$($*)'
MK
  nl_stub_yosys "$d/bin" "$d"
  nd_netlist "$d/canon.json"
  git -c init.defaultBranch=main -C "$d/repo" init -q
  git -C "$d/repo" add -A
  git -C "$d/repo" -c user.email=probe@example -c user.name=probe commit -qm base
  printf '%s' "$d"
}

nb_run() {  # <fixture dir> <ref>
  printf '%s' "cd $1/repo && PATH=$1/bin:\$PATH \
    NETLIST_SYNTH='read_verilog -sv rtl/decoder.v; synth_ice40 -top littlesoc' \
    sh soc/netlist_base.sh $2 $1/base.canon.json"
}

d=$(nb_fixture)
probe "control: another commit's canonical netlist is built from its own tree" 0 \
  "canonical netlist is" "$(nb_run "$d" HEAD)"

d=$(nb_fixture)
probe "a tree that names no synth script of its own says whose was used" 0 \
  "names no synth script of its own" "$(nb_run "$d" HEAD)"

d=$(nb_fixture)
sed -i.bak 's/^print-%:/NETLIST_SYNTH := read_verilog -sv rtl\/decoder.v; synth_ice40 -abc9 -top littlesoc\nprint-%:/' "$d/repo/Makefile"
git -C "$d/repo" -c user.email=probe@example -c user.name=probe commit -qam flags
probe "a base tree whose synth flags moved is digested with ITS flags, and says so" 0 \
  "synthesises with a different script" "$(nb_run "$d" HEAD)"

# A base tree whose make fails for any reason OTHER than having no `print-%`
# rule must not read as one that names no synth script: that fallback
# synthesises the base commit with THIS tree's flags, which is the blind
# comparison this script exists to refuse. A parse error fails the ROM step
# first and is reported there, so what is forced here is the rule itself
# failing -- which is what `| tail -1` used to swallow whole.
d=$(nb_fixture)
cat > "$d/repo/Makefile" <<'MK'
soc-rom:
	@:
print-%:
	@echo '$($*)'; exit 3
MK
git -C "$d/repo" -c user.email=probe@example -c user.name=probe commit -qam broken
probe "a base tree whose make fails is refused, not quietly given ours" 2 \
  "could not be asked which synth" "$(nb_run "$d" HEAD)"

d=$(nb_fixture)
probe "a ref that names no commit is refused, not compared" 2 \
  "does not name a commit" "$(nb_run "$d" v9.9.9)"

d=$(nb_fixture)
git -C "$d/repo" rm -q rtl/decoder.v
git -C "$d/repo" -c user.email=probe@example -c user.name=probe commit -qm drop
probe "a source this tree synthesises that the base lacks is not comparable" 2 \
  "has no rtl/decoder.v" "$(nb_run "$d" HEAD)"
begin_group "formal/busarbiter-probe.py"

# Same shape as the group above, and for the same reason: that file is itself
# the demonstrated red direction for formal/busarbiter.sv's wait bound and its
# indivisibility arm, it needs a solver to be one, and so it runs under `make -C
# formal components_busarbiter` rather than here. What is probed here is its own
# grading -- it builds three arbiters and requires one to prove, one to go red
# at the wait bound and not at the lock, and one to go red at the lock and take
# the anti-vacuity cover down with it. Every one of those comparisons has a
# failure path of its own.
BA="python3 $REPO/formal/busarbiter-probe.py"

cat > "$tmp/sby-busarbiter-stub" <<'STUB'
#!/bin/sh
# Stands in for sby. The case is the name of the directory it runs in and the
# job is the .sby it was handed, and every line number is read out of the copy
# of busarbiter.sv beside it -- so a respelled assertion moves this stub's
# answer exactly the way it moves the real solver's.
for a in "$@"; do last=$a; done
job=${last%.sby}
line_of() { grep -nF -- "$1" src/busarbiter.sv | cut -d: -f1; }
lock=$(line_of 'if (settled && past_grant[h] && past_mem_lock[h]) assert(grant[h]);')
bound=$(line_of 'always_comb if (clocked) assert(waited <= BOUND);')
cover=$(line_of 'cover (settled && grant[h] && past_grant[h] && past_mem_lock[h] &&')
mkdir -p "$job"
: > "$job/logfile.txt"
assert_red() {
  echo "SBY [probe] engine_0.basecase: Assert failed in busarbiter_check:" \
       "busarbiter.sv:$1.9-$1.26" >> "$job/logfile.txt"
}
cover_red() {
  echo "SBY [probe] engine_0: Unreached cover statement at busarbiter_check:" \
       "busarbiter.sv:$1.7-$1.30" >> "$job/logfile.txt"
}
status=PASS
case "$(basename "$PWD")/$job" in
  shipping/prove) status=${STUB_SHIP_PROVE:-PASS} ;;
  shipping/cover) status=${STUB_SHIP_COVER:-PASS} ;;
  fixed-priority/prove)
    status=${STUB_FIXED:-FAIL}
    if [ "$status" = FAIL ]; then
      assert_red "${STUB_FIXED_LINE:-$bound}"
      [ -n "${STUB_FIXED_ALSO_LOCK:-}" ] && assert_red "$lock"
    fi ;;
  grant-mid-lock/prove)
    status=${STUB_MIDLOCK:-FAIL}
    [ "$status" = FAIL ] && assert_red "${STUB_MIDLOCK_LINE:-$lock}" ;;
  grant-mid-lock/cover)
    status=${STUB_COVER_MID:-FAIL}
    [ "$status" = FAIL ] && cover_red "${STUB_COVER_MID_LINE:-$cover}" ;;
esac
[ -n "${STUB_SBY_NO_STATUS:-}" ] && exit 1
if [ -n "${STUB_SBY_EMPTY_STATUS:-}" ]; then : > "$job/status"; exit 1; fi
echo "$status 2 0" > "$job/status"
STUB
chmod +x "$tmp/sby-busarbiter-stub"

ba_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/formal"
  cp "$REPO"/rtl/busarbiter.v "$d/rtl/"
  cp "$REPO"/formal/busarbiter.sv "$d/formal/"
  printf '%s' "$d"
}

bas() {
  printf "%s --repo %s --workdir %s/work --sby %s" \
    "$BA" "$1" "$1" "$tmp/sby-busarbiter-stub"
}

d=$(ba_fixture)
probe "control: both arms admit the shipping arbiter and fail on their mutation" 0 \
  "fail on their own mutation" "$(bas "$d")"

d=$(ba_fixture)
probe "a harness that is red at rest makes the two mutations meaningless" 1 \
  "the shipping arbiter does not prove" "STUB_SHIP_PROVE=FAIL $(bas "$d")"

d=$(ba_fixture)
probe "a cover the shipping arbiter cannot reach is red before any mutation" 1 \
  "does not reach its own cover goals" "STUB_SHIP_COVER=FAIL $(bas "$d")"

# THE TWO THAT MATTER: a bound that admits starvation and a lock that admits a
# torn atomic are the whole reason that file exists.
d=$(ba_fixture)
probe "a wait bound that admits a starved hart is red" 1 \
  "the fixed-priority arbiter proves" "STUB_FIXED=PASS $(bas "$d")"

d=$(ba_fixture)
probe "a lock arm that admits a grant mid-AMO is red" 1 \
  "the mid-lock arbiter proves" "STUB_MIDLOCK=PASS $(bas "$d")"

d=$(ba_fixture)
probe "starvation going red somewhere other than the bound is not evidence" 1 \
  "which does not include" "STUB_FIXED_LINE=9 $(bas "$d")"

d=$(ba_fixture)
probe "two arms that cannot be told apart are not two arms" 1 \
  "as well" "STUB_FIXED_ALSO_LOCK=1 $(bas "$d")"

d=$(ba_fixture)
probe "a torn atomic going red at the wait bound is not evidence either" 1 \
  "not at line" "STUB_MIDLOCK_LINE=9 $(bas "$d")"

d=$(ba_fixture)
probe "an anti-vacuity cover that cannot go red is not a control" 1 \
  "reaches every cover goal" "STUB_COVER_MID=PASS $(bas "$d")"

d=$(ba_fixture)
probe "a cover red for some other goal says nothing about the lock" 1 \
  "cover went red without line" "STUB_COVER_MID_LINE=9 $(bas "$d")"

d=$(ba_fixture)
probe "a solver that wrote no verdict is exit 2, not a red arm" 2 \
  "wrote no status for the" "STUB_SBY_NO_STATUS=1 $(bas "$d")"

d=$(ba_fixture)
probe "an empty status file is refused rather than read as a verdict" 2 \
  "is empty" "STUB_SBY_EMPTY_STATUS=1 $(bas "$d")"

# The three parses, one per pinned line. Each is what a probe pins its answer
# to, so a respelling has to stop the run rather than quietly probe nothing.
d=$(ba_fixture)
sed -i.bak 's/past_mem_lock\[h\]) assert(grant\[h\]);/past_mem_lock[h]) assert(grant[h] == 1);/' \
  "$d/formal/busarbiter.sv"
probe "a respelled indivisibility arm stops rather than pinning nothing" 2 \
  "states the indivisibility assertion 0 times" "$(bas "$d")"

d=$(ba_fixture)
sed -i.bak 's/assert(waited <= BOUND);/assert(waited <= BOUND + 0);/' "$d/formal/busarbiter.sv"
probe "a respelled wait bound stops rather than pinning nothing" 2 \
  "states the wait bound assertion 0 times" "$(bas "$d")"

d=$(ba_fixture)
sed -i.bak 's/cover (settled \&\& grant\[h\] \&\& past_grant\[h\]/cover (grant[h] \&\& settled \&\& past_grant[h]/' \
  "$d/formal/busarbiter.sv"
probe "a respelled lock cover stops rather than pinning nothing" 2 \
  "states the lock cover goal 0 times" "$(bas "$d")"

d=$(ba_fixture)
sed -i.bak "s/else grant <= held ? grant : winner;/else grant <= (held) ? grant : winner;/" \
  "$d/rtl/busarbiter.v"
probe "a respelled mutation site stops rather than proving the shipping core thrice" 2 \
  "no longer spells its tie-break" "$(bas "$d")"

d=$(ba_fixture); rm "$d/formal/busarbiter.sv"
probe "the harness moving away takes the probe with it, loudly" 2 \
  "formal/busarbiter.sv is missing from" "$(bas "$d")"

begin_group "formal/busarbiter-probe.py"

# Same shape as the group above, and for the same reason: that file is itself
# the demonstrated red direction for formal/busarbiter.sv's wait bound and its
# indivisibility arm, it needs a solver to be one, and so it runs under `make -C
# formal components_busarbiter` rather than here. What is probed here is its own
# grading -- it builds three arbiters and requires one to prove, one to go red
# at the wait bound and not at the lock, and one to go red at the lock and take
# the anti-vacuity cover down with it. Every one of those comparisons has a
# failure path of its own.
BA="python3 $REPO/formal/busarbiter-probe.py"

cat > "$tmp/sby-busarbiter-stub" <<'STUB'
#!/bin/sh
# Stands in for sby. The case is the name of the directory it runs in and the
# job is the .sby it was handed, and every line number is read out of the copy
# of busarbiter.sv beside it -- so a respelled assertion moves this stub's
# answer exactly the way it moves the real solver's.
for a in "$@"; do last=$a; done
job=${last%.sby}
line_of() { grep -nF -- "$1" src/busarbiter.sv | cut -d: -f1; }
lock=$(line_of 'if (settled && past_grant[h] && past_mem_lock[h]) assert(grant[h]);')
bound=$(line_of 'always_comb if (clocked) assert(waited <= BOUND);')
cover=$(line_of 'cover (settled && grant[h] && past_grant[h] && past_mem_lock[h] &&')
mkdir -p "$job"
: > "$job/logfile.txt"
assert_red() {
  echo "SBY [probe] engine_0.basecase: Assert failed in busarbiter_check:" \
       "busarbiter.sv:$1.9-$1.26" >> "$job/logfile.txt"
}
cover_red() {
  echo "SBY [probe] engine_0: Unreached cover statement at busarbiter_check:" \
       "busarbiter.sv:$1.7-$1.30" >> "$job/logfile.txt"
}
status=PASS
case "$(basename "$PWD")/$job" in
  shipping/prove) status=${STUB_SHIP_PROVE:-PASS} ;;
  shipping/cover) status=${STUB_SHIP_COVER:-PASS} ;;
  fixed-priority/prove)
    status=${STUB_FIXED:-FAIL}
    if [ "$status" = FAIL ]; then
      assert_red "${STUB_FIXED_LINE:-$bound}"
      [ -n "${STUB_FIXED_ALSO_LOCK:-}" ] && assert_red "$lock"
    fi ;;
  grant-mid-lock/prove)
    status=${STUB_MIDLOCK:-FAIL}
    [ "$status" = FAIL ] && assert_red "${STUB_MIDLOCK_LINE:-$lock}" ;;
  grant-mid-lock/cover)
    status=${STUB_COVER_MID:-FAIL}
    [ "$status" = FAIL ] && cover_red "${STUB_COVER_MID_LINE:-$cover}" ;;
esac
[ -n "${STUB_SBY_NO_STATUS:-}" ] && exit 1
if [ -n "${STUB_SBY_EMPTY_STATUS:-}" ]; then : > "$job/status"; exit 1; fi
echo "$status 2 0" > "$job/status"
STUB
chmod +x "$tmp/sby-busarbiter-stub"

ba_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/rtl" "$d/formal"
  cp "$REPO"/rtl/busarbiter.v "$d/rtl/"
  cp "$REPO"/formal/busarbiter.sv "$d/formal/"
  printf '%s' "$d"
}

bas() {
  printf "%s --repo %s --workdir %s/work --sby %s" \
    "$BA" "$1" "$1" "$tmp/sby-busarbiter-stub"
}

d=$(ba_fixture)
probe "control: both arms admit the shipping arbiter and fail on their mutation" 0 \
  "fail on their own mutation" "$(bas "$d")"

d=$(ba_fixture)
probe "a harness that is red at rest makes the two mutations meaningless" 1 \
  "the shipping arbiter does not prove" "STUB_SHIP_PROVE=FAIL $(bas "$d")"

d=$(ba_fixture)
probe "a cover the shipping arbiter cannot reach is red before any mutation" 1 \
  "does not reach its own cover goals" "STUB_SHIP_COVER=FAIL $(bas "$d")"

# THE TWO THAT MATTER: a bound that admits starvation and a lock that admits a
# torn atomic are the whole reason that file exists.
d=$(ba_fixture)
probe "a wait bound that admits a starved hart is red" 1 \
  "the fixed-priority arbiter proves" "STUB_FIXED=PASS $(bas "$d")"

d=$(ba_fixture)
probe "a lock arm that admits a grant mid-AMO is red" 1 \
  "the mid-lock arbiter proves" "STUB_MIDLOCK=PASS $(bas "$d")"

d=$(ba_fixture)
probe "starvation going red somewhere other than the bound is not evidence" 1 \
  "which does not include" "STUB_FIXED_LINE=9 $(bas "$d")"

d=$(ba_fixture)
probe "two arms that cannot be told apart are not two arms" 1 \
  "as well" "STUB_FIXED_ALSO_LOCK=1 $(bas "$d")"

d=$(ba_fixture)
probe "a torn atomic going red at the wait bound is not evidence either" 1 \
  "not at line" "STUB_MIDLOCK_LINE=9 $(bas "$d")"

d=$(ba_fixture)
probe "an anti-vacuity cover that cannot go red is not a control" 1 \
  "reaches every cover goal" "STUB_COVER_MID=PASS $(bas "$d")"

d=$(ba_fixture)
probe "a cover red for some other goal says nothing about the lock" 1 \
  "cover went red without line" "STUB_COVER_MID_LINE=9 $(bas "$d")"

d=$(ba_fixture)
probe "a solver that wrote no verdict is exit 2, not a red arm" 2 \
  "wrote no status for the" "STUB_SBY_NO_STATUS=1 $(bas "$d")"

d=$(ba_fixture)
probe "an empty status file is refused rather than read as a verdict" 2 \
  "is empty" "STUB_SBY_EMPTY_STATUS=1 $(bas "$d")"

# The three parses, one per pinned line. Each is what a probe pins its answer
# to, so a respelling has to stop the run rather than quietly probe nothing.
d=$(ba_fixture)
sed -i.bak 's/past_mem_lock\[h\]) assert(grant\[h\]);/past_mem_lock[h]) assert(grant[h] == 1);/' \
  "$d/formal/busarbiter.sv"
probe "a respelled indivisibility arm stops rather than pinning nothing" 2 \
  "states the indivisibility assertion 0 times" "$(bas "$d")"

d=$(ba_fixture)
sed -i.bak 's/assert(waited <= BOUND);/assert(waited <= BOUND + 0);/' "$d/formal/busarbiter.sv"
probe "a respelled wait bound stops rather than pinning nothing" 2 \
  "states the wait bound assertion 0 times" "$(bas "$d")"

d=$(ba_fixture)
sed -i.bak 's/cover (settled \&\& grant\[h\] \&\& past_grant\[h\]/cover (grant[h] \&\& settled \&\& past_grant[h]/' \
  "$d/formal/busarbiter.sv"
probe "a respelled lock cover stops rather than pinning nothing" 2 \
  "states the lock cover goal 0 times" "$(bas "$d")"

d=$(ba_fixture)
sed -i.bak "s/else grant <= held ? grant : winner;/else grant <= (held) ? grant : winner;/" \
  "$d/rtl/busarbiter.v"
probe "a respelled mutation site stops rather than proving the shipping core thrice" 2 \
  "no longer spells its tie-break" "$(bas "$d")"

d=$(ba_fixture); rm "$d/formal/busarbiter.sv"
probe "the harness moving away takes the probe with it, loudly" 2 \
  "formal/busarbiter.sv is missing from" "$(bas "$d")"

begin_group "test/dual_build.sh"

# The two-hart programs are source with no machine to run on, so this is the
# whole of what grades them: they still assemble, and the pairings that claim
# they catch something still name them. Both halves of that are graded here, and
# the toolchain stubs are the same ones test/run_tests.sh's group uses -- these
# probes need a cross compiler exactly as little as that group does.
DB="$REPO/test/dual_build.sh"

db_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/dual" "$d/asm"
  : > "$d/asm/sections.lds"
  : > "$d/dual/amocount.S"
  : > "$d/dual/crosspatch.S"
  printf '# a comment\namo-torn  prog  amocount.S FAIL 5\nno-copy  prog  crosspatch.S FAIL 7\nstarved  formal  components_busarbiter\n' \
    > "$d/PAIRINGS"
  printf '%s' "$d"
}

db() { printf "PATH='%s/bin:%s/bin-none:/usr/bin:/bin' %s %s/dual %s/asm %s/PAIRINGS" \
  "$tmp" "$tmp" "$DB" "$1" "$1" "$1"; }

d=$(db_fixture)
probe "control: programs that build and are paired are green" 0 \
  "are paired against a" "$(db "$d")"

# The runner-graded exemption, both directions. A program the dual runner grades
# directly is exempt rather than paired; what must not happen is an exemption
# outliving the program it names, or a program carrying both -- either one hides
# the other's grader rotting.
d=$(db_fixture)
printf 'EXEMPT  smoke.S  make dual-smoke\n' >> "$d/PAIRINGS"
probe "an exemption naming no program is red, not a program quietly excused" 1 \
  "exempts programs that do not exist" "$(db "$d")"

d=$(db_fixture)
: > "$d/dual/smoke.S"
printf 'EXEMPT  smoke.S  make dual-smoke\n' >> "$d/PAIRINGS"
probe "control: an exemption whose program exists is accepted" 0 \
  "exempt to the runner" "$(db "$d")"

d=$(db_fixture)
printf 'EXEMPT  amocount.S  make dual-smoke\n' >> "$d/PAIRINGS"
probe "a program both exempt and paired is red" 1 \
  "both exempts and pairs" "$(db "$d")"

probe "wrong argument count is a usage error, not a silent pass" 1 "usage:" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $DB $d/dual $d/asm"

probe "a dual directory that is not there is the failure this exists to see" 1 \
  "is not a directory" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $DB $d/nowhere $d/asm $d/PAIRINGS"

probe "a mistyped pairings path cannot leave every program unclaimed and pass" 1 \
  "does not exist or is not readable" \
  "PATH='$tmp/bin:$tmp/bin-none:/usr/bin:/bin' $DB $d/dual $d/asm $d/NOPE"

d=$(db_fixture); rm "$d/dual/amocount.S" "$d/dual/crosspatch.S"
probe "a directory the glob stopped matching is red, not a suite of size zero" 1 \
  "contains no .S programs" "$(db "$d")"

d=$(db_fixture); rm "$d/dual/crosspatch.S"
probe "a pairing naming a program nobody wrote is red" 1 \
  "names programs that do not exist" "$(db "$d")"

d=$(db_fixture); : > "$d/dual/unclaimed.S"
probe "the other direction: a program no pairing claims to catch anything" 1 \
  "holds programs no pairing" "$(db "$d")"

# The `formal` leg has no program in its third field, and reading it as one
# would make the set check demand a file called components_busarbiter.S.
d=$(db_fixture); printf 'starved  formal  components_busarbiter\n' > "$d/PAIRINGS"
probe "a mutation with no program leg is a real entry, not a missing file" 1 \
  "holds programs no pairing" "$(db "$d")"

d=$(db_fixture)
probe "a half-installed toolchain says so once, up front" 1 \
  "no RISC-V cross compiler found" "PATH='$tmp/bin-none' $DB $d/dual $d/asm $d/PAIRINGS"

probe "objcopy is probed, not assumed to exist because gcc did" 1 \
  "but not its matching" \
  "PATH='$tmp/bin-noobjcopy:$tmp/bin-none' $DB $d/dual $d/asm $d/PAIRINGS"

probe "a program that stopped assembling is ASSEMBLE-ERROR" 1 "ASSEMBLE-ERROR" \
  "STUB_CC_EXIT=1 $(db "$d")"

probe "assembler output on a successful build is still a defect" 1 \
  "ASSEMBLE-WARNING" "STUB_CC_WARN=1 $(db "$d")"

probe "an objcopy that refuses names the region it refused for" 1 \
  "OBJCOPY-ERROR rom" "STUB_OBJCOPY_FAIL=1 $(db "$d")"

# The quiet one: exit 0 having written nothing. A runner handed that image would
# start, and every check that reads RAM would see zero.
probe "an image objcopy wrote nothing into is red, not empty and accepted" 1 \
  "OBJCOPY-EMPTY rom" "STUB_OBJCOPY_EMPTY=1 $(db "$d")"

begin_group "test/bench/run_coremark.sh"

# Every probe here exits before the toolchain search does, so none needs the
# stub compiler the groups above build. The control run instead reaches and
# fails AT that search -- proof the manifest and pin checks above it passed
# silently, the way a real `make coremark` would.
rc_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/test/bench/coremark"
  cp "$REPO/test/bench/run_coremark.sh" "$REPO/test/bench/coremark.lds" \
     "$REPO/test/bench/bench.lds" "$d/test/bench/"
  cp "$REPO"/test/bench/coremark/*.c "$REPO"/test/bench/coremark/*.h \
     "$REPO/test/bench/coremark/PINNED.sha256" \
     "$REPO/test/bench/coremark/LICENSE.md" \
     "$REPO/test/bench/coremark/coremark.md5" "$d/test/bench/coremark/"
  printf '%s' "$d"
}

rc() { printf "%s/test/bench/run_coremark.sh no-such-sim 1 1 -O2" "$1"; }

d=$(rc_fixture)
probe "control: an unmodified vendor tree passes the manifest and pin checks" 1 \
  "is not an executable runner" "$(rc "$d")"

# A mutated vendored byte: shasum -c catches it, and prints why -- the same
# case CoreMark's own trademark terms exist to guard.
d=$(rc_fixture)
printf '\n' >> "$d/test/bench/coremark/core_main.c"
probe "a mutated vendored byte fails the pin, and says so" 1 \
  "no longer matches PINNED.sha256" "$(rc "$d")"

# A deleted manifest line: shasum -c never sees the file it was never told
# about, so only a two-way name comparison catches it.
d=$(rc_fixture)
grep -v 'core_util\.c$' "$d/test/bench/coremark/PINNED.sha256" \
  > "$d/test/bench/coremark/PINNED.sha256.new"
mv "$d/test/bench/coremark/PINNED.sha256.new" "$d/test/bench/coremark/PINNED.sha256"
probe "a file the manifest stopped naming is red before shasum ever runs" 1 \
  "does not have exactly the files PINNED.sha256" "$(rc "$d")"

# An unlisted file added: the concrete exploit. core_portme.h dropped in next
# to the vendored sources shadows this port's real header for every vendored
# unit's quoted #include, and shasum -c alone would report the tree
# unmodified.
d=$(rc_fixture)
cp "$REPO/test/bench/core_portme.h" "$d/test/bench/coremark/core_portme.h"
probe "an unlisted core_portme.h would shadow the port's header, and is caught" 1 \
  "core_portme.h" "$(rc "$d")"

# A malformed manifest line: shasum -c alone exits 0 on this, printing only a
# WARNING nothing here would have read -- --strict is what turns it red.
d=$(rc_fixture)
sed -i.bak -E 's/^[0-9a-f]{64}(  core_list_join\.c)$/deadbeef\1/' \
  "$d/test/bench/coremark/PINNED.sha256"
rm -f "$d/test/bench/coremark/PINNED.sha256.bak"
probe "a malformed manifest line is red under --strict, not a silent pass" 1 \
  "improperly formatted" "$(rc "$d")"

begin_group "make revendor-coremark"

# The manifest check earlier in this file grades PINNED.sha256 against the
# TREE; this recipe is the other half -- it grades the tree against UPSTREAM.
# A stub curl substitutes what "upstream" answers, so what is forced red
# below is exactly the byte comparison that stands between a substituted
# download and an endorsed vendor tree. RV_PREFIX has to match the pin the
# Makefile states -- it is not read back out of the Makefile here, the same
# way SS_ASSET above is a fixed fixture value rather than a second reader.
RV_PREFIX=coremark-1f483d5b8316753a742cbf5590caf5bd0a4e4777

rv() {  # $1 = bin dir with a curl stub on it
  printf "MAKEFLAGS= MFLAGS= MAKELEVEL= PATH='%s:%s' make --no-print-directory -C '%s' revendor-coremark" \
    "$1" "$PATH" "$REPO"
}

# Builds a real, valid tarball out of the shipping vendored files, optionally
# mutating or dropping one member -- so "the archive itself is well-formed"
# and "its bytes agree with the tree" are tested as two separate questions,
# the way make_curl_stub's garbage bytes alone could not.
rv_tarball() {  # $1 = output path, $2 = member to mutate (or ""), $3 = member to drop (or "")
  local out=$1 mutate=$2 drop=$3
  local src; src=$(mktemp -d "$tmp/rv-src.XXXXXX")
  mkdir -p "$src/$RV_PREFIX"
  cp "$REPO"/test/bench/coremark/*.c "$REPO"/test/bench/coremark/*.h \
     "$REPO/test/bench/coremark/LICENSE.md" "$REPO/test/bench/coremark/coremark.md5" \
     "$src/$RV_PREFIX/"
  [ -n "$mutate" ] && printf '\n/* mutated */\n' >> "$src/$RV_PREFIX/$mutate"
  [ -n "$drop" ] && rm -f "$src/$RV_PREFIX/$drop"
  tar czf "$out" -C "$src" "$RV_PREFIX"
}

# Serves one fixed tarball for every request, whatever -o names -- the same
# shape make_curl_stub uses, parameterised on which bytes to hand back.
rv_curl_stub() {  # $1 = bin dir, $2 = tarball to serve
  mkdir -p "$1"
  cat > "$1/curl" <<STUBEOF
#!/bin/sh
out=; prev=
for a in "\$@"; do
  if [ "\$prev" = "-o" ]; then out=\$a; fi
  prev=\$a
done
[ -n "\$out" ] && cp "$2" "\$out"
exit 0
STUBEOF
  chmod +x "$1/curl"
}

d=$(new_case); rv_tarball "$d/upstream.tar.gz" "" ""
rv_curl_stub "$d/bin" "$d/upstream.tar.gz"
probe "control: an archive matching the vendored tree exactly is endorsed" 0 \
  "matches eembc/coremark at" "$(rv "$d/bin")"

d=$(new_case); mkdir -p "$d/bin"
make_curl_stub "$d/bin/curl"
probe "a substituted, non-archive download is refused for every file, not silently skipped" 2 \
  "does NOT match eembc/coremark" "$(rv "$d/bin")"

d=$(new_case); rv_tarball "$d/upstream.tar.gz" "coremark.h" ""
rv_curl_stub "$d/bin" "$d/upstream.tar.gz"
probe "a real archive with one byte-mutated file is caught, and named" 2 \
  "DIFFERS  : coremark.h" "$(rv "$d/bin")"

d=$(new_case); rv_tarball "$d/upstream.tar.gz" "" "coremark.md5"
rv_curl_stub "$d/bin" "$d/upstream.tar.gz"
probe "a real archive missing one vendored member is caught, and named" 2 \
  "MISSING upstream: coremark.md5" "$(rv "$d/bin")"

if [ "$probes" -ne "$PROBES_EXPECTED" ]; then
  echo "error: ran $probes probes, expected $PROBES_EXPECTED." >&2
  echo "A probe was added or removed. Update PROBES_EXPECTED in the same commit;" >&2
  echo "the literal is what stops this file quietly covering less than it did." >&2
  exit 1
fi

if [ "$failed" -ne 0 ]; then
  echo "error: a graded comparison did not go red where it was supposed to." >&2
  echo "Read the probe above: either the grader stopped checking something, or" >&2
  echo "its diagnostic changed and this file has to be updated to match." >&2
  exit 1
fi

printf "  (%ss)\n" "$((SECONDS - group_started))"
echo "$probes graded comparisons, every failure path executed."
