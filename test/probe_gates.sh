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
# Hermetic: no RISC-V toolchain, no `sim`, no Sail, no yosys, no sby, so this can
# run inside `make test` on any machine. It is all fork and no work, so the wall
# time is the host's property rather than this file's.
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
PROBES_EXPECTED=236

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
       "atomic=0 hazard=10 serialize=0 operand=0 fetch=0 unattributed=$unattr"
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
  "if (!ch0_spec_trap) begin" "$SM $d/monitor.v"

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
d=$(sm_fixture)
sm_mutate "$d/monitor.v" <<'PY'
import sys
p = sys.argv[1]
t = open(p).read().replace('ch0_handle_error(101, "mismatch in trap");', ';', 1)
open(p, 'w').write(t)
PY
probe "layer 3: error 101 vanishing from the OUTPUT is caught after every rule" 1 \
  "the trap comparison survives" "$SM $d/monitor.v"

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
add.S cycles=40 issue=10 divider=0 atomic=0 hazard=20 serialize=0 operand=10 fetch=0 unattributed=0 retires=10
lw.S cycles=40 issue=10 divider=0 atomic=0 hazard=5 serialize=0 operand=25 fetch=0 unattributed=0 retires=10
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
  mkdir -p "$d/rtl" "$d/test/asm" "$d/test/bench"
  cp "$REPO"/rtl/memory.v "$REPO"/rtl/timer.v "$REPO"/rtl/imemory.v \
     "$REPO"/rtl/littlesoc.v "$d/rtl/"
  cp "$REPO"/test/testbench.v "$REPO"/test/cxxrtl.cc "$REPO"/test/cosim.cc "$d/test/"
  cp "$REPO"/test/asm/riscv_test.h "$REPO"/test/asm/sections.lds \
     "$REPO"/test/asm/boot.lds "$d/test/asm/"
  cp "$REPO"/test/bench/bench.lds "$d/test/bench/"
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
  cp "$REPO/docs/ideas/fit-the-core-on-the-up5k.md" "$d/docs/ideas/"
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

echo
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
