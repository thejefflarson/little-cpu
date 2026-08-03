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
PROBES_EXPECTED=125

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
# ladder, so run_tests.sh's grading of that ladder can be probed without the
# elaborated design.
[ -z "${STUB_SIM_NOCOUNTS:-}" ] && \
  echo "RETIRES ${STUB_SIM_RETIRES:-10} SPEC-CHECKED ${STUB_SIM_SPEC:-10}"
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

# `leg-rt` / `leg-rc` are scratch copies of the two suite runners, because each
# resolves its helper scripts relative to its own path.
mkdir -p "$tmp/bin-none" "$tmp/leg-rt" "$tmp/leg-rc" "$tmp/leg-rc-nopy"

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
make_cosim_bin_stub "$tmp/dut"
cp "$HERE/run_tests.sh" "$HERE/check_suite_shape.sh" "$tmp/leg-rt/"
cp "$HERE/run_cosim.sh" "$HERE/check_suite_shape.sh" "$tmp/leg-rc/"
make_cosim_py_stub "$tmp/leg-rc/cosim.py"
cp "$HERE/run_cosim.sh" "$HERE/check_suite_shape.sh" "$tmp/leg-rc-nopy/"
make_cosim_py_stub "$tmp/leg-rc-nopy/cosim.py"
chmod -x "$tmp/leg-rc-nopy/cosim.py"

begin_group "test/check_suite_shape.sh"

SHAPE="$HERE/check_suite_shape.sh"

shape_fixture() {
  local d; d=$(new_case)
  mkdir -p "$d/asm"
  : > "$d/asm/add.S"; : > "$d/asm/lw.S"
  printf '# a comment\nadd.S 10 10\nlw.S 5 5\n' > "$d/MANIFEST"
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
probe "a manifest entry that is not a .S name is a typo, not a phantom" 1 \
  "is not a '<test>.S' name" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); printf 'add.S 10 10\nadd.S 10 10\nlw.S 5 5\n' > "$d/MANIFEST"
probe "a duplicated manifest line would make comm non-symmetric" 1 \
  "names the same program more than once" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); rm "$d/asm/add.S" "$d/asm/lw.S"
probe "an empty asm directory is red rather than a suite of size zero" 1 \
  "no .S programs found" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); rm "$d/asm/lw.S"
probe "a SHRUNK suite: the manifest names a program the tree does not have" 1 \
  "The suite has SHRUNK" "$SHAPE '$d/asm' '$d/MANIFEST'"

d=$(shape_fixture); : > "$d/asm/unlisted.S"
probe "the other direction: a .S that landed without a manifest line" 1 \
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
  "are not '<test>.S <retires> <spec-checked>'" "$(rt "$d")"

d=$(rt_fixture); printf 'add.S ten 10\n' > "$d/FLOOR"
probe "a non-numeric floor is named rather than compared arithmetically" 1 \
  "are not '<test>.S <retires> <spec-checked>'" "$(rt "$d")"

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
probe "control: a full-pass ladder against an empty baseline is green" 0 \
  "Failure list matches" "$(cbs "$d")"

probe "wrong argument count is exit 2 -- the inputs are broken, not the ladder" 2 \
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
probe "a ladder that was never generated is exit 2, not zero checks passing" 2 \
  "the ladder was never" "$(cbs "$d")"

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
probe "a report missing the floor is a ratchet, not a suggestion" 1 \
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
