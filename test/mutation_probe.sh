#!/bin/bash
# Forces every graded comparison in test/mutation_check.sh red, for its own
# reason, against a fixture design nothing else uses.
#
# Usage: mutation_probe.sh          # every case; exit 0 only if all of them hold
#
# WHY THIS EXISTS. mutation_check.sh is the answer to "a grader that cannot fail
# is not a grader" for the .S suite's red directions — and it is a grader itself,
# so the same rule applies to it. It is also too slow to be a merge gate (it
# rebuilds the cxxrtl runner once per mutation), so nothing on `make test`'s path
# would ever notice its comparisons rotting. This runs there instead: it is bash
# and a stub, no cross compiler, no yosys, no iverilog.
#
# THE FIXTURE IS NOT THIS REPO. It is a temporary directory with two one-module
# `rtl/` files, two patches against them, a manifest, and two stub detector legs
# that read what to report out of files this script writes. That is what lets a
# case say "the declared detector stays quiet" without owning a design in which
# that is true.
#
# The revert path is a case like any other: the last two force a red run and an
# interrupted run, and both require the fixture's rtl/ to come back byte for
# byte. A left-behind mutation is worse than no check at all.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
DRIVER=$HERE/mutation_check.sh

command -v git >/dev/null 2>&1 || {
  echo "error: git is not on PATH, so no mutation can be applied and none of" >&2
  echo "the comparisons below can be forced. Skipping would report a green run" >&2
  echo "for checks that never executed." >&2
  exit 1
}

export FIXTURE
FIXTURE=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-mutprobe.XXXXXX")
test -n "$FIXTURE" -a -d "$FIXTURE"
trap 'rm -rf "$FIXTURE"' EXIT

DECODER='module decoder;
  wire serialize;
  assign serialize = a || b || c;
endmodule'
IMEMORY='module imemory;
  wire text_access;
  assign text_access = read || write;
endmodule'

reset_fixture() {
  rm -rf "$FIXTURE"
  mkdir -p "$FIXTURE/rtl" "$FIXTURE/test/mutations" "$FIXTURE/test/asm" \
           "$FIXTURE/stub" "$FIXTURE/legs"

  printf '%s\n' "$DECODER" > "$FIXTURE/rtl/decoder.v"
  printf '%s\n' "$IMEMORY" > "$FIXTURE/rtl/imemory.v"

  cat > "$FIXTURE/test/mutations/alpha.patch" <<'PATCH'
--- a/rtl/decoder.v
+++ b/rtl/decoder.v
@@ -1,4 +1,4 @@
 module decoder;
   wire serialize;
-  assign serialize = a || b || c;
+  assign serialize = a || b;
 endmodule
PATCH

  cat > "$FIXTURE/test/mutations/beta.patch" <<'PATCH'
--- a/rtl/imemory.v
+++ b/rtl/imemory.v
@@ -1,4 +1,4 @@
 module imemory;
   wire text_access;
-  assign text_access = read || write;
+  assign text_access = read;
 endmodule
PATCH

  cat > "$FIXTURE/test/MUTATION_DETECTORS" <<'MANIFEST'
# fixture
alpha bench alpha_tb
beta  asm   alpha.S FAIL 2
MANIFEST

  echo '# fixture: nothing is expected to fail' > "$FIXTURE/test/EXPECTED_FAIL"
  echo '# fixture bench'   > "$FIXTURE/test/alpha_tb.v"
  echo '# fixture program' > "$FIXTURE/test/asm/alpha.S"

  # The stubs report whatever the case wrote for the mutation being applied, so
  # a case configures the design's behaviour by writing a file rather than by
  # owning RTL in which that behaviour is real. `.exit` makes a leg fail to run
  # at all; `.int` interrupts the driver from inside a leg.
  cat > "$FIXTURE/legs/leg.sh" <<'LEG'
#!/bin/bash
f="$FIXTURE/stub/$1.${MUTATION_NAME:-}"
if [ -f "$f.int" ]; then
  kill -INT "$PPID"
  sleep 1
  exit 0
fi
if [ -f "$f.exit" ]; then
  echo "stub leg refusing to run" >&2
  exit 1
fi
[ -f "$f" ] && cat "$f"
exit 0
LEG
  printf '#!/bin/bash\nexec "$FIXTURE/legs/leg.sh" bench\n'  > "$FIXTURE/legs/bench.sh"
  printf '#!/bin/bash\nexec "$FIXTURE/legs/leg.sh" suite\n'  > "$FIXTURE/legs/suite.sh"
  chmod +x "$FIXTURE/legs"/*.sh

  # The control: each mutation is caught by exactly what the manifest pairs it
  # with, and the unmutated tree is green.
  echo 'alpha_tb'       > "$FIXTURE/stub/bench.alpha"
  echo 'alpha.S FAIL 2' > "$FIXTURE/stub/suite.beta"

  export MUTATION_BENCH_LEG=$FIXTURE/legs/bench.sh
  export MUTATION_SUITE_LEG=$FIXTURE/legs/suite.sh
}

cases=0
failed=0
out=""
status=0

drive() {
  set +e
  out=$("$DRIVER" --repo "$FIXTURE" "$@" 2>&1)
  status=$?
  set -e
}

# The fixture's rtl/ must be exactly what it was, whatever the verdict was.
reverted() {
  [ "$(cat "$FIXTURE/rtl/decoder.v")" = "$DECODER" ] &&
  [ "$(cat "$FIXTURE/rtl/imemory.v")" = "$IMEMORY" ]
}

report() {
  local label=$1 verdict=$2
  cases=$((cases + 1))
  if [ "$verdict" = ok ]; then
    echo "ok   $label"
  else
    echo "FAIL $label" >&2
    printf '%s\n' "$out" | sed 's/^/       /' >&2
    failed=$((failed + 1))
  fi
}

# expect_red <label> <want-status> <fragment>...
#
# The fragments are pinned as well as the status. A status alone would be
# satisfied by the driver dying for an unrelated reason, which demonstrates
# nothing about the comparison the case is named for.
expect_red() {
  local label=$1 want=$2; shift 2
  local frag verdict=ok
  [ "$status" -eq "$want" ] || verdict="wrong status: $status, wanted $want"
  for frag in "$@"; do
    case $out in *"$frag"*) ;; *) verdict="did not say: $frag" ;; esac
  done
  reverted || verdict="rtl/ was left mutated"
  if [ "$verdict" = ok ]; then report "$label" ok; else
    out="$verdict"$'\n'"$out"; report "$label" bad; fi
}

expect_green() {
  local label=$1 frag=$2 verdict=ok
  [ "$status" -eq 0 ] || verdict="exited $status"
  case $out in *"$frag"*) ;; *) verdict="did not say: $frag" ;; esac
  reverted || verdict="rtl/ was left mutated"
  if [ "$verdict" = ok ]; then report "$label" ok; else
    out="$verdict"$'\n'"$out"; report "$label" bad; fi
}

echo "== the control: the fixture as declared"
reset_fixture
drive
expect_green "two mutations, each caught by exactly its detectors" \
  "2 mutations, each caught by exactly the detectors"

echo
echo "== a detector that stops firing"
reset_fixture
: > "$FIXTURE/stub/bench.alpha"
drive
expect_red "a declared bench detector that stays quiet" 1 \
  "declared detectors that did NOT go red" "bench alpha_tb"

reset_fixture
: > "$FIXTURE/stub/suite.beta"
drive
expect_red "a declared asm detector that stays quiet" 1 \
  "declared detectors that did NOT go red" "asm alpha.S FAIL 2"

reset_fixture
echo 'alpha.S FAIL 3' > "$FIXTURE/stub/suite.beta"
drive
expect_red "an asm detector that goes red a different way" 1 \
  "asm alpha.S FAIL 2" "asm alpha.S FAIL 3"

echo
echo "== a detector that fires for something it is not paired with"
reset_fixture
echo 'alpha_tb' > "$FIXTURE/stub/bench.beta"
drive
expect_red "an unpaired bench detector going red" 1 \
  "not paired with this mutation" "bench alpha_tb"

echo
echo "== the manifest itself"
reset_fixture
grep -v '^alpha ' "$FIXTURE/test/MUTATION_DETECTORS" > "$FIXTURE/m" \
  && mv "$FIXTURE/m" "$FIXTURE/test/MUTATION_DETECTORS"
drive
expect_red "a mutation whose pairing was removed" 1 \
  "does not pair" "alpha"

reset_fixture
rm "$FIXTURE/test/mutations/alpha.patch"
drive
expect_red "a pairing whose mutation does not exist" 1 \
  "names mutations that are not in" "alpha"

reset_fixture
echo 'alpha bench alpha_tb' >> "$FIXTURE/test/MUTATION_DETECTORS"
drive
expect_red "the same pairing stated twice" 1 \
  "states the same pairing twice"

reset_fixture
echo 'alpha units alpha_tb' >> "$FIXTURE/test/MUTATION_DETECTORS"
drive
expect_red "a leg that is neither bench nor asm" 1 \
  "the second field is the leg"

reset_fixture
echo 'alpha asm nosuch.S FAIL 1' >> "$FIXTURE/test/MUTATION_DETECTORS"
drive
expect_red "a detector naming a program that is not there" 1 \
  "detectors that do not exist" "nosuch.S"

echo
echo "== the mutations themselves"
reset_fixture
printf 'module decoder;\n  wire serialize;\n  assign serialize = a && b && c;\nendmodule\n' \
  > "$FIXTURE/rtl/decoder.v"
DECODER=$(cat "$FIXTURE/rtl/decoder.v")
drive
expect_red "a patch whose context has moved" 1 \
  "no longer applies to this tree"
DECODER='module decoder;
  wire serialize;
  assign serialize = a || b || c;
endmodule'

reset_fixture
cat > "$FIXTURE/test/mutations/alpha.patch" <<'PATCH'
--- a/test/asm/alpha.S
+++ b/test/asm/alpha.S
@@ -1 +1 @@
-# fixture program
+# not the design
PATCH
drive
expect_red "a mutation that edits something other than the design" 1 \
  "edits files outside rtl/" "test/asm/alpha.S"

echo
echo "== the baseline"
reset_fixture
echo 'alpha_tb' > "$FIXTURE/stub/bench.baseline"
drive
expect_red "a bench already red before anything is mutated" 1 \
  "red before any mutation is applied"

reset_fixture
echo 'alpha.S FAIL 9' > "$FIXTURE/stub/suite.baseline"
drive
expect_red "a suite that does not match its own baseline" 1 \
  "does not match" "alpha.S FAIL 9"

echo
echo "== a leg that cannot run at all"
reset_fixture
: > "$FIXTURE/stub/suite.alpha.exit"
drive
expect_red "a leg that fails to run under a mutation" 1 \
  "a leg did not run" "rtl/ has been restored"

echo
echo "== reverting"
# Both of these are about rtl/ coming back, which every case above also
# requires; they are here because a red run and an interrupted run are the two
# paths where it would not.
reset_fixture
: > "$FIXTURE/stub/bench.alpha"
drive
expect_red "rtl/ restored after a failing run" 1 \
  "declared detectors that did NOT go red"

reset_fixture
: > "$FIXTURE/stub/bench.alpha.int"
drive
expect_red "rtl/ restored after an interrupt" 130 \
  "interrupted -- reverting rtl/"

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $cases comparisons in mutation_check.sh did not behave as required." >&2
  exit 1
fi
echo "$cases comparisons, each forced red for its own reason, and rtl/ restored every time."
