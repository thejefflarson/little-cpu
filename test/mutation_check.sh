#!/bin/bash
# Applies a declared RTL mutation, runs the detectors, and requires exactly the
# detectors it is paired with to go red.
#
# Usage: mutation_check.sh [--only <mutation>] [--repo <dir>] [--manifest <file>]
#                          [--patches <dir>] [--expected-fail <file>]
#
# WHY THIS EXISTS, and how it differs from `make probe-gates`. probe-gates asks
# whether a graded comparison can report a failure at all; it is hermetic, so it
# cannot mutate the design and says nothing about whether a program still detects
# the hardware property it was written to detect. That question has a measured
# answer and it is not always yes: deleting `instr_fencei` from `serialize` in
# rtl/decoder.v leaves the whole test/asm suite green, and test/asm/selfmod.S
# carried a note claiming it caught exactly that. Every red-direction claim about
# a program in this repo rests on someone having run the mutation once, by hand,
# on the day it was written. This is what re-runs them.
#
# BOTH DIRECTIONS, against test/MUTATION_DETECTORS. A mutation nothing catches
# is red --
# the oracle it was paired with has gone quiet. A detector that goes red for a
# mutation it is not paired with is red too, because the pairing is the claim
# being graded and a set that drifts is a set nobody is reading. The better the
# pipeline gets the more likely the first case is: a store landing earlier is a
# win everywhere except in the program whose job was to notice it used to land
# late.
#
# NOT ON `make test`'s PATH. It rebuilds the cxxrtl runner for every mutation,
# which is most of a minute each, and it adds no ratchet.
#
# THE MUTATIONS ARE PATCHES, applied with `git apply`, which takes exact context
# or nothing. A patch that no longer applies is reported as such and the run
# stops: the RTL under it moved, so whatever the pairing claims was measured
# against a design that is gone.
#
# REVERTING IS NOT LEFT TO THE PATCH. rtl/ is snapshotted before anything is
# applied and restored from that snapshot on every exit path -- success, failure
# and SIGINT -- because a left-behind mutation is worse than no check at all.
# test/mutation_probe.sh forces that path, and the graded comparisons below,
# against a fixture.
set -euo pipefail
# comm and sort have to agree on collation, and they only do if both are told.
export LC_ALL=C

HERE=$(cd "$(dirname "$0")" && pwd)

REPO=""
MANIFEST=""
PATCH_DIR=""
EXPECTED_FAIL=""
ONLY=""

while [ "$#" -gt 0 ]; do
  case $1 in
    --only)          ONLY=${2:-};          shift 2 ;;
    --repo)          REPO=${2:-};          shift 2 ;;
    --manifest)      MANIFEST=${2:-};      shift 2 ;;
    --patches)       PATCH_DIR=${2:-};     shift 2 ;;
    --expected-fail) EXPECTED_FAIL=${2:-}; shift 2 ;;
    *) echo "usage: mutation_check.sh [--only <mutation>] [--repo <dir>]" >&2
       echo "       [--manifest <file>] [--patches <dir>] [--expected-fail <file>]" >&2
       exit 1 ;;
  esac
done

REPO=${REPO:-$(cd "$HERE/.." && pwd)}
REPO=$(cd "$REPO" && pwd)
MANIFEST=${MANIFEST:-$REPO/test/MUTATION_DETECTORS}
PATCH_DIR=${PATCH_DIR:-$REPO/test/mutations}
EXPECTED_FAIL=${EXPECTED_FAIL:-$REPO/test/EXPECTED_FAIL}
ASM_DIR=$REPO/test/asm
cd "$REPO"

fail() { echo "error: $*" >&2; exit 1; }

command -v git >/dev/null 2>&1 || fail "git is not on PATH; the mutations are patches."

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-mutation.XXXXXX")
test -n "$tmp" -a -d "$tmp" || fail "mktemp -d produced no usable directory."

# The snapshot is taken below, once the manifest has been read. Until then there
# is nothing to restore, and `restore` is a no-op in that window rather than
# depending on where the trap was installed.
SNAPSHOT=""
restore() {
  [ -n "$SNAPSHOT" ] || return 0
  local f base
  for f in "$SNAPSHOT"/*.v; do
    base=$(basename "$f")
    cmp -s "$f" "$REPO/rtl/$base" || cp "$f" "$REPO/rtl/$base"
  done
}
cleanup() { restore; rm -rf "$tmp"; }
# INT and TERM are trapped so that the EXIT trap runs at all: bash does not
# promise to run one when the shell dies from a signal it has no handler for,
# and an interrupted run is exactly when a mutation would be left in the tree.
# A shell started with `&` from a script has SIGINT ignored on entry and cannot
# trap it, so check this path with SIGTERM rather than concluding it is broken.
trap cleanup EXIT
trap 'echo; echo "interrupted -- reverting rtl/" >&2; exit 130' INT TERM

# ---------------------------------------------------------------- the manifest
#
# All of this runs before a single mutation is applied. A manifest that does not
# describe the tree is not something to discover four minutes into a run.

[ -f "$MANIFEST" ] && [ -r "$MANIFEST" ] || fail \
"manifest '$MANIFEST' does not exist or is not readable. It is the pairing
between each mutation and the detectors that must catch it; without it there is
nothing being graded."

lines=$(sed -e 's/#.*//' "$MANIFEST" | awk 'NF { $1=$1; print }')
[ -n "$lines" ] || fail "manifest '$MANIFEST' names no detectors."

malformed=$(printf '%s\n' "$lines" | awk '
  $1 !~ /^[a-z0-9][a-z0-9-]*$/ {
    print "  " $0 "   (a mutation name is lower case, digits and hyphens)"; next }
  $2 == "bench" {
    if (NF != 3) print "  " $0 "   (a bench detector is: <mutation> bench <bench>)"; next }
  $2 == "asm" {
    if (NF < 4) print "  " $0 "   (an asm detector is: <mutation> asm <program> <STATUS>)"
    else if ($3 !~ /\.[Sc]$/) print "  " $0 "   (not a .S or .c program)"
    next }
  { print "  " $0 "   (the second field is the leg: bench or asm)" }')
if [ -n "$malformed" ]; then
  echo "error: $MANIFEST has lines that are not detector pairings:" >&2
  printf '%s\n' "$malformed" >&2
  exit 1
fi

duplicates=$(printf '%s\n' "$lines" | sort | uniq -d)
if [ -n "$duplicates" ]; then
  echo "error: $MANIFEST states the same pairing twice:" >&2
  printf '  %s\n' "$duplicates" >&2
  echo "The comparison below is a set equality, so a repeated line is a line" >&2
  echo "that cannot be wrong." >&2
  exit 1
fi

# A detector naming something that is not there could never fire, and the run
# would read that as an oracle gone quiet.
#
# `if` rather than `case`: the bash macOS ships is 3.2, whose parser reads the
# `)` closing a case pattern inside `$( )` as the one closing the substitution.
missing_detector=$(printf '%s\n' "$lines" | while read -r m leg det rest; do
  if [ "$leg" = bench ]; then
    [ -f "$REPO/test/$det.v" ] || echo "  $m bench $det   (no test/$det.v)"
  elif [ "$leg" = asm ]; then
    [ -f "$ASM_DIR/$det" ]     || echo "  $m asm $det   (no test/asm/$det)"
  fi
done)
if [ -n "$missing_detector" ]; then
  echo "error: $MANIFEST pairs mutations against detectors that do not exist:" >&2
  printf '%s\n' "$missing_detector" >&2
  exit 1
fi

printf '%s\n' "$lines" | awk '{ print $1 }' | sort -u > "$tmp/declared_mutations"

[ -d "$PATCH_DIR" ] || fail "mutation directory '$PATCH_DIR' does not exist."
shopt -s nullglob
patches=("$PATCH_DIR"/*.patch)
shopt -u nullglob
[ "${#patches[@]}" -gt 0 ] || fail "no patches in '$PATCH_DIR'; there is nothing to apply."
for p in "${patches[@]}"; do basename "$p" .patch; done | sort > "$tmp/present_mutations"

rc=0
unpaired=$(comm -13 "$tmp/declared_mutations" "$tmp/present_mutations")
if [ -n "$unpaired" ]; then
  echo "error: $PATCH_DIR has mutations $MANIFEST does not pair:" >&2
  printf '  %s\n' "$unpaired" >&2
  echo "A mutation with no detector is one nothing catches. Pair it, or delete" >&2
  echo "it, in the same commit." >&2
  rc=1
fi
undeclared=$(comm -23 "$tmp/declared_mutations" "$tmp/present_mutations")
if [ -n "$undeclared" ]; then
  echo "error: $MANIFEST names mutations that are not in $PATCH_DIR:" >&2
  printf '  %s\n' "$undeclared" >&2
  echo "Nothing would be applied for them, and their pairings would read as" >&2
  echo "green. Add the patch, or remove the lines." >&2
  rc=1
fi
[ "$rc" -eq 0 ] || exit 1

# Every patch is checked here rather than at the moment it is applied, so one
# whose context has moved is reported before anything is built.
for p in "${patches[@]}"; do
  name=$(basename "$p" .patch)
  [ -s "$p" ] || fail "$p is empty; it would apply cleanly and mutate nothing."
  # Both sides, so a patch that creates or deletes a file is rejected rather
  # than half-checked: /dev/null on either side does not match the pattern
  # below, and a file this run created is one the snapshot cannot restore.
  touched=$(awk '/^(---|\+\+\+) /{ sub(/^[ab]\//, "", $2); print $2 }' "$p")
  [ -n "$touched" ] || fail "$p names no files to patch."
  outside=$(printf '%s\n' "$touched" | grep -v '^rtl/[A-Za-z0-9_]*\.v$' || true)
  if [ -n "$outside" ]; then
    echo "error: $p edits files outside rtl/:" >&2
    printf '  %s\n' "$outside" >&2
    echo "A mutation edits the design. One that edits the tests or the harness" >&2
    echo "grades nothing." >&2
    exit 1
  fi
  if ! git apply --check "$p" 2>"$tmp/apply.err"; then
    echo "error: $name no longer applies to this tree:" >&2
    sed 's/^/  /' "$tmp/apply.err" >&2
    echo "The RTL it describes has moved, so the pairing in $MANIFEST was" >&2
    echo "measured against a design that is gone. Re-derive the mutation and" >&2
    echo "re-measure what catches it." >&2
    exit 1
  fi
done

echo "$MANIFEST: $(wc -l < "$tmp/declared_mutations" | tr -d ' ') mutations," \
     "$(printf '%s\n' "$lines" | wc -l | tr -d ' ') pairings, every patch applies."

SNAPSHOT=$tmp/pristine
mkdir -p "$SNAPSHOT"
cp "$REPO"/rtl/*.v "$SNAPSHOT/"

# ------------------------------------------------------------------- the legs
#
# Each prints the detectors that went red, one per line, and exits nonzero only
# if it could not run at all. Both are overridable so test/mutation_probe.sh can
# drive this script's grading without a toolchain; nothing else sets them.

bench_leg() {
  if [ -n "${MUTATION_BENCH_LEG:-}" ]; then "$MUTATION_BENCH_LEG"; return; fi
  make -s check-unit-benches > "$tmp/benches.log" 2>&1 || {
    echo "the bench list does not match test/*_tb.v:" >&2
    cat "$tmp/benches.log" >&2
    return 1
  }
  local list b
  list=$(make -s unit-bench-list) || { echo "cannot read the bench list" >&2; return 1; }
  [ -n "$list" ] || { echo "the bench list is empty" >&2; return 1; }
  for b in $list; do
    # A bench that will not elaborate under the mutation counts as red. The
    # suite leg below treats a design that will not build as an error, so a
    # mutation the compiler rejects outright never reaches a verdict at all.
    make -s "test-unit-$b" > "$tmp/bench.$b.log" 2>&1 || echo "$b"
  done
}

suite_leg() {
  if [ -n "${MUTATION_SUITE_LEG:-}" ]; then "$MUTATION_SUITE_LEG"; return; fi
  if ! make sim > "$tmp/sim.log" 2>&1; then
    echo "the design under this mutation does not build:" >&2
    tail -20 "$tmp/sim.log" >&2
    return 1
  fi
  set +e
  ./test/run_tests.sh ./sim test/asm "$EXPECTED_FAIL" test/OBSERVED_FLOOR \
    > "$tmp/suite.log" 2>&1
  set -e
  local total rows
  total=$(awk '/^[0-9]+\/[0-9]+ passed/ { split($1, n, "/"); print n[2]; exit }' "$tmp/suite.log")
  if [ -z "$total" ]; then
    echo "the suite did not run to a verdict:" >&2
    tail -20 "$tmp/suite.log" >&2
    return 1
  fi
  # The table is parsed rather than the verdict line read, so each program's
  # status travels with its name. A row shape this cannot read would quietly
  # report an empty detector set, so the same pass counts the rows it read and
  # nothing is published until that count meets the verdict's own denominator.
  awk -v countfile="$tmp/rows" '
    $1 ~ /^[A-Za-z0-9_]+\.[Sc]$/ && /retires=/ {
      rows++
      s = $0
      sub(/^[^ \t]+[ \t]+/, "", s)
      sub(/[ \t]*retires=.*$/, "", s)
      gsub(/^[ \t]+|[ \t]+$/, "", s)
      if (s != "PASS") print $1, s
    }
    END { print rows + 0 > countfile }' "$tmp/suite.log" > "$tmp/suite.rows"
  rows=$(cat "$tmp/rows")
  if [ "$rows" -ne "$total" ]; then
    echo "read $rows table rows out of $total programs; the table shape moved." >&2
    tail -20 "$tmp/suite.log" >&2
    return 1
  fi
  cat "$tmp/suite.rows"
}

# observe <label> -> $tmp/observed.<label>, one sorted detector token per line
observe() {
  local label=$1 out=$tmp/observed.$1
  export MUTATION_NAME=$label
  bench_leg > "$tmp/leg.bench" || return 1
  suite_leg > "$tmp/leg.asm"   || return 1
  { awk 'NF { print "bench", $0 }' "$tmp/leg.bench"
    awk 'NF { print "asm", $0 }'   "$tmp/leg.asm"; } | sort > "$out"
}

# ---------------------------------------------------------------- the baseline
#
# Measured, not assumed. Without it a program that is already red is charged to
# the first mutation applied, and a bench that is already red to all of them.

echo
echo "== baseline: the tree as it stands, unmutated"
observe baseline || fail "the baseline run did not complete; nothing was mutated."

baseline_bench=$(awk '$1 == "bench"' "$tmp/observed.baseline")
if [ -n "$baseline_bench" ]; then
  echo "error: these benches are red before any mutation is applied:" >&2
  printf '%s\n' "$baseline_bench" | sed 's/^/  /' >&2
  echo "Every verdict below would be charged to a mutation that did not cause" >&2
  echo "it. Fix the tree first." >&2
  exit 1
fi

baseline_asm=$(awk '$1 == "asm" { $1 = ""; sub(/^ /, ""); print }' "$tmp/observed.baseline" | sort)
expected_asm=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)
if [ "$baseline_asm" != "$expected_asm" ]; then
  echo "error: the unmutated suite does not match $EXPECTED_FAIL:" >&2
  diff <(printf '%s\n' "$expected_asm") <(printf '%s\n' "$baseline_asm") \
    --label expected --label actual >&2 || true
  echo "A mutation's detectors are the difference from this baseline, so it has" >&2
  echo "to be the baseline the rest of the repo grades against." >&2
  exit 1
fi
echo "baseline clean: no bench red, and the suite matches $EXPECTED_FAIL."

# --------------------------------------------------------------- the mutations

to_run=$(cat "$tmp/declared_mutations")
if [ -n "$ONLY" ]; then
  grep -qxF "$ONLY" "$tmp/declared_mutations" \
    || fail "'$ONLY' is not a mutation in $MANIFEST."
  to_run=$ONLY
fi

graded=0
failed=0
for m in $to_run; do
  echo
  echo "== $m"
  awk '/^\+\+\+ /{ sub(/^b\//, "", $2); print "   edits " $2 }' "$PATCH_DIR/$m.patch"

  git apply "$PATCH_DIR/$m.patch" \
    || fail "$m did not apply after checking cleanly; rtl/ has been restored."

  observe_status=0
  observe "$m" || observe_status=$?
  restore
  [ "$observe_status" -eq 0 ] || fail "$m: a leg did not run; rtl/ has been restored."

  printf '%s\n' "$lines" | awk -v m="$m" '$1 == m { $1 = ""; sub(/^ /, ""); print }' \
    | sort > "$tmp/declared.$m"

  # What this mutation did, over and above what the unmutated tree already does.
  comm -13 "$tmp/observed.baseline" "$tmp/observed.$m" > "$tmp/fired.$m"
  quieted=$(comm -23 "$tmp/observed.baseline" "$tmp/observed.$m")

  mutation_failed=0
  if [ -n "$quieted" ]; then
    echo "   FAIL a detector that is red in the baseline stopped being red:" >&2
    printf '%s\n' "$quieted" | sed 's/^/        /' >&2
    mutation_failed=1
  fi

  silent=$(comm -23 "$tmp/declared.$m" "$tmp/fired.$m")
  extra=$(comm -13 "$tmp/declared.$m" "$tmp/fired.$m")

  if [ -n "$silent" ]; then
    echo "   FAIL declared detectors that did NOT go red:" >&2
    printf '%s\n' "$silent" | sed 's/^/        /' >&2
    echo "        Either the property moved and the oracle stopped seeing it," >&2
    echo "        or it never saw it. Re-measure before editing $MANIFEST." >&2
    mutation_failed=1
  fi
  if [ -n "$extra" ]; then
    echo "   FAIL detectors that went red and are not paired with this mutation:" >&2
    printf '%s\n' "$extra" | sed 's/^/        /' >&2
    echo "        The pairing is the claim. Add the line if it is what you mean." >&2
    mutation_failed=1
  fi

  if [ "$mutation_failed" -eq 0 ]; then
    sed 's/^/   ok   caught by /' "$tmp/declared.$m"
  else
    failed=$((failed + 1))
  fi
  graded=$((graded + 1))
done

restore
# Contents and the file list both, so "rtl/ came back" is the whole statement
# rather than a statement about the files that happened to be there first.
leftover=$({
  for f in "$SNAPSHOT"/*.v; do
    base=$(basename "$f")
    cmp -s "$f" "$REPO/rtl/$base" || echo "  rtl/$base"
  done
  comm -13 <(cd "$SNAPSHOT" && ls ./*.v | sort) \
           <(cd "$REPO/rtl" && ls ./*.v | sort) | sed 's|^\./|  rtl/|'
} | sort -u)
if [ -n "$leftover" ]; then
  echo "error: rtl/ did not come back to what it was:" >&2
  printf '%s\n' "$leftover" >&2
  exit 1
fi

echo
if [ "$failed" -ne 0 ]; then
  echo "$failed of $graded mutations were not caught by exactly their detectors." >&2
  exit 1
fi
echo "$graded mutations, each caught by exactly the detectors it is paired with."
