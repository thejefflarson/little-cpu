#!/bin/bash
# The riscv-formal ladder's gate. Asserts TWO things, and both are set
# equalities in both directions (ADR-0014's contract, applied to the formal
# ladder per ADR-0022 -- an unexpected *pass*, or an unexpected *check*,
# fails this as loudly as a regression):
#
#   1. SHAPE.  The checks genchecks generated (formal/checks/*.sby) are
#      exactly the ones formal/EXPECTED_CHECKS names.
#   2. VERDICT. The checks whose status is not PASS are exactly the ones
#      formal/EXPECTED_FAIL names, WITH THE STATUS EACH ONE CARRIES.
#
# (2) matched on the name alone until ADR-0036 was executed, and that made
# ERROR, TIMEOUT, UNKNOWN and NO-STATUS indistinguishable from FAIL to this
# gate. ADR-0036 measured the consequence rather than argued it: a ladder run
# on a machine WITHOUT `btorsim` reported "82 checks: 72 pass, 10 fail /
# Failure list matches EXPECTED_FAIL exactly" and exited 0, while all ten of
# those checks had status `ERROR 16 2`. `btormc` had found the
# counterexamples; `sby` then failed to render the traces, because the step
# that does so shells out to `btorsim`. "A real counterexample at the
# configured depth" and "the trace renderer is missing" were the same result.
# The failure mode that matters is the inverse: if `btorsim` vanished from CI's
# pinned OSS CAD Suite, every red check would flip FAIL -> ERROR,
# the set equality would still match, and the ladder would stay green having
# stopped distinguishing a proof failure from a tooling failure. Same shape as
# ADR-0033's gaps -- a check that can stop checking without anything going red
# -- one level down, in the status field rather than the check list. This is
# the identical amendment ADR-0035 made to test/EXPECTED_FAIL, applied to the
# formal side, and the reasoning is written out in formal/EXPECTED_FAIL's own
# header.
#
# (1) is not decoration. Without it this script could not tell a ladder that
# shrank from one that passed. formal/checks.cfg's [depth] table is the list
# of checks that EXIST -- genchecks skips any check with no depth line,
# silently -- so deleting one line removes a check from the results AND from
# EXPECTED_FAIL at the same time, and (2) alone reports a clean match on
# less coverage than it claims. That is ADR-0033's gap 1, and (1) closes it.
#
# This script deliberately enumerates the generated `*.sby` FILES, not the
# run directories. `sby` creates a directory only for a check it actually
# STARTS, so globbing directories made a generated-but-never-scheduled check
# invisible: it fell out of the actual set, it was never in the baseline,
# and set equality called that a match. Reading the .sby files instead means
# such a check resolves to NO-STATUS and counts as non-PASS, which is what
# this script's header has promised since it was written -- "make stopped
# scheduling it, or it's still running" is a failure, not a quiet pass.
#
# A check named in EXPECTED_CHECKS with no .sby on disk is likewise
# NO-STATUS, so a lost [depth] line trips BOTH assertions rather than
# neither.
#
# Nothing here is a verdict about the core. Every check on this ladder is
# `mode bmc`: PASS means no counterexample was found within that check's
# configured depth, not that the property holds. And the whole ladder runs
# under RISCV_FORMAL_ALTOPS, so a passing insn_mul/insn_div says nothing
# whatever about the real multiplier or divider (ADR-0010).
#
# Usage: check-baseline.sh <checks-dir> <expected-fail-file> [expected-checks-file]
set -u

if [ $# -lt 2 ] || [ $# -gt 3 ]; then
  echo "usage: $0 <checks-dir> <expected-fail-file> [expected-checks-file]" >&2
  exit 2
fi

CHECKS_DIR=$1
EXPECTED_FAIL=$2
EXPECTED_CHECKS=${3:-$(dirname "$0")/EXPECTED_CHECKS}

# READABLE, not merely present. This script sets `set -u` and neither `-e` nor
# `pipefail`, so a `sed` that cannot open its input writes to stderr and yields
# an EMPTY string, and an empty expected set against an all-passing ladder
# prints "Failure list matches ... exactly" and exits 0 -- having compared
# nothing, and having silently dropped whatever the baseline named. Measured
# with `chmod 000 formal/EXPECTED_FAIL`; see the probe in test/probe_gates.sh.
# ADR-0035 item 4 made exactly this fix on test/run_tests.sh's baseline and it
# was never carried across to the formal side.
for f in "$EXPECTED_FAIL" "$EXPECTED_CHECKS"; do
  if [ ! -f "$f" ]; then
    echo "error: no such file: $f" >&2
    exit 2
  fi
  if [ ! -r "$f" ]; then
    echo "error: $f exists but is not readable." >&2
    echo "An unreadable baseline reads as an EMPTY one here, which matches an" >&2
    echo "all-passing ladder exactly and exits 0. Refusing to grade." >&2
    exit 2
  fi
done

# `#` comments and blank lines out, as both baseline files already allow.
# Interior whitespace is squeezed too, so EXPECTED_FAIL's two fields can be
# column-aligned for a human without changing what is compared. `NF` must gate
# the rebuild rather than follow it: awk forces NF to 1 when $1 is assigned, so
# `{$1=$1} NF` would resurrect every blank and comment-only line as an empty
# entry. (Same idiom, same reason, as test/run_tests.sh.)
strip() {
  sed -e 's/#.*//' "$1" | awk 'NF { $1 = $1; print }' | sort
}

expected_checks=$(strip "$EXPECTED_CHECKS")
expected_fail=$(strip "$EXPECTED_FAIL")

# THE STATUS VOCABULARY, ENUMERATED IN BOTH DIRECTIONS.
#
# `sby` writes its verdict as the first word of <workdir>/status, and the rest
# of that line is engine bookkeeping (`PASS 0 31`, `ERROR 16 2`) that no
# baseline should ever pin. Only the first word is compared.
#
# known_status: everything sby can write, plus this script's own NO-STATUS for
# "there is no status file". A status outside this set means sby's output
# changed under us -- a pin bump, a different engine -- and is reported rather
# than bucketed into "not PASS", because bucketing is the whole defect this
# gate just stopped having.
known_status() {
  case $1 in
    PASS | FAIL | ERROR | UNKNOWN | TIMEOUT | NO-STATUS) return 0 ;;
    *) return 1 ;;
  esac
}

# baselineable_status: the strictly smaller set a line in EXPECTED_FAIL may
# carry. Three are rejected, each for its own reason:
#
#   PASS       is unreachable here by construction -- a PASS check never
#              enters the failure set -- so a line carrying it could never
#              match anything, which is a comparison whose failure branch is
#              the only branch.
#   ERROR      is sby failing to run or to render, not the core failing a
#              property. ADR-0036: "with ERROR never a legitimate baselined
#              value". Baselining one would re-create the btorsim hole with
#              this gate's blessing on it.
#   NO-STATUS  is "the check was generated and never scheduled, or is still
#              running". Same argument: a broken harness, not a known-red
#              property.
#
# TIMEOUT and UNKNOWN ARE accepted. They are budget exhaustion rather than
# tooling breakage -- a real, recorded verdict about a check that did not
# converge (ADR-0023's `reg` was exactly this before ADR-0024 changed the
# engine) -- and a change that turned one into the other is a change this gate
# should report, which is only possible if both are spellable.
baselineable_status() {
  case $1 in
    FAIL | TIMEOUT | UNKNOWN) return 0 ;;
    *) return 1 ;;
  esac
}

# Validate the baseline's FORMAT before anything expensive, and before any
# comparison -- a rejected line must read as "this file is wrong", never as a
# regression in the ladder. Exit 2 is this script's existing code for "the
# inputs are broken", as distinct from exit 1, "the ladder disagrees with
# them".
#
# A one-field line is the pre-ADR-0036 format. Accepting it silently would
# make every legacy entry unmatchable in a way that reads exactly like a
# regression, so it is named instead. An empty file has no lines and is
# therefore fine -- which is the state the ladder is in today.
baseline_errors=""
while IFS= read -r line; do
  [ -n "$line" ] || continue
  # shellcheck disable=SC2086 # deliberate word split: the line is normalised.
  set -- $line
  case $# in
    1)
      baseline_errors+="  $line
      -> no status field. The format is '<check>  <STATUS>'.
"
      ;;
    2)
      if ! known_status "$2"; then
        baseline_errors+="  $line
      -> '$2' is not a status sby can produce. Accepted: PASS FAIL ERROR
         UNKNOWN TIMEOUT NO-STATUS.
"
      elif ! baselineable_status "$2"; then
        baseline_errors+="  $line
      -> '$2' must never be baselined. PASS cannot appear in a failure set at
         all; ERROR and NO-STATUS are a broken harness rather than a known-red
         property, and parking one here is what ADR-0036 exists to forbid.
         Baselineable: FAIL TIMEOUT UNKNOWN.
"
      fi
      ;;
    *)
      baseline_errors+="  $line
      -> $# fields. The format is exactly two: '<check>  <STATUS>'. sby's
         trailing engine numbers ('PASS 0 31') are not part of the status.
"
      ;;
  esac
done <<< "$expected_fail"

if [ -n "$baseline_errors" ]; then
  echo "error: $EXPECTED_FAIL is malformed (ADR-0036 format):" >&2
  printf '%s' "$baseline_errors" >&2
  echo "Read that file's header before editing it; the format is the gate." >&2
  exit 2
fi

# The generated set: one line per checks/<name>.sby. `find` rather than a
# glob, so a missing or empty directory yields nothing instead of the
# literal unexpanded pattern -- which would otherwise become a one-element
# set named `*.sby` and produce a confusing diff instead of the explicit
# error below.
generated=$(find "$CHECKS_DIR" -maxdepth 1 -name '*.sby' \
  -exec basename {} .sby \; 2>/dev/null | sort)

if [ -z "$generated" ]; then
  echo "error: no *.sby files under $CHECKS_DIR -- the ladder was never" >&2
  echo "       generated. Run 'make -C formal checks' first." >&2
  exit 2
fi

failed=0

if [ "$generated" != "$expected_checks" ]; then
  echo "Generated check set does NOT match $EXPECTED_CHECKS:"
  diff <(echo "$expected_checks") <(echo "$generated") \
    --label expected --label generated
  echo
  echo "A check that disappeared here lost its formal/checks.cfg [depth] line"
  echo "(ADR-0033). A check that appeared needs a [depth] line, an"
  echo "EXPECTED_CHECKS line, and -- if red -- an $EXPECTED_FAIL entry."
  echo
  failed=1
fi

# Status is read over the UNION of what was generated and what was expected,
# so a name missing from either side still gets a verdict rather than
# dropping out of the tally entirely.
all_checks=$(printf '%s\n%s\n' "$generated" "$expected_checks" | sort -u)

total=0
declare -a actual_fail=()
declare -a unknown_statuses=()
for name in $all_checks; do
  total=$((total + 1))
  # First word of the first NON-BLANK line. sby's status line is
  # `<VERDICT> <engine> <depth>`; only the verdict is compared, because the
  # numbers after it are bookkeeping that moves with the engine and would
  # make the baseline pin something it is not asserting.
  status=$(awk 'NF { print $1; exit }' "$CHECKS_DIR/$name/status" 2>/dev/null)
  if [ -z "$status" ]; then
    status="NO-STATUS"
  fi
  if ! known_status "$status"; then
    unknown_statuses+=("$name $status")
  fi
  if [ "$status" != "PASS" ]; then
    actual_fail+=("$name $status")
  fi
done

passed=$((total - ${#actual_fail[@]}))
echo "$total checks: $passed pass, ${#actual_fail[@]} fail"

# A status sby has never written here before is reported on its own, not left
# to be read out of a diff. It means the tool's output changed, which is a
# different problem from the ladder disagreeing with its baseline, and the two
# want different fixes. It still counts as non-PASS above, so it cannot pass
# quietly either way.
if [ ${#unknown_statuses[@]} -gt 0 ]; then
  echo
  echo "Unrecognised status(es) on disk -- not one of PASS FAIL ERROR UNKNOWN"
  echo "TIMEOUT. sby's output has changed; do not baseline these, fix the"
  echo "vocabulary in $0 after reading what the pin now emits:"
  printf '  %s\n' "${unknown_statuses[@]}"
  failed=1
fi

actual_sorted=$(printf '%s\n' "${actual_fail[@]:-}" | awk 'NF { $1 = $1; print }' | sort)

if [ "$actual_sorted" = "$expected_fail" ]; then
  echo "Failure list matches $EXPECTED_FAIL exactly (name and status)."
else
  echo
  echo "Failure list does NOT match $EXPECTED_FAIL:"
  diff <(echo "$expected_fail") <(echo "$actual_sorted") \
    --label expected --label actual
  echo
  echo "A name on one side only is a check whose verdict moved. A name on BOTH"
  echo "sides with different statuses is the ADR-0036 case: the check is still"
  echo "red, but for a different reason than the one baselined -- ERROR and"
  echo "NO-STATUS mean the harness broke, not that the property failed."
  failed=1
fi

if [ "$failed" -eq 0 ]; then
  echo "Generated check set matches $EXPECTED_CHECKS exactly ($total checks)."
fi

exit "$failed"
