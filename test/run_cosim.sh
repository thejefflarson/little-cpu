#!/bin/bash
# Runs every test/asm/*.S under Sail co-simulation (test/cosim.py, ADR-0032)
# and checks the resulting table against test/COSIM_EXPECTED_FAIL — the same
# set-equality contract, in both directions, on name-and-status pairs, that
# test/run_tests.sh applies to test/EXPECTED_FAIL (ADR-0014, ADR-0035).
# Invoked by `make cosim-suite`. See docs/adr/0039.
#
# Usage: run_cosim.sh <cosim-binary> <asm-dir> <expected-fail-file> <manifest>
#
# This is NOT on `make test`'s path and NOT in CI's required set, deliberately
# (ADR-0032): it needs a Sail install that `make test` must keep working
# without. What it is instead is the gate the block-RAM regfile change has to
# pass, because it is the only oracle in this repo that reads the real
# register array rather than the core's RVFI self-report.
#
# That makes a FALSE GREEN the failure mode that matters, exactly as in
# run_tests.sh, and the properties below exist only for it:
#
#   * the baseline is read and format-checked BEFORE the suite runs, so a
#     mistyped path fails in a second rather than after a full run having
#     compared nothing;
#   * the program list is checked against a MANIFEST, not merely for emptiness.
#     The emptiness guard caught a suite of size zero; it did nothing about a
#     suite that shrank from 52 programs to 12, which prints "12/12 agreed",
#     matches an empty baseline exactly and exits 0. The manifest is
#     test/OBSERVED_FLOOR — the same file test/run_tests.sh grades against, read
#     by the same test/check_suite_shape.sh, in both directions, before the
#     setup probe. Two legs that disagreed about what the suite is would be
#     worse than neither of them checking;
#   * the per-program status comes from cosim.py's own COSIM-STATUS line, not
#     from its exit code alone, and a run that produced no such line is
#     labelled COSIM-ERROR rather than being scored as a verdict about the
#     core. "The harness broke" and "the core diverged" must not be the same
#     table entry;
#   * the failure set records NAME AND STATUS. A baselined program that
#     starts diverging at a different point, in a different way, or that stops
#     running at all, is a red gate rather than a match.
#
# `set -e` is on; every place a nonzero status is expected handles it at its
# own call site.
set -euo pipefail

if [ "$#" -ne 4 ]; then
  echo "usage: run_cosim.sh <cosim-binary> <asm-dir> <expected-fail-file> <manifest>" >&2
  exit 1
fi

COSIM_BIN=$1
ASM_DIR=$2
EXPECTED_FAIL=$3
MANIFEST=$4
HERE=$(cd "$(dirname "$0")" && pwd)
COSIM_PY="$HERE/cosim.py"

if [ ! -f "$EXPECTED_FAIL" ] || [ ! -r "$EXPECTED_FAIL" ]; then
  echo "error: baseline '$EXPECTED_FAIL' does not exist or is not readable." >&2
  echo "The gate compares the divergence set against it; without it there is no gate." >&2
  exit 1
fi

if [ ! -x "$COSIM_PY" ]; then
  echo "error: $COSIM_PY is missing or not executable." >&2
  exit 1
fi

# Whitespace-normalised name-and-status pairs. `NF` gates the rebuild rather
# than following it: awk forces NF to 1 when $1 is assigned, so `{$1=$1} NF`
# would resurrect blank and comment-only lines as empty entries.
expected_sorted=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)

# A one-field line is a name with no status. Accepting it would make the entry
# unmatchable in a way that reads like a regression, so name the format — and
# do it HERE, before the suite runs, rather than after ten seconds of work that
# was never going to be gradeable.
malformed=$(printf '%s\n' "$expected_sorted" | awk 'NF == 1 {print}')
if [ -n "$malformed" ]; then
  echo "error: $EXPECTED_FAIL has entries with no status (the format is" >&2
  echo "'<test>.S <STATUS>', e.g. 'csr.S DISAGREE AT 17'):" >&2
  printf '  %s\n' "$malformed" >&2
  exit 1
fi

# THE SUITE'S SHAPE, and deliberately BEFORE the setup probe below. It costs
# milliseconds, needs no Sail, and a suite that does not match its manifest was
# never going to produce a gradeable run — so there is no reason to make it wait
# behind a toolchain check. Both directions; see check_suite_shape.sh.
if ! "$HERE/check_suite_shape.sh" "$ASM_DIR" "$MANIFEST"; then
  echo "error: the .S suite does not match its manifest; nothing was run." >&2
  exit 1
fi
echo

# One setup probe for the whole suite: the cross compiler, the pinned and
# digest-checked sail binary, and the cxxrtl co-sim runner. Without Sail this
# is where the run stops, with the message naming `make sail-setup` — which is
# the whole point of the leg being opt-in.
if ! "$COSIM_PY" --check-setup --cosim-binary "$COSIM_BIN"; then
  echo "error: co-simulation setup is incomplete; nothing was run." >&2
  exit 1
fi
echo

# The emptiness guard that used to be here is subsumed by the manifest check
# above, which rejects an empty glob against a non-empty manifest and names
# every program that went missing rather than only reporting that they all did.
shopt -s nullglob
programs=("$ASM_DIR"/*.S)
shopt -u nullglob

tmp=$(mktemp -d "${TMPDIR:-/tmp}/littlecpu-cosim.XXXXXX") || {
  echo "error: could not create a temporary directory under ${TMPDIR:-/tmp}." >&2
  exit 1
}
if [ -z "$tmp" ] || [ ! -d "$tmp" ]; then
  echo "error: mktemp -d produced no usable directory under ${TMPDIR:-/tmp}." >&2
  exit 1
fi
trap 'rm -rf "$tmp"' EXIT

declare -a failures=()
declare -a table=()
agreed=0

for src in "${programs[@]}"; do
  name=$(basename "$src")
  base=${name%.S}
  log="$tmp/$base.log"

  # Error-exit is lifted for exactly this call — cosim.py's nonzero exits are
  # verdicts, not accidents — and restored immediately.
  set +e
  "$COSIM_PY" --quiet --cosim-binary "$COSIM_BIN" "$name" > "$log" 2>&1
  rc=$?
  set -e

  # The status line is the contract; the exit code is the cross-check. They
  # disagreeing means cosim.py changed under this script, which is a broken
  # harness and must not read as a verdict about the core.
  status=$(awk '/^COSIM-STATUS /{ $1=""; sub(/^ /,""); print; exit }' "$log")
  if [ -z "$status" ]; then
    status="COSIM-ERROR $rc"
  elif { [ "$rc" -eq 0 ] && [ "$status" != "AGREE" ]; } ||
       { [ "$rc" -ne 0 ] && [ "$status" = "AGREE" ]; }; then
    status="COSIM-ERROR status/exit mismatch $rc"
  fi

  if [ "$status" = "AGREE" ]; then
    agreed=$((agreed + 1))
  else
    failures+=("$name $status")
    echo "--- $name ---" >&2
    cat "$log" >&2
  fi
  table+=("$(printf '%-16s %s' "$name" "$status")")
done

printf '%s\n' "${table[@]}"
echo
echo "$agreed/${#table[@]} agreed"

# Normalised the same way $expected_sorted was, up at the top of the script.
actual_sorted=$(printf '%s\n' "${failures[@]:-}" | awk 'NF { $1=$1; print }' | sort)

if [ "$actual_sorted" = "$expected_sorted" ]; then
  echo "Divergence list matches $EXPECTED_FAIL exactly (name and status)."
  exit 0
fi

echo
echo "Divergence list does NOT match $EXPECTED_FAIL:" >&2
diff <(echo "$expected_sorted") <(echo "$actual_sorted") \
  --label expected --label actual >&2 || true
exit 1
