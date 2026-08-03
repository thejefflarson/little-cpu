#!/bin/bash
# Runs every test/asm/*.S under Sail co-simulation (test/cosim.py) and grades
# the table against test/COSIM_EXPECTED_FAIL, under the same contract
# test/run_tests.sh applies to test/EXPECTED_FAIL. Invoked by `make cosim-suite`.
#
# Usage: run_cosim.sh <cosim-binary> <asm-dir> <expected-fail-file> <manifest>
#
# Deliberately NOT on `make test`'s path and not in CI's required set: it needs
# a Sail install that `make test` must keep working without (ADR-0032). It is
# the only oracle here that reads the real register array rather than the core's
# RVFI self-report, so a change to rtl/regfile.v gates on it by carrying its
# output in the pull request.
#
# The guards below are run_tests.sh's, against the same false green, and are
# probed from test/probe_gates.sh.
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

# `NF` must gate the rebuild rather than follow it: awk forces NF to 1 when $1
# is assigned, so `{$1=$1} NF` would resurrect blank and comment-only lines as
# empty entries. Same idiom, same reason, as test/run_tests.sh.
expected_sorted=$(sed -e 's/#.*//' "$EXPECTED_FAIL" | awk 'NF { $1=$1; print }' | sort)

malformed=$(printf '%s\n' "$expected_sorted" | awk 'NF == 1 {print}')
if [ -n "$malformed" ]; then
  echo "error: $EXPECTED_FAIL has entries with no status (the format is" >&2
  echo "'<test>.S <STATUS>', e.g. 'csr.S DISAGREE AT 17'):" >&2
  printf '  %s\n' "$malformed" >&2
  exit 1
fi

# Against a manifest, not merely for emptiness: an emptiness guard catches a
# suite of size zero and does nothing about one that shrank to a dozen programs,
# which prints "12/12 agreed", matches an empty baseline exactly and exits 0.
if ! "$HERE/check_suite_shape.sh" "$ASM_DIR" "$MANIFEST"; then
  echo "error: the .S suite does not match its manifest; nothing was run." >&2
  exit 1
fi
echo

if ! "$COSIM_PY" --check-setup --cosim-binary "$COSIM_BIN"; then
  echo "error: co-simulation setup is incomplete; nothing was run." >&2
  exit 1
fi
echo

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

  # Lifted for exactly this call: cosim.py's nonzero exits are verdicts.
  set +e
  "$COSIM_PY" --quiet --cosim-binary "$COSIM_BIN" "$name" > "$log" 2>&1
  rc=$?
  set -e

  # The status line is the contract and the exit code is the cross-check. Them
  # disagreeing is a broken harness, and must not read as a verdict about the
  # core.
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
