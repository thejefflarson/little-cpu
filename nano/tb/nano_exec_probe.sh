#!/bin/bash
# The forced-red direction for nano_exec_cxxrtl.cc: reintroduces each divider defect it
# exists to catch into a scratch copy of nano.v and requires a reported mismatch.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-exec-probe.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

run_mutation() {  # <label> <sed-expr>
  local label=$1 expr=$2
  local mutant="$tmp/$label.v"
  cp "$REPO/nano/nano.v" "$mutant"
  sed -i.bak -e "$expr" "$mutant"
  if cmp -s "$mutant" "$mutant.bak"; then
    echo "error: mutation '$label' matches nothing in nano.v -- probe is stale" >&2
    exit 1
  fi
  rm -f "$mutant.bak"

  local out rc
  set +e
  out=$("$HERE/nano_exec_run.sh" "$mutant" 2>&1)
  rc=$?
  set -e
  if [ "$rc" -eq 0 ] || ! grep -q "^MISMATCH" <<< "$out"; then
    echo "RED PROBE FAILED: mutation '$label' did not produce a MISMATCH (exit $rc)" >&2
    printf '%s\n' "$out" | sed -e 's/^/    /' >&2
    return 1
  fi
  echo "ok: mutation '$label' caught -- $(grep -m1 "^MISMATCH" <<< "$out")"
}

status=0
run_mutation "compare-direction" \
  "s/if (mul_div_x >= mul_div_y) begin/if (mul_div_x <= mul_div_y) begin/" || status=1
run_mutation "iteration-count" \
  "s/mul_div_counter <= 32;/mul_div_counter <= 65;/" || status=1
run_mutation "no-magnitude-conversion" \
  "s/mul_div_x <= {32'b0, div_abs_rs1};/mul_div_x <= {32'b0, regs[rs1[3:0]]};/" || status=1

exit $status
