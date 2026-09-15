#!/bin/bash
# The forced-red direction for run_qspi_loop_buffer_test.sh: reintroduces a real loop-buffer
# defect into a scratch copy of nano_qspi_memory.v and requires the reported FAIL.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: run_qspi_loop_buffer_probe.sh <nano-march-cflags>" >&2
  exit 2
fi
CFLAGS=$1
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

tmp=$(mktemp -d "${TMPDIR:-/tmp}/nano-qspi-loop-probe.XXXXXX")
trap 'rm -rf "$tmp"' EXIT

run_mutation() {  # <label> <sed-expr> <expected-text>
  local label=$1 expr=$2 want=$3
  local mutant="$tmp/$label.v"
  cp "$REPO/nano/tb/nano_qspi_memory.v" "$mutant"
  sed -i.bak -e "$expr" "$mutant"
  if cmp -s "$mutant" "$mutant.bak"; then
    echo "error: mutation '$label' matches nothing in nano_qspi_memory.v -- probe is stale" >&2
    exit 1
  fi
  rm -f "$mutant.bak"

  local out rc
  set +e
  out=$("$HERE/run_qspi_loop_buffer_test.sh" "$CFLAGS" "$mutant" 2>&1)
  rc=$?
  set -e
  if [ "$rc" -eq 0 ] || ! grep -q "$want" <<< "$out"; then
    echo "RED PROBE FAILED: mutation '$label' did not produce '$want' (exit $rc)" >&2
    printf '%s\n' "$out" | sed -e 's/^/    /' >&2
    return 1
  fi
  echo "ok: mutation '$label' caught"
}

status=0
run_mutation "loop-hit-gated-on-xfer-active" \
  "s/loop_hit_now ? 1'b1 :/loop_hit_now ? xfer_active :/" \
  "near-execute-time bound" || status=1
run_mutation "cam-lookup-never-hits" \
  "s/if (cam_valid\[i\] && cam_idx\[i\] == target_index) cam_has0 = 1'b1;/if (1'b0) cam_has0 = 1'b1;/" \
  "still pays a marginal preamble/wait cost" || status=1

exit $status
