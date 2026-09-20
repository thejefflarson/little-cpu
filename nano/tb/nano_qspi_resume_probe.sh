#!/bin/bash
# Neuters every comparison in a scratch copy of nano_qspi_resume_tb.v and requires that
# copy to PASS against the same controller: proves today's FAIL is the comparisons.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-qspi-resume-probe"

if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

neutered="$WORKDIR/nano_qspi_resume_tb.neutered.v"
sed -E "s/if \(rd(\[15:0\])? !== [0-9]+'h[0-9a-fA-F]+\) begin/if (1'b0) begin/" \
  "$REPO/nano/tb/nano_qspi_resume_tb.v" > "$neutered"
if cmp -s "$REPO/nano/tb/nano_qspi_resume_tb.v" "$neutered"; then
  echo "error: no comparison in nano_qspi_resume_tb.v matched the neutering pattern --" \
    "probe is stale against the current test." >&2
  exit 1
fi
if grep -q "!== " "$neutered"; then
  echo "error: a comparison survived neutering -- probe's sed pattern needs updating." >&2
  exit 1
fi

build_and_run() {  # $1 = testbench source -> stdout
  local tb=$1
  local vvp="$WORKDIR/$(basename "$tb").vvp"
  iverilog -g2012 -o "$vvp" "$REPO/nano/qspi.v" "$REPO/nano/tb/nano_qspi_flash_model.v" \
    "$REPO/nano/tb/nano_qspi_psram_model.v" "$tb"
  vvp "$vvp"
}

echo "shipping (real comparisons, against the still-buggy controller):"
shipping_out=$(build_and_run "$REPO/nano/tb/nano_qspi_resume_tb.v")
echo "$shipping_out"

echo
echo "neutered (every comparison replaced by 1'b0, same controller):"
neutered_out=$(build_and_run "$neutered")
echo "$neutered_out"

if ! grep -q '^PASS$' <<< "$neutered_out"; then
  echo "*** RED PROBE FAILED: neutering every comparison did not produce a PASS -- the" \
    "test can fail for a reason other than a value mismatch, which this probe cannot" \
    "distinguish from the real one." >&2
  exit 1
fi

echo
echo "neutering every comparison flips the result on the same controller: the FAIL above" \
  "is the comparisons, not a crash or a timeout."
