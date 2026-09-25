#!/bin/bash
# Forces nano_qspi_resume_tb.v to catch a broken resume: shrinks the second-parcel
# resume's own nibble count by one, requiring the shipping test to PASS first (the
# control) and the mutant to FAIL.
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

mutant="$WORKDIR/qspi.mutant.v"
python3 - "$REPO/nano/qspi.v" "$mutant" <<'PYEOF'
import sys
src = open(sys.argv[1]).read()
old = ("            end else if (fetch_needs_second_parcel) begin\n"
       "              // The first parcel is in hand and the second is owed, with nowhere"
       " to cache\n"
       "              // it ahead of time: stream it and complete this transaction"
       " directly.\n"
       "              second_parcel_pending    <= 1'b1;\n"
       "              second_parcel_first_data <= slot0_data;\n"
       "              slot0_valid <= 1'b0;\n"
       "              active_dev <= DEV_FLASH;\n"
       "              sck_run    <= 1'b1;\n"
       "              sio_phase  <= 1'b0;\n"
       "              nibbles_left <= 4'd4;\n")
new = old.replace("nibbles_left <= 4'd4;", "nibbles_left <= 4'd3;")
if old not in src:
    sys.exit("error: resume branch text not found -- probe is stale")
open(sys.argv[2], "w").write(src.replace(old, new, 1))
PYEOF
if cmp -s "$REPO/nano/qspi.v" "$mutant"; then
  echo "error: nano/qspi.v no longer spells the resume's nibble count the way this probe" \
    "mutates. Re-anchor the sed range on the new spelling." >&2
  exit 1
fi

build_and_run() {  # $1 = qspi.v source -> stdout
  local qspi_v=$1
  local vvp="$WORKDIR/$(basename "$qspi_v").vvp"
  iverilog -g2012 -o "$vvp" "$qspi_v" "$REPO/nano/tb/nano_qspi_flash_model.v" \
    "$REPO/nano/tb/nano_qspi_psram_model.v" "$REPO/nano/tb/nano_qspi_resume_tb.v"
  vvp "$vvp"
}

echo "shipping:"
shipping_out=$(build_and_run "$REPO/nano/qspi.v")
echo "$shipping_out"
if ! grep -q '^PASS$' <<< "$shipping_out"; then
  echo "*** the shipping controller does not pass its own resume test -- the control this" \
    "probe relies on, so a mutant failing the same way would prove nothing." >&2
  exit 1
fi

echo
echo "mutant (resume's own nibble count off by one):"
mutant_out=$(build_and_run "$mutant")
echo "$mutant_out"
if grep -q '^PASS$' <<< "$mutant_out"; then
  echo "*** RED PROBE FAILED: the mutant still passes -- the test cannot tell a shrunk" \
    "resume count from a correct one." >&2
  exit 1
fi

echo
echo "shipping passes, the mutant fails: nano_qspi_resume_tb.v is a real grader."
