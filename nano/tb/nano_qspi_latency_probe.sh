#!/bin/bash
# Reverts the read capture to SCK's rising edge (half the round-trip budget) and requires the
# shipping controller to PASS the one-cycle-latency reproduction first, the mutant to FAIL it.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-qspi-latency-probe"

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

sites = [
    ("ST_FLASH_STREAM", "          if (sio_phase) begin\n            rx_shift[15:0] <= {rx_shift[11:0], sio_in};\n"),
    ("ST_PSRAM_READ",
     "          if (sio_phase) begin\n            rx_shift <= {rx_shift[27:0], sio_in};\n"),
]
out = src
for name, old in sites:
    if old not in out:
        sys.exit(f"error: {name}'s capture-gating text not found -- probe is stale, "
                  "re-anchor it on the current spelling")
    out = out.replace(old, old.replace("if (sio_phase)", "if (!sio_phase)"), 1)

open(sys.argv[2], "w").write(out)
PYEOF
if cmp -s "$REPO/nano/qspi.v" "$mutant"; then
  echo "error: nano/qspi.v no longer spells the compensated capture the way this probe" \
    "mutates. Re-anchor it on the new spelling." >&2
  exit 1
fi

build_and_run() {  # $1 = qspi.v source -> stdout
  local qspi_v=$1
  local vvp="$WORKDIR/$(basename "$qspi_v").vvp"
  iverilog -g2012 -DQSPI_RESUME_TB_DELAY_CYCLES=1 -o "$vvp" "$qspi_v" \
    "$REPO/nano/tb/nano_qspi_flash_model.v" "$REPO/nano/tb/nano_qspi_psram_model.v" \
    "$REPO/nano/tb/nano_qspi_resume_tb.v"
  vvp "$vvp"
}

echo "shipping:"
shipping_out=$(build_and_run "$REPO/nano/qspi.v")
echo "$shipping_out"
if ! grep -q '^PASS$' <<< "$shipping_out"; then
  echo "*** the shipping controller does not pass its own latency test -- the control this" \
    "probe relies on, so a mutant failing the same way would prove nothing." >&2
  exit 1
fi

echo
echo "mutant (capture reverted to SCK's rising edge, half the round-trip budget):"
mutant_out=$(build_and_run "$mutant")
echo "$mutant_out"
if grep -q '^PASS$' <<< "$mutant_out"; then
  echo "*** RED PROBE FAILED: the mutant still passes -- the test cannot tell the" \
    "uncompensated capture phase from the compensated one." >&2
  exit 1
fi

echo
echo "shipping passes, the mutant fails: nano_qspi_resume_tb.v grades the compensation" \
  "at QSPI_RESUME_TB_DELAY_CYCLES=1."
