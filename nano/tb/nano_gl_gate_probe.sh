#!/bin/bash
# Forces the fixture's real dlclkp_1 to a stuck-low GATE and requires its counter to stop.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WORKDIR="$HERE/nano-gl-gate-probe"
CELL_DIR=$1
FIXTURE="$HERE/nano_gl_gate_probe_fixture.v"

if ! command -v iverilog >/dev/null 2>&1; then
  echo "error: iverilog is not on PATH." >&2
  exit 2
fi
if [ ! -d "$CELL_DIR" ] || [ -z "$(ls -A "$CELL_DIR" 2>/dev/null)" ]; then
  echo "error: $CELL_DIR has no cell models -- run 'make nano-sky130-verilog-setup' first." >&2
  exit 2
fi

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

mutant="$WORKDIR/fixture.mutant.v"
python3 - "$FIXTURE" "$mutant" <<'PYEOF'
import sys
src = open(sys.argv[1]).read()
old = "sky130_fd_sc_hd__dlclkp_1 icg (.GCLK(gclk), .GATE(gate), .CLK(clk));"
new = "sky130_fd_sc_hd__dlclkp_1 icg (.GCLK(gclk), .GATE(1'b0), .CLK(clk));"
if old not in src:
    sys.exit("error: nano_gl_gate_probe_fixture.v no longer spells the gate "
              "connection this probe mutates -- re-anchor it.")
open(sys.argv[2], "w").write(src.replace(old, new, 1))
PYEOF
if cmp -s "$FIXTURE" "$mutant"; then
  echo "error: the mutant fixture is identical to the shipping one." >&2
  exit 2
fi

run() {  # $1 = fixture path, $2 = output vvp path
  iverilog -g2012 -D FUNCTIONAL -D UNIT_DELAY= -I "$CELL_DIR" -y "$CELL_DIR" -Y .v -o "$2" "$1"
  vvp "$2"
}

echo "control: GATE driven high"
out=$(run "$FIXTURE" "$WORKDIR/control.vvp")
echo "$out"
if ! printf '%s\n' "$out" | grep -q '^PASS'; then
  echo "*** the shipping fixture does not pass on its own, so a mutant failing the" \
       "same way would prove nothing." >&2
  exit 1
fi

echo
echo "mutant: GATE forced to a constant 0"
out=$(run "$mutant" "$WORKDIR/mutant.vvp")
echo "$out"
if ! printf '%s\n' "$out" | grep -q '^FAIL'; then
  echo "*** a dlclkp cell with GATE stuck low still let its counter advance." >&2
  exit 1
fi

echo
echo "the real dlclkp_1 model catches a stuck-low GATE: the shipping fixture counts, the" \
     "mutant does not."
