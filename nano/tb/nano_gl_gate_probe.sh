#!/bin/bash
# Ties the fixture's real enabled flop's enable low and requires it to stop toggling.
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
old = "sky130_fd_sc_hd__mux2_1 hold_or_flip (.X(next), .A0(q), .A1(q_n), .S(enable));"
new = "sky130_fd_sc_hd__mux2_1 hold_or_flip (.X(next), .A0(q), .A1(q_n), .S(1'b0));"
if old not in src:
    sys.exit("error: nano_gl_gate_probe_fixture.v no longer spells the enable "
              "connection this probe mutates -- re-anchor it.")
open(sys.argv[2], "w").write(src.replace(old, new, 1))
PYEOF
if cmp -s "$FIXTURE" "$mutant"; then
  echo "error: the mutant fixture is identical to the shipping one." >&2
  exit 2
fi

run() {  # $1 = fixture path, $2 = output vvp path
  python3 "$REPO/nano/gl_census.py" "$1" --includes "$2.cells.v" >/dev/null
  iverilog -g2012 -D FUNCTIONAL '-DUNIT_DELAY=#1' -I "$CELL_DIR" -o "$2" "$2.cells.v" "$1"
  vvp "$2"
}

echo "control: enable driven high"
out=$(run "$FIXTURE" "$WORKDIR/control.vvp")
echo "$out"
if ! grep -q '^PASS' <<<"$out"; then
  echo "*** the shipping fixture does not pass on its own, so a mutant failing the" \
       "same way would prove nothing." >&2
  exit 1
fi

echo
echo "mutant: enable tied to a constant 0"
out=$(run "$mutant" "$WORKDIR/mutant.vvp")
echo "$out"
if ! grep -q '^FAIL' <<<"$out"; then
  echo "*** a flop whose enable is tied low still toggled." >&2
  exit 1
fi

echo
echo "the real cell models catch a stuck enable: the shipping fixture toggles, the mutant does not."
