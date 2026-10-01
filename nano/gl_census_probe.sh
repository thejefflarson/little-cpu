#!/bin/bash
# Requires gl_census.py to accept a netlist of sky130 cells, refuse the RTL it came from, and
# refuse a netlist with zero dlclkp cells when one is required.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
WORKDIR="$HERE/gl-census-probe"

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

cat > "$WORKDIR/netlist.v" <<'VEOF'
module m;
  sky130_fd_sc_hd__mux2_1 mx (.A0(a), .A1(b), .S(e), .X(x));
  sky130_fd_sc_hd__dfxtp_1 ff (.D(x), .Q(q), .CLK(c));
endmodule
VEOF

cat > "$WORKDIR/rtl.v" <<'VEOF'
module m(input c, e, a, b, output reg q);
  always @(posedge c) q <= e ? b : a;
endmodule
VEOF

echo "control: a netlist of sky130 cells"
if ! out=$(python3 "$HERE/gl_census.py" "$WORKDIR/netlist.v" --includes "$WORKDIR/cells.v" 2>&1); then
  echo "$out"
  echo "*** a netlist of sky130 cells was refused; the control itself is broken." >&2
  exit 1
fi
echo "$out"
if [ "$(wc -l < "$WORKDIR/cells.v")" -ne 2 ]; then
  echo "*** the control wrote $(wc -l < "$WORKDIR/cells.v") includes for its two cell types." >&2
  exit 1
fi

echo
echo "mutant: the RTL that netlist came from"
if out=$(python3 "$HERE/gl_census.py" "$WORKDIR/rtl.v" 2>&1); then
  echo "$out"
  echo "*** RTL with no sky130 cells was accepted as a netlist." >&2
  exit 1
fi
echo "$out"
if ! printf '%s\n' "$out" | grep -q "no sky130_fd_sc_hd cell instantiations"; then
  echo "*** the mutant was refused, but not for the reason this probe expects." >&2
  exit 1
fi

cat > "$WORKDIR/gated.v" <<'VEOF'
module m;
  sky130_fd_sc_hd__dlclkp_1 icg (.GCLK(g), .GATE(e), .CLK(c));
  sky130_fd_sc_hd__dfxtp_1 ff (.D(d), .Q(q), .CLK(g));
endmodule
VEOF

echo
echo "control: a netlist with a dlclkp cell, which requires one"
if ! out=$(python3 "$HERE/gl_census.py" "$WORKDIR/gated.v" --require dlclkp 2>&1); then
  echo "$out"
  echo "*** a netlist that does contain dlclkp was refused; the control itself is broken." >&2
  exit 1
fi
echo "$out"

echo
echo "mutant: the same enabled register, with no clock gate"
if out=$(python3 "$HERE/gl_census.py" "$WORKDIR/netlist.v" --require dlclkp 2>&1); then
  echo "$out"
  echo "*** a netlist with zero dlclkp cells was accepted." >&2
  exit 1
fi
echo "$out"
if ! printf '%s\n' "$out" | grep -q "found zero of: \['dlclkp'\]"; then
  echo "*** the mutant was refused, but not for the reason this probe expects." >&2
  exit 1
fi

echo
echo "gl_census.py accepts a netlist of sky130 cells, refuses RTL, and refuses a netlist with no clock gate."
