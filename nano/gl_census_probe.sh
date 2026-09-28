#!/bin/bash
# Requires gl_census.py to refuse a netlist with zero dlclkp cells and accept one with.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
WORKDIR="$HERE/gl-census-probe"

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"

cat > "$WORKDIR/gated.v" <<'EOF'
module m;
  sky130_fd_sc_hd__dlclkp_1 icg (.GCLK(g), .GATE(e), .CLK(c));
  sky130_fd_sc_hd__dfxtp_1 ff (.D(d), .Q(q), .CLK(g));
endmodule
EOF

cat > "$WORKDIR/ungated.v" <<'EOF'
module m;
  sky130_fd_sc_hd__mux2_1 mx (.A0(a), .A1(b), .S(e), .X(x));
  sky130_fd_sc_hd__dfxtp_1 ff (.D(x), .Q(q), .CLK(c));
endmodule
EOF

echo "control: a netlist with a dlclkp cell"
if ! out=$(python3 "$HERE/gl_census.py" "$WORKDIR/gated.v" --require dlclkp 2>&1); then
  echo "$out"
  echo "*** a netlist that does contain dlclkp was refused; the control itself is broken." >&2
  exit 1
fi
echo "$out"

echo
echo "mutant: the same enabled register, mapped without the clockgate pass"
if out=$(python3 "$HERE/gl_census.py" "$WORKDIR/ungated.v" --require dlclkp 2>&1); then
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
echo "gl_census.py accepts a gated netlist and refuses one with zero dlclkp cells."
