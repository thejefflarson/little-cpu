#!/bin/bash
# Requires gl_census.py to accept a netlist of sky130 cells and refuse the RTL it came from,
# to require the register-file macro exactly once, and to refuse any other module in a netlist.
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

# The register-file macro: a netlist carries it exactly once, and the model the gate-level
# run reads for it is the module nano.v defines, extracted rather than copied.
cat > "$WORKDIR/macro_netlist.v" <<'VEOF'
module m;
  sky130_fd_sc_hd__dfxtp_1 ff (.D(x), .Q(q), .CLK(c));
  rf_top \core.regfile (.clk(c), .ra_addr(a), .ra_data(d));
endmodule
VEOF

cat > "$WORKDIR/no_macro_netlist.v" <<'VEOF'
module m;
  sky130_fd_sc_hd__dfxtp_1 ff (.D(x), .Q(q), .CLK(c));
endmodule
VEOF

cat > "$WORKDIR/two_macros_netlist.v" <<'VEOF'
module m;
  sky130_fd_sc_hd__dfxtp_1 ff (.D(x), .Q(q), .CLK(c));
  rf_top \core.regfile (.clk(c));
  rf_top \core.regfile2 (.clk(c));
endmodule
VEOF

cat > "$WORKDIR/other_module_netlist.v" <<'VEOF'
module m;
  sky130_fd_sc_hd__dfxtp_1 ff (.D(x), .Q(q), .CLK(c));
  rf_top \core.regfile (.clk(c));
  rf_other \core.other (.clk(c));
endmodule
VEOF

macro_args=(--macro rf_top --macro-source "$HERE/nano.v" --macro-model "$WORKDIR/rf_top_model.v")

echo
echo "control: a netlist with the macro once, and its model extracted from nano.v"
if ! out=$(python3 "$HERE/gl_census.py" "$WORKDIR/macro_netlist.v" "${macro_args[@]}" 2>&1); then
  echo "$out"
  echo "*** a netlist with the register-file macro once was refused; the control is broken." >&2
  exit 1
fi
echo "$out"
if ! grep -q '^module rf_top' "$WORKDIR/rf_top_model.v" ||
   ! grep -q '^endmodule' "$WORKDIR/rf_top_model.v" ||
   grep -q 'module riscv' "$WORKDIR/rf_top_model.v"; then
  echo "*** the extracted model is not exactly rf_top's module definition." >&2
  exit 1
fi

macro_mutant() {  # $1 = label, $2 = netlist, $3 = the reason the census must give, rest = arguments
  local label=$1 netlist=$2 reason=$3
  shift 3
  echo
  echo "mutant: $label"
  if out=$(python3 "$HERE/gl_census.py" "$netlist" "$@" 2>&1); then
    echo "$out"
    echo "*** $label was accepted." >&2
    exit 1
  fi
  echo "$out"
  if ! printf '%s\n' "$out" | grep -q -- "$reason"; then
    echo "*** $label was refused, but not for the reason this probe expects." >&2
    exit 1
  fi
}

macro_mutant "a netlist missing the macro" "$WORKDIR/no_macro_netlist.v" \
  "instantiated 0 times" "${macro_args[@]}"
macro_mutant "a netlist with the macro twice" "$WORKDIR/two_macros_netlist.v" \
  "instantiated 2 times" "${macro_args[@]}"
macro_mutant "a netlist with a module nothing names" "$WORKDIR/other_module_netlist.v" \
  "unexpected instantiated module rf_other" "${macro_args[@]}"
macro_mutant "a netlist with the macro, run without naming it" "$WORKDIR/macro_netlist.v" \
  "unexpected instantiated module rf_top"
macro_mutant "a macro source that does not define the module" "$WORKDIR/macro_netlist.v" \
  "defines no module rf_top" --macro rf_top --macro-source "$WORKDIR/rtl.v" \
  --macro-model "$WORKDIR/never.v"

echo
echo "gl_census.py accepts a netlist of sky130 cells, requires the register-file macro exactly once, and refuses RTL and any other module."
