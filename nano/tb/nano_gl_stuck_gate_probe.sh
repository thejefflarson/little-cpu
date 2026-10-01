#!/bin/bash
# Ties every register-file clock gate's GATE low in a copy of the netlist and requires the
# gate-level test to fail: registers that never load cannot run the program.
set -euo pipefail

CFLAGS=$1
NETLIST=$2
CELL_DIR=$3

HERE=$(cd "$(dirname "$0")" && pwd)
WORKDIR="$HERE/nano-gl-stuck-gate-probe"

rm -rf "$WORKDIR"
mkdir -p "$WORKDIR"
mutant="$WORKDIR/stuck.nl.v"

python3 - "$NETLIST" "$mutant" <<'PYEOF'
import re
import sys

text = open(sys.argv[1]).read()
mutated, n = re.subn(r"\.GATE\(\\core\.g_regs\[\d+\]\.r\.enable \)", ".GATE(1'b0)", text)
if n != 15:
    sys.exit("error: expected 15 register-file clock gates in the netlist, tied %d." % n)
open(sys.argv[2], "w").write(mutated)
PYEOF

echo "mutant: the 15 register-file clock gates' GATE tied low"
if out=$("$HERE/run_nano_gl_test.sh" "$CFLAGS" "$mutant" "$CELL_DIR" 2>&1); then
  echo "$out" | tail -5
  echo "*** a netlist whose register file can never load passed the gate-level test." >&2
  exit 1
fi
if ! printf '%s\n' "$out" | grep -q -e '^FAIL' -e 'TIMEOUT'; then
  echo "$out" | tail -8
  echo "*** the mutant failed, but not by the test's own verdict." >&2
  exit 1
fi
printf '%s\n' "$out" | grep -e '^FAIL' -e 'TIMEOUT' | head -3
echo
echo "a register file whose clock gates never open fails the gate-level test."
