#!/bin/bash
# Ties every clock gate's GATE low in a copy of the netlist, found by cell type and not by name, and
# requires the gate-level test to fail: registers that never load cannot run the program.
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
# Every clock-gate instance, whatever the flow named it: tie its GATE low. An escaped identifier
# (`\name `) can hold any character but whitespace, so it is matched as a unit.
instance = re.compile(r"(sky130_fd_sc_hd__dlclkp_\d+\s+(?:\\\S+\s+|\S+\s*)\()(.*?)(\)\s*;)", re.S)
gate = re.compile(r"\.GATE\s*\((?:\\\S+\s|[^)])*\)")
count = 0

def tie(m):
    global count
    body, n = gate.subn(".GATE(1'b0)", m.group(2))
    count += n
    return m.group(1) + body + m.group(3)

mutated = instance.sub(tie, text)
census = len(re.findall(r"sky130_fd_sc_hd__dlclkp_\d+\s", text))
if count == 0 or count != census:
    sys.exit("error: %d clock gates in the netlist, tied %d." % (census, count))
open(sys.argv[2], "w").write(mutated)
PYEOF

echo "mutant: every clock gate's GATE tied low"
if out=$("$HERE/run_nano_gl_test.sh" "$CFLAGS" "$mutant" "$CELL_DIR" 2>&1); then
  echo "$out" | tail -5
  echo "*** a netlist whose clock gates never open passed the gate-level test." >&2
  exit 1
fi
if ! printf '%s\n' "$out" | grep -q -e '^FAIL' -e 'TIMEOUT'; then
  echo "$out" | tail -8
  echo "*** the mutant failed, but not by the test's own verdict." >&2
  exit 1
fi
printf '%s\n' "$out" | grep -e '^FAIL' -e 'TIMEOUT' | head -3
echo
echo "a design whose clock gates never open fails the gate-level test."
