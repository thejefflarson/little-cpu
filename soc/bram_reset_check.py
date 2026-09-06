#!/usr/bin/env python3
"""Refuse an ECP5 netlist that reads a block RAM through the block's own reset.

THE PART DOES NOT DO WHAT THE NETLIST SAYS HERE. Given a synchronous read whose
out-of-range arm is a constant -- `mem_rdata <= in_range ? ram[index] : 32'b0`,
which is how rtl/memory.v spells a data RAM that must drive zero onto the SoC's
wired-OR read bus -- yosys maps the zero arm onto DP16KD's output reset and
drives RSTA from logic. On silicon that read returns zero whatever the array
holds, so a program's stores land and read back as nothing. It is invisible
everywhere else: the design is correct in RTL simulation, the cell censuses
count the same 36 DP16KD either way, and nextpnr places and times it happily.
The ROM never trips it because a read-only memory needs no reset at all.

So the rule is structural and absolute: no block RAM's reset may be driven by
logic. Spell the out-of-range arm as a mux on the block's output instead, where
it costs LUTs that behave the way the source says. Yosys ships no simulation
model for DP16KD -- the module in its cells_sim.v is a port list with no body --
so no simulation of the mapped netlist can catch this, on any machine, and a
structural refusal is the only check that runs without the board plugged in.

Reads the mapped JSON `synth_ecp5 -json` writes, not a log: the count of block
RAMs is a census question and this is a connectivity one.
"""

import argparse
import json
import sys

# The two block RAM primitives synth_ecp5 infers. TRELLIS_DPR16X4 is LUT RAM,
# built out of logic that does honour a reset, and is deliberately not here.
BRAM_CELLS = ("DP16KD", "PDPW16KD")
RESET_PORTS = ("RSTA", "RSTB", "RST")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("netlist", help="mapped JSON, e.g. ecp5.json")
    parser.add_argument(
        "--gate",
        default="make ecp5-timing",
        help="the target this check runs under, so a failure names the flow "
        "that stopped rather than the other part's. Diagnostic only.",
    )
    args = parser.parse_args()

    try:
        with open(args.netlist) as handle:
            design = json.load(handle)
    except (OSError, ValueError) as error:
        print("*** %s: cannot read %s: %s" % (args.gate, args.netlist, error))
        return 2

    total = 0
    offenders = []
    for module_name, module in design.get("modules", {}).items():
        # Every ECP5 primitive appears here as an empty blackbox declaration
        # alongside the real design, so a module with no cells is skipped
        # rather than counted as a design that instantiates nothing.
        for cell_name, cell in module.get("cells", {}).items():
            if cell.get("type") not in BRAM_CELLS:
                continue
            total += 1
            for port in RESET_PORTS:
                bits = cell.get("connections", {}).get(port, [])
                # yosys writes a constant bit as the string "0"/"1"/"x"/"z" and
                # a net as an integer, so "driven by logic" is "any int".
                if any(isinstance(bit, int) for bit in bits):
                    offenders.append((module_name, cell_name, port))
                    break

    if total == 0:
        print("*** %s: %s instantiates no %s."
              % (args.gate, args.netlist, " or ".join(BRAM_CELLS)))
        print("*** This check has nothing to say about a design with no block")
        print("*** RAM in it, which means the synthesis did not finish or the")
        print("*** memories fell out to logic. Read the cell census above.")
        return 2

    if offenders:
        print("*** %s: %d of %d block RAMs read through the block's own reset."
              % (args.gate, len(offenders), total))
        print("***")
        for module_name, cell_name, port in offenders[:8]:
            print("***   %s.%s  %s is driven by logic" % (module_name, cell_name, port))
        if len(offenders) > 8:
            print("***   ... and %d more" % (len(offenders) - 8))
        print("***")
        print("*** On the part that read returns zero no matter what the array")
        print("*** holds. It is a store that lands and reads back as nothing,")
        print("*** and RTL simulation, the cell census and nextpnr all pass.")
        print("*** Spell the memory's out-of-range or reset arm as a mux on the")
        print("*** block's OUTPUT -- see rtl/memory.v -- so it maps to LUTs.")
        return 1

    print("block RAM resets: %d %s, none driven by logic"
          % (total, " / ".join(BRAM_CELLS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
