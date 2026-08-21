#!/usr/bin/env python3
"""Reads a mapped netlist and says whether the ROM's copies share one write.

    rom_replication.py <netlist.json> --cell SB_RAM40_4K --copies 2 --groups 16

`rtl/imemory.v` asks each bank for one read port per fetch window. Neither part
has a primitive with two read ports, so yosys answers by REPLICATING the bank and
driving every copy from the same write -- which is what makes the text one
storage that two harts read rather than two storages that can disagree. That
claim is about the mapped netlist and about nothing in the RTL: at RTL there is
one array and every window reads it by construction, so a simulation of the
source can neither confirm the replication nor fail on its absence. It is checked
here instead.

WHAT IS CHECKED. Group every block RAM cell by the nets on its WRITE port. Two
cells in one group are two copies of the same words, written by the same enable
from the same address with the same data on the same edge -- so nothing can land
in one and not the other. The check is that every group holds exactly as many
cells as there are windows, and that the cells in a group differ on their READ
port, which is what says they are copies serving different windows rather than
one cell counted twice.

The counts are declared by the caller rather than derived, for the reason
soc/cell_census.py gives: a check that computes its own expectation agrees with
whatever the tool did.

THE RED DIRECTION IS test/imem_share_test.sh's, which synthesises a mutant whose
second window reads a private pair of banks and requires this to report it. A
grouping check with nothing to fail on would pass on a design with one window,
one copy and no sharing at all, which is why `--copies 1` is refused.

Exit: 0 the copies share one write, 1 they do not, 2 the netlist is unreadable.
"""

import argparse
import json
import sys

# The write half and the read half of each part's block RAM, as pin-name
# PREFIXES: ice40 carries a bus on one pin and ECP5 spreads the same bus over
# `ADB0` .. `ADB13`, so a fixed list of whole pin names matches nothing on one of
# them -- which puts every cell in one group and reports a sharing that was never
# looked for. A cell type this script has not been told about is refused for the
# same reason.
#
# ECP5's primitive is true dual-port and yosys writes through port A and reads
# through port B, so A is the write half there; ice40's is one read and one write
# already.
PORTS = {
    "SB_RAM40_4K": {
        "write": ("WCLK", "WE", "WADDR", "WDATA", "MASK"),
        "read": ("RCLK", "RE", "RADDR"),
    },
    "DP16KD": {
        "write": ("CLKA", "CEA", "WEA", "ADA", "DIA", "CSA"),
        "read": ("CLKB", "CEB", "ADB", "CSB"),
    },
}


def refuse(message):
    print(f"*** rom_replication: {message}", file=sys.stderr)
    sys.exit(2)


def signature(cell, prefixes):
    pins = {name: nets for name, nets in cell["connections"].items()
            if name.startswith(prefixes)}
    if not pins:
        refuse("no pin on a %s matched %s, so every cell would group together "
               "and the check would pass without comparing anything."
               % (cell["type"], " or ".join(prefixes)))
    return json.dumps(pins, sort_keys=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("netlist", help="a yosys write_json netlist")
    parser.add_argument("--cell", required=True, help="the block RAM cell type")
    parser.add_argument("--copies", type=int, required=True,
                        help="how many fetch windows, so how many copies of each bank")
    parser.add_argument("--groups", type=int, required=True,
                        help="how many distinct write ports the banks come to")
    args = parser.parse_args()

    if args.copies < 2:
        refuse("--copies is %d. One copy shares a write with nothing, so this "
               "check would pass without looking; run it on the configuration "
               "whose replication is the claim." % args.copies)
    if args.cell not in PORTS:
        refuse("no port list for %s. Add one beside the two in this file rather "
               "than grouping on an empty key, which reports every cell as one "
               "shared write." % args.cell)

    try:
        with open(args.netlist) as handle:
            design = json.load(handle)
    except (OSError, ValueError) as exc:
        refuse(f"{args.netlist} could not be read as a netlist: {exc}")

    cells = [c for module in design.get("modules", {}).values()
             for c in module.get("cells", {}).values()
             if c.get("type") == args.cell]

    expected = args.copies * args.groups
    if len(cells) != expected:
        print(f"*** rom_replication: {len(cells)} {args.cell} cells against the "
              f"{expected} declared ({args.groups} banks' worth by {args.copies} "
              f"copies). Either the banks stopped mapping to block RAM or the "
              f"windows stopped asking for a copy each.", file=sys.stderr)
        return 1

    ports = PORTS[args.cell]
    groups = {}
    for cell in cells:
        groups.setdefault(signature(cell, ports["write"]), []).append(cell)

    sizes = sorted(len(g) for g in groups.values())
    if len(groups) != args.groups or set(sizes) != {args.copies}:
        print(f"*** rom_replication: the {len(cells)} {args.cell} cells fall into "
              f"{len(groups)} distinct write ports with sizes "
              f"{sizes}, against {args.groups} groups of "
              f"{args.copies}. A group smaller than {args.copies} is a copy with "
              f"a write of its own, which is a second storage: a store would land "
              f"in one window's text and not the other's.", file=sys.stderr)
        return 1

    for members in groups.values():
        reads = {signature(cell, ports["read"]) for cell in members}
        if len(reads) != len(members):
            print("*** rom_replication: two cells sharing a write also share "
                  "their whole read port, so they are not copies serving "
                  "different windows and the grouping above counted one window "
                  "twice.", file=sys.stderr)
            return 1

    print(f"{len(cells)} {args.cell}: {args.groups} write ports, {args.copies} "
          f"copies each, every copy on the same enable, address, data and edge.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
