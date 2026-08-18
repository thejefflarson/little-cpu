#!/usr/bin/env python3
"""Attribute a critical path's logic levels to the RTL that built them.

WHY THIS EXISTS. `soc/timing_split.py` names the two ENDS of the path and counts
the levels between them, and that was enough to say the fetch loop starts and
ends in the instruction memory. It is not enough to say where the levels in the
middle sit, and the report's own names cannot be read for that: yosys names a
generated cell after a neighbouring net, so `imem.in_range2_SB_LUT4_I3_O_...` is
ancestry in whichever direction the namer happened to walk, and it turns up on
levels that have nothing to do with `in_range2`. Reading those prefixes as a
per-stage split gives an answer that looks like a measurement and is not one.

WHAT IS DERIVABLE, EXACTLY. Registers and memories keep their RTL names through
synthesis even though the combinational wires between them do not. So for each
level on the path, walk back through the LUTs feeding its SIDE inputs -- every
input except the one the path itself arrives on -- until the walk reaches a
register or a memory, and collect what it reaches. That set is what the level
folds into the path, it is named in rtl/, and none of it is inferred from a
generated name.

The row prints only what is NEW at that level: a source already folded in
upstream is not this level's work. A level with nothing new is doing width or
fan-in reduction on what the path already carries, and is charged to `-`.

Usage: path_stages.py <icetime .rpt> <synth .json> [--top littlesoc]
"""

import argparse
import collections
import json
import re
import sys

# Net names as icetime prints them between hops, and the endpoint line.
NET = re.compile(r"^\s+[0-9.]+ ns \S+ \((.+?)(?:\[(\d+)\])?\)\s*$")
ENDPOINT = re.compile(r"^\s+\S+ -> (\S+?)(?:\[(\d+)\])?\s*$")
HOP = re.compile(r"^\s+(\S+) \((\w+)\)([^:]*): ([0-9.]+) ns")

# The output ports of everything the ice40 flow leaves in a netlist.
DRIVER_PORTS = ("O", "CO", "Q", "RDATA", "DATAOUT", "O0", "O1")
# Where the backward walk stops: the state the path's levels read.
SEQUENTIAL = ("SB_DFF", "SB_RAM40_4K", "SB_SPRAM256KA", "SB_MAC16", "ICESTORM_")
# Clocks and resets reach everything and say nothing about a level.
UNINTERESTING = ("clk", "reset", "CLK", "RESET")

# Fetch-loop order, and the tie-break when a level folds in two modules at once.
# `imem` first would charge decode's work to the memory, because every level on
# this path has the memory's data somewhere behind it.
# `riscv.pc` and `riscv.csr_*` are rtl/littlecpu.v's wiring between two of the
# modules below, so they are charged to the module that drives them rather than
# to a bucket named after the file the wire is declared in.
MODULES = [
    ("riscv.decoder.", "decode"),
    ("riscv.pc", "decode"),
    ("riscv.csr_", "csrs"),
    ("riscv.csrs.", "csrs"),
    ("riscv.accessor", "access"),
    ("riscv.executor", "execute"),
    ("riscv.writeback", "writeback"),
    ("riscv.regs", "regfile"),
    ("riscv.", "core"),
    ("imem.", "imem"),
    ("dmem.", "dmem"),
    ("mtimer.", "timer"),
]
STAGE_ORDER = [stage for _, stage in MODULES] + ["top"]


def stage_of(name):
    for prefix, stage in MODULES:
        if name.startswith(prefix):
            return stage
    return "top"


def load(path, top):
    design = json.load(open(path))
    if top not in design["modules"]:
        sys.exit(f"{path}: no module {top!r}; it has {sorted(design['modules'])[:5]}...")
    module = design["modules"][top]

    driver = {}
    for cell_name, cell in module["cells"].items():
        for port, bits in cell["connections"].items():
            if port in DRIVER_PORTS:
                for bit in bits:
                    if isinstance(bit, int):
                        driver[bit] = cell_name

    # A bit's best RTL name: the shortest one, which is the declared signal
    # rather than an alias yosys hung off it.
    names = {}
    bits_of = {}
    for net_name, net in module["netnames"].items():
        bits_of[net_name] = net["bits"]
        if "$" in net_name or "_SB_" in net_name:
            continue
        for index, bit in enumerate(net["bits"]):
            if not isinstance(bit, int):
                continue
            label = f"{net_name}[{index}]" if len(net["bits"]) > 1 else net_name
            if bit not in names or len(label) < len(names[bit]):
                names[bit] = label
    return {"cells": module["cells"], "driver": driver, "names": names,
            "bits": bits_of, "memo": {}}


def inputs_of(cell):
    """The bits a cell reads, less the clock and reset that reach everything."""
    for port, bits in cell["connections"].items():
        if port in DRIVER_PORTS or port in UNINTERESTING:
            continue
        for bit in bits:
            if isinstance(bit, int):
                yield bit


def driver_of(nl, bit):
    cell_name = nl["driver"].get(bit)
    return nl["cells"][cell_name] if cell_name is not None else None


def sources_of(nl, bit):
    """The named registers and memories reachable backwards from one bit."""
    memo = nl["memo"]
    if bit in memo:
        return memo[bit]
    memo[bit] = frozenset()  # break combinational loops rather than recurse forever
    cell = driver_of(nl, bit)
    if cell is None:
        found = frozenset()
    elif cell["type"].startswith(SEQUENTIAL):
        label = nl["names"].get(bit, nl["driver"][bit])
        found = frozenset([label.split("$")[0] or nl["driver"][bit]])
    else:
        found = frozenset().union(*(sources_of(nl, b) for b in inputs_of(cell)),
                                  frozenset())
    memo[bit] = found
    return found


def path_hops(report):
    """Every hop icetime printed, in order, each with the net it produced.

    `path_nets` below keeps only the hops that end a level, which is what an
    attribution of LEVELS needs. A reader that bins the interconnect needs the
    hops in between as well -- there are five or six of them between two levels
    -- so this walk is the one that keeps them, and `path_nets` is derived from
    it rather than walking the file a second time.

    `net` is the resolvable name icetime printed after the hop, or None where
    the hop is inside a level. A net printed with no hop before it gets an entry
    of its own with no cell in it, so a hop never collects two nets.
    """
    hops = []
    for line in open(report):
        hop = HOP.match(line)
        if hop:
            hops.append({"cell": hop.group(1), "kind": hop.group(2),
                         "what": hop.group(3).strip(), "delay": float(hop.group(4)),
                         "net": None})
            continue
        for pattern in (NET, ENDPOINT):
            match = pattern.match(line)
            if match:
                if not hops or hops[-1]["net"] is not None:
                    hops.append({"cell": None, "kind": None, "what": "",
                                 "delay": 0.0, "net": None})
                hops[-1]["net"] = (match.group(1), match.group(2))
                break
    return hops


def path_nets(report):
    return [(hop["net"][0], hop["net"][1],
             (hop["kind"], hop["what"]) if hop["kind"] else None)
            for hop in path_hops(report) if hop["net"]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report")
    parser.add_argument("netlist")
    parser.add_argument("--top", default="littlesoc")
    args = parser.parse_args()
    sys.setrecursionlimit(100000)

    nl = load(args.netlist, args.top)
    nets = path_nets(args.report)
    if not nets:
        sys.exit(f"{args.report}: no path nets found. That is a failed read, not a flat path.")

    def bit_of(net_name, index):
        bits = nl["bits"].get(net_name)
        if bits is None:
            return None
        bit = bits[int(index)] if index is not None else bits[0]
        return bit if isinstance(bit, int) else None

    on_path = {b for b in (bit_of(n, i) for n, i, _ in nets) if b is not None}

    seen = set()
    charged = collections.Counter()
    print(f"{'lvl':>3}  {'hop':<7} {'stage':<10} state this level folds in, that the path did not carry")
    for level, (net_name, index, hop) in enumerate(nets):
        kind, what = hop if hop else ("start", "")
        cell = driver_of(nl, bit_of(net_name, index))
        folded = set()
        if cell is not None and not cell["type"].startswith(SEQUENTIAL):
            for bit in inputs_of(cell):
                if bit not in on_path:
                    folded |= sources_of(nl, bit)
        new = sorted(folded - seen)
        seen |= folded
        stages = sorted({stage_of(s) for s in new}, key=STAGE_ORDER.index)
        if kind == "LogicCell40":
            charged[stages[0] if stages else "-"] += 1

        tag = "carry" if what == "carryin -> carryout" else ("LUT" if kind == "LogicCell40" else kind)
        shown = ", ".join(new[:3]) if new else "-"
        if len(new) > 3:
            shown += f", +{len(new) - 3} more"
        print(f"{level:>3}  {tag:<7} {'/'.join(stages) or '-':<10} {shown}")

    print()
    print("logic levels charged to each stage:")
    for stage in sorted(charged, key=lambda s: -charged[s]):
        print(f"  {stage:<10} {charged[stage]}")
    print(f"  {'total':<10} {sum(charged.values())}")


if __name__ == "__main__":
    main()
