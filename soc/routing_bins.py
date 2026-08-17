#!/usr/bin/env python3
"""Bin one sweep's critical-path routing by what sits at the two ends of each hop.

WHY THIS EXISTS. Routing is about seven tenths of this SoC's critical path and
nothing here attributed any of it. `soc/depth/path_stages.py` attributes logic
LEVELS to the RTL that built them and stops there by design; `soc/timing_split.py`
totals the interconnect by cell class, which says how much there is and not where
it goes. So "the long hops are the block RAM columns nextpnr re-chooses every
seed" was a sentence nobody could grade.

WHAT A HOP IS BINNED BY. A run of `Odrv4`/`Span4Mux`/`LocalMux`/`InMux` hops
carries ONE net from the cell that drives it to the cell that reads it, and
icetime names that net above the run. So the two ends are derivable: the driver
is whatever drives the carried net in the synthesis JSON, and the sink is
whatever drives the next DIFFERENT net on the path -- the cell this run feeds.
Both come out of the netlist the way `path_stages.py` resolves them, never out
of the placed instance names, which are ancestry and not ownership.

`riscv.pc` is tested by BIT MEMBERSHIP of that declared net, for the same
reason. Half the nets on a typical path here are called
`riscv.decoder.pc_SB_DFF_Q_27_D_...` and are not the pc.

THE BINS RECONCILE OR THIS EXITS NON-ZERO. Every hop `timing_split.py` counts as
routing lands in exactly one bin, and the total is checked against that script's
own routing figure per seed. A histogram of half a path reads exactly like a
histogram of a path, which is the failure this check exists to make loud.

SCOPE, WHICH IS NARROW. `icetime -r` reports the critical path and nothing else.
This is therefore N samples of ONE path per placement. It is not a routing census
of the design, and a bin that is empty here says only that the worst path did not
touch that thing.

Usage: routing_bins.py <baseline_sweep.sh CSV> <synth JSON> [--top littlesoc]
"""

import argparse
import collections
import os
import statistics
import sys

# No `soc/__pycache__` or `soc/depth/__pycache__` for the sake of three imports.
sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "depth"))
import baseline_summary  # noqa: E402
import path_stages  # noqa: E402
import timing_split  # noqa: E402

EBR = "SB_RAM40_4K"
SPRAM = "SB_SPRAM256KA"
PC = "riscv.pc"

# Read in order: the first that matches takes the hop, so a hop touching a block
# RAM and the pc at once is charged to the memory. Nothing on the paths measured
# so far does both, and a hop counted twice would break the reconciliation that
# grades this script.
BINS = [f"EBR ({EBR})", f"SPRAM ({SPRAM})", f"{PC}-sourced", "neither"]


def netlist(path, top):
    """`path_stages.load`, plus the two indexes binning needs on top of it."""
    sys.setrecursionlimit(100000)
    nl = path_stages.load(path, top)
    nl["pc_bits"] = {bit for bit in nl["bits"].get(PC, []) if isinstance(bit, int)}
    if not nl["pc_bits"]:
        sys.exit(f"*** {path}: no net called {PC!r} in {top}, so one of the bins\n"
                 f"*** could never be reached. Fix the name rather than reading a\n"
                 f"*** zero column as a finding.")
    # Only the memories, and only to answer the last hop on a path: where a run
    # feeds no further named net, what it feeds is whatever reads it.
    readers = collections.defaultdict(list)
    for name, cell in nl["cells"].items():
        if cell["type"] in (EBR, SPRAM):
            for bit in path_stages.inputs_of(cell):
                readers[bit].append(name)
    nl["mem_readers"] = readers
    return nl


def bit_of(nl, net):
    """The netlist bit icetime's `<name>[<index>]` refers to, or None."""
    if net is None:
        return None
    name, index = net
    bits = nl["bits"].get(name)
    if bits is None:
        return None
    try:
        bit = bits[int(index)] if index is not None else bits[0]
    except (IndexError, ValueError):
        return None
    return bit if isinstance(bit, int) else None


def driving(nl, bit):
    """(cell name, cell type) for whatever drives this bit, or (None, None)."""
    cell = nl["driver"].get(bit)
    return (cell, nl["cells"][cell]["type"]) if cell else (None, None)


def ends(nl, hops, index):
    """What drives the net hop `index` carries, and what that run feeds.

    The carried net is the last one icetime resolved above the hop -- or, for the
    hops ahead of the first name on the path, the first one below it, which is
    the same net seen from its other side.
    """
    carried = None
    for earlier in hops[:index]:
        if earlier["net"]:
            carried = earlier["net"]
    if carried is None:
        carried = next((hop["net"] for hop in hops if hop["net"]), None)

    sink = next((hop["net"] for hop in hops[index:]
                 if hop["net"] and hop["net"] != carried), None)

    carried_bit = bit_of(nl, carried)
    source, source_type = driving(nl, carried_bit)
    if sink is not None:
        target, target_type = driving(nl, bit_of(nl, sink))
    else:
        reader = next(iter(nl["mem_readers"].get(carried_bit, [])), None)
        target = reader
        target_type = nl["cells"][reader]["type"] if reader else None
    return {"net": carried, "bit": carried_bit,
            "from": source, "from_type": source_type,
            "to": target, "to_type": target_type}


def bin_of(nl, end):
    types = (end["from_type"], end["to_type"])
    if EBR in types:
        return BINS[0]
    if SPRAM in types:
        return BINS[1]
    if end["bit"] in nl["pc_bits"]:
        return BINS[2]
    return BINS[3]


def walk(nl, report):
    """Every routing hop on one path, binned, reconciled against timing_split."""
    split = timing_split.summarise(report)
    hops = path_stages.path_hops(report)
    binned = []
    for index, hop in enumerate(hops):
        if hop["kind"] is None or hop["kind"] in timing_split.LOGIC_CELLS:
            continue
        end = ends(nl, hops, index)
        binned.append({"cell": hop["cell"], "kind": hop["kind"],
                       "delay": hop["delay"], "bin": bin_of(nl, end), **end})
    if not binned:
        sys.exit(f"*** {report}: no routing hop was read out of it at all. That is\n"
                 f"*** a failed read of the report, not a design with no interconnect.")

    walked = sum(hop["delay"] for hop in binned)
    if abs(walked - split["routing"]) > 0.005:
        sys.exit(f"*** {report}: the bins come to {walked:.3f} ns but "
                 f"soc/timing_split.py\n"
                 f"*** reports {split['routing']:.3f} ns of routing on the same path. "
                 f"Some hop is\n"
                 f"*** being binned twice or not at all; fix this script rather than\n"
                 f"*** reading the histogram.")
    return split, binned


def tally(binned):
    counts = collections.Counter()
    delays = collections.Counter()
    for hop in binned:
        counts[hop["bin"]] += 1
        delays[hop["bin"]] += hop["delay"]
    return counts, delays


def named(cell, kind):
    if cell is None:
        return "(unresolved)"
    return f"{cell} ({kind})"


def report_bins(label, counts, delays):
    total = sum(delays.values())
    print(f"  {label}")
    for name in BINS:
        share = 100 * delays[name] / total if total else 0.0
        print(f"    {name:<24s} {counts[name]:>4d} hops  {delays[name]:7.2f} ns  "
              f"{share:5.1f}%")
    print(f"    {'all routing':<24s} {sum(counts.values()):>4d} hops  {total:7.2f} ns")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="a sweep written by soc/baseline_sweep.sh")
    parser.add_argument("netlist", help="the synthesis JSON those seeds placed")
    parser.add_argument("--top", default="littlesoc")
    args = parser.parse_args()

    provenance, rows = baseline_summary.load(args.csv)
    baseline_summary.report(args.csv, provenance, rows)
    print(f"  netlist      : {args.netlist} (top {args.top})")
    print()
    print("  SCOPE: `icetime -r` prints the critical path and nothing else, so this")
    print(f"  is {len(rows)} sample{'' if len(rows) == 1 else 's'} of ONE path each "
          f"and not a routing census of the design.")
    print("  An empty bin says the worst path missed that thing.")
    print()

    where = os.path.dirname(os.path.abspath(args.csv))
    nl = netlist(args.netlist, args.top)

    per_seed = []
    for row in rows:
        report = os.path.join(where, f"{row['variant']}.{row['seed']}.rpt")
        if not os.path.exists(report):
            sys.exit(f"*** {report} is not there, so seed '{row['seed']}' has a row\n"
                     f"*** and no placement behind it. Re-run the sweep rather than\n"
                     f"*** summarising the seeds that survived.")
        split, binned = walk(nl, report)
        counts, delays = tally(binned)
        per_seed.append({"seed": row["seed"], "split": split, "hops": binned,
                         "counts": counts, "delays": delays})

    print("== routing nanoseconds by bin, per placement ==")
    for entry in per_seed:
        report_bins(f"seed {entry['seed']:<8s} {entry['split']['total']:6.2f} ns path, "
                    f"{entry['split']['routing']:6.2f} ns routing",
                    entry["counts"], entry["delays"])
    aggregate_counts = collections.Counter()
    aggregate_delays = collections.Counter()
    for entry in per_seed:
        aggregate_counts.update(entry["counts"])
        aggregate_delays.update(entry["delays"])
    print()
    report_bins(f"aggregate over {len(per_seed)} "
                f"placement{'' if len(per_seed) == 1 else 's'}",
                aggregate_counts, aggregate_delays)

    print()
    print("== the fraction of routing in each bin, where the requirement is read ==")
    worst = per_seed[-1]
    median = per_seed[(len(per_seed) - 1) // 2]
    for label, entry in (("worst", worst), ("median", median)):
        routing = entry["split"]["routing"]
        shares = "  ".join(f"{name.split(' ')[0]} {100 * entry['delays'][name] / routing:.1f}%"
                           for name in BINS)
        print(f"  {label:<7s} seed {entry['seed']:<8s} "
              f"{1000 / entry['split']['total']:5.2f} MHz  {routing:6.2f} ns routing")
        print(f"          {shares}")
    print("  The median is the lower of the two middle placements at an even count,")
    print("  so it names a seed a report can be re-read from.")

    print()
    print("== the single largest routing hop of each placement, both ends named ==")
    for entry in per_seed:
        hop = max(entry["hops"], key=lambda h: h["delay"])
        print(f"  seed {entry['seed']:<8s} {hop['delay']:5.3f} ns  "
              f"{hop['cell']} ({hop['kind']})  [{hop['bin']}]")
        print(f"    from {named(hop['from'], hop['from_type'])}")
        print(f"    to   {named(hop['to'], hop['to_type'])}")

    print()
    print("== the memory instances at the ends of the binned hops ==")
    contacts = collections.Counter()
    for entry in per_seed:
        for hop in entry["hops"]:
            for cell, kind in ((hop["from"], hop["from_type"]),
                               (hop["to"], hop["to_type"])):
                if kind in (EBR, SPRAM):
                    contacts[cell] += 1
    if not contacts:
        print("  none: no placement's critical path touched a memory at all.")
    for cell, count in sorted(contacts.items(), key=lambda item: -item[1]):
        print(f"  {cell:<28s} at one end of {count} hops")


if __name__ == "__main__":
    main()
