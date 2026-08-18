#!/usr/bin/env python3
"""The mapped netlist's digest, and the one direction it is sound in.

A sixteen-seed sweep is about twelve minutes a side, and the shipping SoC's
worst placement of sixteen sits a fraction of a nanosecond over the 12 MHz
requirement -- with area measured twice not to be a margin lever, so there is
nowhere to put cells a change costs. That makes "can this edit have moved the
placer's input at all?" worth answering before any seed is spent, and a tied-off
change is the case where the answer is usually no.

A BARE HASH OF THE NETLIST DOES NOT ANSWER IT, and that is measured rather than
assumed. Synthesis is byte-deterministic run to run -- two builds of one tree,
identical SHA-256. But a comment-only edit moves the mapped JSON in 66 places,
every one of them a `src` or `module_src` line number; and a dead wire yosys
optimises away still moves it, because a `netnames` entry carrying `unused_bits`
stays behind and renumbers every net bit id after it. Strict equality therefore
reports "different" for exactly the change class the gate exists for, which
degenerates the gate into "always sweep" -- a grader with one verdict.

THE CANONICAL FORM is the shipping synth script verbatim, plus `opt_clean
-purge`, with this script dropping `src` and `module_src` from every attributes
object and dropping nothing else. Both halves are load-bearing and were measured
apart: the purge is what removes the dead net and restores the numbering, and the
attribute drop is what survives a comment moving a line. Under it, a tree with a
comment inserted, a tree with a blank line inserted and a tree with a dead
tie-off injected all digest to the base tree's value, and a one-bit constant
change does not.

THE CANONICAL JSON IS A CLASSIFIER, NOT A NETLIST TO PLACE. Measured: the purged
form places to a different bitstream than the shipping form at the same seed,
same design. Nothing may hand it to nextpnr, and what the gate rests on instead
is the direct control -- that the differences this form forgives do not move the
placement of the SHIPPING netlist. `soc/netlist_determinism.sh` re-takes that on
the tree being asked about, and the digest is void without it.

THE CLAIM, sound in one direction by construction and incomplete in the other by
design: digest-equal implies the placer's input differs only in dead nets and
source attributes, so no seeds are owed; digest-different implies nothing at all
about the placement, so spend them. The `creator` string is inside the digest, so
a toolchain that moved reads as different -- which is the sound direction, and
the direction this repo has been bitten in.

A missing, empty, truncated or unparseable netlist is refused. None of those is
"equal", and the whole value of the gate is that its equal verdict is the one
that skips work.

Usage: netlist_digest.py digest  <canonical.json> [--label <name>]
       netlist_digest.py compare <base.json> <new.json>
                                 [--base-label <name>] [--new-label <name>]

Exit: 0 equal (or digested), 1 different, 2 refused.
"""

import argparse
import hashlib
import json
import sys
from collections import Counter
from itertools import islice

# The two attributes a comment moves and nothing else reads. Widening this list
# is how the gate would go quiet: every attribute dropped here is a difference
# the digest forgives, and the placer is not obliged to agree.
DROPPED_ATTRS = ("src", "module_src")

# What a difference means at today's margin. Printed by both subcommands,
# because the number it turns on is not in this file and cannot be read off the
# digest.
MEANING = """\
digest-equal    the placer's input differs only in dead nets and source
                attributes, which `make netlist-determinism` has just placed
                identically on this tree. No sweep is owed.
digest-different implies NOTHING about the placement -- this gate is sound in one
                direction only. A paired sixteen-seed sweep is owed, and at
                today's margin, with the worst placement of sixteen a fraction of
                a nanosecond over the requirement and area measured not to buy
                any back, a netlist that moved is a stop-and-redesign signal
                rather than merely a sweep owed. There is nowhere to put the
                cells."""

REFUSED = 2
DIFFERENT = 1


def refuse(path, why):
    print(f"*** {path}: {why}.", file=sys.stderr)
    print("*** That is a failed digest, not an equal one.", file=sys.stderr)
    sys.exit(REFUSED)


def load(path):
    """One canonical netlist and its top module, or exit saying why not."""
    try:
        with open(path, "rb") as handle:
            text = handle.read()
    except OSError as err:
        refuse(path, f"{err.strerror}, so there is nothing to digest")

    if not text.strip():
        refuse(path, "empty, which is a synthesis that wrote nothing")

    try:
        design = json.loads(text)
    except (json.JSONDecodeError, UnicodeDecodeError) as err:
        refuse(path, f"not parseable as JSON ({err}) -- a truncated write "
                     "reads exactly like this")

    if not isinstance(design, dict) or not isinstance(design.get("modules"), dict):
        refuse(path, "not a yosys netlist: there is no `modules` object in it")

    tops = [name for name, module in design["modules"].items()
            if isinstance(module, dict)
            and module.get("attributes", {}).get("top") not in (None, 0, "0")]
    if len(tops) != 1:
        refuse(path, f"{len(tops)} modules are marked `top`, so there is no one "
                     "design in it")
    if not design["modules"][tops[0]].get("cells"):
        refuse(path, f"the top module `{tops[0]}` has no cells in it, which is a "
                     "failed synthesis")
    return design, tops[0]


def strip_attributes(node):
    """`node` with the source attributes dropped, and nothing else dropped."""
    if not isinstance(node, dict):
        return node
    attributes = node.get("attributes")
    if not isinstance(attributes, dict):
        return node
    kept = {k: v for k, v in attributes.items() if k not in DROPPED_ATTRS}
    return {**node, "attributes": kept}


def canonical(design):
    """The design in the form the digest is taken over.

    Only `attributes` objects are touched, never a name: a net called `src` is a
    real net, and dropping keys by name wherever they appear would forgive it.
    """
    modules = {}
    for name, module in design["modules"].items():
        if not isinstance(module, dict):
            modules[name] = module
            continue
        canon = strip_attributes(module)
        for section in ("cells", "netnames"):
            members = module.get(section)
            if isinstance(members, dict):
                canon[section] = {k: strip_attributes(v) for k, v in members.items()}
        modules[name] = canon
    return {**design, "modules": modules}


def digest(design):
    text = json.dumps(canonical(design), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode()).hexdigest()


def summarise(design, top_name):
    top = design["modules"][top_name]
    ports = top.get("ports", {})
    return {
        "creator": design.get("creator", "(no creator string)"),
        "top": top_name,
        "modules": sorted(design["modules"]),
        "cells": Counter(cell.get("type", "(untyped)")
                         for cell in top.get("cells", {}).values()),
        "ports": {name: f"{port.get('direction', '?')} [{len(port.get('bits', []))}]"
                  for name, port in ports.items()},
        "nets": len(top.get("netnames", {})),
    }


def report(path, label, design, top_name, sha):
    facts = summarise(design, top_name)
    cells = facts["cells"]
    print(f"== netlist digest: {label}")
    print(f"  file      {path}")
    print(f"  creator   {facts['creator']}")
    print(f"  top       {facts['top']} of {len(facts['modules'])} modules")
    print(f"  cells     {sum(cells.values())} in {len(cells)} types: "
          + ", ".join(f"{t} {n}" for t, n in cells.most_common(6))
          + (", ..." if len(cells) > 6 else ""))
    print(f"  ports     {len(facts['ports'])}: " + ", ".join(sorted(facts["ports"])))
    print(f"  nets      {facts['nets']} named")
    print(f"  digest    sha256:{sha}")
    return facts


def name_list(names, limit=8):
    names = sorted(names)
    shown = ", ".join(names[:limit])
    return shown + (f", ... ({len(names)} in all)" if len(names) > limit else "")


def paths_differing(base, new, path=""):
    """Every place the two canonical forms disagree, by path, lazily.

    The structural summary answers "what kind of thing moved" and is what a
    reader wants when a cell count moved. This answers "which one", and is
    printed only where the summary found nothing: a difference that moves no
    module, no cell count and no port still has to be named, or the report is the
    bare "changed" this script exists not to print. A generator so the caller's
    `islice` decides how much of a 7 MB netlist gets walked.
    """
    if isinstance(base, dict) and isinstance(new, dict):
        for key in sorted(set(base) - set(new)):
            yield f"{path}.{key}: in base only"
        for key in sorted(set(new) - set(base)):
            yield f"{path}.{key}: in this tree only"
        for key in sorted(set(base) & set(new)):
            yield from paths_differing(base[key], new[key], f"{path}.{key}")
    elif isinstance(base, list) and isinstance(new, list):
        if len(base) != len(new):
            yield f"{path}: {len(base)} entries -> {len(new)}"
        else:
            for index, (left, right) in enumerate(zip(base, new)):
                yield from paths_differing(left, right, f"{path}[{index}]")
    elif base != new:
        yield f"{path}: {short(base)} -> {short(new)}"


def short(value):
    text = str(value)
    return text if len(text) <= 48 else text[:45] + "..."


def structural_difference(base_facts, new_facts):
    """The named differences between two summaries, in the order they explain."""
    lines = []
    if base_facts["creator"] != new_facts["creator"]:
        lines.append("  toolchain: " + base_facts["creator"])
        lines.append("         ->  " + new_facts["creator"])
        lines.append("             A yosys that moved re-maps the whole design, so"
                     " nothing below is")
        lines.append("             a property of the edit.")
    if base_facts["top"] != new_facts["top"]:
        lines.append(f"  top module: {base_facts['top']} -> {new_facts['top']}")

    gone = set(base_facts["modules"]) - set(new_facts["modules"])
    added = set(new_facts["modules"]) - set(base_facts["modules"])
    if gone:
        lines.append(f"  modules in base only: {name_list(gone)}")
    if added:
        lines.append(f"  modules in this tree only: {name_list(added)}")

    base_cells, new_cells = base_facts["cells"], new_facts["cells"]
    moved = sorted(set(base_cells) | set(new_cells),
                   key=lambda t: -abs(new_cells[t] - base_cells[t]))
    moved = [t for t in moved if new_cells[t] != base_cells[t]]
    if moved:
        lines.append("  cell types in the top module:")
        for cell_type in moved[:10]:
            delta = new_cells[cell_type] - base_cells[cell_type]
            lines.append(f"    {cell_type:<24} {base_cells[cell_type]:>6} ->"
                         f" {new_cells[cell_type]:>6}  ({delta:+d})")
        total = sum(new_cells.values()) - sum(base_cells.values())
        lines.append(f"    {'total':<24} {sum(base_cells.values()):>6} ->"
                     f" {sum(new_cells.values()):>6}  ({total:+d})")

    base_ports, new_ports = base_facts["ports"], new_facts["ports"]
    for name in sorted(set(base_ports) | set(new_ports)):
        if base_ports.get(name) != new_ports.get(name):
            lines.append(f"  port {name}: {base_ports.get(name, '(none)')} ->"
                         f" {new_ports.get(name, '(none)')}")

    if base_facts["nets"] != new_facts["nets"]:
        lines.append(f"  named nets: {base_facts['nets']} -> {new_facts['nets']}")
    return lines


def compare(args):
    base, base_top = load(args.base)
    new, new_top = load(args.new)
    base_digest, new_digest = digest(base), digest(new)
    base_facts = report(args.base, args.base_label, base, base_top, base_digest)
    print()
    new_facts = report(args.new, args.new_label, new, new_top, new_digest)
    print()

    if base_digest == new_digest:
        print("DIGEST-EQUAL. The placer's input differs only in dead nets and")
        print("source attributes, so the placement distribution is unmoved and no")
        print("sweep is owed. This says nothing about behaviour: the suite, the")
        print("proofs and the co-simulation are unaffected by it and still run.")
        return 0

    print("DIGEST-DIFFERENT. The placer's input moved, so this gate says nothing")
    print("and the seeds have to be spent: a paired sixteen-seed sweep, one tree")
    print("apart. At today's margin a difference is a stop-and-redesign signal")
    print("rather than merely a sweep owed -- there is nowhere to put the cells.")
    print()
    print("What moved:")
    lines = structural_difference(base_facts, new_facts)
    if lines:
        print("\n".join(lines))
        return DIFFERENT

    print("  Nothing in the structural summary -- same modules, same cell types")
    print("  and counts, same ports, same net count. What moved is inside")
    print("  connectivity, parameters or a kept attribute:")
    for line in islice(paths_differing(canonical(base), canonical(new)), 12):
        print(f"    {line}")
    return DIFFERENT


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    one = sub.add_parser("digest", help="digest one canonical netlist")
    one.add_argument("json")
    one.add_argument("--label", default="this tree")

    two = sub.add_parser("compare", help="compare two canonical netlists")
    two.add_argument("base")
    two.add_argument("new")
    two.add_argument("--base-label", default="base")
    two.add_argument("--new-label", default="this tree")

    args = parser.parse_args()
    if args.command == "digest":
        design, top = load(args.json)
        report(args.json, args.label, design, top, digest(design))
        print()
        print(MEANING)
        return 0
    return compare(args)


if __name__ == "__main__":
    sys.exit(main())
