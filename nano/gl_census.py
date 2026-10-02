#!/usr/bin/env python3
"""Counts sky130_fd_sc_hd cell instantiations in a hardened gate-level netlist.

Refuses a file with none, which is RTL or an empty flow output rather than a netlist, so a
gate-level run never passes by simulating the wrong thing.

A hard macro the netlist instantiates (the register file's `rf_top`) is named with
`--macro`: it must appear exactly once, and any other instantiated module that is neither
a sky130_fd_sc_hd cell, a named macro nor defined in the same file is refused. The macro's
gate-level view is its behavioural model, which `--macro-source` and `--macro-model`
extract from the RTL that the flow's Verilog sources define it in, so the simulation reads
the model the proofs and the other simulators read.
"""

import argparse
import re
import sys

CELL_RE = re.compile(r"\bsky130_fd_sc_hd__([A-Za-z0-9_]+?)_(\d+)\b")
INSTANCE_RE = re.compile(r"^\s*(\\?[A-Za-z_][\w$.]*)\s+(\\\S+|[A-Za-z_][\w$]*)\s*\(", re.MULTILINE)
NOT_INSTANCES = {
    "module", "input", "output", "inout", "wire", "reg", "assign", "always", "initial",
    "begin", "end", "function", "task", "generate", "parameter", "localparam", "supply0",
    "supply1", "tri", "logic", "if", "else", "case", "for", "endmodule", "defparam",
}


def census(text):
    counts = {}
    types = set()
    for name, strength in CELL_RE.findall(text):
        counts[name] = counts.get(name, 0) + 1
        types.add("sky130_fd_sc_hd__%s_%s" % (name, strength))
    return counts, types


def instantiated_modules(text):
    """Every module type the netlist instantiates, with its count."""
    found = {}
    for kind, _name in INSTANCE_RE.findall(text):
        if kind not in NOT_INSTANCES:
            found[kind] = found.get(kind, 0) + 1
    return found


def check_macros(text, macros):
    """The reasons the netlist's non-cell instances are not exactly `macros`, each once."""
    problems = []
    found = instantiated_modules(text)
    for name in macros:
        if found.get(name, 0) != 1:
            problems.append(
                "macro %s is instantiated %d times, not once -- a netlist without it "
                "simulates a chip with no register file, and a second one is a block "
                "nothing here models." % (name, found.get(name, 0))
            )
    defined = set(re.findall(r"^\s*module\s+([A-Za-z_][\w$]*)", text, re.MULTILINE))
    for kind in sorted(found):
        if kind not in macros and kind not in defined and not kind.startswith("sky130_fd_sc_hd__"):
            problems.append(
                "unexpected instantiated module %s (%d) -- neither a sky130_fd_sc_hd "
                "cell nor a named --macro, so nothing simulates it." % (kind, found[kind])
            )
    return problems


def extract_module(source, name):
    """The text of `module name ... endmodule` from `source`, or None."""
    match = re.search(r"^module\s+%s\b.*?^endmodule\b" % re.escape(name), source,
                      re.MULTILINE | re.DOTALL)
    return match.group(0) + "\n" if match else None


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("netlist")
    parser.add_argument(
        "--includes",
        help="write one `include per cell type here, so every model is read in one "
        "compilation unit and its include guards hold across drive strengths",
    )
    parser.add_argument(
        "--macro", action="append", default=[],
        help="a hard macro the netlist instantiates exactly once",
    )
    parser.add_argument(
        "--macro-source", help="the RTL file whose module definition models each --macro"
    )
    parser.add_argument(
        "--macro-model", help="write each --macro's module definition, extracted from "
        "--macro-source, to this file"
    )
    args = parser.parse_args(argv)

    with open(args.netlist) as f:
        text = f.read()

    counts, types = census(text)
    total = sum(counts.values())
    if total == 0:
        print(
            "error: no sky130_fd_sc_hd cell instantiations found in %s -- this does "
            "not read as a hardened netlist." % args.netlist,
            file=sys.stderr,
        )
        return 1

    problems = check_macros(text, args.macro)
    if args.macro_model:
        if not args.macro or not args.macro_source:
            problems.append("--macro-model needs --macro and --macro-source")
        else:
            with open(args.macro_source) as f:
                source = f.read()
            models = []
            for name in args.macro:
                model = extract_module(source, name)
                if model is None:
                    problems.append("%s defines no module %s to simulate it with"
                                    % (args.macro_source, name))
                else:
                    models.append(model)
            if not problems:
                with open(args.macro_model, "w") as f:
                    f.write("\n".join(models))
    if problems:
        for problem in problems:
            print("error: %s: %s" % (args.netlist, problem), file=sys.stderr)
        return 1

    for name in sorted(counts):
        print("%6d  %s" % (counts[name], name))
    print("%6d  TOTAL" % total)
    for name in args.macro:
        print("%6d  macro %s" % (1, name))

    if args.includes:
        with open(args.includes, "w") as f:
            f.writelines('`include "%s.v"\n' % t for t in sorted(types))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
