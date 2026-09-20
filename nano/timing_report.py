#!/usr/bin/env python3
"""Read `make nano-timing`'s per-register-file-build yosys log and `stat -liberty -json`
output, and print one stamped area-and-delay report -- never a ratchet, since this is a
ranking instrument for two RTL versions, not a gate.

THE ONE READER of ABC's `stime -c` line, the way `nano/area_report.py` is the one reader
of `stat -liberty -json` alone: `test/probe_gates.sh` drives every refusal below against a
fixture log and a fixture liberty file, never against a real yosys run.

The area printed here comes from THIS instrument's own delay-oriented ABC script
(`+strash;dch,-f;map,-B,0.2;topo`), not `nano-area`'s plain `abc -liberty`, so the two
area figures measure different mappings of the same source and are never interchangeable
-- `nano-area`'s NANO_MAX_UM2 ratchet is untouched by this target and stays the one gated
area number. ABC's own `stime -c` line also prints an internal "Area" field of its own;
that field is dropped entirely here rather than read, because it is neither this figure
nor the ratchet's.
"""

import argparse
import json
import re
import sys

from area_report import check_liberty, load_stat, validate_design

STIME_LINE = re.compile(r"Delay\s*=\s*([0-9.]+)\s*ps")
YOSYS_VERSION = re.compile(r"^Yosys\s+\S+.*$", re.MULTILINE)


def read_delay_ps(log_path):
    try:
        text = open(log_path).read()
    except FileNotFoundError:
        sys.exit(
            f"*** make nano-timing: no synthesis log at {log_path}. That is a failed\n"
            "*** run, not a zero-delay design."
        )
    matches = STIME_LINE.findall(text)
    if not matches:
        sys.exit(
            f"*** make nano-timing: {log_path} carries no ABC `stime -c` Delay line.\n"
            "*** `abc -liberty ... -script ...;stime,-c` always prints one; a log\n"
            "*** missing it did not run that script, or died before reaching it."
        )
    return float(matches[-1])


def read_area_um2(json_path, liberty_path, liberty_cells):
    design = load_stat(json_path, target_name="nano-timing")
    validated = validate_design(
        design, json_path, liberty_path, liberty_cells, target_name="nano-timing"
    )
    return validated["area"]


def yosys_version(log_path):
    try:
        text = open(log_path).read()
    except FileNotFoundError:
        return "unknown yosys version"
    found = YOSYS_VERSION.findall(text)
    return found[-1] if found else "unknown yosys version"


def parse_variant(spec):
    parts = spec.split(":", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"{spec!r} is not NAME:LOG:JSON")
    return tuple(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--liberty", required=True)
    parser.add_argument("--liberty-sha256", required=True)
    parser.add_argument(
        "--variant", action="append", type=parse_variant, required=True,
        help="NAME:LOG:JSON, once per register-file build",
    )
    parser.add_argument(
        "--flow-correlation",
        help="a JSON file recording the last local-vs-flow area pair this tree measured",
    )
    args = parser.parse_args()

    liberty_cells = check_liberty(args.liberty, args.liberty_sha256, target_name="nano-timing")

    print("nano local timing/area instrument -- a ranking proxy, never a flow number")
    print(f"  {yosys_version(args.variant[0][1])}")
    print(f"  liberty  : {args.liberty}")
    print("  corner   : tt_025C_1v80 (typical) -- the only sky130hd corner this repo can")
    print("             pin as a standalone file; ss_100C_1v60 ships only inside the full")
    print("             sky130A PDK bundle LibreLane fetches, not as a git blob, so it is")
    print("             not reported here")
    print(
        "  recipe   : synth; dfflibmap -liberty; abc -liberty <lib> -script "
        "+strash;dch,-f;map,-B,0.2;topo;stime,-c"
    )
    print()

    for name, log_path, json_path in args.variant:
        delay_ps = read_delay_ps(log_path)
        area_um2 = read_area_um2(json_path, args.liberty, liberty_cells)
        print(f"register file: {name}")
        print(f"  area  : {area_um2:.2f} um2   (this recipe's own mapping, not NANO_MAX_UM2)")
        print(f"  delay : {delay_ps:.2f} ps    (ABC's mapped estimate, pre-layout)")
        print()

    print(
        "NEITHER FIGURE IS PLACED, ROUTED, OR SIGNED OFF -- no real wires, no per-corner\n"
        "slack, no DRC/LVS. For ranking two RTL versions against each other; not a gate,\n"
        "and never merged with a `nano-tt-area-selfhosted` flow figure."
    )

    if args.flow_correlation:
        print()
        print_correlation(args.flow_correlation)


def print_correlation(path):
    try:
        pair = json.load(open(path))
    except FileNotFoundError:
        sys.exit(f"*** make nano-timing: no correlation record at {path}.")
    required = {"local_um2", "flow_um2", "flow_tool", "flow_tag", "local_tree", "note"}
    missing = required - set(pair)
    if missing:
        sys.exit(
            f"*** make nano-timing: {path} is missing {sorted(missing)} -- a\n"
            "*** correlation record with no provenance is not a correlation."
        )
    factor = pair["flow_um2"] / pair["local_um2"]
    print("correlation against the flow (STALE -- see the note below):")
    print(f"  local tree : {pair['local_tree']}")
    print(f"  local area : {pair['local_um2']:.2f} um2  (make nano-area, that tree)")
    print(
        f"  flow area  : {pair['flow_um2']:.2f} um2  "
        f"({pair['flow_tool']}, {pair['flow_tag']}, {factor:.3f}x the local figure)"
    )
    print(f"  *** {pair['note']}")


if __name__ == "__main__":
    main()
