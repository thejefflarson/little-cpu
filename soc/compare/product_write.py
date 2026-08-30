#!/usr/bin/env python3
"""Write or update one benchmark pair's entry in soc/compare/product.json.

THE ARTIFACT IS THE PROVENANCE; THE PROSE STAYS HAND-WRITTEN. CLAUDE.md's
cross-core paragraph and its ADRs are still a person's sentences about a tree
they read -- this script exists so the NUMBERS those sentences quote have one
place they were actually measured, instead of living only in a paragraph
nothing re-checks. soc/compare/run_product.sh is the only caller; it does the
measuring (placements, simulations) and hands this script the results to
record. Never invoked by hand against invented numbers -- there is nothing here
that would catch that.

Modelled on soc/baseline_sweep.sh's provenance block: every measured pair
carries the commit it was taken against, whether the tree was dirty, the seeds,
the CFLAGS and ISA the shared image was built with, and the resolved toolchain
-- the same fields soc/compare/product_check.py later checks a stamp against
before letting anything print a product from it.

A BENCHMARK NAMES A LIST OF CORES, NOT A FIXED PAIR. Dhrystone against
VexRiscv today is two cores under one target; a third core sharing the same
image (VexRiscv and Hazard3 together, say) is the same shape with one more
`--clock-ns`/`--cycle-factor` pair, not a new schema -- so a "pair" here is a
`(target core, one other core)` PRODUCT, derived for every other core present,
never the benchmark's own unit. `--isa` is stamped alongside `--cflags`
because three cores sharing one image may share a narrower ISA than any one of
them implements on its own, and a reader comparing two measurements needs to
know that changed even when the full flag string still differs only in ways
that do not matter.

One pair per call. The artifact is read back first and only the named
benchmark's entry is replaced, so `make compare-product` writing "dhrystone"
and then "coremark" builds up one file rather than each call erasing the
other's work -- and a run where CoreMark's harness is not yet on this tree
still leaves a prior measurement of it alone rather than clobbering it back to
"not yet measured".

Usage:
  product_write.py OUT.json dhrystone --target-core littlecpu \\
    --base <sha> --dirty no --date <iso8601> --seeds 'default 1 2 ...' \\
    --cflags '...' --isa rv32ic --rom-words 1024 --ram-words 512 \\
    --unit DMIPS/MHz --tool yosys='Yosys 0.68 [/opt/bin/yosys]' [--tool ...] \\
    --clock-ns littlecpu=32.36,31.25,... --clock-ns vexriscv=20.73,20.18,... \\
    --cycle-factor littlecpu=0.748 --cycle-factor vexriscv=0.557

  product_write.py OUT.json coremark --target-core littlecpu \\
    --not-yet-measured --core hazard3 \\
    --reason 'make compare-coremark is not on this tree'
"""

import argparse
import json
import statistics
import sys
from datetime import datetime, timezone

SCHEMA = "compare-product v2"
NOTE = "written by soc/compare/run_product.sh (make compare-product); do not hand-edit"


def kv(spec, what):
    if "=" not in spec:
        sys.exit(f"error: {what} wants NAME=VALUE, got '{spec}'")
    name, value = spec.split("=", 1)
    if not name:
        sys.exit(f"error: {what} '{spec}' names no core or tool")
    return name, value


def floats(spec, what):
    name, value = kv(spec, what)
    try:
        values = [float(v) for v in value.split(",") if v]
    except ValueError:
        sys.exit(f"error: {what} '{spec}' is not a comma-separated list of numbers")
    if not values:
        sys.exit(f"error: {what} '{spec}' names no placements")
    return name, values


def clock_stats(ns_values):
    worst = max(ns_values)
    best = min(ns_values)
    return {
        "n": len(ns_values),
        "worst_ns": worst,
        "median_ns": statistics.median(ns_values),
        "best_ns": best,
        "worst_mhz": 1000.0 / worst,
        "median_mhz": 1000.0 / statistics.median(ns_values),
        "best_mhz": 1000.0 / best,
        # Best-to-worst as a percentage of the best placement, the same
        # convention soc/baseline_summary.py's stats() uses -- so a reader who
        # already knows that column's shape does not have to learn a second one.
        "spread_pct": 100.0 * (worst - best) / best,
    }


def measured_pair(args):
    if len(args.clock_ns) < 2:
        sys.exit("error: --clock-ns must be given at least twice -- the target "
                 f"core ({args.target_core}) and at least one other -- got "
                 f"{len(args.clock_ns)}")
    if len(args.cycle_factor) != len(args.clock_ns):
        sys.exit("error: --cycle-factor must be given once per --clock-ns core, "
                 f"got {len(args.cycle_factor)} for {len(args.clock_ns)} cores")

    clocks = {}
    for spec in args.clock_ns:
        core, values = floats(spec, "--clock-ns")
        clocks[core] = clock_stats(values)
    factors = {}
    for spec in args.cycle_factor:
        core, value = kv(spec, "--cycle-factor")
        try:
            factors[core] = float(value)
        except ValueError:
            sys.exit(f"error: --cycle-factor '{spec}' is not a number")

    if set(clocks) != set(factors):
        sys.exit(f"error: --clock-ns named {sorted(clocks)} but --cycle-factor "
                 f"named {sorted(factors)} -- every core needs both")
    if args.target_core not in clocks:
        sys.exit(f"error: --target-core {args.target_core} has no --clock-ns/"
                 "--cycle-factor of its own; every product is relative to it")

    cores = {core: {"clock_mhz": clocks[core], "cycle_factor": factors[core]}
             for core in clocks}

    products = {}
    target = args.target_core
    for core in clocks:
        if core == target:
            continue
        target_dmips = {
            stat: factors[target] * clocks[target][f"{stat}_mhz"]
            for stat in ("worst", "median")
        }
        other_dmips = {
            stat: factors[core] * clocks[core][f"{stat}_mhz"]
            for stat in ("worst", "median")
        }
        products[core] = {
            f"{target}_dmips": target_dmips,
            f"{core}_dmips": other_dmips,
            "ratio": {
                stat: other_dmips[stat] / target_dmips[stat]
                for stat in ("worst", "median")
            },
        }

    tools = {}
    for spec in args.tool:
        name, value = kv(spec, "--tool")
        tools[name] = value
    if not tools:
        sys.exit("error: --measured wants at least one --tool; a number with no "
                 "recorded toolchain cannot be told apart from one a moved "
                 "toolchain produced")

    for field, value in (("base", args.base), ("dirty", args.dirty),
                         ("seeds", args.seeds), ("cflags", args.cflags),
                         ("isa", args.isa), ("unit", args.unit)):
        if not value:
            sys.exit(f"error: --measured wants --{field.replace('_', '-')}")
    if args.rom_words is None or args.ram_words is None:
        sys.exit("error: --measured wants --rom-words and --ram-words")

    return {
        "status": "measured",
        "target_core": target,
        "unit": args.unit,
        "isa": args.isa,
        "base": args.base,
        "dirty": args.dirty,
        "date": args.date or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "seeds": args.seeds,
        "cflags": args.cflags,
        "rom_words": args.rom_words,
        "ram_words": args.ram_words,
        "tools": tools,
        "cores": cores,
        "products": products,
    }


def unmeasured_pair(args):
    if not args.reason:
        sys.exit("error: --not-yet-measured wants --reason")
    return {
        "status": "not_yet_measured",
        "target_core": args.target_core,
        "cores": args.core,
        "reason": args.reason,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", help="soc/compare/product.json")
    parser.add_argument("benchmark", help="dhrystone or coremark")
    parser.add_argument("--target-core", default="littlecpu", help="the core "
                        "every product in this pair is stated relative to")
    parser.add_argument("--core", action="append", default=[],
                        help="(--not-yet-measured only) a core this pair will "
                        "compare littlecpu against once it is measured -- "
                        "documentation, not data; repeatable")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--not-yet-measured", action="store_true")
    mode.add_argument("--measured", action="store_true")
    parser.add_argument("--reason", help="why --not-yet-measured")
    parser.add_argument("--base", help="commit SHA the measurement was taken at")
    parser.add_argument("--dirty", choices=["yes", "no"])
    parser.add_argument("--date", help="ISO 8601 UTC; defaults to now")
    parser.add_argument("--seeds", help="the seed list swept, verbatim")
    parser.add_argument("--cflags", help="the CFLAGS the shared image was built with")
    parser.add_argument("--isa", help="the bare -march= value the shared image "
                        "was built at, e.g. rv32ic -- may be narrower than any "
                        "one core's own ISA when several share one image")
    parser.add_argument("--rom-words", type=int)
    parser.add_argument("--ram-words", type=int)
    parser.add_argument("--unit", help="DMIPS/MHz or CoreMark/MHz")
    parser.add_argument("--tool", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--clock-ns", action="append", default=[],
                        metavar="CORE=ns1,ns2,...")
    parser.add_argument("--cycle-factor", action="append", default=[],
                        metavar="CORE=VALUE")
    args = parser.parse_args()

    pair = unmeasured_pair(args) if args.not_yet_measured else measured_pair(args)

    try:
        with open(args.out) as handle:
            doc = json.load(handle)
    except FileNotFoundError:
        doc = {}
    except (OSError, json.JSONDecodeError) as exc:
        sys.exit(f"error: cannot read '{args.out}' to merge into it: {exc}")

    doc["schema"] = SCHEMA
    doc["note"] = NOTE
    doc.setdefault("pairs", {})[args.benchmark] = pair

    with open(args.out, "w") as handle:
        json.dump(doc, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(f"wrote {args.benchmark} ({pair['status']}) into {args.out}")


if __name__ == "__main__":
    main()
