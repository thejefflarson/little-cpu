#!/usr/bin/env python3
"""Turns one nano-qspi-sim --bench run's log into a row of the QSPI timing table.

Reads the same `BENCH marks=<n> cycles=<n> verdict=<n> writes=<n>` line
bench_report.py reads, plus the `BUCKETS execute=<n> parcel_wait=<n>
redirect_preamble=<n> psram_wait=<n>` line nano_cxxrtl.cc prints only when built
against nano_qspi_memory.v. Both must be present, both markers must have landed, and
the benchmark's own self-check (verdict) must be 1 -- this is the same validity bar
bench_report.py holds a plain zero-wait run to.
"""

import argparse
import re
import sys

BENCH = re.compile(
    r"^BENCH marks=(?P<marks>\d+) cycles=(?P<cycles>\d+) "
    r"verdict=(?P<verdict>\d+) writes=(?P<writes>\d+)"
)
BUCKETS = re.compile(
    r"^BUCKETS execute=(?P<execute>\d+) parcel_wait=(?P<parcel_wait>\d+) "
    r"redirect_preamble=(?P<redirect_preamble>\d+) psram_wait=(?P<psram_wait>\d+) "
    r"total_cycles=(?P<total_cycles>\d+)"
)
VAX_DHRYSTONES_PER_SEC = 1757.0


def parse(path):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the simulation log: {exc}")
    bench = buckets = None
    for line in text.splitlines():
        if bench is None:
            m = BENCH.match(line.strip())
            if m:
                bench = {k: int(v) for k, v in m.groupdict().items()}
        if buckets is None:
            m = BUCKETS.match(line.strip())
            if m:
                buckets = {k: int(v) for k, v in m.groupdict().items()}
    if bench is None:
        sys.exit(f"no BENCH line in {path}: a run that did not happen, not a run with no result.")
    if buckets is None:
        sys.exit(
            f"no BUCKETS line in {path}: this log was not produced by a build against "
            "nano_qspi_memory.v (NANO_QSPI_TIMING)."
        )
    return bench, buckets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="nano-qspi-sim --bench's output")
    parser.add_argument("--config", required=True, help="a label for the table row")
    parser.add_argument("--kind", choices=["dhrystone", "coremark"], required=True)
    parser.add_argument("--runs", type=int, required=True,
                         help="Dhrystone runs, or CoreMark iterations")
    parser.add_argument("--mhz", type=float, default=64.0,
                         help="the clock this row's absolute figure assumes (default 64, the brief's target)")
    args = parser.parse_args()

    if args.runs <= 0:
        sys.exit(f"--runs is {args.runs}; nothing was measured.")

    bench, buckets = parse(args.log)
    if bench["marks"] != 2:
        sys.exit(f"nano published {bench['marks']} marker(s), not 2 -- the run did not reach both ends.")
    if bench["cycles"] <= 0:
        sys.exit(f"nano measured {bench['cycles']} cycles, which is not a run.")
    if bench["verdict"] != 1:
        sys.exit(
            f"the benchmark's own self-check word is {bench['verdict']}, not 1: this "
            "run's cycle count is not a correct one."
        )

    total_cycles = buckets.pop("total_cycles")
    bucket_total = sum(buckets.values())
    if bucket_total != total_cycles:
        sys.exit(
            f"QSPI TIMING ACCOUNTING MISMATCH: buckets sum to {bucket_total} against "
            f"{total_cycles} simulated cycles."
        )

    cycles = bench["cycles"]
    per_unit = cycles / args.runs
    pct = {k: 100.0 * v / bucket_total for k, v in buckets.items()}
    row = {
        "config": args.config,
        "cycles": cycles,
        "per_unit": per_unit,
        "pct_execute": pct["execute"],
        "pct_parcel_wait": pct["parcel_wait"],
        "pct_redirect_preamble": pct["redirect_preamble"],
        "pct_psram_wait": pct["psram_wait"],
    }
    if args.kind == "dhrystone":
        per_mhz = (args.runs * 1e6 / cycles) / VAX_DHRYSTONES_PER_SEC
        row["metric_name"] = "DMIPS/MHz"
        row["metric"] = per_mhz
        row["absolute_name"] = f"DMIPS at {args.mhz:g} MHz"
        row["absolute"] = per_mhz * args.mhz
    else:
        per_mhz = args.runs * 1e6 / cycles
        row["metric_name"] = "CoreMark/MHz"
        row["metric"] = per_mhz
        row["absolute_name"] = f"CoreMark at {args.mhz:g} MHz"
        row["absolute"] = per_mhz * args.mhz

    print(
        f"{row['config']}\t{args.kind}\t"
        f"{cycles}\t{per_unit:.1f}\t{row['metric_name']}={row['metric']:.4f}\t"
        f"{row['absolute_name']}={row['absolute']:.2f}\t"
        f"execute={pct['execute']:.2f}%\tparcel_wait={pct['parcel_wait']:.2f}%\t"
        f"redirect_preamble={pct['redirect_preamble']:.2f}%\tpsram_wait={pct['psram_wait']:.2f}%"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
