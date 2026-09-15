#!/usr/bin/env python3
"""Turns one nano-qspi-sim --bench run's log into a row of the QSPI timing table.

Reads `BENCH ...`, `MODEL ...` and `BUCKETS ...` (execute/parcel_wait/redirect_preamble/
loop_hit/handshake/psram_wait/window_cycles) lines nano_cxxrtl.cc prints only when built
against nano_qspi_memory.v. All three must be present, both markers must have landed, and
the benchmark's own self-check (verdict) must be 1.
"""

import argparse
import re
import sys

BENCH = re.compile(
    r"^BENCH marks=(?P<marks>\d+) cycles=(?P<cycles>\d+) "
    r"verdict=(?P<verdict>\d+) writes=(?P<writes>\d+)"
)
MODEL = re.compile(
    r"^MODEL depth=(?P<depth>\d+) loop_kind=(?P<loop_kind>\d+) loop_window=(?P<loop_window>\d+) "
    r"preamble=(?P<preamble>\d+) parcel=(?P<parcel>\d+) psram_load=(?P<psram_load>\d+) "
    r"psram_store=(?P<psram_store>\d+)"
)
BUCKETS = re.compile(
    r"^BUCKETS execute=(?P<execute>\d+) parcel_wait=(?P<parcel_wait>\d+) "
    r"redirect_preamble=(?P<redirect_preamble>\d+) loop_hit=(?P<loop_hit>\d+) "
    r"handshake=(?P<handshake>\d+) psram_wait=(?P<psram_wait>\d+) "
    r"window_cycles=(?P<window_cycles>\d+)"
)
VAX_DHRYSTONES_PER_SEC = 1757.0
BUCKET_KEYS = ("execute", "parcel_wait", "redirect_preamble", "loop_hit", "handshake", "psram_wait")


def parse(path):
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as exc:
        sys.exit(f"cannot read the simulation log: {exc}")
    bench = model = buckets = None
    for line in text.splitlines():
        if bench is None and (m := BENCH.match(line.strip())):
            bench = {k: int(v) for k, v in m.groupdict().items()}
        if model is None and (m := MODEL.match(line.strip())):
            model = {k: int(v) for k, v in m.groupdict().items()}
        if buckets is None and (m := BUCKETS.match(line.strip())):
            buckets = {k: int(v) for k, v in m.groupdict().items()}
    if bench is None:
        sys.exit(f"no BENCH line in {path}: a run that did not happen, not a run with no result.")
    if model is None:
        sys.exit(f"no MODEL line in {path}: this binary did not print its own configuration.")
    if buckets is None:
        sys.exit(
            f"no BUCKETS line in {path}: this log was not produced by a build against "
            "nano_qspi_memory.v (NANO_QSPI_TIMING)."
        )
    return bench, model, buckets


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

    bench, model, buckets = parse(args.log)
    if bench["marks"] != 2:
        sys.exit(f"nano published {bench['marks']} marker(s), not 2 -- the run did not reach both ends.")
    if bench["cycles"] <= 0:
        sys.exit(f"nano measured {bench['cycles']} cycles, which is not a run.")
    if bench["verdict"] != 1:
        sys.exit(
            f"the benchmark's own self-check word is {bench['verdict']}, not 1: this "
            "run's cycle count is not a correct one."
        )

    window_cycles = buckets["window_cycles"]
    bucket_total = sum(buckets[k] for k in BUCKET_KEYS)
    if bucket_total != window_cycles:
        sys.exit(
            f"QSPI TIMING ACCOUNTING MISMATCH: buckets sum to {bucket_total} against "
            f"{window_cycles} windowed cycles."
        )
    if window_cycles != bench["cycles"]:
        sys.exit(
            f"QSPI TIMING WINDOW MISMATCH: the bucket window covered {window_cycles} "
            f"cycles against BENCH's own {bench['cycles']} -- they should be the same region."
        )

    cycles = bench["cycles"]
    per_unit = cycles / args.runs
    pct = {k: 100.0 * buckets[k] / window_cycles for k in BUCKET_KEYS}
    row = {
        "config": args.config,
        "model": model,
        "cycles": cycles,
        "per_unit": per_unit,
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
        + "\t".join(f"{k}={pct[k]:.2f}%" for k in BUCKET_KEYS)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
