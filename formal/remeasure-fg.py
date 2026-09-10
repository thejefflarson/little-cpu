#!/usr/bin/env python3
# Re-measures F and G -- the two figures every depth in formal/checks.cfg's [depth] table
# is derived from -- and grades what it measures against the `#derive` lines that declare
# them.

import argparse
import importlib.util
import os
import shutil
import subprocess
import sys

import depth_rules

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "checks.cfg")
PROBE = "fg-probe"
GENCHECKS_DEFAULT = os.path.join(HERE, "genchecks-local.py")

def _load_sibling(name):
    """genchecks-audit.py is a hyphenated filename, so it is loaded by path rather
    than imported by name."""
    spec = importlib.util.spec_from_file_location(
        name.replace("-", "_"), os.path.join(HERE, f"{name}.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

genchecks_audit = _load_sibling("genchecks-audit")

# The two trigger depths G is measured at, so a flip point is bracketed rather than
# sampled once.
TRIGS = (10, 15)

# How far either side of the declared figure to sweep.
BELOW, ABOVE = 2, 1

def expected_cycles(depth_line):
    """The RISCV_FORMAL_*_CYCLE(S) values `depth_line` should produce, read off the
    same fields genchecks-local.py's check generation indexes into: hang is
    (start, depth), liveness is (start, trig, depth)."""
    label, *nums = depth_line.split()
    nums = [int(n) for n in nums]
    if label == "hang":
        return {"CHECK_CYCLE": nums[1]}
    if label == "liveness":
        return {"CHECK_CYCLE": nums[2], "TRIG_CYCLE": nums[1]}
    raise SystemExit(f"error: don't know which fields of {depth_line!r} are which")

def probe(depth_line, check, genchecks):
    """Generate a one-check set from checks.cfg with `depth_line` as the whole
    of [depth], run it, and return sby's status."""
    lines = []
    in_depth = False
    with open(CFG) as f:
        for line in f:
            line = line.rstrip("\n")
            if in_depth:
                if not line.startswith("["):
                    continue
                in_depth = False
            if line.strip() == "[depth]":
                in_depth = True
                lines += [line, depth_line]
                continue
            lines.append(line)
    with open(os.path.join(HERE, f"{PROBE}.cfg"), "w") as f:
        f.write("\n".join(lines) + "\n")

    shutil.rmtree(os.path.join(HERE, PROBE), ignore_errors=True)
    subprocess.run(
        [sys.executable, genchecks, PROBE],
        cwd=HERE,
        check=True,
        stdout=subprocess.DEVNULL,
    )
    sby = os.path.join(HERE, PROBE, f"{check}.sby")
    if not os.path.exists(sby):
        raise SystemExit(
            f"error: `{depth_line}` generated no {check} check. The [depth] key "
            "that names it has been renamed upstream, or the line is malformed."
        )

    generated = {}
    with open(sby) as f:
        for line in f:
            match = genchecks_audit.DEFINE_RE.match(line.rstrip("\n"))
            if match:
                generated[match.group(1)] = int(match.group(2))
    for name, want in expected_cycles(depth_line).items():
        got = generated.get(name)
        if got != want:
            raise SystemExit(
                f"error: {PROBE}/{check}.sby defines RISCV_FORMAL_{name} = {got}, "
                f"not the {want} this row swept ('{depth_line}'). genchecks-local.py's "
                "[depth] field order has drifted from what this script assumes, so "
                "every row would run at the same unasked-for depth while still "
                "reporting PASS/FAIL."
            )

    subprocess.run(
        ["sby", "-f", f"{PROBE}/{check}.sby"],
        cwd=HERE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    status_file = os.path.join(HERE, PROBE, check, "status")
    if not os.path.exists(status_file):
        raise SystemExit(
            f"error: sby wrote no status for {check}. It is on PATH and the "
            "pinned clone is present, or this script would not have got here, "
            f"so read formal/{PROBE}/{check}/logfile.txt."
        )
    with open(status_file) as f:
        return f.read().split()[0]

def sweep(label, check, rows, genchecks):
    """Run one sweep of `check` and return the lowest value that PASSes, or
    None. `rows` is a list of (value, description, depth_line)."""
    print(f"\n{label}")
    flip = None
    saw_fail = False
    for value, description, depth_line in rows:
        status = probe(depth_line, check, genchecks)
        print(f"  {description:<28} {status}")
        if status not in ("PASS", "FAIL"):
            raise SystemExit(
                f"error: {check} reported {status}, which is neither PASS nor "
                "FAIL. Nothing about F or G can be read off a check that did "
                "not run."
            )
        if status == "PASS":
            if flip is None:
                if not saw_fail:
                    raise SystemExit(
                        f"error: {check} passed at {value}, the lowest value "
                        "swept, with no FAIL beneath it. A flip point needs a "
                        "red direction to bracket it, not just a green one; "
                        "widen BELOW in this file so the sweep reaches one."
                    )
                flip = value
        else:
            saw_fail = True
            if flip is not None:
                raise SystemExit(
                    f"error: {check} is red at {value} and green below it. The "
                    "sweep is not monotonic, so there is no flip point to "
                    "report."
                )
    if flip is None:
        raise SystemExit(
            f"error: {check} is red at every value swept, so the flip point is "
            f"above the range. The figure has moved by more than {ABOVE}; widen "
            "ABOVE in this file before it can say by how much."
        )
    return flip

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--genchecks",
        default=GENCHECKS_DEFAULT,
        help="generator to run against checks.cfg; overridable so a probe can "
        "substitute one that reproduces a drifted [depth] field order",
    )
    args = parser.parse_args()

    if os.path.realpath(os.getcwd()) != os.path.realpath(HERE):
        print(f"error: run from {HERE}, not {os.getcwd()}", file=sys.stderr)
        return 1

    derived = depth_rules.read_derived(CFG)
    f_declared, g_declared = derived["F"], derived["G"]

    print(
        "Re-measuring F and G against formal/checks.cfg, under the interrupt\n"
        "tie-off formal/check-interrupt-tie-off.py enforces.\n"
        f"Declared: F = {f_declared}, G = {g_declared}."
    )

    flip = sweep(
        "F -- worst-case first retire, from `hang`'s check cycle:",
        "hang",
        [
            (cycle, f"check cycle {cycle}", f"hang     1     {cycle}")
            for cycle in range(max(1, f_declared - BELOW), f_declared + 1 + ABOVE)
        ],
        args.genchecks,
    )
    # rvfi_hang_check.sv asserts a registered flag, so it first holds one cycle after the
    # last cycle a trace can go without retiring.
    f_measured = flip - 1
    print(f"  => flip point {flip}, so F = {f_measured}")

    g_measured = None
    for trig in TRIGS:
        flip = sweep(
            f"G -- worst-case retire gap, from `liveness` at trig {trig}:",
            "liveness_ch0",
            [
                (
                    gap,
                    f"gap {gap} (check cycle {trig + gap})",
                    f"liveness 1  {trig} {trig + gap}",
                )
                for gap in range(max(1, g_declared - BELOW), g_declared + 1 + ABOVE)
            ],
            args.genchecks,
        )
        print(f"  => flip point {flip}, so G = {flip}")
        if g_measured is not None and flip != g_measured:
            print(
                f"\nG is {g_measured} at trig {TRIGS[0]} and {flip} at trig "
                f"{trig}. The worst gap is the larger, but a figure that "
                "depends on\nwhere it was asked is not the figure the depths "
                "were derived from -- find out why before using either.",
                file=sys.stderr,
            )
            return 1
        g_measured = flip

    print(
        f"\nMeasured: F = {f_measured}, G = {g_measured}. "
        f"Declared: F = {f_declared}, G = {g_declared}."
    )
    if (f_measured, g_measured) == (f_declared, g_declared):
        print(
            f"Both reproduce. F + G = {f_declared + g_declared}, "
            f"F + 2G = {f_declared + 2 * g_declared}."
        )
        return 0

    print(
        "\nThe declaration is stale. Update the `#derive` lines in "
        "formal/checks.cfg and\nrun `make -C formal checks`, which grades every "
        "[depth] entry against them and\nnames the ones that now need more "
        "depth. Depths are never trimmed to fit a\nruntime budget.",
        file=sys.stderr,
    )
    return 1

if __name__ == "__main__":
    sys.exit(main())
