#!/usr/bin/env python3
#
# Re-measures F and G -- the two figures every depth in formal/checks.cfg's
# [depth] table is derived from -- and grades what it measures against the
# `#derive` lines that declare them.
#
# WHY A COMMAND AND NOT A RECIPE IN PROSE. The procedure used to be four
# sentences telling a reader to copy the cfg, cut [depth] down to one line,
# regenerate and sweep the last column. It was carried out correctly each time
# it was carried out at all, which is the problem: the depths it feeds are the
# reason every generated check is worth its PASS, and a depth that has gone
# stale reports nothing.
#
# BOTH DIRECTIONS, ALWAYS. Each sweep prints the red rows either side of its
# flip point as well as the green one, because a flip point with no counter-
# example under it is a measurement that could not have come out any other way:
# `hang` red at F and PASS at F+1, `liveness` red at gap G-1 and PASS at gap G.
# The gap-(G-1) counterexample is also this measurement's non-vacuity witness,
# since rvfi_liveness_check.sv opens with `assume(rvfi_valid)` at its trig
# cycle.
#
# The probe cfg is checks.cfg with [depth] replaced by one line, so everything
# else -- RISCV_FORMAL_ALTOPS, the solver, the source list, formal/wrapper.v's
# tie-off of the timer interrupt -- is what the graded run uses. G in particular
# is only this number under the tie-off: an interrupt costs a cycle that would
# otherwise have issued.

import os
import shutil
import subprocess
import sys

import depth_rules

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "checks.cfg")
PROBE = "fg-probe"

# The two trigs G is taken at. One is a measurement; two agreeing is a
# measurement whose answer does not depend on where in the trace it was asked.
TRIGS = (10, 15)

# How far either side of the declared figure to sweep. Two red rows below the
# flip point and one green row above it.
BELOW, ABOVE = 2, 1


def probe(depth_line, check):
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
        [sys.executable, "genchecks-local.py", PROBE],
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


def sweep(label, check, rows):
    """Run one sweep of `check` and return the lowest value that PASSes, or
    None. `rows` is a list of (value, description, depth_line)."""
    print(f"\n{label}")
    flip = None
    for value, description, depth_line in rows:
        status = probe(depth_line, check)
        print(f"  {description:<28} {status}")
        if status not in ("PASS", "FAIL"):
            raise SystemExit(
                f"error: {check} reported {status}, which is neither PASS nor "
                "FAIL. Nothing about F or G can be read off a check that did "
                "not run."
            )
        if status == "PASS":
            if flip is None:
                flip = value
        elif flip is not None:
            raise SystemExit(
                f"error: {check} is red at {value} and green below it. The "
                "sweep is not monotonic, so there is no flip point to report."
            )
    if flip is None:
        raise SystemExit(
            f"error: {check} is red at every value swept, so the flip point is "
            f"above the range. The figure has moved by more than {ABOVE}; widen "
            "ABOVE in this file before it can say by how much."
        )
    return flip


def main():
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
    )
    # rvfi_hang_check.sv asserts a registered flag, so it first holds one cycle
    # after the last cycle a trace can go without retiring.
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
