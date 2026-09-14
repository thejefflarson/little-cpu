#!/usr/bin/env python3
"""Grades a hand-written memcheck .sby's BMC depth against a floor derived from
the same harness's checks.cfg #derive lines (F, G).

dmemcheck.sby and imemcheck.sby are not genchecks-generated, so
genchecks-audit.py's [depth] floors never see them; this is that same
discipline, read independently. A one-retire property (imemcheck's: the
watched address's own retire carries the right word) floors at F+2, and a
two-retire property (dmemcheck's: a store, then a matching load) at F+G+2. F
and G are step indices read off generated checks whose scripts end in
`chformal -early`; these scripts do not run it, so the checker's clocked assert
fires the step AFTER the retire it reads, and a depth counts steps from zero.

Also ties <sby-file>_cover.sby (its mode cover anti-vacuity control) to the
same depth: complete-cover-probe.py grades that tie for complete_cover by
walking per-goal reachable steps, because complete_cover deliberately
searches deeper than complete; a memcheck's cover job has no reason to
search deeper than its own bmc sibling, so a plain depth equality is the
same tie in its simplest form.

Usage: check-memcheck-depth.py <harness-dir> <sby-file> <retires:1|2>
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import depth_rules


def main():
    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} <harness-dir> <sby-file> <retires:1|2>", file=sys.stderr)
        return 2
    harness_dir, sby_name, retires = sys.argv[1], sys.argv[2], sys.argv[3]
    if retires not in ("1", "2"):
        print(f"error: <retires> must be 1 or 2, not {retires!r}", file=sys.stderr)
        return 2

    cfg = os.path.join(harness_dir, "checks.cfg")
    if not os.path.isfile(cfg):
        print(f"error: {cfg} does not exist.", file=sys.stderr)
        return 1
    derived = depth_rules.read_derived(cfg)

    sby_path = os.path.join(harness_dir, sby_name)
    if not os.path.isfile(sby_path):
        print(f"error: {sby_path} does not exist.", file=sys.stderr)
        return 1
    depth = depth_rules.read_sby_depth(sby_path)
    if depth is None:
        print(f"error: {sby_path} declares no `depth NNN` line to grade.", file=sys.stderr)
        return 1

    label = "F+2" if retires == "1" else "F+G+2"
    floor = depth_rules.evaluate(label, derived, start=None, trig=None)

    if depth < floor:
        print(
            f"error: {sby_path}'s depth {depth} is below {label} = {floor} "
            f"(F={derived['F']}, G={derived['G']}). A depth below the floor does\n"
            "not go red -- it goes green having stopped asking -- so this fails "
            "generation instead of the check.",
            file=sys.stderr,
        )
        return 1

    print(f"{sby_path}: depth {depth} >= {label} = {floor} (F={derived['F']}, G={derived['G']})")

    stem = sby_name[: -len(".sby")]
    cover_path = os.path.join(harness_dir, f"{stem}_cover.sby")
    if not os.path.isfile(cover_path):
        print(f"error: {cover_path} does not exist, so its anti-vacuity depth is untied.",
              file=sys.stderr)
        return 1
    cover_depth = depth_rules.read_sby_depth(cover_path)
    if cover_depth is None:
        print(f"error: {cover_path} declares no `depth NNN` line to grade.", file=sys.stderr)
        return 1
    if cover_depth != depth:
        print(
            f"error: {cover_path}'s depth {cover_depth} does not match {sby_path}'s "
            f"depth {depth}. A deeper cover search could pass on a goal {sby_path} "
            "(mode bmc) never examines.",
            file=sys.stderr,
        )
        return 1
    print(f"{cover_path}: depth {cover_depth} == {sby_path}'s depth {depth}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
