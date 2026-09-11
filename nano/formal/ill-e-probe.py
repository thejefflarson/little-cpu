#!/usr/bin/env python3
"""Forces nano/formal/ill_e.sby to fail against a core that gets the RV32E rule wrong,
and requires the shipping core to pass first.

Usage: ill-e-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. The first ill_e checked a hand-written reference against itself, so
every mutation probe it had could only show its OWN assign/assert pair could be broken,
never that the property was falsifiable against a real core -- see the amendment in
docs/adr/0174-the-rv32e-oracle-patch-its-diff-grader-and-the-ill-e-check.md. This ill_e
instead reads nano.v's own RVFI report, so the control this file provides mutates
nano.v ITSELF: `is_e_illegal`'s bit 4 (x16-x31's own bit) becomes bit 3, the exact
"survivable" mutation that left the reverted design's two .sby files both PASSING. A
core built to that wrong rule executes x16 as ordinary (bit 4 clear) rather than
trapping it, which ill_e.sv's own bit-4 check -- read off RVFI, independent of
whatever nano.v's decode does internally -- must catch.

NOT HERMETIC -- it runs sby, twice. Prerequisite of `make -C nano/formal ill_e`, the
same standing complete-cover-probe.py has for complete_cover. test/probe_gates.sh
covers this file's own logic against a stub sby.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys

RULE_LINE = (
    "  assign is_e_illegal = rd[4] || (rs1_valid && rs1[4]) || (rs2_valid && rs2[4]);\n"
)
WRONG_RULE_LINE = (
    "  assign is_e_illegal = rd[3] || (rs1_valid && rs1[3]) || (rs2_valid && rs2[3]);\n"
)


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mutate(nano_v):
    if RULE_LINE not in nano_v:
        stop(
            "nano/nano.v no longer spells is_e_illegal the way this probe mutates.\n"
            "Re-anchor RULE_LINE/WRONG_RULE_LINE on the new spelling -- left alone\n"
            "this would build the shipping core and prove nothing about a wrong rule."
        )
    return nano_v.replace(RULE_LINE, WRONG_RULE_LINE, 1)


def build_case(repo, root, nano_v):
    """A copy of nano/formal deep enough that ill_e.sby's own relative paths
    (../nano.v, ../../formal/riscv-formal) resolve, with nano.v replaced."""
    shutil.rmtree(root, ignore_errors=True)
    nano_formal = root / "nano" / "formal"
    nano_formal.mkdir(parents=True)
    shutil.copy(repo / "nano" / "formal" / "ill_e.sby", nano_formal / "ill_e.sby")
    shutil.copy(repo / "nano" / "formal" / "ill_e.sv", nano_formal / "ill_e.sv")
    (root / "nano" / "nano.v").write_text(nano_v)
    riscv_formal = repo / "formal" / "riscv-formal"
    if not riscv_formal.is_dir():
        stop(
            f"{riscv_formal} is missing. Fetch the pin first, e.g. by running\n"
            "make -C nano/formal ill_e once."
        )
    (root / "formal").mkdir()
    (root / "formal" / "riscv-formal").symlink_to(riscv_formal)
    return nano_formal


def run_case(repo, workdir, sby, case, nano_v):
    nano_formal = build_case(repo, workdir / case, nano_v)
    proc = subprocess.run(
        [sby, "-f", "ill_e.sby"], cwd=nano_formal, capture_output=True, text=True
    )
    status_file = nano_formal / "ill_e" / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} case, so nothing was proved or\n"
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} case is empty.")
    return status[0]


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo", default=str(here.parent.parent), help="tree to read nano/ and formal/ from"
    )
    parser.add_argument("--workdir", default=str(here / "ill-e-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("nano/formal/ill_e.sby", "nano/formal/ill_e.sv", "nano/nano.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    nano_v = (repo / "nano" / "nano.v").read_text()
    red = []

    status = run_case(repo, workdir, args.sby, "shipping", nano_v)
    print(f"shipping: {status}")
    if status != "PASS":
        red.append(
            "the shipping core does not pass ill_e. That is what make -C nano/formal\n"
            "ill_e is meant to prove about the design as it ships, so a control that\n"
            "starts red proves nothing about a wrong-rule mutant."
        )

    status = run_case(repo, workdir, args.sby, "wrong-rule", mutate(nano_v))
    print(f"wrong-rule (bit 4 -> bit 3): {status}")
    if status != "FAIL":
        red.append(
            "the wrong-rule mutant passes. Checking bit 3 instead of bit 4 lets x16\n"
            "retire as ordinary rather than trapping it, so a check that cannot go red\n"
            "against it is not standing between RISCV_FORMAL_E and a broken E core."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The wrong-rule mutant fails ill_e, and the shipping core passes it.")


if __name__ == "__main__":
    main()
