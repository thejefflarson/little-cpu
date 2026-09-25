#!/usr/bin/env python3
"""Shared driver for nano/formal/traps-region-probe.py and traps-tval-probe.py: both
build a mutated nano.v a few lines from the shipping one, prove nano/formal/traps.sby
against it, and require the shipping core to pass while every mutant fails. Only the
mutation table, the case names and the printed messages differ between the two.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mutate(nano_v, mutations, case):
    old, new = mutations[case]
    if old not in nano_v:
        stop(
            f"nano/nano.v no longer spells what the {case} mutation replaces.\n"
            "Re-anchor it on the new spelling -- left alone it would build the\n"
            "shipping core and report that an arm which was never exercised is fine."
        )
    return nano_v.replace(old, new, 1)


def build_case(repo, root, nano_v, sby_file):
    """A copy of nano/formal deep enough that sby_file's own relative paths
    (../nano.v, ../../formal/riscv-formal) resolve, with nano.v replaced."""
    shutil.rmtree(root, ignore_errors=True)
    nano_formal = root / "nano" / "formal"
    nano_formal.mkdir(parents=True)
    shutil.copy(repo / "nano" / "formal" / sby_file, nano_formal / sby_file)
    shutil.copy(repo / "nano" / "formal" / "traps.sv", nano_formal / "traps.sv")
    (root / "nano" / "nano.v").write_text(nano_v)
    riscv_formal = repo / "formal" / "riscv-formal"
    if not riscv_formal.is_dir():
        stop(
            f"{riscv_formal} is missing. Fetch the pin first, e.g. by running\n"
            "make -C nano/formal components_traps once."
        )
    (root / "formal").mkdir()
    (root / "formal" / "riscv-formal").symlink_to(riscv_formal)
    return nano_formal


def run_case(repo, workdir, sby, case, nano_v, sby_file):
    nano_formal = build_case(repo, workdir / case, nano_v, sby_file)
    proc = subprocess.run(
        [sby, "-f", sby_file], cwd=nano_formal, capture_output=True, text=True
    )
    status_file = nano_formal / sby_file[: -len(".sby")] / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} case, so nothing was proved or\n"
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} case is empty.")
    return status[0]


def main(doc, mutations, workdir_name, arm_noun, success_message):
    """`doc` becomes --help text; `arm_noun` names what a surviving mutant admits
    nothing about, e.g. "this arm" or "the mtval arm"."""
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=doc)
    parser.add_argument(
        "--repo", default=str(here.parent.parent), help="tree to read nano/ and formal/ from"
    )
    parser.add_argument("--workdir", default=str(here / workdir_name))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()
    sby_file = "traps.sby"

    repo = pathlib.Path(args.repo).resolve()
    for name in (f"nano/formal/{sby_file}", "nano/formal/traps.sv", "nano/nano.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    nano_v = (repo / "nano" / "nano.v").read_text()
    red = []

    status = run_case(repo, workdir, args.sby, "shipping", nano_v, sby_file)
    print(f"shipping: {status}")
    if status != "PASS":
        red.append(
            "the shipping core does not pass traps.sby. That is what\n"
            "make -C nano/formal components_traps is meant to prove about the design\n"
            "as it ships, so a control that starts red proves nothing about a mutant."
        )

    for case in mutations:
        status = run_case(
            repo, workdir, args.sby, case, mutate(nano_v, mutations, case), sby_file
        )
        print(f"{case}: {status}")
        if status != "FAIL":
            red.append(
                f"the {case} mutant passes. That mutation is exactly what {arm_noun}\n"
                "was written to catch, so an arm that admits it is asking nothing at all."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print(success_message)
