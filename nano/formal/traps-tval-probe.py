#!/usr/bin/env python3
"""Forces nano/formal/traps.sby's mtval arm to fail, and requires it to fail as that
arm rather than as anything else.

Usage: traps-tval-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. mtval is the one thing a trap saves that no self-reporting oracle
in this tree looks at: riscv-formal ships no spec model for SYSTEM at the pin, so
the generated checks never read it, and the two sim legs see only what a program
chose to load it into. traps.sv's arm is therefore the only statement that a trap
reports the right thing about the right access -- and an arm nobody has watched
fail is worth nothing, which is what `make probe-gates` demands of every other
graded comparison here.

Two cores are built, each one line of nano/nano.v from the shipping one:

  wrong-addr   the load-region-fault arm reports rs1 where it must report the
               effective address. The two differ by the instruction's immediate
               and by nothing else, which is exactly the defect a suite whose only
               out-of-window load carried a zero offset could not see.
  wrong-value  the load-misaligned arm reports zero where it must report the
               address that was actually misaligned.

Both mutations must go FAIL at the mtval comparison. The shipping core is built
once more as the required control, the same reason traps-region-probe.py's own
control exists: a probe that never shows the shipping core passing proves nothing
about a mutant failing for the right reason.

NOT HERMETIC -- it runs sby three times, at nano/formal/traps.sby's own depth.
So it is a prerequisite of `make -C nano/formal components_traps` rather than of
`make test`.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys

MUTATIONS = {
    "wrong-addr": (
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
""",
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = `RF_RS1;
""",
    ),
    "wrong-value": (
        """    end else if (load_misaligned) begin
      trap_cause_value = CAUSE_LOAD_MISALIGNED;
      trap_tval_value  = load_store_address;
""",
        """    end else if (load_misaligned) begin
      trap_cause_value = CAUSE_LOAD_MISALIGNED;
      trap_tval_value  = 32'b0;
""",
    ),
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mutate(nano_v, case):
    old, new = MUTATIONS[case]
    if old not in nano_v:
        stop(
            f"nano/nano.v no longer spells what the {case} mutation replaces.\n"
            "Re-anchor it on the new spelling -- left alone it would build the\n"
            "shipping core and report that an arm which was never exercised is fine."
        )
    return nano_v.replace(old, new, 1)


def build_case(repo, root, nano_v):
    """A copy of nano/formal deep enough that traps.sby's own relative paths
    (../nano.v, ../../formal/riscv-formal) resolve, with nano.v replaced."""
    shutil.rmtree(root, ignore_errors=True)
    nano_formal = root / "nano" / "formal"
    nano_formal.mkdir(parents=True)
    shutil.copy(repo / "nano" / "formal" / "traps.sby", nano_formal / "traps.sby")
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


def run_case(repo, workdir, sby, case, nano_v):
    nano_formal = build_case(repo, workdir / case, nano_v)
    proc = subprocess.run(
        [sby, "-f", "traps.sby"], cwd=nano_formal, capture_output=True, text=True
    )
    status_file = nano_formal / "traps" / "status"
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
    parser.add_argument("--workdir", default=str(here / "tval-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("nano/formal/traps.sby", "nano/formal/traps.sv", "nano/nano.v"):
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
            "the shipping core does not pass traps.sby. That is what\n"
            "make -C nano/formal components_traps is meant to prove about the design\n"
            "as it ships, so a control that starts red proves nothing about a mutant."
        )

    for case in ("wrong-addr", "wrong-value"):
        status = run_case(repo, workdir, args.sby, case, mutate(nano_v, case))
        print(f"{case}: {status}")
        if status != "FAIL":
            red.append(
                f"the {case} mutant passes. That mutation is exactly what the mtval\n"
                "arm was written to catch, so an arm that admits it is asking nothing at all."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("Both mtval mutants fail, and the shipping core passes.")


if __name__ == "__main__":
    main()
