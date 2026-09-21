#!/usr/bin/env python3
"""Forces nano/formal/traps.sby's load/store region arms to fail, and requires the
shipping core to pass first.

Usage: traps-region-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. traps.sv states two things about a plain load or store whose
effective address lands outside the RAM window: that nano.v must trap, and that the
cause must be 5 for a load and 7 for a store. Both are arms of a proof that passes,
and an arm in that position is worth nothing until it has been shown to fail --
which is what `make probe-gates` demands of every other graded comparison in this
tree and what this file does for the two that need a solver.

Two cores are built, each a few lines of nano/nano.v away from the shipping one:

  no-trap      ls_in_range is forced true, so an aligned load or store outside the
               RAM window never faults. traps.sv's own independent oracle still
               expects one (it recomputes the window from RAM_BASE/RAM_WORDS, never
               from nano.v's own signals), so `assert(rvfi_trap)` under
               `expected_trap` must go FAIL.
  wrong-cause  swaps the two causes -- 7 for a load and 5 for a store -- and the
               proof must go FAIL at the mcause comparison. A core that faults the
               right access with the wrong cause is what that arm exists to catch.

The unmutated core is not built to prove anything new here: it is what
`components_traps` proves, and this file is a prerequisite of that target. It is
still built once, as the required control -- a probe that never shows the shipping
core passing proves nothing about a mutant failing for the right reason.

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
    "no-trap": (
        """  assign ls_in_range = load_store_address >= RAM_BASE &&
    load_store_address < RAM_BASE + RAM_WORDS * 4;
""",
        """  assign ls_in_range = 1'b1;
""",
    ),
    "wrong-cause": (
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
    end else if (store_region_fault) begin
      trap_cause_value = CAUSE_STORE_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
    end else begin
""",
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_STORE_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
    end else if (store_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
    end else begin
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
    parser.add_argument("--workdir", default=str(here / "region-probe"))
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

    for case in ("no-trap", "wrong-cause"):
        status = run_case(repo, workdir, args.sby, case, mutate(nano_v, case))
        print(f"{case}: {status}")
        if status != "FAIL":
            red.append(
                f"the {case} mutant passes. That mutation is exactly what this arm\n"
                "was written to catch, so an arm that admits it is asking nothing at all."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("Both load/store region arms fail for their own reason, and the shipping core passes.")


if __name__ == "__main__":
    main()
