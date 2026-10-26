#!/usr/bin/env python3
"""Forces formal/traps.sv's vectored-target arm to fail, and requires it to fail as that
arm rather than as anything else.

Usage: traps-vector-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. A vectored mtvec sends the timer interrupt to BASE + 0x1C, and no
generated check looks at it: riscv-formal has no spec model for an interrupt or for
SYSTEM. formal/traps.sv's arm is the only statement of the vectored target, and an arm
nobody has watched fail asks nothing.

Two cores are built, one line of rtl/executor.v apart:

  control      the shipping offset, and the proof must PASS.
  to-base      the interrupt enters at BASE under a vectored mtvec: the offset term
               is forced to zero.

The mutation must fail at the vectored comparison's own line, which is read out of
traps.sv here rather than written down.

The basecase leg does not reach a vectored interrupt, so the verdict is UNKNOWN (basecase
pass, induction fail); the failing assertion's line is what is read, never the status.

NOT HERMETIC -- it runs sby twice. It is a prerequisite of `make -C formal
components_traps`. test/probe_gates.sh covers this file's own comparisons against a
stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from traps_probe_sby import SOURCES, probe_sby  # noqa: E402

# The comparison being probed, found in traps.sv by its text.
VECTOR_ASSERT = "assert(fetch_pc == {prev2_mtvec[31:2], 2'b00} + (prev2_mtvec[0] ? 32'h1C : 32'h0));"

# The offset term of rtl/executor.v, matched in full so a respelling stops this file
# rather than silently probing nothing.
SITE = "assign vector_offset = interrupt_pending && mtvec[0] ? CAUSE_MACHINE_TIMER[3:0] : 4'b0;"
CASES = {
    "to-base": (SITE, "assign vector_offset = 4'b0;"),
}

def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof.

    A status of its own, because the two are acted on differently: 1 says the
    arm does not behave, and 2 says this file could not ask.
    """
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)

def vector_assert_line(traps_sv):
    """The line traps.sv states the vectored comparison on, 1-based."""
    hits = [n for n, line in enumerate(traps_sv.splitlines(), 1) if VECTOR_ASSERT in line]
    if len(hits) != 1:
        stop(
            f"formal/traps.sv states `{VECTOR_ASSERT}` {len(hits)} times, and this\n"
            "probe pins the failing assertion by its line. Teach it the new spelling\n"
            "rather than dropping the comparison: a probe that only reads the status\n"
            "passes for a proof that went red somewhere else entirely."
        )
    return hits[0]

def mutate(executor_v, case):
    """rtl/executor.v with one arm of the vector mux reporting the wrong thing."""
    if case == "control":
        for site, _ in CASES.values():
            if site not in executor_v:
                stop(
                    "rtl/executor.v no longer spells its vector mux the way this probe\n"
                    "patches it. Re-anchor the mutations on the new spelling -- left\n"
                    "alone this file would build the shipping core twice and\n"
                    "report that an arm it never disturbed is fine."
                )
        return executor_v
    site, replacement = CASES[case]
    if site not in executor_v:
        stop(
            f"rtl/executor.v does not contain this probe's {case} line, so the\n"
            "mutation is a no-op and that core would be the shipping one."
        )
    return executor_v.replace(site, replacement)

def run_case(repo, workdir, sby, config, case):
    """Builds the mutated tree, runs sby, and returns (status, failing lines)."""
    root = workdir / case
    shutil.rmtree(root, ignore_errors=True)
    (root / "src").mkdir(parents=True)
    for name in SOURCES:
        shutil.copy(repo / "rtl" / name, root / "src" / name)
    shutil.copy(repo / "formal" / "traps.sv", root / "src" / "traps.sv")
    executor = (repo / "rtl" / "executor.v").read_text()
    (root / "src" / "executor.v").write_text(mutate(executor, case))
    (root / "probe.sby").write_text(config)

    # sby's own exit status is not read: FAIL is the required outcome of the mutated case,
    # and a non-zero status there says nothing this file does not read out of the
    # workdir instead.
    proc = subprocess.run(
        [sby, "-f", "probe.sby"], cwd=root, capture_output=True, text=True
    )
    status_file = root / "probe" / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} core, so nothing was proved or\n"
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} core is empty.")
    log = (root / "probe" / "logfile.txt").read_text()
    failed = sorted(set(int(n) for n in re.findall(r"Assert failed in traps: traps\.sv:(\d+)", log)))
    return status[0], failed

def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent), help="tree to read the RTL from")
    parser.add_argument("--workdir", default=str(here / "vector-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("formal/traps.sv", "rtl/executor.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    config = probe_sby(repo, stop)
    line = vector_assert_line((repo / "formal" / "traps.sv").read_text())
    print(f"formal/traps.sv states the vectored comparison on line {line}.")

    red = []

    status, failed = run_case(repo, workdir, args.sby, config, "control")
    print(f"  control:    {status}, assertions failed at {failed or 'none'}")
    if status != "PASS":
        red.append(
            "the shipping core does not prove, so neither failure below is evidence\n"
            "about the vector arm. Fix that first: this probe can only read a red\n"
            "result as a finding while the green one holds."
        )

    for case in ("to-base",):
        status, failed = run_case(repo, workdir, args.sby, config, case)
        print(f"  {case}: {status}, assertions failed at {failed or 'none'}")
        if status == "PASS":
            red.append(
                f"the {case} core proves. An interrupt entering at BASE under a vectored\n"
                "mtvec is what the vector arm was written to catch, so an arm that\n"
                "admits it is asking nothing at all."
            )
        elif failed != [line]:
            red.append(
                f"the {case} core went red at {failed}, not at line {line} alone.\n"
                "That is the vectored comparison's line; a proof failing anywhere else\n"
                "is one this probe cannot read as evidence about that arm."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The vector arm proves for the shipping core and fails on the target.")

if __name__ == "__main__":
    main()
