#!/usr/bin/env python3
"""Forces formal/traps.sv's load/store region arms to fail, and requires each to
fail as its own arm rather than as anything else.

Usage: traps-region-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. formal/traps.sv states two things about a plain load or store
at an address its map does not answer: that the core must trap, and that the
cause must be 5 for a load and 7 for a store. Both are arms of proofs that pass,
and an arm in that position is worth nothing until it has been shown to fail --
which is what `make probe-gates` demands of every other graded comparison in this
tree and what this file does for the two that need a solver.

Two cores are built, each two lines of rtl/decoder.v away from the shipping one,
and each faults an aligned `lw` whose address has bit 31 set -- an address
outside all four windows of any map this platform can be given -- and each must
turn `make -C formal components_traps` red at a named assertion:

  no-trap      still waits for the region answer and then never faults on it, so
               the trap the model requires does not happen. The proof must go
               FAIL at the `assert(trap_entry)` under `expected_trap`. This is
               the arm that moved from may-trap to must-trap when the core
               started raising these two causes, and it is the one a model that
               only required the cause would have been satisfied by.

               IT DROPS THE FAULT RATHER THAN THE COMMIT, and the difference is
               new: the composed task reads the decoder with `-formal` now, so a
               core that computes a cause and does not commit it breaks the
               decoder's own `!trap_taken => trap_cause == 0` at step 3 -- in
               decoder.v, which is not this arm and not even this file.
  wrong-cause  swaps the two causes -- cause 7 for a load and 5 for a store --
               and the proof must go FAIL at the mcause comparison. A core that
               faults the right access with the wrong cause is what that arm
               exists to catch.

The unmutated core is not built here: it is what `components_traps` proves, and
this file is a prerequisite of that target.

Each failing assertion is pinned by line, read out of traps.sv here rather than
written down: a probe that only checked the status would be satisfied by a proof
that went red for an unrelated reason, which is the failure mode this whole
mechanism is about.

NOT HERMETIC -- it runs sby twice, about six seconds. So it is a prerequisite of
`make -C formal components_traps` rather than of `make test`, for the reason
pcloop_cover is one: a control that can be run separately from the thing it
controls eventually is not run at all. test/probe_gates.sh covers this file's own
comparisons, against a stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from traps_probe_sby import SOURCES, probe_sby  # noqa: E402

# The two comparisons being probed, found in traps.sv by their text. Each is the
# only statement in that file that says what it says.
ASSERTS = {
    "no-trap": "assert(trap_entry);",
    "wrong-cause": "assert(csr_rdata == prev_cause);",
}

# The lines of rtl/decoder.v each mutation replaces, matched in full so a
# respelling stops this file rather than silently probing nothing. The value is
# what the key is replaced by; an empty replacement is not allowed, because a
# mutation that deletes nothing builds the shipping core twice.
MUTATIONS = {
    "no-trap": (
        """  assign ls_fault = ls_access && ls_answer_valid && !ls_answer &&
                    !load_misaligned && !store_misaligned;
""",
        """  assign ls_fault = 1'b0;
""",
    ),
    "wrong-cause": (
        """  assign load_access_fault  = (atomic_fault && instr_lr) || (ls_fault && instr_ls_load);
  assign store_access_fault = (atomic_fault && instr_atomic_write) ||
                              (ls_fault && instr_ls_store);
""",
        """  assign load_access_fault  = (atomic_fault && instr_lr) || (ls_fault && instr_ls_store);
  assign store_access_fault = (atomic_fault && instr_atomic_write) ||
                              (ls_fault && instr_ls_load);
""",
    ),
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof.

    A status of its own, because the two are acted on differently: 1 says the
    arm does not behave, and 2 says this file could not ask.
    """
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def assert_line(traps_sv, case):
    """The line traps.sv states this case's assertion on, 1-based."""
    needle = ASSERTS[case]
    hits = [n for n, line in enumerate(traps_sv.splitlines(), 1) if needle in line]
    if len(hits) != 1:
        stop(
            f"formal/traps.sv states `{needle}` {len(hits)} times, and this probe\n"
            "pins the failing assertion by its line. Teach it the new spelling rather\n"
            "than dropping the assertion: a probe that only reads the status passes\n"
            "for a proof that went red somewhere else entirely."
        )
    return hits[0]


def mutate(decoder_v, case):
    """rtl/decoder.v with this case's one line replaced."""
    old, new = MUTATIONS[case]
    if old not in decoder_v:
        stop(
            f"rtl/decoder.v no longer spells what the {case} mutation replaces.\n"
            "Re-anchor it on the new spelling -- left alone it would build the\n"
            "shipping core and report that an arm which was never exercised is fine."
        )
    if old == new:
        stop(
            f"the {case} mutation replaces its text with itself, so the core below\n"
            "would be the shipping one and the proof would say nothing."
        )
    return decoder_v.replace(old, new, 1)


def run_case(repo, workdir, sby, case):
    """Builds the mutated tree, runs sby, and returns (status, failing lines)."""
    root = workdir / case
    shutil.rmtree(root, ignore_errors=True)
    (root / "src").mkdir(parents=True)
    for name in SOURCES:
        shutil.copy(repo / "rtl" / name, root / "src" / name)
    shutil.copy(repo / "formal" / "traps.sv", root / "src" / "traps.sv")
    decoder = (repo / "rtl" / "decoder.v").read_text()
    (root / "src" / "decoder.v").write_text(mutate(decoder, case))
    (root / "probe.sby").write_text(probe_sby(repo, stop))

    # sby's own exit status is not read: FAIL is the required outcome of one of
    # the two cases, and a non-zero status there says nothing this file does not
    # read out of the workdir instead.
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
    parser.add_argument("--workdir", default=str(here / "region-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("formal/traps.sv", "rtl/decoder.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    traps_sv = (repo / "formal" / "traps.sv").read_text()
    red = []

    for case in ("no-trap", "wrong-cause"):
        line = assert_line(traps_sv, case)
        print(f"formal/traps.sv states {case}'s assertion on line {line}.")
        status, failed = run_case(repo, workdir, args.sby, case)
        print(f"  {case}: {status}, assertions failed at {failed or 'none'}")
        if status != "FAIL":
            red.append(
                f"the {case} core proves. That mutation is exactly what the arm was\n"
                "written to catch, so an arm that admits it is asking nothing at all."
            )
        elif line not in failed:
            red.append(
                f"the {case} core went red at {failed}, which does not include line\n"
                f"{line} -- the assertion this case is about. A proof failing somewhere\n"
                "else is not evidence about this arm."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("Both load/store region arms fail for their own reason.")


if __name__ == "__main__":
    main()
