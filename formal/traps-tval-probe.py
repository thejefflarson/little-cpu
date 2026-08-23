#!/usr/bin/env python3
"""Forces formal/traps.sv's mtval arm to fail, and requires it to fail as that
arm rather than as anything else.

Usage: traps-tval-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. mtval is the one thing a trap saves that no self-reporting
oracle in this tree looks at: riscv-formal ships no spec model for SYSTEM at the
pin, so the generated checks never read it, and the two sim legs see only what a
program chose to load it into. formal/traps.sv's arm is therefore the only
statement that a trap reports the right thing about the right access -- and an
arm nobody has watched fail is worth nothing, which is what `make probe-gates`
demands of every other graded comparison here.

The arm IS reached by the shipping core, unlike the region arm next door, so
this is a different question from that probe's: not "can the model admit the
mechanism" but "does the comparison still separate values". Three cores are
built, each one line of rtl/decoder.v from the shipping one:

  control      the shipping mtval mux, and the proof must PASS. Without it a
               model that had become unprovable for some unrelated reason would
               make both failures below look like evidence.
  wrong-addr   reports rs1 where a data cause must report the effective address.
               The two differ by the instruction's immediate and by nothing
               else, which is exactly the defect a suite whose only misaligned
               access carried a zero offset could not see.
  wrong-word   reports zero where an illegal instruction must report the
               faulting word.

Both mutations must go FAIL at the mtval comparison's own line. That line is
read out of traps.sv here rather than written down: a probe that only checked
the status would be satisfied by a proof that went red somewhere else, which is
the failure mode this whole mechanism is about.

NOT HERMETIC -- it runs sby three times, about ten seconds. So it is a
prerequisite of `make -C formal components_traps` rather than of `make test`,
for the reason pcloop_cover is one: a control that can be run separately from
the thing it controls eventually is not run at all. test/probe_gates.sh covers
this file's own comparisons, against a stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from traps_probe_sby import SOURCES, probe_sby  # noqa: E402

# The comparison being probed, found in traps.sv by its text. It is the only
# statement in that file comparing a CSR read-back against the modelled mtval.
MTVAL_ASSERT = "assert(csr_rdata == prev_tval);"

# One arm of rtl/decoder.v's mtval mux per case, matched in full so a respelling
# stops this file rather than silently probing nothing. Each replacement leaves
# the cause chain alone, so a core built from it reports the right cause about
# the wrong thing -- which is the only shape that isolates this arm.
CASES = {
    "wrong-addr": (
        "      data_fault:        trap_tval = mem_addr_calc;",
        "      data_fault:        trap_tval = reg_rs1;",
    ),
    "wrong-word": (
        "      instr_illegal:     trap_tval = instr;",
        "      instr_illegal:     trap_tval = 32'b0;",
    ),
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof.

    A status of its own, because the two are acted on differently: 1 says the
    arm does not behave, and 2 says this file could not ask.
    """
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mtval_assert_line(traps_sv):
    """The line traps.sv states the mtval comparison on, 1-based."""
    hits = [n for n, line in enumerate(traps_sv.splitlines(), 1) if MTVAL_ASSERT in line]
    if len(hits) != 1:
        stop(
            f"formal/traps.sv states `{MTVAL_ASSERT}` {len(hits)} times, and this\n"
            "probe pins the failing assertion by its line. Teach it the new spelling\n"
            "rather than dropping the comparison: a probe that only reads the status\n"
            "passes for a proof that went red somewhere else entirely."
        )
    return hits[0]


def mutate(decoder_v, case):
    """rtl/decoder.v with one arm of the mtval mux reporting the wrong thing."""
    if case == "control":
        for site, _ in CASES.values():
            if site not in decoder_v:
                stop(
                    "rtl/decoder.v no longer spells its mtval mux the way this probe\n"
                    "patches it. Re-anchor the mutations on the new spelling -- left\n"
                    "alone this file would build the shipping core three times and\n"
                    "report that an arm it never disturbed is fine."
                )
        return decoder_v
    site, replacement = CASES[case]
    if site not in decoder_v:
        stop(
            f"rtl/decoder.v does not contain this probe's {case} line, so the\n"
            "mutation is a no-op and that core would be the shipping one."
        )
    return decoder_v.replace(site, replacement)


def run_case(repo, workdir, sby, config, case):
    """Builds the mutated tree, runs sby, and returns (status, failing lines)."""
    root = workdir / case
    shutil.rmtree(root, ignore_errors=True)
    (root / "src").mkdir(parents=True)
    for name in SOURCES:
        shutil.copy(repo / "rtl" / name, root / "src" / name)
    shutil.copy(repo / "formal" / "traps.sv", root / "src" / "traps.sv")
    decoder = (repo / "rtl" / "decoder.v").read_text()
    (root / "src" / "decoder.v").write_text(mutate(decoder, case))
    (root / "probe.sby").write_text(config)

    # sby's own exit status is not read: FAIL is the required outcome of two of
    # the three cases, and a non-zero status there says nothing this file does
    # not read out of the workdir instead.
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
    parser.add_argument("--workdir", default=str(here / "tval-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("formal/traps.sv", "rtl/decoder.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    # Built once, with the other input checks: the script is an input to this
    # file the way traps.sv and decoder.v are, and a missing one is a reason not
    # to start rather than something to discover per case.
    config = probe_sby(repo, stop)
    line = mtval_assert_line((repo / "formal" / "traps.sv").read_text())
    print(f"formal/traps.sv states the mtval comparison on line {line}.")

    red = []

    status, failed = run_case(repo, workdir, args.sby, config, "control")
    print(f"  control:    {status}, assertions failed at {failed or 'none'}")
    if status != "PASS":
        red.append(
            "the shipping core does not prove, so neither failure below is evidence\n"
            "about the mtval arm. Fix that first: this probe can only read a red\n"
            "result as a finding while the green one holds."
        )

    for case in ("wrong-addr", "wrong-word"):
        status, failed = run_case(repo, workdir, args.sby, config, case)
        print(f"  {case}: {status}, assertions failed at {failed or 'none'}")
        if status != "FAIL":
            red.append(
                f"the {case} core proves. A trap reporting the wrong value is what\n"
                "the mtval arm was written to catch, so an arm that admits it is\n"
                "asking nothing at all."
            )
        elif failed != [line]:
            red.append(
                f"the {case} core went red at {failed}, not at line {line} alone.\n"
                "That is the mtval comparison's line; a proof failing anywhere else\n"
                "is one this probe cannot read as evidence about that arm."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The mtval arm proves for the shipping core and fails on the value.")


if __name__ == "__main__":
    main()
