#!/usr/bin/env python3
"""Forces formal/traps.sv's load/store region cause arm to fail, and requires it
to fail as that arm rather than as anything else.

Usage: traps-region-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. formal/traps.sv models a load or store access fault the way it
models a refused atomic: it does not require the trap -- an address no memory
answers is read as zero here, and the privileged spec only recommends faulting
-- but it does require the CAUSE, 5 for a load and 7 for a store, and it excuses
only an access the map does not answer from `must_not_trap`. Today's core raises
neither cause for a plain access, so that arm is satisfied by a core that never
reaches it. An arm in that position is worth nothing until it has been shown to
fail, which is what `make probe-gates` demands of every other graded comparison
in this tree and what this file does for one that needs a solver.

Two cores are built, each two lines of rtl/decoder.v away from the shipping one,
and each faults an aligned `lw` whose address has bit 31 set -- an address
outside all four windows of any map this platform can be given:

  right-cause  raises cause 5, and the proof must still PASS. This is the half
               that says the model does not simply forbid the mechanism: every
               prototype of precise load/store faults so far turned this proof
               red for the missing model rather than for a defect.
  wrong-cause  raises cause 7 for that same load, and the proof must go FAIL at
               the mcause comparison. A core that faults the right access with
               the wrong cause is the defect this arm exists to catch.

The failing assertion is pinned by line, read out of traps.sv here rather than
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

# The design under proof, exactly formal/components.sby's `traps` task. Read
# WITHOUT -formal, because -formal would compile rtl/decoder.v's own
# assume(in.pc == pc) into the instance and the fetcher below is what answers
# that here.
SBY = """[options]
mode prove

[engines]
smtbmc

[script]
read -sv structs.v fetcher.v decoder.v regsel.v csrs.v
read -sv -formal structs.v traps.sv
prep -top traps

[files]
src/structs.v
src/fetcher.v
src/decoder.v
src/regsel.v
src/csrs.v
src/traps.sv
"""

SOURCES = ("structs.v", "fetcher.v", "decoder.v", "regsel.v", "csrs.v")

# The comparison being probed, found in traps.sv by its text. It is the only
# statement in that file comparing a CSR read-back against the modelled cause.
MCAUSE_ASSERT = "assert(csr_rdata == prev_cause);"

# The two lines of rtl/decoder.v the mutation replaces, matched in full so a
# respelling stops this file rather than silently probing nothing.
FAULT_SITE = """  assign load_access_fault  = atomic_fault && instr_lr;
  assign store_access_fault = atomic_fault && instr_atomic_write;
"""

# The term the trap chain and the mtval mux share. Patching it commits the
# fault AND makes the mutated core report the address it happened to, which is
# what the model requires of a core that raises cause 5 or 7 -- a prototype that
# faulted and reported nothing would go red at the mtval arm instead, which is a
# fact about the prototype and not about the region.
TRAP_SITE = "  assign data_fault = load_misaligned || store_misaligned || atomic_fault;"

# An aligned `lw` at an address with bit 31 set. No window of this machine's map
# reaches there, whatever the four parameters are set to, so the model's region
# terms hold for it and its misalignment terms do not.
FAULT_TERM = "instr_lw && !word_misaligned && mem_addr_calc[31]"

# Which of the two assignments above the term is added to, and so which cause
# the mutated core reports for one and the same access. Each is a line of
# FAULT_SITE, so the two cases differ by exactly that and nothing else.
CASES = {
    "right-cause": "assign load_access_fault  = atomic_fault && instr_lr",
    "wrong-cause": "assign store_access_fault = atomic_fault && instr_atomic_write",
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof.

    A status of its own, because the two are acted on differently: 1 says the
    arm does not behave, and 2 says this file could not ask.
    """
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mcause_assert_line(traps_sv):
    """The line traps.sv states the cause comparison on, 1-based."""
    hits = [n for n, line in enumerate(traps_sv.splitlines(), 1) if MCAUSE_ASSERT in line]
    if len(hits) != 1:
        stop(
            f"formal/traps.sv states `{MCAUSE_ASSERT}` {len(hits)} times, and this\n"
            "probe pins the failing assertion by its line. Teach it the new spelling\n"
            "rather than dropping the comparison: a probe that only reads the status\n"
            "passes for a proof that went red somewhere else entirely."
        )
    return hits[0]


def mutate(decoder_v, case):
    """rtl/decoder.v with one region fault added, at the case's cause."""
    if FAULT_SITE not in decoder_v:
        stop(
            "rtl/decoder.v no longer spells its two access-fault assignments the way\n"
            "this probe patches them. Re-anchor the mutation on the new spelling --\n"
            "left alone it would build the shipping core twice and report that an\n"
            "arm which was never exercised is fine."
        )
    if TRAP_SITE not in decoder_v:
        stop(
            "rtl/decoder.v no longer spells `data_fault` the way this probe patches\n"
            "it. Without that line the mutated core computes a fault it never\n"
            "commits and never reports, so the proof stays green having asked\n"
            "nothing."
        )
    site = FAULT_SITE.replace(CASES[case] + ";", CASES[case] + " || probe_region_fault;")
    if site == FAULT_SITE:
        stop(
            f"this probe's own {case} line is not one of the two it patches, so the\n"
            "mutation is a no-op and both cores below would be the shipping one."
        )
    body = (
        "  logic probe_region_fault;\n"
        f"  assign probe_region_fault = {FAULT_TERM};\n" + site
    )
    out = decoder_v.replace(FAULT_SITE, body)
    return out.replace(
        TRAP_SITE,
        "  assign data_fault = load_misaligned || store_misaligned || atomic_fault ||\n"
        "                      probe_region_fault;",
    )


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
    (root / "probe.sby").write_text(SBY)

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

    line = mcause_assert_line((repo / "formal" / "traps.sv").read_text())
    print(f"formal/traps.sv states the cause comparison on line {line}.")

    red = []

    status, failed = run_case(repo, workdir, args.sby, "right-cause")
    print(f"  right-cause: {status}, assertions failed at {failed or 'none'}")
    if status != "PASS":
        red.append(
            "the right-cause core does not prove. The model must ADMIT a core that\n"
            "faults an access no memory answers, or it is the reason a prototype of\n"
            "one goes red rather than a check on it."
        )

    status, failed = run_case(repo, workdir, args.sby, "wrong-cause")
    print(f"  wrong-cause: {status}, assertions failed at {failed or 'none'}")
    if status != "FAIL":
        red.append(
            "the wrong-cause core proves. A load faulting with cause 7 is what the\n"
            "region arm was written to catch, so an arm that admits it is asking\n"
            "nothing at all."
        )
    elif failed != [line]:
        red.append(
            f"the wrong-cause core went red at {failed}, not at line {line} alone.\n"
            "That is the mcause comparison's line; a proof failing anywhere else is\n"
            "one this probe cannot read as evidence about the region arm."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The load/store region cause arm admits the mechanism and fails on the cause.")


if __name__ == "__main__":
    main()
