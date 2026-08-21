#!/usr/bin/env python3
"""Forces formal/busarbiter.sv's two load-bearing arms to fail, and requires each
to fail as itself rather than as anything else.

Usage: busarbiter-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. An arbiter's two interesting properties are the ones no program
can test. A fixed-priority arbiter passes every torture program ever written for
it and starves the losing hart anyway; an arbiter that hands the bus away in the
middle of an AMO tears one atomic in a million. Both of those are proved over
there, and a proof of a guarded property is worth what its guard is worth -- so
each arm is made to fail here, by building a core one line away from the
shipping one and requiring the red to land on the arm's own line.

Three cores, two lines of rtl/busarbiter.v between them:

  shipping       the real arbiter. The proof must PASS and the cover must PASS.
                 This is the half that says the two reds below are the
                 mutation's doing and not a harness that reports red at rest.
  fixed-priority a tie always goes to hart 0. The proof must go FAIL at the
                 WAIT BOUND, and must NOT go red at the indivisibility arm: an
                 arbiter that starves a hart is not one that tore an atomic.
  grant-mid-lock the grant is re-arbitrated while a lock is held. The proof must
                 go FAIL at the INDIVISIBILITY arm and not at the wait bound,
                 and the COVER must go FAIL with its lock goal unreached --
                 which is the anti-vacuity control's own red direction, the
                 thing that says it can still see the class it exists for.

The failing lines are pinned, and read out of busarbiter.sv here rather than
written down: a probe that only checked the status would be satisfied by a proof
that went red for an unrelated reason, which is the failure mode this whole
mechanism is about.

NOT HERMETIC -- it runs sby five times, about four seconds. So it is a
prerequisite of `make -C formal components_busarbiter` rather than of `make
test`, for the reason busarbiter_cover.sby is one: a control that can be run
separately from the thing it controls eventually is not run at all.
test/probe_gates.sh covers this file's own comparisons, against a stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

# The two jobs, each a copy of the .sby that ships it -- formal/components.sby's
# `busarbiter` task including its engine, and formal/busarbiter_cover.sby
# including its depth -- paired with the line that job complains on. Read WITH
# -formal, the way both of them do: rtl/busarbiter.v has no `ifdef FORMAL` block,
# so there is no assume of its own to compile into the instance.
JOBS = {
    "prove": (
        """[options]
mode prove

[engines]
smtbmc --keep-going

[script]
read -sv -formal busarbiter.v busarbiter.sv
prep -top busarbiter_check

[files]
src/busarbiter.v
src/busarbiter.sv
""",
        r"Assert failed in busarbiter_check: busarbiter\.sv:(\d+)",
    ),
    "cover": (
        """[options]
mode cover
depth 20

[engines]
smtbmc

[script]
read -sv -formal busarbiter.v busarbiter.sv
prep -top busarbiter_check

[files]
src/busarbiter.v
src/busarbiter.sv
""",
        r"Unreached cover statement at busarbiter_check: busarbiter\.sv:(\d+)",
    ),
}

# The three comparisons being pinned, found in busarbiter.sv by their text.
LOCK_ASSERT = "if (settled && past_grant[h] && past_mem_lock[h]) assert(grant[h]);"
WAIT_ASSERT = "always_comb if (clocked) assert(waited <= BOUND);"
LOCK_COVER = "cover (settled && grant[h] && past_grant[h] && past_mem_lock[h] &&"

# The two lines of rtl/busarbiter.v the mutations replace, matched in full so a
# respelling stops this file rather than silently probing nothing.
TIE_SITE = "  assign winner = (request == 2'b11) ? (grant[0] ? 2'b10 : 2'b01) : request;\n"
HOLD_SITE = "    else grant <= held ? grant : winner;\n"

# Each case's replacement for its site. `shipping` mutates nothing.
CASES = {
    "shipping": None,
    "fixed-priority": (TIE_SITE, "  assign winner = (request == 2'b11) ? 2'b01 : request;\n"),
    "grant-mid-lock": (HOLD_SITE, "    else grant <= winner;\n"),
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof.

    A status of its own, because the two are acted on differently: 1 says an arm
    does not behave, and 2 says this file could not ask.
    """
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def line_of(harness, text, what):
    """The line formal/busarbiter.sv states `text` on, 1-based."""
    hits = [n for n, line in enumerate(harness.splitlines(), 1) if text in line]
    if len(hits) != 1:
        stop(
            f"formal/busarbiter.sv states the {what} {len(hits)} times, and this\n"
            "probe pins the failing line. Teach it the new spelling rather than\n"
            "dropping the comparison: a probe that only reads the status passes for\n"
            "a proof that went red somewhere else entirely."
        )
    return hits[0]


def mutate(arbiter_v, case):
    """rtl/busarbiter.v with this case's one line replaced."""
    if TIE_SITE not in arbiter_v or HOLD_SITE not in arbiter_v:
        stop(
            "rtl/busarbiter.v no longer spells its tie-break and its grant register\n"
            "the way this probe patches them. Re-anchor the mutations on the new\n"
            "spelling -- left alone they would build the shipping core three times\n"
            "and report that arms which were never exercised are fine."
        )
    if CASES[case] is None:
        return arbiter_v
    site, replacement = CASES[case]
    if replacement == site:
        stop(f"this probe's {case} replacement is its own site, so the core is unmutated.")
    return arbiter_v.replace(site, replacement)


def run_case(repo, workdir, sby, case, job):
    """Builds the mutated tree, runs one sby job, and returns (status, lines).

    `lines` is the set of busarbiter.sv lines the run complained about: failing
    assertions for the proof, unreached cover goals for the cover.
    """
    sby_text, pattern = JOBS[job]
    root = workdir / case
    (root / "src").mkdir(parents=True, exist_ok=True)
    shutil.copy(repo / "formal" / "busarbiter.sv", root / "src" / "busarbiter.sv")
    arbiter = (repo / "rtl" / "busarbiter.v").read_text()
    (root / "src" / "busarbiter.v").write_text(mutate(arbiter, case))
    (root / f"{job}.sby").write_text(sby_text)

    # sby's own exit status is not read: FAIL is the required outcome of two of
    # these runs, and a non-zero status there says nothing this file does not
    # read out of the workdir instead.
    proc = subprocess.run(
        [sby, "-f", f"{job}.sby"], cwd=root, capture_output=True, text=True
    )
    status_file = root / job / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} core's {job} run, so nothing was\n"
            "proved or disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} core's {job} run is empty.")
    log = (root / job / "logfile.txt").read_text()
    return status[0], sorted(set(int(n) for n in re.findall(pattern, log)))


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent), help="tree to read the RTL from")
    parser.add_argument("--workdir", default=str(here / "busarbiter-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("formal/busarbiter.sv", "rtl/busarbiter.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    shutil.rmtree(workdir, ignore_errors=True)
    workdir.mkdir(parents=True)

    harness = (repo / "formal" / "busarbiter.sv").read_text()
    lock_line = line_of(harness, LOCK_ASSERT, "indivisibility assertion")
    wait_line = line_of(harness, WAIT_ASSERT, "wait bound assertion")
    cover_line = line_of(harness, LOCK_COVER, "lock cover goal")
    print(
        f"formal/busarbiter.sv: indivisibility on line {lock_line}, wait bound on"
        f" line {wait_line}, lock cover on line {cover_line}."
    )

    red = []

    status, failed = run_case(repo, workdir, args.sby, "shipping", "prove")
    print(f"  shipping       prove: {status}, assertions failed at {failed or 'none'}")
    if status != "PASS":
        red.append(
            "the shipping arbiter does not prove. Whatever the two mutations below\n"
            "report, they report it about a harness that is already red."
        )
    status, unreached = run_case(repo, workdir, args.sby, "shipping", "cover")
    print(f"  shipping       cover: {status}, goals unreached at {unreached or 'none'}")
    if status != "PASS":
        red.append(
            "the shipping arbiter does not reach its own cover goals, so the control\n"
            "for the anti-vacuity control is red and nothing below means anything."
        )

    status, failed = run_case(repo, workdir, args.sby, "fixed-priority", "prove")
    print(f"  fixed-priority prove: {status}, assertions failed at {failed or 'none'}")
    if status != "FAIL":
        red.append(
            "the fixed-priority arbiter proves. A tie that always goes to hart 0\n"
            "starves hart 1, which is the whole reason the wait bound is written\n"
            "down, so a bound that admits it is asking nothing at all."
        )
    elif wait_line not in failed:
        red.append(
            f"the fixed-priority arbiter went red at {failed}, which does not include\n"
            f"line {wait_line}. That is the wait bound's line, and a proof failing\n"
            "anywhere else is not evidence about starvation."
        )
    elif lock_line in failed:
        red.append(
            f"the fixed-priority arbiter went red at line {lock_line} as well. That is\n"
            "the indivisibility arm, which starvation does not touch: the two arms\n"
            "are not telling this probe apart any more."
        )

    status, failed = run_case(repo, workdir, args.sby, "grant-mid-lock", "prove")
    print(f"  grant-mid-lock prove: {status}, assertions failed at {failed or 'none'}")
    if status != "FAIL":
        red.append(
            "the mid-lock arbiter proves. A grant handed away between an AMO's read\n"
            "and its write is exactly what the indivisibility arm was written to\n"
            "catch, so an arm that admits it is asking nothing at all."
        )
    elif failed != [lock_line]:
        red.append(
            f"the mid-lock arbiter went red at {failed}, not at line {lock_line}\n"
            "alone. That is the indivisibility arm's line; a proof failing anywhere\n"
            "else is one this probe cannot read as evidence about it."
        )

    status, unreached = run_case(repo, workdir, args.sby, "grant-mid-lock", "cover")
    print(f"  grant-mid-lock cover: {status}, goals unreached at {unreached or 'none'}")
    if status != "FAIL":
        red.append(
            "the mid-lock arbiter reaches every cover goal. The lock goal cannot be\n"
            "reached by an arbiter that holds nothing back, so a cover that still\n"
            "passes is one that has stopped seeing the class it exists for."
        )
    elif cover_line not in unreached:
        red.append(
            f"the mid-lock arbiter's cover went red without line {cover_line} among\n"
            "the unreached goals. That is the lock goal; a cover failing for some\n"
            "other reason says nothing about whether it can still see a lock."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("Both arms admit the shipping arbiter and fail on their own mutation.")


if __name__ == "__main__":
    main()
