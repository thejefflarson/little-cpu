#!/usr/bin/env python3
"""Forces nano/formal/complete.sv's own cover goals to go unreached, and requires
nano/formal/complete_cover.sby to fail because of it rather than pass regardless of
the environment.

Usage: complete-cover-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. complete_cover.sby exists so a green `complete` (mode bmc) carries
evidence: complete.sv states a `cover property` for every opcode class nano's walk is
meant to reach, and complete_cover proves each one is reachable. A cover job that
reaches its goals under the shipping design says nothing on its own -- it must also be
shown incapable of passing when a retire genuinely cannot happen, the same demand
make probe-gates makes of every other graded comparison.

One mutant is built, one line of complete.sv away from the shipping harness:
`stalled-bus` assumes mem_ready is always low, so nano.v can never leave its wait
state, rvfi_valid never rises, and every cover property in complete.sv must go
unreached. The unmutated harness is not built here: it is what
`make -C nano/formal complete_cover` proves, and this file is a prerequisite of it.

NOT HERMETIC -- it runs sby, up to twice. So it is a prerequisite of
`make -C nano/formal complete_cover` rather than of `make test`, the same reason
pcloop_cover and traps-region-probe are: a control that can be run separately from the
thing it controls eventually is not run at all. test/probe_gates.sh covers this file's
own logic against a stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

COVER_LINE = re.compile(r"^\s*cover property \(")
# sby names a cover statement by a SOURCE RANGE -- `complete.sv:<line>.<col>-<line>.<col>`
# -- whose start sits on the line BEFORE the statement. Both sets below are read with this
# one pattern, so whatever convention sby uses cancels out of the comparison.
COVER_SITE = re.compile(
    r"(?P<un>[Uu]n)?[Rr]eached cover statement.*?complete\.sv:(?P<site>[\d.]+-[\d.]+)"
)

# Grown in place rather than inserted as a new line: every cover property below is
# pinned by line number, and a new line would shift them all by one.
STALLED_BUS_ANCHOR = "  logic trap;\n"


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def cover_lines(complete_sv):
    """1-based line numbers of every `cover property` complete.sv states."""
    return [n for n, line in enumerate(complete_sv.splitlines(), 1) if COVER_LINE.match(line)]


def mutate(complete_sv):
    """complete.sv with mem_ready assumed low, so no bus transaction ever completes."""
    if STALLED_BUS_ANCHOR not in complete_sv:
        stop(
            "nano/formal/complete.sv no longer spells what the stalled-bus mutation\n"
            "replaces. Re-anchor it on the new spelling -- left alone it would build\n"
            "the shipping harness and prove nothing about a stalled bus."
        )
    grown = "  logic trap; always_comb assume(mem_ready == 1'b0);\n"
    return complete_sv.replace(STALLED_BUS_ANCHOR, grown, 1)


def build_case(repo, root, complete_sv):
    """A copy of nano/formal, deep enough that complete_cover.sby's own relative
    paths (../nano.v, ../../formal/riscv-formal) resolve, with complete.sv replaced.
    Returns the directory sby must be run from."""
    shutil.rmtree(root, ignore_errors=True)
    nano_formal = root / "nano" / "formal"
    nano_formal.mkdir(parents=True)
    shutil.copy(
        repo / "nano" / "formal" / "complete_cover.sby", nano_formal / "complete_cover.sby"
    )
    (nano_formal / "complete.sv").write_text(complete_sv)
    shutil.copy(repo / "nano" / "nano.v", root / "nano" / "nano.v")
    riscv_formal = repo / "formal" / "riscv-formal"
    if not riscv_formal.is_dir():
        stop(
            f"{riscv_formal} is missing. Fetch the pin first, e.g. by running\n"
            "make -C nano/formal complete once."
        )
    (root / "formal").mkdir()
    (root / "formal" / "riscv-formal").symlink_to(riscv_formal)
    return nano_formal


def run_case(repo, workdir, sby, case, complete_sv):
    """Builds the case, runs sby's cover job, and returns (status, unreached lines)."""
    nano_formal = build_case(repo, workdir / case, complete_sv)
    proc = subprocess.run(
        [sby, "-f", "complete_cover.sby"], cwd=nano_formal, capture_output=True, text=True
    )
    status_file = nano_formal / "complete_cover" / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} case, so nothing was proved or\n"
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} case is empty.")
    log = (nano_formal / "complete_cover" / "logfile.txt").read_text()
    sites = {"reached": set(), "unreached": set()}
    for m in COVER_SITE.finditer(log):
        sites["unreached" if m.group("un") else "reached"].add(m.group("site"))
    return status[0], sites


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo", default=str(here.parent.parent), help="tree to read nano/ and formal/ from"
    )
    parser.add_argument("--workdir", default=str(here / "complete-cover-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("nano/formal/complete_cover.sby", "nano/formal/complete.sv", "nano/nano.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    complete_sv = (repo / "nano" / "formal" / "complete.sv").read_text()
    goals = cover_lines(complete_sv)
    if not goals:
        stop(
            "nano/formal/complete.sv states no `cover property` line, so there is\n"
            "nothing for this control to reach."
        )

    red = []

    status, ship = run_case(repo, workdir, args.sby, "shipping", complete_sv)
    reached = sorted(ship["reached"])
    print(f"shipping: {status}, reached {len(reached)} of {len(goals)} goals, "
          f"unreached {sorted(ship['unreached']) or 'none'}")
    if len(reached) != len(goals):
        red.append(
            f"the shipping harness reached {len(reached)} cover sites but complete.sv\n"
            f"states {len(goals)} `cover property` lines. The goal set this control\n"
            "grades is not the goal set the harness has.")
    if status != "PASS":
        red.append(
            "the shipping harness does not reach every cover goal. That is what\n"
            "make -C nano/formal complete_cover is meant to prove about the design\n"
            "as it ships, so a control that starts red proves nothing about a mutant."
        )
    elif ship["unreached"]:
        red.append("the shipping harness reported PASS but still lists unreached goals "
                   f"{sorted(ship['unreached'])}.")

    status, mut = run_case(repo, workdir, args.sby, "stalled-bus", mutate(complete_sv))
    print(f"stalled-bus: {status}, unreached {len(mut['unreached'])} of {len(reached)} goals")
    if status != "FAIL":
        red.append(
            "the stalled-bus mutant proves. Assuming mem_ready low forever is exactly\n"
            "what should make every retire-gated cover goal unreachable, so an\n"
            "anti-vacuity control that cannot go red is not a control."
        )
    elif mut["unreached"] != set(reached):
        missed = sorted(set(reached) - mut["unreached"])
        red.append(
            f"the stalled-bus mutant left {sorted(mut['unreached'])} unreached, which is\n"
            f"not every site the shipping harness reached. Still reached under a stalled\n"
            f"bus: {missed} -- a cover job red for some other reason says nothing about\n"
            "whether a stalled bus reaches this harness's own goals."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print(
        "The stalled-bus mutant makes every cover goal unreachable, and the shipping "
        "harness reaches every one."
    )


if __name__ == "__main__":
    main()
