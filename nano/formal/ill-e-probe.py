#!/usr/bin/env python3
"""Forces nano/formal/ill_e.sv's own property and cover goals to go red, and
requires each to go red for the reason it was written for.

Usage: ill-e-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. ill_e.sv checks the E register-naming restriction against a
small reference implementation rather than nano.v -- see that file's header
for why nano.v cannot stand in yet -- and a property checked only against a
reference that was written to satisfy it says nothing on its own. It must
also be shown incapable of passing under a design that gets the restriction
wrong, the same demand `make probe-gates` makes of every other graded
comparison in this tree.

Four mutations, each one line of ill_e.sv away from the shipping file:

  no-trap         ties `trap` to a constant regardless of `e_illegal` --
                  "lets an x16-31 encoding retire normally", the exact defect
                  class this check exists to catch.
  no-rd-clear     ties `rd_addr` to the raw field regardless of `e_illegal`
                  -- an E-illegal retire that still writes a register.
  no-mem-clear    ties `mem_write` to the raw store decode regardless of
                  `e_illegal` -- an E-illegal retire that still writes
                  memory.
  no-illegal      ties `e_illegal` to a constant, so no encoding is ever
                  E-illegal and every `cover property` in ill_e.sv becomes
                  unreachable.

nano/formal/ill_e.sby must go FAIL at each mutation's own assertion line; for
no-illegal, nano/formal/ill_e_cover.sby must go FAIL and name at least one of
the cover sites the shipping file reaches as unreached.

Neither the shipping (unmutated) file nor a mutant proves anything alone: a
mutant that fails is not evidence unless the file it was mutated from is
first shown to pass, or to reach every cover goal.

NOT HERMETIC -- it runs sby up to six times. So it is a prerequisite of
`make -C nano/formal ill_e_cover` rather than of `make test`, the same reason
pcloop_cover and traps-region-probe are: a control that can be run separately
from the thing it controls eventually is not run at all. It is not a
prerequisite of `make -C nano/formal ill_e` alone, the same asymmetry
`complete_cover`'s own probe has against `complete`, so building `ill_e` on
its own does not pay for a solver run twice over. test/probe_gates.sh covers
this file's own logic against a stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

# Each entry: (case name, anchor text, mutated text, the assertion it defeats).
ASSERT_MUTATIONS = {
    "no-trap": (
        "  assign trap      = e_illegal;\n",
        "  assign trap      = 1'b0;\n",
        "assert (trap);",
    ),
    "no-rd-clear": (
        "  assign rd_addr   = e_illegal ? 5'd0 : rd;\n",
        "  assign rd_addr   = rd;\n",
        "assert (rd_addr == 5'd0);",
    ),
    "no-mem-clear": (
        "  assign mem_write = e_illegal ? 1'b0 : class_store;\n",
        "  assign mem_write = class_store;\n",
        "assert (!mem_write);",
    ),
}

ILLEGAL_ANCHOR = (
    "  wire e_illegal = (uses_rd && rd[4]) || (uses_rs1 && rs1[4]) || "
    "(uses_rs2 && rs2[4]);\n"
)
ILLEGAL_MUTANT = "  wire e_illegal = 1'b0;\n"

COVER_LINE = re.compile(r"^\s*cover property \(")
# sby names a cover statement by a SOURCE RANGE, the same convention
# nano/formal/complete-cover-probe.py already reads.
COVER_SITE = re.compile(
    r"(?P<un>[Uu]n)?[Rr]eached cover statement.*?ill_e\.sv:(?P<site>[\d.]+-[\d.]+)"
)


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def assert_line(ill_e_sv, needle):
    """The 1-based line ill_e.sv states this assertion on."""
    hits = [n for n, line in enumerate(ill_e_sv.splitlines(), 1) if needle in line]
    if len(hits) != 1:
        stop(
            f"nano/formal/ill_e.sv states `{needle}` {len(hits)} time(s), and\n"
            "this probe pins the failing assertion by its line. Teach it the\n"
            "new spelling rather than dropping it: a probe that only reads the\n"
            "status passes for a proof that went red somewhere else entirely."
        )
    return hits[0]


def cover_lines(ill_e_sv):
    """1-based line numbers of every `cover property` ill_e.sv states."""
    return [n for n, line in enumerate(ill_e_sv.splitlines(), 1) if COVER_LINE.match(line)]


def mutate(ill_e_sv, anchor, mutant, name):
    if anchor not in ill_e_sv:
        stop(
            f"nano/formal/ill_e.sv no longer spells what the {name} mutation\n"
            "replaces. Re-anchor it on the new spelling -- left alone it would\n"
            "build the shipping file and prove nothing about a mutated core."
        )
    return ill_e_sv.replace(anchor, mutant, 1)


def run_case(workdir, sby, case, ill_e_sv, sby_name, sby_text):
    """Builds the case and runs sby, returning (status, logfile text)."""
    task = sby_name.removesuffix(".sby")
    root = workdir / case
    shutil.rmtree(root, ignore_errors=True)
    root.mkdir(parents=True)
    (root / "ill_e.sv").write_text(ill_e_sv)
    (root / sby_name).write_text(sby_text)

    proc = subprocess.run([sby, "-f", sby_name], cwd=root, capture_output=True, text=True)
    status_file = root / task / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} case, so nothing was proved or\n"
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} case is empty.")
    log = (root / task / "logfile.txt").read_text()
    return status[0], log


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo", default=str(here.parent.parent), help="tree to read nano/formal from"
    )
    parser.add_argument("--workdir", default=str(here / "ill-e-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    names = ("nano/formal/ill_e.sv", "nano/formal/ill_e.sby", "nano/formal/ill_e_cover.sby")
    for name in names:
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    ill_e_sv = (repo / "nano" / "formal" / "ill_e.sv").read_text()
    ill_e_sby = (repo / "nano" / "formal" / "ill_e.sby").read_text()
    ill_e_cover_sby = (repo / "nano" / "formal" / "ill_e_cover.sby").read_text()

    red = []

    # ---- the assertions: a core that lets an x16-31 encoding retire normally ----
    status, _ = run_case(workdir, args.sby, "assert-shipping", ill_e_sv, "ill_e.sby", ill_e_sby)
    print(f"assert shipping: {status}")
    if status != "PASS":
        red.append(
            "the shipping reference fails its own assertions. That is what\n"
            "nano/formal/ill_e.sby is meant to prove about a correct E core, so\n"
            "a control that starts red proves nothing about a mutant."
        )

    for case, (anchor, mutant, needle) in ASSERT_MUTATIONS.items():
        line = assert_line(ill_e_sv, needle)
        mutated = mutate(ill_e_sv, anchor, mutant, case)
        status, log = run_case(workdir, args.sby, case, mutated, "ill_e.sby", ill_e_sby)
        failed = sorted(set(int(n) for n in re.findall(r"Assert failed in ill_e_top: ill_e\.sv:(\d+)", log)))
        print(f"  {case} (line {line}): {status}, failed at {failed or 'none'}")
        if status != "FAIL":
            red.append(
                f"the {case} mutant proves. That mutation is exactly what its own\n"
                "assertion exists to catch, so an arm that admits it is asking\n"
                "nothing at all."
            )
        elif line not in failed:
            red.append(
                f"the {case} mutant went red at {failed}, which does not include\n"
                f"line {line} -- the assertion this case is about. A proof failing\n"
                "somewhere else is not evidence about this arm."
            )

    # ---- the cover goals: shown reachable, and shown to stop being so ----
    goals = cover_lines(ill_e_sv)
    if not goals:
        stop(
            "nano/formal/ill_e.sv states no `cover property` line, so there is\n"
            "nothing for this control to reach."
        )

    status, log = run_case(
        workdir, args.sby, "cover-shipping", ill_e_sv, "ill_e_cover.sby", ill_e_cover_sby
    )
    reached = {m.group("site") for m in COVER_SITE.finditer(log) if not m.group("un")}
    print(f"cover shipping: {status}, reached {len(reached)} of {len(goals)} goals")
    if status != "PASS":
        red.append(
            "the shipping reference does not reach every cover goal. That is what\n"
            "nano/formal/ill_e_cover.sby is meant to prove about the design as it\n"
            "ships, so a control that starts red proves nothing about a mutant."
        )
    elif len(reached) != len(goals):
        red.append(
            f"the shipping reference reached {len(reached)} cover sites but\n"
            f"ill_e.sv states {len(goals)} `cover property` lines. The goal set\n"
            "this control grades is not the goal set the harness has."
        )

    no_illegal = mutate(ill_e_sv, ILLEGAL_ANCHOR, ILLEGAL_MUTANT, "no-illegal")
    status, log = run_case(
        workdir, args.sby, "cover-no-illegal", no_illegal, "ill_e_cover.sby", ill_e_cover_sby
    )
    unreached = {m.group("site") for m in COVER_SITE.finditer(log) if m.group("un")}
    print(f"no-illegal mutant: {status}, unreached {sorted(unreached) or 'none'}")
    if status != "FAIL":
        red.append(
            "the no-illegal mutant proves. Tying e_illegal to a constant is\n"
            "exactly what should make every cover goal unreachable, so an\n"
            "anti-vacuity control that cannot go red is not a control."
        )
    elif not unreached:
        red.append(
            "the no-illegal mutant went red without naming a single unreached\n"
            "cover site, so nothing here is evidence the cover goals are what\n"
            "failed."
        )
    elif not unreached <= reached:
        red.append(
            f"the no-illegal mutant named {sorted(unreached - reached)} unreached,\n"
            "which the shipping reference never reached either -- that is not\n"
            "evidence this mutation is what took a real goal away."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print(
        "Every assertion mutant fails at its own line, and the no-illegal mutant "
        "leaves a cover goal unreached; the shipping reference passes both."
    )


if __name__ == "__main__":
    main()
