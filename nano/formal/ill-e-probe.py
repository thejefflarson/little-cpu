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

Three mutations target the assertion's own three clauses (`trap`, `rd_addr`,
`mem_write`), each independently, tying that one output to the value a core
that got the restriction wrong would report. `ill_e.sby` must go FAIL at
each mutation's own assertion line.

`e_illegal` is an OR of one named term per (class, field) membership --
`ill_op_rs2` names "OP reads rs2", say -- and each is read out of the file
rather than hardcoded, so the probe cannot drift from what the file actually
states. Two more mutation classes exercise it:

  tie-low       ties `e_illegal` to a constant, so every membership is lost
                at once; a cheap smoke test only -- measured against this
                file, sby's cover engine reports just ONE of the 23 lost
                goals before declaring the job FAIL and stopping, not all of
                them the way a single shared gate would, so this arm only
                requires at least one real one, not the full set.
  drop-<term>   removes exactly one term from the OR-list, so exactly ONE
                (class, field) membership is lost; `ill_e_cover.sby` must
                leave EXACTLY that term's own cover goal unreached -- this is
                the precise arm. Deleting a class from a field's membership
                list without it would be invisible: the assertion never
                required a trap for that case in the first place, and a
                shared cover goal would still be satisfied by a sibling
                field going high instead.

Neither the shipping (unmutated) file nor a mutant proves anything alone: a
mutant that fails is not evidence unless the file it was mutated from is
first shown to pass, or to reach every cover goal.

NOT HERMETIC -- it runs sby dozens of times (three assertion mutants, one
per (class, field) membership, plus the shipping and tie-low controls). So
it is a prerequisite of `make -C nano/formal ill_e_cover` rather than of
`make test`, the same reason pcloop_cover and traps-region-probe are: a
control that can be run separately from the thing it controls eventually is
not run at all. It is not a prerequisite of `make -C nano/formal ill_e`
alone: `ill_e` proves the assertion against the reference model, but a
reference model proves nothing about itself without this probe, so both
targets need it and `ill_e_cover` carries it for both -- collapsing the two
into one CI step (`make -C nano/formal ill_e ill_e_cover`) means the shared
PHONY prerequisite still runs only once. test/probe_gates.sh covers this
file's own logic against a stub sby.
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

# `wire e_illegal =\n  term1 || term2 || ... ;` -- read rather than hardcoded, so the
# probe's own term list cannot drift from the file it is probing.
E_ILLEGAL = re.compile(r"wire e_illegal =(?P<body>.*?);", re.S)

COVER_LINE = re.compile(r"^\s*cover property \(")
# sby's witness name embeds a source line one less than the text's own line (measured
# against every cover statement in this file); cover_result() adds the 1 back.
COVER_WITNESS = re.compile(r"(?P<un>[Uu]n)?[Rr]eached cover statement.*?check_cover_ill_e_sv_(?P<line>\d+)_")


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


def illegal_terms(ill_e_sv):
    """The (class, field) membership terms named in e_illegal's OR-list, in order."""
    match = E_ILLEGAL.search(ill_e_sv)
    if not match:
        stop(
            "nano/formal/ill_e.sv states no `wire e_illegal =` statement, so\n"
            "there is nothing for this control to mutate."
        )
    return [t.strip() for t in match.group("body").split("||")]


def cover_line_for_term(ill_e_sv, term):
    """The 1-based line of `term`'s own `cover property (live && e_illegal && term)`."""
    return assert_line(ill_e_sv, f"cover property (live && e_illegal && {term});")


def drop_illegal_term(ill_e_sv, term):
    """ill_e.sv with exactly one (class, field) membership removed from e_illegal.

    Removes only the term's own text and its one adjacent `||`, matched with
    horizontal whitespace alone (never `\\n`), so every OTHER line -- including
    every cover goal's own line -- keeps the exact line number it started with.
    """
    if term not in illegal_terms(ill_e_sv):
        stop(
            f"nano/formal/ill_e.sv's e_illegal OR-list no longer names `{term}`.\n"
            "Re-anchor it on the new spelling -- left alone it would build the\n"
            "shipping file and prove nothing about a mutated core."
        )
    escaped = re.escape(term)
    after = re.compile(rf"\b{escaped}\b[ \t]*\|\|[ \t]*")
    before = re.compile(rf"[ \t]*\|\|[ \t]*\b{escaped}\b")
    new_text, n = after.subn("", ill_e_sv, count=1)
    if n == 0:
        new_text, n = before.subn("", ill_e_sv, count=1)
    if n != 1:
        stop(f"could not remove exactly one occurrence of `{term}` from e_illegal's OR-list.")
    return new_text


def tie_illegal_low(ill_e_sv):
    """ill_e.sv with e_illegal tied to a constant, killing every membership at once.

    Pads the one-line replacement back out to the newline count it replaced, for
    the same reason drop_illegal_term never touches a `\\n`: every line after this
    statement, cover goals included, must keep its own line number.
    """
    match = E_ILLEGAL.search(ill_e_sv)
    if not match:
        stop(
            "nano/formal/ill_e.sv states no `wire e_illegal =` statement, so\n"
            "there is nothing for this control to mutate."
        )
    padding = "\n" * match.group(0).count("\n")
    return ill_e_sv[: match.start()] + "wire e_illegal = 1'b0;" + padding + ill_e_sv[match.end() :]


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


def cover_result(log):
    """(reached lines, unreached lines) sby's log names, both as int sets."""
    reached, unreached = set(), set()
    for m in COVER_WITNESS.finditer(log):
        (unreached if m.group("un") else reached).add(int(m.group("line")) + 1)
    return reached, unreached


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
    terms = illegal_terms(ill_e_sv)
    if not goals:
        stop(
            "nano/formal/ill_e.sv states no `cover property` line, so there is\n"
            "nothing for this control to reach."
        )
    if not terms:
        stop(
            "nano/formal/ill_e.sv's e_illegal OR-list names no terms, so there is\n"
            "no (class, field) membership for this control to drop."
        )
    term_lines = {t: cover_line_for_term(ill_e_sv, t) for t in terms}

    status, log = run_case(
        workdir, args.sby, "cover-shipping", ill_e_sv, "ill_e_cover.sby", ill_e_cover_sby
    )
    reached, _ = cover_result(log)
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

    # tie-low: every (class, field) membership lost at once. Only a cheap smoke test
    # ahead of the drop-<term> loop below, which is the precise one: measured against
    # this file at both depth 2 and depth 100, sby's cover engine reports only ONE
    # unreached goal once the job's overall status is FAIL, not all 23 independently
    # gated ones the way complete-cover-probe.py's single-gated stalled-bus case does
    # -- so "at least one, and only a real e_illegal-gated one" is what this arm can
    # honestly require.
    expected = set(term_lines.values())
    status, log = run_case(
        workdir, args.sby, "cover-tie-low", tie_illegal_low(ill_e_sv), "ill_e_cover.sby", ill_e_cover_sby
    )
    _, unreached = cover_result(log)
    print(f"tie-low mutant: {status}, unreached {sorted(unreached) or 'none'} (of {len(expected)} candidates)")
    if status != "FAIL":
        red.append(
            "the tie-low mutant proves. Tying e_illegal to a constant is exactly\n"
            "what should make every membership's cover goal unreachable, so an\n"
            "anti-vacuity control that cannot go red is not a control."
        )
    elif not unreached:
        red.append(
            "the tie-low mutant went red without naming a single unreached cover\n"
            "site, so nothing here is evidence the cover goals are what failed."
        )
    elif not unreached <= expected:
        red.append(
            f"the tie-low mutant named {sorted(unreached - expected)} unreached,\n"
            "which is not one of e_illegal's own membership goals -- that is not\n"
            "evidence this mutation is what took a real goal away."
        )

    # drop-<term>: exactly one (class, field) membership lost, one at a time.
    for term in terms:
        case = f"cover-drop-{term}"
        status, log = run_case(
            workdir, args.sby, case, drop_illegal_term(ill_e_sv, term), "ill_e_cover.sby", ill_e_cover_sby
        )
        _, unreached = cover_result(log)
        if status != "FAIL":
            red.append(
                f"dropping `{term}` from e_illegal's OR-list still proves\n"
                "ill_e_cover.sby. That membership is what its own cover goal\n"
                "exists to witness, so a drop that cannot go red witnesses\n"
                "nothing."
            )
        elif unreached != {term_lines[term]}:
            red.append(
                f"dropping `{term}` left {sorted(unreached)} unreached, not exactly\n"
                f"line {term_lines[term]} -- `{term}`'s own cover goal. A sibling\n"
                "field's goal going unreached instead means that goal was\n"
                "satisfied by this one too, which is not evidence this membership\n"
                "on its own matters."
            )
    print(f"drop-<term> mutants: {len(terms)} run, one per (class, field) membership")

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print(
        "Every assertion mutant fails at its own line, the tie-low mutant loses a\n"
        f"real e_illegal-gated goal, and each of the {len(terms)} drop-<term> mutants\n"
        "loses exactly its own; the shipping reference passes all of it."
    )


if __name__ == "__main__":
    main()
