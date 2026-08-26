#!/usr/bin/env python3
"""Forces rtl/decoder.v's own Zkt-isolation assertions to fail, and requires
each to fail as its own assertion rather than as anything else.

WHY THIS EXISTS. rtl/decoder.v's `ifdef FORMAL` block states two things about
`region_stall`, the one stall reason Zkt's isolation argument allows to read a
register-file DATA output: it can only assert alongside `ls_access`
(`assert(!region_stall || ls_access)`), and `ls_access` is true for exactly the
eight base load/store encodings (`assert(ls_access == (instr_lb || ...))`).
Both are proved by `make -C formal components_decoder`, and an assertion in
that position is worth nothing until it has been shown to fail -- which is
what `make probe-gates` demands of every graded comparison hermetic enough to
run there and what this file does for the two that need a solver instead
(the pattern `make -C formal traps-region-probe` set for formal/traps.sv).

Two cores are built, each one line of rtl/decoder.v away from the shipping
one, and each must turn `make -C formal components_decoder` red at its own
assertion:

  region-stall-ungated  drops the `ls_access` conjunct from `region_stall`'s
                         own `assign`, so `region_stall` can now assert
                         whether or not `ls_access` does. Must go FAIL at
                         `assert(!region_stall || ls_access)`.
  ls-access-extra        adds `instr_add` to `ls_access`'s own `assign`, so
                         `ls_access` is no longer exactly the eight base
                         load/store encodings. Must go FAIL at
                         `assert(ls_access == (instr_lb || ...))`.

The unmutated core is not built here: it is what `components_decoder` proves,
and this file is a prerequisite of that target.

Each failing assertion is pinned by the LINE rtl/decoder.v states it on, read
out of the file here rather than written down, the same discipline
traps-region-probe.py applies to formal/traps.sv: a probe that only checked
the status would be satisfied by a proof that went red for an unrelated
reason.

NOT HERMETIC -- it runs sby twice. So it is a prerequisite of
`make -C formal components_decoder` rather than of `make test`, for the same
reason pcloop_cover and traps-region-probe are: a control that can be run
separately from the thing it controls eventually is not run at all.
test/probe_gates.sh's own zkt group covers the taint/reachability half this
script does not touch, against a stub sby.

Usage: decoder-zkt-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from traps_probe_sby import script_block  # noqa: E402

# The `decoder` task's own files, read directly rather than inherited from
# traps_probe_sby's SOURCES: that tuple is traps.sv's own dependency list
# (fetcher.v, csrs.v included) and always appends traps.sv itself, neither of
# which the `decoder` task's script names.
SOURCES = ("structs.v", "decoder.v", "regsel.v")

TEMPLATE = """[options]
mode prove

[engines]
smtbmc

[script]
{script}

[files]
{files}
"""

# The two assertions being probed, found in rtl/decoder.v by their text. Each
# is the only statement in that file that says what it says.
ASSERTS = {
    "region-stall-ungated": "assert(!region_stall || ls_access);",
    "ls-access-extra": "assert(ls_access == (instr_lb || instr_lbu || instr_lh || instr_lhu ||",
}

# The lines of rtl/decoder.v each mutation replaces, matched in full so a
# respelling stops this file rather than silently probing nothing.
MUTATIONS = {
    "region-stall-ungated": (
        "  assign region_stall = ls_access && !ls_settled && !ls_answer_valid;\n",
        "  assign region_stall = !ls_settled && !ls_answer_valid;\n",
    ),
    "ls-access-extra": (
        "  assign ls_access = instr_ls_load || instr_ls_store;\n",
        "  assign ls_access = instr_ls_load || instr_ls_store || instr_add;\n",
    ),
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def assert_line(decoder_v, case):
    """The line rtl/decoder.v states this case's assertion on, 1-based. A
    multi-line assert is found by its FIRST line, which is where smtbmc
    reports the failure."""
    needle = ASSERTS[case]
    hits = [n for n, line in enumerate(decoder_v.splitlines(), 1) if needle in line]
    if len(hits) != 1:
        stop(
            f"rtl/decoder.v states `{needle}` {len(hits)} times, and this probe\n"
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


def decoder_probe_sby(repo):
    """The `decoder` task's sby text, read out of formal/components.sby rather
    than copied -- the same reasoning traps_probe_sby.script_block states for
    itself: a probe on a stale script proves a design the shipping task does
    not build."""
    path = repo / "formal" / "components.sby"
    if not path.is_file():
        stop(
            f"{path} is missing, and this probe reads the `decoder` task's script\n"
            "out of it rather than keeping a copy. Without it there is no shipping\n"
            "script to build the mutated cores against."
        )
    block = script_block(path.read_text(), "decoder")
    if not block:
        stop(
            "formal/components.sby states no `decoder:` block under [script], so\n"
            "this probe cannot read the script it is meant to build against. Teach\n"
            "it the new task name rather than restoring a copy here."
        )
    return TEMPLATE.format(
        script="\n".join(block).strip("\n"),
        files="\n".join(f"src/{name}" for name in SOURCES),
    )


def run_case(repo, workdir, sby, config, case):
    """Builds the mutated tree, runs sby, and returns (status, failing lines)."""
    root = workdir / case
    shutil.rmtree(root, ignore_errors=True)
    (root / "src").mkdir(parents=True)
    for name in SOURCES:
        shutil.copy(repo / "rtl" / name, root / "src" / name)
    decoder = (repo / "rtl" / "decoder.v").read_text()
    (root / "src" / "decoder.v").write_text(mutate(decoder, case))
    (root / "probe.sby").write_text(config)

    # sby's own exit status is not read: FAIL is the required outcome, and a
    # non-zero status says nothing this file does not read out of the workdir
    # instead.
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
    failed = sorted(
        set(int(n) for n in re.findall(r"Assert failed in decoder: decoder\.v:(\d+)", log))
    )
    return status[0], failed


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent), help="tree to read the RTL from")
    parser.add_argument("--workdir", default=str(here / "decoder-zkt-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("formal/components.sby", "rtl/decoder.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    config = decoder_probe_sby(repo)
    decoder_v = (repo / "rtl" / "decoder.v").read_text()
    red = []

    for case in ("region-stall-ungated", "ls-access-extra"):
        line = assert_line(decoder_v, case)
        print(f"rtl/decoder.v states {case}'s assertion on line {line}.")
        status, failed = run_case(repo, workdir, args.sby, config, case)
        print(f"  {case}: {status}, assertions failed at {failed or 'none'}")
        if status != "FAIL":
            red.append(
                f"the {case} core proves. That mutation is exactly what the assertion was\n"
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

    print("Both Zkt-isolation assertions fail for their own reason.")


if __name__ == "__main__":
    main()
