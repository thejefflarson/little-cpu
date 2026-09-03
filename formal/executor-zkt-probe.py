#!/usr/bin/env python3
"""Forces rtl/executor.v's own MUL-family constant-latency assertion to fail,
and requires it to fail as its own assertion rather than as anything else.

WHY THIS EXISTS. Zkt's isolation argument has two halves. The decoder half is
graded by test/zkt_isolation_test.py and, for the two facts a 2-safety taint
walk cannot decide on its own, by formal/decoder-zkt-probe.py.
The executor half used to be read rather than graded: `MUL`/`MULH`/`MULHU`/
`MULHSU` resolve in decode's `init` state with no counter, so they take one
cycle regardless of the operands -- true today, and nothing said so if it
stopped being true. rtl/executor.v now states it as an assertion
(`assert(state == init)`, guarded on `$past(state) == init` and one of the
four multiply flags), proved by `make -C formal components_executor`. An
assertion in that position is worth nothing until it has been shown to fail --
which is what `make probe-gates` demands of every graded comparison hermetic
enough to run there and what this file does for the one that needs a solver
instead (the pattern `make -C formal traps-region-probe` and
`make -C formal components_decoder` both set).

One core is built, three lines of rtl/executor.v from the shipping one: `MUL`
with `rs1 == 0 && rs2 != 0` is diverted into the divider's own arm instead of
resolving in `init`, latching `op_is_divu` alongside it so the arm's own
onehot invariant over its four op flags still holds. `rs2 != 0` is not
incidental: the divide arm's OWN first branch is `if (rs2 == 0)`, which -- with
`is_rem`/`is_remu` forced low by the $onehot0 assume, since only `in.is_mul`
is set -- writes `32'hffffffff` into `out.rd_data` while a real `MUL(0, 0)`'s
`mul_lo` is 0. Diverting `rs2 == 0` too would therefore ALSO break the
pre-existing MUL-value assertion four lines above this one, reachable from
reset in the same steps as the latency assertion -- a second, genuine defect
the mutation would have introduced, not evidence about the assertion this
file exists to probe. Excluding it leaves exactly one arm reachable for the
diverted operands: the real divide, `mul_div_counter <= 32; state <= divide`,
which completes at `divu_ref = div_ghost_rs1 / div_ghost_rs2 == 0 / rs2 == 0`
-- so the diverted core's mul-value assertion holds for every choice of rs2
the mutation can reach, and only the latency assertion this file is about can
fail on the basecase's own reachable trace. Diverting through the divider's
own arm, rather than inventing a new state, also keeps every line in the file
at its shipping line number, which is what lets this probe pin the failure by
the line rtl/executor.v itself states rather than by one recomputed for the
mutated tree. The mutated core's BASECASE leg -- the one bounded from reset,
over a concretely reachable trace -- must go FAIL at
`assert(state == init);` and at no other line; this is the shape a
narrow-operand fast path or an early-out would actually take, applied to give
MUL a slow path instead of a fast one.

Only the BASECASE leg is read. `mode prove`'s INDUCTION leg starts from an
arbitrary state that need not be reachable from reset at all, and once one
assertion in the file is generally false its counterexamples can name a
DIFFERENT, otherwise-true assertion -- observed here as line 365, the
pre-existing MUL-value assertion, well before this file's mutation was
narrowed to exclude `rs2 == 0`. Which line the induction leg reports is the
solver's own arbitrary choice among the states it is free to start from, not
a property of the mutation, so it is not evidence either way and this file's
regex requires an `engine_N.basecase:` prefix on the same log line before a
line number counts as a failure at all.

The unmutated core is not built here: it is what `components_executor`
proves, and this file is a prerequisite of that target.

The failing assertion is pinned by the LINE rtl/executor.v states it on, read
out of the file here rather than written down, the same discipline
decoder-zkt-probe.py and traps-region-probe.py apply: a probe that only
checked the status would be satisfied by a proof that went red for an
unrelated reason.

NOT HERMETIC -- it runs sby once. So it is a prerequisite of
`make -C formal components_executor` rather than of `make test`, for the same
reason pcloop_cover and traps-region-probe are: a control that can be run
separately from the thing it controls eventually is not run at all.
test/probe_gates.sh's own group covers this file's own grading, against a
stub sby.

Usage: executor-zkt-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from traps_probe_sby import script_block  # noqa: E402

# The `executor` task's own files, read directly rather than inherited from
# traps_probe_sby's SOURCES: that tuple is traps.sv's own dependency list and
# always appends traps.sv itself, neither of which the `executor` task's
# script names.
SOURCES = ("structs.v", "executor.v")

TEMPLATE = """[options]
mode prove

[engines]
smtbmc

[script]
{script}

[files]
{files}
"""

# The assertion being probed, found in rtl/executor.v by its text. It is the
# only statement in that file that says what it says.
MUL_ASSERT = "assert(state == init);"

# Three lines, each replaced whole so a respelling stops this file rather than
# silently probing nothing. Together they divert `MUL` at rs1 == 0 into the
# divider's arm instead of the mul family's own: excluded from the mul
# family's case item, added to the divide item's, and latched into
# op_is_divu so the divide arm's own onehot invariant over its four op flags
# still holds for a divide it entered by this new route.
MUTATIONS = (
    (
        "            in.is_mul || in.is_mulh || in.is_mulhu || in.is_mulhsu: begin\n",
        "            (in.is_mul && (rs1 != 32'b0 || rs2 == 32'b0)) || in.is_mulh || "
        "in.is_mulhu || in.is_mulhsu: begin\n",
    ),
    (
        "            in.is_div || in.is_divu || in.is_rem || in.is_remu: begin\n",
        "            (in.is_mul && rs1 == 32'b0 && rs2 != 32'b0) || in.is_div || "
        "in.is_divu || in.is_rem || in.is_remu: begin\n",
    ),
    (
        "              op_is_divu <= in.is_divu;\n",
        "              op_is_divu <= in.is_divu || in.is_mul;\n",
    ),
)


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def assert_line(executor_v):
    """The line rtl/executor.v states the MUL constant-latency assertion on,
    1-based."""
    hits = [n for n, line in enumerate(executor_v.splitlines(), 1) if MUL_ASSERT in line]
    if len(hits) != 1:
        stop(
            f"rtl/executor.v states `{MUL_ASSERT}` {len(hits)} times, and this probe\n"
            "pins the failing assertion by its line. Teach it the new spelling rather\n"
            "than dropping the assertion: a probe that only reads the status passes\n"
            "for a proof that went red somewhere else entirely."
        )
    return hits[0]


def mutate(executor_v):
    """rtl/executor.v with MUL at rs1 == 0 diverted into the divider's arm."""
    text = executor_v
    for old, new in MUTATIONS:
        if old not in text:
            stop(
                "rtl/executor.v no longer spells one of the lines this probe patches to\n"
                "divert MUL into the divider's arm. Re-anchor the mutation on the new\n"
                "spelling -- left alone it would build the shipping core and report that\n"
                "an arm which was never exercised is fine."
            )
        text = text.replace(old, new, 1)
    if text == executor_v:
        stop(
            "the mutation replaced nothing, so the core below would be the shipping\n"
            "one and the proof would say nothing."
        )
    return text


def executor_probe_sby(repo):
    """The `executor` task's sby text, read out of formal/components.sby rather
    than copied -- the same reasoning traps_probe_sby.script_block states for
    itself: a probe on a stale script proves a design the shipping task does
    not build."""
    path = repo / "formal" / "components.sby"
    if not path.is_file():
        stop(
            f"{path} is missing, and this probe reads the `executor` task's script\n"
            "out of it rather than keeping a copy. Without it there is no shipping\n"
            "script to build the mutated core against."
        )
    block = script_block(path.read_text(), "executor")
    if not block:
        stop(
            "formal/components.sby states no `executor:` block under [script], so\n"
            "this probe cannot read the script it is meant to build against. Teach\n"
            "it the new task name rather than restoring a copy here."
        )
    return TEMPLATE.format(
        script="\n".join(block).strip("\n"),
        files="\n".join(f"src/{name}" for name in SOURCES),
    )


def run_probe(repo, workdir, sby, config):
    """Builds the mutated tree, runs sby, and returns (status, failing lines)."""
    root = workdir / "mul-into-divide"
    shutil.rmtree(root, ignore_errors=True)
    (root / "src").mkdir(parents=True)
    for name in SOURCES:
        shutil.copy(repo / "rtl" / name, root / "src" / name)
    executor = (repo / "rtl" / "executor.v").read_text()
    (root / "src" / "executor.v").write_text(mutate(executor))
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
            "sby wrote no status for the mul-into-divide core, so nothing was\n"
            "proved or disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop("sby's status file for the mul-into-divide core is empty.")
    log = (root / "probe" / "logfile.txt").read_text()
    failed = sorted(
        set(int(n) for n in re.findall(
            r"engine_\d+\.basecase:.*Assert failed in executor: executor\.v:(\d+)", log))
    )
    return status[0], failed


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent), help="tree to read the RTL from")
    parser.add_argument("--workdir", default=str(here / "executor-zkt-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    for name in ("formal/components.sby", "rtl/executor.v"):
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    config = executor_probe_sby(repo)
    executor_v = (repo / "rtl" / "executor.v").read_text()
    line = assert_line(executor_v)
    print(f"rtl/executor.v states the MUL constant-latency assertion on line {line}.")

    status, failed = run_probe(repo, workdir, args.sby, config)
    print(f"  mul-into-divide: {status}, assertions failed at {failed or 'none'}")

    red = []
    if status != "FAIL":
        red.append(
            "the mul-into-divide core proves. Giving MUL an operand-dependent second\n"
            "cycle is exactly what this assertion was written to catch, so an arm that\n"
            "admits it is asking nothing at all."
        )
    elif line not in failed:
        red.append(
            f"the mul-into-divide core went red at {failed}, which does not include\n"
            f"line {line} -- the assertion this probe is about. A proof failing\n"
            "somewhere else is not evidence about this arm."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The MUL constant-latency assertion fails for its own reason.")


if __name__ == "__main__":
    main()
