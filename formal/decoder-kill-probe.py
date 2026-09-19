#!/usr/bin/env python3
"""Forces rtl/decoder.v's `kill => !issuing` assertion to fail, the way
decoder-zkt-probe.py does for the two Zkt-isolation assertions in the same file.

WHY THIS EXISTS. `kill` is ANDed with `buffer_empty` inside rtl/decoder.v
precisely so the implication holds locally, with no assumption about
`redirect_recovering` (a free input in formal/traps.sv, which has no
fetchctrl to constrain it). That AND is the whole mechanism; drop it and the
assertion has nothing left to lean on in any harness that drives
`redirect_recovering` free while `buffer_empty` is low -- exactly what
`components_decoder`'s own proof does. The mutation below is that exact drop.

NOT HERMETIC -- it runs sby once. A prerequisite of `make -C formal
components_decoder`, the same standing decoder-zkt-probe has.

Usage: decoder-kill-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]
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
# traps_probe_sby's SOURCES -- the same reason decoder-zkt-probe.py gives.
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

ASSERT_TEXT = "assert(!kill || !issuing);"

# Drops the buffer_empty conjunct, so `kill` tracks the free `redirect_recovering` input
# alone -- true whenever an unrelated harness happens to leave the buffer non-empty and
# raises redirect_recovering anyway.
MUTATION = (
    "  assign kill = buffer_empty && redirect_recovering;\n",
    "  assign kill = redirect_recovering;\n",
)

def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)

def assert_line(decoder_v):
    """The line rtl/decoder.v states the kill assertion on, 1-based."""
    hits = [n for n, line in enumerate(decoder_v.splitlines(), 1) if ASSERT_TEXT in line]
    if len(hits) != 1:
        stop(
            f"rtl/decoder.v states `{ASSERT_TEXT}` {len(hits)} times, and this probe "
            "pins the failing assertion by its line. Teach it the new spelling rather "
            "than dropping the assertion: a probe that only reads the status passes "
            "for a proof that went red somewhere else entirely."
        )
    return hits[0]

def mutate(decoder_v):
    """rtl/decoder.v with kill's buffer_empty conjunct dropped."""
    old, new = MUTATION
    if old not in decoder_v:
        stop(
            "rtl/decoder.v no longer spells what the kill mutation replaces. "
            "Re-anchor it on the new spelling -- left alone it would build the "
            "shipping core and report that an arm which was never exercised is fine."
        )
    if old == new:
        stop(
            "the kill mutation replaces its text with itself, so the core below "
            "would be the shipping one and the proof would say nothing."
        )
    return decoder_v.replace(old, new, 1)

def decoder_probe_sby(repo):
    """The `decoder` task's sby text, read out of formal/components.sby rather
    than copied, the same reasoning decoder-zkt-probe.py states for itself."""
    path = repo / "formal" / "components.sby"
    if not path.is_file():
        stop(
            f"{path} is missing, and this probe reads the `decoder` task's script "
            "out of it rather than keeping a copy. Without it there is no shipping "
            "script to build the mutated core against."
        )
    block = script_block(path.read_text(), "decoder")
    if not block:
        stop(
            "formal/components.sby states no `decoder:` block under [script], so "
            "this probe cannot read the script it is meant to build against. Teach "
            "it the new task name rather than restoring a copy here."
        )
    return TEMPLATE.format(
        script="\n".join(block).strip("\n"),
        files="\n".join(f"src/{name}" for name in SOURCES),
    )

def run_case(repo, workdir, sby, config):
    """Builds the mutated tree, runs sby, and returns (status, failing lines)."""
    root = workdir / "kill-not-gated-by-buffer-empty"
    shutil.rmtree(root, ignore_errors=True)
    (root / "src").mkdir(parents=True)
    for name in SOURCES:
        shutil.copy(repo / "rtl" / name, root / "src" / name)
    decoder = (repo / "rtl" / "decoder.v").read_text()
    (root / "src" / "decoder.v").write_text(mutate(decoder))
    (root / "probe.sby").write_text(config)

    # sby's own exit status is not read: FAIL is the required outcome, and a non-zero
    # status says nothing this file does not read out of the workdir instead.
    proc = subprocess.run(
        [sby, "-f", "probe.sby"], cwd=root, capture_output=True, text=True
    )
    status_file = root / "probe" / "status"
    if not status_file.is_file():
        stop(
            "sby wrote no status for the mutated core, so nothing was proved or "
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop("sby's status file for the mutated core is empty.")
    log = (root / "probe" / "logfile.txt").read_text()
    failed = sorted(
        set(int(n) for n in re.findall(
            r"engine_\d+\.basecase:.*Assert failed in decoder: decoder\.v:(\d+)", log)))
    return status[0], failed

def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent), help="tree to read the RTL from")
    parser.add_argument("--workdir", default=str(here / "decoder-kill-probe"))
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
    line = assert_line(decoder_v)
    print(f"rtl/decoder.v states kill's assertion on line {line}.")
    status, failed = run_case(repo, workdir, args.sby, config)
    print(f"  kill-not-gated-by-buffer-empty: {status}, assertions failed at {failed or 'none'}")

    if status != "FAIL":
        sys.exit(
            "*** the mutated core proves. Dropping kill's buffer_empty conjunct is\n"
            "*** exactly what the assertion was written to catch, so an arm that\n"
            "*** admits it is asking nothing at all."
        )
    if line not in failed:
        sys.exit(
            f"*** the mutated core went red at {failed}, which does not include line\n"
            f"*** {line} -- the assertion this probe is about. A proof failing\n"
            "*** somewhere else is not evidence about this arm."
        )

    print("kill's assertion fails for its own reason.")

if __name__ == "__main__":
    main()
