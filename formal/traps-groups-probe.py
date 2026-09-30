#!/usr/bin/env python3
"""Forces traps-groups-test.py to fail, two ways, each for its own reason: a
mis-spelled TRAPS_CHECK_* select really is invisible to every split task, and a
stale manifest really is caught.

Usage: traps-groups-probe.py [--repo DIR]
"""

import argparse
import pathlib
import subprocess
import sys

def stop(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)

def run(repo, traps_path, checker, manifest_path=None):
    cmd = [sys.executable, str(checker), "--repo", str(repo), "--traps", str(traps_path)]
    if manifest_path is not None:
        cmd += ["--manifest", str(manifest_path)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr

def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent))
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    checker = here / "traps-groups-test.py"
    traps_path = repo / "formal" / "traps.sv"
    if not traps_path.is_file():
        stop(f"{traps_path} is missing, so there is nothing to probe.")
    original = traps_path.read_text()

    red = []

    # Case 1: misspell the one TRAPS_CHECK_STATUS select guarding a real assertion,
    # orphaning it -- exactly the failure a real split (TRAPS_SPLIT defined) makes silent.
    needle = "`ifdef TRAPS_CHECK_STATUS\n  always_comb if (clocked && !irq_timer)"
    if needle not in original:
        stop(
            "traps.sv no longer spells the TRAPS_CHECK_STATUS guard this probe "
            "mutates -- re-anchor it on the current text."
        )
    mutated = original.replace(
        needle,
        "`ifdef TRAPS_CHECK_STATUS_TYPO\n  always_comb if (clocked && !irq_timer)",
        1,
    )
    tmp = traps_path.with_suffix(".orphan-probe.sv")
    tmp.write_text(mutated)
    status, output = run(repo, tmp, checker)
    tmp.unlink()
    if status == 0:
        red.append("the orphaned-assertion mutation still passed traps-groups-test.py")
    elif "unknown group" not in output and "not inside any TRAPS_CHECK_* block" not in output:
        red.append(
            "the orphaned-assertion mutation failed for a different reason:\n" + output
        )

    # Case 2: a stale manifest -- traps.sv unchanged, TRAPS_GROUPS wrong.
    real_manifest = (repo / "formal" / "TRAPS_GROUPS").read_text()
    stale = real_manifest.replace("PC 12", "PC 11", 1)
    if stale == real_manifest:
        stop("formal/TRAPS_GROUPS no longer spells 'PC 12' -- re-anchor this probe.")
    tmp_manifest = repo / "formal" / "TRAPS_GROUPS.stale-probe"
    tmp_manifest.write_text(stale)
    status, output = run(repo, traps_path, checker, manifest_path=tmp_manifest)
    tmp_manifest.unlink()
    if status == 0:
        red.append("the stale-manifest mutation still passed traps-groups-test.py")
    elif "traps.sv has" not in output or "update the manifest" not in output:
        red.append(f"the stale-manifest mutation failed for a different reason:\n{output}")

    if red:
        print()
        for why in red:
            print("*** " + why, file=sys.stderr)
        sys.exit(1)

    print("Both traps-groups-test.py mutations fail for their own reason.")

if __name__ == "__main__":
    main()
