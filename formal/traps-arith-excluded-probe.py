#!/usr/bin/env python3
"""Forces traps-arith-excluded-test.py to fail, three ways, each for its own reason:
a stale manifest, an assertion excluded from executor.v's own count with no matching
declared reason, and the macro missing from a traps task's own read (which would
silently leave that task re-proving arithmetic no property there needs).

Usage: traps-arith-excluded-probe.py [--repo DIR]
"""

import argparse
import pathlib
import subprocess
import sys

def stop(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)

def run(repo, checker, executor_path=None, manifest_path=None, sby_path=None):
    cmd = [sys.executable, str(checker), "--repo", str(repo)]
    if executor_path is not None:
        cmd += ["--executor", str(executor_path)]
    if manifest_path is not None:
        cmd += ["--manifest", str(manifest_path)]
    if sby_path is not None:
        cmd += ["--components-sby", str(sby_path)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout + proc.stderr

def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(here.parent))
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    checker = here / "traps-arith-excluded-test.py"
    executor_path = repo / "rtl" / "executor.v"
    manifest_path = repo / "formal" / "TRAPS_ARITH_EXCLUDED"
    sby_path = repo / "formal" / "components.sby"
    for p in (executor_path, manifest_path, sby_path):
        if not p.is_file():
            stop(f"{p} is missing, so there is nothing to probe.")

    red = []

    # Case 1: a stale manifest -- executor.v unchanged, TRAPS_ARITH_EXCLUDED wrong.
    real_manifest = manifest_path.read_text()
    stale = real_manifest.replace("\n13\n", "\n12\n", 1)
    if stale == real_manifest:
        stop("formal/TRAPS_ARITH_EXCLUDED no longer spells '13' alone on a line -- re-anchor this probe.")
    tmp_manifest = manifest_path.with_suffix(".stale-probe")
    tmp_manifest.write_text(stale)
    status, output = run(repo, checker, manifest_path=tmp_manifest)
    tmp_manifest.unlink()
    if status == 0:
        red.append("the stale-manifest mutation still passed traps-arith-excluded-test.py")
    elif "guards" not in output or "update the manifest" not in output:
        red.append(f"the stale-manifest mutation failed for a different reason:\n{output}")

    # Case 2: orphan one exclusion -- unguard a real excluded assertion, so the count
    # drops below the manifest with no declared reason. The Zkt-latency assertion is
    # the one this repo's own probe (executor-zkt-probe.py) also depends on staying
    # inside the guard, so unguarding it is a real, not a token, mutation.
    original = executor_path.read_text()
    needle = " `ifndef TRAPS_SKIP_EXEC_ARITH\n  always_ff @(posedge clk)\n" \
        "    if (clocked && !reset && !$past(reset) && $past(state) == init && $past(launch_is_mul))\n" \
        "      assert(out_rd_data == $past(mul_lo));"
    if needle not in original:
        stop("rtl/executor.v no longer spells the mul-result guard this probe mutates -- re-anchor it.")
    mutated = original.replace(
        needle,
        needle.replace(" `ifndef TRAPS_SKIP_EXEC_ARITH\n", "", 1),
        1,
    )
    tmp_executor = executor_path.with_suffix(".orphan-probe.v")
    tmp_executor.write_text(mutated)
    status, output = run(repo, checker, executor_path=tmp_executor)
    tmp_executor.unlink()
    if status == 0:
        red.append("the unguarded-assertion mutation still passed traps-arith-excluded-test.py")
    elif "guards" not in output or "update the manifest" not in output:
        red.append(f"the unguarded-assertion mutation failed for a different reason:\n{output}")

    # Case 3: drop the macro from one traps task's own read line, leaving the manifest
    # and executor.v untouched -- the exclusion this probe grades would stop applying
    # there while every check of the guard and the count still passes.
    real_sby = sby_path.read_text()
    needle_sby = (
        "traps_cause:\n"
        "read -sv -formal -noassume -D TRAPS_SKIP_EXEC_ARITH structs.v fetcher.v "
        "decoder.v executor.v regsel.v csrs.v\n"
    )
    if needle_sby not in real_sby:
        stop(
            "formal/components.sby no longer spells traps_cause's own -D "
            "TRAPS_SKIP_EXEC_ARITH read line -- re-anchor this probe."
        )
    stale_sby = real_sby.replace(
        needle_sby,
        needle_sby.replace(" -D TRAPS_SKIP_EXEC_ARITH", "", 1),
        1,
    )
    tmp_sby = sby_path.with_suffix(".undefined-probe.sby")
    tmp_sby.write_text(stale_sby)
    status, output = run(repo, checker, sby_path=tmp_sby)
    tmp_sby.unlink()
    if status == 0:
        red.append("the undefined-macro mutation still passed traps-arith-excluded-test.py")
    elif "reads executor.v without" not in output:
        red.append(f"the undefined-macro mutation failed for a different reason:\n{output}")

    if red:
        print()
        for why in red:
            print("*** " + why, file=sys.stderr)
        sys.exit(1)

    print("All three traps-arith-excluded-test.py mutations fail for their own reason.")

if __name__ == "__main__":
    main()
