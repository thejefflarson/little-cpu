#!/usr/bin/env python3
"""Forces imemcheck.sv/dmemcheck.sv's own cover goal to go unreached, and requires
the matching *_cover.sby to fail because of it rather than pass regardless of the
environment -- complete-cover-probe.py's shape, generalised over both memchecks and
both cores.

Usage: memcheck-cover-probe.py --harness {formal,nano/formal} --check {imemcheck,dmemcheck}
                                [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. Neither memcheck states a cover goal proving it ever reaches the
property it names, so an over-constraining assume edit could pass vacuously with CI
green -- the same gap complete_cover closes for `complete`. One mutant is built, one
line away from the shipping harness: on littlecpu it assumes `fetch_stall`, so nothing
ever issues; on nano it assumes `mem_ready` low, so nano.v never leaves its wait state.
Either way rvfi_valid never rises, so the cover goal must go unreached.

NOT HERMETIC -- it runs sby, up to twice, so it is a Makefile prerequisite of the
*_cover targets rather than of `make test`. test/probe_gates.sh covers this file's own
logic against a stub sby.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

# Grown in place: the line stays the same length class either way, so nothing else in
# either file is pinned by line number.
STALLED_BUS_ANCHOR = "  logic trap;\n"
LITTLECPU_MUTANT = "  logic trap; always_comb assume(fetch_stall);\n"
NANO_MUTANT = "  logic trap; always_comb assume(mem_ready == 1'b0);\n"

LITTLECPU_RTL = (
    "structs.v", "fetcher.v", "regfile.v", "csrs.v", "decoder.v",
    "regsel.v", "executor.v", "accessor.v", "writeback.v", "littlecpu.v",
)


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mutate(sv_text, is_nano):
    if STALLED_BUS_ANCHOR not in sv_text:
        stop(
            "no longer spells what the stalled-bus mutation replaces. Re-anchor it "
            "on the new spelling -- left alone it would build the shipping harness "
            "and prove nothing about a stalled bus."
        )
    return sv_text.replace(STALLED_BUS_ANCHOR, NANO_MUTANT if is_nano else LITTLECPU_MUTANT, 1)


def build_case(repo, root, harness, check, sv_text):
    """A copy of `harness`, deep enough that {check}_cover.sby's own relative paths
    resolve, with {check}.sv replaced. Returns the directory sby must be run from."""
    is_nano = harness == "nano/formal"
    shutil.rmtree(root, ignore_errors=True)
    harness_dir = root / harness
    harness_dir.mkdir(parents=True)
    shutil.copy(repo / harness / f"{check}_cover.sby", harness_dir / f"{check}_cover.sby")
    (harness_dir / f"{check}.sv").write_text(sv_text)
    if is_nano:
        (root / "nano").mkdir(exist_ok=True)
        shutil.copy(repo / "nano" / "nano.v", root / "nano" / "nano.v")
    else:
        shutil.copy(repo / "formal" / "arbiter.v", harness_dir / "arbiter.v")
        rtl_dir = root / "rtl"
        rtl_dir.mkdir()
        for name in LITTLECPU_RTL:
            shutil.copy(repo / "rtl" / name, rtl_dir / name)
    riscv_formal = repo / "formal" / "riscv-formal"
    if not riscv_formal.is_dir():
        stop(
            f"{riscv_formal} is missing. Fetch the pin first, e.g. by running\n"
            f"make -C {harness} {check} once."
        )
    (root / "formal").mkdir(exist_ok=True)
    (root / "formal" / "riscv-formal").symlink_to(riscv_formal)
    return harness_dir


def run_case(repo, workdir, sby, harness, check, case, sv_text):
    harness_dir = build_case(repo, workdir / case, harness, check, sv_text)
    job = f"{check}_cover"
    proc = subprocess.run(
        [sby, "-f", f"{job}.sby"], cwd=harness_dir, capture_output=True, text=True
    )
    status_file = harness_dir / job / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} case, so nothing was proved or\n"
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} case is empty.")
    return status[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--harness", required=True, choices=("formal", "nano/formal"))
    parser.add_argument("--check", required=True, choices=("imemcheck", "dmemcheck"))
    here = pathlib.Path(__file__).resolve().parent
    parser.add_argument("--repo", default=str(here.parent), help="tree to read formal/ and nano/ from")
    parser.add_argument("--workdir", default=str(here / "memcheck-cover-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    harness, check = args.harness, args.check
    is_nano = harness == "nano/formal"
    names = [f"{harness}/{check}_cover.sby", f"{harness}/{check}.sv"]
    names.append("nano/nano.v" if is_nano else "formal/arbiter.v")
    for name in names:
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    sv_text = (repo / harness / f"{check}.sv").read_text()

    red = []

    status = run_case(repo, workdir, args.sby, harness, check, "shipping", sv_text)
    print(f"shipping: {status}")
    if status != "PASS":
        red.append(
            "the shipping harness does not reach its own cover goal. That is what\n"
            f"make -C {harness} {check}_cover is meant to prove about the design as "
            "it ships, so a control that starts red proves nothing about a mutant."
        )

    status = run_case(repo, workdir, args.sby, harness, check, "stalled-bus",
                       mutate(sv_text, is_nano))
    print(f"stalled-bus: {status}")
    if status != "FAIL":
        red.append(
            "the stalled-bus mutant proves. Stalling the bus is exactly what should "
            "make\nthe retire-gated cover goal unreachable, so an anti-vacuity "
            "control that cannot go red is not a control."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The stalled-bus mutant makes the cover goal unreachable, and the shipping "
          "harness reaches it.")


if __name__ == "__main__":
    main()
