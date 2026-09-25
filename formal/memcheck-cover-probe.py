#!/usr/bin/env python3
"""Forces imemcheck.sv/dmemcheck.sv's own cover goal to go unreached, and requires
the matching *_cover.sby to fail because of it rather than pass regardless of the
environment -- complete-cover-probe.py's shape, generalised over both memchecks and
both cores.

Usage: memcheck-cover-probe.py --harness {formal,nano/formal}
                                --check {imemcheck,dmemcheck,imemcheck_latch,dmemcheck_latch}
                                [--repo DIR] [--workdir DIR] [--sby SBY]

A `_latch` check runs {check}_cover.sby (its own clk2fflogic script and doubled
depth) against the same {base}.sv the flop check reads unmodified -- NANO_LATCH_RF
lives in nano.v, not in the checker -- so the mutation and the cover-log parse both
key off the base name.

WHY THIS EXISTS. Neither memcheck states a cover goal proving it ever reaches the
property it names, so an over-constraining assume edit could pass vacuously with CI
green -- the same gap complete_cover closes for `complete`. One mutant is built without
moving a line of the shipping harness: on littlecpu it ties the core's own fetch_stall
input high, so decode never issues; on nano it assumes mem_ready low, so nano.v never
leaves its wait state. Either way rvfi_valid never rises, so the goal must go unreached.

The mutant also states a trivially true sentinel, `cover property (!reset)`, which must
be REACHED. A mutant whose assumptions contradict each other has no traces, and sby then
reports every goal unreached, trivially true ones included, so its FAIL says nothing.
Assuming the arbiter's fetch_stall register high was such a mutant: the register starts
at zero. Goals are read per site from sby's log, not from its one-word status.

NOT HERMETIC -- it runs sby, twice, so it is a Makefile prerequisite of the *_cover
targets rather than of `make test`. test/probe_gates.sh covers this file's own logic
against a stub sby.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import cover_log

# Grown in place, and the littlecpu port is rewritten in place, so no line of either
# harness moves and every cover site keeps the name sby gave it in the shipping run.
ANCHOR = "  logic trap;\n"
SENTINEL = "  logic trap; cover property (!reset);"
NANO_STALL = " always_comb assume(mem_ready == 1'b0);"
LITTLECPU_INSTANCE = "littlecpu uut ("
LITTLECPU_PORT = ".fetch_stall(fetch_stall)"
LITTLECPU_TIED = ".fetch_stall(1'b1)"

LITTLECPU_RTL = (
    "structs.v", "fetcher.v", "regfile.v", "csrs.v", "decoder.v",
    "regsel.v", "executor.v", "accessor.v", "writeback.v", "littlecpu.v",
)


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def base_check(check):
    """The checker .sv a `_latch` variant still reads unmodified."""
    return check[: -len("_latch")] if check.endswith("_latch") else check


def mutate(sv_text, is_nano):
    if ANCHOR not in sv_text:
        stop(
            "no longer spells what the stalled-bus mutation replaces. Re-anchor it "
            "on the new spelling -- left alone it would build the shipping harness "
            "and prove nothing about a stalled bus."
        )
    text = sv_text.replace(ANCHOR, SENTINEL + (NANO_STALL if is_nano else "") + "\n", 1)
    if is_nano:
        return text
    start = text.find(LITTLECPU_INSTANCE)
    port = text.find(LITTLECPU_PORT, start) if start >= 0 else -1
    if port < 0:
        stop(
            f"no longer connects the core as `{LITTLECPU_PORT}` inside "
            f"`{LITTLECPU_INSTANCE}`, so the stalled-bus mutation has no port to tie "
            "high. Re-anchor it on the new spelling."
        )
    return text[:port] + LITTLECPU_TIED + text[port + len(LITTLECPU_PORT):]


def build_case(repo, root, harness, check, sv_text):
    """A copy of `harness`, deep enough that {check}_cover.sby's own relative paths
    resolve, with {base}.sv replaced (the checker source the .sby actually reads --
    see base_check)."""
    is_nano = harness == "nano/formal"
    base = base_check(check)
    shutil.rmtree(root, ignore_errors=True)
    harness_dir = root / harness
    harness_dir.mkdir(parents=True)
    shutil.copy(repo / harness / f"{check}_cover.sby", harness_dir / f"{check}_cover.sby")
    (harness_dir / f"{base}.sv").write_text(sv_text)
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
    """Builds the case, runs its cover job, and returns (status, per-site sets)."""
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
    log_file = harness_dir / job / "logfile.txt"
    if not log_file.is_file():
        stop(f"sby wrote no log for the {case} case, so no goal can be read.")
    return status[0], cover_log.parse(log_file.read_text(), f"{base_check(check)}.sv")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--harness", required=True, choices=("formal", "nano/formal"))
    parser.add_argument(
        "--check",
        required=True,
        choices=("imemcheck", "dmemcheck", "imemcheck_latch", "dmemcheck_latch"),
    )
    here = pathlib.Path(__file__).resolve().parent
    parser.add_argument("--repo", default=str(here.parent), help="tree to read formal/ and nano/ from")
    parser.add_argument("--workdir", default=str(here / "memcheck-cover-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    harness, check = args.harness, args.check
    base = base_check(check)
    is_nano = harness == "nano/formal"
    names = [f"{harness}/{check}_cover.sby", f"{harness}/{base}.sv"]
    names.append("nano/nano.v" if is_nano else "formal/arbiter.v")
    for name in names:
        if not (repo / name).is_file():
            stop(f"{name} is missing from {repo}, so there is nothing to probe.")
    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    sv_text = (repo / harness / f"{base}.sv").read_text()
    mutant = mutate(sv_text, is_nano)

    red = []

    status, ship = run_case(repo, workdir, args.sby, harness, check, "shipping", sv_text)
    goals = ship["reached"] | ship["unreached"]
    print(f"shipping: {status}, goals reached {sorted(ship['reached']) or 'none'}")
    if not goals:
        stop(
            f"the shipping case's log names no cover statement in {check}.sv, so "
            "sby's wording has moved and there is no goal to grade."
        )
    if status != "PASS" or ship["unreached"]:
        red.append(
            "the shipping harness does not reach its own cover goal. That is what\n"
            f"make -C {harness} {check}_cover is meant to prove about the design as "
            "it ships, so a control that starts red proves nothing about a mutant."
        )

    status, mut = run_case(repo, workdir, args.sby, harness, check, "stalled-bus", mutant)
    named = mut["reached"] | mut["unreached"]
    sentinel = named - goals
    print(f"stalled-bus: {status}, sentinel {sorted(sentinel)}, "
          f"reached {sorted(mut['reached']) or 'none'}")
    if len(sentinel) != 1 or not goals <= named:
        stop(
            f"the stalled-bus case's log names {sorted(named)}, which is not the "
            f"shipping goal {sorted(goals)} plus the one sentinel the mutant states, so "
            "it cannot be read."
        )
    if not sentinel <= mut["reached"]:
        red.append(
            "the stalled-bus mutant does not reach even its trivially true sentinel, so\n"
            "it has no traces at all: its assumptions contradict each other, and its FAIL\n"
            "would be the same for a goal that no longer needs a retire."
        )
    if status != "FAIL" or goals & mut["reached"]:
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

    print("The stalled-bus mutant reaches its sentinel and not the cover goal, and the "
          "shipping harness reaches the goal.")


if __name__ == "__main__":
    main()
