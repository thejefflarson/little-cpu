#!/usr/bin/env python3
"""Forces nano/formal/memreq.sby to fail against two mutated nano.v: one changes a
store's address mid-request, one lets a jalr target keep its bit 0 (which the FORMAL
block's alignment assertions cover). The shipping core must pass first.

Usage: memreq-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

Mirrors qspi-probe.py's pattern: mutate nano/nano.v's own source text, run sby against
a scratch copy, and read its status file. Not hermetic -- it runs sby twice (the
shipping control plus the mutant). Prerequisite of `make -C nano/formal components_memreq`.
"""

import argparse
import pathlib
import re
import shutil
import subprocess
import sys

OLD = """        finish_store: begin
          if (mem_ready) begin
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            next_pc <= pc + pc_inc;
          end
        end
"""
NEW = """        finish_store: begin
          if (mem_ready) begin
            cpu_state <= fetch_instr;
            mem_valid <= 0;
            next_pc <= pc + pc_inc;
          end else begin
            mem_addr <= mem_addr + 32'd4;
          end
        end
"""


JALR_OLD = "($signed(immediate) + $signed(`RF_RS1)) & 32'hfffffffe :"
JALR_NEW = "($signed(immediate) + $signed(`RF_RS1)) :"


def stop(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def build_case(root, nano_v, sby_src):
    shutil.rmtree(root, ignore_errors=True)
    nano_formal = root / "nano" / "formal"
    nano_formal.mkdir(parents=True)
    shutil.copy(sby_src, nano_formal / "memreq.sby")
    (root / "nano" / "nano.v").write_text(nano_v)
    return nano_formal


def run_case(workdir, sby, case, nano_v, sby_src):
    nano_formal = build_case(workdir / case, nano_v, sby_src)
    proc = subprocess.run(
        [sby, "-f", "memreq.sby"], cwd=nano_formal, capture_output=True, text=True
    )
    status_file = nano_formal / "memreq" / "status"
    if not status_file.is_file():
        stop(
            f"sby wrote no status for the {case} case, so nothing was proved or "
            "disproved. Its output follows.\n\n" + proc.stdout + proc.stderr
        )
    status = status_file.read_text().split()
    if not status:
        stop(f"sby's status file for the {case} case is empty.")
    return status[0]


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo", default=str(here.parent.parent), help="tree to read nano/ from"
    )
    parser.add_argument("--workdir", default=str(here / "memreq-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    nano_path = repo / "nano" / "nano.v"
    sby_src = repo / "nano" / "formal" / "memreq.sby"
    for path in (nano_path, sby_src):
        if not path.is_file():
            stop(f"{path} is missing, so there is nothing to probe.")

    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    nano_v = nano_path.read_text()
    red = []

    status = run_case(workdir, args.sby, "shipping", nano_v, sby_src)
    print(f"shipping: {status}")
    if status != "PASS":
        red.append(
            "the shipping core does not pass memreq.sby. That is what make -C "
            "nano/formal components_memreq is meant to prove about the design as it "
            "ships, so a control that starts red proves nothing about a mutant."
        )

    if OLD not in nano_v:
        stop(
            "nano/nano.v no longer spells finish_store the way this probe mutates it. "
            "Re-anchor the mutation on the new spelling -- left alone this would build "
            "the shipping core twice and prove nothing about a broken one."
        )
    mutant_v = nano_v.replace(OLD, NEW, 1)
    status = run_case(workdir, args.sby, "addr-mid-request", mutant_v, sby_src)
    print(f"addr-mid-request: {status}")
    if status != "FAIL":
        red.append(
            "the 'addr-mid-request' mutant passes memreq.sby. A store that bumps "
            "mem_addr on every wait cycle instead of holding it must be caught."
        )

    if JALR_OLD not in nano_v:
        stop(
            "nano/nano.v no longer spells jump_address's jalr mask the way this probe "
            "mutates it. Re-anchor the mutation on the new spelling."
        )
    mutant_v = nano_v.replace(JALR_OLD, JALR_NEW, 1)
    status = run_case(workdir, args.sby, "unmasked-jalr", mutant_v, sby_src)
    print(f"unmasked-jalr: {status}")
    if status != "FAIL":
        red.append(
            "the 'unmasked-jalr' mutant passes memreq.sby. A jalr target that keeps its "
            "bit 0 reaches next_pc and must be caught by the alignment assertions."
        )
    else:
        log = (workdir / "unmasked-jalr" / "nano" / "formal" / "memreq" / "logfile.txt").read_text()
        lines = mutant_v.splitlines()
        aligned = {
            i + 1 for i, line in enumerate(lines)
            if line.strip() in ("assert(!pc[0]);", "assert(!next_pc[0]);", "assert(!mem_addr[0]);")
        }
        failed = {int(m) for m in re.findall(r"Assert failed in riscv: nano\.v:(\d+)", log)}
        if not failed & aligned:
            red.append(
                "the 'unmasked-jalr' mutant fails memreq.sby, but not at an alignment "
                f"assertion (failed at nano.v lines {sorted(failed)}, wanted one of "
                f"{sorted(aligned)}), so the probe is red for the wrong reason."
            )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("\nThe mutant fails memreq.sby, and the shipping core passes it.")


if __name__ == "__main__":
    main()
