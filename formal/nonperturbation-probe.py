#!/usr/bin/env python3
"""Forces check-nonperturbation.py red against a core that reads its own instrumentation,
and requires the shipping core to pass it first.

Usage: nonperturbation-probe.py {littlecpu,nano} [--repo DIR] [--workdir DIR]
                                [--checker FILE]

WHY THIS EXISTS. The structural gate has three controls that stop it passing vacuously, but
nothing had ever shown that it goes red when an `ifdef RISCV_FORMAL` value reaches a real
signal, which is the one thing it is for. The mutant here is the smallest such leak: a real
output, the memory write strobe on nano and the trap output on littlecpu, ORs in bit 0 of
an rvfi_* output port, under `ifdef RISCV_FORMAL so the plain build still elaborates: without
it the gold build fails to compile and the gate goes red for the wrong reason. That bit survives `delete -port ...rvfi_*` plus the fanout sweep
because a real output now depends on it, so the gate netlist keeps the shadow register and
the logic feeding it and must differ from gold.

The checker is copied into a scratch tree beside the mutated sources, because it locates
its sources from its own path. NOT HERMETIC -- it runs yosys, twice per design. It is a
prerequisite of both Makefiles' `nonperturbation` target, and test/probe_gates.sh covers this
file's own logic against a stub checker.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys

DESIGNS = {
    "littlecpu": {
        "sources": [f"rtl/{f}" for f in (
            "structs.v", "fetcher.v", "regfile.v", "csrs.v", "decoder.v", "regsel.v",
            "executor.v", "accessor.v", "writeback.v", "littlecpu.v")],
        "mutated": "rtl/littlecpu.v",
        "old": "  assign trap = decoder_trap_entry;\n",
        "new": ("`ifdef RISCV_FORMAL\n  assign trap = decoder_trap_entry | rvfi_csr_mscratch_wdata[0];\n"
                "`else\n  assign trap = decoder_trap_entry;\n`endif\n"),
    },
    "nano": {
        "sources": ["nano/nano.v"],
        "mutated": "nano/nano.v",
        "old": "  assign mem_wstrb = cpu_state == finish_store ? store_wstrb : 4'b0000;\n",
        "new": ("`ifdef RISCV_FORMAL\n  assign mem_wstrb = cpu_state == finish_store ? store_wstrb : {3'b000, rvfi_mem_wdata[0]};\n"
                "`else\n  assign mem_wstrb = cpu_state == finish_store ? store_wstrb : 4'b0000;\n`endif\n"),
    },
}


def stop(message):
    """Exit 2: the probe's own inputs are broken, which is not a red proof."""
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def build_case(repo, root, design, mutate, checker):
    shutil.rmtree(root, ignore_errors=True)
    for name in design["sources"]:
        text = (repo / name).read_text()
        if mutate and name == design["mutated"]:
            if design["old"] not in text:
                stop(
                    f"{name} no longer spells the line this probe mutates. Re-anchor it\n"
                    "on the new spelling -- left alone it would build the shipping core\n"
                    "and prove nothing about a leak."
                )
            text = text.replace(design["old"], design["new"], 1)
        target = root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)
    (root / "formal").mkdir(exist_ok=True)
    shutil.copy(checker, root / "formal" / "check-nonperturbation.py")
    riscv_formal = repo / "formal" / "riscv-formal"
    if not riscv_formal.is_dir():
        stop(f"{riscv_formal} is missing. Run make -C formal riscv-formal first.")
    (root / "formal" / "riscv-formal").symlink_to(riscv_formal)


def run_case(root, which):
    proc = subprocess.run(
        [sys.executable, str(root / "formal" / "check-nonperturbation.py"), which],
        capture_output=True, text=True,
    )
    return proc


def main():
    here = pathlib.Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("design", choices=sorted(DESIGNS))
    parser.add_argument("--repo", default=str(here.parent), help="tree to read sources from")
    parser.add_argument("--workdir", default=str(here / "nonperturbation-probe"))
    parser.add_argument("--checker", default=str(here / "check-nonperturbation.py"))
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    workdir = pathlib.Path(args.workdir).resolve()
    checker = pathlib.Path(args.checker).resolve()
    design = DESIGNS[args.design]
    for path in [repo / name for name in design["sources"]] + [checker]:
        if not path.is_file():
            stop(f"{path} is missing, so there is nothing to probe.")

    red = []

    build_case(repo, workdir / "shipping", design, False, checker)
    proc = run_case(workdir / "shipping", args.design)
    print(f"shipping: exit {proc.returncode}")
    if proc.returncode != 0:
        red.append(
            "the shipping core does not pass the non-perturbation gate, so a control\n"
            "that starts red proves nothing about a leaking mutant.\n" + proc.stdout + proc.stderr
        )

    build_case(repo, workdir / "leak", design, True, checker)
    proc = run_case(workdir / "leak", args.design)
    print(f"leak (a real output reads an rvfi_* port): exit {proc.returncode}")
    if proc.returncode == 2:
        stop("the checker could not run on the leaking mutant:\n" + proc.stdout + proc.stderr)
    if (proc.returncode != 1 or "RVFI NON-PERTURBATION: FAIL" not in proc.stdout
            or "cell histogram:                    DIFFERS" not in proc.stdout):
        red.append(
            "the leaking mutant passes. A real output that reads an instrumentation\n"
            "port is exactly what the gate exists to refuse, so a gate that cannot go\n"
            "red against it is not standing between RISCV_FORMAL and the shipped core."
        )

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("The leaking mutant fails the gate, and the shipping core passes it.")


if __name__ == "__main__":
    main()
