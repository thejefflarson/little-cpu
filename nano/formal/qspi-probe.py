#!/usr/bin/env python3
"""Forces nano/formal/qspi.sby to fail against three mutations of nano_qspi_ctrl, one
per invariant the brief names, and requires the shipping controller to pass first.

Usage: qspi-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

Mirrors ill-e-probe.py's pattern: mutate nano/qspi.v's own source text, run sby against
a scratch copy, and read its status file. Not hermetic -- it runs sby four times (the
shipping control plus one mutant per invariant). Prerequisite of `make -C nano/formal
components_qspi`.
"""

import argparse
import pathlib
import shutil
import subprocess
import sys

MUTATIONS = {
    "cs-mutex": (
        "  assign psram_cs_n = (active_dev != DEV_PSRAM);\n",
        "  assign psram_cs_n = 1'b0;\n",
        "invariant 1 (CS0/CS1/CS2 never low together): tying psram_cs_n low "
        "unconditionally, so it reads low even while flash_cs_n also does, "
        "must be caught.",
    ),
    "cs-low-bound": (
        "    if (reset || psram_cs_n) psram_cs_low_count <= '0;\n",
        "    if (reset) psram_cs_low_count <= '0;\n",
        "invariant 2 (no PSRAM CS-low interval exceeds PSRAM_CS_LOW_LIMIT clocks): "
        "dropping the psram_cs_n term lets the counter run forever once released from "
        "reset, whether or not the chip select is actually low, and must be caught.",
    ),
    "queue-addressing": (
        "                slot1_addr  <= stream_next_addr;\n",
        "                slot1_addr  <= stream_next_addr + 31'd1;\n",
        "invariant 3 (the prefetch buffer holds exactly the parcels at "
        "[fetch_pc, fetch_pc+N)): tagging the second slot one parcel ahead of the "
        "parcel it actually received breaks the buffer's own contiguity check and "
        "must be caught.",
    ),
}


def stop(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(2)


def mutate(qspi_v, name, old, new):
    if old not in qspi_v:
        stop(
            f"nano/qspi.v no longer spells the '{name}' mutation site the way this "
            "probe mutates it. Re-anchor the mutation on the new spelling -- left "
            "alone this would build the shipping controller and prove nothing about "
            "a broken one."
        )
    return qspi_v.replace(old, new, 1)


def build_case(root, qspi_v, sby_src):
    shutil.rmtree(root, ignore_errors=True)
    nano_formal = root / "nano" / "formal"
    nano_formal.mkdir(parents=True)
    shutil.copy(sby_src, nano_formal / "qspi.sby")
    (root / "nano" / "qspi.v").write_text(qspi_v)
    return nano_formal


def run_case(workdir, sby, case, qspi_v, sby_src):
    nano_formal = build_case(workdir / case, qspi_v, sby_src)
    proc = subprocess.run(
        [sby, "-f", "qspi.sby"], cwd=nano_formal, capture_output=True, text=True
    )
    status_file = nano_formal / "qspi" / "status"
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
    parser.add_argument("--workdir", default=str(here / "qspi-probe"))
    parser.add_argument("--sby", default="sby")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    qspi_path = repo / "nano" / "qspi.v"
    sby_src = repo / "nano" / "formal" / "qspi.sby"
    for path in (qspi_path, sby_src):
        if not path.is_file():
            stop(f"{path} is missing, so there is nothing to probe.")

    workdir = pathlib.Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)

    qspi_v = qspi_path.read_text()
    red = []

    status = run_case(workdir, args.sby, "shipping", qspi_v, sby_src)
    print(f"shipping: {status}")
    if status != "PASS":
        red.append(
            "the shipping controller does not pass qspi.sby. That is what make -C "
            "nano/formal components_qspi is meant to prove about the design as it "
            "ships, so a control that starts red proves nothing about a mutant."
        )

    for name, (old, new, why) in MUTATIONS.items():
        mutant_v = mutate(qspi_v, name, old, new)
        status = run_case(workdir, args.sby, name, mutant_v, sby_src)
        print(f"{name}: {status}")
        if status != "FAIL":
            red.append(f"the '{name}' mutant passes qspi.sby. {why}")

    if red:
        print()
        for why in red:
            print("*** " + why.replace("\n", "\n*** "), file=sys.stderr)
        sys.exit(1)

    print("\nEach mutant fails qspi.sby for its own invariant, and the shipping "
          "controller passes it.")


if __name__ == "__main__":
    main()
