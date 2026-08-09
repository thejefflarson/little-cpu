#!/usr/bin/env python3
"""Check that the placed harness still contains the core it claims to measure.

THIS IS THE GATE THAT MAKES THE COMPARISON MEAN ANYTHING. The first attempt at
this measurement filled the ROM with NOPs. With no store ever executing, every
output of the design is provably constant, so yosys deleted the datapath: 449
placed logic cells against the 1711 the same core synthesises to on its own, and
a critical path of 33.69 ns through the fragment that was left. `(* keep *)` on
the instance held the cell and did not stop the folding inside it. Nothing in
the flow said a word.

So the placed cell count is compared against what the core synthesises to
ON ITS OWN, with no harness around it to fold against. The harness adds
memories and a few registers and takes nothing away, so the placed count is
normally ABOVE the standalone one -- 1.13x for this core and 1.41x for VexRiscv
as measured. The floor is a fraction rather than an equality because packing,
`abc9` and the harness's own glue all move the number by more than a percent,
and because nothing here needs to be precise: the defect it catches is a factor
of four.

Reads nextpnr's utilisation table for the placed count and yosys's cell census
for the synthesised one, both of which the flow already writes. It does not
re-run either tool, so `test/probe_gates.sh` can drive it against fixture logs.
"""

import argparse
import re
import sys

# nextpnr's utilisation table: `Info: \t ICESTORM_LC:  7447/  7680    96%`
PLACED = re.compile(r"ICESTORM_LC:\s*(\d+)\s*/\s*(\d+)")
# yosys's `stat` census, the same line shape soc/cell_census.py matches.
SYNTH = re.compile(r"^\s+(\d+)\s+SB_LUT4\s*$")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pnr_log", help="nextpnr-ice40 log for the harness")
    parser.add_argument("synth_log", help="yosys log for the core synthesised alone")
    parser.add_argument("core", help="the core's name, printed with the verdict")
    parser.add_argument(
        "--min-ratio",
        type=float,
        required=True,
        help="placed ICESTORM_LC / standalone SB_LUT4 must be at least this",
    )
    args = parser.parse_args()

    placed = None
    for line in open(args.pnr_log):
        found = PLACED.search(line)
        if found:
            placed = int(found.group(1))
    if placed is None:
        sys.exit(
            f"{args.pnr_log}: no ICESTORM_LC utilisation line. nextpnr did not "
            f"finish placing, so there is no placed design to check."
        )

    # The LAST census, matching soc/cell_census.py: yosys prints one per pass
    # and the final one is the whole-design total.
    synthesised = None
    for line in open(args.synth_log):
        found = SYNTH.match(line)
        if found:
            synthesised = int(found.group(1))
    if synthesised is None or synthesised == 0:
        sys.exit(
            f"{args.synth_log}: no SB_LUT4 count. Without the standalone number "
            f"there is nothing to compare the placement against."
        )

    ratio = placed / synthesised
    print(
        f"{args.core}: {placed} placed ICESTORM_LC against {synthesised} SB_LUT4 "
        f"synthesised alone -- {ratio:.2f}x"
    )
    if ratio < args.min_ratio:
        sys.exit(
            f"\n*** {args.core}: the placed design is {ratio:.2f}x the core's own\n"
            f"*** synthesis, under the {args.min_ratio:.2f}x floor. Most of the core is\n"
            f"*** not in what was placed, so the timing number describes a fragment.\n"
            f"*** The usual cause is a harness whose outputs do not depend on the\n"
            f"*** datapath -- an all-NOP ROM does exactly this. Fix the program or\n"
            f"*** the harness; do not lower the floor."
        )
    print(f"RATCHET: {ratio:.2f}x against a {args.min_ratio:.2f}x floor -- OK")


if __name__ == "__main__":
    main()
