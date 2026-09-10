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
normally ABOVE the standalone one. The floor is a fraction rather than an
equality because packing,
`abc9` and the harness's own glue all move the number by more than a percent,
and because nothing here needs to be precise: the defect it catches is a factor
of four.

An all-NOP ROM no longer folds this core, so it is not the stimulus to reach for
when checking that this gate still discriminates. That image only makes every
ROM word identical, which collapses a read-only instruction array -- VexRiscv's,
still red at 0.64x -- and this core's instruction memory carries a write port
the design drives, so its contents are never a constant whatever it holds. What
folds this side makes the datapath dead rather than uniform: the same image with
that write port disconnected, a core given a constant instruction instead of the
memory's answer, and a core port left unconnected, which is how this gate last
went red for real. All three still place and icetime still times the fragment,
which is what makes them the shape of defect worth catching.

Reads nextpnr's utilisation table for the placed count and yosys's cell census
for the synthesised one, both of which the flow already writes. It does not
re-run either tool, so `test/probe_gates.sh` can drive it against fixture logs.

`PARTS` names both cells per fabric -- `ICESTORM_LC` against `SB_LUT4` on up5k,
`TRELLIS_COMB` against `LUT4` on ECP5 -- and `--part` is required with no
default. A part with no row here is refused rather than read with the other
part's names, which would find no count at all and report as a broken log rather
than as the wrong question.
"""

import argparse
import re
import sys

PARTS = {
    "up5k": {
        "placed_cell": "ICESTORM_LC",
        "synth_cell": "SB_LUT4",
        "placed": re.compile(r"ICESTORM_LC:\s*(\d+)\s*/\s*(\d+)"),
        "synth": re.compile(r"^\s+(\d+)\s+SB_LUT4\s*$"),
    },
    "ecp5": {
        "placed_cell": "TRELLIS_COMB",
        "synth_cell": "LUT4",
        "placed": re.compile(r"TRELLIS_COMB:\s*(\d+)\s*/\s*(\d+)"),
        "synth": re.compile(r"^\s+(\d+)\s+LUT4\s*$"),
    },
}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pnr_log", help="nextpnr-ice40 log for the harness")
    parser.add_argument("synth_log", help="yosys log for the core synthesised alone")
    parser.add_argument("core", help="the core's name, printed with the verdict")
    parser.add_argument(
        "--part",
        required=True,
        choices=sorted(PARTS),
        help="whose cell names to read out of the two logs; there is no default",
    )
    parser.add_argument(
        "--min-ratio",
        type=float,
        required=True,
        help="placed logic cells / standalone LUTs must be at least this",
    )
    args = parser.parse_args()
    names = PARTS[args.part]

    placed = None
    for line in open(args.pnr_log):
        found = names["placed"].search(line)
        if found:
            placed = int(found.group(1))
    if placed is None:
        sys.exit(
            f"{args.pnr_log}: no {names['placed_cell']} utilisation line for "
            f"{args.part}. nextpnr did not finish placing, so there is no placed "
            f"design to check."
        )

    # The LAST census, matching soc/cell_census.py: yosys prints one per pass and the
    # final one is the whole-design total.
    synthesised = None
    for line in open(args.synth_log):
        found = names["synth"].match(line)
        if found:
            synthesised = int(found.group(1))
    if synthesised is None or synthesised == 0:
        sys.exit(
            f"{args.synth_log}: no {names['synth_cell']} count for {args.part}. "
            f"Without the standalone number there is nothing to compare the "
            f"placement against."
        )

    ratio = placed / synthesised
    print(
        f"{args.core}: {placed} placed {names['placed_cell']} against "
        f"{synthesised} {names['synth_cell']} synthesised alone -- {ratio:.2f}x"
    )
    if ratio < args.min_ratio:
        sys.exit(
            f"\n*** {args.core}: the placed design is {ratio:.2f}x the core's own\n"
            f"*** synthesis, under the {args.min_ratio:.2f}x floor. Most of the core is\n"
            f"*** not in what was placed, so the timing number describes a fragment.\n"
            f"*** The usual cause is a harness whose outputs do not depend on the\n"
            f"*** datapath -- an unconnected core port and an all-NOP ROM both do\n"
            f"*** exactly this. Fix the harness or the program; do not lower the floor."
        )
    print(f"RATCHET: {ratio:.2f}x against a {args.min_ratio:.2f}x floor -- OK")

if __name__ == "__main__":
    main()
