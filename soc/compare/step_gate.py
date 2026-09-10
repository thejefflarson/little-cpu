#!/usr/bin/env python3
"""Grade an up5k placement against the clock the part can actually be given.

WHY THIS IS A GATE AND NOT A SCORE. up5k's clock is a step function. The board
offers the 12 MHz crystal, and the part's own `SB_HFOSC` offers 48, 24, 12 and 6
MHz -- nothing between them. So a core that places at 22 MHz on this part runs at
12 MHz, exactly like a core that places at 12.1, and the 10 MHz of margin buys
nothing that any program can observe. Treating Fmax as a continuous factor here
counts an advantage that does not exist on any part this project ships.

The comparison on this part is therefore PASS/FAIL and then cycles. A core over
the step clears the gate and its surplus is reported as unspendable; a core under
it does not score a fraction, because the next step down is 6 MHz and a design
that has to halve its clock is not slower in the comparison, it is out of it.

The other half of the argument lives on ECP5, where `EHXPLLL` synthesises
ref x M / N / D on a fine grid: Fmax times cycles is a real product there, and
`make compare-timing COMPARE_PART=ecp5` is what measures it. The two parts are
never averaged and never merged.

Reads the same `icetime -r` report `make compare-timing` already writes, through
soc/timing_split.py's walk rather than a second parser of its own.

  step_gate.py compare.littlecpu.timing.rpt --core littlecpu --step 12.0
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from timing_split import summarise  # noqa: E402

# Every clock an up5k design can be given: the board's crystal, and SB_HFOSC's four
# divider settings. All four rather than just the floor, so a refusal can name the step
# BELOW -- the clock a failing core would actually have to run at.
UP5K_STEPS = (48.0, 24.0, 12.0, 6.0)

def reached(mhz, steps):
    """The fastest step at or under `mhz`, or None if the design is under all of them."""
    for step in sorted(steps, reverse=True):
        if mhz >= step:
            return step
    return None

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", help="output of `icetime -t -r <report> <asc>`")
    parser.add_argument("--core", required=True, help="named in the verdict")
    parser.add_argument(
        "--step",
        type=float,
        required=True,
        help="the oscillator step this comparison requires, in MHz",
    )
    args = parser.parse_args()

    if args.step not in UP5K_STEPS:
        sys.exit(
            f"*** --step {args.step:g} is not a clock this part offers. up5k has "
            f"{', '.join(f'{s:g}' for s in UP5K_STEPS)} MHz and nothing between "
            "them, so a requirement set anywhere else grades against a frequency "
            "no board can supply."
        )

    mhz = 1000 / summarise(args.report)["total"]
    step = reached(mhz, UP5K_STEPS)
    below = max((s for s in UP5K_STEPS if s < args.step), default=None)

    if mhz < args.step:
        sys.exit(
            f"\n*** {args.core}: {mhz:.2f} MHz is under the {args.step:g} MHz step.\n"
            f"*** up5k's next clock down is "
            + (f"{below:g} MHz" if below else "nothing at all")
            + ", so this core does not run\n"
            "*** slower in the comparison -- it is out of it. That is a FAIL, not a\n"
            "*** fractional score, and averaging it with a passing core's cycles\n"
            "*** would report a machine nobody can build."
        )

    surplus = mhz - step
    print(f"{args.core}: places at {mhz:.2f} MHz, clears the {args.step:g} MHz step")
    print(f"  runs at     : {step:g} MHz -- the fastest step up5k offers at or under "
          f"{mhz:.2f}")
    print(f"  unspendable : {surplus:.2f} MHz of margin, which no program can observe")
    print(f"STEP GATE: {args.core} PASSES at {step:g} MHz -- compare it on cycles")

if __name__ == "__main__":
    main()
