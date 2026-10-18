#!/usr/bin/env python3
"""Asserts that the iCESugar-Pro's entropy source hands rtl/trng.v an edge period the
measured jitter can fill and the edge timeout cannot trip.

Usage: entropy_geometry_test.py [repo-root]     # defaults to this script's parent

The nominal edge period in core cycles is CORE_HZ * DIV * 2^DIV_LOG2 / OSCG_NOMINAL_HZ.
OSCG's edge-to-edge jitter is a fixed time (about 50 ns rms at 105 us), so how many core
cycles it spans, and so how much of rtl/trng.v's two sampled counter bits it randomises,
is set by this one product. Below MIN_CYCLES the measured min-entropy falls under the
0.5 bit per sample the adaptive proportion cutoff assumes; at MAX_CYCLES a source 20%
slow trips the 8,192-cycle edge timeout and reads DEAD.

WHAT IT CANNOT SEE. Whether the silicon's OSCG runs at its nominal 310 MHz; that is the
capture in soc/entropy_assess.py, and the numbers below come from it.
"""

import argparse
import os
import re
import sys

ENTROPY = "soc/icesugar_pro_entropy.v"
BOARD = "soc/board_icesugar_pro.v"
TRNG = "rtl/trng.v"

OSCG_NOMINAL_HZ = 310_000_000
MIN_CYCLES = 3000
TIMEOUT_MARGIN = 0.8


def read(root, rel):
    try:
        with open(os.path.join(root, rel)) as handle:
            return handle.read()
    except OSError as exc:
        sys.exit("error: cannot read %s: %s. This check grades the agreement between "
                 "three files and cannot grade a missing one." % (rel, exc))


def parameter(text, name, rel):
    found = re.findall(r"parameter\s+integer\s+%s\s*=\s*([0-9]+)" % name, text)
    if len(found) != 1:
        sys.exit("error: %s states %d defaults for %s, expected exactly one." % (rel, len(found), name))
    return int(found[0])


def check(root):
    entropy = read(root, ENTROPY)
    board = read(root, BOARD)
    trng = read(root, TRNG)

    div = parameter(entropy, "DIV", ENTROPY)
    log2 = parameter(entropy, "DIV_LOG2", ENTROPY)
    core = re.findall(r"localparam\s+integer\s+CORE_HZ\s*=\s*([0-9_]+)\s*;", board)
    if len(core) != 1:
        sys.exit("error: %s states %d CORE_HZ, expected exactly one." % (BOARD, len(core)))
    core_hz = int(core[0].replace("_", ""))
    width = re.findall(r"logic\s+\[(\d+):0\]\s+ticks\s*;", trng)
    if len(width) != 1:
        sys.exit("error: %s declares %d `ticks` counters, expected exactly one." % (TRNG, len(width)))
    if len(re.findall(r"logic\s+late\s*;", trng)) != 1:
        sys.exit("error: %s declares no `late` flag, so the timeout is not two passes of `ticks`." % TRNG)
    timeout = 2 ** (int(width[0]) + 2)

    errors = []
    if not 2 <= div <= 128:
        errors.append("%s sets OSCG DIV to %d, outside the primitive's 2..128." % (ENTROPY, div))
    if log2 < 1:
        errors.append("%s divides by 2^%d in fabric; the output must be a register bit." % (ENTROPY, log2))

    period = core_hz * div * 2 ** log2 / OSCG_NOMINAL_HZ
    ceiling = timeout * TIMEOUT_MARGIN
    if period < MIN_CYCLES:
        errors.append("the nominal edge period is %.0f core cycles, under the %d at which the "
                      "measured min-entropy reached 0.5 bit per sample; a shorter interval "
                      "leaves OSCG's jitter too few cycles to randomise." % (period, MIN_CYCLES))
    if period > ceiling:
        errors.append("the nominal edge period is %.0f core cycles, over %.0f%% of %s's %d-cycle "
                      "edge timeout; an oscillator %.0f%% slow reads DEAD."
                      % (period, TIMEOUT_MARGIN * 100, TRNG, timeout, 100 * (1 - period / timeout)))

    if not re.search(r"icesugar_pro_entropy\s+\w+\s*\(\s*\.raw\(\s*(\w+)\s*\)\s*\)", board):
        errors.append("%s does not instantiate icesugar_pro_entropy." % BOARD)
    else:
        wire = re.search(r"icesugar_pro_entropy\s+\w+\s*\(\s*\.raw\(\s*(\w+)\s*\)", board).group(1)
        if not re.search(r"\.entropy_raw\(\s*%s\s*\)" % re.escape(wire), board):
            errors.append("%s does not connect %r to littlesoc's entropy_raw; with a constant "
                          "there the health tests see no edges and seed reads DEAD." % (BOARD, wire))

    if not re.search(r"assign\s+raw\s*=\s*count\[\s*DIV_LOG2\s*-\s*1\s*\]", entropy):
        errors.append("%s does not drive raw from the divider's top bit." % ENTROPY)

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("entropy-geometry: OSCG DIV %d, 2^%d fabric divide, %.0f core cycles per edge at %d Hz "
          "(between %d and %.0f)." % (div, log2, period, core_hz, MIN_CYCLES, ceiling))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("root", nargs="?",
                        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    args = parser.parse_args()
    if not os.path.isdir(args.root):
        sys.exit("error: '%s' is not a directory, so there is nothing to grade." % args.root)
    return check(args.root)


if __name__ == "__main__":
    sys.exit(main())
