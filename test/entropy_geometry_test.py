#!/usr/bin/env python3
"""Asserts that each board's entropy source hands rtl/trng.v an edge period the measured
jitter can fill and the edge timeout cannot trip.

Usage: entropy_geometry_test.py [repo-root]     # defaults to this script's parent

The nominal edge period in core cycles is CORE_HZ * 2^DIV_LOG2 / OSC_HZ, where the
iCESugar-Pro's OSC_HZ is OSCG's nominal 310 MHz divided by its DIV and the UPduino's is
SB_LFOSC's nominal 10 kHz. The oscillator's edge-to-edge jitter is a fixed time, so how many
core cycles it spans, and so how much of rtl/trng.v's two sampled counter bits it randomises,
is set by this one product. Below a board's floor the measured min-entropy falls under the
0.5 bit per sample the adaptive proportion cutoff assumes; at 80% of the timeout a source 20%
slow trips the edge timeout and reads DEAD.

The floors are measured, per board. iCESugar-Pro: 0.4134 to 0.4994 at 3,154 to 3,171 cycles
by NIST ea_non_iid, 0.28 at 1,577. UPduino: 0.046 at 1,199.5 cycles, 0.437 at 4,797 and 0.7045 at
9,591, so the floor sits between the last failure and the first pass.

WHAT IT CANNOT SEE. Whether the silicon's oscillator runs at its nominal frequency; that is
the capture in soc/entropy_assess.py, and the numbers above come from it.
"""

import argparse
import os
import re
import sys

ICESUGAR_ENTROPY = "soc/icesugar_pro_entropy.v"
ICESUGAR_BOARD = "soc/board_icesugar_pro.v"
UPDUINO_BOARD = "soc/board_upduino.v"
NIST_MK = "mk/nist90b.mk"
TRNG = "rtl/trng.v"
TIMEOUT_MARGIN = 0.8

OSCG_NOMINAL_HZ = 310_000_000
LFOSC_NOMINAL_HZ = 10_000
ICESUGAR_MIN_CYCLES = 3000
UPDUINO_MIN_CYCLES = 9000


def read(root, rel):
    try:
        with open(os.path.join(root, rel)) as handle:
            return handle.read()
    except OSError as exc:
        sys.exit("error: cannot read %s: %s. This check grades the agreement between "
                 "the board, trng and measurement files and cannot grade a missing one." % (rel, exc))


def parameter(text, name, rel, kind="parameter"):
    found = re.findall(r"%s\s+integer\s+%s\s*=\s*([0-9]+)" % (kind, name), text)
    if len(found) != 1:
        sys.exit("error: %s states %d defaults for %s, expected exactly one." % (rel, len(found), name))
    return int(found[0])


def timeout_cycles(trng):
    width = re.findall(r"logic\s+\[(\d+):0\]\s+ticks\s*;", trng)
    if len(width) != 1:
        sys.exit("error: %s declares %d `ticks` counters, expected exactly one." % (TRNG, len(width)))
    if not re.search(r"if\s*\(\s*!edge_seen\s*&&\s*&ticks\s*\)\s*dead\s*<=", trng):
        sys.exit("error: %s does not set `dead` when `ticks` fills with no edge, so the timeout "
                 "is not one pass of `ticks`." % TRNG)
    return 2 ** (int(width[0]) + 1)


def core_hz(board, rel):
    found = re.findall(r"localparam\s+integer\s+CORE_HZ\s*=\s*([0-9_]+)\s*;", board)
    if len(found) != 1:
        sys.exit("error: %s states %d CORE_HZ, expected exactly one." % (rel, len(found)))
    return int(found[0].replace("_", ""))


def judge(name, period, floor, floor_why, timeout, errors):
    ceiling = timeout * TIMEOUT_MARGIN
    if period < floor:
        errors.append("%s: the nominal edge period is %.0f core cycles, under the %d at which the "
                      "measured min-entropy reached 0.5 bit per sample (%s); a shorter interval "
                      "leaves the oscillator's jitter too few cycles to randomise."
                      % (name, period, floor, floor_why))
    if period > ceiling:
        errors.append("%s: the nominal edge period is %.0f core cycles, over %.0f%% of %s's %d-cycle "
                      "edge timeout; an oscillator %.0f%% slow reads DEAD."
                      % (name, period, TIMEOUT_MARGIN * 100, TRNG, timeout, 100 * (1 - period / timeout)))
    return ceiling


def check(root):
    trng = read(root, TRNG)
    timeout = timeout_cycles(trng)
    errors = []
    summary = []

    entropy = read(root, ICESUGAR_ENTROPY)
    board = read(root, ICESUGAR_BOARD)
    div = parameter(entropy, "DIV", ICESUGAR_ENTROPY)
    log2 = parameter(entropy, "DIV_LOG2", ICESUGAR_ENTROPY)
    hz = core_hz(board, ICESUGAR_BOARD)
    if not 2 <= div <= 128:
        errors.append("%s sets OSCG DIV to %d, outside the primitive's 2..128." % (ICESUGAR_ENTROPY, div))
    if log2 < 1:
        errors.append("%s divides by 2^%d in fabric; the output must be a register bit." % (ICESUGAR_ENTROPY, log2))
    period = hz * div * 2 ** log2 / OSCG_NOMINAL_HZ
    ceiling = judge("iCESugar-Pro", period, ICESUGAR_MIN_CYCLES, "OSCG, 3,000-3,171 cycles",
                    timeout, errors)
    summary.append("iCESugar-Pro OSCG DIV %d, 2^%d divide, %.0f core cycles per edge at %d Hz (between %d and %.0f)"
                   % (div, log2, period, hz, ICESUGAR_MIN_CYCLES, ceiling))
    if not re.search(r"icesugar_pro_entropy\s+\w+\s*\(\s*\.raw\(\s*(\w+)\s*\)\s*\)", board):
        errors.append("%s does not instantiate icesugar_pro_entropy." % ICESUGAR_BOARD)
    else:
        wire = re.search(r"icesugar_pro_entropy\s+\w+\s*\(\s*\.raw\(\s*(\w+)\s*\)", board).group(1)
        if not re.search(r"\.entropy_raw\(\s*%s\s*\)" % re.escape(wire), board):
            errors.append("%s does not connect %r to littlesoc's entropy_raw; with a constant "
                          "there the health tests see no edges and seed reads DEAD." % (ICESUGAR_BOARD, wire))
    if not re.search(r"assign\s+raw\s*=\s*count\[\s*DIV_LOG2\s*-\s*1\s*\]", entropy):
        errors.append("%s does not drive raw from the divider's top bit." % ICESUGAR_ENTROPY)

    board = read(root, UPDUINO_BOARD)
    hz = core_hz(board, UPDUINO_BOARD)
    log2 = parameter(board, "ENTROPY_DIV_LOG2", UPDUINO_BOARD, "localparam")
    if log2 < 1:
        errors.append("%s divides SB_LFOSC by 2^%d in fabric; the output must be a register bit."
                      % (UPDUINO_BOARD, log2))
    period = hz * 2 ** log2 / LFOSC_NOMINAL_HZ
    ceiling = judge("UPduino", period, UPDUINO_MIN_CYCLES, "SB_LFOSC, 4,797 failed and 9,591 passed",
                    timeout, errors)
    summary.append("UPduino SB_LFOSC 2^%d divide, %.0f core cycles per edge at %d Hz (between %d and %.0f)"
                   % (log2, period, hz, UPDUINO_MIN_CYCLES, ceiling))
    capture = re.findall(r"^ENTROPY_UPDUINO_FAB\s*\?=\s*([0-9]+)\s*$", read(root, NIST_MK), re.M)
    if len(capture) != 1 or int(capture[0]) != log2:
        errors.append("%s defaults `make entropy-nist-upduino` to a fabric divide of %s, not the "
                      "board's 2^%d, so the default measurement would grade a source that does not ship."
                      % (NIST_MK, capture[0] if len(capture) == 1 else "none", log2))
    if not re.search(r"\.entropy_raw\(\s*entropy_raw\s*\)", board):
        errors.append("%s does not connect entropy_raw to littlesoc; with a constant there the "
                      "health tests see no edges and seed reads DEAD." % UPDUINO_BOARD)
    if not re.search(r"assign\s+entropy_raw\s*=\s*entropy_count\[\s*ENTROPY_DIV_LOG2\s*-\s*1\s*\]", board):
        errors.append("%s does not drive entropy_raw from the divider's top bit." % UPDUINO_BOARD)
    if not re.search(r"always_ff\s*@\(\s*posedge\s+lfosc_out\s*\)\s*entropy_count", board):
        errors.append("%s does not count SB_LFOSC's own edges, so the divide is not the "
                      "oscillator's." % UPDUINO_BOARD)

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("entropy-geometry: %s; %d-cycle edge timeout." % ("; ".join(summary), timeout))
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
