#!/usr/bin/env python3
"""Asserts that the iCESugar-Pro's core clock, its UART divisor and the constraint
handed to the placer are all the same number, derived from the pad and the PLL's
own dividers.

Usage: pll_clock_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. `rtl/uart.v` derives its 115200 8N1 divisor from `littlesoc`'s
`CLOCK_HZ`, so that parameter is not documentation: it is the baud rate. Between
the pad and that parameter there are now three more statements of the same
frequency, each in a different language and each silent when wrong.

  soc/icesugar_pro.lpf         FREQUENCY PORT -- what nextpnr believes the pad is
  soc/icesugar_pro_pll.v       CLKI_DIV / CLKFB_DIV -- what the silicon multiplies by
  soc/icesugar_pro_pll.v       FREQUENCY_PIN_CLKOP -- what nextpnr constrains at
  soc/board_icesugar_pro.v     CORE_HZ -- what the UART divides by

Get the last one wrong and the design still places, still configures, and prints
garbage at a baud rate nothing asked for; the board's only observable output is
that UART, so a wrong divisor looks exactly like a dead core. Get the third one
wrong and the placer grades the design against a period it does not run at,
which is silent in every log. Nothing else in the tree compares them, because
nextpnr reads two of the four and the compiler reads none.

WHAT IT CANNOT SEE. Whether the board's oscillator really is what the LPF says.
That is a board fact and `docs/pin-constraints.md` is where it is recorded; this
check only requires every other statement to agree with it.

Hermetic: three file reads and integer arithmetic. No toolchain.
"""

import argparse
import os
import re
import sys

LPF = "soc/icesugar_pro.lpf"
PLL = "soc/icesugar_pro_pll.v"
BOARD = "soc/board_icesugar_pro.v"

VCO_MIN_HZ = 400_000_000
VCO_MAX_HZ = 800_000_000

SOC_MODULE = "littlesoc"
PLL_MODULE = "icesugar_pro_pll"


def read(root, rel):
    path = os.path.join(root, rel)
    try:
        with open(path) as handle:
            return handle.read()
    except OSError as exc:
        sys.exit("error: cannot read %s: %s. This check grades the agreement "
                 "between three files and cannot grade a missing one." % (path, exc))


def only(matches, what, rel):
    if len(matches) != 1:
        sys.exit("error: %s states %d %s, expected exactly one. Two statements of "
                 "one frequency are two chances to disagree, which is the thing "
                 "this check exists to stop." % (rel, len(matches), what))
    return matches[0]


def hz(text):
    return int(text.replace("_", ""))


def lpf_pad(text):
    """The pad the LPF constrains, and at what frequency."""
    found = re.findall(r'FREQUENCY\s+PORT\s+"([A-Za-z_][A-Za-z0-9_]*)"\s+'
                       r'([0-9]+(?:\.[0-9]+)?)\s+MHZ\s*;', text)
    port, mhz = only(found, "FREQUENCY PORT constraint", LPF)
    scaled = float(mhz) * 1e6
    if scaled != int(scaled):
        sys.exit("error: %s constrains %s at %s MHz, which is not a whole number of "
                 "hertz. Every other statement of this frequency is an integer, so "
                 "no exact comparison against it is possible." % (LPF, port, mhz))
    return port, int(scaled)


def pll_facts(text):
    def param(name):
        found = re.findall(r"\.%s\s*\(\s*([0-9]+)\s*\)" % name, text)
        return int(only(found, "%s parameter" % name, PLL))

    def attribute(name):
        found = re.findall(r'\(\*\s*%s\s*=\s*"([0-9]+(?:\.[0-9]+)?)"\s*\*\)' % name, text)
        return only(found, "%s attribute" % name, PLL)

    feedback = re.findall(r'\.FEEDBK_PATH\s*\(\s*"([A-Z0-9_]+)"\s*\)', text)
    return {
        "clki_div": param("CLKI_DIV"),
        "clkfb_div": param("CLKFB_DIV"),
        "clkop_div": param("CLKOP_DIV"),
        "clki_hz": int(float(attribute("FREQUENCY_PIN_CLKI")) * 1e6),
        "clkop_hz": int(float(attribute("FREQUENCY_PIN_CLKOP")) * 1e6),
        "feedback": only(feedback, "FEEDBK_PATH parameter", PLL),
    }


def instance_connections(text, header):
    """The named port connections of one instance, given the text that opens it."""
    start = text.index(header) + len(header)
    depth = 1
    end = start
    while end < len(text) and depth:
        if text[end] == "(":
            depth += 1
        elif text[end] == ")":
            depth -= 1
        end += 1
    body = text[start:end]
    return dict(re.findall(r"\.([A-Za-z_][A-Za-z0-9_]*)\s*\(\s*([^),]*?)\s*\)", body))


def board_facts(text):
    facts = {}
    for name in ("PAD_HZ", "CORE_HZ"):
        found = re.findall(r"localparam\s+integer\s+%s\s*=\s*([0-9_]+)\s*;" % name, text)
        facts[name] = hz(only(found, "%s declaration" % name, BOARD))

    soc = re.findall(r"(%s\s*#\(\s*\.CLOCK_HZ\(\s*([A-Za-z0-9_]+)\s*\)\s*\)\s*"
                     r"[A-Za-z_][A-Za-z0-9_]*\s*\()" % SOC_MODULE, text)
    header, argument = only(soc, "parameterised %s instance" % SOC_MODULE, BOARD)
    facts["clock_hz_arg"] = argument
    facts["soc_ports"] = instance_connections(text, header)

    pll = re.findall(r"%s\s+[A-Za-z_][A-Za-z0-9_]*\s*\(" % PLL_MODULE, text)
    only(pll, "%s instance" % PLL_MODULE, BOARD)
    facts["pll_ports"] = instance_connections(text, pll[0])
    return facts


def check(root):
    lpf = read(root, LPF)
    pll = pll_facts(read(root, PLL))
    board_text = read(root, BOARD)
    board = board_facts(board_text)
    pad_port, pad_hz = lpf_pad(lpf)

    errors = []

    if pad_hz != board["PAD_HZ"]:
        errors.append("%s constrains the pad at %d Hz and %s declares PAD_HZ = %d. "
                      "The PLL multiplies whatever the pad really is, so these two "
                      "disagreeing means the core clock this check computes is not "
                      "the one the silicon makes."
                      % (LPF, pad_hz, BOARD, board["PAD_HZ"]))

    if pll["clki_hz"] != board["PAD_HZ"]:
        errors.append("%s says its input is %d Hz and %s declares PAD_HZ = %d. "
                      "nextpnr derives the output constraint from the input one, so "
                      "this is the placer being told about a different board."
                      % (PLL, pll["clki_hz"], BOARD, board["PAD_HZ"]))

    if pll["feedback"] != "CLKOP":
        errors.append("%s sets FEEDBK_PATH to %s, not CLKOP. The output frequency "
                      "below is computed as CLKI/CLKI_DIV*CLKFB_DIV, which is the "
                      "CLKOP feedback path's formula and no other's."
                      % (PLL, pll["feedback"]))

    numerator = board["PAD_HZ"] * pll["clkfb_div"]
    if numerator % pll["clki_div"]:
        errors.append("%d Hz * CLKFB_DIV %d is not divisible by CLKI_DIV %d, so this "
                      "PLL makes a fractional frequency and no integer CLOCK_HZ can "
                      "state it."
                      % (board["PAD_HZ"], pll["clkfb_div"], pll["clki_div"]))
    else:
        computed = numerator // pll["clki_div"]
        if computed != board["CORE_HZ"]:
            errors.append("the dividers in %s make %d Hz from a %d Hz pad, and %s "
                          "declares CORE_HZ = %d. CORE_HZ is rtl/uart.v's divisor: "
                          "the baud rate is wrong by exactly this ratio."
                          % (PLL, computed, board["PAD_HZ"], BOARD, board["CORE_HZ"]))
        vco = computed * pll["clkop_div"]
        if not VCO_MIN_HZ <= vco <= VCO_MAX_HZ:
            errors.append("CLKOP_DIV %d puts the VCO at %d Hz, outside the %d-%d Hz "
                          "range the part locks over. Nothing in the build flow "
                          "checks this and an unlocked PLL has no output at all."
                          % (pll["clkop_div"], vco, VCO_MIN_HZ, VCO_MAX_HZ))

    if pll["clkop_hz"] != board["CORE_HZ"]:
        errors.append("%s advertises FREQUENCY_PIN_CLKOP = %d Hz and %s declares "
                      "CORE_HZ = %d. That attribute is the constraint nextpnr grades "
                      "the placement against, so the design would be timed at a "
                      "period it does not run at."
                      % (PLL, pll["clkop_hz"], BOARD, board["CORE_HZ"]))

    if board["clock_hz_arg"] != "CORE_HZ":
        errors.append("%s passes %s to %s's CLOCK_HZ, not CORE_HZ. Everything above "
                      "grades CORE_HZ, and the UART divides by whatever is passed here."
                      % (BOARD, board["clock_hz_arg"], SOC_MODULE))

    pad_wire = board["pll_ports"].get("clk_pad")
    if pad_wire != pad_port:
        errors.append("the PLL in %s takes its input from %r, and %s constrains the "
                      "port %r. The PLL must be fed by the pad the frequency "
                      "constraint names." % (BOARD, pad_wire, LPF, pad_port))

    core_wire = board["pll_ports"].get("clk_core")
    soc_clk = board["soc_ports"].get("clk")
    if core_wire is None or soc_clk != core_wire:
        errors.append("%s clocks %s from %r while the PLL's output is %r. A core "
                      "clocked from the pad runs at the pad's frequency however many "
                      "of the statements above agree."
                      % (BOARD, SOC_MODULE, soc_clk, core_wire))

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1

    print("pll-clock: %s pad %d Hz -> %s core %d Hz, and rtl/uart.v divides by the "
          "same figure nextpnr placed against."
          % (pad_port, board["PAD_HZ"], SOC_MODULE, board["CORE_HZ"]))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("root", nargs="?",
                        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    args = parser.parse_args()
    if not os.path.isdir(args.root):
        sys.exit("error: '%s' is not a directory, so there is nothing to grade."
                 % args.root)
    return check(args.root)


if __name__ == "__main__":
    sys.exit(main())
