#!/usr/bin/env python3
"""Asserts that RISCV_FORMAL_E is never live in nano/formal/checks.cfg while
nano/formal/ill_e.sv still checks a reference model instead of the real core.

Usage: ill_e_wiring_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. nano/formal/ill_e.sv checks the RV32E register-naming
restriction against `ill_e_top`, a hand-written reference, because `nano.v`
is still the unreshaped RV32IMC donor with no such restriction to check
(docs/adr/0174). That reference proves the PROPERTY is reachable and
self-consistent, not that any real core has it -- and the day
`RISCV_FORMAL_E` is wired into the generated per-instruction checks, the
restriction those checks rely on is checked by nothing unless `ill_e.sv` has
also been swapped to the real core by then. A silent gap here is exactly two
green CI steps and no design under test.

Hermetic: two file reads and a string search. No toolchain.
"""

import argparse
import os
import sys

CHECKS_CFG = "nano/formal/checks.cfg"
ILL_E_SV = "nano/formal/ill_e.sv"
REAL_CORE_MARKER = "riscv wrapper ("


def check(root):
    checks_cfg = os.path.join(root, CHECKS_CFG)
    ill_e_sv = os.path.join(root, ILL_E_SV)
    for path in (checks_cfg, ill_e_sv):
        if not os.path.isfile(path):
            sys.stderr.write("error: %s is missing, so there is nothing to grade.\n" % path)
            return 1

    with open(checks_cfg) as f:
        live = "RISCV_FORMAL_E" in f.read()
    with open(ill_e_sv) as f:
        wired_to_core = REAL_CORE_MARKER in f.read()

    if live and not wired_to_core:
        sys.stderr.write(
            "error: %s defines RISCV_FORMAL_E for the generated checks, but %s "
            "does not instantiate the real core (%r). The RV32E register-naming "
            "restriction the assumption relies on is checked by nothing until "
            "ill_e.sv wires the real core in, the way complete.sv and "
            "dmemcheck.sv already do.\n" % (CHECKS_CFG, ILL_E_SV, REAL_CORE_MARKER)
        )
        return 1

    print(
        "ill-e-wiring: RISCV_FORMAL_E is %s, and %s %s the real core."
        % ("live" if live else "not yet wired into " + CHECKS_CFG,
           ILL_E_SV,
           "instantiates" if wired_to_core else "checks a reference model, not")
    )
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
