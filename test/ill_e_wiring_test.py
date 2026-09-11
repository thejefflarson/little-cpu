#!/usr/bin/env python3
"""Asserts that RISCV_FORMAL_E never goes live in nano/formal/checks.cfg without a
real-core ill_e check behind it, backed by a forced-red probe proving that check
catches a wrong RV32E rule.

Usage: ill_e_wiring_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. The first nano/formal/ill_e.sv checked the RV32E register-naming
restriction against a hand-written reference rather than the real core, and its
three assertions turned out to restate the assign lines directly above them: a
mutated wrong rule (testing bit 3 of a register field instead of bit 4, 23 sites)
still passed both its .sby files. The only thing that ever went red was the probe
that mutated the check itself, which proved the PROBE could fail, never that the
PROPERTY could. Once RISCV_FORMAL_E is wired into the generated per-instruction
checks, x16-x31 fall outside what those checks explore at all, so a real-core
ill_e.sv is the only thing that can say those encodings trap -- and it does not
count until a probe has shown it can tell a right rule from a wrong one. That
probe's label must contain the phrase "a wrong RV32E rule", named once here so the
engineer who builds the real ill_e knows exactly what to add. nano.v is still the
unreshaped RV32IMC donor today; this tripwire is what stands between the reshape
turning RISCV_FORMAL_E on and nothing checking the restriction it relies on.

Hermetic: file reads and string search. No toolchain.
"""

import argparse
import os
import sys

CHECKS_CFG = "nano/formal/checks.cfg"
ILL_E_SV = "nano/formal/ill_e.sv"
PROBES_EXPECTED = "test/PROBES_EXPECTED"
REAL_CORE_MARKER = "riscv wrapper ("
WRONG_RULE_PHRASE = "a wrong RV32E rule"


def check(root):
    checks_cfg = os.path.join(root, CHECKS_CFG)
    if not os.path.isfile(checks_cfg):
        sys.stderr.write("error: %s is missing, so there is nothing to grade.\n" % checks_cfg)
        return 1

    with open(checks_cfg) as f:
        live = "RISCV_FORMAL_E" in f.read()

    if not live:
        print(
            "ill-e-wiring: RISCV_FORMAL_E is not yet wired into %s; a future "
            "ill_e.sv is a no-op." % CHECKS_CFG
        )
        return 0

    ill_e_sv = os.path.join(root, ILL_E_SV)
    exists = os.path.isfile(ill_e_sv)
    wired_to_core = False
    if exists:
        with open(ill_e_sv) as f:
            wired_to_core = REAL_CORE_MARKER in f.read()
    if not wired_to_core:
        problem = (
            "does not exist" if not exists
            else "does not instantiate the real core (%r)" % REAL_CORE_MARKER
        )
        sys.stderr.write(
            "error: %s defines RISCV_FORMAL_E for the generated checks, but %s "
            "%s. The RV32E register-naming restriction those checks rely on is "
            "checked by nothing: ill_e.sv must instantiate the real core (%r), "
            "flag every register field of x16 or above -- several at once "
            "included -- as illegal, and be backed by %s in %s.\n"
            % (CHECKS_CFG, ILL_E_SV, problem, REAL_CORE_MARKER, WRONG_RULE_PHRASE,
               PROBES_EXPECTED)
        )
        return 1

    probes_expected = os.path.join(root, PROBES_EXPECTED)
    if not os.path.isfile(probes_expected):
        sys.stderr.write(
            "error: %s is missing, so there is nothing to grade.\n" % probes_expected
        )
        return 1
    with open(probes_expected) as f:
        has_wrong_rule_probe = any(WRONG_RULE_PHRASE in line for line in f)
    if not has_wrong_rule_probe:
        sys.stderr.write(
            "error: %s instantiates the real core, but %s names no forced-red "
            "probe containing %r. RISCV_FORMAL_E's assumption is checked by "
            "nothing until make probe-gates proves ill_e refuses %s -- add that "
            "probe and its label before turning this on.\n"
            % (ILL_E_SV, PROBES_EXPECTED, WRONG_RULE_PHRASE, WRONG_RULE_PHRASE)
        )
        return 1

    print(
        "ill-e-wiring: RISCV_FORMAL_E is live, %s instantiates the real core, "
        "and %s names a forced-red probe for %s."
        % (ILL_E_SV, PROBES_EXPECTED, WRONG_RULE_PHRASE)
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
