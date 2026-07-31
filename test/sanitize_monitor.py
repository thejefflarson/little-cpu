#!/usr/bin/env python3
"""Derive test/monitor.sim.v from the tracked, generated test/monitor.v.

test/monitor.v is riscv-formal's `monitor/generate.py` output at the pinned SHA
(formal/pin.mk). It is tracked but never hand-edited (CLAUDE.md invariant 7);
defects in it are repaired here, at build time, in a gitignored derivative that
BOTH sim legs read -- so the two legs cannot drift into checking different specs
(ADR-0019).

This used to be a two-rule `sed` chain in the root Makefile. It moved into a
script when the third rule arrived, because that one is a structural insert
across a 45-line span rather than a single-line substitution, and because a
`sed` rule that silently stops matching after a pin bump is indistinguishable
from one that did its job. Every rule below declares how many sites it must hit
and this script exits non-zero if the count is wrong.

A site count proves a rule FIRED. It says nothing about WHAT it matched, and for
rule 3 -- which selects a ~45-line span by position, from a first anchor to a
last one -- that gap is the whole risk: a generator change that moves another
check between the anchors keeps the count at 1 while quietly disabling that
check. So rule 3 also asserts on the contents of its span, and this script
re-checks the finished output. See rule 3's comment for the three layers.

The diff `test/monitor.v` -> `test/monitor.sim.v` must stay small enough to read
in full (`diff` it; it is currently eight lines). If it stops being readable at a
glance, that is the signal to fix the generator upstream instead of adding a
rule here.

Usage: sanitize_monitor.py <monitor.v>   # sanitized source on stdout
"""

import re
import sys

# ---------------------------------------------------------------------------
# Rule 1 -- strip `$time` from the `$display` error banners.
#
# yosys's AST_AUTOWIRE elaboration trips on `$time` used as a bare `$display`
# argument (iverilog handles it fine, yosys does not), so the cxxrtl leg cannot
# read the file as generated. The iverilog leg loses a timestamp in its error
# banner; that is a diagnostic nicety, not a check.
# ---------------------------------------------------------------------------
TIME_IN_DISPLAY = (
    re.compile(r' at time %0t --------", ([A-Za-z0-9_]+), \$time\)'),
    r' --------", \1)',
    4,
)

# ---------------------------------------------------------------------------
# Rule 2 -- make signed DIV/REM self-determined (ADR-0019).
#
# monitor_insn_div / monitor_insn_rem compute the signed result as one branch of
# a conditional whose other branches are unsigned. Per IEEE 1800 sign-context
# rules the conditional's type is unsigned and that propagates down into the
# division, which is then evaluated UNSIGNED -- silently, and wrongly, for
# negative operands only. Wrapping the arithmetic in `$signed()` makes it
# self-determined, so the enclosing conditional can no longer downgrade it.
# Anchored on the exact operand names and the trailing semicolon so it cannot
# match anywhere it was not meant to. Two sites: DIV's `/` and REM's `%`.
# ---------------------------------------------------------------------------
UNSIGNED_SIGNED_DIVREM = (
    re.compile(r'\$signed\((rvfi_rs1_rdata)\) ([/%]) \$signed\((rvfi_rs2_rdata)\);'),
    r'$signed($signed(\1) \2 $signed(\3));',
    2,
)

# ---------------------------------------------------------------------------
# Rule 3 -- gate the spec-value checks on `!spec_trap`.
#
# riscv-formal's own checker (checks/rvfi_insn_check.sv) puts rs1_addr,
# rs2_addr, rd_addr, rd_wdata, pc_wdata and every mem_* assertion inside
# `if (!spec_trap)`, leaving only `assert(spec_trap == trap)` outside. The
# monitor generator never emits that gate: it runs all of those comparisons
# unconditionally once spec_valid holds.
#
# On a trapping instruction the spec model still reports what the instruction
# WOULD have done -- for a misaligned `lw`, spec_rd_addr is the destination
# register, spec_rd_wdata is the loaded value, spec_pc_wdata is pc+4 and
# spec_mem_rmask is the byte mask. A correct core writes no register, strobes no
# memory and redirects to `mtvec` (ADR-0028). So the ungated monitor reports
# errors 104/105/106 and 110-113 on correct hardware, in both sim legs, for
# every trapping retire. That is a defect in the generated oracle, and per
# ADR-0019 it is repaired here rather than parked in test/EXPECTED_FAIL.
#
# The span wrapped runs from the rs1_addr comparison (error 102) through the
# mem_addr comparison (error 107) -- i.e. everything the generator emits inside
# `if (chN_spec_valid)` except the trap-flag comparison (error 101), which is
# exactly the partition upstream's checker uses. Two lines are inserted and
# nothing is reindented, so the diff stays readable.
#
# The span is selected by POSITION -- first anchor to last anchor -- so the site
# count alone only proves the rule fired, not what it swallowed. That is not
# enough. `assert(spec_trap == trap)` (error 101) is the one comparison
# upstream's checker deliberately keeps live under `spec_trap`; if a pin bump
# emitted it anywhere between the two anchors, the count would still read 1, the
# diff would still be eight lines, and "did the core trap when the spec says it
# should" would become vacuous on every trapping retire -- in BOTH sim legs,
# silently, exactly as M3 starts producing trapping retires. So the rule also
# asserts on the CONTENTS of what it is about to wrap, in three layers:
#
#   1. two literal markers that must not appear inside the span;
#   2. the exact list of handle_error codes enclosed, which catches a relocated
#      check however its comparison is spelled -- 101 is not on the list;
#   3. after all rules run, that the trap comparison still exists in the output
#      (see _check_trap_comparison_survives), which catches the generator
#      dropping it rather than moving it.
# ---------------------------------------------------------------------------
TRAP_GATE_SPAN = re.compile(
    r'(?P<indent>[ ]*)if \((?P<ch>ch\d+)_rvfi_rs1_addr != (?P=ch)_spec_rs1_addr\b.*?'
    r'(?P=ch)_handle_error\(\d+, "mismatch in mem_addr"\);\n[ ]*end\n',
    re.DOTALL,
)

# Layer 1. Both spellings of the trap comparison the generator emits today. A
# match means the generator relocated it inside the span and gating it would
# disable it.
TRAP_GATE_FORBIDDEN = (
    '"mismatch in trap"',
    '_rvfi_trap != ',
)

# Layer 2. DERIVED, not chosen: this is the exact multiset of handle_error codes
# the generator emits between the two anchors, read off test/monitor.v at the
# SHA in formal/pin.mk. Compared sorted, so a harmless reordering inside the
# span does not trip it but an added, removed or duplicated check does.
#
# After a pin bump, RE-DERIVE this from the new test/monitor.v -- read what the
# generator now emits inside `if (chN_spec_valid)` and confirm each code belongs
# under `!spec_trap` per riscv-formal's checks/rvfi_insn_check.sv. Do NOT edit it
# to make a failure go away: a code appearing here that upstream keeps outside
# the gate is precisely the silent-oracle failure this list exists to catch.
TRAP_GATE_ENCLOSED_CODES = sorted(
    [102, 103, 104, 105, 106, 108, 110, 120, 111, 121, 112, 122, 113, 123, 107]
)

# Layer 3. The generated monitor is `generate.py -c 1` (one retire channel, see
# the root Makefile's MONITOR_GEN), so exactly one channel emits the check.
TRAP_COMPARISON = re.compile(r'ch\d+_handle_error\(101, "mismatch in trap"\)')
TRAP_COMPARISON_SITES = 1


class SanitizerError(Exception):
    """A rule matched, but not the text it was written to match."""


def _wrap_in_trap_gate(match):
    indent = match.group('indent')
    channel = match.group('ch')
    span = match.group(0)

    for marker in TRAP_GATE_FORBIDDEN:
        if marker in span:
            raise SanitizerError(
                f'  rule "gate spec-value checks on !spec_trap": the span it is about\n'
                f'  to wrap contains {marker!r}, i.e. the generator now emits the\n'
                f'  trap-flag comparison BETWEEN the rs1_addr and mem_addr anchors.\n'
                f'  Gating it would silently disable the one check riscv-formal\n'
                f'  (checks/rvfi_insn_check.sv) deliberately keeps live under\n'
                f'  spec_trap. Re-anchor the span so the trap comparison stays\n'
                f'  outside the gate; do not widen the anchors to swallow it.'
            )

    codes = sorted(int(c) for c in re.findall(
        rf'{channel}_handle_error\((\d+),', span))
    if codes != TRAP_GATE_ENCLOSED_CODES:
        raise SanitizerError(
            f'  rule "gate spec-value checks on !spec_trap": the span encloses\n'
            f'  handle_error codes {codes}, expected\n'
            f'  {TRAP_GATE_ENCLOSED_CODES}. The generator has moved a check into or\n'
            f'  out of the gated region. Re-read each code against riscv-formal\'s\n'
            f'  checks/rvfi_insn_check.sv to decide whether it belongs under\n'
            f'  !spec_trap, then re-derive TRAP_GATE_ENCLOSED_CODES from the new\n'
            f'  test/monitor.v -- do not edit the list to silence this.'
        )

    return (
        f'{indent}if (!{channel}_spec_trap) begin\n'
        f'{span}'
        f'{indent}end\n'
    )


TRAP_GATE = (TRAP_GATE_SPAN, _wrap_in_trap_gate, 1)


RULES = [
    ('strip $time from $display banners', *TIME_IN_DISPLAY),
    ('make signed DIV/REM self-determined', *UNSIGNED_SIGNED_DIVREM),
    ('gate spec-value checks on !spec_trap', *TRAP_GATE),
]


def _check_trap_comparison_survives(text):
    """Layer 3: the trap comparison must still be in the sanitized output.

    Layers 1 and 2 catch the generator MOVING error 101 inside the gate. Neither
    notices it disappearing altogether -- the enclosed-code list would still
    match and the site count would still be 1, while the sanitized oracle no
    longer checks whether the core traps at all.
    """
    found = len(TRAP_COMPARISON.findall(text))
    if found != TRAP_COMPARISON_SITES:
        raise SanitizerError(
            f'  post-check "the trap comparison survives": found {found} site(s) of\n'
            f'  handle_error(101, "mismatch in trap") in the sanitized output,\n'
            f'  expected {TRAP_COMPARISON_SITES}. Without it neither sim leg checks\n'
            f'  whether the core traps when the spec model says it must.'
        )


def main(argv):
    if len(argv) != 2:
        sys.stderr.write(f'usage: {argv[0]} <monitor.v>\n')
        return 2

    with open(argv[1]) as handle:
        text = handle.read()

    failures = []
    try:
        for name, pattern, replacement, expected in RULES:
            text, count = pattern.subn(replacement, text)
            if count != expected:
                failures.append(f'  rule "{name}": matched {count} site(s), expected {expected}')
        if not failures:
            _check_trap_comparison_survives(text)
    except SanitizerError as error:
        failures.append(str(error))

    if failures:
        sys.stderr.write(
            f'{argv[0]}: the generated monitor no longer looks the way these rules\n'
            f'expect. A riscv-formal pin bump (formal/pin.mk) may have changed the\n'
            f'generator output -- re-read each rule against {argv[1]} and either fix\n'
            f'or delete it. Do NOT ship a silently-unapplied sanitizer: both sim legs\n'
            f'read this output as their oracle (ADR-0019).\n'
            + '\n'.join(failures) + '\n'
        )
        return 1

    sys.stdout.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
