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
# ---------------------------------------------------------------------------
TRAP_GATE_SPAN = re.compile(
    r'(?P<indent>[ ]*)if \((?P<ch>ch\d+)_rvfi_rs1_addr != (?P=ch)_spec_rs1_addr\b.*?'
    r'(?P=ch)_handle_error\(\d+, "mismatch in mem_addr"\);\n[ ]*end\n',
    re.DOTALL,
)


def _wrap_in_trap_gate(match):
    indent = match.group('indent')
    channel = match.group('ch')
    return (
        f'{indent}if (!{channel}_spec_trap) begin\n'
        f'{match.group(0)}'
        f'{indent}end\n'
    )


TRAP_GATE = (TRAP_GATE_SPAN, _wrap_in_trap_gate, 1)


RULES = [
    ('strip $time from $display banners', *TIME_IN_DISPLAY),
    ('make signed DIV/REM self-determined', *UNSIGNED_SIGNED_DIVREM),
    ('gate spec-value checks on !spec_trap', *TRAP_GATE),
]


def main(argv):
    if len(argv) != 2:
        sys.stderr.write(f'usage: {argv[0]} <monitor.v>\n')
        return 2

    with open(argv[1]) as handle:
        text = handle.read()

    failures = []
    for name, pattern, replacement, expected in RULES:
        text, count = pattern.subn(replacement, text)
        if count != expected:
            failures.append(f'  rule "{name}": matched {count} site(s), expected {expected}')

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
