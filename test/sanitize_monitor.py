#!/usr/bin/env python3
"""Derive test/monitor.sim.v from the tracked, generated test/monitor.v.

test/monitor.v is riscv-formal's `monitor/generate.py` output at the SHA pinned
in formal/pin.mk. Do not edit it by hand. `make monitor-check` regenerates it
into a temp file and diffs, so a hand edit shows up there as a failure, and the
`test/monitor.v` rule overwrites it. Fix it here instead: this script rewrites it
at build time into a gitignored copy that both sim legs read, so the two legs
cannot end up checking different rules.

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

# Rule 1. yosys's AST_AUTOWIRE elaboration trips on `$time` as a bare `$display`
# argument, so the cxxrtl leg cannot read the file as generated. The iverilog leg
# loses a timestamp from an error banner, which is not a check.
TIME_IN_DISPLAY = (
    re.compile(r' at time %0t --------", ([A-Za-z0-9_]+), \$time\)'),
    r' --------", \1)',
    4,
)

# Rule 2. monitor_insn_div / monitor_insn_rem compute the signed result as one
# branch of a conditional whose other branches are unsigned, and per IEEE 1800
# sign-context rules that makes the division evaluate UNSIGNED -- silently, and
# for negative operands only. `$signed()` makes it self-determined so the
# enclosing conditional cannot downgrade it.
UNSIGNED_SIGNED_DIVREM = (
    re.compile(r'\$signed\((rvfi_rs1_rdata)\) ([/%]) \$signed\((rvfi_rs2_rdata)\);'),
    r'$signed($signed(\1) \2 $signed(\3));',
    2,
)

# Rule 3. When an instruction traps, the spec model still reports what it would
# have done had it not. A correct core writes no register, touches no memory and
# jumps to `mtvec`, so those comparisons all disagree. riscv-formal's own checker
# skips them when the spec model says trap; the monitor generator does not, so
# without this it reports errors 104, 105, 106 and 110-113 on hardware that is
# working.
#
# The span below is found by position, from the first anchor to the last. That
# means counting the matches tells us the rule fired but not what it wrapped.
# Error 101 is the one comparison that must keep running when an instruction
# traps -- it is the check that the core trapped at all. If a new version of the
# generator moved it between the two anchors, the count would still be 1 and that
# check would quietly stop working in both sim legs. So there are three more
# checks below: literals that must not appear inside the span, the exact list of
# error codes it is allowed to contain, and one afterwards that 101 is still in
# the output.
TRAP_GATE_SPAN = re.compile(
    r'(?P<indent>[ ]*)if \((?P<ch>ch\d+)_rvfi_rs1_addr != (?P=ch)_spec_rs1_addr\b.*?'
    r'(?P=ch)_handle_error\(\d+, "mismatch in mem_addr"\);\n[ ]*end\n',
    re.DOTALL,
)

# Both ways the generator currently writes the trap comparison. Either one
# inside the span means it has moved and must not be wrapped.
TRAP_GATE_FORBIDDEN = (
    '"mismatch in trap"',
    '_rvfi_trap != ',
)

# Read off test/monitor.v at the current pin, not chosen. After a pin bump, read
# it off again and check each code against riscv-formal's own checker to see that
# it really does belong inside the gate. Do not edit this list to make a failure
# go away — a code turning up here that upstream keeps outside the gate is
# exactly what this list is here to catch.
TRAP_GATE_ENCLOSED_CODES = sorted(
    [102, 103, 104, 105, 106, 108, 110, 120, 111, 121, 112, 122, 113, 123, 107]
)

# The monitor is generated with one retire channel, so there is exactly one.
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
    """Error 101 must still be in the output.

    The other two checks catch the generator moving it inside the gate. Neither
    notices it going away entirely: the code list would still match and the count
    would still be 1, while nothing checked whether the core trapped at all.
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
