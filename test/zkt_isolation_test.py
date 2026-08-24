#!/usr/bin/env python3
"""Zkt claims that a listed set of instructions -- RV32I arithmetic, logical
and shift, MUL/MULH/MULHU/MULHSU, and their compressed forms -- executes in
time independent of their operand VALUES. DIV/REM, loads, stores, branches
and jumps are excluded from the list on purpose and are not checked here.

That is a 2-safety property: run the same instruction with the same register
NUMBERS and two different sets of register VALUES, and the cycle count must
agree. A single-trace BMC check cannot express a 2-safety property, so nothing
riscv-formal generates reaches it and no depth would mean anything if it did.

The sound over-approximation this repo already has a model for is
formal/check-nonperturbation.py's cone check -- "structural, NOT sequential
equivalence." This is the same shape, read off the SOURCE rather than a mapped
netlist: rtl/decoder.v's `stall` is the exact OR of eight named reasons, seven
of them continuous assigns built from register NUMBERS, instruction bits and
control state, and the eighth -- `region_stall` -- built from a register
DATA output (`reg_rs1`) but gated on `ls_access`, which is true only for the
twelve load and store encodings. Zkt's listed instructions are never a load
or a store, so `ls_access` is false for every one of them and `region_stall`
cannot assert regardless of what `reg_rs1` holds. This script decides that
property by building a dependency graph over rtl/decoder.v's own continuous
assigns and asking whether `reg_rs1` or `reg_rs2` reaches any of the seven
value-blind reasons -- and, separately, whether `region_stall` is still gated
the way the argument above requires.

WHAT THIS PROVES, AND WHAT IT DOES NOT. It proves that no continuous assign in
rtl/decoder.v carries a register DATA output into hazard, operand_stall,
atomic_stall, serialize, ls_access, stall, stall_own or stall_other, and that
region_stall's own assign still conjoins ls_access. It does not simulate
anything and it does not reach into rtl/executor.v's multi-cycle divider --
that half of the argument (MUL resolves in decode's `init` state with no
counter; DIV/REM's two early exits are the one place this design is NOT
constant-time, and DIV/REM are excluded from the list for exactly that
reason) is read by eye against rtl/executor.v and recorded in an ADR, not
graded by this script. A signal this script cannot find as a continuous
assign -- because a later edit moved it into an always_ff/always_comb block --
is a hard error, not a signal read as clean: a stall reason that stopped being
a continuous assign would silently escape this graph, and the check would
rather stop than pass over a shape it no longer understands.

Usage: zkt_isolation_test.py [decoder.v]     # defaults to rtl/decoder.v
"""

import os
import re
import sys

# The register-file DATA outputs. Everything else rtl/decoder.v's stall logic
# reads -- rs1, rs2, out.rd, executor_out.rd -- is a register NUMBER or a
# control bit, never a value, and this script's whole job is to prove nothing
# below depends on these two.
SEEDS = {'reg_rs1', 'reg_rs2'}

# The seven reasons Zkt's listed instructions must be blind to, plus the
# aggregates built from them. `stall` and `stall_other` both name
# `region_stall` directly in their own assign, so read at FULL taint they are
# expected to depend on reg_rs1 for a load or a store -- that is the property,
# not a violation of it. What must never happen is a path into either of them
# that does not go THROUGH region_stall, which is what checking them at
# RESTRICTED taint (region_stall blocked from propagating) below decides.
# `divider_stall`, `fetch_stall` and `bus_wait` are module inputs --
# rtl/decoder.v cannot `assign` a module input, so they cannot appear as an
# assign's left-hand side and are excluded from this set by construction
# rather than by name.
MUST_NOT_DEPEND = [
    'ls_access', 'serialize', 'hazard_rs1', 'hazard_rs2', 'hazard',
    'operand_stall', 'atomic_stall', 'stall_own', 'stall_other', 'stall',
]

# The one reason allowed to read a register DATA output, and the term its own
# assign must conjoin so it can only ever assert for a load or a store.
GATED_SIGNAL = 'region_stall'
GATE_TERM = 'ls_access'

# A positive control: signals the real RTL is known to carry reg_rs1 through
# on the way to region_stall. If none of these end up tainted, the graph
# construction below found no edges at all and every PASS above would be
# vacuous -- the same anti-vacuity discipline formal/check-nonperturbation.py
# applies with its one-mux self-test.
EXPECT_TAINTED = ['ls_block', 'ls_text_deep', 'ls_ram_deep', 'ls_settled',
                   GATED_SIGNAL]

ASSIGN_RE = re.compile(r'\bassign\s+(.*?);', re.S)
LHS_RE = re.compile(r'^\s*([A-Za-z_][A-Za-z0-9_$.]*)\s*=(?!=)\s*(.*)$', re.S)
LITERAL_RE = re.compile(r"\d*'[sS]?[bBoOdDhH][0-9a-fA-Fxz_]+|'[01xzXZ]")
IDENT_RE = re.compile(
    r'\b[A-Za-z_][A-Za-z0-9_$]*(?:\.[A-Za-z_][A-Za-z0-9_$]*)*\b')


def strip_comments(text):
    text = re.sub(r'//[^\n]*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    return text


def extract_idents(expr):
    """The signal names an expression reads, with sized/unsized literals
    removed first so a hex literal like 32'hffffffff cannot be mistaken for
    an identifier named hffffffff."""
    cleaned = LITERAL_RE.sub(' ', expr)
    return set(IDENT_RE.findall(cleaned))


def top_level_and_terms(expr):
    """Split on `&&` at paren/bracket/brace depth zero, so a `&&` folded
    inside a sub-expression's parentheses is not mistaken for a top-level
    conjunct."""
    terms = []
    depth = 0
    start = 0
    i = 0
    n = len(expr)
    while i < n:
        c = expr[i]
        if c in '([{':
            depth += 1
        elif c in ')]}':
            depth -= 1
        elif depth == 0 and expr[i:i + 2] == '&&':
            terms.append(expr[start:i].strip())
            i += 2
            start = i
            continue
        i += 1
    terms.append(expr[start:].strip())
    return terms


def strip_parens(term):
    term = term.strip()
    while term.startswith('(') and term.endswith(')'):
        depth = 0
        balanced_to_end = True
        for i, c in enumerate(term):
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0 and i != len(term) - 1:
                    balanced_to_end = False
                    break
        if not balanced_to_end:
            break
        term = term[1:-1].strip()
    return term


def parse_assigns(text):
    """{signal: rhs text}, or a list of errors if a signal is assigned more
    than once. Module-scope `assign` is the only continuous-assignment form
    Verilog allows, so every match here is a real continuous assign
    regardless of what always/generate block surrounds it in the file."""
    assigns = {}
    errors = []
    for m in ASSIGN_RE.finditer(text):
        content = m.group(1)
        lm = LHS_RE.match(content)
        if not lm:
            errors.append(
                f'cannot read the left-hand side of `assign {content.strip()};`')
            continue
        lhs, rhs = lm.group(1), lm.group(2)
        if lhs in assigns:
            errors.append(f'`{lhs}` is assigned more than once')
            continue
        assigns[lhs] = rhs
    return assigns, errors


def taint_closure(assigns, blocked=frozenset()):
    """Every signal whose continuous assign reads, directly or transitively,
    one of SEEDS. A signal never assigned here -- a module input, a register
    written in an always_ff block, a localparam -- is a leaf: it cannot
    propagate taint because it has no entry in `assigns` for the fixpoint
    below to visit, which is the deliberately conservative reading of every
    name this script does not itself define.

    A name in `blocked` can still become tainted itself, but is never used as
    a taint SOURCE for anything downstream of it -- which is what lets
    `region_stall` legitimately depend on reg_rs1 while everything that reads
    `region_stall` (rather than reg_rs1 itself) is judged as if that read
    were invisible. Any OTHER path from reg_rs1/reg_rs2 into the same signal
    still counts; blocking only removes the one path this design intends."""
    deps = {lhs: extract_idents(rhs) for lhs, rhs in assigns.items()}
    tainted = set(SEEDS)
    changed = True
    while changed:
        changed = False
        sources = tainted - blocked
        for lhs, ids in deps.items():
            if lhs not in tainted and ids & sources:
                tainted.add(lhs)
                changed = True
    return tainted


def main():
    if len(sys.argv) not in (1, 2):
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2

    here = os.path.dirname(os.path.abspath(__file__))
    path = sys.argv[1] if len(sys.argv) == 2 else \
        os.path.join(here, os.pardir, 'rtl', 'decoder.v')

    try:
        with open(path) as f:
            text = f.read()
    except OSError as e:
        print(f'error: cannot read {path}: {e}', file=sys.stderr)
        return 2

    assigns, errors = parse_assigns(strip_comments(text))
    if errors:
        print(f'error: {path} could not be parsed as a set of continuous '
              f'assigns:', file=sys.stderr)
        for e in errors:
            print(f'  {e}', file=sys.stderr)
        return 2

    needed = set(MUST_NOT_DEPEND) | {GATED_SIGNAL} | set(EXPECT_TAINTED)
    missing = sorted(n for n in needed if n not in assigns)
    if missing:
        print(f'error: {path} has no continuous assign defining: '
              f'{", ".join(missing)}. Either the file moved these into a '
              f'procedural block, in which case this script no longer knows '
              f'how to trace them, or a name was renamed and this script '
              f'was not taught the new one.', file=sys.stderr)
        return 2

    full = taint_closure(assigns)
    restricted = taint_closure(assigns, blocked={GATED_SIGNAL})

    failures = []

    still_clean = [n for n in EXPECT_TAINTED if n not in full]
    if still_clean:
        failures.append(
            f'the taint graph found no path from reg_rs1/reg_rs2 to '
            f'{", ".join(still_clean)}, which the real RTL is known to '
            f'carry a register value through on the way to region_stall. '
            f'That means this run found no edges at all, and every PASS '
            f'above is a check of nothing.')

    for name in MUST_NOT_DEPEND:
        if name in restricted:
            failures.append(
                f'`{name}` depends on a register-file DATA output through a '
                f'path other than {GATED_SIGNAL}:\n'
                f'    assign {name} = {assigns[name].strip()};\n'
                f'  A Zkt-listed instruction (add, xor, sll, mul, ...) can '
                f'assert this, so its cycle count would depend on rs1 or '
                f'rs2\'s VALUE, not just which registers it names.')

    gate_terms = {strip_parens(t) for t in
                  top_level_and_terms(assigns[GATED_SIGNAL])}
    if GATE_TERM not in gate_terms:
        failures.append(
            f'`{GATED_SIGNAL}` no longer conjoins `{GATE_TERM}` at the top '
            f'level:\n'
            f'    assign {GATED_SIGNAL} = {assigns[GATED_SIGNAL].strip()};\n'
            f'  {GATED_SIGNAL} is the one stall reason allowed to read '
            f'reg_rs1, and only because {GATE_TERM} is false for every '
            f'Zkt-listed instruction. Without that conjunct it could assert '
            f'for any instruction, and the whole claim depends on this line.')

    if failures:
        print('ZKT STALL ISOLATION: FAIL', file=sys.stderr)
        for f in failures:
            print('  ' + f.replace('\n', '\n  '), file=sys.stderr)
        return 1

    print(f'{path}: reg_rs1/reg_rs2 reach only {GATED_SIGNAL} among the '
          f'stall reasons, and it stays gated on {GATE_TERM}.')
    print('ZKT STALL ISOLATION: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
