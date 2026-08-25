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
property with three checks against rtl/decoder.v's own continuous assigns:
whether reg_rs1, reg_rs2 or any other register/CSR-file DATA output reaches
any of the seven value-blind reasons, either directly or by way of a signal
this script cannot trace at all; whether `region_stall`'s own assign still
conjoins `ls_access` and adds no term beside it that could bypass that gate;
and whether every decoder input wider than a register NUMBER has actually
been looked at and classified, so a new one cannot go unconsidered.

WHAT THIS PROVES, AND WHAT IT DOES NOT. It proves that no continuous assign in
rtl/decoder.v carries a register or CSR-file DATA output into hazard,
operand_stall, atomic_stall, serialize, ls_access, stall, stall_own or
stall_other; that region_stall's own assign both conjoins ls_access and adds
no top-level `||` beside it; and that SEEDS and NON_VALUE_INPUTS between them
account for every decoder input (and struct field) wider than 5 bits, so a
new data-carrying port cannot be added without a human classifying it one way
or the other. It does not simulate anything and it does not reach into
rtl/executor.v's multi-cycle divider -- that half of the argument (MUL
resolves in decode's `init` state with no counter; DIV/REM's two early exits
are the one place this design is NOT constant-time, and DIV/REM are excluded
from the list for exactly that reason) is read by eye against rtl/executor.v
and recorded in an ADR, not graded by this script.

A signal read here that this script cannot trace to a continuous assign --
because it is a module input, an always_ff register, a localparam, or a
later edit moved it into an always_comb block -- is read as a hard error, not
as clean: every such leaf reachable from a value-blind reason or from
region_stall must be named in KNOWN_CLEAN_LEAVES (if it can be shown to carry
no register-file value) or in SEEDS (if it is one of the values this script
is checking for), or the run stops rather than passing over a shape it does
not understand.

Usage: zkt_isolation_test.py [decoder.v]     # defaults to rtl/decoder.v
"""

import os
import re
import sys

# The register-file and CSR-file DATA outputs decoder.v is allowed to read
# only on `region_stall`'s side of the gate. Everything else the seven
# value-blind reasons read -- rs1, rs2, out.rd, executor_out.rd -- is a
# register NUMBER or a control bit, never a value. This script's whole job is
# to prove nothing below depends on these.
SEEDS = {'reg_rs1', 'reg_rs2', 'executor_out.rd_data'}

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

# Every OTHER decoder input (or struct field of one) wider than 5 bits --
# wide enough to be a register-file DATA output rather than a register NUMBER
# (rd/rs1/rs2 are all `[4:0]`) -- with the reason it is not one of the values
# SEEDS tracks:
#   in.pc, in.instr, in.next_instr -- the fetch address and the instruction
#     words decode reads register NUMBERS out of, never an operand value.
#   csr_rdata -- the CSR file's read value. CSR instructions are not on
#     Zkt's list, and this script's port-coverage check (below) is what would
#     catch it reaching a value-blind reason if that ever changed.
#   mtvec, mepc -- trap CSRs, read only on the trap and mret arms of
#     `next_pc`, neither of which is on Zkt's list either.
# `classify_wide_inputs` below reads rtl/decoder.v's own port list rather
# than trusting this tuple to be complete on its own: any wide input it finds
# that is in neither this tuple nor SEEDS stops the run.
NON_VALUE_INPUTS = ('in.pc', 'in.instr', 'in.next_instr', 'csr_rdata',
                     'mtvec', 'mepc')

# Leaves the fan-in walk below is expected to reach from the value-blind
# reasons and region_stall, and knows to be clean: module inputs and
# always_ff registers that hold register NUMBERS or control state, never a
# register-file DATA output, plus the localparams the region test's block
# arithmetic uses. `rs1`/`rs2` are a submodule instance's output ports
# (`regsel current_regs (.rs1(rs1), ...)`), so they are never a continuous
# assign's left-hand side either, the same shape as an always_ff register.
#
# Anything reachable that is NOT here and not in SEEDS stops the run: a
# refactor that moves one of these seven reasons' own dependencies into an
# always_comb block, or introduces a new one, is exactly the shape that let
# `out.rs1` and `branch_taken` -- both procedural, both carrying reg_rs1 or
# cmp_eq/cmp_lt through untraced -- pass this check silently before this fix.
KNOWN_CLEAN_LEAVES = {
    'accessor_out_valid', 'bus_wait', 'divider_stall', 'fetch_stall',
    'executor_out.rd', 'executor_out.valid',
    'in.instr',
    'out.is_amo', 'out.rd', 'out.valid',
    'prev_rs1', 'prev_rs2', 'read_taken', 'ls_answer_valid',
    'rs1', 'rs2',
    'LS_BLOCK_BITS', 'LS_RAM_BLOCK', 'LS_RAM_BMASK', 'LS_TEXT_BLOCK',
    'LS_TEXT_BMASK',
}

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


def top_level_split(expr, op):
    """Split on `op` (`&&` or `||`) at paren/bracket/brace depth zero, so an
    operator folded inside a sub-expression's parentheses is not mistaken
    for a top-level one."""
    terms = []
    depth = 0
    start = 0
    i = 0
    n = len(expr)
    oplen = len(op)
    while i < n:
        c = expr[i]
        if c in '([{':
            depth += 1
        elif c in ')]}':
            depth -= 1
        elif depth == 0 and expr[i:i + oplen] == op:
            terms.append(expr[start:i].strip())
            i += oplen
            start = i
            continue
        i += 1
    terms.append(expr[start:].strip())
    return terms


def top_level_and_terms(expr):
    return top_level_split(expr, '&&')


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


def build_deps(assigns):
    """{signal: the identifiers its own continuous assign reads}, the
    dependency graph `taint_closure` and `fanin_leaves` both walk -- built
    once so a signal's RHS is parsed for identifiers a single time rather
    than once per walk."""
    return {lhs: extract_idents(rhs) for lhs, rhs in assigns.items()}


def taint_closure(deps, blocked=frozenset()):
    """Every signal whose continuous assign reads, directly or transitively,
    one of SEEDS. A signal never assigned here -- a module input, a register
    written in an always_ff block, a localparam -- is a leaf: it cannot
    propagate taint because it has no entry in `deps` for the fixpoint below
    to visit, which is the deliberately conservative reading of every name
    this script does not itself define. `fanin_leaves` below is what stops a
    leaf reached this way from being silently read as clean.

    A name in `blocked` can still become tainted itself, but is never used as
    a taint SOURCE for anything downstream of it -- which is what lets
    `region_stall` legitimately depend on reg_rs1 while everything that reads
    `region_stall` (rather than reg_rs1 itself) is judged as if that read
    were invisible. Any OTHER path from reg_rs1/reg_rs2 into the same signal
    still counts; blocking only removes the one path this design intends."""
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


def fanin_leaves(deps, targets):
    """Every identifier reachable from `targets` by following continuous
    assigns' own reads backward -- an ordinary dependency walk, not limited
    to paths from SEEDS -- that is NOT itself a continuous assign's
    left-hand side. `taint_closure` above only ever notices a leaf that
    happens to be a SEED; a leaf that carries a value through some other
    shape (an always_ff register, a submodule's output port, or a signal
    moved into an always_comb block) is invisible to it, because forward
    propagation from SEEDS never reaches a leaf that has no assign of its
    own to test. This walks the graph the other way instead, so it finds
    every leaf the value-blind reasons and region_stall actually touch,
    whether or not it happens to be tainted."""
    seen = set()
    leaves = set()
    frontier = list(targets)
    while frontier:
        name = frontier.pop()
        if name in seen:
            continue
        seen.add(name)
        ids = deps.get(name)
        if ids is None:
            leaves.add(name)
            continue
        for i in ids:
            if i not in seen:
                frontier.append(i)
    return leaves


def matching_paren(text, start):
    """Offset of the `)` closing the `(` at `start`, or None."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return i
    return None


DIRECTIVE_RE = re.compile(r"^\s*`(ifdef|ifndef|elsif|else|endif)\b\s*(\w+)?",
                           re.M)


def strip_formal_ifdefs(text):
    """Blank out everything gated on `` `ifdef RISCV_FORMAL `` (nested
    macros included, such as `` `ifdef RISCV_FORMAL_CSR_MCAUSE ``), keeping
    every other line -- including an unrelated header guard like
    `` `ifndef STRUCTS_V `` -- untouched. SEEDS and NON_VALUE_INPUTS classify
    the shape every build compiles; the RVFI report ports and struct fields
    this strips exist only under RISCV_FORMAL, are never read by any
    stall-reason assign, and would otherwise have to be classified one by
    one for no reason a non-formal build cares about."""
    out = []
    stack = []  # one entry per open `ifdef/`ifndef; True if formal-guarded
    for line in text.split('\n'):
        m = DIRECTIVE_RE.match(line)
        if m:
            kind, name = m.group(1), m.group(2)
            if kind in ('ifdef', 'ifndef'):
                inside_formal = bool(stack and stack[-1]) or \
                    (kind == 'ifdef' and name == 'RISCV_FORMAL')
                stack.append(inside_formal)
            elif kind == 'endif' and stack:
                stack.pop()
            out.append('')
            continue
        out.append(line if not (stack and stack[-1]) else '')
    return '\n'.join(out)


PORT_DECL = re.compile(r'^\s*(input|output|inout)\b(.*)$', re.S)
RANGE = re.compile(r'\[([0-9]+):([0-9]+)\]')
WORD = re.compile(r'[A-Za-z_]\w*')
STRUCT_DEF_RE = re.compile(
    r'typedef\s+struct\s+packed\s*\{(.*?)\}\s*(\w+)\s*;', re.S)
FIELD_RE = re.compile(r'^\s*(\w+)\s*(\[[0-9]+:[0-9]+\])?\s+(\w+)\s*;\s*$')


def bit_width(range_match):
    if range_match is None:
        return 1
    hi, lo = int(range_match.group(1)), int(range_match.group(2))
    return hi - lo + 1


def module_ports(text, module_name):
    """(direction, type, name, width) for every port `module_name` declares
    directly, read off with matched parentheses the way
    test/port_connect_test.py reads littlecpu's -- restricted to text
    `strip_formal_ifdefs` leaves alone, i.e. the ports every build has.
    `width` is None for a struct-typed port; the caller resolves that
    against `parse_structs`. Returns (None, error) if the module, its port
    list, or one entry in it cannot be read."""
    m = re.search(r'\bmodule\s+%s\b' % re.escape(module_name), text)
    if not m:
        return None, f'no `module {module_name}` found'
    open_paren = text.find('(', m.end())
    if open_paren < 0:
        return None, f'module {module_name} has no port list this script can find'
    if text[m.end():open_paren].strip().startswith('#'):
        params_close = matching_paren(text, open_paren)
        if params_close is None:
            return None, "the parameter list has no closing parenthesis"
        open_paren = text.find('(', params_close + 1)
    close = matching_paren(text, open_paren) if open_paren >= 0 else None
    if close is None:
        return None, 'the port list has no closing parenthesis this script can find'
    entries = top_level_split(text[open_paren + 1:close], ',')
    ports = []
    for entry in entries:
        entry = entry.strip()
        if not entry:
            continue
        pm = PORT_DECL.match(entry)
        if not pm:
            return None, ('cannot read this port declaration: '
                           + ' '.join(entry.split()))
        rng = RANGE.search(pm.group(2))
        words = WORD.findall(RANGE.sub(' ', pm.group(2)))
        if not words:
            return None, 'a port entry declares a direction and no name'
        name = words[-1]
        type_name = words[0] if len(words) > 1 else 'logic'
        width = bit_width(rng) if type_name == 'logic' else None
        ports.append((pm.group(1), type_name, name, width))
    if not ports:
        return None, 'the port list parsed as empty'
    return ports, None


def parse_structs(text):
    """{struct_name: {field_name: width}}, restricted to fields
    `strip_formal_ifdefs` leaves alone. A field whose own type is not
    `logic` -- a nested struct -- is skipped rather than expanded: none of
    the port types this script resolves (fetcher_output, executor_output)
    have one outside RISCV_FORMAL, which is already excluded."""
    structs = {}
    for sm in STRUCT_DEF_RE.finditer(text):
        fields = {}
        for line in sm.group(1).split('\n'):
            fm = FIELD_RE.match(line)
            if fm and fm.group(1) == 'logic':
                fields[fm.group(3)] = bit_width(RANGE.search(line))
        structs[sm.group(2)] = fields
    return structs


def classify_wide_inputs(decoder_text, structs_text, module_name='decoder'):
    """{name: width} for every INPUT port of `module_name` (or field of a
    struct-typed one) wider than 5 bits -- wide enough to be a register-file
    DATA output rather than a register NUMBER, since every register-number
    port in this file is `[4:0]`. Returns (None, error) if a port's type is
    a struct this script found no definition for, so a new struct-typed port
    cannot silently skip classification by having no entry to expand."""
    formal_stripped = strip_formal_ifdefs(decoder_text)
    ports, err = module_ports(formal_stripped, module_name)
    if err:
        return None, err
    structs = parse_structs(strip_formal_ifdefs(structs_text))
    wide = {}
    for direction, type_name, name, width in ports:
        if direction != 'input':
            continue
        if type_name == 'logic':
            if width > 5:
                wide[name] = width
            continue
        fields = structs.get(type_name)
        if fields is None:
            return None, (f'port `{name}` has type `{type_name}`, which this '
                           f'script found no `typedef struct packed` for in '
                           f'structs.v')
        for fname, fwidth in fields.items():
            if fwidth > 5:
                wide[f'{name}.{fname}'] = fwidth
    return wide, None


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

    structs_path = os.path.join(os.path.dirname(path), 'structs.v')
    try:
        with open(structs_path) as f:
            structs_text = f.read()
    except OSError as e:
        print(f'error: cannot read {structs_path}: {e}', file=sys.stderr)
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

    wide_inputs, err = classify_wide_inputs(strip_comments(text), structs_text)
    if err:
        print(f'error: {err}', file=sys.stderr)
        return 2
    unclassified_ports = sorted(
        n for n in wide_inputs if n not in SEEDS and n not in NON_VALUE_INPUTS)
    if unclassified_ports:
        print(f'error: {path} has an input wider than 5 bits with no Zkt '
              f'classification: {", ".join(unclassified_ports)}. Add it to '
              f'SEEDS if it can carry a register-file or CSR-file DATA '
              f'output, or to NON_VALUE_INPUTS with the reason it cannot.',
              file=sys.stderr)
        return 2
    stale_classifications = sorted(
        n for n in (SEEDS | set(NON_VALUE_INPUTS)) if n not in wide_inputs)
    if stale_classifications:
        print(f'error: {", ".join(stale_classifications)} is classified as '
              f'a Zkt-relevant input but this script no longer finds it '
              f'among {path}\'s inputs wider than 5 bits. Remove the stale '
              f'entry from SEEDS or NON_VALUE_INPUTS.', file=sys.stderr)
        return 2

    deps = build_deps(assigns)
    targets = list(MUST_NOT_DEPEND) + [GATED_SIGNAL]
    unclassified_leaves = sorted(
        n for n in fanin_leaves(deps, targets)
        if n not in KNOWN_CLEAN_LEAVES and n not in SEEDS)
    if unclassified_leaves:
        print(f'error: {path} reaches {", ".join(unclassified_leaves)} from '
              f'{", ".join(targets)}, and this script found no continuous '
              f'assign defining {"it" if len(unclassified_leaves) == 1 else "them"}. '
              f'A signal read here that is not a module input, an always_ff '
              f'register or submodule output already in KNOWN_CLEAN_LEAVES, '
              f'or a SEEDS entry might be carrying a register value through '
              f'a shape this script cannot trace -- classify it in one of '
              f'those two sets, or fix the RTL, rather than reading it as '
              f'clean by default.', file=sys.stderr)
        return 2

    full = taint_closure(deps)
    restricted = taint_closure(deps, blocked={GATED_SIGNAL})

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

    or_terms = top_level_split(assigns[GATED_SIGNAL], '||')
    if len(or_terms) > 1:
        failures.append(
            f'`{GATED_SIGNAL}` has a top-level `||`, not just `&&`:\n'
            f'    assign {GATED_SIGNAL} = {assigns[GATED_SIGNAL].strip()};\n'
            f'  Every term here is meant to be ANDed with {GATE_TERM}; a '
            f'term joined by `||` instead can make {GATED_SIGNAL} assert '
            f'whether or not {GATE_TERM} does, regardless of what that term '
            f'reads or whether {GATE_TERM} still appears among the `&&` '
            f'terms.')

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

    print(f'{path}: reg_rs1/reg_rs2/executor_out.rd_data reach only '
          f'{GATED_SIGNAL} among the stall reasons, and it stays gated on '
          f'{GATE_TERM}.')
    print('ZKT STALL ISOLATION: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
