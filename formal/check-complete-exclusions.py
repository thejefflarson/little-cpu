#!/usr/bin/env python3
"""Fail if formal/complete.sv's exclusion set has drifted from its baseline.

`complete` is the whole-ISA walk: for every retire the core reports, the
riscv-formal spec model must recognise the instruction and agree it does not
trap. It cannot cover the whole ISA, because riscv-formal ships no spec model
at all for MISC-MEM or SYSTEM at formal/pin.mk's SHA -- so complete.sv declares
an exclusion set, and this script is what stops that set from growing.

Excluding an opcode from an assertion is weakening a check. ADR-0010's rule --
restrict the proof, and record the restriction -- is what makes it admissible,
and a recorded restriction nothing compares against is not recorded. So:

  1. The set declared in complete.sv and the set in COMPLETE_EXCLUSIONS must
     match EXACTLY, in both directions (ADR-0014). Adding an entry to either
     side alone fails.
  2. Every declared entry must have a reason written at the complete.sv site.
  3. Every declared entry must be WIRED: `insn_excluded` must be exactly the
     disjunction of the declared wires, no more and no fewer. A declared-but-
     unwired entry would over-report the restriction; a wired-but-undeclared
     one would under-report it, which is the direction that matters.
  4. Every excluded predicate must key on the INSTRUCTION ENCODING and nothing
     else. This is the property a one-line edit destroys most easily ("use the
     decoder's is_fence flag, it's already there") and it is the one that keeps
     the check honest: a core that mis-decodes an ADD as a FENCE must not be
     able to excuse itself from its own check.
  5. Every excluded mnemonic must really have no spec model at the pin --
     re-derived from the clone, not believed. If a future pin adds
     insns/insn_fence.v, this fails and the exclusion has to come out. That is
     the failure direction a baseline alone cannot give you: a stale exclusion
     silently covers less and less of the ISA while staying green.

Usage: check-complete-exclusions.py <complete.sv> <COMPLETE_EXCLUSIONS> <riscv-formal dir>
"""

import os
import re
import sys

# `// EXCLUDE <CLASS> <7 bits> <mnemonics...>`. The word boundary after EXCLUDE
# is load-bearing: complete.sv's prose says "EXCLUDING AN OPCODE FROM AN
# ASSERTION IS WEAKENING A CHECK" a few lines above, and a looser pattern would
# read that as an entry.
DECL_RE = re.compile(r'^\s*//\s*EXCLUDE\s+(\S+)\s+([01]{7})\s+(\S.*?)\s*$')

# The one predicate shape an entry is allowed to have. Anything else -- a
# decoder flag, a spec_* signal, a funct3 carve-out, an rvfi_trap term -- is
# rejected rather than parsed, because the point is that the reader of a green
# `complete` can trust the exclusions are encoding-keyed without reading them.
# Widening this pattern is a design change and belongs in an ADR, not in a
# regex.
WIRE_RE = re.compile(
    r'^\s*wire\s+(\w+)\s*=\s*insn_uncompressed\s*&&\s*'
    r"insn_opcode\s*==\s*7'b([01]{7})\s*;\s*$")

# The two definitions everything above is built out of. Pinned literally: if
# `insn_opcode` were ever redefined off something other than rvfi_insn, every
# WIRE_RE match above would still pass while meaning something entirely
# different.
REQUIRED_DEFS = {
    'insn_uncompressed': "wire        insn_uncompressed = rvfi_insn[1:0] == 2'b11;",
    'insn_opcode':       "wire [6:0]  insn_opcode       = rvfi_insn[6:0];",
}

EXCLUDED_RE = re.compile(r'^\s*wire\s+insn_excluded\s*=\s*(.+?)\s*;\s*$')


def slug(cls):
    """CLASS name -> the wire name complete.sv must use for it."""
    return 'exclude_' + cls.lower().replace('-', '_')


def parse_complete_sv(path):
    """Return {class: (opcode, [mnemonics])} plus the wired set, or die."""
    errors = []
    with open(path) as f:
        lines = f.read().splitlines()

    stripped = {line.strip() for line in lines}
    for name, want in REQUIRED_DEFS.items():
        if want not in stripped:
            errors.append(
                f'{path}: expected the exact line defining {name}:\n'
                f'    {want}\n'
                f'  Everything this script accepts as an exclusion predicate is\n'
                f'  built from that definition. If it moved, the predicates no\n'
                f'  longer provably key on rvfi_insn.')

    declared = {}
    for i, line in enumerate(lines):
        m = DECL_RE.match(line)
        if not m:
            continue
        cls, opcode, mnemonics = m.group(1), m.group(2), m.group(3).split()
        if cls in declared:
            errors.append(f'{path}:{i + 1}: duplicate exclusion class {cls}')
        declared[cls] = (opcode, mnemonics, i + 1)

        # Clause 2: a reason, on the continuation comment lines under the
        # header. A bare entry with no prose is exactly the "recorded" that
        # records nothing.
        reason = []
        j = i + 1
        while j < len(lines) and lines[j].lstrip().startswith('//'):
            reason.append(lines[j].lstrip()[2:].strip())
            j += 1
        if len(' '.join(reason).strip()) < 40:
            errors.append(
                f'{path}:{i + 1}: exclusion {cls} has no reason written under '
                f'it. ADR-0010 admits a restriction only when it is recorded; '
                f'write what has no spec model and what checks it instead.')

        # Clause 4 (and 3, half of it): the predicate itself.
        if j >= len(lines):
            errors.append(f'{path}:{i + 1}: exclusion {cls} declares no wire')
            continue
        wm = WIRE_RE.match(lines[j])
        if not wm:
            errors.append(
                f'{path}:{j + 1}: exclusion {cls} must be followed by exactly\n'
                f"    wire {slug(cls)} = insn_uncompressed && insn_opcode == 7'b{opcode};\n"
                f'  got: {lines[j].strip()!r}\n'
                f'  The predicate must key on the ENCODING out of rvfi_insn and\n'
                f'  nothing else -- no decoder flag, no spec_* signal. A core\n'
                f'  that mis-decodes an instruction must not be able to excuse\n'
                f'  itself from its own check.')
            continue
        if wm.group(1) != slug(cls):
            errors.append(
                f'{path}:{j + 1}: exclusion {cls} must name its wire '
                f'{slug(cls)}, got {wm.group(1)}')
        if wm.group(2) != opcode:
            errors.append(
                f'{path}:{j + 1}: exclusion {cls} declares opcode {opcode} but '
                f'its wire matches {wm.group(2)}')

    # Clause 3: insn_excluded is exactly the disjunction of the declared wires.
    wired = None
    for i, line in enumerate(lines):
        m = EXCLUDED_RE.match(line)
        if not m:
            continue
        if wired is not None:
            errors.append(f'{path}:{i + 1}: insn_excluded is assigned twice')
        terms = [t.strip() for t in m.group(1).split('||')]
        if any(not re.fullmatch(r'\w+', t) for t in terms):
            errors.append(
                f'{path}:{i + 1}: insn_excluded must be a plain `a || b` over '
                f'the declared exclusion wires, got {m.group(1)!r}')
            terms = []
        wired = terms
    if wired is None:
        errors.append(f'{path}: no `wire insn_excluded = ...;` assignment found')
        wired = []

    return declared, wired, errors


def parse_baseline(path):
    entries = {}
    errors = []
    with open(path) as f:
        for i, raw in enumerate(f, 1):
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) < 3:
                errors.append(
                    f'{path}:{i}: expected `<CLASS>  <opcode>  <mnemonic>...`, '
                    f'got {line!r}')
                continue
            cls, opcode, mnemonics = fields[0], fields[1], fields[2:]
            if not re.fullmatch(r'[01]{7}', opcode):
                errors.append(
                    f'{path}:{i}: {cls} opcode must be seven binary digits, '
                    f'got {opcode!r}')
                continue
            if cls in entries:
                errors.append(f'{path}:{i}: duplicate exclusion class {cls}')
            entries[cls] = (opcode, mnemonics)
    return entries, errors


def check_no_spec_model(mnemonics, rf_dir):
    """Clause 5: re-derive `no spec model at the pin` from the clone."""
    errors = []
    isa_txt = os.path.join(rf_dir, 'insns', 'isa_rv32imc.txt')
    try:
        with open(isa_txt) as f:
            isa = {line.strip() for line in f if line.strip()}
    except OSError as e:
        return [f'cannot read {isa_txt}: {e}. The pinned riscv-formal clone is '
                f'what makes "no spec model" a measurement rather than a claim; '
                f'run `make -C formal riscv-formal` first.']
    for m in sorted(mnemonics):
        fname = 'insn_%s.v' % m.replace('.', '_')
        model = os.path.join(rf_dir, 'insns', fname)
        if os.path.exists(model):
            errors.append(
                f'{m}: {model} EXISTS at the pin, so this mnemonic has a spec '
                f'model and must not be excluded. Delete its entry (and, if it '
                f'is the last of its opcode class, the whole exclusion) and let '
                f'`complete` check it.')
        if m.replace('.', '_') in isa or m in isa:
            errors.append(
                f'{m}: named in {isa_txt}, so rvfi_isa_rv32imc drives a spec '
                f'model for it and the exclusion is stale.')
    return errors


def main():
    if len(sys.argv) != 4:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    sv_path, baseline_path, rf_dir = sys.argv[1:4]

    declared, wired, errors = parse_complete_sv(sv_path)
    baseline, berrors = parse_baseline(baseline_path)
    errors += berrors

    # The set equality, in both directions. Compared on (class, opcode,
    # mnemonic tuple), so a line that keeps its name and quietly widens its
    # opcode or its mnemonic list is a mismatch too.
    decl_set = {(c, o, tuple(m)) for c, (o, m, _) in declared.items()}
    base_set = {(c, o, tuple(m)) for c, (o, m) in baseline.items()}
    for cls, opcode, mnemonics in sorted(decl_set - base_set):
        errors.append(
            f'{sv_path} declares an exclusion that {baseline_path} does not:\n'
            f'    {cls}  {opcode}  {" ".join(mnemonics)}\n'
            f'  Widening the exclusion set is weakening the check. If it is the '
            f'right call, add the line to the baseline in the same commit and '
            f'say why in the pull request.')
    for cls, opcode, mnemonics in sorted(base_set - decl_set):
        errors.append(
            f'{baseline_path} names an exclusion {sv_path} does not declare:\n'
            f'    {cls}  {opcode}  {" ".join(mnemonics)}\n'
            f'  Either the predicate was removed and the baseline was not, or '
            f'the two disagree on the opcode or the mnemonic list.')

    want_wires = {slug(c) for c in declared}
    if set(wired) != want_wires:
        errors.append(
            f'{sv_path}: insn_excluded is the disjunction of {sorted(wired)}, '
            f'but the declared exclusions are {sorted(want_wires)}. Every '
            f'declared entry must be wired in and nothing else may be.')
    if len(wired) != len(set(wired)):
        errors.append(f'{sv_path}: insn_excluded repeats a term: {wired}')

    all_mnemonics = {m for _, ms in baseline.values() for m in ms}
    errors += check_no_spec_model(all_mnemonics, rf_dir)

    if errors:
        print('COMPLETE EXCLUSION SET: FAIL', file=sys.stderr)
        for e in errors:
            print('  ' + e.replace('\n', '\n  '), file=sys.stderr)
        return 1

    print(f'complete.sv exclusion set matches {baseline_path} '
          f'({len(declared)} entries, both directions):')
    for cls in sorted(declared):
        opcode, mnemonics, _ = declared[cls]
        print(f"  {cls:<9} 7'b{opcode}  {len(mnemonics)} mnemonics, "
              f"no spec model at the pin: {' '.join(mnemonics)}")
    print('COMPLETE EXCLUSION SET: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
