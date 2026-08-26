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
     able to excuse itself from its own check. A predicate may narrow past the
     opcode with a funct3, and past that with a funct7, but nothing besides
     those three fields -- naming the field is what keeps a predicate readable
     without cross-referencing rtl/decoder.v.
  5. Every declared mnemonic must really have no spec model at the pin --
     re-derived from the clone, not believed. If a future pin adds
     insns/insn_fence.v to the mnemonics isa_rv32imc.txt names, this fails and
     the exclusion has to come out. That is the failure direction a baseline
     alone cannot give you: a stale exclusion silently covers less and less of
     the ISA while staying green.
  6. A predicate must be AS NARROW AS THE PIN'S OWN MODEL LETS IT BE: this
     script re-derives, from every isa_rv32imc.txt mnemonic's own spec_valid
     line, the opcode (and funct3 and funct7 where that mnemonic's model
     tests them) every modelled instruction retires under, and rejects a
     declared predicate that would ALSO match one of them. A class-wide
     predicate over an opcode the pin partly models is rejected the same way
     a stale mnemonic is: not because it is wrong today, but because it
     excuses more than the pin actually leaves unmodelled. This is what makes
     the widened predicate shape an amendment rather than a loosening -- it
     can only ever cover LESS of an opcode than the shape it replaces, never
     more, and this clause is what enforces that direction.

Clause 5's site is `insns/isa_rv32imc.txt`, and only that site: riscv-formal
vendors per-instruction models for other extensions (Zba, Zbb, ...) under
their own isa_rv32i*.txt lists, entirely apart from the rv32imc one
formal/complete.sv's `rvfi_isa_rv32imc` instance is built from, so a model
file existing somewhere under insns/ says nothing about whether this check can
ever see it. A mnemonic reachable only through one of those other lists is,
for every purpose this script cares about, unmodelled -- re-derived from
isa_rv32imc.txt because that is the one list wired into the instance the
assertion actually reads.

Usage: check-complete-exclusions.py <complete.sv> <COMPLETE_EXCLUSIONS> <riscv-formal dir>
"""

import os
import re
import sys

# `// EXCLUDE <CLASS> <encoding> <mnemonics...>`. The word boundary after
# EXCLUDE is load-bearing: complete.sv's prose says "EXCLUDING AN OPCODE FROM
# AN ASSERTION IS WEAKENING A CHECK" a few lines above, and a looser pattern
# would read that as an entry. <encoding> is validated by parse_encoding_field
# rather than folded into this regex, so a malformed one gets a specific
# error instead of silently failing to match at all.
DECL_RE = re.compile(r'^\s*//\s*EXCLUDE\s+(\S+)\s+(\S+)\s+(\S.*?)\s*$')

# The three predicate shapes an entry is allowed to have: opcode alone,
# opcode+funct3, or opcode+funct3+funct7 -- never funct7 without funct3, which
# RISC-V's own encoding never discriminates on either. Anything else -- a
# decoder flag, a spec_* signal, an rvfi_trap term -- is rejected rather than
# parsed, because the point is that the reader of a green `complete` can trust
# the exclusions are encoding-keyed without reading them. A fourth field is a
# design change and belongs in an ADR, not in a regex.
WIRE_RE = re.compile(
    r"^\s*wire\s+(\w+)\s*=\s*insn_uncompressed\s*&&\s*"
    r"insn_opcode\s*==\s*7'b([01]{7})"
    r"(?:\s*&&\s*insn_funct3\s*==\s*3'b([01]{3})"
    r"(?:\s*&&\s*insn_funct7\s*==\s*7'b([01]{7}))?"
    r")?\s*;\s*$")

# The encoding field of a declaration or a baseline line: a bare opcode, or an
# opcode narrowed by a funct3, or by a funct3 and a funct7 -- the same three
# shapes WIRE_RE accepts, spelled as `7 bits[/3 bits[/7 bits]]` so the format
# stays a single whitespace-separated token.
FIELD_RE = re.compile(r'^([01]{7})(?:/([01]{3})(?:/([01]{7}))?)?$')

# The four definitions everything above is built out of. Pinned literally: if
# `insn_opcode` were ever redefined off something other than rvfi_insn, every
# WIRE_RE match above would still pass while meaning something entirely
# different.
REQUIRED_DEFS = {
    'insn_uncompressed': "wire        insn_uncompressed = rvfi_insn[1:0] == 2'b11;",
    'insn_opcode':       "wire [6:0]  insn_opcode       = rvfi_insn[6:0];",
    'insn_funct3':       "wire [2:0]  insn_funct3       = rvfi_insn[14:12];",
    'insn_funct7':       "wire [6:0]  insn_funct7       = rvfi_insn[31:25];",
}

EXCLUDED_RE = re.compile(r'^\s*wire\s+insn_excluded\s*=\s*(.+?)\s*;\s*$')

# A modelled instruction's own spec_valid line, out of insns/insn_<m>.v.
SPEC_VALID_RE = re.compile(r'assign\s+spec_valid\s*=(.*?);', re.S)
MODEL_OPCODE_RE = re.compile(r"insn_opcode\s*==\s*7'b\s*([01]{7})")
# A compressed mnemonic's own module redeclares insn_opcode as a 2-bit
# quadrant field local to that file; it can never match an
# insn_uncompressed-gated predicate, so it is recognised and skipped rather
# than reported as an unresolvable encoding.
MODEL_COMPRESSED_OPCODE_RE = re.compile(r"insn_opcode\s*==\s*2'b\s*[01]{2}")
MODEL_FUNCT3_RE = re.compile(r"insn_funct3\s*==\s*3'b\s*([01]{3})")
MODEL_FUNCT7_RE = re.compile(r"insn_funct7\s*==\s*7'b\s*([01]{7})")
# RV32's shift-immediate family (SLLI/SRLI/SRAI and their bit-manip cousins)
# tests funct6 -- bits [31:26] -- leaving bit 25 (funct7's low bit, the RV64
# half of a 6-bit shift amount) free. Represented as a funct7 pattern with
# that bit a wildcard, so it still narrows an opcode+funct3 match instead of
# silently vanishing from the index.
MODEL_FUNCT6_RE = re.compile(r"insn_funct6\s*==\s*6'b\s*([01]{6})")


def slug(cls):
    """CLASS name -> the wire name complete.sv must use for it."""
    return 'exclude_' + cls.lower().replace('-', '_')


def format_encoding(opcode, funct3, funct7):
    """The human-readable form of a declared (opcode, funct3, funct7) triple,
    funct3/funct7 omitted where the exclusion does not narrow that far."""
    parts = [f"opcode 7'b{opcode}"]
    if funct3 is not None:
        parts.append(f"funct3 3'b{funct3}")
    if funct7 is not None:
        parts.append(f"funct7 7'b{funct7}")
    return ', '.join(parts)


def parse_encoding_field(raw):
    """`<opcode>[/<funct3>[/<funct7>]]` -> (opcode, funct3_or_None,
    funct7_or_None), or None if the token does not have that shape."""
    m = FIELD_RE.match(raw)
    if not m:
        return None
    return m.group(1), m.group(2), m.group(3)


def parse_complete_sv(path):
    """Return {class: (opcode, funct3, funct7, [mnemonics])} plus the wired
    set, or die."""
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
        cls, encoding_field, mnemonics = m.group(1), m.group(2), m.group(3).split()
        parsed = parse_encoding_field(encoding_field)
        if parsed is None:
            errors.append(
                f"{path}:{i + 1}: exclusion {cls} names encoding "
                f"{encoding_field!r}, which is not `<7 bits>[/<3 bits>"
                f"[/<7 bits>]]`")
            continue
        opcode, funct3, funct7 = parsed
        if cls in declared:
            errors.append(f'{path}:{i + 1}: duplicate exclusion class {cls}')
        declared[cls] = (opcode, funct3, funct7, mnemonics, i + 1)

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
                f"    wire {slug(cls)} = insn_uncompressed && insn_opcode == 7'b{opcode}"
                + (f" && insn_funct3 == 3'b{funct3}" if funct3 else '')
                + (f" && insn_funct7 == 7'b{funct7}" if funct7 else '') + ";\n"
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
        wire_encoding = (wm.group(2), wm.group(3), wm.group(4))
        if wire_encoding != (opcode, funct3, funct7):
            errors.append(
                f'{path}:{j + 1}: exclusion {cls} declares '
                f'{format_encoding(opcode, funct3, funct7)} but its wire '
                f'matches {format_encoding(*wire_encoding)}')

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
            cls, encoding_field, mnemonics = fields[0], fields[1], fields[2:]
            parsed = parse_encoding_field(encoding_field)
            if parsed is None:
                errors.append(
                    f"{path}:{i}: {cls} names encoding {encoding_field!r}, "
                    f"which is not `<7 bits>[/<3 bits>[/<7 bits>]]`")
                continue
            opcode, funct3, funct7 = parsed
            if cls in entries:
                errors.append(f'{path}:{i}: duplicate exclusion class {cls}')
            entries[cls] = (opcode, funct3, funct7, mnemonics)
    return entries, errors


def load_modelled_encodings(rf_dir):
    """Clauses 5 and 6's oracle: {mnemonic: (opcode, funct3_or_None,
    funct7_or_None)} for every mnemonic insns/isa_rv32imc.txt names -- the
    exact set formal/complete.sv's `rvfi_isa_rv32imc` instance can recognise.
    funct7 may carry a trailing '?' wildcard bit, from a shift-immediate
    mnemonic whose own model narrows only a funct6.

    Returns (encodings, errors); encodings is None if the clone could not be
    read at all, in which case nothing downstream can trust its silence.
    """
    isa_txt = os.path.join(rf_dir, 'insns', 'isa_rv32imc.txt')
    try:
        with open(isa_txt) as f:
            mnemonics = [line.strip() for line in f if line.strip()]
    except OSError as e:
        return None, [f'cannot read {isa_txt}: {e}. The pinned riscv-formal '
                       f'clone is what makes "no spec model" a measurement '
                       f'rather than a claim; run `make -C formal riscv-formal` '
                       f'first.']

    encodings = {}
    errors = []
    for m in mnemonics:
        fname = 'insn_%s.v' % m.replace('.', '_')
        path = os.path.join(rf_dir, 'insns', fname)
        try:
            with open(path) as f:
                text = f.read()
        except OSError as e:
            errors.append(
                f'{isa_txt} names {m}, but {path} does not exist ({e}). Every '
                f'mnemonic that file names is supposed to be backed by its own '
                f'model; an unresolvable encoding is a red, not a pass.')
            continue
        sv = SPEC_VALID_RE.search(text)
        if not sv:
            errors.append(
                f'{path}: no `assign spec_valid = ...;` found, so {m}\'s '
                f'encoding cannot be re-derived. Fail closed rather than guess '
                f'whether this mnemonic overlaps a declared exclusion.')
            continue
        expr = sv.group(1)
        op = MODEL_OPCODE_RE.search(expr)
        if not op:
            if MODEL_COMPRESSED_OPCODE_RE.search(expr):
                # A compressed mnemonic: no exclusion in complete.sv can ever
                # match it, since every predicate there is insn_uncompressed-
                # gated. Not an entry in the index, and not an error either.
                continue
            errors.append(
                f"{path}: spec_valid names no 7-bit insn_opcode and no 2-bit "
                f"compressed one either, so {m}'s encoding cannot be classified. "
                f"Fail closed rather than assume it cannot overlap a declared "
                f"exclusion.")
            continue
        f3 = MODEL_FUNCT3_RE.search(expr)
        f7 = MODEL_FUNCT7_RE.search(expr)
        f6 = MODEL_FUNCT6_RE.search(expr)
        funct7 = f7.group(1) if f7 else (f6.group(1) + '?' if f6 else None)
        encodings[m] = (op.group(1), f3.group(1) if f3 else None, funct7)
    return encodings, errors


def _funct7_compatible(pattern, value):
    """`pattern` may carry a wildcard bit from a funct6-only model; True if
    every bit `pattern` actually constrains agrees with `value`."""
    return all(p == '?' or p == v for p, v in zip(pattern, value))


def encoding_overlaps(excl, model):
    """True if a declared exclusion's (opcode, funct3_or_None, funct7_or_None)
    would also match a modelled instruction's own triple -- i.e. the
    exclusion, as written, excuses that instruction from the assertion too.
    A None on either side is that field left unconstrained, which matches
    whatever the other side has."""
    excl_op, excl_f3, excl_f7 = excl
    model_op, model_f3, model_f7 = model
    if excl_op != model_op:
        return False
    if excl_f3 is not None and model_f3 is not None and excl_f3 != model_f3:
        return False
    if excl_f7 is not None and model_f7 is not None:
        if not _funct7_compatible(model_f7, excl_f7):
            return False
    return True


def check_encodings(baseline, rf_dir):
    """Clauses 5 and 6 against every declared exclusion in COMPLETE_EXCLUSIONS,
    re-derived from the clone rather than trusted from the declared mnemonic
    list -- an exclusion is graded on what it actually matches, not on what
    its author meant it to."""
    encodings, errors = load_modelled_encodings(rf_dir)
    if encodings is None:
        return errors

    for cls, (opcode, funct3, funct7, mnemonics) in sorted(baseline.items()):
        for m in mnemonics:
            if m in encodings:
                errors.append(
                    f'{m}: named under {cls}, but rvfi_isa_rv32imc models it at '
                    f'{format_encoding(*encodings[m])} -- delete its entry (and, '
                    f'if it is the last of its opcode class, the whole '
                    f'exclusion) and let `complete` check it.')

        overlap = sorted(m for m, enc in encodings.items()
                          if encoding_overlaps((opcode, funct3, funct7), enc))
        if overlap:
            errors.append(
                f'{cls}: the predicate for {format_encoding(opcode, funct3, funct7)} '
                f'also matches {", ".join(overlap)}, which rvfi_isa_rv32imc models. '
                f'Narrow the predicate with a funct3, or a funct3 and a funct7, so '
                f'it excuses only the row the pin does not model -- a class-wide '
                f'exclusion where a per-encoding one would do is exactly the '
                f'widening this script exists to catch.')
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
    # funct3, funct7, mnemonic tuple), so a line that keeps its name and
    # quietly widens its opcode, narrows to a different funct3/funct7, or
    # widens its mnemonic list is a mismatch too.
    decl_set = {(c, o, f3, f7, tuple(m)) for c, (o, f3, f7, m, _) in declared.items()}
    base_set = {(c, o, f3, f7, tuple(m)) for c, (o, f3, f7, m) in baseline.items()}
    for cls, opcode, funct3, funct7, mnemonics in sorted(decl_set - base_set):
        errors.append(
            f'{sv_path} declares an exclusion that {baseline_path} does not:\n'
            f'    {cls}  {format_encoding(opcode, funct3, funct7)}  {" ".join(mnemonics)}\n'
            f'  Widening the exclusion set is weakening the check. If it is the '
            f'right call, add the line to the baseline in the same commit and '
            f'say why in the pull request.')
    for cls, opcode, funct3, funct7, mnemonics in sorted(base_set - decl_set):
        errors.append(
            f'{baseline_path} names an exclusion {sv_path} does not declare:\n'
            f'    {cls}  {format_encoding(opcode, funct3, funct7)}  {" ".join(mnemonics)}\n'
            f'  Either the predicate was removed and the baseline was not, or '
            f'the two disagree on the encoding or the mnemonic list.')

    want_wires = {slug(c) for c in declared}
    if set(wired) != want_wires:
        errors.append(
            f'{sv_path}: insn_excluded is the disjunction of {sorted(wired)}, '
            f'but the declared exclusions are {sorted(want_wires)}. Every '
            f'declared entry must be wired in and nothing else may be.')
    if len(wired) != len(set(wired)):
        errors.append(f'{sv_path}: insn_excluded repeats a term: {wired}')

    errors += check_encodings(baseline, rf_dir)

    if errors:
        print('COMPLETE EXCLUSION SET: FAIL', file=sys.stderr)
        for e in errors:
            print('  ' + e.replace('\n', '\n  '), file=sys.stderr)
        return 1

    print(f'complete.sv exclusion set matches {baseline_path} '
          f'({len(declared)} entries, both directions):')
    for cls in sorted(declared):
        opcode, funct3, funct7, mnemonics, _ = declared[cls]
        print(f"  {cls:<9} {format_encoding(opcode, funct3, funct7)}  "
              f"{len(mnemonics)} mnemonics, no spec model at the pin: "
              f"{' '.join(mnemonics)}")
    print('COMPLETE EXCLUSION SET: PASS')
    return 0


if __name__ == "__main__":
    sys.exit(main())
