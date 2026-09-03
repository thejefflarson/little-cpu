#!/usr/bin/env python3
"""Fail if the interrupt tie-off has drifted from formal/INTERRUPT_TIE_OFF.

Every riscv-formal harness in formal/ instantiates the core with its timer
input tied low, so the generated checks run with no interrupt in the trace.
That is a RESTRICTION on what those checks cover, and the rule this repo works
to -- restrict the proof, and record the restriction -- is what makes it
admissible. A recorded restriction nothing compares against is not recorded,
which is what this script is for.

Four things are decided here:

  1. The set of files in formal/ that instantiate `littlecpu` and the HARNESS
     set in the baseline must match EXACTLY, in both directions. A harness
     added without a line is red; a line with no harness is red too.
  2. Every declared harness must actually connect `.irq_timer(1'b0)`. A
     declared-but-untied harness would under-report the restriction, which is
     the direction that matters: the checks would be running against a machine
     the baseline says they are not.
  3. The set of files under the pinned clone's checks/ that mention
     `rvfi_intr` and the UPSTREAM set must match EXACTLY, in both directions.
     This is the re-derivation a baseline alone cannot give: the tie-off is
     argued on riscv-formal having nothing to say about an interrupt, and a
     pin bump that adds a check reading `rvfi_intr` means it now does.
  4. No CHECK under the pinned clone's checks/ may name an interrupt CSR --
     mie, mip or mstatus. The moment one does, upstream has a spec model for
     the behaviour this repo asserts by hand, and the tie-off has to be argued
     again rather than inherited. Scoped to `rvfi_*_check.sv`, the files that
     carry assertions: checks/rvfi_macros.vh declares a port for every CSR in
     riscv-formal's table, which is plumbing and says nothing about behaviour.

Usage: check-interrupt-tie-off.py <formal dir> <INTERRUPT_TIE_OFF> <riscv-formal dir>
"""

import os
import re
import sys

INSTANTIATES = re.compile(r'^\s*littlecpu\s+\w+\s*\(\s*$', re.M)

# The one spelling a tie-off is allowed to have. A wire that happens to be low,
# or a parameter, would read the same to a human and mean something a reader
# cannot check locally.
TIE_OFF = re.compile(r"^\s*\.irq_timer\(1'b0\)\s*,?\s*$", re.M)

INTR_SIGNAL = 'rvfi_intr'

# Bounded by anything that is not a letter or a digit, so `rvfi_csr_mstatus_wdata`
# counts -- an underscore-separated component of an identifier is the shape a
# CSR name actually takes upstream -- while `premier` does not. `\b` would miss
# exactly the identifiers that matter.
CSR_NAMES = ('mie', 'mip', 'mstatus')
CSR_RE = {csr: re.compile(rf'(?<![A-Za-z0-9]){csr}(?![A-Za-z0-9])')
          for csr in CSR_NAMES}

SEARCHED_SUFFIXES = ('.v', '.sv', '.vh')

# The files upstream that carry assertions, as opposed to port declarations.
CHECK_FILE = re.compile(r'^rvfi_\w+_check\.sv$')


def scan_harnesses(formal_dir):
    """Files in formal/ that instantiate littlecpu, and whether each ties off."""
    found = {}
    errors = []
    for name in sorted(os.listdir(formal_dir)):
        if not name.endswith(SEARCHED_SUFFIXES):
            continue
        path = os.path.join(formal_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            text = open(path).read()
        except OSError as e:
            errors.append(f'cannot read {path}: {e}')
            continue
        if INSTANTIATES.search(text):
            found[name] = bool(TIE_OFF.search(text))
    return found, errors


def scan_upstream(rf_dir):
    """checks/ files at the pin that mention rvfi_intr, and any CSR modelling."""
    checks = os.path.join(rf_dir, 'checks')
    if not os.path.isdir(checks):
        return None, None, [
            f'{checks} is not a directory. The pinned riscv-formal clone is what '
            f'makes "upstream has no interrupt model" a measurement rather than a '
            f'claim; run `make -C formal riscv-formal` first.']
    mentions = set()
    csr_hits = []
    errors = []
    for name in sorted(os.listdir(checks)):
        if not name.endswith(SEARCHED_SUFFIXES):
            continue
        path = os.path.join(checks, name)
        if not os.path.isfile(path):
            continue
        try:
            text = open(path).read()
        except OSError as e:
            errors.append(f'cannot read {path}: {e}')
            continue
        if INTR_SIGNAL in text:
            mentions.add('checks/' + name)
        if CHECK_FILE.match(name):
            for csr, pattern in CSR_RE.items():
                if pattern.search(text):
                    csr_hits.append((f'checks/{name}', csr))
    return mentions, csr_hits, errors


def parse_baseline(path):
    harnesses, upstream, errors = set(), set(), []
    try:
        lines = open(path).read().splitlines()
    except OSError as e:
        return harnesses, upstream, [f'cannot read {path}: {e}']
    for i, raw in enumerate(lines, 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 2 or fields[0] not in ('HARNESS', 'UPSTREAM'):
            errors.append(
                f'{path}:{i}: expected `HARNESS <path>` or `UPSTREAM <path>`, '
                f'got {line!r}')
            continue
        target = harnesses if fields[0] == 'HARNESS' else upstream
        if fields[1] in target:
            errors.append(f'{path}:{i}: duplicate entry {fields[1]}')
        target.add(fields[1])
    return harnesses, upstream, errors


def main():
    if len(sys.argv) != 4:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    formal_dir, baseline_path, rf_dir = sys.argv[1:4]

    declared_harnesses, declared_upstream, errors = parse_baseline(baseline_path)
    found, harness_errors = scan_harnesses(formal_dir)
    errors += harness_errors

    for name in sorted(set(found) - declared_harnesses):
        errors.append(
            f'{formal_dir}/{name} instantiates littlecpu and {baseline_path} '
            f'does not name it.\n'
            f'  Every riscv-formal harness runs with the interrupt tied off. Add\n'
            f'  a HARNESS line for it, or say in the pull request why this one is\n'
            f'  different.')
    for name in sorted(declared_harnesses - set(found)):
        errors.append(
            f'{baseline_path} names HARNESS {name}, which does not instantiate '
            f'littlecpu (or does not exist).\n'
            f'  Either the harness was removed and the line was not, or the\n'
            f'  instantiation was reshaped and this script can no longer see it.')
    for name in sorted(declared_harnesses & set(found)):
        if not found[name]:
            errors.append(
                f"{formal_dir}/{name} does not connect .irq_timer(1'b0).\n"
                f'  The baseline says the generated checks run with no interrupt\n'
                f'  in the trace, and the depths in formal/checks.cfg are derived\n'
                f'  under that. A free input there is a different machine, checked\n'
                f'  against a spec that does not describe it.')

    upstream, csr_hits, upstream_errors = scan_upstream(rf_dir)
    errors += upstream_errors
    if upstream is not None:
        for name in sorted(upstream - declared_upstream):
            errors.append(
                f'{name} mentions {INTR_SIGNAL} at the pin and {baseline_path} '
                f'does not name it.\n'
                f'  Upstream may now have something to say about an interrupt.\n'
                f'  Read the check and rule on it: if it constrains what a core\n'
                f'  does on entry, the tie-off is hiding a real check and has to\n'
                f'  come out.')
        for name in sorted(declared_upstream - upstream):
            errors.append(
                f'{baseline_path} names UPSTREAM {name}, which does not mention '
                f'{INTR_SIGNAL} at the pin.\n'
                f'  The pin moved and this set was not re-derived.')
        for name, csr in csr_hits:
            errors.append(
                f'{name} names the CSR {csr} at the pin, so riscv-formal now has '
                f'a model of interrupt state.\n'
                f'  formal/traps.sv was written because nothing upstream did. Read\n'
                f'  the new check before deciding which of the two is the oracle.')

    if errors:
        print('INTERRUPT TIE-OFF: FAIL', file=sys.stderr)
        for e in errors:
            print('  ' + e.replace('\n', '\n  '), file=sys.stderr)
        return 1

    print(f'interrupt tie-off matches {baseline_path} (both directions):')
    for name in sorted(declared_harnesses):
        print(f"  {name:<16} instantiates littlecpu with .irq_timer(1'b0)")
    print(f'  {len(declared_upstream)} files at the pin mention {INTR_SIGNAL}, '
          f'and no rvfi_*_check.sv names mie, mip or mstatus')
    print('INTERRUPT TIE-OFF: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
