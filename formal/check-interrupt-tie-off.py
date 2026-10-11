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

nano runs the same check over its own harnesses: `--core nano` swaps the module name and
the tied ports (`riscv`, `.irq_meip(1'b0)` and `.irq_mtip(1'b0)`) and adds two things nano needs. A harness that
instantiates the core and leaves the input free on purpose, nano/formal/traps.sv, is a
`FREE` record: it must instantiate the core and must NOT tie the input off, so the one file
that is meant to see an interrupt is graded as well as the ones that are meant not to. And
every input a harness connects to a constant must be one the baseline declares: an input
that all the `HARNESS` files hold constant, other than the interrupt, is a restriction
nothing recorded (the sweep formal/check-multihart-tie-off.py makes for littlecpu's
multi-hart surface).

Usage: check-interrupt-tie-off.py [--core littlecpu|nano] <formal dir> <INTERRUPT_TIE_OFF>
                                  <riscv-formal dir>
"""

import os
import re
import sys

# Per core: the module a harness instantiates and the inputs it ties low.
CORES = {
    'littlecpu': ('littlecpu', ('irq_timer',)),
    'nano': ('riscv', ('irq_meip', 'irq_mtip')),
}

# Inputs a harness may hold constant besides the interrupt, each a restriction recorded here:
# nano's `mtime_wr` is the bus raising a store to `mtime`, and no harness has a bus, so the
# store never arrives and `mcycle` ticks and takes CSR writes only.
ALLOWED_CONSTANTS = {
    'littlecpu': (),
    'nano': ('mtime_wr',),
}

# A port connected to a constant, whatever the port.
CONST_PORT = re.compile(r"^\s*\.(\w+)\(\s*\d*'[bBhHdD][01xXzZ]+\s*\)\s*,?\s*$", re.M)

INTR_SIGNAL = 'rvfi_intr'

# Bounded by anything that is not a letter or a digit, so `rvfi_csr_mstatus_wdata` counts
# -- an underscore-separated component of an identifier is the shape a CSR name actually
# takes upstream -- while `premier` does not.
CSR_NAMES = ('mie', 'mip', 'mstatus')
CSR_RE = {csr: re.compile(rf'(?<![A-Za-z0-9]){csr}(?![A-Za-z0-9])')
          for csr in CSR_NAMES}

SEARCHED_SUFFIXES = ('.v', '.sv', '.vh')

# The files upstream that carry assertions, as opposed to port declarations.
CHECK_FILE = re.compile(r'^rvfi_\w+_check\.sv$')

def scan_harnesses(formal_dir, module, ports):
    """Files in the harness directory that instantiate the core: which of the interrupt
    inputs each ties off, and which ports each connects to a constant."""
    instantiates = re.compile(rf'^\s*{module}\s+\w+\s*\(\s*$', re.M)
    tie_offs = {port: re.compile(rf"^\s*\.{port}\(1'b0\)\s*,?\s*$", re.M) for port in ports}
    found = {}
    constants = {}
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
        match = instantiates.search(text)
        if match:
            found[name] = {port: bool(tie_off.search(text)) for port, tie_off in tie_offs.items()}
            constants[name] = set(CONST_PORT.findall(text[match.end():text.find(');', match.end())]))
    return found, constants, errors

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
    harnesses, upstream, free, errors = set(), set(), set(), []
    try:
        lines = open(path).read().splitlines()
    except OSError as e:
        return harnesses, upstream, free, [f'cannot read {path}: {e}']
    for i, raw in enumerate(lines, 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 2 or fields[0] not in ('HARNESS', 'UPSTREAM', 'FREE'):
            errors.append(
                f'{path}:{i}: expected `HARNESS <path>`, `UPSTREAM <path>` or '
                f'`FREE <path>`, got {line!r}')
            continue
        target = {'HARNESS': harnesses, 'UPSTREAM': upstream, 'FREE': free}[fields[0]]
        if fields[1] in target:
            errors.append(f'{path}:{i}: duplicate entry {fields[1]}')
        target.add(fields[1])
    return harnesses, upstream, free, errors

def main():
    args = sys.argv[1:]
    core = 'littlecpu'
    if args[:1] == ['--core'] and len(args) > 1:
        core, args = args[1], args[2:]
    if len(args) != 3 or core not in CORES:
        print('usage: check-interrupt-tie-off.py [--core {}] <formal dir> '
              '<INTERRUPT_TIE_OFF> <riscv-formal dir>'.format('|'.join(CORES)),
              file=sys.stderr)
        return 2
    formal_dir, baseline_path, rf_dir = args
    module, ports = CORES[core]

    declared_harnesses, declared_upstream, declared_free, errors = parse_baseline(baseline_path)
    found, constants, harness_errors = scan_harnesses(formal_dir, module, ports)
    errors += harness_errors

    for name in sorted(set(found) - declared_harnesses - declared_free):
        errors.append(
            f'{formal_dir}/{name} instantiates {module} and {baseline_path} '
            f'does not name it.\n'
            f'  Every riscv-formal harness runs with the interrupt tied off. Add\n'
            f'  a HARNESS line for it, or say in the pull request why this one is\n'
            f'  different.')
    for name in sorted(declared_harnesses - set(found)):
        errors.append(
            f'{baseline_path} names HARNESS {name}, which does not instantiate '
            f'{module} (or does not exist).\n'
            f'  Either the harness was removed and the line was not, or the\n'
            f'  instantiation was reshaped and this script can no longer see it.')
    for name in sorted(declared_harnesses & set(found)):
        for port in ports:
            if found[name][port]:
                continue
            errors.append(
                f"{formal_dir}/{name} does not connect .{port}(1'b0).\n"
                f'  The baseline says the generated checks run with no interrupt\n'
                f'  in the trace, and the depths in formal/checks.cfg are derived\n'
                f'  under that. A free input there is a different machine, checked\n'
                f'  against a spec that does not describe it.')

    for name in sorted(declared_free - set(found)):
        errors.append(
            f'{baseline_path} names FREE {name}, which does not instantiate {module} '
            f'(or does not exist).')
    for name in sorted(declared_free & set(found)):
        for port in ports:
            if not found[name][port]:
                continue
            errors.append(
                f"{formal_dir}/{name} ties .{port}(1'b0) and {baseline_path} names it "
                f'FREE.\n'
                f'  The one harness that leaves the input free is the only oracle for\n'
                f'  an interrupt; tied low, nothing grades interrupt entry.')
    for name in sorted(declared_harnesses & declared_free):
        errors.append(f'{name} is named both HARNESS and FREE in {baseline_path}.')
    if core == 'nano':
        # Every constant a HARNESS file connects, other than the interrupt, is a
        # restriction on the checks that nothing has recorded.
        for name in sorted(declared_harnesses & set(found)):
            for other in sorted(constants[name] - set(ports) - set(ALLOWED_CONSTANTS[core])):
                errors.append(
                    f'{formal_dir}/{name} ties .{other} to a constant, and nothing in '
                    f'{baseline_path} records that restriction.')

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
        print(f"  {name:<16} instantiates {module} with "
              + ', '.join(f".{port}(1'b0)" for port in ports))
    for name in sorted(declared_free):
        print(f"  {name:<16} instantiates {module} with "
              + ', '.join(f'.{port}' for port in ports) + ' left free')
    print(f'  {len(declared_upstream)} files at the pin mention {INTR_SIGNAL}, '
          f'and no rvfi_*_check.sv names mie, mip or mstatus')
    print('INTERRUPT TIE-OFF: PASS')
    return 0

if __name__ == '__main__':
    sys.exit(main())
