#!/usr/bin/env python3
"""Fail if the shared-bus tie-off has drifted from formal/MULTIHART_TIE_OFF.

Every riscv-formal harness in formal/ instantiates the core with its bus-wait
and snoop inputs tied to constants, so the generated checks describe one hart on
a bus nobody else uses. That is a RESTRICTION on what those checks cover, and
the rule this repo works to -- restrict the proof, and record the restriction --
is what makes it admissible. A recorded restriction nothing compares against is
not recorded, which is what this script is for.

Four things are decided here:

  1. The set of files in formal/ that instantiate `littlecpu` and the HARNESS
     set in the baseline must match EXACTLY, in both directions. A harness
     added without a line is red; a line with no harness is red too.
  2. Every declared PORT must be an input of rtl/littlecpu.v. This is the
     re-derivation a baseline alone cannot give: a port renamed or deleted
     leaves the line behind describing a tie-off of nothing.
  3. Every declared harness must connect every declared PORT to exactly the
     declared constant. A declared-but-untied harness would under-report the
     restriction, which is the direction that matters -- the checks would be
     running against a machine the baseline says they are not.
  4. Any OTHER input of littlecpu that every harness ties to a constant must be
     declared, here or in another baseline named by an ELSEWHERE line. This is
     the direction that rots: the next tied-off port arrives with the depths
     derived under it and nothing to say so. `irq_timer` is the ELSEWHERE case
     and formal/INTERRUPT_TIE_OFF is where its argument lives.

The Verilog is parsed by test/port_connect_test.py, not by a second parser here:
that file already reads littlecpu's port list and every instantiation's
connections, guard by guard, and two parsers of one language drift apart in
exactly the cases a regex was chosen for.

Usage: check-multihart-tie-off.py <formal dir> <MULTIHART_TIE_OFF> <repo root>
"""

import os
import re
import sys

SEARCHED_SUFFIXES = ('.v', '.sv')

# What counts as tied off: a literal, not a wire that happens to be low. The
# second reads the same to a human and means something no reader can check
# locally. Sized (32'b0), unsized (0) and based (8'hff) literals all match.
CONSTANT = re.compile(r"^(\d+\s*'\s*[sS]?[bodhBODH][0-9a-fA-FxXzZ_]+|\d+)$")


def load_parser(repo):
    """test/port_connect_test.py, imported as the one Verilog parser here."""
    path = os.path.join(repo, 'test')
    if not os.path.isfile(os.path.join(path, 'port_connect_test.py')):
        print(f'error: {path}/port_connect_test.py is missing, and this check\n'
              f'reads littlecpu\'s port list and every connection through it\n'
              f'rather than through a second parser. If it moved, move this\n'
              f'check with it.', file=sys.stderr)
        raise SystemExit(2)
    sys.path.insert(0, path)
    import port_connect_test
    return port_connect_test


def parse_baseline(path):
    harnesses, ports, elsewhere, errors = set(), {}, {}, []
    try:
        lines = open(path).read().splitlines()
    except OSError as e:
        return harnesses, ports, elsewhere, [f'cannot read {path}: {e}']
    for i, raw in enumerate(lines, 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if fields[0] == 'HARNESS' and len(fields) == 2:
            if fields[1] in harnesses:
                errors.append(f'{path}:{i}: duplicate entry {fields[1]}')
            harnesses.add(fields[1])
        elif fields[0] in ('PORT', 'ELSEWHERE') and len(fields) == 3:
            target = ports if fields[0] == 'PORT' else elsewhere
            if fields[1] in ports or fields[1] in elsewhere:
                errors.append(f'{path}:{i}: duplicate entry {fields[1]}')
            target[fields[1]] = fields[2]
        else:
            errors.append(
                f'{path}:{i}: expected `HARNESS <path>`, `PORT <name> <value>` '
                f'or `ELSEWHERE <name> <baseline>`, got {line!r}')
    return harnesses, ports, elsewhere, errors


def scan_harnesses(pct, formal_dir):
    """Files in formal/ instantiating littlecpu, each as {port: expression}."""
    found = {}
    errors = []
    for name in sorted(os.listdir(formal_dir)):
        if not name.endswith(SEARCHED_SUFFIXES):
            continue
        path = os.path.join(formal_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            text = pct.strip_comments_and_strings(open(path).read())
        except OSError as e:
            errors.append(f'cannot read {path}: {e}')
            continue
        for _instance, conns in pct.instantiations(path, text):
            found[name] = {port: ' '.join(expr.split())
                           for port, _macro, _lineno, expr, _guard in conns
                           if port is not None}
    return found, errors


def main():
    if len(sys.argv) != 4:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    formal_dir, baseline_path, repo = sys.argv[1:4]

    pct = load_parser(repo)
    module_file = os.path.join(repo, pct.MODULE_FILE)
    try:
        ports = pct.module_ports(
            pct.strip_comments_and_strings(open(module_file).read()))
    except OSError as e:
        print(f'error: cannot read {module_file}: {e}', file=sys.stderr)
        return 2
    inputs = {p.name for p in ports if p.direction == 'input'}

    declared_harnesses, declared_ports, elsewhere, errors = \
        parse_baseline(baseline_path)
    found, harness_errors = scan_harnesses(pct, formal_dir)
    errors += harness_errors

    for name in sorted(set(found) - declared_harnesses):
        errors.append(
            f'{formal_dir}/{name} instantiates {pct.MODULE} and {baseline_path} '
            f'does not name it.\n'
            f'  Every riscv-formal harness runs with the shared-bus surface tied\n'
            f'  off. Add a HARNESS line for it, or say in the pull request why\n'
            f'  this one is different.')
    for name in sorted(declared_harnesses - set(found)):
        errors.append(
            f'{baseline_path} names HARNESS {name}, which does not instantiate '
            f'{pct.MODULE} (or does not exist).\n'
            f'  Either the harness was removed and the line was not, or the\n'
            f'  instantiation was reshaped and this script can no longer see it.')

    for port in sorted(set(declared_ports) | set(elsewhere)):
        if port not in inputs:
            errors.append(
                f'{baseline_path} names the port {port}, which is not an input '
                f'of {pct.MODULE}.\n'
                f'  The port was renamed or removed and this baseline was not\n'
                f'  re-derived, so it now declares a tie-off of nothing.')

    for name in sorted(declared_harnesses & set(found)):
        for port, value in sorted(declared_ports.items()):
            if port not in inputs:
                continue
            got = found[name].get(port)
            if got != value:
                how = (f'connects .{port}({got})' if got is not None
                       else f'does not connect .{port} at all')
                errors.append(
                    f'{formal_dir}/{name} {how} and '
                    f'{baseline_path} declares {value}.\n'
                    f'  The baseline says the generated checks describe one hart\n'
                    f'  on a bus nobody else uses, and the depths in\n'
                    f'  formal/checks.cfg are derived under that. Anything else\n'
                    f'  there is a different machine, checked against a spec that\n'
                    f'  does not describe it.')

    # The sweep for a tie-off nobody wrote down. An input every harness holds at
    # a constant is a restriction on all of them, whatever it is called.
    if declared_harnesses and declared_harnesses <= set(found):
        for port in sorted(inputs - set(declared_ports) - set(elsewhere)):
            values = {found[name].get(port) for name in declared_harnesses}
            everywhere = values.pop() if len(values) == 1 else None
            if everywhere and CONSTANT.match(everywhere):
                errors.append(
                    f'every harness ties .{port} to one constant and no baseline '
                    f'says so.\n'
                    f'  That is a restriction on every generated check and on the\n'
                    f'  depths derived from them. Declare it with a PORT line, or\n'
                    f'  with an ELSEWHERE line if another baseline carries the\n'
                    f'  argument for it.')

    if errors:
        print('MULTI-HART TIE-OFF: FAIL', file=sys.stderr)
        for e in errors:
            print('  ' + e.replace('\n', '\n  '), file=sys.stderr)
        return 1

    tied = ', '.join(f'{p}={v}' for p, v in sorted(declared_ports.items()))
    print(f'shared-bus tie-off matches {baseline_path} (both directions):')
    for name in sorted(declared_harnesses):
        print(f'  {name:<16} instantiates {pct.MODULE} with {tied}')
    for port, where in sorted(elsewhere.items()):
        print(f'  {port} is held at a constant by all of them too, and {where} '
              f'carries it')
    print(f'  no other input of {pct.MODULE} is held at a constant by all '
          f'{len(declared_harnesses)}')
    print('MULTI-HART TIE-OFF: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
