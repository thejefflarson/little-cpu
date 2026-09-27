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
equivalence." This script is the same shape, built on the same instrument
that check does: yosys's own JSON netlist (`write_json`), not a
regex-and-AST reading of the source.

RE-DERIVED AT THE NEW SITE. The D/X split (ADR-0208) moved every signal this
script used to grade off `rtl/decoder.v` and onto `rtl/executor.v`: D's own
nine stall reasons no longer read a single bit of register-file DATA (only
register NUMBERS and the single `x_busy` bit X hands back), so the whole
argument is vacuous there now. X owns `region_stall` and, new since the
split, the divider's own `divider_busy` -- both legitimately depend on an
operand's VALUE (a load/store address, a divide's magnitude), and both are
Zkt's own named exclusions (region_stall for loads/stores, divider_busy for
DIV/REM). X's only TIMING output is `x_busy`, so the claim this script now
proves is: reg_rs1/reg_rs2 reach `x_busy` only through `region_stall` or
`divider_busy`, never any other path -- which is what would let a
Zkt-listed instruction's cycle count depend on an operand's value. The
`mul`-family constant-latency half of Zkt's claim is not 2-safety and is
proved separately, single-trace, by `rtl/executor.v`'s own `FORMAL` block
(`state == init` the cycle after a mul launches) and `make -C formal
components_executor`.

WHY `region_stall`'S AND `ls_access`'S OWN GATES ARE NOT DECIDED HERE. Both
are single-trace, exact-set-equality invariants with no VALUE comparison at
all (`assert(!region_stall || ls_access)`,
`assert(ls_access == (in_is_lb || ...))`), the same shape as
`assert(is_amo == (is_amoswap || ...))`. `rtl/executor.v` states both as
assertions, `make -C formal components_executor` proves them by
k-induction, and `make -C formal decoder-zkt-probe` demonstrates both fail
at their own assertion for the mutations this script used to catch
structurally. This script keeps only the half of the argument that is
genuinely 2-safety and has no exact decision procedure to hand a solver
instead.

Two checks, against the netlist `yosys -q -s` writes for `rtl/executor.v`
(plus `rtl/structs.v`), never against the source text:

  1. FORWARD REACHABILITY, twice. Starting from the bits of `reg_rs1` and
     `reg_rs2`, follow every cell's inputs to its outputs -- a flip-flop's D
     to its Q included, so a value laundered through a register (the
     divider's own magnitude counter included) is not read as clean -- with
     `region_stall` and `divider_busy` both blocked from being used as a
     source for anything past their own one hop. `x_busy` must not be
     reached this way. Then, separately, starting from
     `ls_capture`/`ls_answer`/`ls_answer_valid`'s own bits with the same two
     blocked, `x_busy` must not be reachable either -- their own
     correctness (a captured region answer is about the access still held
     in X) does not extend to being read by an unrelated instruction's
     timing.
  2. PORT COVERAGE, BOTH WAYS. Every input port of `executor` wider than 5
     bits (5 is the widest a register NUMBER gets) must be classified
     SEED_PORTS/STRUCT_FIELD_SEEDS (can carry a register-file or CSR-file
     DATA output) or NON_VALUE_PORTS/STRUCT_FIELD_NON_VALUE (cannot, with
     the reason why) -- read off the netlist's own measured port width, not
     a `[N:0]` match against source text. A struct-typed port's fields are
     found the same way: a small satellite module, elaborated against
     rtl/structs.v alone, asks yosys what `val.<fieldname>` resolves to.
     Classifications are checked stale in both directions.

Usage: zkt_isolation_test.py [executor.v]     # defaults to rtl/executor.v
"""

import collections
import json
import os
import subprocess
import sys
import tempfile

# X's only TIMING output: how long an instruction takes to commit.
STALL_TARGETS = ['x_busy']

# The two reasons allowed to depend on a register-file DATA output: a load/store's
# region wait, and the divider's own operand-magnitude-dependent iteration count.
# Zkt's own exclusion list names both (loads/stores, DIV/REM).
GATED_SIGNALS = ['region_stall', 'divider_busy']

# Registers derived from region_stall that hold a load's or store's own region answer
# across the cycle it is read on.
REGION_STATE = ['ls_capture', 'ls_answer', 'ls_answer_valid']

# The positive control: named nets the real RTL is known to carry reg_rs1/reg_rs2
# through on the way to each gated signal.
EXPECT_TAINTED = ['ls_block', 'ls_text_deep', 'ls_ram_deep', 'ls_settled',
                   'region_stall', 'div_skip', 'divider_busy']

# Plain (non-struct) executor INPUT ports wide enough (>5 bits) to carry a register-file
# or CSR-file DATA output, and the ones wide enough that provably cannot.
SEED_PORTS = {'reg_rs1', 'reg_rs2'}
NON_VALUE_PORTS = {
    # csr_rdata: the CSR file's read value -- csrrw/rs/rc are not on Zkt's list.
    'csr_rdata',
    # mtvec, mepc: trap CSRs, read only on the trap and mret arms of redirect_target,
    # neither of which is on Zkt's list either.
    'mtvec', 'mepc',
}

# Struct-typed executor INPUT ports: the typedef name (declared in rtl/structs.v) and
# every field it declares, in order, so the total-width check below catches an added,
# removed or resized field rather than silently misreading one.
STRUCT_PORTS = {
    'in': ('dx_output', [
        'valid', 'is_interrupt', 'imem_fault', 'pc', 'instr', 'immediate',
        'rd', 'rs1', 'rs2',
        'is_add', 'is_sub', 'is_xor', 'is_or', 'is_and',
        'is_mul', 'is_mulh', 'is_mulhu', 'is_mulhsu',
        'is_div', 'is_divu', 'is_rem', 'is_remu',
        'is_sll', 'is_slt', 'is_sltu', 'is_srl', 'is_sra',
        'is_lb', 'is_lbu', 'is_lhu', 'is_lh', 'is_lw',
        'is_sb', 'is_sh', 'is_sw',
        'is_amoswap', 'is_amoadd', 'is_amoxor', 'is_amoand', 'is_amoor',
        'is_amomin', 'is_amomax', 'is_amominu', 'is_amomaxu',
        'is_lr', 'is_sc',
        'is_auipc', 'is_lui', 'is_jal', 'is_jalr',
        'is_beq', 'is_bne', 'is_blt', 'is_bltu', 'is_bge', 'is_bgeu',
        'is_ecall', 'is_ebreak', 'is_mret', 'is_wfi', 'is_fence', 'is_fencei',
        'is_csrrw', 'is_csrrs', 'is_csrrc', 'is_csr_imm', 'is_csr_access',
        'is_math_imm',
        'fwd_rs1', 'fwd_rs2',
    ]),
}

# Struct fields (as `port.field`) wide enough (>5 bits) to matter, and their
# classification. `in.rd`/`in.rs1`/`in.rs2` are register NUMBERS, not values, and are
# `[4:0]` -- below the 5-bit threshold -- so they need no entry here at all.
STRUCT_FIELD_SEEDS = set()
STRUCT_FIELD_NON_VALUE = {'in.pc', 'in.instr', 'in.immediate'}

# X has no analogue of D's `out`/`executor_out` feedback (a wide struct field read back
# into the module's own stall computation): `in`'s own register-NUMBER fields are
# already `[4:0]` and skip classification on width alone. Empty on purpose, not
# unfinished -- kept as a table, not deleted, so a future feedback path has somewhere
# to be declared rather than silently falling through classify_inputs' catch-all error.
CONTROL_FIELDS = {}

def run_yosys(script_path):
    """Run `yosys -q -s script_path`. A yosys that fails to elaborate fails
    this gate outright (exit 2) rather than leaving a stale or absent JSON
    for the rest of the script to trip over with a confusing message."""
    proc = subprocess.run(['yosys', '-q', '-s', script_path],
                           capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        return False
    for line in (proc.stdout + proc.stderr).splitlines():
        if line.startswith('Warning:'):
            print('  yosys: ' + line)
    return True

def build_executor_netlist(executor_path, structs_path, out_dir):
    """Elaborate rtl/executor.v (plus rtl/structs.v) to a JSON netlist and
    return its `executor` module. `proc; flatten` resolves every generate
    arm, `` `ifdef ``, procedural block and struct field access into
    ordinary cells and named nets before this script reads any of it;
    `memory_map; simplemap` is the same struct-breaking step
    formal/check-nonperturbation.py applies, for the same reason -- a packed
    struct is one wide cell until simplemap breaks it apart into the
    primitives build_graph and forward_taint know how to read. Deliberately
    NO `opt_clean` here, unlike that check: a pure bit-select such as
    `ls_block = reg_rs1[31:21]` has no cell of its own, so a fanout sweep
    folds its name away in favour of reg_rs1's, and this script needs every
    intermediate's OWN name to walk the graph one text-level term at a
    time. No port is deleted here either -- this reads named INTERNAL wires
    (`x_busy`, `region_stall`, ...), not a diff of two builds, so there is
    nothing for a sweep to clean up."""
    rtl_dir = os.path.dirname(os.path.abspath(executor_path))
    srcs = ' '.join([structs_path, executor_path])
    json_path = os.path.join(out_dir, 'executor.json')
    script = '\n'.join([
        'design -reset',
        'read_verilog -sv -I %s %s' % (rtl_dir, srcs),
        'hierarchy -check -top executor',
        'proc',
        'flatten',
        'memory_map',
        'simplemap',
        'write_json %s' % json_path,
    ])
    script_path = os.path.join(out_dir, 'build.ys')
    with open(script_path, 'w') as f:
        f.write(script + '\n')
    if not run_yosys(script_path):
        return None
    with open(json_path) as f:
        design = json.load(f)
    mod = design.get('modules', {}).get('executor')
    if mod is None:
        print('error: the elaborated netlist has no `executor` module',
              file=sys.stderr)
        return None
    return mod

def probe_struct_fields(structs_path, typedef, fields, out_dir):
    """{field: (offset, width)} within the struct's own bit vector, resolved
    by elaborating a tiny satellite module -- `input <typedef> val` plus one
    output per field, sized with `$bits(val.<field>)` rather than a width
    this script supplies -- against rtl/structs.v alone. yosys connects each
    output straight to the bits of `val` it names with no logic in between,
    so the offset is read off where those bits land in `val`'s own list, and
    a field that no longer exists under that name fails elaboration outright
    rather than silently resolving to the wrong bits."""
    mod_name = '__zkt_field_probe_%s' % typedef
    lines = ['`default_nettype none', 'module %s (' % mod_name,
             '  input %s val' % typedef, ');']
    for i, field in enumerate(fields):
        lines.append('  localparam int W%d = $bits(val.%s);' % (i, field))
        lines.append('  logic [W%d-1:0] field%d;' % (i, i))
        lines.append('  assign field%d = val.%s;' % (i, field))
    lines.append('endmodule')
    probe_path = os.path.join(out_dir, 'probe_%s.v' % typedef)
    with open(probe_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    json_path = os.path.join(out_dir, 'probe_%s.json' % typedef)
    script = '\n'.join([
        'design -reset',
        'read_verilog -sv %s %s' % (structs_path, probe_path),
        'hierarchy -check -top %s' % mod_name,
        'proc',
        'write_json %s' % json_path,
    ])
    script_path = os.path.join(out_dir, 'probe_%s.ys' % typedef)
    with open(script_path, 'w') as f:
        f.write(script + '\n')
    if not run_yosys(script_path):
        return None, ('could not elaborate rtl/structs.v\'s `%s` to probe '
                       'its fields %s -- a field was likely renamed. Update '
                       'STRUCT_PORTS to match.' % (typedef, fields))
    with open(json_path) as f:
        design = json.load(f)
    pmod = design['modules'][mod_name]
    val_bits = pmod['netnames']['val']['bits']
    offsets = {}
    for i, field in enumerate(fields):
        field_bits = pmod['netnames']['field%d' % i]['bits']
        if not field_bits:
            return None, '`%s.%s` probed as zero bits wide' % (typedef, field)
        try:
            start = val_bits.index(field_bits[0])
        except ValueError:
            return None, ('could not locate `%s.%s` within `%s`\'s own bits '
                           'on the elaborated netlist' % (typedef, field, typedef))
        offsets[field] = (start, len(field_bits))
    return offsets, None

def get_field_offsets(cache, structs_path, typedef, fields, out_dir):
    """probe_struct_fields, memoized per typedef for the run. A request for
    a field the cache does not yet have re-probes the UNION of old and new
    fields, so caching never returns a stale answer to a wider ask."""
    cached = cache.get(typedef)
    if cached is not None and set(fields) <= set(cached):
        return cached, None
    union_fields = sorted(set(fields) | set(cached or {}))
    offsets, err = probe_struct_fields(structs_path, typedef, union_fields,
                                        out_dir)
    if err:
        return None, err
    cache[typedef] = offsets
    return offsets, None

def classify_inputs(mod, structs_path, field_cache, out_dir):
    """{seed net bit ids}, or a list of errors. Every executor INPUT port
    wider than 5 bits (5 is the widest a register NUMBER gets) must be
    classified SEED or NON_VALUE, by name for a plain port and by
    `port.field` for a struct-typed one -- and every classification must
    correspond to something the netlist still has, in both directions."""
    seeds = set()
    errors = []
    ports = mod['ports']
    input_names = {name for name, data in ports.items()
                   if data['direction'] == 'input'}
    field_offsets = {}

    for name in sorted(input_names):
        data = ports[name]
        bits = data['bits']
        width = len(bits)
        if name in STRUCT_PORTS:
            typedef, fields = STRUCT_PORTS[name]
            offsets, err = get_field_offsets(field_cache, structs_path,
                                              typedef, fields, out_dir)
            if err:
                errors.append(err)
                continue
            field_offsets[name] = offsets
            total = sum(offsets[f][1] for f in fields)
            if total != width:
                errors.append(
                    'rtl/structs.v\'s `%s` totals %d bits across %s, but '
                    'executor\'s `%s` port measures %d bits on the elaborated '
                    'netlist -- a field was added, removed or resized '
                    'without updating STRUCT_PORTS.'
                    % (typedef, total, fields, name, width))
                continue
            for field in fields:
                start, fwidth = offsets[field]
                if fwidth <= 5:
                    continue
                dotted = '%s.%s' % (name, field)
                field_bits = bits[start:start + fwidth]
                if dotted in STRUCT_FIELD_SEEDS:
                    seeds.update(b for b in field_bits if isinstance(b, int))
                elif dotted in STRUCT_FIELD_NON_VALUE:
                    pass
                else:
                    errors.append(
                        '`%s` is %d bits wide on the elaborated netlist '
                        'with no Zkt classification. Add it to '
                        'STRUCT_FIELD_SEEDS if it can carry a register-file '
                        'or CSR-file DATA output, or to '
                        'STRUCT_FIELD_NON_VALUE with the reason it cannot.'
                        % (dotted, fwidth))
        else:
            if width <= 5:
                continue
            if name in SEED_PORTS:
                seeds.update(b for b in bits if isinstance(b, int))
            elif name in NON_VALUE_PORTS:
                pass
            else:
                errors.append(
                    '`%s` is %d bits wide on the elaborated netlist with no '
                    'Zkt classification. Add it to SEED_PORTS if it can '
                    'carry a register-file or CSR-file DATA output, or to '
                    'NON_VALUE_PORTS with the reason it cannot.'
                    % (name, width))

    for name in sorted(SEED_PORTS | NON_VALUE_PORTS):
        if name not in input_names or len(ports[name]['bits']) <= 5:
            errors.append(
                '`%s` is classified as a Zkt-relevant input but the '
                'elaborated netlist has no such input wider than 5 bits. '
                'Remove the stale entry from SEED_PORTS or NON_VALUE_PORTS.'
                % name)

    for name in sorted(STRUCT_PORTS):
        if name not in input_names:
            errors.append(
                '`%s` is declared in STRUCT_PORTS but the elaborated '
                'netlist has no such input port. Remove the stale entry.'
                % name)

    for dotted in sorted(STRUCT_FIELD_SEEDS | STRUCT_FIELD_NON_VALUE):
        port_name, _, field_name = dotted.partition('.')
        declared_fields = STRUCT_PORTS.get(port_name, (None, []))[1]
        if field_name not in declared_fields:
            errors.append(
                '`%s` is classified as a Zkt-relevant field but is not '
                'among the fields STRUCT_PORTS declares for `%s`. Remove '
                'the stale entry from STRUCT_FIELD_SEEDS or '
                'STRUCT_FIELD_NON_VALUE.' % (dotted, port_name))
            continue
        offsets = field_offsets.get(port_name)
        if offsets is not None and offsets.get(field_name, (0, 0))[1] <= 5:
            errors.append(
                '`%s` is classified as a Zkt-relevant field but the '
                'elaborated netlist measures it at %d bits, not wide enough '
                'to matter. Remove the stale entry.'
                % (dotted, offsets[field_name][1]))

    return seeds, errors

def control_field_bits(mod, structs_path, field_cache, out_dir):
    """Bit ids for CONTROL_FIELDS' register-NUMBER/control fields, blocked
    as taint sources below the same way the gated signals are. Empty for
    executor.v today (see CONTROL_FIELDS), so this always returns an empty
    set with no errors -- kept as a function, not inlined, so a future
    feedback field is one dict entry away rather than a re-plumb."""
    bits = set()
    errors = []
    all_bits = {name: data['bits'] for name, data in mod['netnames'].items()}
    for port_name, (typedef, fields) in CONTROL_FIELDS.items():
        if port_name not in all_bits:
            errors.append(
                '`%s` is declared in CONTROL_FIELDS but the elaborated '
                'netlist has no such signal.' % port_name)
            continue
        offsets, err = get_field_offsets(field_cache, structs_path, typedef,
                                          fields, out_dir)
        if err:
            errors.append(err)
            continue
        whole = all_bits[port_name]
        for field in fields:
            start, width = offsets[field]
            if width > 5:
                errors.append(
                    '`%s.%s` measures %d bits on the elaborated netlist, '
                    'wider than a register NUMBER (5 bits). CONTROL_FIELDS '
                    'blocks it as a taint source on the strength of a width '
                    'argument alone, and a field this wide is exactly what '
                    'that argument no longer covers.'
                    % (port_name, field, width))
                continue
            field_bits = whole[start:start + width]
            bits.update(b for b in field_bits if isinstance(b, int))
    return bits, errors

def cell_io_bits(cell):
    """(input bits, output bits) for one cell, in a single pass over its
    connections -- the two used to be separate functions each re-scanning
    `cell['connections']` with the same `port_directions` test inverted."""
    directions = cell.get('port_directions', {})
    in_bits, out_bits = [], []
    for port, conn in cell['connections'].items():
        target = out_bits if directions.get(port, 'input') == 'output' \
            else in_bits
        target.extend(b for b in conn if isinstance(b, int))
    return in_bits, out_bits

def cell_input_bits(cell):
    return cell_io_bits(cell)[0]

def build_graph(mod):
    """bit -> (driving cell name, cell), and bit -> [(cell name, its own
    output bits)] for every cell that reads `bit` as one of its inputs. Built
    once so forward_taint (from `fanout`) and the undriven-signal check below
    (from `bit_driver`) both walk the same graph without re-deriving it."""
    bit_driver = {}
    fanout = collections.defaultdict(list)
    for cname, cell in mod['cells'].items():
        in_bits, out_bits = cell_io_bits(cell)
        for b in out_bits:
            bit_driver[b] = (cname, cell)
        for b in set(in_bits):
            fanout[b].append((cname, out_bits))
    return bit_driver, fanout

def public_bit_names(mod):
    """bit -> its declared name, for every net a human gave a name (as
    opposed to one of yosys's own auto-generated `$logic_and$...` labels for
    an intermediate it introduced while breaking an expression apart)."""
    names = {}
    for name, data in mod['netnames'].items():
        if data.get('hide_name'):
            continue
        for b in data['bits']:
            if isinstance(b, int):
                names.setdefault(b, name)
    return names

def forward_taint(fanout, seed_bits, blocked_bits=frozenset()):
    """Every bit reachable from `seed_bits` by following cells' inputs to
    their outputs, including through a flip-flop's D to its Q -- so a value
    carried by a register (the divider's own counter included) is reachable
    the same as one computed combinationally this cycle.

    A bit in `blocked_bits` can still be REACHED (added to the returned set)
    but is never used as a SOURCE for tainting anything further downstream of
    it -- the restricted-taint discipline `region_stall` and `divider_busy`
    both need: each is allowed to depend on a register value, and everything
    that reads one of them (rather than reg_rs1/reg_rs2 themselves) is judged
    as if that one read were invisible. Any OTHER path into the same signal
    still counts; blocking removes only the specific paths this design
    intends."""
    reached = set(seed_bits)
    frontier = [b for b in seed_bits if b not in blocked_bits]
    while frontier:
        bit = frontier.pop()
        for cname, out_bits in fanout.get(bit, ()):
            for ob in out_bits:
                if ob not in reached:
                    reached.add(ob)
                    if ob not in blocked_bits:
                        frontier.append(ob)
    return reached

def reachable_targets(reached, name_bits, targets):
    """Which of `targets` has at least one bit in `reached` -- the shared
    shape both reachability checks in main() grade against, so the two only
    differ in what they seeded and blocked, not in how they read the
    result."""
    return [n for n in targets if any(b in reached for b in name_bits[n])]

def main():
    argv = sys.argv[1:]
    if len(argv) > 1:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2

    here = os.path.dirname(os.path.abspath(__file__))
    executor_path = argv[0] if argv else \
        os.path.join(here, os.pardir, 'rtl', 'executor.v')
    if not os.path.isfile(executor_path):
        print('error: cannot read %s' % executor_path, file=sys.stderr)
        return 2

    executor_dir = os.path.dirname(os.path.abspath(executor_path))
    structs_path = os.path.join(executor_dir, 'structs.v')
    if not os.path.isfile(structs_path):
        print('error: cannot read %s' % structs_path, file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix='zkt-isolation.') as out_dir:
        print('Elaborating %s (yosys)...' % executor_path)
        mod = build_executor_netlist(executor_path, structs_path, out_dir)
        if mod is None:
            return 2

        field_cache = {}

        seed_bits, class_errors = classify_inputs(mod, structs_path,
                                                    field_cache, out_dir)
        if class_errors:
            for e in class_errors:
                print('error: %s' % e, file=sys.stderr)
            return 2

        control_bits, control_errors = control_field_bits(mod, structs_path,
                                                            field_cache,
                                                            out_dir)
        if control_errors:
            for e in control_errors:
                print('error: %s' % e, file=sys.stderr)
            return 2

        name_bits = {name: data['bits']
                     for name, data in mod['netnames'].items()}
        needed = (list(STALL_TARGETS) + list(GATED_SIGNALS)
                  + list(REGION_STATE) + list(EXPECT_TAINTED))
        missing = sorted(set(n for n in needed if n not in name_bits))
        if missing:
            print('error: the elaborated netlist has no signal named: %s. '
                  'Either the file renamed one of these, or moved it '
                  'somewhere yosys optimised away entirely.'
                  % ', '.join(missing), file=sys.stderr)
            return 2

        bit_driver, fanout = build_graph(mod)
        bit_names = public_bit_names(mod)

    primary_input_bits = set()
    for pname, pdata in mod['ports'].items():
        if pdata['direction'] == 'input':
            primary_input_bits.update(b for b in pdata['bits']
                                       if isinstance(b, int))
    undriven = sorted(n for n in needed
                       if bit_driver.get(name_bits[n][0]) is None
                       and name_bits[n][0] not in primary_input_bits)
    if undriven:
        print('error: %s has no driving cell on the elaborated netlist -- '
              'it is declared but nothing assigns it, which reachability '
              'through it would read as vacuously safe rather than as the '
              'deleted signal it is.' % ', '.join(undriven),
              file=sys.stderr)
        return 2

    failures = []

    full_reached = forward_taint(fanout, seed_bits)
    vacuous = [n for n in EXPECT_TAINTED
               if not any(b in full_reached for b in name_bits[n])]
    if vacuous:
        failures.append(
            'the netlist graph found no path from reg_rs1/reg_rs2 to %s, '
            'which the real RTL is known to carry a register value through '
            'on the way to region_stall or divider_busy. That means this '
            'run found no edges at all, and every PASS above is a check of '
            'nothing.' % ', '.join(vacuous))

    def report_reachable(reached, template):
        """failures.append(template % name) for every STALL_TARGETS name
        `reached` has a bit in -- the shared shape both reachability checks
        below grade against, so the two only differ in what they seeded and
        the wording of what a hit means, not in how a hit is reported."""
        for name in reachable_targets(reached, name_bits, STALL_TARGETS):
            failures.append(template % name)

    blocked_bits = control_bits
    for name in GATED_SIGNALS:
        blocked_bits |= set(name_bits[name])

    region_reached = forward_taint(fanout, seed_bits,
                                    blocked_bits=blocked_bits)
    report_reachable(
        region_reached,
        '`%s` is reachable, on the elaborated netlist, from a '
        'register-file DATA output (reg_rs1 or reg_rs2) through a path '
        'other than region_stall or divider_busy. A Zkt-listed instruction '
        '(add, xor, sll, mul, ...) can assert this, so its cycle count '
        'would depend on an operand\'s VALUE, not just which registers it '
        'names.')

    region_state_bits = set()
    for name in REGION_STATE:
        region_state_bits.update(name_bits[name])
    state_reached = forward_taint(fanout, region_state_bits,
                                   blocked_bits=blocked_bits)
    report_reachable(
        state_reached,
        '`%s` is reachable, on the elaborated netlist, from %s -- '
        'region_stall\'s own captured answer, held across the cycle '
        'a load or store reads it on. Its correctness does not '
        'extend to being read by an unrelated instruction\'s timing.'
        % ('%s', '/'.join(REGION_STATE)))

    if failures:
        print('ZKT STALL ISOLATION: FAIL', file=sys.stderr)
        for f in failures:
            print('  ' + f, file=sys.stderr)
        return 1

    print('%s: on the elaborated netlist, reg_rs1/reg_rs2 reach x_busy '
          'only through region_stall or divider_busy (each one\'s own gate '
          'is proved separately by make -C formal components_executor).'
          % executor_path)
    print('ZKT STALL ISOLATION: PASS')
    return 0

if __name__ == '__main__':
    sys.exit(main())
