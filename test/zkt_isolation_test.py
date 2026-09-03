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
equivalence." This script is the same shape and, since three rounds of
RTL-text parsing were each defeated by a fact only ELABORATION knows, now
built on the same instrument that check does: yosys's own JSON netlist
(`write_json`), not a regex-and-AST reading of rtl/decoder.v.

WHAT WAS WRONG WITH READING SOURCE TEXT. A stall reason reading
`ls_answer_valid` looked safe because an allowlist said so, and nothing
verified the claim against what `ls_answer_valid` actually depends on. An
`assign` inside an un-taken `generate if (0)` (or a dead `` `ifdef ``) parsed
as a real driver, because SystemVerilog has more than one way to spell a
continuous assignment and a regex cannot tell a live one from a dead one. A
wide input port's bit width came from matching `[N:0]` against a numeric
literal, so a parameterized width read as 1 bit. All three are facts about
ELABORATION -- what a `generate` arm resolves to, what a macro expands to,
how wide a port really is -- and a netlist has already resolved every one of
them before this script ever sees it: there is no leaf to guess at, because
the cell graph says what drives what.

WHAT THIS SCRIPT PROVES, AND WHAT IT DOES NOT. `rtl/decoder.v`'s `stall` is
the OR of eight reasons, and `stall` itself plus `ls_access`, `serialize`,
`hazard_rs1`, `hazard_rs2`, `hazard`, `operand_stall`, `atomic_stall`,
`stall_own` and `stall_other` must never depend, on the elaborated netlist,
on a register-file or CSR-file DATA output (`reg_rs1`, `reg_rs2` or
`executor_out.rd_data`, the three ports of `decoder` wide enough to carry
one) EXCEPT through `region_stall`, the one reason allowed to -- that is what
lets a load or store answer a cycle late instead of faulting. "Except through
region_stall" means what formal/check-nonperturbation.py's own
restricted-taint mechanism means for a signal legitimately downstream of an
`ifdef`'d one: `region_stall` can still become reachable (that is the point
-- `stall` reads it directly, and `out`'s own bubble condition re-lists it
beside `hazard`/`operand_stall`/etc, which is what lets `out.valid`, and
everything built from it such as `pipe_drained` and `serialize`, correctly
bubble on a captured region wait) but is never used as a SOURCE for tainting
anything FURTHER -- so a read of `region_stall` is judged as if it were
invisible to whatever reads it, and any OTHER path from a DATA output into
the same target still counts. Two named registers derived from
`region_stall` -- `ls_capture` and `ls_answer`/`ls_answer_valid`, which hold
a load's or store's own region answer across the cycle it is read on -- get
a second, narrower check on top: none of the nine reasons above may read
THEM directly either, because their own correctness (a captured answer is
about the access still in decode, and nothing else) does not extend to
being read by an unrelated instruction's stall decision, and blocking
`region_stall` as a taint source would hide that if it were checked by
reachability from `reg_rs1`/`reg_rs2` alone. This whole question is
2-safety-shaped -- it is about which VALUE reaches where, not about one
instantaneous fact -- so a structural over-approximation is the right tool
for it, the same one formal/check-nonperturbation.py uses.

WHY `region_stall`'S OWN GATE IS NOT DECIDED HERE ANY MORE. Two further
facts used to be checked the same structural way: that `region_stall` can
only assert alongside `ls_access`, and that `ls_access` is true for exactly
the twelve load and store encodings, none on Zkt's list. Neither is
2-safety -- both are single-trace, exact-set-equality invariants with no
VALUE comparison in them at all, the same shape as
`assert(out.is_amo == (out.is_amoswap || ...))`, which already sits in
`rtl/decoder.v`'s own `ifdef FORMAL` block a few lines away. Three rounds of
deciding them by walking the netlist by hand -- graded `ls_access` by NAME,
then tracked a NOT's polarity, then tracked polarity but not every OTHER
cell type a NOT could sit over -- were three different spellings of the same
defeat: a structural walk is an approximation, and an approximation has
edges an exact decision procedure does not. `rtl/decoder.v` now states both
facts as assertions instead (`assert(!region_stall || ls_access)`,
`assert(ls_access == (instr_lb || instr_lbu || instr_lh || instr_lhu ||
instr_lw || instr_sb || instr_sh || instr_sw))`), `make -C formal
components_decoder` proves them by k-induction, and `make -C formal
decoder-zkt-probe` demonstrates both fail at their own assertion for the
mutations this script used to catch structurally -- the same discipline
`make -C formal traps-region-probe` applies to formal/traps.sv. This script
keeps only the half of the original argument that is genuinely 2-safety and
has no exact decision procedure to hand a solver instead.

Two checks, all against the netlist `yosys -q -s` writes for `rtl/
decoder.v` (plus its dependencies), never against the source text:

  1. FORWARD REACHABILITY, twice. Starting from the bits of `reg_rs1`,
     `reg_rs2` and `executor_out.rd_data`, follow every cell's inputs to its
     outputs -- a flip-flop's D to its Q included, so a value laundered
     through a register is not read as clean the way `out.rs1 <= reg_rs1`
     was before this script existed -- with `region_stall` blocked from
     being used as a source for anything past its own one hop. None of the
     nine reasons above may be reached this way. Then, separately, starting
     from `ls_capture`/`ls_answer`/`ls_answer_valid`'s own bits with nothing
     blocked, the same nine must not be reachable either.
  2. PORT COVERAGE, BOTH WAYS. Every input port of `decoder` wider than 5
     bits (5 is the widest a register NUMBER gets: rd/rs1/rs2 are all
     `[4:0]`) must be classified SEED_PORTS/STRUCT_FIELD_SEEDS (can carry a
     register-file or CSR-file DATA output) or NON_VALUE_PORTS/
     STRUCT_FIELD_NON_VALUE (cannot, with the reason why) -- read off the
     netlist's own measured port width, not a `[N:0]` match against source
     text. A struct-typed port's fields are found the same way: a small
     satellite module, elaborated against rtl/structs.v alone, asks yosys
     what `val.<fieldname>` resolves to, and the offset a field's bits start
     at within the parent port is read from where THAT elaboration put them
     -- so a renamed field fails to elaborate at all, and an added or
     resized one fails a sum-of-widths check, rather than silently omitting
     a bit. Classifications are checked stale in both directions: an entry
     naming a port or field the netlist no longer has is as wrong as a port
     the tables have never seen. CONTROL_FIELDS' own fields (out.rd,
     out.is_amo, out.valid, executor_out.rd, executor_out.valid) are held to
     the same 5-bit bound: unlike a plain input port, a field wider than
     that is not merely unclassified, it is an exemption whose only stated
     justification -- "this is a register NUMBER, not a value" -- has
     stopped being true.

Usage: zkt_isolation_test.py [decoder.v]     # defaults to rtl/decoder.v
"""

import collections
import json
import os
import subprocess
import sys
import tempfile

# The nine stall-reason signals a Zkt-listed instruction's stall must be
# blind to, except through `GATED_SIGNAL`. `stall` is here alongside the
# eight it is built from: `stall = stall_other || region_stall` reads
# `region_stall` directly, and that read is exempted the same way every
# other reason's is, by blocking `region_stall` as a taint SOURCE below
# rather than by severing one wire -- `out`'s own bubble condition re-lists
# `region_stall` beside `hazard`/`operand_stall`/etc, a second direct read
# with no wire in common with `stall`'s, so a single severed edge could not
# have covered both.
STALL_TARGETS = [
    'ls_access', 'serialize', 'hazard_rs1', 'hazard_rs2', 'hazard',
    'operand_stall', 'atomic_stall', 'stall_own', 'stall_other', 'stall',
]

# The one reason allowed to depend on a register-file DATA output. That it
# can only assert alongside `ls_access`, and that `ls_access` is true for
# exactly the eight base load/store encodings, are no longer decided here --
# both are single-trace exact-set-equality facts, proved by k-induction as
# assertions inside rtl/decoder.v's own `ifdef FORMAL` block instead
# (`make -C formal components_decoder`, demonstrated red by
# `make -C formal decoder-zkt-probe`). What THIS script still owns is the
# 2-safety half: that no register-file DATA output reaches any of the nine
# reasons below except by way of a read of `GATED_SIGNAL` itself.
GATED_SIGNAL = 'region_stall'

# Registers derived from region_stall that hold a load's or store's own
# region answer across the cycle it is read on. Blocking `region_stall` as a
# taint source hides a DIRECT read of one of these from the reachability
# check above -- correctly, since it is region_stall's own downstream, not a
# new path from a register value -- so they get their own narrower check:
# none of STALL_TARGETS may read one of THESE by name either, seeded
# independently of reg_rs1/reg_rs2.
REGION_STATE = ['ls_capture', 'ls_answer', 'ls_answer_valid']

# The positive control: named nets the real RTL is known to carry reg_rs1
# through on the way to region_stall. If none of these end up reachable, the
# graph found no edges at all and every PASS above is a check of nothing --
# the same anti-vacuity discipline formal/check-nonperturbation.py applies
# with its one-mux self-test.
EXPECT_TAINTED = ['ls_block', 'ls_text_deep', 'ls_ram_deep', 'ls_settled',
                   GATED_SIGNAL]

# Plain (non-struct) decoder INPUT ports wide enough (>5 bits) to carry a
# register-file or CSR-file DATA output, and the ones wide enough that
# provably cannot.
SEED_PORTS = {'reg_rs1', 'reg_rs2'}
NON_VALUE_PORTS = {
    # csr_rdata: the CSR file's read value. CSR instructions are not on
    # Zkt's list, and the port-coverage check below is what would catch it
    # reaching a value-blind reason if that ever changed.
    'csr_rdata',
    # mtvec, mepc: trap CSRs, read only on the trap and mret arms of
    # next_pc, neither of which is on Zkt's list either.
    'mtvec', 'mepc',
}

# Struct-typed decoder INPUT ports: the typedef name (declared in
# rtl/structs.v, used to size a satellite probe module's own port so yosys
# resolves each field's offset and width by elaborating it, never by this
# script guessing struct layout) and the field names in the struct's own
# declared order. A renamed field fails that elaboration outright; an added,
# removed or resized one fails the sum-of-widths check in classify_inputs.
STRUCT_PORTS = {
    'in': ('fetcher_output', ['valid', 'pc', 'instr', 'next_instr']),
    'executor_out': ('executor_output', ['valid', 'rd', 'rd_data']),
}

# Struct fields (as `port.field`) wide enough (>5 bits) to matter, and their
# classification. executor_out.rd_data is a committed executor result --
# the same shape this repo keeps pricing and declining as a
# forwarding path -- so it is the one struct field that IS a seed. in's three
# wide fields are the fetch address and the instruction words decode reads
# register NUMBERS out of, never an operand value.
STRUCT_FIELD_SEEDS = {'executor_out.rd_data'}
STRUCT_FIELD_NON_VALUE = {'in.pc', 'in.instr', 'in.next_instr'}

# `out` (decoder_output) is decode's OWN output, not an input port -- outside
# classify_inputs' scope, since nothing external can feed decoder a value
# through it. Three of its fields are read back INSIDE decoder.v itself
# (`live_rs1`/`live_rs2`'s RAW-hazard check, and `atomic_stall`), and what
# makes reading them back SAFE is not their width -- a 5-bit field can
# perfectly well carry a value-dependent decision -- it is that every path by
# which a register VALUE can reach one of them is TRAP-MEDIATED: `out.rd`
# reads 0 instead of the real destination exactly when `reg_rs1` decided the
# PREVIOUS instruction was misaligned or out of region (the data-fault
# causes), and `out.valid`/`out.is_amo` change only on that same trap
# decision or an ordinary bubble. Taking a trap -- or not -- is
# architecturally VISIBLE (a different instruction retires, at a different
# pc), not a covert timing channel a Zkt-listed instruction's cycle count
# could leak an operand VALUE through, which is the property that actually
# licenses blocking these as taint sources rather than their width.
# `executor_out` gets the same two fields blocked for the identical reason --
# it is already in STRUCT_PORTS for its DATA field, `rd_data`, which stays a
# seed.
#
# `executor_out.valid`/`executor_out.rd` are inert rather than wrong: unlike
# `out`, `executor_out` is a decoder INPUT port, so its bits have no driving
# cell inside decoder for forward_taint to ever reach -- nothing here can
# taint a primary input, so blocking them is a no-op. Kept anyway, for the
# same reason `out`'s two are named rather than left to a width rule: the
# table states what is safe to read back, not merely what currently matters.
#
# The 5-bit bound control_field_bits enforces below is a USEFUL TRIPWIRE, not
# the reason: it catches a field growing wide enough to plausibly carry a raw
# VALUE rather than a decision ABOUT one, the same line SEED_PORTS/
# NON_VALUE_PORTS already draw for input ports. It cannot by itself tell a
# genuinely trap-mediated 5-bit field from a coincidentally narrow VALUE
# slice -- that argument is made by eye, above, once per field, and is what
# an added CONTROL_FIELDS entry has to repeat.
CONTROL_FIELDS = {
    'out': ('decoder_output', ['valid', 'rd', 'is_amo']),
    'executor_out': ('executor_output', ['valid', 'rd']),
}


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


def build_decoder_netlist(decoder_path, structs_path, regsel_path, out_dir):
    """Elaborate rtl/decoder.v (plus its dependencies) to a JSON netlist and
    return its `decoder` module. `proc; flatten` resolves every generate arm,
    `` `ifdef ``, procedural block and struct field access into ordinary
    cells and named nets before this script reads any of it; `memory_map;
    simplemap` is the same struct-breaking step formal/check-nonperturbation.py
    applies, for the same reason -- a packed struct is one wide cell until
    simplemap breaks it apart into the primitives build_graph and forward_taint
    know how to read. Deliberately NO `opt_clean` here, unlike that check: a pure
    bit-select such as `ls_block = reg_rs1[31:21]` has no cell of its own, so
    a fanout sweep folds its name away in favour of reg_rs1's, and this
    script needs every intermediate's OWN name to walk the graph one
    text-level term at a time. No port is deleted here either -- this reads
    named INTERNAL wires (`stall`, `region_stall`, ...), not a diff of two
    builds, so there is nothing for a sweep to clean up."""
    rtl_dir = os.path.dirname(os.path.abspath(decoder_path))
    srcs = ' '.join([structs_path, regsel_path, decoder_path])
    json_path = os.path.join(out_dir, 'decoder.json')
    script = '\n'.join([
        'design -reset',
        'read_verilog -sv -I %s %s' % (rtl_dir, srcs),
        'hierarchy -check -top decoder',
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
    mod = design.get('modules', {}).get('decoder')
    if mod is None:
        print('error: the elaborated netlist has no `decoder` module',
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
    """probe_struct_fields, memoized per typedef for the run: `executor_output`
    is named by both STRUCT_PORTS (for its DATA field) and CONTROL_FIELDS (for
    its two control fields), and probing it twice would elaborate the same
    satellite module through yosys twice for no new information. A request
    for a field the cache does not yet have re-probes the UNION of old and
    new fields, so caching never returns a stale answer to a wider ask."""
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
    """{seed net bit ids}, or a list of errors. Every decoder INPUT port
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
            # Summed over THIS port's own declared fields, never over
            # `offsets` as a whole: get_field_offsets can return a cache
            # entry widened by some OTHER port sharing the same typedef
            # (executor_output, named again by CONTROL_FIELDS), and that
            # extra field is not part of what STRUCT_PORTS declares here.
            total = sum(offsets[f][1] for f in fields)
            if total != width:
                errors.append(
                    'rtl/structs.v\'s `%s` totals %d bits across %s, but '
                    'decoder\'s `%s` port measures %d bits on the elaborated '
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
    as taint sources below the same way `region_stall` is. A stale or
    renamed entry fails the same way probe_struct_fields always does --
    there is no separate staleness table for this one, because CONTROL_FIELDS
    names exactly three fields against two ports, not a coverage surface a
    human could leave half-classified the way decoder's whole port list can.
    `field_cache` is shared with classify_inputs: `executor_output` is named
    by both, and this is what stops it being elaborated twice.

    The written justification for this exemption is entirely a width
    argument -- `rd` is `[4:0]`, the same width SEED_PORTS/NON_VALUE_PORTS
    draw the line at -- so this function enforces that bound the same way
    classify_inputs already enforces it for input ports: every field wider
    than 5 bits is an error, not a silently-blocked taint source. Widening
    `out.rd` to a full register value would otherwise blanket it from every
    check below with nothing to say so."""
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
        # Only THIS entry's own declared fields -- see classify_inputs'
        # identical guard against a cache entry widened by the other table.
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
    carried by a register is reachable the same as one computed
    combinationally this cycle, which is what stops a signal from passing as
    clean merely for being assigned procedurally rather than continuously.

    A bit in `blocked_bits` can still be REACHED (added to the returned set)
    but is never used as a SOURCE for tainting anything further downstream of
    it -- the same restricted-taint discipline `region_stall` needs: it is
    allowed to depend on a register value, and everything that reads
    `region_stall` (rather than reg_rs1/reg_rs2 themselves) is judged as if
    that one read were invisible. Any OTHER path into the same signal still
    counts; blocking removes only the specific paths this design intends."""
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
    decoder_path = argv[0] if argv else \
        os.path.join(here, os.pardir, 'rtl', 'decoder.v')
    if not os.path.isfile(decoder_path):
        print('error: cannot read %s' % decoder_path, file=sys.stderr)
        return 2

    decoder_dir = os.path.dirname(os.path.abspath(decoder_path))
    structs_path = os.path.join(decoder_dir, 'structs.v')
    regsel_path = os.path.join(decoder_dir, 'regsel.v')
    for p in (structs_path, regsel_path):
        if not os.path.isfile(p):
            print('error: cannot read %s' % p, file=sys.stderr)
            return 2

    with tempfile.TemporaryDirectory(prefix='zkt-isolation.') as out_dir:
        print('Elaborating %s (yosys)...' % decoder_path)
        mod = build_decoder_netlist(decoder_path, structs_path, regsel_path,
                                     out_dir)
        if mod is None:
            return 2

        # Shared across both classification passes: `executor_output` is
        # named by STRUCT_PORTS (for its DATA field) and CONTROL_FIELDS (for
        # its two control fields), and this is what stops it being probed
        # -- and elaborated through yosys -- twice.
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
        needed = (list(STALL_TARGETS) + [GATED_SIGNAL]
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

    # A signal this script watches can be a NAME with no driving cell at
    # all -- Verilog allows a wire with nothing assigning it, and yosys
    # raises no warning for one that is only ever READ, the same silent
    # shape as a deleted `assign hazard = ...;` -- and reachability through
    # it would then trivially and vacuously "pass" (nothing flows out of a
    # wire nothing drives). A bit with no driving cell is legitimate when it
    # is a pure alias of a primary input with no logic in between (`ls_block
    # = reg_rs1[31:21]` shares reg_rs1's own bit ids outright, and the
    # reachability check already treats it correctly for that reason), so
    # only a watched bit that is NEITHER driven NOR a primary input's own
    # bit is the deleted-assign shape.
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
              'deleted stall reason it is.' % ', '.join(undriven),
              file=sys.stderr)
        return 2

    failures = []

    # Full (unblocked) reachability is the anti-vacuity control: it is the
    # graph reg_rs1/reg_rs2 are KNOWN to reach in the real design, on the way
    # to region_stall and, through it, to `stall` and `out`'s bubble
    # condition. Checked before anything is blocked, so a PASS below cannot
    # be a check of a graph with no edges in it at all.
    full_reached = forward_taint(fanout, seed_bits)
    vacuous = [n for n in EXPECT_TAINTED
               if not any(b in full_reached for b in name_bits[n])]
    if vacuous:
        failures.append(
            'the netlist graph found no path from reg_rs1/reg_rs2/'
            'executor_out.rd_data to %s, which the real RTL is known to '
            'carry a register value through on the way to %s. That means '
            'this run found no edges at all, and every PASS above is a '
            'check of nothing.' % (', '.join(vacuous), GATED_SIGNAL))

    # The main check: block `region_stall` AND the register-NUMBER/control
    # fields of `out`/`executor_out` (CONTROL_FIELDS) as taint SOURCES, then
    # ask whether any of the nine stall-reason signals is still reachable
    # from a register-file DATA output. A read of `region_stall` itself --
    # by `stall` directly, or by `out`'s bubble condition and so,
    # downstream, `pipe_drained`/`serialize` -- is invisible to this walk,
    # on purpose: `region_stall` can only be true for a load or a store
    # (`ls_access`), so its influence on a Zkt-listed instruction's own
    # stall decision is zero regardless of what it is gated on. `out.rd`/
    # `out.is_amo`/`out.valid` (and executor_out's) are blocked for the same
    # reason SEED_PORTS stops at 5 bits: reading a REGISTER NUMBER or a
    # single control flag back -- which is what live_rs1/live_rs2's RAW
    # hazard check and atomic_stall do -- is not reading a VALUE, even
    # though which number or flag out.rd/out.valid ends up holding can
    # itself have been decided by a fault check that read one.
    def report_reachable(reached, template):
        """failures.append(template % name) for every STALL_TARGETS name
        `reached` has a bit in -- the shared shape both reachability checks
        below grade against, so the two only differ in what they seeded and
        the wording of what a hit means, not in how a hit is reported."""
        for name in reachable_targets(reached, name_bits, STALL_TARGETS):
            failures.append(template % name)

    blocked_bits = set(name_bits[GATED_SIGNAL]) | control_bits
    region_reached = forward_taint(fanout, seed_bits,
                                    blocked_bits=blocked_bits)
    report_reachable(
        region_reached,
        '`%s` is reachable, on the elaborated netlist, from a '
        'register-file DATA output (reg_rs1, reg_rs2 or '
        'executor_out.rd_data) through a path other than `%s`. A '
        'Zkt-listed instruction (add, xor, sll, mul, ...) can '
        'assert this, so its cycle count would depend on an '
        'operand\'s VALUE, not just which registers it names.'
        % ('%s', GATED_SIGNAL))

    # The narrower check blocking exists to make room for: none of the nine
    # may read `region_stall`'s OWN downstream state -- ls_capture, or the
    # answer it captures -- directly either. Seeded independently of
    # reg_rs1/reg_rs2, with the SAME two blocked sets: region_stall itself
    # (ls_answer_valid feeds back into region_stall's own gate, and that
    # read is exactly as legitimate seeded from here as it is from a
    # register value) and CONTROL_FIELDS (the same out.valid bubble path).
    # What is NOT blocked is ls_capture/ls_answer/ls_answer_valid's own
    # direct fanout, so a stall reason reading one of them straight -- the
    # shape that defeated the RTL-text version -- is still one hop away.
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
        'extend to being read by an unrelated instruction\'s stall '
        'decision.' % ('%s', '/'.join(REGION_STATE)))

    if failures:
        print('ZKT STALL ISOLATION: FAIL', file=sys.stderr)
        for f in failures:
            print('  ' + f, file=sys.stderr)
        return 1

    print('%s: on the elaborated netlist, reg_rs1/reg_rs2/'
          'executor_out.rd_data reach only %s among the stall reasons '
          '(its own gate is proved separately by make -C formal '
          'components_decoder).' % (decoder_path, GATED_SIGNAL))
    print('ZKT STALL ISOLATION: PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
