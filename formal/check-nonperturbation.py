#!/usr/bin/env python3
"""Prove, structurally, that the RVFI instrumentation is unread by the core.

Run from formal/ -- the paths below are relative to it.

WHAT THIS PROVES, AND WHAT IT DOES NOT
--------------------------------------
The obligation is that the `ifdef RISCV_FORMAL` shadow payload threaded
through rtl/structs.v must not change what the core does.  It was once
*argued, not proven*, with the burden on reviewers: "verify by reading that no
`ifdef`'d value reaches a non-`ifdef`'d signal."

That sentence is the property, and it is a STRUCTURAL property, not a
behavioural one.  This script decides it:

    gold = the plain build.
    gate = the `-D RISCV_FORMAL` build with the rvfi_* ports deleted and
           everything that only ever fed them swept away.

If the instrumentation is write-only with respect to the core, then deleting
the rvfi_* ports makes every shadow register and every gate feeding one
unreachable, a pure fanout sweep removes all of it, and what is left must be
the same netlist as gold -- same cells, same wiring.  If ANY `ifdef`'d value
reaches a real signal, that value is reachable from a real output, survives the
sweep, and shows up here as extra cells or as a different connectivity
fingerprint.

THIS IS STRICTLY WEAKER THAN SEQUENTIAL EQUIVALENCE, AND DELIBERATELY SO.  It
proves the instrumentation is *unread*.  It does not prove that two arbitrary
designs behave alike, and it must never be described as if it did.  Two RTL
designs that compute the same function by different structures pass a miter and
fail this check; that is the correct outcome here, because the thing under test
is a preprocessor macro, not a redesign.  What it buys in exchange is that it is
decidable, exact, and runs in about a second -- where the miter it replaces
(formal/equiv.sh) did not converge at any budget, because `equiv_make`'s
name-based matching paired almost nothing across the two builds.

HOW THE COMPARISON AVOIDS THE FAILURE THAT MADE equiv_make USELESS
------------------------------------------------------------------
Cell and wire NAMES differ between the two builds -- yosys' auto-name counters
shift the moment the `ifdef` bodies enter the AST.  So nothing here compares
names.  Two name-free comparisons are made instead:

  1. The cell histogram by type (what `stat` prints), exactly.
  2. A canonical connectivity fingerprint: Weisfeiler-Leman colour refinement
     over the bipartite cell/net-bit graph, seeded from cell types, cell
     parameters, port names and bit positions, module port names, and constant
     bits -- every one of which is name-independent.  Isomorphic netlists give
     identical colour multisets; a rewired netlist does not.

Both builds are normalised the same way: `proc; flatten; memory_map; simplemap`,
then `opt_clean -purge` to a fixpoint.  Two details of that are load-bearing.

  * `simplemap` is what makes the sweep possible at all.  Without it a packed
    struct is one wide `$sdffe`, real fields and shadow fields in the SAME cell,
    and `opt_clean` cannot remove half a cell -- so the shadow logic feeding
    those bits stays live and gold and gate differ by ~47 cells on a core that
    is perfectly well behaved.  Measured, before this pipeline was settled.
  * There is NO `opt` anywhere.  `prep` would run one, on a gate design that
    still contains the shadow logic, and any cross-talk between optimising that
    logic and optimising the core's would land here as a false red.  Removing
    dead logic by pure fanout sweeping has no such coupling.

WHY THIS CANNOT PASS WITHOUT CHECKING ANYTHING
----------------------------------------------
The obvious way for a structural gate to be worthless is for the two builds to
be trivially identical -- a `-D` that stopped taking, a `delete -port` pattern
that stopped matching, a normalisation that erased the difference before the
comparison.  In every one of those the check goes green having compared a design
with itself.  So a third build is made -- instrumented, ports NOT deleted, same
normalisation -- and three controls are asserted before the comparison is
believed:

  * gold declares no rvfi_* port; the instrumented build declares some.
  * after `delete -port`, the gate declares none.
  * the instrumented build has strictly MORE cells than gold, by a wide margin.
    This is the one that catches a normalisation that quietly does the sweep's
    job for it.

A red on any control is a red on the gate.
"""

import collections
import hashlib
import json
import os
import subprocess
import sys
import tempfile

RTL = [
    "structs.v",
    "fetcher.v",
    "regfile.v",
    "csrs.v",
    "decoder.v",
    "regsel.v",
    "executor.v",
    "accessor.v",
    "writeback.v",
    "littlecpu.v",
]

# Repeated rather than looped: `opt_clean` is a single fanout sweep, and a
# removal can expose the next one, so it has to run to a fixpoint. Six is
# comfortably past the observed fixpoint (three) on this design, costs
# milliseconds, and the fixpoint is ASSERTED below rather than assumed -- if a
# future design needs more passes than this, the assertion says so instead of
# the comparison silently reporting a difference that is really leftover trash.
SWEEP_PASSES = 6

# Colour-refinement rounds. The graph diameter that matters is the depth of a
# combinational cone; 8 rounds distinguishes far more than that in practice, and
# the cost is linear.
WL_ROUNDS = 8


# Resolved from this file's own location, not from the working directory: the
# Makefile runs it from formal/, but a bare `python3 formal/check-...` from the
# repo root is the obvious thing to try and would otherwise fail inside yosys
# with a missing-file error that says nothing about why.
RTL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "rtl")


def yosys_script(out_dir):
    """One yosys invocation, three builds, three JSON netlists."""
    srcs = " ".join(os.path.join(RTL_DIR, f) for f in RTL)
    sweep = "\n".join(["opt_clean -purge"] * SWEEP_PASSES)

    def build(name, defines, delete_ports):
        return "\n".join(
            [
                "design -reset",
                f"read_verilog {defines}-sv {srcs}",
                "hierarchy -check -top littlecpu",
                "proc",
                "flatten",
            ]
            + (["delete -port littlecpu/rvfi_*"] if delete_ports else [])
            + [
                "memory_map",
                "simplemap",
                sweep,
                f"write_json {os.path.join(out_dir, name)}.json",
            ]
        )

    return "\n".join(
        [
            build("gold", "", False),
            build("instrumented", "-D RISCV_FORMAL ", False),
            build("gate", "-D RISCV_FORMAL ", True),
        ]
    )


def run_yosys(out_dir):
    script = yosys_script(out_dir)
    path = os.path.join(out_dir, "build.ys")
    with open(path, "w") as f:
        f.write(script + "\n")
    # check=True: a yosys that fails to elaborate must fail this gate, not
    # leave a stale or absent JSON for the comparison to trip over later with a
    # confusing message.
    proc = subprocess.run(
        ["yosys", "-q", "-s", path], capture_output=True, text=True
    )
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise SystemExit("yosys failed (exit %d) -- see output above" % proc.returncode)
    # yosys -q still prints warnings; surface them rather than swallowing them.
    # NOT promoted to errors here, deliberately: CI's `elaborate` job already
    # owns that policy for this RTL, with a curated allowlist (the "Deep
    # recursion in AST simplifier" note), and a second promotion on a different
    # yosys pipeline would go red for reasons that have nothing to do with the
    # property this gate decides. Zero appear today; if that changes, read them.
    for line in (proc.stdout + proc.stderr).splitlines():
        if line.startswith("Warning:"):
            print("  yosys: " + line)


def load(out_dir, name):
    with open(os.path.join(out_dir, name + ".json")) as f:
        design = json.load(f)
    mods = design["modules"]
    if "littlecpu" not in mods:
        raise SystemExit("%s.json has no littlecpu module" % name)
    return mods["littlecpu"]


def histogram(mod):
    return collections.Counter(c["type"] for c in mod["cells"].values())


def port_names(mod):
    return sorted(mod["ports"].keys())


def _h(obj):
    return hashlib.blake2b(repr(obj).encode(), digest_size=16).hexdigest()


def adjacency(mod):
    """cell -> [(port, direction, bit_index, net)], and net -> [(cell, port, direction, bit_index)].

    `net` is an int for a real net bit and a str ("0"/"1"/"x"/"z") for a
    constant, which is exactly the distinction the seed colours want.
    """
    cell_pins = {}
    net_pins = collections.defaultdict(list)
    for cname, cell in mod["cells"].items():
        directions = cell.get("port_directions", {})
        pins = []
        for port, bits in cell["connections"].items():
            direction = directions.get(port, "input")
            for index, bit in enumerate(bits):
                pins.append((port, direction, index, bit))
                net_pins[bit].append((cname, port, direction, index))
        cell_pins[cname] = pins
    return cell_pins, net_pins


def fingerprint(mod):
    """Name-independent canonical form: Weisfeiler-Leman colour multisets.

    Seeds carry no cell or wire name -- only cell type, cell parameters, port
    names, bit positions, module port names and constant values. Two isomorphic
    netlists therefore produce identical multisets no matter how yosys named
    or ordered anything.
    """
    cell_pins, net_pins = adjacency(mod)

    cell_colour = {}
    for cname, cell in mod["cells"].items():
        params = sorted((k, str(v)) for k, v in cell.get("parameters", {}).items())
        cell_colour[cname] = _h(("cell", cell["type"], params))

    # A module port bit is the one net a name may legitimately seed from: port
    # names are the design's interface, identical by construction in both
    # builds, and pinning them is what stops the refinement from being free to
    # permute the boundary.
    port_bit = {}
    for pname, pdata in mod["ports"].items():
        for index, bit in enumerate(pdata["bits"]):
            port_bit[bit] = (pname, pdata["direction"], index)

    net_colour = {}
    for net in net_pins:
        if isinstance(net, str):
            net_colour[net] = _h(("const", net))
        elif net in port_bit:
            net_colour[net] = _h(("port",) + port_bit[net])
        else:
            net_colour[net] = _h(("net",))

    local_cell_colour = None
    for round_index in range(WL_ROUNDS):
        new_cell = {
            cname: _h(
                (
                    cell_colour[cname],
                    sorted(
                        (port, direction, index, net_colour[net])
                        for port, direction, index, net in pins
                    ),
                )
            )
            for cname, pins in cell_pins.items()
        }
        new_net = {
            net: _h(
                (
                    net_colour[net],
                    sorted(
                        (cell_colour[cname], port, direction, index)
                        for cname, port, direction, index in pins
                    ),
                )
            )
            for net, pins in net_pins.items()
        }
        cell_colour, net_colour = new_cell, new_net
        if round_index == 0:
            local_cell_colour = cell_colour

    return (
        collections.Counter(cell_colour.values()),
        collections.Counter(net_colour.values()),
        local_cell_colour,
    )


def unswept_cells(mod):
    """Cells none of whose output bits is read or exported: sweep residue.

    Zero is the expected answer for both designs. A nonzero count means
    SWEEP_PASSES did not reach a fixpoint, and the comparison below would then
    be reporting leftover trash rather than a perturbation.
    """
    read = set()
    for cell in mod["cells"].values():
        directions = cell.get("port_directions", {})
        for port, bits in cell["connections"].items():
            if directions.get(port, "input") != "output":
                read.update(b for b in bits if isinstance(b, int))
    for pdata in mod["ports"].values():
        if pdata["direction"] in ("output", "inout"):
            read.update(b for b in pdata["bits"] if isinstance(b, int))

    residue = []
    for cname, cell in mod["cells"].items():
        directions = cell.get("port_directions", {})
        outs = [
            b
            for port, bits in cell["connections"].items()
            if directions.get(port, "input") == "output"
            for b in bits
            if isinstance(b, int)
        ]
        if outs and not any(b in read for b in outs):
            residue.append((cname, cell["type"]))
    return residue


def selftest_fingerprint(mod):
    """Prove the connectivity fingerprint can fail, before trusting it to pass.

    The cell histogram is the sharp instrument here and its bite is easy to
    demonstrate on real RTL: any perturbation big enough to matter adds cells.
    The fingerprint is the backstop for the case the histogram cannot see -- a
    netlist rewired at constant cell count -- and no mutation of this core
    produces one of those without also moving the histogram. So the fingerprint
    would otherwise be a comparison that has never been observed to fail, which
    is this repo's recurring defect and the one this self-test exists to stop
    repeating.

    So: take gold, swap the A and B inputs of exactly one `$_MUX_`, and require
    that the histogram is unchanged and the fingerprint is not. Same shape as
    test/exec_tb.v's `ref_selftest` -- an oracle that is wrong is worse than no
    oracle, so it is pinned before a single real vector runs.
    """
    victim = None
    for cname in sorted(mod["cells"]):
        cell = mod["cells"][cname]
        if cell["type"] != "$_MUX_":
            continue
        conns = cell["connections"]
        if conns.get("A") != conns.get("B"):
            victim = cname
            break
    if victim is None:
        return ["no $_MUX_ with distinct A/B inputs to perturb -- the "
                "fingerprint self-test could not run, so the fingerprint below "
                "is unvalidated"]

    mutated = {
        "ports": mod["ports"],
        "cells": dict(mod["cells"]),
    }
    cell = mod["cells"][victim]
    swapped = dict(cell)
    swapped["connections"] = dict(cell["connections"])
    swapped["connections"]["A"], swapped["connections"]["B"] = (
        cell["connections"]["B"],
        cell["connections"]["A"],
    )
    mutated["cells"][victim] = swapped

    problems = []
    if histogram(mutated) != histogram(mod):
        problems.append(
            "swapping one $_MUX_'s A/B inputs changed the cell histogram -- the "
            "self-test is not testing what it claims to"
        )
    ref_cc, ref_nc, _ = fingerprint(mod)
    mut_cc, mut_nc, _ = fingerprint(mutated)
    if (ref_cc, ref_nc) == (mut_cc, mut_nc):
        problems.append(
            "the connectivity fingerprint did not notice one $_MUX_ (%s) having "
            "its A and B inputs swapped. It cannot distinguish rewired netlists "
            "and must not be relied on" % victim
        )
    return problems


def main():
    failures = []

    with tempfile.TemporaryDirectory(prefix="nonperturbation.") as out_dir:
        print("Building gold / instrumented / gate netlists (yosys)...")
        run_yosys(out_dir)
        gold = load(out_dir, "gold")
        instrumented = load(out_dir, "instrumented")
        gate = load(out_dir, "gate")

    gold_hist = histogram(gold)
    inst_hist = histogram(instrumented)
    gate_hist = histogram(gate)

    gold_cells = sum(gold_hist.values())
    inst_cells = sum(inst_hist.values())
    gate_cells = sum(gate_hist.values())

    print()
    print("Controls (these are what stop this gate from passing vacuously)")
    print("---------------------------------------------------------------")

    oracle_problems = selftest_fingerprint(gold)
    print(
        "  fingerprint self-test:             %s"
        % ("ORACLE BROKEN" if oracle_problems else "notices a one-mux rewire")
    )
    failures.extend(oracle_problems)

    gold_rvfi = [p for p in port_names(gold) if p.startswith("rvfi_")]
    inst_rvfi = [p for p in port_names(instrumented) if p.startswith("rvfi_")]
    gate_rvfi = [p for p in port_names(gate) if p.startswith("rvfi_")]

    print("  gold rvfi_* ports:                 %d (want 0)" % len(gold_rvfi))
    print("  instrumented rvfi_* ports:         %d (want > 0)" % len(inst_rvfi))
    print("  gate rvfi_* ports after delete:    %d (want 0)" % len(gate_rvfi))
    print("  gold cells:                        %d" % gold_cells)
    print(
        "  instrumented cells:                %d (want > gold, by a wide margin)"
        % inst_cells
    )
    print("  gate cells after the sweep:        %d" % gate_cells)

    if gold_rvfi:
        failures.append(
            "the plain build declares rvfi_* ports (%d) -- `ifdef RISCV_FORMAL` "
            "is not guarding them" % len(gold_rvfi)
        )
    if not inst_rvfi:
        failures.append(
            "the `-D RISCV_FORMAL` build declares no rvfi_* port. The macro is "
            "not taking, so gold and gate are the same build and this gate is "
            "comparing a design with itself."
        )
    if gate_rvfi:
        failures.append(
            "`delete -port littlecpu/rvfi_*` left %d rvfi_* port(s): %s"
            % (len(gate_rvfi), ", ".join(gate_rvfi))
        )
    # The margin is a floor, not a measurement: the instrumentation is thousands
    # of cells (rtl/structs.v's shadow payload alone is >100 bits per stage), so
    # anything under a few hundred means the normalisation swept it before the
    # comparison could see it. Deliberately loose -- this is not a size ratchet.
    if inst_cells <= gold_cells + 200:
        failures.append(
            "the instrumented build is only %d cells larger than gold. The "
            "shadow payload should be thousands. Either the macro is not "
            "taking or the normalisation is sweeping the instrumentation "
            "before the comparison sees it -- in both cases the comparison "
            "below is vacuous." % (inst_cells - gold_cells)
        )

    for name, mod in (("gold", gold), ("gate", gate)):
        residue = unswept_cells(mod)
        print("  %s sweep residue:%s%d cell(s) (want 0)" % (name, " " * (21 - len(name)), len(residue)))
        if residue:
            failures.append(
                "%s still has %d cell(s) driving nothing after %d opt_clean "
                "passes -- the sweep did not reach a fixpoint, so a difference "
                "below would not be attributable. First few: %s"
                % (
                    name,
                    len(residue),
                    SWEEP_PASSES,
                    ", ".join("%s (%s)" % r for r in residue[:5]),
                )
            )

    print()
    print("Structural comparison: gold vs gate")
    print("-----------------------------------")

    if gold_hist == gate_hist:
        print("  cell histogram:                    identical (%d cells)" % gate_cells)
    else:
        types = sorted(set(gold_hist) | set(gate_hist))
        rows = [
            (t, gold_hist.get(t, 0), gate_hist.get(t, 0))
            for t in types
            if gold_hist.get(t, 0) != gate_hist.get(t, 0)
        ]
        print("  cell histogram:                    DIFFERS")
        print("    %-24s %8s %8s %8s" % ("type", "gold", "gate", "delta"))
        for t, g, a in rows:
            print("    %-24s %8d %8d %+8d" % (t, g, a, a - g))
        failures.append(
            "cell histogram differs by %d cell(s) in %d type(s). Cells that "
            "survive `delete -port littlecpu/rvfi_*` plus a full sweep are "
            "cells a REAL signal depends on -- an `ifdef RISCV_FORMAL` value "
            "has reached the core (ADR-0006, ADR-0020)."
            % (gate_cells - gold_cells, len(rows))
        )

    gold_cc, gold_nc, gold_local = fingerprint(gold)
    gate_cc, gate_nc, gate_local = fingerprint(gate)

    if gold_cc == gate_cc and gold_nc == gate_nc:
        print(
            "  connectivity fingerprint:          identical "
            "(%d cell colours, %d net colours, %d WL rounds)"
            % (len(gold_cc), len(gold_nc), WL_ROUNDS)
        )
    else:
        print("  connectivity fingerprint:          DIFFERS")
        only_gate = collections.Counter(gate_cc) - collections.Counter(gold_cc)
        only_gold = collections.Counter(gold_cc) - collections.Counter(gate_cc)
        print(
            "    cell colours only in gate: %d, only in gold: %d "
            "(a single rewire recolours its whole cone, so these are not a count "
            "of changed cells)"
            % (sum(only_gate.values()), sum(only_gold.values()))
        )
        # Names are useless for MATCHING and this is the only place they are
        # used at all: once a difference is known to exist they are the best
        # thing to hand a human. The listing uses the ROUND-1 colours, not the
        # settled ones -- a round-1 colour is a cell plus its immediate
        # neighbourhood, so it localises to cells actually wired differently
        # instead of naming everything downstream of the first one.
        surplus = collections.Counter(gate_local.values()) - collections.Counter(
            gold_local.values()
        )
        # Consume the surplus rather than listing every cell that happens to
        # share a colour with it: a colour gate has 100 of and gold has 99 of is
        # ONE unexplained cell, not a hundred.
        suspects = []
        for cname in sorted(gate_local):
            colour = gate_local[cname]
            if surplus.get(colour, 0) > 0:
                surplus[colour] -= 1
                suspects.append(cname)
        print("    cells with a gold-unmatched immediate neighbourhood: %d" % len(suspects))
        for cname in suspects[:15]:
            print("      %s" % cname)
        if len(suspects) > 15:
            print("      ... and %d more" % (len(suspects) - 15))
        failures.append(
            "the gate netlist is wired differently from gold even though the "
            "cell counts may agree. Same conclusion as a histogram difference: "
            "something under `ifdef RISCV_FORMAL` is reachable from a real "
            "signal (ADR-0006, ADR-0020)."
        )

    print()
    if failures:
        print("RVFI NON-PERTURBATION: FAIL")
        for f in failures:
            print("  - " + f)
        print()
        print(
            "ADR-0020's obligation is that no `ifdef RISCV_FORMAL` value reaches a\n"
            "non-`ifdef`'d signal. Read the change against that; do not relax this\n"
            "check to make it pass (ADR-0047)."
        )
        return 1

    print("RVFI NON-PERTURBATION: PASS")
    print(
        "  The `-D RISCV_FORMAL` build, with its rvfi_* ports deleted, sweeps to a\n"
        "  netlist structurally identical to the plain build. The instrumentation is\n"
        "  unread by the core (ADR-0006, ADR-0020, ADR-0047).\n"
        "  This is NOT sequential equivalence and must not be quoted as such: it\n"
        "  proves the instrumentation is unread, not that two designs behave alike."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
