#!/usr/bin/env python3
#
# Generates the riscv-formal check set and reports which checks came out of it.
# Run from formal/, the same way `python3 genchecks-local.py` was.
#
# The `[depth]` section of checks.cfg looks like a tuning table. It is really the
# list of checks that exist. Both call sites in genchecks end with
#
#     if depth_cfg is None: return
#
# so a check with no depth line is never generated: no `.sby`, no warning, exit
# 0. It then disappears from the results and from EXPECTED_FAIL together, and
# comparing failures alone sees nothing wrong.
#
# The trace below is not a second copy of the naming logic. genchecks-local.py
# has to stay a byte-for-byte copy of the upstream file except for two lines, so
# it cannot be edited to report anything. Instead `sys.settrace` watches each
# `get_depth_cfg` call and records its `patterns` argument and whether it
# returned None. The last pattern is the check name and the FIRST is the family
# it belongs to -- "insn" for all 70 insn_*, "csrc_upcnt" for one per CSR --
# which is what checks.cfg's [depth] table and its `#floor` lines are keyed on.
#
# main() then checks that guess rather than trusting it, against genchecks' own
# lists and against the `.sby` files on disk.
#
# It also grades every generated check's depth against the floor its family
# declares. That arithmetic is the only thing standing between this repo and a
# check that runs, reports PASS, and asked a shorter question than the one it
# was configured for -- a shallow depth is not an error anywhere in sby.

import os
import re
import runpy
import sys

import depth_rules

HERE = os.path.dirname(os.path.abspath(__file__))
GENCHECKS = os.path.join(HERE, "genchecks-local.py")
CFG = os.path.join(HERE, "checks.cfg")
EXPECTED_CHECKS = os.path.join(HERE, "EXPECTED_CHECKS")
CHECKS_DIR = os.path.join(HERE, "checks")

# The three cycles genchecks writes into every .sby it generates. Read back off
# the generated file rather than recomputed from the [depth] columns, because
# which column is which lives in genchecks' call sites and this script does not
# get to hold a second opinion about that.
DEFINE_RE = re.compile(r"^`define\s+RISCV_FORMAL_(\w+_CYCLES?)\s+(\d+)\s*$")

# genchecks' own parser drops every `#` line before it sees a section, so these
# cannot perturb generation.
OMIT_RE = re.compile(r"^#omit\s+(\S+)\s+(\S.*)$")


def read_name_list(path):
    """One name per line. `#` comments and blank lines are ignored, the same way
    formal/EXPECTED_FAIL and test/EXPECTED_FAIL allow them."""
    names = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line:
                names.append(line)
    return names


def read_omit_decls(path):
    decls = {}
    with open(path) as f:
        for line in f:
            match = OMIT_RE.match(line.rstrip("\n"))
            if match:
                decls[match.group(1)] = match.group(2).strip()
    return decls


def report_set_diff(label, expected, actual, expected_label, actual_label):
    """Compare both ways round. Returns True on mismatch."""
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    if not missing and not extra:
        return False
    print(f"\n{label}: MISMATCH", file=sys.stderr)
    for name in missing:
        print(f"  in {expected_label} but not {actual_label}: {name}", file=sys.stderr)
    for name in extra:
        print(f"  in {actual_label} but not {expected_label}: {name}", file=sys.stderr)
    return True


def read_check_cycles(name):
    """The START, TRIG and CHECK cycles out of checks/<name>.sby."""
    cycles = {}
    with open(os.path.join(CHECKS_DIR, f"{name}.sby")) as f:
        for line in f:
            match = DEFINE_RE.match(line)
            if match:
                cycles[match.group(1)] = int(match.group(2))
    return cycles


def audit_depths(families):
    """Every generated check's CHECK cycle against its family's `#floor` rule.
    `families` maps a generated check name to the family it came from."""
    derived = depth_rules.read_derived(CFG)
    floors = depth_rules.read_floors(CFG)

    failed = report_set_diff(
        "checks.cfg #floor rules vs the check families generated",
        set(floors),
        set(families.values()),
        "#floor",
        "generated",
    )
    if failed:
        return True

    # Reported per family, because a family is what one [depth] line
    # configures: naming all 70 insn_* checks would bury the one line that has
    # to move. The evidence is still a generated file, so one of them is named.
    short = {}
    for name, family in sorted(families.items()):
        cycles = read_check_cycles(name)
        depth = cycles.get("CHECK_CYCLE")
        if depth is None:
            print(
                f"error: checks/{name}.sby defines no RISCV_FORMAL_CHECK_CYCLE, "
                "so there is no depth here to grade.",
                file=sys.stderr,
            )
            return True
        start = cycles.get("RESET_CYCLES", 1)
        trig = cycles.get("TRIG_CYCLE")
        terms, reason = floors[family]
        breaches = {
            term: floor
            for term, floor in (
                (t, depth_rules.evaluate(t, derived, start, trig)) for t in terms
            )
            if depth < floor
        }
        if not breaches:
            continue
        entry = short.setdefault(
            family,
            {"example": name, "depth": depth, "breaches": breaches, "reason": reason,
             "count": 0},
        )
        entry["count"] += 1

    if not short:
        print(
            f"[depth] floors: F = {derived['F']}, G = {derived['G']}; "
            f"{len(families)} checks all at or above theirs."
        )
        return False

    print("\n[depth] floors: MISMATCH", file=sys.stderr)
    for family, entry in sorted(short.items()):
        for term, floor in sorted(entry["breaches"].items()):
            print(
                f"  {family}: depth {entry['depth']} is below {term} = {floor}"
                f" -- {entry['count']} check(s), e.g. checks/{entry['example']}.sby",
                file=sys.stderr,
            )
        print(f"      {family} is {entry['reason']}", file=sys.stderr)
    print(
        f"\nchecks.cfg declares F = {derived['F']} and G = {derived['G']}. A "
        "depth below its floor does not go red -- it goes\n"
        "green having stopped asking -- so generation stops here instead. Raise "
        "the [depth]\n"
        "entries named above; or, if F or G is what moved, re-measure both with\n"
        "`make -C formal remeasure-fg` before changing the `#derive` lines to "
        "match.",
        file=sys.stderr,
    )
    return True


records = []


def return_tracer(frame, event, arg):
    if event == "return":
        records.append((tuple(frame.f_locals["patterns"]), arg))
    return None


def call_tracer(frame, event, arg):
    if event == "call" and frame.f_code.co_name == "get_depth_cfg":
        return return_tracer
    return None


def main():
    # genchecks reads `checks.cfg` and writes `checks/` relative to the cwd and
    # takes `corename` from its last component, so running it elsewhere silently
    # produces a check set elsewhere.
    if os.path.realpath(os.getcwd()) != os.path.realpath(HERE):
        print(f"error: run from {HERE}, not {os.getcwd()}", file=sys.stderr)
        return 1

    sys.settrace(call_tracer)
    try:
        genchecks = runpy.run_path(GENCHECKS, run_name="__main__")
    finally:
        sys.settrace(None)

    if not records:
        print(
            "error: traced no get_depth_cfg calls. genchecks-local.py no longer\n"
            "       has a function by that name, or no longer takes `patterns`.\n"
            "       Re-read this script's header against the pin before trusting\n"
            "       any inventory it prints.",
            file=sys.stderr,
        )
        return 1

    considered = {}
    families = {}
    for patterns, result in records:
        name = patterns[-1]
        was_generated = result is not None
        if considered.get(name, was_generated) != was_generated:
            print(
                f"error: {name} was both generated and dropped -- the "
                "last-pattern-is-the-name assumption no longer holds.",
                file=sys.stderr,
            )
            return 1
        considered[name] = was_generated
        if was_generated:
            families[name] = patterns[0]

    generated = {n for n, ok in considered.items() if ok}
    dropped = {n for n, ok in considered.items() if not ok}

    failed = False

    # If the trace disagrees with genchecks' own bookkeeping, none of the set
    # equalities below mean anything.
    genchecks_own = set(genchecks["consistency_checks"]) | set(
        genchecks["instruction_checks"]
    )
    failed |= report_set_diff(
        "traced check names vs genchecks' own sets",
        genchecks_own,
        generated,
        "genchecks",
        "trace",
    )

    on_disk = {
        e[: -len(".sby")] for e in os.listdir(CHECKS_DIR) if e.endswith(".sby")
    }
    failed |= report_set_diff(
        "generated check names vs checks/*.sby on disk",
        generated,
        on_disk,
        "genchecks",
        "disk",
    )

    expected = set(read_name_list(EXPECTED_CHECKS))
    failed |= report_set_diff(
        "generated checks vs formal/EXPECTED_CHECKS",
        expected,
        generated,
        "EXPECTED_CHECKS",
        "generated",
    )

    # Every check upstream offered and this repo declined is declined in
    # writing, next to [depth].
    omitted = read_omit_decls(CFG)
    failed |= report_set_diff(
        "dropped checks vs checks.cfg #omit declarations",
        set(omitted),
        dropped,
        "#omit",
        "dropped",
    )

    # Last, because a depth is only worth grading once the set carrying it is
    # the set this repo committed to. Its verdict is kept apart from the shape's
    # so the diagnostic below answers only the question it was written for.
    depths_failed = audit_depths(families)

    print(
        f"Check-set shape: {len(generated)} generated, {len(dropped)} declined "
        f"for want of a [depth] line."
    )
    if failed:
        print(
            "\nThe check set is not the shape this repo committed to. Either the\n"
            "change was intended -- in which case update formal/EXPECTED_CHECKS\n"
            "and/or checks.cfg's #omit list in the same commit, and say why --\n"
            "or a [depth] line was lost, which is the failure ADR-0033 named.",
            file=sys.stderr,
        )
    if failed or depths_failed:
        return 1

    print(
        f"formal/EXPECTED_CHECKS: {len(expected)} names, exact match. "
        f"checks.cfg #omit: {len(omitted)} names, exact match."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
