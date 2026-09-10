#!/usr/bin/env python3
# Generates the riscv-formal check set and reports what came out of it, against
# whichever harness directory (formal/, nano/formal/) is passed as argv[1].

import os
import re
import runpy
import sys

import depth_rules

# genchecks-local.py is this script's sibling; the harness dir is a separate argument.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
GENCHECKS = os.path.join(SCRIPT_DIR, "genchecks-local.py")

# The three cycles genchecks writes into every .sby it generates.
DEFINE_RE = re.compile(r"^`define\s+RISCV_FORMAL_(\w+_CYCLES?)\s+(\d+)\s*$")

# genchecks' parser drops `#` lines before a section, so these don't perturb generation.
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

def read_check_cycles(checks_dir, name):
    """The START, TRIG and CHECK cycles out of checks/<name>.sby."""
    cycles = {}
    with open(os.path.join(checks_dir, f"{name}.sby")) as f:
        for line in f:
            match = DEFINE_RE.match(line)
            if match:
                cycles[match.group(1)] = int(match.group(2))
    return cycles

def audit_depths(cfg, checks_dir, families):
    """Every generated check's CHECK cycle against its family's `#floor` rule.
    `families` maps a generated check name to the family it came from."""
    derived = depth_rules.read_derived(cfg)
    floors = depth_rules.read_floors(cfg)

    failed = report_set_diff(
        "checks.cfg #floor rules vs the check families generated",
        set(floors),
        set(families.values()),
        "#floor",
        "generated",
    )
    if failed:
        return True

    # Per family: naming all 70 insn_* checks would bury the one [depth] line to move.
    short = {}
    for name, family in sorted(families.items()):
        cycles = read_check_cycles(checks_dir, name)
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
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <harness-dir>", file=sys.stderr)
        return 1
    base = os.path.abspath(sys.argv[1])
    cfg = os.path.join(base, "checks.cfg")
    expected_checks_path = os.path.join(base, "EXPECTED_CHECKS")
    checks_dir = os.path.join(base, "checks")

    # genchecks reads checks.cfg and writes checks/ relative to the cwd, so running it
    # elsewhere silently produces a check set elsewhere.
    if os.path.realpath(os.getcwd()) != os.path.realpath(base):
        print(f"error: run from {base}, not {os.getcwd()}", file=sys.stderr)
        return 1

    # A missing gitignored clone otherwise surfaces later as an ISA-string error.
    riscv_formal_dir = os.path.join(
        os.path.dirname(os.path.realpath(GENCHECKS)), "riscv-formal"
    )
    if not os.path.isdir(riscv_formal_dir):
        print(
            f"error: {riscv_formal_dir} is missing. genchecks-local.py reads its\n"
            "       instruction list out of that clone, so its absence surfaces\n"
            "       downstream as \"Current isa string '...' not supported\" --\n"
            "       nothing is wrong with the ISA string. Fetch the pinned clone:\n"
            "       make -C formal riscv-formal (formal/pin.mk has the SHA).",
            file=sys.stderr,
        )
        return 1

    # genchecks-local.py runs in-process via runpy and reads sys.argv[1] as a cfg name,
    # so this script's own <harness-dir> argument must not leak into it.
    saved_argv = sys.argv
    sys.argv = [GENCHECKS]
    sys.settrace(call_tracer)
    try:
        genchecks = runpy.run_path(GENCHECKS, run_name="__main__")
    finally:
        sys.settrace(None)
        sys.argv = saved_argv

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

    # A trace disagreeing with genchecks' own bookkeeping voids every check below.
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
        e[: -len(".sby")] for e in os.listdir(checks_dir) if e.endswith(".sby")
    }
    failed |= report_set_diff(
        "generated check names vs checks/*.sby on disk",
        generated,
        on_disk,
        "genchecks",
        "disk",
    )

    expected = set(read_name_list(expected_checks_path))
    failed |= report_set_diff(
        "generated checks vs EXPECTED_CHECKS",
        expected,
        generated,
        "EXPECTED_CHECKS",
        "generated",
    )

    omitted = read_omit_decls(cfg)
    failed |= report_set_diff(
        "dropped checks vs checks.cfg #omit declarations",
        set(omitted),
        dropped,
        "#omit",
        "dropped",
    )

    depths_failed = audit_depths(cfg, checks_dir, families)

    print(
        f"Check-set shape: {len(generated)} generated, {len(dropped)} declined "
        f"for want of a [depth] line."
    )
    if failed:
        print(
            "\nThe check set is not the shape this repo committed to. Either the\n"
            "change was intended -- in which case update EXPECTED_CHECKS\n"
            "and/or checks.cfg's #omit list in the same commit, and say why --\n"
            "or a [depth] line was lost, which is the failure ADR-0033 named.",
            file=sys.stderr,
        )
    if failed or depths_failed:
        return 1

    print(
        f"EXPECTED_CHECKS: {len(expected)} names, exact match. "
        f"checks.cfg #omit: {len(omitted)} names, exact match."
    )
    return 0

if __name__ == "__main__":
    sys.exit(main())
