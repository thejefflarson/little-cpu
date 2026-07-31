#!/usr/bin/env python3
#
# Generates the riscv-formal ladder and asserts its SHAPE -- which checks
# exist -- before anything runs them.
#
# Why this exists (ADR-0033, gap 1). `formal/checks.cfg`'s `[depth]` section
# reads as a tuning table. It is not: it is the list of checks that exist.
# Both of genchecks' call sites end with
#
#     if depth_cfg is None: return
#
# so a check with no matching `[depth]` line is not generated at all -- no
# `.sby`, no directory, no status file, no warning, exit 0. Deleting or
# mistyping one line therefore removes a check from the ladder silently, and
# nothing downstream notices: a never-generated check is absent from the
# results AND absent from `formal/EXPECTED_FAIL` at once, so that file's
# set-equality reports a clean match with less coverage than it claims.
#
# This script closes that by deriving, from the generator's own behaviour,
# the two sets a reader cares about:
#
#   generated  every check genchecks emitted a .sby for
#   dropped    every check genchecks CONSIDERED and skipped for want of a
#              [depth] line
#
# and asserting both against files the repo commits to -- `EXPECTED_CHECKS`
# and `checks.cfg`'s `#omit` declarations -- with set equality in BOTH
# directions, per ADR-0014's contract: an unexpected *addition* trips this as
# loudly as a disappearance. That matters after a pin bump (ADR-0013), which
# is exactly when upstream may grow a check nobody here has ruled on.
#
# HOW the derivation works, and why it is not a second copy of genchecks'
# logic. `genchecks-local.py` must stay byte-comparable with the pin except
# for `basedir` (ADR-0031), so it cannot be instrumented. Instead this script
# runs it under `sys.settrace` and records every `get_depth_cfg` call: its
# `patterns` argument and whether it returned None. The LAST pattern in that
# argument is, at every call site, the fully-qualified check name --
# `insn_add_ch0`, `csrw_mcycle_ch0`, `causal_ch0`, `hang` -- which is what
# makes a name recoverable without re-deriving it here.
#
# That assumption is not trusted, it is CHECKED: the names this script
# recovers as generated must equal genchecks' own `consistency_checks |
# instruction_checks` sets, and those must equal the `.sby` files actually on
# disk. If a future pin changes how a check name is built, or adds a
# `filter-checks` drop this script cannot see, the three disagree and this
# fails rather than quietly reporting a wrong inventory.
#
# Run from formal/, exactly like `python3 genchecks-local.py`, which it
# replaces as the `checks` target's recipe.

import os
import re
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GENCHECKS = os.path.join(HERE, "genchecks-local.py")
CFG = os.path.join(HERE, "checks.cfg")
EXPECTED_CHECKS = os.path.join(HERE, "EXPECTED_CHECKS")
CHECKS_DIR = os.path.join(HERE, "checks")

# `#omit <check-name> <one-line reason>` in checks.cfg. genchecks' own cfg
# parser drops every line starting with `#` before it sees a section, so
# these are invisible to it and cannot perturb generation.
OMIT_RE = re.compile(r"^#omit\s+(\S+)\s+(\S.*)$")


def read_name_list(path):
    """One name per line; `#` comments and blank lines ignored, as
    formal/EXPECTED_FAIL and test/EXPECTED_FAIL already do."""
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
    """Both directions, ADR-0014's contract. Returns True on mismatch."""
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


# ---------------------------------------------------------------- the trace

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
    # genchecks reads `checks.cfg` and writes `checks/` relative to the
    # working directory, and takes `corename` from its last path component.
    # Running it from anywhere else silently produces a ladder somewhere
    # else, so refuse rather than do that.
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

    generated = {n for n, ok in considered.items() if ok}
    dropped = {n for n, ok in considered.items() if not ok}

    failed = False

    # 1. The derivation validates itself against genchecks' own bookkeeping.
    #    If these disagree, every count below is wrong and none of the
    #    set-equalities beneath mean anything.
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

    # 2. ...and against what is actually on disk, which is what sby runs.
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

    # 3. The ladder is the size the repo says it is (ADR-0033's assertion).
    expected = set(read_name_list(EXPECTED_CHECKS))
    failed |= report_set_diff(
        "generated ladder vs formal/EXPECTED_CHECKS",
        expected,
        generated,
        "EXPECTED_CHECKS",
        "generated",
    )

    # 4. Every check upstream offered and this ladder declined is declined on
    #    purpose, in writing, next to [depth]. This is what makes the
    #    omission count derived rather than prose.
    omitted = read_omit_decls(CFG)
    failed |= report_set_diff(
        "dropped checks vs checks.cfg #omit declarations",
        set(omitted),
        dropped,
        "#omit",
        "dropped",
    )

    print(
        f"Ladder shape: {len(generated)} generated, {len(dropped)} declined "
        f"for want of a [depth] line."
    )
    if failed:
        print(
            "\nThe ladder is not the shape this repo committed to. Either the\n"
            "change was intended -- in which case update formal/EXPECTED_CHECKS\n"
            "and/or checks.cfg's #omit list in the same commit, and say why --\n"
            "or a [depth] line was lost, which is the failure ADR-0033 named.",
            file=sys.stderr,
        )
        return 1

    print(
        f"formal/EXPECTED_CHECKS: {len(expected)} names, exact match. "
        f"checks.cfg #omit: {len(omitted)} names, exact match."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
