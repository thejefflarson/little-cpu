#!/usr/bin/env python3
#
# The arithmetic behind formal/checks.cfg's [depth] table, parsed out of the
# file that states it.
#
# WHY THIS EXISTS. Every depth in that table is derived from two measured
# figures -- F, the worst-case first retire, and G, the worst-case gap between
# two retires. The derivation used to live only in a comment, and a depth below
# its derived floor does not fail: the check goes green having stopped asking.
# Two entries now clear their floor by exactly one cycle, so the next change
# that lengthens a stall has no margin left to absorb.
#
# So F and G are declared in `#`-prefixed lines that genchecks' own cfg parser
# drops before it sees a section -- the same trick the `#omit` lines use, so
# reading them here perturbs nothing -- and formal/genchecks-audit.py evaluates
# every `#floor` rule against the cycles it reads back off the generated .sby.
# formal/remeasure-fg.py re-measures F and G against the same declaration.
#
# Two readers, one format, one parser.

import re

DERIVE_RE = re.compile(r"^#derive\s+([FG])\s+(\d+)\s*(\S.*)?$")
FLOOR_RE = re.compile(r"^#floor\s+(\S+)\s+(\S+)\s+(\S.*)$")

# The whole vocabulary a `#floor` term may use. Anything else is a typo or a
# rule nobody has thought through, and either way it stops generation rather
# than being skipped: a floor that silently evaluates to nothing is exactly the
# green-having-stopped-asking failure this file exists to close.
#
#   F+1       the check asserts a registered flag, so it flips one cycle after
#             the first retire it is watching for.
#   F+G       one hop: the retire under test may be a second rather than the
#             first out of reset.
#   F+2G      two hops: it may be a third.
#   start+G   a two-retire check shadows the older retire only from its
#             RISCV_FORMAL_RESET_CYCLES cycle, so its window has to hold a
#             whole gap.
#   trig+G    the same window, measured from the cycle the check triggers on.
#   <number>  measured directly, for a check F and G do not bound.
TERMS = ("F+1", "F+G", "F+2G", "start+G", "trig+G")


def read_derived(path):
    """The `#derive` lines: {"F": 6, "G": 6}. Both are required, because every
    term below is written in them."""
    derived = {}
    with open(path) as f:
        for line in f:
            match = DERIVE_RE.match(line.rstrip("\n"))
            if match:
                name, value = match.group(1), int(match.group(2))
                if name in derived:
                    raise ValueError(f"{path}: {name} is declared twice")
                derived[name] = value
    missing = sorted({"F", "G"} - set(derived))
    if missing:
        raise ValueError(
            f"{path}: no `#derive` line for {', '.join(missing)}. Every depth "
            "floor is written in F and G, so neither may be left implicit."
        )
    return derived


def read_floors(path):
    """The `#floor` lines: {check family: ([term, ...], reason)}."""
    floors = {}
    with open(path) as f:
        for line in f:
            match = FLOOR_RE.match(line.rstrip("\n"))
            if not match:
                continue
            family, terms, reason = match.groups()
            if family in floors:
                raise ValueError(f"{path}: {family} has two `#floor` lines")
            terms = terms.split(",")
            for term in terms:
                if term not in TERMS and not term.isdigit():
                    raise ValueError(
                        f"{path}: `#floor {family}` uses the term '{term}', "
                        f"which is not one of {', '.join(TERMS)} or a number. "
                        "Add it to depth_rules.TERMS with what it means, or "
                        "spell the floor in the terms that are there."
                    )
            floors[family] = (terms, reason.strip())
    return floors


def evaluate(term, derived, start, trig):
    """One term's lower bound on a check's CHECK cycle."""
    if term.isdigit():
        return int(term)
    if term == "F+1":
        return derived["F"] + 1
    if term == "F+G":
        return derived["F"] + derived["G"]
    if term == "F+2G":
        return derived["F"] + 2 * derived["G"]
    if term == "start+G":
        return start + derived["G"]
    if term == "trig+G":
        if trig is None:
            raise ValueError(
                "a `trig+G` floor was written for a check with no "
                "RISCV_FORMAL_TRIG_CYCLE, so there is no window to measure"
            )
        return trig + derived["G"]
    raise ValueError(f"unknown floor term '{term}'")
