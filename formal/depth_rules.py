#!/usr/bin/env python3
# The arithmetic behind formal/checks.cfg's [depth] table, parsed out of the file that
# states it.

import re

DERIVE_RE = re.compile(r"^#derive\s+([FG])\s+(\d+)\s*(\S.*)?$")
FLOOR_RE = re.compile(r"^#floor\s+(\S+)\s+(\S+)\s+(\S.*)$")

# The whole vocabulary a `#floor` term may use.
TERMS = ("F+1", "F+2", "F+G", "F+G+2", "F+2G", "start+G", "trig+G")

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

def read_sby_depth(path):
    """The `depth` key of a .sby's [options] section, or None if it states none or
    states something that is not a number. sby keeps the LAST value a key is given, so
    a second `depth` is refused rather than read as the first."""
    section, values = None, []
    with open(path) as f:
        for line in f:
            text = line.strip()
            if text.startswith("[") and text.endswith("]"):
                section = text
                continue
            key, _, value = text.partition(" ")
            if section == "[options]" and key == "depth":
                values.append(value.strip())
    if len(values) > 1:
        raise ValueError(
            f"{path} states `depth` {len(values)} times in [options] "
            f"({', '.join(values)}); sby searches to the last, so no one of them "
            "can be graded"
        )
    if not values or not values[0].isdigit():
        return None
    return int(values[0])

def evaluate(term, derived, start, trig):
    """One term's lower bound on a check's CHECK cycle."""
    if term.isdigit():
        return int(term)
    if term == "F+1":
        return derived["F"] + 1
    if term == "F+G":
        return derived["F"] + derived["G"]
    if term == "F+2":
        return derived["F"] + 2
    if term == "F+G+2":
        return derived["F"] + derived["G"] + 2
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
