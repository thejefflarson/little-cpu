#!/usr/bin/env python3
"""Asserts that `netlist-digest` and `netlist-diff` are prerequisites of
nothing but themselves.

Usage: netlist_digest_gate_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. The digest replaces a sixteen-seed sweep, never a gate: an
equal digest says the placer's input moved by nothing but dead nets and
source attributes, which is sound in one direction only. If some OTHER
target's rule line ever lists `netlist-digest` or `netlist-diff` as its own
prerequisite, that target starts skipping the sweep the digest was only ever
meant to excuse -- silently, because a Makefile dependency edit reads as
plumbing and nothing runs to notice one target quietly started trusting
another's verdict. This reads the Makefile's own rule lines and refuses that
edge from existing anywhere but the two targets' own definitions.

WHAT IT PARSES. `target: prerequisites` lines, continuation-joined the way
make itself joins a trailing backslash. Three kinds of line are set aside
before the scan: a recipe line (tab-prefixed), because a target invoked from
inside another target's recipe -- `@$(MAKE) netlist-digest` -- is a stronger
coupling than this check is written for and would need its own; a variable
assignment (`:=`, and every line with no colon at all, which is every
`?=`/`+=`/`=` assignment and every directive this file uses); and
`.PHONY: netlist-digest`, because declaring a target phony creates no
dependency edge. The two targets' own rule lines are excluded too -- both may
depend on `netlist-determinism` freely, since that is the control that places
what they digest.

Hermetic: reads the Makefile as text. No make, no toolchain, so this runs
inside `make test` anywhere.
"""

import argparse
import os
import re
import sys

TARGETS = ("netlist-digest", "netlist-diff")
TOKEN = {name: re.compile(r"\b" + re.escape(name) + r"\b") for name in TARGETS}


def logical_lines(text):
    """Yield (is_recipe, joined_text) pairs, backslash continuations joined.

    Whether a joined line is a recipe is decided by its FIRST physical line
    alone: a continued dependency list never starts with a tab, even if a
    fragment further down happens to be indented that way.
    """
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        is_recipe = line.startswith("\t")
        buf = line
        while buf.endswith("\\") and i + 1 < len(lines):
            i += 1
            buf = buf[:-1] + " " + lines[i]
        yield is_recipe, buf
        i += 1


def rule_lines(text):
    """Yield (target, prerequisite_text) for every real rule line."""
    for is_recipe, line in logical_lines(text):
        if is_recipe:
            continue
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        colon = stripped.index(":")
        after = stripped[colon + 1:]
        if after.startswith("="):  # `:=` assignment, not a rule
            continue
        yield stripped[:colon].strip(), after.strip()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", default=os.path.dirname(here))
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    makefile = os.path.join(root, "Makefile")
    if not os.path.isfile(makefile):
        sys.exit(f"error: '{makefile}' does not exist, so there is nothing to parse.")
    with open(makefile, encoding="utf-8") as handle:
        text = handle.read()

    rules = list(rule_lines(text))
    seen_own_rule = {name: False for name in TARGETS}
    violations = []
    for target, prereqs in rules:
        if target == ".PHONY":
            continue
        if target in TARGETS:
            seen_own_rule[target] = True
            continue
        for name, pattern in TOKEN.items():
            if pattern.search(prereqs):
                violations.append((target, name))

    missing = [name for name in TARGETS if not seen_own_rule[name]]
    if missing:
        sys.exit("error: no rule line defines " + ", ".join(missing) + f" in {makefile}. "
                 "Either it was renamed and this check needs to move with it, or it "
                 "was deleted and this check is asserting a property of nothing.")

    if violations:
        print("error: a target other than netlist-digest/netlist-diff themselves "
              "lists one as its own prerequisite. THE DIGEST REPLACES A SWEEP, "
              "NEVER A GATE -- a target that depends on it starts skipping the "
              "sixteen-seed sweep silently, which is exactly the corollary this "
              "check exists to hold:", file=sys.stderr)
        for target, name in violations:
            print(f"  {target}: depends on {name}", file=sys.stderr)
        sys.exit(1)

    print(f"netlist-digest and netlist-diff are prerequisites of nothing but "
          f"themselves, across {len(rules)} rule lines.")


if __name__ == "__main__":
    main()
