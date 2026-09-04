#!/usr/bin/env python3
"""Asserts that the placement spread and the edit-churn band are stated in ONE
place, and that the rulebook's copy of them agrees with it.

Usage: band_source_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. Both figures were prose in six files -- soc/baseline_summary.py
three times including the line printed under every delta, plus
soc/timing_sweep.sh, soc/depth/sweep.sh, soc/depth/summary.py,
soc/compare/sweep.sh and the Makefile. Prose copies do not move together: the
placement spread read "1-2%" in all six for as long as it took a sixteen-seed
sweep to measure 9.2%, which is the difference between calling a delta noise and
calling it a placement under the board clock. Nothing went red for that, because
a comment cannot go red. So the copies are gone and this is what keeps them gone.

WHAT IT SCANS AND WHAT IT DOES NOT. Every tracked file except six kinds:

  * soc/bands.py, which is the source and is supposed to carry the figures.
  * docs/adr/, because an ADR is a measurement with a date on it and MUST NOT
    move when a later sweep moves the band. A dated record disagreeing with
    today's figure is the record working, not rotting.
  * docs/ideas/, which are briefs written before the measurements they propose.
  * this file, whose own patterns are spelled out below.
  * test/probe_gates.sh, which forces this check red by planting a band figure
    in a fixture and quotes what it planted in the probe's label. A probe that
    cannot name what it is planting is not a probe -- the same exception, for
    the same reason, that test/march_test.sh grants it.
  * test/PROBES_EXPECTED, which holds test/probe_gates.sh's labels verbatim and
    so carries the same planted figure.

CLAUDE.md IS GRADED RATHER THAN BANNED, and only in one direction. It is the
rulebook and a reader needs the number in it, so the copy stays and is required
to agree with soc/bands.py: every figure that file states must appear there, and
it must name soc/bands.py as the source. The other direction -- no band figure in
CLAUDE.md that soc/bands.py does not state -- is deliberately NOT mechanised,
because that document legitimately quotes dozens of other percentages, several of
them on lines that also say "spread", and a check that noisy would be turned off
within a month. What the forward direction catches is the case that matters: a
re-derived band that was not carried into the rulebook.

THE `fit` CELL BAND IS OUT OF SCOPE. That one is +/-50 packed cells rather than a
percentage of a period, it is derived per tree by a different instrument, and
FIT_MAX_LC is its ratchet. Nothing here looks at it.

Hermetic: git and file reads. No toolchain, so this runs inside `make test`
anywhere.
"""

import argparse
import os
import re
import subprocess
import sys

SOURCE = "soc/bands.py"

# The figures live in the source, so the source is imported rather than parsed.
# A checker with its own copy of the numbers would be the seventh copy.
# test/PROBES_EXPECTED is exempt for test/probe_gates.sh's own reason: it holds
# that script's probe labels verbatim, so the same strings are scanned twice and
# an exemption that covered only one of the two files would be inconsistent by
# accident. One shipping label already names the churn band and another already
# carries a percentage; they are apart today only because the file is sorted and
# nobody chooses a future label's alphabetical neighbours.
EXEMPT_FILES = {SOURCE, "CLAUDE.md", "test/band_source_test.py",
                "test/probe_gates.sh", "test/PROBES_EXPECTED"}
EXEMPT_PREFIXES = ("docs/adr/", "docs/ideas/")

# A band figure is a percentage. Ranges are written with a hyphen or an en dash
# depending on whether the file is prose or code, so both are matched.
PERCENT = re.compile(r"\d+(?:\.\d+)?\s*(?:[-–—]\s*\d+(?:\.\d+)?)?\s*%")

# The words that turn a percentage in a comment into a claim about a band.
#
# WHAT IS DELIBERATELY NOT CHECKED: the bare digits. An earlier version of this
# also banned soc/bands.py's own figure spellings wherever they appeared, and it
# went red on rtl/memory.v recording that a nested spelling of its write/read
# arms costs 3.6% of Fmax -- a real measurement of the design that happens to
# round to the same number as the churn band. Banning the digits bans arithmetic.
# A percentage only claims to be a band when it is written beside one of these
# words, and a copy that does not say what it is a band of is not one a reader
# would rely on either.
BAND_WORD = re.compile(r"churn|spread", re.IGNORECASE)


def tracked(root):
    out = subprocess.run(["git", "-C", root, "ls-files"], check=True,
                         capture_output=True, text=True).stdout
    return [line for line in out.splitlines() if line]


def scanned(root):
    for path in tracked(root):
        if path in EXEMPT_FILES or path.startswith(EXEMPT_PREFIXES):
            continue
        full = os.path.join(root, path)
        try:
            with open(full, encoding="utf-8") as handle:
                yield path, handle.read().splitlines()
        except (OSError, UnicodeDecodeError):
            # A binary or unreadable tracked file states no band figure.
            continue


def spellings(band):
    """Each of soc/bands.py's figures, with every way it gets written down.

    Two figures, not a flat set: a range is hyphenated in code and en-dashed in
    prose, so ONE spelling of each has to be present rather than all of them.
    Requiring all of them would demand that CLAUDE.md write the range twice.
    """
    low, high = band["spread"]
    return {
        "churn band": {f"{band['churn']:g}%"},
        "placement spread": {f"{low:g}{dash}{high:g}%" for dash in "-–—"},
    }


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", default=os.path.dirname(here))
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    if not os.path.isdir(root):
        sys.exit(f"error: '{root}' is not a directory, so there is nothing to scan.")
    if not os.path.exists(os.path.join(root, SOURCE)):
        sys.exit(f"*** {SOURCE} is not in {root}, so the one source of the band\n"
                 "*** figures is gone and every copy of them is now unowned.")

    sys.dont_write_bytecode = True
    sys.path.insert(0, os.path.join(root, "soc"))
    import bands  # noqa: E402  -- resolved against the tree under test

    failures = []

    # ---- no band figure stated anywhere outside the source ----
    #
    # A three-line window rather than a line, because a wrapped comment puts the
    # word and the number on different lines and every one of the six copies this
    # replaced was wrapped that way.
    for path, lines in scanned(root):
        for number, line in enumerate(lines, 1):
            window = "\n".join(lines[max(0, number - 2):number + 1])
            if BAND_WORD.search(line) and PERCENT.search(window):
                failures.append(
                    f"{path}:{number} states a percentage beside 'churn' or "
                    f"'spread'.\n"
                    f"    Every band figure belongs to {SOURCE}; print it from "
                    f"there rather than\n"
                    f"    writing a number into a comment that cannot go red.")

    # ---- the rulebook's copy has to agree with the source ----
    claude = os.path.join(root, "CLAUDE.md")
    if not os.path.exists(claude):
        failures.append("CLAUDE.md is not there, so the rulebook's band figures "
                        "cannot be graded.")
    else:
        text = open(claude, encoding="utf-8").read()
        flat = text.replace(" ", "")
        if SOURCE not in text:
            failures.append(
                f"CLAUDE.md does not name {SOURCE}, so a reader is not told "
                f"where the\n    figures it quotes actually come from.")
        for part in bands.parts():
            try:
                band = bands.band(part)
            except bands.Underived:
                continue
            for figure, forms in sorted(spellings(band).items()):
                if not any(form in flat for form in forms):
                    failures.append(
                        f"CLAUDE.md states no {figure} for {part} matching "
                        f"{' or '.join(sorted(forms))},\n"
                        f"    which is {SOURCE}'s figure. The band was "
                        f"re-derived and the rulebook\n"
                        f"    still quotes the old one.")

    if failures:
        print("*** the band figures are stated in more than one place, or the")
        print("*** rulebook disagrees with the one that owns them:")
        for failure in failures:
            print(f"***   {failure}")
        sys.exit(1)
    print(f"band-source: every band figure is {SOURCE}'s, and CLAUDE.md agrees "
          f"with it.")


if __name__ == "__main__":
    main()
