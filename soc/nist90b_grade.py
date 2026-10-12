#!/usr/bin/env python3
"""Grade NIST's SP 800-90B tools' reports for a one-bit-per-sample source.

  nist90b_grade.py --non-iid ea_non_iid.txt --restart ea_restart.txt [--min 0.5] [--label TEXT]

Reads `ea_non_iid -v` and `ea_restart -n -v` output and exits nonzero unless the assessment is
complete and holds: ea_non_iid printed H_original once, ea_restart printed one pass line and none
of its failure markers, its H_I is the H_original it was given, and neither H_original nor the
minimum of H_r, H_c and H_I that this script recomputes is under --min, the bits per sample
rtl/trng.v's health tests assume. A figure that is missing, repeated or unreadable fails: a tool
that crashed has not assessed anything.
"""

import argparse
import math
import re
import sys

ESTIMATE = re.compile(r"^\t(.+?) = ([0-9.]+) / (\d+) bit\(s\)$", re.M)
FAILURE_MARKERS = ("Restart Sanity Check Failed", "Validation Testing Failed")
TOLERANCE = 1.5e-6


def number(text, name, count, errors):
    """The one value `name: x` printed `count` times, or None with the reason appended to errors."""
    found = re.findall(r"^%s: (.*)$" % re.escape(name), text, re.M)
    if len(found) != count:
        errors.append("%s is printed %d times, not %d" % (name, len(found), count))
        return None
    try:
        values = [float(value) for value in found]
    except ValueError:
        errors.append("%s is not a number: %s" % (name, " / ".join(found)))
        return None
    if not all(math.isfinite(value) for value in values):
        errors.append("%s is not finite: %s" % (name, " / ".join(found)))
        return None
    if max(values) - min(values) > 0:
        errors.append("%s is printed with different values: %s" % (name, " / ".join(found)))
        return None
    return values[0]


def estimators(text, errors):
    found = []
    for name, value, _ in ESTIMATE.findall(text):
        try:
            found.append((name, float(value)))
        except ValueError:
            errors.append("%s is not a number: %s" % (name, value))
    return found


def parse_non_iid(text):
    errors = []
    has_bitstring = re.search(r"^H_bitstring:", text, re.M) is not None
    return {
        "estimators": estimators(text, errors),
        "h_original": number(text, "H_original", 1, errors),
        "h_bitstring": number(text, "H_bitstring", 1, errors) if has_bitstring else None,
        "errors": errors,
    }


def parse_restart(text):
    errors = []
    passes = text.count("Validation Test Passed")
    failed = [marker for marker in FAILURE_MARKERS if marker in text]
    if "Restart Sanity Check Failed" in failed:
        reason = "the sanity check failed (a row or column repeats a value more than the cutoff allows)"
    elif failed:
        reason = "min(H_r, H_c) fell under half of H_I"
    elif passes != 1:
        reason = "the tool printed no verdict" if passes == 0 else "the tool printed its pass line %d times" % passes
    else:
        reason = None
    return {
        "passed": passes == 1 and not failed,
        "reason": reason,
        "estimators": estimators(text, errors),
        "h_r": number(text, "H_r", 1, errors),
        "h_c": number(text, "H_c", 1, errors),
        "h_i": number(text, "H_I", 2, errors),
        "final": number(text, "min(H_r, H_c, H_I)", 1, errors),
        "errors": errors,
    }


def grade(non_iid, restart, floor):
    """(ok, report lines)."""
    lines, ok = [], True
    for name, value in non_iid["estimators"]:
        lines.append("  %-70s %.6f" % (name, value))
    lines.append("H_original: %s" % non_iid["h_original"])
    lines.append("H_bitstring: %s" % (non_iid["h_bitstring"] if non_iid["h_bitstring"] is not None
                                       else "not produced (one-bit samples are the bitstring)"))
    for message in non_iid["errors"]:
        lines.append("FAIL: ea_non_iid: " + message)
        ok = False
    if non_iid["h_original"] is not None:
        assessed = non_iid["h_original"]
        if non_iid["h_bitstring"] is not None:
            assessed = min(assessed, non_iid["h_bitstring"])
        if assessed < floor:
            lines.append("FAIL: %.6f bit/sample is under the %g bit rtl/trng.v assumes" % (assessed, floor))
            ok = False
    if not restart["passed"]:
        lines.append("restart test: FAILED, %s" % restart["reason"])
        return False, lines
    for message in restart["errors"]:
        lines.append("FAIL: ea_restart: " + message)
        ok = False
    figures = [restart["h_r"], restart["h_c"], restart["h_i"]]
    if None in figures or restart["final"] is None:
        return False, lines
    own = min(figures)
    lines.append("restart test: passed, H_r %s, H_c %s, H_I %s, min(H_r, H_c, H_I) %s"
                 % (restart["h_r"], restart["h_c"], restart["h_i"], restart["final"]))
    if abs(own - restart["final"]) > TOLERANCE:
        lines.append("FAIL: ea_restart printed min %s but its figures give %.6f" % (restart["final"], own))
        ok = False
    if non_iid["h_original"] is not None and abs(restart["h_i"] - min(non_iid["h_original"], 1.0)) > TOLERANCE:
        lines.append("FAIL: ea_restart was given H_I %s, not ea_non_iid's H_original %s"
                     % (restart["h_i"], non_iid["h_original"]))
        ok = False
    if own < floor:
        lines.append("FAIL: the restart tests give %.6f bit/sample, under the %g bit rtl/trng.v assumes"
                     % (own, floor))
        ok = False
    return ok, lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--non-iid", required=True)
    parser.add_argument("--restart", required=True)
    parser.add_argument("--min", type=float, default=0.5, dest="floor")
    parser.add_argument("--label", default="")
    args = parser.parse_args()
    with open(args.non_iid) as handle:
        non_iid = parse_non_iid(handle.read())
    with open(args.restart) as handle:
        restart = parse_restart(handle.read())
    ok, lines = grade(non_iid, restart, args.floor)
    print("SP 800-90B assessment%s" % (": " + args.label if args.label else ""))
    print("\n".join(lines))
    print("%s: min-entropy per sample %s %g" % ("PASS" if ok else "FAIL", ">=" if ok else "under", args.floor))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
