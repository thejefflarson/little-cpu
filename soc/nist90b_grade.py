#!/usr/bin/env python3
"""Grade NIST's SP 800-90B tools' reports for a one-bit-per-sample source.

  nist90b_grade.py --non-iid ea_non_iid.txt --restart ea_restart.txt [--min 0.5] [--label TEXT]

Reads `ea_non_iid -v` and `ea_restart -n -v` output and exits nonzero unless the assessment is
complete and holds: ea_non_iid printed H_original, ea_restart printed that its sanity check and
validation test passed, and neither H_original nor the restart tests' min(H_r, H_c, H_I) is under
--min, the bits per sample rtl/trng.v's health tests assume. A report that is missing a figure
fails: a tool that crashed has not assessed anything.
"""

import argparse
import re
import sys

ESTIMATE = re.compile(r"^\t(.+?) = ([0-9.]+) / (\d+) bit\(s\)$", re.M)


def number(text, name):
    found = re.search(r"^%s: ([0-9.]+)$" % re.escape(name), text, re.M)
    return float(found.group(1)) if found else None


def parse_non_iid(text):
    return {
        "estimators": [(name, float(value)) for name, value, _ in ESTIMATE.findall(text)],
        "h_original": number(text, "H_original"),
        "h_bitstring": number(text, "H_bitstring"),
    }


def parse_restart(text):
    passed = "Validation Test Passed" in text
    if "Restart Sanity Check Failed" in text:
        reason = "the sanity check failed (a row or column repeats a value more than the cutoff allows)"
    elif "Validation Testing Failed" in text:
        reason = "min(H_r, H_c) fell under half of H_I"
    elif not passed:
        reason = "the tool printed no verdict"
    else:
        reason = None
    return {
        "passed": passed,
        "reason": reason,
        "estimators": [(name, float(value)) for name, value, _ in ESTIMATE.findall(text)],
        "h_r": number(text, "H_r"),
        "h_c": number(text, "H_c"),
        "h_i": number(text, "H_I"),
        "final": number(text, "min(H_r, H_c, H_I)"),
    }


def grade(non_iid, restart, floor):
    """(ok, report lines)."""
    lines, ok = [], True
    for name, value in non_iid["estimators"]:
        lines.append("  %-70s %.6f" % (name, value))
    lines.append("H_original: %s" % non_iid["h_original"])
    lines.append("H_bitstring: %s" % (non_iid["h_bitstring"] if non_iid["h_bitstring"] is not None
                                       else "not produced (one-bit samples are the bitstring)"))
    if non_iid["h_original"] is None:
        lines.append("FAIL: ea_non_iid printed no H_original")
        ok = False
    else:
        assessed = non_iid["h_original"]
        if non_iid["h_bitstring"] is not None:
            assessed = min(assessed, non_iid["h_bitstring"])
        if assessed < floor:
            lines.append("FAIL: %.6f bit/sample is under the %g bit rtl/trng.v assumes" % (assessed, floor))
            ok = False
    if not restart["passed"]:
        lines.append("restart test: FAILED, %s" % restart["reason"])
        ok = False
    else:
        lines.append("restart test: passed, H_r %s, H_c %s, H_I %s, min(H_r, H_c, H_I) %s"
                     % (restart["h_r"], restart["h_c"], restart["h_i"], restart["final"]))
        if restart["final"] is None:
            lines.append("FAIL: ea_restart printed no min(H_r, H_c, H_I)")
            ok = False
        elif restart["final"] < floor:
            lines.append("FAIL: the restart tests give %.6f bit/sample, under the %g bit rtl/trng.v assumes"
                         % (restart["final"], floor))
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
