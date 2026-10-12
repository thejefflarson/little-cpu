#!/usr/bin/env python3
"""Grades soc/nist90b_grade.py in both directions: the figures NIST's tools printed for the board must
pass, and a source under the 0.5 bit rtl/trng.v assumes, a failed or repeated or contradicted restart
verdict, and a report with a missing, repeated or unreadable figure must each fail.

Usage: nist90b_grade_test.py [repo-root]     # defaults to this script's parent
"""

import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# Lines as ea_non_iid -v and ea_restart -n -v print them (SP800-90B_EntropyAssessment 1.1.8).
NON_IID = """Opening file: 'x.bin'
Loaded 1000000 samples of 2 distinct 1-bit-wide symbols

Running Most Common Value Estimate...
\tMost Common Value Estimate = 0.940324 / 1 bit(s)
\tCollision Test Estimate = %(collision)s / 1 bit(s)
\tMarkov Test Estimate = 0.741577 / 1 bit(s)
\tCompression Test Estimate = 0.516933 / 1 bit(s)
\tLZ78Y Prediction Test Estimate = 0.775772 / 1 bit(s)

%(h_original)s%(extra)s"""

RESTART = """H_I: %(h_i)s
\tCollision Test Estimate (Rows) = 0.500299 / 1 bit(s)
\tCollision Test Estimate (Cols) = 0.790722 / 1 bit(s)

%(h_r)s%(h_c)sH_I: %(h_i2)s

%(verdict)s"""

PASSED = "Validation Test Passed...\n\nmin(H_r, H_c, H_I): %(final)s\n"


def load(root):
    path = os.path.join(root, "soc", "nist90b_grade.py")
    spec = importlib.util.spec_from_file_location("nist90b_grade", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def non_iid_text(h="0.500299", collision=None, extra="", line=True):
    return NON_IID % {"collision": collision or h, "extra": extra,
                      "h_original": "H_original: %s\n" % h if line else ""}


def restart_text(h_r="0.500299", h_c="0.790722", h_i="0.500299", h_i2=None, final="0.500299",
                 verdict=None, drop_h_c=False):
    return RESTART % {
        "h_i": h_i, "h_i2": h_i2 or h_i, "h_r": "H_r: %s\n" % h_r,
        "h_c": "" if drop_h_c else "H_c: %s\n" % h_c,
        "verdict": PASSED % {"final": final} if verdict is None else verdict}


def check(root):
    ng = load(root)
    errors = []

    def verdict(what, want, non_iid=None, restart=None, floor=0.5):
        try:
            ok, _ = ng.grade(ng.parse_non_iid(non_iid or non_iid_text()),
                             ng.parse_restart(restart or restart_text()), floor)
        except Exception as error:
            errors.append("%s: raised %r instead of grading" % (what, error))
            return
        if ok != want:
            errors.append("%s: graded %s, wanted %s" % (what, ok, want))

    verdict("figures at the floor pass", True)
    verdict("a comfortable source passes", True, non_iid_text("0.71"), restart_text("0.71", h_i="0.710000", final="0.71"))
    verdict("H_original just under the floor fails", False, non_iid_text("0.499999"),
            restart_text(h_i="0.499999", h_r="0.499999", final="0.499999"))
    verdict("a figure under the floor fails whatever the floor is", False, floor=0.6)
    verdict("a report with no H_original fails", False, non_iid_text(line=False))
    verdict("H_original printed twice with different values fails", False, non_iid_text(extra="H_original: 0.9\n"))
    verdict("H_original printed twice with the same value fails", False, non_iid_text(extra="H_original: 0.500299\n"))
    verdict("an unreadable H_original fails without a traceback", False, non_iid_text("0.5.1"))
    verdict("an infinite H_original fails", False, non_iid_text("inf"))
    verdict("a NaN H_original fails", False, non_iid_text("nan"))
    verdict("an H_bitstring line that cannot be read fails", False, non_iid_text(extra="H_bitstring: ??\n"))
    verdict("an H_bitstring under the floor fails", False, non_iid_text(extra="H_bitstring: 0.4\n"))
    verdict("a failed sanity check fails", False,
            restart=restart_text(verdict="*** Restart Sanity Check Failed ***\n"))
    verdict("a failed validation test fails", False,
            restart=restart_text(h_r="0.2", verdict="*** min(H_r, H_c) < H_I/2, Validation Testing Failed ***\n"))
    verdict("a restart report with no verdict fails", False, restart=restart_text(verdict=""))
    verdict("a pass line printed twice fails", False,
            restart=restart_text(verdict="Validation Test Passed...\n" + PASSED % {"final": "0.500299"}))
    verdict("a pass line beside a failure marker fails", False,
            restart=restart_text(verdict="*** Restart Sanity Check Failed ***\n" + PASSED % {"final": "0.500299"}))
    verdict("restart figures under the floor fail even when the test passed", False,
            restart=restart_text(h_r="0.4", final="0.4"))
    verdict("a printed minimum above the figures it was taken from fails", False, non_iid_text("0.6"),
            restart_text(h_r="0.7", h_i="0.6", final="0.9"))
    verdict("a restart report with no H_c fails", False, restart=restart_text(drop_h_c=True))
    verdict("a restart given a different H_I than ea_non_iid reported fails", False,
            restart=restart_text(h_i="0.6", final="0.6", h_r="0.7"))
    verdict("an H_I that changes inside the report fails", False, restart=restart_text(h_i2="0.600000"))

    parsed = ng.parse_non_iid(non_iid_text())
    if len(parsed["estimators"]) != 5 or parsed["h_original"] != 0.500299 or parsed["h_bitstring"] is not None:
        errors.append("ea_non_iid's estimators and H_original are read as printed: %s" % parsed)

    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("nist90b-grade: the report grader passes the board's figures and fails each way a report can be short.")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)))
