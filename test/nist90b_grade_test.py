#!/usr/bin/env python3
"""Grades soc/nist90b_grade.py in both directions: the figures NIST's tools printed for the board must
pass, and a source under the 0.5 bit rtl/trng.v assumes, a failed restart test and a report with a
missing figure must each fail.

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

H_original: %(h_original)s
"""

RESTART = """H_I: 0.500299
\tCollision Test Estimate (Rows) = 0.500299 / 1 bit(s)
\tCollision Test Estimate (Cols) = 0.790722 / 1 bit(s)

H_r: %(h_r)s
H_c: 0.790722
H_I: 0.500299

%(verdict)s
"""

PASSED = "Validation Test Passed...\n\nmin(H_r, H_c, H_I): %(final)s\n"


def load(root):
    path = os.path.join(root, "soc", "nist90b_grade.py")
    spec = importlib.util.spec_from_file_location("nist90b_grade", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check(root):
    ng = load(root)
    errors = []

    def verdict(what, want, collision="0.500299", h_original="0.500299", restart=None, floor=0.5):
        non_iid = NON_IID % {"collision": collision, "h_original": h_original}
        if restart is None:
            restart = RESTART % {"h_r": "0.500299", "verdict": PASSED % {"final": "0.500299"}}
        ok, _ = ng.grade(ng.parse_non_iid(non_iid), ng.parse_restart(restart), floor)
        if ok != want:
            errors.append("%s: graded %s, wanted %s" % (what, ok, want))

    verdict("figures at the floor pass", True)
    verdict("a comfortable source passes", True, "0.71", "0.71")
    verdict("H_original just under the floor fails", False, "0.499999", "0.499999")
    verdict("a figure under the floor fails whatever the floor is", False, floor=0.6)
    verdict("a report with no H_original fails", False, h_original="")
    verdict("a failed sanity check fails", False,
            restart=RESTART % {"h_r": "0.5", "verdict": "*** Restart Sanity Check Failed ***\n"})
    verdict("a failed validation test fails", False,
            restart=RESTART % {"h_r": "0.2", "verdict": "*** min(H_r, H_c) < H_I/2, Validation Testing Failed ***\n"})
    verdict("a restart report with no verdict fails", False, restart=RESTART % {"h_r": "0.5", "verdict": ""})
    verdict("restart figures under the floor fail even when the test passed", False,
            restart=RESTART % {"h_r": "0.4", "verdict": PASSED % {"final": "0.4"}})

    parsed = ng.parse_non_iid(NON_IID % {"collision": "0.500299", "h_original": "0.500299"})
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
