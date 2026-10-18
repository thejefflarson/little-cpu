#!/usr/bin/env python3
"""Grades two board-run guarantees by reading the sources that carry them.

soc/seed_probe.S must latch mcycle once, at the first ES16 read, and print it on every word line so a
read opened after the run starts still recovers it. mk/nist90b.mk must default ENTROPY_NIST_LOCK to a
path shared by every worktree and hand it to soc/entropy_nist.py, so an 85-minute capture holds the
board lock.

Usage: seed_probe_test.py [repo-root]     # defaults to this script's parent
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def read(root, *parts):
    with open(os.path.join(root, *parts)) as handle:
        return handle.read()


def instructions(text):
    lines = (re.sub(r"\s+", " ", re.sub(r"\s*(#|//).*", "", line)).strip() for line in text.splitlines())
    return [line for line in lines if line]


def probe_errors(text):
    code = instructions(text)
    errors = []
    latches = [i for i, line in enumerate(code) if re.fullmatch(r"csrr\s+s5,\s*mcycle", line)]
    if len(latches) != 1:
        return ["seed_probe.S latches mcycle into s5 %d times, wanted once" % len(latches)]
    at = latches[0]
    if code[at - 1] != "bnez s5, same":
        errors.append("the latch is not guarded by a test that s5 is still zero")
    if not any(re.fullmatch(r"li\s+t0,\s*2", line) for line in code[max(at - 4, 0):at]):
        errors.append("the latch is not reached only on an ES16 status")
    word = [i for i, line in enumerate(code) if line == "li a0, 'W'"]
    if len(word) != 1:
        return errors + ["seed_probe.S has %d word lines, wanted one" % len(word)]
    tail = code[word[0]:]
    end = next((i for i, line in enumerate(tail) if line == "j poll"), len(tail))
    if "mv a0, s5" not in tail[:end] or "jal puthex" not in tail[:end]:
        errors.append("the word line does not print the first-ES16 cycle")
    return errors


def make_errors(text):
    errors = []
    default = re.search(r"^ENTROPY_NIST_LOCK[ \t]*\?=[ \t]*(\S.*)?$", text, re.M)
    if not default or not default.group(1):
        errors.append("ENTROPY_NIST_LOCK has no default, so the board run holds no lock")
    elif "TOOL_CACHE" not in default.group(1):
        errors.append("ENTROPY_NIST_LOCK defaults to %r, not a path every worktree shares" % default.group(1))
    run = re.search(r"define entropy_nist_run\n(.*?)\nendef", text, re.S)
    if not run or not re.search(r"--lock\s+'\$\(ENTROPY_NIST_LOCK\)'", run.group(1)):
        errors.append("entropy_nist_run does not pass --lock to soc/entropy_nist.py")
    elif "$(if $(ENTROPY_NIST_LOCK)" in run.group(1):
        errors.append("entropy_nist_run drops --lock whenever the variable is empty")
    return errors


def check(root):
    errors = probe_errors(read(root, "soc", "seed_probe.S")) + make_errors(read(root, "mk", "nist90b.mk"))
    for message in errors:
        sys.stderr.write("error: %s\n" % message)
    if errors:
        return 1
    print("seed-probe: the probe latches and prints the first ES16 cycle; the NIST run takes the board lock.")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)))
