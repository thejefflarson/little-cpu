#!/usr/bin/env python3
"""Grades test/CHECKS against the Makefile and against the files each check's recipe names.
Usage: checks_manifest_test.py [repo-root]

  1. The rows and the Makefile's TEST_CHECKS name the same checks, in both directions. A row the
     Makefile does not list must be a build step some listed row needs, so a check cannot be
     dropped from `make test-full`, or added to the Makefile and never scheduled, in silence.
  2. Every row resolves to a Makefile target (or carries its own command), and every input
     pattern matches a file, so a typo cannot declare an input nothing ever changes.
  3. Every tracked file a check's recipe names is claimed by its row, and for a row that does not
     key on every file (@all) so is every tracked file those scripts name in their own text.
  4. The forced-red sweeps are in the full tier only, which is the one placement `make test`
     is defined by.
"""
import concurrent.futures
import importlib.util
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location("checks", os.path.join(ROOT, "test", "checks.py"))
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)

FULL_ONLY = {"probe-gates", "nano-qspi-loop-probe"}
failures = []


def red(msg):
    print(f"  RED  {msg}", file=sys.stderr)
    failures.append(msg)


def make(*args):
    return subprocess.run(["make", "-s", *args], cwd=ROOT, capture_output=True, text=True)


def recipe_text(name):
    r = make("-n", "-B", name)
    return r.stdout if r.returncode == 0 else None


def main():
    rows = checks.load_manifest(os.path.join(ROOT, "test", "CHECKS"))
    by_name = {c.name: c for c in rows}
    listed = set(make("print-test-checks").stdout.split())
    if not listed:
        red("`make print-test-checks` printed nothing")

    needed = {d for c in rows for d in c.needs}
    for name in sorted(listed - set(by_name)):
        red(f"{name} is in the Makefile's TEST_CHECKS and has no row in test/CHECKS")
    for name in sorted(set(by_name) - listed):
        if name not in needed:
            red(f"{name} has a row in test/CHECKS, is not in TEST_CHECKS and no row needs it")

    for name in sorted(FULL_ONLY):
        if name not in by_name or by_name[name].tier != "full":
            red(f"{name} must be a full-tier row")
    for c in rows:
        if c.tier == "full" and c.name not in FULL_ONLY:
            red(f"{c.name} is full-tier but is not one of the forced-red sweeps {sorted(FULL_ONLY)}")

    files = sorted(checks.repo_files(ROOT))
    tracked = set(files)
    for c in rows:
        for pat in c.inputs:
            if not any(checks.claims(pat, f) for f in files):
                red(f"{c.name}: input {pat!r} matches no file")

    scheduled = [c for c in rows if not c.command]
    texts = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        for c, text in zip(scheduled, pool.map(lambda c: recipe_text(c.name), scheduled)):
            if text is None:
                red(f"{c.name}: `make -n {c.name}` fails, so it is not a Makefile target")
            else:
                texts[c.name] = text

    def named_files(text):
        out = set()
        for tok in re.findall(r"[A-Za-z0-9_./+-]+", text):
            tok = tok[2:] if tok.startswith("./") else tok
            if tok in tracked:
                out.add(tok)
        return out

    for c in scheduled:
        if c.name not in texts or c.always or not c.inputs:
            continue
        direct = named_files(texts[c.name])
        for f in sorted(direct):
            if not checks.claimed_by(c, f):
                red(f"{c.name}: its recipe names {f}, which no input pattern claims")
        if c.scan_all:
            continue
        for script in sorted(direct):
            if not script.endswith((".sh", ".py")):
                continue
            with open(os.path.join(ROOT, script), errors="replace") as fh:
                for f in sorted(named_files(fh.read()) - direct - {script}):
                    if not checks.claimed_by(c, f):
                        red(f"{c.name}: {script} names {f}, which no input pattern claims")

    if failures:
        print(f"checks-manifest: {len(failures)} problem(s)", file=sys.stderr)
        return 1
    print(f"checks-manifest: {len(rows)} rows, {len(listed)} listed checks, "
          f"{sum(1 for c in rows if c.tier == 'full')} full-tier only; "
          "the Makefile and test/CHECKS agree and every recipe's files are claimed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
