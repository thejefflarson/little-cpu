#!/usr/bin/env python3
"""Grades test/checks.py on a throwaway repository whose checks are shell one-liners.

Every case runs the real driver and counts how many times each check's command ran, so a
skip that should not happen and a run that should not happen both go red.
Usage: checks_test.py [path/to/checks.py]
"""
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(HERE, "checks.py")

MANIFEST = """\
# name | tier | lane | needs | inputs | command
alpha | fast | - | - | src/a.txt | echo x >> runs/alpha
beta  | fast | - | - | src/b.txt docs/ | echo x >> runs/beta
gamma | fast | - | - | @all | echo x >> runs/gamma
bld   | fast | - | - | !always | echo x >> runs/bld
delta | full | - | - | src/a.txt | echo x >> runs/delta
LANE1 | fast | shared | - | src/a.txt | echo s1 >> runs/order; sleep 1; echo e1 >> runs/order
LANE2 | fast | shared | - | src/a.txt | echo s2 >> runs/order; sleep 1; echo e2 >> runs/order
first | fast | - | - | src/a.txt | sleep 1; echo f >> runs/dep
after | fast | - | first | src/a.txt | echo a >> runs/dep
"""

failures = []


def check(cond, what):
    print(("  ok   " if cond else "  RED  ") + what)
    if not cond:
        failures.append(what)


def count(repo, name):
    p = os.path.join(repo, "runs", name)
    return len(open(p).read().split()) if os.path.exists(p) else 0


def reset_counts(repo):
    for f in os.listdir(os.path.join(repo, "runs")):
        os.remove(os.path.join(repo, "runs", f))


def driver(repo, *args, tier="fast", env=None):
    e = dict(os.environ)
    e.update(env or {})
    r = subprocess.run(
        [sys.executable, DRIVER, "--tier", tier, "--root", repo,
         "--manifest", os.path.join(repo, "test", "CHECKS"),
         "--state", os.path.join(repo, "state-x"), "--jobs", "4", *args],
        capture_output=True, text=True, env=e)
    return r


def make_repo(root):
    for d in ("src", "docs", "test", "runs", "other"):
        os.makedirs(os.path.join(root, d), exist_ok=True)
    for f, body in (("src/a.txt", "a1"), ("src/b.txt", "b1"), ("docs/n.md", "n1")):
        with open(os.path.join(root, f), "w") as fh:
            fh.write(body)
    with open(os.path.join(root, "test", "CHECKS"), "w") as fh:
        fh.write(MANIFEST)
    with open(os.path.join(root, ".gitignore"), "w") as fh:
        fh.write("runs/\nstate-x/\n")
    subprocess.run(["git", "init", "-q"], cwd=root, check=True)


def edit(repo, rel, body):
    with open(os.path.join(repo, rel), "w") as fh:
        fh.write(body)


def main():
    with tempfile.TemporaryDirectory(prefix="checks-test.") as repo:
        make_repo(repo)

        print("cold start")
        r = driver(repo)
        check(r.returncode == 0, "a cold run of green checks exits 0")
        check(all(count(repo, n) == 1 for n in ("alpha", "beta", "gamma")),
              "a cold run runs every fast check once")
        check(count(repo, "delta") == 0, "the fast tier leaves a full-tier check alone")

        print("warm start")
        reset_counts(repo)
        r = driver(repo)
        check(r.returncode == 0 and sum(count(repo, n) for n in ("alpha", "beta")) == 0,
              "an unchanged tree runs nothing that declares inputs")
        check(count(repo, "gamma") == 0, "a check declaring @all skips when nothing at all changed")
        check(count(repo, "bld") == 1, "a check marked !always runs every time")

        print("editing a declared input")
        reset_counts(repo)
        edit(repo, "src/b.txt", "b2")
        r = driver(repo)
        check(count(repo, "beta") == 1, "editing a file a check declares re-runs that check")
        check(count(repo, "alpha") == 0, "editing it leaves a check that does not declare it skipped")
        check(count(repo, "gamma") == 1, "editing any file re-runs a check declaring @all")

        print("editing under a directory input")
        reset_counts(repo)
        edit(repo, "docs/n.md", "n2")
        driver(repo)
        check(count(repo, "beta") == 1 and count(repo, "alpha") == 0,
              "a directory pattern claims every file under it")

        print("a file no check claims")
        reset_counts(repo)
        edit(repo, "other/stray.txt", "x")
        r = driver(repo)
        check(count(repo, "alpha") == 1 and count(repo, "beta") == 1,
              "a changed file no row claims runs every check")
        check("other/stray.txt" in r.stdout, "and the run names the unclaimed file")
        reset_counts(repo)
        driver(repo)
        check(count(repo, "alpha") == 0, "once that run is green the same tree skips again")

        print("--no-skip")
        reset_counts(repo)
        driver(repo, "--no-skip")
        check(count(repo, "alpha") == 1 and count(repo, "beta") == 1, "--no-skip runs everything")

        print("the full tier")
        reset_counts(repo)
        r = driver(repo, tier="full")
        check(count(repo, "delta") == 1 and count(repo, "alpha") == 1,
              "the full tier runs its own rows and the fast ones")
        reset_counts(repo)
        driver(repo, tier="full")
        check(count(repo, "delta") == 0, "and skips them on an unchanged tree")

        print("a failing check")
        edit(repo, "test/CHECKS", MANIFEST.replace("echo x >> runs/alpha", "echo x >> runs/alpha; exit 3"))
        reset_counts(repo)
        r = driver(repo)
        check(r.returncode == 1 and "alpha" in r.stderr, "a failing check fails the run and is named")
        check(count(repo, "beta") == 1, "the manifest is no check's input, so editing it ran everything")
        reset_counts(repo)
        r = driver(repo)
        check(count(repo, "alpha") == 1 and r.returncode == 1,
              "a failed check is not stamped, so it runs again")
        check(count(repo, "beta") == 0,
              "while a sibling that passed in the same epoch is not repeated")
        edit(repo, "other/stray.txt", "changed again")
        reset_counts(repo)
        driver(repo)
        check(count(repo, "beta") == 1, "a second change to the unclaimed file starts a new epoch")
        edit(repo, "test/CHECKS", MANIFEST)
        reset_counts(repo)
        r = driver(repo)
        check(r.returncode == 0, "restoring the manifest goes green")
        reset_counts(repo)
        driver(repo)
        check(count(repo, "beta") == 0 and count(repo, "alpha") == 0,
              "and the green run is skippable afterwards")

        print("a new toolchain")
        reset_counts(repo)
        driver(repo)
        reset_counts(repo)
        bindir = tempfile.mkdtemp(prefix="checks-test-bin.")
        with open(os.path.join(bindir, "yosys"), "w") as fh:
            fh.write("#!/bin/sh\necho Yosys fake-1\n")
        os.chmod(os.path.join(bindir, "yosys"), 0o755)
        env = {"PATH": bindir + os.pathsep + os.environ["PATH"]}
        driver(repo, env=env)
        check(count(repo, "alpha") == 1, "a changed tool version re-runs the checks")
        reset_counts(repo)
        driver(repo, env=env)
        check(count(repo, "alpha") == 0, "and the same tool version skips again")
        shutil.rmtree(bindir)

        print("lanes and ordering")
        reset_counts(repo)
        driver(repo, "--no-skip")
        order = open(os.path.join(repo, "runs", "order")).read().split()
        check(order in (["s1", "e1", "s2", "e2"], ["s2", "e2", "s1", "e1"]),
              "two checks in one lane never overlap")
        dep = open(os.path.join(repo, "runs", "dep")).read().split()
        check(dep == ["f", "a"], "a check runs after the one it needs")

        print("a manifest that lies")
        edit(repo, "test/CHECKS", MANIFEST + "bad | fast | - | nosuch | src/a.txt | true\n")
        r = driver(repo)
        check(r.returncode == 2 and "nosuch" in r.stderr, "a dependency no row names is refused")
        edit(repo, "test/CHECKS", MANIFEST + "alpha | fast | - | - | src/a.txt | true\n")
        r = driver(repo)
        check(r.returncode == 2 and "twice" in r.stderr, "a name listed twice is refused")
        edit(repo, "test/CHECKS", MANIFEST + "noin | fast | - | - |  | true\n")
        r = driver(repo)
        check(r.returncode == 2, "a row declaring no inputs is refused")
        edit(repo, "test/CHECKS", MANIFEST + "odd | fast | - | - | !alwayz | true\n")
        r = driver(repo)
        check(r.returncode == 2 and "!alwayz" in r.stderr, "an unknown flag is refused")

    if failures:
        print(f"checks-test: {len(failures)} case(s) red", file=sys.stderr)
        return 1
    print("checks-test: skipping, lanes, tiers and refusals all behave.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
