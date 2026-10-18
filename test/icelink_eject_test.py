#!/usr/bin/env python3
"""Grades that the iCELink volume is ejected before every board command.

soc/icelink_eject.sh is run against a fake `mount` and `diskutil`, and the Makefile and soc/ are scanned:
every loader or board-read invocation must have an eject ahead of it in its recipe (Makefile) or its
script (soc/).

Usage: icelink_eject_test.py [repo-root]     # defaults to this script's parent
"""

import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
MARK = re.compile(r"ICESUGAR_EJECT|icelink_eject\.sh|eject_icelink\(")
MAKE_USE = re.compile(r"\$\(ICESUGAR_LOAD\)|soc/board_read\.py|soc/entropy_assess\.py|soc/entropy_stream\.py"
                      r"|soc/entropy_nist\.py")
SOC_USE = re.compile(r"python3 soc/board_read\.py|\"\$LOADER\"|\"cmsisdap\"|read_frames\(args")
RESET = re.compile(r"^(define\b|[^\s#=][^=]*:($|[^=]))")


def make_violations(text):
    bad, ejected = [], False
    for number, line in enumerate(text.splitlines(), 1):
        if RESET.match(line):
            ejected = False
        if MARK.search(line):
            ejected = True
        elif MAKE_USE.search(line) and line[:1] in ("\t", "@") and not ejected:
            bad.append("Makefile:%d" % number)
    return bad


def script_violations(root):
    bad = []
    soc = os.path.join(root, "soc")
    for name in sorted(os.listdir(soc)):
        if not name.endswith((".sh", ".py")) or name == "icelink_eject.sh":
            continue
        with open(os.path.join(soc, name)) as handle:
            lines = handle.read().splitlines()
        use = [i for i, l in enumerate(lines) if SOC_USE.search(l) and not l.lstrip().startswith("#")]
        if not use:
            continue
        mark = [i for i, l in enumerate(lines) if MARK.search(l)]
        if not mark or (name.endswith(".sh") and mark[0] > use[0]):
            bad.append("soc/%s:%d" % (name, use[0] + 1))
    return bad


def helper_cases(root, expect):
    helper = os.path.join(root, "soc", "icelink_eject.sh")
    with tempfile.TemporaryDirectory() as scratch:
        log = os.path.join(scratch, "calls")
        for tool, body in (("mount", 'printf "%s\\n" "$FAKE_MOUNT"'),
                           ("diskutil", 'echo "$@" >> "%s"; sleep ${FAKE_DISKUTIL_SLEEP:-0}; exit ${FAKE_DISKUTIL_STATUS:-0}' % log)):
            path = os.path.join(scratch, tool)
            with open(path, "w") as handle:
                handle.write("#!/bin/bash\n%s\n" % body)
            os.chmod(path, 0o755)

        def run(mounts, status="0", sleep=None):
            if os.path.exists(log):
                os.remove(log)
            env = dict(os.environ, PATH=scratch + os.pathsep + os.environ["PATH"], FAKE_MOUNT=mounts,
                       FAKE_DISKUTIL_STATUS=status,
                       ICELINK_EJECT_S="1")
            if sleep:
                env["FAKE_DISKUTIL_SLEEP"] = sleep
            try:
                done = subprocess.run([helper], env=env, capture_output=True, text=True, timeout=15)
            except subprocess.TimeoutExpired:
                return None, []
            calls = open(log).read().split("\n") if os.path.exists(log) else []
            return done, [c for c in calls if c]

        done, calls = run("/dev/disk4s1 on /Volumes/iCELink (msdos, local)")
        expect("a mounted volume is ejected by its exact mount point", calls == ["eject /Volumes/iCELink"] and done.returncode == 0)
        done, calls = run("/dev/disk4s1 on /Volumes/iCELink2 (msdos, local)\n/dev/disk5 on /Volumes/x/Volumes/iCELink (apfs)")
        expect("a different volume sharing the prefix is left alone", calls == [] and done.returncode == 0)
        done, calls = run("")
        expect("an absent volume is a no-op", calls == [] and done.returncode == 0 and not done.stderr)
        done, calls = run("/dev/disk4s1 on /Volumes/iCELink (msdos, local)", status="1")
        expect("a failed eject warns and does not fail", done.returncode == 0 and "warning" in done.stderr)
        done, calls = run("/dev/disk4s1 on /Volumes/iCELink (msdos, local)", sleep="60")
        expect("a diskutil that stalls past the limit does not hang the helper, which warns and exits 0",
               done is not None and done.returncode == 0 and "Finder may hang" in done.stderr
               and "Disk Utility" in done.stderr)


def wrapper_case(root, expect):
    with tempfile.TemporaryDirectory() as scratch:
        stub = os.path.join(scratch, "slow.sh")
        with open(stub, "w") as handle:
            handle.write("#!/bin/bash\nsleep 30\n")
        os.chmod(stub, 0o755)
        sys.path.insert(0, os.path.join(root, "soc"))
        try:
            import board_port
            for what, helper in (("a hung helper", stub), ("a missing helper", os.path.join(scratch, "none"))):
                try:
                    board_port.eject_icelink(helper, timeout=1)
                    returned = True
                except Exception:
                    returned = False
                expect("the Python wrapper returns when %s" % what, returned)
        finally:
            sys.path.remove(os.path.join(root, "soc"))
            sys.modules.pop("board_port", None)
    for name in ("entropy_stream.py", "entropy_assess.py"):
        with open(os.path.join(root, "soc", name)) as handle:
            text = handle.read()
        expect("%s ejects through the shared wrapper" % name,
               "board_port.eject_icelink()" in text and "icelink_eject.sh" not in text)


def check(root):
    errors = []

    def expect(what, ok):
        if not ok:
            errors.append(what)

    helper_cases(root, expect)
    wrapper_case(root, expect)
    with open(os.path.join(root, "Makefile")) as handle:
        make = handle.read()
    with open(os.path.join(root, "mk", "nist90b.mk")) as handle:
        make += "\n" + handle.read()
    expect("every Makefile loader or board read has an eject first (unejected: %s)" % make_violations(make)[:3],
           not make_violations(make))
    expect("every soc/ script that loads or reads the board ejects first (unejected: %s)" % script_violations(root)[:3],
           not script_violations(root))
    expect("the grader sees the Makefile's board commands", len(MAKE_USE.findall(make)) >= 8)
    if errors:
        for message in errors:
            sys.stderr.write("error: %s\n" % message)
        return 1
    print("icelink-eject: every board command ejects the iCELink volume first.")
    return 0


if __name__ == "__main__":
    sys.exit(check(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(HERE)))
