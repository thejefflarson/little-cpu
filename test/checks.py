#!/usr/bin/env python3
"""Runs the checks test/CHECKS names, in parallel, skipping the ones whose inputs have not moved.

    checks.py --tier fast|full [--jobs N] [--no-skip] [--plan] [--manifest F] [--state DIR]

`fast` runs the rows marked fast; `full` runs every row. Format and rationale:
docs/manifests/checks.md.

A check is skipped only when all of these hold: skipping is on (not --no-skip), the tier has a
green snapshot, no file changed since that snapshot that no row claims, and the check's own
stamp equals the hash of the files its row declares. Any doubt runs the check.
"""
import argparse
import fnmatch
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

TIERS = ("fast", "full")
INTERRUPT_GRACE_S = 10
SLOTS_TOKEN = re.compile(r"!slots=(\d+|[A-Z_][A-Z0-9_]*(:\d+)?)$")
TOOL_PROBES = (
    ["yosys", "-V"],
    ["iverilog", "-V"],
    ["riscv-none-elf-gcc", "--version"],
    ["clang++", "--version"],
    ["python3", "--version"],
)


class Check:
    def __init__(self, name, tier, lane, needs, tokens, command, row):
        self.name = name
        self.tier = tier
        self.lane = lane
        self.needs = needs
        self.always = "!always" in tokens
        self.scan_all = "@all" in tokens
        self.slots = [t[len("!slots="):] for t in tokens if t.startswith("!slots=")]
        self.inputs = [t for t in tokens if t[0] not in "!@"]
        self.command = command
        self.row = row


def weight(check, jobs):
    """How many of the `jobs` slots a check holds while it runs.

    `!slots=N` is a count; `!slots=NAME[:N]` reads the environment variable NAME (else N, else 1),
    so a check that fans out into NAME children is charged for them. A weight above `jobs` is
    clamped to `jobs`: the check then waits for an idle machine and runs alone, never forever.
    """
    if not check.slots:
        return 1
    spec = check.slots[0]
    name, _, default = spec.partition(":")
    raw = name if name.isdigit() else os.environ.get(name) or default or "1"
    try:
        w = int(raw)
    except ValueError:
        w = 1
    return max(1, min(w, jobs))


def die(msg):
    print(f"checks: {msg}", file=sys.stderr)
    sys.exit(2)


def load_manifest(path):
    checks = []
    seen = set()
    with open(path) as f:
        for n, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            cols = [c.strip() for c in line.split("|")]
            if len(cols) not in (5, 6):
                die(f"{path}:{n}: want name|tier|lane|needs|inputs[|command], got {len(cols)} columns")
            name, tier, lane, needs, inputs = cols[:5]
            command = cols[5] if len(cols) == 6 and cols[5] else None
            if tier not in TIERS:
                die(f"{path}:{n}: tier {tier!r} is not one of {TIERS}")
            if name in seen:
                die(f"{path}:{n}: {name} is listed twice")
            seen.add(name)
            tokens = inputs.split()
            if not tokens:
                die(f"{path}:{n}: {name} declares no inputs; write !always to say it always runs")
            for t in tokens:
                if t[0] in "!@" and t not in ("!always", "@all") and not SLOTS_TOKEN.match(t):
                    die(f"{path}:{n}: {t} is not !always, @all or !slots=N|NAME[:N]")
            checks.append(Check(
                name, tier,
                name if lane == "-" else lane,
                [] if needs == "-" else needs.split(","),
                tokens, command, line))
    by_name = {c.name: c for c in checks}
    for c in checks:
        for dep in c.needs:
            if dep not in by_name:
                die(f"{c.name} needs {dep}, which no row names")
            if c.tier == "fast" and by_name[dep].tier != "fast":
                die(f"{c.name} is fast but needs {dep}, which is full-tier only")
    return checks


def repo_files(root):
    out = subprocess.run(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
        cwd=root, check=True, capture_output=True).stdout
    names = sorted(set(p for p in out.decode().split("\0") if p))
    hashes = {}
    for p in names:
        full = os.path.join(root, p)
        if not os.path.isfile(full):
            continue
        with open(full, "rb") as f:
            hashes[p] = hashlib.sha256(f.read()).hexdigest()
    return hashes


def claims(pattern, path):
    if pattern.endswith("/"):
        return path.startswith(pattern)
    return fnmatch.fnmatchcase(path, pattern)


def claimed_by(check, path):
    return any(claims(p, path) for p in check.inputs)


def tool_probes():
    """The tools whose versions key a stamp, plus any executables CHECKS_TOOLS names.

    CHECKS_TOOLS is a PATH-style list that exists so a test can stand in a tool whose version
    it controls; the OSS CAD Suite's python launcher puts its own bin first on PATH, so a fake
    put on PATH is never the one found.
    """
    extra = [[t, "--version"] for t in os.environ.get("CHECKS_TOOLS", "").split(os.pathsep) if t]
    return list(TOOL_PROBES) + extra


def tool_key():
    parts = []
    for argv in tool_probes():
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=30)
            first = (r.stdout or r.stderr).strip().splitlines()
            parts.append(f"{argv[0]}={first[0] if first else ''}")
        except (OSError, subprocess.SubprocessError):
            parts.append(f"{argv[0]}=absent")
    return "\n".join(parts)


def check_key(check, hashes, global_key):
    h = hashlib.sha256()
    h.update(global_key.encode())
    h.update(b"\0" + check.row.encode())
    for p in sorted(hashes):
        if check.scan_all or claimed_by(check, p):
            h.update(f"\0{p}\0{hashes[p]}".encode())
    return h.hexdigest()


def read_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def write_json(path, obj):
    tmp = f"{path}.{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(obj, f, sort_keys=True, indent=0)
    os.replace(tmp, path)


def child_env():
    env = dict(os.environ)
    flags = env.get("MAKEFLAGS", "")
    flags = re.sub(r"--jobserver-(auth|fds)=\S+", "", flags)
    flags = re.sub(r"(^| )-j\d*\b", " ", flags)
    env["MAKEFLAGS"] = flags.strip()
    env.pop("MFLAGS", None)
    # The OSS CAD Suite's python launcher exports these for its own interpreter. A check that
    # runs a different python3 (a probe with a minimal PATH) would fail to start under them.
    for var in ("PYTHONHOME", "PYTHONEXECUTABLE"):
        env.pop(var, None)
    return env


def killpg(proc, sig):
    """Signals a check's whole process group; its children are its own session's."""
    try:
        os.killpg(proc.pid, sig)
    except OSError:
        pass


def decide(selected, all_checks, hashes, state, tier, no_skip, global_key):
    """Returns (plan, keys, epoch, headline); plan maps a name to why it runs, or None to skip.

    The epoch names the reason every stamp is suspect. It is "cold" before the tier has had a
    green run, a hash of the changed files no row claims while some exist, and "" otherwise. A
    stamp is trusted when its key matches and, if the epoch is not empty, it was taken in this
    same epoch: that is what lets a run that failed halfway be resumed without repeating the
    checks that passed, and still re-run them all when the unclaimed file changes again.
    """
    snap = read_json(os.path.join(state, f"files-{tier}.json"), None)
    keys = {c.name: check_key(c, hashes, global_key) for c in selected}
    unclaimed = []
    if snap is None:
        epoch, headline = "cold", "no green run to compare against"
    else:
        moved = sorted(p for p in set(snap) | set(hashes) if snap.get(p) != hashes.get(p))
        unclaimed = [p for p in moved if not any(claimed_by(c, p) for c in all_checks)]
        if unclaimed:
            h = hashlib.sha256()
            for p in unclaimed:
                h.update(f"\0{p}\0{hashes.get(p, '')}".encode())
            epoch = h.hexdigest()
            shown = ", ".join(unclaimed[:3]) + (" ..." if len(unclaimed) > 3 else "")
            headline = f"{shown} changed and no check claims it"
        else:
            epoch, headline = "", "incremental"
    if no_skip:
        return {c.name: "skipping is off" for c in selected}, keys, epoch, "skipping is off"
    plan = {}
    for c in selected:
        stamp = read_json(os.path.join(state, "stamps", c.name), {})
        trusted = (not c.always and stamp.get("key") == keys[c.name]
                   and (not epoch or stamp.get("epoch") == epoch))
        plan[c.name] = None if trusted else (
            "it is marked !always" if c.always else headline if epoch else "its inputs changed")
    return plan, keys, epoch, headline


def run_checks(root, state, selected, plan, keys, epoch, jobs, verbose):
    logs = os.path.join(state, "logs")
    stamps = os.path.join(state, "stamps")
    os.makedirs(logs, exist_ok=True)
    os.makedirs(stamps, exist_ok=True)
    times_path = os.path.join(state, "times.json")
    times = read_json(times_path, {})
    todo = {c.name: c for c in selected if plan[c.name] is not None}
    done = {c.name for c in selected if plan[c.name] is None}
    failed = {}
    results = {}
    running = {}
    busy_lanes = set()
    used = 0
    procs = {}
    stopping = threading.Event()
    cond = threading.Condition()
    env = child_env()

    def execute(c):
        """Runs one check to completion and returns (exit status, seconds, its log)."""
        path = os.path.join(logs, c.name + ".log")
        argv = ["/bin/sh", "-c", c.command] if c.command else ["make", "-s", c.name]
        start = time.time()
        try:
            with open(path, "wb") as log:
                p = subprocess.Popen(argv, cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                     env=env, start_new_session=True)
                procs[c.name] = p
                if stopping.is_set():
                    killpg(p, signal.SIGTERM)
                rc = p.wait()
            with open(path, "rb") as f:
                body = f.read().decode(errors="replace").rstrip("\n")
        except OSError as e:
            rc, body = 127, f"checks: cannot run {c.name}: {e}"
        return rc, time.time() - start, body

    def worker(c):
        nonlocal used
        rc, secs, body = execute(c)
        with cond:
            print(f"{'ok  ' if rc == 0 else 'FAIL'} {c.name}  ({secs:.0f}s)", flush=True)
            if rc != 0 or verbose:
                print(body, flush=True)
            if rc != 0:
                print(f"---- {c.name} exited {rc}", flush=True)
                failed[c.name] = rc
            elif stopping.is_set():
                failed[c.name] = "interrupted"
            else:
                write_json(os.path.join(stamps, c.name),
                           {"key": keys[c.name], "epoch": epoch, "secs": round(secs, 1)})
                times[c.name] = round(secs, 1)
                done.add(c.name)
            results[c.name] = secs
            used -= weight(c, jobs)
            del running[c.name]
            busy_lanes.discard(c.lane)
            cond.notify_all()

    threads = []
    try:
        with cond:
            while todo or running:
                for c in [c for c in todo.values() if any(d in failed for d in c.needs)]:
                    del todo[c.name]
                    failed[c.name] = "needs"
                    print(f"SKIP {c.name}: a check it needs failed", flush=True)
                ready = [c for c in todo.values()
                         if c.lane not in busy_lanes and all(d in done for d in c.needs)]
                ready.sort(key=lambda c: -times.get(c.name, 0))
                for c in ready:
                    if used + weight(c, jobs) > jobs or c.lane in busy_lanes:
                        continue
                    del todo[c.name]
                    used += weight(c, jobs)
                    running[c.name] = c
                    busy_lanes.add(c.lane)
                    t = threading.Thread(target=worker, args=(c,), daemon=True)
                    threads.append(t)
                    t.start()
                if todo or running:
                    cond.wait(timeout=5.0)
    except KeyboardInterrupt:
        stopping.set()
        for p in list(procs.values()):
            killpg(p, signal.SIGTERM)
        deadline = time.time() + INTERRUPT_GRACE_S
        for t in threads:
            t.join(max(0.0, deadline - time.time()))
        for p in list(procs.values()):
            killpg(p, signal.SIGKILL)
        for t in threads:
            t.join()
        raise
    for t in threads:
        t.join()
    write_json(times_path, times)
    return failed, results


def main():
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tier", choices=TIERS, required=True)
    ap.add_argument("--jobs", type=int, default=int(os.environ.get("TEST_JOBS") or 0))
    ap.add_argument("--no-skip", action="store_true",
                    default=os.environ.get("TEST_NO_SKIP", "") not in ("", "0"))
    ap.add_argument("--plan", action="store_true", help="print each decision and run nothing")
    ap.add_argument("--manifest", default=None)
    ap.add_argument("--state", default=None)
    ap.add_argument("--root", default=os.getcwd())
    ap.add_argument("--only", default="", help="comma list: run just these rows")
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    manifest = args.manifest or os.path.join(root, "test", "CHECKS")
    state = args.state or os.path.join(root, os.environ.get("BUILD", "build"), "test-state")
    jobs = args.jobs or (os.cpu_count() or 2)

    checks = load_manifest(manifest)
    selected = [c for c in checks if args.tier == "full" or c.tier == "fast"]
    if args.only:
        wanted = set(args.only.split(","))
        selected = [c for c in selected if c.name in wanted]
    hashes = repo_files(root)
    os.makedirs(state, exist_ok=True)
    plan, keys, epoch, headline = decide(selected, checks, hashes, state, args.tier, args.no_skip, tool_key())

    n_run = sum(1 for v in plan.values() if v is not None)
    print(f"checks[{args.tier}]: {len(selected)} checks, {n_run} to run ({headline}), "
          f"{jobs} jobs", flush=True)
    if args.plan:
        for c in selected:
            print(f"{'run ' if plan[c.name] else 'skip'} {c.name}"
                  + (f"  ({plan[c.name]})" if plan[c.name] else ""))
        return 0

    t0 = time.time()
    try:
        failed, results = run_checks(root, state, selected, plan, keys, epoch, jobs,
                                     os.environ.get("TEST_VERBOSE", "") not in ("", "0"))
    except KeyboardInterrupt:
        print("checks: interrupted; running checks stopped, nothing unfinished stamped",
              file=sys.stderr, flush=True)
        return 130
    wall = time.time() - t0
    slow = sorted(results.items(), key=lambda kv: -kv[1])[:8]
    print(f"checks[{args.tier}]: {n_run} ran, {len(selected) - n_run} skipped, "
          f"{len(failed)} failed, {wall:.0f}s wall", flush=True)
    for name, secs in slow:
        print(f"  {secs:6.0f}s  {name}")
    if failed:
        print("FAILED: " + " ".join(sorted(failed)), file=sys.stderr)
        return 1
    write_json(os.path.join(state, f"files-{args.tier}.json"), hashes)
    return 0


if __name__ == "__main__":
    sys.exit(main())
