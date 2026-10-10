#!/usr/bin/env python3
"""Grade the three Hazard3 builds the comparison harness runs against their authors' own.

The harness runs Hazard3 in three named configurations: `hazard3` (the area build, from the
iCE40 example), `hazard3_perf` (the performance build, from the two ECP5 examples) and
`hazard3_c` (the performance build plus the C extension). The first two are builds its
authors shipped; the third is a harness choice that NO shipped example sets, which is why
it is graded as the performance column plus exactly one named difference rather than
against an example. That claim is three files agreeing, and this script checks each hop:

  1. soc/compare/bench_hazard3.v elaborates, at PERF=0, PERF=1 and PERF=1 WITH_C=1, the
     values in soc/compare/hazard3_builds.txt, for every parameter the file lists and no
     other.
  2. The parameters the bench lets PERF move are exactly the ones the area and performance
     columns differ on, and the ones it lets WITH_C move are exactly the ones the
     performance and C columns differ on, so neither switch can be a no-op or move
     something the columns agree on.
  3. The C column differs from the performance column in EXTENSION_C alone.
  4. The file names the Hazard3 SHA soc/compare/hazard3_pin.mk pins.
  5. When the pinned clone is present, the area and performance columns are what the
     clone's example files say, an unset parameter read at the core's own default.
     `--require-clone` makes an absent clone an error; `make test` runs without it and
     stays offline.

Usage: hazard3_config_test.py [repo-root] [--require-clone]
"""

import re
import sys
from pathlib import Path

# Written in the bench's core instance, but not a build choice: addresses and the
# interrupt count the harness wires itself.
NOT_A_BUILD_CHOICE = {"RESET_VECTOR", "MTVEC_INIT", "NUM_IRQS"}

# The one parameter hazard3_c changes from the performance build.
C_BUILD_DIFFERENCE = {"EXTENSION_C"}

AREA_EXAMPLE = "icebreaker"
PERF_EXAMPLES = ("ulx3s", "orangecrab_25f")


def fail(message):
    print(f"error: {message}", file=sys.stderr)
    sys.exit(1)


def read(path):
    try:
        return Path(path).read_text()
    except OSError as exc:
        fail(f"cannot read {path}: {exc}")


def read_verilog(path):
    """The file's text with `//` and `/* */` comments removed, so a commented-out setting
    is not read as live."""
    return re.sub(r"//[^\n]*|/\*.*?\*/", " ", read(path), flags=re.S)


def read_builds(path):
    pin = None
    builds = {}
    for line in read(path).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if fields[0] == "pin" and len(fields) == 2:
            pin = fields[1]
        elif len(fields) == 4 and all(f.isdigit() for f in fields[1:]):
            if fields[0] in builds:
                fail(f"{path} lists {fields[0]} twice")
            builds[fields[0]] = tuple(int(f) for f in fields[1:])
        else:
            fail(f"{path}: cannot read the line '{line}'")
    if pin is None:
        fail(f"{path} names no pin")
    if not builds:
        fail(f"{path} lists no parameter, so there is nothing to grade")
    return pin, builds


def read_bench(path):
    """Parameter -> (area, perf, perf+C) values as bench_hazard3.v's core instance states it,
    and the parameters PERF and WITH_C each move."""
    text = read_verilog(path)
    match = re.search(r"hazard3_cpu_2port\s*#\((.*?)\)\s*core\s*\(", text, re.S)
    if not match:
        fail(f"{path}: no 'hazard3_cpu_2port #(...) core (' instance to read")
    bench = {}
    moved = {"PERF": set(), "WITH_C": set()}
    for name, value in re.findall(r"\.(\w+)\s*\(([^()]*)\)", match.group(1)):
        value = value.strip()
        choice = re.fullmatch(r"(PERF|WITH_C)\s*\?\s*(\d+)\s*:\s*(\d+)", value)
        if re.fullmatch(r"\d+", value):
            bench[name] = (int(value),) * 3
        elif choice:
            switch, on, off = choice.group(1), int(choice.group(2)), int(choice.group(3))
            # WITH_C is only ever set together with PERF, so it moves the third column alone.
            bench[name] = (off, off, on) if switch == "WITH_C" else (off, on, on)
            moved[switch].add(name)
        elif name not in NOT_A_BUILD_CHOICE:
            fail(f"{path}: {name} is '{value}', which this check cannot read as a "
                 "number or a PERF or WITH_C choice")
    for switch in moved:
        if not re.search(rf"parameter\s+bit\s+{switch}\b", text):
            fail(f"{path} has no {switch} parameter")
    return bench, moved


def pinned_sha(path):
    match = re.search(r"HAZARD3_SHA\s*:=\s*([0-9a-f]{40})", read(path))
    if not match:
        fail(f"{path}: no HAZARD3_SHA to compare the build file's pin with")
    return match.group(1)


def core_defaults(clone):
    text = read_verilog(clone / "hdl" / "hazard3_config.vh")
    defaults = dict(re.findall(r"^parameter\s+(\w+)\s*=\s*(\d+)\s*,?", text, re.M))
    return {k: int(v) for k, v in defaults.items()}


def example_values(path, defaults, names):
    text = read_verilog(path)
    block = re.search(r"example_soc\s*#\((.*?)\)\s*soc_u\s*\(", text, re.S)
    if not block:
        fail(f"{path}: no 'example_soc #(...) soc_u (' instance to read")
    set_here = {}
    for name, value in re.findall(r"\.(\w+)\s*\(([^()]*)\)", block.group(1)):
        value = value.strip()
        if name in names and not re.fullmatch(r"\d+", value):
            fail(f"{path}: {name} is '{value}', which this check cannot read as a "
                 "decimal number; it would otherwise fall back to the core's default")
        if name in names:
            set_here[name] = int(value)
    values = {}
    for name in names:
        if name in set_here:
            values[name] = set_here[name]
        elif name in defaults:
            values[name] = defaults[name]
        else:
            fail(f"{path} does not set {name} and the core has no default for it")
    return values


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    require_clone = "--require-clone" in sys.argv[1:]
    repo = Path(args[0]) if args else Path(__file__).resolve().parents[2]
    here = repo / "soc" / "compare"

    pin, builds = read_builds(here / "hazard3_builds.txt")
    bench, moved = read_bench(here / "bench_hazard3.v")
    rc = 0

    def red(message):
        nonlocal rc
        print(f"error: {message}", file=sys.stderr)
        rc = 1

    if pin != pinned_sha(here / "hazard3_pin.mk"):
        red(f"hazard3_builds.txt pins {pin}, not the SHA hazard3_pin.mk pins")

    for name, want in sorted(builds.items()):
        if name not in bench:
            red(f"{name} is in hazard3_builds.txt and not in bench_hazard3.v's core instance")
            continue
        if bench[name] != want:
            red(f"{name}: bench_hazard3.v elaborates area={bench[name][0]} "
                f"perf={bench[name][1]} perf_c={bench[name][2]}, the build file says "
                f"area={want[0]} perf={want[1]} perf_c={want[2]}")
    for name in sorted(set(bench) - set(builds) - NOT_A_BUILD_CHOICE):
        red(f"{name} is set in bench_hazard3.v and graded against nothing in hazard3_builds.txt")

    perf_differ = {n for n, (area, perf, _) in builds.items() if area != perf}
    if not perf_differ:
        red("the area and performance columns of hazard3_builds.txt are identical, so PERF "
            "selects nothing")
    if moved["PERF"] != perf_differ:
        red("PERF moves " + (", ".join(sorted(moved["PERF"])) or "nothing") +
            " in bench_hazard3.v; the authors' builds differ on " +
            (", ".join(sorted(perf_differ)) or "nothing"))
    c_differ = {n for n, (_, perf, perf_c) in builds.items() if perf != perf_c}
    if c_differ != C_BUILD_DIFFERENCE:
        red("the C build differs from the performance build on " +
            (", ".join(sorted(c_differ)) or "nothing") + ", not on " +
            ", ".join(sorted(C_BUILD_DIFFERENCE)) + " alone")
    if moved["WITH_C"] != c_differ:
        red("WITH_C moves " + (", ".join(sorted(moved["WITH_C"])) or "nothing") +
            " in bench_hazard3.v; the C build differs from the performance build on " +
            (", ".join(sorted(c_differ)) or "nothing"))

    clone = here / "hazard3"
    if (clone / "example_soc").is_dir():
        defaults = core_defaults(clone)
        examples = clone / "example_soc" / "fpga"
        area = example_values(examples / f"fpga_{AREA_EXAMPLE}.v", defaults, builds)
        perfs = [example_values(examples / f"fpga_{n}.v", defaults, builds)
                 for n in PERF_EXAMPLES]
        if perfs[0] != perfs[1]:
            red("the pinned clone's two performance examples disagree on " +
                ", ".join(sorted(k for k in perfs[0] if perfs[0][k] != perfs[1][k])))
        for name, (want_area, want_perf, _) in sorted(builds.items()):
            if area[name] != want_area:
                red(f"{name}: hazard3_builds.txt area={want_area}, "
                    f"fpga_{AREA_EXAMPLE}.v says {area[name]}")
            if perfs[0][name] != want_perf:
                red(f"{name}: hazard3_builds.txt perf={want_perf}, "
                    f"fpga_{PERF_EXAMPLES[0]}.v says {perfs[0][name]}")
        note = ("and the pinned clone's examples (hazard3_c is a harness choice no example "
                "sets: checked as the performance build plus EXTENSION_C)")
    elif require_clone:
        red(f"{clone} is not present, so hazard3_builds.txt was not checked against the "
            "authors' files")
        note = ""
    else:
        note = "(pinned clone absent: not re-read against the authors' files)"

    if rc:
        sys.exit(rc)
    print(f"hazard3 builds: {len(builds)} parameters agree between bench_hazard3.v, "
          f"hazard3_builds.txt {note}".rstrip())


if __name__ == "__main__":
    main()
