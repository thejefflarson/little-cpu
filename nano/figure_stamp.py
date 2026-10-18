#!/usr/bin/env python3
"""The one definition of a design stamp: the provenance that travels with a figure.

A figure printed for a design is followed by `[design . isa . target . flow . sha]`
(the separator is a middle dot, U+00B7, with a space either side):

    45,120 um2 [nanocpu · rv32ec_zicsr · sky130hd tt_025C_1v80 · yosys 0.69+158 · 1a2b3c4d5e6f]

design  one of DESIGNS
isa     the -march string the measured software or RTL configuration was built with
target  the part, corner, tile or simulated memory the figure describes
flow    the tool and version that produced it
sha     this tree's short commit, `+dirty` when the tree is not clean

The stamp has no `|`, so it survives a markdown table cell and a TSV column unchanged.

Every nano script that prints a figure routes it through `stamped`. Every tool that forms a
ratio, a product or a shared table calls `combine` first, which refuses rows whose design
(or isa, target or flow) differ and names the field. This file is the only place the
grammar and the design list are written; `test/figure_stamp_test.py` refuses a second copy.

CLI:
  figure_stamp.py check [FILE]       exit 1 naming every line that holds a figure and no valid stamp
  figure_stamp.py ratio A.json B.json  print A/B only if the two rows' stamps combine
"""

import json
import os
import re
import subprocess
import sys

DESIGNS = ("littlecpu", "nanocpu")
FIELDS = ("design", "isa", "target", "flow", "sha")
SEPARATOR = " · "
COMBINE_FIELDS = ("design", "isa", "target", "flow")
FIRST_STAMPED_ADR = 253

FIELD_TEXT = re.compile(r"[^\[\]·|\n]+")
SHA_TEXT = re.compile(r"(?:[0-9a-f]{7,40}(?:\+dirty)?|unrecorded)")
STAMP = re.compile(r"\[([^\[\]\n]*" + "·" + r"[^\[\]\n]*)\]")
FIGURE = re.compile(
    r"\d(?:[\d,.]*\d)?\s*(?:um2|ps|DMIPS/MHz|CoreMark/MHz|cycles)\b|"
    r"\b(?:DMIPS/MHz|CoreMark/MHz)\s*[:=]\s*\d"
)
MARCH = re.compile(r"-march=(\S+)")


class StampError(ValueError):
    pass


def validate(stamp):
    for name in FIELDS:
        if name not in stamp:
            raise StampError(f"stamp is missing field `{name}`")
    extra = sorted(set(stamp) - set(FIELDS))
    if extra:
        raise StampError(f"stamp carries unknown field `{extra[0]}`")
    if stamp["design"] not in DESIGNS:
        raise StampError(
            f"stamp field `design` is {stamp['design']!r}, not one of {', '.join(DESIGNS)}"
        )
    for name in ("isa", "target", "flow"):
        if not FIELD_TEXT.fullmatch(stamp[name].strip()) or stamp[name] != stamp[name].strip():
            raise StampError(f"stamp field `{name}` is {stamp[name]!r}, which is not plain text")
    if not SHA_TEXT.fullmatch(stamp["sha"]):
        raise StampError(f"stamp field `sha` is {stamp['sha']!r}, not a short commit id")
    return stamp


def render(stamp):
    validate(stamp)
    return "[" + SEPARATOR.join(stamp[name] for name in FIELDS) + "]"


def parse(text):
    """The stamp inside a bracket, or StampError naming what is wrong with it."""
    match = STAMP.search(text)
    if not match:
        raise StampError("no stamp bracket")
    parts = match.group(1).split(SEPARATOR)
    if len(parts) < len(FIELDS):
        leading = "design" if parts[0] not in DESIGNS else FIELDS[len(parts)]
        raise StampError(f"stamp is missing field `{leading}`")
    if len(parts) > len(FIELDS):
        raise StampError(f"stamp has {len(parts)} fields, expected {len(FIELDS)}")
    return validate(dict(zip(FIELDS, parts)))


def stamped(text, stamp):
    return f"{text} {render(stamp)}"


def isa_from_cflags(cflags):
    match = MARCH.search(cflags or "")
    if not match:
        raise StampError(f"no -march= in the build flags {cflags!r}, so no isa to stamp")
    return match.group(1)


def _run(cmd):
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    return done.stdout if done.returncode == 0 else None


def flow_version():
    forced = os.environ.get("FIGURE_FLOW")
    if forced:
        return forced
    out = _run(["yosys", "-V"])
    first = out.splitlines()[0].split(" (")[0].strip() if out and out.strip() else ""
    return first.replace("Yosys", "yosys", 1) if first else "yosys unknown"


def tree_sha():
    forced = os.environ.get("FIGURE_SHA")
    if forced:
        return forced
    here = os.path.dirname(os.path.abspath(__file__))
    sha = _run(["git", "-C", here, "rev-parse", "--short=12", "HEAD"])
    if not sha:
        return "unrecorded"
    dirty = _run(["git", "-C", here, "status", "--porcelain", "--untracked-files=no"])
    return sha.strip() + ("+dirty" if dirty and dirty.strip() else "")


def nano_stamp(cflags, target, flow=None, sha=None):
    return validate({
        "design": "nanocpu",
        "isa": isa_from_cflags(cflags),
        "target": target,
        "flow": flow or flow_version(),
        "sha": sha or tree_sha(),
    })


def combine(rows):
    """Refuse rows that do not describe one design on one target. Returns the common stamp
    fields; raises StampError naming the field it refused on."""
    if len(rows) < 2:
        raise StampError("combining needs at least two rows")
    for row in rows:
        validate(row)
    first = rows[0]
    for row in rows[1:]:
        for name in COMBINE_FIELDS:
            if row[name] != first[name]:
                raise StampError(
                    f"refusing to combine rows whose `{name}` differs: "
                    f"{first[name]!r} against {row[name]!r}"
                )
    return {name: first[name] for name in COMBINE_FIELDS}


def unstamped_figures(lines):
    """(line number, reason) for each line that holds a figure and no valid stamp."""
    bad = []
    for number, line in enumerate(lines, 1):
        if not FIGURE.search(line):
            continue
        try:
            parse(line)
        except StampError as exc:
            bad.append((number, str(exc)))
    return bad


def _load_row(path):
    with open(path) as handle:
        row = json.load(handle)
    return float(row["value"]), validate(row["stamp"])


def main(argv):
    if len(argv) >= 2 and argv[1] == "check":
        handle = open(argv[2]) if len(argv) > 2 else sys.stdin
        lines = handle.read().splitlines()
        bad = unstamped_figures(lines)
        for number, reason in bad:
            print(f"line {number}: figure without a valid stamp ({reason}): {lines[number - 1]}",
                  file=sys.stderr)
        return 1 if bad else 0
    if len(argv) == 4 and argv[1] == "ratio":
        try:
            (a, stamp_a), (b, stamp_b) = _load_row(argv[2]), _load_row(argv[3])
            common = combine([stamp_a, stamp_b])
        except StampError as exc:
            print(f"*** figure_stamp ratio: {exc}", file=sys.stderr)
            return 1
        if b == 0:
            print("*** figure_stamp ratio: the divisor is zero", file=sys.stderr)
            return 1
        print(f"{a / b:.4f}x [{SEPARATOR.join(common[n] for n in COMBINE_FIELDS)}]")
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
