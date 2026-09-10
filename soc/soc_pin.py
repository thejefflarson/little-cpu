#!/usr/bin/env python3
"""The pinned placement: what `make soc-timing` grades by default.

The evidence is that three re-rolls of IDENTICAL RTL semantics -- the same
netlist under yosys's chaotic-but-deterministic cell-name ordering -- place close
enough to `SOC_MIN_MHZ` that one draw of a sixteen-seed sweep already landed under
it. A sixteen-seed sweep is the honest instrument for "does this change move the
design", never for "does the shipping build clear the board clock": nothing stops
one draw from landing under 12.0 while the design that produced it is unchanged.
So `make soc-timing` grades ONE recorded placement, `soc/pin.json`, rather than
the worst of a sweep taken fresh every run.

A pin is a claim about ONE set of sources, never about the design's typical
Fmax. Its staleness is keyed on the files synthesis READS, not the netlist it
produces: yosys is deterministic for one input and one build, but the OSS CAD
Suite floats, so identical sources map to a different netlist on every release.
A netlist-keyed pin would demand a re-pin for a reason that is not a design
change, and would do it on every CI run. Hashing the inputs asks the question
that has a stable answer.

A stale pin WARNS, it does not fail. The gate is Fmax: `soc-timing` places at
the pinned seed and grades that measurement, which is real whether or not the
sources moved. The warning says only that a better seed may now exist -- and,
the reason it matters, that a regression can hide behind a pinned seed which
still clears while the design got worse. A comment edit counts as moved,
because a comment re-rolls the mapping and so re-rolls the draw.

`make soc-seed-search` is what writes a pin: it sweeps high-entropy seeds
(never 1..N -- a small integer seed is not an
independent draw on this placer), and writes the seed with the best margin
over `SOC_MIN_MHZ` alongside the whole distribution it was chosen from, so a
future reader can see it was a considered choice and not a lucky one.

Usage: soc_pin.py digest        <source file>...
       soc_pin.py check-sources <pin.json> <source file>...
       soc_pin.py seed          <pin.json>
       soc_pin.py write         <pin.json> --digest <hex> --seed <n> --mhz <f>
                                 --min-mhz <f> --toolchain <text>
                                 --distribution <json file, or - for stdin>
                                 [--synth-knob <text>] [--date <text>]

Exit (digest):        0 printed
      (check-sources): 0 always -- a mismatch warns on stderr, it does not fail
      (seed):        0 printed, 1 refused
      (write):        0 written, 2 refused (margin too thin, bad input)
"""

import argparse
import json
import hashlib
import pathlib
import sys
from pathlib import Path

REFUSED = 2

MIN_MARGIN_PCT = 5.0

def refuse(code, *lines):
    for line in lines:
        print(f"*** {line}", file=sys.stderr)
    sys.exit(code)

def load_pin(path):
    """The pin file's parsed contents, or refuse saying which field is missing."""
    try:
        text = Path(path).read_text()
    except OSError as err:
        refuse(REFUSED, f"{path}: {err.strerror}, so there is no pin to check "
                        "against.",
               "Run `make soc-seed-search` to write one.")
    try:
        pin = json.loads(text)
    except json.JSONDecodeError as err:
        refuse(REFUSED, f"{path}: not parseable as JSON ({err}). A hand-edited "
                        "pin that no longer parses is as invalid as one that "
                        "was never written.")
    required = ("sources_digest", "seed", "measured_mhz", "min_mhz", "toolchain")
    missing = [key for key in required if key not in pin]
    if missing:
        refuse(REFUSED, f"{path}: missing {', '.join(missing)}. A pin file "
                        "carries the configuration it grades AND the evidence "
                        "it was chosen with; a partial write is not a pin.")
    return pin

def sources_digest(paths):
    """sha256:<hex> over the files synthesis reads, not the netlist it produces.

    A netlist digest cannot answer "is this pin stale": yosys is deterministic for
    one input and one build, but the OSS CAD Suite floats, so the same sources map
    to a different netlist on every release and a netlist-keyed pin would demand a
    re-pin for a reason that is not a design change. Hashing the inputs asks the
    question that has a stable answer -- have the sources this seed was chosen for
    moved. A comment counts as moved, because a comment re-rolls the mapping.
    """
    h = hashlib.sha256()
    for path in sorted(paths):
        body = pathlib.Path(path).read_bytes()
        h.update(path.encode())
        h.update(hashlib.sha256(body).hexdigest().encode())
    return f"sha256:{h.hexdigest()}"

def cmd_digest(args):
    """Printed rather than recomputed by each caller, so `check-sources` and
    `soc_seed_search.sh`'s write path can never compute this two different ways."""
    print(sources_digest(args.sources))
    return 0

def cmd_check_sources(args):
    """WARNS, never refuses. A stale pin still measures a real placement, and
    `soc-timing` grades that measurement -- so the gate is Fmax, and this only
    says whether a better seed may now exist. Refusing here would fail a build
    for a comment edit."""
    pin = load_pin(args.pin)
    current = sources_digest(args.sources)
    pinned = pin.get("sources_digest")
    if current == pinned:
        print(f"pin OK: sources match {args.pin}")
        print(f"  seed {pin['seed']}, measured {pin['measured_mhz']:.2f} MHz "
              f"on {pin.get('date', '(no date recorded)')}")
        return 0
    print(f"*** PIN STALE: the sources moved since {args.pin} was written.",
          file=sys.stderr)
    print(f"***   pinned:  {pinned}", file=sys.stderr)
    print(f"***   current: {current}", file=sys.stderr)
    print("*** Not a failure: soc-timing still places at the pinned seed and "
          "grades that. But the recorded distribution describes different "
          "sources, so a better seed may exist and a regression can hide behind "
          "one that still clears. Run `make soc-seed-search` to re-take it.",
          file=sys.stderr)
    return 0

def cmd_seed(args):
    pin = load_pin(args.pin)
    seed = pin["seed"]
    if not isinstance(seed, int):
        refuse(REFUSED, f"{args.pin}: seed {seed!r} is not an integer.")
    print(seed)
    return 0

def cmd_write(args):
    if args.margin_pct < MIN_MARGIN_PCT:
        refuse(REFUSED,
               f"a {args.margin_pct:.2f}% margin over min-mhz is under the "
               f"{MIN_MARGIN_PCT:.1f}% floor this pin requires: the "
               "pin has to survive toolchain drift, not merely clear the "
               "requirement today.")
    if args.distribution == "-":
        dist_text = sys.stdin.read()
    else:
        try:
            dist_text = Path(args.distribution).read_text()
        except OSError as err:
            refuse(REFUSED, f"{args.distribution}: {err.strerror}, so there is "
                            "no distribution to record.")
    try:
        distribution = json.loads(dist_text)
    except json.JSONDecodeError as err:
        refuse(REFUSED, f"--distribution: not parseable as JSON ({err}).")

    pin = {
        "sources_digest": args.digest,
        "seed": args.seed,
        "synth_knob": args.synth_knob,
        "measured_mhz": args.mhz,
        "min_mhz": args.min_mhz,
        "margin_pct": args.margin_pct,
        "toolchain": args.toolchain,
        "date": args.date,
        "distribution": distribution,
    }
    Path(args.pin).write_text(json.dumps(pin, indent=2, sort_keys=True) + "\n")
    print(f"wrote {args.pin}: seed {args.seed}, {args.mhz:.2f} MHz "
          f"({args.margin_pct:.2f}% over {args.min_mhz:.2f})")
    return 0

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    digest_cmd = sub.add_parser("digest")
    digest_cmd.add_argument("sources", nargs="+", help="the files synthesis reads")

    check = sub.add_parser("check-sources")
    check.add_argument("pin", help="soc/pin.json")
    check.add_argument("sources", nargs="+", help="the files synthesis reads")

    seed = sub.add_parser("seed")
    seed.add_argument("pin")

    write = sub.add_parser("write")
    write.add_argument("pin")
    write.add_argument("--digest", required=True, help="sha256:<hex>")
    write.add_argument("--seed", required=True, type=int)
    write.add_argument("--mhz", required=True, type=float)
    write.add_argument("--min-mhz", required=True, type=float)
    write.add_argument(
        "--margin-pct", required=True, type=float,
        help="100 * (mhz - min_mhz) / min_mhz, computed by the caller so this "
        "script never rounds the number it grades")
    write.add_argument("--toolchain", required=True)
    write.add_argument("--distribution", required=True,
                       help="a JSON file (or - for stdin): the sweep this seed "
                       "was chosen from")
    write.add_argument("--synth-knob", default=None)
    write.add_argument("--date", required=True)

    args = parser.parse_args()
    return {
        "digest": cmd_digest,
        "check-sources": cmd_check_sources,
        "seed": cmd_seed,
        "write": cmd_write,
    }[args.command](args)

if __name__ == "__main__":
    sys.exit(main())
