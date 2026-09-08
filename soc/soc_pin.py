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

A pin is a claim about ONE netlist, never about the design's typical Fmax. It
is sound only while `soc.json`'s canonicalised form -- the same form
`soc/netlist_digest.py` takes a sha256 of, dead nets purged and source-line
attributes dropped so a comment cannot move it -- still hashes to what the pin
recorded, MINUS the toolchain's own `creator` string that form deliberately
keeps: a pin's required margin exists precisely so it survives ordinary
toolchain drift (a floating OSS CAD Suite release), and folding the toolchain
into the digest would force a re-pin on every one of those even when nothing
about the netlist moved. The toolchain that measured a pin is recorded
separately, in its own field, never inside the digest. A netlist that moved
invalidates every placement recorded against the old one; this is the RE-PIN
NEEDED failure, and it is deliberately not the same failure as a placement
that reproduces but falls under `SOC_MIN_MHZ`. The first says "this pin
describes a different design"; the second says "this design does not meet its
requirement". Reporting one as the other would send a reader to the wrong fix
-- re-synthesise and re-place versus find what lengthened the path.

`make soc-seed-search` is what writes a pin: it sweeps high-entropy seeds
(never 1..N -- a small integer seed is not an
independent draw on this placer), and writes the seed with the best margin
over `SOC_MIN_MHZ` alongside the whole distribution it was chosen from, so a
future reader can see it was a considered choice and not a lucky one.

Usage: soc_pin.py digest       <canon.json>
       soc_pin.py check-digest <canon.json> <pin.json>
       soc_pin.py seed          <pin.json>
       soc_pin.py write         <pin.json> --digest <hex> --seed <n> --mhz <f>
                                 --min-mhz <f> --toolchain <text>
                                 --distribution <json file, or - for stdin>
                                 [--synth-knob <text>] [--date <text>]

Exit (digest):       0 printed
      (check-digest): 0 pin matches this netlist, 3 RE-PIN NEEDED, 2 refused
      (seed):        0 printed, 1 refused
      (write):        0 written, 2 refused (margin too thin, bad input)
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import netlist_digest  # noqa: E402  -- sibling module, path fixed above

RE_PIN_NEEDED = 3
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
    required = ("netlist_digest", "seed", "measured_mhz", "min_mhz", "toolchain")
    missing = [key for key in required if key not in pin]
    if missing:
        refuse(REFUSED, f"{path}: missing {', '.join(missing)}. A pin file "
                        "carries the configuration it grades AND the evidence "
                        "it was chosen with; a partial write is not a pin.")
    return pin

def canon_digest(canon_path):
    """sha256:<hex> of a canonicalised netlist, reusing netlist_digest.py's form --
    minus the `creator` field that form deliberately keeps.

    netlist_digest.py folds the toolchain's version string into its digest on purpose,
    to catch a build that moved under an unchanged tree (its own docstring: "the
    direction this repo has been bitten in"). A pin's digest asks a different question
    -- is this still the netlist a seed was chosen for -- and a pin is required to
    clear its margin precisely so it SURVIVES ordinary toolchain drift; folding the
    toolchain string in here would force a re-pin on every OSS CAD Suite release even
    when the RTL, and everything synthesis derived from it, is unchanged. The
    toolchain that measured a pin is recorded separately, in its own field.
    """
    design, _top = netlist_digest.load(canon_path)
    design = {k: v for k, v in design.items() if k != "creator"}
    return f"sha256:{netlist_digest.digest(design)}"

def cmd_digest(args):
    """Printed rather than recomputed by each caller, so `check-digest` and
    `soc_seed_search.sh`'s write path can never compute this two different ways."""
    print(canon_digest(args.canon))
    return 0

def cmd_check_digest(args):
    pin = load_pin(args.pin)
    current = canon_digest(args.canon)
    pinned = pin["netlist_digest"]
    if current == pinned:
        print(f"pin OK: {current} matches {args.pin}")
        print(f"  seed {pin['seed']}, measured {pin['measured_mhz']:.2f} MHz "
              f"on {pin.get('date', '(no date recorded)')}")
        return 0
    refuse(
        RE_PIN_NEEDED,
        f"RE-PIN NEEDED: {args.canon}'s digest does not match {args.pin}.",
        f"  pinned:  {pinned}",
        f"  current: {current}",
        "This is NOT a timing failure -- the netlist this pin was measured "
        "against no longer exists, so the recorded MHz is evidence about a "
        "different design. Run `make soc-seed-search` to place the current "
        "netlist and write a new pin.",
    )

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
        "netlist_digest": args.digest,
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
    digest_cmd.add_argument("canon", help="this build's canonicalised netlist JSON")

    check = sub.add_parser("check-digest")
    check.add_argument("canon", help="this build's canonicalised netlist JSON")
    check.add_argument("pin", help="soc/pin.json")

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
        "check-digest": cmd_check_digest,
        "seed": cmd_seed,
        "write": cmd_write,
    }[args.command](args)

if __name__ == "__main__":
    sys.exit(main())
