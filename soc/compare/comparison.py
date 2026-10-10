#!/usr/bin/env python3
"""Render docs/comparison.md from soc/compare/product.json, and grade it.

  comparison.py render            print the document
  comparison.py write             rewrite docs/comparison.md (`make compare-doc`)
  comparison.py check             fail when the committed document is not a fresh render
  comparison.py ratchet           fail when littlecpu's cycle columns moved off CYCLE_FLOOR

Everything reads the committed stamp, so nothing here simulates or places. Re-taking the
stamp stays `make compare-product`'s job; this file only keeps what is published from it
honest. docs/adr/0244 records why the cycle half is ratcheted and the clock half is not.

CYCLE_FLOOR holds one `benchmark core isa factor` line per benchmark; a stamp whose
littlecpu factor differs in EITHER direction fails, as test/OBSERVED_FLOOR does, so a
regression is caught and an improvement owes an update. Only the target core is
ratcheted: an opponent's number is reported, never graded.
"""

import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from product_write import PAIR_NAME_RE  # noqa: E402

BENCHMARKS = [("dhrystone", "Dhrystone 2.1"), ("coremark", "CoreMark")]
PARTS = [("", "iCE40 UP5K"), ("_ecp5", "ECP5 LFE5U-25F")]
# The feature-matched section's pairs are named <benchmark>_imac[_ecp5]: the same benchmarks
# at RV32IMAC, against the builds of VexRiscv and Hazard3 that carry C and A.
MATCHED = "_imac"
CORE_ORDER = ["littlecpu", "vexriscv", "vexriscv_lrsc", "hazard3", "hazard3_perf", "hazard3_c"]
# Every row names the build it ran. A core in the stamp with no entry here is refused, so a
# new opponent cannot be published without stating its configuration (docs/adr/0246).
CORE_LABELS = {
    "littlecpu": "littlecpu",
    "vexriscv": "vexriscv (performance build)",
    "hazard3": "hazard3 (area build)",
    "hazard3_perf": "hazard3_perf (performance build)",
    "vexriscv_lrsc": "vexriscv_lrsc (performance build plus LR/SC)",
    "hazard3_c": "hazard3_c (performance build plus C)",
}
CORE_NOTES = {
    "littlecpu": "this core, built at the ISA each section names (RV32IM, then RV32IMAC)",
    "vexriscv": "the VexRiscv generated from `soc/compare/vexriscv/GenLittleCpuCompare.scala` "
                "at the pinned SHA, its authors' no-MMU no-cache performance configuration: "
                "M and C (`compressedGen = true`), no A",
    "vexriscv_lrsc": "the same VexRiscv with `withLrSc = true` on its data bus (Zalrsc): "
                     "M, C and LR/SC. The pinned VexRiscv's no-cache data bus has no AMO "
                     "option and the repository has no atomic plugin, so the nine AMO "
                     "instructions are not implemented and trap as illegal",
    "hazard3": "Hazard3's two-port build from its iCE40 example (`fpga_icebreaker.v`): "
               "bit-serial multiply, no branch predictor, no counters, no fence.i; its "
               "disclosed bus wait is counted in its cycles",
    "hazard3_perf": "Hazard3's two-port build from its two ECP5 examples "
                    "(`fpga_ulx3s.v`, `fpga_orangecrab_25f.v`): single-cycle multiply, "
                    "branch predictor, counters, fence.i; the same bus adapter and wait",
    "hazard3_c": "`hazard3_perf` with `EXTENSION_C` on, a harness choice no Hazard3 example "
                 "ships (the config test grades it as that one difference); A is on in every "
                 "Hazard3 build",
}
# A pair carrying the first of these and not the second is publishing one build of a core
# that ships two, and says so.
SIBLING_BUILDS = {"hazard3": "hazard3_perf"}
CLOCK_NOT_RATCHETED = ("not ratcheted, because the placer's spread is wider than any "
                       "difference a gate could grade")

CAVEATS = [
    "**Hazard3's ECP5 clock** carries a standing flag: the same RTL read 33.26 MHz in one "
    "session and 48.50 in a later one, and the unpinned nextpnr-ecp5 is the likely, "
    "unconfirmed cause. Read it as measured and inherit the flag.",
    "**One standard for every opponent** (docs/adr/0246): each core runs the configuration "
    "its authors ship for a part with room, and a core that ships a small-part build as well "
    "gets that build as its own named column. No ratio here is against an unnamed build, "
    "and an ISA choice (C, A, M) is the harness row's, never an opponent's tuning.",
    "**CoreMark's cycles are simulated at a larger map than the clock is placed at**: its "
    "text does not fit the up5k's placed ROM, so the cycle half and the clock half come "
    "from different geometries. Dhrystone fits, and nothing in its rows is distorted by "
    "memory size. Each row's simulated geometry is on its provenance line.",
    "**A product is a measurement only when both halves came off one tree and one "
    "toolchain.** Every row here shares one stamp commit and one tool list.",
    "**Parts are never blended.** The up5k and ECP5 sections answer different questions "
    "and are not averaged or ranked against each other.",
    "**The first two sections are RV32IM**, the widest ISA their columns share (the Hazard3 "
    "builds there have no C, the stock VexRiscv has no A), so littlecpu's A and C hardware "
    "sits unused in them. The "
    "feature-matched section compiles all three cores at RV32IMAC; its VexRiscv carries "
    "LR/SC and not the AMOs, and neither benchmark contains an atomic instruction, so what "
    "that section measures is what each core pays for carrying A and C, not their use.",
    "**nanocpu is not in this comparison** and is never quoted beside littlecpu.",
]


# Every string the stamp contributes to the document is matched against one of these first,
# so a stamp cannot inject markup, a code-span break or a link into a published page.
FIELD_PATTERNS = {
    "base": r"[0-9a-f]{40}",
    "date": r"\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ",
    "dirty": r"yes|no",
    "seeds": r"[A-Za-z0-9 ]+",
    "isa": r"rv32[a-z0-9_]+",
    "unit": r"[A-Za-z]+/MHz",
    "cflags": r"[A-Za-z0-9 =_.,+/:-]+",
    "reason": r"[A-Za-z0-9 .,;:'()/_+-]+",
    "target_core": r"[a-z0-9_]+",
    "out_of_comparison": r"[a-z0-9_]+ [0-9.]+ to [0-9.]+ MHz(; [a-z0-9_]+ [0-9.]+ to [0-9.]+ MHz)*",
}
TOOL_NAME_PATTERN = r"[A-Za-z0-9_.-]+"
TOOL_VERSION_PATTERN = r"[A-Za-z0-9 .,+()_/:\"'-]+"
MEASURED_FIELDS = ("base", "date", "dirty", "seeds", "isa", "unit", "cflags", "target_core")
CLOCK_NUMBERS = ("worst_mhz", "median_mhz", "best_mhz", "spread_pct")


def malformed(where, value):
    sys.exit(f"*** {where} is {value!r}, which is not a value this document "
             "publishes verbatim; the stamp is malformed or tampered with.")


def require(where, value, pattern):
    if not isinstance(value, str) or not re.fullmatch(pattern, value):
        malformed(where, value)


def require_dict(where, value):
    if not isinstance(value, dict):
        malformed(where, value)
    return value


def require_number(where, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        malformed(where, value)
    return value


def require_count(where, value):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        malformed(where, value)
    return value


def validate_measured(name, pair):
    """The numbers and core names a measured pair contributes, and their arithmetic."""
    for field in ("rom_words", "ram_words"):
        require_count(f"{name}.{field}", pair.get(field))
    if pair.get("step_mhz") is not None:
        require_number(f"{name}.step_mhz", pair["step_mhz"])
    cores = require_dict(f"{name}.cores", pair.get("cores"))
    products = require_dict(f"{name}.products", pair.get("products"))
    unlabelled = sorted((set(cores) | set(products)) - set(CORE_LABELS))
    if unlabelled:
        sys.exit(f"*** {name}: {', '.join(unlabelled)} in the stamp with no "
                 "configuration label in comparison.py's CORE_LABELS; a ratio against an "
                 "unnamed build is not published.")
    target = pair["target_core"]
    if target not in cores:
        malformed(f"{name}.target_core, absent from its cores,", target)
    if set(products) != set(cores) - {target}:
        malformed(f"{name}.products, against its cores {sorted(cores)},", sorted(products))
    seeds = len(pair["seeds"].split())
    for core, c in sorted(cores.items()):
        c = require_dict(f"{name}.cores.{core}", c)
        require_number(f"{name}.cores.{core}.cycle_factor", c.get("cycle_factor"))
        clock = require_dict(f"{name}.cores.{core}.clock_mhz", c.get("clock_mhz"))
        for field in CLOCK_NUMBERS:
            require_number(f"{name}.cores.{core}.clock_mhz.{field}", clock.get(field))
        if require_count(f"{name}.cores.{core}.clock_mhz.n", clock.get("n")) != seeds:
            malformed(f"{name}.cores.{core}.clock_mhz.n, against {seeds} seeds,", clock["n"])
    for core, p in sorted(products.items()):
        p = require_dict(f"{name}.products.{core}", p)
        ratio = require_dict(f"{name}.products.{core}.ratio", p.get("ratio"))
        mine = require_dict(f"{name}.products.{core}.{core}_dmips", p.get(f"{core}_dmips"))
        theirs = require_dict(f"{name}.products.{core}.{target}_dmips", p.get(f"{target}_dmips"))
        for key in ("worst", "median"):
            r = require_number(f"{name}.products.{core}.ratio.{key}", ratio.get(key))
            a = require_number(f"{name}.products.{core}.{core}_dmips.{key}", mine.get(key))
            b = require_number(f"{name}.products.{core}.{target}_dmips.{key}", theirs.get(key))
            if b == 0 or not math.isclose(r, a / b, rel_tol=1e-9):
                malformed(f"{name}.products.{core}.ratio.{key}, against {a!r} / {b!r},", r)


def validate(stamp):
    """Exit on the first stamp value that is not what this document publishes verbatim."""
    pairs = require_dict("the stamp's pairs", require_dict("the stamp", stamp).get("pairs"))
    for name, pair in sorted(pairs.items()):
        if not PAIR_NAME_RE.fullmatch(name):
            malformed("pair name", name)
        pair = require_dict(f"pair {name}", pair)
        measured = pair.get("status") == "measured"
        for field in MEASURED_FIELDS if measured else ("reason",):
            require(f"{name}.{field}", pair.get(field), FIELD_PATTERNS[field])
        for tool, version in sorted(require_dict(f"{name}.tools", pair.get("tools", {})).items()):
            require(f"{name}.tools key", tool, TOOL_NAME_PATTERN)
            if not isinstance(version, str):
                malformed(f"{name}.tools.{tool}", version)
            require(f"{name}.tools.{tool}", tool_version(version), TOOL_VERSION_PATTERN)
        if measured:
            if "out_of_comparison" in pair:
                require(f"{name}.out_of_comparison", pair["out_of_comparison"],
                        FIELD_PATTERNS["out_of_comparison"])
            validate_measured(name, pair)


def f(value, places=2):
    return f"{value:.{places}f}"


def clock_cell(clock):
    return (f"{f(clock['worst_mhz'])} / {f(clock['median_mhz'])} / {f(clock['best_mhz'])} "
            f"({f(clock['spread_pct'], 1)}%, n={clock['n']})")


def tool_version(text):
    return re.sub(r" \[[^\]]*\]$", "", text)


def provenance(name, pair):
    return [
        f"- `{name}`: commit `{pair['base']}`, taken {pair['date']}, dirty: {pair['dirty']}, "
        f"seeds: {pair['seeds']}",
        f"  - ISA `{pair['isa']}`, simulated ROM {pair['rom_words']} words, "
        f"RAM {pair['ram_words']} words",
        f"  - CFLAGS `{pair['cflags']}`",
    ]


def render_pair(title, pair, out, level=3):
    out += [f"{'#' * level} {title}", ""]
    if pair is None or pair["status"] != "measured":
        reason = "no pair stamped" if pair is None else pair.get("reason", "no reason recorded")
        out += [f"Not measured: {reason}.", ""]
        return
    unit, step, target = pair["unit"], pair.get("step_mhz"), pair["target_core"]
    score = unit.split("/")[0]
    if step:
        out.append(f"Cycles alone: every core that clears the {f(step)} MHz step quantises to "
                   "it, so the product is the cycle factor at one shared clock. The placed "
                   "clock is graded pass/fail against the step and shown for provenance.")
        header = f"| core | {unit} | placed clock MHz worst / median / best | {score} at {f(step)} MHz | vs {target} |"
        sep = "|---|---:|---|---:|---:|"
    else:
        out.append("The clock is a real factor on this part, read at the worst and the "
                   "median of the paired placements.")
        header = (f"| core | {unit} | placed clock MHz worst / median / best | "
                  f"{score} worst | {score} median | vs {target} (worst / median) |")
        sep = "|---|---:|---|---:|---:|---|"
    out += ["", header, sep]
    for core in CORE_ORDER:
        if core not in pair["cores"]:
            continue
        c = pair["cores"][core]
        row = f"| {CORE_LABELS[core]} | {f(c['cycle_factor'], 4)} | {clock_cell(c['clock_mhz'])} |"
        if core == target:
            prod = next(iter(pair["products"].values()))[f"{target}_dmips"]
            ratio = "1.000x" if step else "1.000x / 1.000x"
        else:
            p = pair["products"][core]
            prod = p[f"{core}_dmips"]
            r = p["ratio"]
            ratio = f"{f(r['median'], 3)}x" if step else f"{f(r['worst'], 3)}x / {f(r['median'], 3)}x"
        if step:
            row += f" {f(prod['median'])} | {ratio} |"
        else:
            row += f" {f(prod['worst'])} | {f(prod['median'])} | {ratio} |"
        out.append(row)
    if pair.get("out_of_comparison"):
        out += ["", f"Out of this comparison, placed under the {f(step)} MHz step (the part's "
                "next step down is 6 MHz, so a core there does not score a fraction, it is "
                f"out): {pair['out_of_comparison']}, worst to best placement."]
    for core, sibling in SIBLING_BUILDS.items():
        if core in pair["cores"] and sibling not in pair["cores"]:
            out += ["", f"Not stamped: `{sibling}` is absent from this pair, so `{core}` "
                    "above is that core's small-part build alone; the next "
                    "`make compare-product` run adds the other."]
    out += ["", f"Cycle factor: ratcheted for `{target}` only (`soc/compare/CYCLE_FLOOR`); the "
            f"other cores' factors are reported. Clock: {CLOCK_NOT_RATCHETED}.", ""]


def render(stamp):
    out = [
        "# Cross-core comparison",
        "",
        "<!-- generated by soc/compare/comparison.py from soc/compare/product.json; "
        "do not hand-edit. Regenerate with `make compare-doc`. -->",
        "",
        "Generated from the stamp `make compare-product` writes. `make test` fails when this "
        "file is not a fresh render of the committed stamp, and when littlecpu's cycle "
        "factors move off `soc/compare/CYCLE_FLOOR`. Opponents' figures and every clock "
        "are reported, not ratcheted (docs/adr/0244). The measurements behind each caveat "
        "are in docs/adr/0146, 0160, 0171 and 0232.",
        "",
    ]
    pairs = stamp["pairs"]
    for suffix, label in PARTS:
        out += [f"## {label}", ""]
        for bench, title in BENCHMARKS:
            render_pair(title, pairs.get(bench + suffix), out)
    out += ["## Feature-matched (RV32IMAC)", "",
            "Every core compiled at `rv32imac_zicsr_zifencei`, the richest ISA all three "
            "carry: littlecpu, VexRiscv with LR/SC (`vexriscv_lrsc`) and Hazard3's "
            "performance build with C (`hazard3_c`). The first two sections above stay "
            "RV32IM and are not replaced by this one.", ""]
    for suffix, label in PARTS:
        out += [f"### {label}", ""]
        for bench, title in BENCHMARKS:
            render_pair(f"{title}, RV32IMAC", pairs.get(bench + MATCHED + suffix), out, level=4)
    out += ["## Caveats that travel with the numbers", ""]
    out += [f"- **{core}**: {note}." for core, note in CORE_NOTES.items()]
    out += [f"- {caveat}" for caveat in CAVEATS]
    unstamped = [title for bench, title in BENCHMARKS
                 if not any("hazard3" in (pairs.get(bench + sfx) or {}).get("cores", {})
                            for sfx, _ in PARTS)]
    if not any(bench + MATCHED + sfx in pairs for bench, _ in BENCHMARKS for sfx, _ in PARTS):
        out.append("- **The feature-matched section is not stamped yet**: the committed stamp "
                   "predates it, and the next `make compare-product` run adds it. The "
                   "measured figures are in docs/adr/0250.")
    if unstamped:
        out.append(f"- **Hazard3 has no {' or '.join(unstamped)} row** in this stamp.")
    out += ["", "## Stamp provenance", ""]
    measured = {n: p for n, p in sorted(pairs.items()) if p["status"] == "measured"}
    for name, pair in measured.items():
        out += provenance(name, pair)
    tools = {json.dumps(p["tools"], sort_keys=True) for p in measured.values()}
    out.append("")
    if len(tools) == 1:
        out += ["Tools, identical across every pair:", ""]
        out += [f"- {k}: {tool_version(v)}" for k, v in sorted(next(iter(measured.values()))["tools"].items())]
    else:
        out += ["Tools, per pair:", ""]
        for name, pair in measured.items():
            out.append(f"- `{name}`: " + "; ".join(
                f"{k}: {tool_version(v)}" for k, v in sorted(pair["tools"].items())))
    out.append("")
    return "\n".join(out)


def read_floor(path):
    rows = {}
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line and not line.startswith("#"):
                bench, core, isa, factor = line.split()
                rows[bench] = (core, isa, factor)
    return rows


def ratchet(stamp, floor):
    problems = []
    seen = set()
    for matched, suffix, bench in [(m, sfx, b) for m in ("", MATCHED) for sfx, _ in PARTS
                                   for b, _ in BENCHMARKS]:
        name = bench + matched + suffix
        key = bench + matched
        pair = stamp["pairs"].get(name)
        if matched and pair is None:
            continue
        if pair is None or pair["status"] != "measured":
            problems.append(f"{name}: no measured pair to grade")
            continue
        seen.add(key)
        if key not in floor:
            problems.append(f"{name}: CYCLE_FLOOR has no {key} line")
            continue
        core, isa, want = floor[key]
        got = pair["cores"][core]["cycle_factor"]
        if pair["target_core"] != core or pair["isa"] != isa:
            problems.append(f"{name}: stamp is {pair['target_core']} at {pair['isa']}, "
                            f"floor is {core} at {isa}; a different row is a new floor")
        elif math.isclose(got, float(want), rel_tol=1e-9):
            continue
        elif got < float(want):
            problems.append(f"{name}: REGRESSION, {core} {got!r} is below floor {want}")
        else:
            problems.append(f"{name}: IMPROVEMENT, {core} {got!r} is above floor {want}; "
                            "update soc/compare/CYCLE_FLOOR to bank it")
    problems += [f"CYCLE_FLOOR line {b} matches no stamped pair" for b in sorted(set(floor) - seen)]
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["render", "write", "check", "ratchet"])
    parser.add_argument("--stamp", default=os.path.join(HERE, "product.json"))
    parser.add_argument("--doc", default=os.path.join(REPO, "docs", "comparison.md"))
    parser.add_argument("--floor", default=os.path.join(HERE, "CYCLE_FLOOR"))
    args = parser.parse_args()
    with open(args.stamp) as handle:
        stamp = json.load(handle)
    validate(stamp)

    if args.mode == "render":
        sys.stdout.write(render(stamp))
    elif args.mode == "write":
        with open(args.doc, "w") as handle:
            handle.write(render(stamp))
    elif args.mode == "check":
        try:
            with open(args.doc) as handle:
                have = handle.read()
        except FileNotFoundError:
            sys.exit(f"*** {args.doc} does not exist; run `make compare-doc`.")
        if have != render(stamp):
            sys.exit(f"*** {args.doc} is not a render of {args.stamp}; "
                     "run `make compare-doc` and commit the result.")
        print("comparison document matches the stamp")
    else:
        problems = ratchet(stamp, read_floor(args.floor))
        for problem in problems:
            print(f"*** {problem}", file=sys.stderr)
        if problems:
            sys.exit(1)
        print("littlecpu's cycle columns match CYCLE_FLOOR")


if __name__ == "__main__":
    main()
