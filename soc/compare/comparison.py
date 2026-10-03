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

BENCHMARKS = [("dhrystone", "Dhrystone 2.1"), ("coremark", "CoreMark")]
PARTS = [("", "iCE40 UP5K"), ("_ecp5", "ECP5 LFE5U-25F")]
CORE_ORDER = ["littlecpu", "vexriscv", "hazard3", "hazard3_perf"]
# Every row names the build it ran. A core in the stamp with no entry here is refused, so a
# new opponent cannot be published without stating its configuration (docs/adr/0246).
CORE_LABELS = {
    "littlecpu": "littlecpu",
    "vexriscv": "vexriscv (performance build)",
    "hazard3": "hazard3 (area build)",
    "hazard3_perf": "hazard3_perf (performance build)",
}
CORE_NOTES = {
    "littlecpu": "this core, built at the shared RV32IM subset",
    "vexriscv": "the generated VexRiscv in the pinned riscv-formal clone, its authors' "
                "performance configuration (it ships no other): M, no A, no C",
    "hazard3": "Hazard3's two-port build from its iCE40 example (`fpga_icebreaker.v`): "
               "bit-serial multiply, no branch predictor, no counters, no fence.i; its "
               "disclosed bus wait is counted in its cycles",
    "hazard3_perf": "Hazard3's two-port build from its two ECP5 examples "
                    "(`fpga_ulx3s.v`, `fpga_orangecrab_25f.v`): single-cycle multiply, "
                    "branch predictor, counters, fence.i; the same bus adapter and wait",
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
    "**Hazard3 has no Dhrystone row** in the stamp; only CoreMark carries its columns.",
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
    "**The comparison is RV32IM**, the widest ISA all the cores share; no pairwise "
    "wider-ISA row is stamped, so none is rendered.",
    "**nanocpu is not in this comparison** and is never quoted beside littlecpu.",
]


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


def render_pair(title, pair, out):
    out += [f"### {title}", ""]
    if pair is None or pair["status"] != "measured":
        reason = "no pair stamped" if pair is None else pair.get("reason", "no reason recorded")
        out += [f"Not measured: {reason}.", ""]
        return
    unit, step, target = pair["unit"], pair.get("step_mhz"), pair["target_core"]
    score = unit.split("/")[0]
    if step:
        out.append(f"Cycles alone: every core quantises to the {f(step)} MHz step, so the "
                   "product is the cycle factor at one shared clock. The placed clock is "
                   "graded pass/fail against the step and shown for provenance.")
        header = f"| core | {unit} | placed clock MHz worst / median / best | {score} at {f(step)} MHz | vs {target} |"
        sep = "|---|---:|---|---:|---:|"
    else:
        out.append("The clock is a real factor on this part, read at the worst and the "
                   "median of the paired placements.")
        header = (f"| core | {unit} | placed clock MHz worst / median / best | "
                  f"{score} worst | {score} median | vs {target} (worst / median) |")
        sep = "|---|---:|---|---:|---:|---|"
    out += ["", header, sep]
    unlabelled = sorted(set(pair["cores"]) - set(CORE_LABELS))
    if unlabelled:
        sys.exit(f"*** {title}: {', '.join(unlabelled)} in the stamp with no configuration "
                 "label in comparison.py's CORE_LABELS; a ratio against an unnamed build "
                 "is not published.")
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
    out += ["## Caveats that travel with the numbers", ""]
    out += [f"- **{core}**: {note}." for core, note in CORE_NOTES.items()]
    out += [f"- {caveat}" for caveat in CAVEATS]
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
    for suffix, _ in PARTS:
        for bench, _ in BENCHMARKS:
            name = bench + suffix
            pair = stamp["pairs"].get(name)
            if pair is None or pair["status"] != "measured":
                problems.append(f"{name}: no measured pair to grade")
                continue
            seen.add(bench)
            if bench not in floor:
                problems.append(f"{name}: CYCLE_FLOOR has no {bench} line")
                continue
            core, isa, want = floor[bench]
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
