#!/usr/bin/env python3
"""Turn the runner's per-program `STALLS` lines into a cycle-accounting table.

WHAT THIS IS FOR. The `.S` suite's cycle count was the only CPI number this
project had, and nothing decomposed it. The synchronous-read regfile cost a
measured +18.0% on this suite and that was accepted deliberately; a decision
like that is much easier to defend, and to revisit honestly, with an invoice
attached. This is the invoice. It is the same move `make fit` made for area:
one aggregate figure, argued about from structure, replaced by a measurement.

EVERY CYCLE IS CHARGED EXACTLY ONCE. test/cxxrtl.cc reads the decoder's own
`stall` signal and its six named reasons every cycle. A cycle where `stall` is
low is an issue cycle; a cycle where it is high goes to the first reason that is
true, in the order rtl/decoder.v tries them. So the columns add up to the cycle
count by construction, and this script checks that they do -- a mismatch means
the runner and the report disagree about the field names, not that the core got
slower.

`unattributed` IS THE ONE THAT MATTERS. It counts cycles the decoder called a
stall that none of the six named reasons explains. It is zero, and if it ever is
not, the taxonomy has fallen behind rtl/decoder.v -- a stall reason nobody has
written down. That is a finding, so this script exits nonzero on it rather than
printing it as a curiosity.

THE CPI IT PRINTS DESCRIBES A WORKLOAD, NOT THIS CORE. Which workload is the
caller's to say, in `--workload`, and it is printed next to the number because
that is where it will be read. `make cycles` runs the hand-written assembly
suite; `make dhrystone` runs compiled code and the two sentences are not
interchangeable.
"""

import argparse
import sys

# The five the decoder has, in the order it tries them: it holds `decoder_out`
# for the divider and bubbles for the other four. The runner writes these names,
# so a rename has to happen in both places at once -- which the field check
# below turns into an error rather than a silent zero.
REASONS = ["divider", "hazard", "serialize", "operand", "fetch"]

# What the CPI above it describes. It is an argument the caller has to supply,
# because the honest one differs: `make cycles` runs the hand-written assembly
# suite and `make dhrystone` runs compiled code, and printing the sentence below
# under a table of the second would be a claim about the workload that is simply
# false. The default is the suite's because that is the caller with a merge gate
# behind it.
SUITE_WORKLOAD = (
    "READ THE CPI AS A PROPERTY OF THIS SUITE. These are small hand-written\n"
    "assembly programs with dense back-to-back dependencies and almost no\n"
    "loop structure. Their instruction mix is not real code's, so this\n"
    "number describes the suite and not what the core would do on a\n"
    "program anybody wanted to run. It is useful for comparing one commit\n"
    "against another, and for nothing else."
)
HEADINGS = {
    "divider": "DIVIDER",
    "hazard": "HAZARD",
    "serialize": "SERIAL",
    "operand": "OPERAND",
    "fetch": "FETCH",
}
REQUIRED = ["cycles", "issue", "retires", "unattributed"] + REASONS


def parse(path):
    """`<program> key=value ...` per line. Returns a list of (name, counts)."""
    rows = []
    for lineno, line in enumerate(open(path), 1):
        line = line.strip()
        if not line:
            continue
        name, *fields = line.split()
        counts = {}
        for field in fields:
            if "=" not in field:
                sys.exit(f"{path}:{lineno}: '{field}' is not key=value")
            key, value = field.split("=", 1)
            try:
                counts[key] = int(value)
            except ValueError:
                sys.exit(f"{path}:{lineno}: {key} is '{value}', not a number")
        missing = [k for k in REQUIRED if k not in counts]
        if missing:
            sys.exit(
                f"{path}:{lineno}: {name} is missing {', '.join(missing)}. The "
                f"runner's STALLS line and this script's REASONS list have to "
                f"name the same things."
            )
        rows.append((name, counts))
    return rows


def cpi(cycles, retires):
    return f"{cycles / retires:.2f}" if retires else "-"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "counts", help="one `<program> key=value ...` line per program"
    )
    parser.add_argument(
        "--workload",
        default=SUITE_WORKLOAD,
        help="what the CPI describes, printed under the table",
    )
    args = parser.parse_args()

    rows = parse(args.counts)
    if not rows:
        sys.exit(
            f"{args.counts} is empty: no program reported its cycles, so there "
            f"is nothing to account for. A table over no programs would report "
            f"a clean 0 of 0."
        )

    total = {k: 0 for k in REQUIRED}
    for _, counts in rows:
        for key in REQUIRED:
            total[key] += counts[key]

    # Checked per program as well as over the suite. One program whose columns
    # do not add up is invisible in a total that happens to.
    broken = []
    for name, counts in rows:
        parts = counts["issue"] + counts["unattributed"] + sum(counts[r] for r in REASONS)
        if parts != counts["cycles"]:
            broken.append(f"  {name}: columns sum to {parts}, cycles is {counts['cycles']}")

    width = max(len(name) for name, _ in rows)
    header = f"{'PROGRAM':<{width}} {'CYCLES':>8} {'RETIRED':>8} {'CPI':>6} {'ISSUE':>8}"
    header += "".join(f"{HEADINGS[r]:>9}" for r in REASONS)
    header += f"{'UNATTR':>8}"

    print()
    print("== cycle accounting: where the cycles go ==")
    print()
    print(header)
    for name, counts in rows:
        line = (
            f"{name:<{width}} {counts['cycles']:>8} {counts['retires']:>8} "
            f"{cpi(counts['cycles'], counts['retires']):>6} {counts['issue']:>8}"
        )
        line += "".join(f"{counts[r]:>9}" for r in REASONS)
        line += f"{counts['unattributed']:>8}"
        print(line)

    suite = (
        f"{'SUITE':<{width}} {total['cycles']:>8} {total['retires']:>8} "
        f"{cpi(total['cycles'], total['retires']):>6} {total['issue']:>8}"
    )
    suite += "".join(f"{total[r]:>9}" for r in REASONS)
    suite += f"{total['unattributed']:>8}"
    print(suite)

    def share(n):
        return f"{100 * n / total['cycles']:.1f}%"

    pct = f"{'% of cycles':<{width}} {'':>8} {'':>8} {'':>6} {share(total['issue']):>8}"
    pct += "".join(f"{share(total[r]):>9}" for r in REASONS)
    pct += f"{share(total['unattributed']):>8}"
    print(pct)

    stalled = sum(total[r] for r in REASONS) + total["unattributed"]
    biggest = max(REASONS, key=lambda r: total[r])
    print()
    print(
        f"{len(rows)} programs, {total['cycles']} cycles, {total['retires']} "
        f"instructions retired, CPI {cpi(total['cycles'], total['retires'])}."
    )
    print(f"{stalled} of those cycles ({share(stalled)}) issued nothing.")
    print(
        f"The largest single reason is {biggest}: {total[biggest]} cycles, "
        f"{share(total[biggest])} of all cycles and "
        f"{100 * total[biggest] / stalled:.1f}% of the stalled ones."
    )
    print()
    print(args.workload)
    print()
    print(
        "A COLUMN IS CYCLES CHARGED, NOT CYCLES THE SIGNAL WAS HIGH. Several\n"
        "reasons are true on the same cycle often, and each cycle goes to the\n"
        "first one the decoder itself would try, so the columns add up. Measured\n"
        "on the three writable-text programs: fetch_stall is high on 26 cycles\n"
        "and is charged 8, because on the other 18 something else was already\n"
        "holding the same instruction."
    )

    if broken:
        sys.exit(
            "\n*** the columns do not add up to the cycle count:\n"
            + "\n".join(broken)
            + "\n*** Every cycle is charged to exactly one column by the runner,\n"
            "*** so this is a field name that has drifted between test/cxxrtl.cc\n"
            "*** and this script, not a slower core."
        )

    if total["unattributed"]:
        sys.exit(
            f"\n*** {total['unattributed']} cycles stalled for a reason this "
            f"report does not name.\n"
            "*** rtl/decoder.v raised `stall` and none of the six signals this\n"
            "*** counts was high, so there is a stall reason nobody has written\n"
            "*** down. Add it to kStallReasons in test/cxxrtl.cc and to REASONS\n"
            "*** here, and to the stall-reason list in CLAUDE.md."
        )


if __name__ == "__main__":
    main()
