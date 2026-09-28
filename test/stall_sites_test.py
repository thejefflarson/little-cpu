#!/usr/bin/env python3
"""Asserts that every place declaring a stall reason names the same set, the way
test/march_test.sh already does for the ISA string's seven sites.

Usage: stall_sites_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. Nothing checked that the sites agreed with each other; a reason dropped
from one site, or a new signal ORed into `stall` that nobody taught the other sites, went
unnoticed until someone read a CPI number and wondered why it changed.

TWO VOCABULARIES, SEVEN AND SIX LONG, AND THEY ARE NOT THE SAME LIST. The D/X split made
the divider's own busy bit, `x_busy`, opaque at D's level (D does not need to know why X
is still working `out`). But test/cxxrtl.cc's cycle accounting reaches inside the
executor instance and still charges a stalled cycle to `divider_busy` by name, because
that distinction is what makes `make cycles`' CPI table useful. So there are two
canonical lists, not one: DECODER_REASONS (what actually gates `stall`, at the
raw-signal granularity `stall`'s own OR is built from -- hazard_rs1 and hazard_rs2 are
two signals here, since that is the literal shape of test/decoder_tb.v's OR-identity
check) and CPI_REASONS (what a stalled cycle is charged to for reporting, where
hazard_rs1/hazard_rs2 collapse to one "hazard" bucket). B3 deleted the load/store region
wait outright, so x_busy is now exactly `divider_busy`'s own condition, restated rather
than aliased (region_stall no longer exists to fold together with it); that identity is
asserted directly rather than assumed.

A SIGNAL THIS SCRIPT DOES NOT RECOGNIZE IS RED, NOT SILENT, in whichever vocabulary the
site is graded against.

Hermetic: file reads only. No toolchain, so this runs inside `make test` anywhere.
"""

import pathlib
import re
import sys

DECODER_REASONS = ["hazard_rs1", "hazard_rs2", "serialize", "fetch", "atomic", "x_busy", "bus"]
SIGNAL_TO_DECODER_REASON = {
    "hazard_rs1": "hazard_rs1",
    "hazard_rs2": "hazard_rs2",
    "serialize": "serialize",
    "fetch_stall": "fetch",
    "atomic_stall": "atomic",
    "x_busy": "x_busy",
    "bus_wait": "bus",
}

CPI_REASONS = ["divider", "atomic", "hazard", "serialize", "fetch", "bus"]
SIGNAL_TO_CPI_REASON = {
    "divider_busy": "divider",
    "atomic_stall": "atomic",
    "hazard_rs1": "hazard",
    "hazard_rs2": "hazard",
    "serialize": "serialize",
    "fetch_stall": "fetch",
    "bus_wait": "bus",
}

CLAUDE_STALL_OWN = "stall_own = hazard || serialize || fetch_stall || atomic_stall || x_busy"
CLAUDE_STALL = "stall = stall_own || bus_wait"


def read(path):
    if not path.is_file():
        sys.exit(f"error: '{path}' does not exist, so there is nothing to scan.")
    return path.read_text()


def split_or_terms(rhs):
    return [t.strip() for t in re.split(r"\|\|", rhs) if t.strip()]


def assign_rhs(text, name):
    m = re.search(r"\bassign\s+" + re.escape(name) + r"\s*=\s*(.*?);", text, re.DOTALL)
    return m.group(1) if m else None


def unknown_and_missing(label, tokens, signal_to_reason, required_reasons):
    """Grades a site by the SET of reasons its raw signal tokens cover, never by how
    many tokens it took."""
    errors = []
    unknown = sorted({t for t in tokens if t not in signal_to_reason})
    for tok in unknown:
        errors.append(
            f"error: {label} names '{tok}', which this script does not recognize in "
            f"that vocabulary.\n"
            f"  If '{tok}' is a new stall reason: teach it to the right SIGNAL_TO_*_REASON "
            f"table here, then to every site it belongs in -- rtl/decoder.v's signal, its "
            f"OR, its publish arm and its FORMAL hold-assert; test/decoder_tb.v's "
            f"OR-identity check; test/cxxrtl.cc's kStallLabels/kStallReasons; "
            f"test/stall_report.py's REASONS and HEADINGS; and CLAUDE.md's commitment 8.\n"
            f"  If '{tok}' is a non-stall kill bubble: it must stay OUT of every one of "
            f"those ORs -- a kill belongs in the cycle-accounting identity "
            f"test/stall_report.py already keeps, as its own column beside `issue`, never "
            f"folded into `stall`."
        )
    covered = {signal_to_reason[t] for t in tokens if t in signal_to_reason}
    for reason in sorted(set(required_reasons) - covered):
        errors.append(f"error: {label} is missing reason '{reason}'.")
    return errors


def check_decoder_v(text):
    errors = []
    label = "rtl/decoder.v's stall composition (stall_own/stall)"

    hazard_rhs = assign_rhs(text, "hazard")
    if hazard_rhs is None:
        errors.append("error: rtl/decoder.v has no 'assign hazard = ...;' to read.")
        hazard_terms = []
    else:
        hazard_terms = split_or_terms(hazard_rhs)

    top_terms = []
    for name, skip in (("stall_own", set()), ("stall", {"stall_own"})):
        rhs = assign_rhs(text, name)
        if rhs is None:
            errors.append(f"error: {label} has no 'assign {name} = ...;' to read.")
            return errors
        for t in split_or_terms(rhs):
            if t in skip:
                continue
            if t == "hazard":
                top_terms += hazard_terms
            else:
                top_terms.append(t)
    errors += unknown_and_missing(label, top_terms, SIGNAL_TO_DECODER_REASON, DECODER_REASONS)

    label = "rtl/decoder.v's publish arm (the hold branch)"
    m = re.search(r"end else if \((.*?)\) begin\s*\n\s*out <= out;", text, re.DOTALL)
    if m is None:
        errors.append(f"error: {label} could not be found.")
    elif m.group(1).strip() != "x_busy":
        errors.append(
            f"error: {label} holds on '{m.group(1).strip()}', not exactly 'x_busy'. "
            f"Commitment 8 says x_busy is the only reason that holds; every other "
            f"reason bubbles.")

    label = "rtl/decoder.v's FORMAL hold-assert"
    if not re.search(r"\$past\(x_busy\)\)\s*assert\(out == \$past\(out\)\)", text):
        errors.append(
            f"error: {label} could not be found -- expected an "
            f"'if (... $past(x_busy)) assert(out == $past(out));' temporal check.")

    return errors


def check_executor_v(text):
    label = "rtl/executor.v's x_busy composition"
    x_rhs = assign_rhs(text, "x_busy")
    d_rhs = assign_rhs(text, "divider_busy")
    if x_rhs is None:
        return [f"error: {label} has no 'assign x_busy = ...;' to read."]
    if d_rhs is None:
        return [f"error: {label} has no 'assign divider_busy = ...;' to read."]
    if x_rhs.strip() != d_rhs.strip():
        return [
            f"error: {label} is '{x_rhs.strip()}', not the same expression "
            f"divider_busy is defined as ('{d_rhs.strip()}'). region_stall is gone, so "
            f"x_busy is exactly divider_busy's own condition, restated rather than "
            f"aliased (an alias collapses to one netlist bit, which breaks "
            f"test/zkt_isolation_test.py's one-hop block)."
        ]
    return []


def check_executor_tb(text):
    label = "test/executor_tb.v's x_busy identity check"
    m = re.search(r"if \(x_busy !== (dut\.\w+)\) begin", text, re.DOTALL)
    if m is None:
        return [f"error: {label} could not be found."]
    term = m.group(1)[len("dut."):]
    if term != "divider_busy":
        return [f"error: {label} compares x_busy against '{term}', not 'divider_busy'."]
    return []


def check_decoder_tb(text):
    label = "test/decoder_tb.v's OR-identity check"
    m = re.search(r"if \(dut\.stall !== \((.*?)\)\) begin", text, re.DOTALL)
    if m is None:
        return [f"error: {label} could not be found."]
    terms = []
    for t in split_or_terms(m.group(1)):
        terms.append(t[len("dut."):] if t.startswith("dut.") else t)
    return unknown_and_missing(label, terms, SIGNAL_TO_DECODER_REASON, DECODER_REASONS)


def cpi_signals_from_reasons_items(pairs, label):
    errors = []
    sigs = []
    for item, _bucket in pairs:
        sig = None
        for prefix in ("uut decoder ", "uut executor "):
            if item.startswith(prefix):
                sig = item[len(prefix):]
                break
        if sig is None:
            errors.append(
                f"error: {label} has an item '{item}' not shaped "
                f"'uut decoder <signal>' or 'uut executor <signal>'.")
            continue
        sigs.append(sig)
    return sigs, errors


def check_cxxrtl(text):
    errors = []
    m = re.search(r"kStallLabels\[\]\s*=\s*\{(.*?)\};", text, re.DOTALL)
    if m is None:
        return ["error: test/cxxrtl.cc's kStallLabels could not be found."]
    labels = re.findall(r'"([^"]*)"', m.group(1))
    if labels != CPI_REASONS:
        errors.append(
            f"error: test/cxxrtl.cc's kStallLabels is {labels}, not {CPI_REASONS}.")

    m = re.search(r"kStallReasons\[\]\s*=\s*\{(.*?)\};", text, re.DOTALL)
    if m is None:
        return errors + ["error: test/cxxrtl.cc's kStallReasons could not be found."]
    pairs = re.findall(r'\{"([^"]*)",\s*(\d+)\}', m.group(1))
    label = "test/cxxrtl.cc's kStallReasons"
    sigs, prefix_errors = cpi_signals_from_reasons_items(pairs, label)
    errors += prefix_errors
    for (item, bucket), sig in zip(
            [p for p in pairs if p[0].startswith(("uut decoder ", "uut executor "))], sigs):
        reason = SIGNAL_TO_CPI_REASON.get(sig)
        if reason is not None and reason in labels:
            want = labels.index(reason)
            if int(bucket) != want:
                errors.append(
                    f"error: {label} buckets '{sig}' at {bucket}, not {want} "
                    f"(kStallLabels' index for '{reason}').")
    errors += unknown_and_missing(label, sigs, SIGNAL_TO_CPI_REASON, CPI_REASONS)
    return errors


def check_stall_report(text):
    errors = []
    m = re.search(r"REASONS\s*=\s*\[(.*?)\]", text, re.DOTALL)
    if m is None:
        return ["error: test/stall_report.py's REASONS could not be found."]
    reasons = re.findall(r'"([^"]*)"', m.group(1))
    if reasons != CPI_REASONS:
        errors.append(
            f"error: test/stall_report.py's REASONS is {reasons}, not {CPI_REASONS}.")

    m = re.search(r"HEADINGS\s*=\s*\{(.*?)\}", text, re.DOTALL)
    if m is None:
        return errors + ["error: test/stall_report.py's HEADINGS could not be found."]
    keys = re.findall(r'"([^"]+)":', m.group(1))
    missing = set(CPI_REASONS) - set(keys)
    extra = set(keys) - set(CPI_REASONS)
    for reason in sorted(missing):
        errors.append(f"error: test/stall_report.py's HEADINGS is missing reason '{reason}'.")
    for key in sorted(extra):
        errors.append(f"error: test/stall_report.py's HEADINGS names '{key}', not one of "
                       f"the six CPI-accounting reasons.")
    return errors


def check_claude_md(text):
    flat = re.sub(r"\s+", " ", text)
    errors = []
    if CLAUDE_STALL_OWN not in flat:
        errors.append(
            f"error: CLAUDE.md no longer states rtl/decoder.v's stall_own composition "
            f"verbatim ('{CLAUDE_STALL_OWN}').")
    if CLAUDE_STALL not in flat:
        errors.append(
            f"error: CLAUDE.md no longer states rtl/decoder.v's stall composition "
            f"verbatim ('{CLAUDE_STALL}').")
    if "Six" not in flat and "six" not in flat:
        errors.append(
            "error: CLAUDE.md's commitment 8 no longer says how many reasons raise "
            "`stall`.")
    return errors


def main(argv):
    default_repo = pathlib.Path(__file__).resolve().parent.parent
    repo = pathlib.Path(argv[1]) if len(argv) > 1 else default_repo
    if not repo.is_dir():
        sys.exit(f"error: '{repo}' is not a directory, so there is nothing to scan.")

    errors = []
    errors += check_decoder_v(read(repo / "rtl" / "decoder.v"))
    errors += check_decoder_tb(read(repo / "test" / "decoder_tb.v"))
    errors += check_executor_v(read(repo / "rtl" / "executor.v"))
    errors += check_executor_tb(read(repo / "test" / "executor_tb.v"))
    errors += check_cxxrtl(read(repo / "test" / "cxxrtl.cc"))
    errors += check_stall_report(read(repo / "test" / "stall_report.py"))
    errors += check_claude_md(read(repo / "CLAUDE.md"))

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1

    print(f"the decoder's seven raw stall signals ({', '.join(DECODER_REASONS)}) and the "
          f"six CPI-accounting reasons ({', '.join(CPI_REASONS)}) each agree across "
          f"their declared sites.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
