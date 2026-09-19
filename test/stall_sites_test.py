#!/usr/bin/env python3
"""Asserts that every place declaring a decoder stall reason names the same eight, the
way test/march_test.sh already does for the ISA string's seven sites.

Usage: stall_sites_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. `stall` is built by rtl/decoder.v out of eight named reasons and
CLAUDE.md's stall-broadcast paragraph says a reason is declared in six places. Nothing
checked that the six agreed with each other; a reason dropped from one site, or a new
signal ORed into `stall` that nobody taught the other five, went unnoticed until someone
read a CPI number and wondered why it changed.

THE TRAP THIS SCRIPT IS BUILT NOT TO FALL INTO: test/decoder_tb.v's OR-identity check ORs
NINE signals for EIGHT reasons, because hazard_rs1 and hazard_rs2 are the two halves of
one reason ("hazard"). Counting signals rather than the reasons they cover would read
that site as having a phantom ninth reason. SIGNAL_TO_REASON maps every raw signal name
onto the reason it covers -- both hazard_rs1 and hazard_rs2 land on "hazard" -- and every
site below is graded on the SET of reasons its signals cover, never on how many signals
it took to cover them.

A SIGNAL THIS SCRIPT DOES NOT RECOGNIZE IS RED, NOT SILENT. rtl/decoder.v's own stall
composition and publish arm, test/decoder_tb.v's OR-identity check, test/cxxrtl.cc's
bucket table and formal/pcloop.sv's f_may_stall are all read back through
SIGNAL_TO_REASON; a name absent from that table is reported rather than ignored, whether
it is a genuinely new stall reason nobody taught this script yet, or the future non-stall
"kill" bubble a pipeline restructure is expected to add. Either way it must not join any
of those ORs undetected: a new stall reason belongs in SIGNAL_TO_REASON and every site
below, and kill belongs only in the cycle-accounting identity test/stall_report.py
already keeps (as a column beside `issue`, never inside `stall`).

Hermetic: file reads only. No toolchain, so this runs inside `make test` anywhere.
"""

import pathlib
import re
import sys

CANONICAL_REASONS = ["divider", "atomic", "hazard", "serialize", "operand", "fetch",
                      "bus", "region"]

SIGNAL_TO_REASON = {
    "divider_stall": "divider",
    "atomic_stall": "atomic",
    "hazard": "hazard",
    "hazard_rs1": "hazard",
    "hazard_rs2": "hazard",
    "serialize": "serialize",
    "operand_stall": "operand",
    "buffer_empty": "fetch",
    "bus_wait": "bus",
    "region_stall": "region",
}

# f_may_stall cannot name a decoder-internal signal at all (a harness cannot reach inside
# an instance), so it is graded as its own fixed vocabulary rather than through
# SIGNAL_TO_REASON.
PCLOOP_MAY_STALL_TERMS = {
    "divider_stall", "buffer_empty", "bus_wait", "f_live_rs1", "f_live_rs2", "f_system",
    "f_fencei", "f_operand_fetch", "f_amo_wait", "f_load_store",
}

CLAUDE_PHRASES = {
    "divider": "the divider",
    "atomic": "the atomic write cycle",
    "hazard": "the decode scoreboard",
    "serialize": "serialization",
    "operand": "the operand-fetch cycle",
    "fetch": "an empty fetch buffer",
    "bus": "the ungranted bus",
    "region": "the load/store region wait",
}


def read(path):
    if not path.is_file():
        sys.exit(f"error: '{path}' does not exist, so there is nothing to scan.")
    return path.read_text()


def split_or_terms(rhs):
    return [t.strip() for t in re.split(r"\|\|", rhs) if t.strip()]


def assign_rhs(text, name):
    m = re.search(r"\bassign\s+" + re.escape(name) + r"\s*=\s*(.*?);", text, re.DOTALL)
    return m.group(1) if m else None


def unknown_and_missing(label, tokens, required_reasons, allowed_extra=frozenset()):
    """Grades a site by the SET of reasons its raw signal tokens cover, never by how
    many tokens it took -- the fix for the hazard_rs1/hazard_rs2 trap."""
    errors = []
    unknown = sorted({t for t in tokens if t not in SIGNAL_TO_REASON and t not in allowed_extra})
    for tok in unknown:
        errors.append(
            f"error: {label} names '{tok}', which SIGNAL_TO_REASON in this script does "
            f"not recognize as one of the eight declared stall reasons.\n"
            f"  If '{tok}' is a new stall reason: teach it to SIGNAL_TO_REASON here, then "
            f"to every other site -- rtl/decoder.v's signal, its stall composition, its "
            f"publish arm and its FORMAL asserts; test/decoder_tb.v's OR-identity check; "
            f"test/cxxrtl.cc's kStallLabels and kStallReasons; test/stall_report.py's "
            f"REASONS and HEADINGS; formal/pcloop.sv's f_may_stall; and CLAUDE.md's "
            f"stall-broadcast list.\n"
            f"  If '{tok}' is the future non-stall kill bubble: it must stay OUT of every "
            f"one of those ORs -- kill belongs in the cycle-accounting identity "
            f"test/stall_report.py already keeps, as its own column beside `issue`, "
            f"never folded into `stall`."
        )
    covered = {SIGNAL_TO_REASON[t] for t in tokens if t in SIGNAL_TO_REASON}
    for reason in sorted(set(required_reasons) - covered):
        errors.append(f"error: {label} is missing reason '{reason}'.")
    return errors


def check_decoder_v(text):
    errors = []
    label = "rtl/decoder.v's stall composition (stall_own/stall_other/stall)"
    top_terms = []
    for name, skip in (("stall_own", set()), ("stall_other", {"stall_own"}),
                       ("stall", {"stall_other"})):
        rhs = assign_rhs(text, name)
        if rhs is None:
            errors.append(f"error: {label} has no 'assign {name} = ...;' to read.")
            return errors
        top_terms += [t for t in split_or_terms(rhs) if t not in skip]
    errors += unknown_and_missing(label, top_terms, set(CANONICAL_REASONS) - {"serialize"})

    label = "rtl/decoder.v's hazard composition (assign hazard = ...)"
    hazard_rhs = assign_rhs(text, "hazard")
    if hazard_rhs is None:
        errors.append(f"error: {label} has no 'assign hazard = ...;' to read.")
    else:
        errors += unknown_and_missing(label, split_or_terms(hazard_rhs),
                                      {"hazard", "serialize"})

    label = "rtl/decoder.v's publish arm (the always_ff bubble condition)"
    m = re.search(
        r"end else if \(divider_stall\) begin\s*\n\s*out <= out;\s*\n"
        r"\s*end else if \((.*?)\) begin", text, re.DOTALL)
    if m is None:
        errors.append(f"error: {label} could not be found (divider hold then bubble arm).")
    else:
        bubble_terms = split_or_terms(m.group(1))
        errors += unknown_and_missing(label, bubble_terms,
                                      set(CANONICAL_REASONS) - {"serialize", "divider"},
                                      allowed_extra={"interrupt_pending"})

    label = "rtl/decoder.v's FORMAL asserts (the hold/bubble combo block)"
    start = text.find("logic prev_hold_and_empty")
    end_marker = "prev_region_only)     assert(out == '0);"
    end = text.find(end_marker, start) if start != -1 else -1
    if start == -1 or end == -1:
        errors.append(f"error: {label} could not be found.")
    else:
        block = text[start:end + len(end_marker)]
        for sig in ("divider_stall", "buffer_empty", "bus_wait", "region_stall",
                    "atomic_stall"):
            if not re.search(r"\b" + re.escape(sig) + r"\b", block):
                errors.append(f"error: {label} no longer names '{sig}'.")
    return errors


def check_decoder_tb(text):
    label = "test/decoder_tb.v's OR-identity check"
    m = re.search(r"if \(dut\.stall !== \((.*?)\)\) begin", text, re.DOTALL)
    if m is None:
        return [f"error: {label} could not be found."]
    terms = []
    for t in split_or_terms(m.group(1)):
        terms.append(t[len("dut."):] if t.startswith("dut.") else t)
    return unknown_and_missing(label, terms, set(CANONICAL_REASONS))


def check_cxxrtl(text):
    errors = []
    m = re.search(r"kStallLabels\[\]\s*=\s*\{(.*?)\};", text, re.DOTALL)
    if m is None:
        return ["error: test/cxxrtl.cc's kStallLabels could not be found."]
    labels = re.findall(r'"([^"]*)"', m.group(1))
    if labels != CANONICAL_REASONS:
        errors.append(
            f"error: test/cxxrtl.cc's kStallLabels is {labels}, not {CANONICAL_REASONS}.")

    m = re.search(r"kStallReasons\[\]\s*=\s*\{(.*?)\};", text, re.DOTALL)
    if m is None:
        return errors + ["error: test/cxxrtl.cc's kStallReasons could not be found."]
    pairs = re.findall(r'\{"([^"]*)",\s*(\d+)\}', m.group(1))
    prefix = "uut decoder "
    sigs = []
    for item, bucket in pairs:
        if not item.startswith(prefix):
            errors.append(
                f"error: test/cxxrtl.cc's kStallReasons has an item '{item}' not shaped "
                f"'{prefix}<signal>'.")
            continue
        sig = item[len(prefix):]
        sigs.append(sig)
        reason = SIGNAL_TO_REASON.get(sig)
        if reason is not None and reason in labels:
            want = labels.index(reason)
            if int(bucket) != want:
                errors.append(
                    f"error: test/cxxrtl.cc's kStallReasons buckets '{sig}' at {bucket}, "
                    f"not {want} (kStallLabels' index for '{reason}').")
    errors += unknown_and_missing("test/cxxrtl.cc's kStallReasons", sigs,
                                  set(CANONICAL_REASONS))
    return errors


def check_stall_report(text):
    errors = []
    m = re.search(r"REASONS\s*=\s*\[(.*?)\]", text, re.DOTALL)
    if m is None:
        return ["error: test/stall_report.py's REASONS could not be found."]
    reasons = re.findall(r'"([^"]*)"', m.group(1))
    if reasons != CANONICAL_REASONS:
        errors.append(
            f"error: test/stall_report.py's REASONS is {reasons}, not {CANONICAL_REASONS}.")

    m = re.search(r"HEADINGS\s*=\s*\{(.*?)\}", text, re.DOTALL)
    if m is None:
        return errors + ["error: test/stall_report.py's HEADINGS could not be found."]
    keys = re.findall(r'"([^"]+)":', m.group(1))
    missing = set(CANONICAL_REASONS) - set(keys)
    extra = set(keys) - set(CANONICAL_REASONS)
    for reason in sorted(missing):
        errors.append(f"error: test/stall_report.py's HEADINGS is missing reason '{reason}'.")
    for key in sorted(extra):
        errors.append(f"error: test/stall_report.py's HEADINGS names '{key}', not one of "
                       f"the eight declared stall reasons.")
    return errors


def check_pcloop(text):
    rhs = assign_rhs(text, "f_may_stall")
    if rhs is None:
        return ["error: formal/pcloop.sv's f_may_stall could not be found."]
    terms = set(split_or_terms(rhs))
    errors = []
    for missing in sorted(PCLOOP_MAY_STALL_TERMS - terms):
        errors.append(f"error: formal/pcloop.sv's f_may_stall no longer names '{missing}'.")
    for extra in sorted(terms - PCLOOP_MAY_STALL_TERMS):
        errors.append(
            f"error: formal/pcloop.sv's f_may_stall names '{extra}', which this script "
            f"does not expect there. f_may_stall is a deliberate over-approximation built "
            f"from signals pcloop can read from outside the decoder instance, never the "
            f"decoder's own named reasons (a harness cannot reach inside an instance); if "
            f"this widens the approximation on purpose, teach PCLOOP_MAY_STALL_TERMS.")
    return errors


def check_claude_md(text):
    flat = re.sub(r"\s+", " ", text)
    m = re.search(
        r"reasons raise `stall`, and it is exactly their OR:(.*?)\.", flat)
    if m is None:
        return ["error: CLAUDE.md's 'reasons raise `stall`...' sentence could not be found."]
    sentence = m.group(1)
    errors = []
    for reason in CANONICAL_REASONS:
        phrase = CLAUDE_PHRASES[reason]
        if phrase not in sentence:
            errors.append(
                f"error: CLAUDE.md's stall-broadcast sentence no longer names '{phrase}' "
                f"({reason}).")
    return errors


def main(argv):
    default_repo = pathlib.Path(__file__).resolve().parent.parent
    repo = pathlib.Path(argv[1]) if len(argv) > 1 else default_repo
    if not repo.is_dir():
        sys.exit(f"error: '{repo}' is not a directory, so there is nothing to scan.")

    errors = []
    errors += check_decoder_v(read(repo / "rtl" / "decoder.v"))
    errors += check_decoder_tb(read(repo / "test" / "decoder_tb.v"))
    errors += check_cxxrtl(read(repo / "test" / "cxxrtl.cc"))
    errors += check_stall_report(read(repo / "test" / "stall_report.py"))
    errors += check_pcloop(read(repo / "formal" / "pcloop.sv"))
    errors += check_claude_md(read(repo / "CLAUDE.md"))

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1

    print(f"the eight stall reasons ({', '.join(CANONICAL_REASONS)}) agree across all "
          f"six declared sites.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
