#!/usr/bin/env python3
"""Read yosys's `stat -liberty -json` output for nanocpu, apply the NANO_MAX_UM2
ratchet, and refuse every shape of "nothing was measured".

THE ONE READER of that JSON, the way `soc/ecp5_report.py` is the one reader of
nextpnr's report and `soc/fit_report.py` of nextpnr's fit.log: `test/probe_gates.sh`
drives every refusal below against a fixture JSON and a fixture liberty file, never
against a real yosys run, so this group needs no PDK and cannot silently skip for
lack of one.

`synth; dfflibmap -liberty; abc -liberty; stat -liberty -json` is the whole
measurement -- no placement, no STA, nothing TT's own LibreLane flow later grades.

A hard macro (the register file's `rf_top`) is a black box to that script, which prices
it at zero. This reader adds its fixed footprint back from the SIZE line of the macro's
pinned LEF, prints it apart from the soft logic, and refuses a report in which the macro
appears other than exactly once: the instrument never silently drops it, and the ratchet
grades soft logic plus macro.
The brief measures TT-flow area at about 0.915x this number (FazyRV-ExoTiny:
61,673 here, shipped 56,448); the two are different instruments over different
flows and are never merged, the same rule that keeps `make fit`, `make soc-timing`
and `make ecp5-timing` apart.
"""

import argparse
import hashlib
import json
import math
import re
import sys

LEF_SIZE = re.compile(r"^\s*SIZE\s+([0-9.]+)\s+BY\s+([0-9.]+)\s*;", re.MULTILINE)
CELL_LINE = re.compile(r'^\s*cell\s*\(\s*"?([A-Za-z_][A-Za-z0-9_.]*)"?\s*\)\s*\{')


def liberty_digest_and_cells(path):
    """Hashes the liberty file and collects its cell names in one pass over its bytes,
    rather than two -- this file is the multi-megabyte sky130hd liberty.
    """
    digest = hashlib.sha256()
    names = set()
    with open(path, "rb") as handle:
        for line in handle:
            digest.update(line)
            m = CELL_LINE.match(line.decode(errors="replace"))
            if m:
                names.add(m.group(1))
    return digest.hexdigest(), names


def check_liberty(path, want_sha256, target_name="nano-area"):
    """Refuses a liberty file that is not on disk or is not the pinned bytes.

    This is checked before the JSON at all, the way `soc/ecp5_report.py` reads the
    textcfg before the report: every other refusal below is a statement about
    THIS liberty, so a substituted or stale one has to be caught first. `target_name`
    is the `make` target quoting this refusal -- `nano/timing_report.py` shares this
    function and is not `nano-area`.
    """
    try:
        got, names = liberty_digest_and_cells(path)
    except FileNotFoundError:
        sys.exit(
            f"*** make {target_name}: no liberty file at {path}. Run\n"
            "*** `make nano-liberty-setup` to fetch the pin into the tool cache."
        )
    if got != want_sha256:
        sys.exit(
            f"*** make {target_name}: {path} does not match the pinned digest --\n"
            f"***   expected : {want_sha256}\n"
            f"***   actual   : {got}\n"
            "*** The liberty is a pin, exactly as SAIL_RISCV_VERSION and\n"
            "*** RISCV_FORMAL_SHA are: bytes this repo has not verified do not\n"
            "*** get read into an area figure. Re-fetch with `make nano-liberty-setup`."
        )
    return names


def load_stat(path, target_name="nano-area"):
    try:
        with open(path) as handle:
            report = json.load(handle)
    except FileNotFoundError:
        sys.exit(
            f"*** make {target_name}: {path} does not exist, so NOTHING was\n"
            "*** measured. That is a failed run, not a zero-area design."
        )
    except json.JSONDecodeError as exc:
        sys.exit(
            f"*** make {target_name}: {path} is not JSON ({exc}). yosys writes this\n"
            "*** file last via `tee`, so a truncated one means the run died\n"
            "*** mid-script."
        )
    design = report.get("design")
    if not isinstance(design, dict):
        sys.exit(
            f"*** make {target_name}: {path} carries no 'design' totals. `stat -json`\n"
            "*** always emits one; a report missing it measured nothing."
        )
    return design


def macro_area_um2(lef_path, want_sha256, name, target_name="nano-area"):
    """The footprint of macro `name`: the SIZE line of its LEF, read only after the LEF
    matches its pinned digest, since a substituted LEF would price a different block.
    """
    try:
        with open(lef_path, "rb") as handle:
            raw = handle.read()
    except FileNotFoundError:
        sys.exit(
            f"*** make {target_name}: no macro LEF at {lef_path}. Run\n"
            "*** `make nano-rf-macro-setup` to fetch the pin into the tool cache."
        )
    got = hashlib.sha256(raw).hexdigest()
    if got != want_sha256:
        sys.exit(
            f"*** make {target_name}: {lef_path} does not match the pinned digest --\n"
            f"***   expected : {want_sha256}\n"
            f"***   actual   : {got}\n"
            "*** Re-fetch with `make nano-rf-macro-setup`."
        )
    text = raw.decode(errors="replace")
    block = re.search(r"^\s*MACRO\s+%s\b" % re.escape(name), text, re.MULTILINE)
    size = LEF_SIZE.search(text, block.end()) if block else None
    if size is None:
        sys.exit(
            f"*** make {target_name}: {lef_path} has no `MACRO {name}` with a SIZE line,\n"
            "*** so the macro's footprint cannot be read."
        )
    area = float(size.group(1)) * float(size.group(2))
    if not math.isfinite(area) or area <= 0:
        sys.exit(f"*** make {target_name}: {lef_path} gives {name} a footprint of {area} um2.")
    return area


def add_macro_args(parser):
    parser.add_argument(
        "--macro", action="append", default=[],
        help="a hard macro the synthesis script black-boxed, instantiated exactly once; "
        "its footprint comes from --macro-lef",
    )
    parser.add_argument("--macro-lef", help="the pinned LEF holding each --macro's SIZE")
    parser.add_argument(
        "--macro-lef-sha256", help="NANO_RF_MACRO_LEF_SHA256: the digest --macro-lef must match"
    )


def macro_areas(args, target_name="nano-area"):
    """{name: footprint um2} for every --macro, or {} when none was named."""
    if not args.macro:
        return {}
    if not args.macro_lef or not args.macro_lef_sha256:
        sys.exit(
            f"*** make {target_name}: --macro needs --macro-lef and --macro-lef-sha256, "
            "or the macro is priced at zero."
        )
    return {
        name: macro_area_um2(args.macro_lef, args.macro_lef_sha256, name, target_name)
        for name in args.macro
    }


def read_excluded(path, target_name="nano-area"):
    try:
        with open(path) as f:
            return {line.strip() for line in f if line.strip() and not line.startswith("#")}
    except FileNotFoundError:
        sys.exit(f"*** make {target_name}: no excluded-cell list at {path}.")


def validate_design(
    design, stat_path, liberty_path, liberty_cells, excluded_cells, target_name="nano-area",
    macros=None,
):
    """Checks one `stat -liberty -json` `design` entry against every shape of "nothing
    was measured". `nano/timing_report.py` shares this with `summarise` below, since
    both read the same report shape off a different ABC script's own mapping.

    `macros` maps each black-boxed macro to its footprint: each must be in the report
    exactly once, and its footprint is added to the returned `area`, which `soft_area`
    keeps apart from.
    """
    by_type = design.get("num_cells_by_type")
    area = design.get("area")
    num_cells = design.get("num_cells")
    if not isinstance(by_type, dict) or area is None or num_cells is None:
        sys.exit(
            f"*** make {target_name}: {stat_path}'s design entry carries "
            f"{sorted(design)},\n"
            "*** not the area, num_cells and num_cells_by_type fields "
            "`stat -liberty -json`\n"
            "*** writes. yosys has changed the shape of its report; fix "
            "nano/area_report.py\n"
            "*** rather than reading what is left."
        )

    if not isinstance(area, (int, float)) or not math.isfinite(area):
        sys.exit(
            f"*** make {target_name}: {stat_path}'s area ({area!r}) is not a finite "
            "number.\n"
            "*** `stat -liberty -json` never writes NaN or an Infinity; a report\n"
            "*** that does was not read from a real synthesis run, and every\n"
            "*** numeric comparison against it is false on either side of it."
        )

    if num_cells <= 0 or not by_type:
        sys.exit(
            f"*** make {target_name}: {stat_path} reports zero cells. That is a run\n"
            "*** that mapped nothing -- a failed `synth -top`, an empty top\n"
            "*** module, an `abc -liberty` that never ran -- not a design with\n"
            "*** no area."
        )

    macros = macros or {}
    for name in macros:
        if by_type.get(name) != 1:
            sys.exit(
                f"*** make {target_name}: {stat_path} instantiates macro {name} "
                f"{by_type.get(name, 0)} times, not once. A missing macro measured a chip\n"
                "*** without its register file; a second one is a block this instrument "
                "was not told to price."
            )
    by_type = {k: v for k, v in by_type.items() if k not in macros}

    unknown = sorted(set(by_type) - liberty_cells)
    if unknown:
        sys.exit(
            f"*** make {target_name}: {stat_path} names cell type(s) not in\n"
            f"*** {liberty_path}: {', '.join(unknown)}.\n"
            "*** `stat -liberty` only prints a cell's liberty name once it is\n"
            "*** actually mapped to one; a name outside the read liberty means\n"
            "*** either the wrong liberty was read or `abc -liberty` left an\n"
            "*** unmapped generic cell ($_DFF_, $_AND_, ...) behind, which\n"
            "*** `stat -liberty` would otherwise silently price at zero."
        )

    forbidden = sorted(set(by_type) & excluded_cells)
    if forbidden:
        sys.exit(
            f"*** make {target_name}: {stat_path} uses cell type(s) the Tiny Tapeout\n"
            f"*** flow excludes from synthesis: {', '.join(forbidden)}.\n"
            "*** A figure built from cells the flow never places measures a different\n"
            "*** chip; the synthesis script is not applying the excluded-cell list."
        )

    return {
        "area": float(area) + sum(macros.values()),
        "soft_area": float(area),
        "macros": dict(macros),
        "sequential_area": float(design.get("sequential_area", 0.0)),
        "num_cells": int(num_cells),
        "by_type": by_type,
    }


def summarise(stat_path, liberty_path, liberty_sha256, excluded_path, macros=None):
    liberty_cells = check_liberty(liberty_path, liberty_sha256, target_name="nano-area")
    excluded_cells = read_excluded(excluded_path)
    design = load_stat(stat_path)
    return validate_design(
        design, stat_path, liberty_path, liberty_cells, excluded_cells, target_name="nano-area",
        macros=macros,
    )


def finite_positive_um2(raw):
    """argparse type for --max-um2: `type=float` alone accepts the strings "nan" and
    "inf", against which `area > args.max_um2` is false for every area, real or
    forged -- closing that route the same way the area-side isfinite check above
    closes json.load's NaN/Infinity/-Infinity literals.
    """
    try:
        value = float(raw)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{raw!r} is not a number") from exc
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError(
            f"{raw!r} is not a finite, positive um2 budget"
        )
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stat_json", help="yosys `stat -liberty -json` output")
    parser.add_argument("--liberty", required=True, help="the liberty file synthesis read")
    parser.add_argument(
        "--liberty-sha256", required=True,
        help="NANO_LIBERTY_SHA256: the pinned digest the liberty file must match",
    )
    parser.add_argument(
        "--max-um2", type=finite_positive_um2, required=True, help="NANO_MAX_UM2 budget"
    )
    parser.add_argument(
        "--excluded", required=True, help="the cells the flow excludes from synthesis"
    )
    parser.add_argument(
        "--previous", type=float,
        help="the figure NANO_MAX_UM2 was last derived from, printed as a trend. "
        "Diagnostic only -- it never changes the exit status.",
    )
    add_macro_args(parser)
    args = parser.parse_args()

    s = summarise(
        args.stat_json, args.liberty, args.liberty_sha256, args.excluded, macro_areas(args)
    )
    area, num_cells = s["area"], s["num_cells"]

    print(f"cells         : {num_cells}")
    print(f"soft logic    : {s['soft_area']:.4f} um2  (sequential {s['sequential_area']:.4f} um2)")
    for name, macro_um2 in sorted(s["macros"].items()):
        print(
            f"macro {name:8s}: {macro_um2:.4f} um2  (SIZE of {args.macro_lef}, "
            f"sha256 {args.macro_lef_sha256[:12]}...)"
        )
    print(f"area          : {area:.4f} um2  (soft logic plus macros)")
    for name in sorted(s["by_type"]):
        print(f"  {name:32s} {s['by_type'][name]:6d}")

    if args.previous is not None:
        print(
            f"\nTREND: {area - args.previous:+.1f} um2 against the {args.previous:.1f} "
            "the Makefile records -- a diagnostic; only the ratchet below can fail."
        )

    if area > args.max_um2:
        sys.exit(
            f"\n*** make nano-area: {area:.1f} um2 is over the {args.max_um2:.1f} "
            "um2 budget.\n"
            "*** This is a ratchet, not a suggestion. Find what grew; raising\n"
            "*** NANO_MAX_UM2 needs a reason in the commit."
        )
    print(f"\nRATCHET: {area:.1f} of {args.max_um2:.1f} um2 budgeted -- OK")


if __name__ == "__main__":
    main()
