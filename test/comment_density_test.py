#!/usr/bin/env python3
"""Fails a file whose comment lines are more than THRESHOLD_PCT of its
non-blank lines. THRESHOLD_PCT is declared once, in this file -- see
docs/comment-budget.md for how it was chosen.

Usage: comment_density_test.py [repo-root]     # defaults to this script's parent

Comment syntax is per line, per language: `//`/`/* */` for SystemVerilog and
C-family, `#` for shell/Python/Make and this repo's own `#`-driven manifests,
both for `.S` (a `.S` line starting `#include`/`#define`/etc. is code, not
comment). An unrecognised extension is not scanned.

MIN_COMMENT_LINES excuses a file whose entire comment content is one or two
lines, so a manifest reduced to a single pointer does not fail for having
little else in it.

Hermetic: git and file reads. No toolchain, so this runs inside `make test`
anywhere.
"""

import argparse
import io
import os
import re
import subprocess
import sys
import tokenize

THRESHOLD_PCT = 5.0
MIN_COMMENT_LINES = 2

HASH_STYLE = {".sh", ".mk", ".cfg", ".sby", ".pcf", ".toml"}
PY_STYLE = {".py"}
YAML_STYLE = {".yml", ".yaml"}
BOTH_STYLE = {".lpf"}   # ECP5 constraint files use either marker
C_STYLE = {".v", ".sv", ".c", ".h", ".cc", ".cpp", ".lds"}
HASH_BASENAMES = {"Makefile", ".gitignore", ".gitattributes"}
ASM_STYLE = {".S"}

# Manifests with no extension that speak this repo's `#`-baseline dialect.
HASH_MANIFESTS = {
    "test/EXPECTED_FAIL",
    "test/COSIM_EXPECTED_FAIL",
    "test/OBSERVED_FLOOR",
    "test/MUTATION_DETECTORS",
    "test/MUTATION_COVERAGE",
    "test/PROBES_EXPECTED",
    "test/dual/MUTATION_PAIRINGS",
    "formal/EXPECTED_FAIL",
    "formal/EXPECTED_CHECKS",
    "formal/MULTIHART_TIE_OFF",
    "formal/INTERRUPT_TIE_OFF",
    "formal/COMPLETE_EXCLUSIONS",
    "nano/formal/EXPECTED_FAIL",
    "nano/formal/EXPECTED_CHECKS",
}

EXCLUDE_PREFIXES = (
    "test/bench/coremark/",   # vendored unmodified from eembc/coremark
    "soc/compare/vexriscv/",  # generated from the pinned riscv-formal clone
)

EXCLUDE_FILES = {
    "test/monitor.v",             # generated at the riscv-formal pin
    "formal/genchecks-local.py",  # must differ from the pin by header and basedir only
    "test/bench/dhry.h",          # Dhrystone 2.1 as Weicker published it
    "test/bench/dhry_1.c",
    "test/bench/dhry_2.c",
}

# Lines that carry a comment marker and are CODE. Each is parsed by something:
# `#derive`/`#floor`/`#omit` by formal/depth_rules.py and genchecks-audit.py,
# `// EXCLUDE` by formal/check-complete-exclusions.py (which also requires 40
# characters of prose under it), a shebang by the kernel, and the rest by a
# linter. Exempting the whole file instead would stop grading its real prose.
FUNCTIONAL = re.compile(
    r"^(#!"
    r"|\s*(#derive|#floor|#omit)\b"
    r"|\s*//\s*EXCLUDE\b"
    r"|\s*(#|//)\s*(shellcheck|noqa|pylint|type:|nosec|yamllint|codespell|ruff:|mypy:"
    r"|fmt:|isort:|pragma|coding[:=]|-\*-|SPDX|Copyright|See LICENSE|Licensed under"
    r"|verilator|lint_off|lint_on|synthesis|synopsys))", re.I)

EXPLICIT_EXEMPT = set()

PREPROC_DIRECTIVES = (
    "include", "define", "ifdef", "ifndef", "endif", "else", "undef", "pragma",
    "if",
)


def _style_for(path):
    base = os.path.basename(path)
    if base in HASH_BASENAMES:
        return "hash"
    ext = os.path.splitext(path)[1]
    if ext in ASM_STYLE:
        return "asm"
    if ext in PY_STYLE:
        return "python"
    if ext in YAML_STYLE:
        return "yaml"
    if ext in BOTH_STYLE:
        return "both"
    if ext in HASH_STYLE:
        return "hash"
    if ext in C_STYLE:
        return "c"
    if path in HASH_MANIFESTS:
        return "hash"
    return None


HEREDOC = re.compile(r"""<<-?\s*['"]?([A-Za-z_][A-Za-z0-9_]*)['"]?\s*(?:$|[|&;)])""")
BLOCK_SCALAR = re.compile(r":\s*[|>][-+]?\d*\s*$")


def _classify_hash(lines, markers=("#",)):
    """Hash-commented text. A `#` inside a heredoc is the heredoc's, not ours."""
    code = comment = 0
    heredoc = None
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if heredoc is not None:
            code += 1
            if stripped == heredoc:
                heredoc = None
            continue
        if stripped.startswith(markers) and not FUNCTIONAL.match(line):
            comment += 1
        else:
            code += 1
        opener = HEREDOC.search(line)
        if opener:
            heredoc = opener.group(1)
    return code, comment


def _classify_yaml(lines):
    """YAML. A `#` inside a block scalar is part of the script it holds."""
    code = comment = 0
    indent = 0
    inside = False
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if inside:
            if len(line) - len(line.lstrip()) <= indent:
                inside = False
            else:
                code += 1
                continue
        if stripped.startswith("#") and not FUNCTIONAL.match(line):
            comment += 1
        else:
            code += 1
        if BLOCK_SCALAR.search(line):
            inside = True
            indent = len(line) - len(line.lstrip())
    return code, comment


def _classify_python(lines):
    """Python through `tokenize`: a `#` inside a string is never a comment, and
    a shebang is not one either."""
    text = "\n".join(lines)
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(text).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return _classify_hash(lines)
    marked = set()
    for tok in toks:
        if tok.type != tokenize.COMMENT:
            continue
        row = tok.start[0] - 1
        if lines[row][:tok.start[1]].strip():
            continue                       # a trailing comment; the line is code
        if FUNCTIONAL.match(lines[row]):
            continue
        marked.add(row)
    code = sum(1 for i, l in enumerate(lines) if l.strip() and i not in marked)
    return code, len(marked)


def _classify_asm(lines):
    code = comment = 0
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if FUNCTIONAL.match(line):
            code += 1
        elif stripped.startswith("//"):
            comment += 1
        elif stripped.startswith("#"):
            directive = stripped[1:].lstrip().split(None, 1)
            word = directive[0] if directive else ""
            if word in PREPROC_DIRECTIVES:
                code += 1
            else:
                comment += 1
        else:
            code += 1
    return code, comment


def _classify_c(lines):
    code = comment = 0
    in_block = False
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if FUNCTIONAL.match(line):
            code += 1
            continue
        rest = stripped
        line_is_comment = False
        line_has_code = False
        while rest:
            if in_block:
                line_is_comment = True
                end = rest.find("*/")
                if end == -1:
                    rest = ""
                else:
                    in_block = False
                    rest = rest[end + 2:].strip()
                continue
            if rest.startswith("//"):
                line_is_comment = True
                rest = ""
                continue
            if rest.startswith("/*"):
                line_is_comment = True
                in_block = True
                rest = rest[2:]
                continue
            # Some non-comment text sits here; find the next comment opener
            # (if any) on the same line and treat everything before it as code.
            openers = [i for i in (rest.find("//"), rest.find("/*")) if i != -1]
            line_has_code = True
            rest = rest[min(openers):] if openers else ""
        if line_has_code:
            code += 1
        elif line_is_comment:
            comment += 1
    return code, comment


def classify(path, lines):
    style = _style_for(path)
    if style is None:
        return None
    if style == "hash":
        return _classify_hash(lines)
    if style == "both":
        return _classify_hash(lines, markers=("#", "//"))
    if style == "yaml":
        return _classify_yaml(lines)
    if style == "python":
        return _classify_python(lines)
    if style == "asm":
        return _classify_asm(lines)
    return _classify_c(lines)


def tracked(root):
    out = subprocess.run(["git", "-C", root, "ls-files"], check=True,
                          capture_output=True, text=True).stdout
    return [line for line in out.splitlines() if line]


def excluded(path):
    if path in EXCLUDE_FILES or path in EXPLICIT_EXEMPT:
        return True
    return any(path.startswith(prefix) for prefix in EXCLUDE_PREFIXES)


def scan(root):
    """Yields (path, code, comment) for every graded file."""
    for path in tracked(root):
        if excluded(path):
            continue
        full = os.path.join(root, path)
        try:
            with open(full, encoding="utf-8") as handle:
                lines = handle.read().splitlines()
        except (OSError, UnicodeDecodeError):
            continue
        counted = classify(path, lines)
        if counted is None:
            continue
        code, comment = counted
        if code + comment == 0:
            continue
        yield path, code, comment


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", default=os.path.dirname(here))
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    if not os.path.isdir(root):
        sys.exit(f"error: '{root}' is not a directory, so there is nothing to scan.")

    failures = []
    scanned = 0
    for path, code, comment in scan(root):
        scanned += 1
        total = code + comment
        pct = 100.0 * comment / total
        if comment > MIN_COMMENT_LINES and pct > THRESHOLD_PCT:
            failures.append((path, comment, total, pct))

    if scanned == 0:
        sys.exit("error: no file in this tree matched a recognised comment "
                  "convention. That is this check finding nothing to grade, "
                  "not a clean result.")

    if failures:
        print(f"*** {len(failures)} file(s) exceed the {THRESHOLD_PCT:g}% "
              f"comment-density budget (docs/comment-budget.md):")
        for path, comment, total, pct in sorted(failures):
            print(f"***   {path}: {comment}/{total} lines are comment "
                  f"({pct:.1f}%)")
        sys.exit(1)
    print(f"comment-density: {scanned} file(s) scanned, all at or under "
          f"{THRESHOLD_PCT:g}%.")


if __name__ == "__main__":
    main()
