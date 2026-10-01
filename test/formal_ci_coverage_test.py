#!/usr/bin/env python3
"""Asserts that every target in the `all` list of formal/Makefile and nano/formal/Makefile
is run by a CI step.

Usage: formal_ci_coverage_test.py [repo-root]     # defaults to this script's parent

WHY THIS EXISTS. `all` is each design's statement of what its formal suite is, and CI is
where that suite is graded. Nothing tied the two: nano's `components_traps`, `ill_e` and
`ill_e_cover` sat in `all` for weeks with no CI step, so the only oracle for nano's trap
entry and interrupt path ran on nobody's machine but the engineer's.

A target counts as run when a `make -C <dir> ...` line in ci.yml names it, or when a
target that line names lists it as a prerequisite, transitively: `complete` runs
`complete-exclusions` because the rule says so, and the workflow need not repeat it. A
matrix step written `components_${{ matrix.proof }}` expands over every `proof:` value
the workflow declares.

ONE SPELLING IS AN EQUIVALENCE, NOT A NAME. `check` is `checks`, a parallel run of the
generated set, and `check-baseline`. CI slices the run into `check-shard` jobs and grades
the union in a collector that calls check-baseline.sh directly, so `check` is run when
both halves are present: a `check-shard` line and a check-baseline.sh line naming that
design's checks directory.

EXCEPTIONS is the list of targets a design's `all` names and CI deliberately does not
run, each with its reason. It is empty: every target in both lists is run.

Hermetic: file reads only.
"""

import pathlib
import re
import sys

DESIGNS = ("formal", "nano/formal")

# {design: {target: reason}}
EXCEPTIONS = {"formal": {}, "nano/formal": {}}


def read(path):
    try:
        return path.read_text()
    except OSError as err:
        raise SystemExit(f"error: cannot read {path}: {err}")


def logical_lines(text):
    """Makefile or YAML text with backslash-continued lines joined."""
    return re.sub(r"\\\n[ \t]*", " ", text).splitlines()


def parse_rules(makefile_text):
    """{target: set(prerequisites)} for every plain (non-pattern) rule, order-only
    prerequisites dropped. A target named by several rules gets the union."""
    graph = {}
    for line in logical_lines(makefile_text):
        if not line or line[0] in " \t#" or "=" in line.split(":", 1)[0]:
            continue
        match = re.match(r"^([^:=%]+?)\s*:(?!=)\s*([^|#]*)", line)
        if not match:
            continue
        targets, prereqs = match.group(1).split(), match.group(2).split()
        for target in targets:
            if target.startswith(".") or "%" in target:
                continue
            graph.setdefault(target, set()).update(p for p in prereqs if "%" not in p)
    return graph


def run_targets(ci_text, design):
    """Targets a `make -C <design>` line in ci.yml names, matrix values expanded."""
    proofs = re.findall(r"^\s*-?\s*proof:\s*(\w+)\s*$", ci_text, re.M)
    proofs += [p.strip() for group in re.findall(r"proof:\s*\[([^\]]*)\]", ci_text)
               for p in group.split(",")]
    named = set()
    line_re = re.compile(r"\bmake\s+-C\s+" + re.escape(design) + r"\s+([^\n]*)")
    for line in logical_lines(ci_text):
        line = re.sub(r"(^|\s)#.*$", "", line)
        if re.match(r"\s*-?\s*name:", line):
            continue
        for match in line_re.finditer(line):
            for token in match.group(1).replace("${{ matrix.proof }}", "${{proof}}").split():
                if token.startswith((">", "2>", "|", "&&", ";")):
                    break
                if "=" in token:
                    continue
                if "${{proof}}" in token:
                    named.update(token.replace("${{proof}}", p) for p in proofs)
                elif "${{" not in token:
                    named.add(token)
    return named


def closure(graph, roots):
    seen, stack = set(), list(roots)
    while stack:
        target = stack.pop()
        if target in seen:
            continue
        seen.add(target)
        stack.extend(graph.get(target, ()))
    return seen


def check_design(root, design, ci_text):
    makefile = root / design / "Makefile"
    graph = parse_rules(read(makefile))
    if "all" not in graph or not graph["all"]:
        return [f"{makefile} has no `all` rule with prerequisites, so there is nothing to grade."]
    named = run_targets(ci_text, design)
    covered = closure(graph, named)
    baseline = re.search(r"check-baseline\.sh\s+" + re.escape(design) + r"/checks\b", ci_text)
    if "check-shard" in named and baseline:
        covered.add("check")
    problems = []
    for target in sorted(graph["all"]):
        if target in covered:
            continue
        if target in EXCEPTIONS[design]:
            continue
        problems.append(
            f"{design}/Makefile's `all` names {target}, and no ci.yml step runs it (by "
            f"name or as a prerequisite of a target that does).\n"
            f"  Add a `make -C {design} {target}` step to the workflow beside its "
            f"siblings, or record it in EXCEPTIONS with the reason CI cannot."
        )
    for target in sorted(EXCEPTIONS[design]):
        if target not in graph["all"]:
            problems.append(
                f"EXCEPTIONS names {design} target {target}, which is not in `all`.")
        elif target in covered:
            problems.append(
                f"EXCEPTIONS names {design} target {target}, which CI runs anyway.")
    return problems


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1
                        else pathlib.Path(__file__).resolve().parent.parent)
    if not root.is_dir():
        print(f"error: {root} is not a directory", file=sys.stderr)
        return 1
    ci_text = read(root / ".github" / "workflows" / "ci.yml")
    problems = []
    for design in DESIGNS:
        problems += check_design(root, design, ci_text)
    if problems:
        print("FORMAL CI COVERAGE: FAIL", file=sys.stderr)
        for problem in problems:
            print("  " + problem, file=sys.stderr)
        return 1
    for design in DESIGNS:
        graph = parse_rules(read(root / design / "Makefile"))
        print(f"{design}: all {len(graph['all'])} targets in `all` are run by ci.yml")
    print("FORMAL CI COVERAGE: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
