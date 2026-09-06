#!/usr/bin/env python3
"""Refuses two shapes of drift in test/probe_gates.sh's own fixtures.

A bare `sed -i` proves nothing about the fixture it edits: a pattern that
matches nothing exits 0 having mutated nothing, and the probe built on the
unmutated copy still goes red, but for the wrong reason ("exited 0, expected
1"), which accuses the grader under test rather than the fixture that drifted.
Every mutation is required to go through `mutate`/`mutate_remove`, which
compare the file before and after and fail by name when nothing changed.

A fixture that TYPES OUT an artifact's shape by hand -- a nextpnr utilisation
block, a `localparam` line -- rather than copying the real file has the
opposite failure mode: it never goes red on its own, because nothing checks
that the shape it invented still matches anything real. It just grades a
format nothing produces any more, forever. `fixture_anchor` closes that by
requiring the fixture to name the real file (or, for a generated artifact with
no tracked source, the parser that reads it) and the literal text it copied
from there.

Usage: fixture_freshness_test.py [repo-root]   # defaults to this script's parent

Both checks compare against an ALLOWLIST below, graded BOTH ways like every
other table in this repo: a site outside the allowlist is red because it is
unreviewed, and an allowlist entry that no longer describes anything in the
file is red too, because an exemption kept past its reason is how the next
one gets waved through. Converting an allowlisted site is a one-line deletion
here in the same commit that fixes it.

Hermetic: reads test/probe_gates.sh as text. No toolchain, no simulator, no
yosys, so this runs inside `make test` anywhere.
"""
import os
import re
import sys

# Every remaining raw `sed -i` in test/probe_gates.sh, normalized (leading and
# trailing whitespace stripped). Empty on purpose: every call site converted to
# `mutate`/`mutate_remove` in the same change that added them. An entry here is
# a call site not yet converted -- state which one and why it is still bare.
SED_I_ALLOWLIST = []

# Fixture functions (name containing "fixture") that type out an artifact's
# shape with a literal heredoc, copy no real file, and carry no
# `fixture_anchor` -- so a rewritten format would leave them grading nothing
# real, silently. Not yet converted; each entry is a function name to anchor
# next, not a permanent exemption.
FIXTURE_ANCHOR_ALLOWLIST = {
    "cp_fixture": "test/cosim.py's own trace/dut-output shapes, invented for this suite",
    "ts_fixture": "an icetime timing report, invented for soc/timing_split.py",
    "ts_carry_fixture": "an icetime timing report with a carry-hop column",
    "rb_fixture": "soc/routing_bins.py's routing report shape",
    "cc_fixture": "a yosys cell census, invented for soc/cell_census.py",
    "ecp5_fixture": "nextpnr-ecp5's report/config pair for soc/ecp5_report.py",
    "sr_fixture": "test/stall_report.py's own per-program counts line",
    "fr_fixture": "a fault-channel trace shape",
    "dd_fixture": "soc/compare/dhry_dmips.py's run.log shape",
    "dd_fixture3": "the three-core variant of dhry_dmips.py's run.log shape",
    "dd_fixture_solo": "the solo-core variant of dhry_dmips.py's run.log shape",
    "cd_fixture": "soc/compare/coremark_dmips.py's run.log shape",
}

FUNC_START_RE = re.compile(r'^([a-zA-Z0-9_]+)\(\) \{(\s*#.*)?$')
# ONE definition of "a heredoc opens here", read by both the masker below and
# the anchor check: an earlier pair of regexes disagreed about the UNQUOTED
# delimiter a fixture needs when its body interpolates a `$1`, so the anchor
# check skipped `br_fixture` -- the fixture behind the only detector of a block
# RAM read through its own reset -- while the masker saw it.
# `(?<!<)`/`(?!<)` rule out a here-string (`<<<`), which is not a heredoc and
# has no closing delimiter line to hunt for -- matching it here sent an
# earlier version of this scan looking for a line that never comes and masked
# the rest of the file.
HEREDOC_START_RE = re.compile(r"(?<!<)<<(?!<)-?\s*'?([A-Za-z_][A-Za-z_0-9]*)'?")


def heredoc_mask(lines):
    """True at every line that is BODY TEXT of a heredoc (or its own closing
    delimiter), so neither check below mistakes planted fixture text -- this
    file's own probes for these checks plant a fake `sed -i` and a fake
    `_fixture() {` this way -- for a real invocation or a real function."""
    mask = [False] * len(lines)
    i, n = 0, len(lines)
    while i < n:
        m = HEREDOC_START_RE.search(lines[i])
        if m:
            token = m.group(1)
            j = i + 1
            while j < n and lines[j].rstrip('\n').strip() != token:
                mask[j] = True
                j += 1
            if j < n:
                mask[j] = True
            i = j + 1
        else:
            i += 1
    return mask


def function_bodies(lines, mask):
    """name -> (start, end), 0-based, end inclusive, for every top-level
    `name() {` ... `}` in the file. This file's convention is a bare `}` at
    the start of its own line closing every such function, so that is the
    boundary read here."""
    bodies = {}
    i, n = 0, len(lines)
    while i < n:
        if mask[i]:
            i += 1
            continue
        m = FUNC_START_RE.match(lines[i])
        if m:
            start = i
            j = i + 1
            while j < n and lines[j].rstrip('\n') != '}':
                j += 1
            bodies[m.group(1)] = (start, j)
            i = j + 1
        else:
            i += 1
    return bodies


def unquoted_sed_i_lines(text):
    """0-based line indices where `sed -i` starts OUTSIDE any quote, tracking
    quote state across the whole file the way a shell would. This is what
    tells an actual invocation apart from the same six characters sitting
    inside a probe's quoted fixture text or its expected-output string --
    both of which this file's own probes for this check have to plant."""
    in_squote = in_dquote = False
    line = 0
    hits = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '\n':
            line += 1
            i += 1
            continue
        if in_squote:
            if c == "'":
                in_squote = False
            i += 1
            continue
        if in_dquote:
            if c == '\\' and i + 1 < n:
                # A backslash escapes the next character, and that character
                # is a NEWLINE on every line-continued command here. Counting
                # it is what keeps the index reported below the line the text
                # is actually on: an earlier version skipped it, drifted 58
                # lines by the end of the file, and dropped a real `sed -i`
                # because the line it named happened to be masked.
                if text[i + 1] == '\n':
                    line += 1
                i += 2
                continue
            if c == '"':
                in_dquote = False
            i += 1
            continue
        if c == '\\' and i + 1 < n:
            if text[i + 1] == '\n':
                line += 1
            i += 2
            continue
        if c == "'":
            in_squote = True
            i += 1
            continue
        if c == '"':
            in_dquote = True
            i += 1
            continue
        if c == '#':
            nl = text.find('\n', i)
            i = nl if nl != -1 else n
            continue
        if text.startswith('sed -i', i):
            hits.append(line)
        i += 1
    return hits


def check_sed_i(lines, mask, exclude_ranges):
    text = ''.join(lines)
    hits = []
    for idx in unquoted_sed_i_lines(text):
        if mask[idx]:
            continue
        if any(lo <= idx <= hi for lo, hi in exclude_ranges):
            continue
        hits.append((idx + 1, lines[idx].strip()))

    rc = 0
    allowed = set(SED_I_ALLOWLIST)
    seen = set()
    for lineno, call in hits:
        seen.add(call)
        if call not in allowed:
            rc = 1
            print(
                f"error: test/probe_gates.sh:{lineno} calls `sed -i` directly: "
                f"{call}",
                file=sys.stderr,
            )
            print(
                "Route it through mutate/mutate_remove, which fail by name when "
                "the mutation lands on nothing.",
                file=sys.stderr,
            )
    for call in allowed:
        if call not in seen:
            rc = 1
            print(
                f"error: SED_I_ALLOWLIST exempts '{call}', and it no longer "
                f"appears in test/probe_gates.sh. Delete the entry.",
                file=sys.stderr,
            )
    return rc


def check_fixture_anchors(lines, mask):
    bodies = function_bodies(lines, mask)
    rc = 0
    unanchored = set()
    for name, (start, end) in bodies.items():
        if 'fixture' not in name:
            continue
        body = ''.join(lines[start:end + 1])
        has_heredoc = bool(HEREDOC_START_RE.search(body))
        has_cp_repo = 'cp "$REPO' in body or "cp '$REPO" in body
        has_anchor = 'fixture_anchor' in body
        if has_heredoc and not has_cp_repo and not has_anchor:
            unanchored.add(name)

    for name in sorted(unanchored):
        if name not in FIXTURE_ANCHOR_ALLOWLIST:
            rc = 1
            start = bodies[name][0] + 1
            print(
                f"error: test/probe_gates.sh:{start} {name}() types out an "
                f"artifact's shape with no fixture_anchor and no cp \"$REPO/...\" "
                f"of a real file. It will grade a format nothing produces any "
                f"more and never go red on its own.",
                file=sys.stderr,
            )
    for name, reason in FIXTURE_ANCHOR_ALLOWLIST.items():
        if name not in unanchored:
            rc = 1
            print(
                f"error: FIXTURE_ANCHOR_ALLOWLIST exempts '{name}' ({reason}), "
                f"and it is no longer an anchorless synthetic fixture. Delete "
                f"the entry.",
                file=sys.stderr,
            )
    return rc


def main():
    repo = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..'
    )
    path = os.path.join(repo, 'test', 'probe_gates.sh')
    if not os.path.isfile(path):
        sys.exit(f"error: '{path}' does not exist, so there is nothing to scan.")

    with open(path) as f:
        lines = f.readlines()

    mask = heredoc_mask(lines)
    bodies = function_bodies(lines, mask)
    exclude_ranges = [
        bodies[name] for name in ('mutate', 'mutate_remove', 'fixture_anchor')
        if name in bodies
    ]
    if len(exclude_ranges) != 3:
        sys.exit(
            "error: mutate(), mutate_remove() and fixture_anchor() are not all "
            "defined in test/probe_gates.sh, so a raw sed -i cannot be told "
            "apart from their own implementation."
        )

    rc = check_sed_i(lines, mask, exclude_ranges)
    rc |= check_fixture_anchors(lines, mask)
    if rc:
        sys.exit(1)

    print(
        "no bare sed -i outside mutate()/mutate_remove(), "
        f"{len(FIXTURE_ANCHOR_ALLOWLIST)} fixtures still owed an anchor"
    )


if __name__ == '__main__':
    main()
