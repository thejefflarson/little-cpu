#!/usr/bin/env python3
"""Refuses two shapes of drift in test/probe_gates.sh's own fixtures.

A bare `sed -i` proves nothing about the fixture it edits: a pattern that
matches nothing exits 0 having mutated nothing, and the probe built on the
unmutated copy still goes red, but for the wrong reason ("exited 0, expected
1"), which accuses the grader under test rather than the fixture that drifted.
Every mutation is required to go through `mutate`/`mutate_remove`, which
compare the file before and after and fail by name when nothing changed. The
same failure mode has other spellings -- `sed --in-place`, `perl -i`,
`awk -i inplace` -- and this file catches those too.

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

Both checks read ONE quote/comment/escape-aware scan of the shell text
(`_live_chars`), never two: a divergence between two hand-rolled trackers is
how a fixture line went invisible to one check and not the other here before.
`shlex` does not replace it -- it opens a comment on any `#`, including one
buried mid-word, and knows nothing of heredocs or brace depth.

Hermetic: reads test/probe_gates.sh as text. No toolchain, no simulator, no
yosys, so this runs inside `make test` anywhere.
"""
import os
import re
import sys

# Every remaining raw in-place edit in test/probe_gates.sh, normalized
# (leading and trailing whitespace stripped). Empty on purpose: every call
# site converted to `mutate`/`mutate_remove` in the same change that added
# them. An entry here is a call site not yet converted -- state which one and
# why it is still bare.
RAW_EDIT_ALLOWLIST = []

# Fixture functions (name containing "fixture") that type out an artifact's shape with a
# literal heredoc, copy no real file, and carry no `fixture_anchor` -- so a rewritten
# format would leave them grading nothing real, silently.
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

# `{` need not be last on the line: a head that opens its body on the same
# line is a definition too, and _find_function_end reads from the head itself,
# so the rest of that line is already counted.
FUNC_START_RE = re.compile(r'^([a-zA-Z0-9_]+)\(\)\s*\{')
# ONE definition of "a heredoc opens here", read by both the masker below and
# the anchor check: an earlier pair of regexes disagreed about the UNQUOTED
# delimiter a fixture needs when its body interpolates a `$1`, so the anchor
# check skipped `br_fixture` -- the fixture behind the only detector of a block
# RAM read through its own reset -- while the masker saw it. The delimiter may
# be bare, single-quoted or double-quoted; `\1` requires the closing mark, if
# any, to match the opening one.
# `(?<!<)`/`(?!<)` rule out a here-string (`<<<`), which is not a heredoc and
# has no closing delimiter line to hunt for -- matching it here sent an
# earlier version of this scan looking for a line that never comes and masked
# the rest of the file. Group 1 is the `-` of `<<-`, which is the ONLY form
# that strips the closing delimiter line's leading whitespace; a plain
# `<<TOK` requires that line to have none, and stripping it anyway closes the
# mask early on an indented line that merely equals TOK after stripping.
HEREDOC_START_RE = re.compile(
    r"(?<!<)<<(-)?(?!<)\s*([\"'])?([A-Za-z_][A-Za-z_0-9]*)\2?"
)

def heredoc_mask(lines):
    """True at every line that is BODY TEXT of a heredoc (or its own closing
    delimiter), so neither check below mistakes planted fixture text -- this
    file's own probes for these checks plant a fake `sed -i` and a fake
    `_fixture() {` this way -- for a real invocation or a real function.
    `cmd <<A <<B` is legal bash and opens two heredocs off one line, A's body
    first and then B's, so every `<<` on the line is walked in order rather
    than just the first."""
    mask = [False] * len(lines)
    i, n = 0, len(lines)
    while i < n:
        matches = list(HEREDOC_START_RE.finditer(lines[i]))
        if matches:
            j = i + 1
            for m in matches:
                token = m.group(3)
                strip = m.group(1) == '-'
                while j < n:
                    body_line = lines[j].rstrip('\n')
                    delim = body_line.strip() if strip else body_line
                    mask[j] = True
                    j += 1
                    if delim == token:
                        break
            i = j
        else:
            i += 1
    return mask


def _live_chars(lines, mask):
    """Yields (lineno, char) for every character of real, unquoted,
    uncommented shell text. A heredoc-body line yields nothing; quoted text
    and an escaped character are DATA and yield nothing, so neither a quote
    nor a brace can be smuggled past by escaping it; `#` opens a comment only
    at the start of a word, so `x=foo#bar sed -i f` cannot hide what follows.
    A trailing unescaped backslash eats its own newline the way bash does,
    and every other line boundary yields a real newline, so two lines that
    are not continued cannot glue into one word.

    `$'...'` is its own quote form, not a `$` beside a plain `'...'`: inside
    it a backslash escapes the next character, so `\\'` is a literal quote
    that does NOT close the string, the same escaping rule a double-quoted
    string already gets and a plain single-quoted one does not.
    `escaping_quote` holds the two escaping forms in one state, closed by
    whichever character opened it (`"` or `'`), so that rule is written once.
    """
    in_squote = False
    escaping_quote = None
    at_word_start = True
    for lineno, raw in enumerate(lines):
        if mask[lineno]:
            continue
        line = raw.rstrip('\n')
        i, n = 0, len(line)
        continued = False
        while i < n:
            c = line[i]
            if escaping_quote is not None:
                if c == '\\' and i + 1 < n:
                    i += 2
                    continue
                if c == escaping_quote:
                    escaping_quote = None
                i += 1
                continue
            if in_squote:
                if c == "'":
                    in_squote = False
                i += 1
                continue
            if c == '\\' and i + 1 == n:
                continued = True
                break
            if c == '\\' and i + 1 < n:
                i += 2
                at_word_start = False
                continue
            if c in ' \t':
                at_word_start = True
                yield (lineno, c)
                i += 1
                continue
            if c == '#' and at_word_start:
                break
            if c == "'":
                if i > 0 and line[i - 1] == '$':
                    escaping_quote = "'"
                else:
                    in_squote = True
                at_word_start = False
                i += 1
                continue
            if c == '"':
                escaping_quote = '"'
                at_word_start = False
                i += 1
                continue
            yield (lineno, c)
            at_word_start = False
            i += 1
        if not continued:
            yield (lineno, '\n')
            at_word_start = True


def _find_function_end(lines, mask, start, name):
    """Real brace depth from `start`'s opening `{`, over the live-code
    stream, so a nested `helper() { ...; }` or a bare `{ ...; }` grouping
    block inside a fixture cannot be mistaken for the fixture's own close --
    the previous version stopped at the first line that was exactly `}`,
    which either one supplies early."""
    depth = 0
    for lineno, c in _live_chars(lines[start:], mask[start:]):
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return start + lineno
    sys.exit(
        f"error: test/probe_gates.sh:{start + 1} {name}() never closes its "
        f"opening brace."
    )


def function_bodies(lines, mask):
    """name -> list of (start, end), 0-based, end inclusive, for every
    top-level `name() {` ... `}` in the file, closed by real brace depth. A
    name defined more than once yields one entry per definition -- an
    earlier definition must not go invisible to either check just because a
    later one reused its name."""
    bodies = {}
    i, n = 0, len(lines)
    while i < n:
        if mask[i]:
            i += 1
            continue
        m = FUNC_START_RE.match(lines[i])
        if m:
            name = m.group(1)
            start = i
            end = _find_function_end(lines, mask, start, name)
            bodies.setdefault(name, []).append((start, end))
            i = end + 1
        else:
            i += 1
    return bodies


# Each tool's flag shape, checked against every token on the same statement
# after the tool's own word (i.e. up to the next real newline in the
# live-code stream). sed and perl both accept a suffix glued directly onto
# the flag (`-i.bak`), so `\S*` rather than `\b` closes those patterns; perl
# also combines `-i` with other single-letter flags in one token (`-pi`,
# `-npi`), restricted to the ones it actually combines with to avoid an
# unrelated flag that merely contains the letter i (`-Iinc`).
_SED_FLAG_RE = re.compile(r'^(-i\S*|--in-place\S*)$')
_PERL_FLAG_RE = re.compile(r'^-[pn]*i(\.\S+)?$')


def _raw_edit_hits(lines, mask, exclude_ranges):
    """(lineno, call-text) for every raw in-place edit -- `sed -i`,
    `sed --in-place`, `perl -i`/`-pi`/`-npi`, `awk -i inplace` -- that starts
    outside any quote, comment, or heredoc body, and outside
    mutate()/mutate_remove()/fixture_anchor()'s own implementations."""
    text = []
    linenos = []
    for lineno, c in _live_chars(lines, mask):
        text.append(c)
        linenos.append(lineno)
    text = ''.join(text)

    hits = []
    for m in re.finditer(r'\bsed\b|\bperl\b|\bawk\b', text):
        lineno = linenos[m.start()]
        if any(lo <= lineno <= hi for lo, hi in exclude_ranges):
            continue
        nl = text.find('\n', m.end())
        rest = text[m.end():nl if nl != -1 else len(text)]
        tokens = rest.split()
        tool = m.group(0)
        hit = False
        if tool == 'sed' and any(_SED_FLAG_RE.match(t) for t in tokens):
            hit = True
        elif tool == 'perl' and any(_PERL_FLAG_RE.match(t) for t in tokens):
            hit = True
        elif tool == 'awk':
            for k, t in enumerate(tokens[:-1]):
                if t == '-i' and tokens[k + 1] == 'inplace':
                    hit = True
                    break
        if hit:
            hits.append((lineno, lines[lineno].strip()))
    return hits


def check_raw_edits(lines, mask, exclude_ranges):
    rc = 0
    allowed = set(RAW_EDIT_ALLOWLIST)
    seen = set()
    for lineno, call in _raw_edit_hits(lines, mask, exclude_ranges):
        seen.add(call)
        if call not in allowed:
            rc = 1
            print(
                f"error: test/probe_gates.sh:{lineno + 1} calls a raw "
                f"in-place edit directly: {call}",
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
                f"error: RAW_EDIT_ALLOWLIST exempts '{call}', and it no longer "
                f"appears in test/probe_gates.sh. Delete the entry.",
                file=sys.stderr,
            )
    return rc


def check_fixture_anchors(lines, mask, bodies):
    rc = 0
    unanchored = {}
    for name, ranges in bodies.items():
        if 'fixture' not in name:
            continue
        for start, end in ranges:
            body = ''.join(lines[start:end + 1])
            has_heredoc = bool(HEREDOC_START_RE.search(body))
            has_cp_repo = 'cp "$REPO' in body or "cp '$REPO" in body
            has_anchor = 'fixture_anchor' in body
            if has_heredoc and not has_cp_repo and not has_anchor:
                unanchored.setdefault(name, []).append(start)

    for name in sorted(unanchored):
        if name in FIXTURE_ANCHOR_ALLOWLIST:
            continue
        rc = 1
        for start in unanchored[name]:
            print(
                f"error: test/probe_gates.sh:{start + 1} {name}() types out an "
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
        r for name in ('mutate', 'mutate_remove', 'fixture_anchor')
        for r in bodies.get(name, [])
    ]
    if not all(name in bodies for name in ('mutate', 'mutate_remove', 'fixture_anchor')):
        sys.exit(
            "error: mutate(), mutate_remove() and fixture_anchor() are not all "
            "defined in test/probe_gates.sh, so a raw in-place edit cannot be "
            "told apart from their own implementation."
        )

    rc = check_raw_edits(lines, mask, exclude_ranges)
    rc |= check_fixture_anchors(lines, mask, bodies)
    if rc:
        sys.exit(1)

    print(
        "no bare in-place edit outside mutate()/mutate_remove(), "
        f"{len(FIXTURE_ANCHOR_ALLOWLIST)} fixtures still owed an anchor"
    )

if __name__ == '__main__':
    main()
