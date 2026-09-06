# ADR-0164: The fixture ratchet reads a naming convention, and grandfathers twelve

**Status:** Accepted · 2026-09-06

## Context

`test/probe_gates.sh` builds its own fixtures two ways. Most copy a real file
out of the tree and edit it; a dozen TYPE OUT an artifact's shape by hand,
because the artifact is a tool's stdout and nothing in the tree produces it —
an icetime timing report, a nextpnr utilisation block, a yosys cell census.

Both shapes drift, and each is silent in its own way. A `sed -i` whose pattern
stopped matching exits 0 having changed nothing, hands `probe` an unmutated
copy, and the probe goes red for the wrong reason: "exited 0, expected 1"
accuses the grader under test rather than the fixture. A hand-typed fixture
that no longer matches the format it imitates never goes red at all — it grades
a shape nothing produces, forever, and reports success.

`test/fixture_freshness_test.py` closes both: every mutation goes through
`mutate`/`mutate_remove`, which compare the file against the copy `sed` itself
made and fail by name when nothing moved, and a hand-typed fixture declares the
real file (or, for a generated artifact, the PARSER that reads it) and the
exact literal it copied from there, through `fixture_anchor`.

The question this records: **how far does the second check reach?**

## Decision

**Both checks ship. The anchor check inspects functions whose name contains
`fixture`, and twelve pre-existing anchorless fixtures are grandfathered by
name, each with the shape it invented written beside it.** The raw-`sed -i`
half is NOT ratcheted: `SED_I_ALLOWLIST` is empty and all 218 call sites were
converted in one change.

Both tables are graded in both directions, like every other baseline here. An
unlisted anchorless fixture is red because it is unreviewed; a listed one that
has since gained an anchor is red too, so an exemption cannot outlive its
reason.

### Why a naming convention and not a shape

The alternative is to inspect every function for a literal heredoc and demand
an anchor. That reads far more than fixtures: helper functions, stub writers,
the `probe` mechanism's own scratch files. Each would need an exemption, which
is a longer allowlist grading nothing, and a long allowlist is how the next
real entry gets waved through.

The cost is stated plainly: **a fixture named without the word `fixture` is not
inspected.** That is a convention this file already keeps at every one of its
sites, and it is cheap to re-widen later — the predicate is one `in` test.

### Why grandfathering twelve rather than sweeping them now

Anchoring a fixture is not mechanical. It means finding a literal in the real
file that the invented shape actually depends on, which is a judgment per
fixture; twelve of those inside a change that already converts 218 call sites
would bury the conversion. Each entry names the shape it invented, so the next
fixture to anchor is chosen by reading the table rather than by re-deriving it.

## Consequences and what it caught immediately

**The reach question is not theoretical: the check shipped with a blind spot on
the highest-consequence fixture in the file.** Two regexes disagreed about what
opens a heredoc. The masker accepted an unquoted delimiter; the anchor check
matched only a quoted one. `br_fixture` writes `<<JSON` unquoted — it has to,
its body interpolates a `$1` — so the anchor check skipped it silently, and it
is the fixture behind `soc/bram_reset_check.py`, the only detector of a block
RAM read through its own reset (ADR-0163). The two regexes are now one, the
fixture is anchored against the parser's cell table and its connection key
path, and an unquoted delimiter has its own red direction in
`test/probe_gates.sh`.

A second defect fell out of the same reading. The `sed -i` scanner counted
lines itself and skipped a backslash-escaped newline, so its line numbers
drifted 58 lines by the end of the file. It reported a real raw `sed -i` at
whatever line the drift pointed at — and when that line happened to be inside a
masked heredoc, dropped it with nothing said. Its own probe passed throughout,
because the probe read only the verb of the diagnostic and not the line quoted
after it. The scanner counts escaped newlines now and the probe reads through
the quoted line.

**What generalises: a checker that reads source text needs its own red
direction per SHAPE of text it claims to read, not one per rule.** Both defects
here were a rule that worked on the shape its probe planted and not on the
shape the tree actually contains.
