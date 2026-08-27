# ADR-0140: `PROBES_EXPECTED` becomes a named manifest, not a scalar

**Status:** Accepted · 2026-08-26

## Context

`test/probe_gates.sh` pinned its own coverage ratchet as a single literal, `PROBES_EXPECTED=586`,
bumped by whoever added or removed a `probe` call. Four incidents in one session, the fourth
arriving after this ticket was opened:

1. A coordinator read the literal off a pre-rebase branch and quoted it to two engineers as main's
   value; it was six lower than main's real value. Caught before it reached the tree by the standing
   rule "read it from the script's own error output, never compute it" — a human-process fix, not a
   mechanism one.
2. A rebase that git reported as clean produced a silently wrong value. Two commits' diffs to the
   literal — one changing it from 515 to 556, the next from 556 to 562 — were replayed onto a base
   where main had already moved the literal to 562 by a different route. The first hunk's removed
   line no longer matched anything current, so it landed as a no-op; the second hunk's context still
   matched well enough to apply, landing on 563. Six probes' worth of coverage went missing along
   with the value that was supposed to notice, and nothing conflicted.
3. Three PRs in a row (#232, #233, #234) each touched that same line and conflicted on it — an
   ordinary, visible conflict, unlike incident 2, but a tax on every PR that adds a probe.
4. A later pair of PRs (#235/#236) touched textually disjoint lines of the file — a clean, honest
   rebase — and the integrator still had to run the grader by hand to be sure 586 was the real
   count rather than trust the arithmetic.

The literal earns its place: it is what stops a deleted, skipped, or early-returned probe from
silently shrinking coverage while the summary still prints green (see CLAUDE.md, "A grader that
cannot fail is not a grader"). The question is whether a bare scalar is the right shape for it.

## Options considered

**A. A merge driver, or per-group counts summed at runtime.** Removes the textual conflict, but a
union merge of two groups' counts can silently accept both while a third group's count went missing
from the sum — the same class of defect the literal exists to prevent, moved one level up.

**B. Generate the count; check in the expectation separately.** Worth doing only if the generated
side and the checked-in side are independent enough that one going stale is visible before it is
run. A count *derived from the same file it grades* is not independent — it would trivially agree
with whatever the file currently contains, including a version with a probe silently missing,
because both sides move together.

**C. Keep the scalar; write the discipline down.** Already fails closed at `make test`, and the
existing "read it from the script's own error output" rule is exactly this. But incident 2 happened
to an engineer who *was* following that rule — the corruption was in the git-level patch replay,
before anyone looked at anything. Discipline does not fix a mechanism that can produce a
self-consistent wrong answer during a rebase nobody flagged as conflicted.

**Chosen: a named manifest, closer to B but keyed on content rather than on a derived total.**
`test/PROBES_EXPECTED` now holds one line per probe, its label verbatim, sorted, checked against the
labels actually run under set equality in both directions — the exact rule `test/EXPECTED_FAIL` and
`test/OBSERVED_FLOOR` already use for the same reason. This is not option B's generated-count
version, which was rejected above: nothing here is derived from `test/probe_gates.sh` itself, the
manifest is a hand-maintained baseline the same way `test/EXPECTED_FAIL` is, and a probe silently
disappearing from the script produces a real mismatch against an independent file, not a comparison
against its own edited self. It also avoids option A's stated failure — a union merge silently
accepting two groups while dropping a third — because the comparison is exact-set-equality over
every individual label, not an arithmetic sum: dropping one entry from either side is a mismatch
regardless of how many other entries moved around it.

Why this beats keeping the scalar (declining C, with reasons, per the ticket's instruction): a
scalar's checked-in diff is an arithmetic *transform* — "was 556, now 562" — replayed by textual
patching that has no way to detect it is being applied against an unexpected base value except by
conflicting, and incident 2 showed it does not always conflict. A named manifest's diff is a set of
independent line *insertions* ("add these labels"), which is the one shape git's line-based merge
is sound for: replaying "insert line L" after other unrelated insertions always inserts line L,
regardless of what else changed, because there is no arithmetic step to get quietly wrong. Sorting
the manifest alphabetically also spreads unrelated PRs' insertions across the file by their label
text rather than concentrating every PR on the same single line, which is what produced incident 3;
two PRs adding differently-worded probes now conflict only if their labels happen to sort adjacent,
and when they do, the result is a real, visible conflict rather than a silent one.

Incident 4's complaint — that even a textually clean change still needed the grader run to be
believed — is not fully answered by any option, including this one, because running the grader is
what CI is for and no static mechanism substitutes for it. What this ADR removes is the specific
failure incident 2 demonstrated: a clean-looking merge that is *actually* wrong. A named manifest
cannot reproduce that failure, because there is no scalar arithmetic left for a rebase to get
quietly wrong.

## Decision

1. `test/PROBES_EXPECTED` is a new checked-in file: a header (in the same voice as
   `test/EXPECTED_FAIL`'s) followed by one label per line, sorted, one line per `probe` call site —
   duplicates included, since several probes intentionally share an English description across
   different fixtures and the comparison is a multiset, not a set.
2. `test/probe_gates.sh`'s `probe()` records every label it runs into an array. The final check
   sorts that array and compares it against the manifest (comments and blank lines stripped) with a
   plain string comparison; a mismatch prints a `diff` naming exactly which labels are missing or
   extra, then exits 1. `PROBES_EXPECTED` the scalar is gone; `PROBES_MANIFEST` (a path) replaces
   it, checked for existence and readability the way `test/run_tests.sh` already checks
   `OBSERVED_FLOOR`.
3. The manifest is edited by hand, in the same commit as the probe it names, exactly like
   `test/EXPECTED_FAIL` — never regenerated wholesale from a run, which would launder a dropped
   probe into the baseline instead of catching it.

## Forced red direction

The check this ADR replaces is `test/probe_gates.sh`'s own coverage ratchet, so it cannot be
exercised by a `probe` call inside that same file without the file invoking itself — the same
reason four other failure paths in this file are already demonstrated by hand rather than probed.
Demonstrated by hand on this tree, both directions, each on a working-tree edit reverted immediately
after the run (never committed):

- Deleted one `probe` call (`"a missing asm directory is named"`) from `test/probe_gates.sh`
  without touching the manifest. `make probe-gates` exited 2 (the script's own exit 1, via `make`):
  `error: the probes that ran do not match .../test/PROBES_EXPECTED.`, diff `138d137 < a missing
  asm directory is named` and nothing else.
- Appended one line, `bogus probe that does not exist`, to `test/PROBES_EXPECTED` with no matching
  `probe` call. `make probe-gates` exited 2: same error text, diff `371d370 < bogus probe that does
  not exist` and nothing else.
- Reverted both edits and re-ran: exit 0, `586 graded comparisons, every failure path executed.`

## Consequences

- `make probe-gates` still reports 586 and still fails closed on a shrinking or renamed probe; the
  diagnostic is now a named diff instead of a mismatched pair of totals.
- Adding a probe is now two edits in the same commit — the `probe` call and one sorted line in
  `test/PROBES_EXPECTED` — instead of one edit to a call plus a one-line arithmetic bump. A PR
  description that only shows the second half of that (the manifest line) makes the addition
  self-evident in the diff, which a `+41 -1 / +1 -1` scalar bump never was.
- Deleting or renaming a probe requires removing or updating its manifest line by hand, the same
  discipline `test/EXPECTED_FAIL` already imposes — not enforced by tooling beyond the exact-match
  check itself, which is the same level of enforcement the scalar had.
- No RTL changed; `make netlist-digest` is unaffected.
