# ADR-0150: A union merge driver replaces hand-rebasing `docs/adr/README.md`

**Status:** Accepted · 2026-09-01

## Context

`docs/adr/README.md` is a sequentially-numbered table, and every PR that adds an ADR appends one
row to it at the same anchor: the end of the table. Two such appends touch no line the other
wrote, but git's default line-based merge sees two edits converging on one location and refuses to
guess an order between them, so the second of any two ADR-adding PRs to land conflicts on this
file — regardless of what either PR actually changed.

This is not hypothetical. `gh pr list` on this tree right now shows five PRs (#240-#244) that each
needed a number; two independently claimed 0141, each correctly from its own view of `main` at the
time it branched, and the duplicate was caught by hand before either merged. Then #244 merged, and
`gh pr list` now shows #240, #241, #242 and #243 all `CONFLICTING` — four PRs blocked on one file,
none of whose *content* actually collides with another's.

ADR-0140 fixed the same shape of problem for `PROBES_EXPECTED` by turning a contested scalar into a
sorted multiset of independent lines, so unrelated insertions land at different points in the file
and rarely collide. **Sorting cannot do that here.** `PROBES_EXPECTED`'s lines sort by label text,
which scatters unrelated PRs' insertions across the file. An ADR row's sort key is the ADR number
itself, and a number is assigned in order — every new row's natural position is still the last row,
by construction, so sorting the same list that already collides would still collide.

## Options considered

**1. A `.gitattributes` merge driver for `docs/adr/README.md`.** Cheapest — no new tooling, no
change to what any of the graders check. Tested directly on this tree: two branches taken from one
base, each appending a distinct row and adding its own new ADR file, merge with `Auto-merging
docs/adr/README.md` and no conflict marker under `merge=union`, where the same two branches conflict
under git's default strategy. The merged file carries both rows and both files, and
`test/adr_numbering_test.sh` reports it clean — `139 ADR files, each with exactly one README row, no
number claimed twice.`

*Does this let something invalid pass?* Checked, because a union merge's whole mechanism is
"include lines from both sides, drop exact duplicates" and that is a different question from "is
the result correct." Two things it does **not** protect against, tested the same way:

- **Row order.** A union merge does not sort — a row for a lower ADR number can land after a row for
  a higher one if the branch that added it merged second. `test/adr_numbering_test.sh` does not
  check order (it never has; it is a set-equality check between filenames and row-linked filenames),
  so an out-of-order table is cosmetic, not a defect this change introduces or the grader is meant to
  catch.
- **Duplicate ADR numbers.** A union merge cannot know that two branches picked the same integer
  independently — from git's point of view they are two ordinary, non-overlapping edits, one adding
  a row for `0141-fake-a.md` and the other a row for `0141-fake-c.md`, and both survive the merge.
  Tested directly: after merging two branches that each add a distinct file under the reserved
  number 0141, `test/adr_numbering_test.sh` still reports `error: ADR number 0141 is claimed by more
  than one file` and exits 1 — because that check reads the number out of the **filenames on disk**,
  never out of the README, and a union merge changes nothing about which files exist. The collision
  still needs a person to rename one of the two files and its row; what this change removes is the
  four-PR pile-up on a file neither PR's content actually touched, not the rarer case of two people
  picking the same integer at once.

**2. Generate the index from the directory listing at test time.** Removes row-vs-file drift by
construction — nothing to contend on, because nothing is checked in. Declined for two reasons this
ticket's evidence does not support paying for. First, it does not fix the problem #240-#243 actually
hit: their conflict is not on `docs/adr/README.md`'s *content*, which a generated index would
remove, but their `CONFLICTING` state on GitHub is produced by git's merge algorithm regardless of
whether the file is later regenerated — removing the file from version control changes what a
*fresh* rebase looks like, but the tested union driver already produces a clean fast-forward-free
merge with no rebase step at all, which is the cheaper fix for the same symptom. Second, a generated
index needs its own decision about what grades it: `docs/adr/README.md` today carries hand-written
prose for what each row's "Decision" column says, not just a title lifted from the filename or the
file's own first line, and no generator this repo has can currently produce that prose — building
one is a larger change than the collision this ticket exists to close, for a benefit (no tracked
index at all) the union driver already delivers for the common case.

**3. Document central allocation.** Zero code, and honest if the other two do not hold up — but they
do: the tested union driver removes the four-PR pile-up directly, and the numbering test that
already exists is the mechanism that catches the one case a merge driver cannot (two people picking
the same integer). No allocation procedure is added on top of it.

**Chosen: option 1**, `docs/adr/README.md merge=union` in a repo-root `.gitattributes`. It is the
cheapest of the three, it directly removes the failure `gh pr list` shows on this tree right now, it
was tested to still let `test/adr_numbering_test.sh` catch every defect that test already catches —
a row with no file, a file with no row, a file with two rows, two files claiming one number — and it
adds no new surface for those checks to grade, because it changes how two texts combine, not what
either text may contain.

## Decision

- `.gitattributes` at the repo root sets `merge=union` on `docs/adr/README.md` only — not on ADR
  body files, and not repo-wide, because a union merge is sound only for a file whose edits are
  independent line insertions at one shared point. The table row format already has that shape; ADR
  body prose does not, and a union merge silently keeping both sides of a real edit conflict there
  would be worse than the conflict marker it replaced.
- `test/adr_numbering_test.sh` is unchanged. It already grades the one failure mode a union merge
  cannot fix (two files claiming one number) by reading the filenames on disk, independent of
  anything in the README, and it already demonstrates both directions of the row-vs-file check in
  `test/probe_gates.sh`'s `test/adr_numbering_test.sh` group — a row with no file, a file with no
  row, a file with two rows, two files with one number, and the false-positive control that a gap in
  the sequence stays green. Re-run on this tree as part of this change; all pass.
- Allocation stays first-come, no lock and no reservation list. A collision is now rare (both PRs
  have to branch from the same base and pick the same next integer before either merges) rather than
  routine (every PR touching the file at all), and when it happens the test still names it and the
  fix is renaming one file and one row — not rebasing four PRs' worth of unrelated table edits.

## Consequences

- Two branches each adding an ADR now merge with no conflict on `docs/adr/README.md`, demonstrated
  directly on this tree (see above), as long as they claim different numbers — which is the normal
  case, since a number is picked by reading the last row at branch time.
- The rare case of two branches claiming the same number is unchanged in outcome (a person renames
  one file) and unchanged in detection (`test/adr_numbering_test.sh`, already required by `make
  test`) — this change does not touch that path.
- `docs/adr/README.md`'s row order after a multi-way merge is no longer guaranteed to track ADR
  number order. Nothing reads the table's order today; if something later does, that is a new
  requirement on a file this ADR deliberately leaves unsorted, not a regression in one that was ever
  sorted by merge.
- No RTL changed; `make netlist-digest` is unaffected.
