# ADR-0194: The paired sweep is scripted, and both parts' bands are re-derived

**Status:** Accepted · 2026-09-18

## Context

`soc/bands.py` is the one place the placement-spread and edit-churn figures live, and both of
up5k's carried `"derived": "PLACEHOLDER"` -- a measurement with no tree, no toolchain and no
sweep behind it. ECP5's entry had no figures at all: `band("ecp5")` raised `Underived`, so the
coming pipeline-restructure programme, whose headline number is an ECP5 clock, could report an
ECP5 delta but never grade one.

Deriving a band by rule owes sixteen paired up5k seeds and twelve paired ECP5 seeds per side of
every comparison, driven by hand through `soc/timing_sweep.sh` -- whose own default of four seeds
is "a look, not a verdict". This ticket builds the one-command runner the restructure work will
need repeatedly, and spends its own first real run deriving both bands with that runner's
instruments rather than by prose.

## What was built

**`soc/paired_sweep.sh`**: one command that takes a base ref and sweeps it against the working
tree, both parts, paired by seed, running each part's base and candidate sweep as a pair rather
than sequentially since they write into two different directories and nothing collides. It reuses
`soc/baseline_sweep.sh` for each half -- a base ref sweeps inside a tree extracted with
`git archive` (the same pattern `soc/netlist_base.sh` already uses for `make netlist-diff`), the
working tree sweeps in place -- and calls `soc/baseline_summary.py` on the resulting pair with
neither `--allow-mismatch` (so a toolchain disagreement between the two sweeps refuses the
verdict, `soc/baseline_summary.py`'s own existing behaviour) nor any way to lower the seed floor:
it refuses before touching a placement tool if either part's own seed list, including an override,
names fewer than twelve.

**The base leg's extracted tree lands outside this repo, in the tool cache, not a subdirectory of
it.** First built nested under the sweep's own output directory, inside this repo's working tree:
`git archive` carries no `.git`, and a plain `git rev-parse`/`git diff` run inside a tree nested in
a real git working directory does not fail -- it walks up and silently answers for the ENCLOSING
repo instead, which a first real end-to-end run of this script demonstrated by reading `dirty: yes`
and the working tree's own state for a base leg whose own copy of `soc/baseline_sweep.sh` (from a
ref that predates this ticket) had no override to read. `soc/baseline_sweep.sh` gains
`BASELINE_BASE_OVERRIDE`/`BASELINE_DIRTY_OVERRIDE`, so a base leg swept at or after this commit
never calls git inside the extracted tree at all; moving the extraction itself to
`~/.cache/little-cpu/paired-sweep/<sha>`, alongside the OSS CAD Suite and the pinned RISC-V gcc,
means a base ref whose own script predates the override fails loudly (`fatal: not a git
repository`) instead of silently sweeping the wrong tree -- confirmed both ways on a real run.

**Resume rather than restart, in `soc/baseline_sweep.sh` itself.** A sixteen-seed sweep is
tens of minutes; a CSV already stamped with the run's own base commit, dirty flag and part is
read rather than truncated, and a seed already carrying both its artifact and its row is
skipped. Building this ticket's own data found the second, unrelated defect it exists to
prevent from recurring: `make soc-timing`'s recipe writes `soc.timing.rpt` and only *then* applies
`SOC_MIN_MHZ`, so a placement under the floor is a real measurement with a nonzero exit code, not
a build failure -- and the sweep was stopping there, silently truncating exactly the tail a
placement-spread sweep exists to see (this run's own sixteen-seed up5k sweep hit it at seed 9,
11.99 MHz). Only artifacts genuinely missing now stop the sweep.

**`soc/baseline_summary.py` gains `--min-seeds N`**, a refusal distinct from `--allow-mismatch`'s:
a verdict taken under N placements is not a mismatch between two sweeps, it is one sweep too short
to read a worst, a median and a spread from, and no flag covers it.

**`soc/bands.py`'s `spread` field now accepts a single-sweep figure, not only a range.** ECP5 has
had exactly one sixteen-seed sweep; forcing that into `(low, high)` would print `10.3-10.3%`, which
reads as a typo rather than the honest fact that no second sweep has yet narrowed or widened it.
`spread_text()` renders a tuple as a range and a plain number as itself, with a note that it is not
yet a range; `test/band_source_test.py`'s `spellings()` follows the same branch so CLAUDE.md's
copy is graded either way.

## What was measured

All four sweeps below are sixteen seeds (`default 1`..`15`), Yosys 0.68+48 (`ff5817c34`),
2026-09-18, on tree `84fe92b7084d` (clean -- this ticket's own tooling commit, which touches no
RTL or synth script). `soc/paired_sweep.sh`'s own netlist-digest step (`make netlist-diff`, up5k
only -- the Makefile's `NETLIST_PART` table has no ECP5 entry) confirmed each edit below
`DIGEST-DIFFERENT` for up5k before any seed was spent on it.

**Placement spread**, one sweep per part, on the unchanged netlist:

| part | worst | median | best | spread |
|---|---|---|---|---|
| up5k (nextpnr-ice40 0.11-1-`g62e659ed`, icetime 20260811) | 83.38 ns / 11.99 MHz | 80.44 ns / 12.43 MHz | 78.10 ns / 12.80 MHz | **6.8%** |
| ecp5 (nextpnr-ecp5 0.11-1-`g62e659ed`) | 30.29 ns / 33.01 MHz | 29.37 ns / 34.05 MHz | 27.46 ns / 36.42 MHz | **10.3%** |

up5k's 6.8% sits inside the existing 4-9% and re-confirms it rather than replacing it. ECP5's
10.3% is the first figure this part has ever had, from one sweep -- wider than up5k's, and not
yet a range a second sweep could narrow.

**Edit-churn**, two real edit classes at the same seeds, both to `rtl/csrs.v` -- the large
representative file ADR-0170 already showed a comment-only diff moving (a small file, per that
ADR's own header, has never been measured to). The comment class replays ADR-0170's own recorded
diff (42 changed comment lines, `git show 45f0b9b^:rtl/csrs.v`, re-applied on top of this
ticket's tree); the blank-line class inserts the same net line count (20) as pure blank lines at
the same position, so the two classes differ only in what moved, not by how much:

| part | edit class | worst | median | best |
|---|---|---|---|---|
| up5k | comment (ADR-0170's diff) | **-3.6%** | -3.0% | -2.2% |
| up5k | blank-line (matched count) | -1.7% | -3.5% | -3.3% |
| ecp5 | comment (same diff) | 0.0% | 0.0% | 0.0% |
| ecp5 | blank-line (same diff) | 0.0% | 0.0% | 0.0% |

up5k's worst-case comment-class delta, -3.6%, matches the existing figure almost exactly and is
the ceiling both classes stay inside. **ECP5 read exactly 0.0% on both classes: the placed
bitstream came out byte-identical, seed for seed, to the unedited tree**, even though the same
two diffs are independently confirmed to move up5k's synthesised cell count. That is a measured
null under these two fixtures, not a proof this part's synthesis can never be moved by a comment
-- ABC9's cell-name sort order is the up5k mechanism ADR-0170 traced, and this run gives no
evidence about whether or how ECP5's own flow is sensitive to it. `soc/bands.py`'s `derived`
field states this caveat inline rather than letting a bare `0%` be read as a guarantee.

**Two small single-line edits, tried first, measured no movement at all** -- one comment line and
one blank line inserted after `rtl/csrs.v`'s header comment placed byte-identical to the unedited
tree on both parts, at all sixteen seeds. This is consistent with ADR-0170's own point: the
mechanism is a lexical-rank boundary a line-number shift may or may not cross, and a single line
is not guaranteed to cross it even in a large file. It is why this ADR's churn figures replay
ADR-0170's own already-demonstrated 42-line diff rather than a fresh guess at a smaller one.

## Decision

`soc/bands.py`'s two entries are re-derived with real provenance:

- **up5k**: `spread = (4.0, 9.0)` (confirmed, not replaced -- 6.8% sits inside it), `churn = 3.6`
  (confirmed by the comment class, the blank-line class inside it too).
- **ecp5**: `spread = 10.3` (a single sweep, first ever taken), `churn = 0.0` (a measured null
  under two real, up5k-confirmed-moving fixtures, flagged as such rather than smoothed into a
  guarantee).

Both entries' `derived` field carries the tree, both tool versions, and the sweep that produced
the figures, matching `soc/bands.py`'s own header claim that a band is a measurement with a date
on it. `soc/bands.py ecp5 --require` now exits zero.

`soc/paired_sweep.sh` ships as the one-command runner future restructure stages sweep with; its
two refusals (fewer than twelve seeds, a toolchain disagreement between halves) are demonstrated
red in `test/probe_gates.sh` without spending a real placement, and `soc/baseline_sweep.sh`'s
resume behaviour and ratchet-vs-failure distinction ship as corrections to the instrument itself,
not scoped to this ticket's own run.

## What was considered and rejected

**Reporting ECP5's spread as a range, `(10.3, 10.3)`.** Rejected: `soc/bands.py`'s existing
`sentence()` would print `10.3-10.3%`, which reads as a typo. A plain float with its own rendering
path is the honest shape of one sweep's own figure.

**Constructing the blank-line fixture as a fresh, arbitrary insertion rather than matching the
comment class's line count.** Rejected: matching the count isolates the one variable the ticket
asks about (comment text vs. blank text) from a second one (how many lines moved), which
ADR-0170's own mechanism -- line-NUMBER shift, not line content -- says would otherwise confound
the comparison.

**Forcing a nonzero ECP5 churn figure by trying larger or differently-placed edits until one
moved it.** Rejected: the two edits tried are the ones already proven, independently, to move
up5k's netlist, which is the strongest available claim that the null is about ECP5's flow and not
about the fixtures being too small. Chasing a bigger edit until something moved would answer a
different question (does ECP5 churn at all, under some edit) with a number that looks like this
one (how much does ECP5 churn under the up5k-calibrated fixtures) but is not.

**Adding ECP5 support to `make netlist-digest`.** Out of scope: `NETLIST_PART` and its whole table
are up5k-specific by construction (`NETLIST_MUTANT`, `NETLIST_COMMENT_FILE`), and building a
second synthesis-only digest path for ECP5 is a feature this ticket did not need -- the placement
sweep itself is ECP5's own, real digest for the question that matters here.

## Consequences

- A restructure stage now sweeps both parts against a named base in one command, with the
  toolchain and (where the part has one) netlist digest recorded, and cannot silently ship a
  verdict on too few seeds or across a toolchain change.
- ECP5 has a placement-spread ceiling for the first time, at 10.3%, which is wider than the gap
  between several past ECP5 clock candidates and their targets in this repo's own history --
  exactly the "say so loudly" case: a future ECP5 gate needs more than a handful of seeds to
  clear this spread with confidence, and this ADR is where that number now lives.
- ECP5's edit-churn figure is a floor, not a ceiling: the next edit that visibly moves an ECP5
  bitstream should update `soc/bands.py`'s `derived` field and this ADR's own table, not be
  treated as a surprise the tooling failed to predict.
