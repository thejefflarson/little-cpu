# ADR-0244: The comparison document is generated, and only littlecpu's cycle half is ratcheted

**Status:** Accepted · 2026-10-02 · *Builds on ADR-0232's stamp and ADR-0233's refresh. No `rtl/` change
ships from this ADR.*

## Context

The cross-core numbers lived in CLAUDE.md's prose and ADR-0232's table, each copied from
`soc/compare/product.json` by hand. Nothing noticed when a copy and the stamp disagreed, and
nothing graded littlecpu's own cycle counts between weekly re-takes, so a CPI regression showed up
only when someone next ran `make compare-product`.

## Decision

- **`docs/comparison.md` is rendered, never edited.** `soc/compare/comparison.py` renders it from the
  committed stamp: one section per part, no blended row, no cross-part average, each row carrying
  its cycle half, its placed clock (worst, median, best, spread, n), the product where both halves
  came off one stamp, and the caveats that travel with the numbers. `make compare-doc` rewrites it.
  `make compare-doc-test`, on `make test`'s path, fails when the committed file is not a fresh render,
  in `make monitor-check`'s shape. It reads the stamp and nothing else, so no `make compare-*`
  measurement runs on a pull request.
- **Only littlecpu's cycle factors are ratcheted.** `soc/compare/CYCLE_FLOOR` holds the Dhrystone and
  CoreMark factors at RV32IM. A factor below its line is a regression and a factor above it owes the
  file an update, both red, as `test/OBSERVED_FLOOR` grades. This is sound because the cycle half
  is a deterministic simulation: the same tree and toolchain return the same count to the digit
  (ADR-0232 re-ran it locally and read the stamp's digits).
- **The clock half is published, not ratcheted.** A placed clock is a distribution, not a number:
  up5k's placement spread is 4–9% and edit churn about 3.6%, ECP5's is about 10% (`soc/bands.py`
  states both). Any floor tight enough to catch a regression would fail on an unchanged netlist,
  and any floor loose enough not to would catch nothing. The up5k clock already has its own gate,
  `SOC_MIN_MHZ` over the pinned placement (ADR-0171). The document says "not ratcheted" beside each
  table's clock.
- **Opponents are reported, never graded.** VexRiscv's and Hazard3's cycles and clocks come from
  pinned third-party builds whose movement is not this repository's regression.
- **The stamp's staleness check is unchanged.** `product_check.py` already excludes the artifact it
  reads from the paths it watches, and `test/probe_gates.sh` has forced-red probes for that; the
  document check is independent of it, because a stale-against-`rtl/` stamp is a reason to re-take,
  not a reason to refuse to render what was measured.

## Consequences

- A refresh of `soc/compare/product.json` (ADR-0233) now also owes `make compare-doc`, and an
  improvement owes a `CYCLE_FLOOR` update. The refresh pull request fails `make test` until it
  carries both. That is the intent: the document and the floor never trail the stamp.
- The floor moves only by hand from a committed stamp, never regenerated from a run, so a
  regression cannot launder itself into the baseline.
- A new benchmark, ISA or target core is a new `CYCLE_FLOOR` line; the ratchet refuses a stamped
  benchmark with no line and a line with no stamped pair.
