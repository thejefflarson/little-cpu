# ADR-0066: Twelve megahertz is a requirement

**Status:** Accepted · 2026-08-02 · *Amends
[ADR-0038](0038-area-is-measured-in-logic-cells-and-two-levers-are-rejected.md) decision 2, which
declared 12 MHz an intent. Cashes
[ADR-0062](0062-twelve-megahertz-is-reachable-and-the-bypass-select-is-the-cost.md)'s divider
argument and [ADR-0064](0064-the-write-through-bypass-is-addressed-from-the-held-pair.md)'s
measurement; ADR-0064 explicitly left this question open. No `rtl/` change ships from this ADR.*

## Context

ADR-0038 declared Fmax at 12 MHz before this project could place a design, and every ADR since has
measured against it as an aspiration: 11.30 MHz at ADR-0054, 10.18–10.32 at ADR-0059, 12.32–12.91 at
ADR-0062's spike, 12.69–13.45 at ADR-0064 where the change landed.

**It was written as an intent because the design missed it.** That is no longer true, and an intent
that is met is a requirement nobody has agreed to enforce.

## Why 12 is a number and not a target

`soc/littlesoc.pcf` targets the iCEBreaker up5k/sg48. Its board crystal is 12 MHz and the part's own
oscillator is 48 MHz nominal with a divider of 1, 2, 4 or 8 — 48, 24, 12, 6. Twelve is native from
either source and **the step below it is six**.

So a design at 11.9 MHz does not run 1% slower than one at 12.0. It runs at half the clock, because
6 MHz is the next thing the board can supply. This is ADR-0062's argument and it is the whole reason
the number is not negotiable downward by small amounts.

## Decision

1. **`SOC_MIN_MHZ` is 12.0**, up from 10.9 (ADR-0064) and 10.0 before that.
2. **The convention changes with the number.** 10.0 and 10.9 were *regression* floors: each sat
   about 11.5% below the then-current measurement so that placement noise and edit churn could not
   turn a working design red. A regression floor slides every time the design moves. **A requirement
   floor does not slide.** It sits at the number the hardware demands, and tripping it means the
   design no longer runs on the board.
3. **ADR-0038's declaration becomes a requirement**, amended in place there.
4. **`soc-timing` is a required status check** on `main`. The required set is seven: `elaborate`,
   `test`, `components`, `monitor-freshness`, `lint`, `formal`, `soc-timing`. That was done in the
   repository settings before this ADR landed, after CI measured 12.72 MHz on its own hardware.
   `fit` stays non-required: area is a design constraint, and 3880 of 5280 cells is not a cliff.

## What it measures

`soc/timing_sweep.sh`, four placements — the default plus `nextpnr-ice40 --seed 1/2/3` — on the
branch with `SOC_MIN_MHZ = 12.0`. **Homebrew Yosys 0.67+post (`b8e7da6f`)**,
nextpnr-0.10-108-g68c1acd8, `icetime` from the same install. Machine load 7.4–9.7 with two sibling
agents building; `icetime` is a static analysis and nextpnr is seeded, so load moves the wall time
(2m47s for the four) and not the numbers.

| seed | ns | MHz |
|---|---|---|
| default | 76.88 | 13.01 |
| 1 | 78.80 | 12.69 |
| 2 | 75.81 | 13.19 |
| 3 | 74.34 | 13.45 |

All four clear 12.0 and the target exits 0. This reproduces ADR-0064's sweep to the hundredth of a
nanosecond, which is what a seeded placer on an unchanged netlist should do — **no timing
measurement is re-derived here**, and none should be: the measurements are ADR-0062's and
ADR-0064's.

nextpnr's own estimator now reports `13.72 MHz (PASS at 12.00 MHz)`, which is worth knowing because
the Makefile's comment about its exit status said the opposite. Its status is still not the signal:
it grades its own default with its own estimator, and what this repo grades is `icetime`'s report of
the `.asc`.

Probe, re-run: `make soc-timing SOC_MIN_MHZ=99` exits 2 (the script exits 1) with
`13.01 MHz is under the 99.00 MHz floor`.

## The consequence, accepted deliberately

At 12.0 the margin against the worst local placement (12.69) is **5.75%**, against a measured
**3.6% edit-churn band** (ADR-0054) and a 1–2% placement band (ADR-0057). CI's pinned OSS CAD Suite
is a third axis that has never been measured for timing; it reported 12.72 MHz on the same tree,
which is 6.0%.

**That is tighter than any other ratchet in this repo, and it is correct anyway.** A regression floor
buys margin so that noise does not cry wolf, because a design that regressed by 2% is still a working
design. A requirement floor has no such licence. If an edit lands the design at 11.9 MHz, the honest
report is that it stopped meeting its requirement — whether the cause was the edit's logic or the
placer's mood, the part still has to be clocked at 6.

**When it trips, the fix is the design, not the floor.** That sentence is written at the site, in the
Makefile and in `soc/timing_split.py`'s failure message. Lowering `SOC_MIN_MHZ` later is a decision
to stop targeting the board clock and needs its own ADR.

## What this does not say

- **It is not a claim that the design is fast.** 12 MHz on a part whose oscillator offers 48 is a
  consequence of a combinational fetch window in one cycle, which is invariant 1 and is the design.
- **It does not raise the bar above 12.** Nothing here asks for 24. The next divider step up is a
  different project, and ADR-0038's test still applies: anyone proposing it is proposing to change
  invariant 1 or invariant 6 and must say which.
- **It is one placement flow on one toolchain.** `make soc-timing` is a static estimate at the
  worst-case corner; no silicon has been clocked. What the requirement gates is the estimate.

## Consequences

- `Makefile`, `soc/timing_split.py`, `.github/workflows/ci.yml` and `CLAUDE.md` stop describing the
  floor as a margin under the measurement. `test/probe_gates.sh` keeps the same probe, relabelled.
- **A red `soc-timing` now blocks merge.** Before this, a change that cost 15% of the clock reached
  `main` with a green summary and a number in a step report nobody had to read.
- **Every future timing ADR has a pass/fail, not a trend.** The last four each reported a percentage
  against a moving reference; the question now is whether the sweep clears 12.0.
- `make fit`'s `FIT_MAX_LC` is untouched and stays a regression ratchet with margin over its ±50-cell
  churn floor. The two ratchets now have different characters on purpose, and that difference is the
  thing to carry: area has no cliff at 5280 minus one cell, and the clock has one at 12 MHz.
