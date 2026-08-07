# ADR-0076: The decode head is a plateau, not a lever

**Status:** Accepted · 2026-08-06 · *A measured null. Corrects
[ADR-0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md)'s closing recommendation and the
sentence it put in `CLAUDE.md`, which
[ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md) then relied on
when it deferred the operand-fetch cycle to "a change that buys back headroom in the decode head".
Graded against [ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement.*

## What was proposed, and what it rested on

Two of this design's bottlenecks were measured within a week of each other and both were attributed
to the same place.

ADR-0070 charged the `.S` suite's 28632 cycles to a named reason: issue 49.0%, decode scoreboard
29.3%, operand fetch 16.4%, the rest 5.4%. ADR-0074 then built the operand-fetch removal, got
CPI 2.07 → 1.79 out of it, and declined it because it measured 11.56 MHz at one placement. Its own
finding was that the two big columns overlap: removing 93% of the operand-fetch cycles moved the
scoreboard column *up*, 8381 → 8837, and a core with no operand-fetch cycle at all would still
spend 43% of this suite's cycles not issuing.

Both ADRs then pointed at the same lever. ADR-0058 named it — "the only change that moves this
design's Fmax is one that shortens `imem.in_range → instr → {rs1/rs2, immediate, hazard}`" —
and `CLAUDE.md` carried it as a rule. Nobody had measured what shortening it is worth.

## Decision

**No RTL change, and the recommendation itself is withdrawn.** The decode head as ADR-0058 named it
is worth **3.3% at its ceiling**, which is inside the 3.6% edit-churn band. There is no implementable
change in that space whose effect could be demonstrated, let alone one that clears the bar for
amending a design commitment.

The reason generalises past the decode head, and it is what the title means: **every input to
`next_pc` measures nothing on its own and all of them together are worth 21%.** This is a plateau,
not a peak with a lever on it. A candidate that shortens one term of the fetch loop can be declined
on the tables below without being built.

Nothing in `rtl/` or `formal/` ships from this ADR. What ships is the tables below and the dead-end
list they add to.

## Method

`make soc-timing` with the default `SOC_PROG=add.S`, `soc/timing_sweep.sh` across four or eight
placements per variant, which is ADR-0058's method with ADR-0074's wider sweep on the rows that
matter. **Homebrew Yosys 0.68+post (`c12172fb`)**, nextpnr-0.10-108-g68c1acd8, `icetime` from the
same install; everything measured on one tree against `2d79878`. Area is `make fit`, whose churn
floor is ±50 cells and whose absolute number is toolchain-dependent — quote CI's.

The baseline reproduces ADR-0074's area number exactly (3880 cells) on a different Yosys, and its
eight placements span 77.38 – 80.07 ns, a 3.5% spread. Every row below is a comparison against those
eight, not a measurement from a different world. **The tables report periods, so a positive `vs
baseline` is slower.**

Ceilings are function-breaking RTL: rather than implement a candidate, the logic it hopes to shorten
is deleted outright. No implementation can beat its own ceiling, so a ceiling inside the noise
retires every spelling of that candidate at once. That is what makes this ADR's null result a
finding rather than a failure to find one.

## What the decode head is worth

| variant | seeds | ns, sorted | median | vs baseline | `fit` LC |
|---|---|---|---|---|---|
| **baseline `2d79878`** | 8 | 77.38 · 78.34 · 78.51 · 78.55 · 78.61 · 78.80 · 79.20 · 80.07 | 78.58 | — | 3880 |
| A — the width mask deleted | 4 | 78.91 · 79.29 · 80.26 · 80.60 | 79.78 | **+1.5%** | 3955 |
| B — RVC `rs1`/`rs2` selection deleted | 4 | 75.99 · 76.00 · 79.54 · 81.91 | 77.77 | −1.0% | 4003 |
| C — scoreboard cut from three slots to one | 4 | 76.67 · 77.00 · 77.16 · 77.75 | 77.08 | −1.9% | 3914 |
| D — the 18-way `immediate` mux cut to one arm | 4 | 77.06 · 77.38 · 77.82 · 79.01 | 77.60 | −1.2% | 3859 |
| **A+B+C+D — the whole head at its ceiling** | 8 | 74.19 · 74.79 · 75.38 · 75.87 · 76.08 · 76.12 · 76.76 · 79.85 | 75.98 | **−3.3%** | 3805 |

Read the last row first. It is `{rs1/rs2, immediate, hazard}` and the width mask that feeds them, all
four gone at once, on a design that then cannot execute anything. It buys 3.3% of the period and
75 cells — one LUT level off a path that is 23 of them. The critical path does not move: start and
end stay `imem.in_range → imem.rom_even`, and the split stays 36% logic and 64% routing.

Two of the four rows need a word beyond their number:

- **Row A goes the wrong way: the width mask earns its keep.** Zeroing `instr[31:16]` for a
  compressed instruction is what lets every field cut from that half fold to a constant, and a raw
  upper half hands each consumer of `funct7`, `csr_addr` and the I/S/B/U/J immediates a live input
  where it had a zero. Deleting it is 1.5% slower and 75 cells bigger, and no version that keeps the
  core working can do better than that.
- **Row C is the largest of the four and is not implementable.** Comparing one in-flight slot instead
  of three is 1.9% — half the edit-churn band — and the scoreboard cannot be narrowed anyway: every
  in-flight non-`x0` `rd` has to be visible on every cycle between issue and the regfile
  write-through, with no gap.

## Two implementable spellings, both declined

These are real changes, not ceilings: both preserve semantics exactly, and each was picked because it
reads at least as well as what is checked in.

| candidate | seeds | ns, sorted | median | vs baseline | `fit` LC |
|---|---|---|---|---|---|
| **R** — the width mask hoisted out of the register path | 8 | 78.45 · 79.38 · 79.50 · 79.77 · 80.53 · 80.83 · 80.96 · 81.59 | 80.15 | **+2.0%** | 3982 |
| **M** — the two duplicate `rd_field` arms merged into one | 8 | 78.68 · 78.85 · 79.54 · 79.65 · 79.70 · 80.18 · 81.73 · 82.59 | 79.68 | **+1.4%** | 3905 |

**R is the honest implementation of "decode `rs1`/`rs2` before quadrant resolution".** Every
compressed arm of the `rs1`/`rs2` case reads only bits the width mask never touches, and the
uncompressed arm is the raw word's `instr[19:15]` / `instr[24:20]`, so `rs1 = uncompressed ?
in.instr[19:15] : <compressed select>` is the same function with the mask off the register path. It
measures 2.0% slower and 102 cells bigger. The mask was not a level in front of the mux; it was
folded *into* the mux's default arm, and splitting them adds a 5-bit mux at the end of the chain and
a second fanout point on `uncompressed`.

**M merges two arms of the `rs1` case that select the same value** — `instr_cjr || instr_cjalr ||
instr_cslli` and `instr_caddi || instr_caddi16sp || instr_cadd` both give `rd_field`. It is a strictly
smaller mux and one fewer line, and it is 1.4% slower at the median with a worst placement of
82.59 ns — **12.11 MHz, 0.9% of margin over the board clock**. A change that reads better, is
provably the same function, and eats four fifths of the margin is not one this repo takes; the rule
is that a change improves one goal and the other three still hold, measured.

## What every other term of `next_pc` is worth

The ceilings were extended past the decode head to bound what any change to the fetch loop could
be worth. This is the part that outlives the nanoseconds.

| variant | seeds | ns, sorted | median | vs baseline | `fit` LC |
|---|---|---|---|---|---|
| T — `trap_pending` off `next_pc` | 4 | 73.99 · 74.00 · 75.20 · 75.41 | 74.60 | **−5.1%** | 3858 |
| T2 — T without `instr_illegal` only | 4 | 77.06 · 79.36 · 79.50 · 82.03 | 79.43 | +1.1% | 3864 |
| T3 — T without the misalignment terms only | 4 | 78.91 · 79.01 · 80.45 · 81.02 | 79.73 | +1.5% | 3879 |
| BR — the branch comparators off `next_pc` | 4 | 74.77 · 78.03 · 79.09 · 80.72 | 78.56 | 0.0% | 3880 |
| E — `stall` off `next_pc` | 4 | 74.71 · 77.09 · 77.74 · 79.09 | 77.42 | −1.5% | 3928 |
| **all of them, plus A+B+C+D** | 4 | 60.83 · 61.49 · 62.22 · 62.81 | 61.86 | **−21.3%** | 3622 |

Deleting the whole decode cone's influence on the pc takes the design to 61.9 ns and 16 MHz.
Deleting any one term of it takes the design nowhere. The largest single term is `trap_pending` at
5.1%, and **neither half of `trap_pending` reproduces any part of that** — dropping `instr_illegal`
measures +1.1% and dropping the misalignment terms +1.5%, both *slower* than the baseline. The paths
under the top one are stacked so close that removing the first uncovers the second at almost the same
length.

ADR-0058 saw this from the tail side and read it as a head-and-tails structure: "the head is shared
and the tails are not; shortening a tail uncovers the next path." The head behaves the same way, so
there is no shared lever to pull — only a plateau, which moves when the whole of it leaves the cycle
and not before.

Its second ceiling reproduces at the current design too: row E takes `stall` off `next_pc`, the
critical path leaves the fetch loop entirely and lands on `→ riscv.csrs.minstret`, and the design
measures 77.42 ns. A change that shortens only the fetch loop is still capped at about 1.5%, against
the 1.7% ADR-0058 measured.

## Consequences

- **`CLAUDE.md`'s decode-head sentence is withdrawn and replaced with this measurement.** It was
  ADR-0058's recommendation promoted to a rule before anybody had measured it; the rulebook now says
  what the ceiling is instead of where to look next.
- **ADR-0074's reopening condition cannot be met by shortening the decode head.** Its consequence
  section defers the operand-fetch cycle to "a change that buys back headroom in the decode head",
  and there is at most 3.3% there against the 3.6% that variant's median cost. The operand-fetch
  idea is now blocked on something structural rather than on a tuning change, and its 0.83%-margin
  variant is still the only version that meets the requirement.
- **The dead-end list grows by five, and they are dead at their ceilings.** Re-running one is the
  waste this list exists to prevent:
  1. `rs1`/`rs2` decoded off the raw fetch word before quadrant resolution — **1.5% slower with the
     mask deleted outright, 2.0% slower with it correctly hoisted, +102 cells.**
  2. The RVC register-selection muxes deleted — **null, on a 7.8% spread**, and *bigger* by 123 cells.
  3. The scoreboard narrowed from three in-flight slots to one — **1.9%, half the churn band**, and
     not implementable anyway.
  4. The 18-way `immediate` mux collapsed to one arm — **1.2%, inside the churn band.**
  5. Merging the two `rs1` arms that select the same value — **1.4% slower, and it leaves 0.9% of
     margin over the board clock.**
- **T, T2, T3 and BR are why a single-term candidate gets declined unbuilt:** the term you delete is
  not the term that was costing you. Bring a ceiling for the whole cone or bring nothing.
- **The 21% is real and it names its own price.** Reaching it means the pc no longer depends on this
  cycle's decode, and every way of arranging that is state a later cycle has to un-commit. That is
  the no-wrong-path-state commitment, and amending it is a decision this ADR does not make and this
  measurement does not by itself justify — 21% of a period the design already meets by 4.5% buys
  nothing on its own. It is written down so the next attempt starts from a number.
- **`make cycles` was not re-run and did not need to be.** No candidate here changes which cycles
  issue: the ceilings do not execute, and R and M are the same function as what is checked in.
- **The suite was run to say the tree is unchanged, not to grade a change.** 59/59 on `make test`
  with both baselines matching, on a working tree byte-identical to `2d79878` under `rtl/`.
