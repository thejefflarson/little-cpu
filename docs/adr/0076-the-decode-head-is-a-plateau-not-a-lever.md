# ADR-0076: The decode head is a plateau, not a lever

**Status:** Accepted · 2026-08-06 · *A measured null. Corrects
[ADR-0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md)'s closing recommendation and the
sentence it put in `CLAUDE.md`, which
[ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md) then relied on
when it deferred the operand-fetch cycle to "a change that buys back headroom in the decode head".
Graded against [ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement.*

**Amended at integration, before merge.** The combined row below was written as though its 21%
needed the decode head deleted along with the `next_pc` terms. It does not:
[ADR-0078](0078-a-one-deep-kill-is-cheap-and-buys-a-clock-the-board-cannot-use.md) reached the same
ceiling with the entire decode head left standing, and measured an implemented fetch-loop change at
four times what this ADR's row E predicted was possible. The attribution of the 21%, the cap this
ADR put on fetch-loop work, and the rows whose deltas sit inside the churn band are corrected here
rather than in a third document. The measurements are unchanged; what they are read to mean is not.

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

The reason generalises past the decode head, and it is what the title means: **no single input to
`next_pc` is worth more than 5% on its own, and all of them together are worth 21%.** This is a
plateau, not a peak with a lever on it. The plateau is the `next_pc` input set — the decode head
reaches it only through `stall`, and contributes nothing to the 21% once the pc is off decode.

A candidate that shortens one term of the fetch loop still has to be measured. This ADR originally
said such a candidate could be declined on the tables below without being built; that is withdrawn,
because the whole set is worth far more than any ceiling here bounds. What the tables do license
unbuilt is a candidate confined to the **decode head**, whose 3.3% ceiling is inside the churn band.

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
finding rather than a failure to find one. A ceiling bounds only the terms it deletes; it says
nothing about the terms it leaves standing, so a single-term ceiling cannot cap a set.

**How to read a row against the noise.** The instrument has a 1–2% placement spread and a 3.6%
edit-churn band, and the baseline's own eight placements span 3.5%. **A median inside that band is
reported as indistinguishable from baseline, in both directions** — a row that reads +1.4% is not
evidence a candidate is slower any more than a row reading −1.2% is evidence one is faster. Several
rows below are declined on their `fit` number or on their worst placement instead, and each says
which. This rule is applied symmetrically on purpose: a dead-end list recorded on noise stops work
that was never measured to be dead.

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
four gone at once, on a design that then cannot execute anything. It buys 3.3% of the period —
inside the churn band, so the ceiling of the whole head is itself barely distinguishable from doing
nothing — and 75 cells, one LUT level off a path that is 23 of them. The critical path does not move: start and
end stay `imem.in_range → imem.rom_even`, and the split stays 36% logic and 64% routing.

Two of the four rows need a word beyond their number:

- **Row A buys nothing and costs cells: the width mask earns its keep.** Zeroing `instr[31:16]` for a
  compressed instruction is what lets every field cut from that half fold to a constant, and a raw
  upper half hands each consumer of `funct7`, `csr_addr` and the I/S/B/U/J immediates a live input
  where it had a zero. Its +1.5% is inside the churn band and is **indistinguishable from
  baseline**; what carries the row is area, **+75 cells against a ±50 floor**, in the one direction
  a ceiling is not supposed to move. Deleting the mask outright buys no measurable period and costs
  cells, and no version that keeps the core working can beat a ceiling that already buys nothing.
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
in.instr[19:15] : <compressed select>` is the same function with the mask off the register path. Its
median is 2.0% the wrong way — inside the churn band, so **no measurable period change** — and it is
**102 cells bigger, twice the churn floor**, which is what declines it. The mask was not a level in
front of the mux; it was folded *into* the mux's default arm, and splitting them adds a 5-bit mux at
the end of the chain and a second fanout point on `uncompressed`.

**M merges two arms of the `rs1` case that select the same value** — `instr_cjr || instr_cjalr ||
instr_cslli` and `instr_caddi || instr_caddi16sp || instr_cadd` both give `rd_field`. It is a strictly
smaller mux and one fewer line. Its median is 1.4% the wrong way — inside the churn band, and its
+25 cells are inside the ±50 floor, so on both instruments it is **indistinguishable from what is
checked in**. What declines it is the worst of its eight placements, 82.59 ns — **12.11 MHz, 0.9% of
margin over the board clock**, against the baseline's 12.49. 12 MHz is a requirement graded on the
worst placement, not on the median, and ADR-0074 declined its second variant on the same ground at
0.83%. A change that reads better, is provably the same function, and buys no measured period is not
worth four fifths of the margin.

## What every other term of `next_pc` is worth

The ceilings were extended past the decode head to bound what any change to the fetch loop could
be worth. This is the part that outlives the nanoseconds.

| variant | seeds | ns, sorted | median | vs baseline | `fit` LC |
|---|---|---|---|---|---|
| T — `trap_pending` off `next_pc` | 4 | 73.99 · 74.00 · 75.20 · 75.41 | 74.60 | **−5.1%** | 3858 |
| T2 — T without `instr_illegal` only | 4 | 77.06 · 79.36 · 79.50 · 82.03 | 79.43 | +1.1% (null) | 3864 |
| T3 — T without the misalignment terms only | 4 | 78.91 · 79.01 · 80.45 · 81.02 | 79.73 | +1.5% (null) | 3879 |
| BR — the branch comparators off `next_pc` | 4 | 74.77 · 78.03 · 79.09 · 80.72 | 78.56 | 0.0% | 3880 |
| E — `stall` off `next_pc` | 4 | 74.71 · 77.09 · 77.74 · 79.09 | 77.42 | −1.5% | 3928 |
| **all of them, plus A+B+C+D** | 4 | 60.83 · 61.49 · 62.22 · 62.81 | 61.86 | **−21.3%** | 3622 |

Taking the pc off this cycle's decode takes the design to 61.9 ns and 16 MHz. No one term of it gets
close: the largest is `trap_pending` at 5.1%, a fifth of what the set is worth, and it is the only
row in this table outside the churn band in either direction. **Neither
half of `trap_pending` reproduces any part of that** — dropping `instr_illegal` and dropping the
misalignment terms both land inside the churn band, indistinguishable from baseline on period and on
area alike. That is the finding: neither half accounts for any part of T's 5.1%, because the paths
under the top one are stacked so close that removing the first uncovers the second at almost the
same length. Neither row is evidence that either half is *slower* than what is checked in, and
neither is a dead end on its own — they are two more terms of a set that is only worth something
whole.

**The 21% belongs to the `next_pc` terms alone, not to the decode head.** The combined row deletes
A+B+C+D as well, and did not need to: ADR-0078's `K∞E` ceiling deletes every redirect term *and*
`stall` from the fetch address with the whole decode head left standing, and measures 61.27 ns over
four placements against this row's 61.86 over four — 0.95% apart, distributions overlapping
(60.77 – 62.38 against 60.83 – 62.81), which is inside every band this instrument has. The
decode-head half of this row contributes nothing measurable to its 21%. The decode head is in the
fetch loop only through `stall` → `hazard` → `rs1`/`rs2`, and once `stall` leaves the fetch address
the head stops mattering at all.

ADR-0058 saw this from the tail side and read it as a head-and-tails structure: "the head is shared
and the tails are not; shortening a tail uncovers the next path." The `next_pc` terms behave the
same way among themselves, so there is no shared lever to pull — only a plateau, which moves when
the whole of it leaves the cycle and not before. The decode head is not part of that plateau; it is
a separate, flatter one worth 3.3% at its own ceiling.

Row E is where this ADR over-read its own method, and it is corrected here. It takes `stall` off
`next_pc`, the critical path leaves the fetch loop and lands on `→ riscv.csrs.minstret` at 77.42 ns,
and this ADR concluded that a change shortening only the fetch loop is capped at about 1.5%. **That
cap is withdrawn.** Row E is a single-term ceiling, and by this ADR's own thesis a single term
bounds nothing: it leaves every redirect term in the fetch loop, so the 77.42 ns path it uncovers is
not a floor for the set. ADR-0078 deleted the redirect terms instead and measured −6.8% at that
ceiling, and **built** the change — a registered fetch address with a one-deep kill — at −4.5% over
eight placements, which clears the churn band. Nothing in this table predicted that, and the row
that claimed to rule it out could not have.

## Consequences

- **`CLAUDE.md`'s decode-head sentence is withdrawn and replaced with this measurement.** It was
  ADR-0058's recommendation promoted to a rule before anybody had measured it; the rulebook now says
  what the ceiling is instead of where to look next.
- **ADR-0074's reopening condition cannot be met by shortening the decode head.** Its consequence
  section defers the operand-fetch cycle to "a change that buys back headroom in the decode head",
  and there is at most 3.3% there against the 3.6% that variant's median cost. The operand-fetch
  idea is now blocked on something structural rather than on a tuning change, and its 0.83%-margin
  variant is still the only version that meets the requirement. ADR-0078 then found where the
  headroom actually is — 4.5% of period from a registered fetch address — and declined it for
  reasons of its own, so the enabler exists but is not available. The operand-fetch cycle is blocked
  behind a commitment amendment, not behind a decode-head edit.
- **The dead-end list grows by five, and every one of them is dead for buying nothing measurable —
  not for measuring slower.** No row here is outside the churn band in the wrong direction, and none
  is quoted as though it were. Re-running one is the waste this list exists to prevent:
  1. `rs1`/`rs2` decoded off the raw fetch word before quadrant resolution — **no measurable period
     either way, with the mask deleted outright or correctly hoisted; +75 and +102 cells.**
  2. The RVC register-selection muxes deleted — **null, on a 7.8% spread**, and *bigger* by 123 cells.
  3. The scoreboard narrowed from three in-flight slots to one — **1.9%, half the churn band**, and
     not implementable anyway.
  4. The 18-way `immediate` mux collapsed to one arm — **1.2%, inside the churn band.**
  5. Merging the two `rs1` arms that select the same value — **no measurable period or area change,
     and its worst placement leaves 0.9% of margin over the board clock.**
- **T2, T3 and BR are not dead ends and are not on that list.** They are single-term ceilings inside
  the noise, which is a statement about the term and not about any candidate that touches it. What
  they show is that **the term you delete is not the term that was costing you** — bring a ceiling
  for the whole set or expect a null. They do not license declining a fetch-loop candidate unbuilt;
  ADR-0078 built one this ADR's rows would have talked it out of, and measured −4.5%.
- **The 21% is real, it belongs to the `next_pc` terms alone, and it names its own price.** Reaching
  it means the pc no longer depends on this cycle's decode, and every way of arranging that is state
  a later cycle has to un-commit. That is the no-wrong-path-state commitment, and amending it is a
  decision this ADR does not make and this measurement does not by itself justify — 21% of a period
  the design already meets by 4.5% buys nothing on its own. It is written down so the next attempt
  starts from a number, and ADR-0078 is that attempt: it priced the commitment, found the cost
  affordable and the 21% unreachable at a clock the board can select, and declined it on the
  oscillator rather than on the commitment.
- **`make cycles` was not re-run and did not need to be.** No candidate here changes which cycles
  issue: the ceilings do not execute, and R and M are the same function as what is checked in.
- **The suite was run to say the tree is unchanged, not to grade a change.** 59/59 on `make test`
  with both baselines matching, on a working tree byte-identical to `2d79878` under `rtl/`.
