# ADR-0119: The AMO result mux is a truth table, and an adder cannot be shared on this fabric

**Status:** Accepted · 2026-08-18 · *Collects the last open row of
[ADR-0112](0112-the-per-module-census-is-not-a-ceiling-and-four-more-blocks-are-closed.md)'s accessor
table and closes the block. Ships one `rtl/accessor.v` edit — **−54 packed cells at all sixteen
placements, median period −2.67%, worst placement 12.21 → 12.27 MHz.** Declines the sharing
arrangement the row was written for, on a measurement rather than on the obstacle
[ADR-0106](0106-the-a-extension-is-built-and-the-board-still-closes.md) recorded. Adds no stall
reason, lengthens no stage, widens no scoreboard; F and G were re-measured and both reproduce at 6.*

## Context

ADR-0112 priced every sub-block of `rtl/accessor.v` deleted whole and found exactly one row that
clears the ±50-cell band: **the AMO result mux with its 33-bit adder/subtractor, 241 packed cells and
32 carry bits.** That row is required function, so 241 is a budget and not a saving, and the record
named one way to spend it — share `rtl/executor.v`'s merged subtractor, whose identity is the same
one the four min/max compares use.

ADR-0106 had already recorded the obstacle: the executor's subtractor works on rs1 and rs2 **at
issue**, and an AMO's operands are the memory word and rs2, which do not exist until two cycles
later. What was never measured is whether the 241 is worth attacking at all, and how it splits.

## Method

Every cell number here was taken on base `16c5cad` with the cached OSS CAD Suite on this machine —
yosys 0.68+48 (`ff5817c34`), nextpnr-0.11-1-g62e659ed, `icetime` from suite `20260811` — with
`SOC_PROG` at its default `datainit.c`. Variants lived in a scratch copy of the tree and nothing was
written into the checkout. `SB_LUT4` comes from synthesis and the cell count from the placement,
because [the packed cell is the number](0112-the-per-module-census-is-not-a-ceiling-and-four-more-blocks-are-closed.md)
and it disagrees with `SB_LUT4` in magnitude and in sign.

The base reads **4254 `SB_LUT4`, 688 `SB_CARRY`, 4693 placed cells** on the SoC and **3932** on
`fit`, against ADR-0112's 4315 / 4754 / 3969 on `fe618f4`: the tree has moved under that table, which
is why the row was re-taken rather than subtracted from.

## The 241 splits into 72 of adder and 162 of mux, and they add up

| deleted, on `16c5cad` | `SB_LUT4` | Δ | Δ carry | placed LC | **Δ LC** | `fit` | Δ |
|---|---|---|---|---|---|---|---|
| **base** | 4254 | — | — | 4693 | — | 3932 | — |
| the result mux and the adder, write data becomes the held rs2 | 4024 | −230 | −32 | 4461 | **−232** | 3646 | −286 |
| the same, write data tied to zero *(cascading: the held rs2 goes too)* | 3973 | −281 | −32 | 4378 | −315 | 3585 | −347 |
| **the 33-bit adder alone**, its carry chain replaced by an XOR of the same two operands | 4184 | −70 | −32 | 4621 | **−72** | 3888 | −44 |
| **the result mux, its five non-add arms and its select**, the adder left driving the write | 4094 | −160 | −1 | 4531 | **−162** | 3770 | −162 |
| of which the three bitwise arms alone | 4168 | −86 | 0 | 4605 | −88 | 3847 | −85 |

The first row reproduces ADR-0112's −241 at **−232** on a tree 61 cells smaller, so the ceiling
travelled. The next two decompose it and **they add: 72 + 162 = 234 against the 232 measured
together**, which is as close as this instrument gets.

The adder is deleted against an XOR of the same operands rather than against a constant, because a
constant sum takes the compare, the sign correction and the mux's select with it and the row would
then bound four things. Read that −72 as "the carry chain and the inverted addend above a bitwise op
of the same fan-in".

**The three bitwise arms are 88 cells for 96 bit-operations**, which is one LUT per output bit with
the one-hot select folded into it. That is the fabric's floor: nothing computes 32 bits of `xor` in
fewer than 32 LUT4s. The saving, if there is one, is in the other 74.

## Sharing an adder is not a saving on this part, and that is measurable without building it

A probe of four tiny designs, same registers, same reduction, same flow — `synth_ice40` then placed
on the up5k:

| 33-bit construct | `SB_LUT4` | `SB_CARRY` | placed LC |
|---|---|---|---|
| one adder | 90 | 32 | 224 |
| one 2:1 mux | 92 | 0 | 226 |
| **one adder behind two operand muxes** | 154 | 32 | **288** |
| **two adders behind one result mux** | 154 | 64 | **288** |

**An adder and a 2:1 mux of the same width cost the same here**, because a carry cell packs into the
logic cell of the LUT beside it and costs nothing above it — the fact ADR-0112 measured as "67 carry
bits are worth three cells between them". So sharing one adder between two operand sources buys one
adder and pays two muxes, and the two arrangements place at **exactly the same cell count**.

That closes the proposal on arithmetic rather than on plumbing. The 72-cell adder in the accessor
cannot be paid for by a mux pair costing at least as much, and that is before the 66 wires of
cross-stage operand and 33 of sum, before the executor's issue-cycle arithmetic acquires an input
from the accessor, and before anything is asked of the period. **The accessor's adder is also
already shared across every use it has** — the add and all four compares are one chain — so there was
no second sharing left inside the module either.

Un-sharing it was measured too, since the fabric fact points that way: two chains, one `+` and one
`−` whose 32 difference bits are dead, reads **+14 `SB_LUT4`, +31 carry, +13 placed cells** and −6 on
`fit`. A null. The merged spelling stays.

## What does pay is the mux, and only after the arms stop being 32 bits wide

| candidate | `SB_LUT4` | Δ | placed LC | **Δ LC** | `fit` | Δ |
|---|---|---|---|---|---|---|
| two chains instead of one add/sub | 4268 | +14 | 4706 | +13 | 3926 | −6 |
| five arms instead of six — swap folded into the compare's pick | 4251 | −3 | 4688 | −5 | 3889 | −43 |
| **the per-bit truth table, `case` form (ships)** | 4205 | −49 | 4639 | **−54** | 3945 | +13 |
| the per-bit truth table, AND-OR form | 4203 | −51 | 4639 | **−54** | 3875 | −57 |

Folding `amoswap` into the min/max pick — all three choose between the memory word and rs2 — is worth
**five cells**, which is ABC having already found it. That is the null ADR-0088's rule predicts, and
it is the reason the win had to come from somewhere the expression could not say.

**Eight of the nine functions are one bit function of a memory bit and an rs2 bit repeated 32
times.** `and`, `or`, `xor`, `swap` and both directions of the four compares are each a four-entry
truth table indexed by that pair; only the add is not, because its bits depend on the carry into them.
So the op picks the table once and every bit indexes it: one LUT deep, where five 32-bit arms into a
one-hot mux are two. The add stays a mux over the adder's sum.

**Two independent texts of that idea place at the identical 4639 cells**, which is what says −54 is
the netlist and not a spelling. They differ by 70 cells on the `fit` top (+13 and −57) — the
spelling dependence [ADR-0097](0097-the-decode-stack-pays-only-in-the-fetch-loop-and-that-is-where-it-cannot.md)
measured at 44 — so `fit` is a null here in both directions and the shipping top is where this is
graded. The `case` form ships, because it is the module's own idiom and keeps the
`(* parallel_case *)` marking paired with a `$onehot0` assertion over exactly its arms.

## The period, at sixteen placements paired per seed

Both sides swept with `soc/baseline_sweep.sh` at the same sixteen seeds, one tree apart, one
toolchain, `datainit.c`.

| | before (`16c5cad`) | after (`6562c7e`) |
|---|---|---|
| worst of 16 | 81.92 ns — **12.21 MHz** | 81.53 ns — **12.27 MHz** |
| median | 79.10 ns — 12.64 MHz | 77.81 ns — 12.85 MHz |
| best | 75.86 ns — 13.18 MHz | 76.00 ns — 13.16 MHz |
| placed LC, all 16 | 4693 | 4639 |
| LUT levels | 21–23 | 19–23 |

**Median of the per-seed deltas: −2.67%. Sign test: 11 faster, 5 slower, p = 0.210.** The requirement
holds at sixteen of sixteen and the worst placement improves by 0.5%, which is inside the placement
spread; read the median as the result and the tail as a requirement that was met, not as a win. It is
consistent with what the cell count says — the block loses a LUT level in the worst case — and this
is not the fetch loop, so nothing here was expected to move the tail.

The warning in ADR-0117 is the reason the sweep exists at all: an edit that looks like pure area can
cost 2.83% of median period and a placement. This one does not, and that is a measurement rather
than a prediction.

## Decision

**Ship the truth table.** `rtl/accessor.v` builds eight of the nine read-modify-write functions from
one four-entry table indexed per bit, and the add stays a mux over the adder's sum. The
`(* parallel_case *)` arm list is re-derived to the five arms the table selects, with the add's own
mux in the same `$onehot0` assertion, because a function selecting the table and the sum at once
would take the sum with entries chosen for something else.

`test/accessor_tb.v` drives every one of those functions at operands covering **all four
combinations of a memory bit and an rs2 bit** — `0xcccccccc` against `0xaaaaaaaa`, eight bits per
combination — so no table entry is graded by fewer than eight bits. No test fails without this
change and none can: the behaviour is identical, and what those vectors buy is that a single wrong
entry cannot pass. The red direction was run: flipping one entry of `amoand`'s table makes both the
old vector and the new one go red for that reason.

**Decline sharing `rtl/executor.v`'s subtractor**, on the measured fabric arithmetic above rather
than on the cross-stage operand alone.

**The accessor is interrogated.** Every sub-block in it now has a measured ceiling, and the one row
that cleared the band has been decomposed and collected. Nobody needs to re-open it for cells:

- The 33-bit adder — 72 cells, already shared across every use in the module, and un-sharing it is
  +13.
- The three bitwise arms — 88 cells for 96 bit-operations, at one LUT per output bit.
- The result mux — 162 cells, of which 54 have been taken by the table and the rest is the add mux,
  the compare's sign correction and a select ABC had already collapsed.
- Everything else — the reservation, the load lane shifter, the `launch_is_*` fan-out, the request
  block — closed by ADR-0112 and not re-taken.

**Unchanged:** `SOC_MIN_MHZ` at 12.0, `FIT_MAX_LC` at 4088, the BMC depths, every exclusion set and
every baseline. `make cycles` is identical to the cycle — 34051 cycles, 18352 retires, CPI 1.86, and
the same count in each of the six stall columns — because nothing about control changed.

## Consequences

- **An adder and a 2:1 mux of the same width cost the same on this part.** Any proposal to share an
  arithmetic unit between two operand sources therefore starts at zero and pays routing, and the
  ones in this design cross a stage boundary. This is a general result, measured on four small
  designs, and it is the answer to the shape of proposal ADR-0112's table invited.
- **A ceiling says what a block costs and not where the cost is inside it.** 241 cells looked like
  an adder worth sharing; it is 72 of adder, 88 of bitwise arms at the fabric floor and 74 of select,
  and the only 54 anyone could collect were in the part nobody had named. Decompose before
  restructuring — the three rows above cost three synthesis runs.
- **Two texts of one idea agreeing on the shipping top is what promotes a band-edge delta.** −54
  cells is barely outside ADR-0112's ±50; measured twice from two spellings at the identical 4639,
  with `fit` a null in both directions, it is the netlist. A single text at −54 would have been
  churn under ADR-0094's rule.
- The A extension's arithmetic is now spelled in a way that is harder to read than nine named arms,
  and the mitigation is that its formal block still asserts each of the nine against the expression
  the spec names for it. **The assertions are what make the dense spelling affordable**, which is the
  standing argument for keeping them.
