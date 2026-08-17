# ADR-0117: The group is not the edit — the trap decode buys 2.5% of period and the wen mask costs it

**Status:** Accepted · 2026-08-17 · *Prices two items for
[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md)'s list and collects
none of them. Moves the Dhrystone figure
[ADR-0089](0089-the-operand-fetch-guess-ships-and-one-workload-feels-it.md) and
[ADR-0099](0099-the-memory-transaction-launches-from-the-execute-slot.md) quote. Amends no
commitment: no stall reason is added, no stage lengthens, the scoreboard does not widen, and F and G
were re-measured and both reproduce at 6.*

## Context

Six changes, found by reading `rtl/` and the placement artifacts, filed as three groups: two edits
billed as pure timing on the measured critical path, one cycle win, three cleanups. Each group was
built and swept at **sixteen seeds paired per seed**, against a base sweep of the same sixteen on the
same tree and the same resolved toolchain.

The first result is about method, not about any of the six. **Two edits filed as one group point in
opposite directions, and bundled they read as a wash that loses a placement.** Split, one of them is
the largest period win measured on this tree in some time and the other is an anti-win. A group is a
unit of *filing*; it is not a unit of *measurement*.

## Decision

**Ship three of the six as one stack.** `rtl/decoder.v` decodes `instr_error` from the raw encoding
fields; the operand pair presented to the register file is held through a stolen fetch window; and
three flags leave `decoder_output` — `is_lui`, `is_ecall`, `is_ebreak` — while one, `is_amo`, joins
it in place of two nine-term ORs.

**Decline the fourth**, the deletion of `rtl/writeback.v`'s `wen` masking, on its own measurement.

Every number below is worst-of-16 for the requirement and the median of per-seed deltas with a sign
test beside it, because worst-of-N is the gate and not the analysis.

| variant | `fit` (local) | placed LC | median Δ period | sign test | worst of 16 |
|---|---|---|---|---|---|
| base | 3969 | 4754 | — | — | **12.39 MHz** |
| raw-field `instr_error` | 3935 | 4743 | **−2.47%** | 13 faster / 3 slower, p = 0.021 | **12.40 MHz** |
| `wen` mask deleted | 3937 | 4715 | +2.83% | 6 / 10, p = 0.45 | **11.89 MHz** |
| both, as filed | 3907 | 4651 | +0.98% | 6 / 10, p = 0.45 | **11.84 MHz** |
| held pair across a steal | 3952 | 4794 | +1.67% | 3 / 13, p = 0.021 | **11.76 MHz** |
| the three cleanups | 3921 | 4716 | +2.31% | 3 / 13, p = 0.021 | 12.17 MHz |
| **the three that ship** | **3932** | **4693** | +1.39% | **8 / 8, p = 1.000** | **12.21 MHz** |
| all four | 3875 | 4651 | +1.03% | 5 / 11, p = 0.21 | **11.90 MHz** |

### The trap decode is the win, and it is a fetch-loop win

`instr_error` read the *muxed* `rs1`/`rd` rather than the encoding fields. Every SYSTEM form with
`funct3 == 0` is told apart by `funct12` alone, so both fields have to be tested for zero — and
reading the muxed pair put the whole compressed register-select cone in front of `trap_pending`,
which chooses the next pc. The two values are equal for a different reason on each side:
`uncompressed` settles the rs1 mux, because every arm of it is a compressed encoding; the rd mux
falls to its default arm because its first arm covers uncompressed branches and stores and a SYSTEM
opcode is neither.

**−2.47% of median period, 13 of 16 seeds faster, worst placement flat at 12.40 MHz against 12.39.**
At 78 ns that is about **1.9 ns of median**, and 11 cells — a null on area in both instruments. It
buys median rather than tail, which is the shape of a win that removes depth from a cone the placer
was already routing loosely.

### The mask deletion is the anti-win, and the reason is the one already recorded

`waddr`/`wdata` reach only `rtl/regfile.v`, and all four consumers there already gate on `wen`, so
the masks are dead logic sitting in the loop from a committed result back to the branch comparator.
Deleting them buys 39 placed cells and **costs +2.83% of median period, with the worst of sixteen at
11.89 MHz — under the board clock.** In the full stack it is the difference between 12.21 MHz and
11.90.

That is the third measurement of the same shape: an edit that deletes logic from this SoC's fetch
loop and makes the period worse. It joins the flattened `next_pc` chain and the muxed adder operand,
and it is the reason the masks now carry a comment saying they stay.

### The held pair is a cycle win the tail cannot pay for alone

A stolen fetch window delivers a data word, so the pair decoded from it is one no instruction reads.
Presenting it threw away the successor guess made before the steal, and the instruction behind the
steal paid an operand-fetch cycle it had already earned. Holding the pair costs at worst what
presenting garbage cost, because `operand_stall` is the same compare either way — the guard is
untouched, which is the rule that matters here.

**Dhrystone: 1 564 022 → 1 502 022 cycles, −4.0%, 0.727 → 0.757 DMIPS/MHz**, the operand column
falling 295 585 → 232 308. The `.S` suite sees 21 cycles of it, because almost nothing there loads
from the text window; the compiled workload is where `.data` is copied and `.rodata` is read.

Alone it swept **11.76 MHz** at its worst seed. Stacked with the trap decode it does not: the three
that ship hold 12 MHz at sixteen of sixteen.

### The cleanups are a null on area and a spelling on period

One `is_amo` bit published beside the nine functions replaces two identical nine-term ORs, one in
decode's stall chain and one in `rtl/accessor.v`. LUI reaches the executor as an add of its
immediate with rs2 zeroed — the route a CSR read already takes — so its flag and its executor arm are
gone. `is_ecall`/`is_ebreak` were constant zero downstream, because everything that sets them raises
`trap_pending` and the trap arm clears every execution flag.

−38 placed cells and −48 on `fit`, both inside the band. The median moved +2.31%, which is inside the
churn band and is a null in both directions, and the worst of sixteen holds at 12.17 MHz.

### What the new bit costs in assurance, and what pays for it

`is_amo` is published rather than derived, so two readers now take it on trust. Three statements
replace the derivation: `rtl/decoder.v` asserts the bit is exactly the nine functions,
`rtl/accessor.v` assumes it, and `formal/traps.sv` requires it clear on a trapping issue beside the
eleven flags it already required.

`formal/pcloop.sv` had to move with them and that is worth recording, because it went red for a
reason that is not a bug: it is a `prove`, so an induction step may start in a state where the bit
and the nine flags disagree, and there the decoder raises the atomic wait for a cycle the harness's
excuse list does not cover. The harness reads the published bit now. **A derived mirror of a signal
is free until the signal stops being derived.**

## Consequences

- **The Dhrystone figure moves to 0.757 DMIPS/MHz, 9.08 DMIPS at the board's 12 MHz**, from 0.727 and
  8.72. The flags, the compiler and the string library travel with it as always.
- **Two prices for ADR-0111's list, neither collected.** The `wen` mask deletion is an anti-win at
  +2.83% of median and is not on the list at all — it is a cost with no payoff. The held pair is
  *collected here* at a stacked price of +1.39% of median for −4.0% of Dhrystone, which is the first
  entry taken by stacking rather than by waiting for margin.
- **A group is not a unit of measurement.** Two edits filed together, one of them worth −2.47% and
  the other +2.83%, measured as +0.98% and lost a placement. Split a filed group before believing
  its number, and split it again if the two halves touch different cones.
- **F and G are unchanged at 6 and 6**, re-measured by `make -C formal remeasure-fg`; both flip
  points reproduce. `make cycles` still reports six stall reasons with zero unattributed.
