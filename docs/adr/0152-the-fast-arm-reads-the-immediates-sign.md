# ADR-0152: The region fast arm can read the immediate's sign, and the layout convention got there first

**Status:** Declined (the measurement stands) · 2026-09-03 · superseded as a change by ADR-0158,
which recovers the same cycles in the linker for no cells and no period.

## Context

ADR-0129 moved the load/store region answer off the fetch loop and a cycle late: `reg_rs1` deep
inside a mapped window — in it, and in neither its first 2 KB block nor its last — is answered in the
issuing cycle off raw register bits with no adder, and anything nearer an edge bubbles one cycle and
reads a flip-flop. The price was **+13.79% of Dhrystone's cycles**, 9.10 → 7.97 DMIPS, and it bought
causes 5 and 7 for the twelve plain load and store encodings.

That fast arm is symmetric and the offset is not. A 12-bit signed immediate moves the base's 2 KB
block by at most one step, and only in the direction its sign allows: `immediate[31] == 0` can leave
the block upward and never downward, `immediate[31] == 1` the reverse. So "in the window and not its
LAST block" is enough for a non-negative offset and "in the window and not its FIRST block" for a
negative one, and both edge blocks become answerable in the issuing cycle for half the offsets each.
The decoder's own `ifdef FORMAL` block already asserts the fact this stands on —
`immediate[31:12] == {20{immediate[31]}}` for every `ls_access` encoding — and the compressed load
and store forms zero-extend, so they take the non-negative arm.

**The prior evidence predicted this would be declined.** ADR-0145 added a second conditional case to
this same `ls_text_deep`/`ls_supported` block and measured 16 of 16 paired seeds slower, median
−5.89% of period, and 5 of 16 placements under the 12.00 MHz floor, with every candidate placement's
critical path ending on a `region_stall`-adjacent net. `ls_settled` feeds
`region_stall → stall → next_pc`, which is the fetch loop, and ADR-0116 and ADR-0128 measured seven
spellings of the same-cycle answer at +9.10% to +17.30% of median period. This was built anyway on
this repo's own rule that an inherited conclusion is not a measurement: ADR-0145 measured a second
window ORed into the range test, not a sign-selected pair of block tests.

## The decision

**`rtl/decoder.v`'s `ls_text_deep` and `ls_ram_deep` split into an in-window test and the two end-block
tests, and pick the end by `immediate[31]`.** `ls_settled`, `region_stall`, `ls_capture`, the held
answer and `ls_supported` are untouched; the arm only ever widens, so nothing that waited before can
fault now and nothing that faulted before can stop waiting. A window of a single block is both ends at
once and both arms fold to constant zero, which keeps the timer's, the UART's and the SPI
controller's windows off the fast path in the safe direction. The refusal stays one-sided — a miss
means "wait for the flip-flop", never "fault" — so `mcause` is still a function of the access and not
of the base register, and the neighbourhood test ADR-0129 declined for exactly that reason is still
declined.

## Correctness, before timing

`rtl/decoder.v`'s `if (ls_settled) assert(ls_supported)` is the whole correctness argument and it is
**proved by k-induction on the changed arm** (`make -C formal components_decoder`: successful proof
by k-induction), with `formal/decoder-zkt-probe.py`'s two red directions still failing at their own
lines (1387, 1389) as its prerequisite. `test/zkt_isolation_test.py` passes unchanged: `immediate[31]`
is an instruction bit, `in.instr` is classified non-value, and the `EXPECT_TAINTED` chain
(`ls_block`, `ls_text_deep`, `ls_ram_deep`, `ls_settled`, `region_stall`) survives verbatim. All **86
generated riscv-formal checks pass**, matching `EXPECTED_FAIL` and `EXPECTED_CHECKS` exactly;
`components_traps`, `components_pcloop` and `nonperturbation` pass; `complete` passes over its
recorded exclusion set. **F and G re-measured and both flip points reproduce at 6 and 6**, which is
expected rather than lucky — the arm strictly narrows when `region_stall` asserts, so neither the
worst-case first retire nor the worst-case retire gap can grow. `make test` is 74/74 and
`make cosim-suite` is 69/74 agreed, both matching their baselines.

Three graders moved and each is a retire count rather than a claim. `test/decoder_tb.v`'s fast-path
vectors were symmetric and are now stated in both directions — a first-block base waits under a
negative offset and issues under a non-negative one, a last-block base the other way round — and five
vectors that took the wait through a first-block base were moved to a last-block one, since the
sign-aware arm settles the old base. `test/OBSERVED_FLOOR`'s `uart.S` line is re-derived, 1381 → 1380:
it is a poll loop around a 10-bit frame, so how many times it goes round is a CPI question, and this
change moves it the way the region wait moved it when that landed. `test/MUTATION_DETECTORS` gains one
pairing: `lrsclock.S` is a retry loop under a live timer, and it now detects `mtip-fires-a-tick-early`
by falling through its floor. **That pairing was one retire away from existing already** — measured
both ways here, the mutation leaves `lrsclock.S` at exactly its floor of 156 on the base tree and at
155 on this one, against 156 and 159 unmutated — so the claim it replaces ("the whole suite stays
green under it") was true by a margin of zero. It is a weak, accidental grader kept for the reason
`uart.S` is kept as one, and the file says so at the line.

## The cycles

`make cycles`, the 74-program `.S`/`.c` suite: **42 319 → 40 885 cycles, −3.39%**, CPI 2.00 → 1.94,
with the REGION column **2224 → 780, −64.9%**.

`make dhrystone`, `-O2`, 3580 bytes of the SoC's 8 KB ROM: **1 756 248 → 1 661 678 cycles, −5.38%**,
REGION **212 752 → 118 171, −44.5%**, and **0.664 → 0.703 DMIPS/MHz**. At the board's 12 MHz that is
**7.97 → 8.44 DMIPS**, which is a throughput figure and not margin.

**The prize is 2.9× what the sketch that proposed it predicted.** That sketch put the whole win at
the 32 516 first-block accesses (−1.85% of Dhrystone's cycles) and said the stack — 84.7% of the
region column, which it read as a last-block base under positive offsets — was exactly the crossing
this cannot settle. 94 581 cycles came out, so either that decomposition or the sign it assigned the
stack's offsets is wrong; the measurement does not say which, and the 118 171 that remain are what a
future attempt has left to price.

Two probes in `rtl/littlecpu.v` are deliberately **not** updated and no longer predict the REGION
column: `probe_ls_edges` counts what a region test answered from `reg_rs1` ALONE would stall on, which
is a family of designs rather than the shipping arm, and it reads no immediate. It stays at 212 752 on
Dhrystone against a column of 118 171, and the gap between the two is now the value of reading the
sign. `probe_ls_bypasses` moves 4053 → 70 088 for a different reason: near-edge accesses issue a cycle
earlier now, so many more of them land on a cycle the register file is writing their base register
through.

## The period, which was the whole question

**Sixteen paired placements, `datainit.c`, same tree (`dba70e9`), same toolchain, base against
candidate:**

| | worst | median | best | spread | under 12.00 MHz |
|---|---|---|---|---|---|
| base | 12.06 MHz | 12.99 MHz | 13.37 MHz | 10.8% | **0 of 16** |
| candidate | **12.53 MHz** | 12.85 MHz | 13.30 MHz | 6.1% | **0 of 16** |

**The median period is +1.07% and 13 of 16 seeds are slower** (two-sided sign test p = 0.021) — a real
systematic cost, and one smaller than the churn band `soc/bands.py` states for this part. The tail did
not follow it: the worst placement is 3.8% FASTER, and **no placement on either side is under the
12.00 MHz requirement**. Read the worst column as one sample of a distribution the way this repo reads
every other margin — the base's own worst of sixteen has read 12.39, 12.21, 12.03 and 12.06 MHz on four
trees — not as evidence that the change made the part faster.

`make fit` is a null: **4109 → 4108 packed `ICESTORM_LC`** on the core's own top. The SoC's placed
count is not: **4943 → 5055, +112 cells**, deterministic across all sixteen seeds a side. That is the
top-dependent split ADR-0094 measured for a different edit, and at 5055 of 5280 it is the number a
future change in this area has to budget against.

## What this does not establish

It does not refute ADR-0145, which measured a different term: a second window ORed into
`rtl/imemory.v`'s range test and `rtl/decoder.v`'s, with a bank-index clamp beside it. It says only
that "a term added to `ls_settled`" is not one class with one price — this one is two 5-bit equalities
per window and a 2:1 mux on a bit the decoder already has, and it measures inside the churn band where
that one measured −5.89% and missed the floor.

It is not a claim that the fetch loop got cheaper. +1.07% at the median with p = 0.021 is a cost, and
what pays for it is 5.38% of Dhrystone's cycles at a clock that still closes; margin above 12 MHz is
not speed, so the trade is a CPI win bought with headroom this design had.

It does not close the region column. 118 171 of Dhrystone's cycles are still spent waiting, the two
locality probes no longer bound them, and nothing here measured which accesses they are.


## Why it does not ship

Measured against the CONVENTIONAL layout, this arm is worth 5.38% of Dhrystone's cycles. ADR-0158
insets `.data` and the stack by one 2 KB block in every shipping linker script and takes the same
`REGION` column from 212 752 cycles to 2, for no RTL, no cells and no period. Stacked on that
layout — both changes in one tree, `make cycles` and `make dhrystone` re-run — this arm is worth
**2 cycles** of Dhrystone's 1 543 495 and 208 of the suite's 41 052.

Its costs do not shrink with its prize. The SoC grows **112 placed `ICESTORM_LC`**, outside the
±50 churn band, on a design already at 94% of the part; the median period is **+1.07% at 13 of 16
paired seeds, two-sided sign test p = 0.021**, which is the most significant of the three period
readings taken this day. Two cycles do not buy that.

**What the measurement is still good for.** It is the counter-example to reading ADR-0145's verdict
as a rule about the block: a different term added to `ls_text_deep`/`ls_ram_deep` measured −5.89% of
median with five placements under the floor, and this one measured +1.07% with none, so that ADR
prices its own term and not the signal. It also establishes the arm's correctness, proved by
k-induction against the unchanged `if (ls_settled) assert(ls_supported)`, so a future reader who
needs it for a workload that cannot follow the layout convention has the design and its price
rather than an estimate.

**What would reopen it.** Firmware that cannot inset its own layout, or a second window narrow
enough that the convention has nowhere to put a hot base. Re-take the sweep stacked on the shipping
layout rather than against the conventional one; the numbers above are the only paired figures that
describe the tree as it now stands.
