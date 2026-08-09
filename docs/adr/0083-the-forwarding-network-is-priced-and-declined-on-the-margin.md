# ADR-0083: The forwarding network is priced, and declined on the margin

**Status:** Accepted · 2026-08-08 · *A measured null, and the first attempt to price the stall-only
commitment rather than restate it. Replaces
[ADR-0042](0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md)'s 44/52 as the evidence
`CLAUDE.md` offers against a forwarding network. CPI baseline from
[ADR-0070](0070-the-suites-cycles-are-charged-to-a-named-stall-reason.md); ceiling method and churn
bands from [ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md); graded against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement.*

## What was proposed

`make cycles` says the decode scoreboard is the largest non-issuing column and not by a little: of
32624 suite cycles, 15851 issue (48.6%), **9561 are charged to a RAW hazard (29.3%)**, 5287 to the
operand-fetch cycle (16.2%), and 1925 to everything else. The stall-only commitment is what those
9561 cycles buy, and `CLAUDE.md` says commitments change when a change moves all four goals
together.

So: forward the executor's result to decode instead of waiting for it to reach the register file.
No new state, no reorder buffer, no second issue port — the scoreboard's three in-flight slots stay
three, and what changes is only where an operand comes from.

## Decision

**No.** Two spellings were built, both run the suite and both pass every proof. The blocker on both
is the clock.

**The full network measures 9.49 MHz at its worst placement against a 12.00 MHz requirement — it
misses by 21%.** The narrow one, kept deliberately out of the fetch loop, holds 12 MHz on **0.48% of
margin** where the tree it is measured against holds it on 3.35%. It buys 8.1% of throughput and
spends 85% of the margin that keeps the requirement met, on an instrument whose own placement spread
is 1–2%.

Nothing in `rtl/`, `formal/` or `test/` ships from this ADR. What ships is the tables below, the
dead-end entries they add, and the two `CLAUDE.md` corrections they force.

## The published prior, and what it got right

VexRiscv publishes two RV32I configurations on iCE40 with the same five-stage shape, one with a
datapath bypass and one interlocking instead: **1130 LC / 92 MHz / 0.52 DMIPS/MHz against 1596 LC /
63 MHz / 0.82.** Forwarding bought 58% per clock and cost 32% of the clock and 41% of the area, for
a net 8%.

Two cautions were carried into this work and both mattered. The configuration labels are terse, so
the two may differ in more than the bypass; and **DMIPS/MHz is not CPI** — their per-clock number
and this suite's 2.08 are not the same measure and are never put in the same column below.

What the prior predicted correctly is *where* it would hurt: forwarding muxes land on the operand
path, the operand path is the decode head, and the decode head is inside the fetch loop. Their clock
fell 32%; the full network here costs 32.7% of the period. What it did not predict is the size of
the cycle win, and the reason is this core's shape: the regfile's write-through bypass already
covers the writeback slot for free, so **the cheap half of forwarding is built and only the
expensive half was left to buy.**

## Method

One tree, one toolchain: **Homebrew Yosys 0.68+post (`c12172fb`)**, nextpnr-0.10-108-g68c1acd8,
`icetime` from the same install, everything measured against `2773832`. `make soc-timing` at the
default `SOC_PROG`; `soc/timing_sweep.sh` for the sweeps. Area is `make fit`, whose churn floor is
±50 cells and whose absolute number is toolchain-dependent — quote CI's. **The tables report
periods, so a positive `vs baseline` is slower**, and a median inside the 3.6% edit-churn band is a
null in both directions.

Three designs:

- **S∞** is an ADR-0076-style ceiling: `hazard_rs1` and `hazard_rs2` deleted from `hazard` outright,
  so the core does not execute anything. It deletes exactly what a forwarding network hopes to
  remove and adds none of the muxes such a network needs, so **no implementation in this family can
  beat it on period or on area.**
- **F** forwards `executor_out.rd_data` into every reader of an operand in decode: the executor's
  operands, the branch comparators, the `jalr` target, the effective address and the RVFI report.
  The scoreboard's executor slot then waits only for a load, whose data does not exist until the
  accessor unpacks the memory word.
- **Fn** is F with the forward confined to the executor's operands. The branch comparators, the
  `jalr` target and the effective address keep reading the register file, and the scoreboard keeps
  interlocking for them — because a 32-bit mux in front of those three sits in the fetch loop.

Both F and Fn are working cores: **62/62 on `make test`** with retire counts identical to baseline,
and both sim legs' per-retire monitor agrees, which is not a formality here — the monitor keeps its
own record of every register's last write and checks `rs1_rdata`/`rs2_rdata` against it, so a
forwarded value that is not the architectural one is a monitor error on the retire that reads it.

## The ceiling, measured before anything was built

| variant | seeds | ns, sorted | median | vs baseline | `fit` LC |
|---|---|---|---|---|---|
| **baseline `2773832`** | 8 | 75.69 · 76.44 · 76.44 · 76.84 · 77.33 · 78.07 · 78.93 · 80.63 | 77.09 | — | 3958 |
| **S∞ — the whole scoreboard deleted** | 4 | 75.90 · 76.12 · 76.56 · 77.89 | 76.34 | **−1.2% (null)** | 3907 |

**Deleting the entire decode scoreboard buys no measurable period and no measurable area.** −1.2% is
a third of the edit-churn band, the two distributions interleave — the baseline's best placement is
faster than the ceiling's best — and −51 cells is the ±50 churn floor. This reproduces ADR-0076's
row C, which narrowed the scoreboard from three slots to one and measured −1.9%, and extends it to
the whole thing.

The cycle side of the ceiling is a bound rather than a measurement, because a core with no
scoreboard computes wrong answers: **at most 9561 cycles can be saved, so at best 23063 cycles,
CPI 1.473, throughput ×1.414.** It is an upper bound in the strict sense — a cycle charged to
another reason is not made cheaper by removing this one — and ADR-0078's overlap trap says the true
figure is lower, which both rows below confirm.

**The product of the two is what the ceiling is for: ×1.414 on cycles, ×1.00 on the clock.** It is
positive, so this could not be decided here — but it names the terms everything after it is settled
on. Every nanosecond a forwarding mux costs comes straight off the margin over the board clock,
because removing the scoreboard hands none back.

## What each spelling buys in cycles

| | cycles | retires | CPI | vs baseline | per clock |
|---|---|---|---|---|---|
| baseline | 32624 | 15654 | 2.0841 | — | — |
| **F** — every reader forwarded | 28416 | 15654 | 1.8153 | **−12.9%** | **+14.8%** |
| **Fn** — the executor's operands only | 30172 | 15654 | 1.9275 | **−7.5%** | **+8.1%** |
| S∞ — the bound | ≥23063 | 15654 | ≤1.4733 | ≤ −29.3% | ≤ +41.4% |

**ADR-0078's overlap trap reproduces exactly, and it is half the gap between F and the bound.** F
removes 5110 cycles from the hazard column (9561 → 4451) and gets 4208 of them: the operand-fetch
column moves *up* by 852 and serialization by 50, because an instruction that stops waiting on a
register starts waiting on its operand fetch instead. The other half of the gap is structural — the
scoreboard's other two slots cannot be forwarded from at all. An instruction sitting in
`decoder_out` is being executed this cycle and its result does not exist yet; a load in
`accessor_pending` has not had its memory word unpacked. What is left after F is 4451 cycles of
exactly those two.

## What each spelling costs in period, which is the decision

| variant | seeds | ns, sorted | median | vs baseline | worst | `fit` LC |
|---|---|---|---|---|---|---|
| baseline | 8 | 75.69 · 76.44 · 76.44 · 76.84 · 77.33 · 78.07 · 78.93 · 80.63 | 77.09 | — | **12.40 MHz** | 3958 |
| **F** | 4 | 100.15 · 101.06 · 103.61 · 105.34 | 102.34 | **+32.7%** | **9.49 MHz — FAILS** | 4017 |
| **Fn** | 8 | 77.16 · 78.59 · 80.18 · 81.78 · 81.96 · 82.06 · 82.19 · 82.93 | 81.87 | **+6.2%** | **12.06 MHz** | 4006 |

**F is not close.** Its critical path stays where the baseline's is, `imem.in_range →
imem.rom_even`, and goes from 23 logic levels to 27–30. At ~3.3 ns per LUT level that is the whole
of the 25 ns it lost. Every placement is under the requirement; the worst is 21% under it. Since the
part's oscillator divides 48 by 1, 2, 4 or 8, the clock below 12 is 6 — so on the board F is not a
quarter slower, it is a **half-speed** core, and its 14.8% per clock against that is a net loss of
43%.

**Fn is the one that had to be weighed rather than dismissed.** Taking the mux off the branch
comparators, the `jalr` target and the effective address moves the critical path off `imem.in_range`
on half its placements and holds it at 23 logic levels. It still costs 6.2% of the median period —
outside the churn band, so a real slowdown and not noise — and the reason is that **the mux left the
fetch loop but its select did not**: `decode_reads_rs1`, `decode_reads_rs2` and `exec_is_load` all
feed `hazard`, which feeds `stall`, which is an input to `next_pc`.

At the board clock the period does not matter directly — 12 MHz is the crystal, not a number the
design picks — so Fn's 8.1% per clock is 8.1% of real throughput. What it spends is the margin:
**3.35% over the requirement becomes 0.48%.** That is the blocker, on three grounds and not one:

1. **0.48% is inside the instrument.** `make soc-timing` has a 1–2% placement spread; the eight
   placements here span 7.5%. A ninth seed missing 12 MHz is not unlikely, it is a coin flip, and
   the requirement is graded on the worst placement rather than the median precisely because the
   worst one is the one the board has to run.
2. **It is below both standing precedents.** ADR-0074 declined its second operand-fetch variant at
   0.83% of margin and ADR-0076 declined its `rd_field` merge at 0.9%. Fn is thinner than either.
   Those two bought nothing measurable and this one buys 8.1%, which is why it was taken to eight
   placements rather than declined on four — but the thing being spent is the requirement itself,
   and ADR-0066 settled that the requirement does not slide.
3. **The gain is on this suite and the cost is not.** `make cycles` says in its own output that these
   are small hand-written programs with dense back-to-back dependencies and almost no loop structure,
   so 8.1% is nearer a best case for real code than an average. The 6.2% of period is a property of
   the hardware and does not vary with the program at all.

**Area objects to nothing.** F is +59 cells and Fn is +48, against a ±50 churn floor and 142 cells
of headroom under `FIT_MAX_LC`. A two-source mux on two operands is not what makes this expensive,
and VexRiscv's 41% area cost does not reproduce here — because only one of three slots is being
forwarded from, and the other two are the ones that would need real hardware.

## What it costs the proofs

Less than anything else here. Measured on **Fn**, which is the candidate that got far enough to be
worth proving.

- **F and G do not move.** `hang` is red at 6 and PASS at 7 — F = 6. `liveness` is red at gap 5 and
  PASS at gap 6, at trig 10 — G = 6. Both flip points reproduce the baseline exactly, so **no
  `[depth]` line changes** and `insn 19` and `reg 15 22` keep their one cycle of margin. This was the
  outcome least safe to assume: the scoreboard is what those depths are derived around, and a
  narrowed one could have lengthened the worst-case retire gap. It does not, because the gap is set
  by a load turnaround and a scoreboard chain that forwarding does not shorten — the load-use case
  is exactly what Fn still stalls for.
- **The generated checks are 85/85** at unchanged depths, both set equalities holding in both
  directions.
- **All four component proofs close by k-induction**, `components_traps` included, with no new
  assertion, no hand-written invariant and no proof-only port. `nonperturbation`, `dmemcheck` and
  `imemcheck` PASS.
- **The standing `reg_ch0` liveness probe still fires.** Deleting the rs2 write-through bypass from
  `rtl/regfile.v` gives `bad state property 1 reachable at bound k = 22 SATISFIABLE` — the rs2
  assertion specifically, the same signature ADR-0040 and ADR-0042 recorded. This was run before any
  green `reg_ch0` result was believed, because a change to operand delivery is exactly the class
  that makes that check pass by having stopped asking.
- `make cosim-suite`: **60/62 AGREE**, matching `test/COSIM_EXPECTED_FAIL`. Worth having rather than
  redundant — `test/cosim.cc` reads the core's real `regs_a` and no `rvfi_*` signal, so it is the one
  grader that could not be fooled by a forwarded value being reported consistently and stored wrong.
- `complete` and `complete_cover` were not run: they select `abc bmc3`, which the Homebrew Yosys used
  here rejects, and the same failure reproduces on the unmodified tree. They need the pinned OSS CAD
  Suite.

**`operand_stall` was not touched and did not need to be.** Forwarding overrides the register file's
answer; it does not change which address pair was presented or when. The rule that the regfile's
answer belongs to the pair presented last cycle survives intact, which is why `reg_ch0` and
`test/regfile_tb.v` had nothing to say about either candidate.

## What it costs to read

Worse, and this is where Fn separates from F.

F states one new idea and nothing else: an operand comes from the executor when the executor has it,
and the executor has it unless it is a load. A reader who knows the pipeline can check that in one
sitting, and it needed no new bench vector beyond re-timing the one that had a live executor slot.

Fn is **+42/−7 in `rtl/decoder.v` and +5 in `test/decoder_tb.v`**, and the extra lines are not
mechanism, they are an exception list. `decode_reads_rs1` and `decode_reads_rs2` classify every
instruction a second time — by whether decode reads the operand itself — and the scoreboard stops
saying *is this register in flight* and starts saying *is this register in flight in a way this
particular instruction cannot use*. Nothing in the ISA motivates that split. It exists because a mux
in front of the branch comparators lands in the fetch loop, which is a placement fact promoted into
the decode of every instruction, and a reader has no way to recover the reason from the file.

The stall taxonomy survives — `stall` is still exactly the OR of six reasons, `test/decoder_tb.v`'s
identity check passes unchanged, and `make cycles` reports zero unattributed cycles on both
candidates. Nothing here needs a seventh bucket or a second category, unlike ADR-0078's kill.

## Consequences

- **The stall-only commitment is not amended, and it is now priced rather than assumed.** Of the four
  goals it exists to serve, Fn moves one (8.1% of throughput at the board clock), holds one (every
  proof, at unchanged depths, with the liveness probe still firing), and loses two: it is measurably
  slower per cycle and it makes the scoreboard's rule conditional on a list of opcodes. The bar for
  amending a commitment is all four together.
- **`CLAUDE.md`'s stall-only bullet cited the wrong measurement and is corrected.** It offered
  ADR-0042's 44/52 as the evidence to beat. That number is a one-array regfile spike that failed
  eight programs — it says what a *broken* implementation costs, not what a working forwarding
  network costs, and it has been the only thing standing behind a design commitment for a milestone.
  The rulebook now carries these numbers instead.
- **The blocker is the margin, not the mechanism** — and that is the opposite of ADR-0078's finding,
  which is why both are worth keeping. There the design got faster and the board had no clock to
  spend it on. Here the win is in cycles, which the board *can* use at a fixed 12 MHz, and the cost
  lands on the one number that is a requirement. **A future candidate in this space has to show
  12 MHz at its worst placement with margin outside the 1–2% placement spread, and the ceiling of the
  whole space is that removing the scoreboard entirely buys 0% of period.** That sentence retires the
  direction, not just these two spellings.
- **The dead-end list grows by three** — the list ADR-0076 opened and ADR-0078 extended. Unlike
  ADR-0076's five, two of these are dead for measuring *slower*, not for buying nothing:
  1. The decode scoreboard's hazard terms removed by any means — **no measurable period and no
     measurable area at the ceiling: −1.2% of period on 4 placements, −51 cells against a ±50 floor.**
     There is no Fmax dividend in this direction to pay a network's muxes with.
  2. `executor_out.rd_data` forwarded to every operand reader in decode — **built and measured:
     −12.9% of cycles, +32.7% of period, +59 cells, 9.49 MHz at its worst placement against a
     12.00 MHz requirement.** The mux lands in the fetch loop and the path goes from 23 logic levels
     to 30.
  3. The same forward confined to the executor's operands, with the fetch-loop readers left
     interlocking — **built and measured: −7.5% of cycles, +6.2% of period, +48 cells, 12.06 MHz at
     its worst placement, 0.48% of margin against the baseline's 3.35%.** Every proof passes and F
     and G do not move; it is declined on the margin and on the exception list it puts in decode.
- **The two slots F could not reach cap any future attempt.** Reaching either means a combinational
  result path from the executor or from `mem_rdata` back into decode — strictly more logic on the
  path that already costs F its clock — and 4451 of the baseline's 9561 hazard cycles sit behind
  them. So the reachable part of the 29.3% is the 12.9% F measured, and what is left is the
  expensive part.
- **It did not become an out-of-order machine, and that was the thing to watch.** Neither candidate
  adds state, a reorder buffer, a second issue port or anything a later cycle un-commits. Both are
  operand muxes over the three slots that already exist. The commitment survives on its cost, not on
  a slippery slope.
- **VexRiscv's number transfers and its ratio does not.** The clock cost reproduces — they measured
  −32% of frequency, F measures +32.7% of period — but their +58% per clock becomes +14.8% here,
  because the regfile's write-through bypass already forwards the writeback slot. Reading their net
  +8% as this design's expected outcome would have been wrong in both terms.
- **`make cycles` is unchanged and needed no new bucket.** `stall` stays the OR of six reasons on
  both candidates, `test/decoder_tb.v`'s identity check passes, and `unattributed` is zero — so
  unlike ADR-0078's kill, forwarding costs the accounting nothing.
- **The suite was run to say the tree is unchanged, not to grade a change.** 62/62 on `make test`
  with the failure list matching `test/EXPECTED_FAIL`, on a working tree byte-identical to `2773832`
  under `rtl/`, `formal/` and `test/`.
