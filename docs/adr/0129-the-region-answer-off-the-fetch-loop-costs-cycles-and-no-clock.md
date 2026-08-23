# 0129 — The region answer off the fetch loop costs cycles and no clock

Status: accepted · closes the conformance gap ADR-0104, ADR-0116 and ADR-0128 each priced and
declined. Supersedes none of them: every measurement in them stands, and this is the direction none
of them tried.

## Context

An out-of-region plain load read zero and an out-of-region plain store was dropped, silently — the
last deviation from the privileged spec's strong recommendation that access faults be precise, and
recorded as a deviation rather than defended as a design choice.

Seven spellings had been priced and every one of them answered the region question **in the issuing
cycle**, which puts it in `next_pc`'s trap arm and so in the fetch loop. The cost is monotone in
which bit of `immediate + reg_rs1` the loop waits for: bit 31 is +17.30% of median period, bit 12 is
+9.10% (ADR-0116), and the cheapest exact spelling of all — three tests per window on raw register
bits with the carry into bit 12 recovered from the existing adder — re-measured at **+9.40% of
median period, 9 of 16 placements under 12.00 MHz and the median itself at 11.97** (ADR-0128).

**12 MHz is the board's crystal and it does not slide.** So the same-cycle direction is finished, and
what was left was to stop answering in that cycle.

## Decision — answer a cycle late, and only when the answer is not already known

Two arms, and neither is in the fetch loop:

- **`reg_rs1` deep inside a window** — in it, and in neither its first 2 KB block nor its last. A
  12-bit signed offset reaches 2 KB either way, so the effective address is in the block `reg_rs1`
  names or one either side; when all three are inside one window the access is answered whatever the
  immediate is. This reads raw register bits with no adder in front of them, which ADR-0116 measured
  at a **null**. The cycle issues with no region term in it.
- **`reg_rs1` near an edge** — the sum may cross, so the instruction bubbles one cycle. The answer
  about its own effective address is registered on that cycle and read on the next.

**The fast arm is one-sided on purpose.** A miss means "wait for the flip-flop", never "fault". A
window narrower than three blocks — the timer's, the UART's — therefore never reaches it, and
`mcause` stays a function of the access rather than of the base register. That is the whole
difference between this and the neighbourhood test that held 12 MHz at 16 of 16 placements and was
declined in ADR-0116 for exactly that reason.

**Both design commitments survive, which is why this is not an amendment to either.** The trap is
still detected and committed in decode — the instruction is held there, nothing issued, nothing to
un-commit, no flush and no kill. A deferred *answer* is not a deferred *trap*.

## The period is a null

Sixteen placements a side, paired by seed, one tree, one toolchain, one commit. The control is the
shipping text with two lines replaced — `region_stall` and `ls_fault` forced low — so what is
subtracted is the whole region cone including the wait, and not only the fault.

| | worst | median | best | under 12.00 MHz |
|---|---|---|---|---|
| control | 12.03 | 12.625 | 13.14 | 0 of 16 |
| with the fault | **12.23** | **12.660** | 13.03 | **0 of 16** |

**Median of the per-seed period deltas: −1.23%. Five of sixteen seeds slower; two-sided sign test
p = 0.21.** There is no effect to find. The feature tree's WORST placement is better than the
control's, 12.23 against 12.03, which is the clearest way to say that the thin end of this
distribution is what the tree does and not margin the feature spent.

**This table was re-taken, and the first one is why the rule exists.** An earlier sweep of the same
design read 12.04 worst and −0.74% median, and it was measured before the deadlock fix and the
answer-hold correction below landed. `make netlist-diff` against that commit reports the netlist
moved — one flop from `SB_DFFSR` to `SB_DFFESR` and nine named nets — so the numbers described a
design that no longer existed. They are not quoted here. A sweep is owed whenever the digest moves,
however small the move looks.

Read that beside the spelling it replaces, on the same instrument:

| | median Δ period | seeds slower | p | under 12.00 |
|---|---|---|---|---|
| same-cycle, bit 12 (ADR-0128) | +9.40% | 16 of 16 | 3.05e-05 | 9 of 16 |
| deferred (this) | **−1.23%** | 5 of 16 | 0.21 | **0 of 16** |

## The price is cycles, and the prediction held

The two load/store locality counters were built to price this design without building it
(ADR-0084's successors), and they did, to three digits:

| | predicted | measured |
|---|---|---|
| Dhrystone cycles | +13.8% | **+13.79%** — 1 755 208 against 1 542 455 |
| DMIPS/MHz | 0.666 | **0.664**, from 0.758 |
| DMIPS at 12 MHz | 7.99 | **7.97**, from 9.10 |

`REGION` is **212 752** cycles, which is exactly the near-edge counter — one stall for every access
that needed one and none for the other 43.4%. The fast arm is free, measured, and not merely argued.

**The `.S` suite is the pessimal case and says so**: 100.0% of its load/stores sit near an edge
against Dhrystone's 56.6%, because its programs are small and hand-written, so every access waits
and `REGION` is 5.2% of 41 675 cycles. Quote the compiled number. A suite that cannot reach the fast
arm cannot price it.

## Consequences

- **The eighth stall reason.** It BUBBLES — nothing issued, so nothing is lost and the instruction
  re-decodes. Declared in the six places `CLAUDE.md` enumerates, asserted in `rtl/decoder.v`'s
  `FORMAL` block and vectored both ways round in `test/decoder_tb.v`, because the arm is only arm
  order in the publish block and no single-hart program can otherwise tell which answer shipped.
- **F and G did not move**, re-measured by `make -C formal remeasure-fg` with both flip points in
  both directions. A stall reason that adds no retire gap adds no depth, so every derived BMC floor
  stands.
- **It cost `components_traps` its induction, and buying that back moved the harness split.** The
  answer is the first state in this core computed from a decode input on one cycle and read on the
  next, so k-induction may start from a held answer that belongs to no access — valid, refusing, and
  the map answering the address the instruction really uses — and `must_not_trap => !trap_entry`
  then fails on a trace no run reaches. The invariant that rules it out is a statement about two
  decoder registers, and `formal/traps.sv` can reach neither: **yosys resolves no hierarchical
  reference and ignores `bind`, and both are silent** — `decoder.ls_answer_valid` parses as an
  implicitly declared undriven wire whose value the solver picks, which is exactly the shape that
  passes induction and fails the base case. So the assertion is stated in `rtl/decoder.v`, the
  environment fact it stands on is assumed there in the same shape as the standalone fetcher model
  beside it, and `formal/components.sby` reads the decoder `-formal -noassume` for this task: the
  assertions come in, the standalone model does not. That draws the split one statement finer than
  the whole file, which is where it always belonged — it was the fetcher assume alone that made
  `pcloop`'s echo circular. The proof went from 174 seconds of failing to 23 of proving.
- **Two floors were re-derived and no others moved.** `uart.S` and `lrsclock.S` are the only suite
  programs whose retire count is a function of CPI rather than of their instruction sequence, and
  both lines already said so. A change that altered anything but timing would not land on exactly
  the two lines annotated as timing-dependent and leave the other 71 alone.
- **What the counters can price, build against.** This is the second design decided by them rather
  than by a synthesis run, and the first whose predicted CPI came back inside a tenth of a percent.
  The instrument is worth more than the ticket it settled.
- **Not claimed: that a deferred answer is free in general.** It is free *here* because the fast arm
  covers 43.4% of compiled accesses and the slow arm's cycle is spent off the loop. A workload whose
  base registers all sit near edges pays the suite's price, not Dhrystone's.
