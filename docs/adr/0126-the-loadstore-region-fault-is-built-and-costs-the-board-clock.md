# 0126 — The load/store region fault is built, and it costs the board clock

Status: accepted (the measurement); the design question it opens is not closed

## Context

Causes 5 and 7 were raised for the eleven A encodings and for nothing else. A plain load or store to
an address no memory answers read zero or was dropped, silently — the last conformance gap against
the privileged spec's strong recommendation that precise access faults be raised, and a recorded
deviation rather than a design choice.

Two earlier measurements priced it and declined it. ADR-0104 measured a decode-time test that waits
on the top of `immediate + reg_rs1` at four logic levels in the fetch loop and 10.57–11.00 MHz over
four seeds. ADR-0116 decomposed that: the cost is monotone in which bit of the sum the fetch loop
waits for — bit 31 is +17.30% of median period, bit 12 is +9.10%, and waiting for none of it is a
null. Six spellings have been priced across three sessions.

This one is the cheapest exact spelling, and it waits for bit 12. Every immediate that can reach the
test is sign-extended above bit 11, so `sum[31:12] = rs1[31:12] + adj` with `adj` in {−1, 0, +1}.
Each window is tested three ways on raw register bits, the immediate's sign picks the pair early,
and the carry into bit 12 — recovered from the existing address adder as
`sum[12] ^ rs1[12] ^ imm[12]`, never a second adder — picks between two precomputed answers one mux
deep.

It was measured once before on an older tree at **12.04 MHz worst of sixteen**, 0.27 ns over the
requirement. That margin is narrower than this part's measured placement spread, so it was recorded
as "it clears" and explicitly not as "it reliably clears". This is the re-take that number asked
for.

## Decision

**Record the price. The design as measured here does not meet the board clock.**

Sixteen placements a side, paired by seed, one tree, one toolchain. The control is the shipping text
with one line replaced — `assign ls_fault = 1'b0;` — so what is subtracted is the whole region cone
and nothing else.

| | worst | median | best | under 12.00 MHz | packed `ICESTORM_LC` |
|---|---|---|---|---|---|
| control | 12.63 | 12.93 | 13.36 | 0 of 16 | 4692 |
| with the fault | 11.52 | 11.97 | 12.28 | **9 of 16** | 4787 |

**Median of the per-seed deltas: +9.40% of period. Sixteen of sixteen seeds slower; two-sided sign
test p = 3.05e-05. +95 cells.**

Read against this part's own bands — ~3.6% edit churn, 4–9% placement spread — a delta this size at
this sign consistency is not churn, and the sign test says so without leaning on either band. It
also reproduces ADR-0116's bit-12 figure of +9.10% almost exactly, on a different tree, which is the
strongest thing in the table: the price did not move when the tree did, so "the tree got faster
underneath it" is not what the earlier 12.04 was.

**The median is under the requirement.** Not the tail — the median. `SOC_MIN_MHZ` is the board's
12 MHz crystal, whose next divider step down is 6, and it does not slide.

## Consequences

**A single-seed gate does not see this.** `make soc-timing` at the default seed reads 12.06 MHz and
passes, which is how a change that misses at nine placements of sixteen arrives looking green. That
is the argument for the paired sweep, made again: a go/no-go on this part is twelve to sixteen
seeds, and one placement is a look.

**What is built is complete and is not the doubtful part.** The eleven A encodings and the twelve
plain load and store encodings now raise the same two causes; `formal/traps.sv`'s region arm is
must-trap; the reference model's memory map is this core's map, so `loadfault.S` and `storefault.S`
co-simulate without a baseline entry between them; and the generated riscv-formal checks grade the
refusal's reported address and write strobe against the spec model's own effective address and mask,
which is an oracle the atomic half never had. None of that is in question. The period is.

**Three ways out, none of them measured here.** Raise the clock (the `SB_HFOSC` step above 12 is 24,
and 41.67 ns is not within reach of a +9.4% edit); find the 9.4% elsewhere in the fetch loop, which
ADR-0076 bounds at 21% for deleting every input to `next_pc` and the no-wrong-path-state commitment
forbids collecting; or accept a deferred answer, which the same three sessions priced at 5–17% of
cycles. The spelling is not the lever — six of them have been tried and this is the cheapest exact
one.

**What must not happen is the floor moving.** Lowering `SOC_MIN_MHZ` is a decision to stop targeting
the board clock and needs an ADR of its own, arguing for 6 MHz.
