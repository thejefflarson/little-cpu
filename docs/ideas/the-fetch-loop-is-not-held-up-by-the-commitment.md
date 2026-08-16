# The fetch loop is not held up by the commitment

**A design spike on amending the no-wrong-path-state commitment. The conclusion is: do not
amend it. Take the margin from the clock model instead.**

## Why this was asked

Every lever that does not touch the commitment has been measured and declined: the decode
head ([ADR-0076](../adr/0076-the-decode-head-is-a-plateau-not-a-lever.md), 3.3% and inside
the churn band), the instruction memory at either end of the fetch loop
([ADR-0087](../adr/0087-the-instruction-memory-does-not-come-out-of-the-fetch-loop.md),
+0.4% at its single best cell and negative on the board), synthesis flags, and eleven
behaviour-preserving spelling changes worth −239 cells that moved the up5k median 1.6% the
wrong way.

The path attribution says decode is 11 of 23 levels and is the only part nothing has
touched, because what is in it is not a spelling — it is six stall reasons and a trap
resolving before the fetch address can move. That is a legitimate reason to *price* the
commitment. It is not by itself a reason to amend it.

## What the spike found

**The 21–28% is stale.** [ADR-0078](../adr/0078-a-one-deep-kill-is-cheap-and-buys-a-clock-the-board-cannot-use.md)
states its tree: `rtl/` and `formal/` byte-identical to `2d79878`. `rtl/timer.v` does not
exist at that commit, and `rtl/` has changed by +328/−53 lines since. Its ceiling was
measured on a core with no machine timer, no interrupt arm in the trap priority, and no
`mip`/`mie`. ADR-0087 corroborates from the current tree: its `both` variant, with the
fetch loop registered at both ends, reports its residual critical path as
`mtip` → `minstret`, 25 LUT levels, 85.45 ns at the worst placement. That path did not
exist when 61.27 ns was measured.

**The lowest usable clock is 15.938 MHz.** `icepll` on a 12 MHz reference rejects 13.5 and
11.5 outright — the output range starts at 15.938 MHz, with roughly 0.4% granularity above
it. So the family's function-breaking ceiling, 62.38 ns, clears the 62.74 ns that the
lowest selectable step needs by **0.6%** — before implementation loss, before the timer,
and before any margin policy.

**The free member buys nothing.** Moving `stall` off the fetch address's data path and onto
the enable pins of the pc register and the ROM's address register is functionally
bit-identical to what ships: no state, no buffer, no discard, no amendment. Its ceiling is
already in the tree as ADR-0076's row E, at **−1.5%**. The version that costs no commitment
at all returns nothing, which is what says the plateau is structural rather than a
consequence of the commitment.

**A real member cannot reach the ceiling.** The ceiling deletes the redirect terms; a real
design relocates them. As long as a redirect computed in cycle N must reach the fetch
address register by the end of cycle N, the path `instruction word → decode → redirect
target → fetch address register` exists and is about 18 levels before any buffer penalty.
ADR-0087 measured that shape at 25 levels against a base of 22–23. Only a two-cycle
redirect gets under it, and that is a two-deep kill, which nothing here measures.

## The decision

**The no-wrong-path-state commitment is not amended.**

Against the four goals: *fast* moves by +2.2% of product for the only member anyone has
built, and is unmeasured-but-unpromising for the best design in the family. *Simple* and
*readable* move backwards by ADR-0078's own measurement — +105/−36 in the decoder, four new
concepts, a seventh stall bucket. *Formally verified* is affordable but not free. Three of
four go the wrong way, which is not the bar this project sets for changing a commitment.

## Take the margin from the clock model instead

Both blocked improvements are blocked on Fmax margin and not on merit:
[ADR-0083](../adr/0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md)'s
narrow forwarding is net +4.5% holding 12 MHz on 0.48% of margin, and the cheaper
operand-fetch spelling is net +9% on 0.83%.

[ADR-0066](../adr/0066-twelve-megahertz-is-a-requirement.md)'s stated premise — that the
next step down is 6 MHz — is measurably false. One `SB_PLL40` fed by the crystal gives
about 0.4% granularity from 15.938 MHz up, and one fabric divide-by-two extends that
downward from about 7.97 MHz. Clocking the operand-fetch variant at roughly 11.7 MHz
restores today's margin and collects its cycle win: **+6.4% of product, three times what
the amendment's built member returns, for one PLL instance — no proof change, no F and G
re-measurement, and no commitment amendment.**

That is a different ADR from this one, and it is the piece with a real payoff.

## Two corrections to merged decisions

**ADR-0087's fetch-buffer ruling is corrected.** It ruled a fetch buffer discarded before
its valid bit is set to be forbidden by `pc == $past(next_pc)`. That is true only of a
design that lets the pc run ahead of decode. Under a design where the pc stays
architectural and buffer-empty is a stall reason, `next_pc = stall ? pc : …` holds the pc
while the buffer refills and the assertion holds verbatim. The amendment would therefore be
*additive* — new clauses about the buffer — rather than a weakening of the clause that is
actually enforced.

**ADR-0047's divergence does not transfer.** That was unbounded sequential equivalence
between two whole netlists matched by name, on a gate build whose names `opt` had already
destroyed: 495 unproven `$equiv` cells and no invariant strengthening. The generated checks
are bounded, over one netlist, with no matching problem. The real exposure is the two
`mode prove` component proofs — and ADR-0078 measured `components_pcloop` still closing by
k-induction under a kill, with no hand-written invariant, and getting simpler.

## If the direction is ever reopened

**Build the decoupled fetch stream, not the kill.** ADR-0078's proof invoice — two
proof-only decoder ports, one property split across two hops, `components_traps` red at
step 3 — is specific to registering the pc itself. Keep the pc architectural and
`components_traps`' `prev_trap_entry → pc == prev_mtvec` survives unchanged. That assertion
is the only thing in the tree that says a trap lands on `mtvec` and saves the right state,
because riscv-formal ships no spec model for SYSTEM or MISC-MEM at the pin.

**Buffer-empty is a stall reason, not a kill.** It becomes the seventh, declared in
`rtl/decoder.v`, `test/decoder_tb.v` and `CLAUDE.md`, and charged in `make cycles`. This
closes by construction the blind spot ADR-0078 recorded, where 224 cycles were silently
counted as issues because `kill` was not a stall reason: there is never a cycle that issues
nothing without raising `stall`.

**The replacement invariant keeps the strong clause and adds three.** Nothing issues from
an empty buffer; a held word's pc is the pc decode is deciding about; and after a redirect
the only word that may still be held is the one at the target. Together with
`pc == $past(next_pc)`, which survives, the issued stream is exactly the chain `next_pc`
names — the buffer contributes only entries that never reach writeback.

**The reopening bar is 62.7 ns at the worst of four placements**, which is the lowest
frequency `SB_PLL40` produces directly from the crystal. The first experiment is not a
design: re-measure the family ceiling on the current tree, function-breaking, four
placements, and attribute the residual with `soc/depth/path_stages.py`. If the residual is
the timer path, `mtip` is already registered and breaking that path costs one cycle of
interrupt latency against a measured worst case of 33 cycles — an enabler, not a
substitute, because today the fetch loop is critical.

## Risks in this brief

The stale-ceiling claim is an inference across two trees: it rests on ADR-0087's `both` row
naming a timer path, and `both` is not the function-breaking ceiling. Four placements settle
it, and that is the one experiment gating the whole direction.

Occupancy may be the binding constraint rather than levels. The −239-cell change moved the
up5k median 1.6% *worse*, and about 55 endpoints sit within 3.6 ns of critical with about
290 within 7.3 ns. Every member of this family adds cells — the one built member added 124 —
so a design that buys levels and spends area can return slower than its level count
predicts. That is why the built member captured 66% of its ceiling rather than 97%.

A two-deep kill is genuinely open and unmeasured. It is the only shape that gets the
redirect path under 18 levels. It moves F and G, needs a kill network over two slots, and
costs roughly 15% of Dhrystone cycles. It is a separate decision and this brief does not
license it.

## Dead ends, recorded so they are not rediscovered

- `stall` off the fetch address by any spelling, including clock enables: bounded at −1.5%.
- The one-deep kill as built: dominated on every axis by the decoupled fetch stream,
  including the proof invoice.
- Changing parts: the same RTL is 2.4× faster on hx8k, and none of the deferred work is
  unblocked by an FPGA nobody has.
