# ADR-0078: A one-deep kill is cheap, and it buys a clock the board cannot use

**Status:** Accepted · 2026-08-08 · *A measured null, and the first attempt to price the
no-wrong-path-state commitment rather than restate it. Reads
[ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md)'s 21% as its opening bid and prices
what collecting it would cost. Graded against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement and against 24 MHz, the
next clock the board can actually run. CPI baseline from
[ADR-0070](0070-the-suites-cycles-are-charged-to-a-named-stall-reason.md); depth derivation from
[ADR-0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md).*

## What was proposed

`CLAUDE.md`'s design commitments say they can and should change when a change moves all four goals
forward together, and that none is held harder than the others. **No wrong-path state** is a
commitment like any other, and ADR-0076 ended by naming the price of the only remaining lever:

> Reaching [the 21%] means the pc no longer depends on this cycle's decode, and every way of
> arranging that is state a later cycle has to un-commit.

So: register the fetch address, let the sequential successor of a redirecting instruction be
fetched, and kill it. One valid bit cleared, one cycle deep, no predictor, no skid buffer, no
speculative pipeline. Call it **K**.

Nothing else in the design changes. The pc still moves once per redirect and the target is still
computed in decode; what moves is *when the fetch address sees it* — a cycle later, out of a
register, instead of combinationally in the same cycle.

## Decision

**No.** K was built, run and measured. It is cheaper than expected on almost every axis the
commitment was written to protect — the suite passes, the ladder passes at unchanged depths,
`pcloop` still closes by k-induction, and a trap does not become revocable. One proof goes red:
`components_traps` can no longer say a trap lands on `mtvec` in one cycle, and re-establishing it
costs two proof-only decoder ports. K is declined anyway, on a different axis than either of those.

**K measures 13.32 MHz. Its own function-breaking ceiling is 13.66 MHz. The ceiling of the entire
family — including the skid buffer that would also take `stall` off the fetch address — is
16.32 MHz.** The part's oscillator divides 48 by 1, 2, 4 or 8, so the clock above 12 is 24, which
needs 41.67 ns. The best ceiling in this direction is 61.27 ns: **47% longer than 24 MHz needs, and
there is nothing in between to land on.** Every variant here buys a frequency the board cannot
select, at the price of a commitment.

The spike RTL is discarded. What ships is this ADR.

## Method

Four designs, one tree, one toolchain: **Homebrew Yosys 0.68+post (`c12172fb`)**,
nextpnr-0.10-108-g68c1acd8, `icetime` from the same install. `rtl/` and `formal/` are byte-identical
to `2d79878`, which is what ADR-0076 measured, so its eight baseline placements are reused as the
comparison distribution; four of them were re-run here and reproduced to the hundredth of a
nanosecond (77.38 · 78.34 · 78.51 · 78.55). **The tables report periods, so a negative `vs baseline`
is faster.**

Two of the four are ADR-0076-style ceilings — function-breaking RTL that deletes the logic a
candidate hopes to shorten, so that no implementation of it can do better:

- **K∞** deletes every redirect term from `next_pc`, leaving `reset ? 0 : stall ? pc : pc + pc_inc`.
  That is exactly what K takes off the fetch-address path and no more: `stall` stays, because
  holding the fetch address on a stall is what re-presents the instruction, and the only alternative
  is a buffer.
- **K∞E** deletes `stall` as well, leaving `pc + pc_inc` unconditionally. That is the ceiling of K
  *plus* a skid buffer — the change anybody who read K's result would propose next.

**K itself is a working core**: 59/59 on `make test` with identical retire counts, 59/59 AGREE on
`make cosim-suite`, 85/85 on the generated ladder.

## What it costs in cycles

Measured first, because it was the cheapest of the five questions and it was expected to be the
expensive one. It is not.

| | cycles | retires | CPI |
|---|---|---|---|
| baseline | 28632 | 13853 | 2.0668 |
| **K** | **29175** | 13853 | **2.1060** |

**+1.9%.** The suite redirects on 767 issuing cycles — 576 taken branches, 135 `jal`, 22 traps,
22 `mret`, 12 `jalr` — which is 5.5% of retires and 2.7% of cycles. The kill costs 543 of those
767; the other 224 land on a cycle the pipeline was going to spend anyway.

**The operand-fetch cycle does not hide the kill, and that was the open question.** Classifying the
cycle after each of 708 redirects on the *unmodified* core:

| the cycle after a redirect | count | share |
|---|---|---|
| issued | 447 | 63.1% |
| operand-fetch bubble | 249 | 35.2% |
| decode scoreboard | 7 | 1.0% |
| serialization | 5 | 0.7% |
| divider / accessor / stolen fetch | 0 | 0.0% |

The 35.2% looks like free cover and is not. `operand_stall` is decode *presenting* the target's
register pair, and it cannot start until the target's word arrives — which under K is a cycle later
than today. The bubble still happens; it just happens one cycle further out. Only the 12 cycles in
the bottom two rows are stalls that resolve with time and could absorb a kill.

The 224 that were absorbed came from somewhere else: a kill cycle does not care that the fetch
window was stolen, so a `fetch_stall` landing on it is free, and a redirect target whose register
pair happens to match the wrong-path word's issues without an operand-fetch cycle.

Per instruction, the fastest are hit hardest, as expected: `jal` reads no register, so it issues in
one cycle today and two under K — **+100% on 135 of 13853 retires.** `lui` and `auipc` are the other
one-cycle instructions and are untouched; neither redirects.

**One finding here outlives the decision.** `make cycles` mis-reports K. `kill` is not a `stall`
reason, so on the 224 kill cycles where no stall reason happened to be true, the runner counted an
**issue** cycle — for a cycle on which nothing issued. Every graded identity still holds: `stall` is
still exactly the OR of six reasons, the columns still sum, `unattributed` is still zero. The
accounting stays green while its largest column stops meaning what it says. A seventh bucket would
not be enough; the taxonomy would need a second *category*, because "issued nothing" would stop
being the same thing as "stalled".

## What it buys

| variant | seeds | ns, sorted | median | MHz | vs baseline | `fit` LC |
|---|---|---|---|---|---|---|
| **baseline** | 8 | 77.38 · 78.34 · 78.51 · 78.55 · 78.61 · 78.80 · 79.20 · 80.07 | 78.58 | 12.73 | — | 3880 |
| **K** — implemented | 8 | 73.85 · 73.99 · 74.24 · 74.85 · 75.28 · 76.06 · 76.38 · 77.43 | 75.07 | 13.32 | **−4.5%** | 4004 |
| K∞ — its ceiling | 8 | 70.11 · 71.79 · 73.01 · 73.20 · 73.26 · 73.30 · 75.37 · 77.14 | 73.23 | 13.66 | −6.8% | 3656 |
| K∞E — ceiling with `stall` off too | 4 | 60.77 · 60.97 · 61.57 · 62.38 | 61.27 | 16.32 | −22.0% | — |

Read the last two rows against 41.67 ns, which is 24 MHz.

**K is real but thin.** −4.5% clears the 3.6% edit-churn band by less than a point, and the two
eight-seed distributions overlap: K's worst placement (77.43) is slower than the baseline's best
(77.38). It does improve the margin over the board clock — worst placement 12.91 MHz against the
baseline's 12.49 — and it costs 124 cells, leaving 96 under `FIT_MAX_LC`.

**The critical path does not move.** Under K it is still `imem.in_range → imem.rom_even`, 22 logic
levels against the baseline's 23, 35% logic and 65% routing. Taking the redirect terms out of the
fetch loop does not take the fetch loop off the critical path, because `stall` and `pc_inc` are
still in it — and `stall` is fed by `hazard`, which needs `rs1`/`rs2`, which needs the width mask
and the compressed register muxes. **The whole decode head is still inside the fetch loop after K,
which is why K collects a third of what K∞E does.**

K∞E is where the path finally leaves: `por_done → riscv.csrs.mstatus_mie`, 16 levels, the same
next-path-underneath ADR-0076's row E found. That is the floor of this direction, and it is
16.32 MHz.

**ADR-0076's 21% is confirmed and re-attributed.** Its `−21.3%` row deleted the whole decode head
(A+B+C+D) *and* every `next_pc` term, and landed at 61.86 ns. K∞E deletes no decode-head logic at
all and lands at 61.27 ns. **The 21% is entirely the fetch loop; the decode head contributes
nothing once the pc is off decode.** ADR-0076 read that structure correctly and had one term too
many in the row that showed it.

## What it costs the proofs

Less than expected, and in a different place.

- **F and G do not move.** `hang` is red at 6 and PASS at 7 — F = 6. `liveness` is red at 5 and PASS
  at 6 at trig 10 — G = 6. Both reproduce the baseline's flip points exactly, so **no `[depth]` line
  changes**, and `insn 19` and `reg 15 22` keep the one cycle of margin they had. A kill lengthens
  the gap after a redirect, but the worst-case gap was never set by a redirect: it is set by a load
  turnaround and a scoreboard chain that a redirect cannot stack onto.
- **The ladder is 85/85** at those unchanged depths, both set equalities in both directions.
- **`components_pcloop` still closes by k-induction, with no hand-written invariant** — the one
  outcome this was most expected to break. Induction does not need help because `redirect_pending`
  is real state the assertion can read: the increment guard stops being a transcribed list of
  jump/branch/trap/`mret` encodings and becomes one bit. That guard genuinely gets simpler.
- **`components_traps` goes red, and it is the one that matters.** `assert(pc == prev_mtvec)` fails
  at step 3. That assertion is, by `CLAUDE.md`'s own account, the only thing in the tree that says a
  trap lands on `mtvec` and saves the right state — riscv-formal ships no spec model for SYSTEM at
  the pin, its instruction check drops every value comparison once an instruction traps, and its two
  pc checks accept whatever target the core reports.
- Re-establishing it costs **two proof-only output ports on the decoder** (`redirect_pending` and
  `redirect_target`) and splits one property into two hops: the registered target is `mtvec`, and
  the pc takes the registered target. It closes by k-induction after that. `formal/pcloop.sv` cannot
  state the landing at all any more and keeps only "the redirect was taken".
- `components_decoder`, `components_executor`, `nonperturbation`, `dmemcheck` and `imemcheck` PASS.
  The standing `reg_ch0` liveness probe still fires: delete the rs2 write-through bypass and
  `reg_ch0` is SAT at k = 22.
- `complete` and `complete_cover` were not run. They select `abc bmc3`, and the Homebrew Yosys used
  here rejects `abc -g AND -fast`; the same failure reproduces on the unmodified tree, so it is the
  toolchain and not the change. They need the pinned OSS CAD Suite.
- `make cosim-suite`: **59/59 AGREE**, matching `test/COSIM_EXPECTED_FAIL`.

## What it costs trap commit

The coupling between "all traps are detected and committed in decode" and a kill is real, and it
resolves the opposite way round from the way it was posed. **A trap does not become revocable.**

The killed word is the sequential successor of the instruction that redirected. It is fetched and
decoded, but the kill is known before it can issue — the redirect was registered on the edge that
latched it — so it never issues. Everything decode can do to architectural state hangs off one
term, and adding `&& !kill` to `issuing` suppresses `trap_entry`, `mret_entry`, `instret`,
`csr_ren`, `csr_wen` and `out.valid` at once. The wrong-path word decodes like any other and raises
`trap_pending` and `csr_write_op` on its own; one assertion says none of that reaches anything:

    always_comb if (kill) assert(!trap_entry && !mret_entry && !instret && !csr_wen && !csr_ren);

There is no reorder buffer to add and no commit to undo. That is one AND term, one assertion and two
bench vectors.

**What breaks is the proof that the commit lands, not the commit.** Today the trap target is a
one-cycle statement: `trap_entry` last cycle means `pc == mtvec` now. Under K the landing is one
cycle later *and a freeze can hold it there indefinitely*, so the statement has to be carried across
an unbounded gap or split. That is the `components_traps` failure above, and it is the whole of what
this commitment costs.

## What it costs to read

Worse, and not close. The spike is `rtl/decoder.v` +105/−36, `formal/pcloop.sv` +23/−8,
`formal/traps.sv` +23/−4, `test/decoder_tb.v` +12/−1.

Nothing is removed. The `next_pc` priority chain does not shrink — it moves into `redirect_pc` and
acquires a register pair, a `hold_stall` term and a second priority chain on top of it. Four
concepts arrive that a reader currently never meets: a wrong-path word; a kill that is *not* a stall
and so is missing from the six-reason taxonomy; a `redirect_pending` that outranks every bubbling
stall but not a freeze; and a decoder output whose only consumers are proofs. The RVFI retire also
stops being able to report `next_pc` and needs a second, architectural next-pc expression beside it.

One thing does get simpler, and it is worth recording because it is the only place the commitment
was paying for itself: the increment guard in `formal/pcloop.sv` and in `rtl/decoder.v`'s `FORMAL`
block stops being a hand-transcribed list of jump, branch, trap and `mret` encodings — the list two
of those files warn about keeping in step — and becomes one state bit. If this direction is ever
reopened, that is the part to keep.

## Consequences

- **The no-wrong-path-state commitment is not amended, and it is now priced rather than assumed.**
  The bar for amending it is all four goals moving together. Two of them are affordable here —
  1.9% of CPI, and a formal cost of one re-stated property — readability is worse, and the goal it
  was supposed to serve does not move to anywhere useful.
- **The blocker is the clock, not the design.** 13.32 MHz is not a step this part has. Between the
  board's 12 and the oscillator's 24 there is nothing, so a change that lands at 13, or at 16, buys
  a number and not a product. **Any future candidate in this space has to show 41.67 ns, and the
  ceiling of the whole space is 61.27 ns.** That single sentence retires the direction, not just
  this spelling of it.
- **The dead-end list grows by three, and two are dead at their ceilings.**
  1. A registered fetch address with a one-deep kill — **built and measured: −4.5% of period,
     +1.9% of CPI, +124 cells, 13.32 MHz.**
  2. Every redirect term off `next_pc` by any means — **−6.8% at its ceiling, 13.66 MHz.**
  3. That plus a skid buffer taking `stall` off the fetch address — **−22.0% at its ceiling,
     16.32 MHz.**
- **ADR-0076's plateau is one term narrower than it was written.** The 21% is the fetch loop alone;
  the decode head contributes nothing to it. Its five dead ends stand, and its rule — bring a
  ceiling for the whole cone or bring nothing — is what made this ADR cheap to write.
- **`make cycles` has a blind spot that only a kill would expose.** A cycle that issues nothing
  without raising `stall` is counted as an issue cycle, and every grader stays green. Nothing in the
  tree can produce one today, and this is written down so that the next change which can does not
  discover it by shipping a CPI number that is 224 cycles optimistic.
- **Two goals that were expected to conflict did not.** The formal harness absorbed a kill almost
  without complaint — one property, re-stated. The reason is worth keeping: the kill lands *before*
  issue, so it adds no state that a later cycle un-commits in the sense the commitment is about. A
  future proposal that kills *after* issue is a different animal and this measurement says nothing
  about it.
- **Nothing in `rtl/`, `formal/` or `test/` ships from this ADR.** `make test` (59/59),
  `make test-units`, `make cycles`, `make -C formal check` (85/85), the four component proofs,
  `nonperturbation`, `dmemcheck`, `imemcheck` and `make cosim-suite` (59/59) were all run on the
  spike and again on the reverted tree, which is byte-identical to `0b1728a` under `rtl/`,
  `formal/` and `test/`.
