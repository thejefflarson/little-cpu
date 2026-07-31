# ADR-0025: The ladder's depths are derived from the pipeline, not inherited from the serialized core

**Status:** Accepted · 2026-07-29 · *Supplements ADR-0004, ADR-0009, ADR-0010, ADR-0012, ADR-0015, ADR-0023*

## Context

Every check `formal/genchecks-local.py` generates is `mode bmc`: a PASS means "no counterexample
exists within the check's configured depth," not "the property holds." That is not a flaw — a
pipeline has bounded state, so checking far enough past its own statefulness is genuinely complete
for these properties, not merely a heuristic. But "far enough" is a claim about *this* pipeline, and
nobody had computed it. `formal/checks.cfg`'s depths (`insn 15`, `reg 15 20`, `pc_fwd`/`pc_bwd`
`10 30`, `causal 10 20`, `liveness`/`unique 1 10 30`, `csrw 30`) came from the wave-0 harness
(`86e2721`, porting the *serialized* FSM core's era numbers forward, per ADR-0006), unexamined since.
That core had no decode scoreboard, no accessor load-response stall, and no 32-iteration divider —
three structures the current pipeline added. If today's worst-case instruction lifetime exceeds a
depth carried over from a different core, a check can PASS only because the counterexample lies past
the horizon it's allowed to look — precisely the danger ADR-0023 names in the abstract for `reg`
before ADR-0024 gave it an engine that could reach its own configured depth at all.

This ADR derives the depths analytically, checks the derivation empirically by running the full
ladder at raised depths, and records both the numbers and the argument next to them in
`formal/checks.cfg` so a future reader can check the claim instead of trusting it.

## The pipeline's own bound

`littlecpu` has three *registered* stages between issue and retire — `decoder_out`, `executor_out`,
`accessor_out` — with writeback combinational off `accessor_out` (`rtl/writeback.v`'s `wen`/`waddr`/
`wdata`/`rvfi_*` are `always_comb`, not `always_ff`). So an instruction that issues into `decoder_out`
during cycle *t*, with no stall anywhere, has `executor_out` valid at *t+1* and retires (`accessor_out`
valid, `rvfi_valid` combinationally high) at *t+2*. Call this the **base latency: 3 cycles**
(`decoder_out` → `executor_out` → `accessor_out`/retire).

`formal/genchecks-local.py`'s `check_insn` hardcodes `` `RISCV_FORMAL_RESET_CYCLES 1``, so the DUT's
own reset (`littlecpu.reset`) is high for exactly cycle 0; the earliest any instruction can even reach
`decoder_out` is cycle 1. So the **unstalled floor is 4** (reset cycle + base latency): the very
first instruction in a trace retires at cycle 4 at the earliest — the check itself must be able to
reach at least this cycle for its `assume(spec_valid)` (`rvfi_insn_check.sv`) to be satisfiable by
*any* trace, let alone an interesting one.

### The three stall sources, and what they cost a single dependency

Each is already named and bounded in an existing ADR; this section is where those bounds are turned
into cycle counts against the actual RTL (`rtl/decoder.v`'s `live_producer`, `rtl/accessor.v`'s
`stalled`/`pending_valid`, `rtl/executor.v`'s `state`/`divide` arm), not re-derived from scratch.

1. **The decode scoreboard (ADR-0004, `rtl/decoder.v:362-370`)** tracks a live producer in exactly
   two places: `decoder_out` (`out.valid && out.rd == r`) and `executor_out`
   (`executor_out.valid && executor_out.rd == r`). A dependent instruction bubbles for as long as
   either holds the producer — **at most 2 cycles**, matching ADR-0004's own "worst-case 2-cycle RAW
   bubble." `test/asm/hazard.S`'s back-to-back RAW cases are the zero-separation instance of this.

2. **The accessor's load-response turnaround (ADR-0015)** adds a third producer slot,
   `accessor_pending_valid`/`accessor_pending_rd`, that exists for exactly the one cycle a load's
   answer is in flight after its request leaves `decoder_out`/`executor_out`. Traced directly against
   `rtl/accessor.v`: a load P issuing into `decoder_out` at cycle *t* requests memory at cycle *t+1*
   (`stalled = in_valid && is_load`), and its real data reaches `accessor_out` at *t+3* — one cycle
   later than a non-load's *t+2* (matches ADR-0015's "loads cost one extra cycle"). Because
   `accessor_pending` is a third tracked slot, a dependent instruction can be held by all three in
   sequence. Hand-tracing `lw x1, 0(x2); add x3, x1, x1` against the actual RTL semantics (decode
   bubbles on the scoreboard hit against `decoder_out`, then `executor_out`, then holds — not
   bubbles — against `accessor_pending`, per invariant 8(a)/ADR-0015 point 4) puts the dependent's
   successful decode exactly **3 cycles later than its unstalled slot**, and its retire lands at
   absolute cycle 8 (4 unstalled + 2 scoreboard + 1 accessor + 1 for the ADR-0015 load-latency
   already paid by the producer itself, which is where the extra cycle beyond the naive 2+1=3 sum
   comes from) for a trace starting right after reset. This is not a bound pulled from the ADR text
   alone; it is the same number the RTL's own mechanics produce when traced cycle-by-cycle.

3. **The divider (ADR-0009, ADR-0010, ADR-0012)** holds `executor_out.valid` at 0 for the entire time
   `state == divide`. Under the ladder's `` `RISCV_FORMAL_ALTOPS `` (`formal/checks.cfg`'s
   `[defines]`), `rtl/executor.v`'s `divide` arm completes unconditionally on its first cycle — no
   `mul_div_counter` check — so `state == divide` lasts **exactly 1 cycle** under ALTOPS, against
   **32 cycles** (33 from issue to `executor_out.valid`, ADR-0010's own count) for the real restoring
   divider. Tracing a divide-then-dependent pair (`div ...; add x3, x1, x1` where the `add` depends on
   the divide's `rd`) the same way as the load case lands on the identical absolute retire cycle, 8 —
   the ALTOPS-collapsed divide costs a dependent instruction the same class of delay as a load,
   because both add exactly one extra producer-visibility cycle beyond the plain 2-cycle scoreboard
   bound.

**Single-hop floor (one producer, one dependent, ALTOPS in force): 8 absolute cycles.** This is the
minimum `` `RISCV_FORMAL_CHECK_CYCLE `` at which even the simplest hazard-exercising trace — the exact
shape ADR-0004 and ADR-0015 were written to fix — is reachable at all. Below it, an `insn_*` check's
`assume(spec_valid)` can never be satisfied by a trace containing that shape, and the check is
vacuously true of it: a PASS that says nothing.

### Why one hop of margin, not an unbounded chain

A checked instruction can in principle sit at the end of an arbitrarily long true-dependency chain
(`I0→I1→I2→…→I_checked`), and hand-tracing a two-hop all-load chain
(`lw x1,0(x0); lw x2,0(x1); add x3,x2,x2`) puts the third instruction's retire at absolute cycle 12,
not 8 — each additional hop costs roughly 4 cycles (the 3-cycle single-hop delay, plus one cycle
because the producer's own retire is itself pushed back by the same mechanism it's paying into). A
depth chosen only from the single-hop floor (8) would make a two-hop trace of this shape unreachable.

But a longer chain does not exercise anything the scoreboard's own state doesn't already exhibit at
one hop: `live_producer` (§ above) is a function of exactly three fixed slots
(`decoder_out`, `executor_out`, `accessor_pending`), memoryless beyond them — ADR-0015 says as much
("not a general widening of the scoreboard to a third pipeline stage"). A third or fourth hop
relocates the identical single-hop interaction later in program order; it does not present the
scoreboard, the accessor, or the divider with any state a one-hop or two-hop trace didn't already
reach. And the property under test (`rvfi_insn_check.sv`'s assertions) is entirely about the checked
instruction's own retiring values, not about how many instructions preceded it. So depth beyond
"floor plus one extra hop of margin, to also cover a stall interacting with a second, adjacent stall"
buys negligible additional verification power for `insn_*`/`csrw_*` specifically, even though BMC
completeness in the abstract would eventually want to explore deeper traces too.

**Two-hop floor: 12 absolute cycles** (4 unstalled + 2×4 for two stacked worst-case hops). This is
the number this ADR treats as "the derived minimum a reader should expect the depths to clear."

### The retire-gap checks (`reg`, `pc_fwd`, `pc_bwd`, `causal`, `liveness`, `unique`)

These consistency checks (`formal/genchecks-local.py`'s `check_cons`) don't bound a single
instruction's lifetime — they bound the **maximum number of consecutive cycles with no retiring
instruction** (a "retire gap"), because each watches the `rvfi_valid`/`rvfi_order` stream over a
window (`depth − start`) rather than one instruction's own path through the pipeline. Tracing the
same load-dependency pair above against the retire stream (not the decode-attempt stream): the
producer retires at cycle 5, the dependent at cycle 8 — a **2-cycle retire gap** (cycles 6–7), not the
3-cycle decode-delay computed above, because the producer's own retire absorbs one of the delay
cycles rather than adding a new gap. The ALTOPS-divide case produces the identical 2-cycle gap. So
the retire-gap bound is smaller than the instruction-lifetime bound, and every `check_cons` window
already inherited from the wave-0 harness (`reg`: 5; `pc_fwd`/`pc_bwd`: 20; `causal`: 10;
`liveness`/`unique`: 20) clears the derived 2-cycle (single-hop) / ~4-cycle (two-hop margin) gap with
substantial room, independent of the `insn_*` question above.

## ALTOPS scope, stated explicitly

Every number above is computed **under `` `RISCV_FORMAL_ALTOPS ``**, because that is how
`formal/checks.cfg` runs every check (`[defines]`) and this ADR is about the depths *as configured*,
not a hypothetical unconfigured ladder. Recomputing without it: the real divider holds
`executor_out.valid` at 0 for 32 cycles (not 1), so a single-hop floor involving a real divide becomes
`1(reset) + 3(base) + 2(scoreboard) + 32(divider)` ≈ **38 cycles**, and a two-hop chain that stacks a
real divide against a second hazard would need on the order of 70+. **None of the depths this ADR
sets are sufficient for a ladder that ever drops `RISCV_FORMAL_ALTOPS`.** That is not a gap in this
derivation — ADR-0010 already scopes ALTOPS-checked `insn_mul`/`insn_div`/etc. as saying nothing about
real arithmetic, and the real divider's operand-routing defect (the one ALTOPS-scoped ladder failure
this repo already knows about, `formal/EXPECTED_FAIL`'s div/rem history) has zero formal coverage in
any form today (ADR-0010). If ALTOPS is ever dropped from a future ladder run (ADR-0010's own
"if the BMC proves tractable at full depth, promote it later"), the depths in `formal/checks.cfg` must
be re-derived from the 32-cycle divider, not reused from this ADR — a distinct, much more expensive
exercise this ticket does not attempt.

## Empirical check

Analytical derivation is only as good as the hand-tracing behind it, so the actual test is running the
ladder at depths well past the derived floor and watching for a PASS→FAIL flip — the concrete signal
that a check was passing only because a real counterexample sat beyond its horizon.

Baseline (current `formal/checks.cfg`, `insn 15` / `reg 15 20` / `pc_fwd`&`pc_bwd 10 30` /
`causal 10 20` / `liveness`&`unique 1 10 30` / `csrw 30`), full from-scratch `make -C formal check -k`:

| | count |
|---|---|
| generated | 78 |
| pass | 67 |
| non-pass | 11 (`formal/EXPECTED_FAIL`, exact match) |

Raised sweep (`insn 30` / `reg 25 40 1800` / `pc_fwd`&`pc_bwd 15 50` / `causal 15 35` /
`liveness`&`unique 1 15 50` / `csrw 50` — roughly 2× the window of every check family, well past the
12-cycle two-hop floor derived above), full from-scratch `make -C formal check -k`:

| | count |
|---|---|
| generated | 78 |
| pass | 66 |
| non-pass, matching the baseline | 11 |
| inconclusive (`reg_ch0`, did not return) | 1 |

**No check's verdict differs between the two depth configurations.** Every check that passes at the
current depths still passes at roughly double them, and the non-pass set is the same 11 entries in
both runs — the 9 trap-group checks awaiting M3's `rvfi_trap` and the 2 `csrw_*` awaiting CSRs. No
check flipped PASS→FAIL, which is the signal this sweep existed to look for.

Two mechanical caveats on the raised run, neither a verdict:

- The 11 non-pass checks reported sby status `ERROR 16` rather than `FAIL`, because btormc found its
  counterexample and then failed to shell out to a trace-writing tool absent from the run
  environment (`engine_0.trace: COMMAND NOT FOUND`). The check still failed; only the witness is
  missing. Same set, same reason as at the current depths.
- `reg_ch0` and both `csrw_*` did not return before the run was stopped, so they carry no status
  file at the raised depths at all.

That last point is the runtime finding. At the current depths the full 78-check sweep completes in
**4m05s** wall with `-j6` (ADR-0024). At roughly doubled depths it had not finished after **10
minutes**, and `reg_ch0` — a single depth-40 query — was still in the solver. `reg` at depth 21
passes in about 8 seconds; at depth 40 it is at best minutes and at worst non-convergent, the same
shape ADR-0022 and ADR-0024 record for it under the previous engine. So depth is not free here, and
the cost is concentrated in exactly the check whose convergence was hardest won.

## Decision

**Keep `formal/checks.cfg`'s numeric depths as they are; replace the justification.** The empirical
sweep confirms the derivation: every current depth already clears the analytically-derived floor
(single-hop 8 / two-hop 12 for `insn`/`csrw`; ~2-4-cycle retire gap for the consistency checks) with
comfortable margin, and raising every depth further finds no new counterexample. Per this ticket's own
instruction, depths move only if the empirical evidence demands it — it does not, so the numbers stay,
and `formal/checks.cfg` gains the derivation inline (§ above, condensed) so the numbers read as
*derived-and-confirmed-sufficient* rather than *inherited-and-never-checked*.

The sweep found no flip, so no depth is raised on empirical grounds. Had one flipped, the
correct response would have been to raise that depth and treat the newly-exposed counterexample as
the ticket's result — not to lower it back.

## Rationale

Raising every depth to "obviously enough" (10× or 100×) was considered and rejected: `formal/EXPECTED_FAIL`'s
`reg_ch0` history (ADR-0022/ADR-0024) already shows this class of BMC query can go from single-digit
seconds to non-convergent as depth grows, and the raised-sweep run reproduces the same shape at `reg`'s
and `csrw`'s doubled depths (§ above) — so "deeper is strictly safer" is false for wall time even when
it's true for soundness. The right target is the smallest depth the derivation can defend, checked
empirically, not the largest one that still finishes.

## Consequences

- `formal/checks.cfg`'s `[depth]` section carries the derivation next to the numbers, replacing the
  ADR-0006-era "these came from the other core" silence.
- The floor/two-hop-margin argument (§ "Why one hop of margin, not an unbounded chain") is the citable
  answer the next person reaches for before changing a depth, rather than re-deriving from scratch or
  guessing.
- The ALTOPS-scoping caveat is now written down as a standing debt: dropping `` `RISCV_FORMAL_ALTOPS ``
  from a future ladder run requires a new depth derivation from the 32-cycle divider, not a reuse of
  this one.
- M2 is unaffected by this ADR either way — it was never blocked on depth justification, only on the
  three holes ADR-0023 named (two now closed: ADR-0021's fix and ADR-0024's engine switch; ALTOPS
  itself and `equiv.sh` remain open).

## Addendum, 2026-07-31: serialization is a fourth stall reason

ADR-0026 adds CSR/`mret` serialization as a fourth stall reason (bubble-flavoured, folded into the
existing `hazard` term). The derivation above enumerates three: scoreboard 2, accessor 1, divider 1
under ALTOPS. Serialization holds a CSR instruction in decode until execute/access/writeback drain
— **up to 3 further cycles**.

That moves the single-hop floor from 8 to roughly 11, and the two-hop figure from 12 to roughly 15.

Consequences for the configured depths:

- **`csrw 30` clears the new floor comfortably.** No change needed.
- **`insn 15` currently sits above the two-hop figure with margin; after serialization lands it sits
  on it.** That is not yet a soundness failure — a CSR instruction is not a hazard producer in the
  ordinary sense, so the worst-case chain that reaches 15 does not obviously involve one — but the
  margin is gone, and the empirical sweep should be repeated once the CSR RTL exists rather than
  assumed to still hold.

Traps themselves add **no** cycles: they commit in decode and the payload rides the same three
registered stages, so the trap-group `insn_*` checks are unaffected on that count.

The ALTOPS caveat above is untouched and still governs everything here.
