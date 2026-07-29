# ADR-0015: The accessor's load-response turnaround is the third stall source

**Status:** Accepted · 2026-07-28 · *Extends ADR-0009; ratified on integrating JEF-607*

## Context

ADR-0009 fixed the stall protocol: one global broadcast, upstream freezes, downstream drains,
`valid = 0` injected at the stalling stage. It named exactly two stall sources — the decode
scoreboard's RAW hazard and the multi-cycle divider — because those were the only two backpressure
sources anyone had found. It says so explicitly: "there is no backpressure source in this core
other than the divider and the decode scoreboard."

That is wrong, and JEF-607 found out the hard way while implementing ADR-0004.

**Both memory models — `rtl/memory.v` and the model inside `test/testbench.v` — register
`mem_rdata` one cycle after the address is presented.** That is not a modelling choice that could
be relaxed; it is what a synchronous BRAM is, and the up5k's block RAM is exactly that. So a load
presents its address in the cycle it occupies the accessor and the data is not back until the
cycle after. Every other instruction — stores included, since a store's data goes *out* with the
address — settles in the one cycle each pipeline stage takes.

Without something to hold the pipeline for that cycle, the completing load and the next
instruction collide over the accessor's single output register and the single-port memory bus.
ADR-0009's text offers no answer, because ADR-0009 did not know the question existed.

## Decision

**Treat the load response as a third source of the same global stall.** No new mechanism: no
flush, no per-stage ready/valid handshake, no skidbuffer. Concretely:

1. **`accessor.stalled = in.valid && is_load`** — high for exactly the cycle a load's request
   fires, and provably for **at most one consecutive cycle** (see liveness below). `littlecpu.v`
   ORs it into the same broadcast that already carries the divider's stall.

2. **The executor freezes.** It is upstream of the accessor, so per ADR-0009 it emits a bubble
   (`out <= 0`) and holds every other register — `state`, the mul/div datapath, the latched op
   selects — unchanged.

3. **The accessor latches a pending-load record** at the request cycle (`pending_rd`, the width /
   sign / sub-word-offset bits) and consumes it one cycle later when `mem_rdata` is actually the
   answer to that address.

4. **Decode *holds*, it does not bubble.** This is the one genuinely new rule, and it is the
   asymmetry worth reading twice:

   | Stall reason | `pc` | `decoder_out` |
   |---|---|---|
   | RAW hazard (scoreboard) | holds | **bubble** (`out <= 0`) |
   | `divider_stall` | holds | **hold unchanged** |
   | `accessor_stall` | holds | **hold unchanged** |

   Both downstream stalls fire *one cycle behind their cause*, so decode has already been given one
   free cycle to issue the instruction *after* the one that made the executor or accessor busy.
   That instruction is sitting in `decoder_out`, not yet consumed by anything. Bubbling it there
   would **silently drop an instruction** — a correctness bug that no test would obviously name.
   For the hazard case the reverse holds: nothing stops the executor reading `decoder_out` every
   cycle, so holding it would have the executor execute the same instruction repeatedly.

5. **The decode scoreboard gains a third producer check**, `accessor_pending_valid` /
   `accessor_pending_rd`, alongside `decoder_out` and `executor_out`. This is not a general
   widening of the scoreboard to a third pipeline stage; it exists only to cover the one extra
   cycle a load spends in the accessor.

### The invariants this rests on

State them, because the correctness argument is non-local and a later change can break it silently.

- **I1 — Hold-not-bubble safety.** When `divider_stall` or `accessor_stall` is asserted, nothing
  downstream may consume `decoder_out` that cycle. This holds for two *different* reasons: the
  executor's `case (state)` only reads `in` from its `init` arm, never while `state == divide`; and
  for the accessor case the executor is itself frozen by the same combinational signal in the same
  cycle. A change that lets the executor read `in` while frozen breaks this.
- **I2 — No producer-visibility gap.** Every in-flight non-`x0` `rd` is visible to the scoreboard
  on *every* cycle between issue and the regfile write-through, with no hole. The chain is
  contiguous: `decoder_out` → `executor_out` → (`accessor_pending` for loads only) → `accessor_out`,
  which is combinationally the regfile's write port, so write-through covers that last cycle.
  The divider is the one case where `executor_out.valid` is 0 while the instruction is live; that
  window is covered because `divider_stall` freezes decode for exactly those cycles, and
  `executor_out.valid` rises on the first cycle the stall drops.
- **I3 — At most one consecutive `accessor_stall` cycle.** The cycle after the request, the
  executor's frozen bubble is what the accessor sees, so `in.valid` is 0 and `stalled` falls. This
  is what makes the stall a stall and not a livelock.
- **I4 — `accessor_stall` and `divider_stall` are mutually exclusive.** A load reaching the
  accessor forces the executor to bubble, so the next instruction cannot enter the divider until
  the load has drained; and while dividing, decode is frozen, so nothing new reaches the accessor.

### Why this does not violate the no-flush invariant

Nothing is killed. Nothing is squashed. The executor's `out <= 0` on `accessor_stall` is a bubble
*inserted*, not an instruction *discarded* — the instruction it would have executed is still
sitting in `decoder_out`, held there by rule 4, and it executes on the next cycle. CLAUDE.md
invariant 1 stands untouched: there is still no wrong-path instruction anywhere in this core, so
there is still nothing to flush.

## Rationale

Considered and rejected: **give every instruction two cycles in the accessor.** Uniform, no stall
source, no third scoreboard entry, and arguably the most readable thing on the table — which
matters on a project whose stated point is readability. Rejected because it costs a full cycle of
CPI on every instruction to avoid machinery that *already exists for the divider*, and because the
stall broadcast is the shape ADR-0009 already committed to. It stays on the table as a
simplification if the interlock ever proves hard to reason about.

Considered and rejected: **a per-stage ready/valid handshake.** ADR-0009 forbids it by name, and
the skidbuffer implementation of it was deliberately deleted in `49b317a`. A handshake network for
two customers is machinery, which is exactly what CLAUDE.md says not to add.

Considered and rejected: **a combinational-read memory model.** It would make the whole problem
vanish in simulation and reintroduce it on the first day of FPGA bring-up. The up5k has
synchronous block RAM. Modelling something the target part cannot do would be lying to ourselves
in the one direction that is expensive to discover late.

## Consequences

- **Loads cost one extra cycle.** Accepted, on a core that already accepts a 2-cycle RAW bubble
  and a ~32-cycle divide.
- **A new CLAUDE.md invariant**, since I1 and I2 are precisely the kind of non-local property a
  later change violates without noticing.
- **`test/asm/hazard.S`** covers the reachable shapes: back-to-back RAW on rs1, on rs2, on both;
  load-use; divide→dependent; divide→independent-load. It is the regression test for I2.
- **The executor's component proof assumes `assume(!accessor_stall)`.** Correct for a standalone
  proof about arithmetic — the signal is a free input there and the solver would otherwise hold it
  high forever and defeat every assertion — but it means **no formal proof currently covers the
  executor's freeze branch**. `hazard.S` is the only thing that does. M2's full-core riscv-formal
  wrapper is where that gap actually closes, and it should be treated as one of M2's jobs.
- **ADR-0009's claim that the divider and the scoreboard are the only backpressure sources is
  superseded** by this ADR. ADR-0009 otherwise stands unchanged: the broadcast, the
  freeze-upstream/drain-downstream rule, and the ban on handshakes all survive intact — this is a
  third customer for the same mechanism, not a new mechanism.
