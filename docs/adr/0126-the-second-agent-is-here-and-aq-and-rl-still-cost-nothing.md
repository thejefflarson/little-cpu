# ADR-0126: The second agent is here, and `.aq`/`.rl` still cost nothing

**Status:** Accepted · 2026-08-19 · *Re-derives ADR-0108's `.aq`/`.rl` argument for two harts and
discharges the first of its four named invalidators. Changes no RTL, so no instrument reads
anything and no number is quoted. The requirement ADR-0110 attached to this — that a two-core
arbiter keep per-access completion in program order — is named, met by construction, and its
mechanised coverage is stated with the hole in it.*

## Context

ADR-0108 decoded `.aq` and `.rl` and ignored them, and was careful that the argument was **a
property of the configuration rather than of the design**: one hart, in order, no store buffer, no
cache, one outstanding access, every access completing before the next issues. Memory order is
program order, which is sequentially consistent, which is stronger than anything those bits ask
for. It named four things that would invalidate it — a second agent on memory, a store buffer, a
cache, posted MMIO writes — and one requirement for the work that would introduce the first: **a
two-core arbiter must keep per-access completion in program order**, or the question reopens as a
design question rather than as a comment.

ADR-0110 restated that as the standing condition on the dual-core SoC. The second hart is being
built now, so the argument has to be re-derived rather than inherited. This is that derivation.

## Decision — `.aq` and `.rl` stay decoded and ignored on two harts

The eleven A encodings keep their acquire and release bits decoded, checked for legality and
otherwise ignored. Nothing in the datapath changes and nothing is added, so this decision costs
exactly zero.

## The derivation, in five premises

Each is a property of this configuration, and each is written so it can be checked rather than
believed.

**1 — each hart is in order, with one outstanding access.** A memory transaction is presented from
the execute slot and its answer is there when the instruction reaches the accessor; nothing issues a
second access before the first has been answered. There are no non-blocking loads, no miss-status
registers and no write buffer. `test/accessor_tb.v`'s transaction count and `components_accessor`
are what say so, single-hart.

**2 — no store buffer and no cache.** A store is on the bus on the cycle it is granted and there is
nowhere else in the machine for it to be. The one peripheral on the data bus is the machine timer,
and its writes land on the cycle they are granted — nothing on this bus is posted.

**3 — no speculative access.** The no-wrong-path-state commitment forbids any state a later cycle
must un-commit, so no access is ever launched and retracted, and there is no access whose ordering
is provisional.

**4 — one serialization point.** The arbiter's grant is `$onehot0`, proved by k-induction over the
module's whole reachable state, so at most one hart drives the bus on any cycle and memory commits
the granted access on that cycle. **The total order over all memory accesses is the sequence of
grants** — a physical order the machine really has, not an order constructed to make the argument
work.

**5 — each hart's accesses appear in that order in its own program order.** This is 1 and 3
together: hart *i*'s *k*-th access is granted, and answered, before its *(k+1)*-th is presented. The
arbiter cannot reorder a hart against itself because it is never holding two of that hart's requests.

Premises 4 and 5 are exactly the requirement ADR-0108 wrote down. **Per-access completion in program
order is met by construction here** — not by an ordering rule the arbiter enforces, but because a
hart never has two accesses outstanding for it to get wrong.

## What follows

Every execution of this machine is an interleaving of the two harts' program orders, and every
interleaving is a sequentially consistent execution. **Sequential consistency is strictly stronger
than any ordering `.aq` or `.rl` can request.** An acquire annotation forbids later accesses of the
same hart from being ordered before the atomic; a release forbids earlier ones from being ordered
after it. Under premise 5 no access of a hart is ordered against another of that hart's at all,
so **there is no reordering for those bits to forbid**. Ignoring them is exact, not approximate —
the same claim ADR-0108 made, resting now on a different fifth premise.

## What was measured, and what was not

**Nothing was measured, because nothing changed.** No RTL moved, so `make fit` and `make soc-timing`
cannot have moved and no figure appears above. That is a fact about the diff and not a measurement.

**No program tests this and none can be written that does.** The programs in `test/dual/` observe
values, and a value cannot show the absence of a reordering the machine is incapable of performing.
A memory-model litmus test — the store-buffer shape, the message-passing shape — would run on this
machine and pass under any implementation of `.aq`/`.rl`, including a wrong one, because premises 1
through 5 already forbid the outcome it looks for. Adding one would be a grader that cannot fail.

## Where the mechanised coverage stops, said plainly

The argument above is sound and its parts are unevenly checked.

- **Premise 4 is proved**, over the arbiter module alone: `$onehot0` grant, a lock indivisible, and a
  per-hart wait bound of 2 and 3 cycles, each covered as well as asserted.
- **Premises 1, 2, 3 and 5 are checked single-hart only.** `components_accessor`, `test/accessor_tb.v`
  and the pipeline's own proofs describe one core. There are no generated riscv-formal checks for a
  dual configuration and no proof composes the core surface with the arbiter.
- **The arbiter's wait bound rests on an assumption the core surface owes it** — no hart raises
  `mem_lock` on two cycles running — and the core surface asserts exactly that, in `rtl/accessor.v`'s
  `FORMAL` block, together with the equality that every locked cycle is followed by the write-back it
  bought. The two halves exist. **Nothing in this tree puts them in one proof**, because nothing
  instantiates both, so the composition is an argument on this page rather than a verdict from a
  solver.

## What reopens this

The list ADR-0108 wrote for one hart, re-derived for two. Any one of these makes the derivation
above false and the question a design question again:

- **a store buffer**, or any place a store can sit after issue;
- **a cache**, private or shared;
- **a second outstanding access per hart** — a non-blocking load, hit-under-miss, a write that
  retires before it lands. This is premise 1, and it is the one an optimisation is most likely to
  take, because it looks like a CPI win and not like a memory-model change;
- **a third master not on the same grant** — DMA, a debug port, anything that reaches memory without
  passing premise 4's serialization point. A third master *on* the grant is covered by this
  derivation; one beside it is not covered at all;
- **posted writes**, MMIO or otherwise, where a write is acknowledged before it lands;
- **an arbiter that can hold two requests from one hart**, which would make premise 5 an ordering
  rule it has to enforce rather than a shape it cannot violate.

The first four are ADR-0108's, restated. The last two are new, and they are new because the second
agent arrived: they are properties of the arbiter, and there was no arbiter to state them about.

## Consequences

- `.aq`/`.rl` remain decode-only, `misa`'s A bit is unaffected, and the A extension's decode rows do
  not move.
- **ADR-0110's standing condition is discharged for this second agent and for no other.** The
  sentence it left — "only until a second agent touches memory" — is answered here for the second
  hart behind this arbiter. It is not answered for a DMA engine, and a future one gets this page
  again rather than a citation of it.
- The composition hole above is the honest cost of landing the arbiter and the core surface in
  separate changes. Closing it means a proof over the dual top, which is the dual top's work.
- The next person to make a hart's memory access non-blocking is changing the memory model. That is
  the sentence this ADR exists to put in `git blame`.
