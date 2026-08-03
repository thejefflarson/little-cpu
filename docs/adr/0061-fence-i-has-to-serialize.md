# ADR-0061: `fence.i` has to serialize

**Status:** Accepted · 2026-08-02 · *Amends
[ADR-0002](0002-isa-target-rv32imc-zicsr.md), which claims Zifencei on the grounds that a conformant
`fence.i` is free here. Lands with
[ADR-0060](0060-the-steal-reaches-decode-as-the-sixth-stall-reason.md) and is separate from it on
purpose.*

## Context

ADR-0002 claims Zifencei in `misa`'s ISA string, and CLAUDE.md's ISA-target section gives the
reason: one hart, no icache, so a conformant `fence.i` is a NOP, which is what `rtl/decoder.v`
already does.

That reasoning has a premise it does not state. A `fence.i` orders *stores to instruction memory*
against *fetches on the same hart*. It was free here because no store could reach instruction
memory — ADR-0008's separate address spaces made the ordering vacuous. ADR-0059 removed that
premise. **"Correct and for free" was a property of the split, not of the core.**

## The defect, in cycles

The fetch address is published one cycle early (ADR-0054), so a store's write and the fetch of the
instruction that should observe it land two edges apart:

| cycle | store | the instruction after `fence.i` |
|---|---|---|
| D | the store is on the bus | |
| D+1 | | `fence.i` issues; its successor's fetch address is latched at this edge |
| D+2 | the array is written at this edge | |
| D+3 | | the successor executes, from the word fetched at D+1 |

The successor executes stale text after `fence.i` retired. That is a Zifencei violation, and it is
reachable the moment `mem_ren` and `fetch_stall` are live.

Nothing else in the machine covers it. The steal (ADR-0059) forbids a fetch read and a store write
meeting on one edge, so a *torn* word is impossible — but it says nothing about a fetch that
happened before the write and is therefore cleanly stale.

## Decision

**`fence.i` joins the serialization drain that Zicsr instructions and `mret` already use.** One OR
term on the existing predicate in `rtl/decoder.v`; no new mechanism, no new stall reason.

`pipe_drained` requires all four in-flight slots empty — `decoder_out`, `executor_out`,
`accessor_pending_valid` and `accessor_out.valid` — and the fourth is the one that makes this work.
A store is in `accessor_out` for the cycle *after* its write edge and appears in none of the other
three (ADR-0026), so a drained pipe means the write edge has already passed. The address published
on `fence.i`'s own issue edge is therefore latched against the written array, and everything after
it is fetched from post-write text.

It costs 3-4 cycles when executed and nothing otherwise. `fence` (the data-ordering one) is
untouched and stays a plain NOP: one hart, one bus, one access in flight, nothing to order.

Instructions *between* a text store and the `fence.i` may see old or new text. That is legal — the
spec only requires ordering across the fence — and they cannot see a torn value, because the steal
forbids the same-edge read and write.

## Consequences

- **CLAUDE.md invariant 5 names three things now**: CSR instructions, `mret` and `fence.i`. The
  first two serialize to keep a one-cycle-wide architectural update from interleaving; this one
  serializes to order a memory write against a fetch. Same mechanism, different reason, and the
  invariant says so rather than implying one reason covers both.
- **The ISA-target section stops saying Zifencei is free.** It is claimed and implemented, and it
  costs a drain.
- **No oracle here checks it.** riscv-formal ships no spec model for `fence.i` at the pin — it is on
  `formal/COMPLETE_EXCLUSIONS` for exactly that — so `spec_valid` is 0 for it and the ladder's
  semantic block is skipped. What covers the decision is `test/decoder_tb.v` (the drain predicate,
  driven directly) and `formal/pcloop.sv` (the pc holds while it waits). What would cover the
  *behaviour* is a program that stores to text, fences and executes the result: `selfmod.S`, in the
  brief's step 5, not in this change. Until it exists this is an argued fix with a unit test, and
  that is the honest description of it.
- **The general lesson is the one to keep.** A conformance claim that is free because of a
  structural accident is a claim with an unstated premise, and the premise outlives the memory of
  it. `misa`'s C bit was in the same position once (CLAUDE.md: "no longer an untested claim"). The
  place to look for the next one is any line in this repo saying something is correct *and* free.
