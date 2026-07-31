# ADR-0026: Stalls are four reasons over two mechanisms

**Status:** Accepted · 2026-07-31 · *Amends ADR-0009 and CLAUDE.md invariant 8*

## Context

ADR-0009 established the stall protocol and CLAUDE.md invariant 8 records it as "a single global
broadcast with three sources — the decode scoreboard, the divider, and the accessor's one-cycle
load-response turnaround."

M3 adds a fourth reason to stall: ADR-0005 requires CSR instructions and `mret` to serialize, held
in decode until execute/access/writeback drain. The question this ADR settles is whether that is a
fourth *mechanism* or a fourth *reason* riding an existing one.

## Decision

**Four reasons, still two mechanisms.**

ADR-0009's rule (a) partitions stalls by what they do to `decoder_out`:

- **hold** — the divider and the accessor stall hold `decoder_out` unchanged, because the
  instruction has already issued and downstream must not re-consume it.
- **bubble** — the scoreboard bubbles `decoder_out`, because the hazarded instruction has *not*
  issued and must be retried.

CSR serialization is bubble-shaped: the CSR instruction has not issued yet, and the executor reads
`in` every cycle, so holding would have it reprocess the same instruction repeatedly. It therefore
folds into the existing `hazard` term rather than becoming new machinery:

```systemverilog
assign stall = hazard || csr_serialize || divider_stall || accessor_stall;
```

where `csr_serialize` is "this is a CSR or `mret` instruction and the pipe is not yet drained."

## The gap this exposes

The decoder currently sees three producer slots: its own `out`, `executor_out`, and
`accessor_pending_{valid,rd}`. **`accessor_pending_valid` covers loads only** — see the comment at
`rtl/decoder.v:350-354`.

A genuine drain condition needs more than that. A store in flight in the accessor is not visible in
any of those three slots, so "drained" and "no pending load" are different predicates. Computing
`csr_serialize` from the existing signals would let a CSR instruction issue while a store is still
in the accessor.

**So the decoder gains a new input: `accessor_out.valid`, wired in `rtl/littlecpu.v`.**

That is easy to miss, and missing it produces a `minstret` that is off by one only when a store
happens to be in flight — an intermittent, load-dependent wrong answer.

## Consequences

- CLAUDE.md invariant 8 is restated as four reasons over two mechanisms, with the hold/bubble
  partition as the thing that actually matters.
- The decoder's port list grows by one input. That is a `rtl/littlecpu.v` change and touches the
  component-proof harnesses that instantiate the decoder (`formal/components.sby`'s `decoder` task
  and `formal/pcloop.sv`).
- ADR-0025's depth derivation gains a fourth term — see ADR-0029.
- Nothing about ADR-0009's freeze-upstream/drain-downstream rule changes. This is a new customer
  for an existing mechanism, not a new mechanism.
