# ADR-0030: Trap cause priority, and why the causes are currently disjoint

**Status:** Accepted · 2026-07-31 · *Supplements ADR-0005*

## Context

ADR-0005 lists the trap causes this core implements — illegal instruction (2), breakpoint (3), load
address misaligned (4), store address misaligned (6), environment call from M-mode (11) — but not
their **priority**, because with all traps committed in a single decode-stage branch, something has
to decide which cause wins if two conditions hold at once.

## Decision

**Priority order, highest first:** illegal instruction (2) → breakpoint (3) → environment call (11)
→ load misaligned (4) → store misaligned (6).

The reasoning is that an instruction which is not legal cannot meaningfully be said to have a
misaligned operand: the address computation is only defined for an instruction the decoder
recognises.

## The part worth writing down: the cases cannot co-occur

**In this core the priority encoder is vacuous today**, and that is the fact most likely to be lost.

- Illegal instruction is raised only when `instr_valid` is false, which clears every execution flag
  including `is_lw`/`is_sw` and friends (`rtl/decoder.v:563-573`). An illegal instruction has no
  memory operand, so it cannot also be misaligned.
- `ecall` and `ebreak` are distinguished by `funct12` and are mutually exclusive with each other and
  with any load or store.
- Load-misaligned and store-misaligned are distinguished by the access direction and cannot both
  hold.

So the order above is currently unobservable. It is recorded anyway for two reasons, and both are
about the *next* person:

1. Without the order written down, someone reading the priority encoder concludes it is dead code
   and simplifies it away — then a later change makes the cases overlap and the behaviour is
   whatever the simplification happened to produce.
2. Without the **disjointness argument** written down, someone else concludes the encoder is
   load-bearing, and "fixes" a priority that was never wrong, or adds a new cause that breaks
   disjointness without realising disjointness was the thing making the encoder vacuous.

The component proof asserts mutual exclusivity, so if a future change breaks it, the proof fails
rather than the behaviour silently changing.

## Consequences

- The trap-cause selection carries a comment stating both the order and the disjointness argument.
- The component proof asserts the causes are mutually exclusive and that the committed `mcause`
  matches the stated order — the assertion is what keeps this ADR honest as the core grows.
- Adding any new trap cause requires re-checking disjointness, not just appending to the encoder.
  Interrupts in particular (deliberately out of scope, ADR-0005) would break it, since an interrupt
  can be pending during any instruction.
