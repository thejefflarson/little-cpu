# 0132 — The completeness exclusion predicate keys on the encoding, not the class

Status: accepted · answers the decision ADR-0131 left open. Amends the mechanism ADR-0014 built and
`formal/COMPLETE_EXCLUSIONS` documents; supersedes nothing, because every exclusion standing today
stays legal under the widened shape.

## Context

ADR-0131 built Zba's `sh2add`, measured it on both axes, and declined it — not on cost, which is a
null at sixteen paired seeds, but because `make -C formal complete` cannot be made to pass for it.
`sh2add` retires non-trapping under the OP major opcode `0110011`, riscv-formal's `rv32imc` spec
model has no arm for funct7 `0010000`, and so `spec_valid` is 0 and `complete.sv`'s assertion fires.

The exclusion mechanism cannot express the fix. `formal/check-complete-exclusions.py`'s `WIRE_RE`
accepts exactly one predicate shape:

```
wire <slug> = insn_uncompressed && insn_opcode == 7'bXXXXXXX;
```

A whole seven-bit opcode class, nothing narrower. Excluding `0110011` outright is refused by design:
`check_no_spec_model` reads `insns/isa_rv32imc.txt` from the pinned clone, finds `add`, `sub` and the
M mnemonics named there, and rejects the line — that refusal is the point of the script. So the only
two moves available are to excuse the modelled instructions beside the new one, or to narrow the
predicate shape. The script's own comment says which of those is in scope: widening the pattern "is a
design change and belongs in an ADR, not in a regex."

**The finding is structural, not about `sh2add`.** Every extension this repo's ISA-target section
names as a candidate shares a major opcode with an already-modelled instruction: Zba's other two
shifts sit in the same `0110011`, Zbb and Zbs mostly extend OP and OP-IMM, and Zknh's rotations live
under OP-IMM `0010011` beside ADDI and the shifts. A probe of any of them stops here. Until this is
decided, `complete` is a wall in front of every extension claim, and the cheapest way past a wall is
the one the exclusion file exists to prevent — widening the set until the red goes away.

## Decision

**The predicate may key on the full encoding — opcode plus funct3 and funct7 — and the checker must
be widened to match.** Building it is a ticket of its own; this record is what makes that ticket
legitimate rather than a weakening nobody reviewed.

The reasoning is that a per-encoding exclusion is **strictly narrower** than the class exclusion it
replaces. Today, excluding a class excuses every encoding in it, including ones that arrive later and
have models. Naming one funct7 row excuses exactly that row. The direction of the change is more
checking, not less — which is the opposite of what the exclusion file warns about, and the reason
this is an amendment rather than a refusal.

Three properties make today's mechanism trustworthy, and the widened one **must keep all three**. Any
implementation that drops one is a different decision and needs its own record.

1. **The predicate reads `rvfi_insn` and nothing else.** No decoder flag, no `spec_*` signal, no
   `rvfi_trap` term. This is what lets a reader of a green `complete` trust the exclusions are
   encoding-keyed without reading them, and it is the property `WIRE_RE` really enforces — the
   opcode-only shape is a means to it, not the property itself. The widened regex pins the same two
   `REQUIRED_DEFS` and adds field definitions for funct3 and funct7 the same literal way.
2. **The checker re-derives "no spec model" from the pinned clone, per encoding.** `check_no_spec_model`
   must answer the narrower question — does riscv-formal model *this* encoding — and must still refuse
   a line whose encoding it can model. It may not fall back to the mnemonic list when it cannot
   resolve an encoding: an unresolvable encoding is a red, not a pass. Fail closed.
3. **The set stays a tracked, reviewable diff in both directions.** `formal/COMPLETE_EXCLUSIONS` and
   the wires in `complete.sv` match exactly, an entry on one side without the other fails, and the
   per-entry reason stays at the `complete.sv` site.

Two consequences worth stating because they are easy to lose. The three exclusions standing today —
MISC-MEM, SYSTEM, AMO — are whole classes for a real reason: the pin ships no model for **any**
encoding in them, so they keep the class-shaped predicate and the widened regex must accept it. And
this does not decide that any extension ships. It removes a mechanism from the list of reasons one
cannot, leaving the cost question exactly where ADR-0131 left it: the period is flat, the area is not,
and `sh2add` alone spent 49 of `FIT_MAX_LC`'s roughly 117 cells of headroom.

## The alternative that was rejected

**Teaching riscv-formal a spec model for the new instruction**, rather than excluding it, is the
better answer on the merits — an oracle beats a hole. It is rejected because it cannot be done at the
pin. The models come from the clone `formal/pin.mk` names, adding one means carrying a patch against
an upstream this repo deliberately takes verbatim, and a local fork of the oracle is a one-way door:
every future pin bump inherits it, and the thing being forked is the only independent check the
generated walk has. If upstream gains Zba models, `formal/COMPLETE_EXCLUSIONS`'s own mechanism goes
red until the exclusion comes out, which is the behaviour that makes the hole temporary by
construction.

## What this does not claim

An exclusion is still a hole, and a per-encoding hole is a smaller hole rather than a check. An
instruction excluded this way has **no oracle** — `complete` stops asking about it, and what remains
is whatever `test/decoder_tb.v`, `test/exec_tb.v`, the component proofs and a self-checking `.S`
program say, which is assertions this repo wrote and not a specification. That is the same standing
caveat SYSTEM, MISC-MEM and AMO carry, and the third question in `COMPLETE_EXCLUSIONS`'s header — is
something else checking the excluded behaviour, and "nothing" is an acceptable and loud answer —
applies unchanged to every per-encoding line.
