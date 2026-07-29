# ADR-0021: The formal ladder runs the compressed checks, and it found a real bug on the first run

**Status:** Accepted · 2026-07-29 · *Supplements ADR-0002, ADR-0003, ADR-0006*

## Context

`formal/checks.cfg` sets the ISA that `genchecks-local.py` generates per-instruction checks from.
The obvious-looking choice at the ladder port was `rv32im` — matching what the `.S` suite actually
assembles (`-march=rv32im_zicsr`, because ADR-0003's dual-word fetch window does not exist yet, so
the sim legs cannot execute a compressed program). Under that setting the ladder would have
generated 45 checks.

`isa rv32imc` was kept instead, generating **70** checks: 45 uncompressed plus 25 `insn_c_*`.

The reason this is sound, and not a check running against a capability the core lacks, is a
property of riscv-formal's own harness: the base `insn_*` checks never constrain how `imem_data`
arrives. It is a free value every cycle (`formal/wrapper.v`), with no alignment or fetch-window
assumption anywhere. That constraint lives only in `formal/imemcheck.sv`, which is a separate task.
So an `insn_c_*` check drives `rtl/decoder.v`'s compressed decode path directly and is not gated on
the missing fetch window.

## Decision

**The ladder's ISA stays `rv32imc` — the C-extension checks run.** This is a deliberate choice,
not a leftover from the wave-0 config.

It is the only mechanised coverage the compressed decoder has. The `.S` suite cannot reach it
(it assembles without C), and `test/monitor.v` — self-checking per retire though it is — can only
check instructions a test actually executes. The compressed decode path was, until this ladder ran,
**entirely unexercised by anything in the repository.**

## Consequences

The decision paid for itself on the first run. `insn_c_jr` and `insn_c_jalr` both fail at
`rvfi_insn_check.sv:178` — `assert(rvformal_addr_eq(spec_pc_wdata, pc_wdata))`, a wrong jump
target — and the counterexample points at a real, synthesizable defect in shipping RTL:

- `rtl/decoder.v:145` — `instr_jalr = instr_jalr_op || instr_cjr || instr_cjalr`
- `rtl/decoder.v:114` — `instr_load_op || instr_jalr: immediate = i_immediate`
- `rtl/decoder.v:90`  — `i_immediate = {{20{instr[31]}}, instr[31:20]}`

C.JR and C.JALR are 16-bit instructions. `rtl/fetcher.v` drives `out.instr = imem_data` raw, with
no masking, so for a compressed instruction `instr[31:16]` is whatever halfword happens to sit next
in memory. Routing C.JR/C.JALR through the JALR arm therefore reads `instr[31:20]` — adjacent
memory — as a jump displacement, and `rtl/decoder.v:485-487` adds it to `rs1`. **Every C.JR and
C.JALR target is corrupted by an arbitrary neighbouring instruction word.** Per the ISA spec both
have an implicit zero offset; the correct behaviour is `immediate = 0`, which is what the `default`
arm already yields once `instr_jalr` stops claiming them.

This is not behind any `ifdef`. It is in the synthesized core.

**The fix does not land here, and it should not land alone.** `rtl/` was out of scope for the
harness port, and more importantly a one-line immediate-select repair would be unverifiable by
this repository's own standards: the `.S` suite cannot assemble a program that executes C.JR until
ADR-0003's fetch window exists, so the only evidence a fix worked would be the same formal check
that found it. The fix belongs with the fetch window, where it can be tested in all three legs.
Until then it is recorded in CLAUDE.md's defect ledger, and the two failing checks are named in
the ladder's expected-failure baseline (ADR-0022) so their disappearance is noticed.

The general lesson is worth more than the bug: **a leg of verification that only checks what the
other legs can already execute buys nothing.** The 47-test suite and the per-retire monitor were
both structurally incapable of seeing this. Formal saw it in one second of solver time. Keep the
oracle's alphabet wider than the simulator's.
