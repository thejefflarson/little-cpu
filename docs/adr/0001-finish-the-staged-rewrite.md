# ADR-0001: Finish the staged rewrite rather than resurrect the serialized core

**Status:** Accepted · 2026-07-27

## Context

Little CPU has been through two architectural teardowns. A 491-line serialized FSM core
(`rtl/riscv.v` + 159-line `rtl/alu.v`, single memory port with a picorv32-style
`mem_valid`/`mem_instr`/`mem_ready` handshake) **passed riscv-formal sans CSRs**. It was replaced
by a handshake/skidbuffer pipeline in 2021 (`9758a39` → `1709433` → `9ea584a`), which was then
stripped down to the current fused staged design in 2023 (`49b317a` → `4fbd650` → `13fec44`).

The rewrite is unfinished. Operands are one instruction stale (registered-read regfile with no
compensating structure), `executor.stalled` is computed but unwired, RVFI ports are declared but
never driven, and the `formal/` harness still speaks the deleted core's interface. The project has
therefore gone from *formally verified* to *unverified*.

## Decision

**Finish the staged rewrite.** Do not resurrect the serialized core as mainline, as a component, or
as a co-simulation oracle.

The serialized core's role is **reference text**: its RVFI retire block, `rs1_valid`/`rs2_valid`
masking, `rvfi_order` accumulation, and memory rmask/wmask conventions port near line-for-line into
the new shadow-payload design (`git show 1709433^:rtl/riscv.v`). Its arithmetic is the reference
for fixing the mul/div bugs the rewrite inherited (`git show e67875c^:rtl/alu.v`).

The wave-0 `formal/` harness is **ported, not rewritten** — see ADR-0006.

The milestone ladder carries an explicit **parity checkpoint (M2)**: the pipelined core must
re-prove everything the serialized core proved (rv32imc insn/reg/pc_fwd/pc_bwd/unique/causal, no
CSRs) before any new ground is broken.

## Rationale

1. The staged design is the repo's stated identity, and the author tore the old core down *twice*.
   Reverting reverses years of intent to save roughly one milestone of work.
2. The fused-fetch design retains the serialized core's key verification property — no wrong-path
   instructions, therefore no flush — while pipelining. It is the rare rewrite that keeps the old
   verification story intact.
3. The old core's green run depended on the single-port handshake memory interface that no longer
   exists anywhere in the SoC. Resurrecting it means maintaining two cores *and* two memory systems.
4. A co-sim oracle against the old core is redundant machinery: riscv-formal's ISA spec modules are
   already the golden model, and sequential equivalence between an FSM core and a pipeline is not
   something `equiv_simple`/`equiv_induct` can establish.

## Consequences

- Work is framed as **completing** the rewrite, not repairing damage. Defects are sorted into
  *genuine bugs* (wrong in any architecture — six of them) and *unreached scaffolding*. The two
  read very differently and are planned differently.
- The project stays unverified until M2. That is the single largest risk carried by this plan, and
  the reason M2 is a hard checkpoint rather than a soft goal.
- Git history becomes load-bearing documentation. `CLAUDE.md` must carry the rewrite narrative and
  the `git show` incantations, or the next reader loses the reference text.

## Alternatives considered

- **Port CSRs and C onto the known-good serialized core, treat the staged version as a later
  refactor.** Rejected: discards the design the author is proud of, and forfeits the no-wrong-path
  property that makes the staged core unusually easy to verify.
- **Keep both cores, use the old as a co-sim reference.** Rejected: two cores plus two memory
  systems to maintain, for an oracle riscv-formal already provides better.
