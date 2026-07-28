# ADR-0004: Stall-only hazard interlock with a combinational-read regfile

**Status:** Accepted · 2026-07-27

## Context

The core has **no hazard handling at all** — no forwarding, no stalls, no load-use interlock.
Worse, it is broken before hazards even matter: `rtl/regfile.v:15-17` performs *registered* reads,
so the operand values the decoder consumes in cycle N belong to the rs1/rs2 indices of the
*previous* instruction. Every register-consuming instruction receives wrong operands, always. This
— not the missing CSRs — is why the core does not work.

Separately, `rtl/executor.v:23-25` computes a `stalled` signal that is wired to nothing. During the
multi-cycle divide, nothing upstream stalls, and `executor_out` holds the previous instruction's
load/store flags, so the accessor re-issues that memory operation once per divide iteration.

## Decision

Adopt a **stall-only interlock**, keeping the fused front end:

1. **Regfile becomes combinational-read with write-through bypass.** If `wen && waddr == rs`,
   forward `wdata` directly. This fixes the operand-timing defect and covers the writeback-stage
   hazard for free.
2. **Every inter-stage struct carries a `valid` bit.** A bubble is `valid = 0`. Retire is `valid`
   reaching writeback, which also gates `wen` and drives `rvfi_valid`.
3. **A stall-only scoreboard in decode.** If rs1 or rs2 — when actually used by this instruction —
   matches a live `rd` in `decoder_out` or `executor_out`, freeze the PC and emit a bubble.
   Write-through covers the writeback stage, so only two stages need checking.
4. **The divider freezes decode globally** via the now-wired `stalled` signal, and drops `valid`
   until it completes.
5. **No forwarding network.** Adding one is a CPI-only optimisation and requires a new ADR.

## Rationale

This is the smallest delta from what exists, and it preserves the design's one genuinely elegant
idea (ADR-0003's no-wrong-path invariant). Every simplification compounds: traps-in-decode
(ADR-0005) makes CSR commit precise for free; the valid-bit protocol doubles as the RVFI retire
signal (ADR-0006); and stall-only hazards keep the formal state space small.

Alternatives rejected:

- **Textbook 5-stage — registered IF/ID, branch resolution in EX, flush on taken branch, full
  forwarding.** Roughly 3× the new state machinery: PC ownership moves out of the decoder (a
  rewrite of the repo's central module), plus kill/flush logic, wrong-path RVFI suppression, three
  forwarding muxes, and a load-use interlock. It buys Fmax the project does not need yet, at the
  price of the complexity class the project exists to avoid — and flush × stall × divider
  interaction is the classic source of pipeline bugs.
- **Collapse to a multicycle FSM core (picorv32 shape).** Trivially precise and easy to verify, but
  it discards the staged-modules-with-structs identity of the repo. That would be a different
  project — and it is the direction ADR-0001 explicitly declines.

## Consequences

- **CPI cost:** worst-case 2-cycle RAW bubble; ~32-cycle divide (see ADR-0002's brief for the
  iteration-count fix). Accepted — this core optimises for readability, not throughput.
- **Fmax risk:** the combinational path is long — `pc` → ROM → decode → branch compare → `pc`, now
  with a combinational regfile read and a CSR read on it. Real, accepted, deferred with FPGA timing.
  Documented escape hatch: negedge-read BRAM regfile.
- **Area:** the regfile moves from BRAM to flip-flops (~992 FF on up5k). The budget still closes at
  roughly 55–70% of the part.
- Forwarding paths can be added later as pure CPI upgrades without disturbing anything above.
- Invariant for `CLAUDE.md`: **no flush logic may be introduced.** If a change appears to need one,
  the design has gone wrong somewhere upstream.
