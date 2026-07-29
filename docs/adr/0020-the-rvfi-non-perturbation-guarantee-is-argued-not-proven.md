# ADR-0020: The RVFI non-perturbation guarantee is argued, not proven

**Status:** Accepted · 2026-07-28 · *Supplements ADR-0006*

## Context

ADR-0006 kept `formal/equiv.sh` deliberately, and named its job precisely: it proves that **RVFI
instrumentation does not change core behaviour** — gold is the plain build, gate is the
`RISCV_FORMAL` build with the `rvfi_*` ports deleted. The ADR called it out as earning its keep
"with large `ifdef` shadow payloads landing in the structs", which is exactly what JEF-628 landed:
a 138-bit `rvfi_shadow` threaded through three inter-stage structs, plus five more mem-capture
fields in `accessor_output`, plus a latch pair in the accessor's load-response path.

At integration that guarantee is **not proven**. `equiv.sh` does not run — it still references the
deleted `rtl/riscv.v` and module `riscv`, and porting it belongs to the outstanding wrapper /
`checks.cfg` ticket, not this one. A hand-built equivalent (`equiv_make` / `equiv_simple` /
`equiv_induct` on gold vs. gated) did not converge in a practical budget, which is unsurprising:
inducting equivalence across a pipelined core containing a 32-cycle sequential divider FSM is a
hard problem, not a misconfiguration.

## Decision

**Merge on the compensating evidence, and record the gap as outstanding rather than closed.**

The compensating evidence:

- Strict elaboration (CI's `proc; opt_clean; check` pipeline) is clean **both with and without**
  `-D RISCV_FORMAL` — zero promoted warnings, zero check problems.
- Both component proofs (`components_decoder`, `components_executor`) pass unchanged. They build
  with `-formal` and never define `RISCV_FORMAL`, so they exercise the un-instrumented core.
- The full `.S` suite is 47/47 under the instrumented build, and was 47/47 under the
  un-instrumented build at `a4662a2`.

What makes this tolerable rather than reckless is the *shape* of the instrumentation, which is
reviewable by inspection in a way a general RTL change would not be. Every `ifdef RISCV_FORMAL`
block in this change is **write-only with respect to the core**: the shadow fields are assigned
from values the core already computes and are read exclusively by `rtl/writeback.v`'s RVFI outputs.
No core signal — no PC, no stall, no valid bit, no register write, no bus request — is a function of
anything under an `ifdef`. That is a structural argument for non-perturbation, and it is checkable
by reading the diff. It is strictly weaker than a proof, and it is not robust to the next change:
the first `ifdef` block that feeds a value *back* into the datapath breaks it silently, and nothing
in CI would notice.

## Consequences

- **This is the standing follow-up.** Porting `equiv.sh` to `littlecpu` is required work in the
  ADR-0006 harness ticket, not optional cleanup, and its result is the thing that closes this ADR.
  If the ported script also fails to converge, that is itself a finding to record — the answer is
  then likely a bounded `equiv` (SAT-based miter to a fixed depth) or blackboxing the divider, and
  either of those is a decision to write down rather than a switch to flip.
- Until then, "RVFI costs no LUTs and changes no behaviour" is a claim resting on `ifdef` discipline
  and reviewer inspection. Reviewers of any future RVFI change carry that burden explicitly: verify
  by reading that no `ifdef`'d value reaches a non-`ifdef`'d signal.
- CLAUDE.md's M2 line should not be read as closed until the ladder port lands. JEF-628 cleared the
  blocker; it did not reach the milestone.
