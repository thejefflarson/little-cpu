# ADR-0020: The RVFI non-perturbation guarantee is argued, not proven

**Status:** Accepted · 2026-07-28 · *Supplements ADR-0006 · **Consequences rewritten by
[ADR-0047](0047-non-perturbation-is-proved-structurally-and-equiv-sh-is-retired.md)**, which retired
`formal/equiv.sh` and proved the guarantee structurally instead. The Context and Decision below
stand as written; the Consequences section is the part that has moved.*

## Context

ADR-0006 kept `formal/equiv.sh` deliberately, and named its job precisely: it proves that **RVFI
instrumentation does not change core behaviour** — gold is the plain build, gate is the
`RISCV_FORMAL` build with the `rvfi_*` ports deleted. The ADR called it out as earning its keep
"with large `ifdef` shadow payloads landing in the structs", which is exactly what `b2dafcc` landed:
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

*Rewritten by ADR-0047. What this section said — that `equiv.sh` was the standing follow-up, that a
bounded miter or blackboxing the divider was the likely answer, and that reviewers carried the burden
in the meantime — was written before anyone measured `equiv.sh`. All three are wrong, and the
measurement is in ADR-0047. The original text is in this file's git history; it is replaced rather
than annotated because leaving a live instruction to reviewers that a machine now performs is how a
manual step outlives its need.*

- **The burden is on a gate, not on reviewers.** `make -C formal nonperturbation`
  (`formal/check-nonperturbation.py`, ADR-0047) decides the property this ADR names: it builds the
  `-D RISCV_FORMAL` design with the `rvfi_*` ports deleted, sweeps everything that only fed them, and
  requires the result to be structurally identical to the plain build — same cell histogram, same
  name-independent connectivity fingerprint. It runs on every pull request in about nine seconds, and
  both of its failure directions were demonstrated on real mutations before it landed, including one
  worth only 11 cells.
- **It is strictly weaker than sequential equivalence, and that is the correct trade.** It proves the
  instrumentation is *unread*; it does not prove two arbitrary designs behave alike. For this
  obligation that is the whole property, and it is decidable where the miter was not.
- **`formal/equiv.sh` is deleted** — with `formal/Makefile`'s `equiv` target and the nightly step, in
  the same change. The route this section used to propose (bounded miter, blackboxed divider) is
  measured and dead: the failure is `equiv_make`'s name-based matching, which pairs almost nothing
  across the two builds, so no budget and no blackbox reaches it.
- **Keep writing to the textual rule anyway** — no `ifdef`'d value reaching a non-`ifdef`'d signal.
  The gate enforces the *property* (the netlist is unchanged), which a pure alias could satisfy while
  breaking the rule. The rule is the habit that keeps the property true.
- **Do not weaken the check to make it pass.** That instruction was in this ADR before the instrument
  changed and it survives the change of instrument. A red is a measurement.
