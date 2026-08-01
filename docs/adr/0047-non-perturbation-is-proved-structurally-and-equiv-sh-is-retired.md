# ADR-0047: The RVFI non-perturbation guarantee is proved structurally, and `equiv.sh` is retired

**Status:** Accepted · 2026-08-01 · *Supersedes `formal/equiv.sh`. Amends
[ADR-0020](0020-the-rvfi-non-perturbation-guarantee-is-argued-not-proven.md). Discharges
[ADR-0037](0037-an-empty-baseline-is-not-m2.md)'s M2 term 4 via the route
[ADR-0045](0045-two-m2-terms-close-by-amendment-and-one-was-already-met.md) chose. Supplements
[ADR-0006](0006-port-the-wave-0-formal-harness.md).*

## Context

ADR-0006 kept an obligation: the `ifdef RISCV_FORMAL` shadow payload threaded through
`rtl/structs.v` must not change what the core does. ADR-0020 recorded that obligation as **argued,
not proven**, and put the burden on people:

> Reviewers of any future RVFI change carry that burden explicitly: verify by reading that no
> `ifdef`'d value reaches a non-`ifdef`'d signal.

That has been the state since `b2dafcc`, and the instrumentation has grown under it — `6309b3e`
added `rvfi_shadow.trap`, `e4f5250` added `operand_stall`, `rtl/csrs.v` added the CSR shadow fields.

`formal/equiv.sh` was supposed to mechanise it and never did. It sat in `formal-nightly.yml` behind
`continue-on-error: true` and `timeout 3600`, which is the worst possible resting place for a check
that has never produced a verdict: **"it's on the nightly" reads as coverage.** A suppressed red on
the repo's only mechanical statement about instrumentation perturbation has been indistinguishable
from a working control for months. This repo has now shipped that shape five times — ADR-0019,
ADR-0033, ADR-0035, ADR-0037, ADR-0040 — and this is the sixth.

### The measurement, and why more solver time cannot fix it

Run at the current RTL against a 900 s bound, `yosys 0.67+post`:

| stage | result |
|---|---|
| `equiv_make` | **495** unproven `$equiv` cells |
| `equiv_simple` | proves **36**, leaving **459** |
| `equiv_induct` | fails the induction step at **every** extension, then degrades to per-cell proofs, all failing. Past a 900 s bound it was still grinding through them and was abandoned without a verdict |

The induction step does not approach closure — it diverges:

```
Proving induction step 1. (1200950 clauses over  429312 variables)  failed
Proving induction step 2. (1864751 clauses over  664549 variables)  failed
Proving induction step 3. (2528552 clauses over  899786 variables)  failed
Proving induction step 4. (3192353 clauses over 1135023 variables)  failed
Proof for induction step failed. Trying to prove individual $equiv from workset.
```

That is **+663,800 clauses and +235,237 variables per step**, flat, with no sign of the invariant
strengthening. (ADR-0045 recorded 494 / 35 / 459 and the same ~660k slope from an earlier tree; the
residual 459 is identical and the difference in the first two figures is RTL drift since, not
disagreement.)

The unproven set is `mem_addr[2..31]`, `accessor.pending_valid`, `accessor.pending_rd`,
`executor.mul_div_y`/`mul_div_x`, `regfile.wdata`, `accessor.write_request` — **essentially the whole
datapath.** The cause is visible in `equiv_make`'s own output: it matches **by name**, and the gate
build (`-D RISCV_FORMAL`, `delete -port`, `opt`) optimises to a differently-named netlist, so almost
nothing pairs. `equiv_induct` is then being asked to prove sequential equivalence of two
near-arbitrary FSMs from scratch, one of which contains a 32-cycle sequential divider.

**This diagnosis kills ADR-0020's own proposed remedy.** ADR-0020 suggested blackboxing the divider
or falling back to a bounded miter. Neither helps: the problem is matching, not the divider, and a
bounded miter over an unmatched netlist is the same problem with a smaller budget. Note also that
ADR-0040's standing warning about bespoke yosys scripts losing clock polarity was **corrected in
`cdb53cb`** and never applied here — `equiv.sh` has no backend and was never exposed to it. It is
not why this failed, and the corrected claim must not be re-inherited.

### The property is structural, and the miter was the wrong instrument for it

ADR-0020's sentence — *no `ifdef`'d value reaches a non-`ifdef`'d signal* — is not a statement about
traces. It is a statement about the netlist. If the instrumentation is write-only with respect to the
core, then deleting the `rvfi_*` ports makes every shadow register and every gate feeding one
unreachable, a pure fanout sweep removes all of it, and what is left **is** the plain build.

## Decision

**1. `formal/equiv.sh` is deleted**, together with `formal/Makefile`'s `equiv` target and
`formal-nightly.yml`'s step. All three go in one change, so no orphan remains. It is not fixed,
because there is nothing here to fix: name-based matching cannot pair these two netlists, and this
repo does not keep a check whose only possible outcomes are "no verdict" and "no verdict, slower".

**2. `formal/check-nonperturbation.py` replaces it**, behind `make -C formal nonperturbation` and a
new `nonperturbation` CI job. It builds three netlists in one `yosys` invocation and compares two of
them:

* **gold** — the plain build.
* **instrumented** — the `-D RISCV_FORMAL` build, ports intact. Used only as a control.
* **gate** — the `-D RISCV_FORMAL` build with `delete -port littlecpu/rvfi_*` and the sweep.

All three are normalised identically: `proc; flatten; memory_map; simplemap`, then `opt_clean -purge`
to a fixpoint. Two details of that are load-bearing and were both found by measurement:

* **`simplemap` is what makes the sweep possible at all.** A packed struct is one wide `$sdffe` —
  real fields and shadow fields in the *same cell* — and `opt_clean` cannot remove half a cell. Without
  splitting to bit-level gates first, gold and gate differ by **47 cells** on a core that is perfectly
  well behaved, and the check is a false red on day one.
* **There is no `opt` anywhere.** `prep` would run one, on a gate design that still contains the shadow
  logic, and cross-talk between optimising that logic and optimising the core's would land here as a
  false red. Removing dead logic by pure fanout sweeping has no such coupling.

Names are never compared — they are exactly what defeated `equiv_make`. Two name-free comparisons are:

1. the **cell histogram by type**, exactly; and
2. a **canonical connectivity fingerprint**: Weisfeiler-Leman colour refinement over the bipartite
   cell/net-bit graph, seeded only from cell types, cell parameters, port names, bit positions, module
   port names and constant bits. Isomorphic netlists give identical colour multisets.

**3. This is strictly weaker than sequential equivalence and says so at the site.** It proves the
instrumentation is **unread**, not that two arbitrary designs behave alike. Two designs computing the
same function by different structures pass a miter and fail this — which is correct here, because the
subject is a preprocessor macro, not a redesign. In the one direction that matters it is *stronger*
than the bounded miter: it fails the moment an `ifdef`'d value reaches a real signal, not merely when
that value changes an output within some depth.

## Both failure directions were demonstrated on real runs

Green on `main`, 8.8 s wall, exit 0:

```
  fingerprint self-test:             notices a one-mux rewire
  gold rvfi_* ports:                 0 (want 0)
  instrumented rvfi_* ports:         33 (want > 0)
  gate rvfi_* ports after delete:    0 (want 0)
  gold cells:                        23872
  instrumented cells:                40733 (want > gold, by a wide margin)
  gate cells after the sweep:        23872
  cell histogram:                    identical (23872 cells)
  connectivity fingerprint:          identical (13895 cell colours, 14056 net colours, 8 WL rounds)
RVFI NON-PERTURBATION: PASS
```

Red, mutation 1 — an `ifdef`'d shadow value ORed into `rtl/decoder.v`'s global `stall` term
(`stall = ... || (out.rvfi.insn == 32'hdead_beef)`), exit 1:

```
  gate cells after the sweep:        24065
  cell histogram:                    DIFFERS
    $_DFF_P_ 2879 -> 2911 (+32)   $_MUX_ 9487 -> 9583 (+96)   $_NOT_ 306 -> 307 (+1)
    $_OR_    4233 -> 4265 (+32)   $_XOR_ 1199 -> 1231 (+32)
  connectivity fingerprint:          DIFFERS
    cells with a gold-unmatched immediate neighbourhood: 193
RVFI NON-PERTURBATION: FAIL
```

Red, mutation 2 — an `ifdef`'d shadow value gating `rtl/accessor.v`'s `out.valid`
(`out.valid <= !in.rvfi.trap`), exit 1:

```
  gate cells after the sweep:        23883
  cell histogram:                    DIFFERS
    $_DFF_P_ 2879 -> 2881 (+2)   $_MUX_ 9487 -> 9493 (+6)
    $_NOT_    306 ->  308 (+2)   $_OR_  4233 -> 4234 (+1)
  connectivity fingerprint:          DIFFERS
    cells with a gold-unmatched immediate neighbourhood: 12
RVFI NON-PERTURBATION: FAIL
```

**The 11-cell case is the one worth remembering.** A gate that only catches large perturbations is
not much of a gate; this one resolves a single `valid` bit acquiring one shadow term. Both mutations
were reverted; no `rtl/` file changed in this ADR's commit.

## The controls, and why they exist

A structural gate's characteristic failure is comparing a design with itself — a `-D` that stopped
taking, a `delete -port` pattern that stopped matching, a normalisation that swept the difference
before the comparison saw it. Each of those is green and worthless. So the script asserts, before it
believes any comparison:

* gold declares **no** `rvfi_*` port; the instrumented build declares some (33 today);
* after `delete -port`, the gate declares none;
* the instrumented build has **strictly more** cells than gold by a wide margin (40733 vs 23872 — the
  floor is +200, deliberately loose, this is not a size ratchet);
* both designs' sweeps reached a **fixpoint** — zero cells driving nothing.

And one more, because the fingerprint is otherwise a comparison that has never been observed to fail:
**a self-test.** The histogram's bite is easy to demonstrate on real RTL, but no mutation of this core
produces a constant-cell-count rewire, so the fingerprint alone has nothing to prove itself against.
The script therefore takes gold, swaps the `A` and `B` inputs of exactly one `$_MUX_`, and requires
that the histogram is unchanged and the fingerprint is not. Same shape as `test/exec_tb.v`'s
`ref_selftest`, for the same reason: an oracle that cannot fail is not an oracle.

## Consequences

* **ADR-0037's M2 term 4 is discharged**, by its own "or ... proven another way" clause and along the
  route ADR-0045 chose. The term's wording is amended in place to name this mechanism. Five of six
  M2 terms are now closed; term 6's CSR half remains.
* **ADR-0020's reviewer burden is now carried by a gate**, and ADR-0020's consequences paragraph is
  rewritten to say so rather than leaving a live instruction to reviewers that a machine now performs.
  **The habit is still worth keeping** — see the residual below.
* **The gate is not in branch protection.** Adding it to main's required set is a repository-settings
  change for a human to make deliberately, exactly as ADR-0036 says of `lint` and as ci.yml says of
  `formal`. Landing it unrequired is the correct state; it is the maintainer's call after a few runs.
* **Cost:** ~9 s wall, `yosys` and `python3` only — no `sby`, no solver, no riscv-formal clone. That
  is what makes it a PR gate rather than a nightly, and it is why deleting the nightly step costs
  nothing.

### What this does not cover — read before trusting it further than it goes

* **It is scoped to `RISCV_FORMAL`.** `RISCV_FORMAL_ALTOPS` deliberately *does* change
  `rtl/executor.v`'s arithmetic; that is ADR-0010's subject, not this one, and nothing here says a
  word about it.
* **It proves the property, not the coding rule.** If an `ifdef`'d name were a pure alias of a real
  signal — zero cells — and a real signal read that alias, the netlist would be unchanged and this
  would pass. Behaviourally that is non-perturbing and the pass is correct; but ADR-0020's *textual*
  rule is stricter than what a netlist can see. Keep writing to the textual rule: it is the habit that
  keeps the property true, and it is checkable by eye in a way this gate's green does not replace.
* **It compares what yosys builds.** It is silent about any other frontend, and about the behaviour of
  the instrumented build with its ports intact — which is the build the whole formal ladder actually
  runs against.
* **A red here is not automatically a perturbation.** It is a difference. The first question is always
  whether a normalisation assumption broke (a new memory shape `memory_map` handles differently, a
  cell type `simplemap` does not split); the script prints the histogram delta and localises to cells
  whose *immediate* neighbourhood has no counterpart in gold, which is where to start. **Do not relax
  the comparison to make it pass** — ADR-0020 forbids weakening this check by name, and the reason
  survives the change of instrument.
