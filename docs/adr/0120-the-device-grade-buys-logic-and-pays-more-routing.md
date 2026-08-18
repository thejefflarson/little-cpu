# ADR-0120: Naming the part for the mapper buys logic and pays more routing

**Status:** Accepted · 2026-08-18 · *Re-takes
[ADR-0080](0080-twenty-four-megahertz-is-not-reachable-on-the-up5k.md)'s dead-end row for
`-abc9 -device u` at 64 paired placements where that row was one placement, confirms it, and
corrects its reasoning. Adds nothing to
[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md)'s balance. Amends no
commitment.*

## Context

`synth_ice40`'s abc9 pass maps LUTs against a **device grade**, and the SoC's synthesis line names
none — so every placement this project has measured on an up5k was mapped against **hx1k** delays,
which is the flag's default and nobody's decision. The numbers really do differ: `SB_LUT4`'s
`I0 => O` is 449 ps on hx against 1245 on up5k and `SB_CARRY`'s `CI => CO` is 126 against 278, so a
LUT level is worth 3.6 carry hops in the numbers abc9 was given and 4.5 in the part's own. A mapper
told the wrong part under-buys the carry chain.

The grade also decides which `ICE40_{HX,LP,U}` define `cells_sim.v` is read with, and **all 28 of
that file's `ICE40_U` blocks open a `specify`** — timing annotation, no behaviour. So a result here
can only ever be about mapping.

It was reopened because a sixteen-seed paired sweep on `16c5cad` read −2.71% of median period with
14 of 16 seeds faster and a better worst placement, which would have made it the first lever
measured here to move the tail rather than the middle. That sweep's candidate arm is stamped
`dirty: yes`, and a dirty stamp means its base line names no tree — the documented reason
`soc/baseline_summary.py` refuses such a subtraction, and the reason this was re-taken on a clean
stamped pair before being banked.

## Decision — the shipping line keeps the default, and the grade is declined on its own measurement

Built as one word, swept **64 placements a side, paired by seed**, both arms clean and stamped:
base `ac7ee91`, candidate `3334de5`, one toolchain (yosys 0.68+48 `ff5817c34`, nextpnr-0.11-1
`g62e659ed`, oss-cad-suite 20260811 icetime), `datainit.c` into 2048 ROM words. The only provenance
field that differs between the arms is `base`, and the diff between those two commits is the word.

Read by this repo's convention — the median of per-seed deltas with a sign test — it is a null at
every sample size, **including the sixteen seeds that produced the number above**:

| seeds | median Δ | sign test | worst of N |
|---|---|---|---|
| first 16 | +0.75% | 7 faster / 9 slower, p = 0.80 | 83.10 → 80.00 ns |
| first 32 | −0.17% | 17 / 15, p = 0.86 | 83.10 → 81.42 ns |
| **all 64** | **+0.50%** | **26 / 38, p = 0.17** | **83.10 → 81.91 ns** (12.03 → 12.21 MHz) |

Worst-of-N is the gate, so the tail gets its own test rather than an eyeball. The arms are paired,
so the null is exchangeability within a pair: flip a random subset of the 64 pairs, recompute,
200 000 times.

| statistic | base | `-device u` | Δ | p |
|---|---|---|---|---|
| max | 83.10 ns | 81.91 ns | −1.19 ns | **1.000** |
| p95 | 79.38 | 80.59 | +1.21 | 0.065 |
| p90 | 79.06 | 80.11 | +1.05 | **0.006** |
| p75 | 78.20 | 79.32 | +1.12 | **0.006** |
| median | 77.32 | 78.20 | +0.89 | 0.019 |
| spread | 12.49% | 9.80% | −2.70 pts | 0.52 |
| stdev | 1.42 ns | 1.67 ns | +0.25 | 0.23 |

**The one number that favours the grade is the one with p = 1.000.** The better worst placement is
what flipping labels produces essentially always; the two statistics that do move — p75 and p90, both
about 1.1 ns slower — survive a Bonferroni correction over the seven tested, and the median's +0.89 ns
does not. Nothing here is a win anywhere, and the direction of what evidence there is points the
wrong way.

Both arms hold the requirement at **64 of 64 placements**, so this decision is about a lever that
did not pay, not about a gate.

### Why: it optimises the third of the path it can see

The median placement's split says it plainly — the grade buys logic and pays routing, and routing is
where the period lives:

| | median logic | median routing | routing share |
|---|---|---|---|
| base | 24.56 ns | 52.45 ns | 68.2% |
| `-device u` | 24.23 ns | **53.86 ns** | 68.9% |

−0.33 ns of the delay abc9 models, +1.41 ns of the delay it does not. That is the same shape as
`nextpnr --opt-timing` and as ADR-0097's flat mux: **a pre-placement optimiser given a better model
of logic still cannot see the two thirds of this path that is interconnect.**

At the netlist level the change is almost nothing: **+13 `SB_LUT4`** (4238 → 4251), `SB_CARRY`
identical at 688, every flop, DSP, block RAM and SPRAM count identical. `SB_SPRAM256KA` is 2 and
`SB_RAM40_4K` is 20 on both arms, as the census demands. Three runs of each recipe hash to one
digest, so the netlist is reproducible either way.

### What is not reproducible is the packed cell count

`base` packs to **4676 at all 64 placements**. `-device u` packs to **seven different values between
4692 and 4780** over its 64 — an 88-cell span, wider than the ±50 churn band, out of one
bit-identical JSON. The mapping leaves nextpnr's LUT/flop pairing more choices and its RNG resolves
them, so the SoC's placed cell count would stop being a number and start being a distribution. Every
future area ceiling on this design would have to be swept rather than read.

### The disagreement with the earlier sweep is not sample size

Sixteen seeds *on this tree* already read null (+0.75%, p = 0.80) — the same sixteen the −2.71% came
from. So the verdict did not change with N; it changed with the tree, and two things are candidates
without either being proved here: `main` moved two commits, one of which took −2.47% of median out of
this very fetch loop (ADR-0117), and the earlier candidate arm was stamped dirty and so names no
tree at all. **A margin is a measurement with a date on it in both directions** — a decline can
expire, and so can a win.

## The tail is a set of cones, and endpoint migration is the tell

Over the 64 base placements, **every** critical path ends in the same bucket — a net yosys named
after the even ROM bank's read data — while the starts spread across six: `imem.rom_odd` 20,
`imem.in_range` 18, `imem.rom_even` 15, `mtimer` 6, `por_done` 3, `riscv.accessor` 2. With the grade
on, 62 of 64 still end in that bucket. Those are net *names* after flatten, which are ancestry rather
than ownership, so what they support is the shape and not an attribution: many different cones, one
destination, and the destination is the fetch address.

The consequence is a class of attempt, not one attempt. When the increment behind that endpoint was
hoisted off the shared tail — three RTL spellings of it, in the decoder alone and plumbed through to
the memory — the sweeps came back null and **the reported endpoint migrated to a `minstret` net at
the same length**. Deleting one cone crowns the next. Those spellings were measured on `16c5cad`
with dirty candidate arms and are recorded here as context rather than as prices; what carries is
that the tail on this design has no single owner to remove.

## Two more levers, declined without re-measurement

**`nextpnr --opt-timing`.** ADR-0080 placed it once at 85.05 ns against 78.51 for the shipping
recipe — **below the 12 MHz requirement** — and a later run took 35 minutes for a single placement
and came out about 10 ns worse on `icetime`'s report. Not re-taken here: two independent runs already
agree in direction, one of them fails the requirement outright, and it is the same shape as the
result above, an optimiser tuned against its own delay model and graded by a different one.

**`soc/compare/` and the `fit` synthesis line are deliberately untouched**, and would be separate
decisions rather than quiet consequences of this one. The comparison harness places both cores and
only means anything while they are synthesised alike; `-device u` on the `fit` line would move a
count `FIT_MAX_LC` is graded against, for a flag this ADR has just declined on the instrument that
can actually see a period.

## Consequences

- **The default stays, and now it is a decision instead of an omission.** The next reader who
  notices that an up5k is being mapped as an hx1k finds this file rather than repeating the sweep.
- **A better delay model for a pre-placement optimiser is not a period lever on this SoC**, and that
  is now measured twice over — the grade here, `--opt-timing` in ADR-0080 — with the mechanism the
  same both times and visible in the split: 68% of this path is routing. Re-take it if that fraction
  ever moves; do not assume it from the flag's description.
- **Sixteen paired placements cannot bank a tail claim.** ADR-0113 recorded eight producing a
  different verdict from sixteen. Here sixteen produced a tail claim that 64 placements and a
  permutation test retire at p = 1.000, and the worst-of-N statistic is exactly where a small sweep
  is least trustworthy, because it is one sample of an extreme.
- **Zero cycles, and checked rather than asserted.** No RTL changed and abc9 mapping is
  equivalence-preserving, so `make cycles` (34 051 cycles, 18 352 retired, CPI 1.86) and
  `make dhrystone` (1 544 344 cycles, 0.757 DMIPS/MHz) are identical on both trees, byte for byte.
  `make fit` reads 3932 on both, its line being untouched.
- **The equivalence claim was graded, not argued.** On the tree carrying the flag:
  `make -C formal nonperturbation`, the four component proofs, the 86 generated checks against
  `EXPECTED_FAIL` and `EXPECTED_CHECKS`, `complete`, `complete_cover`, `make test`, `test-units`,
  `probe-gates`, `mutation-check`, `window-test`, `cosim-suite`, `lint` and `waves` all pass.
- **The toolchain is not pinned**, so this verdict carries its resolved versions and a date. A suite
  bump that changes abc9's mapping invalidates the number here before it invalidates the decision:
  re-sweep, do not re-argue.
