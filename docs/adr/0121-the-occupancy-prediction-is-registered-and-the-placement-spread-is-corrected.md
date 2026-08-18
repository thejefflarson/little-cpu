# ADR-0121: the occupancy prediction is registered before the sweeps, and the placement spread is corrected

**Status:** Registered · 2026-08-18 · *Pre-registration. No measurement is in this file yet, and the
commit that carries it is dated before the first placement so a reader can check that order in `git
log`. The result is appended to this same file by a later commit, whichever way it goes.*

## The question this registers

[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md) leaves one thing
open in as many words: **whether occupancy sets the placement tail at 89%.** Every SoC-side area
argument depends on the answer, because area is only a period lever if it is.

The prior is two measurements, both nulls and both in the removal direction:
[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md) at −169 placed cells over eight
seeds (+0.3% of median, −0.5% at the worst) and
[ADR-0090](0090-the-executor-carries-half-the-registers-it-had.md) at −400 with the period a null
again. ADR-0088 also ballasted the design from 77% to 95% occupancy for no change in period at all —
but at one placement per point, which is the instrument this repo has since learned not to trust:
[ADR-0113](0113-the-successor-pair-table-is-declined-and-its-price-is-the-tail.md) is a candidate
whose whole cost was in the tail, invisible at the median and at eight seeds.

Pointing the other way is [ADR-0093](0093-the-compressed-successor-is-decoded-and-the-compiled-workload-is-what-moves.md):
a candidate declined on a lost placement passed after ~490 cells came out of the SoC. That ADR
declines to claim congestion was the cause, and ADR-0111 repeats the refusal. Neither is evidence;
both are the reason this is worth one experiment.

## Registered prediction: null

**Occupancy does not set the placement tail at 89%.** Concretely, for each arm below:

- the median of the per-seed period deltas sits inside the ~3.6% edit-churn band, or moves in a
  direction the cell dose does not predict;
- the sign test over sixteen paired seeds does not separate the deltas from a coin at p ≤ 0.05;
- the worst-of-sixteen moves by less than one churn band in the direction the dose predicts.

**The strong form, and the one worth being wrong about:** the *sign* of a period move will not track
the sign of the cell dose. A removal that speeds the tail and an addition that also speeds the tail
are the same result — the dose is not what moved it.

## The falsifier, fixed before any number exists

The occupancy hypothesis predicts a signed effect: **fewer cells → faster, more cells → slower.**
The null is falsified **for an arm** when both of these hold on that arm's paired sweep:

1. **at least 13 of the 16 per-seed deltas carry the sign the hypothesis predicts.** Thirteen is the
   two-sided sign test at p = 0.021; twelve is p = 0.077 and is not enough. This is the same rule
   ADR-0117 and ADR-0118 already grade candidates by, written down here before the data rather than
   after.
2. **the worst-of-sixteen moves in that same direction by more than 3.6%**, one edit-churn band.

Either alone is not enough, and the reason is in the record. Condition 1 alone can be a path change
that a small dose came packaged with — ADR-0117's `instr_error` respelling is eleven cells and 13 of
16 seeds faster, and it is a depth result, not an area one. Condition 2 alone is one placement, which
is the sample size this repo has been wrong at twice.

**The interaction, which is the sharper form.** If occupancy sets the tail at 89%, then a removal
must be worth *more* at 95% than at 89%. The stacked arm tests exactly that: the hypothesis predicts
the harvest's paired deltas are more negative under ballast than without it, by more than a churn
band at the median. A flat interaction is a null and will be reported as one.

**Worst-of-sixteen is reported per arm because `SOC_MIN_MHZ` grades it.** It is the gate, not the
analysis: worst-of-N against worst-of-N discards the pairing, and against a spread this wide it has
almost no power. The analysis is the paired column.

## Design

**One pinned base for every arm: `c51ebba`**, which is `origin/main` at registration. Each arm is
measured against *that* base, never against another arm's tree.

**Three arms, sixteen seeds each, paired by seed** — `default 1 … 15`, `soc/baseline_sweep.sh`,
`SOC_PROG=datainit.c`:

| arm | what it is | dose direction |
|---|---|---|
| **A — the harvest** | the AMO result mux rebuilt as a per-bit truth table, `rtl/accessor.v` | cells **down** |
| **B — the ballast** | free-running LFSRs that nothing reads, `(* keep *)`, wired to nothing | cells **up** |
| **A+B — stacked** | both at once | the interaction |

**The same toolchain on every side of every comparison**, named in the result: the cached OSS CAD
Suite `20260811` on this machine — yosys 0.68+48 (`ff5817c34`), nextpnr-0.11-1-g62e659ed, `icetime`
from the same suite. `soc/baseline_sweep.sh` stamps each sweep with all three and with its base, and
`soc/baseline_summary.py` refuses to subtract two whose stamps disagree.

All four sweeps run with `SOC_MIN_MHZ` overridden to zero on the command line, so a placement under
the requirement produces a row instead of stopping the sweep — the ballast arm may well produce one,
and it is a result and not an error. The requirement itself does not move, and the worst-of-sixteen
is read against 12.00 MHz by hand in the result.

**Nothing from any arm lands.** Arm A is another ticket's change, applied here as a probe against the
pinned base; arm B is a scratch-only ballast module that exists to occupy cells and is thrown away,
the way ADR-0118's variants were. No `rtl/`, no test and no baseline changes under this record.

## What will not be claimed, whatever the numbers say

**The arms differ in *where* the cells are as well as in how many.** Arm A's are the accessor's;
arm B's are wherever the placer puts a register chain that reads nothing. A difference between the
two arms is therefore confounded with locality and will not be read as a dose-response curve. The
strongest claim available from this design is **falsification of the null in at least one arm**, and
the strongest claim available from a null is that the effect is smaller than this instrument's own
noise — which is exactly the number the second half of this ADR corrects.

**Arm A is not blind.** Its own ticket has already swept it at sixteen seeds on an earlier base and
reports −54 placed cells at every placement, a median of −2.67% and a worst placement moving
12.21 → 12.27 MHz. That is disclosed rather than worked around: what this experiment adds for arm A
is the paired column against *this* base and its place in a two-signed design. **Arm B and the
stacked arm are blind** — nobody has measured either.

## The design was amended, and this is the amendment rather than a rewrite

**Registered first, executed last** was the point of this record, and the plan it was written against
did not survive contact with the queue. Both are here because a pre-registration edited once the arms
are known is worth nothing.

**As originally designed:** two area harvests, both removals, each swept against a pinned base, plus
a stacked arm to test additivity.

**What happened:** one harvest exists and one does not. The AMO mux harvest was built and is arm A.
The second candidate — producing `mtip` incrementally instead of by a fresh 64-bit compare — was
built, measured at **+138 placed cells** and declined (ADR-0118); its variants lived in a scratch
clone and were thrown away, so there is no patch to re-apply and re-deriving one from the ADR's
description would be authoring RTL under a ticket that must not.

**As amended:** the second arm becomes a **ballast** — cells with no function, no connection to the
datapath and therefore no path of their own. That is a strictly better probe of *this* hypothesis
than a second removal would have been, and it is the reason the amendment is not merely damage:

- it is **the opposite sign**, so the design tests whether the tail tracks the dose in both
  directions rather than only in the direction two nulls already cover;
- it is **the only dose with no locality confound at all** — the one thing the paragraph above says
  this design cannot otherwise claim;
- and the stack stops being an additivity test, which two harvests would have needed, and becomes the
  **interaction** test registered above, which is the hypothesis stated exactly.

What is lost is that additivity of two harvests goes unmeasured. Nothing here answers it, and this
ADR will not be cited as if it did.

## Everything else in this record is a correction, not a measurement

The instrument's own noise figure is wrong in five places in this repo, and the correction does not
depend on any sweep above: `make soc-timing`'s **placement spread is not 1–2%**. That figure is
[ADR-0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md)'s, taken over four
placements of two designs in 2026-06, and every sixteen-seed table taken since is several times
wider. The corrected figure, the sites that restate it, and the go/no-go convention that moves with
it are written up with the result.
