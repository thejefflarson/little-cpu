# ADR-0121: the occupancy prediction is registered before the sweeps, and the placement spread is corrected

**Status:** Accepted · 2026-08-18 · *Pre-registered, then measured: three arms of sixteen placements
each against one pinned base, paired by seed. **Supersedes
[ADR-0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md)'s 1–2%
placement-spread figure**, which fourteen ADRs, six live prose sites and two briefs inherited.
Closes the question [ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md)
left open.
Amends no commitment. Nothing measured here lands: no `rtl/`, no test, no baseline.*

## The verdict, first

**Occupancy does not set the placement tail at 89%.** A dose of **+352 cells — 88.6% to 95.2% —
costs about 2% of the median period and moves the worst of sixteen placements the *other* way.** The
registered falsifier needed both a sign test and a tail move, and got one of the two, so the null
stands as registered: `SOC_MIN_MHZ` grades the tail, the tail did not move with the dose, and **area
is not a margin lever on this part.**

What the same experiment does say, and what ADR-0088's single placement per point could not have
seen, is that occupancy is not *nothing* either: those 352 cells are +2.03% of median with **13 of 16
seeds slower, p = 0.021**. That is inside the ~3.6% edit-churn band, so it is a null by the
instrument's own rule and a real effect by the sign test — which is the honest description of an
effect smaller than the noise this ADR also corrects.

**And the noise is four to nine percent, not one to two.** The base netlist alone spans **76.10 to
83.10 ns over sixteen placements — 9.2%.** The 1–2% in the rulebook was four placements taken in one
sitting; a four-seed spread is not a tighter instrument, it is a shorter look at the same
distribution.

## The pre-registration, reproduced unedited

Everything from here to the end of the amendment is the text commit `364e5de` carried at
**2026-08-18T03:13:58Z**, under the status line it had then — *Registered · 2026-08-18 ·
pre-registration, no measurement in this file yet*. The first placement of the first sweep is stamped
**03:16:22Z** in `soc/baseline_sweep.sh`'s own provenance block. Nothing below has been edited; the
result begins after it.

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

## The pre-registration ends here. What was measured

**Base `c51ebba`, sixteen seeds, one toolchain, one sitting.** `soc/baseline_sweep.sh` with
`SOC_SEEDS='default 1 … 15'`, `SOC_PROG=datainit.c`, `SOC_MIN_MHZ=0` on every side so a placement
under the requirement produces a row rather than stopping a sweep; the cached OSS CAD Suite
`20260811` — yosys 0.68+48 (`ff5817c34`), nextpnr-0.11-1-g62e659ed, `icetime` from the same suite.
Each arm was committed in a scratch clone so its stamp reads a clean tree, which makes each pair
differ in exactly one recorded field — `base` — and `soc/baseline_summary.py` was passed
`--allow-mismatch` with that as the only difference. The four sweeps ran concurrently: a placement is
a function of the netlist and the seed, so sharing the machine moves wall clock and no number.

**The doses, from synthesis and from the placement:**

| arm | what it is | `SB_LUT4` | flops | placed LC | occupancy |
|---|---|---|---|---|---|
| base | `c51ebba` | 4238 | 1051 | **4676** | 88.6% |
| **A** | the AMO result mux as a per-bit truth table | 4205 | 1051 | **4639** (−37) | 87.9% |
| **B** | eight 40-bit LFSRs reading nothing, `(* keep *)` | 4279 | 1371 | **5028** (+352) | 95.2% |
| **A+B** | both | 4197 | 1371 | **4947** (+271) | 93.7% |

**The ballast, in full**, appended to `rtl/littlesoc.v` in a scratch clone and committed nowhere:
eight generate-loop instances of `(* keep *) logic [39:0] lfsr`, clocked by the SoC's `clk`, reset to
`40'h1 << i`, stepping `{lfsr[38:0], lfsr[39]^lfsr[37]^lfsr[20]^lfsr[18]}`. `keep` is the whole
mechanism: without it yosys deletes logic that drives nothing, and with it the placer has to find
room for all of it.

**The ballast changes occupancy and nothing else, and the census says so**: against the base it is
+320 flops and +41 `SB_LUT4` with `SB_CARRY`, `SB_DFF`, `SB_DFFE`, `SB_DFFESR`, `ICESTORM_RAM` and
`ICESTORM_DSP` identical cell for cell. The design's own mapping is untouched. Arm A's dose is −37
here rather than the −54 its own record quotes, because the base moved 4693 → 4676 under it while the
arm's netlist stayed at 4639 — the ceiling travelled and the base did not.

**Even a disconnected addition is not additive.** A+B places 44 cells below A plus B's doses, and
ABC maps the design 49 `SB_LUT4` smaller there than in A alone, on RTL that differs by a module
nothing reads. That is the ±50 band arriving from a direction it has no business arriving from, and
it is why the stack is measured rather than computed.

## The three arms, paired by seed

Nanoseconds. The dose in cells is in each header.

| seed | base | A −37 | Δ | B +352 | Δ | A+B +271 | Δ |
|---|---|---|---|---|---|---|---|
| default | 76.40 | 78.69 | +3.00% | 77.61 | +1.58% | 76.61 | +0.27% |
| 1 | 77.22 | 77.12 | −0.13% | **82.18** | +6.42% | 78.35 | +1.46% |
| 2 | 77.34 | 77.67 | +0.43% | 78.81 | +1.90% | 77.58 | +0.31% |
| 3 | 76.61 | 77.85 | +1.62% | 77.73 | +1.46% | 78.07 | +1.91% |
| 4 | 77.41 | 76.00 | −1.82% | 79.95 | +3.28% | 78.84 | +1.85% |
| 5 | 77.30 | 80.84 | +4.58% | 78.12 | +1.06% | 77.08 | −0.28% |
| 6 | **83.10** | 78.85 | −5.11% | 77.14 | −7.17% | 82.02 | −1.30% |
| 7 | 79.38 | 78.45 | −1.17% | 81.79 | +3.04% | 76.69 | −3.39% |
| 8 | 76.72 | **81.53** | +6.27% | 80.57 | +5.02% | 79.30 | +3.36% |
| 9 | 76.60 | 77.58 | +1.28% | 78.26 | +2.17% | 77.21 | +0.80% |
| 10 | 77.61 | 77.47 | −0.18% | 79.82 | +2.85% | 76.42 | −1.53% |
| 11 | 76.10 | 77.98 | +2.47% | 78.63 | +3.32% | 82.60 | +8.54% |
| 12 | 77.21 | 77.40 | +0.25% | 76.42 | −1.02% | **84.39** | +9.30% |
| 13 | 77.96 | 77.77 | −0.24% | 79.21 | +1.60% | 81.74 | +4.85% |
| 14 | 77.22 | 77.74 | +0.67% | 82.07 | +6.28% | 82.69 | +7.08% |
| 15 | 81.13 | 79.51 | −2.00% | 80.56 | −0.70% | 78.16 | −3.66% |

| | worst of 16 | median | best | best-to-worst |
|---|---|---|---|---|
| base | 83.10 ns, **12.03 MHz** | 77.26 ns, 12.94 MHz | 76.10 ns | **9.2%** |
| A | 81.53 ns, **12.27 MHz** | 77.81 ns, 12.85 MHz | 76.00 ns | 7.3% |
| B | 82.18 ns, **12.17 MHz** | 79.01 ns, 12.66 MHz | 76.42 ns | 7.5% |
| A+B | 84.39 ns, **11.85 MHz** | 78.25 ns, 12.78 MHz | 76.42 ns | 10.4% |

| arm | median Δ | sign test | in the hypothesis's direction | worst-of-16 Δ | falsified? |
|---|---|---|---|---|---|
| A, −37 cells | **+0.34%** | 7 faster / 9 slower, p = 0.804 | 7 of 16 | −1.89% | **no** |
| B, +352 cells | **+2.03%** | 3 faster / **13 slower, p = 0.021** | 13 of 16 | **−1.11%**, the wrong way | **no** |
| A+B, +271 cells | +1.13% | 5 faster / 11 slower, p = 0.210 | 11 of 16 | +1.55% | **no** |
| interaction: A under ballast against A alone | −1.58 points | 10 faster / 6 slower, p = 0.454 | 10 of 16 | — | **no** |

**Arm B is the whole result.** It meets the falsifier's first condition and fails its second, and the
second is the one that was about the tail: 352 cells and 6.6 points of occupancy make thirteen of
sixteen placements slower by a median 2.03%, and the worst placement gets **faster**. Registering the
conjunction before the numbers is what stops that from being written up either as "occupancy costs
2%" or as "occupancy is free", both of which this table supports if you pick a column.

**Arm A is a null twice over.** Its own record swept it at sixteen seeds on an earlier base and read
a median of −2.67%; here it reads +0.34%. Both are inside the churn band, both are coins by the sign
test, and the 3.0 points between them is the instrument. A median inside the band is a null in both
directions and **does not reproduce** — which is a stronger statement of the existing rule than the
rule had.

**The interaction is flat.** If occupancy set the tail, a removal would be worth more at 95% than at
89%; the harvest's paired deltas are 1.58 points better under ballast at the median, on ten of
sixteen seeds, p = 0.454. That is a coin.

**One placement of A+B misses the requirement, at 11.85 MHz.** It is a probe and not a design, and
its miss is worth exactly one sentence: the stacked arm is also the arm with the widest spread
(10.4%), which is what a tail looks like when nothing systematic is moving it.

## Every dose this repo has measured, and what its tail did

Each row is a within-tree paired sweep on its own base and toolchain. **They are not one curve** —
the trees differ, the toolchains differ, and this repo's own tooling refuses to subtract across
stamps. They are seven independent answers to one question.

| dose, placed cells | where the cells are | median Δ | tail (worst of N) | record |
|---|---|---|---|---|
| **−400** | the executor's registers, negator, partial-product row | null | null | ADR-0090 |
| **−169** | eleven facts across five modules, 8 seeds | +0.3% | −0.5% | ADR-0088 |
| **−37** | the AMO result mux (arm A) | +0.34% | −1.89% | here |
| **+90** | the learned successor-pair table | +1.17% | **+3.89%**, a placement lost | ADR-0113 |
| **+108** | `synth_ice40 -device u`, a synthesis flag | not re-measured here | **−1.33 ns** | a sibling change in flight, reported not reproduced |
| **+138** | `mtip` from registered partial compares | −1.01% | −1.7% (12.21 → 12.42 MHz) | ADR-0118 |
| **+352** | nothing — a ballast (arm B) | **+2.03%** | −1.11% | here |

**The sign of the dose predicts neither column.** Cells came out for a slower median twice and cells
went in for a faster tail three times. The one row whose tail moved by more than a churn band is
+90 cells, and ADR-0113 attributes it to variance rather than to size — the same +90 is a null at the
median. Whatever sets this design's tail, it is not how full the part is.

## The limits of this result, before anyone asks

**The arms differ in where the cells are as well as how many.** Arm A's come out of the accessor,
which is not the fetch loop; arm B's go wherever the placer puts a register chain. So a difference
*between* the arms is confounded with locality and is not read here as a dose-response curve. The
claim is per-arm: the null was not falsified in any of the three.

**The ballast is not free of the design.** Its 320 flops are sinks on the clock and reset nets, whose
fanouts go 1093 → 1413 and 800 → 1120; both are promoted to global buffers on both sides, so the
extra load rides dedicated routing rather than the fabric, but "no connection to the datapath" means
no *logic* connection and not no interaction at all.

**A ballast is not a harvest.** It says what occupancy alone costs. A real removal takes a path with
it, which is the reason ADR-0117's eleven cells bought 2.47% of median and ADR-0097's −103 cost 9.1%
of it. Nothing here says a removal cannot buy period; it says the *occupancy* it buys will not.

**One dose is one shape.** 352 cells of shift register is one way to fill a part. A ballast of wide
combinational cones with high fanout might congest differently, and that is not measured.

**Additivity of two harvests goes unmeasured**, because the second harvest was declined and thrown
away before this ran. The amendment above is where that is recorded, and this ADR must not be cited
as if the stacked arm answered it.

## The placement spread is 4–9%, and 1–2% was four placements

This half needs none of the sweeps above. It is a correction to the instrument's own noise figure,
and the evidence for it is every table this repo has taken since the figure was written down.

**Where 1–2% came from.** ADR-0057 (2026-08-02) placed the baseline and three variants at four seeds
each and read "placement noise on one netlist is about 1–2% for the baseline and the proposal and
about 6% for the read-mux variant". The registration above dates it 2026-06 and calls it two
designs; both are wrong, and both are left as written, because that block is a record of what was
believed before the sweeps and not a source of fact.

The baseline row is 87.43 · 87.55 · 87.94 · 88.51 ns — 1.2%. The 6% was in the same table and did not
travel; the 1–2% did, into fourteen ADRs, six live prose sites and two briefs.

**It was contradicted the next day and has been ever since.** Best-to-worst on an *unchanged*
netlist:

| sweep | placements | ns, best–worst | spread |
|---|---|---|---|
| ADR-0057's baseline — the origin of the figure | 4 | 87.43–88.51 | **1.2%** |
| ADR-0074's shipped tree, one day later | 4 | 78.87–82.65 | 4.8% |
| ADR-0108's base | 8 | 76.48–79.86 | 4.4% |
| ADR-0088's base | 8 | 76.73–82.00 | 6.9% |
| `d737240`, ADR-0113's and ADR-0116's base | 16 | 76.51–80.68 | 5.5% |
| `16c5cad`, the AMO-mux sweep's base | 16 | 75.86–81.92 | 8.0% |
| **this ADR's base, `c51ebba`** | **16** | **76.10–83.10** | **9.2%** |

And on a changed netlist it goes further: 7.3% and 7.5% for the two single arms here, 10.4% for the
stack, 8.4% for ADR-0108's changed tree, and 11.2% for the pair table whose entire price was that
number (ADR-0113).

**Four placements is not a tighter instrument, it is a shorter look at the same distribution.** The
spread is a range statistic: its expected value grows with the sample, so a four-seed sweep
systematically under-reports it and cannot do otherwise. That is the same fact as "eight seeds passed
a candidate sixteen declined" and "four seeds read +4.35% where eight read inside the band", stated
about the instrument instead of about a candidate.

**The corrected figure: `make soc-timing` has a best-to-worst placement spread of 4–9% on an
unchanged netlist at eight to sixteen placements, and up to 11% on a netlist whose cost is a
variance.** It lives in CLAUDE.md; `soc/timing_sweep.sh`, `soc/depth/sweep.sh`, `soc/depth/summary.py`,
`soc/compare/sweep.sh`, `soc/baseline_summary.py` and the Makefile's compare comment restate it or
point at it, and none of them carries a second number of its own.

**The go/no-go convention moves with it.** A verdict is **twelve to sixteen placements, paired by
seed, quoting worst, median and spread** — not worst-of-four, and not worst-of-N against worst-of-N,
which throws away the pairing and has almost no power against a spread this wide. Four seeds remain
what `soc/timing_sweep.sh` runs by default, and that is a look and not a verdict.

**Nothing in the record flips.** Every ADR that quoted 1–2% used it to call something a null or to
refuse a margin claim, and a wider band makes each of those *more* null, not less:
[0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md),
[0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md),
[0066](0066-twelve-megahertz-is-a-requirement.md),
[0076](0076-the-decode-head-is-a-plateau-not-a-lever.md),
[0080](0080-twenty-four-megahertz-is-not-reachable-on-the-up5k.md),
[0083](0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md),
[0087](0087-the-instruction-memory-does-not-come-out-of-the-fetch-loop.md),
[0088](0088-the-win-is-in-what-the-expression-cannot-say.md),
[0090](0090-the-executor-carries-half-the-registers-it-had.md),
[0091](0091-the-stall-costs-the-same-at-a-memory-pin.md),
[0092](0092-the-writeback-slot-costs-more-than-the-bypass-it-replaces.md),
[0104](0104-the-fetch-bus-refuses-and-the-data-bus-cannot-afford-to.md),
[0116](0116-the-loadstore-region-test-is-priced-three-ways-and-every-price-is-the-sum.md) (which
quoted it to contradict it) and
[0118](0118-the-incremental-mtip-costs-more-than-the-compare-it-replaces.md). They are left as
written: an ADR is a dated record, and this one supersedes the number rather than editing fourteen
files that were right about their own verdicts. The live prose is what changes, because that is what
the next reader is told to use.

**What the correction does change is what a margin claim costs to make.** ADR-0083 declined narrow
forwarding partly on 0.48% of margin; against a 9.2% spread, 0.48% at any number of placements below
twelve is not a measurement at all. ADR-0111's price list is arithmetic in **churn bands** and is
untouched — 3.6% is ADR-0054's number, measured across edits, and this ADR does not move it.

**Margin at the tail is a sample too, and it is smaller than the recent ADRs suggest.** The shipping
design's worst of sixteen on `c51ebba` with this toolchain is **12.03 MHz — it meets the requirement
at sixteen of sixteen placements, with 0.25% to spare**, where `16c5cad` read 12.21 and `d737240`
12.39 at their own worst of sixteen. Nothing regressed: 17 cells came out between the last two of
those and the tail is a draw from a 9.2%-wide distribution. Quote a margin with the sweep it came
from, the way a price is quoted with its tree.

**The `fit` gap is not an offset.** The same warning one instrument over: `make fit` read 54 cells
apart between a local build and the job on one tree, 32 on another, 3 on two more — and the sign is
not the same either time. There is no correction factor to apply, because it is re-mapping and not a
bias. Quote the job's number; a local run is a sanity check.

## Decision

**The registered null stands: occupancy does not set the placement tail at 89%, and area is not a
margin lever.** ADR-0111's open question is closed in the direction ADR-0088 and ADR-0090 predicted,
now on a dose of the opposite sign and on the statistic the requirement is graded by. Work that
frees cells is justified by headroom on the up5k's 5280 and by whatever path it takes with it —
never by the tail it is expected to buy.

**Occupancy is not nothing, and the figure is 2% of median for 6.6 points.** It is inside the churn
band, so it will not settle any candidate on its own; it is a reason to expect a large area change to
show up at the median, and a reason not to read a 2% median move on a growing design as a path
problem.

**`make soc-timing`'s placement spread is 4–9% on an unchanged netlist, superseding ADR-0057's
1–2%.** One authoritative statement in CLAUDE.md; every other site restates or points at it.

**A go/no-go is twelve to sixteen placements, paired by seed, quoting worst, median and spread.**

## Consequences

- **A pre-registration is worth its cost, and the cost was one commit.** The falsifier's two
  conditions split a result that reads as a win on one column and a null on another: arm B is 13 of
  16 slower *and* faster at the tail. Written afterwards, either sentence could have been the
  finding.
- **A pre-registration that is edited is worth nothing, so this one records its own amendment.** The
  design lost an arm between being written and being run — the second harvest was declined and its
  patch thrown away — and the replacement is a ballast, which is a better probe of *this* hypothesis
  and answers none of the additivity question the original design would have. Both halves are above,
  in the text the registering commit carried.
- **The instrument's noise figure is now measured at the size sweeps are actually run at.** Fourteen
  ADRs quoted a number taken from four placements; none of their verdicts flips, and every future
  margin claim gets more expensive to make.
- **Two of this repo's standing habits gain a worked example each.** "A ceiling is perishable" — arm
  A's dose is −37 here against −54 five commits ago, because the base moved under it. "A median
  inside the band is a null in both directions" — the same arm read −2.67% then and +0.34% now.
- **The ballast is a reusable probe and is not committed**, for the reason ADR-0118's variants were
  not: eight `(* keep *)` LFSRs in `rtl/littlesoc.v` is fifteen lines, and a probe that lives in the
  tree is a thing to keep green. The recipe is in this ADR; rebuild it when there is a question it
  answers.
- **Nothing else moved.** `rtl/` is untouched, so `FIT_MAX_LC`, `SOC_MIN_MHZ`, `SOC_EXPECT_SPRAM`,
  `SOC_EXPECT_EBR`, the BMC depths, F and G, every exclusion set and every failure baseline are
  exactly as they were, and the verification legs cannot have moved.
