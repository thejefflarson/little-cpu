# ADR-0113: The learned successor-pair table is declined, and its price is the placement tail

**Status:** Accepted (declined) · 2026-08-16 · *Supersedes
[ADR-0101](0101-the-successor-pair-is-learned-and-deferred-for-margin.md)'s deferral, which is now
closed rather than pending. Corrects the top row of
[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md)'s price list. The
payoff it measured reproduces exactly; the price does not.*

## Context

ADR-0101 built the table, measured it, held 12 MHz at every placement of six, and deferred it on
margin arithmetic rather than declining it — the only candidate in the tree in that position.
ADR-0111 made it the first item on the price list: **−8.0% of Dhrystone's cycles, 0.727 → 0.790
DMIPS/MHz**, priced at +3.2% of worst-seed margin and +5.0% of median period, takeable at about 7%.

Every input to that arithmetic had moved. The A extension added roughly 450 cells, causes 1, 5 and 7
landed, `misa` claims A, and the worst-seed margin is a different number. ADR-0111's own first rule
is that a price is stale until re-swept, at eight seeds or more.

So it was rebuilt on today's tree rather than argued forward, and swept at **sixteen** seeds a side.

## Decision

**Declined on measurement.** The worst of sixteen placements is **11.93 MHz**, under a requirement
whose next divider step down is 6 — and `make fit` is over its ratchet at the same time. Neither is
a correctness failure and the payoff is real; both are the design not fitting the part it is for.

The RTL is on the branch this ADR lands on and comes back out before it merges, the way ADR-0100's
and ADR-0101's did. What survives is the measurement.

### The payoff reproduces exactly

| | base | with the table |
|---|---|---|
| suite | 34 072 cycles, 18 352 retired, CPI 1.86 | 34 237, CPI 1.86 (**+0.48%**) |
| suite operand column | 798 | 967 |
| Dhrystone | 1 607 636 cycles, CPI 1.69, 0.727 DMIPS/MHz | 1 478 453, CPI 1.56, **0.790** (**−8.03%**) |
| Dhrystone operand column | 295 585 | 166 415 |

−129 183 Dhrystone cycles, against ADR-0101's −129 179 on a tree several designs older.

**The suite's +0.48% is not 70 programs getting worse, and reading it as one is what ADR-0101's
table hid.** Per program: 6 better, 47 worse, 17 unchanged, −259 cycles against +424. The two that
improve are the only two in the suite with a loop that redirects repeatedly — `mtimer.S` at −227
cycles, its operand column collapsing from 253 to 26, and the compiled `datainit.c` at −28. The 47
that pay are short, straight-line, hand-written assembly, each losing a handful of cycles because a
cold or stale entry outranks a sequential guess that was already right. **The two programs that
resemble code anybody would run agree with Dhrystone; the ones that do not, disagree.** That is
ADR-0084's warning stated sharply enough to act on.

The shortest programs also report one more retire with the table in. That is the end of the run, not
the program: the harness stops on the cycle `tohost` lands in RAM, and one more instruction has got
down the pipe by then. Every program's spec-checked count moves with its retire count, `make test`
is 70/70 both ways and the failure baseline matches exactly.

### The price is not what it was priced at

Sixteen seeds a side, `make soc-timing` on 4c1958c and on that tree plus the table, OSS CAD Suite
(yosys 0.68+48, nextpnr-0.11-1-g62e659ed) — CI's toolchain family, not a Homebrew yosys. This ADR
lands three commits later; none of them touches `rtl/` and none changes what `make soc-timing` or
`make fit` build, so the sweeps are of the tree they are quoted against. They were taken with
`soc/timing_sweep.sh`, the toolchain and base written down by hand; `soc/baseline_sweep.sh` merged
during this work and is what a fourth attempt should use, because stamping a sweep with its own
provenance is exactly what a re-take of a stale price needs.

The with-table sweep was run with `SOC_MIN_MHZ` overridden to zero on the command line, because
`soc/timing_sweep.sh` hands the target's status straight back and would otherwise have stopped at the
first placement under the requirement — which is the whole result. The requirement itself is not
touched and the grading below is done on the ns column. Paired by seed, ns:

| seed | base | with table | Δ |
|---|---|---|---|
| default | 80.22 | 78.27 | **−2.43%** |
| 1 | 76.92 | 75.63 | **−1.68%** |
| 2 | 79.99 | 82.08 | +2.61% |
| 3 | 77.01 | 81.04 | +5.23% |
| 4 | 78.44 | 79.01 | +0.73% |
| 5 | 80.68 | 77.70 | **−3.69%** |
| 6 | 78.36 | 79.63 | +1.62% |
| 7 | 79.06 | 82.23 | +4.01% |
| 8 | 79.42 | 75.40 | **−5.06%** |
| 9 | 76.51 | **83.82** | +9.55% |
| 10 | 76.52 | 80.53 | +5.24% |
| 11 | 77.22 | 81.21 | +5.17% |
| 12 | 78.23 | 81.45 | +4.12% |
| 13 | 79.89 | 75.90 | **−4.99%** |
| 14 | 78.14 | 77.62 | **−0.67%** |
| 15 | 78.65 | 76.27 | **−3.03%** |

| | worst | median | best | best-to-worst |
|---|---|---|---|---|
| base | 80.68 ns, **12.39 MHz**, +3.29% over the requirement | 78.40 ns, 12.76 MHz | 76.51 ns | 5.5% |
| with the table | 83.82 ns, **11.93 MHz**, −0.58% **under** it | 79.32 ns, 12.61 MHz | 75.40 ns | 11.2% |

**The median cost is +1.17% and the mean +1.05%, both well inside the 3.6% edit-churn band — a null.
Nine of sixteen seeds are slower, which a sign test cannot separate from a coin (p = 0.80). The
table's best placement is faster than the base's best.** ADR-0101's +5.0% of median period does not
reproduce and should not be quoted again.

What the table does instead is **widen the distribution**, and the price is entirely in that tail.
Since `SOC_MIN_MHZ` grades worst-of-sweep, a candidate that costs nothing at the median and
everything at the tail is priced at its tail, and one placement of sixteen is enough to decline it.

**Eight seeds would have shipped it.** The first eight of this very sweep bottom out at 82.23 ns,
12.16 MHz — a pass with 1.3% left over. So would four, and so would six, which is what ADR-0101 ran.
Seed 9 is the placement that misses. CLAUDE.md records that four seeds have twice given a different
verdict than eight; this is the same fact one step out, and it is the reason a candidate whose
effect is a variance change cannot be graded on a short sweep at all.

### Area

`make fit` is over its ratchet independently of any of the above.

| | cells |
|---|---|
| `fit` job on the base (run 31977544440, 4c1958c) | **3966** |
| local, same OSS CAD Suite, base | 3969 |
| local, same OSS CAD Suite, with the table | **4059** |
| `FIT_MAX_LC` | 4000 |

+90 cells, against ADR-0101's +77 on a smaller core. The local build and the job agree to **3 cells**
on the base, so the delta transfers and the projected job number is about 4056 — over the ratchet by
roughly 56. `FIT_MAX_LC` is not raised to admit it: the ratchet exists to make growth visible, and
this is growth. The with-table number was not taken from the job itself, because the RTL does not
land; a fourth attempt takes it from a branch that carries the RTL through a pull request.

`SOC_EXPECT_EBR` moves 20 → 21, exactly the one 256×16 block RAM the table asks for, and
`SOC_EXPECT_SPRAM` is unchanged at 2. The SoC does not synthesise without that edit, so the extra
block is not an estimate.

### Nothing else objects

Green on the mechanism, so the decline is attributable to area and period alone: `make test` 70/70
with the failure baseline matching, `make test-units` including the restored `pairtable_tb`,
`make waves`, `make lint` clean in both passes, and `components_pcloop` (after `pcloop_cover`),
`components_traps` and `components_decoder` all proved by k-induction with `pair_hit`/`pair_rs*` free
— which is wider than the table can be, so those proofs cover every entry it could hold and every one
it could not.

**F and G are untouched and were not re-measured.** The table adds no stall reason — the guess is
checked by the `operand_stall` comparison that already exists — lengthens no stage, and widens no
scoreboard, which are the three triggers. `make cycles` still accounts for six reasons and reports
no unattributed cycle.

## Consequences

### ADR-0111's price list, corrected

The pair-table row reads +3.2% worst / +5.0% median, takeable at ~7%. Re-taken, it is:

| | old | re-taken, sixteen seeds |
|---|---|---|
| median period | +5.0% | **+1.17%, a null** |
| worst seed | +3.2% | **+3.89%** |
| spread | not measured | **5.5% → 11.2%** |
| area | +77, +1 EBR | **+90, +1 EBR, over the ratchet** |
| payoff | −8.0% of Dhrystone | **−8.03%, unchanged** |

The "takeable at ~7%" figure survives as arithmetic — its +3.89% worst-seed price plus one 3.6%
churn band of residual is 7.5% — but **what it is a price for has changed**. It is not a slower
design that needs headroom to absorb it. It is a design whose placement tail gets longer, and
worst-of-sweep is the only instrument that sees that.

### What a fourth attempt needs

Not "wait for margin" in general. Three specific things:

- **`make fit` under 4000 with the table in.** +90 cells is unconditional; no amount of period work
  makes the ratchet pass. Either the harvest ADR-0112 opened frees the cells, or the table gets
  cheaper — its tag is six bits over an eight-bit index for 32 KB of distinct text against an 8 KB
  ROM, which is four times the separation the part can use.
- **A worst-of-sixteen baseline at 7.5% or better**, or a demonstrated reduction in the placement
  tail. Variance reduction counts fully here and is the cheaper of the two, because the median cost
  is already nothing: this candidate does not need the design to be faster, it needs the tail to stop
  moving. That is the same lever the margin brief names for EBR column pinning, and this measurement
  is the strongest evidence in the tree that it is the right one.
- **Sixteen seeds, paired, on both sides.** Eight passed this candidate and sixteen failed it. A
  re-take at eight seeds proves nothing about it either way.

If those hold, the payoff is not in doubt: it has now been measured at −8.0% of Dhrystone's cycles on
two trees a milestone apart.

### Against the four goals

**Fast** — the largest single CPI win the price list ever carried, and it goes unbought. **Simple** and
**readable** — a module carrying learned state, which the rest of the design is deliberately free of;
that cost is not paid, which is the one consolation in declining it. **Formally verified** —
unaffected either way; the proofs pass with the table's answer left free, and F and G never moved.

### What was not claimed

That congestion caused the tail to lengthen. The table adds 90 cells and a block RAM, and the tail
lengthens; occupancy either side was not measured and the two are not connected here, for the same
reason ADR-0093 and ADR-0111 decline to connect them. One candidate is one data point.

That the median null means the table is free. It is not — a null median with a doubled spread is a
worse thing to own than a small honest cost, because it cannot be measured out of a short sweep and
the requirement is graded at the tail.

That the suite is now a proxy for Dhrystone. It is not. The two programs in it that agree with
Dhrystone agree because they have the loop structure Dhrystone has, and 47 that do not, do not.
