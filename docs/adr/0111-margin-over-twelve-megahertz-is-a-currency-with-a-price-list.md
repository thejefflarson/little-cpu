# ADR-0111: Margin over 12 MHz is a currency, and the queue is its price list

**Status:** Accepted · 2026-08-16 · *Qualifies ADR-0089's rule that Fmax above the requirement is
margin and not speed. Generalises the transaction ADR-0093 already completed. Amends no commitment.*

## Context

ADR-0089 records that Fmax above the requirement is margin and not speed, because a CPI win converts
to throughput and a placement that closes higher does not. That is true about what can be *quoted*,
and it has been read as making period work above 12 MHz a non-goal.

Read that way it is wrong, and this repo has already proved it wrong once.

**ADR-0093 is the counter-example and the precedent.** The version of the operand-fetch guess that
decodes a compressed successor lost a placement outright at 11.56 MHz and was declined on it. After
roughly 490 cells came out of the SoC it was re-measured, lost none of ten placements, and bought
**16.3% of Dhrystone's cycles**. It ships. Margin bought a CPI win, and the only reason the win was
available to buy was that the margin had appeared.

Meanwhile the declined list is not a graveyard. It is a queue of measured wins, each blocked on the
same resource.

## Decision — margin is a currency, and the queue is priced

Period above 12 MHz is not a goal in itself and is still not quotable as speed. **It is the resource
that unlocks CPI wins already measured and declined.** Work that buys margin is therefore justified
by what the margin then buys, and must be argued that way — with the intended purchase named.

The price list, every entry perishable and every entry a worst-seed cost on the tree it was measured
on:

| item | price | payoff | takeable at |
|---|---|---|---|
| ~~operand-fetch guess~~ | — | 16.3% of Dhrystone | **collected**, ADR-0093 |
| ~~successor-pair table (ADR-0101)~~ | +3.9% worst, median a **null**, +90 cells | −8.0% of Dhrystone, 0.727 → 0.790 DMIPS/MHz | **declined, ADR-0113** |
| narrow forwarding (ADR-0083) | ~2.9% | −7.5% of suite cycles, Dhrystone unmeasured | ~11% |
| early register write (ADR-0100) | +8.8% | −6.5% of Dhrystone | ~12–13%, dominated |
| causes 5 and 7, loads and stores (ADR-0104) | ~13–16% | conformance | ~17–20% |
| full forwarding (ADR-0083) | ~21% below the requirement | −12.9% of suite cycles | **unpurchasable** |

An item is takeable when worst-seed margin covers **its price plus one 3.6% churn band of
residual**, so the next change's go/no-go stays attributable — the arithmetic ADR-0101 used to defer
itself.

**Three rules come with the list.**

**Prices are stale until re-swept.** Every one was measured on a tree that no longer exists. A
purchase re-takes its own price first, at eight seeds or more; four seeds have twice produced a
different verdict than eight, most recently in ADR-0108. **Eight has now produced a different verdict
than sixteen too** — the top row was re-taken and the first eight seeds passed it where all sixteen
declined it (ADR-0113), so read "eight or more" as a floor and not a target.

**A price can be a variance, and then only the tail sees it.** The pair table's re-taken median cost
is inside the churn band and its worst seed is +3.9%: it does not slow the design down, it widens the
distribution, and `SOC_MIN_MHZ` grades worst-of-sweep. Quote a price as a distribution, never as one
number, and expect margin bought by reducing variance to buy such an item more cheaply than margin
bought by shortening a path.

**Payoffs do not sum.** Removing operand stalls moves the scoreboard column *up* — ADR-0074 measured
8381 becoming 8837 — and the pair table and early write measured sub-additive. Tiers are measured as
stacks.

**The programme is bounded.** Above roughly 12% of margin nothing on the list has a buyer until
conformance at 17–20%. Buying margin past the point where something is waiting for it is not
justified by this ADR.

## What this does not license

**It does not reopen the declined ceilings as a class.** Three things get confused under the word
"declined", and only one of them stacks:

- a **real effect hidden by the churn band** — stackable, land a themed group and measure the group,
  which is ADR-0088's method;
- a **ceiling measured by deleting the block whole** — not stackable, because it bounds every
  spelling of that block and stacking zeros is zero;
- an **anti-win** — a measured cost.

ADR-0076's decode head and ADR-0087's memory-out are ceilings. The flattened `next_pc` chain
(ADR-0088) and the operand mux (ADR-0097) are anti-wins. **There is no queue of hidden period
positives waiting to be stacked**, and this ADR must not be cited to imply otherwise.

**It does not amend a commitment.** Nothing on the price list requires it, and the 21% that deleting
all of `next_pc`'s inputs would buy stays declined on the no-wrong-path-state commitment.

**It does not make area a period lever by assumption.** The removal direction has been measured
twice and come back null — ADR-0088 at −169 placed cells over eight seeds, ADR-0090 at −400. What
ADR-0093 shows is that a decline did not survive re-measurement after cells came out; it explicitly
declines to claim congestion was the cause, and so does this ADR. Whether occupancy sets the
placement tail at 89% is open, and the next harvest is designed to answer it with a registered
prediction rather than a hope.

## Consequences

- **A margin-buying change states what it is buying.** "This buys 3% of worst-seed margin" is not a
  result; "this buys 3%, which puts narrow forwarding in reach at its re-taken price" is. And the
  purchase has to be re-taken before it is spent, because the one item priced highest here did not
  survive its own re-take.
- **The re-read of a ceiling is free and the re-measure is not.** The numbers are in the ADRs. Re-take
  a ceiling in the tree you mean to spend it in, at spend time — doctrine already, reinforced here.
- **ADR-0089 is qualified, not replaced.** Its rule stands for quoting: a higher-closing placement is
  still not speed and DMIPS is still quoted at the board's 12 MHz.
- **The list is a living artifact.** An item's price or payoff changing is a reason to update this
  ADR, and an item leaving it is struck either way — the operand-fetch guess collected, the pair
  table declined on its own re-take.
