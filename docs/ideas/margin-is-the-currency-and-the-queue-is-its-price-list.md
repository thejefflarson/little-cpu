# Margin is the currency, and the queue is its price list

**Worst-seed margin over 12 MHz is not a trophy. It is what buys the CPI wins this repo has
already priced and declined. Get it to about 7%, spend it on the successor-pair table, stretch
toward 12% if a second unlock is worth it, and stop — above that nothing has a buyer.**

## The question this answers, and the answer it replaces

The starting question was where the next timing or area win comes from, at 1.0% of worst-seed
margin and 89% occupancy. The first answer back was that period work above 12 MHz is a non-goal,
resting on the rule that Fmax over the requirement is margin and not speed, and that only a CPI win
converts to throughput.

**That was wrong, and this repo had already proved it wrong.** The compressed-successor
operand-fetch guess lost a placement outright at 11.56 MHz and was declined on it. After roughly
490 cells came out of the SoC it was re-measured, lost none of ten placements, and bought **16.3%
of Dhrystone's cycles** (ADR-0093). Margin bought a CPI win once already. That transaction is the
precedent this brief generalises, and it is worth stating exactly as ADR-0093 stated it: what is
claimed is that the decline did not survive re-measurement, not that congestion was the cause.

So the question is not "what is the period worth?" It is **"what is the cheapest margin, and what
does each increment unlock?"**

## The price list

Every price is a worst-seed cost measured on the tree it was taken on, and every one is perishable.
Re-sweep at eight seeds or more before spending: four seeds have twice given a different verdict
than eight in this repo, most recently in ADR-0108.

An item is takeable when worst-seed margin covers **its price plus one 3.6% churn band of
residual**, so the next change's go/no-go stays attributable. That is the arithmetic ADR-0101 used
to defer itself at a 3.0% residual.

| item | worst-seed price | payoff | takeable at |
|---|---|---|---|
| ~~operand-fetch guess~~ | — | 16.3% of Dhrystone | **already collected** (ADR-0093) |
| **successor-pair table** (ADR-0101) | +3.2% worst, +5.0% median, +1 EBR | **−8.0% of Dhrystone, 0.727 → 0.790 DMIPS/MHz** | **~7%** |
| narrow forwarding (ADR-0083) | ~2.9% | −7.5% of suite cycles; **Dhrystone unmeasured** | ~11% |
| early register write (ADR-0100) | +8.8% worst, missed 12 MHz at four of six | −6.5% of Dhrystone | ~12–13%, and dominated |
| causes 5 and 7, loads and stores (ADR-0104) | ~13–16% | conformance, no CPI | ~17–20% |
| causes 5 and 7, atomics only | **unpriced** | conformance slice | possibly ~5% |
| full forwarding (ADR-0083) | ~21% below the requirement | −12.9% of suite cycles | **struck — unpurchasable** |

**The columns interact, so tiers must be measured as stacks and never summed.** Removing operand
stalls moves the scoreboard column *up* — ADR-0074 measured 8381 becoming 8837 — and the pair table
and the early write measured sub-additive against each other. The pair table also *spends* margin
that forwarding then *needs*, so the order they land in changes what fits.

**The tiers are the decision:**

- **Below about 4.6%** — nothing lands attributably, including nulls. Today's 1.0% headline is one
  draw; the same netlist family placed at 12.75 MHz worst two merges ago.
- **About 5%** — the instrument is honest again and the atomics-only region test is worth pricing.
  No CPI item yet.
- **About 7% — one unlock: the pair table.** It beats forwarding for the slot on three grounds: its
  payoff is quantified on the workload that matters, it adds machinery beside the pipeline rather
  than into the bypass loop, and the forwarding family is nought for four at that loop
  (ADR-0083 twice, ADR-0092, ADR-0100).
- **About 11–12% — two unlocks**, plausibly 0.82–0.84 DMIPS/MHz, roughly 10 DMIPS at the board's
  12 MHz.
- **Above about 12% — no buyer** until conformance at 17–20%. **This is what bounds the programme.**

## Where margin is bought, cheapest first

**1. Instrument honesty, free.** `SOC_SEEDS` already takes a list. Re-baseline at 12–16 seeds and
correct the rulebook: `make soc-timing`'s recorded "1–2% placement spread" is contradicted by this
repo's own eight-seed tables, which run **4.4% to 8.4%** — ADR-0088's base row alone spans 76.73 to
82.00 ns. On an honest baseline the effective worst-seed margin is plausibly **3–5% already**.

**2. Tree hygiene, near-free.** Null respellings move the worst seed by up to 3.3% — ADR-0106
measured two cells doing exactly that, and A shipped on its better spelling.

**3. EBR placement constraints, unknown, falsifier free.** The fetch loop starts and ends in sixteen
enumerable `SB_RAM40_4K` whose columns nextpnr re-chooses every seed. Pinning is a nextpnr
constraint and **not RTL**, so it is two-part-safe by construction — the ECP5 flow never sees it.
Because the gate is worst-of-sweep, **variance reduction counts fully as margin.** Falsify before
writing a single constraint: histogram the largest routing hops in the icetime reports the sweeps
already produce. If the long hops are not at EBR boundaries or the `riscv.pc` source, this is dead
in an afternoon.

**4. Fan-out replication**, `(* keep *)` copies of `pc` and the `stall` cone. RTL but behaviourally
identical. Needs the hx8k check.

**5. Area, if the congestion question resolves that way.** Settled by the harvest's own sweeps —
see below.

## Do sub-band nulls stack for period the way they stack for cells?

**The method transfers. The declined list does not.** Three things get confused under one word:

- A **real effect hidden by the band** — stackable. Land a themed group, measure the group. This is
  ADR-0088's method and it works for period too, given twelve to sixteen seeds a side: ADR-0076's
  own decomposition shows `next_pc`'s terms are about 5% each and 21% together.
- A **ceiling measured by deleting the block whole** — not stackable. It bounds every spelling of
  that block, and stacking zeros is zero.
- An **anti-win** — a measured cost.

Re-read against that: ADR-0076's decode head (3.3%, deleted whole) and ADR-0087's memory-out
(+0.4% at its best cell) are **ceilings**. The flattened `next_pc` chain and the operand mux are
**anti-wins** at +3% to +9%. **There is no queue of hidden period positives waiting to be stacked.**
The stackable frontier is whatever the hop histogram names, because per-hop routing edits are
exactly the sub-band-individually, real-in-aggregate shape.

## The census, whole design

`synth_ice40 -top littlesoc -dsp -noflatten`, local counts excluding submodules. Rows run a few
percent high because `-noflatten` loses cross-boundary sharing. The `regfile` row is discarded: its
arrays infer flops here instead of taking the block RAM the real flow gives them, and ADR-0096's
measured 133 fabric LUTs stands instead. Read decisions off the seed-stable placed count and the
`fit` job's number, never a local absolute — the local-to-CI gap has no fixed size **and no fixed
sign**, measured at +53 and −4 cells for one bit of one read-only constant.

| module | LUT4 | carry | DFF | status |
|---|---|---|---|---|
| `rtl/executor.v` | 1182 | 268 | 147 | closed, ADR-0090 |
| `rtl/decoder.v` | 1077 | 124 | 186 | closed in two blocks, grew since |
| `rtl/csrs.v` | 591 | 127 | 259 | closed, ADR-0096; growth measured and explained |
| `rtl/accessor.v` | 534 | 32 | 151 | **never asked** |
| `rtl/timer.v` | 322 | 126 | 161 | **never asked** |
| `rtl/memory.v` | 299 | 0 | 4 | window facts only |
| `rtl/imemory.v` | 249 | 9 | 96 | window facts only |
| `rtl/fetcher.v` | 177 | 28 | 0 | never asked |
| `rtl/regsel.v` ×2 | 58 each | 0 | 0 | small by design |
| `littlesoc` glue | 45 | 2 | 10 | never asked |
| `rtl/writeback.v` | 40 | 0 | 0 | never asked, nothing there |

**`make fit` cannot see the SoC side at all** — it builds `littlecpu` with memories external. So
`timer`, `memory`, `imemory` and the glue buy no ratchet headroom, but they do buy occupancy, and
under this brief's thesis occupancy may buy the tail. That makes the SoC side's value
**conditionally larger and currently unproven**, against the core side's **small and certain**.

## The harvest, ranked

| rank | block | what the question finds | estimated yield |
|---|---|---|---|
| 1 | `rtl/accessor.v` | the reservation is `[29:0]` with a 30-bit comparator but is only ever set inside one aligned 64 KB window, so it needs **14 bits**; the two 32-bit bus muxes are **ORs**, because decode publishes an all-zeros bubble on every `take_amo` cycle; `take_amo_addr[1:0]` are provably zero; census-guided flag encoding | −80 to −180 |
| 2 | `rtl/timer.v` | **not** the carry chains — those are two spec-mandated 64-bit operations and the width is the job. The write path (`mtimecmp`'s byte lanes may be pure `SB_DFFE`), a compare that only needs a fresh 64-bit magnitude on an `mtimecmp` write since a one-cycle-late `mtip` is architecturally unobservable, and the read mux | −40 to −120 |
| 3 | `rtl/memory.v` | per-SPRAM arm decode, the `wstrb`-to-`MASKWREN` nibble mapping, the rdata zero-gate | −20 to −60 |
| — | `rtl/imemory.v` | **excluded.** Every net is loop-adjacent; an area edit there is a period experiment whether intended or not. It belongs to the placement work, not the harvest | — |

**Total −150 to −400 placed cells**, of which only −90 to −220 is visible to `make fit`. Occupancy
89% to roughly 85–87% if the middle lands.

**The timer carries a warning label.** CLAUDE.md records that shaving a 64-bit counter on this
fabric — riding the adder's carry-in instead of a clock enable — freed three cells, moved 128 flops
from `SB_DFFESR` to `SB_DFFSR`, and **missed 12 MHz at six placements of six**. The timer's
write-or-increment structure is the same shape. Full seed sweeps even for an area null.

**And the timer's period risk is already handled in the file.** `mtip` is registered precisely so
the 64-bit compare does not land on the decoder's trap term, which is in the fetch loop; the file
says so at the site. ADR-0087's `mtip → minstret` residual therefore implicates mtip's consumers in
the CSR and trap logic, not the timer's internals.

## Two groups, and why that is better than one

Land the harvest as **two separately-swept groups**: core-side (accessor plus the decoder's A rows)
and SoC-side (timer, memory, glue).

That is not bookkeeping. **It upgrades the congestion experiment from one data point to two.** The
open question is whether occupancy sets the placement tail at 89% — the removal direction has been
measured twice and come back null (ADR-0088 at −169 placed cells over eight seeds, −0.5% worst and
+0.3% median; ADR-0090 at −400), but both were core-side, and ADR-0093's episode hints the other
way without claiming causation. A core-side removal and a SoC-side removal, each eight seeds a side
against the same base, is a dose-response measurement the single-group version cannot give.

**Register the prediction before landing either**: null, per ADR-0088 and ADR-0090. If the tail
tightens instead, area becomes a margin lever and the consequence of ADR-0088 needs qualifying by
ADR.

## Which closed ceilings have most likely rotted

Not a blanket re-run — a ranked list of re-*reads*, which are free because the numbers are already
in the ADRs.

1. **`rtl/decoder.v`'s two closed blocks.** The ceilings predate the eleven A flags, the A
   immediate-mux arm, the `imem_fault` arm and the atomic stall, all in that file. ADR-0094 itself
   proved these are the most context-sensitive kind: the same edit read −50 synthesising one top and
   −1 synthesising the other. **Re-take only at spend time, in the spend tree.**
2. **ADR-0087's fetch-tail register.** Priced standalone at +0.4% of product and rejected against a
   bar that margin-as-currency invalidates. Re-read now; re-measure only if the cheap margin falls
   short, and then on both parts, because that ADR measured the two disagreeing in sign.
3. **`rtl/executor.v`.** Design moved around it but ADR-0090 was thorough, and deleted-whole
   ceilings age slowest.
4. **`rtl/regfile.v`, the read-back bus, the `mem_rdata` floor.** Deleted-whole ceilings on unchanged
   designs.

**`rtl/csrs.v` is discharged, not deferred.** Its growth was measured directly when the 87
performance-monitor CSRs landed — **+53 synthesised SoC LUTs**, explained at the time: ADR-0096
priced only the write side, and a read-only-zero CSR costs its address decode. There is nothing to
re-take.

## What this does not touch

No commitment is amended. The no-wrong-path-state commitment and the stall-only-hazards commitment
are untouched by everything above, and the 21% that deleting all of `next_pc`'s inputs would buy
stays declined.

**One source serves both parts.** The dual-core SoC targets ECP5; the single-core design stays a
first-class up5k target; there is no fork, no `ifdef`, no parameterised datapath split. up5k remains
the binding constraint at 89% and 12.12 MHz worst-seed, against ECP5's 20% and 35.33 MHz — so every
argument here is an up5k argument, and ECP5 is where a candidate must **not regress** rather than
where it must fit. That is the cheaper test. `soc/compare/` already places on hx8k, so the machinery
for a two-part check exists.

**The operand-fetch cycle is paid on both parts.** `rtl/regfile.v`'s read is a clocked assignment
and decode's present-bubble-issue protocol is written into `rtl/decoder.v`. Technology mapping
cannot delete a cycle the RTL takes, and exploiting ECP5's asynchronous LUT-RAM read would mean
changing the read protocol — a commitment amendment plus exactly the fork that is ruled out. So the
pair table's 18.4%-of-Dhrystone column is real on both parts, and the table is portable: one EBR on
ice40, one of 56 block RAMs on ECP5.

## Sequence

1. **Harvest group 1** — accessor plus the decoder's A rows. Pays the threatened ratchet. Land with
   eight or more seeds a side and the prediction registered.
2. **Harvest group 2** — timer, memory, glue. SoC-side. Read the `SB_DFFE` census before any write
   path edit. Full sweeps even on an area null.
3. **Rebuild the instrument** — 16-seed baseline on the landed tree, the hop histogram off the
   reports that already exist, and the placement-spread correction by ADR.
4. **Spend at the thresholds.** Pair table at 7% or better on a fresh sweep, its price re-taken
   because it is two trees stale. Narrow forwarding only after its Dhrystone payoff is measured, and
   only at 11% or better.

Steps 1 to 3 are committable. Step 4 is the decision they produce.

## Risks

Every price in the table is a stale distribution and genuinely open until re-swept. The hop
histogram may find nothing, which costs an afternoon. The harvest may read smaller under CI's
toolchain than locally, which is why the `fit` job's number is what counts. Forwarding's Dhrystone
payoff is unmeasured and its family is nought for four at the bypass loop. The atomics-only region
test is the one unpriced experiment and could be a cheap conformance win or another four levels.
