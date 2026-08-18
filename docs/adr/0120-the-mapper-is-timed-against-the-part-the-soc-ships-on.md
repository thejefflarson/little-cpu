# ADR-0120: The mapper is timed against the part the SoC ships on, and its value is a property of the tree

**Status:** Accepted · 2026-08-18 · *Supersedes
[ADR-0080](0080-twenty-four-megahertz-is-not-reachable-on-the-up5k.md)'s dead-end row for
`-abc9 -device u`, which was one placement, and corrects its reasoning. Buys margin
[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md) may not bank.
Amends no commitment: no RTL changes and no stall reason, stage or scoreboard moves.*

## Context

`synth_ice40`'s abc9 pass maps LUTs against a **device grade**, and the SoC's synthesis line named
none — so every placement this project has ever measured on an up5k was mapped against **hx1k**
delays, which is the flag's default and nobody's decision. The numbers really do differ. `SB_LUT4`'s
`I0 => O` is 449 ps on hx against 1245 on up5k and `SB_CARRY`'s `CI => CO` is 126 against 278, so a
LUT level is worth **3.6 carry hops in the numbers abc9 was given and 4.5 in the part's own**: told
the wrong part, the mapper under-buys the carry chain.

The grade also decides which `ICE40_{HX,LP,U}` define `cells_sim.v` is read with, and **all 28 of
that file's `ICE40_U` blocks open a `specify`** — timing annotation, no behaviour. A result here can
only ever be about mapping, which is why the equivalence question below is answerable at all.

ADR-0080 declined this. It measured **one placement per row** on a tree long gone, and it bundled
two flags: today `-abc9` is `synth_ice40`'s default — `-noabc9` is the flag that turns it off, and
the shipping log already runs `abc9_map`/`abc9_model` with no flag asked for — so only the grade was
ever in question, and one placement is a sample by this repo's own rule.

## Decision — name the part, and treat what it buys as perishable

Measured paired by seed, both arms clean-stamped, one toolchain throughout (yosys 0.68+48
`ff5817c34`, nextpnr-0.11-1 `g62e659ed`, oss-cad-suite 20260811 icetime), `datainit.c` into 2048 ROM
words. **On the tree this lands on it is one of the larger period results in this repo's record:**

| tree | pairs | median Δ period | sign test | worst of N | placed LC |
|---|---|---|---|---|---|
| `553a241`, today's `main` | **32** | **−2.28%** | **28 faster / 4 slower, p = 0.00003** | 81.59 → **80.00 ns** (12.26 → **12.50 MHz**) | 4639 → 4658 |
| `ac7ee91`, three commits earlier | 64 | **+0.74%** | 25 / 39, p = 0.10 | 83.10 → 81.91 ns | 4676 → 4692 |

Both arms of both pairs hold 12 MHz at every placement, `SB_SPRAM256KA` is 2 and `SB_RAM40_4K` is 20
on all four, and the requirement is not what this decision is about.

The tail gets its own test, because `SOC_MIN_MHZ` is read against the worst placement and a sign
test only sees the middle. The arms are paired, so the null is exchangeability within a pair: flip a
random subset of pairs, recompute, 200 000 times. On today's tree the whole distribution moves:

| statistic | base | `-device u` | Δ | p |
|---|---|---|---|---|
| max | 81.59 ns | 80.00 ns | −1.59 | 0.125 |
| p90 | 80.75 | 77.99 | −2.76 | 0.012 |
| p75 | 78.85 | 77.29 | −1.56 | 0.003 |
| median | 77.81 | 76.24 | −1.57 | 0.0001 |

**The worst placement is the one number that is not significant** — it moves the right way and it is
a single sample of an extreme, which is exactly where a sweep of this size is weakest. Read this as
a median win that carries its tail along, not as a tail lever.

### The same flag on the tree three commits earlier is a small loss

At **64 paired placements** on `ac7ee91` the median goes the other way (+0.74%, p = 0.10 by the sign
test) and the same permutation test puts p75 and p90 about 1.1 ns **slower** at p ≈ 0.006 with the
better worst placement at **p = 1.000**. Three commits separate the two trees and only one touches
`rtl/` — [ADR-0119](0119-the-amo-result-mux-is-a-truth-table-and-an-adder-cannot-be-shared.md)'s AMO
result mux, worth −37 placed cells on the SoC.

The split says what the difference is made of. The grade always buys a little of the delay abc9
models; what decides the outcome is whether the two thirds it does not model follow or fight:

| tree | median logic | median routing | routing share |
|---|---|---|---|
| `553a241` base → `-device u` | 25.20 → **24.67 ns** | 52.85 → **51.84 ns** | 68% |
| `ac7ee91` base → `-device u` | 24.56 → **24.23 ns** | 52.45 → **53.86 ns** | 68% |

−0.53 ns and −0.33 ns of logic, against −1.01 ns and **+1.41 ns** of routing. So the flag's worth is
a property of the netlist it maps and not of the flag, it swings ±3% across trees a few dozen cells
apart, and **it is not margin that ADR-0111's price list may bank** — a purchase there re-takes its
own price, and this price expires faster than most.

At the netlist level the change is almost nothing: **+12 `SB_LUT4`** (4205 → 4217) and +3
`SB_CARRY` (685 → 688) on today's tree, +13 and none on the earlier one, with every flop, DSP, block
RAM and SPRAM count identical on both. Three runs of each recipe hash to one digest, so synthesis
stays reproducible with the grade named — which is what makes the paragraph below a detector rather
than a guess.

### The measurement caught a hazard that has nothing to do with the flag

Six placements of one arm were taken while `make mutation-check` had `rtl/` patched **in the same
worktree** — it applies its mutations to the tracked files and restores them at exit, so a
concurrent `make soc-timing` synthesises a mutated design and reports a period for it. Those six
read 4693–4780 placed cells where every other placement of that arm read 4692, which is how they
were found; re-placed alone, they agree with their arm. **The packed-LC column is the detector**: a
sweep whose cell count moves seed to seed is measuring more than one design. Nothing else may run in
a worktree that is sweeping.

## The tail is a set of cones, and endpoint migration is the tell

Over 64 base placements on `ac7ee91`, **every** critical path ends in one bucket — a net yosys named
after the even ROM bank's read data — while the starts spread across six: `imem.rom_odd` 20,
`imem.in_range` 18, `imem.rom_even` 15, `mtimer` 6, `por_done` 3, `riscv.accessor` 2. Those are net
*names* after flatten, which are ancestry rather than ownership, so what they support is the shape
and not an attribution: many cones, one destination, and the destination is the fetch address.

That retires a class of attempt rather than one attempt. Three RTL spellings that hoisted the
even-bank increment off that shared tail — in the decoder alone, and plumbed through to the memory —
all measured null, and the one that got closest **moved the reported endpoint to a `minstret` net at
the same length**. Deleting one cone crowns the next. Those were swept on `16c5cad` with dirty
candidate arms and are recorded as context rather than as prices.

## Two levers declined, and one of them is why this one was doubted

**`nextpnr --opt-timing`** is declined and not re-taken here. ADR-0080 placed it once at 85.05 ns
against 78.51 for the shipping recipe — **below the requirement** — and a later run spent 35 minutes
on a single placement to come out about 10 ns worse on `icetime`'s report. Two independent runs agree
in direction and one fails the requirement outright. It is the mirror of this ADR's result: nextpnr
optimises against its own delay model and `icetime` is the instrument that grades, so improving what
an optimiser believes is not the same as improving what is measured — which is precisely why the
grade needed sweeping instead of arguing.

**`soc/compare/` and the `fit` synthesis line stay as they are**, deliberately and not by oversight.
The comparison harness means something only while both cores are synthesised alike, and `-device u`
on the `fit` line would move a count `FIT_MAX_LC` is graded against. Each is its own decision with
its own measurement; neither follows from this one.

## Consequences

- **The SoC is mapped for an up5k, and `make fit` still is not.** The two instruments now differ in
  one more way, which is stated here so a later reader does not "align" them without measuring.
- **What this buys is not bankable.** It is worth −2.28% of median on the tree it lands on and
  +0.74% on a tree three commits back, so it does not put an item on ADR-0111's price list within
  reach — that list is priced in worst-seed margin, and the worst placement here moves at p = 0.125.
  A purchase re-takes its own price anyway.
- **Zero cycles, and checked rather than asserted.** No RTL changed and the grade only rewrites
  `specify` blocks, so `make cycles` (34 051 cycles, 18 352 retired, CPI 1.86) and `make dhrystone`
  (1 544 344 cycles, 0.757 DMIPS/MHz) are byte-identical either way, and `make fit` reads 3945
  either way because its line is untouched.
- **The equivalence claim is graded, not argued**: `make -C formal nonperturbation`, the four
  component proofs, the 86 generated checks against `EXPECTED_FAIL` and `EXPECTED_CHECKS`,
  `complete`, `complete_cover`, `make test`, `test-units`, `probe-gates`, `mutation-check`,
  `window-test`, `cosim-suite`, `lint` and `waves` all pass with the grade named.
- **A synthesis flag is a measurement with a date on it, the same as an RTL edit.** Sixteen seeds on
  one tree said −2.71%, sixty-four on the next said +0.74%, thirty-two on the next said −2.28%. If
  this line is ever suspected of costing period, sweep it paired before touching it — the flag is one
  word to remove and the sweep is the only thing that says whether it should be.
- **The toolchain is not pinned.** A suite bump changes abc9's mapping before it changes anything
  else here, so it belongs on the list of causes when `soc-timing` moves.
