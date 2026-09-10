# ADR-0172: The pinned placement is what `SOC_MIN_MHZ` grades

**Status:** Accepted · 2026-09-07 · *Amends [ADR-0066](0066-twelve-megahertz-is-a-requirement.md).
The requirement is unchanged — `SOC_MIN_MHZ` stays 12.0, the board crystal, with 6 MHz the
next `SB_HFOSC` step down. Only what is measured against it changes: `make soc-timing`, with
no `SOC_SEED` override, now grades one recorded placement rather than an unseeded run.*

## The measurement

Sixteen seeds each, one toolchain (Yosys 0.68+48 ff5817c34, nextpnr-ice40 0.11-1-g62e659ed):

| Netlist — identical RTL semantics | cells | worst | median | best | under 12.0 |
|---|---|---|---|---|---|
| `45f0b9b^` | 6,266 | 12.44 | 12.81 | 13.09 | 0 |
| `45f0b9b` (= `main` at the time) | 6,289 | **11.99** (seed 9) | 12.45 | 12.80 | **1** |
| `45f0b9b^` + 10 blank lines in `csrs.v` | 6,327 | 12.28 | 12.87 | **13.24** | 0 |

Three re-rolls of one netlist's RTL semantics — no functional change in any of the three —
span a worst-of-sixteen spread of about 3.7%, against a 3.5% clearance over `SOC_MIN_MHZ` at
the middle row's worst seed. One draw of three failed. `make soc-timing`'s worst-of-a-sweep
grading was never a property of the design; it is a property of which sixteen seeds happened
to be asked.

## Why the netlist moves under a blank line

Yosys names a generated cell after the source `file:line` it came from. `abc9_ops.cc` builds
a `TopoSort` over those names using a plain string comparison, and `write_xaiger` emits ABC's
input boxes in that order; ABC's `&dch`/`&if` passes are order-sensitive. Everything before
`abc9` runs is a pure renaming of the same logic — insert a blank line in `rtl/csrs.v` and no
gate changes meaning — but the renaming changes the string order the boxes are handed to ABC
in, and the mapped netlist follows. This is not a defect this repo introduced: the same
phenomenon is open upstream against sky130/OpenLane as
[yosys#3713](https://github.com/YosysHQ/yosys/issues/3713) since 2023, with a maintainer
description that fits this measurement exactly — "Yosys is deterministic, but chaotic" — and
it is deliberately exploited in the other direction by
[yosys#3277](https://github.com/YosysHQ/yosys/pull/3277)'s `rename -scramble-name`, which
exists to let a user reach a different point in the same chaos on purpose.

`soc/netlist_digest.py` (ADR-0122's line of work) already answers "did this edit change what
reaches the placer" for the comment and dead-net classes. It does not answer "does the
shipping build clear the board clock" — a netlist that digests unchanged still places to a
spread of placements, and this ADR's table is exactly that spread measured on one unchanged
netlist.

## Seeds 1..16 are not an independent sample — state the provable part, no further

nextpnr-ice40's placer RNG sets `rngstate = seed`, runs five `rng64()` warm-up calls, and then
serves outputs from a 64-bit xorshift generator. Xorshift's state *update* is linear over
GF(2): `state(3) = state(1) XOR state(2)` exactly, for any two seeds 1 and 2 and the seed 3
that is their XOR. Seeds 1..16 therefore span only a 4-dimensional subspace (basis 1, 2, 4, 8)
of the state's 64 dimensions — this is provable from the generator's definition and nothing
else is claimed about it.

**Whether that produces correlated placements is unmeasured.** The placer consumes many
successive `rng64()` outputs over the run, and each one is `state * constant` with carries —
a nonlinear operation — so a linear relationship among the seeds does not obviously survive
into a linear relationship among the placements. This ADR does not quote 3.7% as an estimate
of a distribution's spread, only as what sixteen *specific* draws produced. It does not weaken
the decision either way: the observed spread already exceeds the clearance regardless of how
correlated those sixteen draws turn out to be, so the case for pinning does not lean on the
unmeasured part.

## Why pinning, and not buying the margin back in the design

The margin this ADR would need to buy back is on the order of the whole edit-churn band, and
every attack on the fetch loop's period this repo has tried is priced and declined in
CLAUDE.md: the `next_pc` adder cost −103 SoC LUTs and missed 12 MHz at six of six seeds; the
roughly 21% a redirect-free PC would be worth requires the PC to stop depending on the current
cycle's decode, which is the no-wrong-path-state commitment (invariant 1) and is not on offer.
[ADR-0114](0114-the-critical-paths-routing-is-flat-and-block-ram-pinning-is-closed.md) measured
the critical path's own routing as flat, with no column to pin against. There is no cheaper
lever than the placer seed, and the placer seed is free.

Pinning `--seed` is ordinary practice for exactly this reason elsewhere: `orbcode/orbtrace`,
`bitcraze/lighthouse-fpga` (which ships its own `tools/pnr_until_close.sh`),
`SymbioticEDA/MARLANN`, `machdyne/zwolf`, `open-ephys/onix-breakout` and `daveshah1/pmods` all
pin a placer seed in their build. ADR-0066 already treats a seed as part of what a timing
number is quoted with ("Homebrew Yosys 0.67+post... four placements... on the branch").

## Decision

1. **`make soc-timing`, with no `SOC_SEED` override, places at a pinned seed and grades that
   one recorded placement.** `soc/pin.json` is the record: a netlist digest, the placer seed,
   the measured MHz, the toolchain string, and the full distribution the seed was chosen from.
2. **A pin is a claim about one netlist, not about the design's typical Fmax.** It is sound
   only while `soc.json`'s canonical form — `opt_clean -purge`, the same form
   `soc/netlist_digest.py` already hashes — still digests to what the pin recorded. A netlist
   that moved invalidates every placement recorded against the old one.
3. **Best-of-N is a biased order statistic, legitimate here only because the exact pinned
   configuration is what gets built.** It would misstate the design's Fmax as a general claim;
   it is sound as a description of the one artifact that ships, the same way a `git`
   commit's build is one specific set of bytes and not a sample of the source tree's typical
   behaviour.
4. **The pin's margin is required to be at least 5% over `SOC_MIN_MHZ`** — 12.6 MHz — so it
   survives ordinary toolchain drift: the pinned OSS CAD Suite floats, and twelve seeds have
   already been measured moving another core's placement 4.5% between two yosys builds on this
   harness (CLAUDE.md's cross-core comparison section). A candidate that cannot clear 12.6 MHz
   is not written as a pin.
5. **A digest mismatch is a distinct failure from a timing miss** — RE-PIN NEEDED, never
   phrased as "the design got slower" — because the two point a reader at different fixes:
   re-synthesise and re-place, versus find what lengthened the path.
6. **`SOC_MIN_MHZ` itself does not move.** It is still the board crystal; nothing about a
   chaotic mapper changes what clock the up5k actually has.

See `soc/soc_pin.py`'s module docstring for the mechanism and `soc/soc_seed_search.sh` for how
a pin is produced; this ADR is the argument, that file is the implementation.

## What shipped

`make soc-seed-search`'s default twelve-seed draw against today's netlist (digest
`sha256:cd83a72f9a212db18570649de878a96eac84bce803daf80f7276234767f81dd2`) read
11.91–12.61 MHz — every one of the twelve within the table's own ~3.7% spread, and the
worst of them (11.91 MHz) itself under the 12.0 floor, which is this ADR's argument
reproducing on the very sweep that picked the pin. The best, seed 125781539 at
**12.61 MHz**, clears `SOC_MIN_MHZ` by 5.08%: over the 5% floor this ADR's decision
requires, but only just, which is consistent with the placer-seed dimension being a
genuinely narrow lever on this tree rather than a wide one. `soc/pin.json` records that
seed, the digest, and the full twelve-seed distribution it was chosen from. Should a
future netlist's search be exhausted at count without reaching 5%, the next lever is
synthesis cell-name order (yosys's `rename -scramble-name`), named in the decision above
but not wired into `soc/soc_seed_search.sh` and not verified here to be seedable and
byte-reproducible; a future ADR should verify that before relying on it.

## What this does not say

- **It does not claim the design got faster, or slower.** The design is unchanged; what
  changed is which one placement of it CI is required to clear the floor at.
- **It does not replace `soc/timing_sweep.sh`.** An explicit `SOC_SEED=` — every row of a sweep
  included — still bypasses the pin and places unseeded or at the named seed, which is what a
  go/no-go over twelve to sixteen seeds (ADR-0121) still needs.
- **It is not a claim that 3.7% is *the* placement-noise band.** ADR-0106's 4–9% placement
  spread and `soc/bands.py`'s churn band are unaffected; this ADR's table is one more measured
  sample of the same phenomenon, at a margin narrow enough that the sample decided the
  question on its own.
