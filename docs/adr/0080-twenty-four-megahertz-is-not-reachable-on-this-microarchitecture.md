# ADR-0080: Twenty-four megahertz is not reachable on this microarchitecture

**Status:** Accepted · 2026-08-08 · *Answers a question three spikes have already answered
separately and nothing has recorded. Reads
[ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md),
[ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md) and
[ADR-0078](0078-a-one-deep-kill-is-cheap-and-buys-a-clock-the-board-cannot-use.md) as its evidence,
and adds a prior-art survey plus one synthesis-flag null of its own. Nothing in `rtl/`, `formal/`
or `soc/` changes. Graded against the 24 MHz target `CLAUDE.md` records under
Measurements and ratchets, not against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement, which is met.*

## The question

"Can we hit 24 MHz?" 24 is the divider step above the board's 12, so it is the only faster clock a
change would be aiming at, and `CLAUDE.md` names 41.67 ns as the target for Fmax work. The question
has now been asked three times, answered three times by measurement, and written down nowhere as a
verdict. That is exactly how the decode head came back after ADR-0058 asserted it without measuring
it, and ADR-0076 had to spend four to eight placements per variant to retire it.

Nothing here changes the core. This ADR collects results that already exist, adds a survey of what
comparable cores actually place at on this part, and retires the question the way ADR-0076's
dead-end list retires an individual candidate.

## Decision

**24 MHz is not reachable on this microarchitecture.** The honest ceiling for the whole family of
changes that takes the pc off this cycle's decode is **16.32 MHz / 61.27 ns**, and reaching even
that costs the no-wrong-path-state commitment. That ceiling is **47% longer** than the 41.67 ns
24 MHz needs.

A future proposal is welcome and is measured against 41.67 ns, not against "faster than today". What
this ADR forbids is nothing; what it supplies is a bar and a starting number, so the next attempt
does not re-derive them.

## The three spikes, and what each one already measured

| result | what was built | what it measured | verdict |
|---|---|---|---|
| ADR-0074 | register file read off the guessed next instruction | CPI 2.07 → **1.79**; **11.56 MHz** at one placement | misses the 12 MHz requirement |
| ADR-0076 | the whole decode head deleted, function-breaking | **−3.3%** of period, eight placements against eight | inside the 3.6% churn band |
| ADR-0078 | a registered fetch address with a one-deep kill, built | **13.32 MHz**, +1.9% CPI, +124 cells | declined |
| ADR-0078 | that family's ceiling, including a skid buffer | **16.32 MHz / 61.27 ns** | 47% short of 41.67 ns |

Read them together and the shape is clear. ADR-0074 attacked cycles and lost clock. ADR-0076 deleted
the named lever outright and got a number inside the instrument's noise; it also established that no
single input to `next_pc` is worth more than 5% alone and that all of them together are worth 21%.
ADR-0078 collected that 21% at its ceiling and landed at 61.27 ns. Three different attacks, one
plateau.

**At the 16.32 MHz ceiling the residual path is 16 logic levels, ending in CSR commit**
(`por_done → riscv.csrs.mstatus_mie`). That is 3.83 ns per level on this routing-dominated fabric,
against the ~3.3 ns `CLAUDE.md` records. So 41.67 ns buys about **11 or 12 levels for every path in
the design**, and the best path this direction can produce is already 16. The gap is not one more
lever.

## What reaching 24 would actually take

Stated so a proposal can be checked against it rather than against optimism:

- Every path in the design under ~12 logic levels, including the CSR commit path that surfaces once
  the fetch loop leaves.
- The pc off this cycle's decode, which is the no-wrong-path-state commitment — priced by ADR-0078
  at 1.9% of CPI, one re-stated proof, and a worse read.
- A kill network deeper than one cycle, because a longer front end means more than one word in
  flight behind a redirect. ADR-0078's measurement says nothing about that case: its kill lands
  *before* issue, and it says so explicitly.
- `make cycles`'s taxonomy extended with a second category. ADR-0078 found that a cycle which issues
  nothing without raising `stall` is silently counted as an issue cycle, and every grader stays
  green.

Any one of those is an ADR. Together they are a rewrite.

## What comparable cores place at on this part

Measured here rather than quoted, because the source's own documentation disagreed with its own
build config.

**FemtoRV, iceBreaker configuration.** The tutorial claims 25 MHz. The checked-in
`RTL/CONFIGS/icebreaker_config.v` says `NRV_FEMTORV32_ELECTRON` — **RV32IM**, no compressed
instructions, no interrupts — with `NRV_FREQ 20` and the comment *"Recomm: 15 MHz Overclocking:
20-25 MHz"*. Built and placed on the same toolchain as this repo's numbers: Homebrew Yosys 0.68+post
(`c12172fb`), nextpnr-0.10-108-g68c1acd8, `icetime` from the same install.

| | FemtoRV ELECTRON | this SoC |
|---|---|---|
| `icetime` | **56.44 ns — 17.72 MHz** | 78.51 ns — 12.74 MHz |
| nextpnr's own estimate | 18.43 MHz | 13.21 MHz |
| placed `ICESTORM_LC` | 2382 / 5280 (45%) | 4383 / 5280 (83%) |
| SPRAM / EBR | 4/4 · 4/30 | 2/4 · 20/30 |
| PLL | 1 | 0 |
| ISA | RV32IM | RV32IMC_Zicsr_Zifencei |
| M-mode CSRs and traps | `cycle`/`cycleh` only, no traps | full mandated set, five traps |
| instruction source | SPI flash (`NRV_RESET_ADDR 32'h00820000`) | block RAM |

Three caveats belong with that 17.72 ns number and none of them is small:

- **Their PLL module had to be rewritten to build under current yosys.** The generated
  `pll_icebreaker.v` puts `defparam` inside a `generate case`, which current yosys does not resolve.
  It was replaced with a parameter list carrying **their own constants for the `freq = 20` case**
  (`DIVR = 0`, `DIVF = 7'b0110100`, `DIVQ = 5`, `FILTER_RANGE = 1`). Faithful, but not literally
  their file.
- **Their own configuration does not meet its own clock here.** Those constants give
  12 × 53 / 2⁵ = **19.875 MHz**, and the placement times at 17.72. So the 25 MHz in the tutorial is
  not reproduced, and neither is the 20 in the config.
- **Their nextpnr runs `--freq 12 --opt-timing`**, so it stopped optimising once it cleared 12. A
  higher target might place better, and this measurement does not bound what their core can do.

**The rest of the field, from published sources and not reproduced here.** Quoted as claims, not as
measurements: PicoRV32 / PicoSoC at about 12 MHz on up5k, which is multi-cycle and spends roughly
twice this core's 2.07 cycles per instruction at the same clock; SERV at 16 MHz, which is
bit-serial at about 32 cycles per instruction; VexRiscv's 63–92 MHz figures, which are **hx8k-class
fabric and not up5k** and so are not a comparison at all.

**This survey found no published result running a five-stage single-issue RV32IMC with a same-cycle
redirect at 24 MHz on an up5k.** Every core that gets near that clock on this part does
categorically less per cycle — no compressed decode, or a serial datapath, or a multi-cycle
sequencer. That is not proof none exists. It is the state of the search, and it agrees with the
three ceilings above.

## One more measured null: FemtoRV's synthesis flags do not help us

FemtoRV synthesizes with `-abc9 -device u`, and its nextpnr adds `--opt-timing`. Tried on this SoC,
one placement each, same toolchain, `icetime` as the instrument:

| synthesis | LC | `icetime` |
|---|---|---|
| `synth_ice40 -dsp -spram` (what ships) | 4383 | 78.51 ns — 12.74 MHz |
| `+ -abc9 -device u` | 4363 | 81.53 ns — 12.27 MHz |
| `+ --opt-timing` on top | 4363 | **85.05 ns — 11.76 MHz** |

The path length is 24 logic levels in all three. Single placements are a sample and the spread is
1–2%, so this is indicative rather than settled — but the direction is consistent across the three
rows, the last one is **below the 12 MHz requirement**, and nothing here is a free win. It goes on
ADR-0076's dead-end list so the flags are not tried again on the strength of another project using
them.

## Consequences

- **The question is retired, and the bar it is retired against is 41.67 ns.** A candidate in this
  space is read against that number first. ADR-0076's rule still applies underneath: bring a
  ceiling for the whole set or expect a null.
- **The dead-end list grows by one, and it is a flag change, not a design change.**
  `-abc9 -device u` with or without `--opt-timing` — **slower on this design at every row measured,
  and below the requirement with `--opt-timing`.**
- **ADR-0078's "there is nothing in between to land on" is narrower than it reads, and the repo has
  already corrected it.** `SB_HFOSC` has no step between 12 and 24, but an `SB_PLL40` fed by the
  crystal does — recorded in `CLAUDE.md` at commit `1ecbd49`, verified there with `icepll` and a
  placed `SB_PLL40_PAD`. So a design landing at 13 or 16 MHz is not automatically worthless; it
  costs the part's one PLL, which FemtoRV spends and this SoC does not. **ADR-0078's decline does
  not depend on that sentence** — it declines on all four goals failing to move together, with
  readability measured worse and a proof re-stated — and its numbers are unchanged. Nothing here
  reopens it. The correction is repeated here because this ADR quotes the sentence's neighbourhood
  and a reader will meet both.
- **The comparison is not like for like and must not be quoted as one.** FemtoRV's 17.72 MHz is
  RV32IM with no compressed decode, one CSR, no traps, and instruction fetch from SPI flash, at
  45% of the part against this SoC's 83%. The gap it shows is what a smaller core buys, not what
  this core is leaving on the table.
