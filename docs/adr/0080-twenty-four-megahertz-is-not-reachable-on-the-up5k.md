# ADR-0080: Twenty-four megahertz is not reachable on the up5k

**Status:** Accepted · 2026-08-08 · *Answers a question three spikes have already answered
separately and nothing has recorded. Reads
[ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md),
[ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md) and
[ADR-0078](0078-a-one-deep-kill-is-cheap-and-buys-a-clock-the-board-cannot-use.md) as its evidence,
and adds one cross-fabric measurement, a prior-art survey and a synthesis-flag null of its own.
Nothing in `rtl/`, `formal/` or `soc/` changes. Graded against the 24 MHz target `CLAUDE.md` records
under Measurements and ratchets, not against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement, which is met.*

## The question

"Can we hit 24 MHz?" 24 is the divider step above the board's 12, so it is the only faster clock a
change would be aiming at, and `CLAUDE.md` names 41.67 ns as the target for Fmax work. The question
has now been asked three times, answered three times by measurement, and written down nowhere as a
verdict. That is exactly how the decode head came back after ADR-0058 asserted it without measuring
it, and ADR-0076 had to spend four to eight placements per variant to retire it.

Nothing here changes the core. This ADR collects results that already exist, adds the one
measurement that says where the period actually goes, and retires the question the way ADR-0076's
dead-end list retires an individual candidate.

## Decision

**24 MHz is not reachable on the up5k, and the binding constraint is the part rather than the
design.** The ceiling for the whole family of changes that takes the pc off this cycle's decode is
**16.32 MHz / 61.27 ns** on up5k, and reaching even that costs the no-wrong-path-state commitment.
That ceiling is **47% longer** than the 41.67 ns 24 MHz needs.

**The same RTL placed on an hx8k runs at 31.08 MHz over 22 logic levels** — near-identical depth to
the shipping up5k build's 24, at **1.46 ns per level against 3.27**. So this microarchitecture
already clears 24 MHz, unmodified, on a faster fabric. What it cannot do is clear it on the up5k,
whose interconnect is about 2.2× slower per logic level.

The distinction decides where a future reader looks. **Do not go hunting for 47% of period inside a
design that is already delivering it somewhere else.** A proposal to reach 24 MHz on the up5k is
still measured against 41.67 ns, and this ADR forbids nothing — it supplies the bar, the starting
number, and the reason the bar is where it is.

## The three spikes, and what each one already measured

All three are up5k measurements, which is exactly the scope of the claim above.

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
(`por_done → riscv.csrs.mstatus_mie`). That is 3.83 ns per level, against the ~3.3 ns `CLAUDE.md`
records and the 3.27 the shipping build measures. So on this part 41.67 ns buys about **11 or 12
levels for every path in the design**, and the best path this direction can produce is already 16.

## The same RTL on an hx8k

One build, same tree, same toolchain — Yosys 0.68+post (`c12172fb`), nextpnr-0.10-108-g68c1acd8,
`icetime` from the same install.

| | logic levels | `icetime` | ns per level | placed LC |
|---|---|---|---|---|
| up5k sg48, shipping | 24 | 78.51 ns — **12.74 MHz** | 3.27 | 4383 / 5280 (83%) |
| **hx8k ct256** | **22** | **32.18 ns — 31.08 MHz** | **1.46** | 7393 / 7680 (96%) |

`nextpnr` reports 30.45 MHz for the hx8k placement; `icetime -d hx8k -P ct256` reports 32.18 ns over
22 logic levels. **Same RTL, near-identical logic depth, 2.4× the clock.** The up5k costs **2.2× the
delay per logic level**, and its 3.27 ns is a direct confirmation of the ~3.3 ns `CLAUDE.md`
records — a figure that until now had nothing to be compared against.

Three caveats belong with the 31 MHz, and all three work *against* it rather than for it:

- **The data RAM is 4 KB, not 64 KB** — `RAM_WORDS` 16384 → 1024, the only RTL difference from what
  ships. hx8k's entire block RAM is 32 × 4 kbit = 16 KB, and the ROM and register file already claim
  20 of its 32 blocks. This is a smaller-memory variant of the SoC, not the SoC.
- **No `-dsp`**, because hx8k has no `SB_MAC16`. The multiplier expands into LUTs: 3921 `SB_LUT4`
  and 4 `SB_MAC16` on up5k become **6944 `SB_LUT4` and no DSP**.
- **96% utilisation**, against the up5k build's 83%. Routing congestion on a nearly-full device is
  pressure on the critical path, not help.

A smaller-memory variant, with the multiplier in soft logic, on a device that is 96% full, is
**still 2.4× faster**. The fabric is the variable that matters.

**This does not mean the project should change parts.** The board is what it is, the crystal is
12 MHz, and none of the deferred work — the bootloader, the forwarding network, the radix-4
divider, interrupts — is unblocked by an FPGA nobody has. What the measurement settles is the
*reason*, and therefore where not to look.

## What reaching 24 on the up5k would take

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

Any one of those is an ADR. Together they are a rewrite — and the hx8k number says the rewrite would
be buying back what the fabric is charging, not fixing something wrong with the design.

## What comparable cores place at

Measured here rather than quoted, because the source's own documentation disagreed with its own
build config.

**FemtoRV, iceBreaker configuration.** The tutorial claims 25 MHz. The checked-in
`RTL/CONFIGS/icebreaker_config.v` says `NRV_FEMTORV32_ELECTRON` — **RV32IM**, no compressed
instructions, no interrupts — with `NRV_FREQ 20` and the comment *"Recomm: 15 MHz Overclocking:
20-25 MHz"*. Built and placed on the same toolchain as this repo's numbers.

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

Three caveats belong with that 17.72 MHz and none of them is small:

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

**PicoRV32 and SERV, from published sources and not reproduced here.** Quoted as claims:
PicoRV32 / PicoSoC at about 12 MHz on up5k, which is multi-cycle and spends roughly twice this
core's 2.07 cycles per instruction at the same clock; SERV at 16 MHz, which is bit-serial at about
32 cycles per instruction.

**VexRiscv is now a same-fabric comparison, and it is the useful one.** Their published 63–92 MHz
figures are hx8k, and so is the 31.08 MHz above — **a real 3× gap on the same silicon**, which is a
far more honest number than the fabric mismatch that previously had to be disclaimed away. What
accounts for it is stated rather than guessed: **every VexRiscv iCE40 number is RV32I**, no M
extension and no compressed, and their RV32IM rows are Artix-7 again. The rows are 1130 LC at
92 MHz for "small, no datapath bypass, no interrupt", 1292 at 85 with interrupts, and 1596 at 63 for
the productive configuration. That 92 MHz configuration has **no datapath bypass**, the same
stall-only choice this core makes, so the field's nearest architectural relative is 1130 LC of RV32I
against our 6944 `SB_LUT4` carrying M, C, the mandated M-mode CSR set, precise traps and a soft
multiplier.

**The core alone still cannot be placed on hx8k, or anywhere.** `littlecpu` synthesizes for hx8k at
6889 / 7680 LC and then fails on pins rather than logic — **265 `SB_IO` against ct256's 206**, the
largest hx8k package, with `ERROR: Unable to find a placement location for cell
'imem_addr[24]$sb_io'`. That is the same reason `make fit`'s top never places on up5k, and it is why
the hx8k number above is the SoC and not the core.

**This survey found no published result running a five-stage single-issue RV32IMC with a same-cycle
redirect at 24 MHz on an up5k.** Every core that gets near that clock on this part does
categorically less per cycle — no compressed decode, or a serial datapath, or a multi-cycle
sequencer — and the fastest-looking counter-example is a different fabric *and* a smaller ISA. That
is not proof none exists. It is the state of the search, and it agrees with the three ceilings.

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

- **The question is retired for the up5k, and the bar it is retired against is 41.67 ns.** A
  candidate is read against that number first. ADR-0076's rule still applies underneath: bring a
  ceiling for the whole set or expect a null.
- **The claim is about the part, not the microarchitecture, and the difference is load-bearing.**
  "Not this microarchitecture" points a reader at redesigning the core; "not this part" points at
  the fabric. The three ceilings are all up5k measurements and support only the narrower claim. The
  same RTL is 31.08 MHz on hx8k with no change at all.
- **The dead-end list grows by one, and it is a flag change, not a design change.**
  `-abc9 -device u` with or without `--opt-timing` — **slower on this design at every row measured,
  and below the requirement with `--opt-timing`.**
- **The up5k's cost is now a number: ~2.2× the delay per logic level.** That is the first
  cross-fabric measurement this project has, and it is what makes `CLAUDE.md`'s ~3.3 ns per LUT
  level mean something. A future estimate of what a change is worth can use it.
- **The fair comparison against VexRiscv is still owed.** Ours is now on their fabric, but theirs
  has never been placed on ours. It was not attempted because VexRiscv is SpinalHDL and generating
  its Verilog needs a Scala toolchain — an **open route**, not a dead end.
- **ADR-0078's "there is nothing in between to land on" is narrower than it reads, and the repo has
  already corrected it.** `SB_HFOSC` has no step between 12 and 24, but an `SB_PLL40` fed by the
  crystal does — recorded in `CLAUDE.md` at commit `1ecbd49`, verified there with `icepll` and a
  placed `SB_PLL40_PAD`. So a design landing at 13 or 16 MHz is not automatically worthless; it
  costs the part's one PLL, which FemtoRV spends and this SoC does not. **ADR-0078's decline does
  not depend on that sentence** — it declines on all four goals failing to move together, with
  readability measured worse and a proof re-stated — and its numbers are unchanged. Nothing here
  reopens it. The correction is repeated here because this ADR quotes the sentence's neighbourhood
  and a reader will meet both.
- **The FemtoRV comparison is not like for like and must not be quoted as one.** Its 17.72 MHz is
  RV32IM with no compressed decode, one CSR, no traps, and instruction fetch from SPI flash, at 45%
  of the part against this SoC's 83%. The gap it shows is what a smaller core buys, not what this
  core is leaving on the table.
