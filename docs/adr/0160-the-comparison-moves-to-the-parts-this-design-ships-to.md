# 0160 — The cross-core comparison moves to the parts this design ships to

Status: Accepted · 2026-09-05 · supersedes the hx8k half of
[ADR-0086](0086-both-cores-in-one-harness-and-the-gap-is-the-fetch-loop.md) and reverses the
direction of the product [ADR-0098](0098-dhrystone-on-both-cores-and-their-published-rate-reproduces.md)
has reported since

## What was wrong

ADR-0086 put the comparison on an hx8k for two stated reasons: it is where both sides already had a
number, and it is the iCE40-HX8K Breakout Board's own pin assignments. Both were true. Neither
survives contact with what the harness is actually used for.

**The hx8k cannot hold this design.** Measured on `122ef7b`: `littlesoc` synthesised for hx8k needs
**148 block RAMs against the part's 32**, because hx8k has no SPRAM and the 64 KB data RAM then
costs 128 blocks of the 32 available. It also has no `SB_MAC16`, so the multiplier falls to soft
logic and costs **+3,010 LUTs**. The dual configuration needs 14,299 LUTs and 168 block RAMs. What
fits an hx8k is the comparison bench and only because that bench was cut to a 4 KB ROM and a 2 KB
RAM to make it fit.

**So no program of consequence runs there.** Dhrystone's shipping build needs 12,384 bytes of RAM;
the largest data RAM an hx8k can hold alongside this core is 4 KB. `make compare-dhrystone` says so
on every run — *the image does not fit the placed geometry, so the cycles below are simulated at a
larger map than the clock they get multiplied by was measured at*. The product this repo has quoted
since ADR-0098 therefore multiplies a clock measured on one netlist by cycles measured on a
different one. That hybrid exists on no silicon and cannot be validated on any.

## What the step function does to the comparison

up5k's clock is not continuous. The board runs from a 12 MHz crystal, and `SB_HFOSC` offers
48 / 24 / 12 / 6. There is nothing between 12 and 24, so **Fmax above the requirement is margin and
not speed** — this repo already says exactly that (ADR-0066, ADR-0089), and it is what the whole
comparison turned on without anybody noticing.

Twelve placements a core on up5k/sg48, ROM 1024 words and RAM 16384 words in SPRAM, one toolchain:

| core | worst | median | best | step reached | short of 24 by |
|---|---|---|---|---|---|
| **littlecpu** | 12.60 | 13.14 | 13.74 | **12** | 42.8% |
| hazard3 | 12.56 | 12.97 | 13.18 | **12** | 45.1% |
| vexriscv | 18.37 | 19.10 | 19.89 | **12** | **17.1%** |

**No seed of any core reaches 24 MHz.** VexRiscv's best of twelve is 19.89 and still 17% short,
which is far outside any placement spread this part exhibits. All three quantise to the same step
and **all three run at 12 MHz on the board**. Each also clears the 12.00 MHz requirement at its own
worst placement, so this is not one design scraping in.

The comparison is then decided entirely by cycles, at the one ISA all three share:

| at 12 MHz | DMIPS/MHz | DMIPS |
|---|---|---|
| **littlecpu** | 0.779 | **9.35** |
| hazard3 | 0.712 | 8.54 |
| vexriscv | 0.590 | 7.08 |

**1.32× over VexRiscv and 1.09× over Hazard3, both in this core's favour** — against the hx8k
harness's 1.16× *against* us. Same two cores, same program, same seeds, same toolchain. The entire
difference is whether the part's clock is continuous.

**This is not a flattering measurement, it is a correct one.** VexRiscv is genuinely 1.53× this
core in raw critical path and that is not in dispute; ADR-0086 measured it and this ADR does not
overturn it. What changed is the recognition that the advantage is **unspendable** on a part whose
oscillator offers four frequencies, and that a harness crediting it was answering a question nobody
asks of this design.

## What ships instead

**Two parts, because they answer two different questions.**

- **up5k/sg48 — "what is fastest on the board this ships to".** The UPduino's own part and package.
  The bench needs three pads, a clock and two LEDs, and `soc/compare/bench_up5k.pcf` gives them
  `soc/upduino.pcf`'s own pin numbers, so a bitstream built from it runs on the board `make prog`
  already flashes. At ROM 1024 / RAM 16384 all three cores fit — 12, 12 and 26 block RAMs of 30 —
  and Dhrystone's 382-word image fits with room, so **the placed geometry and the simulated
  geometry are the same thing** and the product stops being a hybrid.
- **ECP5 — "what is architecturally faster on a large continuous fabric".** No step function there,
  so VexRiscv's critical path advantage is real and will likely still show. All three fit at the
  full 8 KB ROM and 64 KB RAM: 36, 36 and 40 `DP16KD` of 56.

Read them as two answers, never averaged. A core can be behind on one and ahead on the other, and
this one is.

## What this does NOT establish

- **No core has been run on silicon in this harness**, this one included. up5k makes that possible
  for the first time; it does not make it done. What has run on the UPduino is `littlesoc`, a
  different top (ADR-0130).
- **The ECP5 side has no derived band.** `soc/bands.py` refuses to answer for that part and still
  should; a delta there is not a change or a null until somebody sweeps it.
- **The clock halves remain `icetime`/nextpnr static estimates**, for every core equally. Silicon
  would validate the cycle half first, since all three would be clocked from the same crystal.
- The 12-seed figures above are the bench top, not `littlesoc`. `make soc-timing` remains the
  SoC's own instrument and its numbers do not merge with these.
