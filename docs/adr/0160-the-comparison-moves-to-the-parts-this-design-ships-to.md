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

**Neither part's clock is continuous; what differs is how coarse the grid is.** up5k runs from a
12 MHz crystal, and `SB_HFOSC` offers
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
difference is how COARSE the part's clock grid is.

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
- **ECP5 — "what is architecturally faster when the clock is close to yours to choose".** Its clock
  is not continuous either: `EHXPLLL` synthesises `ref × M / N / D` on integer dividers from a
  board's fixed reference, so a design still rounds DOWN to a reachable output. The difference from
  up5k is degree, not kind — a fine grid costs a fraction of a percent where four frequencies cost
  half the machine — and it is enough that a critical-path advantage survives here. All three fit at the
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

## Amendment, 2026-09-05 — the VexRiscv this was measured against was not a peer

The figures above are correct about the PARTS and wrong about the RESULT, and the reason is the
opponent's configuration rather than anything on either part.

`formal/riscv-formal/cores/VexRiscv/VexRiscv.v` is generated from **`FormalSimple`** — riscv-formal's
own *verification* configuration. Read its plugin list: **no `MulPlugin`, no `DivPlugin`, no
`CsrPlugin`, and every one of `HazardSimplePlugin`'s four bypasses disabled.** So this core's
RV32IMAC_Zicsr_Zifencei_Zkt, with traps, a full mandated M-mode CSR set, a timer and executor-only
forwarding, was being measured against something with no multiplier, no privileged architecture and
**no register forwarding at all**.

That distorts BOTH halves at once, in opposite directions: it flatters VexRiscv on period (no bypass
network, no CSR file and no multiplier to place) and flatters this core on cycles (nothing forwards
there, so everything stalls).

`soc/compare/vexriscv/GenLittleCpuCompare.scala` replaces it, on the principle that **each core
should be in the configuration its own authors ship for performance, at a comparable ISA** —
`GenFullNoMmuNoCache` with all four bypasses on, plus `FormalPlugin` because this bench reads
`rvfi_*` to count Dhrystone's writes, plus `compressedGen`, minus the `DebugPlugin` the bench does
not wire. It is deliberately NOT hobbled to match this core's narrower forwarding; picking a weaker
config for the other core earns the same criticism in reverse.

| | FormalSimple (what was measured) | comparable (what ships) |
|---|---|---|
| M extension | **none** | `MulPlugin` + `DivPlugin` |
| CSRs / traps | **none** | `CsrPlugin` |
| hazard forwarding | **none** | all four bypasses |
| cycles per Dhrystone | 1021.9 | **640.1** |
| DMIPS/MHz | 0.590 | **0.889** |
| up5k, worst of 12 | 18.37 MHz | **21.34 MHz** |
| ECP5 | 47.49 MHz | **54.77 MHz** |

**It is better on both halves at once**, which is the shape of a configuration change rather than a
design change. It is also *faster* despite being much larger, because `FormalSimple` uses
`DYNAMIC_TARGET` prediction with a 1024-entry block-RAM predictor and `GenFullNoMmuNoCache` uses
`STATIC`: block RAM falls 26 → 12 and the predictor's path goes with it.

### The corrected result

| | up5k (all quantise to 12 MHz) | ECP5 |
|---|---|---|
| **vexriscv** | **10.67 DMIPS** | **48.69** |
| littlecpu | 9.35 | 26.93 |
| hazard3 | 8.54 | 33.26 |
| vs vexriscv | **1.14× THEIRS** | **1.81× THEIRS** |
| vs hazard3 | 1.09× ours | 1.24× theirs |

**What survives from the body above:** hx8k cannot hold this design; the product it reported was a
hybrid of two netlists; up5k's clock is quantised and no core reaches the 24 MHz step, VexRiscv's
21.34 included, so its clock advantage is still discarded there. Every one of those is unchanged.

**What does not survive is the conclusion.** This core does not win on up5k. It lost the cycle half
the moment the opponent was given forwarding, and the 1.32× reported above was an artifact of
measuring against a core that had none. It still leads hazard3 on up5k, and it trails both on ECP5.

**The lesson is the one this ADR was already about, turned on its author.** ADR-0160 corrected a
harness that measured the wrong PART and did not think to ask whether it measured the wrong
CONFIGURATION. A comparison is only as good as its least examined assumption, and "the vendored core
is a reasonable opponent" had never once been checked.

`soc/compare/vexriscv_pin.mk` pins the upstream SHA, the generator config and a digest of the
generated Verilog, because a generated artifact is reproducible only with all three.

## Amendment, 2026-09-06 — the shared ISA widens to RV32IM

Both benchmark images were narrower than the three cores actually share. Dhrystone's
`COMPARE_DHRY_CFLAGS` was `rv32i`, dating from when the vendored VexRiscv had no `MulPlugin`; the
amendment above gave it one. CoreMark's `COMPARE_COREMARK_CFLAGS` was `rv32ima`, and the **A** in it
is what excluded VexRiscv from that harness entirely — its generated build has no `AtomicPlugin`.
Verified against each source directly:

| | I | M | A | C |
|---|---|---|---|---|
| littlecpu | yes | yes | yes | yes |
| vexriscv (generated peer build) | yes | yes | **no** | yes |
| hazard3 (iCE40, `bench_hazard3.v` params) | yes | yes | yes | **no** |

**RV32IM is the widest ISA all three implement in hardware, on both benchmarks.** Both
`COMPARE_DHRY_CFLAGS` and `COMPARE_COREMARK_CFLAGS` now read `-march=rv32im`, and CoreMark's
`coremark_tb.v` gains a third DUT (`bench_vexriscv`), reusing `soc/compare/dhry_monitor.v` — the
marker mechanism built for VexRiscv's Dhrystone gap — rather than inventing a second one; the wiring
is the whole change.

**A route-3 attempt to also remove Hazard3's disclosed adapter wait was tried and reverted in the
same window this amendment was written** — see `docs/adr/0146-*.md`'s own amendment for the full
account. Every number below is measured on the adapter this ADR's own body already describes:
`bench_hazard3.v` is confirmed byte-identical (checksum) to the tree `make compare-dhrystone` first
measured it on, so Hazard3's disclosed wait share stays 9.01% of Dhrystone's cycles and 1.98% of
CoreMark's, exactly as stated above — the ISA widening is the only variable moving in this amendment.

### What widening Dhrystone's own ISA costs or buys, isolated

Dhrystone multiplies little, so this is measured rather than assumed — the same tree, same
`compare.dhry.vvp`, `COMPARE_DHRY_CFLAGS` toggled between `rv32i` and `rv32im` and nothing else:

| | RV32I cyc/dhry | RV32IM cyc/dhry | move |
|---|---|---|---|
| **littlecpu** | 731.1 | 727.1 | −0.55% |
| vexriscv | 640.1 | 635.1 | −0.78% |
| hazard3 | 799.1 | 829.1 | **+3.75%** |

littlecpu and vexriscv both get slightly *faster* with real multiply instructions, the direction a
reader would guess. **Hazard3 gets slower.** `hazard3_muldiv_seq.v`'s sequencer has no early exit —
`MULDIV_UNROLL=1` runs a fixed `XLEN`-iteration shift-accumulate loop regardless of the operands'
values — so its hardware multiply pays the same latency every time, where libgcc's software routine
apparently resolves faster for whatever multiplicands Dhrystone's own workload happens to hand it.
That is a plausible mechanism, read from the sequencer's source, not a traced instruction stream —
the exact call sites and operand values are unmeasured here. The net effect on the three-way ratio
(1.093× RV32I → 1.140× RV32IM) is real and belongs to the RV32IM row below, not folded silently into
"the ISA widened."

### Both cycle halves, one tree, RV32IM

400 Dhrystone runs, 1 CoreMark iteration, this tree:

| | Dhrystone cycles | DMIPS/MHz | CoreMark cycles | CoreMark/MHz |
|---|---|---|---|---|
| **littlecpu** | 290825 | 0.783 | 433240 | 2.308 |
| vexriscv | 254026 (0.873×) | 0.896 | 427008 (0.986×) | 2.342 |
| hazard3 | 331632 (1.140×) | 0.686 | 714984 (1.650×) | 1.399 |

All three verdicts PASS and every data RAM matches littlecpu's, both benchmarks. Hazard3 discloses
8.69% of its Dhrystone cycles and 1.98% of its CoreMark cycles in a bus wait state the other two
cores do not pay — the same disclosure this ADR's body and `docs/adr/0146-*.md` already carry, not a
new cost from the ISA widening (the 8.69% here against the 9.01% elsewhere is RV32IM's own slightly
larger cycle count changing the denominator; the wait cycle COUNT, 28805, is identical either ISA).

### The clock half — up5k, twelve seeds (`default`, `1`–`11`), same tree

| | worst | median | best | step reached |
|---|---|---|---|---|
| **littlecpu** | 12.40 MHz | 12.85 MHz | 13.23 MHz | **12** |
| vexriscv | 21.92 MHz | 22.78 MHz | 23.65 MHz | **12** |
| hazard3 | 12.58 MHz | 13.04 MHz | 13.67 MHz | **12** |

**All three still reach the 12 MHz step, at every one of twelve seeds each.** Hazard3's own figures
here reproduce the ADR body's original 12.56–13.18 MHz shape closely (12.58–13.67 against a
five-seed 12.56–13.18) — the small movement is sample noise between a five-seed and a twelve-seed
sweep, not a design or harness change; `bench_hazard3.v` is untouched.

ECP5 (`LFE5U-25F-6CABGA381`, one placement each, this session's cached toolchain): littlecpu
33.23 MHz, vexriscv 57.64 MHz (both reproduce the amendment above exactly), hazard3 **48.50 MHz**,
reproducible on repeated runs against the same RTL (confirmed byte-identical to the ADR body's own
tree). That is a large move from the 33.26 MHz this ADR originally quoted for the same design.
**ECP5 has no derived band and its own toolchain — nextpnr-ecp5 — is the one tool this repository
does not pin** (`CLAUDE.md`'s toolchain section says so explicitly), so a version difference between
whenever the original 33.26 MHz was taken and this session's cached build is the most likely
explanation, though it is not confirmed against a recorded version number on either side. Reported
as measured, not reconciled with the older figure; a re-take with a pinned or explicitly-versioned
ECP5 toolchain is owed before this number is spent on a decision.

### The product, both parts, at the step each core actually reaches

| | up5k DMIPS | up5k CoreMark | ECP5 DMIPS | ECP5 CoreMark |
|---|---|---|---|---|
| **littlecpu** | 9.39 (@12) | 27.70 (@12) | 26.01 | 76.70 |
| vexriscv | 10.75 (@12) | 28.10 (@12) | 51.66 | 134.99 |
| hazard3 | 8.24 (@12) | 16.78 (@12) | 33.29 | 67.83 |

up5k: vexriscv is **1.14× littlecpu** on Dhrystone (10.75/9.39, materially unchanged from the
amendment above's 10.67/9.35) and **1.01×** on CoreMark, the first time that pair has a CoreMark
product at all. Littlecpu is **1.14× hazard3** on Dhrystone (9.39/8.24) and **1.65× hazard3** on
CoreMark (27.70/16.78) — both pairs at the same 12 MHz step, so the ratio is the cycle half alone;
CoreMark separates the pair the way it already did in `docs/adr/0146-*.md`'s own product, because it
leans on the M extension Hazard3's `MULDIV_UNROLL=1` sequencer pays for one bit at a time.

**ECP5 reverses one pairing Dhrystone shows on up5k.** With Hazard3's ECP5 clock measured this
session at 48.50 MHz — the figure the paragraph above flags as unreconciled with the ADR's own
33.26 MHz — Hazard3's Dhrystone product (33.29) is **ahead of littlecpu's (26.01)**, 1.28×, despite
taking 1.140× more cycles per Dhrystone: its clock advantage on this session's ECP5 measurement more
than offsets the cycle deficit. CoreMark does not reverse (76.70 against 67.83, littlecpu still
1.13× ahead) because CoreMark's cycle gap (1.650×) is wider than Dhrystone's (1.140×) and the same
clock ratio cannot close it. **This ECP5 Dhrystone reversal inherits the paragraph above's own
caveat in full** — it is a product of a clock figure this ADR does not yet trust enough to reconcile
with its own prior measurement, and should be read as "not yet settled," not as a finding about
Hazard3's architecture, until the ECP5 toolchain question above is closed. vexriscv leads both cores
on ECP5 regardless: 1.99× littlecpu and 1.55× hazard3 on Dhrystone, 1.76× and 1.99× on CoreMark.

**Not restamped**: `soc/compare/product.json` is already stale on its own check against this tree
(base `122ef7b`, dirty, pre-peer-VexRiscv, hx8k-derived clocks) and re-taking that stamp is a
separate ticket's job. Every number above is this session's own fresh run, quoted rather than read
off the artifact.
