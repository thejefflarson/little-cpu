# ADR-0100: The early register write is built, buys 6.5% of Dhrystone, and costs the clock

**Status:** Accepted (the mechanism is declined) · 2026-08-11 · *Third measurement in the same
chain as [ADR-0083](0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md) and
[ADR-0092](0092-the-writeback-slot-costs-more-than-the-bypass-it-replaces.md): a change that shortens
the load-use or RAW shadow by touching the second fetch loop. All three are declined and all three
are declined on the clock. Built on top of
[ADR-0099](0099-the-memory-transaction-launches-from-the-execute-slot.md) and measured against it.*

## Context

After ADR-0099 removed the load turnaround, 253 431 Dhrystone cycles — 16.9% — are the decode
scoreboard waiting on a register. A large share of those wait on a result that is **already
finished** and sitting in `executor_out` one stage above a register-file write port that is doing
nothing that cycle, because the instruction retiring below it writes no register.

The design already owns the invariant that makes taking it legal. **No wrong-path state** means an
instruction that reached the executor is architecturally committed: nothing can un-commit it, traps
are detected and committed in decode, and an interrupt is taken before issue. So its result may
reach the register file early without any of the machinery a speculative core would need.

Nor does it need an operand mux, which is what separates it from ADR-0083's forwarding network. The
consumer issues in that same cycle and reads the value through the register file's **existing**
write-through bypass. There is no new path into the operand datapath at all.

## Decision

**Built, measured, declined.** The RTL is on the branch this ADR lands on (PR #164) and is taken
back out before it merges; the ADR is what survives it.

What was built: `writeback` takes `executor_out` as a second source. On a cycle the retiring
instruction is not writing (`wen` low), a valid `executor_out` with a non-`x0` `rd` whose result is
not still on the bus drives `waddr`/`wdata` instead. `executor_output` gains one bit, `rd_from_bus`,
because a load's result is not in it yet. Decode drops the executor slot from its scoreboard for
exactly that cycle. The retire writes the same value again next cycle, so there is no arbitration
state and nothing to reconcile.

### It works, and it buys what was modelled

| | before (ADR-0099's tree) | with the early write |
|---|---|---|
| suite | 28 092 cycles, CPI 1.80 | 25 534, CPI 1.63 (**−9.1%**) |
| Dhrystone | 1 607 619 cycles, CPI 1.69, 0.727 DMIPS/MHz | 1 503 228, CPI 1.58, 0.779 (**−6.5%**) |
| `make fit` | 3473 | 3482 (+9, inside the ±50 band) |

`make test`, `make test-units`, `make probe-gates`, `components_decoder`, `components_pcloop` with
its cover, and `components_traps` are all green on it. New `test/regfile_tb.v` vectors cover the two
shapes it produces — a write followed by an identical write to the same address, and a fetch cycle
coinciding with the early write — and pass. **This is not a correctness decline.**

### It costs the clock, and that is a requirement

Six placements a side, `soc/timing_sweep.sh`'s seeds run one at a time so a placement under the
floor still reports its number:

| | ns, sorted | worst | verdict |
|---|---|---|---|
| ADR-0099's tree | 73.34 73.93 76.16 77.17 77.21 78.40 | 78.40 (12.76 MHz) | 5.9% over |
| with the early write | 81.58 82.43 83.43 84.72 85.24 85.29 | 85.29 (**11.72 MHz**) | **under at four of six** |

**+9.4% of median period, and the distributions do not overlap.** That is two and a half times the
3.6% edit-churn band, so it is a measurement and not noise. The critical path is the fetch loop —
`imem.rom_odd RDATA` to `imem.rom_even RDATA`, 27 logic levels against the low twenties — which is
the loop CLAUDE.md names: `accessor_out.rd_data → writeback → the regfile's write-through bypass →
branch comparator → next_pc → ROM address`. The mechanism puts a 32-bit mux in front of `wdata` and
a 5-bit mux in front of `waddr`, and both feed that bypass.

**12 MHz is a requirement, not a regression floor** (ADR-0066). The next divider step down is 6 MHz,
so a design at 11.72 runs at half speed and the 6.5% cycle win becomes a 47% throughput loss. The
claim in the brief — that the source mux's select is a registered bit and the per-bit mux folds into
the LUT the 5-bit comparator already occupies — is what the sweep answers, and the answer is no.

### Against the four goals

**Fast** — it is faster in cycles and slower in seconds, and seconds are what the board runs on.
**Simple** — it adds a second write-port source, a struct bit and a scoreboard exception; the
scoreboard is no longer a plain "is it in flight". **Readable** — the exception is a paragraph, not
a line. **Formally verified** — it kept everything green, so it costs nothing here. One goal moves
forward, one clearly back, and the one that moves back is a stated requirement.

## Consequences

The mechanism does not ship. `executor_output` keeps no `rd_from_bus` bit and `rtl/writeback.v`'s
write port keeps one source.

**What survives is a measured ceiling.** Anything that lets a committed result reach the register
file before its own retire is worth **9.1% of suite cycles and 6.5% of Dhrystone's** on this
datapath, and it has to cost less than 9.4% of period to be worth taking. That is the third entry in
one table:

| change | cycles | median period | shipped |
|---|---|---|---|
| forwarding to every operand reader (ADR-0083) | −12.9% suite | 9.49 MHz | no |
| forwarding confined to the executor (ADR-0083) | −7.5% suite | 0.48% margin | no |
| a fourth scoreboard slot replacing the bypass (ADR-0092) | +19.5% suite | −2.8% | no |
| **the early write (this)** | **−9.1% suite, −6.5% Dhrystone** | **+9.4%** | **no** |

Every one of them touches the bypass or the loop it sits in, and every one is declined. **The next
idea in this direction should be one that does not**, and the measurement to beat is what this
branch ships instead: ADR-0099 alone is −11.8% of Dhrystone at 12 MHz.
[ADR-0101](0101-the-successor-pair-is-learned-and-deferred-for-margin.md) is a fourth entry in this
chain that does *not* touch the bypass, holds the clock, and is deferred for margin rather than
declined — which is the shape to look for.

One residual worth recording rather than guessing at: the +9.4% was not attributed between the two
halves of the mechanism — the muxes in front of the write port, and the `!early_write` term the
scoreboard gained. A spelling that kept the write and dropped the scoreboard relaxation would buy no
cycles, so the attribution is only interesting to a future variant that finds a cheaper relaxation,
and it was not measured here.

## Amendment, 2026-09-06: re-taken on the forwarding tree, and it still misses the clock

Rebuilt from this ADR's own "What was built" paragraph on top of `main` at `1b66af2` —
`rtl/writeback.v` takes `executor_out` as a second source and drives `waddr`/`wdata` from it on a
cycle the retiring instruction writes no register, `rtl/decoder.v` drops the executor slot from
`live_rsN` for exactly that cycle on a new `ex_early` input, `rtl/littlecpu.v` joins the two. The one
departure from the original is that no `rd_from_bus` bit was added: `executor_output.rd_ready`, which
ADR-0154 landed for the forwarding path, already says the same thing — the value behind `rd_data` is
the finished one — so the early write is gated on it. Candidate tree `567c0f0`. Sixteen paired seeds
(`default 1`…`15`), `SOC_MIN_MHZ=0`, one toolchain on both arms: OSS CAD Suite — Yosys 0.68+48
(`ff5817c34-dirty`), nextpnr-0.11-1-g62e659ed, icetime oss-cad-suite 20260811
(`sha256:25a4ecb76c094f00`).

| arm | worst | median | best | spread | placed `ICESTORM_LC` | under 12.00 MHz |
|---|---|---|---|---|---|---|
| base `4697eb8` | 80.41 ns / 12.44 MHz | 78.03 / 12.82 | 76.38 / 13.09 | 5.3% | 4904 | 0 of 16 |
| early write `567c0f0` | 85.82 ns / **11.65 MHz** | 82.92 / **12.06** | 80.93 / 12.36 | 6.0% | 4941 | **7 of 16** |

**+6.28% of median period, +6.73% at the worst placement, sixteen of sixteen seeds slower and none
faster.** The +9.4% this ADR recorded has narrowed to +6.3% and has not gone away: it is still most
of twice the ~3.6% churn band, the sign consistency is perfect, and **seven of sixteen placements are
under the board clock** against the original's four of six. The placed SoC is +37 cells and `make
fit` 4101 → 4135 (+34), both inside ±50, so this remains a period result and not an area one.

**It still works, and it still buys cycles — less than it did, because ADR-0154 got there first.**

| | base `4697eb8` | early write `567c0f0` |
|---|---|---|
| suite | 38 746 cycles, CPI 1.77 | 36 986, CPI 1.68 (**−4.54%**) |
| Dhrystone, 2000 runs | 1 506 772 cycles, CPI 1.59 | 1 430 686, CPI 1.51 (**−5.05%**) |
| DMIPS/MHz | 0.777 | **0.820 (+5.5%)** |
| Dhrystone hzC (a ready result forwarding does not reach) | 143 356 | 62 164 |

The original bought −6.5% of Dhrystone's cycles on a tree with no forwarding at all; it buys −5.05%
now, and the `hzC` column says where the difference went — executor-only forwarding already collects
most of what the early write would have. `make test` is green on the candidate through the `.S`/`.c`
suite and every unit bench.

**The product is now a wash rather than a loss, and the requirement still decides it.** At the median
placement of each distribution, 0.777 DMIPS/MHz × 12.82 MHz = 9.96 DMIPS against 0.820 × 12.06 =
9.89, which is −0.7%; at the worst, 9.67 against 9.55, −1.2%. When this ADR was filed the same
arithmetic read −47%, because the clock fell past a divider step. **That is a real change and it does
not change the answer**: `SOC_MIN_MHZ` is 12.0 and does not slide, and a candidate whose median
placement reads 12.06 MHz with seven of sixteen underneath has no margin to ship into. **HELD.** The
RTL is not carried on `main`; only this amendment is.
