# 0171 — The comparison places on up5k and ECP5 only, and the up5k clock is a gate

Status: Accepted · 2026-09-09 · completes
[ADR-0160](0160-the-comparison-moves-to-the-parts-this-design-ships-to.md), which moved
the comparison's default to up5k but left hx8k selectable and left the step function as prose

## What was still wrong

ADR-0160 established both facts this ADR mechanises and mechanised neither.

**hx8k was still in the Makefile's part table**, still had a `.pcf`, and `soc/compare/sweep.sh`
still ended by printing `soc/bands.py hx8k --note` — so the sweep every go/no-go runs read its
band paragraph off a part the default no longer placed on. `soc/depth/sweep.sh` kept an hx8k arm
against the same `.pcf`, and `soc/compare/dhry_fit.py` and `soc/compare/coremark_fit.py` still
defaulted `--part hx8k --part-blocks 32` — so `make compare-dhrystone` printed *"the image needs
4 + 22 = 26 SB_RAM40_4K, and hx8k has 32 in total"* on a run that had placed on up5k. Neither
caller passed either flag, so the wrong part was the only part that line ever named.

**The step function was in the prose and in no grader.** ADR-0160's own table shows every core
reaching 12 MHz and none reaching 24; nothing in the flow said so. `make compare-timing` printed a
frequency to two decimal places and stopped there, `soc/compare/sweep.sh` sorted those frequencies,
and `soc/compare/run_product.sh` multiplied them into a product. A core placing at 22.66 MHz on a
part whose next clock above 12 is 24 was scoring 1.78× a core at 12.70, and every one of those
1.78 is unspendable.

**The ECP5 arm existed and was not selectable.** `make compare-ecp5-timing` worked, but
`COMPARE_PART` knew only `up5k` and `hx8k`, so the sweep, the product run and every caller that
went through `make compare-timing` could not reach it. It also had none of the gates the SoC's own
ECP5 flow has: no `DP16KD`/`TRELLIS_DPR16X4`/`MULT18X18D` census, no `soc/bram_reset_check.py`, and
no `placed_vs_synth.py` — the check ADR-0086 exists because of.

## The measurement that expired

`docs/pin-constraints.md` justified hx8k as *"the only ice40 with enough logic to hold this core at
all"*. Re-taken on this tree, at the default seed, ROM 1024 words and RAM 16384:

| core | placed `ICESTORM_LC` | of 5280 | clears 12 MHz |
|---|---|---|---|
| littlecpu | 4466 | 84% | yes |
| vexriscv | 3481 | 65% | yes |
| hazard3 | 3275 | 62% | yes |

All three fit, this one the largest of the three, and all three clear the step. The sentence was
true when it was written and is false now. **hx8k is also the part that cannot hold `littlesoc`**
(ADR-0160: 148 block RAMs against 32, no SPRAM, no `SB_MAC16`), so the comparison was placing on
the one part in the set that could not run the design being compared.

## What ships

**`COMPARE_PART` selects `up5k` or `ecp5`, and anything else is a hard error.** There is no third
row to add without measuring one, and `soc/compare/bench_hx8k.pcf`, `soc/bands.py`'s `hx8k` entry
and `soc/depth/sweep.sh`'s hx8k arm are gone with it. Outside `docs/`, the name survives in exactly
three places and each of them is a refusal: two probes requiring `soc/bands.py hx8k` and
`placed_vs_synth.py --part hx8k` to be REFUSED, and `soc/compare/sweep.sh`'s own message saying the
part was removed rather than renamed. A part this repo stopped placing is refused the same way one
it never placed is, rather than kept as a row that answers.

**up5k: a pass/fail gate, then cycles.** `soc/compare/step_gate.py` reads the same `icetime -r`
report `soc/timing_split.py` already walks (imported, not re-parsed) and grades it against
every clock the oscillator offers. A core over 12 MHz passes and its margin is printed as *unspendable*; a
core under it exits non-zero saying the next clock down is 6 MHz, so it is out of the comparison
rather than slower in it. A `--step` that is not one of 48/24/12/6 is refused, because a
requirement set between the steps grades against a frequency no board can supply. Six probes,
including both refusals and the unspendable line.

**ECP5: the real product, with the SoC flow's own gates.** `make compare-timing COMPARE_PART=ecp5`
reaches the ECP5 path; `compare_ecp5.<core>.json` now gates three mapping censuses and
`soc/bram_reset_check.py`, and `compare-ecp5-timing` runs `placed_vs_synth.py` against a new
`compare_ecp5.<core>.core.log`. `soc/compare/placed_vs_synth.py` grew `--part`, required and with
no default: `ICESTORM_LC` against `SB_LUT4` on up5k, `TRELLIS_COMB` against `LUT4` on ECP5. Asking
with the wrong part's names finds no count at all, which is why the refusal names the part rather
than blaming the log. The frequency PUBLISHES and carries no ratchet, and `soc/compare/sweep.sh`
prints `soc/bands.py ecp5 --note` under it to say so.

**Hazard3's multiplier is soft logic on ECP5 and the other two cores' are not**: 0 `MULT18X18D`
against 4 each. That is declared as `COMPARE_ECP5_EXPECT_DSP_hazard3` rather than rediscovered,
and it is exactly what a census exists to surface — it is silent in a frequency and enormous in
area.

**The fit scripts model the part they actually place on.** `--part up5k --part-blocks 30
--part-spram 4`, with the data RAM routed to SPRAM where the part has it and falling back to block
RAM where it does not. That changes an answer rather than a label: **Dhrystone's image FITS the
placed geometry on up5k** — 1,332 bytes of text against a 4,096-byte ROM budget and 10,592 bytes of
RAM against 65,536 — so its cycles are no longer simulated at a larger map than the clock is placed
at. CoreMark's 10,768 bytes of text still are. The two benchmarks stopped sharing that caveat and
CLAUDE.md said they shared it.

## What did not move

The cycle half is untouched by this pass and re-measured to confirm it: Dhrystone 294,025 cycles
for littlecpu alone at its native ISA, and CoreMark 433,240 / 427,008 / 665,416 for littlecpu,
VexRiscv and Hazard3 — identical to the figures ADR-0146's third amendment recorded. Nothing here
is an RTL change.

`soc/compare/product.json` is stale on its own check against this tree and was already stale before
this pass, for reasons this ADR did not create (a different ISA and a different RAM geometry in the
stamp). Re-stamping it is still a separate ticket's.

**`soc/compare/run_product.sh` is not made part-aware here, and its up5k clock column is now known
to be the wrong shape.** It reads `critical path :` out of `make compare-timing`, which only the
up5k arm prints, so `COMPARE_PART=ecp5` makes it stop with its own "exited 0 with no critical path
line" refusal rather than misparse — loud, but not the ECP5 product. And on up5k the column it does
read is a critical path in nanoseconds, which the step function says is not a clock any program
gets: the product on that part is the cycle ratio at one shared 12 MHz. Reshaping the stamp to
carry a part, a step verdict on one arm and a frequency distribution on the other is the next
ticket, not this one.

## What this does not claim

* The ECP5 arm publishes a frequency and grades nothing about it. **No churn band has been derived
  for that part**, `soc/bands.py` refuses to answer for it, and up5k's own band was derived on
  `littlesoc` rather than on this bench — so a delta on either part's bench is not yet readable as
  a change or a null.
* One placement is a sample. The tables in the pull request that carries this ADR are twelve seeds
  a core a part, paired by seed.
* Nothing here was packed, programmed or run on either board.
