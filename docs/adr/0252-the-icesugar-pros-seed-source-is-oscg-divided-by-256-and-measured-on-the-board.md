# ADR-0252: The iCESugar-Pro's seed source is OSCG divided by 256, and it is measured on the board

**Status:** Accepted · 2026-10-11. Measured on `main` at fef77be7 plus this tree, one iCESugar-Pro
(LFE5U-25F-6BG256C), openFPGALoader over the board's CMSIS-DAP, SRAM only, 2026-10-11.
Follows ADR-0245 (the `seed` CSR and `rtl/trng.v`) and ADR-0172 (the 30 MHz PLL).

## Context

`soc/board_icesugar_pro.v` held `littlesoc`'s `entropy_raw` low, so `rtl/trng.v` saw no edges and
`seed` read DEAD after 4,096 cycles. The up5k has `SB_LFOSC`; the ECP5 has `OSCG`, a free-running
on-chip oscillator at a nominal 310 MHz divided by `DIV` (2 to 128). It is a different circuit, so
nothing measured on the up5k transfers, and ADR-0245 left the claim of 0.5 bit of min-entropy per
sample, the sample bits `ticks[1] ^ ticks[0]` and the fold depth unmeasured on any board.

## What was built

`soc/icesugar_pro_entropy.v` instantiates `OSCG` at `DIV = 128` and divides it by 2^8 in fabric on
its own clock; the top counter bit is `entropy_raw`. `soc/board_icesugar_pro.v` wires it in and
`littlesoc` is unchanged. `make ecp5-timing` builds `littlesoc` without any board top, so its censuses cannot move; the
board bitstream's own `soc/bram_reset_check.py` still reads 36 block RAMs, none reset by logic (the
divider is eight flip-flops), and the 30 MHz PLL still closes (twelve seeds below).

**The ratio.** OSCG at `DIV = 128` measured 2.435 MHz (a 311.7 MHz ring), so the core sees an edge
every 3,154 cycles at 30 MHz (105.1 us, 9.5 kHz). `SB_LFOSC`'s ~10 kHz gives the same edge rate in
time and about 1,200 cycles at the up5k's 12 MHz; the cycle count here is 2.6 times larger because
the core clock is 2.5 times faster, and that is the right comparison, because what randomises the
sampled counter bits is the oscillator's accumulated jitter, a time (about 50 ns rms at 105 us),
read against the core clock's 33 ns quantum. A longer interval holds more cycles of jitter. The
ceiling is `rtl/trng.v`'s 4,096-cycle edge timeout: 3,154 sits at 77% of it, so an oscillator 23%
slower than this board's still reads ES16. `test/entropy_geometry_test.py` holds the nominal period
(`CORE_HZ * DIV * 2^DIV_LOG2 / 310 MHz`, 3,171) between 3,000 cycles and 80% of the timeout, with
forced-red probes for a divide one step short, one step long and the source tied low.

## The capture

`soc/entropy_capture_icesugar_pro.v` is a core-less top: PLL, the same `icesugar_pro_entropy`, the
trng's 3-flop synchroniser, a 16-bit counter of clock cycles between rising edges, a 4,096-entry
block RAM and a UART that streams `0xA5 0x5A` and the interval run, then captures a fresh run.
`soc/entropy_capture.sh` builds one setting; `soc/entropy_assess.py read` collects frames and
`assess` runs the estimators. The same method serves `SB_LFOSC` on the UPduino, whose capture is
owed: only the top and the clock differ.

Four settings, intervals in core cycles, 30 MHz PLL core unless marked:

| setting | cycles per edge | frames | interval sd | min-entropy per sample, min / median / max | APT trips of 512-sample windows |
|---|---|---|---|---|---|
| OSCG, no fabric divide | 12.3 (12 or 13) | 3 | 0.47 | 0 (sample bit constant) | 24 of 24 |
| OSCG / 2^6 | 789 | 4 | 0.54 | 0.060 / 0.068 / 0.096 | 32 of 32 |
| OSCG / 2^7 | 1,577 | 4 | 0.86 | 0.276 / 0.415 / 0.592 | 10 of 32 |
| **OSCG / 2^8 (shipped)** | **3,154** | **24** | **1.58** | **0.634 / 0.714 / 0.780** | **0 of 192** |
| OSCG / 2^8, core on the 25 MHz pad | 2,628 | 24 | 1.28 | 0.491 / 0.577 / 0.680 | 0 of 192 |

Min-entropy is the minimum of four SP 800-90B 6.3 estimators (most common value, collision, Markov,
compression) over the one-bit sample `rtl/trng.v` takes, per 4,096-sample run. The estimators are
this repo's own implementations, graded against sources of known answer by
`test/entropy_assess_test.py`, not the NIST reference tool.

**The 0.5 bit assumption fails below 2^8 and holds at it.** At 2^7 the interval's sd of 0.86 cycles
is too small for the low two bits to spread: two of four runs estimate under 0.5 and ten of 32
proportion windows would trip the 410-of-512 cutoff, so a board built at 2^7 would reach DEAD, not
ES16. At 2^8 the cutoff holds with no window tripping in 192, and no health test was changed. The
margin is thin on the low side: the 25 MHz-pad run, with a 17% shorter period, drops to 0.491 in its
worst run, which is why the floor in the geometry test is 3,000 and not lower. The jitter in time is
the same in both clocks (50.7 ns at 30 MHz, 51.2 ns at 25 MHz), so it is OSCG's own.

**Periodicity.** Undivided, OSCG beat-locks to the core clock: intervals alternate 12 and 13 with a
self-match of 1.0 on the sample bit, the lock ADR-0245 warns of. Divided by 2^8 the intervals
(sd 1.58, from 3,147 to 3,160 cycles) still wander slowly, with a lag-1 autocorrelation of 0.55 to 0.7 and
a sample-bit self-match of 0.58 at shift 1, ten standard errors from one half. That is slow drift
of the ring, not a beat: averaged over the 24 frames the self-match decays smoothly, 0.584, 0.561,
0.540, 0.529, 0.525, 0.516, 0.515, 0.515 at shifts 1 to 8, with no peak at those shifts. The
corrector's output does not carry it: the 2,545 folded bits pooled over 24 frames have 0.514 ones
(one sigma 0.010) and a self-match of 0.49 to 0.51 at every shift from 1 to 8. The pooled folded
bits are too few for min-entropy beyond a first look (MCV 0.89).

## On the board

`soc/seed_probe.S` (`make icesugar-prog ICESUGAR_PROG=soc/seed_probe.S`) prints each status change
with `mcycle` and every word read from ES16. Run on the board built from this tree: the first ES16
arrived at 5.1 million cycles (171 ms), each later 16-bit word 2.9 million cycles apart (97 ms), and
`DEAD` never appeared in 60 s. The 919 words returned in that window hold 0.495 ones in 14,704 bits
(one sigma 0.004), a bit self-match of 0.496 to 0.509 at every shift from 1 to 16, and 6 repeated
words against 6.4 expected for uniform 16-bit words. `test/asm/seed.S` and `test/asm/seedaccess.S`
pass on the board. `make icesugar-suite-board` (`soc/run_suite_icesugar.sh`) ran
80 of 80 on the board with the new top, `seed.S` and `seedaccess.S` among them. Twelve nextpnr seeds
of the shipping top (`icesugar_pro_top` at the default 8 KB ROM, seeds 1 to 12) place the core clock
at 37.15 MHz worst, 38.8 median and 40.34 best against its 30 MHz constraint, none failing.

## What this does not settle

- One board, one temperature, one afternoon. OSCG is uncalibrated: the part-to-part spread of its
  frequency is not known, and the geometry test holds the nominal period 5% above the floor where
  this board's own clock is 0.5% off nominal. A part whose ring runs 6% fast lands under 3,000
  cycles and estimates under 0.5; a slower one is safe down to 23%.
- The estimators are this repo's, not NIST's reference tool, and a 4,096-sample run is shorter than
  the 1,000,000 samples 90B asks for.
- The folded output was only looked at on 2,545 bits. A conditioned-output test (90B 4.3, or
  dieharder) over megabits is not done.
- The SB_LFOSC capture on the UPduino is not taken; nothing here transfers to it.
