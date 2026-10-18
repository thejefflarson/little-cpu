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

## Amendment: NIST's tool on a million consecutive samples (2026-10-11)

Measured on the same board (iCESugar-Pro, LFE5U-25F-6BG256C, openFPGALoader over the CMSIS-DAP,
SRAM only) on 2026-10-11, tree at `main` c454c8e5 plus this change. **NIST's reference tool puts
the shipped setting at 0.4994 bit of min-entropy per sample, under the 0.5 `rtl/trng.v` assumes,
and the 25 MHz-pad worst case at 0.4134.** The repo's own estimators above (median 0.71) were
optimistic for this source. No health test was changed.

**The tool.** `usnistgov/SP800-90B_EntropyAssessment` at commit
`68ed165fd7a3eeef26b87a546ba23f338e82a3f3` (tag v1.1.8; its binaries print `noniid 1.1.7`),
tarball SHA-256 `067946993b73de947c435c48d811f5f9b2ce9084d8408512f263515816e19582`, pinned in
`mk/nist90b.mk` with the command-line override refused, installed by `make nist90b-setup` into
`~/.cache/little-cpu/sp800-90b` and built there (`ea_non_iid`, `ea_restart`) with clang++ and
Homebrew's libomp, jsoncpp, libdivsufsort and openssl@3 on macOS arm64.

**Why a new capture.** The interval capture above cannot give consecutive samples. It fills 4,096
intervals in 0.43 s, then sends them in 0.71 s at 115200 baud while about 6,800 edges pass
unrecorded, and 16-bit intervals at 9.5 kHz need 19 kB/s on a line that carries 11.5 kB/s.
`soc/entropy_stream_icesugar_pro.v` and `soc/entropy_bit_framer.v` stream only the bit `rtl/trng.v`
reads, `ticks[1] ^ ticks[0]` of every edge's interval, from the same `OSCG`, fabric divide and
3-flop synchroniser. A frame is 64 sample bits behind a 16-bit sequence number and a 16-bit sum:
14 bytes, 1.2 ms of line time per 6.7 ms of edges, so a frame is always sent before the next
completes, and if one were not the sequence number would still advance. A run is consecutive
exactly when the sequence numbers advance by one; `soc/entropy_stream.py` discards a run at any
gap. `test/entropy_stream_test.py` simulates the framer against the intervals and forces each loss
red: edges faster than the UART, a flipped byte, a skipped number, a wrong sample bit.

**ea_non_iid, 1,000,000 consecutive samples, one bit each** (so H_bitstring is not produced: the
sample is the bitstring), the final run of each setting:

| estimator | OSCG / 2^8, 30 MHz PLL core (shipped) | same, core on the 25 MHz pad |
|---|---|---|
| most common value | 0.973558 | 0.993922 |
| collision | **0.499409** | 0.445219 |
| Markov | 0.762291 | 0.723553 |
| compression | 0.507934 | **0.413438** |
| t-tuple | 0.601708 | 0.511482 |
| longest repeated substring | 0.918066 | 0.819853 |
| MultiMCW prediction | 0.720606 | 0.661168 |
| lag prediction | 0.742791 | 0.679895 |
| MultiMMC prediction | 0.742791 | 0.677693 |
| LZ78Y prediction | 0.742790 | 0.679895 |
| **H_original** | **0.499409** | **0.413438** |

The shipped setting sits on the line: two earlier 1M captures of the same design read 0.500299 and
0.503930, so the figure is 0.5 within a spread of about 0.004, and the collision estimate sets it
each time. The 25 MHz-pad setting is under 0.5 by NIST's estimate by far more than that spread, the
direction ADR-0252's own estimators gave (0.49 worst run).

**Restart test.** 1,000 restarts of 1,000 samples, each restart a fresh JTAG load of the capture
bitstream (`openFPGALoader -m`, SRAM) with the first 1,000 samples after the load kept; a load takes
4.4 s because `ecppack --compress` shrinks the stream bitstream to 105 KB. Shipped setting:
`ea_restart` passed its sanity check (largest count of one value in a row or column 632, cutoff
770) and its validation test, with H_r 0.502356, H_c 0.832244 and H_I 0.499409, so
min(H_r, H_c, H_I) is 0.499409. The restart rows are no worse than the continuous run: the column
estimates (0.83 to 0.999) show no first-sample pattern across restarts. The pad setting's matrix
stopped at 275 of 1,000 restarts, when the board left the USB bus (the debugger enumerated no
device for the following hours and nothing was unplugged on purpose), so `ea_restart` has not been run
for it. Its H_original is under 0.5 on its own, and `ENTROPY_NIST_RESUME=1 make entropy-nist-pad`
continues from row 276 once the board is back.

**What a JTAG reload resets.** It erases and rewrites the FPGA's configuration SRAM: every fabric
flip-flop returns to its initial value, the PLL is reconfigured and re-locks, and the capture
starts again at sequence 0 with its first edge discarded. It does not reset what a power cycle
would: the die and board temperature, the supply rails, the 25 MHz crystal, and the debugger.
Whether the erase also stops `OSCG`'s ring, or only its configuration bit, was not observed, so
the restart rows test the start-up of the design and not a cold start of the oscillator.

**What the board needs.** The claim of 0.5 bit per sample is not supported with margin by NIST's
tool. Not done here, and not a health-test change: raise the margin in the source (a faster core
clock quantises the same jitter more finely, as the pad run's slower clock lost 0.09 bit; the
placed core clock closes at 37.15 MHz worst, ADR-0252, so the 30 MHz PLL is not the ceiling) or
claim less than 0.5, which raises the proportion-test cutoff and is the owner's call. Either needs
a new `make entropy-nist` run, which `test/nist90b_grade_test.py` grades against the 0.5 floor.

**Reproducing.** `make entropy-nist-pll` and `make entropy-nist-pad` build the capture, load it,
capture, run both tools and grade them (`soc/entropy_nist.py`, `soc/nist90b_grade.py`); they exit
nonzero when the report is under 0.5, as the shipped setting's is. About 85 minutes each, off
`make test`. `ENTROPY_NIST_LOCK` names a directory held around each board access and
`ENTROPY_NIST_RESUME=1` continues a partly filled matrix.

**What keeps a run honest.** A from-boot run counts nothing until the serial port has been silent for
100 ms after the loader starts (the SRAM erase stops the old design's stream), then must begin at
sequence 0 and ends for good at the first gap, because a later sequence 0 is the 16-bit counter
wrapping after 441 s and not a board starting. The load and the capture happen under one hold of the
board lock. Captured data and the restart matrix are bound by a sidecar (`<tag>_meta.json`, and
`<matrix>.meta.json`) to the bitstream digest, core clock, sample count and restart count; `--resume`
refuses data without one, with a different one, or of the wrong size, and a capture is written to
`.part` and moved into place whole. The grader requires one pass line from `ea_restart` and none of its
failure markers, recomputes min(H_r, H_c, H_I) itself, and requires the H_I it was given to be
`ea_non_iid`'s H_original. A framer that numbers a boot (so a PLL relock mid-run is visible) and a
CRC-16 in place of the additive sum change the bitstream and wait for the next board capture.

## Amendment: a 2^9 divide and an 8,192-cycle edge timeout (2026-10-17)

The 2^8 source read 0.4994 bit per sample, with no margin over the 0.5 the proportion cutoff
assumes. A shorter period had measured worse (2^7 read 0.28), so the longer one is tried: the
fabric divide in `soc/icesugar_pro_entropy.v` becomes 2^9, 6,342 core cycles per edge at 30 MHz,
and `rtl/trng.v`'s edge timeout doubles to 8,192 cycles so the period sits at 77% of it.
`test/entropy_geometry_test.py` keeps its rule (3,000 cycles to 80% of the timeout, here 6,554) and
now reads the timeout as two passes of `ticks`, the second marked by a `late` flag.

**The rule's cost on the up5k.** The shared RTL carries the longer timeout on both boards. Three
spellings of a 13-bit counter placed at 5,292 to 5,310 `ICESTORM_LC` and did not fit the part's
5,280; a 12-bit counter with the `late` flag placed at 5,265 (+56 against 5,209 on main). Sixteen
serial placements of that tree, paired by seed against main on the same toolchain (yosys 0.68+48,
nextpnr 0.11-1): worst 12.33 MHz (main 12.27), median 13.01 (12.91), best 13.54 (13.30), spread
9.8% (8.4%); `make fit` reads 4,549 against 4,503 and `FIT_MAX_LC` 4,586; `make ecp5-timing` reads
39.20 MHz. `soc/pin.json` is stale against this tree.

**NIST at 2^9, 2026-10-17** (`make entropy-nist-pll` with the capture built at 2^9, the core on
the 30 MHz PLL, SRAM only, 1,000,000 consecutive samples and a 1000 x 1000 restart matrix). `ea_non_iid`
reads H_original 0.810058: most common value 0.995023, collision 0.818688, Markov 0.979534,
compression 0.810058, t-tuple 0.919089, longest repeated substring 0.967367, MultiMCW 0.990734,
lag 0.977158, MultiMMC 0.977680, LZ78Y 0.977005. `ea_restart` passed, with H_r 0.803011, H_c
0.824950 and H_I 0.810058, so min(H_r, H_c, H_I) is 0.803011, against 0.4994 at 2^8 and a 0.5
floor. The compression estimator is the lowest of the ten. The seed probe on the
shipping design printed 273 words with no DEAD, and a word every 9,935,539 core cycles (0.33 s at
30 MHz) after the first.

## Amendment: the first ES16 word, read from the probe itself (2026-10-18)

`soc/seed_probe.S` now latches `mcycle` at the first ES16 read and prints it as the second field of
every word line (`W <word> <mcycle>`), so a read opened after the run starts still recovers it; the
read used to open too late to see the first status changes. On the board at 2^9, the 30 MHz PLL core
clock and the shipping 8,192-cycle timeout, the first ES16 arrived at 9,103,936 cycles
(0x008aea40), 0.303 s at 30 MHz, the same value on every word line of the eight-second read. The
estimate of about 16 million cycles (1,024 BIST samples plus one word) was high by 1.8x; 1,024
samples at 6,342 cycles is 6.5 million, which leaves 2.6 million for the first word. This is one
load on one board. `make entropy-nist-pll` and `-pad` now pass `--lock` with `ENTROPY_NIST_LOCK`
defaulting to `~/.cache/little-cpu/icesugar.lock`, so the 85-minute capture holds the board lock;
`test/seed_probe_test.py` grades both.
