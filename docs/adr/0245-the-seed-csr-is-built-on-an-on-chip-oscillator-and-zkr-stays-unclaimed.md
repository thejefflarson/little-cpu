# 0245 — The `seed` CSR is built on an on-chip oscillator, and Zkr stays unclaimed

Status: Accepted · 2026-10-03. Measured on `main` at 4a83e482 and the tree that adds `rtl/trng.v`.
Follows ADR-0240 (the legality term's spelling) and `docs/ideas/entropy-behind-seed.md`.

## What was built

`seed` (0x015) is implemented. A read returns OPST in bits 31:30 (BIST 00, WAIT 01, ES16 10,
DEAD 11), zero in 29:16, and 16 bits of entropy in 15:0 only under ES16. A read consumes the word.

**Legality is ADR-0240's spelling (b).** `rtl/executor.v` ORs `in_is_csr_access && csr_addr ==
12'h015 && !csr_write_op` into `instr_illegal`, so `csrrs`/`csrrc` with a zero source field,
register or immediate form, raises cause 2. `csrrw` always writes, so it is legal even with
`rs1 = x0`; with `rd = x0` it does not read and consumes nothing. Writes are ignored.
`rtl/csrs.v` adds `SEED` to its read case and so to the implemented set. `mstatus` has no
interaction. `mseccfg` exists to let S and U mode reach `seed`, and this core has neither mode;
M-mode access is unconditional.

**The source is `rtl/trng.v`, inside `rtl/csrs.v`.** `littlecpu` gains one input, `entropy_raw`,
and the module does the rest: a three-flop synchroniser, a 12-bit counter of `clk` cycles since the
last rising edge, the low two counter bits XORed into one raw bit, a von Neumann corrector, an
8-to-1 XOR fold and a 17-bit shift register whose top bit is both the sentinel and the "full" flag.
Two health tests report DEAD, sticky: 32 identical raw bits in a row (a repetition-count test), and
4,096 cycles with no edge (a stuck or stopped source; the nominal period is about 1,200 cycles).
BIST lasts until the first word is buffered. DEAD carries no entropy and no state ever reads
ES16 after it. `seed` never stalls, so no stall reason, F or G moved (`make -C formal
remeasure-fg`: F = 5, G = 4) and `test/zkt_isolation_test.py` stays green.

**The oscillator is `SB_LFOSC`**, instantiated in `soc/board_upduino.v` (a hard macro, no logic
cell). `soc/board_icesugar_pro.v`, `rtl/littledual.v`, `soc/compare/bench_littlecpu.v` and every
formal harness hold `entropy_raw` low, which reads DEAD after the timeout. `test/testbench.v`
drives a seeded LFSR-jittered square wave, half periods one to four cycles, so every run
reproduces and ES16 arrives in about 2,500 cycles.

## Departures from the brief

- **The device sits in the core's CSR file, not beside the UART.** The brief put the conditioner in
  the SoC behind a read port and a strobe. That adds a strobe output and a data bus to `littlecpu`,
  tied off in five formal harnesses; this adds one input. The input rides
  `formal/MULTIHART_TIE_OFF` as a `PORT` line, which is a stretch of that file's name and is
  documented there.
- **No fabric ring oscillator.** The brief's table (yosys deletes an RTL ring; a primitive ring is
  refused by nextpnr's loop check) stands and was not re-run.
- **No raw tap at 0x0002_0030** and so no firmware adaptive-proportion test. The tap is the
  validation path the board measurement needs; it is deferred with that measurement.
- **The BIST window is the first buffered word**, not a counted window, which saves a counter.

## Why Zkr is not in the ISA string

The brief's rule is that the claim follows the measurement, and nothing here has been run on a
board: the interval bits that feed the corrector, the fold depth and the repetition cutoff are
guesses until the raw intervals of a real `SB_LFOSC` are captured. A claim also owes `mseccfg`
(0x747, which the privileged spec says Zkr adds) and a firmware adaptive-proportion test, neither
built. `test/march_test.sh`'s seven sites are unchanged.

## Measurements

`soc/paired_sweep.sh origin/main up5k`, sixteen placements (`default`, seeds 1..15), one stamp
(Yosys 0.68+48, nextpnr-ice40 0.11-1-g62e659ed, icetime oss-cad-suite 20260811, Darwin arm64),
2026-10-03, run on the committed tree.

| | main (4a83e482) | this tree |
|---|---|---|
| packed `ICESTORM_LC`, SoC | 5,084 | 5,205 (+121; 98.6% of 5,280) |
| `make fit`, local | 4,347 | 4,492 (+145) |
| worst placement | 83.09 ns, 12.04 MHz | 78.60 ns, 12.72 MHz |
| median | 76.59 ns, 13.06 MHz | 76.15 ns, 13.13 MHz |
| best | 74.60 ns, 13.40 MHz | 74.16 ns, 13.48 MHz |
| under 12.00 MHz | 0 of 16 | 0 of 16 |

Per-seed MHz, `default, 1..15`:

- main: 12.68 13.01 12.85 13.04 13.11 12.61 13.09 13.07 13.40 13.09 12.04 13.29 13.09 12.72 13.04 13.25
- this tree: 13.43 13.16 13.06 13.01 13.48 12.72 13.33 12.80 12.97 13.32 13.08 13.10 13.06 13.19 13.26 13.38

**The result is a draw of the mapper, not a property of the design.** An earlier tree that differed
only in where one `case` arm stood in `rtl/csrs.v` packed to 5,256 cells and placed three of
sixteen seeds under 12.00 MHz (11.64 worst); the arm order in this tree packs to 5,205. At 98.6%
occupancy a respelling moves the count by 51 cells, so the next edit to any file the synthesis
reads owes this sweep again. `make fit`'s `FIT_MAX_LC` moves 4,441 to 4,586: 4,492 local, plus the
40-cell churn band and 54-cell toolchain gap of ADR-0220. The CI job's own count is not yet
measured on this tree.

## Graders

`test/trng_tb.v` drives five sources (healthy, stuck low, stuck high, constant period, one that
stops) and requires DEAD from the last four, ES16 only from the first, DEAD sticky and empty, with
three forced-red comparisons. `test/csr_tb.v` reads BIST and ignores writes. `test/executor_tb.v`
vectors every access form. `test/asm/seedaccess.S` agrees with Sail once
`test/sail/rv32imac_zicsr.json` claims Zkr and `seed`'s value joins `NONCOMPARABLE_CSRS`;
`test/asm/seed.S` is a baselined `DISAGREE AT 3`, since the model's source is ready at once
(`docs/manifests/cosim-expected-fail.md`). `components_traps` models the read-only case from the
instruction word alone; `rtl/decoder.v` asserts the two flags that model reads, and
`traps-region-probe.py` has a third arm that must fail at the must-trap assertion. The mutation
`seed-read-only-legal` is caught by `executor_tb` and `seedaccess.S`.

## What this does not settle

- Whether the oscillator's interval LSBs carry entropy. No board has run it.
- ECP5 and the dual top read DEAD; neither has an oscillator wired.
- The sweep is one machine's draw of sixteen.

## Amendment: Zkr is claimed (2026-10-04)

The title records the decision as first made. Zkr is now in the `-march` string, `_zkr` at the
seven sites `test/march_test.sh` grades, and the sections above that say otherwise ("Why Zkr is
not in the ISA string", the single repetition test) describe the tree before this amendment.

**Four health tests set the sticky DEAD.** Each is graded by a forced-red probe in
`test/probe_gates.sh` against `test/trng_tb.v`, and the old raw 32-identical test is deleted.

| Test | Looks at | Dead when |
|---|---|---|
| Edge timeout | cycles since a rising edge | 4,096 with none (unchanged) |
| Starvation | raw samples per 64-sample aligned block | a block ends with no corrected bit |
| Repetition count | the folded output bits | 32 identical bits in a row, counting from the first |
| Adaptive proportion | raw samples, 512-sample windows | one value fills 410 of a window |

BIST lasts 1,024 raw samples, and no word reaches the buffer until the sample counter wraps once,
so a source must pass all four tests for that long before the first word. `test/run_tests.sh` and
`test/cosim.py` move from a 5,000-cycle to an 8,000-cycle budget because `seed.S` now waits about
5,650 retired instructions for ES16 on the bench oscillator.

**Two design calls.**

- *The repetition count reads the folded bits, not the raw samples.* A raw-sample run test passes
  a source that alternates 0,1 or repeats a short period, because the corrector and the fold turn
  a periodic input into a periodic output. Counting repeats of the emitted bit also catches a
  period-2 emitted pattern, which is the failure a user of `seed` would see.
  The count includes the run's first bit and reset counts none, so the alarm lands on exactly the
  32nd identical bit; `test/trng_tb.v` feeds 31, 32 and 33 folded ones and 31 and 32 folded zeros
  from reset to pin that.
- *The starvation check exists because a beat pattern would otherwise sit in WAIT forever.* A
  source whose pairs never straddle a transition (ten 0s, ten 1s, repeating) emits no corrected
  bit, never trips the edge timeout (edges keep arriving) and never trips the proportion test
  (it is balanced). Aligning the check to 64-sample blocks costs one flop and reuses the sample
  counter, where a free-running six-bit counter cost more cells; a healthy source misses a block
  with probability 2^-32.

**The cutoff is an assumption.** The proportion cutoff of 410 in 512 is NIST SP 800-90B 4.4.2's
value for a claimed 0.5 bit of min-entropy per sample at a 2^-20 false-positive rate. That 0.5 bit
is **unmeasured**: no board has captured the raw intervals of an `SB_LFOSC`, so neither the claim
nor the sample bits (`ticks[1] ^ ticks[0]`) nor the fold depth is settled until one does. The ISA
string makes the Zkr claim ahead of that measurement, because the ticket asked for it, and this
paragraph is where the claim is bounded.

**The continuous tests do not rule out a periodic lock.** They detect gross failure, not the
absence of entropy: an oscillator beat-locked to the core clock with intervals repeating every 7
(a, a, a+1, a, a+1, a, a+1) emits corrected bits of period 6 and folds to 1,0,1,1,0,1, whose runs
never pass 2 and whose proportions are balanced, so it reaches ES16. Only the board capture of
`SB_LFOSC`'s raw intervals settles whether such a lock occurs; a lag-match test over lags 1 to 8
would catch it and costs far more than the part's free cells.

**`mseccfg` (0x747) and `mseccfgh` (0x757) read zero and ignore writes.** The privileged
specification says "Implementations may implement mseccfg such that sseed and useed is a read-only
constant value 0", and this core has neither S nor U mode to grant `seed` to. Both addresses are
implemented because the spec lists `mseccfgh` as the RV32 upper half; the neighbouring addresses
stay unimplemented, which `test/asm/mseccfg.S` and `test/csr_tb.v` grade, and Sail agrees on
`mseccfg.S`. `nano/asm/LITTLECPU_FLOOR` excludes `mseccfg.S` as a CSR program.

**Measurements** (`soc/paired_sweep.sh origin/main up5k`, sixteen placements `default, 1..15`, same
stamp as above: Yosys 0.68+48, nextpnr-ice40 0.11-1-g62e659ed, icetime oss-cad-suite 20260811,
Darwin arm64). Base is `main` at e008baa1 and the candidate is the tree of this amendment. The
script's own verdict refuses the pair, because it compares the two trees' commit ids; the numbers
below come from `soc/baseline_summary.py --allow-mismatch` over its two CSVs.

| | main (e008baa1) | this tree |
|---|---|---|
| packed `ICESTORM_LC`, SoC | 5,205 | 5,209 (+4; 98.7% of 5,280) |
| `make fit`, local | 4,492 | 4,503 (+11) |
| worst placement | 78.60 ns, 12.72 MHz | 81.50 ns, 12.27 MHz |
| median | 76.15 ns, 13.13 MHz | 77.47 ns, 12.91 MHz |
| best | 74.16 ns, 13.48 MHz | 75.16 ns, 13.30 MHz |
| spread | 6.0% | 8.4% |
| under 12.00 MHz | 0 of 16 | 0 of 16 |

Per-seed MHz, `default, 1..15`:

- main: 13.43 13.16 13.06 13.01 13.48 12.72 13.33 12.80 12.97 13.32 13.08 13.10 13.06 13.19 13.26 13.38
- this tree: 13.25 12.70 12.82 13.12 12.70 13.30 12.97 12.61 12.93 13.00 12.27 13.18 12.45 12.89 12.86 12.98

The worst placement clears 12.00 by 2.2%. **The cell count is a draw of the mapper.** The first
spelling of these tests packed to 5,325 cells and nextpnr refused it (`Failed to expand region`);
the same logic respelled packed anywhere from 5,238 to 5,316 across eight carry-chain spellings and
six statement orders, and a comment's presence moved it too. Making the repetition count land on
exactly 32 moved it again, in both directions: the carry-out spelling with the run's first bit
counted packed to 5,281 and did not place, and five more spellings read 5,209 to 5,319. This tree
ships the one that packed to 5,209: a 10-bit sample counter whose wrap ends BIST, a repetition
count that compares against 30 and resets to all ones so its first increment lands on zero, a
block-aligned starvation flag in place of a six-bit counter, and an adaptive count offset by 102 so
the 410th match is the one that finds it all ones. At 98.7% occupancy the next edit to any file
synthesis reads owes this sweep again, and no test may be dropped to buy cells back.

`make soc-seed-search` re-pinned `soc/pin.json` to seed 20382078 at 13.19 MHz (9.9% over 12.00).
`make fit` reads 4,503 against `FIT_MAX_LC` 4,586, unchanged: the budget holds with 83 cells to
spare locally, and the churn band of ADR-0220 is about that wide, so a CI count above the local one
can still trip it. `make ecp5-timing` reads 39.95 MHz (25.03 ns, one placement at the default seed) with `DP16KD` 36, `TRELLIS_DPR16X4` 32 and
`MULT18X18D` 4 as declared and no block-RAM reset driven by logic. `make -C formal all` passes
with the baselines unchanged, and `make cosim-suite` agrees on 73 of 81 against
`COSIM_EXPECTED_FAIL`.

## Amendment: SB_LFOSC measured on the UPduino, and the shipped source fails (2026-10-18)

Measured on one UPduino v3.0 (iCE40UP5K-SG48), tree at `main` b995cb32 plus this change, the core-less
capture tops below, `SB_HFOSC` at 12 MHz as the capture clock, 1,000,000-sample captures graded by NIST's
`ea_non_iid` at the commit `mk/nist90b.mk` pins. **`SB_LFOSC` as `soc/board_upduino.v` wires it
carries 0.046 bit of min-entropy per sample, not the 0.5 the proportion cutoff assumes, and the
adaptive proportion test would set DEAD within a few windows of every boot.** No health test, cutoff or
fold depth was changed. The estimates and the proposed source change are below; the source change is
not made here.

**Only the SB_HFOSC clock could be measured.** This board's crystal reaches the FPGA through a solder
jumper that is open (ADR-0130). A capture built on the crystal pin sent no byte in 6 s, where the same
design on `SB_HFOSC` filled the UART. The "both clocks" comparison, which would show whether the two
oscillators couple, is therefore not made. `SB_HFOSC` read 1,199.5 cycles per `SB_LFOSC` edge, so the
oscillator runs at 10.004 kHz if the clock is at its nominal 12 MHz.

**The capture.** `soc/entropy_capture_upduino.v` is `soc/board_upduino.v` with the core replaced: the same
`SB_LFOSC`, the same 3-flop synchroniser, the same UART pin and its release gate, and either
`soc/entropy_interval_capture.v` (the edge intervals, moved out of the iCESugar-Pro capture top, which now
instantiates it) or `soc/entropy_bit_framer.v` (the gapless sample bits). `FAB_LOG2` divides
`SB_LFOSC` by 2^FAB_LOG2 in fabric on its own clock before the edges are timed. The top places at
about 45 MHz against the 12 MHz clock. `soc/entropy_upduino.py` drives the board: `iceprog` writes the
configuration flash once, `iceprog -t` reboots the FPGA from it with no write, and `ftread` reads the
UART, each as the root-owned copy `sudo` allows. **`iceprog -S` was tried and is not used: it ends with the
FPGA's CDONE low.** `ftread` prints a byte outside 0x20 to 0x7e as `<xx>`, which the script decodes, and
the frame CRC and sum catch the rare text that reads two ways.

**The line must never idle.** With nothing to send, the framer left the UART silent between frames and
the host saw 28% of them, with `a5 xx` stray bytes ahead of each marker; the same bitstream with a zero
byte sent in every idle slot (`FILL`, off for the iCESugar-Pro) delivered frames with no bad CRC in
10 s. The cause is on the host side of the FT232H and is not understood. The reader still loses
about one frame in a hundred, so a capture is the good frames in order with a **seam** where one was lost,
and the count is printed. Seams per million samples: 51 with no divide, 1,046 at 2^2 (a loaded machine),
322 at 2^3, and 58 per 500,000 at 2^4. A seam joins two stretches 64 or more samples apart, which touches
at most one adjacent pair in a thousand.

**Results.** Intervals are in 12 MHz core cycles. The repo's own estimators run on 4,096-interval runs
(`soc/entropy_assess.py`, as for the iCESugar-Pro) and read high against NIST's, as they did there.

| fabric divide | cycles per edge | interval sd | repo estimator, min / median / max (runs) | NIST H_original (1M samples) | proportion windows tripped |
|---|---|---|---|---|---|
| none (**shipped**) | 1,199.5 | 0.71 | 0.178 / 0.460 / 0.728 (24) | **0.0459** (t-tuple; LRS 0.082, MultiMCW 0.068) | 895 of 1,953 on the stream |
| 2^1 | 2,398 | 1.37 | 0.459 minimum (8) | not captured | 0 of 64 (model) |
| 2^2 | 4,797 | 2.52 | 0.753 minimum (12) | **0.4369** (compression; collision 0.477, t-tuple 0.469) | 0 of 1,953 |
| 2^3 | 9,591 | 6.17 | 0.859 minimum (2) | **0.7045** (collision) | 0 of 1,953 |
| 2^4 | about 19,200 | not captured | not captured | **0.8087** (compression; 500,000 samples) | not run |

The shipped source's sample bit is 1 for 77.5% of the million samples, and the share moves from 0.51 to
0.95 between consecutive 4,096-sample blocks: the interval is 1,199.5 cycles plus a slow drift of a
fraction of a cycle, and the sample bit is the parity of that interval's low two bits, so a drift of
the oscillator against the clock re-rolls it far more than the jitter does. Dividing multiplies the
drift by the divisor and the white jitter only by its square root, which is why 2^1 and 2^2 are not
enough and 2^3 is. The 2^2 setting passes the proportion test on every window and is still under 0.5
by NIST's estimate, which is the case the 0.5 floor is for: a stream the health tests accept and
the estimators do not.

**The periodic lock.** The sample-bit streams show none at lags to 4,096. `soc/entropy_upduino.py
periodic` takes each lag's self-match and subtracts the median of the eight lags either side, so slow
drift, which moves the match smoothly, leaves no peak and a repeating pattern does (a period-7 pattern
stands 0.4 above its neighbours in the test). The most prominent lag stands 0.009 above its neighbours at
no divide, 0.012 at 2^3 and 0.003 at 2^4, under the 0.02 limit. **At 2^2 it stands 0.035 above at lag 2 and
the tool reports PERIODIC**; that is a weak excess of lag-2 agreement, nowhere near a lock (a lock sits
near 1), and it is part of what the collision estimate prices at 0.477. The shipped intervals have a
white spectrum over every period under 64 intervals (the strongest bin is 9.3 times the mean at the median frame against
7.6 expected of noise), so no beat with the clock shows at short period either.

**What the shipped design does on this board.** 895 of 1,953 aligned 512-sample windows hold at
least 410 equal samples, so each window has a 46% chance of setting the sticky DEAD and BIST alone
spans two windows: `seed` reads DEAD within a few tenths of a second of nearly every boot. The Zkr claim is true of the
`-march` string and false of this board.

**Proposed, not made: divide `SB_LFOSC` by 8 in fabric and lengthen the edge timeout.** Three flops in
`soc/board_upduino.v` clocked by the oscillator give 9,591 cycles per edge and 0.70 bit. That is 117% of
`rtl/trng.v`'s 8,192-cycle edge timeout, so the timeout must grow to about 12,288 cycles (the 80% rule of
`test/entropy_geometry_test.py` puts the floor at 11,989) before the source can be used, and the timeout
is the 12-bit `ticks` counter plus the `late` flag whose cells already cost +56 (ADR-0252, amended). The
three flops cost three cells outside `littlesoc`; the timeout costs the cells of one more bit of
`late`. Neither is measured on the full SoC, because the UPduino board top does not place on `main`
today (5,306 of 5,280 `ICESTORM_LC`, measured by another run). Those numbers belong to whoever makes
the change; **it is the owner's call whether the cells come from a smaller timeout counter, a different
sampler, or a claim of less than 0.5.**

**Restart.** Only the 2^3 setting, the one that clears 0.5, was put through the matrix: 1,000 reboots of
1,000 samples, each a `iceprog -t` reset of the FPGA from its flash with the first 1,000 samples after
sequence 0 kept. `ea_restart` passed, with H_r 0.716889, H_c 0.825295 and H_I 0.704548, so
min(H_r, H_c, H_I) is **0.704548**, and `soc/nist90b_grade.py` graded it PASS against 0.5. The other
settings fail on H_original alone (0.046 and 0.437), and a restart test cannot raise a minimum above its
own H_I. Roughly one attempt in two was discarded and retaken for a lost frame or a late reader (the matrix took
three resumed runs, 2,000 or so attempts), so the kept rows are the attempts that read cleanly, and the
loss belongs to the reader and not to the source. **The capture design holds the first frame back 2^25 clock cycles (2.8 s)
after configuration so the reader is listening**, so a row is the 1,000 samples that start 2.8 s after the
oscillator began, not the first ones: the restart rows test the source warm, and a cold start of
`SB_LFOSC` is not seen here. The oscillator is enabled at configuration and the SoC reaches ES16
only after BIST discards 1,024 samples (0.1 s at 10 kHz), so the first 2.8 s are not what a user would
read either.

**Reproducing.** `make entropy-nist-upduino ENTROPY_UPDUINO_FAB=3` builds, flashes and captures (the
board lock is held for a restart matrix's whole run); `soc/entropy_upduino.py intervals` collects interval
frames and `soc/entropy_upduino.py periodic FILE` scans a sample file. `test/entropy_upduino_test.py`
grades the decoding, the scan and the loader without a board, with forced-red probes.
