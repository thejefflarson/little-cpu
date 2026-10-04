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
| Repetition count | the folded output bits | the 32nd repeat of the previous bit |
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
| packed `ICESTORM_LC`, SoC | 5,205 | 5,241 (+36; 99.3% of 5,280) |
| `make fit`, local | 4,492 | 4,560 (+68) |
| worst placement | 78.60 ns, 12.72 MHz | 83.10 ns, 12.03 MHz |
| median | 76.15 ns, 13.13 MHz | 78.03 ns, 12.82 MHz |
| best | 74.16 ns, 13.48 MHz | 75.98 ns, 13.16 MHz |
| spread | 6.0% | 9.4% |
| under 12.00 MHz | 0 of 16 | 0 of 16 |

Per-seed MHz, `default, 1..15`:

- main: 13.43 13.16 13.06 13.01 13.48 12.72 13.33 12.80 12.97 13.32 13.08 13.10 13.06 13.19 13.26 13.38
- this tree: 13.11 12.87 12.03 12.78 12.76 12.95 12.93 12.86 12.87 13.16 12.73 12.52 12.58 12.44 12.76 13.15

The worst placement clears 12.00 by 0.25%. **This is a draw of the mapper, as the first
measurement above was.** The first spelling of these tests packed to 5,325 cells and nextpnr
refused it (`Failed to expand region`); the same logic respelled packed anywhere from 5,238 to
5,316 across eight carry-chain spellings and six statement orders, and a comment's presence moved
it too. This tree ships a spelling that packed to 5,241: a 10-bit sample counter whose wrap ends
BIST, a carry out for the repetition count, a block-aligned starvation flag in place of a six-bit
counter, and an adaptive count offset by 102 so the 410th match is the one that finds it all ones.
At 99.3% occupancy the next edit to any file synthesis reads owes this sweep again, and no test
may be dropped to buy cells back.

`make soc-seed-search` re-pinned `soc/pin.json` to seed 125781539 at 13.14 MHz (9.5% over 12.00).
`make fit` reads 4,560 against `FIT_MAX_LC` 4,586, unchanged: the budget holds with 26 cells to
spare, and the churn band of ADR-0220 is wider than that, so a CI count above the local one trips
it. `make ecp5-timing` reads 37.95 MHz (26.35 ns) with `DP16KD` 36, `TRELLIS_DPR16X4` 32 and
`MULT18X18D` 4 as declared and no block-RAM reset driven by logic. `make -C formal all` passes
with the baselines unchanged, and `make cosim-suite` agrees on 73 of 81 against
`COSIM_EXPECTED_FAIL`.
