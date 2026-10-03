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
