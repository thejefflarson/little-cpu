# Entropy behind `seed`, with the physical claim staged behind a board measurement

Status: brief. Settled 2026-09-02. Ticketing is `/plan-sprint`'s; building is `/work`'s.

## The idea

Give the core a real entropy source behind `Zkr`'s `seed` CSR (0x015), and answer whether
it and Zicond (`czero.eqz`/`czero.nez`) both fit the SoC's free cells.

## Why now

Entropy is the blocker on the embedded-HSM direction with a *hardware* dependency. `snow`
needs a fresh X25519 ephemeral per handshake, so without a noise source the target protocol
does not run slowly — it does not run. `seed` is the ratified interface to a source, not a
source. This board has none: SPRAM has no initialisation, the flash controller is read-only
and not board-wired, and the UART has no receiver, so entropy cannot be injected either.

It is *a* blocker rather than *the* blocker, and the brief does not pretend otherwise: a
transmit-only UART cannot run a Noise handshake in either direction, and 8 KB of text is
tight for X25519 plus snow. Entropy is the right piece to build first because it is the one
gated on silicon.

## The source: `SB_LFOSC`

The up5k has exactly two on-chip oscillators (Lattice FPGA-TN-02008): `SB_HFOSC` at up to
48 MHz with a divider, which already clocks this board, and **`SB_LFOSC` at ~10 kHz**. Both
are hard macros costing zero logic cells.

`SB_LFOSC` goes in `soc/board_upduino.v`, which already owns the part-specific `SB_HFOSC`
and sits outside every placed instrument, and drives one new `littlesoc` input,
`entropy_raw`. Everything downstream is ordinary synchronous RTL: synchroniser, interval
counter, von Neumann corrector, XOR fold, buffer, status FSM, repetition-count test.

Portability comes from the port, not from portable oscillator RTL. Each board file supplies
its own source; simulation and formal drive `entropy_raw` deterministically, and a tied
input folds before mapping.

## Why not a ring oscillator — closed, not deferred

Measured on yosys 0.68+post and nextpnr-ice40, both targets:

| spelling | yosys | nextpnr |
|---|---|---|
| plain RTL ring | destroyed, `out` undriven | — |
| `(* keep *)` on the wires | destroyed | — |
| `keep_hierarchy` submodule | destroyed, 0 LUTs | — |
| `SB_LUT4` / `LUT4` primitives | survives, 3 cells | refused; places only with `--ignore-loops` |
| `SB_LFOSC` | modelled | **placed** |

**yosys never identifies an oscillator.** `check.cc` builds a wire-to-wire graph, runs
`TopoSort`, and only *reports* cycles; an SCC pass inside `synth_ice40` finds the loop too
(`Found an SCC`) and lets it through, because SCC detection exists for ABC9's benefit. What
destroys an RTL ring is that **`keep` does not propagate through technology mapping** — the
NOT gates become LUTs, the attribute is lost in translation, and `opt_clean` sweeps them as
unused. That is why only instantiated primitives survive, and why a primitive is
part-specific by construction.

The primitive ring is then refused by nextpnr — *"Timing analysis failed due to
combinational loops"* — and places only under `--ignore-loops`, which is **global**. Buying
one intended loop by unwatching every accidental one is not a trade this project makes, in
either the graded flow or the bitstream flow that reaches silicon.

So `SB_LFOSC` is not a substitute for a ring. It is strictly better: no loop anywhere, loop
detection on everywhere, zero cells.

## Decisions

1. **`seed` never stalls.** A not-ready read returns `WAIT` as data in the same cycle count
   as `ES16`. No ninth stall reason, F and G untouched, the eight-reason OR untouched, the
   six-place declaration list untouched. It also makes the Zkt claim safe by construction:
   no value-dependent timing exists, and `seed_data` arrives as a port rather than from
   `reg_rs1`/`reg_rs2`/`rd_data`, so the netlist taint grader finds no path. Run it anyway.
2. **Access semantics per the spec.** Reads require a read-write instruction; CSRRS/CSRRC
   forms raise illegal instruction, committed in decode with every other cause. Reads are
   destructive. Writes ignored. No `mseccfg` — M-mode only.
3. **Status FSM**: reset → `BIST` until the first repetition-count window passes; buffer
   empty → `WAIT`; sixteen bits ready → `ES16`; a hard repetition-count failure → `DEAD`,
   sticky.
4. **Health tests split, and the split is stated rather than hidden.** Repetition-count in
   hardware, because it is what gives `DEAD` meaning and catches a stuck oscillator.
   Adaptive-proportion in boot firmware through the raw tap. SP 800-90B's full apparatus is
   not owed in fabric on a hobby core; honesty about which half is where is.
5. **Conditioning**: von Neumann then XOR-fold in hardware; cryptographic conditioning is
   software's, per the spec's own architecture. Throughput is irrelevant — one 256-bit DRBG
   seed at boot.
6. **The claim follows the measurement**, the way the A extension's did. Device, CSR and raw
   tap land **unclaimed**; `_zkr` enters the ISA string in the same change as the
   board-measurement ADR.
7. **Raw tap**: one read-only word at `0x0002_0030` — next free, outside the timer's
   reserved span. It is the validation path and the firmware APT path.

## Testability

Four separable layers, only one non-deterministic:

- The digital path is fully deterministic behind `entropy_raw` and testable with a driven
  stream.
- The health tests are deterministic functions of their input, which makes them good
  `probe-gates` material: a stuck-at stream must trip repetition-count, a biased stream must
  trip adaptive-proportion, a passing stream must not.
- Statistical batteries run offline on captured output. Not a CI gate.
- Physical entropy is a bring-up measurement, the way `SB_HFOSC` was measured at
  11.98 ± 0.11 MHz two independent ways.

Co-simulation diverges, as every device access does; `test/asm/uart.S` and `spioverlay.S`
are the precedent and `test/COSIM_EXPECTED_FAIL`'s header is the decision procedure.

## Board day

`SB_HFOSC` measures 11.98 ± 0.11 MHz on this board — nominal within error. **The sampling
clock is quiet**, so the entropy rests on `SB_LFOSC`'s RC jitter against a stable ruler, not
on mutual wander.

Capture **full raw intervals**, not just LSBs: the interval is ~1,200 cycles of an 83 ns
clock, and the analysis needs to see how many low bits are noisy, because that sets
`DECIMATE` and may lower `ACCUM`. Look for per-bit bias and serial correlation of the
interval LSBs, `ea_non_iid` min-entropy on the raw stream, and slow drift of the interval
mean. `ACCUM = 8` is the default going in and the capture's to settle in either direction —
the measured HFOSC stability makes a shorter accumulation *more* likely to suffice.

## Fallback ladder

1. Raise `ACCUM` (and `DECIMATE`). Independent jitter accumulates as √N, and throughput is
   nearly free here.
2. If a longer accumulation still fails the analysis, **decline `_zkr`** and record the
   decline with its data. Claim-follows-measurement already makes that first-class.

There is no cheap third rung. A primitive-ring board build carrying `--ignore-loops` would
trade a global safety check for an entropy source, and nothing measured suggests it is
needed.

## Budget, and Zicond

| piece | instrument | packed LC |
|---|---|---|
| `seed` CSR + decode + ports | fit and soc-timing | 30–60 |
| `rtl/trng.v` conditioner + RCT | soc-timing | 100–140 |
| raw tap | soc-timing | 15–25 |
| Zicond | fit and soc-timing | 30–90 |
| total | | 175–315 of ~337 |

Nominally both fit. But that lands the SoC near 97–98.5% occupancy, past the 95.2% measured
harmless, so the answer is **sequenced, not asserted**: land the entropy stack, take its
sixteen-seed paired sweep and real cell count, then admit Zicond against the measured
remainder.

Scope levers in order if anything trips: hardware adaptive-proportion (already firmware),
then the raw tap into a bring-up bitstream, then **Zicond declined with its measurement**.
Never the entropy stack.

Zicond gets a real oracle that Zkr cannot: sail supports it, so co-simulation grades it.

## Sequence

1. **Core seam** — `seed` in `rtl/csrs.v`, decode legality, ports, tie-off baseline,
   `csr_tb` vectors both ways, the Zkt grader, `remeasure-fg`, `fit` and a sixteen-seed sweep.
2. **Device** — `rtl/trng.v`, raw tap, `trng_tb.v` with its red directions into
   `probe-gates`, a suite program against a harness-driven stream, co-sim baselines, sweep.
3. **Board day** — `SB_LFOSC` in `soc/board_upduino.v`, capture firmware, 90B offline,
   settle `ACCUM`/`DECIMATE`, measurement ADR, `_zkr` into the ISA string.
4. **Zicond**, gated on step 2's measured remainder.

Steps 1–2 are one sprint's shippable core, fully graded, with the source tied off.

## Deferred

Hardware adaptive-proportion (firmware at boot). Any fabric ring oscillator (closed above).
ECP5 entropy — tie `entropy_raw` low, which reads `DEAD` and is spec-honest. Everything else
the HSM needs, a UART receiver most of all.
