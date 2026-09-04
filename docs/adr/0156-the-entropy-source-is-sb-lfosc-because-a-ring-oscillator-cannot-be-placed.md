# ADR-0156: The entropy source is `SB_LFOSC`, because a ring oscillator cannot be placed

**Status:** Accepted · 2026-09-02

## Context

The embedded-HSM direction needs an entropy source. `Zkr`'s `seed` CSR is the ratified
interface to one, not a source, and this board has none: SPRAM has no initialisation, the
flash controller is read-only and not board-wired, and the UART has no receiver, so entropy
cannot be injected either.

The obvious candidate was a fabric ring oscillator. This records why it is closed, and what
replaces it. The design brief is `docs/ideas/entropy-behind-seed.md`.

## What was measured

yosys 0.68+post (`c12172fb`) and its bundled nextpnr-ice40, on this machine, 2026-09-02:

| spelling | yosys | nextpnr `--up5k --package sg48` |
|---|---|---|
| plain RTL ring | destroyed — output left undriven | — |
| `(* keep *)` on the loop wires | destroyed, both `synth_ice40` and `synth_ecp5` | — |
| `keep_hierarchy` submodule + `keep` wires | destroyed, 0 LUTs, both targets | — |
| `SB_LUT4` ring (ice40) | survives, 3 cells | `ERROR: Timing analysis failed due to combinational loops`; places only with `--ignore-loops` |
| `LUT4` ring (ECP5) | survives, 3 cells | not taken further |
| `SB_LPOSC` | does not exist | does not exist |
| `SB_LFOSC` | modelled | **placed, exit 0** |
| `SB_HFOSC` | modelled | placed, exit 0 |

## Why an RTL ring is destroyed, and why it is not loop detection

yosys does not identify oscillators, and its loop detection does not delete anything.
`passes/cmds/check.cc` builds a wire-to-wire dependency graph, runs `TopoSort`, and for each
cycle emits `log_warning("found logic loop in module %s")` — it reports and stops. An SCC
pass *inside* `synth_ice40` also finds the cycle (`Found an SCC: l1 l0 l2`) and carries it
through, because SCC detection exists so ABC9 can map a cyclic netlist at all.

What destroys an RTL ring is duller: **`keep` does not propagate through technology
mapping.** The inverters become LUTs, the attribute is lost in the translation, and ordinary
`opt_clean` sweeps the now-unmarked cells as unused. That is why `keep` on a wire yields
"wire used but has no driver" — the wire is kept and its driver is not — and why only
already-mapped primitives survive.

**This is a tripwire.** The next person to want a ring will write `(* keep *)` on the wires,
get a design with no oscillator, and see nothing but an undriven-wire warning.

## Why the primitive ring is still declined

It survives synthesis and is refused by nextpnr, placing only under `--ignore-loops`. That
flag is **global**: it disables combinational-loop detection for the whole design, not for
the intended loop. Buying one deliberate loop by unwatching every accidental one is not a
trade this project makes — not in `make soc-timing`, and not in the bitstream flow that
reaches silicon either.

## Decision

**The source is `SB_LFOSC`** — the up5k's ~10 kHz low-frequency oscillator, a hard macro
costing zero logic cells (Lattice FPGA-TN-02008; the part has exactly two on-chip
oscillators, and the design is already clocked from the other one). It is instantiated in
`soc/board_upduino.v`, which already owns the part-specific `SB_HFOSC` and sits outside every
placed instrument, and drives one new `littlesoc` input, `entropy_raw`.

This is not a substitute for a ring; it is strictly better. No combinational loop anywhere,
so loop detection stays on across every flow, and no `keep` survives-mapping archaeology.

**Portability comes from the port, not from portable oscillator RTL.** A ring cannot be
written portably — only instantiated per part — so the seam belongs at the board file, where
part-specific instantiation is already allowed. Each board supplies its own source;
simulation and formal drive `entropy_raw` deterministically, and an input tied to a constant
folds before mapping, so the tie-off costs nothing.

## Consequences

- ECP5 gets no entropy for now: `entropy_raw` tied low reads `DEAD`, which is spec-honest.
  `OSCG` exists there if it is ever wanted, and needs no loop either.
- The fallback if the source proves too quiet is to raise `ACCUM`, and then to decline the
  `_zkr` claim with its measurement. A ring under `--ignore-loops` is not the third rung.
- The physical entropy claim is staged behind a board measurement, the way the A extension's
  `misa` bit was staged behind its suite: the device, CSR and raw tap land **unclaimed**, and
  `_zkr` joins the ISA string in the same change as the measurement ADR.
