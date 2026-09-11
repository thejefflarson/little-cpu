# ADR-0177: nanocpu keeps its features before cutting them, and RV32E is the one cut

**Status:** Accepted · 2026-09-11

## Context

nanocpu targets a Tiny Tapeout shuttle. A 2×2 tile holds **72,565 µm² of core area** in Tiny Tapeout's layout flow. Shipped 2×2 RISC-V designs measure TinyQV at 53,888 and 66,006 µm² and FazyRV-ExoTiny at 56,448 µm², 91% of the tile.

`make nano-area` is a local estimate: `synth; dfflibmap; abc -liberty` against the pinned sky130hd liberty. It reads the donor core at **84,290.8 µm²** on 0f66638. The layout flow comes out at about **0.915×** the local figure, calibrated on FazyRV-ExoTiny, which reads 61,673 µm² locally and shipped at 56,448. Below, "local" means the `make nano-area` figure and "layout flow" means local × 0.915.

The donor also lacks two things any shippable nanocpu needs: a CSR and trap layer, about +6,000 µm² local, and a QSPI front end, about +5,500 µm² local, because Tiny Tapeout has no RAM and code runs from QSPI flash.

Two documents disagreed about the 64-bit `mcycle`/`minstret` counters, which cost about 5,000 µm². The nanocpu brief allowed them to be the last cut if area ran short. CLAUDE.md says conformance is not negotiable against minimality.

## Decision

**Keep features before cutting them.** A feature is cut only when no saving that keeps it closes the gap.

- **The counters stay.** They are not a candidate cut. CLAUDE.md's conformance rule applies to nanocpu unchanged, and no exception is written into it.
- **M stays.**
- **RV32E is the one cut, on a 2×2 tile.** Keeping every feature does not fit:

| Configuration | Local µm² | Layout flow µm² | Fits a 2×2 (72,565)? |
| -- | -- | -- | -- |
| Every feature: donor + CSRs/traps + QSPI front end | ~95,800 | ~87,700 | no |
| Every feature, plus the one-port and latch register files | ~85,300 | ~78,000 | no |
| **RV32E only: M and the counters kept** | **~72,400** | **~66,200** | **at ~91%** |
| RV32E and no M | 48,000–55,000 | 44,000–50,000 | yes |

A 3×2 tile would hold every feature, at six tiles instead of four, about +€140 at €70 a tile. It was considered and not taken.

**If area still runs short, cut in this order, and cut a feature only after both:**

1. A one-port register file: −3,500 µm² local, at +1 cycle per instruction. That costs speed, not a feature.
2. A latch-array register file, as TinyQV builds one: about −7,000 µm² local. Latches complicate Tiny Tapeout's hold fixing and yosys's formal model, so this comes second.

**Decisions about nanocpu go in ADRs, not CLAUDE.md.** CLAUDE.md states rules for this repository's designs in general. A decision about one design, including any exception to a general rule, is recorded here.

## Consequences

**The fit is tight, and the RV32E saving is not yet measured.** About 66,200 µm² in the layout flow is 91% of the tile, and above the 56,000 µm² freeze line the nanocpu plan set for itself. The RV32E saving, −23,400 µm² local, is an upper bound: it was measured on register files synthesized alone. In the real core, yosys folds the register file's read multiplexers into the logic that consumes them, so the saving actually realized can only be smaller. The RV32E reshape measures it. If it comes in short, the cut order above applies before any feature goes.

**The RV32E oracle patch is needed and kept.** The `RISCV_FORMAL_E` assumption in `nano/formal/rvfi_insn_check.sv` is what stops riscv-formal from failing a correct RV32E core. The `ill_e` check built beside it is reverted, as ADR-0174's amendment records. A new one, wired to the real core, must go red on a wrong-rule mutation before RV32E is switched on.
