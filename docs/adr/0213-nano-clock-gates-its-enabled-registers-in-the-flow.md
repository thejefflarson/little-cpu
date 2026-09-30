# ADR-0213: nano clock-gates its enabled registers in the flow

**Status:** Superseded by ADR-0219 · 2026-09-27

**yosys 0.62, which the Tiny Tapeout flow bundles, gates a sync-reset flop on its enable alone, so the gated netlist never reset `next_pc`. ADR-0219 turns clock gating off.**

## Context

The full nano chip did not fit a 4×2 Tiny Tapeout tile after every spec-legal RTL cut (ADR-0212): run
36153840938 reached 86,767 µm² flow synthesis, 68.2% placement and 145.5% routing demand, and died at
detailed placement. Relaxing the clock target and changing the synthesis strategy moved neither area
nor congestion. The owner asked for something other than the three structural cuts on the table
(ADR-0209's latch register file, read-only-zero counters, a bit-serial datapath).

Half of nano is flip-flops, and open_pdks' sky130 `no_synth.cells` excludes `sky130_fd_sc_hd__edfxtp_1`.
So LibreLane builds each of nano's 800 enabled flip-flop bits as a plain `dfxtp` plus a separate
`mux2_1` holding the old value — 800 cells and 1,600 short nets that exist only because the enable
flop is unavailable, concentrated on met1, the saturated layer.

## Decision

`nano/tt/src/config.json` sets `SYNTH_CLOCKGATE_MIN_WIDTH` to 8 and `SYNTH_CLOCKGATE_POSEDGE_ICG` to
`sky130_fd_sc_hd__dlclkp_1/GATE/CLK/GCLK`. LibreLane's synthesis then runs yosys's `clockgate` pass,
which replaces each group of at least eight flip-flops sharing an enable with one integrated
clock-gating cell driving their clocks, and drops the per-bit multiplexers. The RTL is unchanged: every
`always_ff ... if (en)` stays as written, with no `ifdef` and no hand-instantiated cell.

Width 8 because 4 and 8 give the same grouping on today's design, and 8 keeps a future small register
from growing a clock-tree leaf for one multiplexer's worth of area.

tt-support-tools builds `config_merged.json` by applying its own structural keys over `config.json`,
and for sky130 those are only `DESIGN_NAME`, `VERILOG_FILES`, `DIE_AREA`, `FP_DEF_TEMPLATE`, the power
pins and `RT_MAX_LAYER`, so both settings reach LibreLane unchanged.

## Measurement

Run 36291860337, 2026-09-27: 4×2, `AREA 2`, congestion allowed through, today's RTL, only these two
settings added.

| 4×2 | without (36153840938) | with clock gating (36291860337) |
| -- | -- | -- |
| flow synthesis | 86,767 µm² | 74,171 µm² |
| placement | 68.2% | 59.3% |
| routing demand | 145.5%, 85,071 overflow | 88.3%, 14 overflow |
| met1 / met2 / met3 / met4 | 185.6 / 130.9 / 144.7 / 87.5% | 93.5 / 98.1 / 85.3 / 59.6% |
| hold endpoints (`RSZ-0046`) | 760 | 506 |
| furthest stage | detailed placement, failed | detailed routing, cancelled |

The 12,596 µm² drop matches the 800 × 11.26 µm² `mux2_1` cells the gating removes, plus their share of
the repair buffers. The full chip placed on a 4×2 and passed global routing for the first time.
Detailed routing started at about 59,000 violations and was at 48,632 after 80 minutes when the job's
150-minute timeout cancelled the run. Peak memory was 6,026,825,728 bytes, against the runners' 6 GiB
limit.

**This does not yet fit a 4×2.** Detailed routing has not finished, and DRC, LVS and antenna have not run.

## Consequences

- The self-hosted workflow's job timeout rises from 150 to 360 minutes, so a 4×2 run can finish
  detailed routing and be read rather than cancelled.
- **Nothing in the tree can see clock gating.** It exists only in the hardened netlist; RTL simulation
  and every formal check read the RTL. A gate-level simulation of the hardened netlist, with forced-red
  probes, is owed before any tapeout (ADR-0163 is the precedent for a mapped netlist behaving unlike its
  RTL). Until it exists, clock gating is unverified.
- `make nano-area` still synthesizes without clock gating, so `NANO_MAX_UM2` describes a netlist the flow
  no longer builds. Mirroring the pass in `nano/synth_script.sh` and re-deriving the ceiling is owed.
- Hold into gated flip-flops is the named risk: a gated clock arrives later, and if clock-tree synthesis
  does not balance through the gating cell, delay cells eat the saving. Read `RSZ-0046` and the delay-cell
  census on every run.
- The slow corner is unchanged and unsolved; a gated launch path adds about 0.3 ns.
- `docs/ideas/nano-on-a-4x2-the-tile-is-wiring.md` holds the plan this is Tier 0 of.
