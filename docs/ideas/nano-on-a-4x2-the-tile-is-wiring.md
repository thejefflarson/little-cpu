# nano on a 4×2 — the tile is wiring, and the wiring is copies

**Status:** planned · Tier 0 landed · 2026-09-27. Every number below is measured and says where; the
flow figures are LibreLane 3.0.14 on the self-hosted Tiny Tapeout workflow, the local figures are
`make nano-area`'s recipe (yosys 0.68+48, pinned `sky130_fd_sc_hd__tt_025C_1v80.lib`). Local and flow
numbers are never merged. Where this brief and a later ADR disagree, the ADR wins.

## The problem

The full nano chip — core, QSPI controller, UART, GPIO, bus — fits a 6×2 Tiny Tapeout tile (run
36101051908: 49.8% placement, 57.7% routing demand, full flow complete) but not a 4×2. After every
spec-legal RTL cut (ADR-0209 to ADR-0212) the 4×2 still failed (run 36153840938): 86,767 µm² flow
synthesis, 68.2% placement, **145.5% routing demand** with met1 at 185.6%, dead at detailed placement.
The owner wants the 4×2 and has ruled out the latch register file, read-only-zero counters and a
bit-serial datapath.

Two measurements narrowed where the problem is not. Relaxing the clock target from 15.625 ns to 40 ns
changed nothing — identical synthesis area, identical placement, 144.9% routing — because the area and
the congestion are fixed at synthesis and placement, before timing repair runs. And `AREA 2` is the
smallest synthesis strategy measured (`AREA 1` 87,411 µm², `AREA 3` 110,985 µm²).

## What the design is made of

Flip-flops are half the chip: 1,371 of them are 35,460 µm² of 70,873 µm² local. Two findings follow.

**The flow adds a multiplexer to every enabled flip-flop.** open_pdks' sky130 `no_synth.cells` excludes
`sky130_fd_sc_hd__edfxtp_1`, so each of the 800 flip-flops with a write enable is built as `dfxtp` plus
a separate `mux2_1` — 800 extra cells and 1,600 extra short nets on exactly the layer that is saturated.

**About 400 flip-flops hold a copy of a value something else already holds.** `regs[0]` stores a constant
zero (32); `rd`/`rs1`/`rs2` are functions of the held `instr` (15); `mem_wdata`/`mem_wstrb` replicate
`op_rs2` (36); in the QSPI controller `tx_shift`, `mem_rdata`, the four `psram_*_pending` registers, the
prefetch slot and `second_parcel_first_data` copy a bus the core holds stable until `mem_ready` (~190).
Removing a copy saves about 24 µm² local per flip-flop, not the 36 a flop count suggests, because the
removed register's input mux is usually replaced by an equivalent output mux. `regs[0]` is the exception
(44 per flop), because the read mux and the write decode each lose a term too.

## The plan — tiers, each closed by one 4×2 run

**Tier 0 — clock gating in the flow, RTL untouched.** `SYNTH_CLOCKGATE_MIN_WIDTH` 8 and
`SYNTH_CLOCKGATE_POSEDGE_ICG` `sky130_fd_sc_hd__dlclkp_1/GATE/CLK/GCLK` in `nano/tt/src/config.json`.
yosys's `clockgate` pass replaces each group's per-bit enable multiplexers with one integrated
clock-gating cell. Measured on a 4×2 (run 36291860337):

| 4×2 | before (36153840938) | clock gating (36291860337) |
| -- | -- | -- |
| flow synthesis | 86,767 µm² | **74,171 µm²** |
| placement | 68.2% | **59.3%** |
| routing demand | 145.5%, 85,071 overflow | **88.3%, 14 overflow** |
| met1 / met2 / met3 / met4 | 185.6 / 130.9 / 144.7 / 87.5% | **93.5 / 98.1 / 85.3 / 59.6%** |
| hold endpoints | 760 | 506 |
| furthest stage | detailed placement, failed | **detailed routing**, cancelled |

For the first time the full chip placed on a 4×2 and passed global routing. Detailed routing started
at about 59,000 violations and reached 48,632 after 80 minutes, when the workflow's 150-minute job
timeout cancelled it; peak memory was 6,026,825,728 bytes against the runners' 6 GiB limit. The
timeout is raised to 360 minutes so a routing run can finish. Clock gating lands as ADR-0213.

**Tier 1 — three RTL edits every existing check already reads** (local, measured): `regs[0]` gets no
storage (−1,409 µm²), `mem_wdata`/`mem_wstrb` become wires (−855), `rd`/`rs1`/`rs2` are read from
`instr` (−211). With clock gating, 70,873 → 61,868 µm² local. met1 and met2 sit at 94–98% after Tier 0;
Tier 1 is what should pull them off the ceiling. Closed by a 4×2 run that finishes detailed routing.

**Tier 2 — the QSPI controller owns each datum once** (derived, about −4.3k µm² local): drop the prefetch
slot, the four pending copies, `mem_rdata`, `second_parcel_first_data` (into `rx_shift[31:16]`) and
`tx_shift` (a nibble mux feeding a registered `sio_out`, keeping ADR-0204's pad timing). Invariant 3 is
restated, not weakened. About one cycle faster per fetch. Only if Tier 1's run still does not route.

**Tier 3 — the core state machine collapses** (derived, about −3k local, and about three fewer cycles per instruction — roughly 10% faster on the pin-level benchmarks, at the price of longer combinational paths). Conditional
on Tier 2's run still failing; F and G are re-measured before it lands.

## Decisions

1. **Clock gating is a flow property, not an RTL one.** No `ifdef`, no hand-instantiated PDK cell, no
   behavioural gating model. If the flow ever refuses the cell, the fallback is a hand-instantiated
   `dlclkp_1` for the register file only, under a define — shipped Tiny Tapeout designs do this.
2. **A gate-level simulation becomes a required gate before any tapeout.** Clock gating exists only in the
   hardened netlist, so RTL simulation and formal verification cannot see it. The hardened netlist runs
   through the existing pins-only `nano/tb/nano_tt_tb.v`, against the PDK's cell models pinned by SHA,
   with forced-red probes: a gating cell whose enable is forced low must fail, and a netlist with no
   `dlclkp` cells while gating is configured must fail. ADR-0163 is the precedent for a mapped netlist
   behaving unlike its RTL.
3. **The local instrument mirrors the flow's netlist shape** (`clockgate` in `nano/synth_script.sh`, and
   `NANO_MAX_UM2` re-derived) — Tier 1's work, so the ratchet stops describing a netlist the flow never builds.
4. **The controller keeps `stream_next_addr`.** A one-bit "sequential" hint from the core would save about
   0.9k local but make the controller's correctness depend on a core property.
5. **The fallback trigger is a number.** If a tier's 4×2 run does not finish detailed routing with zero
   violations and the next tier's projected saving cannot plausibly close it, stop and plan a 3×4 — the
   squarer 12-tile option (Tiny Tapeout's sky130 templates: 1x1, 1x2, 2x2, 3x2, 3x4, 4x2, 4x4, 5x4, 6x2,
   6x4, 8x2, 8x4; 4×2 is the only 8-tile shape).

## Risks

- **Detailed routing near a full layer.** Global routing says the wiring fits at 88%; the detailed router
  was still clearing violations slowly at met2 98%. Tier 1 exists to buy that margin.
- **Hold into gated flip-flops.** A gated clock arrives later; if clock-tree synthesis does not balance
  through the gating cell, delay cells eat the saving. Read `RSZ-0046` and the `dlygate` census per run.
- **Runner memory.** The Tier 0 run peaked within 0.4 GB of the 6 GiB limit.
- **The slow corner** is unsolved and unchanged by this work; a gated launch path adds about 0.3 ns.

## Prior art

Shipped Tiny Tapeout sky130 netlists contain `sky130_fd_sc_hd__dlclkp` gating cells —
`tt_um_MichaelBell_canon` (tinytapeout-08), `tt_um_riscyv02` (sky-26a), `tt_um_kul_chromechain` (sky-26c) —
and tt-support-tools' `precheck.py` restricts no cell type. LibreLane added `SYNTH_CLOCKGATE_MIN_WIDTH` and
`SYNTH_CLOCKGATE_POSEDGE_ICG` in 3.0.0; Tiny Tapeout runs 3.0.14.

## Handoff to plan-sprint

Theme: **"The tile is wiring, and the wiring is copies."** Tier 0 has landed. Sprint 1 is Tier 1's three
RTL edits plus the local instrument's mirror, closed by a 4×2 run that finishes detailed routing, and
the gate-level simulation gate as its own ticket, which must land before any tapeout. Tier 2 is the
second sprint and Tier 3 a conditional third, each triggered by the previous run's result.
