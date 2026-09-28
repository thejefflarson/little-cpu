# ADR-0216: nano Tier 1 -- x0, mem_wdata/mem_wstrb and rd/rs1/rs2 stop being copies

**Status:** Accepted · 2026-09-27

## Context

ADR-0213 (Tier 0) turned on clock gating in the Tiny Tapeout flow with the RTL unchanged: on a
4×2 the full chip went from 145.5% to 88.3% routing demand and placed for the first time (run
36291860337), but detailed routing was still clearing violations -- met1 93.5%, met2 98.1% -- when
the job's 150-minute timeout (now 360) cancelled it. `docs/ideas/nano-on-a-4x2-the-tile-is-wiring.md`
names three of nano's ~800 enabled flip-flops as copies of a value something else already holds:
`regs[0]` stores a constant zero; `mem_wdata`/`mem_wstrb` replicate `op_rs2`/`store_wstrb`;
`rd`/`rs1`/`rs2` are functions of the held `instr`. Tier 1 removes all three, local-instrument
measured on their own, and mirrors Tier 0's clock gating in `make nano-area` so the ratchet
describes the netlist the flow actually builds.

## Decision

`nano/nano.v`, on top of clock gating already in the flow:

- `regs[0:15]`, declaration unchanged, but `regs[0]` is never written and never observably read.
  Every read through `rf_raddr` (`fetch_rs1`/`fetch_rs2`) and every read of `regs[rd[3:0]]` (the
  register-write state, RVFI's retired-value report) is guarded by `|rf_raddr`/`|rd[3:0]`,
  returning 0 rather than the array's value; the write to `reg_write` is skipped outright when
  `rd[3:0]` is 0. `rd[4]` (an RV32E out-of-range register number) always traps before either read
  guard is reached, so `|rd[3:0]` reads exactly the values `|rd` used to. **`regs[1:15]` was tried
  first and reverted**: narrowing the declared range made `nano/formal`'s generic `cover` check
  come back `ERROR` rather than `PASS` -- `mode cover` still found the goal reachable (`bad state
  property 0 reachable`), but `yosys-witness wit2yw`'s replay of the witness died with "out of
  bounds address in BTOR witness file". `op_rs1 <= |rf_raddr ? regs[rf_raddr] : 32'b0` computes
  `regs[rf_raddr]` as an ordinary dataflow operand of the mux, not a conditionally-skipped
  read, so the read port's address carries the RTL-level value 0 on every cycle an x0 operand is
  fetched (constantly, since the suite uses x0 as a source on nearly every branch and comparison);
  yosys rebases a `regs[1:15]` array's physical addressing to its declared low bound, so that
  address arrives at the array as `0 - 1`, genuinely out of the 15-word physical range. Leaving
  the declaration 0-based sidesteps the rebasing arithmetic: address 0 is a legal index into a
  16-word array, so nothing is ever out of range, and `regs[0]`'s storage is still dead code by
  the same two guards -- confirmed below to synthesize away for the same area, in fact slightly
  more.
- `mem_wdata` and `mem_wstrb` become continuous assigns: `mem_wdata` selects `op_rs2`'s byte,
  halfword or full word by `is_sb`/`is_sh`/is_sw (default), and `mem_wstrb` is `store_wstrb` gated
  on `cpu_state == finish_store` and zero otherwise -- the same value the old registered pair held
  for the same cycles, since both were previously written on the same clock edge that entered
  `finish_store` and held until `fetch_instr` cleared them. `nano/uart.v`'s `start_frame` still
  gates on `mem_wstrb[0] && !busy`, unaffected: the wire is asserted for exactly the cycles
  `cpu_state == finish_store`, the same span the flip-flop covered.
- `rd`, `rs1` and `rs2` become continuous assigns over `instr` directly, in place of the
  `decode_instr`-state flip-flops. Every reader of the three (`fetch_rs1`/`fetch_rs2` through
  `rf_raddr`, `is_e_illegal`, `is_error`, the reg-write state, every RVFI field) reads them no
  earlier than `decode_instr`, and `instr` does not change again until the next `ready_instr`
  completes -- after the whole current instruction has retired -- so a live read and the old
  latched one agree at every point either was ever read. The three assigns are ternary chains,
  not `case(1'b1)` inside `always_comb`: yosys's `proc` pass treats either shape identically, but
  iverilog's automatic sensitivity list for an `always_comb` case with a constant part-select of a
  wider signal (`instr[9:7]`, `instr[24:20]`, ...) falls back to sensitizing the whole word --
  the same "sorry" `rtl/writeback.v` is allowlisted for and no other file may reintroduce.

`nano/synth_script.sh` and `nano/timing_script.sh` insert
`clockgate -min_net_size 8 -pos sky130_fd_sc_hd__dlclkp_1 GATE:CLK:GCLK` between `synth` and
`dfflibmap`, the same pass and cell ADR-0213 configured for the flow. `sky130_fd_sc_hd__dlclkp_1`
is already in the pinned liberty, so `nano/area_report.py`'s cell-name check needs no change --
confirmed by a real run below.

## Measurement

`make nano-area`, this tree, before clock gating: baseline 70,873.0 µm², matching ADR-0212's
figure. Each RTL edit ablated alone, clock gating off:

| edit | area (µm²) | delta | brief's ablation |
| -- | -- | -- | -- |
| baseline | 70,873.0 | -- | -- |
| `regs[0]` unstored (`regs[0:15]`, guarded) | 68,699.6 | −2,173.4 | −1,409 |
| `mem_wdata`/`mem_wstrb` wires | 69,351.5 | −1,521.5 | −855 |
| `rd`/`rs1`/`rs2` from `instr` | 70,983.1 | **+110.1** | −211 |

The third ablation disagrees with the brief's estimate in sign, reproducibly (re-run identical to
the last digit). Removing a flip-flop and reading its source combinationally instead does not
always shrink the mapped netlist -- ABC's cone factoring around a register boundary is not the
same question as counting the register it removes, the same point CLAUDE.md's `rtl/writeback.v`
`wen`-mask history already makes for littlecpu, now reproduced here. All three edits are shipped
regardless, per the brief's brief being a scope list rather than a per-edit area gate.

All three together, clock gating off: **67,509.7 µm²**, taken with the reverted `regs[1:15]`
spelling before the `regs[0:15]` fix below was found, not re-taken since the fix measured slightly
better in isolation (not the sum of the three ablations above either way, since yosys shares logic
across edits once more than one is present, the same non-additivity ADR-0210 recorded for its own
paired cuts). All three together (shipping `regs[0:15]` spelling), clock gating mirrored in
`nano/synth_script.sh`: **60,626.9 µm²**, against the brief's own 61,868 µm² estimate for the same
combination, and confirming `nano/area_report.py` accepts the `sky130_fd_sc_hd__dlclkp_1` gating
cell with no code change. Clock gating alone (no RTL edits), for scale: 63,948.8 µm², −6,924.2
against baseline -- a different instrument from ADR-0213's flow figure (−12,596 µm² on the 4×2 TT
run) and never merged with it, the same rule that keeps `make fit`, `make soc-timing` and the TT
flow apart.

`NANO_MAX_UM2`: 72,700 → **62,300** (60,626.9 µm² measured, a 2.76% buffer, in line with the ~2.5%
this ratchet's last several steps have carried).

## Consequences

- `make nano-area` now measures the netlist shape the flow builds (clock-gated), closing the gap
  ADR-0213 opened between the local instrument and the flow.
- The three RTL edits are functionally inert: no cycle count, no stall reason and no retire changes,
  confirmed by unchanged `make nano-test`/`make nano-littlecpu-test`/`make nano-qspi-loop-test`
  retire counts and all 76 generated riscv-formal checks -- `cover` included -- passing against an
  empty `EXPECTED_FAIL` on the shipping `regs[0:15]` spelling.
- A narrower array declaration is not free even when every access is guarded: what matters to a
  BTOR-backed formal tool is the address a read or write PORT carries, not which branch of an
  outer mux consumes the result. Re-check this the next time a "this index is never really used"
  guard is paired with narrowing the thing it indexes.
- Whether Tier 1 buys enough margin for the 4×2 to finish detailed routing is a flow-run question,
  not a local-instrument one; `docs/ideas/nano-on-a-4x2-the-tile-is-wiring.md` names the run this
  ADR was written alongside, and its result is not this ADR's to state.
- The gate-level simulation Tier 0 named as owed before any tapeout is unaffected and still owed;
  nothing here touches `make nano-tt-test` or the pin-level harnesses beyond re-confirming them
  green.
