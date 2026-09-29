# ADR-0218: nano Tier 3 -- the state machine loses four states

**Status:** Accepted · 2026-09-29

## Context

`docs/ideas/nano-on-a-4x2-the-tile-is-wiring.md`'s Tier 3, promoted to "do it regardless" because the goal
is silicon that is fast as well as small. A register-to-register instruction walked
`fetch_instr → ready_instr → decode_instr → fetch_rs1 → fetch_rs2 → execute_instr → reg_write`
(seven states, `check_pc` added for jumps and branches). Four of those states existed to move a value
into a register or to wait. Each removal below is its own commit, and each names the property that
makes it safe.

## Decision

`nano/nano.v`:

1. **`decode_instr` goes.** Tier 1 already made `rd`/`rs1`/`rs2` combinational functions of `instr`,
   which left the state doing nothing but waiting a cycle. `ready_instr` now enters `fetch_rs1`.
2. **`fetch_rs2` and `op_rs2` go.** `rf_raddr` selects rs1 in `fetch_rs1` and rs2 in every other
   state, so rs2 is read live for as long as the instruction is held. Safe because the register file
   has one writer and it fires only on the edge that ends the instruction: rs2 cannot change under a
   held store's `mem_wdata`. `components_memreq` proves `mem_wdata` stable for the whole request.
3. **`check_pc` and `pc_wdata` go.** With C every jump and branch target is 2-byte aligned (`jalr`
   clears bit 0, every offset is even), so the misaligned-target trap is unreachable and a jump writes
   its target straight into the next pc. The property is now asserted, not believed: `nano.v`'s
   `FORMAL` block asserts `!pc[0]` and `!mem_addr[0]`, `components_memreq` proves them, and
   `memreq-probe.py` gained a second forced-red case (a `jalr` that keeps its bit 0) that must fail
   at one of those two assertion lines, not merely fail. `nano/tb/nano_cxxrtl.cc` loses its "misaligned
   jump/branch target" classification, since nothing can reach it.
4. **`reg_write` and `reg_wdata` go.** The register file is written on the edge that ends
   `execute_instr` (ALU, `lui`, `auipc`, `jal`, `jalr`, CSR read) or `finish_load`, from a
   combinational `wb_data`/`load_data`. `execute_instr` now reads rs2, runs the ALU and writes the
   register in one cycle. Traps and `mret` do not write, by the same `!take_trap` term that gates
   `instret`. RVFI's `rvfi_rd_wdata` still reads `regs[rd]` at fetch entry, one cycle after the write
   as before, so the retire block reports the same values at the same time.
5. **`next_pc` and `mem_addr` become one register.** Only one is live at a time: an instruction ends
   with its successor's address in `mem_addr`, a load or store holds its own address there only while
   the bus request lives, and fetch no longer copies one into the other. `memreq`'s stability
   assertions grade exactly that: an address that moved mid-request would fail them, and the
   `finish_store` mutation in `memreq-probe.py` was re-anchored on the new spelling and still goes red.

`skip_reg_write` disappears with `check_pc` (it only ever selected `check_pc`'s exit).
`cpu_trap` stays: nothing reaches it now, and removing it is a separate question.

## Measurements

All local instruments. Area is `make nano-area` (clock-gated local synthesis) and **is local, not the
flow's**: Tier 1 measured -10k locally and about -2.5k in the flow, so treat these as an upper bound.
Every row was measured on its own commit in its own tree. Zero-wait is `make nano-dhrystone` (200 runs,
105,987 retires) and `make nano-coremark` (5 iterations, 3,810,704 retires); QSPI is the pin-level
harness, `NANO_DHRY_RUNS=5 NANO_DHRY_CYCLES=100000000 make nano-qspi-pins-dhrystone` (13,944 retires)
and `NANO_COREMARK_ITERATIONS=1 NANO_COREMARK_CYCLES=40000000 make nano-qspi-pins-coremark` (777,270
retires). Retire counts are identical in every row.

| after | local area um2 | zero-wait Dhrystone | zero-wait CoreMark | QSPI Dhrystone | QSPI CoreMark |
| --- | --- | --- | --- | --- | --- |
| main (7b111c6) | 58,790.1 | 629,527 | 24,943,488 | 124,672 | 24,518,338 |
| `decode_instr` | 58,776.4 | 535,108 | 21,151,667 | 122,309 | 23,759,951 |
| `fetch_rs2` | 57,559.0 | 499,700 | 19,496,420 | 121,422 | 23,428,892 |
| `check_pc` | 56,829.5 | 478,298 | 18,373,156 | 120,885 | 23,204,241 |
| `reg_write` | 55,869.8 | 415,887 | 15,696,013 | 119,329 | 22,668,799 |
| `mem_addr` merge | 54,536.1 | 415,887 | 15,696,013 | 119,329 | 22,668,799 |
| total | -4,254.0 (-7.2%) | -33.9% | -37.1% | -4.3% | -7.5% |

Zero-wait CPI on Dhrystone goes from 5.94 to 3.92 cycles per instruction: two cycles, not the ticket's
three, because the target path `fetch_instr → ready_instr → fetch_rs1 → execute_instr` is four states
and the average includes loads, stores and the fetch cost. The QSPI harness gains much less because most
core cycles already hid under the flash fetch, as the ticket said. The merge saves area and no cycles.
`NANO_MAX_UM2` steps 60,300 → 56,050, keeping the prior 1,509.9 um2 of headroom over the new local
measurement.

## F, G and every depth

`make -C nano/formal remeasure-fg`: **F 13 → 11, G 11 → 9** (both G trigger points agree). Every depth
that was hand-set to a floor plus one cycle of margin moves the same way: `insn`/`ill`/`csrw` 36 → 30;
`reg`/`pc_fwd`/`pc_bwd`/`causal`/`causal_mem` `14 26` → `12 22`; `liveness`/`unique` `1 14 26` → `1 12 22`;
`hang` 15 → 13; `cover` and `csrc_upcnt` 17 → 15 (measured constants, back to their pre-one-port
value, both run clean); `dmemcheck` and its cover 26 → 22 (F+G+2); `imemcheck` and its cover 16 → 14.
`complete` (20), `ill_e` (40) and `traps` (25) are not F/G formulas and stay: a shallower core reaches
every goal earlier, so those depths only gain slack, and `complete_cover` and its tie probe stay green.
`test/probe_gates.sh`'s nano depth fixture followed `hang`'s new value.

`nano/bench/run_qspi_loop_buffer_test.sh`'s `WINDOW_BOUND` returns to its pre-one-port shape, 9 cycles per
rep: a resident loop measures exactly 8 (1,600 cycles and 400 loop hits over 200 extra reps, both
loop-buffer shapes; it was 13), and 9 is the measured figure plus one cycle of slack. The old bound of 14 had been masking the probe: with the bound at 9, the probe's
`loop-hit-gated-on-xfer-active` mutation (+400 cycles, 2,000 against a bound of 1,800) is caught again,
and all four probe mutations stay red for their own text.
`meip.S`'s retire floor (42) does not depend on cycle counts and is unchanged; its forced-red probe passes.

## Checks

`make -C nano/formal check` passes all 76 generated checks with an empty `EXPECTED_FAIL`; `ill_e`,
`complete`, `complete_cover`, `dmemcheck`, `imemcheck` (with covers), `components_qspi`, `components_traps`
and `components_memreq` pass. **`ill_e_cover` fails, and it fails identically on main (7b111c6)**: its one
goal, an E-illegal load reported with `rvfi_rd_addr == 16` and `rvfi_trap`, is unreachable there too (a
trapping retirement reports `rd` as 0 since an earlier change). That is a pre-existing defect, not
introduced here, and is left for its own ticket.

The 4x2 flow run, including per-corner setup, is not part of this ADR: it runs on the branch after merge
review, and the longer `execute_instr` path (rs2 read, ALU and register write in one cycle) is what it
will grade.
