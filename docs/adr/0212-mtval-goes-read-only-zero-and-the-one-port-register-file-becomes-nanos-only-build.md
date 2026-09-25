# ADR-0212: mtval goes read-only zero, and the one-port register file becomes nano's only build

**Status:** Accepted · 2026-09-25

## Context

The owner's direction (2026-09-25): keep cutting toward a 4×2 Tiny Tapeout tile, using
only spec-legal cuts. The full chip (two register-file read ports, all four CSR groups
at full state) fits a 6×2 but not a 4×2. Three cuts were scoped, each measured alone on
main before landing: `mtval` read-only zero (−2,152 µm² alone), the one-read-port
register file (−2,053 µm² alone, previously measured and declined for its cycle cost on
ADR-0195's zero-wait-state harness), and cheaper 64-bit counters. `mcycle`/`minstret`
being read-only zero (−7,360 µm²) and `mscratch` being read-only zero (−1,341 µm²) are
both **not legal** — the privileged spec requires both to be implemented and writable —
and stay out of scope.

## mtval: read-only zero

The privileged spec: "The `mtval` register must always be implemented. If the hardware
platform specifies that no exceptions set `mtval` to a nonzero value, then `mtval` is
read-only zero." Nano's platform now makes exactly that declaration. The register and
its per-cause `trap_tval_value` computation are deleted from `nano/nano.v`; `CSR_MTVAL`
stays a real case arm of the read mux (reading a constant `32'b0`, keeping the access
legal rather than falling to the illegal-instruction default) and falls out of the
write-case entirely, so a write is legal and silently discarded — the WARL shape the
spec allows for a field with no stored bits.

`nano/formal/traps.sv`'s mtval assertion is re-derived rather than deleted: it no longer
compares `dbg_mtval` against a shadow model's expected fault address, only against the
constant `dbg_mtval == 32'b0`, asserted on every retirement rather than only a trapping
one. `nano/formal/traps-tval-probe.py`'s two mutations are rewritten to match — the old
mutations named `trap_tval_value` text that no longer exists; the new ones make the
tie-off report the trap's own cause value or `rs1`'s value instead of the constant zero,
each a real nonzero value the solver finds within the check's depth. Both still fail,
and the shipping core still passes, so `make -C nano/formal components_traps` proves the
new rule with a demonstrated red direction rather than an assertion nobody has watched
fail.

**`make nano-area`: 74,574.0 → 72,534.6 µm², −2,039.4 µm² (measured on top of the
one-port cut below in the same tree; the isolated ablation the ticket was scoped
against read −2,152 µm² alone).** `make nano-timing` (flops recipe, pre-layout ABC
estimate, no ratchet): 78,197.50 → 76,728.59 µm² / 13,174.76 → 12,984.98 ps, −189.78 ps
(−1.44%).

## The one-read-port register file: measured again, and shipped unconditional

`NANO_ONE_PORT_RF` (ADR-0195) already existed as an off-by-default build option: rs1 and
rs2 are read from `regs[]` one address at a time, over two extra FSM states
(`fetch_rs1`, `fetch_rs2`, the second skipped when the instruction doesn't read rs2),
rather than through a second read port. ADR-0195 measured and declined it on a
**zero-wait-state** memory harness (`make nano-dhrystone`/`make nano-coremark`): +25.6%
Dhrystone cycles, +23.3% CoreMark cycles, for a 1.7%-of-synthesis-area win that also made
routing worse on the real flow. That harness has no QSPI front end, and one has existed
since (ADR-0202); this ticket re-measures the same lever on the pin-level QSPI harness
(`make nano-qspi-pins-dhrystone`/`make nano-qspi-pins-coremark`, `sck`/`cs_n`/`sio`
against the flash and PSRAM behavioural models — the harness closest to what the tile
actually runs on) with `NANO_DHRY_RUNS=5 NANO_DHRY_CYCLES=100000000` and
`NANO_COREMARK_ITERATIONS=1 NANO_COREMARK_CYCLES=40000000`, a fresh two-port baseline
taken on this same tree alongside it:

| Metric | Two ports (baseline, this tree) | One port (this tree) | Change |
|---|---|---|---|
| Dhrystone cycles (5 runs) | 123,785 | 127,035 | **+2.63%** |
| Dhrystone cycles/run | 24,757 | 25,407 | +2.63% |
| Dhrystone retires | 13,944 | 13,944 | unchanged |
| CoreMark cycles (1 iteration) | 24,187,279 | 25,276,725 | **+4.50%** |
| CoreMark retires | 777,270 | 777,270 | unchanged |

Retire counts are identical both sides on both benchmarks, so every added cycle is the
one-port read schedule and nothing else moved — the same signature ADR-0195 recorded,
at a much smaller relative price now that QSPI flash/PSRAM wait states dominate the
cycle count and the extra register-fetch cycle is a small addition on top of them,
rather than most of the budget.

**Decision: the option is made unconditional rather than left as a flipped default.**
`nano/nano.v`'s `` `ifdef NANO_ONE_PORT_RF``/`` `else``/`` `endif`` arms around the
`` `RF_RS1``/`` `RF_RS2`` macros, the `fetch_rs1`/`fetch_rs2` FSM states and the
decode-to-execute transition all collapse to the one-port text; there is no longer a
two-port code path in the file. A flipped default (defining `NANO_ONE_PORT_RF`
everywhere except a lone build that still doesn't) would have left an unstated question
open — `nano/formal`'s harnesses build `nano.v` directly with no register-file define at
all, so whichever spelling reads as "no define" is silently what every formal proof
covers, and leaving that pointed at a two-port implementation nobody ships would be
proving the wrong core. One implementation removes the question rather than answering
it carefully in a comment. `nano/tb.mk`'s `nano-oneport-{sim,test,startup-test}` targets
and their `.vvp`/`.cc` build rules are deleted (redundant with the plain targets, which
are the one-port build now); `Makefile`'s `test` prerequisite list drops
`nano-oneport-test nano-oneport-startup-test`; `.github/workflows/nano-tt-area-selfhosted.yml`
drops its `ports` (two/one) input and the `VERILOG_DEFINES` branch that read it, since
there is only one register-file shape to dispatch now.

**`make nano-area` (on top of the mtval cut): 72,534.6 → 70,873.0 µm², −1,661.6 µm²**
(the isolated ablation read −2,053 µm² alone; the two cuts overlap some synthesised
logic, so the stacked number is smaller than either sum). `make nano-timing` (flops
recipe, pre-layout, no ratchet): 76,728.59 → 73,076.34 µm² / 12,984.98 → 19,802.79 ps,
**+6,817.81 ps (+52.5%)**. This instrument is an unplaced, unrouted ABC estimate over
the whole netlist as one combinational cone (`nano/nano.mk`'s own header: "for ranking
two RTL versions against each other; not a gate, and never merged with a
`nano-tt-area-selfhosted` flow figure") and nano carries no Fmax ratchet yet ("no clock
is placed for nano yet"), so this number is reported for the record rather than treated
as a regression; ADR-0210 already found this same instrument's delay estimate move the
wrong way on a cut that measured better on every real, gated axis (the area ratchet, the
pin-level cycle counts), which is the standing reason not to read it as a decision
input for this class of edit.

**F and G move.** Every instruction now passes through the unconditional `fetch_rs1`
state (and `fetch_rs2` when it reads rs2), so retire latency grows by at least one cycle
across the board — not only in the worst case. `make -C nano/formal remeasure-fg`
against the new shipping core: **F = 13 (was 12), G = 11 (was 10)**. `nano/formal/checks.cfg`'s
`#derive` lines are updated to match, and every `[depth]` row that was hand-set to its
old F/G-derived floor plus a one-cycle margin is raised the same way against the new
floor (`insn`/`ill`/`csrw`: 33→36; `reg`/`pc_fwd`/`pc_bwd`/`causal`/`causal_mem`:
`13 24`→`14 26`; `liveness`/`unique`: `1 13 24`→`1 14 26`; `hang`: `1 14`→`1 15`).
`nano/formal/dmemcheck.sby`/`imemcheck.sby` and their `_cover` siblings carry the same
F+G+2/F+2 formulas as hand-set `depth` values (`check-memcheck-depth.py` grades them
against `checks.cfg`'s derived F/G rather than deriving its own): 24→26 and 15→16
respectively. `cover` and `csrc_upcnt`, whose floors are measured constants rather than
F/G formulas, are bumped conservatively (15→17 each) to keep the same margin over the
now-longer minimum instruction latency; both ran clean at the new depth.

A hardcoded cycle-cost regression outside `nano/formal` also needed re-basing:
`nano/bench/run_qspi_loop_buffer_test.sh`'s `WINDOW_BOUND` asserted a resident,
loop-buffer-hit loop paid no more than 11 cycles/iteration (a measured 10 plus one
cycle of slack). The one-port register file's extra fetch state raises every
instruction's minimum cost, including a hot loop's, so the measured figure is now
exactly 13 cycles/iteration (+2,600 cycles over 200 extra reps, both the `tagged` and
`cam` loop-buffer builds); the bound is raised to 14 (same one-cycle-of-slack
convention) and `nano/bench/run_qspi_loop_buffer_probe.sh`'s four forced-red mutations
— which compare the test's own stderr text — pass again once the bound stops masking
their real failure with an unrelated threshold trip.

## Cheaper counters: no reduction found

The counters stay 64-bit, writable and exact. What was tried, on top of both cuts
above:

- **A narrower low-half incrementer with an explicit carry into the high half** —
  `{mcycle_hi + {31'b0, &mcycle_lo}, mcycle_lo + 32'd1}` in place of `mcycle + 64'd1`,
  the same restructuring applied to `minstret`'s conditional increment. Functionally
  identical arithmetic. Measured: `make nano-area` read 70,890.5 µm² against the
  70,873.0 µm² baseline above — **+17.5 µm², trivially worse**. yosys/ABC already
  produce the carry-chain structure this restates from the plain 64-bit `+1`
  expression; rewriting it by hand bought nothing and cost a hair, the same
  "redundant source text is not redundant hardware, and restating the same arithmetic
  is a null" finding this repository has made before (`rtl/memory.v`'s declined flat
  arms, `rtl/executor.v`'s counter width). Reverted.
- **One shared incrementer between `mcycle` and `minstret`** was not built: `mcycle`
  increments every cycle it isn't written, and `minstret` increments on every retiring
  instruction, which is most cycles too — both routinely need `+1` on the *same* clock
  edge, so there is no cycle to time-multiplex one adder across without either
  stalling something this repo has no stall budget for or breaking the "count every
  cycle / every retired instruction exactly" requirement. No viable design was found.
- **The read path** (`csr_rdata`'s `CSR_MCYCLE`/`CSR_MCYCLEH`/`CSR_MINSTRET`/
  `CSR_MINSTRETH` arms) already reads the plain wire slices `mcycle_lo`/`mcycle_hi`/
  `minstret_lo`/`minstret_hi` with no logic between the register and the mux input —
  nothing there to cut.
- **The write muxes** (write-a-new-value beats free-run-increment beats hold) are the
  minimum structure that keeps a counter simultaneously writable (a spec requirement)
  and exactly self-incrementing (`csrc_upcnt`'s requirement); no narrower mux shape
  was found that keeps both.

The 128 flops (64 bits each for `mcycle` and `minstret`) are the whole ablated cost
this ticket could not reduce further; `csrc_upcnt_mcycle_ch0`/`csrc_upcnt_minstret_ch0`
(exact-increment) and the counters' `csrw_*` checks are two of the 76 generated checks
graded below, unchanged in behavior and still green at the new depth.

## NANO_MAX_UM2

`nano/nano.mk`: 76,500 → **72,700** (70,873.0 µm² measured, the same ~2.5% buffer over
measurement the ratchet has carried through its last several steps).

## Verification

- `make nano-test`: 8/8 on both the cxxrtl and iverilog legs, agreeing program by
  program, identical retire counts to before this ticket (`alu.S` 66, `branch.S` 33,
  `compressed.S` 75, `csrimm.S` 19, `divide.S` 52, `loadstore.S` 44, `meip.S` 42,
  `mul.S` 52).
- `make nano-qspi-pins-test`: 8/8 on both legs, agreeing program by program, same
  retire counts; its own forced-red prerequisites (`nano-qspi-pins-probe`,
  `nano-qspi-derived-clock-probe`) both pass.
- `make nano-qspi-loop-test`: all four sections pass (branch-free parity, tagged and
  CAM resident-loop cost under the re-derived bound, the straddling-load shape), and
  all four of `run_qspi_loop_buffer_probe.sh`'s forced-red mutations are caught again.
- `make nano-littlecpu-test`: 43/43 (31 excluded), matching `LITTLECPU_EXPECTED_FAIL`
  exactly, unchanged retire counts (`zicsr.S` 241 included).
- `make nano-startup-test`: PASS, 19 retires, unchanged.
- `make -C nano/formal components_traps`: passes; `traps-region-probe.py` (unchanged
  mutations, re-anchored on the cause-only `trap_cause_value` block after `mtval`'s
  removal) and `traps-tval-probe.py` (rewritten mutations, see above) both still red
  for their own reason, the shipping core passing first as the required control.
- `make -C nano/formal check`: all **76** generated riscv-formal checks pass against
  an empty `EXPECTED_FAIL`, matching `EXPECTED_CHECKS` exactly, at the re-derived
  depths (F=13, G=11).
- `make -C nano/formal dmemcheck imemcheck`: both pass at their re-derived depths (26,
  16); their `_cover` siblings are tied to the same depths by
  `check-memcheck-depth.py` and were not independently re-run beyond that tie, the
  same standing they had before this ticket.
- `make test` and `make probe-gates`: green (littlecpu's own suite is untouched by
  this ticket and unaffected).

## Tiny Tapeout flow, 4×2

`gh workflow run nano-tt-area-selfhosted --ref <branch> -f tiles=4x2 -f synth_strategy="AREA 2" -f disallow_congestion=false -f stop_after_synthesis=false`
(the `ports` input no longer exists, per the decision above).

<!-- TT-FLOW-RESULT -->

## What was left alone

`mcycle`/`minstret` read-only zero (−7,360 µm² ablated) and `mscratch` read-only zero
(−1,341 µm² ablated) are both spec-illegal and out of scope, per the brief this ticket
was scoped against. `nano/formal/complete.sby`, `complete_cover.sby`, `ill_e.sby` and
`ill_e_cover.sby` carry hand-set depths (20, 100, 40, 100) with no documented F/G tie
in this tree; they were not re-derived or re-run beyond what `make test` already
exercises, since nothing in this ticket's engineering rules requires it and no test
here failed against them.
