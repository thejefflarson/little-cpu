# 0213 — Stage B2 forwards the X/M register into X

Status: Proposed. 2026-09-25. Ships as a PR against
`thejefflarson/jef-1056-stage-b1-split-decode-and-execute` (B1, #402), stacked on it the
way B1 stacked on `thejefflarson/fetch-refactor`. `make fit` and `make soc-timing` are
red on this whole stack by the owner's own decision — cells are trimmed after the
restructure finishes — and this ADR reports both anyway, against `main`'s numbers, since
the owner's area pass reads from these reports rather than from a green gate.

## What this is

B1 (ADR-0208) split decode into D and X and, deliberately, added no forwarding path at
all: a RAW hazard against anything still in flight — the instruction currently in `out`
(D/X's own register) or already registered into `executor_out` (X/M) — simply stalled.
That cost measured 12.0% of the suite's cycles and 9.5% of Dhrystone's DMIPS/MHz on B1's
own tree. B2 is the recovery this stage was always going to need: give X a forwarding
mux, and let the regfile's own existing write-through bypass (commitment 6) do the rest.

## The mechanism

**Only one of the two matches needs a mux.** D's scoreboard already computed two
matches: `dx_match` (this instruction's rs1/rs2 equals `out.rd`, the instruction
currently entering X this very cycle) and `ex_match` (equals `executor_out.rd`, the
instruction X finished one cycle ago). Tracing both through the pipeline by the cycle:

- A `dx_match` producer is processed by X the cycle this instruction is decoded, and its
  result registers into `executor_out` exactly one cycle later — precisely the cycle
  this instruction reaches X and needs it. That result is nowhere else yet: the
  regfile's own read for this instruction was presented the cycle it was decoded and
  answers one cycle later from `regs_a`/`regs_b`, which the producer has not written.
  This is the case B2 forwards: X selects `executor_out.rd_data` over the regfile's
  answer, IF the producer will actually have a same-cycle result (`out_has_result`,
  mirroring `executor.v`'s own `in_has_result`: every op except a load, an AMO, `lr.w`,
  `sc.w`, or a div/rem just starting).
- An `ex_match` producer already registered its result into `executor_out` at the SAME
  cycle this instruction is decoded, one cycle before this instruction reaches X. By the
  time it does, that result has moved one stage further — into `accessor_out` — and
  `writeback`'s combinational `wen`/`waddr`/`wdata` land on the SAME cycle this
  instruction's regfile read (presented at decode, held one cycle) resolves via the
  EXISTING write-through bypass. No mux needed; the pipeline was already delivering the
  right answer one cycle late relative to what the pre-B2 code assumed, and B1's
  stall-everything code was paying for a hazard that had already resolved itself.

So: `fwd_rs1`/`fwd_rs2`, two bits precomputed in D from `dx_match_rs1`/`dx_match_rs2` and
`out_has_result` (register NUMBERS and class flags, never a register VALUE), ride the D/X
register into X and select `executor_out.rd_data` over `reg_rs1`/`reg_rs2` at every use
except `csr_arg` (a CSR access's own rs1, which reads `reg_rs1` verbatim — see below).
`ex_match` keeps its existing role as a hazard SOURCE when the producer's own result is
not yet unpacked (a load, an AMO, `lr.w`, `sc.w`: `!executor_out.rd_ready`), and drops
out entirely — no stall, no mux — once the producer is ready, since the bypass reaches
it on its own.

**Load-use is what survives.** A `dx_match` against a load/AMO/`lr.w`/`sc.w`/div-just-
starting cannot forward (the value genuinely does not exist yet anywhere reachable next
cycle) and stalls; on the cycle after, it shows up as an `ex_match` still not unpacked
and stalls again; the cycle after THAT, the bypass has it. Two stall cycles for a true
load-use dependency, matching the brief's "load-use... becomes the only RAW stall."

**A CSR access's own rs1 never forwards.** `csr_arg` in X reads `reg_rs1` directly, not
a forwarded mux — `fwd_rs1` is gated `&& !instr_csr_access` in D, and
`rtl/decoder.v`'s `FORMAL` block asserts `out_is_csr_access ⇒ !out_fwd_rs1` directly.
This is provably unreachable via `dx_match` anyway (a CSR access serializes — commitment
5 — so it cannot even be decoded while `out.valid` is true), but the exclusion is
stated as a fact about the mux rather than relying on that argument holding forever.

## RVFI and the monitor

`rvfi_rs1_rdata`/`rvfi_rs2_rdata` report the forwarded value (`fwd_rs1_val`/
`fwd_rs2_val`), not the regfile's own answer — the monitor checks `rd_wdata` against
exactly those two fields, so reporting the unforwarded operand would make every
forwarded retire self-contradictory. `test/cosim.cc`, which reads `regs_a` directly and
never an `rvfi_*` signal, is the oracle that would catch a consistently wrong forward
even if RVFI's own self-check somehow missed it.

## Stall-reason taxonomy: unchanged at the top, refined underneath

The seven top-level reasons (divider, atomic, hazard, serialize, fetch, bus, region) are
untouched — B2 changes what `hazard` is built from, not the OR it feeds. `hazard`'s own
three-way split (hzA/hzB/hzC, already scaffolded in `test/stall_report.py` and
`test/cxxrtl.cc` since B1, folded entirely into hzC there) now means what B2's mechanism
actually produces: hzA is a `dx_match` without a forward select (the producer will not
be ready next cycle — a load/AMO/LR/SC, a div/rem just starting, or a CSR access's own
excluded rs1); hzB is an `ex_match` whose producer is not yet unpacked; hzC — a ready
`ex_match` forwarding has no path to — reads zero, because B2 gives it none: it needs
none. `rtl/decoder.v` exposes `hazard_rs1_dx`/`hazard_rs1_ex`/`hazard_rs2_dx`/
`hazard_rs2_ex` so `test/cxxrtl.cc` can classify without re-deriving the split.

## Verification

- `make test`: PASS, every gate green (cxxrtl suite 75/75, unit benches, probe-gates,
  window-test, imem-share-test, board-elaborate, mutation-probe, dual-build, comment
  density and every repo-scanning `*-test` target), `STALL_REPORT=1`'s cycle-accounting
  identity included. Getting here needed two fixes, both landed on this branch:
  `fwd_rs1`/`fwd_rs2` lacked the `rs1 != 0`/`rs2 != 0` gate `hazard_rs1_dx`/
  `hazard_rs2_dx` already carry, so a producer that (attempted to) write x0 forwarded its
  stale result into any later instruction whose rs1/rs2 field decoded as 0 — caught by 22
  of the 75 suite programs going `MONITOR-ERROR 105` (mismatch in `rd_wdata`); and one
  `test/probe_gates.sh` fixture (the region-wait anti-vacuity mutation) still matched
  `reg_rs1` where the source now reads `fwd_rs1_val`.
- `make cosim-suite`: PASS, 69/75 agree, divergence list matches
  `test/COSIM_EXPECTED_FAIL` exactly (the same six as B1: three `INCONCLUSIVE SAIL-LIMIT`
  timer programs and three UART/SPI-flash `DISAGREE`s the model has no device for).
- `make -C formal remeasure-fg`: F=6 G=6, unchanged from B1.
- `make -C formal components_decoder`: PASS by k-induction. `components_executor`: PASS
  by k-induction. Neither's zkt probe needed re-deriving beyond what
  `test/zkt_isolation_test.py` already covers below. `components_pcloop` and
  `components_traps` were not re-run: B2 touches no file either proof's module set
  reads (`rtl/littlecpu.v`, `formal/traps.sv`), so neither proof's environment changed.
- `make mutation-check`: PASS, 11/11 mutations caught by exactly their paired detectors
  — after adding one pairing. B2's forwarding lets `mtimermask.S`'s six back-to-back
  independent `csrr mscratch` reads retire without each waiting for the pipe to drain,
  which shifts when its 30-cycle-armed timer interrupt lands relative to the test's
  sample of `irq_count`, so `serialize-drops-csr-mret` now reads `FAIL 17` there where
  the suite used to stay green under it. `decoder_tb` still catches the mutation
  directly; this is a new true detector, declared in `test/MUTATION_DETECTORS` and
  `docs/manifests/mutation-detectors.md` rather than silenced.
- `make dual-smoke`: OK — two harts counted 32, one hart counted 16.
- `make lint`, `make elaborate-strict`: both clean.
- `test/zkt_isolation_test.py`: re-derived, not edited to pass — PASS. `STRUCT_PORTS`
  gains `fwd_rs1`/`fwd_rs2`, and the forward reachability check confirms `reg_rs1`/
  `reg_rs2` still reach `x_busy` only through `region_stall`/`divider_busy` on the
  elaborated netlist: the new path through `out.rd_data`/`fwd_rs1_val`/`fwd_rs2_val`
  terminates at the same two gates (mirroring the divider's own selects, which were
  already exempt).
- The standing liveness probe: `reg_ch0` PASSes on the shipping `rtl/regfile.v` (56s,
  k=22 both properties UNSAT); deleting the rs2 write-through bypass alone
  (`reg_rs2 = (held_rs2 == 5'd0) ? 32'b0 : read_b;`, no `wen`/`waddr` term) makes it go
  SAT at k=22 (34s) — the probe still fires, and the deletion was reverted before commit.

## Measured

`make cycles`, full suite (75 programs):

| | cycles | retired | CPI | hazard | hzA | hzB | hzC |
|---|---|---|---|---|---|---|---|
| B1 (ADR-0208) | 44,620 | — | 2.06 | 36.2% | — | — | — |
| B2 | 30,893 | 21,680 | 1.42 | 2,286 (7.4%) | 1,306 | 980 | 0 |

hzC reads zero everywhere, including the CSR register-form read B1's own hzCcsr carve-out
existed for (now folded into hzA, since a CSR access's own rs1 never gets a forward
select). fetch is now the largest single stalled reason (3,026 cycles, 9.8%), ahead of
hazard.

Dhrystone (2,000 runs) and CoreMark (100 iterations, 16 KB simulated ROM):

| | Dhrystone cycles | DMIPS/MHz | CoreMark cycles | CoreMark/MHz |
|---|---|---|---|---|
| main (pre-B1) | 1,613,644 | — | — | 2.155 |
| B1 (ADR-0208) | 1,698,022 | 0.670 | 53,326,372 | — |
| B2 | 1,394,022 | 0.816 | 41,424,774 | 2.414 |

B2 against B1: Dhrystone −17.9% cycles (+21.8% DMIPS/MHz), CoreMark −22.3% cycles. B2
against main: Dhrystone −13.6% cycles, CoreMark/MHz +12.0%. B2 beats not only B1's
stall-only D/X split but also the pre-B1 fused-decoder's guessed-pair scheme on both
benchmarks, consistent with forwarding removing a real dependency stall that the guess
mechanism could only sometimes avoid by prediction.

F = 6, G = 6 (unchanged from B1, ADR-0208).

Area (reported, not gated — `make fit` and `make soc-timing` are expected red on this
whole stack by the owner's own decision, ADR-0207):

| | `make fit` (ICESTORM_LC) | placed SoC demand |
|---|---|---|
| main | — | 4,920 / 5,280 |
| B1 | 4,658 | ≥5,418 |
| B2 | 4,850 | 5,550 / 5,280 (105%, does not place) |

`make soc-timing` fails to produce a bitstream at all: nextpnr cannot legalise 5,550
`ICESTORM_LC` into the part's 5,280, so there is no Fmax number for up5k on this tree,
only the demand. `make ecp5-timing` (more headroom on that part) does place: **34.97 MHz**
against main's 35.70 MHz (−2.0%), with the reported critical path ending at
`riscv.executor.in_fwd_rs1_TRELLIS_FF_Q` — the new forwarding mux's own register — so the
mechanism this stage adds is visibly what the placer now spends the path on.

## Kill check

Owner-directed: if Dhrystone after B2 is not below main's 1,613,644 cycles, stop before
B3. Measured: **1,394,022 cycles, below main's floor** (and below B1's 1,698,022). The
kill check passes; B3 proceeds.

## Decision

**SHIPPED.** B2 recovers all of B1's regression and then some, on both benchmarks, at the
cost of area B1 already had the owner's standing permission to spend (ADR-0207) — the
placed SoC does not fit the part on this tree, same as B1, and Fmax on ECP5 moves inside
the noise the fetch loop's own churn band would produce. The two correctness bugs found
in getting here (the unguarded x0 forward, the stale probe fixture) are both fixed and
covered: 22 suite programs plus the co-sim and zkt-isolation oracles would have caught a
regression of the first kind again, and `make probe-gates` the second.
