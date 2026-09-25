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

- `make test`: <PASS/FAIL, gate list>
- `make cosim-suite`: <PASS/FAIL>
- `make -C formal remeasure-fg`: F=<N> G=<N> (unchanged/changed from B1's 6/6)
- `make -C formal components_decoder` / `components_executor` / `components_pcloop` /
  `components_traps`: <PASS/FAIL each, by k-induction>
- `make mutation-check`: <PASS/FAIL>
- `make dual-smoke`: <PASS/FAIL>
- `make lint`, `make elaborate-strict`: <PASS/FAIL>
- `test/zkt_isolation_test.py`: re-derived, not edited to pass — `STRUCT_PORTS` gains
  `fwd_rs1`/`fwd_rs2`, and the forward reachability check confirms `reg_rs1`/`reg_rs2`
  still reach `x_busy` only through `region_stall`/`divider_busy`: the new path through
  `out.rd_data`/`fwd_rs1_val`/`fwd_rs2_val` terminates at the same two gates (mirroring
  the divider's own selects, which were already exempt).
- The standing liveness probe (delete the rs2 write-through bypass, `reg_ch0` must go
  SAT): <status>.

## Measured

<make cycles table, Dhrystone/CoreMark cycles against main and against B1, F/G, fit,
placed SoC count, ECP5 Fmax, filled in from the actual runs>

## Kill check

Owner-directed: if Dhrystone after B2 is not below main's 1,613,644 cycles, stop before
B3. Measured: <N> cycles, <below/not below> main's floor.

## Decision

<SHIPPED / BLOCKED, with the numbers above as the record>
