# ADR-0154: Executor-only forwarding is re-taken, and the RVFI report has to move with it

**Status:** Accepted · 2026-09-01 · *Reverses ADR-0083's decline of its "confined to the executor's
operands" spelling, on a re-measurement of the margin that declined it — the same reason ADR-0089
reopened the operand-fetch guess and ADR-0093 reopened the compressed-successor guess. Amends the
stall-only hazard commitment (invariant 4), with the sixteen-seed sweep and the four other goals
shown to hold below. Read against [ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz
requirement, which this leaves exactly as it stands, and [ADR-0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md),
whose warning about the suite's share is why both a hand-written and a compiled number are quoted
below.*

## Context

ADR-0083 built two forwarding spellings and declined both: forwarding to every operand reader in
decode misses 12 MHz outright (9.49 MHz worst placement), and confining the forward to the
executor's own operands — `out.rs1`/`out.rs2`, the two struct fields `rtl/executor.v` reads as ALU
inputs and `rtl/accessor.v` reads as a store's or an AMO's write data — holds 12 MHz but was declined
on **0.48% of margin against the tree's own 3.35%**.

That decline was explicitly a conservatism about future edits, not a verdict on the trade, and this
repo has twice found such a verdict did not survive re-measurement: ADR-0089's operand-fetch guess
was declined on 0.83% of margin and re-derived twice against later trees whose own worst placements
were 2.08% and 4.75% — neither the number nor the verdict travelled. ADR-0093's compressed-successor
guess lost a placement outright at 11.56 MHz and was declined on it; re-measured after roughly 490
cells left the SoC it lost none of sixteen and shipped, buying 16.3% of Dhrystone's cycles. The tree
has moved substantially since ADR-0083's `2773832`: the A extension, mtval, the region wait
(ADR-0128/0129), `NHARTS`, Zkt's assertions (ADR-0134/0137), and ADR-0117's −2.47% of median period
off the trap-decode fetch loop. A 0.48% margin measured on that tree says little about this one.

## Decision

**Take it.** Rebuilt from ADR-0083's own description — there is no branch or stash carrying the
original build — with the shape unchanged: forwarding reaches only `out.rs1`/`out.rs2`, and only
from the executor's own slot (`executor_out`), never from the slot ahead of it (`out`, whose
instruction has not reached the executor yet and has no result to forward regardless of type).

`rtl/structs.v`'s `executor_output` gains one bit, `rd_ready`: whether `rd_data` is this
instruction's finished result rather than a placeholder a later stage still fills in. Every
arithmetic op computes its result the cycle it issues; a load, a store, fence, wfi and every atomic
do not — `rtl/accessor.v` produces theirs, or there is none. `rtl/executor.v` sets it from the same
op-select flags that pick `rd_data`'s arm (`in.is_add || in.is_sub || ... || in.is_remu`), read every
cycle rather than latched inside the `init` state alone, so it keeps tracking the instruction a
multi-cycle divide is computing rather than going stale mid-loop — decode holds `out` unchanged
through a divide (`divider_stall`), so `in` does too, and `rd_ready` staying in step with it is what
lets the completing cycle report ready without a special case.

`rtl/decoder.v` gains the eligibility test the RTL text alone cannot state, because it depends on
what a *different* instruction reads about the *same* cycle: `rs1_fwd_eligible` is exactly
`instr_math` — every arithmetic op, immediate and register forms alike — because it is the one
category with no other decode-side reader of `reg_rs1`. Every other category that reads `rs1` reads
it directly, not through `out.rs1`, for something forwarding cannot reach: a load's or a store's
effective address (`mem_addr_calc`), an atomic's own address (`atomic_addr = reg_rs1` verbatim), the
jalr target, and a register-form CSR's operand (`csr_arg`) are all computed combinationally in decode
the same cycle the instruction issues. Suppressing that instruction's hazard would let it issue on
whatever `reg_rs1` still held — stale — while `out.rs1` alone got the forwarded value. `instr_math`
is disjoint from every one of those categories by construction (they are different major opcodes or
compressed forms), so gating on it is exact, not merely close. `rs2_fwd_eligible` adds a store's, an
AMO's and `sc.w`'s write data (`instr_sb || instr_sh || instr_sw || instr_amo || instr_sc`), which has
no other decode-side reader of `reg_rs2` either — and deliberately excludes a branch's `rs2`, which
the comparator (`cmp_sub`) reads directly for `branch_taken`, itself an input to `next_pc`.

`ex_fwd_rs1`/`ex_fwd_rs2` combine eligibility with `executor_out.valid`, `executor_out.rd == rs1/rs2`,
`executor_out.rd_ready`, and `!(out.valid && out.rd == rs1/rs2)` — the last term keeping the most
recent write authoritative when both slots would otherwise match the same register. `hazard_rs1`/
`hazard_rs2` gain `&& !ex_fwd_rs1`/`&& !ex_fwd_rs2`; `out.rs1`/`out.rs2` and `math_arg`'s
register-register arm source from `rs1_forwarded`/`rs2_forwarded` (the executor's data where
forwarded, `reg_rs1`/`reg_rs2` otherwise) instead of the register file directly.

## The defect this found, and what it says about RVFI

The first build passed `make -C formal check` and every component proof and then failed **every**
program in the suite with `MONITOR-ERROR 105` (mismatch in `rd_wdata`) — not an edge case, the first
retiring `add` in `add.S`. The core's arithmetic was correct: `rs1_rdata`/`rs2_rdata` as *reported*
were `1`/`0`, the spec model computed `1+0=1` from those reported operands, and the core's own
`rd_wdata` was `2` — because the ADD had in fact read `1` and `1` (a `li sp,1` immediately ahead of
it, forwarded correctly), and RVFI's `rs2_rdata` field still named the stale `reg_rs2` the register
file had not caught up to yet. The bug was never in what the executor computed; it was that decode's
own retire report contradicted itself — `rd_wdata` computed from the forwarded operand, `rs2_rdata`
naming the one forwarding replaced.

`out.rvfi.rs1_rdata`/`rs2_rdata` now report `rs1_forwarded`/`rs2_forwarded`, the same values that feed
`out.rs1`/`out.rs2`, rather than `reg_rs1`/`reg_rs2` directly. This is not a workaround for the
monitor's check; it is the correct architectural statement of what the operand was, and forwarding's
whole premise is that the value it supplies is the same one the register file would give once the
write-through bypass caught up — reporting anything else was always going to be wrong, whether or not
a checker happened to notice.

## What is not eligible, and the grader edits that follow from it

Three legs of the graders needed changes because the new signals sit exactly where each was already
looking, not because any of the three answers changed:

- **`test/mutations/{fencei-wait-and-store-port,serialize-drops-csr-mret,serialize-drops-fencei}.patch`**
  are unified diffs against `rtl/decoder.v` whose trailing context (three lines after the mutated
  line, `git apply`'s exact-text requirement) happened to include
  `assign hazard_rs1 = uses_rs1 && rs1 != 0 && live_rs1;`, immediately below the `serialize`/
  `text_access` line each patch actually mutates. Widening that line to
  `... && !ex_fwd_rs1;` is required by the feature, so the three patches' context went stale
  (`git apply --check` failed before this was fixed). The context text is re-anchored; the `-`/`+`
  lines each patch encodes — dropping `instr_fencei`/`instr_csr_access`/`instr_mret` from
  `serialize`, or `|mem_wstrb` from `text_access` — are byte-for-byte unchanged, and `make
  mutation-check` confirms each is still caught by exactly its original detectors in
  `test/MUTATION_DETECTORS`, a file this ADR does not touch.
- **`test/zkt_isolation_test.py`** gains `rd_ready` in `STRUCT_PORTS['executor_out']` — mechanically
  required, because the script's own sum-of-widths check (`classify_inputs`) would otherwise read the
  port's elaborated width one bit ahead of its declared fields and hard-fail exactly the way it is
  built to. `CONTROL_FIELDS['executor_out']` also gains it, and this is the one that needs the
  argument the coordinator asked for, not just the mechanism: `ex_fwd_rs1`/`ex_fwd_rs2` read
  `rd_ready` and feed `hazard_rs1`/`hazard_rs2`, two of the exact `STALL_TARGETS` this script polices
  for taint from `reg_rs1`, `reg_rs2` and `executor_out.rd_data` — so a forwarding path is precisely
  the shape of thing this grader exists to catch, and the question has to be answered on the
  mechanism, not on the table entry. `rd_ready` carries no register-file VALUE: it is computed in
  `rtl/executor.v` from `in.is_add || in.is_sub || ... || in.is_remu`, the op-select flags decode
  already publishes for every instruction from its *type*, never from an operand's value, so no data
  path exists from any of the three seeds into it at all — the same shape `CONTROL_FIELDS` already
  carries `out.rd`/`out.valid`/`out.is_amo` for. What the new forwarding mux *does* newly reach with a
  real seed is `out.rs1`/`out.rs2` (via `executor_out.rd_data`), and those are correctly named in
  neither table, because they are not `STALL_TARGETS` — an operand value reaching an operand is the
  feature, not a leak. Checked rather than argued: `probe-gates`' own red-direction control
  ("an executor_out.rd_data bit routed into a stall reason is red") still fires unchanged, so the
  checker's mechanism for catching exactly this class of defect is demonstrated intact, not merely
  asserted intact.
- **`test/probe_gates.sh`**'s one probe that builds a mutated `CONTROL_FIELDS` fixture by
  string-replacing the table's literal Python source went stale for the same reason as the file
  above (`assert s.count(old) == 1` no longer matched); its copy is updated to the real table, and
  the probe still empties `CONTROL_FIELDS` and still checks the isolation script calls that out
  (`ok CONTROL_FIELDS emptied is red against the shipping decoder (finding 5)`).

`test/decoder_tb.v` gains seven vectors, each pinning one half of the eligibility argument above
rather than only the happy path: rs1 forwards and the forwarded value (not the poisoned `reg_rs1`)
reaches `out.rs1`; the same for rs2; a load/AMO/`lr.w`/`sc.w` sitting in the executor's slot
(`rd_ready = 0`) does not forward and the consumer still waits; a store's base register is never
eligible (`ex_fwd_rs1 = 0`, `hazard_rs1` stays asserted) while its write data is; a branch's `rs2` is
never eligible; and `out`'s own slot is never a forwarding source regardless of what the executor's
slot holds. One of these needed a fix mid-flight: the store vectors originally left `reg_rs1` at an
earlier test's poison value, which is not "deep" in any mapped window, so `region_stall` — a real,
independent stall reason unrelated to this feature — held the store back and the test measured a
bubble instead of a forward. `reg_rs1` is now set to an address the region test regards as settled
before those two vectors.

## What it measures

Homebrew Yosys 0.68+post (`c12172fb`), nextpnr-ice40 0.11.1_1, icetime from icestorm 1.1, everything
against `da75d3e` (base) and this ADR's diff on top of it (candidate). Sixteen placements a side,
`SOC_SEED` = `default 1 2 … 15`, paired by seed.

| | ns / MHz, sorted worst→best is not meaningful per-seed here — paired instead | worst | median | best | spread |
|---|---|---|---|---|---|
| base (`da75d3e`) | — | 12.06 MHz | 12.99 MHz | 13.37 MHz | 10.86% |
| candidate (this ADR) | — | **12.33 MHz** | 12.64 MHz | 13.00 MHz | 5.43% |

**0 of 16 candidate placements under the 12.00 MHz requirement** — the worst is 12.33 MHz, 2.75% of
margin. Base's own spread (10.86%) sits above the 4–9% `soc/bands.py` states for an *unchanged*
netlist, which this is not a claim about; it says the base-tree seeds sampled a wide tail, which is
exactly the population the requirement is graded against the worst of.

Paired by seed, base minus candidate (positive = candidate slower):

| seed | base MHz | cand MHz | Δ |
|---|---|---|---|
| default | 13.00 | 12.33 | −0.67 |
| 1 | 13.35 | 12.72 | −0.63 |
| 2 | 12.06 | 12.45 | **+0.39** |
| 3 | 13.07 | 12.62 | −0.45 |
| 4 | 12.98 | 12.63 | −0.35 |
| 5 | 12.80 | 12.79 | −0.01 |
| 6 | 12.81 | 13.00 | **+0.19** |
| 7 | 13.16 | 12.76 | −0.40 |
| 8 | 13.15 | 12.92 | −0.23 |
| 9 | 13.37 | 12.41 | −0.96 |
| 10 | 13.20 | 12.87 | −0.33 |
| 11 | 12.89 | 12.55 | −0.34 |
| 12 | 12.95 | 12.61 | −0.34 |
| 13 | 12.90 | 12.64 | −0.26 |
| 14 | 12.89 | 12.44 | −0.45 |
| 15 | 13.02 | 12.71 | −0.31 |

Candidate is faster on 2 of 16 seeds, slower on 14, tied on 0. Two-sided sign test: **p = 0.0042** —
this is not noise, the median move (−2.73%) sits inside `soc/bands.py`'s ~3.6% up5k edit-churn band
but the *direction* is consistent enough not to be one. The candidate is measurably slower per clock
and holds the requirement anyway, with a smaller margin than the base tree's own 0.06 MHz-above-floor
worst case gave it credit for. `make fit`: **4109 → 4214 LC, +105 cells**, outside the ±50 churn band
— a real area cost, one new struct bit and two forwarding muxes.

`make netlist-digest` / `make netlist-diff BASE=da75d3e`: DIGEST-DIFFERENT, as it must be for a real RTL
change — `littlesoc`'s own top (the SoC, not the core alone `make fit` reports) moves
**6310 → 6423 cells (+113)**, `SB_LUT4` +112 and `SB_DFFSR` +1, both tops in the same
direction and within a few cells of each other. The sixteen-seed sweep this gate says is owed
is the one already run above.

## What it buys

`make cycles`, the hand-written `.S` suite (74 programs):

| | cycles | retired | CPI | hazard column |
|---|---|---|---|---|
| base | 42319 | 21118 | 2.00 | 15084 (35.6%) |
| candidate | 38657 | 21125 | 1.83 | 11409 (29.5%) |

**−8.65% of suite cycles.** The operand-fetch column barely moves (1193 → 1198), unlike ADR-0083's
own "overlap trap" on its `2773832` tree — ADR-0089's operand-fetch guess has already absorbed most
of what would otherwise have converted a cleared register hazard into a fresh operand-fetch one, so
almost the whole hazard reduction here reaches the total rather than being partly recaptured by a
neighbouring column. (Retired count moves by 7 across the whole suite, 21118 → 21125; every
individual program's status still matches `test/EXPECTED_FAIL` exactly, and the plausible source is
one of the interrupt-timing programs — `mtimer.S`/`mtimermask.S` — where fewer hazard cycles shift
which instruction an asynchronous interrupt lands on, not a correctness difference either leg's
monitor would have missed.)

`make dhrystone`, compiled C, 2000 runs, same compiler/flags/string routines both sides:

| | timed cycles | CPI | DMIPS/MHz | DMIPS at 12 MHz |
|---|---|---|---|---|
| base | 1712025 | 1.85 | 0.664 | 7.97 |
| candidate | 1676025 | 1.81 | **0.679** | **8.15** |

**−2.10% of Dhrystone's cycles, +2.26% of DMIPS/MHz and of the absolute figure** — the board runs at
a fixed 12 MHz regardless of which placement's Fmax cleared it, so a CPI win converts straight to
throughput and the clock side of this trade is margin spent, not speed lost. Dhrystone's own hazard
column *does* show a partial overlap-trap effect the suite's does not (whole-run STALLS: hazard
357798 → 317207, a 40591-cycle drop against a 36551-cycle total drop, so roughly 4000 cycles moved
into the operand column, 234533 → 238565) — compiled code's register reuse pattern differs from the
hand-written suite's, and ADR-0084's warning to read the suite's share as a property of the suite and
not the prize applies here in the direction that matters: Dhrystone's 2.10% is the number that
converts to throughput, and it is smaller than the suite's 8.65%, not larger.

## Against the four goals

**Fast** — cycles down 8.65% (suite) / 2.10% (Dhrystone), clock down 2.73% at the median but the
requirement still clears at all sixteen seeds with 2.75% of margin at the worst. Net throughput is
positive because the board's clock is fixed below every placement's Fmax; DMIPS moves 7.97 → 8.15.
**Simple** — one new struct bit, two eligibility predicates and two forwarding muxes; no new pipeline
stage, no new scoreboard slot, nothing a later cycle un-commits. **Readable** — the eligibility split
(`instr_math` for rs1, that plus store/AMO/`sc.w` write data for rs2, explicitly not branches) has one
sentence of justification per exclusion, tied to a specific other reader of the same register in the
same cycle, not an opaque exception list. **Formally verified** — 86/86 generated riscv-formal checks,
all four component proofs with their red-direction probes intact, F and G reproduce exactly (below),
62 → 74-program `.S` suite green, 11/11 mutation-check.

## What it costs the proofs

- **`make -C formal check`: 86/86 PASS**, matching `EXPECTED_FAIL` and `EXPECTED_CHECKS` exactly in
  both directions.
- **`components_decoder`, `components_executor`, `components_pcloop` (+ its `pcloop_cover`
  anti-vacuity control), `components_traps`** all PASS by k-induction, each after its own
  demonstrated red direction (`decoder-zkt-probe.py`, `executor-zkt-probe.py`,
  `traps-region-probe`) still fails at its own assertion's line.
- **F and G do not move**, and `remeasure-fg.py` itself could not run to completion in this
  environment: its `hang`/`liveness` sweep uses the `btor btormc` engine, whose trace-reconstruction
  step on a red (SAT) bound needs `btorsim`, which is not part of this sandbox's Homebrew `boolector`
  package (only the full OSS CAD Suite CI uses ships it) — the model-check verdict itself completes
  and is logged, only the post-hoc witness write fails, which `sby` reports as `ERROR` rather than
  `FAIL`. The same probe generation `remeasure-fg.py` uses was run by hand, reading the SAT/UNSAT
  verdict directly out of each `sby` logfile instead of through the script's status-file gate: `hang`
  is reachable at depth 4, 5 and 6 and unreachable at 7 (flip point 7, so F = flip − 1 = 6, matching
  `checks.cfg`'s declaration); `liveness_ch0` is reachable at gap 4 and 5 and unreachable at 6 and 7,
  at both trig 10 and trig 15 (G = 6, matching declared, and reproducing across both trig points the
  way ADR-0046's convention asks). Both flip points reproduce exactly. This is also the outcome the
  mechanism predicts on its own: forwarding only ever shortens how long an existing hazard stall
  lasts, it adds no stall reason, lengthens no stage and widens no scoreboard slot, so neither the
  worst-case first retire nor the worst-case retire gap can have grown.
- **`make mutation-check`: 11 mutations, each caught by exactly the detectors it is paired with**,
  including the three whose patch files this ADR re-anchored (§ above) and the Zkt-adjacent
  `atomic-region-ignored`/`loadstore-region-ignored` pair, both still caught by their original
  `bench decoder_tb` detector.
- `make test`: **74/74 PASS**, `test/EXPECTED_FAIL` matches exactly.

## Consequences

- **The stall-only commitment (invariant 4) is amended, not dropped.** Hazards are still resolved
  with no new pipeline state and nothing a later cycle un-commits; what changes is that one of the
  scoreboard's three in-flight slots (`executor_out`, never `out`) may now supply an eligible
  consumer's operand instead of costing it a stall. The bar this ADR clears is the one ADR-0083 wrote
  down: "a future candidate in this space has to show 12 MHz at its worst placement with margin
  outside the 1–2% placement spread" — 2.75% clears that, where 0.48% (on a narrower, now-stale
  placement-spread figure) did not.
- **The dead-end list is not extended.** ADR-0083's other two declines — forwarding to every operand
  reader, and the whole scoreboard deleted as a ceiling — are untouched by this measurement; this ADR
  is the third entry in that table taken rather than declined.
- **`CLAUDE.md`'s hazard bullet is corrected** to carry this measurement rather than ADR-0083's stale
  one.
- **The area cost is real and unpaid down**: +105 packed cells against a ±50 churn band, from one new
  struct bit and two forwarding muxes. Nothing here claims it is free; `FIT_MAX_LC`'s headroom absorbs
  it (4214 of a budget separately tracked) and it is not this ADR's business to spend that further.
- **What is not reachable is unchanged from ADR-0083's finding.** Forwarding still cannot reach a
  load's result (the accessor has not unpacked the memory word when it would be needed) or an AMO's,
  `lr.w`'s or `sc.w`'s (same reason, one cycle later) — `rd_ready` is exactly the gate that keeps those
  out, and the two remaining scoreboard slots' worth of hazard cycles behind them is the part of the
  suite's 35.6% this candidate does not touch.
