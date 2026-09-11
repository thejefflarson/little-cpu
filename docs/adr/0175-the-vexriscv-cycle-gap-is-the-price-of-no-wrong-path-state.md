# ADR-0175: The VexRiscv cycle gap is the price of no wrong-path state, and hazard cause C cannot close it

**Status:** Accepted · 2026-09-10 · *Reads [ADR-0083](0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md)
and [ADR-0154](0154-executor-only-forwarding-ships-on-a-tree-that-has-moved.md) against the hazard
column's own three-cause split, added by the same PR that split `HAZARD` into `hzA`/`hzB`/`hzC` in
`test/cxxrtl.cc` and `test/stall_report.py` (no ADR filed there — see that PR's own decision note).
Closes the open question [ADR-0160](0160-the-comparison-moves-to-the-parts-this-design-ships-to.md)'s
amendment left standing: whether the reversed cross-core comparison is closeable, and whether the
`hzC` slice behind a CSR register-form read is a candidate for anything.*

## Context

ADR-0160's amendment measured VexRiscv ahead of this core on Dhrystone once both cores are compared
in the configuration their own authors ship for performance — `GenFullNoMmuNoCache`, all four
`HazardSimplePlugin` bypasses on, `STATIC` branch prediction — rather than riscv-formal's
verification-only `FormalSimple`. That is a standing fact, not a bug: nobody has recorded whether the
gap is closeable, or by how much, so the next person to read a 21% hazard column has nothing to save
them re-deriving ADR-0083 from a blank page.

This ADR reads that gap against the one column the hazard split names as a candidate — `hzC`, a ready
result forwarding has no path to — and answers it directly: the addressable population is smaller
than the gap.

## What Dhrystone's cycle accounting says

`make dhrystone`, `-O2`, 2000 runs, on the tree this ADR merged on (commit `13f5f48`):

```
RETIRES 950437 SPEC-CHECKED 950432
STALLS cycles=1506772 issue=950439 divider=192 atomic=0 hazard=317207 serialize=83
       operand=238562 fetch=287 bus=0 region=2
       hzA=167844 hzB=6007 hzC=143356 hzCcsr=0 unattributed=0
```

CPI is 1.59 and nothing outside ISSUE (63.1%), HAZARD (21.1%) and OPERAND (15.8%) rounds to a
thousandth of the total — divider, serialize, fetch, bus and region are each 0.0%, atomic is exactly
zero. Retires (950,437) and issue cycles (950,439) differ by 2 in a million, the operand-fetch
guess's own residual (ADR-0089).

**The hazard column's three causes**, per `test/cxxrtl.cc`'s per-cycle charge and
`test/stall_report.py`'s identity check that they sum to the column: `hzA` (167,844 cycles, 11.1% of
the run) is a producer still in `out`, one stage before the executor, so no result exists anywhere
yet to hand over. `hzB` (6,007 cycles, 0.4%) is a producer in `executor_out` whose result is not
unpacked yet — a load, an AMO, `lr.w`, `sc.w` — gated by `executor_out.rd_ready`. Both are structural:
nothing forwarding could reach exists to forward. Only `hzC` (143,356 cycles, 9.5% of the run) is a
ready result forwarding has no path to, and it is the only one of the three this ADR treats as a
candidate for anything.

## Cause C's population, read off `rtl/decoder.v`

`rs1_fwd_eligible` is exactly `instr_math` (line 517) and `rs2_fwd_eligible` adds a store's, an AMO's
and `sc.w`'s write data (line 518). Every other category that reads `reg_rs1`/`reg_rs2` directly,
rather than through `rs1_forwarded`/`rs2_forwarded`, is a `hzC` reader by construction — there is no
sixth category, because `rs1_fwd_eligible` and `rs2_fwd_eligible` are the only two
forwarding-eligibility gates in the file, and every reader that falls outside them is one of the
following:

- `csr_arg` (line 291): a register-form CSR's operand.
- `mem_addr_calc` (line 331), `ls_block` (line 365) and `mem_addr_low` (line 377): a load's or
  store's effective address, region test and misalignment test, all built from `reg_rs1` alone or
  summed with the immediate.
- `atomic_addr` (line 333): an atomic's own effective address, `reg_rs1` verbatim (ADR-0109's
  no-adder spelling).
- `cmp_sub`/`cmp_lt` (lines 611, 614): the branch comparator, read from `reg_rs1`/`reg_rs2` directly
  because `branch_taken` must be a same-cycle function of the *current* instruction's operands, not a
  forwarded one meant for a different reader.
- The `instr_jalr` arm of `next_pc` (line 636): `reg_rs1` read directly for the jump target.

**`csr_arg`'s own measured contribution is zero on this workload (`hzCcsr=0`), but not for the reason
that category is structurally excluded.** `hazard_rs1`/`hazard_rs2` are checked ahead of `serialize`
in the decoder's publish order (`test/cxxrtl.cc`'s bucket priority), so a register-form CSR access
whose `rs1` is live in `executor_out` is held by the hazard check *before* `serialize`'s
`pipe_drained` gate is ever reached — `serialize` narrows the window this can happen in (it forces
the CSR to wait for an empty pipe once the hazard clears) but does not close it, and
`test/asm/csr.S`'s own vectors (`csrrw a1, mscratch, a0` immediately after `a0` is computed) hit
exactly this case, landing in `hzC` (`test/stall_report.py`'s own docstring records this — it is
measured, not proved zero). Dhrystone's own register-form CSR reads are `csrr %0, mcycle` and
`csrr %0, minstret` (`test/bench/dhry_port.c`), GCC's pseudo-ops for `csrrs rd, csr, x0` —
`rs1 == 0` — and `hazard_rs1` requires `rs1 != 0` unconditionally, the same guard every other
category is subject to. `csr_arg`'s zero here is a property of what this program reads a CSR
*with*, not a property of CSR accesses in general.

The remaining five readers all terminate at `next_pc`: `mem_addr_calc`/`ls_block`/`mem_addr_low`
through `region_stall` (line 382) and the misalignment trap (lines 402-410); `atomic_addr` through
`rtl/memory.v`'s range test (`atomic_supported`, lines 34-38); `cmp_sub`/`cmp_lt` through
`branch_taken`; and the `instr_jalr` arm directly. Every one of them sits inside one of the two loops
CLAUDE.md's fetch-loop section names — the region/misalignment and atomic-fault paths feed `stall` →
`next_pc` → the ROM address, and the branch comparator reads `reg_rs1`/`reg_rs2` the same cycle
`next_pc` does, which is why `cmp_sub`/`cmp_lt` read the register file directly rather than through a
mux forwarding could join.

## The ceiling

Spending every `hzC` cycle at zero clock cost and zero operand-column recapture — the two concessions
ADR-0154's own measurement shows are not available in reality (below) — moves this run's timed
Dhrystone figure from `make dhrystone`'s own 732.0 cycles/Dhrystone (1,464,021 timed cycles / 2000
runs) down by `hzC`'s 143,356 cycles / 2000 runs = 71.7:

```
732.0 - 71.7 = 660.3 cycles/Dhrystone
```

VexRiscv's own comparator, `GenFullNoMmuNoCache` with all four bypasses and `STATIC` prediction, read
640.1 cycles/Dhrystone when ADR-0160's amendment first measured it and reads 635.1 on this tree now
that the shared-ISA build carries the M extension (`soc/compare/vexriscv_pin.mk`, CLAUDE.md's
cross-core section). **660.3 clears neither figure** — spending the entire addressable population of
cause C does not reach VexRiscv's cycle count, it only narrows the gap. The addressable population
(143,356 cycles, 9.5% of the run) is smaller than the gap it would need to close, so this is not a
margin a later tree could hand back the way ADR-0154 re-took ADR-0083's confined-forwarding spelling:
that reversal rested on a *placement* margin (0.48% of clock, re-measured wider on a moved tree), not
on a population ceiling. A population ceiling does not move when the tree does; only its two
concessions could move it, and both are unavailable structurally, not provisionally:

- **The clock-cost concession is unavailable because cause C's readers are exactly the fetch loop's
  inputs.** ADR-0083 measured full-network forwarding — reaching operand readers past the executor's
  own slot — at 9.49 MHz against the 12.00 MHz requirement, a 21% miss, because `mem_addr_calc`,
  `atomic_addr`, `cmp_sub`/`cmp_lt` and the `instr_jalr` arm are precisely the terms CLAUDE.md's
  "two loops around the fetch address" section names as the ones a LUT level cannot be spent on at
  any area price. Cause C is that same term list under a different name.
- **The operand-column-recapture concession is unavailable because ADR-0154 measured the opposite on
  this exact benchmark.** Dhrystone's hazard column fell 357,798 → 317,207 cycles when executor-only
  forwarding shipped, and roughly 4,000 of the 40,591-cycle drop reappeared as *new* operand-fetch
  cycles (234,533 → 238,562) rather than converting straight to throughput — about 10% of a hazard
  reduction on this workload has already been shown to recur elsewhere in the same accounting, not to
  vanish.

## Decision

**Accept the standing.** Do not amend the stall-only hazard commitment (invariant 4) for Dhrystone's
sake. VexRiscv's cycle advantage on this benchmark is bought with two things that commitment and the
12 MHz requirement forbid outright: `STATIC` branch prediction is wrong-path state by definition —
committing to a guessed target before it is known correct is exactly the un-commit invariant 1
refuses to allow — and reaching every one of cause C's readers is the full forwarding network ADR-0083
already priced against the fetch loop and missed 12 MHz by 21%. Their cycle count is not a lever this
core's stated commitments leave on the table; it is the price of the two commitments this repo has
already measured and kept.

This ADR states the price rather than paying it.

## Falsifier

This decision is reopenable, not closed on principle. If a `hzC` figure above 10% of Dhrystone's
cycles is ever demonstrated **together with** a non-fetch-loop consumer of it — a reader among
cause C's population that does not feed `next_pc`, `region_stall`, the misalignment trap or
`rtl/memory.v`'s atomic range test — this ADR's ceiling argument no longer holds, and the right next
step is to reopen ADR-0083's re-measurement rather than to re-file this one. On this measurement,
`hzC` is 9.5% of Dhrystone's cycles and every one of its readers is a fetch-loop input; both halves of
the falsifier are currently false.

## What is not an opportunity here

`rtl/decoder.v:606` (`read_rs1 = fetch_stall ? prev_rs1 : stall ? rs1 : next_rs1`, and the identical
shape for `read_rs2`) already presents the *issuing* instruction's own register pair — not a guessed
successor's — on every cycle that instruction is held by a stall. CLAUDE.md's regfile commitment (6)
leaves "what is presented on a stalled cycle" as an explicitly separate question from the bypass
guard `operand_stall` protects; this line is that question's answer, and it predates this ADR. Cause
C's hazard is not a case of the wrong pair being read on a stalled cycle — the pair is always right —
it is a case of no path existing from a ready result to the raw register input at all. Re-deriving
this line's behavior is not a route to closing the gap either.

## Consequences

- **CLAUDE.md's hazard bullet (invariant 4) gains the three-cause split, the measured ceiling, and a
  fourth entry in the "still declined on the clock" list**, pointing here. The narrative stays in
  this ADR; CLAUDE.md states the rule and the number.
- **The dead-end list ADR-0083 opened is not extended by a re-measurement — it is extended by a
  ceiling.** Every prior entry in that list (full-network forwarding, a fourth scoreboard slot, an
  early register write) was declined on a margin a later tree could in principle re-measure past.
  Cause C is declined on a population count smaller than the target; no placement sweep changes a
  population count.
- **Nothing in `rtl/`, `formal/` or `test/` ships from this ADR.** It records a measurement and a
  decision against a gap that already exists in the tree; the hazard split's own graders
  (`test/stall_report.py`'s identity check, `test/cxxrtl.cc`'s bucket order) are unchanged.

## Amendment, 2026-09-10 — three accuracy corrections

A security review of this ADR found three defects; the decision is unchanged by all three.

1. **The operand-column figure disagreed with itself**: 238,562 in the cycle-accounting table
   above and 238,565 in the ceiling section's before/after pair. Re-running `make dhrystone` on
   this ADR's own tree (commit `13f5f48`) reproduces the table's figure exactly; the ceiling
   section's copy is corrected to 238,562 to match.
2. **The sentence introducing cause C's reader sites mis-described what it was counting.** It said
   "these are the only two assignments in the file that read the raw register inputs outside the
   eligibility-gated forwarding mux itself" immediately before listing five more sites that do
   exactly that — the sentence was describing `rs1_fwd_eligible`/`rs2_fwd_eligible`, the two
   forwarding-eligibility gates, not the raw-register readers the list names. Reworded to say what
   it means: those two are the only eligibility gates, and every reader outside them is one of the
   sites listed.
3. **This ADR's total (1,506,772 cycles) differs from CLAUDE.md's Dhrystone total (1,506,943).**
   Both are correct: they were measured on different trees. CLAUDE.md's figure is ADR-0154's own
   measurement, taken on the tree that landed executor-only forwarding, before this ADR's hazard
   split existed. This ADR's figure is this run, on the tree named above. Neither total is owed a
   re-take against the other — the cause-C split this ADR reads did not exist on ADR-0154's tree.
