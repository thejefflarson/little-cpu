# ADR-0168: nanocpu joins riscv-formal now; iverilog and cxxrtl wait

**Status:** Accepted · 2026-09-07

## Context

CLAUDE.md's verification section names four legs for the existing core --
cxxrtl (primary runner), iverilog (the only leg that sees an X), riscv-formal
(per-instruction oracle) and Sail co-simulation (independent architectural
oracle) -- and states each is load-bearing for a different reason. Importing
`nano/nano.v` (ADR-0167) needs the same question answered for nanocpu: which
of those apply today, at the import step, versus later.

## Decision

**riscv-formal only, for now, and it is wired into CI.** `formal/check-baseline.sh`
(reused unmodified, not forked) grades 79 generated checks plus `dmemcheck`,
`imemcheck` and `complete` against `nano/formal/EXPECTED_FAIL`/`EXPECTED_CHECKS`.
The `formal-checks` CI job runs `make -C nano/formal check` and gates on that
baseline as a new step after the main core's own; `formal-extra` runs
nanocpu's `imemcheck`, `dmemcheck` and `complete` as new steps after the main
core's own. Neither is on `make test`'s path, which CLAUDE.md already flags as
tight at `timeout-minutes: 25`; the nanocpu checks add well under four minutes
to `formal-checks`' own budget on this measurement, dmemcheck/imemcheck/complete
each add single-digit seconds to `formal-extra`.

**iverilog: not yet, and the gap is explicit, not silent.** ADR-0167 records
why: the donor declares most of its decode flags below the code that reads
them, iverilog 13 cannot elaborate that ordering, and reordering the whole
file is reshape work this ticket does not do -- reordering even a handful of
declarations already makes the file diverge from the pristine `c55efd6` copy
this ticket ships instead. CLAUDE.md is explicit that iverilog is "the only
leg that can see an X" -- an undefined memory word turning the pipeline
four-state-unknown is invisible to cxxrtl and to every formal check, which
drive an undriven input as a free two-state variable. That gap stays real for
nanocpu until a reshape pass fixes the ordering. It matters concretely for one
thing CLAUDE.md calls out by name: **a pad tri-state check**, which the
brief's own sequence (step 6) puts on the iverilog leg for exactly the
X-visibility reason. Any bring-up work that reaches the `tt_um_` top's
`uio_oe` before nanocpu has an iverilog leg again owes that check its own plan
for seeing an X, because there won't be one for free.

**cxxrtl: not yet, deliberately smaller in scope than it sounds.** The
existing `test/cxxrtl.cc` runner is built around `rtl/littlecpu.v`'s ports,
`rvfi_valid`-gated retire, and this repo's own stall-bucket accounting; none
of that exists for the donor's picorv32 bus and halt-on-trap `cpu_state`
machine. A cxxrtl runner for nanocpu is new harness work belonging to the
`.S`-suite-equivalent step the brief's sequence puts after the QSPI front end
and the CSR/trap layer land (steps 4-6), not to this import step, so it is not
attempted here.

**Sail co-simulation: deferred by the brief itself**, decision 10 -- `base.E =
true`, a second config json for flash/PSRAM/peripheral regions -- none of which
exists yet because the QSPI front end and the memory map it needs are later
steps.

## Consequence

Only one baseline is owed by this ticket, and it is the one already recorded:
`nano/formal/EXPECTED_FAIL` (2 known-red: `csrw_mcycle_ch0`,
`csrw_minstret_ch0`) and `nano/formal/EXPECTED_CHECKS` (79 names), graded both
ways. `formal/check-baseline.sh`'s script is reused unmodified, so a future
pin bump that changes its vocabulary or format is one file to update, not two.
Every later step in the brief's sequence -- E, no-M, the QSPI front end, CSRs
and traps, the pad tri-states -- is the point at which the leg it specifically
needs (iverilog for the tri-states, a new cxxrtl runner for the `.S`-suite
equivalent, Sail once the memory map exists) gets built, not before.

## F and G, re-derived rather than inherited, and reproduced on a clean run

The brief itself flags the donor's 2020 depths (`insn 10`, `pc 20`,
`liveness 20`) as inherited and unproven on this core; CLAUDE.md's own rule is
that a shallow depth does not go red, it goes green having stopped asking. This
donor is a **multicycle** design -- one instruction in flight over
fetch/ready/decode/execute/finish_load/finish_store/check_pc/reg_write/multiply/divide
-- so its F and G have no reason to resemble the pipelined core's (F = 6,
G = 6): a single retire here costs several state-machine cycles, not one.

Re-derived by the same method `formal/remeasure-fg.py` uses against the main
core (`nano/formal/remeasure-fg.py`, HERE = `nano/formal`, `depth_rules.py`
imported from `formal/` rather than copied): sweep `hang`'s check-cycle column
for the first depth that passes (F = flip - 1, since `rvfi_hang_check.sv`
asserts a registered flag one cycle after the true bound), sweep
`liveness_ch0`'s gap at two trigger cycles (10 and 15) for the first gap that
passes at both (G), and require both to reproduce inside the script's
BELOW/ABOVE = 2/1 window around the declared `#derive` lines.

**F = 12, G = 10** -- `hang` flips FAIL to PASS exactly at check-cycle 13 (F+1,
matching `formal/checks.cfg`'s own `#floor hang F+1` term); `liveness_ch0`
flips at gap 10 from both trigger 10 and trigger 15, so G does not depend on
where it was asked, the sweep's own soundness check. `nano/formal/checks.cfg`
declares both as `#derive` lines and every `[depth]` row is set from them using
the same term vocabulary `depth_rules.py` already defines (`F+2G` for `insn`
and `csrw`, `F+G,start+G` for `reg`/`pc_fwd`/`pc_bwd`/`causal`,
`F+G,trig+G` for `liveness`/`unique`, `F+1` for `hang`), with a small margin
above each bare floor the same way `formal/checks.cfg`'s own rows carry one.
`python3 nano/formal/remeasure-fg.py` reproduces this exactly: "Both
reproduce. F + G = 22, F + 2G = 32." Unlike `formal/genchecks-audit.py`,
nothing here grades that margin automatically yet -- nanocpu has no audit
script of its own, and building one is follow-up work rather than a silent
gap, since the alternative (skipping the derivation) is exactly the failure
mode CLAUDE.md names.
