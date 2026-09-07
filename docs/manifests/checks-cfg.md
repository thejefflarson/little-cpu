# `formal/checks.cfg`

genchecks' configuration for the generated riscv-formal check set. Most of its
lines are read as data by `genchecks-local.py`, `formal/genchecks-audit.py` or
`formal/depth_rules.py` — including the `#omit`, `#floor` and `#derive` lines,
which look like comments but are parsed, in the `#`-prefixed shape genchecks'
own cfg parser already skips before it sees a section. That is why this file
is not reduced to a one-line pointer the way the other manifests are: deleting
those lines to shrink it would delete the audit `genchecks-audit.py` runs
against them. What follows is the prose that used to sit beside them.

## `[options]`

The `insn_c_*` checks this generates exercise `rtl/decoder.v`'s compressed
decode on its own: `imem_data` is a free value every cycle here, so no fetch
window is involved. The `.S` suite is what covers the two together.

`isa` stays `rv32imc` although the core executes the eleven A instructions:
genchecks reads `insns/isa_<this>.txt` out of the pinned clone, and there is
no `isa_rv32ia*.txt` there to read. Spelling one in loses the whole
instruction loop to a caught `FileNotFoundError` — measured, `isa rv32imac`
generates 16 checks rather than 86, gaining no atomic and dropping all 70
`insn_*` — and prints one line about it. `formal/COMPLETE_EXCLUSIONS` is
where that absence is recorded and re-derived from the clone.

`solver` selects genchecks' engine, not just an SMT solver name: `bmc3` means
`abc bmc3`, `btormc` means `btor btormc`, anything else means `smtbmc <that>`.
Keep `btormc` — `reg_ch0` is one query that did not converge under
`smtbmc yices` in over 20 minutes and returns in 8-12s here.

## `[csrs]`

A name here generates that CSR's `csrw_*` check. A name followed by a test
list also generates one `csrc_<test>_<name>` check per spelling, which is the
only way upstream's `rvfi_csrc_{any,const,hpm,inc,upcnt,zero}_check.sv` models
are reachable at all.

This is not the set of CSRs the core implements. It is the subset
`rvfi_csrw_check.sv` can say something true about. Adding a WARL CSR
(`mtvec`, `mepc`, `mcause`, `mstatus`) or a read-only-zero one (`mie`, `mip`,
`mtval`) puts a FAIL on a correct core: that check has no WARL model, it
asserts that every bit the instruction offered appears set in what landed,
and masked bits do not land. Those are checked field by field in
`test/csr_tb.v` instead.

A name here also makes the generated `rvfi_macros.vh` declare
`rvfi_csr_<name>_{rmask,wmask,rdata,wdata}`, which `formal/wrapper.v`
connects to `littlecpu`. Adding a CSR is therefore new `ifdef RISCV_FORMAL`
output in `rtl/csrs.v` — which exports shadows for these three and no others
— plus a re-run of `make -C formal nonperturbation`.

`inc` is spelled here with no `csrc_inc` line in `[depth]` on purpose: that is
what makes genchecks consider and drop those two checks, which is what their
`#omit` lines below then have to declare. Delete the word and the decline
becomes prose only. `rvfi_csrc_inc_check.sv` is red on a correct core here: it
clears `csr_written` every non-check cycle, so its post-write fallback lives
one cycle, and CSR serialization means two CSR retires are never adjacent.
Measured PASS at 6..12, red at 13..16; no depth escapes it.

`upcnt` says the counter strictly increases between two reads, with writes
assumed away. It does not say `minstret` advances by exactly the
non-trapping issues, so `test/asm/minstret.S` and `test/csr_tb.v` still carry
that half.

## `[depth]`

This table is the list of checks that EXIST, not a tuning table. Both of
genchecks' call sites return early on a check with no depth line, so such a
check is not generated at all — no `.sby`, no directory, no status, no
warning, exit 0. It is then missing from the results and from
`formal/EXPECTED_FAIL` at once, and set equality reports a clean match on the
smaller set.

So the declined set is declared rather than described. Each `#omit` line
names one check genchecks considered and skipped, with a short reason;
`formal/genchecks-audit.py` traces the generator's own `get_depth_cfg` calls
and compares everything it dropped against these lines in both directions, so
a check upstream adds at the next pin bump fails generation until someone
rules on it. Only the check NAME is graded — the reason text is for the next
reader, not the check.

`[BLOCKED]` means the property is wanted and there is no hardware here to
state it about. `[DESIGN]` means it is declined on the merits; no hardware
change reopens it.

THE FETCH BUS REFUSES AND THE DATA BUS DOES NOT, and that split is what the
`#omit` rulings on the bus checks turn on. `rtl/imemory.v` reports a fetch
outside the text window on `imem_fault`, which decode raises as an
instruction access fault. A load or a store outside the map faults too —
cause 5 or 7 — but off a range answer that arrives with the address
(`rtl/memory.v`'s for an atomic, the decoder's own copy of the map, a cycle
late, for a plain access), never off a fault the data bus carries with its
response. So `fault_ch0` is generated and covers both the arm that check
calls `ifault` and the load/store faults, which reach it through
`rvfi_mem_fault` and its two masks, while no `rvfi_bus_*` refusal exists to
model.

### Depths are derived, not inherited

Depths are derived from two figures the checks measure about themselves in
seconds each: F, the worst-case first retire (from `hang`), and G, the
worst-case retire gap (from `liveness`), both declared by `#derive` lines. A
depth below its derived floor does not fail — it goes green having stopped
asking — and two entries clear their floor by exactly one cycle. The
`#derive` and `#floor` lines are what `formal/genchecks-audit.py` evaluates,
against the START, TRIG and CHECK cycles it reads back off each `.sby` it
just generated; a depth below its floor fails generation and names itself.

F: sweep `hang`'s check cycle — red at 4, 5 and 6, PASS at 7.
`rvfi_hang_check.sv` asserts on a registered flag, so the flip point is
F + 1. G: sweep `liveness`'s trig-to-check distance — red at 3, 4 and 5, PASS
at 6, at trig 10 and again at trig 15. The gap-5 counterexample is also this
measurement's non-vacuity witness, since `rvfi_liveness_check.sv` opens with
`assume(rvfi_valid)` at its trig cycle.

`make -C formal remeasure-fg` is that sweep, in both directions, and it
grades what it measures against the `#derive` lines. Any change that adds a
stall reason, lengthens a stage or widens the scoreboard past its fixed slots
has to run it before landing.

G's worst gap has the AMO write cycle in it: an AMO writes its result back on
the cycle after the executor takes it, nothing issues on that cycle, so the
worst gap between two retires is one longer than the pipeline alone would
make it. F is set by the first retire out of reset, which no atomic is part
of.

THE LOAD/STORE REGION WAIT DID NOT MOVE EITHER OF THEM, which is measured and
not assumed: `make -C formal remeasure-fg` reproduces both flip points
exactly with the eighth stall reason in the core. So an added stall reason is
not the same thing as a longer worst gap — the gap of 6 is set by a sequence
this wait can share a cycle with, and a reason has to be on the worst path to
move it. Run the sweep anyway; that is what turned this from a guess into a
line.

Both numbers hold with the machine timer interrupt in the core, because every
harness here ties the interrupt input off — `formal/INTERRUPT_TIE_OFF`
declares that and `formal/check-interrupt-tie-off.py` enforces it. Both flip
points above were taken under the tie-off. An interrupt costs a cycle that
would otherwise have issued, so a free input there would move G and every
depth built on it.

The first column of the numeric `[depth]` rows is `RISCV_FORMAL_RESET_CYCLES`,
which `checks/rvfi_testbench.sv` wires to the CHECKER's shadow state only;
the DUT is held in reset for cycle 0 regardless. So a two-retire check has to
clear G over the window between the cycle its shadow starts recording and the
cycle it asserts on — `start+G` and `trig+G` in the `#floor` lines — and
`reg 15 22` is a window of 7 against G = 6 rather than something vacuous. A
check that asserts about ONE retire is bounded the other way, by how late
that retire may be: `F+2G` buys two hops, so the retire under test can be a
third rather than the first out of reset.

The thinnest entries clear their floor by one cycle, which is the atomic
wait's cost stated where it is enforced. None of the depths is raised over
it, because a depth at or above its derived floor is asking the whole
question and raising one costs the `formal` job's twenty-minute budget for
nothing measured.

`fault` takes `insn`'s number and not one of its own, so the two move
together when F or G moves. Both clear F + 2G = 18 by one.

ITS OWN COUNTEREXAMPLES ARE REACHABLE MUCH SHALLOWER THAN THAT, and the depth
is the derivation rather than a measured need. Probe: report the read half of
the fault mask on a fetch fault (add `imem_fault` to the `{4{...}}` that
builds `out.rvfi.mem_fault_rmask` in `rtl/decoder.v`), which makes the check
read an instruction fault as a load fault and assert `insn != 0` against a
word that is zero. Swept 19 down to 1, it is FAIL at every depth — a fetch
fault is reachable on the first retire out of reset, so nothing here needs
the two hops. The number is kept at `insn`'s because a shallower one would
stop asking about later retires for no measured reason.

`csrc_upcnt` and `csrc_any` are the only two-retire checks here, and F and G
do not bound them: they shadow a value from one retire of a CSR instruction
naming the CSR under test and assert against a second, and CSR instructions
serialize. Their floor is measured at 9 — three one-line mutations of
`rtl/csrs.v` each PASS at 6..8 and go red at 9..16 — and 15 clears it.

What a depth below its floor buys is silence, and this is what that silence
looks like. The probe for the 70 `insn_*` checks: delete `rtl/executor.v`'s
`rs2[4:0]` shift masking and `insn_sll_ch0`/`insn_srl_ch0`/`insn_sra_ch0` each
report a counterexample at the depth below. Sweep `insn` down on that same
mutation and the counterexample survives to 5 and disappears at 4, where no
instruction can retire and `assume(rvfi_valid && spec_valid)` is
unsatisfiable. Reach for it before believing any `insn_*` result taken under
a changed configuration. The three `csrc_*` checks have one-line probes of
their own: `assign mcycle_plus = mcycle;`, `assign minstret_plus = minstret;`,
and deleting `MSCRATCH: mscratch <= warl;` from `rtl/csrs.v`.

Depths move only on evidence, in either direction, and there is no per-check
wall bound — the only budget is the `formal` job's `timeout-minutes: 20`, so
a raised depth that stops converging takes the hand-authored tasks down with
it.

All of the above assumes `RISCV_FORMAL_ALTOPS` (see `[defines]`). Without it
the real divider holds `executor_out.valid` for up to 31 cycles — 15 when the
dividend's top half is zero — F becomes ~36 and G ~35, and every number here
must be re-derived.

## `[defines]`

`RISCV_FORMAL_MEM_FAULT` declares `rvfi_mem_fault` and its two masks. Without
it those three signals do not exist in the macro set at all and `fault_ch0`
cannot be generated, whatever its depth line says.

`mcause` is deliberately NOT in `[csrs]` above, and `RISCV_FORMAL_CSR_MCAUSE`
is not that: a name in `[csrs]` generates `csrw_mcause_ch0`, which has no
WARL model and would fail a correct core. This define only declares the four
`rvfi_csr_mcause_*` signals, which `rtl/csrs.v` exports under the same macro.
It is REQUIRED, not optional: `checks/rvfi_fault_check.sv` at the pin puts
its whole `mcause` block behind this `ifdef` and leaves a dangling `else`
behind when it is absent, so the check does not parse without it — measured,
`ERROR rc=16` with a syntax error at line 96.
