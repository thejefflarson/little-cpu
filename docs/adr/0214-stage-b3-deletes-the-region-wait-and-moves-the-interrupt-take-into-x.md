# 0214 — Stage B3 deletes the region wait and moves the interrupt take into X

Status: Proposed. 2026-09-27. Ships as a PR against `thejefflarson/fetch-refactor`, the
integration branch B1 (ADR-0208) and B2 (ADR-0213) already landed on. `make fit` and
`make soc-timing` are red on this whole stack by the owner's own decision (ADR-0207): cells
are trimmed after the restructure finishes. This ADR reports both anyway, against `main`'s
and B2's own numbers, since the owner's area pass reads from these reports rather than from
a green gate.

This ADR covers two checkpoints landed under one number, because the second continues the
first's own argument rather than opening a new one: once the D/X split (B1) gave X a stage
of its own with no fetch-loop timing to protect, two things the fused decoder was forced to
defer turned out not to need deferring at all.

## What this is

**Checkpoint 1 — delete the load/store region wait.** Every same-cycle spelling of the plain
load/store region test tried on the *fused* decoder put the effective-address sum in the
fetch loop and cost the board clock (seven priced: ADR-0104, ADR-0116, ADR-0128), which is
why the fused decoder answered from `reg_rs1` alone where it could and deferred a cycle at
an edge otherwise (ADR-0129). B1's own D/X split moved the region test into X unchanged in
shape — a two-tier deferred answer, `ls_answer`/`ls_capture`/`ls_answer_valid` — without
re-asking whether X, now a stage removed from the fetch loop's own budget, still needed the
deferral. It did not. `rtl/executor.v`'s region test now reads `mem_addr_calc` (the
effective address, already computed combinationally for every load/store) directly against
the map's windows and answers the same cycle, for every window — text, RAM, timer, UART,
flash — with no asymmetry between a wide window and a narrow one. `x_busy` loses one of its
two reasons to hold X across multiple cycles; only the divider remains.

**Checkpoint 2 — move the timer interrupt's take into X.** D used to decide the interrupt on
its own: it read `interrupt_pending` a cycle before X ever saw it and, when armed and not
otherwise stalled, discarded whatever it had just decoded and published a synthetic
one-cycle bubble (`is_interrupt`, pc only, every other field zero) into `out` instead. This
is the one thing `test/decoder_tb.v`'s own module comment still called out as "every trap
but the timer interrupt" moved to X. It moves now: X reads `interrupt_pending` itself,
live, and — on any cycle it is not mid-divide and D has handed it a real instruction —
takes the trap in place of executing it, rather than D pre-empting the decode a cycle ahead
with a bubble of its own. D no longer knows `interrupt_pending` exists.

**The accessor's memory-transaction launch was checked, not re-done.** The brief that opened
this checkpoint named moving it "from `decoder_out`" as parallel work; reading
`rtl/accessor.v`'s own port comment and `rtl/executor.v`'s `launch.*` assigns confirms this
already happened as a side effect of B1's D/X split (ADR-0208) — `launch` is X's own
combinational view of the instruction it is resolving this cycle, gated to zero on a trap,
and `rtl/littlecpu.v` presents it to the accessor the same cycle X computes it, not a cycle
earlier from D. Nothing here needed changing; ADR-0099's idempotency argument (a
re-presented request is harmless for RAM and not for a device) is unaffected by either
checkpoint and is not re-measured.

## The mechanism

### Checkpoint 1: the region test loses its deferred half

`rtl/executor.v`'s `ls_supported` (renamed from the fused decoder's `ls_answer`) is now a
single combinational expression over `mem_addr_calc`, tested the same cycle against every
mapped window's own power-of-two or few-byte mask. There is no register left to capture an
answer a cycle late, and no `ls_answer_valid` gate on it. `x_busy` — the one signal telling
D to hold `out` rather than publish a new instruction — is now exactly the divider's own
`state != init`, restated (not aliased, so `test/zkt_isolation_test.py`'s one-hop taint
block still lands on a real flip-flop's output) rather than a second condition ORed in.

**The layout preference this used to create is retired.** ADR-0158's convention — start
`.data` one block clear of a mapped-region edge, so a program's own accesses hit the fused
decoder's fast (undeferred) arm on essentially every access — is no longer load-bearing: the
region test answers in one cycle from every offset now, so there is no fast arm to keep
landing on. Dhrystone and CoreMark measure no cycle change from this deletion, because their
own linker scripts already followed the convention and the wait it paid was already down to
2 cycles each; the win this deletion buys is a program *no longer needing* the convention to
reach that floor, not a faster number on the two programs that already had it. The linker
scripts and `test/probe_gates.sh`'s layout `ASSERT`s are left in place as harmless structure
— retiring them is separate, unstarted work.

### Checkpoint 2: `interrupt_pending` moves from a D input to an X input

- **`rtl/structs.v`**: `dx_output` loses `is_interrupt`. D no longer has anything of the
  kind to publish.
- **`rtl/decoder.v`**: loses the `interrupt_pending` port and the register-update branch
  that used to build the one-cycle bubble. The `always_ff` chain is now
  `reset → x_busy hold → x_redirect kill → stall bubble → normal decode`, with no fifth
  arm. D's FORMAL block drops its `out_is_interrupt` wire and the assert that used it
  (`out_is_interrupt ⇒ out_rd == 0`), and the "not `&& !out_is_interrupt`" caveat on the
  class-flags onehot0 check is gone, because there is no longer a second way for `out` to
  be valid with no class flag set.
- **`rtl/executor.v`**: gains an `interrupt_pending` input and one new signal,
  `take_interrupt = in_valid && !x_busy && interrupt_pending`, read fresh every cycle
  rather than carried on `in`. Every place that used to read `in_is_interrupt` — the
  trap-cause priority chain (still first in line, so an interrupt still outranks the same
  instruction's own fault), `trap_pending`/`trap_taken`, `executing`/`launch.valid`, and the
  RVFI `pending_intr` latch — now reads `take_interrupt` instead, with no other change to
  their shape. Because `take_interrupt` already carries its own `in_valid`/`!x_busy` gate,
  `take_interrupt ⇒ trap_entry` is now provable **standalone**, inside `rtl/executor.v`'s
  own FORMAL block — this used to be a fact only the composed `formal/traps.sv` proof could
  state, since it depended on D never handing X an interrupt while `x_busy` held. That
  dependency is gone along with D's own role in the decision.
- **`rtl/littlecpu.v`**: `csr_interrupt_pending` rewires from `decoder`'s (deleted) port to
  `executor`'s new one. Nothing else moves.
- **`formal/traps.sv`** and **`formal/pcloop.sv`**: both compose `decoder` and `executor`
  directly (no `littlecpu` wrapper), so both rewire `.interrupt_pending(...)` from the
  decoder instance to the executor instance. `formal/traps.sv`'s own `dx_is_interrupt` —
  used to build `prev_interrupt_pending`/`prev_interrupt_entry` and gate the trap-priority
  checks — used to read `dx_out.is_interrupt` directly; it now restates X's own decision,
  `dx_valid && !x_busy && interrupt_pending`, since there is no field left to read it off.

## Verification

- `make test-units`: all sixteen benches PASS, `test/decoder_tb.v` and `test/executor_tb.v`
  included. `test/decoder_tb.v`'s interrupt vector is deleted outright (D has nothing left
  to test there); `test/executor_tb.v` gains five in its place: an armed interrupt
  displacing an ordinary instruction (cause, `mtvec`, `trap_epc` at the displaced
  instruction's own pc, no `instret`/CSR/`mret`, no tval, and `launch.valid` staying low);
  an armed interrupt on a genuine bubble (`in == '0`) taking nothing, since there is no
  victim; an armed interrupt outranking the same instruction's own would-be illegal fault;
  and an armed interrupt failing to preempt a divide already in flight, drained before the
  next vector so a held `in.is_div` cannot silently relaunch (X has no "consumed" input of
  its own — only D's own hold logic prevents that in the real pipeline, and this bench has
  no D). `test/exec_tb.v` and the two formal harnesses that instantiate `executor` directly
  needed the same new port wired (tied to `1'b0` where the bench never arms it).
- `make lint`: clean, both RVFI passes.
- `make elaborate-strict`: clean.
- `make -C formal remeasure-fg`: **F = 5, G = 5, unchanged from checkpoint 1's own
  re-measurement** (down from B1/B2's 6/6). Moving the interrupt decision into X changes
  *where* it is computed, not *when* it takes effect: D used to publish a bubble one cycle
  before X read it and redirected; X now reads the live line and redirects the same cycle
  it would otherwise have processed whatever D handed it — the identical number of cycles
  from "interrupt becomes pending" to "fetch redirects to `mtvec`". `formal/checks.cfg`
  needed no edit.
- `make -C formal all`'s individual targets, run directly since `components_executor`'s own
  prerequisite is broken independently of this PR (see "Known gap" below): `check` (86/86
  generated riscv-formal checks, `EXPECTED_FAIL`/`EXPECTED_CHECKS` both match), `complete`
  and `complete_cover` (depth 50, 13 cover goals all reached by step 4), `imemcheck`/
  `imemcheck_cover`, `dmemcheck`/`dmemcheck_cover`, and `components_decoder`/
  `components_pcloop`/`components_traps`/`components_busarbiter`/`components_accessor` all
  close by k-induction. `components_traps` in particular exercises the new standalone
  `dx_is_interrupt` restatement (`formal/traps.sv`) by composition, `traps_cover` still
  reaches `mtval_interrupt_reached`/`mcause_interrupt_reached`, and `traps-region-probe.py`/
  `traps-tval-probe.py` still fail at their own named lines. Only `components_executor`
  itself was not obtained this run.
- `make cosim-suite`: PASS, same 69/75 baseline as B1/B2 — this checkpoint touches nothing
  Sail's own timer-model limitation didn't already exclude.
- `test/zkt_isolation_test.py`: PASS, re-derived rather than edited around. `STRUCT_PORTS`
  drops `is_interrupt` from `dx_output`'s field list (matching the struct); the new
  `interrupt_pending` input on `rtl/executor.v` is a single bit, below the 5-bit threshold
  the script classifies at all, and it reaches nothing the taint check follows (`x_busy`'s
  only path is still through `state`) — an asynchronous condition external to the
  instruction's own operands cannot make a Zkt-listed instruction's cycle count depend on
  its own register VALUES, which is the only claim this script grades.
- `make dual-smoke`, `make dual-build`: OK, unaffected — `rtl/littledual.v` instantiates
  `littlecpu` twice and never wires either stage's internals directly.
- `make probe-gates`: PASS, no new probe needed. Neither checkpoint added a new named
  grader with its own forced-red demonstration to build; the new standalone assertion in
  `rtl/executor.v` (`take_interrupt ⇒ trap_entry`) is composed and proved by
  `components_traps`, which does close (above); `components_executor`'s own copy could not
  be obtained this run for the unrelated, pre-existing reason below.

## Known gap: `executor-zkt-probe.py`, not this checkpoint's

`components_executor` — the one target under `make -C formal all` this ADR could not
report — is gated on `executor-zkt-probe.py`, a forced-red control that mutates a copy of
`rtl/executor.v` (diverting `MUL(0, nonzero)` into the divider's own arm) and requires the
BASECASE leg to fail at exactly the MUL constant-latency assertion's own line, on the theory
that the mutated core's *value* stays correct (`0 / reg_rs2 == 0 == mul_lo`) and only its
*latency* breaks. On this session's toolchain it instead fails at the pre-existing
MUL-value assertion a few lines above (`out_rd_data == $past(mul_lo)`), because that
assertion is checked one cycle after ANY launch, and the mutated arm leaves `out.rd_data`
holding whatever a PRIOR instruction last wrote there rather than freshly computing
`mul_lo` — a case the probe's own docstring argues around but does not appear to rule out
for a trace where the diverted MUL is not the first instruction since reset.

**This is not a regression from either checkpoint.** Rebuilding `rtl/executor.v` at this
PR's parent commit (`4ddd7d0`, checkpoint 1's own tip) and re-running the identical probe
reproduces a different but equally red outcome: the mutated core PROVES (`mode prove`
reports no counterexample within the induction bound at all), which the probe's own script
also treats as a failure — "an arm that admits [an operand-dependent second cycle] is
asking nothing at all." Neither tree passes this probe; they fail it two different ways,
consistent with a toolchain-sensitive (solver-order-dependent) gap in the probe's own
mutation rather than anything either checkpoint's RTL touched — neither touches
`launch.is_mul`, the divide arm, or `out.rd_data`'s own assignment. Fixing the probe is a
design decision about its own reachability argument (whether to require `out.rd_data`
reset alongside `state`, or to scope the value check to `$past(state) == init` from a fresh
launch only) that belongs to whoever owns `formal/executor-zkt-probe.py`, not to this
checkpoint. Filed here rather than silently worked around, per this repo's own rule that a
graded comparison's forced-red direction is load-bearing: this one is currently forcing red
for the wrong reason, on both the tree behind this PR and the tree in front of it.

## Measured

`make cycles`, full suite (77 programs, `STALL_REPORT=1`):

| | cycles | Dhrystone cycles | DMIPS/MHz | CoreMark cycles | CoreMark/MHz |
|---|---|---|---|---|---|
| main (pre-B1) | — | 1,613,644 | — | — | 2.155 |
| B2 (ADR-0213) | 30,893 | 1,394,022 | 0.816 | 41,424,774 | 2.414 |
| B3 checkpoint 1 (region wait deleted) | 30,754 | 1,394,022 | 0.816 | 41,424,774 | 2.414 |
| B3 checkpoint 2 (this PR, freshly measured) | **30,753** | **1,394,022** | **0.816** | **41,424,774** | **2.414** |

Dhrystone and CoreMark are measured, not assumed, on this PR's own tree and read digit-for-
digit identical to checkpoint 1's — neither ever arms the timer interrupt, so nothing in
either benchmark's instruction stream reaches the code this checkpoint touched. The
hand-written suite's own total reads one cycle under checkpoint 1's own reported 30,754,
inside the noise of which of `mtimer.S`/`mtimermask.S`/`lrsclock.S` — the three suite
programs whose own cycle count depends on exactly when the interrupt lands relative to a
sampled instruction, by `test/OBSERVED_FLOOR`'s own header — happens to sample the interrupt
one cycle differently now that X, not D, decides it. All three retire at exactly their
baselined floors (`mtimer.S` 1103, `mtimermask.S` 296, `lrsclock.S` 177) and the suite's
`EXPECTED_FAIL` list matches exactly, so this is a measured, harmless one-cycle shift in
*when* a timing-sensitive program's own interrupt lands, not a retire-count regression.

Area (reported, not gated — `make fit` and `make soc-timing` are expected red on this whole
stack, ADR-0207):

| | `make fit` (ICESTORM_LC) | SoC synthesized demand (ICESTORM_LC) |
|---|---|---|
| main | — | 4,920 / 5,280 |
| B2 (ADR-0213) | 4,850 | 5,550 / 5,280 (105%) |
| B3 (this tree) | **4,733** | **5,483 / 5,280 (103%)** |

Both area figures fall from B2's — checkpoint 1 deletes the region test's own deferred-answer
registers (`ls_capture`/`ls_answer_valid` and their gating), and checkpoint 2 deletes D's own
interrupt branch (one register-update arm, one struct field) while adding one input and one
OR term to X's existing `trap_taken`. `make fit`: 4,733 against 4,850 (−117 cells). `make
soc-timing`'s synthesized demand: 5,483/5,280 against 5,550/5,280 (−67 cells) — still over the
part, so nextpnr still cannot place it and there is still no Fmax number for up5k on this
tree; `soc/pin.json`'s pin also reads PIN STALE against these sources, as expected for any
source change. `make ecp5-timing` is not re-measured in this ADR: checkpoint 1 removes logic
from the region test's own comparator and checkpoint 2 adds one more OR term to X's already
five-way `trap_taken`, neither the kind of change this stack's own placement-spread band
(ADR-0194) would read as distinguishable from churn at one placement.

## Kill check

Neither checkpoint carried an owner-directed kill criterion of its own the way B2's did
(B2's was "beat main's Dhrystone floor or stop before B3"); both are declared measured
rather than killed against. Checkpoint 1's own criterion, implicit in the brief that opened
it — delete a deferred cycle with no regression on the two programs that already avoided it
— is met: Dhrystone and CoreMark are unmoved, and the hand-written suite drops 140 cycles
(30,893 → 30,753) against B2. Checkpoint 2's own criterion — move the decision without
changing its timing — is met by F/G reproducing at 5/5 unchanged and by Dhrystone and
CoreMark reading digit-for-digit identical to checkpoint 1's own freshly-remeasured figures.

## Decision

**SHIPPED**, with one gap filed rather than fixed: `components_executor` could not be
obtained this run for a pre-existing, toolchain-sensitive reason in `executor-zkt-probe.py`
unrelated to either checkpoint (reproduced identically on this PR's parent commit; see
"Known gap" above). Both checkpoints are otherwise pure simplifications with no measured
cost: checkpoint 1 deletes a deferred answer nothing needed once it left the fetch loop, and
checkpoint 2 moves a decision to the stage that actually owns it, provable standalone where
it used to depend on a cross-module argument. `make fit`/`make soc-timing` stay on the
owner's own expected-red list (ADR-0207) until the restructure's area pass lands.
