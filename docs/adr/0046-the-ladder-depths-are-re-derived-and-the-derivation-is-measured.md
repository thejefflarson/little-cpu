# ADR-0046: The ladder's depths are re-derived for the five-reason pipeline, and the derivation is measured rather than traced

**Status:** Accepted · 2026-08-01 · *Supersedes [ADR-0025](0025-formal-ladder-depths-are-derived-not-inherited.md)'s
numbers and its addendum; supplements ADR-0004, ADR-0009, ADR-0015, ADR-0026, ADR-0033, ADR-0042.
Closes the term-1 reopening [ADR-0045](0045-two-m2-terms-close-by-amendment-and-one-was-already-met.md) recorded.*

## Context

ADR-0025 established the rule that `formal/checks.cfg`'s `[depth]` numbers are **derived, not
inherited**, and derived them by hand-tracing the pipeline. Two things then invalidated the
derivation without invalidating the rule.

1. **ADR-0025's own addendum** recorded that CSR serialization (ADR-0026) moved the two-hop figure
   from 12 to "roughly 15 — exactly the configured `insn 15`, with zero margin", and said the
   empirical sweep *"should be repeated once the CSR RTL exists rather than assumed to still hold."*
   It never was.
2. **`e4f5250` added `operand_stall`** (ADR-0042), a **fifth** stall reason that fires on nearly every
   instruction — +18.0% suite cycles, measured — and touched neither `checks.cfg` nor ADR-0025.

A depth that no longer clears its own floor does not fail. It goes green having stopped asking. That
is ADR-0037's failure mode landing on the one M2 term currently marked met, so "82/82 with both
baselines exact" was a number whose meaning was unverified.

This ADR re-derives the depths, and changes *how* they are derived: the two quantities the whole
argument rests on are now **measured by the ladder about itself**, in seconds, rather than
hand-traced. Hand-tracing is what went stale; a measurement a reader can re-run cannot.

## A correction that has to come first: what the first `[depth]` column actually does

ADR-0025 said `genchecks-local.py` "hardcodes `` `RISCV_FORMAL_RESET_CYCLES 1``, so the DUT's own
reset (`littlecpu.reset`) is high for exactly cycle 0". The conclusion is right for `insn`; the
reasoning is wrong, and the wrong reasoning generalises to a false statement about every
two-column check.

`checks/rvfi_testbench.sv` wires `cycle < `RISCV_FORMAL_RESET_CYCLES` to the **checker instance
only**. The DUT is instantiated separately and gets the testbench's own `reset` port, which
`assume(reset == $initstate)` pins to cycle 0 and nothing else.

So the first column of a `check_cons` line is **how long the check's shadow state is held clear**,
not how long the core is held in reset. `reg 15 20` is not "reset for 15 cycles, then check at 20";
it is a **five-cycle checker window over a core that has been running since cycle 1**. Read the
other way round, `reg` looks vacuous by construction — the earliest possible retire would land on
the check cycle itself, so no producer could precede it — and it is not vacuous. § "The probes" below
shows it catching a real defect at exactly these numbers.

## The derivation, in two measured numbers

Every depth on this ladder is checked against two quantities:

| | | measured by |
|---|---|---|
| **F = 6** | the **worst-case first retire** — the latest cycle by which *some* instruction must have retired, counting from the DUT's reset | sweeping `hang`'s check cycle |
| **G = 4** | the **worst-case retire gap** — the most cycles that can separate two consecutive retires | sweeping `liveness`'s trig-to-check distance |

Both come from checks already on the ladder, driven at reduced depths against unmodified RTL.

### F, from `hang`

`rvfi_hang_check.sv` asserts that a retire has happened by its check cycle, with no trigger
assumption. Its flip point is therefore `F + 1` — the flag it asserts on is registered, so a retire
at cycle *r* is visible at *r+1*.

| `hang` check cycle | verdict |
|---|---|
| 4 | `bad state property 0 reachable at bound k = 4 SATISFIABLE` |
| 5 | `bad state property 0 reachable at bound k = 5 SATISFIABLE` |
| 6 | `bad state property 0 reachable at bound k = 6 SATISFIABLE` |
| **7** | **PASS** |
| 8 | PASS |

**F = 6.**

### G, from `liveness`

`rvfi_liveness_check.sv` assumes a retire at its trig cycle and asserts the next instruction retired
by its check cycle, so the smallest passing `check − trig` is exactly the worst-case gap.

| trig | gap | verdict |
|---|---|---|
| 10 | 2 | `bad state property 0 reachable at bound k = 12 SATISFIABLE` |
| 10 | 3 | `bad state property 0 reachable at bound k = 13 SATISFIABLE` |
| 10 | **4** | **PASS** |
| 10 | 5 | PASS |
| 15 | 3 / 4 | red / **PASS** |
| 20 | 3 / 4 | red / **PASS** |

**G = 4**, and stable at three different trigger cycles rather than an artefact of one.

**The gap-3 counterexample is also this measurement's own non-vacuity witness.** `liveness` opens
with `assume(rvfi_valid)` at its trig cycle; had that trigger been unreachable, the check would have
PASSed at *every* gap instead of failing at 3, and the sweep would have reported a floor of 0. It
failed where it should, so the trigger is real.

### Where the five stall reasons land

This is the accounting F and G confirm, not a second independent derivation:

```
  base latency      3   decoder_out -> executor_out -> accessor_out/retire
                        (writeback is combinational off accessor_out)
  + reset           1   the DUT's reset is high for cycle 0 only
  + operand fetch   1   ADR-0042's fifth reason; the first instruction of any
                        trace pays it unconditionally (`!read_taken`)
  = unstalled floor 5   the EARLIEST any instruction can retire
  + load or divide  1   ADR-0015's load-response turnaround, or `state == divide`,
                        which ALTOPS collapses to one cycle
  = F               6

  scoreboard        2   ADR-0004's worst-case RAW bubble: a live producer is
                        tracked in decoder_out and executor_out, no longer
  + producer slot   1   accessor_pending (loads only), or the divider's extra
                        cycle of executor_out.valid == 0
  + drain           1   the producer's own retire, which the dependent waits out
  = G               4
```

**Two of the five reasons add nothing to G, and both corrections matter.**

**The operand-fetch cycle is absorbed.** A RAW hazard, a serialization hold and a divider/accessor
freeze all hold the PC, and holding the PC holds the rs1/rs2 address pair with it — so the operand
read completes during the first held cycle and `operand_stall` is already low on the second. This is
measured, not argued: forcing `assign operand_stall = 1'b0` in `rtl/decoder.v` and re-running both
sweeps moves `hang`'s flip point from 7 to 6 and leaves `liveness`'s gap at 4.

| `operand_stall` | F | G |
|---|---|---|
| as shipped | 6 | 4 |
| forced to `1'b0` | **5** | **4** |

So the fifth stall reason costs exactly **+1 on F and +0 on G**. That is the whole of its effect on
this ladder, and it is the reason a change that cost the *suite* 18% of its cycles moves the depth
floors by one.

**Serialization replaces the scoreboard term rather than stacking on it.** `rtl/decoder.v`'s
`pipe_drained` requires all four in-flight slots empty; `live_producer` requires only a matching
`rd`. For a `csrr*`/`mret` the serialization stall therefore **strictly dominates** the RAW stall,
and the two cannot be added. ADR-0025's addendum estimated "+3, additive" and thereby over-stated
both figures by 2.

### The two figures the depths are checked against

```
  single hop  F + G  = 10   one producer, one dependent, both maximally stalled
  two hops    F + 2G = 14   one extra hop of margin
```

ADR-0025's argument for stopping at two hops is unchanged and still governs: `live_producer` reads
three **fixed** slots and is memoryless beyond them, so a third or fourth hop relocates the identical
interaction later in program order rather than reaching new scoreboard state.

Against ADR-0025's numbers: single hop 8 → **10**, two hops 12 → **14**. Of the +2 on each, **+1 is
the operand-fetch cycle** and **+1 is the load/divide term moving from the per-hop column into F**,
where the measurement puts it.

## Verdict, per check family

| line | floor it must clear | margin |
|---|---|---|
| `insn 15` | two-hop **14** | **1** |
| `ill 15` | same configuration as `insn` (one column, `RESET_CYCLES` 1), same floor | 1 |
| `csrw 30` | 14 — a `csrr*` is the serializing case | 16 |
| `reg 15 20` | window 5 against **G = 4** | 1 |
| `pc_fwd 10 30`, `pc_bwd 10 30` | window 20 against G = 4 | 16 |
| `causal 10 20`, `causal_mem 10 20` | window 10 against G = 4 | 6 |
| `liveness 1 10 30`, `unique 1 10 30` | trig-to-check 20 against G = 4 | 16 |
| `hang 1 30` | F + 1 = 7 | 23 |

**Every configured depth clears its floor.** `insn` and `reg` clear theirs by one cycle, which is
thin — `insn`'s margin was 2 before ADR-0042 — and thin is the finding, not a defect.

## The probes

A floor argument is only worth what its liveness evidence is worth. This ladder had a liveness probe
for exactly one check.

### The `insn_*` probe, which did not exist before

**Delete `rtl/executor.v`'s `rs2[4:0]` shift masking** — an already-fixed historical defect, so a
mutation with provenance rather than an invented one. At the shipping `insn 15`:

```
insn_sll_ch0   bad state property 9 reachable at bound k = 15 SATISFIABLE   DONE (ERROR, rc=16)
insn_srl_ch0   bad state property 9 reachable at bound k = 15 SATISFIABLE   DONE (ERROR, rc=16)
insn_sra_ch0   bad state property 9 reachable at bound k = 15 SATISFIABLE   DONE (ERROR, rc=16)
```

It is specific rather than a blanket break: `insn_slli_ch0` (immediate shamt, untouched) and
`insn_add_ch0` stay **PASS** on the same mutation.

**Swept downward on that mutation, the probe exhibits the vacuity failure mode at exactly the derived
unstalled floor:**

| `insn` depth | `insn_sll_ch0` on the mutant |
|---|---|
| 4 | **PASS** — on a broken core |
| 5 | `bad state property 9 reachable at bound k = 5 SATISFIABLE` |
| 6, 7, 8, 10, 12, 15, 20 | red at `k = depth` |

At depth 4 no instruction can retire at cycle 4, so `assume(rvfi_valid && spec_valid)` is
unsatisfiable and the check passes without asking anything. **The flip lands on 5, which is the
unstalled floor the derivation computes.** That is the strongest single piece of evidence in this
ADR: the derived number is not a bound anyone has to take on trust, it is where the ladder measurably
stops seeing.

### `reg_ch0`'s probe, re-run against the five-reason pipeline

ADR-0040 named it and ADR-0042 re-ran it against the new regfile. Re-run again here, because `reg`'s
window is one of the two thin margins:

```
rtl/regfile.v, rs2 write-through bypass deleted, at the shipping `reg 15 20`:
  bad state property 1 reachable at bound k = 20 SATISFIABLE
```

Bad state property **1** is `rvfi_reg_check.sv`'s rs2 assertion specifically. `reg 15 20` still
clears its floor.

## ADR-0042's `imem_data` stability assumption, checked against the probe

An assumption can only make checks easier, and nothing had demonstrated the ladder still catches
things with this one in force. It does, and it is not what makes the catching possible.

The `insn_*` probe was re-run with `formal/wrapper.v`'s same-address stability assumption disabled:

| | assumption in force | assumption disabled |
|---|---|---|
| `insn_sll_ch0` on the mutant, depth 15 | red, `k = 15` | red, `k = 15` |
| ...depth 5 | red, `k = 5` | red, `k = 5` |
| ...depth 4 | PASS (vacuous) | PASS (vacuous) |

**Identical in every cell**: same property, same bound, same floor. The assumption narrows nothing
that any `insn_*` check depends on.

The control that makes this a measurement rather than a no-op: with the assumption disabled, `hang`
at depth 30 reports `bad state property 0 reachable at bound k = 30 SATISFIABLE` — reproducing
ADR-0042 decision 2 exactly, and proving the disabling edit really did weaken the environment.

So the assumption's cost is confined to where ADR-0042 said it was: liveness, and only liveness.

## `components_pcloop` was failing, and nothing ran it

ADR-0017 put the fetcher↔decoder pc loop in **M2's scope** and `formal/pcloop.sv` discharges the
standalone decoder task's `assume(in.pc == pc)` — the one place that proof's content went. There was
no `formal/Makefile` target for it and no CI step. Running it here, as part of deciding whether it
belonged on CI, found it **red on `main`**:

```
failed assertion pcloop._witness_.check_assert_pcloop_sv_273_83 at pcloop.sv:273.7-273.66 step 3
DONE (FAIL, rc=2)
```

Line 273 is the sequential-advance assertion, and the cause is this ADR's subject in a second place:
`f_may_stall` over-approximates the decoder's stall reasons and **predates ADR-0042's fifth one**, so
it called a cycle quiet on which the decoder was legitimately holding the pc for the operand fetch.
Attributed by mutation rather than by reading: forcing `assign operand_stall = 1'b0` in
`rtl/decoder.v` makes the task pass by k-induction.

It has been red since `e4f5250` — the same commit, uncaught for the same reason CI now exists to
prevent. **An M2-scope proof nothing runs is a prose-only guard**, and this is what that costs.

The fix transcribes `rtl/decoder.v`'s `prev_rs1`/`prev_rs2`/`read_taken` register into the harness and
over-approximates the comparison the way every other term there does. It does **not** hollow out the
assertion, and that is checked rather than asserted: the mutation `formal/pcloop.sv`'s own header
names — breaking `rtl/fetcher.v`'s `out.pc = pc` — still fails the task, now on the **echo**
assertion at line 299, which is the property this task exists to add.

| | verdict |
|---|---|
| as shipped, after the fix | `successful proof by k-induction`, `DONE (PASS, rc=0)` |
| `rtl/fetcher.v`'s `out.pc = pc` → `0` | `failed assertion ... pcloop_sv_299_81 ... step 3`, `DONE (FAIL, rc=2)` |

**One engine finding came with it, and it is ADR-0024's finding on a different query.**
`components.sby`'s `[engines] all: smtbmc` is yosys-smtbmc's default solver, yices. That is fine for
the two standalone tasks and *not* fine for this one: yices sat in the basecase at step 6 for over
**fourteen minutes** without returning, where `smtbmc boolector` cleared basecase and induction in
about **five seconds**. Same task, same depth, same RTL. `pcloop` gets a per-task engine line saying
so. The `btor` engine the generated ladder uses is not available here — btormc does `bmc` and `cover`
only, and this task is `mode prove`.

## The empirical sweep

### Baseline, at the configured depths

`make -C formal check JOBS=6`, from scratch (including the `riscv-formal` clone at the pin):

| | |
|---|---|
| generated | **82** |
| PASS | **82** |
| non-PASS set | **empty** — `Failure list matches EXPECTED_FAIL exactly` |
| shape | `Generated check set matches EXPECTED_CHECKS exactly (82 checks)` |
| wall | **3m56s** |

### Raised, at roughly double — **bounded, and it did not finish**

`insn 30` / `reg 25 40` / `pc_fwd`,`pc_bwd 15 50` / `liveness`,`unique 1 15 50` / `causal`,
`causal_mem 15 35` / `hang 1 50` / `ill 30` / `csrw 50`. Generated as a separate config so the
tracked ladder was never edited.

| | |
|---|---|
| generated | **82** |
| returned within the bound | **49** |
| non-PASS among those that returned | **0** |
| outstanding | **33** — `reg_ch0` plus 32 `insn_*` |

**No check's verdict differs between the two configurations among the checks that returned.** That is
the signal this sweep exists to look for, and it did not fire.

It is a *partial* answer and this ADR says so rather than implying otherwise. What it does and does
not cover is worth stating precisely, because "49 of 82" understates the structural coverage:

- **Every consistency check except `reg_ch0` returned** — `causal_ch0`, `causal_mem_ch0`, `hang`,
  `ill_ch0`, `liveness_ch0`, `pc_bwd_ch0`, `pc_fwd_ch0`, `unique_ch0` — as did **all three
  `csrw_*`**. Those are the checks whose depths have distinct *shapes*, and every one of them cleared
  a doubled window.
- **`reg_ch0` at depth 40 did not return**, reproducing ADR-0025's finding exactly. It is the one
  check whose cost is known to explode with depth (ADR-0022, ADR-0024), and depth 21 is where it
  converges in seconds.
- The 32 outstanding checks are all `insn_*`, which are homogeneous: one generated `.sby` per
  instruction over the same wrapper, the same window and the same single-retire property. **38 of
  the 70 returned** at doubled depth, spanning the arithmetic, branch, jump and compressed families
  and `insn_div_ch0`/`insn_divu_ch0` under ALTOPS. The uncompressed shifts, the loads and stores and
  the four `insn_mul*` are in the outstanding set — worth naming rather than glossing, because the
  shift checks are precisely the ones the liveness probe below fires on, and the probe was run at
  the *shipping* depth, not this one.

The sweep was bounded on wall time, not abandoned on a result. Re-running it to completion is a
matter of budget, and the recipe is in this ADR.

**The raised config was not committed, deliberately.** It is a scratch copy of `checks.cfg` with one
`[depth]` table replaced, and the depths are listed above, so it is reproducible in a line. Tracking
it would put a second depth table in the tree that nothing gates and that would silently drift from
the real one — which is the failure mode this whole ADR is about.

**Wall times here are not comparable to an idle machine.** These runs were taken on a contended box —
ADR-0040 already records a 1.7× run-to-run spread on this hardware, and it is worse under load. No
timing comparison below ~2× means anything, which is why the raised sweep is reported by *verdict*
and *count* rather than by duration.

## Decision

**Keep every numeric depth in `formal/checks.cfg` as it is.** Every one clears its re-derived floor;
no check flipped verdict under the raised sweep among those that returned; ADR-0025's rule is that
depths move only on empirical evidence, and the evidence does not demand a move. Raising `insn` from
15 to buy a fatter margin would be raising a depth on a feeling, which is the same act as lowering
one to make a sweep finish.

**Replace the derivation, in place, with the measured one**, including the recipe for re-taking F and
G — a copy of `checks.cfg` carrying one `[depth]` line, `genchecks-local.py`, and a sweep of the last
column. The next person to add a stall reason should re-measure in ten seconds rather than re-trace
in an afternoon or, as happened here, not at all.

**Adopt the shift-mask deletion as the ladder's `insn_*` liveness probe**, alongside the rs2
write-through bypass deletion for `reg_ch0`.

## Consequences

- **`insn 15` and `reg 15 20` each carry one cycle of margin.** That is recorded in `checks.cfg` next
  to the numbers as the thinnest margins on the table. **The sunset condition is explicit: any change
  that adds a stall reason, lengthens a pipeline stage, or widens the scoreboard past its three fixed
  slots must re-measure F and G before it lands.** Two of the last three such changes did not, which
  is why this ADR exists.
- **A green ladder now has a demonstrated way to be red for the `insn_*` family.** Before this, 70 of
  the 82 checks had no probe of any kind, and "82 pass" rested on nothing more than the checks having
  once failed for reasons that were later fixed.
- **ADR-0025's addendum is superseded, not merely updated.** Its "+3 serialization, additive"
  arithmetic double-counted a stall that dominates rather than stacks, and its conclusion — `insn 15`
  at zero margin — was pessimistic by two cycles for a reason that had nothing to do with the fifth
  stall reason that actually eroded the margin.
- **ADR-0025's claim about `` `RISCV_FORMAL_RESET_CYCLES `` is corrected** (§ above). Anyone reasoning
  about a two-column depth line from the old statement will conclude `reg` is vacuous, and it is not.
- **The ALTOPS caveat stands and grows.** Without `` `RISCV_FORMAL_ALTOPS ``, the real divider holds
  `executor_out.valid` for 32 cycles: F becomes ~37 and G ~36, and every number here must be
  re-derived. ADR-0045 decided against dropping ALTOPS; if that is revisited, this table is part of
  the cost.
- **`components_pcloop` joins CI's `components` job**, with a `formal/Makefile` target and a per-task
  `smtbmc boolector` engine line. It is an M2-scope obligation (ADR-0017) that nothing ran, and the
  first thing running it did was find it red. The general rule this is an instance of already has
  five names in this repo — ADR-0019, ADR-0033, ADR-0035, ADR-0037, ADR-0040 — and it is the same
  rule: a gate that can pass without checking anything eventually does.
- **M2 term 1 is met again, and for a stated reason.** ADR-0045 reopened it because the depths were
  unverified against the pipeline that ships. They are verified now — with one cycle of margin in two
  places and a probe that shows the checks can still fail.
