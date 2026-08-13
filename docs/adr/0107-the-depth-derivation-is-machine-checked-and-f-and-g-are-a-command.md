# ADR-0107: The depth derivation is machine-checked, and F and G are a command

**Status:** Accepted · 2026-08-12 · *Mechanises
[ADR-0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md)'s rule, which
until now was a comment nothing evaluated. Confirms
[ADR-0106](0106-the-a-extension-is-built-and-the-board-still-closes.md)'s decision 2 by re-measuring
F and G independently, and its decision 4 by running the exclusion's cover goal. No depth moves, no
RTL moves, and generation is byte-identical.*

## The gap this closes

Every depth in `formal/checks.cfg`'s `[depth]` table is derived from two measured figures — F, the
worst-case first retire, and G, the worst-case gap between two retires. The derivation lived in a
comment beside the table. Nothing computed it, and **a depth below its floor does not fail: the
check runs, finds no counterexample inside a window too short to hold one, and reports PASS.** That
is the one failure mode this repository cannot see from a job colour.

It stopped being theoretical when the atomic write cycle moved G from 5 to 6. Two entries that had
cleared their floor by three now clear it by one, so the next change that adds a stall reason
overruns them — and would find that out from nothing at all.

## Decision 1 — F and G are declared, and every depth is graded against its own rule

`checks.cfg` now carries the two figures in `#`-prefixed lines that genchecks' own cfg parser drops
before it sees a section — the same trick the `#omit` lines have always used, so generation is
untouched:

```
#derive F 6  worst-case first retire, swept out of `hang`
#derive G 6  worst-case gap between two retires, swept out of `liveness`
```

and one `#floor` line per check family, each naming the terms its depth must clear and why:

| family | floor | why that shape |
|---|---|---|
| `insn` `fault` `ill` `csrw` | `F+2G` | one retire, `assume(valid)` at the check cycle, every assertion about it; two hops let it be a third rather than the first out of reset |
| `reg` `pc_fwd` `pc_bwd` `causal` `causal_mem` | `F+G`, `start+G` | two retires, the older shadowed from `RISCV_FORMAL_RESET_CYCLES` |
| `liveness` `unique` | `F+G`, `trig+G` | two retires, the window measured from the trigger cycle |
| `hang` | `F+1` | the first retire alone, asserted on a registered flag |
| `csrc_upcnt` `csrc_any` | `9` | measured directly; CSR instructions serialize, so F and G do not bound these |

`formal/genchecks-audit.py` — already a prerequisite of `make -C formal check` — evaluates each
one. Two things about how it does it are deliberate:

- **It reads the START, TRIG and CHECK cycles back off the `.sby` it has just generated**, not off
  the `[depth]` columns. Which column is which lives in genchecks' `check_cons` call sites, and this
  repo's vendored copy has to stay byte-comparable with upstream, so a second opinion about that
  mapping is exactly the thing that would rot at a pin bump.
- **The `#floor` set and the set of families actually generated are compared both ways**, like every
  other table here. A check family upstream adds fails generation until someone rules on its depth,
  and a `#floor` line for a family that no longer exists fails too.
- **The term vocabulary is closed.** `F+1`, `F+G`, `F+2G`, `start+G`, `trig+G` or a number; anything
  else stops generation rather than evaluating to nothing, because a floor that silently computes
  no bound is the failure this whole decision is about.

**Demonstrated, because a grader that cannot fail is not a grader.** Editing `#derive G 6` to
`#derive G 7` and touching nothing else fails generation and names what needs more depth:

```
[depth] floors: MISMATCH
  fault: depth 19 is below F+2G = 20 -- 1 check(s), e.g. checks/fault_ch0.sby
  ill:   depth 19 is below F+2G = 20 -- 1 check(s), e.g. checks/ill_ch0.sby
  insn:  depth 19 is below F+2G = 20 -- 70 check(s), e.g. checks/insn_add_ch0.sby
```

`ill` is in that list and was in nobody's prose: it has `insn`'s shape exactly — one channel,
`assume(valid && insn == 0)`, assertions about that one retire — so it takes `insn`'s floor. It
already cleared it; it is named now because the arithmetic names it, not because anyone remembered.

**With F and G unchanged, generation is byte-identical**: `diff -r` between the check set built from
`origin/main`'s `checks.cfg` and from this one reports nothing, and `EXPECTED_CHECKS` matches both
ways at 86.

## Decision 2 — the re-measurement is a target, not four sentences

`make -C formal remeasure-fg` sweeps `hang` and `liveness` out of a one-line copy of `checks.cfg`
and grades both flip points against the `#derive` lines. Six sby runs, 17 seconds.

**It prints the red rows as well as the green one, and that is the point.** A flip point with no
counterexample under it is a measurement that could not have come out any other way. The gap-5
counterexample is also this measurement's non-vacuity witness, since `rvfi_liveness_check.sv` opens
with `assume(rvfi_valid)` at its trig cycle.

```
F -- worst-case first retire, from `hang`'s check cycle:
  check cycle 4  FAIL     check cycle 5  FAIL
  check cycle 6  FAIL     check cycle 7  PASS      => F = 6
G -- worst-case retire gap, from `liveness` at trig 10:
  gap 4 FAIL   gap 5 FAIL   gap 6 PASS   gap 7 PASS => G = 6
G -- the same at trig 15:
  gap 4 FAIL   gap 5 FAIL   gap 6 PASS   gap 7 PASS => G = 6
```

**Both reproduce ADR-0106's numbers exactly**, on merged main, taken under the interrupt tie-off the
way the last two were — the probe cfg is `checks.cfg` with one line changed, so the solver, the
defines and `formal/wrapper.v`'s tie-off are the graded run's. G is only this number under that
tie-off: an interrupt costs a cycle that would otherwise have issued.

**It is not on CI and adds no ratchet.** What CI enforces is that the depths clear the *declared*
figures, which costs a second inside `make -C formal checks`. This target is what says the
declaration is still true of the design, and it is what a change that adds a stall reason or
lengthens a stage runs before landing. Two graders, two questions.

## Decision 3 — A's formal boundary is confirmed where it was recorded, not re-litigated

ADR-0106 landed the `AMO 0101111` line in `COMPLETE_EXCLUSIONS`, the encoding-keyed predicate in
`complete.sv` with its reason at the site, and the thirteenth cover goal. All three are re-run here
rather than rebuilt:

- `make -C formal complete-exclusions` accepts all eleven mnemonics, re-derived from the clone at
  the pin — the direction that means something, since the script refuses a mnemonic that *has* a
  model.
- `make -C formal complete_cover` reaches all thirteen goals at step 5. The AMO goal's trace
  (`complete_cover/engine_0/trace0.vcd`) retires `rvfi_insn = 0x1fffafaf` with `rvfi_trap` low:
  opcode `0101111`, funct3 `010`, funct5 `00011` — a `sc.w.aqrl` that failed and wrote 1. So the
  exclusion is a live restriction on a class the core really reaches, not a line about instructions
  nobody executes.
- `checks.cfg` stays `isa rv32imc`, and the reason is now at that line rather than only in the
  exclusion file. Measured rather than argued: `isa rv32imac` generates **16 checks instead of 86**,
  gaining no atomic and losing all 70 `insn_*`, because genchecks catches the `FileNotFoundError`
  from the missing `insns/isa_rv32imac.txt` around the whole instruction loop.

## Decision 4 — `rvfi_dmem_check` is not extended, and `dmemcheck.sv` gets the sentence instead

Checked in the clone: `rvfi_dmem_check.sv` asserts `dmem_shadow == rvfi_mem_rdata` for each `rmask`
byte *before* applying `wmask` to the shadow, with blocking assignments, so an AMO retire carrying
both masks full is already checked old-read against new-write. There is nothing to add.

What was worth confirming is the harness around it. `formal/dmemcheck.sv`'s one-address bus model
sees an AMO's two transactions in adjacent cycles, and the second is a write to the address the
first read — so its shadow update and its read-back assume fire in the same time step. The read sees
the old word because the update is non-blocking. That was true and unstated, and one clause of the
comment beside it ("`rtl/accessor.v` has already put `mem_addr` back to 0 by then") had stopped
being true for an AMO. Both are fixed at the site.

**Its red direction was re-forced rather than quoted**: reporting the read word as an AMO's
`rvfi_mem_wdata` (`take_amo ? mem_rdata : ...` in `rtl/accessor.v`) takes `make -C formal dmemcheck`
from PASS to FAIL at `rvfi_dmem_check.sv:37`, and restoring it passes again.

## What it cost

Nothing measurable, and nothing to measure: no RTL changed, no depth moved, no check was added or
removed.

| gate | result |
|---|---|
| `make -C formal check` | **86 checks, 86 pass**; `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact both ways |
| generation vs. `origin/main`'s cfg | `diff -r` reports no difference |
| `#derive G 7` with `[depth]` untouched | generation fails, naming `insn`, `fault` and `ill` |
| `make -C formal remeasure-fg` | F = 6, G = 6, both flip points red-then-green, both trigs agreeing |
| `make -C formal complete` / `complete_cover` | PASS / PASS, 13 of 13 goals reached |
| `make -C formal complete-exclusions` / `interrupt-tie-off` | PASS / PASS |
| `make -C formal dmemcheck` / `imemcheck` / `nonperturbation` | PASS / PASS / PASS |
| `make -C formal components_{decoder,executor,accessor,pcloop,traps}` | all PASS |
| `make test` | 64/64, baselines exact both ways |

The `formal` job's runtime is unchanged, because its input is: the same 86 checks at the same 86
depths. It measured 5m36s on the change that landed the A datapath, against `timeout-minutes: 20`.

## Consequences

- **A stale F or G is now a build failure**, and the failure names the `[depth]` entries that need
  more depth rather than the figure that moved. Depths are still never trimmed to fit a runtime
  budget; the diagnostic says so where someone under time pressure will read it.
- **`ill` joined the two-hop family** on its shape, and cleared its floor without being touched.
  Writing the rule down found a check the prose had never classified.
- **The vocabulary is the limit.** A future check whose floor is not one of five terms or a measured
  number stops generation until someone extends `formal/depth_rules.py` with what the new term
  means. That is deliberate: the alternative is a `#floor` line that reads as a rule and evaluates
  to nothing.
- **What is still not machine-checked is the classification itself** — that `reg` is a two-retire
  check and `insn` a one-retire one is read out of the pinned check sources by a person, and a pin
  bump that changes a check's shape would leave a floor that computes the wrong bound. The pin-bump
  procedure already re-derives `COMPLETE_EXCLUSIONS` and the sanitizer's site counts; this is one
  more thing on that list.
