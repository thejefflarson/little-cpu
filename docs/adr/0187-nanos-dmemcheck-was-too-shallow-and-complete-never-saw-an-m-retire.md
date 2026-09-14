# ADR-0187: nano's dmemcheck was too shallow, and `complete` never saw an M retire

**Status:** Accepted · 2026-09-13

## Context

A security audit of PR #352 found three ways nano's (and, for item 3, littlecpu's)
formal checks could report PASS without evaluating the property they name -- the same
shape as the five recorded defects CLAUDE.md's "a grader that cannot fail is not a
grader" rule already names.

## dmemcheck's depth

`nano/formal/dmemcheck.sby` needs two retires -- a store, then a matching load -- to
reach its one assertion. F and G are step indices, read by `remeasure-fg.py` off
generated checks whose scripts end in `chformal -early`, which checks a clocked assert on
the step it samples. The hand-written memcheck scripts do not run it, so
`rvfi_dmem_check`'s clocked assert fires the step *after* the load retires, and
`depth N` covers steps 0 to N-1. The floor is therefore `F+G+2` for dmemcheck and `F+2`
for imemcheck: at nano's `F=12, G=10`, 24 and 14, against the 15 dmemcheck shipped at.
**Found at integration**: this ADR first floored at `F+G+1` = 23, one short. A
small model in the generated checks' shape -- a clocked `assert` gated on
`cycle == 3` -- is silent at depth 4 and red at depth 5 without `chformal -early`, and red
at depth 4 with it. `formal/check-memcheck-depth.py` reads a harness's
`checks.cfg` `#derive` lines the way `genchecks-audit.py` reads them for the generated
family checks, and grades `dmemcheck.sby`/`imemcheck.sby`'s own `depth` line against
`F+G+2`/`F+2` -- neither is genchecks-generated, so neither harness's `[depth]` table
ever saw them. Wired as a Makefile prerequisite of both targets on both cores and into
`make test` as `memcheck-depth-test`. littlecpu's own `dmemcheck.sby`/`imemcheck.sby`
(`F=G=6`, floors 14/8) were already above both floors at 15; nano's `imemcheck.sby` was
already above its floor (14) at 15 too -- only nano's `dmemcheck.sby` was short. Runtime
at depth 24: 11s (Apple Silicon, one sby job at a time), against 9s at 23 locally;
`nano/formal`'s `check` job already budgets minutes for the generated set, so this adds
one job well inside the CI pods' 4 CPU / 3Gi.

## `complete` never saw an M instruction

ADR-0181 already named this in passing: "`complete`'s own depth (20) is short of the
real divide loop's latency either way, so it never observed either the broken or the
fixed sequencer completing" -- true of every M instruction, not only the divider.
`nano/formal/complete.sby`/`complete_cover.sby` now define `RISCV_FORMAL_ALTOPS`, which
collapses nano's multiply/divide state to one cycle (the same substitution the generated
checks already run under), so an M retire lands at the same worst case any other
instruction does. `complete.sv` gained a thirteenth cover goal, `insn_is_m` (opcode
`0110011`, funct7 `0000001`), reached at step 8 against `complete`'s depth of 20 --
`complete_cover` proves all thirteen reachable within that depth (13.0s locally),
`complete` itself is unaffected by ALTOPS (it asserts only `spec_valid`/`!spec_trap`,
which ALTOPS's spec model preserves) and runs in 3.7s.

## The depth tie and the memcheck cover goals

`complete_cover` searches depth 100 against `complete`'s own 20, so a goal reachable
only between steps 20 and 99 would pass the anti-vacuity control while `complete` never
examines that retire. `nano/formal/complete-cover-probe.py` now reads `complete.sby`'s
own depth and reds if any cover site's first-reached step is at or beyond it -- reusing
the per-step log line `sby` already writes, not a second BMC run.

Neither memcheck, on either core, had a cover goal proving it reaches its own assertion,
or a forced-red direction -- `complete` had `complete_cover` and
`complete-cover-probe.py` for exactly this; the memchecks had nothing.
`formal/memcheck-cover-probe.py` generalises `complete-cover-probe.py`'s shape over both
memchecks and both cores: one mutant, one line from the shipping harness --
`assume(fetch_stall)` on littlecpu (nothing ever issues), `assume(mem_ready == 1'b0)` on
nano (the bus never completes) -- and either way `rvfi_valid` never rises, so the cover
goal must go unreached. `imemcheck_cover.sby`/`dmemcheck_cover.sby` (`mode cover`) are
new on both cores; unlike `complete_cover`'s many opcode-class goals, each has exactly
one goal, so its own depth tie is a plain equality against its `mode bmc` sibling's
depth rather than a per-goal reachable-step walk. That equality is exact only because
each memcheck cover is itself a clocked `cover`, fired a step after what it samples just
as the assertion it backs is; a module-level `cover property` is reached a step earlier,
so a goal first reached on a bmc job's last step would have passed with the assertion
unchecked. The goals are reached at steps 5 and 6 (littlecpu imem, dmem) and 7 and 13
(nano) -- `check-memcheck-depth.py` grades
both floor and tie in one pass, so a future F/G re-measurement that moves a `.sby`'s
depth cannot silently reopen the vacuity hole this fix closes. All four combinations
pass in 1-3s each; the four Makefile prerequisites (one probe per memcheck per core)
add under 15s combined.

`formal/depth_rules.py` gains `read_sby_depth()` and the `F+2`/`F+G+2` terms, so
`check-memcheck-depth.py` and `complete-cover-probe.py` share one depth-line parser and
one floor evaluator with `genchecks-audit.py` rather than each carrying its own copy.

## Consequence

`nano/formal/dmemcheck.sby`'s depth is 15 -> 24. `nano/formal/complete.sby` and
`complete_cover.sby` gain `RISCV_FORMAL_ALTOPS`; `complete.sv` gains one cover goal.
Four new `_cover.sby` files and one new shared probe script
(`formal/memcheck-cover-probe.py`) plus one shared depth grader
(`formal/check-memcheck-depth.py`) cover both cores. Twenty-six new labels in
`test/PROBES_EXPECTED`, all forced red at least once. No RTL changed on either core;
`nano/nano.v` and every `rtl/*.v` file are untouched.
