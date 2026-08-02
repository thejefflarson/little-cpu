# ADR-0053: Every graded comparison carries an executable probe of its own red direction

**Status:** Accepted · 2026-08-01 · *Extends ADR-0035 (the merge gate's false-green audit) and
ADR-0033 (a check that can stop checking without anything going red) to the whole grading layer.
Applies ADR-0037 §4's rule about pipelines. Records one false green found in
`formal/check-baseline.sh` and one unreachable failure path found in `test/cxxrtl.cc`.*

## Context

Five of this repository's recorded defects live in the layer that decides what "green" means, and
**every one of them was in a script**:

1. the graded comparison piped into `tee`, so a `run:` block's errexit — which is not pipefail —
   took `tee`'s status and the gate could not go red. ADR-0022's central guarantee had therefore
   never held (ADR-0037 §4);
2. `make -C formal check` re-grading the previous run, because its `checks` target named a
   directory whose mtime `sby` bumps (ADR-0040);
3. `formal/check-baseline.sh` globbing the run directories `sby` creates only for checks it
   started, so a never-scheduled check fell out of the results and out of the baseline at once and
   set equality called that a match (ADR-0033 gap 1);
4. `test/sanitize_monitor.py`'s rule 3, whose site count proved a rule *fired* but not what it
   *swallowed* — mutation showed the pre-change script accepting every attack at exit 0;
5. `formal/check-baseline.sh` computing a verdict and discarding it.

The class is **the comparison whose failure path was never executed**. Each of the five was green,
in CI, for weeks or months, while checking less than it claimed. In every case the reasoning
written at the site was *correct*; what was missing was that nobody had ever made the comparison
fail. Reading the script is what missed all five.

Three questions were put to every graded comparison in `test/run_tests.sh`, `test/run_cosim.sh`,
`test/cosim.py`, `test/cosim.cc`, `test/cxxrtl.cc`, `test/check_suite_shape.sh`,
`formal/check-baseline.sh`, `formal/check-complete-exclusions.py` and
`formal/genchecks-audit.py`:

1. does every failure reach the process exit status?
2. is any graded command inside a pipeline?
3. **has the failure path ever been executed?**

Questions 1 and 2 came back clean. Every pipeline in those scripts feeds a `$(...)` whose *value*
is then compared; the comparison itself is never a pipeline. Both workflows keep the graded command
off a pipe with the reasoning written at the site, and the one `| tee` in `.github/workflows/ci.yml`
(the `elaborate` job's yosys invocation) sits under an explicit `set -euo pipefail` and is not the
graded command — the `grep` on the next line is.

Question 3 is where the work was, and it produced two findings.

## Decision

### 1. `test/probe_gates.sh` — 108 probes, and it runs

Every graded comparison that can be driven without the elaborated design is **forced to fail**, and
required to fail *for the reason it was written for*. A probe pins the exit **status** and a
distinguishing fragment of the **diagnostic**. The second half is not decoration: a script that
exited 1 because a fixture path was mistyped would satisfy every exit-status-only probe in the file
while demonstrating nothing.

**Every group carries a control** — the same fixtures, unmutated, required to exit 0. Without one, a
grader that had degenerated into `exit 1` would pass every probe. That is this file's own
anti-vacuity argument and it is the same one `formal/complete_cover.sby` makes for `complete`.

The file is **hermetic**: no RISC-V toolchain, no `sim`, no Sail, no yosys, no `sby`. The failing
`objcopy`, the `objcopy` that exits 0 having written nothing, the unstartable runner, the missing
cross compiler, the reference model that diverges and the one that produces no trace are all stubs
on a scratch `PATH`. That is what lets it hang off `make test` without narrowing where `make test`
runs.

**It is a prerequisite of `make test` rather than a target of its own.** A gate reached by no
automation reads like coverage and is not — `CLAUDE.md` records `make waves` and
`make -C formal all` as exactly that — and hanging it off `test` puts it inside CI's required
`test` job with no workflow change and no branch-protection change, the latter being a human action
(ADR-0036).

The probe count is pinned as a literal, for the reason `test/exec_tb.v` pins its vector count: a
deleted probe would otherwise shrink this file's coverage while it kept printing a green summary.

### 2. `formal/check-baseline.sh` accepted an unreadable baseline as an empty one. Fixed.

The script sets `set -u` and neither `-e` nor `pipefail`, and it checked its two baseline files for
existence (`-f`) but not readability (`-r`). A `sed` that cannot open its input writes to stderr and
yields the **empty string**, and an empty expected set against an all-passing ladder matches
exactly. Measured before the fix:

```
$ chmod 000 EF && formal/check-baseline.sh $D/checks $D/EF $D/EC
sed: .../EF: Permission denied
1 checks: 1 pass, 0 fail
Failure list matches .../EF exactly (name and status).
exit=0
```

— with `alpha FAIL` in that baseline and `alpha` passing on disk, so it also swallowed an
**unexpected pass**, which ADR-0014's contract exists to catch. After the fix the same command
exits 2 naming the file. This is ADR-0035 item 4's fix, made on `test/run_tests.sh`'s baseline and
never carried across to the formal side; `test/run_cosim.sh` and `test/check_suite_shape.sh` both
already check `-r`.

### 3. `test/cxxrtl.cc`'s exit 5 was unreachable in the scenario it was written for. Fixed.

`finish()` routes the run's verdict through the observation counters, so a run with zero retires
returns 6 (`MONITOR-SILENT`) whatever else happened. Exit 4 was already exempted, on the argument
that a monitor errcode is *direct evidence the monitor fired* rather than a verdict the program
reached. **The identical argument applies to exit 5** and was not made: `trap_to_zero` is a direct
observation of the machine that owes nothing to the monitor.

It is not hypothetical. Traps are detected and committed in **decode** (invariant 2) while a retire
happens in writeback, so a fault in the first few instructions of `_start` raises `trap_to_zero`
several cycles before anything has retired. Measured:

| program | before | after |
|---|---|---|
| `ecall` as the second instruction of `_start` | exit **6**, `RETIRES 0` | exit **5** |
| the same `ecall` after six retiring instructions | exit 5, `RETIRES 6` | exit 5 |

`test/run_tests.sh` labels the first case `MONITOR-SILENT` — pointing at the monitor, for a program
that faulted before installing `mtvec`, which is precisely the misattribution ADR-0029 added the
`TRAP-TO-ZERO` label to prevent. A named failure path that is unreachable in its own scenario is
this repo's grading-layer defect one level down.

The silence gate is not weakened in any direction that matters: both outcomes are red either way,
and the change only decides which of two red labels a reader gets.

## What is NOT probed, and why

Named rather than left to be discovered, per ADR-0033's rule that a guard which is named but absent
is worse than no guard:

- **`test/cxxrtl.cc` exit 4** (a live RVFI monitor mismatch) needs the elaborated design.
  Demonstrated by hand instead: `test/testbench.v`'s monitor hookup mutated to
  `.rvfi_rd_wdata(rvfi_rd_wdata ^ 32'd1)`, rebuilt, `add.S` reported `RVFI monitor error` and
  exited 4. Reverted.
- **`formal/genchecks-audit.py`'s two self-validation cross-checks** — the traced names against
  `genchecks`' own `consistency_checks`/`instruction_checks` sets, and the traced names against
  `checks/*.sby` on disk. Both would need `formal/genchecks-local.py` to behave differently from
  the pin, and ADR-0031 holds that file byte-comparable with the clone. Placing a stray `.sby` in
  `checks/` does not reach the disk cross-check either, because `genchecks` recreates the directory
  on every run — measured. Its other two set equalities *were* demonstrated by hand (below).
- **`test/cosim.cc`'s `regs_a` / `regs_b` divergence check** needs an `rtl/regfile.v` mutation.
  Recorded as a finding rather than demonstrated here.
- **`formal/genchecks-audit.py`'s "traced no `get_depth_cfg` calls" guard**, for the same
  ADR-0031 reason.

Demonstrated by hand, with the commands recorded in the pull request: `genchecks-audit.py`'s
`EXPECTED_CHECKS` set equality (delete `insn_add_ch0` from the file → `in EXPECTED_CHECKS but not
generated`), its `#omit` set equality and the lost-`[depth]`-line case together (delete
`checks.cfg`'s `hang` line → both mismatches at once, 84 generated / 15 declined), and
`test/cxxrtl.cc`'s exits 0, 1, 2, 3, 5 and 6 against the real runner.

## Consequences

- `make test` gains a step that can fail for a reason unrelated to the core, exactly as
  `check-unit-benches` already can. It is **all fork and no work**, so its wall time is a property
  of the host: ~1500 `exec()`s, about 4s of user time, and 90s of wall on the laptop it was written
  on, which takes ~27ms to `exec` `/bin/echo`. Do not shrink it by deleting probes without first
  measuring whether the host is the reason.
- A grader's **diagnostic text is now part of its contract**. Rewording an error message breaks a
  probe, deliberately: the message is how a probe knows which comparison went red, and this repo
  has twice been bitten by a comparison that failed for the wrong reason.
- `formal/EXPECTED_FAIL` and `formal/EXPECTED_CHECKS` must now be readable, not merely present.
- One asymmetry is left in place on purpose: `test/run_tests.sh` iterates `"$ASM_DIR"/*.S` with no
  `nullglob`, where `test/run_cosim.sh` has both `nullglob` and an emptiness guard. The unglobbed
  literal fails `ASSEMBLE-ERROR`, so it fails **red** — and since ADR-0033's manifest check now runs
  before the loop, an empty or missing `test/asm` is rejected before the glob is ever evaluated.
  Both facts are probed; neither is worth a change.
- This does not make any gate *correct*, only harder to fool. Every caveat in ADR-0023 and
  ADR-0037 stands untouched, and nothing here changes `rtl/`.
