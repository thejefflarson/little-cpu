# ADR-0079: All six M2 terms are re-measured against merged main, and M2 is declared

**Status:** Accepted · 2026-08-08 · *Rules on [ADR-0037](0037-an-empty-baseline-is-not-m2.md)'s six-term
M2 conjunction as amended by [ADR-0045](0045-two-m2-terms-close-by-amendment-and-one-was-already-met.md)
(terms 2, 3), [ADR-0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md)
(term 1), [ADR-0047](0047-non-perturbation-is-proved-structurally-and-equiv-sh-is-retired.md) (term 4),
[ADR-0050](0050-the-nightly-is-deleted-and-its-checks-move-to-the-gate.md) (term 6),
[ADR-0051](0051-the-multiply-proof-is-decomposed-not-mitered.md) (term 2 again) and
[ADR-0052](0052-m2-term-6-is-verified-and-the-fit-ratchet-gets-a-job.md) (term 6). Narrows
[ADR-0027](0027-minstret-counts-non-trapping-issues.md) rather than closing it. Amends `CLAUDE.md`'s
M2 text. No `rtl/`, `formal/` or `test/` file changes.*

## Why this audit exists at all

ADR-0037 exists because a milestone table drifted from what it measured. ADR-0045 then found the
same drift **inside ADR-0037** — term 3 was met and the document called it inconclusive. Term 2 has
since regressed once, because ADR-0045 closed it against an oracle nobody had run a mutation
against, and ADR-0049 measured that closure false in an afternoon.

The failure mode is consistent, and it is not carelessness: **a term stays marked met on a
measurement two changes stale, because the terms already ticked are exactly the ones nobody
re-checks.** ADR-0052 closed the last open term and declined to declare the milestone, correctly,
leaving the declaration as a human call on complete evidence. But that evidence was a week of
separate runs against separate trees, each closing one term and inheriting five. This ADR takes all
six in one sitting, against merged `main` at `0b1728a`, and re-runs every one — terms 1, 3 and 4
included, which are precisely the ones nobody was going to look at again.

## The six terms

Everything below was measured on 2026-08-08 against `0b1728a` with a **clean worktree**
(`git status --porcelain` empty before and after), on macOS with Homebrew Yosys 0.68+post
(`c12172fb`), `sby`, `btormc`, `iverilog` and `riscv64-elf-gcc`.

| # | term | verdict | command | wall | date |
|---|---|---|---|---|---|
| 1 | `formal/EXPECTED_FAIL` empty **and** `formal/EXPECTED_CHECKS` matching | **HOLDS** (re-run) | `make -C formal check` | 463 s | 2026-08-08 |
| 1b | …and the depths still clear their derived floor | **HOLDS** (re-derived) | `genchecks-local.py` on a `hang`-only and a `liveness`-only `checks.cfg` copy | 45 s, 13 runs | 2026-08-08 |
| 2 | a named oracle covers the real multiplier and divider | **HOLDS** (re-run, both oracles, both falsified) | `make -C formal components_executor`; `make test-units` | 24 s; 5 s | 2026-08-08 |
| 3 | `reg_ch0` returns a verdict rather than exhausting its budget | **HOLDS** (re-run + standing liveness probe) | within `make -C formal check`; probe below | 463 s; 22 s | 2026-08-08 |
| 4 | non-perturbation is proved | **HOLDS** (re-run) | `make -C formal nonperturbation` | 8 s | 2026-08-08 |
| 5 | `complete` passes, or every check it declines has a recorded reason | **HOLDS** (gate evidence; local run blocked by the toolchain) | `make -C formal complete` (ERROR, environmental); `complete_cover`; `complete-exclusions`; gate run `31136230025` step 11 | 2 s; 14 s; <1 s; 24 s | 2026-08-08 / 2026-08-07 |
| 6 | every check the repo owns is on a gate that can fail, and that gate is green | **HOLDS** as read, two findings | `gh api …/branches/main/protection`; runs `31136230025` and `31135672684` | — | 2026-08-08 |

Supporting runs, all 2026-08-08, all exit 0: `make -C formal components_decoder` `PASS 0 6` (6 s),
`components_pcloop` `PASS 0 1` (1 s), `components_traps` `PASS 0 2` (2 s), `imemcheck` PASS (98 s),
`dmemcheck` PASS (50 s), `cover` PASS (6 s), `complete_cover` PASS (14 s), `make test` 59/59 (46 s),
`make test-units` (5 s), `make cosim-suite` 59/59 agreed (33 s), `make cycles` 28632 cycles / CPI
2.07 / **0 unattributed** (3 s), `make -C formal genchecks-check` matching the pin (<1 s).

### Term 1 — the two set equalities, and the floor underneath them

`85 checks: 85 pass, 0 fail`, *Failure list matches `EXPECTED_FAIL` exactly (name and status)*,
*Generated check set matches `EXPECTED_CHECKS` exactly (85 checks)*.

That is the part everyone re-runs. The part nobody does is the one ADR-0045 reopened the term over
and ADR-0046 closed: **a depth below its floor does not fail, it goes green having stopped asking.**
So F and G were re-derived here by `checks.cfg`'s own recipe rather than read off `checks.cfg`:

| sweep | result |
|---|---|
| `hang` at depth 4, 5, 6 | counterexample |
| `hang` at depth 7, 8 | PASS |
| `liveness` trig 10, gap 4 / 5 | counterexample |
| `liveness` trig 10, gap 6 / 7 | PASS |
| `liveness` trig 15, gap 4 / 5 | counterexample |
| `liveness` trig 15, gap 6 / 7 | PASS |

`rvfi_hang_check.sv` asserts on a registered flag, so the flip at 7 gives **F = 6**; the flip at gap
6, at both trig points, gives **G = 6**. Identical to the figures `checks.cfg` records, and the
`[depth]` table still clears the single hop F + G = 12 and the two hops F + 2G = 18 — with `insn 19`
and `reg 15 22`'s 7-cycle window still clearing by exactly one cycle, which is the thinnest margin
in the file and stays the first thing to re-measure after any stall-reason change.

### Term 2 — third move, and the reason the third one stuck

**The term has moved three times.** ADR-0037 wrote it with an "or" clause; ADR-0045 took the clause
and named `components_executor` + `test/exec_tb.v` as the oracle; ADR-0051 re-closed it after
ADR-0049 measured ADR-0045's central sentence false. ADR-0045's own closing line says a third move
should prompt asking whether the criterion describes anything real.

It does, and the difference is not rhetorical. **The first two closures were claims about oracles
that existed; the third ships mutation tables for both named oracles** — eleven mutations each, with
the six that ADR-0049 had measured *passing* the pre-change proof now caught. A term that closes
without its mutation table is not closed; that is the rule the third move established, and it is why
this audit spot-checked it rather than reading it.

Both oracles re-run, and both falsified here rather than inherited:

- `components_executor` — `PASS 0 26`, *successful proof by k-induction*, 24 s.
- Masking the product's high half (`multiply & 64'h00000000ffffffff`, ADR-0051's M5) on a scratch
  mutation of `rtl/executor.v` → **`DONE (FAIL, rc=2)`**, *failed assertion … at `executor.v:335`*,
  which is decomposition lemma 3 — the only obligation in the multiply section that constrains the
  product term's own value. Restored immediately; the tree is untouched.
- `test/exec_tb.v` runs green inside `make test-units`.

### Term 3 — the verdict, and the probe that says the verdict means something

`reg_ch0` returned `PASS` inside the ladder run above. On its own that is the weaker half of the
term: `CLAUDE.md` requires the standing liveness probe **before believing any `reg_ch0` result under
a changed configuration**, and a Homebrew Yosys 0.68 workstation is a changed configuration against
CI's pinned suite.

Run: delete the rs2 write-through bypass from `rtl/regfile.v` and `reg_ch0` reports
`bad state property 1 reachable at bound k = 22 SATISFIABLE`. The check has a live red direction at
its configured depth. Restored immediately.

### Term 4 — re-run, unchanged

`make -C formal nonperturbation` exits 0 in 8 s: the `-D RISCV_FORMAL` build with its `rvfi_*` ports
deleted sweeps to a netlist structurally identical to the plain build. That proves the
instrumentation is **unread** — strictly weaker than sequential equivalence, strictly stronger than
a bounded miter.

### Term 5 — the local run cannot execute, so the term is taken from the gate

`make -C formal complete` fails here in model preparation, before any solving:

```
aig: ERROR: Command syntax error: Unknown option or option in arguments.
aig: > abc -g AND -fast
```

`complete.sby` selects `abc bmc3`, whose AIGER lowering passes `-g AND -fast` to `yosys-abc`;
Homebrew's build rejects it. **Reproduced on the unmodified tree, so it is the toolchain, not a
regression** — `complete_cover`, which runs `smtbmc`, passes here in 14 s over the same
`complete.sv`.

Substituting `smtbmc z3` for the configured engine on a scratch copy was tried and is not a route:
it reached **step 16 of the depth-50 walk in 17 m 08 s** and was abandoned. A depth-50 whole-ISA
walk is what `abc bmc3` is in that file for.

So the term's evidence is the pinned toolchain's, and it is not stale: run `31136230025` is
**`main`'s own push run at `0b1728a`**, and its `formal` job step 11, `complete (whole-ISA BMC walk,
depth 50)`, succeeded in 24 s, with `complete_cover` succeeding in 16 s behind it. The half of the
term this workstation *can* check was checked: `make -C formal complete-exclusions` passes with the
exclusion set matching `complete.sv` in both directions, two entries — MISC-MEM (`fence`, `fence.i`)
and SYSTEM (`ecall ebreak mret wfi csrrw csrrs csrrc csrrwi csrrsi csrrci`), each declined because
riscv-formal ships no spec model for the encoding at the pin.

**This is not the "or" clause being taken a second time.** Both halves hold independently:
`complete` passes, *and* everything it declines is recorded and set-equality-checked. What is
unavailable is a workstation reproduction of the first half.

### Term 6 — the mechanism changed, so it is shown satisfied by the new one

Term 6's mechanism moved from a scheduled nightly (ADR-0022) to a required PR check (ADR-0050), and
ADR-0052 verified the new mechanism against the gate's own run. **A criterion that changed mechanism
has to be shown satisfied by the new one, not re-asserted**, so the new one was read live rather
than from comments:

- `gh api …/branches/main/protection` returns seven required contexts — `elaborate`, `test`,
  `components`, `monitor-freshness`, `lint`, `formal`, `soc-timing` — out of nine CI jobs.
- `grep -c continue-on-error .github/workflows/` is **0 in every file**, which is the line ADR-0052
  found unmet and fixed.
- The `formal` job carries the ladder, its baseline gate, `imemcheck`, `dmemcheck`, `cover`,
  `complete` and `complete_cover` as seven separate steps; the `components` job carries all four
  `mode prove` proofs as four separate steps, `components_traps` included.
- The merge gate for `main`'s content ran green. The branch head it admitted, `c0de20a6`, carries
  tree `24e8f436` — **byte-identical to `0b1728a`'s tree** — and run `31135672684` reports all nine
  CI jobs successful, the last of them at 00:53:38Z on 2026-08-07.

## The CI outage is an outage, not a term-6 failure — and here is the argument

At the time of this audit the self-hosted pool has **zero registered runners**
(`gh api …/actions/runners` → `total_count: 0`). Eight of the nine CI jobs are
`runs-on: little-cpu-runners`; only `formal` is GitHub-hosted, and it still completes — on today's
11:51Z run the `formal` job succeeded while the other eight sat queued. **A required check that
cannot run is, right now, a gate that cannot fail**, and the term says every check the repo owns is
on a gate that can fail. That reading deserves an answer rather than a shrug.

The answer is that **"can fail" in this repo's usage means "is not suppressed", not "is
available"**, and the outage is not an instance of the defect the term was written against. Every
one of the recorded instances is a **false green**: `make -C formal check || true`; a graded command
piped into `tee` under errexit-without-pipefail; `equiv.sh` behind `continue-on-error` reading as
coverage for months; the nightly's own baseline comparison that could not go red for its entire
life; `continue-on-error: true` on the step that ran the whole ladder. In each, a change reached
`main` with a check reporting success on work it had not done.

An empty runner pool produces the opposite shape. A queued required check is not `success`, branch
protection admits only `success`, and so the gate is **fail-closed**: no unverified change can reach
`main` through it. Nothing is laundered. The instrument is unavailable, which is a bad day; it is
not a criterion that has stopped meaning anything.

**Two things the outage does surface, and they are findings rather than colour.**

1. **`main`'s head commit does not have a green push run.** Run `31136230025` at `0b1728a`
   concluded **failure**: `elaborate`, `fit`, `formal`, `lint` and `monitor-freshness` succeeded,
   and `test`, `components`, `soc-timing` and `nonperturbation` were **cancelled** on 2026-08-08
   after roughly 23 hours in queue. Three of those four are required. So "the gate is green" is
   currently true of the **merge gate** and false of **`main`'s own post-merge run**, and no
   document here had ever had to distinguish them. The merge gate is the load-bearing one — branch
   protection is `strict`, so the verdict that admits a commit is taken on the up-to-date branch,
   and the push run is a duplicate of a check already passed on an identical tree. Term 6 is read
   against the merge gate, and that is written down here so the next reader does not have to
   re-derive which run counts.
2. **Term 6 is the only one of the six that cannot be re-measured from a workstation**, because its
   subject *is* CI. Every other term was re-run locally today. That asymmetry is exactly what makes
   term 6 the one most likely to go stale next, and an outage is exactly the condition under which
   it matters.

## What M2 does not mean

All six terms hold, so **M2 is declared**. The residuals are named in the same breath, because M2's
own wording is *"the pipelined core re-proves everything the serialized core proved"* and the
serialized core's result was not bounded, not ALTOPS and not self-refereed.

- **The multiplier is checked differentially, not exhaustively.** There is no miter: the proof
  establishes the 33-bit operands, the retired slice, and three points of the product function
  (`rs1 == 0`, `rs2 == 0`, `rs2 == 1`). What covers the rest of the function is `test/exec_tb.v`'s
  randomized vectors. A standalone miter of two `bvmul` terms does not return at depth 1 with the
  pipeline removed, so no bound buys it.
- **The divider is proved under a recorded magnitude restriction** — `div_x`/`div_y` capped to
  −15..15, sign free — and its four completion assertions are **basecase-unreachable at the
  configured depth**: the real divider needs 33 cycles from issue and `components.sby`'s basecase
  runs 20 steps, so the divide result is established inductively and never simulated inside the
  task. Breaking one of those assertions shows as `UNKNOWN rc=4`, not `FAIL`; that is the class's
  normal detection signal, not a weakness, and it is the reason a control mutation sits in
  ADR-0051's table.
- **`complete` passes over a recorded exclusion set**, not over the ISA — the two entries under
  term 5 above, declined by declaration rather than by omission.
- **Every generated check is bounded BMC.** There is still no `mode prove` **on the ladder**, and a
  PASS means no counterexample within that check's configured depth. What has changed since ADR-0037
  wrote that caveat is that there are now **four `mode prove` component proofs off the ladder** —
  `decoder`, `executor`, `pcloop` and `traps`, all k-induction, all re-run above. The caveat is
  about the ladder and survives M2 unchanged; the four proofs are why some of the properties that
  matter most are not held only by a bound.
- **riscv-formal ships no spec model for `ecall`/`ebreak`/`mret`/`csrr*` at the pin**, so the whole
  of M3's trap and CSR semantics rests on assertions **this repo wrote** — `test/asm/trap.S`,
  `test/csr_tb.v`, `test/decoder_tb.v`, and `components_traps`, which is the only thing in the tree
  that says a trap lands on `mtvec` and saves the right state. This caveat survives M2 by design and
  is not a burn-down item.
- **ADR-0027 is narrowed, not closed.** `csrc_upcnt_minstret_ch0` says the counter **strictly
  increases** between two reads with writes assumed away. It does **not** say `minstret` advances by
  exactly the non-trapping issues, which is the rule ADR-0027 actually states. That half is carried
  by `test/asm/minstret.S`, `test/csr_tb.v`, and `components_traps`' two assertions — on the count
  enable and on the register read through a held address, asserted twice precisely because a counter
  ticking from somewhere else would satisfy the first alone. Recording the narrowing rather than
  absorbing it into a green ladder is the whole point of this section.
- **`make cosim-suite` and `make waves` remain outside every gate.** The co-sim's exclusion is
  decided; it is the only oracle here that reads the core's real `regs_a` and no `rvfi_*` signal, so
  a change that needs its verdict still carries pre/post output in the PR. It ran 59/59 agreed today.
- **`make cycles` has a latent gap**: a cycle that issues nothing *without* raising `stall` is
  charged as an issue cycle. Nothing in the tree can produce one, and `test/decoder_tb.v`'s identity
  between `stall` and the OR of the six named reasons is what keeps it that way.

## Two things this audit found that were not written down

**`btorsim` is not in a Homebrew formal toolchain, and its absence turns every red check into
`ERROR`.** Swept `hang` down to its floor and the log reads
`bad state property 0 reachable at bound k = 6 SATISFIABLE`, then
`bash: btorsim: command not found`, then `DONE (ERROR, rc=16)`. This is ADR-0036's defect reproduced
live rather than recalled, and it is why CI opens the `formal` job with a step that refuses to
report a verdict when any of `yosys`, `sby`, `btormc`, `btorsim` is missing. Two consequences for
reading this ADR: the 85/85 above is still sound, because `ERROR` is not `PASS` and an empty
`EXPECTED_FAIL` is a set equality that an `ERROR` breaks; and every **red** result quoted here is
quoted from the engine's `SATISFIABLE` line, never from the status word.

**`nonperturbation` — the mechanism that discharges term 4 — is still not a required check.**
ADR-0047 landed it unrequired and wrote that this "is the correct state; it is the maintainer's call
after a few runs." It has had its few runs. It is the only M2-term-discharging check outside the
required set, and it is a 9-second structural job with no capacity argument against it, unlike `fit`,
whose non-required status ADR-0052 decided on the merits. Branch protection is a repository setting
no pull request may touch, so this is a recommendation, not a change: **promote `nonperturbation` to
the required set.** Recorded here so the next reader of term 4 does not have to notice it again.

## Consequences

- **M2 is reached.** `CLAUDE.md`'s M2 text says so, names the residuals above, and stops pointing at
  ADR-0037 as a live burn-down; ADR-0037 and its successors are the record of how the criterion was
  built and met, which is history and reads as such.
- **The declaration is a claim about six terms, not about the core being correct.** Every sentence
  under "What M2 does not mean" is still true the day after this lands. The instruction to read an
  empty `formal/EXPECTED_FAIL` as necessary and not sufficient is unchanged and stays in
  `CLAUDE.md`.
- **The next milestone inherits a live question**: term 6's evidence is the only kind this repo
  cannot take for itself, and today it had to be taken from a day-old run rather than a fresh one.
  If the runner pool stays empty, the honest statement about any future gate claim degrades the same
  way, and the first document to notice will be whichever one re-reads a term it had already ticked.
- **Re-auditing a met term is cheap.** The whole of the above, minus the abandoned `complete`
  substitution, is about 14 minutes of wall time on a laptop. Both of the drifts this repo has
  recorded against its own milestone table — ADR-0045 finding term 3 already met, ADR-0049 finding
  term 2's oracle vacuous — were found by running something that was already marked green. Neither
  was found by reading the document that said so.
