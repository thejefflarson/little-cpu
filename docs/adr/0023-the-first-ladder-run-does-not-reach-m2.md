# ADR-0023: The first ladder run does not reach M2 — three named holes

**Status:** Accepted · 2026-07-29 · *Supplements ADR-0006, ADR-0010, ADR-0020 · Blocks M2*

## Context

`86e2721` ported `formal/wrapper.v`, `checks.cfg`, `imemcheck`, `dmemcheck`, `cover`, `complete`
and `equiv.sh` from the deleted `rtl/riscv.v`-era harness to `littlecpu`, and ran the riscv-formal
ladder against the pipelined core **for the first time since the 2021 teardown.** That is a large
result and it will be tempting to write it down as M2.

It is not M2. This ADR is the record of exactly what the run establishes, so that nobody has to
reconstruct it later from a green badge.

Re-verified independently at integration, from a clean `genchecks-local.py` sweep:

| | |
|---|---|
| generated | 78 checks (70 `insn_*` = 45 uncompressed + 25 compressed, 2 `csrw_*`, `pc_fwd`, `pc_bwd`, `liveness`, `unique`, `causal`, `reg`) |
| `insn_*` | **55 pass, 15 fail** |
| pipeline checks | `pc_fwd`, `pc_bwd`, `liveness`, `unique`, `causal` — all pass |
| hand-authored | `imemcheck` pass, `dmemcheck` pass, `cover` all 5 goals reached |
| `reg` | **inconclusive** |

All 15 `insn_*` failures were traced to a specific assertion and a specific cause, and each cause
was confirmed by minimal experiment rather than by argument:

- **9** (`lh`, `lhu`, `lw`, `sh`, `sw`, `c_lw`, `c_lwsp`, `c_sw`, `c_swsp`) fail on
  `rvfi_insn_check.sv:198`, `assert(spec_trap == trap)`. `rvfi_trap` is hardwired 0 until
  misalignment traps land in M3 (ADR-0011). Every byte-granularity access (`lb`, `lbu`, `sb`)
  passes — the failing set is exactly the set of accesses whose address can be misaligned, which is
  what makes this attribution checkable rather than asserted.
- **2** (`c_jr`, `c_jalr`) fail on `:178`, the jump target. A real synthesizable defect — ADR-0021.
  Changing exactly one token at `rtl/decoder.v:114` (`instr_jalr` → `instr_jalr_op`) turns both
  green with `insn_jalr` and `insn_lb` still passing.
- **4** (`div`, `divu`, `rem`, `remu`) fail on `:177`, the result value. `rtl/executor.v:221-224`,
  inside `` `else `` of `` `ifndef RISCV_FORMAL_ALTOPS ``, reads `in.rs1`/`in.rs2` in the `divide`
  state — one cycle after issue, when decode has already bubbled `decoder_out` (ADR-0009). The
  non-ALTOPS branch six lines above gets this right and says so in a comment ("Uses the op-select
  and sign bits latched at issue, not `in`"); the ALTOPS branch was not held to the same rule.
  Substituting the already-latched `mul_div_x`/`mul_div_y` makes `insn_div` pass.

The harness is not vacuous and is not over-constrained. `cover` reaches all five goals including
the compound one (≥2 retired loads **and** ≥2 stores **and** ≥2 uncompressed **and** ≥2 compressed
in a single trace), `imemcheck` ties `rvfi_insn` to fetched memory, and `dmemcheck` ties
`rvfi_mem_*` to the data bus. A miswired wrapper does not produce that combination.

## Decision

**Merge, and state plainly that M2 is not reached.** The three holes:

### 1. `reg` is inconclusive, and it is the load-bearing one

`reg_ch0` is a single BMC query at depth 21 (`skip 20`). It did not return in >20 minutes here or
in the porting run. Not a pass, not a fail — no result.

This matters more than its one-line-in-a-table appearance suggests. Every `insn_*` check reads the
core's **self-report**: it compares `rvfi_rd_wdata` against the spec model's expectation. `reg` is
the check that ties that self-report to the actual register file — that the value RVFI *claims* was
written is the value a later instruction *reads back*. Without it, 55 green checks establish that
**the core's story about itself is spec-consistent**, not that the story is true. A core that
computed garbage but reported consistent garbage would pass all 55.

That is not a hypothetical class of bug in this repository. ADR-0019 is exactly a case of an oracle
being self-consistently wrong, and the pre-`a4662a2` regfile — reads one instruction stale — is
exactly the class of defect `reg` exists to catch and `insn_*` does not.

Accepted as a merge condition because the compensating evidence is real and independent: the `.S`
suite is 47/47 through `rtl/regfile.v` under cxxrtl, `test/monitor.v` checks every retire against
architectural state in both sim legs (`b2dafcc`), and `test/regfile_tb.v` covers write-through and
x0 directly (`814ee44`). None of those is exhaustive; together they are not nothing.

The nightly is where `reg` gets a real budget. It needs a bounded `timeout` first (ADR-0022) so an
unsolvable query is reported rather than silently consuming the job, and if it still does not
converge the answer is a decision to write down — split the depth, change engine, or bound it — not
a switch to flip. Same shape as ADR-0020's finding.

### 2. ALTOPS means the ladder never checked the multiplier or the divider

Restating ADR-0010 because a 55/70 headline will invite forgetting it: every check ran with
`RISCV_FORMAL_ALTOPS`. `mul`/`mulh`/`mulhu`/`mulhsu` "pass" against a substituted
`(rs1 + rs2) ^ const`, **not** against multiplication. The real arithmetic is covered only by the
`.S` suite and `test/exec_tb.v`'s randomized differential bench. A green `insn_mul` is a statement
about operand routing and retire timing, and nothing whatever about arithmetic.

For div/rem it is currently worse than that: because of the defect above, ALTOPS does not even
check the plumbing. The divider's operand routing and stall interaction are **unproven by formal in
any form** until that one-line fix lands.

### 3. `equiv.sh` still does not converge

Ported and runs; `equiv_induct` does not close, failing on `executor.mul_div_y` bits. This is the
outcome ADR-0020 predicted and asked to be recorded rather than worked around. ADR-0020 stays open
and its guarantee stays **argued, not proven**. Inducting equivalence across a 32-cycle sequential
divider is a hard problem; the likely answers are a bounded miter or blackboxing the divider, and
either is a decision to write down.

## Consequences

- **M2 stays open.** CLAUDE.md's milestone table must not be marked reached. M2's own wording is
  "the pipelined core re-proves everything the serialized core proved" — the serialized core proved
  the ladder sans CSRs, and 15 red checks plus an inconclusive `reg` is not that.
- Anything summarizing this work — release notes, README, commit messages — says *progress toward
  M2*, names `reg` as inconclusive, and does not let "55/70" stand unqualified. The project's own
  history is the argument for this discipline: it went from *formally verified* to *unverified*
  partly because the claim outran the evidence.
- Closing M2 requires, at minimum: the C.JR fix (ADR-0021), the ALTOPS divide fix, `rvfi_trap` and
  misalignment traps (M3, ADR-0011), a conclusive `reg`, and a decision on `equiv.sh` (ADR-0020).
  The `formal/EXPECTED_FAIL` baseline of ADR-0022 reaching empty is the signal.
