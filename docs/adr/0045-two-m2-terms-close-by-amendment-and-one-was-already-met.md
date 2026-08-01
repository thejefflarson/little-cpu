# ADR-0045: Two M2 terms close by amendment, one was already met, and the measurements that force it

**Status:** Accepted · 2026-08-01 · *Amends [ADR-0037](0037-an-empty-baseline-is-not-m2.md). Supplements
ADR-0010, ADR-0020, ADR-0025.*

## Context

ADR-0037 rewrote M2 as a six-term conjunction after an empty `formal/EXPECTED_FAIL` turned out to be
satisfiable without the milestone. That was the right correction. This ADR is the next one: **two of
those six terms are not reachable on this toolchain, one was already met when it was written, and
none of that was known because nobody had measured it.**

ADR-0037 anticipated this. Terms 2, 4 and 5 each carry an explicit "or" clause. This ADR takes two
of them, and records the measurements that make taking them a conclusion rather than a concession.

## Term 3 was already met, and ADR-0037 contradicts itself about it

ADR-0037 §1 item 3 says `reg_ch0` **"is inconclusive — a single depth-21 BMC query that does not
return in a practical budget."** That was true when ADR-0023 wrote it and false by the time ADR-0037
quoted it: ADR-0024 switched the ladder to `btor btormc` and `reg_ch0` returned in seconds.

The contradiction is internal. ADR-0037's own premise is that the ladder is 82 pass / 0 fail, and
`formal/EXPECTED_FAIL` is empty — by `check-baseline.sh`'s contract that *means* `reg_ch0` passed.
The document asserts both "82 pass" and "one of them does not return."

**Term 3 is met.** ADR-0042 §3 re-measured it against the new synchronous regfile at `PASS 0 31` in
32 s, with the rs2-write-through-bypass probe still flipping it to
`bad state property 1 ... reachable at bound k = 20 SATISFIABLE` — a verdict, plus a demonstration
that the verdict means something.

This is ADR-0037's own failure mode, in ADR-0037. A criterion is a claim about the world and decays
like any other.

## Term 2 — the literal wording is unreachable under either engine

**Decision: term 2 closes by naming `components_executor` + `test/exec_tb.v` as the oracle for real
mul/div, not by dropping `RISCV_FORMAL_ALTOPS`.**

Three measurements force it:

| | result |
|---|---|
| Non-ALTOPS `insn_mul_ch0`, depth 15, `btor btormc` | bad-state property 9 of 33 after 6 min — no verdict |
| Non-ALTOPS `insn_mul_ch0`, depth 15, `smtbmc z3` | **15 m 26 s on a single assertion** at step 15 — no verdict |
| `formal/genchecks-local.py:534` | `check_insn` reads only the global `[defines]`; the per-check form exists only in `check_cons` |

The z3 figure is the one that settles it. SMT handles `bvmul` natively where btormc bit-blasts it,
which is precisely why `components_executor` proves *the same multiplier* at full 32-bit width in
seconds. **So the obstacle is not the engine — it is the surrounding pipeline.** Proving
`rd_wdata == rs1 * rs2` in isolation is easy; proving it fifteen cycles into an unrolled core with
the whole datapath's state free is not.

And because `check_insn` has no per-check defines, ALTOPS is all-or-nothing across all 70 `insn_*`
checks. Dropping it means re-deriving every depth from the 32-cycle divider (ADR-0025 puts that near
38 single-hop). The escapes — forking `genchecks` (ADR-0031 forbids it, `make genchecks-check`
enforces it) or hand-authored companion `.sby` files outside the graded ladder — are worse than the
hatch.

**What was already true and undocumented:** `rtl/executor.v`'s `ifdef FORMAL` block proves
MUL/MULH/MULHU/MULHSU at full width against SystemVerilog `*`, unrestricted, by `mode prove`, **on
CI today**. ADR-0010's gap is half-closed and neither ADR-0010 nor CLAUDE.md says so.

The composition argument, and its residual, belong in the ticket that finishes the proof — written
with the work, not ahead of it.

## Term 4 — `equiv.sh` is misdiagnosed, and its stated remedy cannot work

**Decision: term 4 closes by proving the structural claim, not by making `equiv.sh` converge.**

ADR-0020 and ADR-0023 record that `equiv_induct` fails on `executor.mul_div_y` bits, and ADR-0020
proposes blackboxing the divider. Measured at a 900 s bound:

- `equiv_make` produces **494 unproven `$equiv` cells** in 38 groups
- `equiv_simple` proves **35**, leaving **459**
- `equiv_induct` then fails the induction step at every extension, clauses growing ~660k per step

The unproven set is `mem_addr[2..31]`, `accessor.pending_valid`, `accessor.pending_rd`,
`executor.mul_div_y` — essentially the whole datapath. **`equiv_make`'s name-based matching pairs
almost nothing**, because the gate build optimises to a differently-named netlist. It is being asked
to prove sequential equivalence of two near-arbitrary FSMs from scratch.

**So blackboxing the divider — ADR-0020's own suggested remedy — cannot help.** The problem is
matching, not the divider, and more solver time does not fix it.

The replacement proves what the guarantee is actually about: if the instrumentation is write-only
with respect to the core, then `opt_clean` on the gate build must sweep every shadow register and
leave a netlist structurally identical to gold. Decidable, cheap, CI-able — and *stronger* than a
bounded miter, because it fails the moment an `ifdef`'d value reaches a real signal.

## Consequences

- **This sprint closes five of six terms, not six.** Term 6's CSR half needs `rtl/littlecpu.v`'s RVFI
  CSR port set extended beyond `mcycle`/`minstret`/`mscratch` and a repo-owned check written, because
  `rvfi_insn_check.sv` has generic CSR plumbing for `misa` alone and `rvfi_csrw_check.sv` has no WARL
  model. That is a sprint, not a ticket. **The milestone table says five of six rather than letting
  "nearly M2" drift into "M2"** — which is the whole reason ADR-0037 exists.
- **Term 1 is quietly reopened** and is the sprint's first ticket. ADR-0025's addendum records
  `insn 15` sitting on its derived floor with zero margin and says the sweep "should be repeated once
  the CSR RTL exists." It never was — and `e4f5250` then added a fifth stall reason firing on nearly
  every instruction without touching `checks.cfg`. A depth that no longer clears its floor does not
  fail; it goes green having stopped asking.
- Each remaining term's closure carries its own ADR, written **with** the measurement rather than
  ahead of it — the pattern ADR-0038 set when it declined to pre-decide the regfile.
- **Amending a criterion is not the same as meeting it, and the milestone table must not blur them.**
  Terms 2 and 4 are closed here by taking clauses ADR-0037 wrote for exactly this case. That is
  legitimate, and it is also the second time this criterion has moved. A third should prompt asking
  whether the criterion describes anything real.
