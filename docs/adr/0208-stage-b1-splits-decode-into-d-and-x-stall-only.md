# 0208 — Stage B1 splits decode into D and X, stall-only

Status: Proposed. 2026-09-23. Ships as a PR against `thejefflarson/fetch-refactor`, the
integration branch ADR-0207 landed on. `make fit` and `make soc-timing` are red on that
branch by the owner's own decision (the refactor finishes before cells are trimmed);
every correctness gate this ADR reports on is graded against `main`'s bar, not that
branch's expected-red state.

## What this is

The fused decoder ADR-0207 shipped reads register VALUES in the same cycle it reads the
instruction word, using a guessed pair presented a cycle early (`rtl/regsel.v` on
`in.next_instr`) and answered by the write-through-bypassed register file just in time.
That fusion is what the fetch-refactor brief (`docs/ideas/the-fetch-address-reads-registers.md`)
names as the actual cost: branch resolution, the load/store region test, every trap and
CSR access all happen in the same cycle as decode, off a guess rather than the
instruction actually being decoded.

This is the textbook split, stall-only: **D** decodes the buffered word and presents the
register file the instruction's own pair — never a guess — so the answer arrives exactly
when **X**, one cycle later, needs it. X is where every register value first exists in
this pipeline, so branch resolution, the effective address and its region test, CSR
access and every trap but the timer interrupt all move there. B2 (forwarding) and B3
(any further stage moves B1 turns out to force) are separate steps; this one adds no
forwarding path at all — a RAW hazard against anything still in flight simply stalls.

## The mechanism

- **`rtl/structs.v`** gains `dx_output`, the D/X register: `pc`, `instr` (so X can pull a
  CSR immediate's raw uimm field and report RVFI's `insn`/illegal `trap_tval` without a
  second copy of the field-extraction wires), `immediate`, `rd`, `rs1`/`rs2` as register
  NUMBERS (not values), and every `is_*` decode flag X needs to finish resolving —
  including the ones the fused decoder never had to carry past itself (`is_auipc`,
  `is_jal`, `is_beq`..`is_bgeu`, `is_wfi`, `is_fence`, `is_fencei`, `is_csrrw/rs/rc`,
  `is_csr_imm`, `is_math_imm`, `is_interrupt`, `imem_fault`).
- **`rtl/decoder.v`** shrinks to D: the same field extraction and instruction-class
  decode as before (unchanged), `rtl/regsel.v` instantiated once on the buffered word
  (no guess, no second instance), a RAW-only scoreboard (a match against `out`, the
  instruction X is currently resolving, or `executor_out`, X's own registered output —
  no forwarding, no eligibility list), and the same three-slot `serialize` wait for
  CSR/`mret`/`fence.i`. The timer interrupt still short-circuits D — it needs no register
  value — but the actual CSR commit moves one cycle out: D injects a one-cycle
  `is_interrupt` bubble (pc only, everything else zero) and X commits the trap off it.
- **`rtl/executor.v`** absorbs the fused decoder's second half — `mem_addr_calc`, the
  region test's two-tier deferred answer (relocated verbatim, `ls_answer`/`ls_capture`/
  `ls_answer_valid` unchanged in shape), misalignment, atomic fault, branch compare and
  target selection, trap detection and cause priority, and CSR access — alongside its
  existing ALU/shift/mul/div, now fed `reg_rs1`/`reg_rs2` (the regfile's answer to D's
  presentation) instead of pre-resolved operand fields. It produces two outputs: `launch`
  (still the `decoder_output`-shaped struct `rtl/accessor.v` reads, now X's own
  combinational view of the instruction it is resolving, gated to zero on a trap so no
  bus transaction and no register write happen for one) and `executor_output` (unchanged
  in shape). `x_busy` — divider mid-flight or the region test still deferred — is a new
  single signal telling D to hold `out` rather than overwrite it; D folds it into `stall`
  too, so the regfile presentation and the fetch address both freeze with it, not just
  the D/X register (see "Two bugs" below).
- **`rtl/littlecpu.v`** moves fetch-address ownership out of decoder.v (which no longer
  has anywhere to compute a real `next_pc`) into a `fetch_pc` register at the top level:
  it holds while D is stalled, takes X's `redirect_target` when X's resolved target
  differs from D's own sequential guess (`fetcher_pc + pc_inc`, `pc_inc` 2 or 4 off the
  buffered word's own compression bit — **not** F's word-granular `imem_addr_next`, which
  only ever advances a whole 32-bit ROM word at a time; see "Two bugs"), and otherwise
  advances on that guess. `rtl/fetcher.v` is untouched: its own BTFN/`jal` guess
  machinery (built for ADR-0207) is not wired into `fetch_pc` in this stage, so **every**
  taken branch or jump costs the same one-cycle redirect bubble a genuine miss would —
  B1 does not spend F's guess, on purpose, to keep this stage stall-only in truth as well
  as in name. `redirect` is exactly "the resolved target differs from what D guessed,
  or a trap, or `mret`", never an instruction class, matching ADR-0207's own framing.
- **CSR access, `csrs.v`'s ports, and the bus transaction's launch point all move to X**,
  forced by the split rather than chosen: `csr_arg` needs `reg_rs1` for the register-form
  encodings, the effective address needs it for the region test, and neither exists in D.
  This is the "move the minimum" the ticket allowed for. Everything else — the
  serialize wait, the load/store scoreboard's shape, the accessor's own protocol — is
  unchanged.

## Two bugs the split's own timing produced, not decode's logic

Both surfaced as `RVFI Monitor error` divergences under `make test`'s cxxrtl leg, and
both are about a signal racing ahead of the register it should have waited one cycle
for:

1. **The fetch guess must be architectural-size-aware.** `rtl/fetcher.v`'s own
   `imem_addr_next` is a ROM PRE-FETCH pointer — it only ever advances one whole 32-bit
   word, because that is what deciding whether to ask ROM for a new word needs. Feeding
   it to `fetch_pc` as "the guessed next architectural pc" skipped every compressed
   instruction's real 2-byte successor and landed two bytes past where the next real
   instruction was. `predicted_pc = fetcher_pc + (uncompressed ? 4 : 2)`, computed in D
   off the buffered word's own low bits, fixed it.
2. **A redirect must also kill the word D already holds.** X's `redirect` steers
   `fetch_pc` for the *next* fetch, but the word already sitting in `in` when X
   redirects was fetched down the wrong path a cycle earlier — nothing stopped D from
   issuing it anyway. `x_redirect` is now a top-priority bubble in D's own `out<=`,
   ahead of `stall`, discarding that word unconditionally: no counter, no list, matching
   "nothing before X ever commits" for wrong-path fetch the same way it already held for
   wrong-path register writes.

A third bug was in the RVFI reporting path itself, not the pipeline: `launch.valid` was
gated on `!trap_taken`, so a trapping instruction never reached the accessor, writeback,
or RVFI at all — an off-by-one in the retire stream that broke every trap-bearing
program. `executing` (a new signal, `!trap_taken` folded into it) now gates only the
operation-class flags and `rd`; `launch.valid` does not, so a trap retires exactly the
way the fused decoder always did — `rd` forced to zero, no operation flag set, `rvfi.trap`
asserted. A fourth, RVFI-reporting-only bug used the wrong `uses_rs2` (X's own necessary
copy of D's real one, needed since RVFI reports the operand VALUE which only exists in
X) — it dropped the `&& !is_math_imm` term, so `addi`/`slti`/... reported a garbage
`rs2_addr`/`rs2_rdata`. And the interrupt bubble's `rvfi_intr` never reached RVFI at all,
for the structural reason above (the bubble itself never retires): a `pending_intr`
latch now carries it to the first real retire afterward, the one landing at `mtvec`.

## Verified

- **`make test`'s cxxrtl leg: 75/75.** `test/OBSERVED_FLOOR`'s `uart.S` line is the one
  legitimate number this stage moved (1336/1330 → 1143/1137): its poll loop is a CPI-
  sensitive count by the file's own header, and B1's hazard column genuinely rose.
- **`make cosim-suite`: divergence list matches `test/COSIM_EXPECTED_FAIL` exactly**
  (69/75 AGREE, the same baselined DISAGREE/INCONCLUSIVE set as before this stage).
  `test/cosim.cc`'s one hardcoded hierarchical path, `uut decoder pc`, followed `pc`'s
  move to `uut fetch_pc` — diagnostic-print only, not the `regs_a` comparison itself.
- **`make lint`: clean in both passes**, after one dangling-comma fix in
  `rtl/executor.v`'s nested `RISCV_FORMAL_CSR_MCAUSE` port block (`decoder.v`'s original
  had a port after it to absorb the comma; this block does not).
- **`make elaborate-strict`: clean.**
- **`make -C formal remeasure-fg`: F = 6, G = 6, unchanged** from the fused decoder.
  This measures off `rvfi_valid`'s own observable pattern via the generic `hang`/
  `liveness` checks, wrapping `littlecpu` as a whole rather than decode or execute in
  isolation, so it needed no rewrite to run against this split.
- **`make -C formal check`: 86/86 generated riscv-formal checks pass**, matching
  `formal/EXPECTED_CHECKS` and `formal/EXPECTED_FAIL` exactly (both empty). Per-
  instruction bounded-model-check semantics, independent of `test/monitor.sim.v`,
  confirm every instruction the pin has a spec model for. `reg_ch0`, the standing
  liveness probe for the write-through bypass, passes.
- **`make -C formal imemcheck` and `dmemcheck`: PASS.**
- **`make -C formal nonperturbation`: PASS** (structurally identical netlist with and
  without RVFI instrumentation).

## Not yet done — the largest piece of Stage B, by the brief's own estimate

`formal/pcloop.sv` and `formal/traps.sv` wire `fetcher` and `decoder` together by hand,
reproducing `rtl/littlecpu.v`'s OLD topology (`decoder`'s port list included `reg_rs1`,
`reg_rs2`, `executor_out`, `csr_rdata`, `mtvec`, `mepc` directly) rather than
instantiating `littlecpu` as a whole the way `remeasure-fg`'s generic checks do. Both
need rewiring to compose `decoder` and `executor` the way the new `littlecpu.v` does,
and `pcloop`'s own properties need the redesign the brief itself flagged as its riskiest
piece: `pc == $past(next_pc)` no longer holds architecturally once `fetch_pc` can guess
sequentially and be corrected a cycle later by X. `rtl/executor.v` also has no `` `ifdef
FORMAL `` block yet — the ALU/branch/trap assertions that lived in the fused decoder's
own block need to move and be re-targeted at X's new signal names, or `components_executor`
proves nothing. `test/decoder_tb.v` (1345 lines) and `test/exec_tb.v` (571 lines) still
carry the fused decoder's port list and vector set; splitting them is not a rename, since
several of the OLD decoder's per-cycle behaviors (the guessed pair, the operand-fetch
stall, same-cycle branch resolution) no longer exist to test. `test/zkt_isolation_test.py`
and both `-zkt-probe.py` scripts still name `region_stall`'s old site inside decoder.v
and fail their own probe rather than pass vacuously (`decoder-zkt-probe.py` reports the
stale assertion by name rather than silently green). `test/MUTATION_DETECTORS`'s patches
are keyed to line numbers in the region-test logic that moved to executor.v and no
longer apply — `make mutation-check` refuses to run rather than mutate blind. None of
these five are graded on `make test`'s own path except `zkt-isolation-test` and
`mutation-probe` (the forced-red prerequisite, not the check itself); all five are real,
open work for the PR that follows this one.

## Measured: cycles moved the direction the ticket's kill criterion predicted

Both binaries built at commit 28f929f (`thejefflarson/fetch-refactor`'s head) for the
base column, this stage's head for the other, same toolchain, same session.

| | branch base | Stage B1 | delta |
|---|---|---|---|
| `make cycles` suite total | 39,827 cycles | 44,620 cycles | +12.0% |
| `make cycles` suite CPI | 1.82 | 2.06 | |
| `make cycles` hazard share | 30.4% | 36.2% | |
| `make cycles` operand share | 2.2% | **0% (deleted)** | |
| Dhrystone cycles | 1,550,023 | 1,698,022 | +9.5% |
| Dhrystone DMIPS/MHz | 0.734 | 0.670 | |
| CoreMark cycles (100 iter, 16 KB ROM) | 47,720,254 | 53,326,372 | +11.7% |
| CoreMark CPI | 1.65 | 1.85 | |

The operand column existing at all was the guessed-pair mechanism; deleting the guess
deletes the column outright rather than moving it to zero. Every workload's cycles moved
worse by roughly the same 9–12%, and the hazard share rose by roughly the same amount the
operand share vacated — consistent with removing the guess and adding a genuine stall in
its place, with no forwarding yet to buy any of it back. This is the checkpoint the
ticket's kill criterion asked for, and it reads as the plan predicted, not as a surprise.

## Area: reported, not gated

`make fit`: 4,677 `ICESTORM_LC` against `main`'s pin of 4,097 (+580) and the branch's own
4,219-cell budget (over by 458). Both `make fit` and `make soc-timing` are expected red on
`thejefflarson/fetch-refactor` by the owner's standing decision (ADR-0207) to finish the
refactor before trimming cells; this stage's own contribution to that overrun — the wider
D/X register, X's absorbed branch/address/trap/CSR logic, the deleted second `regsel`
instance and guessed-pair machinery — is not separated out here, since the trim pass reads
the tree Stage B leaves, not one stage's isolated delta.

## Decision

Land Stage B1 with the five formal/test-harness items above as explicitly open work,
rather than block this PR on completing them. The evidence available without them —
three independent oracles (cxxrtl's per-retire RVFI monitor, Sail cosim reading raw
`regs_a`, and 86 generated riscv-formal per-instruction proofs) all agreeing, plus
`imemcheck`/`dmemcheck`/`nonperturbation`/`remeasure-fg` all passing against the new
topology unmodified — is real, independent, and substantial; the remaining five items
test the SAME architectural change from angles those oracles cannot reach (component-
level k-induction, mutation coverage, the standing Zkt isolation claim under the new
signal names) and are owed before Stage B is declared complete, not before this stage
ships.
