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

## Closed in the follow-up commits: `components_traps` and its neighbors

`formal/pcloop.sv` and `formal/traps.sv` are rewired to compose `fetcher`, `decoder`,
`executor` (and, for `traps.sv`, `csrs`) the way `rtl/littlecpu.v` actually does, in place
of the old hand-wired topology this ADR originally shipped with. `rtl/executor.v` has its
own `` `ifdef FORMAL `` block (the ALU/branch/trap assertions the fused decoder's block
used to carry, re-targeted at X's own signal names). `components_pcloop` and
`components_decoder` closed without further work; `components_traps` did not, and closing
it is the substance of this update.

**The gap was not in the RTL.** `formal/traps.sv` reads D and X with `-formal -noassume`,
which drops every standalone-only `assume` the two modules' own `` `ifdef FORMAL `` blocks
state about their inputs, and turns their own `assert`s into required properties of the
composed proof instead. `rtl/executor.v`'s standalone block assumed several structural
facts about `in` (its dx_output input) for free — its class flags are mutually exclusive
(onehot0), `is_csr_access` is derived from `is_csrrw`/`is_csrrs`/`is_csrrc`, a bubble is the
struct zeroed — and `formal/traps.sv`'s own reference model independently re-derives
`is_ebreak`/`is_ecall`, every load/store's immediate, the eleven A encodings, and one
positive case (a plain `add`) straight from `dx_instr`'s raw bits, trusting none of D's
decode. Every one of these is true by construction — `rtl/decoder.v` captures `out`'s
fields together, from the same `instr`, in the same branch — but k-induction cannot use a
fact that is merely true in the RTL text; it needs the fact stated as an `assert` on `out`
itself, in the module that owns that state (`rtl/decoder.v`), read into the composition
with `-formal -noassume` the same way `formal/components.sby`'s `traps` task already reads
`decoder`'s and `executor`'s own asserts. Six such asserts were missing (onehot0 over the
full class-flag set, `is_csr_access`'s derivation, the reserved-opcode/zero-word converse,
is_ebreak/is_ecall, the twelve load/store encodings' class-flag-and-immediate equivalence,
the eleven A encodings' equivalence and zero immediate, and the one `add` case) and each was
found the same way: an induction counterexample at `rtl/executor.v`'s own line, read back
through the VCD to the exact unreachable `out` combination the induction's free starting
state had picked, then closed with the matching assert. One of the six additions
(`!out_is_interrupt` excluding the onehot0/derivation asserts) was itself a bug in the fix,
not the RTL: the interrupt bubble zeroes every class flag too, so excluding it left them
free during an interrupt cycle and broke a check unrelated to interrupts entirely
(`in_is_srl`'s shift reference in `rtl/executor.v`, gated on `in_is_srl` alone). No
assertion, probe, or proof mode was weakened to close this; `components_traps` is still
`mode prove`, still k-induction, still unbounded.

`traps-tval-probe` (retargeted at `rtl/executor.v`, not yet re-run when this ADR was first
written) and `traps-region-probe` both fail at their own named assertion against the
shipping RTL's own mutations, confirming the composed proof did not lose its own red
direction while these six facts were added.

`test/exec_tb.v` is rebuilt against `rtl/executor.v`'s real ports — operand values ride the
separate `reg_rs1`/`reg_rs2` ports now, not `in.rs1`/`in.rs2` (which carry register NUMBERS
in the split, not values) — and passes with its full required coverage.
`test/zkt_isolation_test.py` is retargeted at `rtl/executor.v`: D's own nine former stall
reasons no longer read a bit of register-file data at all, so the whole structural argument
moved to X's one timing output (`x_busy`), gated by `region_stall` and the divider's own
`divider_busy` — Zkt's own two named exclusions — rather than a blanket ban.

## A second, independent defect: six assertions were vacuous, since the split landed

`components_traps` closing (above) and its six checks actually checking something are two
different claims. `traps-region-probe` timed out at minutes per attempt instead of its
documented ~6 seconds, and chasing that — not a code review — is what surfaced this: `mode
cover` proved the six `prev_trap_entry`/`prev_interrupt_entry`-gated CSR-read assertions
(MCAUSE and MTVAL, the normal case, the fetch-fault case, and the interrupt case) were each
*unreachable*, not merely slow to disprove. They had been since B1 landed, and every
`components_traps` PASS since then proved nothing about mcause or mtval.

The mechanism: D's own mandatory post-redirect bubble (no wrong-path state) means the
earliest instruction able to read a CSR back reaches X two cycles after `trap_entry`
commits, not one. `prev_trap_entry` looks back exactly one cycle, so `csr_addr ==
MCAUSE`/`MTVAL` could never be true on the cycle the guard checked — the arm was structured
exactly like `rtl/decoder.v`'s (formerly) guessed register pair, one cycle short of the
pipeline it was actually measuring. `prev2_trap_entry`, a second delay tap mirroring the
`prev2_rdata`/`prev2_mstatus_*` chain the interrupt-independent checks already used, fixed
the guard itself.

Fixing the guard surfaced a second bug underneath it, of the class ADR-0207 and B1 both
already produced (a signal racing ahead of the state it should have waited for): the
cause/tval VALUES the four regular-trap checks compared against (`expected_cause`/
`expected_tval`) were built from `instr`, `formal/traps.sv`'s own read of `fetcher_out` —
the fetcher's live, still-advancing word — rather than from `dx_out.instr`, the stable word
X is actually committing. By the time a CSR-read instruction reached X two cycles later,
fetch had typically moved on to a completely different instruction, so the comparison's
right-hand side described the wrong access. A parallel signal family already existed for
exactly this reason (`c_expected_trap`/`c_must_not_trap`, built off `dx_instr` and checked
same-cycle against `trap_entry`, added when `components_traps` first closed); the fix adds
`c_expected_cause`/`c_expected_tval` alongside it, mirroring the same case statement, and
retargets `prev_cause`/`prev_tval`'s own capture onto them. The fetch-fault MTVAL arm had
the identical defect one level down — it compared against `past_fetch_pc`, a tap on the
same live `fetch_pc` — and is now `past2_dx_pc`, a tap on `dx_pc` (the same stable pc `mepc`
already reads). The two interrupt-entry checks had a third instance of the same class:
`prev_interrupt_entry`/`prev_interrupt_pending` read the CSR file's live `interrupt_pending`
rather than `dx_out.is_interrupt`, X's own captured decision that a given commit is an
interrupt entry rather than a real instruction — the two can disagree once mie/mip have
moved between the decision and the commit.

Each of the six was confirmed the same way: a direct property, stated against the reference
model's own spec rule with no cycle-offset dependency, proven by `mode prove` with the other
five assertions disabled — first against `is_amo_op`/`is_lr`'s tval rule (holds at full
depth, no counterexample, confirming the earlier hand-read of a VCD trace that had produced
two disagreeing values for the same signal was the unreliable step, not the RTL or the
spec) — then against the fetch-fault and interrupt arms directly, each of which did produce
a real, fast counterexample (traps.sv:583, then traps.sv:628 and :686) pointing at exactly
the live-wire tap named above. No RTL changed for any of the six; `rtl/executor.v`'s own
`trap_tval`/`trap_cause` case statements (`in_imem_fault: trap_tval = in_pc;` etc.) were
confirmed correct by the same direct-property method and are what the fixed reference model
now agrees with.

Six permanent `cover()` statements (`mcause_normal_reached`, `mcause_fetch_fault_reached`,
`mtval_normal_reached`, `mtval_fetch_fault_reached`, `mtval_interrupt_reached`,
`mcause_interrupt_reached`) replace the one-off diagnostic covers used to find this:
`formal/traps_cover.sby`, wired into `components_traps` as a prerequisite the same way
`pcloop_cover` already is, proves all six reachable under `prev2_` in under 20 seconds.
`traps-region-probe.py` and `traps-tval-probe.py` are re-pinned to the `prev2_cause`/
`prev2_tval` spelling; both still fail their two-and-two mutations at the right line.

**Left open**: the real `components_traps` proof (`mode prove`, `bitwuzla`, the composed
fetcher/D/X/csrs environment, no narrowing) passed basecase cleanly through step 19 of 20
over roughly 45 minutes in one full run and, in a second, independent run against the exact
tracked file, reached step 16 of 20 at the 20-minute mark with no counterexample either
time — meaning the fix is not in question, but whether this proof now fits inside the
`components-proof` CI job's wall clock is. `traps_cover`, `traps-region-probe` and
`traps-tval-probe`'s own two mutation cases are all fast (under 20 seconds combined);
`traps-tval-probe`'s **control** case (the shipping core, required to PASS) is the one
still-open exception — it inherits the slow default solver its own header says is
deliberate (`traps_probe_sby.py`, "the engine is not [read from components.sby], deliberately,"
so a probe cannot inherit a fast choice made for some other task's runtime), and that
choice was cheap against a vacuous property and is not cheap against a real one. Whether
`components_traps` itself clears CI's window, and whether `traps-tval-probe`'s control case
needs its own engine override now that its property is real, are both undecided.

## Not yet done

`test/decoder_tb.v` (1345 lines) still carries the fused decoder's port list and its whole
vector set is built on the two-cycle guess-then-fetch protocol B1 deleted (the guessed
pair, the operand-fetch stall, same-cycle branch resolution) — not a rename, a rewrite
against D's actual single-cycle present-then-issue shape, and larger than the rest of this
closure combined. `test/stall_sites_test.py` (and, with it, `test/decoder_tb.v`'s OR-identity
check, `CLAUDE.md`'s own stall-broadcast list, and every other of the six declared sites)
needs re-deriving against `x_busy` folding two reasons into one bit at D's level and the
operand reason's deletion; `CLAUDE.md`'s prose for this is rewritten in this same PR but the
grading script that would catch it drifting is not. `test/MUTATION_DETECTORS`'s five
patches keyed to the region-test logic that moved into `rtl/executor.v` still do not apply;
re-keying and re-measuring each is blocked on `test/decoder_tb.v` for the two whose detector
is that bench. None of these three is graded on `make test`'s own path except
`zkt-isolation-test` (now closed) and `mutation-probe` (the forced-red prerequisite, not the
check itself).

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

`make fit`: 4,658 `ICESTORM_LC` (after the `/simplify` pass below) against `main`'s pin of
4,097 (+561) and the branch's own 4,219-cell budget (over by 439). Both `make fit` and
`make soc-timing` are expected red on `thejefflarson/fetch-refactor` by the owner's
standing decision (ADR-0207) to finish the refactor before trimming cells; this stage's
own contribution to that overrun — the wider D/X register, X's absorbed branch/
address/trap/CSR logic, the deleted second `regsel` instance and guessed-pair machinery —
is not separated out here, since the trim pass reads the tree Stage B leaves, not one
stage's isolated delta.

## `/simplify` pass

Four review angles (reuse, simplification, efficiency, altitude) over the diff found two
genuine duplicate-expression cases in `rtl/executor.v`, both fixed: `instr_atomic`/
`instr_atomic_write` each re-OR'd the same nine AMO flags `is_amo` computes a few lines
later (hoisted and reused, `-19` `ICESTORM_LC`), and `redirect`'s comparison recomputed
`in_pc + pc_inc` a second time next to `resolved_target`'s own copy (hoisted to
`seq_pc`). `make test`'s cxxrtl leg and `make cosim-suite` both re-confirmed green after
each change.

## Decision

Land Stage B1 with `test/decoder_tb.v`, `test/stall_sites_test.py` and
`test/MUTATION_DETECTORS`'s five region-test patches as explicitly open work, rather than
block this PR on completing them. `components_pcloop`, `components_decoder`,
`components_traps` and `zkt-isolation-test` — the four items this ADR originally left open
alongside those three — are now closed, k-induction unbounded and unweakened throughout.
The evidence available for the remaining three — three independent oracles (cxxrtl's
per-retire RVFI monitor, Sail cosim reading raw `regs_a`, and 86 generated riscv-formal
per-instruction proofs) all agreeing, `imemcheck`/`dmemcheck`/`nonperturbation`/
`remeasure-fg` all passing, and every unit bench but `decoder_tb` passing against the new
topology — is real, independent, and substantial; the remaining three test the SAME
architectural change from an angle those oracles cannot reach (a vector-level bench built
on D's own new single-cycle protocol) and are owed before Stage B is declared complete, not
before this stage ships.

**Amended**: "`components_traps` ... closed" above was true of the induction generalizing,
not of what six of its assertions actually checked — see the vacuity section above. The
guard is fixed and proven non-vacuous by `traps_cover`, but whether the now-real proof fits
CI's `components-proof` window is undecided and stays open alongside the other three.
