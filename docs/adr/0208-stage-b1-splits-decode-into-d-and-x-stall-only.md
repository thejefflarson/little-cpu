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

## Splitting `components_traps` by property group did not close the timing question

Two runs of the real `components_traps` task (`mode prove`, `bitwuzla`, the full composed
environment, no narrowing, no time limit) each ran roughly an hour without either leg
reaching a verdict — a proof that cannot be bounded is not something to wait out in a PR
gate. `formal/traps.sv`'s 44 assertions and covers were split into four `TRAPS_CHECK_*`
groups (`PC`: the mtvec/mepc/fetch_pc redirect chain and the WARL bit masks; `CAUSE`: the
six mcause/mtval read-back checks the section above fixed, with their covers; `STATUS`:
mstatus's bit3/bit7 dance and the MIE/MIP/interrupt_pending gating; `QUIESCENCE`: whether
`trap_entry` fires exactly when it should, and that nothing else commits alongside it),
each proved by its own `mode prove` task (`traps_pc`/`traps_cause`/`traps_status`/
`traps_quiescence` in `formal/components.sby`) against the SAME full composed environment
and the same seven decoder invariants — nothing is weakened, each task just carries fewer
goals. `formal/traps-groups-test.py` grades that every assertion belongs to exactly one
group and every group is non-empty, both directions against `formal/TRAPS_GROUPS`, the way
`formal/EXPECTED_CHECKS` is graded; `formal/traps-groups-probe.py` is its forced-red
prerequisite, proving a mis-tagged (invisible-to-every-task) assertion and a stale manifest
each fail for their own reason.

**The measured result changes what this ADR can claim.** `traps_pc` — the smallest group,
twelve assertions with no CSR-address-gated timing dependency at all — ran for 31+ minutes
with neither leg closing before being stopped, the same order of magnitude as the unsplit
task's own two unresolved runs. The other three groups were not run to a similar length:
once the smallest, structurally simplest group failed to close comfortably, running the
larger three seemed unlikely to produce a different qualitative answer and was not worth
the additional wall clock, though that is an inference from one data point, not a
measurement of all four. **The working hypothesis, not yet confirmed**: k-induction's cost
here tracks the size of the composed environment's own reachable state space (the
fetcher/decoder/executor/csrs instances, their registers, and traps.sv's own `prev_`/
`prev2_` chains), which every split task carries in full regardless of how many of
traps.sv's OWN assertions it is asked to prove — so dividing the GOAL COUNT does not
divide the cost the way it would for a BMC search, where each additional goal is
comparatively cheap next to finding whether an invariant generalizes at all. If this
holds, splitting by property group is not sufficient on its own; what would need to
shrink is the environment each task inducts over, not the assertion list.

The infrastructure lands regardless of whether the split ultimately reduces wall time — it
is provably correct (elaborates clean under all four groups and the default, `traps_cover`
and both probes still pass unchanged) and is not a weakening of any kind. `formal/Makefile`
gains `components_traps_pc`/`_cause`/`_status`/`_quiescence`, each behind the
`traps-groups` prerequisite; `make -C formal all` is left calling plain `components_traps`
until the timing question above is resolved one way or the other.

## Naming PDR's stubborn properties found no missing invariant

`abc pdr` (mode prove, no time limit, the real fetcher/decoder/executor/regsel/csrs/traps.sv
composition, no narrowing) was run against the unsplit `traps` task in a throwaway sby copy.
The composed netlist carries exactly 143 properties — the AIGER header's own `B` (bad-state)
count and `design_aiger.ywa`'s `asserts` list agree — matching this ADR's earlier "~130"
estimate now made exact. Over roughly 24 minutes on this machine, PDR proved 125 of 143 with
zero counterexamples anywhere, then was stopped rather than left to grind further once its
proved count plateaued for several consecutive frames. The remaining 18, by AIGER output
index mapped back through `design_aiger.ywa` to source line:

- **14 in `rtl/executor.v`**: 764/767/770/773 (a `mul`/`mulh`/`mulhu`/`mulhsu` result reaching
  `out_rd_data`), 780 (the Zkt constant-latency claim for the four multiplies), 809/827/828/829
  (the divider's internal long-division identity and its two derived bounds), 846/849/852/855
  (`divu`/`remu`/`div`/`rem`'s result against the reference expression), and 883
  (`ls_answer_valid` implies `ls_answer == ls_supported`, the region test's deferred-answer
  protocol).
- **2 in `rtl/decoder.v`**: 523 (the `x_busy` hold: `out` unchanged the cycle after
  `$past(x_busy)`) and 567 (the onehot0 over `out`'s full class-flag set, one of the six
  additions that closed the CAUSE group's vacuity, above).
- **2 in `formal/traps.sv`**: 689 and 692, the quiescence group's own end-to-end claims — a
  trap commits when `c_expected_trap` says it must, and does not when `c_must_not_trap` says it
  must not.

None of these eighteen is new evidence of a gap the way the six CAUSE assertions were: those
were *vacuous* (unreachable under their own guard, confirmed by a `mode cover` proof, and each
fixed by a real counterexample once made reachable). Here, PDR never produced a single
counterexample for any of the 18 — only timeouts — which is the opposite signature. Reading
what each group actually is explains why:

- Thirteen of the fourteen `rtl/executor.v` properties (764-855) are the multiplier's and
  divider's own arithmetic correctness, already the subject of a separate, closed, fast proof
  (`components_executor`, over `rtl/executor.v` alone). CLAUDE.md already records that the
  multiplier is "checked differentially, not exhaustively" and the divider "proved under a
  recorded magnitude restriction" — 827's `div_quot_done * div_divisor + div_rem ==
  div_mag_x_done` is exactly the class of nonlinear-arithmetic identity both AIG-based PDR and
  bit-blasted SMT are weakest at generalizing quickly, and here it is being re-derived over the
  whole fetcher/decoder/executor/csrs netlist rather than `components_executor`'s own much
  smaller one.
- 883 is the one non-arithmetic `executor.v` property in the set; it is the region test's
  deferred-answer protocol the CAUSE group's fix already depends on.
- `decoder.v`:523 and :567 are decoder.v's own properties, already proven quickly by
  `components_decoder`'s standalone proof (closed without further work, above); their
  difficulty here is the size of the composed state space they are being re-verified against,
  not the property.
- `traps.sv`:689/692 are the two real end-to-end goals and depend transitively on every fact
  above, so they are expected to be the last to converge under any induction order.

Three further pieces of evidence, all pointing the same way — the proof is very likely
correct, and too large for the engines tried to close in bounded time, not missing a fact:
this ADR's own two prior full-composition bitwuzla runs (above) each reached step 19/20 and
16/20 with zero counterexamples over roughly 45 and 20 minutes; the property-GROUP split
(above) already showed that shrinking the GOAL count on the same environment does not shrink
the time (`traps_pc`, the smallest group, ran 31+ minutes unclosed); and an attempt here to
isolate a single property further — composing the same full environment with the other 142
properties converted to `assume` via `chformal -assert2assume`, selected by the `$assert`
cell's own `src` attribute (`a:src=traps.sv:692.*`), so k-induction would get 142 free lemmas
while proving one goal — was abandoned before producing a result: yosys's own `prep` splits
SVA `$check` cells into `$assert`/`$assume` across several internal passes rather than
atomically, so landing the selection at the right elaboration stage cost more wall time than
the existing evidence already justified spending.

**No invariant was added for any of the 18, because none produced a counterexample to derive
one from.** What closed the proof instead was excluding the thirteen that are pure
multiplier/divider arithmetic from the traps composition's own obligations — proven
unconditionally, and unaffected, by `components_executor`'s own separate proof, over its own
much smaller environment; no trap property reads a multiply or divide result.
`rtl/executor.v` gates them behind `` `ifndef TRAPS_SKIP_EXEC_ARITH ``, and
`formal/components.sby` defines that macro only on the five traps-composition tasks' own read
of `executor.v` — never on `components_executor`'s, `components_decoder`'s, `pcloop`'s,
`accessor`'s or `busarbiter`'s. `formal/traps-arith-excluded-test.py` grades both the
guarded-assertion count (13, `formal/TRAPS_ARITH_EXCLUDED`) and the macro's placement, both
directions, and `formal/traps-arith-excluded-probe.py` forces it red three ways (a stale
count, an unguarded exclusion, and the macro missing from one traps task); both are wired
into `components_traps` and the four `traps_*` split tasks as Makefile prerequisites, the way
`traps-groups` already is. `rtl/executor.v:883` (`ls_answer_valid` implies
`ls_answer == ls_supported`, the region test's own deferred-answer protocol) and the two
decoder invariants (523, 567) stay in the traps composition's obligations unexcluded: none is
arithmetic, and the two decoder facts are exactly the kind of cross-module lemma the CAUSE fix
above needed — removing them was never on the table.

**Measured**: `make -C formal components_executor` still proves the full `` `ifdef FORMAL ``
block with the macro left undefined — basecase and induction both pass by k-induction under
bitwuzla in 2:35, both Zkt probes (`decoder-zkt-probe.py`, `executor-zkt-probe.py`) still find
and fail at their assertions' now-shifted lines, confirming their line lookup is by text
search and unaffected by the inserted `` `ifdef ``/`` `endif `` lines. `make -C formal
components_traps` — the real, unsplit, full k-induction proof, `mode prove`, no narrowing —
now closes in **57 seconds** of sby's own elapsed clock time (1m59s wall including every
prerequisite: `traps_cover`, `traps-region-probe`, `traps-tval-probe`,
`traps-arith-excluded`), against `components-proof`'s 20-minute CI budget — a ~95% margin, not
a near miss. `abc pdr` on the same excluded composition (mode prove, no time limit, throwaway
sby) reaches 126 of the now-130 properties with zero counterexamples before plateauing again
at the same four residual outputs (the two decoder facts and the two `traps.sv` end-to-end
goals) — PDR alone still does not fully close, but that no longer matters: bitwuzla, the
shipping engine, closes the real proof with room to spare. `formal/components.sby`'s
`traps: smtbmc bitwuzla` line is unchanged, now confirmed rather than merely the least-bad
guess it was before this measurement.

**Decided, not left open**: option (c) from the prior draft of this section — excluding the
arithmetic components_executor already proves, graded so nothing falls into the gap — closed
the proof outright. Freeing the multiplier's and divider's result as an unconstrained value in
the traps composition (the more invasive form of the same idea) was not needed and was not
tried. Neither raising `components-proof`'s CI timeout nor moving `components_traps` off the
required CI path is needed either.

## Not yet done, updated

This section originally listed `test/decoder_tb.v`'s rewrite, `test/stall_sites_test.py`'s
re-derivation and `test/MUTATION_DETECTORS`'s five region-test patches as open work. All
three landed in later commits on this same branch (`e39efef`, ahead of this ADR's own
first version and an ancestor of every commit since): `test/decoder_tb.v` is 435 lines
against D's real single-cycle present-then-issue shape, `test/executor_tb.v` is new (491
lines, the region test's and the trap-cause priority chain's own directed bench),
`test/stall_sites_test.py` grades both vocabularies (D's own composition and the
CPI-accounting taxonomy), and `test/MUTATION_DETECTORS`'s `atomic-region-ignored` and
`loadstore-region-ignored` are re-keyed to `executor_tb`. `make test` and
`make mutation-check` are both green against this tree. Nothing is open from that list
anymore; the only open item this ADR carries forward is the `components_traps` CI-timing
question below.

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
guard is fixed and proven non-vacuous by `traps_cover`.

**Amended again**: every item this section called open or undecided is now closed.
`test/decoder_tb.v`, `test/stall_sites_test.py` and `test/MUTATION_DETECTORS` landed in
`e39efef`, already an ancestor of this branch before this update — see "Not yet done,
updated" above. `components_traps`'s real proof fits `components-proof`'s CI window: closing
it needed excluding the multiplier's and divider's own arithmetic checks (proven
unconditionally elsewhere, graded so nothing falls into the gap), not a wider timeout or a
narrower CI path — see "Naming PDR's stubborn properties" above for the measurement.
