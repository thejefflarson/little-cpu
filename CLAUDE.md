# Little CPU

A hobby RISC-V core in SystemVerilog on the open toolchain (Yosys / iverilog / SymbiYosys —
no vendor EDA). Target **RV32IMC_Zicsr**, machine mode only; eventual home is an ice40 up5k.

**This project optimizes for readability, not throughput.** That is the stated point of it. CPI is
deliberately sacrificed for a design that reads well. Do not "improve" it by adding machinery.

## Current state — read this before believing anything else

**This is a half-finished rewrite. As of `a4662a2` it is a rewrite that computes correct
results — but it is still unverified.** A serialized FSM core (`rtl/riscv.v` + `rtl/alu.v`) once
passed riscv-formal sans CSRs; it was torn down in two waves (2021: `9758a39`→`1709433`; 2023:
`49b317a`→`4fbd650`→`13fec44`) into today's staged design, and the rewrite stalled partway. The
project went from *formally verified* to *unverified*, and getting back is the whole plan. **M1 is
green; M2's blocker (RVFI) is cleared but M2 itself is not reached.** Do not read "47/47" as "the
core is correct."

The README is the project's voice and is deliberately left as-is. **Ground truth lives here.**

**M1 is reached** (`a4662a2`). `rtl/regfile.v` was combinational-read with write-through
bypass (it is synchronous-read as of ADR-0042 — see below), every inter-stage struct carries a `valid` bit, and decode runs a stall-only hazard
scoreboard (ADR-0004 / ADR-0009 / ADR-0015). `make test` was **47/47** at that commit with
`test/EXPECTED_FAIL` empty (it is seeded with M3 debt now — see below); ADR-0014's set-equality
check runs in both directions, so an unexpected *pass* is caught too. The same change fixed three
datapath defects found on the way:
AUIPC computed `reg_rs1 + reg_rs2` instead of `pc + immediate`, `mem_wdata` was registered a cycle
behind `mem_addr`/`mem_wstrb`, and SLL/SRL/SRA did not mask the shift amount to `rs2[4:0]`.

**`b2dafcc` lands RVFI and makes the monitor live in both sim legs.** `rtl/structs.v` carries an
`ifdef RISCV_FORMAL` shadow payload (insn, pc, rs1/rs2 addr+rdata) captured in decode and forwarded
stage to stage riding the existing valid-bit protocol; `rtl/accessor.v` adds the mem
addr/rmask/wmask/rdata/wdata capture (the one stage that actually knows those values, including the
one-cycle load-response latch); `rtl/writeback.v` drives `rvfi_valid`/`rvfi_order`/etc. from the
retiring `accessor_output`, exactly where invariant 3 below says retire happens. `trap`/`halt`/
`intr`/`mode`/`ixl`/the CSR fields were hardwired constants in `rtl/littlecpu.v` at that commit,
because no traps and no CSRs existed yet; only `halt`/`intr`/`mode`/`ixl` still are (the CSR fields
came alive with `rtl/csrs.v`, `rvfi_trap` with trap entry — both below). `test/monitor.v` stays pristine and tracked; a gitignored, build-time-derived
`test/monitor.sim.v` (`test/sanitize_monitor.py`) carries the build-time repairs it needs —
originally two, stripping the `$time`-in-`$display` yosys can't elaborate and ADR-0019's DIV/REM
signedness fix, now three with the `!spec_trap` gate below — and **both** legs read it, so they
cannot drift into checking different specs. `make test` is now per-retire
self-checking in both legs, not merely end-state-checking via `tohost`.

**The generated monitor's DIV/REM spec model is defective, and the sanitizer fixes it**
(ADR-0019). `monitor_insn_div`/`monitor_insn_rem` compute signed division as a conditional whose
other branches are unsigned; per IEEE 1800 sign-context rules that propagates down and evaluates the
division **unsigned**, silently, for negative operands only. Confirmed by driving the two modules
directly with known-correct RVFI values (`-7 / 2` → `0x7ffffffc`, not `0xfffffffd`) and by the
`A_SIGNED 0` parameter yosys puts on the `$div`/`$mod` cell — not a simulator quirk, and not this
core: with the sanitizer rule in place `div.S`/`rem.S` pass. `test/monitor.sim.v` rewrites the two
sites to make the arithmetic self-determined. **`test/EXPECTED_FAIL` is not the place to park a
monitor defect** — read ADR-0019 before adding a line back for one.

**That defect is a repo-wide hazard, not a monitor bug, and it has now caught work here twice** —
the generated monitor's DIV and REM spec models above, and then `test/exec_tb.v`'s new SRA
reference, whose natural one-line form
(`cond ? ... : $signed(a) >>> sh`) reported six mismatches against RTL that was correct. **Any
signed arithmetic written into a *reference model* in this repo must be a self-determined statement
of its own, never an arm of a conditional expression.** `exec_tb`'s shift references are written
that way with the reasoning in place, and a `ref_selftest` block pins all three against
hand-computed literals *before* a single RTL vector runs, so a reference that degraded would say
`ORACLE BROKEN` and stop rather than blaming the core. An oracle that is wrong is worse than no
oracle: it fails correct hardware and teaches the reader to distrust the bench.

**ADR-0003's dual-word combinational fetch window lands, closing the gap ADR-0021 found.**
`rtl/fetcher.v` now reads two adjacent words every cycle (`imem_addr`/`imem_addr2 = imem_addr + 4`)
and windows the 32 bits starting at `pc` out of them — stateless, so a 32-bit instruction straddling
`pc % 4 == 2` costs nothing (no aligner FSM, no stall, no flush; invariant 1 below stays intact).
`rtl/decoder.v` also now masks `instr[31:16]` to zero whenever `quadrant != 2'b11`, the defence
behind the immediate-mux fix ADR-0021 already landed. `test/run_tests.sh` assembles
`-march=rv32imc_zicsr` (ADR-0014's sunset condition, now met): the assembler freely compresses
eligible instructions throughout the whole 49-file suite — the C.JR/C.JALR fix and this window are
no longer verified only by the formal ladder — and two new files target the window directly:
`test/asm/straddle.S` (a straight-line and a branch-redirected straddle at `pc % 4 == 2`) and
`test/asm/rvc.S` (riscv-tests' own RVC corner-case suite). All 49 pass, `test/EXPECTED_FAIL` stays
empty, and the per-retire monitor confirms real compressed retires in both sim legs (`rvfi_insn` a
zero-extended 16-bit value, `rvfi_pc_wdata` stepping by 2) rather than merely that the assembler
happened not to compress anything. `formal/imemcheck.sv` — the check that models fetch at 16-bit
granularity, per ADR-0003's own consequences section — is re-pointed at the split
`imem_addr`/`imem_addr2` interface and still passes. `misa`'s C bit is no longer an untested claim.

**The harness can trap and recover, and the monitor no longer lies about trapping retires.**
Three things landed together, all of them test-side — no `rtl/` file changed. (1)
`test/asm/riscv_test.h` gains **opt-in** trap macros: a `.align 2` handler that records
`mcause`/`mepc`/a trap counter into RAM and resumes at `mepc+4`, a fatal variant that fails the
test from inside the handler, and an installer for `mtvec`. `RVTEST_CODE_BEGIN` is byte-identical
(verified with `gcc -E`), so the 49 existing tests do not start executing CSR instructions before
the CSR RTL exists. The handler **cannot read the faulting instruction** — Harvard buses,
ADR-0008 — so it cannot tell a 2-byte fault from a 4-byte one, and every trapping instruction in a
resuming test must be wrapped in `.option norvc`. (2) The sanitizer moved from an inline `sed`
chain to `test/sanitize_monitor.py` and gained a **third rule**: gate the monitor's spec-value
checks on `!ch0_spec_trap`, mirroring riscv-formal's own `checks/rvfi_insn_check.sv`, which the
monitor generator never emits. Without it a misaligned `lw` — where the spec model reports the
loaded value and `pc+4` while a correct core writes nothing and redirects to `mtvec` (ADR-0028) —
reports errors 104/105/106/111-113 in both sim legs **on correct hardware**. Same situation as
ADR-0019's DIV/REM defect and the same remedy; the script now also asserts a site count per rule,
so a pin bump that changes the generator fails loudly instead of silently shipping an unapplied
sanitizer. (3) `test/monitor_tb.v` joins `make test-units`, driving the sanitized monitor directly
with hand-built trapping and non-trapping RVFI vectors — including a deliberately wrong one, so a
gate that disabled all spec checking could not pass.

**A site count proves a sanitizer rule fired, not what it swallowed.** Rule 3 selects its ~45-line
span by *position* — first anchor (`rs1_addr`) to last (`mem_addr`) — so a pin bump that moved the
trap comparison (`handle_error(101, "mismatch in trap")`) between those anchors would keep the count
at 1, keep the diff at eight lines, and silently gate off the one assertion riscv-formal's
`checks/rvfi_insn_check.sv` deliberately keeps live under `spec_trap`. A core that failed to trap on
a misaligned load would then retire clean in **both** sim legs. So rule 3 now also asserts on
*contents*: two forbidden literals in the span, the exact multiset of `handle_error` codes it
encloses (derived from the current output — re-derive it after a pin bump, never edit it to silence
a failure), and a post-check that error 101 survives into the sanitized output at all. All three
were confirmed by mutation; the pre-change sanitizer accepted every one of them at exit 0.
`test/monitor_tb.v` carries the simulation-side half (vector 7: a retire claiming `rvfi_trap = 0`
where the spec model says it must trap — the existing wrong-retire control is non-trapping and
passes happily without error 101). `trap.S` sits in `test/EXPECTED_FAIL` as `MONITOR-ERROR 101`
today, which is that check doing its job against the missing M3 trap logic.

**`make -C formal genchecks-check` is the drift control for the repo's other vendored generated
file.** ADR-0031 permits `formal/genchecks-local.py` to differ from the pin's
`checks/genchecks.py` by this repo's header and `basedir` and nothing else — but that rule lived
only in a hand-run `cp` + `diff -u` recipe in the script's own header, which nothing enforced, and
unenforced drift there is exactly what produced the five-year-stale fork ADR-0031 exists to end.
`formal/check-genchecks.py` undoes those two documented edits and then requires byte equality with
the clone, so any residual diff is drift by construction and is printed. It runs in CI in the
`monitor-freshness` job, which has already paid for the from-scratch clone at the pin.

`test/EXPECTED_FAIL` was seeded with three M3 tests — `csr.S`, `minstret.S` and `trap.S` — written
ahead of the RTL that pays them off, which is the direction ADR-0014's burn-down contract was
designed for. All three are paid off now (the third by the trap change below), so it is **empty
again** and `make test` is **52 pass / 0 expected-fail**, exiting 0 only on set equality. That
equality is over
**name-and-status pairs** (ADR-0035) — the baselined entry is `trap.S      MONITOR-ERROR 101`, so a
baselined test that starts failing some *other* way (a broken assembly, a `TIMEOUT`, an unstartable
runner) is a red gate rather than a match. The same change made every build step's exit status
checked and `mktemp` fatal: an unchecked `objcopy` that emitted an **empty** RAM image used to make
`tohost` read zero and every data-independent test still report `PASS`.
`test/run_tests.sh` also grew a `MONITOR-ERROR` label for runner exit 4, which used to fall into
`RUNNER-ERROR` and read as "the sim would not start" when it actually means the per-retire oracle
disagreed with the core mid-run, and a `TRAP-TO-ZERO` label for exit 5 (ADR-0029, below).
**Nothing here checks M3 semantics against an oracle**:
riscv-formal ships no spec model for `csrr*`/`ecall`/`ebreak`/`mret` at the pinned SHA, so
`spec_valid` is 0 for every one of them and the monitor's whole semantic block is skipped. The
trap and CSR tests assert on values their own handler recorded. Error 133 ("expected intr after
trap") stays unreachable in this single-channel config — `shadow_pc_valid <= !rvfi_trap` gates 130
and 133 off for the retire after a trap — so `rvfi_intr` hardwired to 0 does not break anything.

**`make test` now measures that its oracle fired, instead of inferring it.** The monitor is the
gate's per-retire oracle and **nothing counted whether it ever looked**: `test/cxxrtl.cc` sampled
the errcode and counted nothing, so a monitor whose `rvfi_valid` never asserted — ADR-0037's
under-sensitivity class, an `ifdef` that dropped the shadow payload, a `write_cxxrtl` that optimised
the instance away — left all 52 programs PASSing off `tohost` alone, with an empty
`test/EXPECTED_FAIL` and so no red entry whose disappearance would say otherwise. `test/testbench.v`
counts retires (`rvfi_valid`) and spec-checked retires (`spec_valid`, from a **second
`monitor_isa_spec`** instantiated in the bench — `test/monitor.v` is generated-but-tracked and a
yosys hierarchical reference silently *implicitly declares* the name rather than resolving it, both
measured); the runner prints `RETIRES <n> SPEC-CHECKED <m>` and adds **exit 6** for zero of either;
`test/run_tests.sh` labels that `MONITOR-SILENT`, carries both counts as a third table column, and
grades them against **`test/OBSERVED_FLOOR`** — names by set equality both ways, numbers by `>=`,
because instruction counts legitimately move and a 52-number ratchet is one nobody would keep.
**The probe is one line and both directions are measured**: `assign rvfi_valid_observed = 1'b0;` in
`test/testbench.v` gives 52 × `MONITOR-SILENT` (`retires=0 spec-checked=0`) and exit 1 here, and the
same blinding on the pre-change tree gave **52/52 and exit 0** — every program still reaches
`tohost` and still passes its own assertions, which is the whole defect. `rvfi_valid_observed` is
the single wire the monitor, the probe and the counters all read, so they cannot go blind
independently. **A low spec-checked count is the pin's coverage boundary, not a bug** — no spec
model exists for `ecall`/`ebreak`/`mret`/`csrr*`, so `csr.S` is 108/82, `minstret.S` 42/31 and
`trap.S` 303/259; that is written at `test/OBSERVED_FLOOR` so nobody has to rediscover it.

**M3 opens: `rtl/csrs.v` lands the CSR file and the Zicsr access path.** ADR-0005's set, exactly —
RW `mstatus` (MIE/MPIE, MPP WARL→`2'b11`), `mtvec` (direct mode, 4-byte-aligned base, resets to 0
per ADR-0029), `mepc` (bit 0 only, because C makes 2-byte targets legal), `mcause`, `mscratch`,
`mcycle`/`mcycleh`, `minstret`/`minstreth`; RO `mtval`/`mie`/`mip` = 0, `misa` = `0x4000_1104`,
`mvendorid`/`marchid`/`mimpid`/`mhartid` = 0. It is a **sibling of the decoder, not a pipeline
stage**: every access is read and committed in decode on the edge the accessing instruction issues,
so no CSR state exists downstream and the read result rides the `is_add` pass-through `lui`/`jal`
already use. `rtl/decoder.v` separates `csrrwi`/`csrrsi`/`csrrci` from the register forms (they were
folded together, losing the zimm-vs-rs1 distinction), excludes the immediate forms from `uses_rs1`
and from `rvfi_rs1_valid`, and folds serialization into `hazard` per ADR-0026 — with
`accessor_out.valid` newly routed in, because a store in flight is invisible to the other three
slots. `minstret` increments at issue, gated on `instr_valid`, which is ADR-0027's non-trapping rule
written now rather than retrofitted. **Nothing reads `mtvec`/`mepc`/`mcause` yet** — trap entry is
deliberately the next step (ADR-0011) — and an unimplemented CSR stays an *unrecognised
instruction* via the existing `instr_valid` path rather than raising a trap, which is exactly what
it did before this file existed.

`csr.S` and `minstret.S` came out of `test/EXPECTED_FAIL`; `test/csr_tb.v` joins `make test-units`.
On the ladder, **`csrw_mcycle_ch0` and `csrw_minstret_ch0` went FAIL → PASS** and came out of
`formal/EXPECTED_FAIL` in the same commit, and `formal/checks.cfg`'s `[csrs]` gained **`mscratch`
only** — 78 checks became 79, `csrw_mscratch_ch0` passes. `mtvec`/`mepc`/`mcause`/`mstatus` are
deliberately **not** on that list and must not be added: `rvfi_csrw_check.sv` has no WARL model, so
a correctly masked WARL CSR fails there **on a correct core** (the reasoning is written out next to
the `[csrs]` list). Those four are checked field-by-field in `test/csr_tb.v` instead. Note also that
`genchecks` defines `RISCV_FORMAL_CSRWH` for `mcycle`/`minstret` by itself, so the `h` halves are
exercised whether or not `checks.cfg` asks for it.

**The regfile read is synchronous now, the core's logic fits the up5k, and the ladder gained an
assumption** (ADR-0042). `rtl/regfile.v` is two block-RAM arrays (an ice40 EBR has one read port, so
a second read port is a second copy — `yosys` infers `4 x SB_RAM40_4K` with no attribute), posedge
write, **posedge registered read**, with two forwarding points left in fabric: write-first into the
read register, and the existing write-through bypass. `rtl/decoder.v` gains `operand_stall` — present
the address pair, bubble, issue — which is a fifth stall *reason* on ADR-0009's existing bubble
mechanism and adds **no flush** (the PC simply holds, so invariant 1 holds by construction).
**`make fit` goes 6971/132% to 4236/80%**, a reduction of 2735 logic cells, and becomes a ratchet.
The price is **+18.0% cycles across the 52-program suite**, measured — CLAUDE.md's fourth line being
cashed. The stall is gated on `uses_rs1`/`uses_rs2`, the scoreboard's own predicates: ungated it
costs +27.8%, so the gate recovers 35% of the penalty for one extra term.

**Two things about that change are non-obvious and are why it has an ADR.** First, `formal/wrapper.v`
now assumes **instruction memory is a function of its address** (same address as last cycle ⇒ same
data). Without it `hang` and `liveness_ch0` produce real counterexamples at k=30, because decode
decides whether the fetched operand belongs to the instruction it is issuing by comparing
`rs1`/`rs2`, which are combinational out of `imem_data` — so a free `imem_data` lets the environment
starve forward progress forever. That port did not exist before this change, and `wrapper.v`'s
`RISCV_FAIRNESS` comment (which said no such port existed) is corrected in place. `imemcheck.sv`
already relied on the same fact in a stronger form. Second, **the negedge alternative was rejected
despite being cheaper and free in cycles**: sby's `prep` fails closed on mixed clock polarity, so the
generated ladder cannot run against it *at all* — not one red check, all 82 — and `clk2fflogic` stays
rejected (ADR-0040). Serialising the two read ports onto one array was also built and rejected: 44/52,
and it needs a second bypass level, which is the first step toward the forwarding network invariant 4
forbids.

What does not work right now — **one live entry, then six resolved ones kept in place.** Five of the
six are struck through, and every one of those outlived its own fix here by several commits. That is
the failure this section exists to prevent, so they stay as markers rather than being deleted.

- **Three things this repo treats as gates are reached by no automation, and one of them computes a
  verdict nothing reads.** Found by reading `Makefile`, `formal/Makefile`, both workflows and
  `test/run_tests.sh` end to end; the four-column inventory that came out of it lives in the pull
  request that added this bullet and is destined for the coverage-map ADR. (1) **`make fit` is a
  ratchet nothing pulls.** The ratchet works — `make fit FIT_MAX_LC=4200` against today's 4236-cell
  tree exits **2** with "4236 logic cells is over the 4200-cell budget" — but
  `grep -rn fit .github/workflows/` returns two prose matches about runner memory and **no
  invocation of `make fit`**, so nothing but a human has ever pulled it. Run it by hand on anything
  that touches `rtl/`. (2) **`make waves` grades nothing.** It runs the full pipeline under iverilog
  with the per-retire monitor live, but `ch0_handle_error` only `$display`s and sets `errcode` —
  there is not one `$fatal`, `$finish` or `$stop` in `test/monitor.v` or `test/monitor.sim.v` — and
  on the iverilog leg nothing turns `errcode` into an exit status (`test/testbench.v:55-60` says so
  in its own comment; `test/cxxrtl.cc` is what reads it, and that is the other leg). So `make waves`
  exits 0 on a monitor mismatch, and it is in no workflow either. The consequence is the part worth
  keeping: **`testbench.vvp` is elaborated on every PR and executed by no gate anywhere.** The
  iverilog *simulation* CI actually runs is the six `make test-units` benches, which do `$fatal(1)`
  and are gated. (3) **`make -C formal all` is dead** — no workflow names it. That one is
  redundancy rather than a hole: every target it lists (`complete`, `check`, `dmemcheck`,
  `imemcheck`, the three `components_*`) is invoked separately by `ci.yml` or `formal-nightly.yml`.
  A target that names seven gates and is reached by nothing still reads like coverage, which is why
  it is written down here rather than left to be rediscovered.
- ~~**The ALTOPS divide branch reads stale operands**~~ — **fixed, and this bullet outlived it by
  several commits.** The ALTOPS issue arm latches its operands (`rtl/executor.v:52-60`,
  `div_alt_rs1`/`div_alt_rs2` off `mul_div_x`/`mul_div_y`) exactly as the non-ALTOPS branch does, so
  the completion arm no longer reads an `in` that decode has bubbled. `insn_div_ch0`,
  `insn_divu_ch0`, `insn_rem_ch0` and `insn_remu_ch0` all PASS and none of them is in
  `formal/EXPECTED_FAIL`, which is empty. What is still true, and is the part worth keeping: under
  `` `RISCV_FORMAL_ALTOPS `` those four checks say nothing about the real divider's arithmetic
  (ADR-0010), and ADR-0045 decided that gap closes by naming `components_executor` +
  `test/exec_tb.v` as the oracle rather than by dropping ALTOPS.
- ~~**`formal/equiv.sh` runs but does not converge**~~ — **the file is deleted and the guarantee is
  proved another way** (ADR-0047). Measured: `equiv_make` leaves **459 of 495** `$equiv` cells
  unproven — `mem_addr`, `accessor.pending_*`, `executor.mul_div_*`, `regfile.wdata`, essentially the
  whole datapath — because it matches **by name** and the gate build optimises to a differently-named
  netlist; `equiv_induct` then diverges at **~660k clauses per step** (1.20M → 1.86M → 2.53M → 3.19M)
  and never returns. So ADR-0020's own remedy (blackbox the divider, or bound the miter) cannot work:
  the failure is matching, not the divider. It sat in the nightly behind `continue-on-error` +
  `timeout 3600`, which made a check that had **never produced a verdict** read as coverage.
  `make -C formal nonperturbation` replaces it and runs on the PR gate in ~9s: build with
  `-D RISCV_FORMAL`, `delete -port littlecpu/rvfi_*`, sweep, and require the result to be
  **structurally identical** to the plain build — cell histogram plus a name-independent connectivity
  fingerprint. **It is strictly weaker than sequential equivalence and must not be quoted as if it
  were**: it proves the instrumentation is *unread*, not that two designs behave alike. Both failure
  directions are demonstrated on real mutations (a shadow value ORed into decode's `stall`, +193
  cells; one gating the accessor's `out.valid`, **+11** cells).
- ~~**`reg` is inconclusive**~~ — **false since ADR-0024, and this bullet is where ADR-0037 caught
  itself repeating it.** ADR-0023 wrote it under `smtbmc yices`; the ladder moved to `btor btormc`
  and `reg_ch0` returns in seconds. ADR-0042 §3 re-measured it against the synchronous regfile at
  `PASS 0 31`, and ADR-0046 re-ran its probe against the five-reason pipeline: deleting the rs2
  write-through bypass gives `bad state property 1 reachable at bound k = 20 SATISFIABLE` at the
  shipping `reg 15 20`. It returns a verdict, and the verdict means something. What ADR-0023's
  sentence was reaching for is still true and lives in ADR-0032 instead: `reg_ch0` is the *only*
  check tying RVFI's self-report back to `rtl/regfile.v`, and an architectural write past its bound
  is invisible to the whole ladder.
- ~~**The formal nightly cannot go red**~~ — **fixed, and it was never the `|| true`** (ADR-0037).
  This bullet described `formal-nightly.yml` as `make -C formal check || true` with no `-k`; both
  were fixed earlier, and the bullet outlived them. The real defect survived both fixes: the graded
  `check-baseline.sh` call was piped into `tee`, and a `run:` block without an explicit `shell:` key
  is `bash -e {0}` — errexit but **not** pipefail — so the step's exit status was `tee`'s and was
  always 0. **ADR-0022's central guarantee had never held.** `1961234` fixed both copies of the step
  (nightly and the new PR-gate `formal` job) and demonstrated both failure directions on real runs.
- ~~**Three of the five component-proof tasks are vacuous**~~ — **they were deleted; there are
  three tasks now and all three assert something.** `formal/components.sby` carries `decoder`,
  `executor` and `pcloop`; `fetcher`, `accessor` and `writeback` are gone, and `formal/Makefile`
  says so where the targets used to be. ADR-0006's slate is discharged. The rule the bullet was
  protecting stands and is worth keeping: a green run of a task with no assertions is not a result.
- **`components_pcloop` was failing on `main`, and nothing ran it** (ADR-0046). It has failed since
  `e4f5250` — `failed assertion ... at pcloop.sv:273 step 3`, the sequential-advance assertion —
  because its `f_may_stall` over-approximation predates ADR-0042's fifth stall reason and therefore
  covered a cycle on which the decoder legitimately holds the pc. Attributed by mutation: forcing
  `operand_stall = 1'b0` makes the task pass by k-induction. Fixed by transcribing
  `rtl/decoder.v`'s `prev_rs1`/`prev_rs2`/`read_taken` register into the harness, and the task is on
  CI now — ADR-0017 puts the fetcher↔decoder pc loop in M2's scope, and **an M2-scope proof nothing
  runs is a prose-only guard**.

**The ladder now asserts its own shape, so a green ladder can no longer shrink quietly**
(ADR-0033's gap 1). `formal/checks.cfg`'s `[depth]` table is the list of checks that *exist*, not a
tuning table — `genchecks` returns early on any check with no depth line, silently — and
`formal/check-baseline.sh` used to glob the run directories `sby` creates only for checks it
actually started, so a never-generated check was missing from the results and from
`formal/EXPECTED_FAIL` at once and set equality called that a clean match. Two files close it, both
set equalities in **both** directions per ADR-0014: `formal/EXPECTED_CHECKS` names every check that
must be generated, and `checks.cfg`'s `#omit` lines name every check `genchecks` considered and
skipped, with a reason each. `formal/genchecks-audit.py` derives both sets by tracing `genchecks`'
own `get_depth_cfg` calls — it does not re-implement the naming, and it cross-checks its trace
against `genchecks`' own `consistency_checks`/`instruction_checks` sets and against the `.sby` files
on disk, so a wrong inventory fails rather than being reported. `check-baseline.sh` now enumerates
`checks/*.sby` rather than directories, so a generated-but-never-scheduled check resolves to
NO-STATUS and counts non-PASS, and `make -C formal check` ends in that comparison rather than in
`sby`'s exit code, which `expect pass,fail` makes meaningless. Deleting any one `[depth]` line —
including a **passing** check's — now fails at generation in a second and again at the post-run
gate.

Three checks joined the ladder in the same change, taking it from 79 to **82**: `ill_ch0` (red — see
`formal/EXPECTED_FAIL`), `causal_mem_ch0` and `hang` (both pass). `ill_ch0` is deliberately landed
**red**: ADR-0033's rule is that a known-red check on the ladder is the system working and a
known-red check off the ladder is the system lying. Fifteen upstream checks remain declined, each
with a `#omit` line; the count is derived from the generator, not asserted in prose. **One of the
fifteen is a trap for a future reader**: adding a bare `csrc` depth line generates three checks
reading `rvfi_csrc_check.sv`, which does not exist at the pin — the six real models
(`rvfi_csrc_{any,const,hpm,inc,upcnt,zero}_check.sv`) are reached only through a per-CSR *test list*
in `[csrs]`.

**`make -C formal check` could not re-run the ladder, and had been re-grading the previous run**
(ADR-0040). `formal/Makefile`'s `checks` was a plain file target naming a **directory**, and a
directory's mtime bumps whenever an entry is created inside it — `sby` creates `checks/<name>/` for
every check it runs, so after one ladder run `checks/` was newer than `checks.cfg`,
`genchecks-local.py` and `EXPECTED_CHECKS` at once and make called it up to date forever. Editing
`checks.cfg` between two runs did not regenerate. Compounding it, `genchecks-local.py:72` is
`sbycmd = "sby"` with no `-f` and the makefile it emits hardcodes that string, so every
re-invocation aborted inside sby and left the prior `status` file in place; `make -BC checks` forces
the **rule**, not sby. Measured at `317b86c`: run 1 took 310.4s and wrote 82 status files, run 2 took
22.0s, printed "Directory already exists" for all 82, wrote **none**, and still reported "82 checks:
82 pass, 0 fail" — a verdict it had not computed, about RTL that may have changed underneath it.
`checks` is now `.PHONY` and deletes the run directories before regenerating (~1s), and `-B` is off
the sub-make. **`make -C formal check` is now always a fresh run**; `make -C formal check-baseline`
is still how you re-grade a finished one without re-running it.

**The ladder will not silently mis-model a negedge regfile — it refuses to run at all** (ADR-0040).
This matters because ADR-0038 now *recommends* a negedge-BRAM regfile on area grounds. A raw
`yosys ... prep -flatten -nordff; write_btor` does discard clock polarity silently (`clk` becomes an
unused input; the read register and the storage array advance on the same step) — but the ladder
never takes that path. sby runs its own `prep` model step after the `.sby`'s `[script]`, and
`formalff -clk2ff` there fails closed: *"CLK clock ... also used with opposite polarity, run
clk2fflogic instead"*, `DONE (ERROR, rc=16)`, which `check-baseline.sh` counts as red. **The silent
loss is scoped to bespoke yosys scripts that end in `write_btor`/`write_smt2` without going through
sby** — any standalone regfile proof written that way would be one. (`formal/equiv.sh` was the
example this line used to give; it is deleted as of ADR-0047, and it was never exposed anyway — it
had no backend, a correction `cdb53cb` already made.)
`clk2fflogic` is **rejected** as the remedy: at genchecks' own depths it makes `reg_ch0` PASS on a
regfile with the rs2 write-through bypass deleted, which the stock ladder catches in 13.9s, because
`clk2fflogic` makes a clock cycle two BMC steps while genchecks ties `skip`/`depth`/
`RISCV_FORMAL_CHECK_CYCLE` to one `[depth]` column — and no `checks.cfg` expression can decouple
them. **Deleting the rs2 write-through bypass is this repo's liveness probe for `reg_ch0`**: one
line, a direct invariant-6 violation, counterexample on bad state property 1 at k=20. Reach for it
before believing any `reg_ch0` result taken under a changed configuration.

**M3's keystone: every trap is now detected AND committed in decode, and the formal baseline is
empty.** `rtl/decoder.v` computes misalignment from the same `$signed(immediate) +
$signed(reg_rs1)` it already had, decides illegal-instruction (including ADR-0005's two illegal-CSR
rules), and commits `ecall`/`ebreak`/illegal/load-misaligned/store-misaligned in the *same*
non-stalled `else` arm that issues an instruction — which is what buys, for free, no trap on a
stalled cycle, no trap from a bubble, no double commit, and a `reg_rs1` the scoreboard has already
made hazard-clear. **A trap is a branch**: `pc <= mtvec` is the same override `jal` and the
branches use, so there is still no flush anywhere (invariant 1). `rtl/csrs.v` gains a second write
port (`trap_entry`/`trap_cause`/`trap_epc`/`mret_entry`) and hands `mtvec`/`mepc` back to decode;
`mret` gets its own decode, restores MIE from MPIE, and serializes on the existing drain
predicate; `wfi`, `fence` and `fence.i` become the NOPs ADR-0005 always said they were, which was
optional right up until "unrecognised" started meaning "illegal instruction". `rtl/accessor.v`'s
`mem_misaligned` and `rtl/littlecpu.v`'s `mem_misaligned_trap` are **gone** — ADR-0011's deferred
debt, discharged — and the top-level `trap` port is redefined per ADR-0028 as a one-cycle "trap
entry committed" pulse that something finally consumes. `rvfi_trap` is driven from the retiring
instruction's shadow instead of being hardwired 0. **`formal/EXPECTED_FAIL` reaches EMPTY: 82
checks, 82 pass** — the nine misalignment entries and `ill_ch0` all closed at once, exactly as the
attribution predicted, and `insn_lb`/`insn_lbu`/`insn_sb` (the byte accesses, which cannot be
misaligned) never moved. `test/EXPECTED_FAIL` reaches empty too, at 52/52. **This does not by
itself mean M2** — see the milestone table.

ADR-0034's sunset condition **fired**: `decoder_output`'s `csr_addr`/`is_csr_imm` were kept as
scaffolding on the explicit condition that they be struck in the trap change if neither acquired a
downstream consumer, and neither did. They are gone. ADR-0029's harness half landed with the RTL
that makes it reachable: `test/testbench.v` raises a `(* keep *) trap_to_zero` flag when a trap
redirects to address 0, `test/cxxrtl.cc` turns that into exit code 5 and `test/run_tests.sh` labels
it `TRAP-TO-ZERO`, so a test that faults before installing `mtvec` gets a named failure instead of
a timeout. And the Sail config (`test/sail/rv32imc_zicsr.json` since ADR-0043;
`test/sail/memory-map.json` at the time) sets `memory.misaligned.exceptions.load_store` to
`{"Some": "AlignmentException"}` — the **global** key, which is what actually governs; the
per-region `misaligned_exceptions` attribute is checked after address translation and with no MMU
is never consulted. Without it the reference model completes misaligned accesses and reports a
false divergence on every one; verified both ways (`trap.S` agrees 214/214 with the key, diverges
at architectural change #6 without it).

**The iverilog leg was dead, and this change is what noticed.** `rtl/decoder.v`'s hazard
scoreboard called a `function automatic live_producer(r)` from two continuous assigns. iverilog
builds such an assign's sensitivity list from the *call's arguments*, so a function body that also
reads module-level signals (`out`, `executor_out`, `accessor_pending_*`) never re-evaluated when
those changed — **under**-sensitivity, the one direction CLAUDE.md's `sorry:` exception says is a
real bug, and iverilog emits no diagnostic for it. Measured: `hazard_rs1` latched high at the first
RAW hazard of every program and never fell, so `make waves`' own baked-in loop executed two
instructions and then span (0 memory writes in 200 cycles, on `main`, before this change), and no
`.S` program could reach `tohost`. yosys evaluates the function correctly, so cxxrtl and the whole
formal ladder were unaffected — which is precisely why it went unnoticed: the leg the verification
table calls the microscope was reporting nothing, and nothing it reported was wrong. The two
assigns are now written out. All 52 `.S` programs run to `PASS` under `vvp` with the per-retire
monitor live, the first time that has ever been true. **There is still no tracked runner for it**:
`test/testbench.v` has a baked-in program and no image loader, so the 52/52 above was measured with
a scratch harness. `make waves` is the tracked iverilog exercise and it now executes real
instructions rather than deadlocking.

What does work: `yosys ... write_cxxrtl` elaborates the current RTL cleanly — **one** warning,
`Deep recursion in AST simplifier`, which is a recursion-depth notice rather than a correctness
signal and is the single entry on the `elaborate` gate's allowlist; everything else yosys prefixes
`Warning:` is promoted to an error there (`.github/workflows/ci.yml`, which cross-references this
note). It is not zero, and this line said zero until ADR-0037. The
cxxrtl binary builds and runs, the whole `.S` suite passes under it with `test/EXPECTED_FAIL`
empty, `make test-units` passes (six benches: `exec_tb` — 10,000 randomized differential vectors
per op across `mul`/`mulh`/`mulhu`/`mulhsu`/`div`/`divu`/`rem`/`remu` **and**
`sll`/`srl`/`sra`, plus 384 directed shift vectors per shift op sweeping every amount 0-31 with a
clean and a dirty rs2 so the `rs2[4:0]` mask is checked rather than coincided with — `mem_tb`,
`decoder_tb`,
`regfile_tb` — which covers the write-through bypass and x0 semantics — `csr_tb`, which covers
`rtl/csrs.v`'s read mux, its `implemented` address set, its WARL masks and its trap-entry/`mret`
port, and `monitor_tb`, which checks the oracle itself rather than the core), and **all three**
component proofs — decoder, executor and `pcloop` — pass by k-induction (`mode prove`; re-run and
read off each `sby` summary, not inferred from a green job. See ADR-0017 for what the decoder proof
does and does not establish, and ADR-0046 for what `pcloop` discharges). `make waves`
now runs the iverilog leg (`testbench.vvp`) instead of the cxxrtl runner, matching the verification
table below, and produces a real `waves.vcd` — **but it grades nothing**; see the first bullet under
"what does not work" above. CI (`.github/workflows/ci.yml`) runs **seven** jobs on every PR:
elaborate, test, components, lint, nonperturbation, monitor-freshness and formal. **Six of the seven
are required** — `elaborate`, `test`, `components`, `monitor-freshness`, `lint`, `formal`, read live
from `gh api repos/thejefflarson/little-cpu/branches/main/protection -q
'.required_status_checks.contexts'` rather than from any comment in the workflow — and
`nonperturbation` is the one that runs without gating. This line named four jobs and stopped there
until the gate inventory; `lint` and `formal` had both been promoted to required in the meantime,
and `ci.yml`'s own comments still said otherwise.

**A Sail co-simulation spike measured what the existing oracles structurally cannot see**
(ADR-0032). Both the RVFI monitor and every `insn_*` ladder check read the core's *self-report*;
`reg_ch0` is the one check that ties that report back to `rtl/regfile.v`, and it is a single
`mode bmc` query at depth 21. An injected extra architectural write outside the retiring
instruction's `rd` (`regs[31] <= wdata` alongside `regs[waddr]`) was **missed by all 49 `.S` tests
with the per-retire monitor live**, caught by `reg_ch0` — and then, gated to fire only after cycle
40, **missed by the entire 78-check ladder** (67 pass / 11 fail, matching `formal/EXPECTED_FAIL`
exactly) while the co-sim reported it in 0.6s with instruction number, PC and both values. The
class is *architectural state no retiring instruction names, corrupted past the BMC bound*. The
mutation was reverted; `rtl/` is untouched by that change.

**Co-simulation now runs the whole suite against a baseline** (ADR-0039). `make cosim-suite` is
`test/run_cosim.sh` in the shape of `test/run_tests.sh`, graded by ADR-0014's set equality in both
directions against `test/COSIM_EXPECTED_FAIL` in ADR-0035's name-and-status format: **52 programs,
50 agree, 10s wall.** The two baselined entries are read-only-CSR *value* divergences no change to
this core could close — `csr.S` on `misa` (Sail derives it from its extension set; matching it needs
seven extensions disabled and a cascade of dependent keys, measured and recorded rather than chased)
and `minstret.S` on `mcycle` (an ISA model has no pipeline, so it cannot count cycles). `trap.S`
agrees 214/214. **The mutation was re-run against the integrated leg rather than inherited**: the
same post-cycle-40 `regs[31] <= wdata` is missed by `make test` at 52/52 and caught by
`make cosim-suite` in 49 of 52 programs (the three that still agree are the three too short to reach
cycle 40), and `csr.S`'s baselined status moves from `DISAGREE AT 17` to `DISAGREE AT 11` — the
second baseline field doing its job.

**`test/COSIM_EXPECTED_FAIL` is now EMPTY and the suite is 52/52** (ADR-0043), and no `rtl/` file
changed to get there. Both entries above were artefacts of how the reference model was configured
or compared, not of this core. The `misa` one was the more important: the model was a
`--config-override` on Sail's **default RV32 machine**, and an override inherits everything it does
not mention — so the thing this repo cross-checks against had A, B, D, F, S, U and V, none of them
implemented here and none of them chosen. `misa` was simply the one register a test read.
`test/sail/rv32imc_zicsr.json` replaces it with a **complete `--config`** describing ADR-0002's
RV32IMC_Zicsr: `--config` is rejected outright if a key is missing, so a Sail version bump that
adds a knob now fails loudly here instead of picking a default nobody read. `mcycle` is handled by
`test/cosim.py`'s `NONCOMPARABLE_CSRS`, which exempts **one register's value at one change** and
prints every exemption it takes. **Fixing `misa` unmasked two divergences it had been hiding**, and
those are the ones worth remembering: `mip.MTIP` and `mie`'s writable bits are read-only zero here
(ADR-0002) and neither is configurable in sail-riscv 0.13.1, so `csr.S`'s assertions about them made
the *reference model* run the program to `fail` — a different branch, not a different value, which
no value exemption can or should paper over. Those assertions moved to `test/csr_tb.v`, which
already made every one of them. **Before adding a line to that baseline, read its header**: the
three questions in it are the decision procedure, and a baseline entry is the last of them.

The same change made `tohost` an 8-byte-aligned `.dword` in `test/asm/riscv_test.h`, which is what
the HTIF protocol its encoding is borrowed from always specified (ADR-0039 amends ADR-0008). It was
a 32-bit `.word` at the base of RAM, so `.data` began four bytes inside the doubleword IO window any
HTIF consumer claims at the symbol; Sail answered every `lw` from `RAM_BASE+4` with zero. **This is
shared test infrastructure both existing sim legs read** — every program's `.data` moves four bytes
and every program executes one extra store — and `make test` (52/52) and `make test-units` were
re-run green in the same change. The verdict macros write the upper word first and the verdict last,
so the reference model terminates on the same store the cxxrtl runners stop on; the `objcopy
--strip-symbol=tohost` workaround and the spin-loop convergence heuristic are both gone.

**`86e2721` ports the riscv-formal ladder to `littlecpu`** — the first time any riscv-formal
check has run against the pipelined core since the 2021 teardown. `formal/wrapper.v` speaks the
real bus (no handshake: `imem_data`/`mem_rdata` are free every cycle, per invariant 1 and
ADR-0015). Of 78 generated checks: **55 of 70 `insn_*` pass**; `pc_fwd`, `pc_bwd`, `liveness`,
`unique`, `causal`, `imemcheck` and `dmemcheck` pass; `cover` reaches all five goals, including
≥2 loads and ≥2 stores and ≥2 uncompressed and ≥2 compressed retires in one trace — which is what
rules out a vacuous harness. **Read ADR-0023 before quoting any of these numbers.** Every one of
the 15 failures is accounted for (9 = the M3 trap gap, 2 = the C.JR/C.JALR defect, 4 = the ALTOPS
divide defect — all three since fixed, and the last two no longer have bullets of their own above),
`reg` was inconclusive *under the `smtbmc yices` engine that run used* (ADR-0024 replaced it; see
the struck bullet above), and everything ran under `RISCV_FORMAL_ALTOPS`, so
a green `insn_mul` says nothing whatever about multiplication (ADR-0010). **M2 is not reached.**

## Invariants — do not break these

These are the design. Violating one is a bug even if tests pass.

1. **Fetch is combinational and the decoder owns the PC.** There are never wrong-path instructions.
   **No flush logic may be introduced, ever.** If a change appears to need one, something upstream
   has gone wrong — stop and reconsider rather than adding a kill signal.
2. **All traps are detected and committed in decode.** Nothing faults after decode. This is what
   makes CSR commit precise without a reorder buffer.
3. **Every inter-stage struct carries a `valid` bit.** A bubble is `valid = 0`; retire is `valid`
   reaching writeback, which gates `wen` and drives `rvfi_valid`.
4. **Hazards are handled by stall-only interlock in decode.** No forwarding network. Adding one is
   a CPI-only optimization and requires a new ADR.
5. **CSR instructions and `mret` serialize** — held in decode until execute/access/writeback drain.
6. **The regfile read is synchronous and takes one cycle** (ADR-0042). Decode observes, in the
   cycle it issues, the architectural value of rs1/rs2 **including a writeback committed in that
   same cycle** — two forwarding points in fabric (write-first into the read register, then the
   write-through bypass) are what make that true. The second sentence is the load-bearing one and is
   unchanged: it is what ADR-0004's stall-only scoreboard depends on, and why decode needs no
   forwarding path for the writeback slot. The flip-flop array that used to make the read
   combinational was one implementation of that contract, not the contract.
7. **`test/monitor.v` is generated but tracked.** Regenerate it; never hand-edit it.
8. **Stalls are a single global broadcast: five reasons over two mechanisms** (ADR-0026 amending
   ADR-0009; ADR-0042 adding the fifth). The reasons are the decode scoreboard, the divider, the
   accessor's one-cycle load-response turnaround (ADR-0015), CSR serialization (invariant 5), and
   the operand-fetch cycle (invariant 9) — which bubbles, exactly like a RAW hazard, and adds no
   third mechanism. The **mechanisms** are
   what actually matters, and three non-local rules hold them together, each breakable silently:
   (a) while `divider_stall` or `accessor_stall` is asserted, decode **holds** `decoder_out`
   unchanged rather than bubbling it, and nothing downstream may consume it that cycle; a RAW
   hazard **bubbles** instead, and so does CSR serialization — the CSR instruction has not issued
   yet, so it folds into the existing `hazard` term rather than becoming new machinery. (b) Every
   in-flight non-`x0` `rd` is visible to the scoreboard on every cycle between issue and the
   regfile write-through, with no gap: `decoder_out` → `executor_out` → `accessor_pending` (loads
   only) → write-through. (c) The serialization drain predicate reads **four** slots, not the
   scoreboard's three: `accessor_pending_valid` covers loads only, so `accessor_out.valid` is
   routed into the decoder separately (`rtl/littlecpu.v`) — without it a *store* in flight is
   invisible and a CSR instruction issues early, giving a `minstret` that is wrong only when a
   store happens to be in flight.
9. **The regfile's answer belongs to the address pair presented in the PREVIOUS cycle** (ADR-0042).
   Decode holds `pc` across the pair and bubbles — that is `operand_stall`, and it is why invariant 6
   costs a cycle. Nothing may consume `reg_rs1`/`reg_rs2` in a cycle whose address pair differs from
   the one presented last cycle. Same shape as invariant 8's rules: true today only because
   `operand_stall` enforces it, breakable silently by a later change, and not visible from reading
   `rtl/regfile.v` alone. `test/regfile_tb.v` pins it directly — it fetches x5, points `rs1` at x6 in
   the use cycle, and asserts that **x5's** value is what comes back.

## ISA target

RV32IMC_Zicsr, M-mode only. `misa = 0x4000_1104`.

Traps: illegal instruction = 2, breakpoint = 3, load address misaligned = 4, store address
misaligned = 6, environment call from M-mode = 11. **Instruction-address-misaligned (0) is
unreachable** — C makes 2-byte targets legal — and is not implemented. No interrupts: `mie` and
`mip` are read-only zero.

The C extension stays because **code density is a product constraint** on the up5k, not a
preference. See ADR-0002 and ADR-0003.

## Commands

```sh
make setup          # install the RISC-V toolchain (brew on macOS)
make test           # assemble test/asm/*.S, run under cxxrtl, pass/fail table with
                    # per-program retire / spec-checked-retire counts, graded against
                    # test/EXPECTED_FAIL (set equality) and test/OBSERVED_FLOOR (>=)
make waves          # iverilog + VCD (testbench.vvp's baked-in program) -> waves.vcd.
                    # GRADES NOTHING: the per-retire monitor is live but only
                    # $displays, so this exits 0 on a mismatch. Read the output.
make monitor-check  # regenerate test/monitor.v at the pin and diff
make fit            # the ONE area number: nextpnr logic cells on up5k/sg48 (ADR-0038).
                    # Placement always fails (231 SB_IO vs 39) and that is expected --
                    # the utilisation table printed before placement is the measurement.
                    # A RATCHET as of ADR-0042: over FIT_MAX_LC (4400) exits nonzero.
                    # RATCHET DECLARED, NEVER PULLED: no workflow runs this, so the
                    # only thing that has ever enforced it is a human typing it. The
                    # mechanism itself is live -- `make fit FIT_MAX_LC=4200` on the
                    # 4236-cell tree exits 2 -- so this is an unrun gate, not a
                    # broken one. Run it by hand on anything that touches rtl/.

make -C formal components_decoder   # component proofs. THREE tasks, all with real assertions:
make -C formal components_executor  # decoder, executor, pcloop -- the assertion-free fetcher /
make -C formal components_pcloop    # accessor / writeback tasks were deleted, not left unrun.
                                    # pcloop is the composed fetcher+decoder proof that discharges
                                    # ADR-0017's assume(in.pc == pc); it needs `smtbmc boolector`
                                    # (see formal/components.sby) and is on CI as of ADR-0046
make -C formal check                # the riscv-formal ladder (82 checks; see ADR-0023). ALWAYS a
                                    # fresh run -- it deletes checks/ first (ADR-0040)
make -C formal check-baseline       # re-grade a finished ladder: EXPECTED_CHECKS + EXPECTED_FAIL
make -C formal nonperturbation      # ADR-0047: the RVFI instrumentation is UNREAD by the core.
                                    # yosys + python3 only, ~9s, on the PR gate. NOT sequential
                                    # equivalence -- it replaced equiv.sh, which never converged

make sail-setup     # once: fetch the pinned sail-riscv release into tools/sail/
make cosim-run      # Sail co-simulation on ONE program (PROG=add.S). Opt-in; see ADR-0032
make cosim-suite    # the whole .S suite under co-sim, graded against COSIM_EXPECTED_FAIL
```

Toolchain: macOS `brew install riscv64-elf-gcc`; Linux `apt install gcc-riscv64-unknown-elf`.
Tests are freestanding assembly (`-march=rv32imc_zicsr -mabi=ilp32 -nostdlib -T sections.lds`), so
no multilib or newlib is needed. Formal needs a pinned YosysHQ OSS CAD Suite.

## Verification — three legs, each load-bearing

| Leg | Role | Catches what the others can't |
|---|---|---|
| **cxxrtl** | primary runner | real mul/div arithmetic (formal runs under ALTOPS), long/randomized runs |
| **iverilog** | microscope | waveforms, `$display`, second elaboration frontend |
| **riscv-formal** | oracle | exhaustive per-instruction semantics, pipeline corners |

**The iverilog leg was inert for the whole of M1 — `a4662a2` to `6309b3e` — and this table did not
know** (ADR-0037). `rtl/decoder.v`'s hazard scoreboard called a `function automatic
live_producer(r)` from two continuous assigns, and iverilog derives such an assign's sensitivity
list from the **call's arguments**, so a body that also read `out`/`executor_out`/
`accessor_pending_*` never re-evaluated when those changed. Under-sensitivity — the one direction
the `sorry:` exception below says is a real bug — and iverilog emits no diagnostic for it at all.
Measured on `main` before the fix: `hazard_rs1` latched high at the first RAW hazard and never
fell, the PC advanced `0x0`→`0x4` and froze, and `make waves` did **0 memory writes in 200 cycles**
(201 reads, all to address 0). After the fix the same program does 22 writes. yosys evaluates the
function correctly, so cxxrtl and the whole ladder were unaffected — which is exactly why nothing
caught it. **A leg that cannot fail is not a leg**: treat a green iverilog run as evidence only if
you can point at something it would have failed on.

**riscv-formal runs with `RISCV_FORMAL_ALTOPS`, so it never checks the real multiplier or divider.**
That arithmetic is covered only by the `.S` suite and the executor component proof. Do not assume a
green formal ladder means the ALU is correct.

**There is a fourth thing you can run, and it is deliberately not a leg** (ADR-0032, ADR-0039).
`make cosim-suite` diffs the core's *real* `regs_a` array — read through cxxrtl `debug_items` by
`test/cosim.cc`, which touches no `rvfi_*` signal at all — against the Sail RISC-V model, over the
whole suite, graded against `test/COSIM_EXPECTED_FAIL`: **52 of 52 agree, 7.3s** (ADR-0043; it was
50 of 52 at ADR-0039), against 7.3s for
`make test`. `make cosim-run PROG=x.S` does one program. It is **not** on `make test`'s path and
**not** in CI's required set, on purpose: `make test` must keep working on a machine with no Sail
installed (demonstrated by moving `tools/sail` aside), and **adding it to branch protection is what
ADR-0032 forbids**. The way a change gates on it is by carrying its pre/post output in the pull
request. `test/cosim.cc` reading no `rvfi_*` signal is the property that makes this leg worth
having; do not "align" it against `rvfi_valid`.

`test/monitor.v` rides along in both sim legs (`b2dafcc`), so every run is self-checking per-retire —
a test that corrupts state transiently but converges to the right final registers fails loudly, not
just end-state assertions passing. Both legs read the sanitized `test/monitor.sim.v`, which is
therefore load-bearing for correctness and not merely for elaboration: a change to it is a change to
the oracle (ADR-0019).

## Engineering rules in force

- **Compiler and elaboration warnings are errors.** Fix them; don't silence them.
  **One documented exception**, and only this one: iverilog emits `sorry: constant selects in
  always_* processes are not fully supported` for every struct-field read inside an
  **`always_comb`** block — 20 of them, all from `rtl/writeback.v`, against
  `in[952:0]` (`accessor_output`, with the `RISCV_FORMAL*` macros on — 953 bits; it was 952 before
  `6309b3e` added `rvfi_shadow.trap`, and the count held at 20 across that change). It cannot build a precise sensitivity entry for a constant
  part-select, so it falls back to whole-struct sensitivity.
  **That fallback is provably safe**: over-sensitivity re-evaluates redundantly and can never
  yield a stale value; only *under*-sensitivity is a correctness bug. Everywhere the select was
  cheap to hoist into a named continuous assign it has been (`rtl/decoder.v`'s register-index
  fields, `rtl/accessor.v`'s payload fields, `test/testbench.v`'s ROM index, and
  `rtl/writeback.v`'s twelve CSR payload reads) — and it reads better besides.
  `rtl/writeback.v` keeps the `in.` form in its two `always_comb` blocks because hoisting all
  30 reads would add more plumbing than it removes. Copying the struct to a local does not help
  (still a constant select); `always @(in)` would, but trades away `always_comb`'s latch
  checking for exactly the sensitivity iverilog already infers.
  **The diagnostic is specific to `always_comb`**, where the sensitivity list is *inferred*.
  `rtl/executor.v` emits **zero** of these despite its `case (1'b1)` over 39 `in.is_*` flags,
  because that body sits in `always_ff @(posedge clk)`, whose sensitivity is written out.
  Attribute them by struct width rather than by the diagnostic, which carries **no filename at
  all** (it prints `:0:`): `$bits` gives `decoder_output` 943, `executor_output` 921,
  `accessor_output` **953**, and only the last matches `in[952:0]`. ADR-0034 printed
  `iverilog -g2012 -s <mod> rtl/structs.v rtl/<mod>.v` for this; **that command does not run** —
  it needs `-I./rtl/`, and even then reports 0 for every module, because the reads in question are
  inside `ifdef RISCV_FORMAL` and it passes no macros (ADR-0037). Use `make testbench.vvp`.
  **Do not add new ones outside `rtl/writeback.v`**, and prefer a continuous
  assign for any new struct-field read in an `always_comb` — that is what holds the count at 20
  (ADR-0034; this bullet named `rtl/executor.v` until then, which was measurably wrong).
- **Every non-trivial change adds or updates tests and runs the full suite** before being declared
  done. Elaboration succeeding is not a substitute for tests passing.
- **Never commit build artifacts.** `test/rtl.cc`, `sim`, `*.vvp`, `*.vcd`, `rvfi_macros.vh`, and
  `formal/` output dirs are all generated. (`test/monitor.v` is the one deliberate exception.)
- **riscv-formal is SHA-pinned.** Bumping the pin requires regenerating `test/monitor.v` and
  rerunning the ladder.
- **Shared repo knowledge must not depend on the tracker.** No ticket IDs in code, comments, ADRs,
  docs, or commit messages — a bare ticket number is meaningless to anyone reading this repo
  without access to the tracker, and it rots the moment the tracker does. Cite the thing that
  lives here instead: the **ADR** that decided it, the **commit SHA** that landed it, or — best —
  just say what the reason *is*. "Held for one cycle because the memory registers `mem_rdata`
  (ADR-0015)" beats a ticket number and always will.
  **PR titles and descriptions are the exception** — those live on GitHub, not in the checked-out
  repo, so a ticket ID there costs nothing and buys automatic issue closing. Naming the tracker
  itself once under Pointers is likewise fine. It is the *scattered* IDs that rot.
- Prefer verified/first-party GitHub Actions. Simplest approach unless asked otherwise.

## Milestone ladder

| | Milestone | Green means |
|---|---|---|
| M0 | Foundation | this file, riscv-formal SHA-pinned, dead references gone |
| M1 | Finish the pipeline | all RV32IM `.S` tests pass under cxxrtl — **reached, `a4662a2`** |
| M2 | **Parity checkpoint** | **all six** conditions below hold. An empty `formal/EXPECTED_FAIL` is *one* of them and on its own means nothing — it was reached at `6309b3e` and M2 was not (ADR-0037) |
| M3 | Past the old core | CSRs + machine-mode traps — **both landed** (`rtl/csrs.v`; trap commit in `rtl/decoder.v`) |
| M4 | Full ladder + CI | nightly formal green; tag a release (PR gate `c66527d`; nightly `86e2721`, report-only until ADR-0022) |

M1 is reached. **M2 is the milestone that erases the verified→unverified regression** — treat it
as the real finish line, not M1. `b2dafcc` cleared M2's blocker (RVFI is driven, the monitor is live
in both sim legs) and `86e2721` landed the ladder port itself (`wrapper.v` / `checks.cfg` /
`imemcheck` / `dmemcheck` / `cover` / `equiv.sh`, ADR-0006) — but **M2 is not reached.** M2's own
wording is "re-proves everything the serialized core proved", and 15 red checks plus an inconclusive
`reg` plus a non-converging `equiv.sh` is not that (ADR-0047 has since deleted `equiv.sh`; see
term 4). ADR-0023 lists what closing it takes. An empty
`formal/EXPECTED_FAIL`, read together with `formal/EXPECTED_CHECKS` per ADR-0033, is a **necessary
condition and not a sufficient one** — this file called it "the signal" until ADR-0037, which is
the language of sufficiency for something that was only ever necessary, and a change nobody
believes completes the milestone duly satisfied it.

`rtl/csrs.v` took that baseline from 11 entries to 9 (both `csrw_*`) on a 79-check
ladder; landing `ill_ch0` red made it 10 on an 82-check ladder; **the trap change took it to zero
on that same 82-check ladder.** All ten were the trap gap — nine misalignment, one
illegal-instruction — and all ten closed together, which is the attribution behind them holding.

**The baseline is empty and M2 is still not reached** (ADR-0037). **M2 requires all six of the
following.** Closing it means deleting terms from this list — a burn-down with the same shape as
`formal/EXPECTED_FAIL` itself. **Term 4 is met as of ADR-0047**; ADR-0045 additionally closes terms 2
and 3, which this list has not yet been rewritten for. Read each term's own text, not this sentence:

1. **`formal/EXPECTED_FAIL` empty and `formal/EXPECTED_CHECKS` matching — MET at `6309b3e`**: 82
   generated, 82 pass, both set equalities in both directions. **ADR-0045 reopened this and ADR-0046
   closed it again**, which is the more useful thing to know about it: the depths those 82 checks run
   at were derived against a pipeline two changes old, and a depth below its floor goes green having
   stopped asking. They are re-derived now, and the derivation is *measured* — `hang` gives the
   worst-case first retire (F = 6), `liveness` the worst-case retire gap (G = 4), both in seconds,
   both re-runnable. No depth moved. `insn 15` and `reg 15 20` clear their floors by **one cycle**,
   so **any change that adds a stall reason, lengthens a stage, or widens the scoreboard past its
   three fixed slots must re-measure F and G before it lands.** Two of the last three such changes
   did not.
2. **The mul/div checks run without `RISCV_FORMAL_ALTOPS`**, or ADR-0010's gap is closed by a named
   oracle that does. Today `insn_mul`/`insn_div`/`insn_rem` passing says nothing whatever about the
   real multiplier or divider.
3. **`reg_ch0` returns a verdict** rather than exhausting its budget (ADR-0023). It is the one
   check tying RVFI's self-report back to `rtl/regfile.v`; ADR-0032 measured exactly that hole by
   injecting an architectural write the entire ladder missed.
4. ~~**`formal/equiv.sh` converges**~~ — **MET at ADR-0047**, on ADR-0037's second clause: the
   guarantee is proven another way and `equiv.sh` is deleted rather than fixed. The mechanism is
   `make -C formal nonperturbation`, a structural check that the `-D RISCV_FORMAL` build with its
   `rvfi_*` ports deleted sweeps to a netlist identical to the plain build. **Amending a criterion is
   not meeting it** (ADR-0045's own closing line): this term closed by taking an "or" clause
   ADR-0037 wrote for the case, and it is the third of the six to close that way.
5. **`formal/complete` passes**, or every check it declines has a recorded reason. It fails today
   on `ecall`/`ebreak`/`mret`/CSR retires, on no gate.
6. **The nightly can go red, and is green.** It can go red as of `1961234` — but not for the reason
   this file used to give. The `|| true` and the missing `-k` were fixed earlier; what remained is
   that the graded comparison was piped into `tee`, and a `run:` block without an explicit `shell:`
   key is `bash -e {0}` — errexit but **not** pipefail — so its exit status was `tee`'s, always 0.
   **ADR-0022's "that comparison step's exit status is the job's real signal" had therefore never
   held**, and stayed accidentally true only because the ladder kept matching its baseline
   (ADR-0037). General rule: never put the graded command in a pipeline in a `run:` block.

**Two caveats qualify what any green ladder here means, and neither is closable by burning
down a list.** Every check is `mode bmc` — PASS means "no counterexample within this check's
configured depth", not that the property holds, and there is no `mode prove` anywhere on the
ladder. And riscv-formal ships **no spec model at all** for `ecall`/`ebreak`/`mret`/`csrr*` at the
pin, so the behaviour M3 actually added is checked by `test/asm/trap.S`, `test/csr_tb.v`,
`test/decoder_tb.v` and `rtl/decoder.v`'s own component proof — against assertions this repo wrote,
not against an oracle. An empty baseline is loudest exactly where the ladder is quietest.

## Pointers

- Design brief: [`docs/ideas/finish-the-rewrite.md`](docs/ideas/finish-the-rewrite.md)
- Area/fit brief: [`docs/ideas/fit-the-core-on-the-up5k.md`](docs/ideas/fit-the-core-on-the-up5k.md) —
  **the core's logic now fits: 4236/5280 logic cells, 80%**, down from 6971/132%, which the
  synchronous-read regfile achieves on its own (ADR-0042). `make fit` is a ratchet against
  `FIT_MAX_LC`. **The design still does not place**, and that is expected and unrelated to logic:
  the fit top presents **231 `SB_IO` against sg48's 39**, so nextpnr always fails on a pad. A
  placing design needs a real pinout, which means the SoC memory system, which ADR-0038 decision 1a
  puts out of scope. Read ADR-0038 before quoting any area number: yosys cell counts are blind to
  unpaired flip-flops and have produced two wrong estimates already. **The brief's two `reg_ch0`
  claims are false and are struck in place** (ADR-0040), and its recommendation of a *negedge* read
  is superseded (ADR-0042).
- **`make fit` has a churn floor of roughly ±50 cells, and a ratchet has to sit above it.**
  Measured by sweeping `rtl/executor.v`'s `mul_div_counter` across four widths that are all
  functionally identical — the counter's range is 0..32 and yosys already constant-folds every bit
  above 5, so the **flip-flop count is byte-identical in all four** (1757). The logic cells are not:
  `[6:0]` (what is checked in) → **6971**, `[5:0]` → 7008, `[7:0]` → 7027, `[8:0]` → 7024 —
  non-monotonic, spanning 56 cells, entirely LUT4 churn from ABC re-mapping and nextpnr packing
  (LUT4-only 5159 → 5198, LUT4+DFF 587 → 587, DFF-only 1170 → 1170, CARRY 41 → 39 for the first
  two). Each figure is reproducible run to run; it is the *edit* the number is unstable under, not
  the tool. Two consequences: **a `make fit` delta smaller than ~50 cells is not evidence of
  anything**, and ADR-0038 decision 1a's utilisation ratchet needs a margin wider than this floor
  or it will go red on changes that synthesize to the same hardware.
  This is also why `mul_div_counter` is still `[6:0]` for a value that never exceeds 32. Narrowing
  it is a legibility change that removes no register and **costs 37 cells measured**; ADR-0038
  rejected the shifter merge at 19 cells *saved* on legibility grounds, and accepting a hygiene
  change at 37 cells *spent* would cut against that ruling. Read this before proposing the
  narrowing again — it is measured and declined, not overlooked.
- Decisions: [`docs/adr/`](docs/adr/) — **forty-seven ADRs, forty-six of them accepted**, plus a
  deferred list. Re-derived by counting: `ls docs/adr/*.md | wc -l` is 48, one of which is
  `README.md`, and the status column in that README carries exactly one non-accepted entry
  (ADR-0016, superseded by ADR-0018). This line said "forty-five accepted" and was two behind.
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic)
- Work is tracked in Linear, project **Little CPU** (team JEF). Named here so you know where the
  queue is — but nothing in this repo should depend on it, and no ticket ID belongs in the code.

**Deferred behind future ADRs** — forwarding network, radix-4 divider, FPGA
timing closure, interrupts. (The negedge-BRAM regfile came off this list in ADR-0042, **decided
against**: it is 99 cells cheaper and costs no cycles, but the generated ladder cannot model a
mixed-polarity design at all, so `reg_ch0` would never again run against `rtl/regfile.v`.) Each trades away simplicity the current design depends on; none are
safe to build while the core is unverified. (Sail co-sim came off this list in ADR-0032: the
harness exists and ADR-0039 runs it over the whole suite behind `make cosim-suite`, still opt-in;
wiring it into `make test` or CI's required set is decided *against* and needs a new ADR to change.)
