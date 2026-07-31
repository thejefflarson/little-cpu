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

**M1 is reached** (`a4662a2`). `rtl/regfile.v` is combinational-read with write-through
bypass, every inter-stage struct carries a `valid` bit, and decode runs a stall-only hazard
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
`intr`/`mode`/`ixl`/the CSR fields are hardwired constants in `rtl/littlecpu.v` (no traps or CSRs
exist yet — M3). `test/monitor.v` stays pristine and tracked; a gitignored, build-time-derived
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

`test/EXPECTED_FAIL` is **no longer empty**: `csr.S`, `minstret.S` and `trap.S` were seeded there
ahead of the RTL that pays them off, which is the direction ADR-0014's burn-down contract was
designed for. Two of the three are paid off (below); `trap.S` remains, so `make test` is
**51 pass / 1 expected-fail** and still exits 0 only on set equality.
`test/run_tests.sh` also grew a `MONITOR-ERROR` label for runner exit 4, which used to fall into
`RUNNER-ERROR` and read as "the sim would not start" when it actually means the per-retire oracle
disagreed with the core mid-run. **Nothing here checks M3 semantics against an oracle**:
riscv-formal ships no spec model for `csrr*`/`ecall`/`ebreak`/`mret` at the pinned SHA, so
`spec_valid` is 0 for every one of them and the monitor's whole semantic block is skipped. The
trap and CSR tests assert on values their own handler recorded. Error 133 ("expected intr after
trap") stays unreachable in this single-channel config — `shadow_pc_valid <= !rvfi_trap` gates 130
and 133 off for the retire after a trap — so `rvfi_intr` hardwired to 0 does not break anything.

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

What does not work right now:

- **The ALTOPS divide branch reads stale operands.** `rtl/executor.v:221-224` reads `in.rs1`/
  `in.rs2` in the `divide` state, one cycle after issue, when decode has already bubbled
  `decoder_out` (ADR-0009). Scoped to `` `ifdef RISCV_FORMAL_ALTOPS ``, so the synthesized divider
  is fine and `components_executor` still passes — but it means `insn_div`/`divu`/`rem`/`remu` fail,
  and the divider's operand routing has **no** formal coverage in any form (ADR-0010 already says
  the arithmetic has none). The non-ALTOPS branch six lines above gets this right and explains why.
- **`formal/equiv.sh` runs but does not converge** — `equiv_induct` fails on `executor.mul_div_y`
  bits, exactly the outcome ADR-0020 predicted. So ADR-0006's guarantee that RVFI instrumentation
  does not perturb core behaviour is still **argued, not proven**. The argument is that every
  `ifdef RISCV_FORMAL` block is write-only with respect to the core. Any future RVFI change must be
  read against that: an `ifdef`'d value reaching a non-`ifdef`'d signal breaks it, and CI would not
  notice.
- **`reg` is inconclusive** (ADR-0023) — a single depth-21 BMC query that does not return in a
  practical budget. It is the check that ties RVFI's self-report to the actual register file, so
  without it the other 55 passing checks establish that the core's story about itself is
  spec-consistent, not that the story is true.
- **A green ladder is narrower than it reads** (ADR-0033). `formal/checks.cfg`'s `[depth]` table is
  the list of checks that *exist*, not a tuning table: `genchecks` drops any check with no depth
  line, silently, and `formal/check-baseline.sh` globs directories `sby` creates only for checks
  that ran — so a never-generated check is missing from the results and from `formal/EXPECTED_FAIL`
  at once, and set equality reports a clean match. Fourteen upstream checks are dropped today;
  `checks.cfg` reasons about ten of them, and one of the other four (`ill`) is applicable, would
  fail today, and is off the ladder. **So `formal/EXPECTED_FAIL` reaching empty — M2's declared
  signal — is necessary but not sufficient until the check count is asserted.** Read it with the
  generated count beside it.
- **The formal nightly cannot go red** (ADR-0022). `.github/workflows/formal-nightly.yml:48` is
  `make -C formal check || true`, and `formal/Makefile`'s `check` has no `-k`, so the sub-make also
  stops scheduling at the first failure. Today's nightly runs a partial ladder and reports it as
  success. It needs `-k` and a `formal/EXPECTED_FAIL` baseline before its green means anything.
- **Three of the five component-proof tasks are vacuous.** `components_fetcher`,
  `components_accessor`, and `components_writeback` contain no assertions at all, only reset
  assumptions, so they "pass" meaninglessly. CI deliberately does not run them; ADR-0006 slates
  them for deletion. A green run of one of those is not a result.

What does work: `yosys ... write_cxxrtl` elaborates the current RTL cleanly with zero warnings, the
cxxrtl binary builds and runs, the `.S` suite passes under it except the one M3 test still seeded in
`test/EXPECTED_FAIL`, `make test-units` passes (six benches: `exec_tb`, `mem_tb`, `decoder_tb`,
`regfile_tb` — which covers the write-through bypass and x0 semantics — `csr_tb`, which covers
`rtl/csrs.v`'s read mux, its `implemented` address set and its WARL masks, and `monitor_tb`, which
checks the oracle itself rather than the core), and the decoder and executor component proofs pass
by k-induction (see ADR-0017 for what the decoder proof does and does not establish). `make waves`
now runs the iverilog leg (`testbench.vvp`) instead of the cxxrtl runner, matching the verification
table below,
and produces a real `waves.vcd`. CI (`.github/workflows/ci.yml`) runs elaborate / test / components
/ monitor-freshness on every PR.

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

**`86e2721` ports the riscv-formal ladder to `littlecpu`** — the first time any riscv-formal
check has run against the pipelined core since the 2021 teardown. `formal/wrapper.v` speaks the
real bus (no handshake: `imem_data`/`mem_rdata` are free every cycle, per invariant 1 and
ADR-0015). Of 78 generated checks: **55 of 70 `insn_*` pass**; `pc_fwd`, `pc_bwd`, `liveness`,
`unique`, `causal`, `imemcheck` and `dmemcheck` pass; `cover` reaches all five goals, including
≥2 loads and ≥2 stores and ≥2 uncompressed and ≥2 compressed retires in one trace — which is what
rules out a vacuous harness. **Read ADR-0023 before quoting any of these numbers.** Every one of
the 15 failures is accounted for (9 = the M3 trap gap, 2 = the C.JR defect below, 4 = the ALTOPS
divide defect below), `reg` is inconclusive, and everything ran under `RISCV_FORMAL_ALTOPS`, so
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
6. **The regfile is combinational-read with write-through bypass.**
7. **`test/monitor.v` is generated but tracked.** Regenerate it; never hand-edit it.
8. **Stalls are a single global broadcast: four reasons over two mechanisms** (ADR-0026, amending
   ADR-0009). The reasons are the decode scoreboard, the divider, the accessor's one-cycle
   load-response turnaround (ADR-0015), and CSR serialization (invariant 5). The **mechanisms** are
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
make test           # assemble test/asm/*.S, run under cxxrtl, pass/fail table
make waves          # iverilog + VCD (testbench.vvp's baked-in program) -> waves.vcd
make monitor-check  # regenerate test/monitor.v at the pin and diff

make -C formal components_decoder   # component proofs
make -C formal check                # the riscv-formal ladder (79 checks; see ADR-0023)

make sail-setup     # once: fetch the pinned sail-riscv release into tools/sail/
make cosim-run      # Sail co-simulation on ONE program (PROG=add.S). Opt-in; see ADR-0032
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

**riscv-formal runs with `RISCV_FORMAL_ALTOPS`, so it never checks the real multiplier or divider.**
That arithmetic is covered only by the `.S` suite and the executor component proof. Do not assume a
green formal ladder means the ALU is correct.

**There is a fourth thing you can run, and it is deliberately not a leg** (ADR-0032). `make
cosim-run` diffs the core's *real* `regs` array — read through cxxrtl `debug_items` by
`test/cosim.cc`, which touches no `rvfi_*` signal at all — against the Sail RISC-V model. All 49
programs agree (24s for the suite, against 7.3s for `make test`). It is **not** on `make test`'s
path and **not** a CI gate, on purpose: `make test` must keep working on a machine with no Sail
installed. Do not wire it in without reading ADR-0032's consequences first — the `tohost`/HTIF
overlap it works around is real and the fix belongs in shared test infrastructure.

`test/monitor.v` rides along in both sim legs (`b2dafcc`), so every run is self-checking per-retire —
a test that corrupts state transiently but converges to the right final registers fails loudly, not
just end-state assertions passing. Both legs read the sanitized `test/monitor.sim.v`, which is
therefore load-bearing for correctness and not merely for elaboration: a change to it is a change to
the oracle (ADR-0019).

## Engineering rules in force

- **Compiler and elaboration warnings are errors.** Fix them; don't silence them.
  **One documented exception**, and only this one: iverilog emits `sorry: constant selects in
  always_* processes are not fully supported` for every struct-field read inside an `always_*`
  block — 20 of them, all from `rtl/executor.v`'s `in.is_*` flags. It cannot build a precise
  sensitivity entry for a constant part-select, so it falls back to whole-struct sensitivity.
  **That fallback is provably safe**: over-sensitivity re-evaluates redundantly and can never
  yield a stale value; only *under*-sensitivity is a correctness bug. Everywhere the select was
  cheap to hoist into a named continuous assign it has been (`rtl/decoder.v`'s register-index
  fields, `rtl/accessor.v`'s payload fields, `test/testbench.v`'s ROM index) — that silenced 17
  of 37 and reads better besides. `rtl/executor.v` keeps the `in.` form because its body is one
  `case (1'b1)` over 39 flags, and hoisting all of them would add ~80 lines of plumbing to the
  module whose legibility matters most. Copying the struct to a local does not help (still a
  constant select); `always @(in)` would, but trades away `always_comb`'s latch checking for
  exactly the sensitivity iverilog already infers. Do not add new ones outside `executor.v`.
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
| M2 | **Parity checkpoint** | the pipelined core re-proves everything the serialized core proved |
| M3 | Past the old core | CSRs + machine-mode traps — CSRs done (`rtl/csrs.v`); traps are next |
| M4 | Full ladder + CI | nightly formal green; tag a release (PR gate `c66527d`; nightly `86e2721`, report-only until ADR-0022) |

M1 is reached. **M2 is the milestone that erases the verified→unverified regression** — treat it
as the real finish line, not M1. `b2dafcc` cleared M2's blocker (RVFI is driven, the monitor is live
in both sim legs) and `86e2721` landed the ladder port itself (`wrapper.v` / `checks.cfg` /
`imemcheck` / `dmemcheck` / `cover` / `equiv.sh`, ADR-0006) — but **M2 is not reached.** M2's own
wording is "re-proves everything the serialized core proved", and 15 red checks plus an inconclusive
`reg` plus a non-converging `equiv.sh` is not that. ADR-0023 lists what closing it takes; the
`formal/EXPECTED_FAIL` baseline of ADR-0022 reaching empty is the signal — read together with the
generated check count, per ADR-0033. `rtl/csrs.v` took that baseline from 11 entries to 9 (both
`csrw_*`) on a 79-check ladder; the remaining 9 are all the trap gap, so the next M3 step closes
them or the attribution behind them was wrong.

## Pointers

- Design brief: [`docs/ideas/finish-the-rewrite.md`](docs/ideas/finish-the-rewrite.md)
- Decisions: [`docs/adr/`](docs/adr/) — thirty-three accepted ADRs, plus a deferred list
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic)
- Work is tracked in Linear, project **Little CPU** (team JEF). Named here so you know where the
  queue is — but nothing in this repo should depend on it, and no ticket ID belongs in the code.

**Deferred behind future ADRs** — forwarding network, radix-4 divider, negedge-BRAM regfile, FPGA
timing closure, interrupts. Each trades away simplicity the current design depends on; none are
safe to build while the core is unverified. (Sail co-sim came off this list in ADR-0032: the
harness exists, opt-in; wiring it into `make test` or CI has not been decided.)
