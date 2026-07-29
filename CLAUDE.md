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
scoreboard (ADR-0004 / ADR-0009 / ADR-0015). `make test` is **47/47** and `test/EXPECTED_FAIL` is
empty, so it is a plain all-pass gate — ADR-0014's set-equality check still runs in both directions,
so an unexpected *pass* is caught too. The same change fixed three datapath defects found on the way:
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
`test/monitor.sim.v` (Makefile) carries the two build-time fixes it needs — stripping the
`$time`-in-`$display` yosys can't elaborate, and ADR-0019's DIV/REM signedness repair — and **both**
legs read it, so they cannot drift into checking different specs. `make test` is now per-retire
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

What does not work right now:

- **`formal/checks.cfg` and `formal/wrapper.v` still reference the deleted core** — `checks.cfg`'s
  `[script-sources]` reads `rtl/riscv.v` and `rtl/alu.v`, and `wrapper.v` instantiates `riscv`.
- **`formal/equiv.sh` does not run**, so ADR-0006's guarantee that RVFI instrumentation does not
  perturb core behaviour is **argued, not proven** (ADR-0020). The argument is that every
  `ifdef RISCV_FORMAL` block is write-only with respect to the core. Any future RVFI change must be
  read against that: an `ifdef`'d value reaching a non-`ifdef`'d signal breaks it, and CI would not
  notice.
- **Three of the five component-proof tasks are vacuous.** `components_fetcher`,
  `components_accessor`, and `components_writeback` contain no assertions at all, only reset
  assumptions, so they "pass" meaninglessly. CI deliberately does not run them; ADR-0006 slates
  them for deletion. A green run of one of those is not a result.
- **`make waves` is `waves.vcd: sim`** — a cxxrtl target, not the iverilog+VCD flow the Commands
  section below describes. Unverified either way.

What does work: `yosys ... write_cxxrtl` elaborates the current RTL cleanly with zero warnings, the
cxxrtl binary builds and runs, the full `.S` suite passes under it, `make test-units` passes, and
the decoder and executor component proofs pass by k-induction (see ADR-0017 for what the decoder
proof does and does not establish). CI (`.github/workflows/ci.yml`) runs elaborate / test /
components / monitor-freshness on every PR.

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
8. **Stalls are a single global broadcast with three sources** — the decode scoreboard, the
   divider, and the accessor's one-cycle load-response turnaround (ADR-0015). Two non-local rules
   hold it together, and a later change can break either silently: (a) while `divider_stall` or
   `accessor_stall` is asserted, decode **holds** `decoder_out` unchanged rather than bubbling it,
   and nothing downstream may consume it that cycle; a RAW hazard does the opposite (bubble). (b)
   Every in-flight non-`x0` `rd` is visible to the scoreboard on every cycle between issue and the
   regfile write-through, with no gap: `decoder_out` → `executor_out` → `accessor_pending` (loads
   only) → write-through.

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
make waves          # iverilog + VCD for one program (currently broken — see above)
make monitor-check  # regenerate test/monitor.v at the pin and diff

make -C formal components_decoder   # component proofs
make -C formal check                # the riscv-formal ladder (needs the M2 port)
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

`test/monitor.v` rides along in both sim legs (`b2dafcc`), so every run is self-checking per-retire —
a test that corrupts state transiently but converges to the right final registers fails loudly, not
just end-state assertions passing. Both legs read the sanitized `test/monitor.sim.v`, which is
therefore load-bearing for correctness and not merely for elaboration: a change to it is a change to
the oracle (ADR-0019).

## Engineering rules in force

- **Compiler and elaboration warnings are errors.** Fix them; don't silence them.
- **Every non-trivial change adds or updates tests and runs the full suite** before being declared
  done. Elaboration succeeding is not a substitute for tests passing.
- **Never commit build artifacts.** `test/rtl.cc`, `sim`, `*.vvp`, `*.vcd`, `rvfi_macros.vh`, and
  `formal/` output dirs are all generated. (`test/monitor.v` is the one deliberate exception.)
- **riscv-formal is SHA-pinned.** Bumping the pin requires regenerating `test/monitor.v` and
  rerunning the ladder.
- **The repository is self-contained. Never cite an issue tracker in code, comments, commit
  messages, ADRs, or docs.** A ticket ID is meaningless to anyone reading this repo without
  access to that tracker, and it rots the moment the tracker does. Cite the thing that lives
  here instead: the **ADR** that decided it, the **commit SHA** that landed it, or — best — just
  say what the reason *is*. "Held for one cycle because the memory registers `mem_rdata`
  (ADR-0015)" beats "per JEF-615" and always will.
- Prefer verified/first-party GitHub Actions. Simplest approach unless asked otherwise.

## Milestone ladder

| | Milestone | Green means |
|---|---|---|
| M0 | Foundation | this file, riscv-formal SHA-pinned, dead references gone |
| M1 | Finish the pipeline | all RV32IM `.S` tests pass under cxxrtl — **reached, `a4662a2`** |
| M2 | **Parity checkpoint** | the pipelined core re-proves everything the serialized core proved |
| M3 | Past the old core | CSRs + machine-mode traps |
| M4 | Full ladder + CI | nightly formal green; tag a release (PR gate landed, `c66527d`) |

M1 is reached. **M2 is the milestone that erases the verified→unverified regression** — treat it
as the real finish line, not M1. `b2dafcc` cleared M2's blocker (RVFI is driven, the monitor is live
in both sim legs — see Current State above), but M2 itself is not reached: the riscv-formal ladder
port (`wrapper.v` / `checks.cfg` / `imemcheck` / `dmemcheck` / `cover` / `equiv.sh`, ADR-0006) is
separate, outstanding work.

## Pointers

- Design brief: [`docs/ideas/finish-the-rewrite.md`](docs/ideas/finish-the-rewrite.md)
- Decisions: [`docs/adr/`](docs/adr/) — twenty accepted ADRs, plus a deferred list
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic)

**Deferred behind future ADRs** — forwarding network, radix-4 divider, negedge-BRAM regfile, FPGA
timing closure, interrupts, Spike co-sim. Each trades away simplicity the current design depends
on; none are safe to build while the core is unverified.
