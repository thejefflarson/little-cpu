# Little CPU

A hobby RISC-V core in SystemVerilog on the open toolchain (Yosys / iverilog / SymbiYosys —
no vendor EDA). Target **RV32IMC_Zicsr**, machine mode only; eventual home is an ice40 up5k.

**This project optimizes for readability, not throughput.** That is the stated point of it. CPI is
deliberately sacrificed for a design that reads well. Do not "improve" it by adding machinery.

## Current state — read this before believing anything else

**This is a half-finished rewrite, not a working core.** A serialized FSM core
(`rtl/riscv.v` + `rtl/alu.v`) once passed riscv-formal sans CSRs; it was torn down in two waves
(2021: `9758a39`→`1709433`; 2023: `49b317a`→`4fbd650`→`13fec44`) into today's staged design, and
the rewrite stalled partway. The project went from *formally verified* to *unverified*, and getting
back is the whole plan.

The README is the project's voice and is deliberately left as-is. **Ground truth lives here.**

What does not work right now:

- **The core computes wrong results.** `rtl/regfile.v` reads registers synchronously with no
  compensating structure, so operands are one instruction stale — always. Confirmed still true
  while landing JEF-606: `rtl/decoder.v` consumes `reg_rs1`/`reg_rs2` the same cycle it asserts a
  *new* `rs1`/`rs2` request, one cycle before that request's answer is back, so every non-`x0`
  operand — including the pass/fail write to `tohost` — is actually the regfile's answer to the
  *previous* instruction's read. This is why all 46 `.S` tests currently time out, `simple.S`
  included; see `test/EXPECTED_FAIL` and the JEF-606 PR for the trace.
- **`make test` now exists** (JEF-606): `test/asm/riscv_test.h`, a two-region `sections.lds`
  (ADR-0008), and a cxxrtl runner (`test/cxxrtl.cc`, ADR-0007) assemble and run every
  `test/asm/*.S` and check the result against `test/EXPECTED_FAIL`, the sprint-1 baseline. Right
  now that file lists all 46 tests — burn it down as the regfile bug above and others get fixed,
  not in one shot.
- **The iverilog leg does not elaborate at all** — ~60 declaration-after-use bind errors in
  `rtl/decoder.v` and `rtl/littlecpu.v`. Yosys tolerates these; iverilog does not.
- **`test/monitor.v` does not elaborate under yosys** — `$time` inside `$display` at line 90 hits
  `ERROR: Don't know how to detect sign and width for AST_AUTOWIRE node`. The build needs a
  sanitized derived copy; the tracked file stays pristine.
- **RVFI ports are declared and never driven**, so no riscv-formal check can pass.
- **`formal/checks.cfg` and `formal/wrapper.v` still reference the deleted core.**
- `make riscv.json` references `rtl/handshake.v` / `rtl/skidbuffer.v`, deleted in `49b317a`.

What does work: `yosys ... write_cxxrtl` elaborates the current RTL cleanly with zero warnings, and
the cxxrtl binary builds and runs. That is the foundation everything else is built on.

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

`test/monitor.v` rides along in both sim legs, so every run is self-checking per-retire — a test
that corrupts state transiently but converges to the right final registers still fails loudly.

## Engineering rules in force

- **Compiler and elaboration warnings are errors.** Fix them; don't silence them.
- **Every non-trivial change adds or updates tests and runs the full suite** before being declared
  done. Elaboration succeeding is not a substitute for tests passing.
- **Never commit build artifacts.** `test/rtl.cc`, `sim`, `*.vvp`, `*.vcd`, `rvfi_macros.vh`, and
  `formal/` output dirs are all generated. (`test/monitor.v` is the one deliberate exception.)
- **riscv-formal is SHA-pinned.** Bumping the pin requires regenerating `test/monitor.v` and
  rerunning the ladder.
- Prefer verified/first-party GitHub Actions. Simplest approach unless asked otherwise.

## Milestone ladder

| | Milestone | Green means |
|---|---|---|
| M0 | Foundation | this file, riscv-formal SHA-pinned, dead references gone |
| M1 | Finish the pipeline | all RV32IM `.S` tests pass under cxxrtl |
| M2 | **Parity checkpoint** | the pipelined core re-proves everything the serialized core proved |
| M3 | Past the old core | CSRs + machine-mode traps |
| M4 | Full ladder + CI | nightly formal green; tag a release |

M1 is the critical path. **M2 is the milestone that erases the verified→unverified regression** —
treat it as the real finish line, not M1.

## Pointers

- Design brief: [`docs/ideas/finish-the-rewrite.md`](docs/ideas/finish-the-rewrite.md)
- Decisions: [`docs/adr/`](docs/adr/) — fourteen accepted ADRs, plus a deferred list
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic)
- Tracker: Linear, project **Little CPU** (team JEF)

**Deferred behind future ADRs** — forwarding network, radix-4 divider, negedge-BRAM regfile, FPGA
timing closure, interrupts, Spike co-sim. Each trades away simplicity the current design depends
on; none are safe to build while the core is unverified.
