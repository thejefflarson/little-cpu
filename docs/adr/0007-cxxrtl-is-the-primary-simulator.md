# ADR-0007: cxxrtl is the primary simulator; iverilog is the microscope

**Status:** Accepted · 2026-07-27

## Context

The repo carries two simulation stacks. `test/cxxrtl.cc` (27 lines) drives a yosys `write_cxxrtl`
build; `test/testbench.v` (155 lines) drives an iverilog build with the generated riscv-formal
monitor (`test/monitor.v`, 7008 lines) instantiated at lines 102–126.

Neither is wired to a test suite. The README advertises `make test`, but **the root Makefile has no
`test` target**. `test/asm/*.S` are riscv-tests-style but nothing assembles or links them, and every
one of them `#include`s `riscv_test.h`, which is not in the repo.

An initial recommendation was to delete the cxxrtl path and standardise on iverilog, on the grounds
that two sim stacks is double maintenance.

## Decision

**Keep both, with distinct roles. cxxrtl is the primary runner.**

### cxxrtl — the workhorse

`test/cxxrtl.cc` grows to a ~150-line test runner:

- Parse `--rom <hex> --cycles N [--vcd out.vcd]`.
- Load the program hex directly into ROM via `debug_items` (more robust than `$readmemh` through
  yosys).
- Run until a write of the magic pass/fail value to the `tohost` address.
- Return a real process exit code.

`make test` = assemble every `.S` → `objcopy -O verilog --verilog-data-width=4` → run each under
the cxxrtl binary.

### iverilog — the microscope

`test/testbench.v` stays the debugging bench: VCD waveforms, `$display` tracing, same monitor
instance. CI runs one smoke program through it to keep the second-frontend elaboration path alive.
Not the bulk runner.

### riscv-formal — the oracle

Exhaustive per-instruction semantics and pipeline corners to bounded depth, with ALTOPS masking the
real mul/div arithmetic — which is precisely the hole the `.S` suite covers.

### `test/monitor.v` stays tracked

It is deterministic output of `riscv-formal/monitor/generate.py -i rv32imc -c 1 -a -p monitor`, and
that ISA string is already correct for ADR-0002. It stays in-tree because it is directly useful for
debugging. Keeping it honest: the generation rule stays in the Makefile with a header comment
naming the riscv-formal pin; regeneration is required when the pin or ISA string changes; **CI
verifies freshness by regenerating and diffing**; conflicts are resolved by regenerating, never by
hand-editing.

## Rationale

**cxxrtl runs one to two orders of magnitude faster than iverilog** for a core this size, and that
matters exactly where this design hurts. Even at the corrected 32 divider iterations (ADR-0004),
`div.S` and `rem.S` run to thousands of cycles; a hazard/divider torture test or an overnight
randomised run is only practical under cxxrtl. It also keeps the yosys frontend honest, since it
uses the same elaboration path CI depends on.

iverilog earns its place as a waveform microscope and a second elaboration frontend — the two jobs
cxxrtl is worst at. This is not double maintenance; it is one runner and one debugger.

Keeping `test/monitor.v` in both builds means **every run is self-checking per-retire**, not merely
end-state-checking. That catches what `.S` pass/fail cannot: a test that corrupts state transiently
but converges to the right final register values still fails loudly, and torture or randomised runs
become self-checking without expected-output files.

## Consequences

- `test/rtl.cc` (2533 generated lines) stays **generated at build**. It is already gitignored; the
  stale on-disk copy should be removed from disk, not from git.
- `make test` depends on a RISC-V toolchain. Because the tests are freestanding assembly, no
  multilib or newlib is needed: `-march=rv32imc_zicsr -mabi=ilp32 -nostdlib -T sections.lds`.
  macOS: `brew install riscv64-elf-gcc`. CI: `apt install gcc-riscv64-unknown-elf iverilog` plus a
  pinned YosysHQ OSS CAD Suite release for yosys/sby/solvers.
- A minimal local `riscv_test.h` must be written (RVTEST_CODE_BEGIN / PASS / FAIL writing the
  `tohost` magic). M1 work.
- The 13 RV64-only `.S` files are deleted (ADR-0002); `rv32uc/rvc.S` and a straddle test are added;
  trap/CSR tests arrive in M3.
- **Residual risk:** if the monitor's `$display` error reporting misbehaves under cxxrtl, the
  fallback is a C++-side check of the monitor's error flag. Small and contained.
