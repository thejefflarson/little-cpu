# ADR-0008: Test memory map and the `tohost` protocol

**Status:** Accepted · 2026-07-27

## Context

ADR-0007 specifies a cxxrtl runner that "runs until a write of the magic pass/fail value to the
`tohost` address" — without defining the address, the encoding, or how the iverilog bench observes
the same event.

Worse, the link script makes the tests unpassable as written. `test/asm/sections.lds` has a `*(*)`
catch-all that places `.text` **and** `.data` into a single output section at address 0. This core
is Harvard: `imem_addr` indexes ROM based at 0, `mem_addr` indexes RAM based at 0, and the two are
disjoint. Every `.S` file's `TEST_DATA` block therefore links into instruction memory where loads
cannot reach it — `lw.S`, `sw.S`, `lb.S` and the rest cannot pass regardless of how correct the
core is.

There is also no `tohost` symbol anywhere in the repo, and `formal/dmemcheck.sv` will eventually
need to agree with whatever convention is chosen.

## Decision

**Two regions, two images, riscv-tests' `tohost` encoding.**

1. **Split `sections.lds` into two regions.** `.text` goes to `>rom` (base 0). `.data`, `.rodata`,
   and `.bss` go to `>ram` at a non-zero base decoded by both testbenches. Export `tohost` as a
   symbol in the RAM region.
2. **The runner takes two images:** `--rom <hex>` and `--ram <hex>`. Both are produced by
   `objcopy -O verilog --verilog-data-width=4` from their respective sections.
3. **`tohost` is a fixed word address in RAM.** The runner watches writes to it via `debug_items`.
4. **Encoding follows riscv-tests**, which `test/asm/test_macros.h` already assumes:
   pass = `1`; fail = `(testnum << 1) | 1`. The runner reports the failing test number.
5. **Exit codes:** 0 = pass, 1 = fail (with the test number), 2 = cycle-limit timeout.
6. **The iverilog bench observes the same address** with the same encoding, so both sim legs score
   tests identically.

## Rationale

Two regions is the only option that makes load/store tests passable on a Harvard core without
inventing a unified address space the RTL does not have. Two images follows directly, and keeps the
runner honest about which memory it is filling.

Reusing riscv-tests' `tohost` encoding costs nothing — `test_macros.h` is already written against
it — and means the `.S` files need no edits beyond the missing `riscv_test.h`.

## Consequences

- `test/asm/sections.lds` is rewritten. `test/asm/riscv_test.h` is written against this map.
- The cxxrtl runner CLI grows `--ram`. The brief's single-`--rom` sketch was insufficient.
- `test/testbench.v` and `rtl/littlesoc.v` must decode the RAM base consistently.
- `formal/dmemcheck.sv` inherits this map when it is ported in M2.
- A per-test cycle budget is needed. Until the divider iteration-count fix lands (bug 6), `div.S`
  and `rem.S` run ~2× longer than they should, so the budget must be generous enough that a
  timeout means a hang, not a slow divide.
- Verify rather than assume: `MASK_XLEN` in `test/asm/test_macros.h:11` keys off `__riscv_xlen`, so
  the 64-bit literals in the RV64-sourced files should truncate correctly at `rv32im_zicsr`.
