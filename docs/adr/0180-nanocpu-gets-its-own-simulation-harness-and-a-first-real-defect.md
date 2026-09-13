# ADR-0180: nanocpu gets its own simulation harness, and its first real defect

**Status:** Accepted · 2026-09-12

## Context

`nano/nano.v` had never executed a program. Everything that had touched it was formal --
`nano/formal/checks.cfg`'s generated riscv-formal checks, `ill_e`, `imemcheck`, `dmemcheck`
-- and formal proves properties, it does not run Dhrystone. `nano/nano.mk` has an area
ratchet and nothing else. This ticket builds the missing piece: a cxxrtl harness for the
`.S` suite, and Dhrystone/CoreMark targets, over nano's own picorv32-style bus.

## What changed since the brief

nano reshaped to RV32E between the brief and this work (ADR-0177, ADR-0178): 16 registers,
x0-x15, M and the counters kept. The harness below builds for `rv32emc`/`ilp32e` from the
start, not RV32I, and every hand-written `.S` program in it uses x0-x15 only.

## The memory model

`nano/tb/nano_memory.v`: one flat word array on nano's `mem_valid`/`mem_ready`/`mem_addr`/
`mem_wdata`/`mem_wstrb`/`mem_rdata` bus, zero-wait by default, with a `WAIT_STATES`
parameter for a later QSPI-latency approximation. It is deliberately separable: a later
ticket that wires the real QSPI front end replaces this module, not `nano.v`.

**A fetch address is not word-aligned the way a data address is.** `nano.v` masks a load or
store's address to its containing word before issuing it, but a compressed instruction's
own fetch address can land on either half of a word (`next_pc` advances by 2 for a 16-bit
instruction), and nano asks for exactly that address with no masking. The first version of
this memory model read `mem[mem_addr[31:2]]` unconditionally -- word-aligned only -- and
the very first `.S` program run against it (`alu.S`) hit a real RVFI monitor mismatch at
its ninth instruction: a `bne` that should not have taken did, because the "instruction"
nano decoded was two unrelated bytes straddled across a word boundary, read from the wrong
half. The fix reads the low and high neighbouring words and selects between them on
`mem_addr[1]`, matching what a byte-addressable QSPI stream would answer. This is a fact
about the bus contract, not about `nano.v`, and belongs in the memory model rather than the
core.

## nano.v is not iverilog-elaborable today

Two independent problems, neither new: the donor brief (`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md`)
already recorded "under iverilog 13 it does not build -- three declaration-after-use sites
and one ivl segfault" for `c55efd6`, and it is still true of the reshaped core. `regs[]` and
most of the `is_*` decode flags are referenced in continuous assigns before their own
declarations, which iverilog's parser needs in scope first for an indexed array (a
scalar's forward reference merely warns); separately, `` `RVFI_OUTPUTS `` declares the RVFI
ports as nets, and `nano.v`'s `always_ff` block drives them, which iverilog's stricter
net/variable split rejects outright. Both are fixable, but fixing them means reordering a
large fraction of `nano.v`'s declarations and changing how its RVFI block is typed -- a
change to the core with formal-proof and area-ratchet stakes, not a harness change. **This
harness ships cxxrtl-only.** Filed as a follow-up: give `nano.v` an iverilog leg the way
`rtl/littlecpu.v` has one, as its own reviewed change.

## The suite

`test/asm`'s programs are not portable to nano as written, for a reason a plain grep of
the raw source does not show: `test_macros.h`'s shared `TEST_CASE`/`TEST_AMO` macros use
`x29` as scratch in nearly every test (73 of 75 files, found by preprocessing each file and
grepping the expansion), so almost the whole littlecpu suite traps as E-illegal on nano
regardless of which specific instruction each program means to exercise. Porting it means a
low-register macro library, which is out of this ticket's scope to build well. Six small,
hand-written, nano-local programs stand in: `nano/asm/alu.S`, `branch.S`, `loadstore.S`,
`mul.S`, `divide.S`, `compressed.S` -- every register in every one of them is x0-x15, and
none uses a CSR, an atomic, the timer, or the UART, none of which nano has. This is the
decision the ticket asked for recorded: **nano-local copies, not a port**, because the
littlecpu suite's macros are what needs rewriting, not any one program.

`nano/asm/run_nano_tests.sh` grades them the way `test/run_tests.sh` grades littlecpu's
suite -- `nano/asm/EXPECTED_FAIL`/`OBSERVED_FLOOR`, set equality both ways, reusing
`test/check_suite_shape.sh` unmodified since it is already generic on an asm directory and
a manifest. `test/march_test.sh` gained one exception entry
(`nano/asm/run_nano_tests.sh rv32emc 1`) rather than a new required site, since nano's ISA
has nothing in common with littlecpu's declared string to share it with.

## A real, first-time defect: nano's divider

`divide.S` disagrees with the reference model on `div 20, 6` (and every other real
division `mul.S` doesn't already cover): nano computes 0, the spec computes 3. This is not
a harness bug -- the same `sw`/`lw` pair, the same register file, and the same monitor
agree on every other program in the suite, and a hand-traced run of nano's own divide state
machine (`mul_div_counter <= 65`, comparing `mul_div_x <= mul_div_y` against a divisor
pre-shifted by 31) reproduces the wrong answer outside the RTL too. Nothing before this
ticket could have caught it: `nano/formal/checks.cfg` sets `` `define RISCV_FORMAL_ALTOPS ``,
so every generated riscv-formal check substitutes a cheap function-of-the-operands stand-in
for the real divider (the same reason `rtl/executor.v`'s real divider needs
`test/exec_tb.v` rather than the generated checks), and nano has no `exec_tb.v` equivalent.
`divide.S` is baselined `MONITOR-ERROR 105` in `test/EXPECTED_FAIL`'s nano twin rather than
deleted or skipped, so the defect stays visible and graded rather than silently absent from
the suite. **Fixing the divider is not this ticket's** -- it is core RTL with formal-proof
and area-ratchet stakes, filed as a follow-up.

## Dhrystone and CoreMark

`nano/bench/` reuses `soc/compare/dhry_monitor.v` unmodified: it watches
`mem_addr`/`mem_wdata`/`mem_wstrb` for two magic addresses and needs no `mcycle`, which is
exactly nano's situation (no CSR file at all) and the same gap the module was built for
(VexRiscv's CSR-free formal build, Hazard3's `CSR_COUNTER=0`). `nano/bench/dhry.lds` and
`coremark.lds` place `ram` at the same origin `soc/compare/dhry.lds` does, `0x0001_0000`,
so the monitor's hardcoded `CTL_MARK`/`CTL_DONE` addresses need no change. `nano/bench/dhry_port.c`
and `coremark_port.c` are nano-local ports modelled on `soc/compare`'s CSR-free ones (both
already portable C with no CSR read anywhere); `test/bench/dhry_1.c`/`dhry_2.c` and the
vendored `test/bench/coremark/` tree are reused read-only, exactly as `soc/compare/` already
reuses them for its own cross-core harness.

**Neither benchmark reaches a verdict yet.** `make nano-dhrystone` traps partway through
zeroing Dhrystone's `.bss` (before the timed region starts), reading an unrelated
instruction word as all-zero -- not a generic long-loop defect, since a 700-iteration
store loop written directly against this harness passes clean (2811 retires, no error).
The difference between that probe and Dhrystone itself was not root-caused within this
ticket's time: **`make nano-dhrystone` and `make nano-coremark` are real, working targets
-- they build, link, and run under `nano-sim --bench` -- but neither has produced a
DMIPS/MHz or CoreMark/MHz figure yet.** Printing one before this is understood would be
worse than printing none. Filed as a follow-up, with the passing 700-iteration store loop
recorded here as the starting point for whoever picks it up.

## What this does not touch

Nothing here edits `test/run_tests.sh`, `test/cosim.py`, the Makefile's `soc-rom`,
`test/dual_smoke.sh`, or `test/dual_build.sh` -- CLAUDE.md's "five places" rule for the
suite build shape is littlecpu's, and nano's harness reuses `test/check_suite_shape.sh` and
the vendored Dhrystone/CoreMark sources without editing any of them. `nano/nano.v` itself
is untouched: the halfword-fetch fix lives in the memory model, and the divider defect is
recorded, not patched.

## Consequence

`nano-test` joins `make test`'s prerequisite list: six programs, five real passes and one
baselined defect, adding about 18 seconds to a cold `make test` (mostly `nano-sim`'s one-time
cxxrtl elaboration and compile; a warm rebuild is under a second). `make nano-dhrystone` and
`make nano-coremark` stay off `make test` and CI, the same standing littlecpu's own
`make dhrystone`/`make coremark` already have, and neither prints a comparable figure to
littlecpu's -- different design, different memory system, and for now, no verdict at all.
