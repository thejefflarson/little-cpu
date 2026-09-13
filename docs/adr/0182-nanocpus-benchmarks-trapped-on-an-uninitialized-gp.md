# ADR-0182: nanocpu's benchmarks trapped on an uninitialized `gp`, not a fetch bug

**Status:** Accepted · 2026-09-12

## Context

ADR-0180 shipped nano's simulation harness with `make nano-dhrystone` trapping at cycle
9,629, partway through zeroing Dhrystone's `.bss`, and `make nano-coremark` trapping later
but before either reached a verdict. Neither trap was root-caused: the fault-report at the
time was a bare "trap taken at cycle N", with no PC, no instruction word, and no cause.

## The runner now reports what trapped

`nano/tb/nano_cxxrtl.cc` reads `uut pc` and `uut instr` at the moment `trap_latched` fires
-- both stay parked once `cpu_state` reaches `cpu_trap`, since nothing in that state further
updates them -- and classifies the cause from the same combinational decode signals
`nano.v`'s own `cpu_trap` arms read (`is_valid`, `is_e_illegal`, `is_ecall`, `is_ebreak`,
the misaligned-load/store predicates, and `pc_wdata`'s low bit for a misaligned jump or
branch target). nano has no `mcause`, so this is the runner's own classification, not a
value nano exposes; a signal missing from a future build degrades the message to
"unclassified" rather than aborting the run.

## Root cause: `nano/bench/start.S` never sets `gp`

Every other startup file in this repository -- `test/crt0.S`, `soc/compare/dhry_start.S`,
`soc/compare/coremark_start.S` -- opens with:

```
.option push
.option norelax
la      gp, __global_pointer$
.option pop
```

`nano/bench/start.S` did not. The RISC-V linker's default relaxation turns a `la` of a
symbol within +-2KB of `__global_pointer$` into a single gp-relative `addi`, and Dhrystone's
own `la a1, __bss_start` qualifies: the shipped binary's first instruction touching `a1` is
`addi a1, gp, -1804`, not the two-instruction absolute sequence its disassembly comment
implies. With `gp` at this simulator's register-file reset value of zero, `a1` starts at
`-1804` (`0xfffff8f4`) instead of `__bss_start` (`0x100fc`). The `.bss`-zeroing loop then
increments `a1` by 4 every iteration, storing zero at each address, until it wraps through
`0xffffffff` back to `0x00000000` and keeps going -- straight through nano's own ROM. A
direct store trace confirms it:

```
cycle=9505 WRITE addr=0x00000000 ... pc=0x00000018 instr=0x0005a023   (sw zero, 0(a1))
cycle=9526 WRITE addr=0x00000004 ...
cycle=9547 WRITE addr=0x00000008 ...
cycle=9568 WRITE addr=0x0000000c ...
cycle=9589 WRITE addr=0x00000010 ...
cycle=9610 WRITE addr=0x00000014 ...
trap at cycle=9629 pc=0x00000014 instr=0x00000000
```

Word 5 (byte address `0x14`) is the loop's own `beq a1, a2, 2f`; once it reads back as the
zero this same loop just wrote there, decode calls it illegal and traps. This is a startup-code
defect in the harness's own `.S`, not a `nano.v` defect, so fixing it needed no coordination
with the divider work in flight on that file.

## The fix and its regression

`nano/bench/start.S` gained the same `.option push`/`norelax`/`la gp, __global_pointer$`/
`.option pop` sequence the other three startups already carry.

`nano/bench/startup_test.c` is the regression: a two-instruction `main` that reads `gp` back
via inline asm and reports over `tohost` whether it equals `&__global_pointer$`, built
against the real `nano/bench/start.S` with its own tiny linker script
(`nano/bench/startup_test.lds`) so the check needs neither a benchmark's cycle count nor its
divide instructions. `nano/bench/run_startup_test.sh` builds and runs it in about 2,000
cycles and is wired in as `nano-startup-test`, a `make test` prerequisite; `test/probe_gates.sh`
forces both directions (a passing sim verdict and a failing one) so the check itself is a
grader that can fail.

## What the fix uncovers: the divider, earlier work already knows about

With `gp` fixed, Dhrystone advances from 1,834 retires (cycle 9,629) to 105,367 retires
(cycle 563,926) before an RVFI monitor mismatch: `div a4, a2, a5` on `rs1=9, rs2=7` computes
`0`, where the spec model says `1`. CoreMark hits the same shape far earlier -- 128 retires,
cycle 744 -- on a `divu` computing `0` where the spec says `0x29a` (`2000 / 3 = 666`). Both
are the same zero-quotient failure ADR-0180 recorded for `divide.S`'s `MONITOR-ERROR 105`
(`div 20, 6` computing `0` instead of `3`): nano's real divider is the subject of a defect
already filed and in progress on `nano.v` itself. Neither benchmark can print a DMIPS/MHz or
CoreMark/MHz figure until that lands; this ticket does not touch `nano.v` to avoid
colliding with that work, and does not print a number it cannot back.

## Consequence

`make nano-dhrystone` and `make nano-coremark` now get past their own startup and well into
each program before stopping on the divider -- a real distance, not the same trap moved
later by chance, since the two benchmarks reach it by entirely different code paths.
`nano-startup-test` adds well under a second to `make test` (`nano-sim` is already built for
`nano-test`). Once the divider is fixed, re-running both benchmarks is the next step; if a
third blocker turns up, it gets the same treatment this one did.
