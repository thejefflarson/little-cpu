# 0084 — Dhrystone is the number this core can be quoted by, and the suite's RAW share does not transfer

Status: Accepted

## Context

Every performance figure this project had described the project. CPI 2.08 is measured on
sixty-two hand-written assembly programs, and `test/stall_report.py` says in its own output that
the number describes the suite rather than the core. `make fit` and `make soc-timing` are areas and
nanoseconds. None of it answers "is this core fast?" in a form anyone outside the repository can
check, because the field quotes DMIPS/MHz and this repository had never run Dhrystone.

That mattered more once the forwarding network was priced and declined (ADR-0083). Part of that
decision rests on the RAW-stall share — 29.3% of suite cycles charged to the decode scoreboard —
as the ceiling on what forwarding could buy. That share is measured on programs written to hammer
one instruction at a time, with dense back-to-back dependencies and almost no loop structure. Until
something compiled ran, nobody knew whether it was a property of the core or of the programs.

The blocker was `.data`. Dhrystone has initialised globals and a 10 KB array, and until ADR-0081's
`test/crt0.S` copy stub there was no image shape that could carry them.

## Decision

**Dhrystone 2.1 is ported as it is published, and lives in `test/bench/`, outside the graded
suite.** `make dhrystone` builds it, checks its ROM image against the SoC's 8 KB, runs it under the
cxxrtl leg with the cycle count taken from `mcycle`, and prints the program's own report followed
by the same per-reason cycle accounting `make cycles` prints.

Four things about that shape are the decision, not the implementation:

**It is not a gate and adds no ratchet.** There is no CPI ratchet in this repository, deliberately,
and a DMIPS/MHz figure is a measurement to compare against the last one and against other cores —
not something to fail a merge on. It is also why it is not in `test/asm`: that directory *is* the
suite, both sim legs glob it, and a program needing two million cycles would time out against the
runner's 5000. Nothing in `make test` reaches it.

**The flags are compiled into the number.** `DHRY_CFLAGS` in the Makefile is both what the sources
are built with and, through `-DDHRY_FLAGS`, what the program prints beside its own result;
`test/bench/dhry_port.c` refuses to compile without it. Dhrystone is string-dominated, tiny and
notoriously sensitive to the optimiser — a good `-O` level deletes part of the work — so a
DMIPS/MHz figure without its flags, its compiler and its string library is not a measurement. That
is the rule this repository already applies to `make fit` and `make soc-timing` being
toolchain-dependent, and here it is enforced by the build rather than asked for in a comment.

**The 8 KB budget is enforced by the linker, and overflowing it is the answer.**
`test/bench/bench.lds` gives the `rom` region 8 KB — `rtl/imemory.v`'s 2048 words — so `ld` reports
an overflow in bytes and no image is produced. Trimming Dhrystone to fit would produce a number
nothing could be compared against, which is the one thing this benchmark exists to avoid. The
runner reads the budget back out of that file rather than carrying its own copy.

**The benchmark checks its own results.** The published version prints every global for a human to
read against a table; nothing reads this one's output, so `dhry_1.c` makes the comparison itself and
reports through `tohost`. Without it most of Dhrystone's results are unused and an optimiser would
be within its rights to delete the loop that produced them. `dhry_1.c` and `dhry_2.c` stay separate
translation units with no LTO for the same reason, which is also how every published number is
built.

**The simulated data RAM becomes the SoC's.** `test/testbench.v`'s `RAM_WORDS` goes from 1024 to
16384 — 64 KB, exactly `rtl/littlesoc.v`'s two `SB_SPRAM256KA`, ending one word below the timer at
`0x0002_0000`. Dhrystone's `Arr_2_Glob` alone is 10 KB. The harness was smaller than the hardware it
stands for; now a program that fits the simulator fits the part.

## The measurement

Measured at this branch's head, `riscv64-elf-gcc 16.1.0`, 2000 runs, under the cxxrtl leg with the
RVFI monitor live and the benchmark's self-check passing:

```
-march=rv32imc_zicsr_zifencei -mabi=ilp32 -O2 -std=c11 -ffreestanding
-fno-tree-loop-distribute-patterns -Wall -Wextra -Werror
```

| | |
|---|---|
| cycles (between the two `mcycle` reads) | 2,148,031 |
| instructions retired in that window | 922,026 |
| CPI | 2.32 |
| cycles per Dhrystone | 1074 |
| instructions per Dhrystone | 461 |
| Dhrystones/s/MHz | 931.0 |
| **DMIPS/MHz** | **0.529** |
| ROM image (`.text` + `.data`'s load copy) | 3572 bytes of 8192; 4620 free |

Where that sits, against figures the other projects publish rather than anything measured here:

| core | configuration | DMIPS/MHz |
|---|---|---|
| VexRiscv small | RV32I, 5-stage, no bypass | 0.52 |
| **this core** | **RV32IMC, no bypass, no cache** | **0.529** |
| FemtoRV | | ~0.57 |
| VexRiscv small and productive | RV32I, 5-stage, with bypass | 0.82 |
| VexRiscv full no cache | RV32IM | 1.21 |

The comparison is only as good as its caveats, and they are printed by the program: the string
routines here are this port's own byte loops rather than a libc's, on a benchmark where string work
dominates, and the other projects do not publish theirs.

## The stall split, and what it says about ADR-0083

The same accounting, over the whole run in each case:

| | issue | divider | accessor | hazard | serialize | operand | fetch | CPI |
|---|---|---|---|---|---|---|---|---|
| `test/asm` suite | 48.6% | 2.8% | 1.3% | **29.3%** | 1.8% | 16.2% | 0.0% | 2.08 |
| Dhrystone | 43.1% | 0.0% | **10.3%** | **16.0%** | 0.0% | **30.5%** | 0.0% | 2.32 |

Four reasons move by more than a few points, and one of them is the one ADR-0083 rests on.

**The RAW scoreboard nearly halves, 29.3% → 16.0%.** The 29.3% figure does not survive contact with
compiled code, so the ceiling on what a forwarding network could buy is roughly half what the suite
suggested. ADR-0083 declined forwarding on a 0.48% timing margin against a 7.5% cycle win; this
makes that decision easier, not harder.

**The operand-fetch cycle nearly doubles, 16.2% → 30.5%, and becomes the largest single reason.**

**The load turnaround goes up eightfold, 1.3% → 10.3%**, which is what real code doing real loads
looks like next to programs that mostly compute in registers.

**The divider and serialization all but vanish**, 2.8% → 0.0% and 1.8% → 0.0%. Dhrystone divides
once per iteration and never touches a CSR; the suite has whole programs for each.

**The first two of those are not independent, and the honest statement is weaker than either
number.** `stall` is charged to the first reason the decoder itself would try, and `hazard` is tried
before `operand`, so a cycle that is both goes to `hazard`. `operand_stall` is high only on the
first cycle a new register pair is presented, so at most one cycle per instruction can be
reallocated — but the sum of the two columns is 45.5% on the suite and 46.5% on Dhrystone, which is
flat. So part of the 13.3-point fall in the RAW column is the operand-fetch cycle being charged for
cycles the suite charged to the scoreboard, and this measurement cannot say how much. What it can
say is that **the total decode-side bubble is unchanged** and that **29.3% was never a clean RAW
number** — it is an upper bound that already contained operand-fetch cycles a forwarding network
would not have removed. Separating them needs a counter that counts cycles a reason was *high*
rather than *charged*, which would break the property that every cycle is charged exactly once, and
is left as a separate question.

**CPI is worse on compiled code, not better: 2.08 → 2.32.** The suite was flattering this core,
which is the opposite of what its own report's caveat implies.

## Consequences

- The project can place itself on the table the field uses, with one command, and the figure carries
  its own flags wherever it is quoted.
- `make dhrystone` takes about 25 seconds and needs the cross compiler and `./sim`. It stays off CI.
- ADR-0083's decline is better supported than it was, and its 29.3% is now qualified in two
  directions at once: it does not transfer to compiled code, and it was never separable from the
  operand-fetch cycle on the suite either.
- ADR-0074 declined removing the operand-fetch cycle on a measured 13% of suite CPI. On compiled
  code that cycle is charged 30.5% of all cycles. That is not a reason to reopen it — the decline
  was on timing margin, not on the size of the prize — but the prize is bigger than the suite said,
  and a future re-measurement should use this workload.
- The SoC's 8 KB ROM holds a whole Dhrystone with 4620 bytes to spare, which is a real data point
  for the deferred SPI-flash boot: the part is not out of room yet.
- `test/stall_report.py` now takes `--workload` for the paragraph under its table. The suite's
  wording is still the default; printing "these are small hand-written assembly programs" under a
  table of compiled C would have been simply false.
