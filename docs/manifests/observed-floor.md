# `test/OBSERVED_FLOOR`

The suite's MANIFEST — `test/asm/*.S` and `test/asm/*.c` — and how much the
per-retire RVFI monitor was OBSERVED to do on each program.

TWO JOBS, ONE LIST, DELIBERATELY. This file is the list of programs the suite
must contain, and it is the list of what each of them was measured to retire.
It carries both because it cannot carry the second without carrying the
first, and a second file naming the same programs would be a second surface
to keep in sync — at which point the interesting failure becomes "the two
lists disagree" rather than "the suite shrank". Both sim legs read this one
file: `test/run_tests.sh` and `test/run_cosim.sh` each call
`test/check_suite_shape.sh` against it BEFORE they run a single program, so
they cannot come to disagree about what the suite is either.

THE MANIFEST HALF ANSWERS A QUESTION NEITHER BASELINE CAN. `test/EXPECTED_FAIL`
and `test/COSIM_EXPECTED_FAIL` record which programs are known to fail;
between them they name five of seventy-four, so almost nothing in them would
disappear if the suite itself did. A bad rebase, a directory rename, a
`.gitignore` change or a glob that stopped matching would leave `make test`
reporting `12/12 passed`, matching an empty baseline exactly, and exiting 0.
The formal side has the same shape: a verdict baseline cannot report whether a
check stopped existing, which is why `formal/EXPECTED_CHECKS` exists. The name
set below is the same assertion for simulation. It is checked in BOTH
directions: a program in `test/asm` with no line here is red, so a `.S` that
lands without being wired in fails immediately rather than joining the suite
unmeasured.

## Why this file exists

`make test` is the merge gate and `test/monitor.sim.v` is its per-retire
oracle, but nothing in the gate measured whether the oracle ever fired.
`test/cxxrtl.cc` sampled the monitor's errcode and counted nothing, so a
monitor whose `rvfi_valid` never asserted — an under-sensitivity defect of
exactly the kind already found in the iverilog leg, an `ifdef` that dropped
the shadow payload, a `write_cxxrtl` that optimised the instance away — left
every program here reporting PASS off `tohost` alone. Checking the end state
alone was already measured to be blind to real architectural corruption, and
`test/EXPECTED_FAIL` is empty, so there was no red entry whose disappearance
would have said otherwise either. This file turns observation into a graded
quantity instead of an inference.

## Format

Three fields:

```
<program>  <retires>  <spec-checked retires>
```

where `<program>` is a `test/asm` file name, `.S` or `.c`. The NAME half
treats the two identically. The NUMBERS do not, and the section below on C
programs is the part to read before writing a `.c` line.

`retires` is the number of cycles the monitor examined (`rvfi_valid` high).
`spec-checked` is the subset of those whose VALUES it compared against its
spec model (`spec_valid` high as well). Both are counted in
`test/testbench.v`, printed by `test/cxxrtl.cc` as
`RETIRES <n> SPEC-CHECKED <m>`, and graded by `test/run_tests.sh`, which also
prints them as the table's third column.

GRADED WITH `>=`, NOT SET EQUALITY — that is the difference between this file
and `test/EXPECTED_FAIL` or `formal/EXPECTED_CHECKS`, and it is deliberate.
The numbers move for legitimate reasons: the assembler is free to compress
differently, a test gains an instruction, a macro in
`test/asm/riscv_test.h` grows one. An exact ratchet over every number in this
file is one nobody would keep, and a ratchet nobody keeps gets raised until it
means nothing. What must not happen is a program going QUIET, and `>=`
catches exactly that.

## A C program's two numbers are a silence bound, not an observation

That is the one place this file's format means different things on different
lines. An `.S` program assembles to a fixed instruction sequence, so
recording what it retired costs nothing and holds across toolchains by
construction. A `.c` program's count is whatever that gcc chose to inline and
schedule: `test/asm/datainit.c` retired 400 under a local riscv64-elf-gcc and
395 on the CI runner, from identical source. Recorded as a floor, that
difference is red — and red for the wrong reason, because the floor exists to
catch a program going QUIET and cannot tell "the monitor stopped observing"
from "a different gcc emitted five fewer instructions". This repo already
knows its two toolchains disagree; `make fit` reads tens of cells apart on
identical RTL.

So pick a `.c` line's numbers from what the CHECK needs, never from the
table. Every recorded instance of the defect this file exists for produced
ZERO retires — `rvfi_valid` never asserting, an `ifdef` dropping the shadow
payload, `write_cxxrtl` optimising the monitor instance away — or ONE, the
iverilog under-sensitivity defect, measured at "0 writes and 1 retire". Zero
is already the runner's own exit 6, so the bound only has to clear one.
**16** is the number these lines use: an order of magnitude above the defect,
two below any C program's real count, and nothing a compiler chooses can move
it. `spec-checked` has the same problem and takes the same bound — it tracked
retires exactly on `datainit.c` (395 of 395 on CI), so whatever moves one
moves the other.

`test/run_tests.sh` REJECTS a `.c` floor above 64 rather than trusting this
paragraph, because a paragraph is exactly what the next person will not read
and "copy the third column" is the instruction that produced the red gate.

The NAME set, on the other hand, IS a set equality in both directions, the
same contract the two baselines carry — that is the manifest half, described
above. It is checked by `test/check_suite_shape.sh` BEFORE either leg
assembles anything, so a mismatch in either direction names the programs
involved and stops, rather than being discovered after a full run that was
never going to be gradeable. That is true of `.c` names exactly as it is of
`.S` ones — the numbers are what differ, never the manifest. Adding a program
to `test/asm` means adding its line here, in the same commit.

ZERO IS NEVER A FLOOR. Zero of either count is `MONITOR-SILENT` — the
runner's own exit 6 — before this file is ever consulted, so a line of `0 0`
cannot be used to excuse a blind oracle. There is no way to write "this
program is allowed not to be checked" here, on purpose.

## A low spec-checked number is expected and is not a defect

riscv-formal ships NO SPEC MODEL AT ALL for `ecall`, `ebreak`, `mret`, the
`csrr*` family or ANY OF THE ELEVEN A ENCODINGS at the pinned SHA
(`formal/pin.mk`), so `spec_valid` is 0 for every one of those retires by
design and the monitor's whole semantic block is skipped for them. The
behaviour M3 added is checked by `test/asm/trap.S`, `test/csr_tb.v` and
`test/decoder_tb.v` instead, against assertions this repo wrote rather than
against an oracle. `csr.S`, `minstret.S` and `trap.S` therefore show the
widest gap between the two columns, and that gap is the pin's coverage
boundary, not a bug in this core. Whoever first sees a low number here should
not have to rediscover that.

THE SIX A PROGRAMS ARE THE SHARPEST CASE OF THAT, AND A LINE HERE SAYS LESS
ABOUT THEM THAN ABOUT ANY OTHER PROGRAM. `test/monitor.sim.v` wraps every
value comparison in `if (ch0_spec_valid)` with no `else`, so on an `lr.w`, an
`sc.w` or any of the nine AMOs the per-retire oracle grades pc continuity and
nothing else — not the register written, not the memory access. A number in
the `spec-checked` column below therefore counts the ORDINARY instructions
surrounding the atomics, and the difference between the two columns is
roughly the count of atomics executed. So the floor on `amo.S`,
`amominmax.S`, `amoregion.S`, `amotrap.S`, `lrsc.S` and `lrsclock.S` is a
bound on those programs still RUNNING, and every claim they make about the A
extension is an in-band assertion in the program itself. Reading a green line
here as "the atomics were checked" is exactly the inference this file exists
to prevent.

## How to re-measure

For `.S` lines only. Run `make test` and copy the table's third column.
Record the commit SHA it was measured at in the PR that changes a number, the
way every other measurement in this repo is recorded. Numbers only ever go
DOWN by accident; if one legitimately drops (a different binutils compressing
more aggressively, a test deliberately shortened), lower it in the same
commit as the change that caused it and say which. A `.c` line has nothing to
re-measure: it is 16 because 16 is what the silence check needs, and it stays
16 when the compiler changes its mind.

Measured at 18d17a2 (the branch point) with riscv64-elf-gcc, the pinned
riscv-formal monitor, and `CYCLES=5000` in `test/run_tests.sh`.

Three lines carry an inline note because their `spec-checked` count is not
just "the pin has no model for this": `datainit.c`'s is a silence bound
rather than an observation (see above); `lrsclock.S` and `uart.S` each run a
retry/poll loop under conditions (a live timer, a fixed frame width) where how
many times it goes round is itself a CPI-sensitive quantity, so a change to
the region wait or similar moves the count for a reason that has nothing to
do with the monitor going quiet.
