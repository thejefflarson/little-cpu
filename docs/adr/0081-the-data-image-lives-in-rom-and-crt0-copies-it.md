# ADR-0081: The `.data` image lives in ROM and a `crt0` copies it, and the 8 KB budget is measured

**Status:** Accepted · 2026-08-08 · *Closes the bootloader half that
[ADR-0054](0054-the-memory-system-and-the-first-real-timing-number.md) deferred and
[ADR-0044](0044-what-the-memory-system-has-to-be.md) named. Keeps ADR-0044's SPI-flash boot
deferred, now against a measured number rather than a guess. Widens the suite contract in
[ADR-0035](0035-the-baseline-pins-the-failure-mode.md) and `test/OBSERVED_FLOOR` from `.S`
programs to programs. Amends `CLAUDE.md`'s State section. No `rtl/` or `formal/` change.*

## The gap

The SoC placed, met 12 MHz and could not run a program that reads its own `.data`. Its data RAM is
SPRAM, and no bitstream initialises SPRAM: the only memory that comes up holding anything is the
block RAM behind `.text`. So a global with an initialiser had no value at power-on, and the core
that executes RV32IMC correctly still could not run a C program.

The `.S` suite never saw this. Its programs' `.data` is linked at its virtual address in RAM and
poked straight into the simulated RAM by `test/run_tests.sh` — which is precisely the thing the
hardware cannot do. Every one of the 59 assembly programs passes on a machine that has no way to
get a byte of `.data` into memory.

## Decision

The standard answer, and no RTL.

1. `test/asm/boot.lds` links `.data`'s **load** address into `rom` (`AT>rom`) while its **virtual**
   address stays in `ram`, and exports `__data_load_start`, `__data_start`, `__data_end`,
   `__bss_start`, `__bss_end` and `__stack_top`.
2. `test/crt0.S` sets `gp` and `sp`, copies `[__data_load_start, …)` to `__data_start`, zeroes
   `.bss`, and calls `main`.
3. `test/asm/datainit.c` is the first C program in the suite. It runs under `make test`, under
   `make cosim-suite`, and it is now the default `SOC_PROG`, so the ROM image CI builds is the
   shape the hardware can boot.

The copy reads its source over the **data** bus at ROM addresses. That works because text on this
core is readable and writable from that bus
([ADR-0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md)); `test/asm/textload.S`
is the program that already said so. Nothing in `rtl/` changed.

### Two linker scripts, not one

`test/asm/sections.lds` is untouched and the 59 assembly programs are byte-identical through it —
`gcc -E` output, ROM image and RAM image, all 59, verified against `25cbce0`. They are freestanding
and must not grow a copy loop they do not need, and a single script cannot give one program's
`.data` a ROM load address and another's a RAM one. The cost is a duplicated four-line memory map,
which each file names and points at the other about.

The visible consequence is that the two shapes are extracted differently: a `.S` program's ROM
image is `.text` and its RAM image is everything else, while a `.c` program's ROM image is `.text`
plus `.data` and its RAM image is `.tohost` alone. `.tohost` stays loadable at its own address in
`ram` because it is an IO window rather than program data — the runners watch the first word of RAM
for a verdict from cycle 0, so it has to read zero before the copy has run.

That build branch is now spelled out in three places: `test/run_tests.sh`, `test/cosim.py`'s
`assemble()` and the Makefile's `soc-rom`. All three already carried their own copy of the single
shape's build line, so this does not add a surface — but it does widen one, and consolidating the
three is the obvious follow-up.

## What the `.bss` test is worth, and why check 2 is not the one that grades it

Simulated RAM comes up zeroed. A `.bss` that was never cleared therefore reads exactly like one
that was, and a boot-time "is `.bss` zero?" check passes whether or not `crt0` ever ran a zeroing
loop. That is a grader that cannot fail, which this repo has shipped five of.

So `test/asm/datainit.c` scribbles `0xdeadbeef` over both tables and calls `runtime_init` again —
the same routine the reset path ran, now over memory that is demonstrably not zero — and re-checks.
`runtime_init` is separately callable for exactly this reason.

Both red directions were run, not argued:

| mutation of `test/crt0.S` | result |
|---|---|
| the `.bss` zeroing loop deleted | `FAIL 4`, 269 retires |
| the `.data` copy loop deleted | `FAIL 1`, 67 retires |
| neither | `PASS`, 400 retires |

The `.bss` mutation fails at check 4 and not at check 2, which is the point: check 2 is the one
that cannot fail here, and the table says so.

## A C entry's floor is a silence bound, not an observation

`test/OBSERVED_FLOOR` was designed for assembly and does not transfer. An `.S` program assembles to
a fixed instruction sequence, so its retire count holds across toolchains by construction — which
is exactly why a `>=` floor over the measured number is a meaningful check there. A C program's
count is whatever that gcc chose to inline and schedule. `test/asm/datainit.c` retired **400**
under a local `riscv64-elf-gcc` and **395** on the CI runner, from identical source, and the first
version of this change recorded 400 as its floor and went red on the gate.

Lowering it to 395 would have made CI pass and left the defect. The floor exists to catch a program
going **quiet**, and a number tracking a compiler's output conflates that with "a different gcc
emitted five fewer instructions" — two failures the gate cannot tell apart. This repo already knows
its toolchains disagree: `make fit` reads about 21 cells apart on identical RTL, and `CLAUDE.md`
says to quote the CI number for that reason.

So a `.c` line's two numbers are **16**, picked from the check rather than the measurement. Every
recorded instance of the defect this file exists for produced zero retires — `rvfi_valid` never
asserting, an `ifdef` dropping the shadow payload, `write_cxxrtl` optimising the monitor instance
away — or one, the iverilog under-sensitivity defect measured at "0 writes and 1 retire". Zero is
already `test/cxxrtl.cc`'s exit 6, so the bound only has to clear one; 16 is an order of magnitude
above the defect, two below any C program's real count, and nothing a compiler chooses can move it.
`spec-checked` has the same problem and takes the same bound — it tracked retires exactly here
(395 of 395 on CI), so whatever moves one moves the other.

Two options were rejected. **Pinning the toolchain for C** is heavier than this ticket and would
buy reproducibility the check does not need. **Dropping `.c` from the floor entirely** relies on
exit 6 alone, and exit 6 covers only *zero* — the one recorded defect that produced a nonzero count
produced 1, which no `MONITOR-SILENT` path catches.

The rule is enforced rather than left to the header: `test/run_tests.sh` rejects a `.c` floor above
64 before it runs a program, with a message naming the reason. The header is a decision procedure
someone follows under pressure, and its "copy the table's third column" instruction is what
produced the red gate in the first place. The name set is untouched by any of this — a `.c` program
that vanishes from the suite is red in both directions exactly as an `.S` one is.

## The number that decides whether SPI-flash boot becomes urgent

Measured at `25cbce0` with `riscv64-elf-gcc`, `-Os`, on `test/asm/datainit.c`:

| | bytes |
|---|---|
| `.text` | 252 |
| — of it `test/crt0.S` (`_start` 22 + `runtime_init` 60) | 82 |
| — of it the test program itself | 170 |
| `.data` initialiser, carried in ROM | 32 |
| **ROM image total** | **284** |
| ROM (`SOC_ROM_WORDS` 2048, 16 EBR) | 8192 |
| **left** | **7908 (96.5%)** |

**The reusable runtime costs 82 bytes, one percent of the ROM.** There is no C prologue beyond it:
no libc, no `.init_array`, no stack unwinder. On this evidence SPI-flash boot stays deferred — the
8 KB ceiling is a real constraint on how much *program* fits, and it is not measurably a constraint
on being able to boot one at all. The number to watch is the ROM image total, not the copy stub.

`make soc-timing` measured 78.51 ns / 12.74 MHz for both `SOC_PROG=add.S` and
`SOC_PROG=datainit.c` at the default seed — bit-identical, because a program only changes the
block RAM's initial contents and nothing nextpnr places. Switching the default is therefore free,
and it buys an image shape CI builds on every run instead of one nothing exercises.

## What this does not do

It is not the SPI-flash bootloader, and it does not make SPRAM initialisable. A program still
cannot have more `.data` than fits in ROM alongside its `.text`, because the initialiser rides in
the same 8 KB. `test/asm/datainit.c` is a test program, not a demonstration that a real C
application fits; the next honest measurement is a program with actual work in it.
