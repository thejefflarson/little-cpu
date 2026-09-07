# ADR-0166: CoreMark fits the part's 8 KB ROM

**Status:** Accepted · 2026-09-06

## Context

`make coremark` has always been SIMULATED AT 16 KB OF ROM, double the up5k's
8 -- CoreMark does not fit the smaller one at `-O2`, and Dhrystone had already
made the trip to real hardware (ADR-0130, ADR-0163) while CoreMark had not. A
planning pass on this tree measured the image at every combination of `-O2`,
`-Os` and `-Os -flto`, with and without the UART, and found the lever is not
the flags: `test/bench/coremark_port.c` compiled to 3,040 bytes at `-Os` on
that pass, larger than `core_main.c`'s own 2,780, and roughly 1.1 KB of that
was this repo's own explanatory report prose baked into the ROM image.

## Decision

**The target is `coremark-rom-up5k`, not `coremark-rom`.** A second board route
for the iCESugar-Pro's 16 KB ROM landed the same sprint (ADR-0165) under the
name this one first claimed, at `-O2` against `test/bench/coremark.lds`. Make
resolves a redefined recipe **last-wins**, with only a
`warning: overriding recipe for target`, while a `?=` default beside it is
**first-wins** — so one name would have paired this route's `-Os -flto` and 800
iterations with that route's 16 KB linker script, or the reverse. That is not a
build failure: it links, it runs, and the flags string is compiled INTO the
image for EEMBC's disclosure line, so the report would have misstated its own
build. `test/makefile_target_test.sh` now grades it, off make's own warning.

The `PINNED.sha256` check is `test/bench/coremark_pin_check.sh`, shared with the
ECP5 route, rather than a third hand-copy — this route as first proposed
compiled the vendored sources with no check at all, and it is the route that
runs `iceprog`. CoreMark's trademark terms permit the name only for an
unmodified copy.


**The report shrinks; nothing about the benchmark does.** The five vendored
algorithm files under `test/bench/coremark/` are untouched and still checked
against `PINNED.sha256`. `core_portme.h` and `coremark_port.c` -- the only
files EEMBC's run rules permit changing -- now print EEMBC's own one-line
report syntax:

```
CoreMark 1.0 : 2.203 / GCC 16.2.0 <flags> / STACK, ROM <bytes>, RAM 64K / 1
Cycles         : <n>
Iterations     : <n>
Self-check     : PASS
```

`<bytes>` is read from three linker symbols `test/bench/{bench,coremark}.lds`
already define (`__data_load_start`, `__data_start`, `__data_end`) rather than
passed in, so it cannot drift from what the linker actually placed. The two
paragraphs of caveat text that used to live in the binary -- SIMULATED AT
16 KB, read the flags with the number -- are RELOCATED into
`test/bench/run_coremark.sh`, printed host-side after the run, where they cost
the ROM nothing. CPI and a separate instructions line are dropped from the
port's own report; `core_main.c`'s own report above it, and the stall
accounting below it, already carry that detail.

**Board flags are `-Os -flto`, not `COREMARK_CFLAGS`'s `-O2`.** Measured on
this tree, GCC 16.2.0: `make coremark`'s default `-O2` build (this slimmed
port, not the pre-slimming one the planning pass measured) is 2.203
CoreMark/MHz at 11,980 bytes against the 16 KB simulated budget -- still 3,788
bytes over the part's real 8,192. `-Os -flto` reads 1.780 CoreMark/MHz (a
19.2% cost against the `-O2` figure) at 6,960 bytes in the same simulated
build, 3,016 cycles faster over 20 iterations than `-Os` alone at the same
score. EEMBC's own run rules make this legal: "changing toolchain and
build/load/run options" is Allowed, and every source file is still compiled
with the one flag set. Linked against the shipping `test/bench/bench.lds`
(unchanged, 8 KB) with `COREMARK_UART` on and `COREMARK_UP5K_ITERATIONS`'s
800 iterations, `make coremark-rom-up5k` produces a **7,076-byte image, 1,116
bytes under the part's 8,192-byte ROM.**

**`COREMARK_UP5K_ITERATIONS ?= 800`, with `COREMARK_HZ` passed per board.**
`core_main.c`'s own `>=10 secs` self-check is genuine, not nominal: measured
on this tree, `-Os -flto` costs 561,530 cycles per iteration (11,230,611
cycles over 20), so 800 iterations is 449,224,000 cycles, 37.4 s at 12 MHz --
comfortably over the ten seconds required and comfortably under `mcycle`'s
32-bit wrap (10.5% of 2^32). `COREMARK_HZ` is a new build-time override on
`coremark_port.c`'s `time_in_secs()` (previously a hardcoded 12 MHz) so the
same port serves a differently-clocked board without lying to its own
self-check, the same role `CLOCK_HZ` already plays for `rtl/uart.v` on the
iCESugar-Pro.

**`make coremark-rom-up5k` and `make coremark-board` mirror `dhrystone-rom` and
`dhrystone-board` exactly**: one combined compile-and-link invocation (LTO
means no cross-TU-inlining concern here, unlike the three-separate-units rule
`run_coremark.sh` and `run_dhrystone.sh` both hold to for the number that gets
quoted against other cores), banked into `soc/rom_even.hex`/`soc/rom_odd.hex`,
flashed with `make prog`. The 8 KB budget is enforced by the linker, the same
way it already is for Dhrystone -- no new linker script, no relaxed one.

**The port also passes EEMBC's second required self-check.** "Required 2" of
EEMBC's own run rules is the 2K VALIDATION configuration (seeds
0x3415/0x3415/0x66), not just the scored 2K PERFORMANCE one -- distinct from
the seeds already run, and checked against EEMBC's own published CRCs (list
0xe3c1, matrix 0x0747, state 0x8d84; `core_main.c`'s own `known_id` 4).
`coremark_port.c` states both 2K configurations' CRCs unconditionally so
either can be grepped out of the source text regardless of which one an
`#ifdef COREMARK_VALIDATION` build compiles against, and `make coremark` now
builds and runs both images -- the second at one iteration, since
`crclist`/`crcmatrix`/`crcstate` latch from the first iteration only. It
prints no score: a validation run exercises the same code at a different
seed, not a different machine, and printing a second CoreMark/MHz figure
would only invite comparing it to the scored one by mistake.
`test/bench/run_coremark.sh` also cross-checks `coremark_port.c`'s own
`COREMARK_2K_VALIDATION_CRC*` literals against the pinned vendor `core_main.c`
array's own fifth entry before any compiler runs, the same defense the
existing 2K performance CRCs already get for the reason
`coremark_port.c`'s header states -- `test/probe_gates.sh` forces a mutated
literal there red.

## What this does not settle

**No RTL change, no period change, no area change** -- this is all in
`test/bench/`, `test/PROBES_EXPECTED` and the Makefile's board recipes.
`make coremark`'s own figure is unmoved: **2.203 CoreMark/MHz**, confirmed
by re-running it after this change start to finish, including the layout
inset ADR-0158 gives Dhrystone and the executor-only forwarding ADR-0154
measured -- the report now runs after `stop_time()` exactly as before, so a
smaller report costs nothing inside the measured interval.

**The 7,076-byte figure is a link, not a board run.** `make coremark-rom-up5k`
links successfully against the shipping 8 KB `bench.lds` and `./sim` runs the
same port (both configurations) under cxxrtl; no UPduino was available to this
change to flash `board.bin` and read a real number off the wire the way
ADR-0130 and ADR-0163 did for Dhrystone. `make coremark-board` needs the
board and root for `iceprog`, exactly like `make dhrystone-board`, and is off
`make test` and CI for the same reason. Getting a real CoreMark-on-part
figure, and confirming it is cycle-identical to `./sim` the way Dhrystone's
was, is the next measurement this ADR does not make.
