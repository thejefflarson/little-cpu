# 0134 — CoreMark lands simulated, and there is no ice40 Hazard3 number to compare it to

Status: accepted · 2026-08-24 · a second benchmark, and the comparison it does not yet support

## Context

`make dhrystone` was described as "the only figure comparable to another project's". That was true
against VexRiscv, which publishes Dhrystone. It stopped being true generally once Hazard3 — the
RISC-V core inside the RP2350 — became a reasonable core to compare against, and Hazard3 publishes
CoreMark/MHz, not Dhrystone.

**The obvious next step is also the mixed-configuration error this repo already has a name for.**
Hazard3's published **4.15 CoreMark/MHz** is its RP2350 configuration: Zba/Zbb/Zbs/Zbkb, a fast
multiplier, a branch predictor. Its iCE40/iCEBreaker build
(`example_soc/fpga/fpga_icebreaker.v` in that project) has none of that —
`EXTENSION_C=0`, every bitmanip parameter `0`, `MULDIV_UNROLL=1` (one bit of division per cycle),
`MUL_FAST=0`, `BRANCH_PREDICTOR=0`, `U_MODE=0`, `PMP_REGIONS=0`, `CSR_COUNTER=0`, at 12 MHz — the
same board clock this core targets. Quoting 4.15 against this core's own number the way this ADR
was almost written would have been the same error ADR-0098 spent several paragraphs correcting for
VexRiscv: two numbers from two configurations, reported as one comparison.

**So this ADR does not compare to Hazard3.** It records what CoreMark reports on this core, states
the configuration it was measured on, and states plainly that Hazard3 has published no CoreMark/MHz
figure for its iCE40 build — so no same-configuration comparison exists yet. What is comparable is
this: Hazard3's own README frames 4.15 as *near the ceiling of what an in-order RV32 core can reach
on CoreMark*, driven by exactly the features this core is priced and declined for. The gap this ADR
measures is the gap between "stall-only, no bitmanip" and "everything CoreMark rewards", not the gap
to a specific number nobody has published on a comparable part.

## What shipped

`test/bench/coremark/` vendors five files unmodified from `eembc/coremark` at commit
`1f483d5b8316753a742cbf5590caf5bd0a4e4777` (2025-05-01), Apache-2.0: `core_list_join.c`,
`core_main.c`, `core_matrix.c`, `core_state.c`, `core_util.c`, plus `coremark.h` and upstream's own
`coremark.md5`. `test/bench/coremark/PINNED.sha256` is this repo's own manifest, checked on every
`make coremark` run — not upstream's `coremark.md5`, which disagrees with a byte-identical fetch of
`coremark.h` at the same commit and so cannot be trusted as a tripwire for this tree. Keeping these
five files untouched is not a style choice: CoreMark's trademark agreement (bundled in the same
`LICENSE.md` as the Apache-2.0 grant) permits using the name "CoreMark" only "in connection with...
an unmodified copy of the software", and the port below touches none of them.

`test/bench/core_portme.h` and `test/bench/coremark_port.c` are the porting layer EEMBC's own
documentation asks every target to write — the same split `test/bench/dhry_port.c` uses for
Dhrystone. `mcycle`/`minstret` time the run, `portable_fini` reads the live `core_results` back
through the `core_portable` pointer it is handed (an `offsetof` reach into a struct this file never
redefines), and the report goes into a RAM buffer and, when `COREMARK_UART` is set, out the wire —
the same `DHRY_UART` pattern, so a simulated run and a wired one cannot disagree about what was
reported. `COREMARK_FLAGS` is `#error`'d without a definition, for the same reason `DHRY_FLAGS` is:
a CoreMark/MHz figure whose flags are unknown is not a measurement, and EEMBC's own run rules require
disclosing them.

## Simulated at 16 KB of ROM, not this part's 8

CoreMark does not fit `rtl/imemory.v`'s shipping 2048-word ROM — the linked image is 13,272 bytes,
5,080 over the 8,192-byte budget `test/bench/bench.lds` gives Dhrystone, the same wall
`test/asm/rvc.S` hits at 12,256. `test/bench/coremark.lds` links against `test/testbench.v`'s
`ROM_WORDS = 4096` instead — 16 KB, a configuration nothing under `rtl/` implements yet — so a link
that succeeds here is not one `rtl/littlesoc.v` can boot. That caveat is stated by the Makefile
target, by `run_coremark.sh`, and by the program's own printed report: three places, because a
number that silently forgot its own memory configuration would be worse than no number.

This is the first of two measurements. The second is on hardware, once the SPI-flash boot path this
repo's own state notes lists as deferred lands and the shipping ROM can hold something this size.

## The number

100 iterations, `-O2`, `-march=rv32imac_zicsr_zifencei`:

```
Compiler       : GCC 16.2.0
Compiler flags : -march=rv32imac_zicsr_zifencei -mabi=ilp32 -O2 -std=c11 -ffreestanding
                  -fno-tree-loop-distribute-patterns -Wall -Wextra -Werror
Memory config  : MEM_STACK, TOTAL_DATA_SIZE 2000 bytes, SEED_VOLATILE performance run
Iterations     : 100
Cycles         : 55,200,557
Instructions   : 28,485,349
CPI            : 1.93
CoreMark/MHz   : 1.811
Self-check     : PASS (list/matrix/state CRCs matched against the published 2K performance values)
```

**1.811 CoreMark/MHz, simulated at 16 KB of ROM.** `core_main.c`'s own CRC check — unmodified,
comparing this run's `crclist`/`crcmatrix`/`crcstate` against the values EEMBC published for the
2K-performance seed set — is what says the benchmark ran correctly, not merely that it terminated.

**The ratio is stable well inside EEMBC's own 10-second minimum.** That rule exists to average out
a real clock's jitter, which a cycle-exact simulator does not have: 1.811 reproduced to three decimal
places at 10, 40 and 100 iterations (5.5M, 22.1M and 55.2M cycles respectively), so the fixed
per-run cost — `test/crt0.S` zeroing 2000 bytes, `core_main.c`'s own report — is already negligible
against the measured interval. `COREMARK_ITERATIONS` defaults to 100 for that reason: it is a
couple of minutes of `--stalls`-instrumented simulation rather than the roughly fifteen a literal
120M-cycle run (10 seconds at the board's 12 MHz) would cost at this simulator's own rate, and it
is not buying a different number, only a longer wait for the same one. A caller who wants the
literal duration can still ask for it with `COREMARK_ITERATIONS=<n>`.

## The decomposition

`--stalls` gives the same per-reason accounting `make cycles` prints for the hand-written suite, on
CoreMark's own compiled mix:

```
issue     51.6%
hazard    28.9%
region    10.2%
operand    9.3%
(divider, atomic, serialize, fetch, bus: under 0.6% together)
```

**Hazard is priced already, and it is the largest single stall reason here.** ADR-0083 measured
forwarding the executor's result to every operand reader at 12.9% of suite cycles, missing 12 MHz
outright, and confining it to the executor's own operands at 7.5%, holding 12 MHz on a thinner
margin — both declined. CoreMark's own hazard share, 28.9%, sits between the hand-written suite's
35.7% and Dhrystone's 22.5%, which is what "CoreMark leans on it harder than Dhrystone does" means
concretely rather than as a qualitative claim: more of CoreMark's own instruction mix reads a
register a nearby instruction just wrote than Dhrystone's does.

**What this ADR does not decompose is the bitmanip share**, because nothing in this tree has priced
it yet — no Zba/Zbb/Zbs build of this core exists to measure the delta against. That is a real
number this repo does not have, not an omission from this report: `operand` and `issue` above are
upper bounds on where a bitmanip win could come from (narrower immediate sequences, fewer
instructions retired for the same work), not a measurement of it.

## What this is not

Not a gate and no ratchet, the same as `make dhrystone`: there is no CPI floor in this repo, and
nothing here invents one. Not on CI, for the same reason Dhrystone's own multi-minute runs are not —
this one runs several times longer. Not a claim that this core is slow in some absolute sense: it is
stall-only by choice (ADR-0083) and has no bitmanip extension at all, and CoreMark is built to reward
both. A figure well under a core with forwarding and Zba/Zbb/Zbs is the price of the four goals,
quantified rather than assumed — the same reading ADR-0098 asks for when this core's Dhrystone number
is read beside VexRiscv's.
