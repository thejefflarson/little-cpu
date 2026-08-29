# 0143 — Why `DHRY_RUNS` and `DHRY_CFLAGS` are what they are

Status: Accepted · 2026-08-29

## Context

`make dhrystone` builds and runs Dhrystone 2.1 under the simulator. Its two knobs, `DHRY_RUNS` and
`DHRY_CFLAGS`, were each set from a measurement, and a 23-line comment carried that measurement
directly above them in the Makefile — long enough to be one of the largest comment blocks in a file
CLAUDE.md holds to "one or two plain sentences." This ADR is that derivation, moved out; the
Makefile keeps the values and a one-line pointer here.

The text below is carried over verbatim from the comment it replaces, as a measurement with a date
on it; it is not re-verified against a current tree.

## The derivation

Dhrystone is the one number this core can be quoted against other cores'. Not a prerequisite of
anything and not on CI: there is no CPI ratchet here and this adds none. It reports DMIPS/MHz, the
ROM image against the SoC's 8 KB, and the same per-reason cycle accounting `make cycles` prints —
which is the first read of that split on compiled code rather than on hand-written assembly.

**`DHRY_CFLAGS` is the measurement's other half.** The same string compiles the benchmark and is
compiled into it, so the flags print beside the number and cannot be separated from it;
`test/bench/dhry_port.c` will not build without them. Changing them changes the number — Dhrystone
is famously sensitive to the optimiser — so quote both or neither, the same rule `make fit` and
`make soc-timing` already carry about their toolchains.

`-O2` rather than the suite's `-Os`: it is what the cores in the comparison set publish, and the ROM
budget is what says whether this part can afford it. `-fno-tree-loop-distribute-patterns` keeps gcc
from rewriting the byte loops in `dhry_port.c` into calls to the very routines they define.

2000 runs, not the smallest number that produces a figure. `test/crt0.S` zeroes Dhrystone's 10 KB
`Arr_2_Glob` a word at a time before `main`, and that loop's stall mix is nothing like the
benchmark's — at 500 runs it is a tenth of the accounted cycles and at 2000 it is under three
percent. The runner prints the residual so the number is checked rather than assumed.

## Consequences

- `Makefile`'s `DHRY_RUNS` and `DHRY_CFLAGS` keep a one-line pointer to this ADR; the reasoning
  behind their values lives only here.
- Changing either is a re-derivation, not a tweak: re-measure the residual and the comparability
  argument against the tree the change lands on, and record it as a new ADR if the values move.
