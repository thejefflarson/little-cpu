# 0143 — Why `COREMARK_ITERATIONS` and `COREMARK_CFLAGS` are what they are

Status: Accepted · 2026-08-29

## Context

`make coremark` builds and runs CoreMark under the simulator, at 16 KB of ROM rather than the part's
8 (see ADR-0136). Its two knobs, `COREMARK_ITERATIONS` and `COREMARK_CFLAGS`, were each set from a
measurement, and a 28-line comment carried that measurement directly above them in the Makefile,
mirroring the one `DHRY_RUNS`/`DHRY_CFLAGS` carried (moved to ADR-0142) — long enough to be one of
the largest comment blocks in a file CLAUDE.md holds to "one or two plain sentences." This ADR is
that derivation, moved out; the Makefile keeps the values and a one-line pointer here.

The text below is carried over verbatim from the comment it replaces, as a measurement with a date
on it; it is not re-verified against a current tree.

## The derivation

CoreMark is the figure the cores worth comparing to now (Hazard3's RP2350 build publishes
4.15 CoreMark/MHz and no Dhrystone number at all). Not a prerequisite of anything and not on CI, the
same as `make dhrystone`: no CPI ratchet exists here and this adds none.

**Simulated at 16 KB of ROM, double the part's 8.** `test/bench/coremark.lds` links against
`test/testbench.v`'s `ROM_WORDS` rather than `rtl/imemory.v`'s shipping 2048 words, because CoreMark
does not fit the smaller one — see `test/bench/run_coremark.sh`'s header for the wall it hits. The
figure this prints describes a machine that cannot be built until this part's deferred SPI-flash
boot path lands and the ROM grows; the program's own report and the runner both say so on every line
that matters.

**`COREMARK_FLAGS` is the measurement's other half**, the same rule `DHRY_CFLAGS` states for
Dhrystone: this string compiles the benchmark and is printed beside its own result, and
`test/bench/coremark_port.c` will not build without it. `-O2` rather than the suite's `-Os` for the
same reason Dhrystone takes it — it is what the cores in the comparison set publish.

**`COREMARK_ITERATIONS` is a compiled-in constant** (`SEED_METHOD SEED_VOLATILE`, not the
auto-tuning loop `core_main.c` offers) because a cxxrtl run has no wall clock to tune against.
EEMBC's own rule — run at least 10 seconds — exists to average out a real clock's jitter, which a
cycle-exact simulator does not have: measured here, the CoreMark/MHz figure this prints was
identical to three decimal places at 10 and at 40 iterations, so the ratio is already stable well
short of it. 100 keeps `make coremark` at a couple of minutes rather than the ~15 minutes 120M
cycles' worth of iterations would cost this simulator at its own `--stalls` rate. Raise it for a
longer run.

## Consequences

- `Makefile`'s `COREMARK_ITERATIONS` and `COREMARK_CFLAGS` keep a one-line pointer to this ADR; the
  reasoning behind their values lives only here.
- Changing either is a re-derivation, not a tweak: re-check the stability-at-N argument and the
  comparability argument against the tree the change lands on, and record it as a new ADR if the
  values move.
