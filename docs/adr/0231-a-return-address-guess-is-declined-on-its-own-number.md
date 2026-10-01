# 0231 — A return-address guess is declined on its own number

Status: Accepted. 2026-10-01. Measured on `main` at 11cc506 (the merged fetch refactor,
ADR-0221 as amended), compiled with the pinned xPack `riscv-none-elf-gcc` 15.2.0.

## The question

D guesses a `jal` and a backward conditional branch taken and falls through for everything
else, so a `jalr` — in compiled code, almost always a function return — is never guessed
and every one is a redirect. This ADR measures what a perfect return guess could save,
what a one-entry return-address register would actually hit, and whether either is worth
the up5k's remaining ~196 `ICESTORM_LC` (5,084 of 5,280) and a new arm on the fetch loop.

## What was counted, and how

`test/cxxrtl.cc` reads rtl/executor.v's own signals each cycle under `--stalls` and
`test/stall_report.py` prints and cross-checks the result; no RTL changed, so the netlist
digest is unmoved and no sweep is owed for the instrument. A **return** is a committed
`jalr` with `rd = x0` and `rs1 = x1` or `x5`. A redirect's **cost** is the run of cycles X
resolves nothing after it, up to the next resolving cycle (the dropped wrong-path word and
the fetch bubble). Every redirect measures exactly two such cycles, `jalr` and branch miss
alike, on Dhrystone and CoreMark, and 2.04 on average on the suite (one `jalr` there
waits a cycle longer). The return-register replay pushes the link address on
every committed `jal`/`jalr` with `rd` of `x1` or `x5` and pops on a return; a hit is a
return whose resolved target equals the register, charged with its own redirect's two
cycles. A 64-entry stack runs beside it as the bound for unlimited nesting. The replay
updates at commit with no lag, so it is an upper bound on a real X-owned register
(below). `test/stall_report.py` fails on any subset that exceeds its superset (returns
over `jalr`, one-entry hits over stack hits, saved cycles over `jalr` cycles), and
`test/probe_gates.sh` forces each red direction.

## The numbers

Percent of the workload's own counted cycles, from `make cycles`, `make dhrystone` (2,000
runs) and `make coremark` (100 iterations, simulated at 16 KB of ROM).

| | `.S` suite | Dhrystone | CoreMark |
|---|---|---|---|
| counted cycles | 30,986 | 1,236,944 (1,206,025 in the program's own window) | 36,044,621 |
| committed instructions | 23,964 | 946,449 | 28,848,750 |
| `jalr`, % of committed | 0.10% (23) | 2.12% (20,057) | 0.74% (214,083) |
| returns, % of `jalr` | 52.2% (12) | 100% | 85.0% (182,034) |
| `jalr` redirect cost, % of cycles | 0.15% | 3.24% | 1.19% |
| every other redirect, % of cycles | 5.92% | 2.31% | 4.71% |
| one-entry register: hit rate on returns | 91.7% | 80.0% | 91.4% |
| one-entry register: cycles saved, % of cycles | 0.07% | 2.60% | 0.92% |
| 64-entry stack: hit rate | 91.7% | 100% | 100% |
| 64-entry stack: cycles saved, % of cycles | 0.07% | 3.24% | 1.01% |

Against the programs' own figures: Dhrystone's one-entry saving is 32,100 cycles of
1,206,025, 0.943 to about 0.968 DMIPS/MHz; CoreMark's is 332,824 of 36,010,251, 2.776 to
about 2.80 CoreMark/MHz. The ceiling for a perfect return guess is 3.24% and 1.01%. The
one-entry register misses nested returns: 20% of Dhrystone's returns and 9% of
CoreMark's, which the 64-entry stack's 100% shows is the whole gap. A leaf `ret` one
instruction behind its `jal` would lose more again on a real build, because the register
is not yet written.

## Decision: decline

Build nothing. The case against, in order of weight:

1. **The win is one workload deep.** The suite's 0.07% is nothing, CoreMark's 0.92% is
   about what a compiler bump moves (ADR-0190 measured 2.203 to 2.155 from the compiler
   alone), and only Dhrystone clears 2%. The up5k clock is a step function (12 MHz), so
   cycles are the whole product and nothing else rises with them.
2. **Dhrystone is the best case for the idea.** Every one of its `jalr` is a return and
   its call tree is shallow, the shape this register serves.
3. **The register belongs in X, which makes it laggier than the replay.** D also guesses
   on the wrong path, so a push in D is exactly the state commitment 1 forbids ("no state
   may exist that a later cycle must un-commit"). X owns the committed stream, so the
   register would be written when the call resolves and read by D two cycles behind the
   replay. Near-adjacent call and return pairs miss, so the replay's hit rates are
   ceilings.
4. **It adds an arm to the loop that sets the clock.** `predicted_pc`'s select would gain
   a return term decoded from the raw word, with a registered source. That is shallower
   than the `jal` arm's adder, so the likely cost is small, but it is a variance (ADR-0113:
   sixteen seeds, not eight) spent against a 12.0 MHz floor that does not slide
   (ADR-0066).
5. **The area is an estimate, not a measurement.** About fourteen flops (the low address
   bits of the text window plus a valid) and the mux arm, by structure alone; no cell was
   measured and none is claimed. That is well inside 196 LC, so area is not the objection.

The larger lever this measurement exposes is not the return. Every redirect costs two
cycles, and the non-`jalr` redirects (branch and `jal` misses, traps, `mret`) are 5.92%
of the suite, 2.31% of Dhrystone and 4.71% of CoreMark: more than the `jalr` share on two
of three workloads, and on CoreMark about five times what a perfect return guess saves.
They are the next place to look, and where the 196 LC is better spent.

## What would reopen it

A workload with `jalr` above ~3% of committed instructions and returns above 80%
(Dhrystone is the only one here), or a sixteen-seed measurement that the arm costs
nothing. If the architect builds it anyway, the specification is: one register in X
holding the committed link's low `PREDICT_LOW_BITS` plus a valid, written when a
`jal`/`jalr` with `rd` of `x1`/`x5` commits and cleared when a return commits; D guesses a
return taken at that register's address only while valid; X's existing
`resolved_target != guessed_pc` test already corrects a wrong guess; budget 60 LC;
sixteen paired seeds against this commit (`soc/paired_sweep.sh`); and F and G re-measured
with `make -C formal remeasure-fg` and the `pcloop` proof re-closed, since the guess adds
a term to the fetch address.
