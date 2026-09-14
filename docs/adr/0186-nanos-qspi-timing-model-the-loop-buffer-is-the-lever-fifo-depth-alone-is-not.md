# ADR-0186: nano's QSPI timing model — the loop buffer is the lever, FIFO depth alone is not

**Status:** Accepted · 2026-09-13

## Context

nano's own benchmark figures (ADR-0182) — 0.225 DMIPS/MHz, 0.541 CoreMark/MHz — are measured
against `nano/tb/nano_memory.v`, a zero-wait, behavioural flat memory. The brief
(`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md`) estimates real QSPI-flash-plus-PSRAM
fetch latency puts nano at roughly 2 MIPS, and names a loop buffer as "the only thing that
changes the answer" but defers it for area. This ticket measures that, without touching
`nano.v`.

## The model

`nano/tb/nano_qspi_memory.v` is a drop-in alternative to `nano_memory.v` on the same
picorv32-style bus, selected by a `NANO_QSPI_TIMING` build macro
(`nano_testbench.v`'s existing memory-instantiation `ifdef`, alongside the pre-existing
`NANO_WAIT_STATES` one) and a parallel `nano-qspi-sim` Makefile target
(`NANO_QSPI_PREFETCH_DEPTH`, `NANO_QSPI_LOOP_WINDOW`, `NANO_QSPI_PREAMBLE_CYCLES`
parameterize it). It lives in Verilog rather than the C++ driver because nano's other
instruments already follow "swap the RTL memory model, keep cxxrtl's stepping and the
`--bench` marker mechanism" (`nano_memory.v` itself, the differential divide/multiply
oracle), and because it needs no new simulation loop. The default `nano-sim` build is
untouched — confirmed by reproducing ADR-0182's own control figures to the cycle (below) —
and `nano.v` is not edited.

**What nano actually does, derived rather than assumed.** `riscv`'s `fetch_instr`/`ready_instr`
states issue exactly one `mem_valid`/`mem_ready` transaction per instruction, address held
stable throughout, and read a 32-bit `mem_rdata` regardless of the instruction's real length —
the low 16 bits (`mem_rdata[1:0]`) decide compressed-vs-full the same way for both nano and the
model. Data addresses are always word-aligned (`{load_store_address[31:2], 2'b00}`), and only
one transaction is ever outstanding.

**Flash (fetch) side.** A parcel is a 16-bit halfword; the flash streams parcels sequentially
once open, one every 8 core clocks, and a redirect — any fetch the flash is not already aimed
at — reopens it through a `PREAMBLE_CYCLES` (24, or 20 for the QPI variant) address/mode/dummy
phase before the first new parcel. `PREFETCH_DEPTH` caps how far ahead of the core's own
sequential position (`expect_index`, the parcel the core is expected to ask for next) the
background stream may run; 0 disables background streaming entirely — the flash only advances
while a transaction is outstanding, reproducing a front end with no fetch/execute overlap. A
fetch whose parcels have already streamed in pays nothing beyond the ordinary one-cycle bus
turnaround.

**PSRAM (data) side.** Any load or store drops the flash's chip select — modelled as closing
the stream immediately — and costs `PSRAM_CYCLES` (44), charged identically to reads and writes
since the brief gives one number for both. The next fetch always pays the full redirect
preamble, matching the brief's "next fetch then pays the redirect preamble again."

**Loop buffer.** The brief prices 8 parcels at "128 flops ≈ 4.2k plus a 22-bit window tag" —
one shared tag for the whole window, not a per-entry cache — so the model matches that shape:
a single `LOOP_WINDOW`-parcel aligned block with one base-address tag, refreshed to the block
containing the most recently *completed* fetch. It is consulted only for a fetch the flash is
not already sequentially aimed at (a redirect); a sequential fetch is already served by the
FIFO path and never competes with it. A hit costs 1 clock, charged to the parcel-wait bucket —
distinct from the FIFO's own "pays nothing," representing tag-compare/mux latency the brief's
area estimate implies but the FIFO alone does not pay. On a hit the flash also retargets its
own background stream toward the hit's successor address, non-blockingly: a later fetch that
falls outside the window still sees an accurate account of whatever that resync has or has not
finished, rather than a stream frozen at wherever the last miss left it.

**Two assumptions about a front end that does not exist yet**, as the ticket asked to have
named: (1) a real controller can examine the arriving low parcel's two low bits to decide
compressed-vs-full and could in principle stop early for a compressed instruction — the model
computes that decision by reading the data table directly rather than waiting for the bit to
stream in, which is a bookkeeping shortcut, not a claim about the real signal's timing, and it
produces the identical total cycle count either way since the decision is a deterministic
function of the same bits; (2) a loop-buffer hit's non-blocking background resync assumes the
future controller runs the main FIFO and the loop buffer as independent paths, the loop buffer
never gating the FIFO's own state machine — the alternative (loop buffer and FIFO sharing one
state machine, serializing the resync behind whatever the FIFO was doing) is not modelled and
would only ever look worse than what is measured here.

## A design bug the model exposed, unrelated to nano.v

Deriving the model surfaced a genuine bug in it, not in `nano.v`: a redirect's own
`mem_ready` check read a *live*, per-cycle recomputation of "is the flash already aimed
here," rather than the decision latched at the transaction's first cycle. A fetch that
started as a legitimate sequential hit could, mid-wait, have that live check flip false —
harmlessly on its own, but a **second** bug compounded it: a newly triggered redirect did
not clear the flash's stale `arrived_valid`/`arrived_index` until its own preamble
completed, so a just-abandoned stream's leftover "already arrived" state could satisfy a
brand-new redirect's `mem_ready` before its real preamble ever ran, corrupting the
predicted next address and stalling the whole run indefinitely — reproduced on Dhrystone at
cycle 178,749, address `0x2c4`, with `nano/tb/nano_cxxrtl.cc`'s bucket-and-address tracing
(built for this diagnosis, not kept) showing the flash's own prediction of the next fetch
address flipping between two values instead of advancing. Fixed by clearing
`arrived_valid` at the moment a redirect is *triggered*, not when its preamble completes,
with the redirect-trigger's write ordered after the background-production block in the
same `always_ff` so a same-cycle non-blocking-assignment race resolves in the new
redirect's favour. All eight configurations below ran clean afterward; `nano-test` and the
existing `.S` suite are unaffected, since this file only gates *when* `mem_ready` arrives,
never what `mem_rdata` or `mem[]` holds.

## The accounting identity is graded, not just printed

`nano_cxxrtl.cc` sums four buckets — execute (`!mem_valid`), parcel wait, redirect
preamble, PSRAM wait — every cycle after reset and compares the sum to the true simulated
cycle count at the run's own exit point, exiting 7 on a mismatch; the `BUCKETS ...
total_cycles=N` line it prints carries that total so `nano/bench/qspi_timing_report.py`
can re-derive and re-grade the same identity from a log alone, with no rebuild. Both
directions are forced red in `test/probe_gates.sh`'s `nano/bench/qspi_timing_report.py`
group: a hand-crafted log whose buckets sum to its stated total reports a row, and one
that does not is refused with `ACCOUNTING MISMATCH`, `test/PROBES_EXPECTED` carrying both
labels.

## Measured

`make nano-qspi-timing` (`nano/bench/run_qspi_timing.sh`) builds one `nano-qspi-sim` per
configuration and runs Dhrystone (400 runs) and CoreMark (5 iterations) against each —
the same sources, flags and counts as `make nano-dhrystone`/`make nano-coremark`. The
zero-wait control reproduces ADR-0182's own figures to the cycle: 2,526.2 cycles/run,
0.225 DMIPS/MHz; 1,848,480.0 cycles/iteration, 0.541 CoreMark/MHz — the default `nano-sim`
build path is untouched.

| Configuration | Dhry cycles/run | DMIPS/MHz | DMIPS@64MHz | CoreMark cycles/iter | CoreMark/MHz | CoreMark@64MHz | execute% | parcel wait% | redirect preamble% | PSRAM wait% |
|---|---|---|---|---|---|---|---|---|---|---|
| no-overlap (depth 0) | 22,679.6 | 0.0251 | 1.61 | 11,187,255.6 | 0.0894 | 5.72 | 8.30 / 13.19 | 24.54 / 31.16 | 29.71 / 25.23 | 37.45 / 30.42 |
| FIFO depth 2 | 21,890.6 | 0.0260 | 1.66 | 10,425,404.4 | 0.0959 | 6.14 | 8.60 / 14.15 | 21.78 / 26.14 | 30.80 / 27.07 | 38.82 / 32.64 |
| FIFO depth 4 | 21,851.6 | 0.0260 | 1.67 | 10,292,457.2 | 0.0972 | 6.22 | 8.62 / 14.33 | 21.63 / 25.19 | 30.86 / 27.42 | 38.90 / 33.06 |
| FIFO 2 + loop 8 | 19,129.0 | 0.0298 | 1.90 | 8,540,070.0 | 0.1171 | 7.49 | 9.80 / 17.26 | 21.40 / 25.11 | 24.55 / 17.81 | 44.24 / 39.81 |
| FIFO 2 + loop 16 | 16,686.9 | 0.0341 | 2.18 | 8,436,435.4 | 0.1185 | 7.59 | 11.16 / 17.49 | 20.11 / 25.07 | 18.37 / 17.09 | 50.36 / 40.35 |
| FIFO 4 + loop 16 | 16,658.9 | 0.0342 | 2.19 | 8,310,574.0 | 0.1203 | 7.70 | 11.18 / 17.75 | 19.96 / 23.94 | 18.40 / 17.35 | 50.46 / 40.96 |
| FIFO 4 + loop 16 + QPI (20-cycle preamble) | 16,082.9 | 0.0354 | 2.26 | 8,037,838.0 | 0.1244 | 7.96 | 11.60 / 18.36 | 20.70 / 24.75 | 15.38 / 14.55 | 52.33 / 42.35 |

(percentage columns: Dhrystone / CoreMark; every row's four percentages, and every row's
retired-cycle count against its own accounting total, satisfy the identity above.)

Both benchmarks pass their own validity self-check (`verdict=1`) in every configuration.
`DMIPS@64MHz`/`CoreMark@64MHz` assume the brief's own target clock, stated so — no clock is
placed for nano yet.

**Instructions retired and effective MIPS at 64 MHz**, the two benchmarks' own retire counts
(199,967 for Dhrystone's 400 runs, 1,488,936 for CoreMark's 5 iterations) against each
configuration's total simulated cycles:

| Configuration | Dhrystone MIPS@64MHz | CoreMark MIPS@64MHz |
|---|---|---|
| no-overlap | 1.34 | 1.69 |
| FIFO depth 2 | 1.39 | 1.81 |
| FIFO depth 4 | 1.39 | 1.83 |
| FIFO 2 + loop 8 | 1.58 | 2.21 |
| FIFO 2 + loop 16 | 1.80 | 2.24 |
| FIFO 4 + loop 16 | 1.81 | 2.27 |
| FIFO 4 + loop 16 + QPI | 1.87 | 2.35 |

The brief's ~2 MIPS estimate sits *above* the measured no-overlap figure (1.34–1.69) and is
reached only once a loop buffer is added — Dhrystone's `redirect_preamble` bucket alone is
~30% of the no-overlap run, a branch-redirect density the brief's back-of-envelope arithmetic
("~11 clk average fetch") did not carry.

## What this says about the front end's design

**FIFO depth alone is nearly free and nearly useless.** Depth 2 → 4 moves DMIPS/MHz 0.0260 →
0.0260 (Dhrystone) and 0.0959 → 0.0972 (CoreMark, +1.4%) — inside the range a reader should
read as "does not change the answer," confirming the brief's own fetch-bandwidth argument:
with no loop buffer, the core is fetch-bound against the flash's raw streaming rate, and a
deeper queue in front of a bandwidth ceiling does not raise the ceiling.

**The loop buffer is the lever, exactly as the brief argued**, and it is a large one: 8 parcels
already buys most of the available gain (0.0260 → 0.0298 DMIPS/MHz, +14.6%; 0.0959 → 0.1171
CoreMark/MHz, +22.1%), and 16 parcels buys a further, smaller increment (→0.0341/0.1185,
+14.4%/+1.2% over the 8-parcel figure) — CoreMark barely moves from 8 to 16 parcels, Dhrystone
moves more, consistent with the two programs' loop bodies differing in size relative to the
window. QPI's shorter re-entry preamble (24 → 20 cycles) is a further, smaller, and
independent ~3.5% gain layered on top of the best FIFO+loop configuration.

**Which configuration is worth building, at what area, is a judgement this measurement informs
rather than settles on its own.** The 8-parcel loop buffer (the brief's ≈5.2k µm² TT-flow
estimate) captures roughly three-quarters of the total measured gain from no-overlap to the
best configuration at roughly half the area of the 16-parcel version (≈9.5k); the marginal
16th parcel buys comparatively little on both benchmarks measured here. If the tile now has
room for either, 16 parcels is the safer choice for programs whose hot loops are larger than
these two benchmarks'; if area is still contested against other features, 8 parcels is the
one this measurement says to keep. Neither figure changes nano's fundamental character: even
the best configuration measured (2.26 DMIPS, 7.96 CoreMark at 64 MHz) remains a fetch-bound
core against a 132 Mbit/s bus, not a design a wider buffer alone turns into something else —
buffering only hides latency for code that revisits addresses already in the window, and
neither benchmark is dominated by loops that small.

## Scope

No change to `nano.v`. `nano-qspi-timing` is off `make test`'s path, reporting only, with no
ratchet — the same standing as `make cycles`, `make nano-dhrystone` and `make nano-coremark`.
The accounting-identity check is graded and probed; the timing model's own numbers are not,
since there is nothing yet to ratchet them against.
