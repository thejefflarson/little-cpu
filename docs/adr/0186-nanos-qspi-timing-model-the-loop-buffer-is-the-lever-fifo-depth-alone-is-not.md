# ADR-0186: nano's QSPI timing model — the loop buffer is the lever, FIFO depth alone is not

**Status:** Accepted · 2026-09-14

## Context

nano's own benchmark figures (ADR-0182) — 0.225 DMIPS/MHz, 0.541 CoreMark/MHz — are measured
against `nano/tb/nano_memory.v`, a zero-wait, behavioural flat memory. The brief
(`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md`) estimates real QSPI-flash-plus-PSRAM
fetch latency puts nano at roughly 2 MIPS, and names a loop buffer as "the only thing that
changes the answer" but defers it for area. This ticket measures that, without touching
`nano.v`.

**This ADR supersedes its own first draft.** A security review of that draft's model found two
real correctness bugs in the loop buffer — one that undercounted a hit's cost by a cycle, one
that could deadlock a run outright — and a red-team read of the first draft's own conclusion
showed its headline gain was largely an artifact of those bugs, not of the loop buffer itself.
Both are fixed here, the model now carries two tests built specifically to catch them again, and
the table and conclusions below are re-measured on the fixed model. Where a claim below differs
from the original draft, it is the corrected one.

## The model

`nano/tb/nano_qspi_memory.v` is a drop-in alternative to `nano_memory.v` on the same
picorv32-style bus, selected by a `NANO_QSPI_TIMING` build macro
(`nano_testbench.v`'s existing memory-instantiation `ifdef`, alongside the pre-existing
`NANO_WAIT_STATES` one) and a parallel `nano-qspi-sim` Makefile target
(`NANO_QSPI_PREFETCH_DEPTH`, `NANO_QSPI_LOOP_KIND`, `NANO_QSPI_LOOP_WINDOW`,
`NANO_QSPI_PREAMBLE_CYCLES` parameterize it). It lives in Verilog rather than the C++ driver
because nano's other instruments already follow "swap the RTL memory model, keep cxxrtl's
stepping and the `--bench` marker mechanism" (`nano_memory.v` itself, the differential
divide/multiply oracle), and because it needs no new simulation loop. The default `nano-sim`
build is untouched — confirmed by reproducing ADR-0182's own control figures to the cycle
(below) — and `nano.v` is not edited.

**What nano actually does, derived rather than assumed.** `riscv`'s `fetch_instr`/`ready_instr`
states issue exactly one `mem_valid`/`mem_ready` transaction per instruction, address held
stable throughout, and read a 32-bit `mem_rdata` regardless of the instruction's real length —
the low 16 bits (`mem_rdata[1:0]`) decide compressed-vs-full the same way for both nano and the
model. Data addresses are always word-aligned (`{load_store_address[31:2], 2'b00}`), and only
one transaction is ever outstanding.

**Flash (fetch) side.** A parcel is a 16-bit halfword; the flash streams parcels sequentially
once open, one every `PARCEL_CYCLES` (8) core clocks, and a redirect — any fetch the flash is
not already aimed at — reopens it through a `PREAMBLE_CYCLES` (24, or 20 for the QPI variant)
address/mode/dummy phase before the first new parcel. `PREFETCH_DEPTH` caps how far ahead of the
core's own sequential position (`expect_index`, the parcel the core is expected to ask for next)
the background stream may run; 0 disables background streaming entirely — the flash only
advances while a transaction is outstanding, reproducing a front end with no fetch/execute
overlap. A fetch whose parcels have already streamed in pays nothing beyond the ordinary
one-cycle bus turnaround.

**PSRAM (data) side.** A load and a store are priced separately, matching TinyQV's own PSRAM
commands rather than one shared number: a load costs `PSRAM_LOAD_CYCLES` (44, Fast Read `0Bh`
with its 4-cycle dummy phase), a store `PSRAM_STORE_CYCLES` (33, `02h`, no dummy phase). Either
one drops the flash's chip select, modelled as closing the stream immediately. The flash's
re-open preamble for the next fetch starts the moment the PSRAM transaction *completes* — not
when that next fetch actually issues — because the flash's chip select is already free by then
and there is no reason the resync should wait for the core to ask; a fetch that starts inside a
still-running resync still pays whatever is left of it.

**Loop buffer.** The brief prices 8 parcels at "128 flops ≈ 4.2k plus a 22-bit window tag," and
this model measures two shapes of that idea rather than assuming one:

- **`LOOP_KIND=1`, tagged block**: one `LOOP_WINDOW`-parcel aligned block (`LOOP_WINDOW` a power
  of two) with a single base tag and one valid bit per slot, matching the brief's "one shared
  tag for the whole window" reading. A miss — same block or a different one — always re-tags and
  restarts the block at the missed target; there is no attempt to resume a partial fill from
  wherever the flash's background stream happened to be (the first draft tried that and it could
  deadlock — see below).
- **`LOOP_KIND=2`, last-N-parcels CAM**: a fully-associative record of the last `LOOP_WINDOW`
  parcels the core was actually delivered, in delivery order, with no alignment requirement.
  This is the architect's "last N parcels fetched" reading of the same area budget, priced
  identically per parcel.

Both are checked on **every** fetch that is not already being sequentially streamed to — a
redirect never skips the loop-buffer lookup just because the fetch that triggered it wasn't
itself a hit — and both answer in the cycle the hit is recognized: an on-chip SRAM or CAM read,
modelled the same way a parcel that has already streamed in also resolves same-cycle, not the
FIFO's own "already aimed, wait for delivery" path (a bug in the first draft charged a hit an
extra cycle by gating it on the same signal a streaming delivery uses; see below). A hit is
charged to the loop-hit bucket, distinct from the FIFO's "pays nothing," representing the
tag-compare/mux latency the brief's area estimate implies but the FIFO alone does not pay. On a
redirect that misses the loop buffer, the flash's background stream also retargets toward the
new address, non-blockingly, so a later fetch outside the window sees an accurate account of
whatever that resync has or has not finished.

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

## Two design bugs the model exposed, unrelated to `nano.v`

Deriving and then re-checking the model surfaced two genuine bugs in it, not in `nano.v`.

**A loop-buffer hit answered a cycle late.** `mem_ready`'s loop-hit arm read
`loop_hit_now ? xfer_active : ...`, and `xfer_active` is only true starting a transaction's
*second* cycle — so every hit paid a minimum of 2 clocks instead of 1, silently understating the
FIFO's own advantage over the loop buffer on the very numbers meant to show the loop buffer's
gain. Fixed to `loop_hit_now ? 1'b1 : ...`: a hit is an on-chip lookup and answers the cycle it
is recognized, same as an already-arrived streaming parcel.

**A same-block miss's resume logic could deadlock.** The first draft, on a `LOOP_KIND=1` miss
inside the block already tagged, tried to resume the flash's background production from wherever
it had reached rather than restarting the block — an optimization meant to avoid re-fetching
parcels the block already had. Under a large sweep at `LOOP_WINDOW=16` this hung an entire
Dhrystone run: `parcel_wait` consumed nearly the whole windowed cycle count
(3,758,938 of 3,766,316) with the flash's background stream evidently chasing a target it could
never actually satisfy in bounded time under some sequence of straddling redirects. Rather than
formally re-deriving the resume logic's correctness under time pressure, it was removed outright:
every miss, same-block or not, now re-tags and restarts the block at the missed target, the
already-reliable behavior the cross-block case already used. This is not only a safety fix —
`fifo2-tagged8`'s own CoreMark figure *improved* after removing it (0.1210 against the buggy
code's 0.1165), so the resume optimization was a net loss even where it did not hang outright.

**Both bugs point the same direction: the first draft's loop-buffer gain was overstated by
mechanisms that had nothing to do with the loop buffer working.** The architect's own read of
that draft noted that its measured "loop buffer" gain was reproducible with the FIFO alone once
the two bugs were accounted for — the loop buffer was doing less work than the numbers implied.
The corrected model's gain (below) is smaller than the first draft's and is now backed by two
tests built to fail if either bug's shape returns.

## The loop buffer's two invariants are now graded, not just measured

`make nano-qspi-loop-test` (`nano/bench/run_qspi_loop_buffer_test.sh`) checks two properties, each
built to be able to fail:

1. **A branch-free program costs identical cycles with the loop buffer off, tagged-block, or
   CAM.** `qspi_loop_micro.S`'s `KIND=0` body is 80 straight-line `addi`s; nothing in it ever
   redirects, so the loop buffer should never be consulted and never change the total. All three
   builds' `BUCKETS` lines must be byte-identical.
2. **A loop resident in the buffer pays no marginal preamble/redirect cost per iteration once
   warm.** `qspi_loop_micro.S`'s `KIND=1` body is a `REPS`-iteration decrement loop aligned to sit
   entirely inside one 16-byte block. Run at `REPS=200` and `REPS=400` and take the delta: the
   marginal 200 iterations must add zero `redirect_preamble`/`parcel_wait` cycles, a positive
   number of `loop_hit`s, and no more than 11 cycles/iteration of total window time (the measured
   figure is 10; 11 gives slack while still catching a hit that costs one cycle too many).

`nano/bench/run_qspi_loop_buffer_probe.sh` is the forced-red prerequisite
(`nano-qspi-loop-probe`, mirroring `nano_exec_probe.sh`'s pattern): it builds two mutated scratch
copies of `nano_qspi_memory.v` directly via yosys/clang++, bypassing `make` so the probe never
touches the checked-out tree, and requires each to fail for its own stated reason — reintroducing
the `xfer_active`-gated hit must fail invariant 2's cycle bound, and disabling the CAM's lookup
(`if (1'b0) cam_has0 = 1'b1;`) must fail invariant 2's "no marginal preamble/wait" check. Both
tests and both probe mutations are on `make test`'s path.

## The accounting identity is graded, not just printed

`nano_cxxrtl.cc` computes six independent, positive conditions every cycle after reset —
execute, parcel wait, redirect preamble, loop hit, handshake, PSRAM wait — as six separate `if`s
rather than a priority chain, and requires exactly one to be true: `reasons != 1` (whether zero
or more than one) exits 7 with the cycle and every reason's value printed, so a reason that goes
tied-low or two reasons that overlap are both caught directly rather than only showing up as a
wrong total later. The `BUCKETS ... window_cycles=N` line it prints then lets
`nano/bench/qspi_timing_report.py` re-derive and re-grade the same sum-equals-total identity from
a log alone, with no rebuild, and only cycles inside a benchmark's own marked region are counted
so a config comparison is not diluted by setup cycles outside it. Both the per-cycle
exactly-one-reason check and the printed-log identity are forced red in `test/probe_gates.sh`: a
tied-low reason, an overlapping pair of reasons, and a hand-crafted log whose buckets do not sum
to their stated total are each required to report the specific failure (`test/PROBES_EXPECTED`
carries all three labels).

## Measured

`make nano-qspi-timing` (`nano/bench/run_qspi_timing.sh`) builds one `nano-qspi-sim` per
configuration and runs Dhrystone (200 runs) and CoreMark (5 iterations) against each — the same
sources and flags as `make nano-dhrystone`/`make nano-coremark`. The zero-wait control reproduces
ADR-0182's own figures to the cycle in every run: 505,295 cycles / 2,526.5 cycles per run / 0.225
DMIPS/MHz; 9,242,400 cycles / 1,848,480.0 cycles/iteration / 0.541 CoreMark/MHz — the default
`nano-sim` build path is untouched, and the sweep script now exits nonzero if that control ever
drifts from these figures rather than only reporting them.

**16-parcel windows are excluded from this sweep.** Both loop-buffer kinds at `LOOP_WINDOW=16`
show a real, unexplained CoreMark pathology — `fifo2-tagged16` times out at 15,000,000 cycles
without reaching CoreMark's first marker, and `fifo2-cam16`, re-checked in isolation, times out
the same way with `parcel_wait` consuming 13,637,776 of 14,555,105 windowed cycles. This
reproduced on a clean, uncontended run, so it is not a diagnostic-process artifact. It was not
root-caused under this ticket's time budget; see "Open question" below rather than reading its
absence from the table as "16 parcels measured worse."

| Configuration | Dhry cycles/run | DMIPS/MHz | DMIPS@64MHz | CoreMark cycles/iter | CoreMark/MHz | CoreMark@64MHz | execute% | parcel wait% | redirect preamble% | loop hit% | handshake% | PSRAM wait% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| no-overlap (depth 0) | 21,089.7 | 0.0270 | 1.73 | 10,678,943.6 | 0.0936 | 5.99 | 8.84 / 13.84 | 24.72 / 30.60 | 27.35 / 22.69 | 0.00 / 0.00 | 2.23 / 2.76 | 36.85 / 30.10 |
| FIFO depth 2 | 20,300.5 | 0.0280 | 1.79 | 9,917,092.4 | 0.1008 | 6.45 | 9.18 / 14.91 | 21.80 / 25.27 | 28.41 / 24.44 | 0.00 / 0.00 | 2.32 / 2.97 | 38.28 / 32.42 |
| FIFO depth 4 | 20,261.5 | 0.0281 | 1.80 | 9,784,145.2 | 0.1022 | 6.54 | 9.20 / 15.11 | 21.65 / 24.25 | 28.47 / 24.77 | 0.00 / 0.00 | 2.33 / 3.01 | 38.36 / 32.86 |
| FIFO 2 + tagged-block loop 8 | 17,010.5 | 0.0335 | 2.14 | 8,265,043.8 | 0.1210 | 7.74 | 10.96 / 17.89 | 16.78 / 22.42 | 23.80 / 17.24 | 0.95 / 0.85 | 1.82 / 2.72 | 45.69 / 38.90 |
| FIFO 2 + CAM loop 8 | 16,939.5 | 0.0336 | 2.15 | 8,528,141.4 | 0.1173 | 7.50 | 11.01 / 17.33 | 17.65 / 23.20 | 22.68 / 18.31 | 0.89 / 0.69 | 1.90 / 2.76 | 45.88 / 37.70 |
| FIFO 2 + CAM loop 8 + QPI (20-cycle preamble) | 16,219.3 | 0.0351 | 2.25 | 8,238,715.0 | 0.1214 | 7.77 | 11.49 / 17.94 | 18.43 / 24.02 | 19.25 / 15.44 | 0.92 / 0.72 | 1.98 / 2.86 | 47.92 / 39.02 |

(percentage columns: Dhrystone / CoreMark; every row's six percentages, and every row's retired
cycle count against its own accounting total, satisfy the exactly-one-reason and
sum-equals-total identities above — both graded by `nano_cxxrtl.cc` and re-derived by
`qspi_timing_report.py`, per run.)

Both benchmarks pass their own validity self-check (`verdict=1`) in every configuration.
`DMIPS@64MHz`/`CoreMark@64MHz` assume the brief's own target clock, stated so — no clock is
placed for nano yet. The sweep auto-selects the QPI variant on top of whichever configuration
reads fewest Dhrystone cycles; that was `fifo2-cam8` here, by 48,400 cycles over
`fifo2-tagged8` (0.28%), so `fifo2-cam8-qpi` rather than a tagged-block QPI row is what's shown.

**Instructions retired and effective MIPS at 64 MHz**, the two benchmarks' own retire counts
(105,767 for Dhrystone's 200 runs, 1,488,936 for CoreMark's 5 iterations) against each
configuration's total simulated cycles:

| Configuration | Dhrystone MIPS@64MHz | CoreMark MIPS@64MHz |
|---|---|---|
| no-overlap | 1.60 | 1.78 |
| FIFO depth 2 | 1.67 | 1.92 |
| FIFO depth 4 | 1.67 | 1.95 |
| FIFO 2 + tagged-block loop 8 | 1.99 | 2.31 |
| FIFO 2 + CAM loop 8 | 2.00 | 2.23 |
| FIFO 2 + CAM loop 8 + QPI | 2.09 | 2.31 |

The brief's ~2 MIPS estimate sits *above* the measured no-overlap figure (1.60–1.78) and is
reached only once a loop buffer is added — Dhrystone's `redirect_preamble` bucket alone is
~27% of the no-overlap run, a branch-redirect density the brief's back-of-envelope arithmetic
("~11 clk average fetch") did not carry.

## What this says about the front end's design

**FIFO depth alone is nearly free and nearly useless**, unchanged from the first draft's finding:
depth 2 → 4 moves DMIPS/MHz 0.0280 → 0.0281 (+0.4%, Dhrystone) and 0.1008 → 0.1022 (+1.4%,
CoreMark) — both inside the range a reader should read as "does not change the answer,"
confirming the brief's own fetch-bandwidth argument: with no loop buffer, the core is
fetch-bound against the flash's raw streaming rate, and a deeper queue in front of a bandwidth
ceiling does not raise the ceiling.

**The loop buffer is still the lever, but the corrected gain is smaller than the first draft
reported, and it is now backed by tests rather than table-reading alone.** From FIFO depth 2 to
an 8-parcel loop buffer: tagged-block reaches 0.0335 DMIPS/MHz (+19.6%) and 0.1210 CoreMark/MHz
(+20.0%); CAM reaches 0.0336 DMIPS/MHz (+20.0%) and 0.1173 CoreMark/MHz (+16.4%). Both shapes
of the same 8-parcel area budget land within a few percent of each other, in opposite order on
the two benchmarks: CAM edges ahead on Dhrystone (+0.3%), tagged-block ahead on CoreMark
(+3.2%) — neither shape dominates the other, which the two-tests-per-shape grading above exists
precisely to keep honest as either implementation changes. QPI's shorter re-entry preamble
(24 → 20 cycles) is a further, smaller, and independent gain layered on top of the
fewest-Dhrystone-cycles configuration: +4.5% DMIPS/MHz, +3.5% CoreMark/MHz over `fifo2-cam8`.

**Which shape and which window size is worth building, at what area, remains a judgement this
measurement informs rather than settles.** At 8 parcels the two loop-buffer shapes are close
enough (within ~3% of each other on either benchmark) that the choice between "one tag, one
aligned block" and "a fully-associative last-N-parcels record" is more about implementation
simplicity and the exact area the brief's 4.2k µm² figure assumes than about measured
performance. Neither figure changes nano's fundamental character: even the best configuration
measured here (2.25 DMIPS, 7.77 CoreMark at 64 MHz) remains a fetch-bound core against a
132 Mbit/s bus, not a design a wider buffer alone turns into something else — buffering only
hides latency for code that revisits addresses already in the window, and neither benchmark is
dominated by loops that small.

## Open question: 16-parcel windows (DECISION NEEDED)

Both loop-buffer shapes show a real CoreMark-specific slowdown or hang at `LOOP_WINDOW=16` that
was not root-caused under this ticket. The same-block-miss deadlock fixed above was the first,
more severe version of this shape of bug and is fixed for both window sizes; what remains at
window 16 is narrower — Dhrystone was not observed to reproduce it, only CoreMark — but is
unexplained rather than merely unmeasured, so it is left out of the shipped sweep instead of
reported as a (possibly wrong) number. Two ways to close this out, neither attempted here for
lack of time against the ticket's "correctness over speed" priority: instrument the same
bucket/address tracing this ADR's earlier bug hunt used, on a CoreMark run specifically (its
larger working set and different branch mix are the one input Dhrystone's clean run at window 16
does not share); or narrow `LOOP_WINDOW=16`'s state space directly against the invariant tests
above with a wider `REPS` sweep, since the existing two tests only exercise window 8.

## Scope

No change to `nano.v`. `nano-qspi-timing` is off `make test`'s path, reporting only, with no
ratchet — the same standing as `make cycles`, `make nano-dhrystone` and `make nano-coremark`.
`nano-qspi-loop-test` and its forced-red prerequisite `nano-qspi-loop-probe` are on `make test`'s
path and graded. The accounting-identity and exactly-one-reason checks are graded and probed; the
timing model's own comparative numbers are not, since there is nothing yet to ratchet them
against.
