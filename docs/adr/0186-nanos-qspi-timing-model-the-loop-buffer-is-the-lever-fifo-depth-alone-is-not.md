# ADR-0186: nano's QSPI timing model — the loop buffer is the lever, FIFO depth alone is not

**Status:** Accepted · 2026-09-14

## Context

nano's own benchmark figures (ADR-0182) — 0.225 DMIPS/MHz, 0.541 CoreMark/MHz — are measured
against `nano/tb/nano_memory.v`, a zero-wait, behavioural flat memory. The brief
(`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md`) estimates real QSPI-flash-plus-PSRAM
fetch latency puts nano at roughly 2 MIPS, and names a loop buffer as "the only thing that
changes the answer" but defers it for area. This ticket measures that, without touching
`nano.v`.

**This ADR supersedes its own two earlier drafts.** A security review of the first draft found
two correctness bugs in the loop buffer, and a red-team read showed its headline gain was largely
an artifact of them. A review of the second draft found a third bug that both drafts shared: a
loop-buffer hit moved the flash queue's head (defined below). That third bug is what hung
CoreMark at 16-parcel windows, and it also moved the 8-parcel rows the second draft published —
the tagged block by 4.2% of Dhrystone's cycles. All three are fixed, each has a test that is
forced red, and every number below is re-measured on the fixed model.

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
commands rather than one shared number: a load holds the bus `PSRAM_LOAD_CYCLES` (44, Fast Read
`0Bh` with its 4-cycle dummy phase) clocks beyond the one-cycle handshake every access pays, a
store `PSRAM_STORE_CYCLES` (33, `02h`, no dummy phase). Either
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
  wherever the flash's background stream happened to be (the first draft tried that; see below).
- **`LOOP_KIND=2`, last-N-parcels CAM**: a fully-associative record of the last `LOOP_WINDOW`
  parcels the core was actually delivered, in delivery order, with no alignment requirement.
  This is the architect's "last N parcels fetched" reading of the same area budget, priced
  identically per parcel.

Both are checked on **every** fetch, sequential ones included, and both answer in the cycle the
hit is recognized: an on-chip SRAM or CAM read, modelled the way an already-streamed parcel also
resolves same-cycle. A hit is charged to its own loop-hit bucket. A fetch that misses the loop
buffer and is not where the flash is aimed is a redirect, and the flash re-aims at it.

**The flash queue's head is tracked apart from the core's position.** `fifo_head` is the next
parcel the flash's queue will hand the core; `expect_index` is the core's own next sequential
parcel, which every delivery moves. A fetch the loop buffer does not serve is taken from the
flash only at `fifo_head`, and only once the flash has streamed its parcels in the current run,
which starts at the redirect target (`preamble_target`); `fifo_head` never falls below it. A hit
moves the core, not the queue: `fifo_head` advances only when the delivered instruction starts at
it. So the parcels a loop's exit falls through to, streamed before the loop began to hit, are
still queued when it exits. A hit at `fifo_head` before the flash has reached that parcel moves
the head past the stream, which streams the parcel anyway and drops it, because the flash cannot
skip ahead without a new address phase.

**Every transaction ends in bounded time.** A hit answers at once. A redirect's preamble counts
down unconditionally and the flash then streams from `fifo_head`. A fetch waiting at `fifo_head`
waits on a stream that keeps going while its lead over `fifo_head` (`produce_lead`, signed) is
below `PREFETCH_DEPTH`, which holds until the fetch's last parcel arrives. That is why
`PREFETCH_DEPTH` must be 0 or at least 2: at 1, a 32-bit instruction's second parcel could never
be queued, and the model refuses that value at elaboration. A PSRAM access counts down
unconditionally.

**One simplification is pessimistic.** A 32-bit instruction that straddles the tagged block's
edge is never a hit, and it is fetched whole through a redirect even when its first parcel is in
the block — about 8 clocks more than a controller that fetched only the missing parcel.

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

## Three bugs in the model, none in `nano.v`

**A loop-buffer hit answered a cycle late.** `mem_ready`'s loop-hit arm read
`loop_hit_now ? xfer_active : ...`, and `xfer_active` is only true starting a transaction's
*second* cycle — so every hit paid a minimum of 2 clocks instead of 1, silently understating the
FIFO's own advantage over the loop buffer on the very numbers meant to show the loop buffer's
gain. Fixed to `loop_hit_now ? 1'b1 : ...`: a hit is an on-chip lookup and answers the cycle it
is recognized, same as an already-arrived streaming parcel.

**A same-block miss resumed a partial fill.** The first draft, on a `LOOP_KIND=1` miss inside
the block already tagged, resumed the flash from wherever it had reached rather than restarting
the block. It was blamed for a Dhrystone hang at `LOOP_WINDOW=16` in which `parcel_wait` took
3,758,938 of 3,766,316 windowed cycles. That signature is also the third bug's, and the first
draft's code is not re-run here, so which of the two hung that run is not established. The
resume logic stays removed: every miss re-tags and restarts the block, the simpler rule.

**A loop-buffer hit moved the flash queue's head.** Both earlier drafts used one register,
`expect_index`, for two things: the core's next sequential parcel, which every delivery moves,
and the head of the flash's queue, which only the flash's own stream should move. A hit moved it
without moving the stream, and that went wrong three ways:

- **Backward, a phantom fetch.** After a hit to an earlier parcel, a later fetch the buffer did
  not hold matched `expect_index`, and the stream's last-arrived parcel already lay beyond it, so
  the model served parcels the flash had never streamed in its current run. The second draft's
  8-parcel tagged block did this on 2,579 Dhrystone fetches and 10,158 CoreMark fetches.
- **Forward, a deadlock.** After a hit to a parcel beyond the stream, the stream's room test — an
  unsigned difference — wrapped, the stream stopped for good, and the next fetch the buffer did
  not hold waited forever. This is the 16-parcel CoreMark hang. Both shapes stop in the same
  state: the fetch is aimed, the stream is behind it, and the stream cannot advance — the CAM at
  parcel 1,372 against a next-to-stream parcel of 1,369, the tagged block at 912 against 907. It
  is not specific to 16 parcels; a larger buffer only makes a hit past the stream likelier.
- **A clobber.** The preamble's completion rewrote `expect_index` to where the flash had been
  aimed, undoing any hits taken during the resync, so the next fetch looked unaimed and paid a
  redirect it did not owe: 7,697 times in the 8-parcel tagged block's Dhrystone run.

The phantom and the clobber push in opposite directions. Fixing all three moves the 8-parcel
tagged block from 17,010.5 to 16,290.5 Dhrystone cycles per run (−4.2%) and from 8,265,043.8 to
8,208,735.2 CoreMark cycles per iteration (−0.7%); the 8-parcel CAM moves +0.006% and +0.022%.
The three configurations without a loop buffer are cycle-identical before and after, because
there every delivery comes from the stream and the two registers never disagree.

## The loop buffer's three invariants are graded, not just measured

`make nano-qspi-loop-test` (`nano/bench/run_qspi_loop_buffer_test.sh`) checks three properties,
each built to be able to fail:

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
3. **Every fetch the loop buffer does not serve comes from parcels its flash run streamed.**
   `nano_qspi_memory.v` publishes `stream_fault`, computed from the run's first parcel and its
   last-arrived parcel rather than from the aim logic, and every `nano-qspi-sim` exits 7 on the
   first cycle it is set — on these micro-programs and on every benchmark run of the sweep below.
   `qspi_loop_micro.S`'s `KIND=2` body is a loop with a load whose 32-bit `addi` straddles the
   16-byte block's end; it must run to PASS at both `REPS` in both shapes.

`nano/bench/run_qspi_loop_buffer_probe.sh` is the forced-red prerequisite
(`nano-qspi-loop-probe`, mirroring `nano_exec_probe.sh`'s pattern): it builds four mutated
scratch copies of `nano_qspi_memory.v` directly via yosys/clang++, bypassing `make` so the probe
never touches the checked-out tree, and requires each to fail for its own stated reason.
Reintroducing the `xfer_active`-gated hit must fail invariant 2's cycle bound. Disabling the
CAM's lookup must fail invariant 2's "no marginal preamble/wait" check. Letting the queue head
follow every hit — the phantom shape of the third bug — must trip invariant 3's exit 7 on the
tagged block's straddling loop. An unsigned `produce_lead` — the deadlock shape — must time out
on the CAM's. All three tests and all four mutations are on `make test`'s path.

**Two costs re-derived by hand**, as marginal cycles per iteration between 200 and 400
iterations. A two-instruction loop resident in either shape costs 10: 8 execute cycles plus two
one-cycle hits, exactly its zero-wait cost. The straddling loop is resident in the 8-parcel CAM
and costs 75 there: its zero-wait 31 plus 44 for the load, with no preamble or parcel wait. In
the 8-parcel tagged block it costs 152.5, against 157 with FIFO 2 alone, because the block
alternates between re-tagging at the straddler and missing at the loop top — 1.5 hits per
iteration, as measured.

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

| Configuration | Dhry cycles/run | DMIPS/MHz | DMIPS@64MHz | CoreMark cycles/iter | CoreMark/MHz | CoreMark@64MHz | execute% | parcel wait% | redirect preamble% | loop hit% | handshake% | PSRAM wait% |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| no-overlap (depth 0) | 21,089.7 | 0.0270 | 1.73 | 10,678,943.6 | 0.0936 | 5.99 | 8.84 / 13.84 | 24.72 / 30.60 | 27.35 / 22.69 | 0.00 / 0.00 | 2.23 / 2.76 | 36.85 / 30.10 |
| FIFO depth 2 | 20,300.5 | 0.0280 | 1.79 | 9,917,092.4 | 0.1008 | 6.45 | 9.18 / 14.91 | 21.80 / 25.27 | 28.41 / 24.44 | 0.00 / 0.00 | 2.32 / 2.97 | 38.28 / 32.42 |
| FIFO depth 4 | 20,261.5 | 0.0281 | 1.80 | 9,784,145.2 | 0.1022 | 6.54 | 9.20 / 15.11 | 21.65 / 24.25 | 28.47 / 24.77 | 0.00 / 0.00 | 2.33 / 3.01 | 38.36 / 32.86 |
| FIFO 2 + tagged-block loop 8 | 16,290.5 | 0.0349 | 2.24 | 8,208,735.2 | 0.1218 | 7.80 | 11.44 / 18.01 | 16.36 / 22.73 | 21.59 / 16.51 | 1.21 / 0.88 | 1.68 / 2.71 | 47.71 / 39.16 |
| FIFO 2 + CAM loop 8 | 16,940.5 | 0.0336 | 2.15 | 8,530,037.4 | 0.1172 | 7.50 | 11.01 / 17.33 | 17.70 / 23.27 | 22.63 / 18.26 | 0.89 / 0.69 | 1.90 / 2.76 | 45.88 / 37.69 |
| FIFO 2 + tagged-block loop 16 | 15,387.5 | 0.0370 | 2.37 | 8,117,862.2 | 0.1232 | 7.88 | 12.12 / 18.21 | 15.55 / 22.52 | 18.76 / 16.04 | 1.48 / 0.96 | 1.58 / 2.67 | 50.51 / 39.60 |
| FIFO 2 + CAM loop 16 | 14,645.5 | 0.0389 | 2.49 | 8,046,371.6 | 0.1243 | 7.95 | 12.73 / 18.37 | 13.07 / 21.75 | 17.91 / 16.27 | 1.80 / 1.09 | 1.41 / 2.57 | 53.07 / 39.95 |
| FIFO 2 + CAM loop 16 + QPI (20-cycle preamble) | 14,149.3 | 0.0402 | 2.57 | 7,801,608.4 | 0.1282 | 8.20 | 13.18 / 18.95 | 13.53 / 22.41 | 15.03 / 13.66 | 1.87 / 1.12 | 1.46 / 2.65 | 54.93 / 41.21 |

(percentage columns: Dhrystone / CoreMark; every row's six percentages, and every row's retired
cycle count against its own accounting total, satisfy the exactly-one-reason and
sum-equals-total identities above — both graded by `nano_cxxrtl.cc` and re-derived by
`qspi_timing_report.py`, per run.)

Both benchmarks pass their own validity self-check (`verdict=1`) in every configuration.
`DMIPS@64MHz`/`CoreMark@64MHz` assume the brief's own target clock, stated so — no clock is
placed for nano yet. The sweep auto-selects the QPI variant on top of whichever configuration
reads fewest Dhrystone cycles; that was `fifo2-cam16` here, by 148,400 cycles over
`fifo2-tagged16` (4.8% of the tagged block's), so `fifo2-cam16-qpi` is the QPI row shown.

**Instructions retired and effective MIPS at 64 MHz**, the two benchmarks' own retire counts
(105,767 for Dhrystone's 200 runs, 1,488,936 for CoreMark's 5 iterations) against each
configuration's measured-region cycles. The retire counts cover the whole run, setup included,
so each figure slightly overstates the region's own rate:

| Configuration | Dhrystone MIPS@64MHz | CoreMark MIPS@64MHz |
|---|---|---|
| no-overlap | 1.60 | 1.78 |
| FIFO depth 2 | 1.67 | 1.92 |
| FIFO depth 4 | 1.67 | 1.95 |
| FIFO 2 + tagged-block loop 8 | 2.08 | 2.32 |
| FIFO 2 + CAM loop 8 | 2.00 | 2.23 |
| FIFO 2 + tagged-block loop 16 | 2.20 | 2.35 |
| FIFO 2 + CAM loop 16 | 2.31 | 2.37 |
| FIFO 2 + CAM loop 16 + QPI | 2.39 | 2.44 |

The brief's ~2 MIPS estimate sits *above* the measured no-overlap figure (1.60–1.78) and is
reached only once a loop buffer is added — Dhrystone's `redirect_preamble` bucket alone is
~27% of the no-overlap run, a branch-redirect density the brief's back-of-envelope arithmetic
("~11 clk average fetch") did not carry.

## What this says about the front end's design

**FIFO depth alone is nearly free and nearly useless**, unchanged from the first draft's finding:
depth 2 → 4 moves DMIPS/MHz 0.0280 → 0.0281 (+0.2%, Dhrystone) and 0.1008 → 0.1022 (+1.4%,
CoreMark) — both inside the range a reader should read as "does not change the answer,"
confirming the brief's own fetch-bandwidth argument: with no loop buffer, the core is
fetch-bound against the flash's raw streaming rate, and a deeper queue in front of a bandwidth
ceiling does not raise the ceiling.

**The loop buffer is the lever.** Against FIFO depth 2 alone, as a percentage of FIFO 2's
DMIPS/MHz and CoreMark/MHz: the 8-parcel tagged block gains 24.6% and 20.8%, the 8-parcel CAM
19.8% and 16.3%, the 16-parcel tagged block 31.9% and 22.2%, and the 16-parcel CAM 38.6% and
23.2%.

**Which shape leads flips with the window's size, and on both benchmarks at once.** At 8 parcels
the tagged block leads: the CAM takes 4.0% more Dhrystone cycles and 3.9% more CoreMark cycles.
At 16 the CAM leads: the tagged block takes 5.1% more Dhrystone cycles and 0.9% more CoreMark
cycles. Doubling the window from 8 to 16 parcels cuts the CAM's Dhrystone cycles by 13.5% and
its CoreMark cycles by 5.7%, and the tagged block's by 5.5% and 1.1%. The measurement does not
say why the order flips. One candidate, unmeasured: a record of the last N parcels delivered
evicts a loop's top before reaching it again whenever the body is longer than N, while an aligned
block keeps whatever part of a longer loop falls inside it. QPI's shorter re-open preamble
(24 → 20 cycles) is a further, independent gain on the fewest-Dhrystone-cycles configuration:
+3.5% DMIPS/MHz and +3.1% CoreMark/MHz over `fifo2-cam16`.

**Which shape and which window size is worth building, at what area, remains a judgement this
measurement informs rather than settles.** The shapes sit within about 5% of each other at either
size, so the choice between "one tag, one aligned block" and "a fully-associative
last-N-parcels record" turns on area and implementation more than on these cycles; a CAM's
per-slot comparators cost more area than the brief's one shared tag, and 16 parcels double the
brief's 128 flops. Neither figure changes nano's fundamental character: even the best
configuration measured here (2.57 DMIPS, 8.20 CoreMark at 64 MHz) remains a fetch-bound core
against a 132 Mbit/s bus, not a design a wider buffer alone turns into something else —
buffering only hides latency for code that revisits addresses already in the window, and neither
benchmark is dominated by loops that small.

## Scope

No change to `nano.v`. `nano-qspi-timing` is off `make test`'s path, reporting only, with no
ratchet — the same standing as `make cycles`, `make nano-dhrystone` and `make nano-coremark`.
`nano-qspi-loop-test` and its forced-red prerequisite `nano-qspi-loop-probe` are on `make test`'s
path and graded. The accounting-identity, exactly-one-reason and stream-fault checks are graded
and probed, and all three run on every cycle of every benchmark run in the sweep; the timing
model's own comparative numbers are not, since there is nothing yet to ratchet them against.
