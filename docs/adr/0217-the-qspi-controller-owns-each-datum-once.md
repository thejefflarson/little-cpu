# ADR-0217: the QSPI controller owns each datum once

**Status:** Accepted · 2026-09-27

## Context

Tier 0 (ADR-0213) clock-gated the flow's enabled flip-flops and got the full chip placing
and globally routing on a 4×2 Tiny Tapeout tile, but met1/met2 sit at 94-98% demand. Tier 2
of the brief (`docs/ideas/nano-on-a-4x2-the-tile-is-wiring.md`) is `nano/qspi.v` itself:
about 190 of its flip-flops hold a copy of a bus `nano.v` already holds stable until
`mem_ready`, matching this repo's own finding that "about 400 flip-flops hold a copy of a
value something else already holds."

## What was removed, and what stayed

**The prefetch slot.** With one slot and no look-ahead (ADR-0210), the slot only ever held
the parcel for the request already pending, and consuming it cost a one-cycle bounce through
`ST_IDLE`. A completing parcel now retires directly out of `rx_shift` in the same cycle
`ST_FLASH_STREAM` finishes it -- the request it completes is, by construction, the one
`nano.v` is still holding on the bus, since nothing enters `ST_FLASH_STREAM` except in
answer to an outstanding request. `slot0_valid`/`slot0_data`/`slot0_addr` and the
`fetch_hit0`/`fetch_needs_second_parcel`/`fetch_hit_ready`/`retire_one` wires they fed are
gone; a four-byte instruction's second parcel is decided and streamed without leaving
`ST_FLASH_STREAM` at all, instead of bouncing through `ST_IDLE` twice.

**The PSRAM pending registers.** `psram_addr_pending`, `psram_wdata_pending` and
`psram_wstrb_pending` copied `mem_addr`/`mem_wdata`/`mem_wstrb`, which `nano.v` holds
unchanged for the whole transaction; the controller now reads them live wherever they were
read from the copies. `psram_is_write`/`psram_rmw_pending` collapse to one bit,
`psram_write_phase`, since a partial store's read-modify-write is the only case where "is
this a write" needs to survive past what the live `mem_wstrb` says (`mem_wstrb == 4'b1111`
otherwise answers it directly). The read-modify-write's *merged* word has no bus copy to
read back -- `nano.v`'s own `mem_wdata` is still the original partial store -- so it keeps
its own register, renamed `psram_rmw_result` to say what it now holds.

**`mem_rdata`.** No longer its own register: `assign mem_rdata = fetch_two_parcels ?
{rx_shift[15:0], rx_shift[31:16]} : rx_shift`. A plain fetch's or a PSRAM read's result is
already sitting in `rx_shift` the cycle `mem_ready` reads high, because both the shift that
assembles it and the `mem_ready`-setting branch fire from the same non-blocking assignments
on the same edge; `nano.v` masks a compressed parcel's upper half itself
(`mem_rdata[1:0] == 2'b11 ? mem_rdata : {16'b0, mem_rdata[15:0]}`), so an unmasked upper half
costs nothing when it is garbage. A two-parcel fetch needs the swap because the second
parcel (which belongs in the *upper* half of the result) is the one still shifting into
`rx_shift[15:0]` when it completes; the swap is wire crossing, not new logic.

**`second_parcel_first_data` folds into `rx_shift[31:16]`.** Those bits idle during a flash
stream except right here: the first parcel of a two-parcel fetch is stashed there the cycle
the second parcel starts, and stays untouched while the second one shifts into
`rx_shift[15:0]`.

**`tx_shift` stays a 40-bit shift register.** A nibble mux over the differently-packed
command fields (8+23+1+8 bits for a cold flash open, 8+23+1 for PSRAM, 32 for a write, with
the address's own 3-bit remainder never landing on a nibble boundary) is real added muxing
and verification surface for one register this cut's own measurement (below) shows is a
small share of the total; the ticket's own fallback applies.

## Invariant 3, restated

The prefetch slot is gone, so "the slot holds exactly the parcel behind `stream_next_addr`"
no longer parses. What it protected -- that the controller cannot lose or double-count a
parcel -- is restated in terms of what still exists: **on `mem_ready && mem_instr`,
`stream_next_addr - req_parcel_addr` equals the number of parcels the completing fetch just
delivered.** `fetch_two_parcels`, a new one-bit register set the same cycle as `mem_ready`,
is what makes "the number of parcels just delivered" a value the assertion can read after
the edge that decided it — without it, both the one-parcel and two-parcel completions read
identically (`second_parcel_pending` has already dropped back to 0 either way).

Removing the registered copies means the fetch path now reads `mem_addr`/`mem_wdata`/
`mem_wstrb`/`mem_instr` live at more than one point across a multi-cycle transaction, where
the earlier design captured them once and never looked again. `nano/formal/qspi.sby` has no
assumption at all about those inputs, so k-induction is free to move them between cycles of
the same outstanding request -- and did, immediately, failing invariant 3 with `mem_addr`
changing on every step of the counterexample. A standing assumption now states the real
contract (`nano.v` holds `mem_valid`/`mem_addr`/`mem_wdata`/`mem_wstrb`/`mem_instr` fixed
from one cycle to the next whenever `mem_valid` was already high and `mem_ready` was not):
this is not a new fact about the system, only the first time this proof needed to state it,
since the earlier design's registered copies made it true by construction and never had to
say so. `nano/formal/qspi-probe.py`'s `queue-addressing` mutation is re-anchored on
`fetch_two_parcels`'s own assignment (claiming a two-parcel completion delivered one parcel);
`nano/tb/nano_qspi_resume_probe.sh` and `nano/tb/nano_qspi_pins_probe.sh` are re-anchored on
the new second-parcel and read-modify-write text. All three invariants re-prove by k-induction
(`abc pdr`) in about the same time as before.

## Area: measured per copy, then combined

`make nano-area`, baseline re-taken on this tree (matches main ae27d5b's own 70,873.0 exactly):

| variant | um2 | delta |
| --- | --- | --- |
| baseline | 70,873.0 | -- |
| + PSRAM pending registers removed, alone | 70,851.7 | -21.3 (-0.03%) |
| + prefetch slot / `mem_rdata` / `second_parcel_first_data` removed, bundled | 68,819.8 | -2,053.2 (-2.90%) |
| combined | 67,408.4 | -3,464.6 (-4.89%) |

The fetch-path cut is reported as one bundle rather than three separate rows: `mem_rdata`
becoming a wire and `second_parcel_first_data` folding into `rx_shift` both only parse once
the prefetch slot's removal has already put completion inside `ST_FLASH_STREAM` itself, so
an isolated "`mem_rdata` alone, slot still present" variant would not be the change this
ticket makes, only a smaller one nobody is shipping. The PSRAM cut saves far less than a
bit count predicts (about 100 bits of registers removed for 21 um2), which matches this
repo's own standing note that a removed register's input mux is usually replaced by an
equivalent live-read mux, not deleted outright. The combined saving (-3,464.6) exceeds the
two bundles' sum (-2,074.5) by 1,390.1 um2 -- yosys shares logic across the two edits that
it could not share when either shipped alone, the same pattern ADR-0210 measured.

The table above was measured before Tier 1 (`x0`/`mem_wdata`/`mem_wstrb`/`rd`/`rs1`/`rs2`
losing their own copies, ADR-0216) landed and before the local instrument mirrored the
flow's clock gating; see "Rebased onto Tier 1", below, for the combined, re-taken number
`NANO_MAX_UM2` now carries.

## Cycles: measured before and after, both harnesses

Zero-wait (`nano-sim`, `nano/tb/nano_memory.v`) does not build `nano/qspi.v` at all, so it is
an unchanged-by-construction control, and it reproduces exactly:

| | Dhrystone cycles | Dhrystone retires | CoreMark cycles | CoreMark retires |
| --- | --- | --- | --- | --- |
| `make nano-dhrystone` / `make nano-coremark` | 629,527 | 105,987 | 24,943,488 | 3,810,704 |

Pin-level (`NANO_DHRY_RUNS=5 NANO_DHRY_CYCLES=100000000 make nano-qspi-pins-dhrystone`,
`NANO_COREMARK_ITERATIONS=1 NANO_COREMARK_CYCLES=40000000 make nano-qspi-pins-coremark`),
baseline re-taken on this tree and matching the ticket's own figures exactly:

| | Dhrystone cycles | Dhrystone retires | CoreMark cycles | CoreMark retires |
| --- | --- | --- | --- | --- |
| before | 127,035 | 13,944 | 25,276,725 | 777,270 |
| after | 124,672 | 13,944 | 24,518,338 | 777,270 |
| delta | -2,363 (-1.86%) | 0 | -758,387 (-3.00%) | 0 |

Retire counts are unchanged on both benchmarks: this is the same programs running fewer
cycles, not different programs. The saving is real but smaller than "about one cycle per
fetch" over Dhrystone's whole retire count would predict (-2,363 over 13,944 fetches, about
0.17 cycles/fetch); CoreMark reads much closer to that estimate (-758,387 over 777,270
fetches, about 0.98 cycles/fetch). The two benchmarks' instruction mix (compressed-instruction
and redirect density) evidently determines how often the removed one-cycle `ST_IDLE` bounce
was actually on a fetch's critical path; both numbers are measured, not adjusted to match
the brief's estimate, which the CoreMark row happens to confirm and the Dhrystone row does
not contradict, only undershoots. `nano-qspi-pins-test` passes both simulator legs, program
by program, with retire counts unchanged from before this ticket (`alu.S` 66, `branch.S` 33,
`compressed.S` 75, `csrimm.S` 19, `divide.S` 52, `loadstore.S` 44, `meip.S` 42, `mul.S` 52).
`nano/bench/run_qspi_loop_buffer_test.sh`'s `WINDOW_BOUND` is untouched and still passes: it
builds against `nano/tb/nano_qspi_memory.v`, the abstract bus-timing model, which has no
reference to `nano/qspi.v` at all and so cannot move with this change.

## Rebased onto Tier 1, and the bus-stability contract proved rather than assumed

Tier 1 (ADR-0216) landed first: `x0` goes unstored, `mem_wdata`/`mem_wstrb` become wires
read live off `nano.v`'s own registers instead of copies, `rd`/`rs1`/`rs2` read `instr`
live, and the local instrument (`nano/synth_script.sh`) gained the flow's own `clockgate`
pass. Rebasing this tier onto that tree is a clean fast-forward -- neither tier's own files
overlap -- but two things it left undone become owed once both tiers share a tree.

**Area, re-measured under the clock-gated instrument, both halves on their own first.**
`make nano-area`, this tree, this toolchain:

| variant | um2 |
| --- | --- |
| Tier 1 alone (rebased, clock-gated instrument) | 60,626.9 |
| Tier 1 + this ticket's cuts (combined) | 58,790.1 |
| delta | -1,836.8 (-3.03%) |

The 60,626.9 figure is this session's own re-measurement of Tier 1 alone, not the
60,800.8 `nano.mk`'s prior comment quoted -- the ~0.3% gap is toolchain drift of the kind
this repo's own measurement notes already document (`make nano-area` is quoted with the
tree and treated as a local sanity check, never merged across sessions without
re-confirming). `NANO_MAX_UM2` steps 62,300 -> 60,300, the same ~1,500 um2 headroom style
the prior step used, over this ticket's own fresh 58,790.1 measurement.

**The bus-stability assumption is now a proof.** Invariant 3's restatement (above) needed
`nano/formal/qspi.sby` to assume `nano.v` holds a request's `mem_valid`/`mem_addr`/
`mem_wdata`/`mem_wstrb`/`mem_instr` stable from the cycle it is raised until `mem_ready`.
That assumption was trivially true before Tier 1: `mem_wdata` and `mem_wstrb` were plain
registers, written once per transaction and read back unchanged. Tier 1 made both wires,
continuously read off `op_rs2`/`instr`/`cpu_state`/`store_wstrb` -- still stable in every
reachable execution, since nothing rewrites those registers between an instruction issuing
its one bus transaction and that transaction's own `mem_ready`, but no longer stable *by
construction*, so the claim needs its own proof rather than inheriting one from qspi.v's
assumption of it.

`nano/nano.v` gains its own `` `ifdef FORMAL `` block -- the module that owns the state, so
no hierarchical reference is needed -- asserting exactly the property qspi.v assumes, one
cycle at a time: `mem_valid_q`/`mem_ready_q`/`mem_addr_q`/`mem_wdata_q`/`mem_wstrb_q` shadow
the previous cycle's ports, and whenever a request was outstanding and unanswered last
cycle, this cycle's ports must match. `nano/formal/memreq.sby` proves it by k-induction
(`abc pdr`, `mode prove`), converging in 3 frames, about 0.65s. The first two attempts read
FAIL at step 1 and at frame 0 respectively: `clocked_q`, a one-cycle-delayed copy of the
standing `clocked` idiom, was needed to keep the reset cycle's own unconstrained
`mem_valid`/`mem_ready` -- neither has an `initial` value, so before `clocked_q` gated the
antecedent, PDR was free to pick a free `mem_valid_q`/`mem_ready_q` pair at the very first
frame and "prove" a violation against a request that was never really outstanding.
`nano/formal/memreq-probe.py` is the forced-red prerequisite: a mutated `finish_store` that
bumps `mem_addr` by 4 on every wait cycle instead of holding it fails the assertion at a
reachable step (8, in the version this ADR was written against). Neither `memreq.sby` nor
its probe reaches `test/probe_gates.sh` -- `make -C nano/formal components_memreq`, the
`components_qspi` shape, is where it lives and is graded, the same standing `qspi-probe.py`
already has.

## `make nano-coremark` fixed

`NANO_COREMARK_CYCLES`'s default (20,000,000) was smaller than what the zero-wait harness's
own 5 default iterations cost (24,943,488, measured above), so the target has exited 2 on its
own default since whichever earlier change grew that cost. The default moves to 30,000,000,
keeping margin over the measured figure; the banner's stale "No QSPI/PSRAM front end exists
yet" (a pin-level one has existed since ADR-0202) now points at `make nano-qspi-pins-coremark`
instead. Nothing on `make test`'s path grades `nano-coremark` or `nano-dhrystone` -- like
`make cycles`, `make dhrystone` and `make coremark` for littlecpu, they are reporting
instruments with no ratchet, which is why a budget that quietly stopped covering its own
default went unnoticed. Left open rather than decided here: whether either belongs on a
checked path is a question for whoever owns nano's CI shape, not a QSPI-controller ticket.

## Scope

Out of scope, per the brief: `nano/synth_script.sh`, `nano/timing_script.sh` and
`.github/workflows/nano-tt-area-selfhosted.yml` are Tier 1's and the gate-level-simulation
ticket's respectively, and untouched here. `nano/nano.v` and `nano/nano.mk`'s
`NANO_MAX_UM2` line were both out of scope until Tier 1 merged; once both tiers shared a
tree, discharging qspi.v's own formal assumption and re-deriving the ceiling both needed
touching them, and "Rebased onto Tier 1", above, is that record. `stream_next_addr` stays
in `nano/qspi.v`, per the brief's own fourth decision: where the flash is is a fact about
an external device, not the core.
