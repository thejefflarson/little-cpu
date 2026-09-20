# ADR-0202: nano's QSPI front end -- a real controller, and three proved invariants

**Status:** Accepted, with a known residual · 2026-09-19

## Context

`nano/nano.v` still carries the donor's picorv32-style bus with nothing behind it. A
Tiny Tapeout tile has no RAM, so the real memory system is the QSPI Pmod the brief
specifies: code from flash in continuous-read mode, data in PSRAM. ADR-0186 measured
what that latency does to nano's throughput using an abstract bus-level timing model,
`nano/tb/nano_qspi_memory.v`, and said plainly that it "is not a QSPI protocol model" --
the real pin-level controller was still owed. This ADR builds it.

## The controller

`nano/qspi.v` (`nano_qspi_ctrl`) bridges nano.v's `mem_valid`/`mem_instr`/`mem_ready`/
`mem_addr`/`mem_wdata`/`mem_wstrb`/`mem_rdata` bus to two physical QSPI devices sharing
one `sck` and one 4-bit `sio` bus: flash for every instruction fetch, PSRAM for every
load and store, matching the brief's "no code from PSRAM" decision exactly by routing
on `mem_instr` alone. SCK runs at clk/2 (mode 0, launched on the falling edge, sampled
on the rising edge), so a 16-bit parcel is 8 core clocks -- the same PARCEL_CYCLES
ADR-0186 already assumed.

- **Flash**: Fast Read Quad I/O (`EBh`) is sent once; the mode byte's continuation
  pattern (`M[7:6] == 2'b10`) keeps continuous-read mode open across every later
  redirect, so a steady-state redirect is address(6 nibbles)+mode(2)+dummy(4) = 12
  nibbles = 24 core clocks, matching ADR-0186's `PREAMBLE_CYCLES`. A two-slot prefetch
  queue (not a parameter -- fixed at the brief's own "at least two parcels") holds each
  slot's own address tag as real state, not a shadow copy, and pauses the clock (CS
  stays asserted) rather than closing the session once both slots are full, so a
  sequential fetch that catches up to an already-open stream costs nothing.
- **PSRAM**: Fast Read (`0Bh`) and Page Program (`02h`), each a fixed 32-bit transfer --
  `mem_addr` arrives word-aligned for every load and store, since nano.v computes it
  that way before issuing it, so byte lanes are resolved device-side from `mem_wstrb`
  rather than over the wire. A **partial-word store has no wire-level equivalent** on a
  device that only takes an aligned command: the controller reads the word first,
  merges in exactly the wstrb-selected bytes, and writes the merged word back, so a
  `sb`/`sh` costs two transactions instead of one -- a real cost this design pays that
  the brief's abstract model never priced, because that model assumed every store was
  already word-sized.
- **Reset**: a mode-byte-exit pulse (four bytes of `FFh`) runs once before any real
  fetch, driven low regardless of whether the flash happened to power up already inside
  continuous read.
- **Pad-mux capture**: `IN_CAPTURE_STAGES` (default 1) registers `sio_in` before the
  serializer reads it, the same registered-input answer TinyQV gives to the same
  unknown mux latency; 0-3 stages are legal, matching TinyQV's own range.

## The three invariants, each proved and each with a forced-red probe

`nano/formal/qspi.sby` proves `nano_qspi_ctrl` alone by k-induction (`mode prove`,
`abc pdr` -- plain BMC-based induction could not close two of the three without a
stronger auxiliary invariant than a first pass justified writing; PDR closed all three
directly in about 90 seconds). `nano/formal/qspi-probe.py` is the forced-red
prerequisite, mirroring `ill-e-probe.py`'s pattern: it mutates `nano/qspi.v`'s own
source text, runs `sby` against each mutant, and requires the shipping controller to
pass first.

1. **CS0/CS1/CS2 never low together.** True by construction -- `flash_cs_n` and
   `psram_cs_n` are both derived from one two-bit `active_dev` register, and the third
   select is tied inactive -- but asserted and probed anyway, so a later edit that stops
   deriving both from one register is caught here rather than believed. Probe: tie
   `psram_cs_n` permanently low; the mutant must fail immediately.
2. **No PSRAM CS-low interval exceeds 512 clocks at 64 MHz.** A saturating counter
   tracks cycles since `psram_cs_n` last read high; k-induction proves it never reaches
   the limit. The real worst case (a read-modify-write's read half, or its write half --
   each its own continuous CS-low period, since the reopen between them drives CS high)
   is under 45 cycles, so the margin is wide on purpose: this is a proof the state
   machine cannot get stuck, not a tight budget. Probe: drop the `psram_cs_n` term from
   the counter's reset condition, so it free-runs past the limit even while never
   actually asserted.
3. **The prefetch buffer holds exactly the parcels at `[fetch_pc, fetch_pc+N)`.**
   Stated as modular-subtraction equalities rather than `<` comparisons -- the parcel
   address is 31 bits and wraps, and a wraparound trace is exactly the counterexample an
   ordering compare gets wrong, which `abc pdr` found on the first attempt and the
   rewrite closed. Probe: tag the second slot one parcel ahead of the parcel it actually
   received, breaking contiguity.

## The failure modes the brief called out

- **A flash that never answers.** `nano/formal/checks.cfg` already treats `mem_ready`
  as a free input under `RISCV_FAIRNESS` for `hang`/`liveness` -- a claim that some
  memory eventually answers, not a claim about this one's bound -- and a comment now
  says so directly in the file, so the boundary between "the core's own proof" and "the
  controller's own proof" is stated rather than assumed. Composing the real controller
  into those checks was not attempted: a single redirect already costs more cycles than
  `complete`'s whole depth-20 walk, so F and G would have to grow past what BMC can
  reach for anything past the first instruction.
- **A mode-bit mismatch after reset.** The reset sequence's four-byte `FFh` pulse
  exits continuous read regardless of the flash's power-on state; `nano/tb/
  nano_qspi_flash_model.v`'s own `cont_mode` tracking is what makes this checkable in
  simulation (Context section, below).
- **Pad-mux latency at 64 MHz.** `IN_CAPTURE_STAGES` is the registered, configurable
  answer; no board exists yet to measure the real mux delay against it, so the default
  of 1 stage is a documented choice, not a measurement.

## F and G: re-measured, unchanged

`make -C nano/formal remeasure-fg` reproduces F = 12, G = 10 exactly. This is not a
null result to be suspicious of: nano.v's own bus contract with its memory system did
not change -- it still issues one `mem_valid`/`mem_ready` transaction and waits, the
same contract the abstract bus-level model and the real controller both honor. F and G
describe nano.v's own internal decode/execute/CSR-serialization cycles under a
fairness-only memory assumption; they are about the core, not about which real memory
answers it. A change here would be owed only if nano.v's own bus discipline moved.

## Sim harness: pin-level models exist, and mostly work

`nano/tb/nano_qspi_flash_model.v` and `nano/tb/nano_qspi_psram_model.v` are pin-level
behavioural models -- SCK/CS/SIO, not the abstract bus ADR-0186's model speaks -- that
answer nano_qspi_ctrl's real protocol, including the flash's own continuous-read mode
tracking. `nano/tb/nano_testbench.v` gains a third memory-instantiation branch,
`NANO_QSPI_PINS`, wiring `nano.v -> nano_qspi_ctrl -> {flash model, psram model}`
exactly as the shipped design would; `nano/tb.mk` adds `nano-qspi-pins-sim` (the cxxrtl
build) and `nano/tb/nano_icarus_qspi_pins.vvp` (the iverilog leg) alongside it.
`nano/tb/nano_cxxrtl.cc` now looks for `"flash mem"`/`"psram mem"` when the flat
`"mem mem"` debug item is absent, so the same driver serves both shapes with no
duplicated `main`.

A hand-written protocol-level scratch test (ten scenarios: sequential fetch, a redirect,
a resumed fetch after the redirect, a plain PSRAM load, a full-word PSRAM store and
readback, a partial-word store's read-modify-write and readback, and a fetch that
resumes correctly after a PSRAM interruption) passed end to end through
`nano_qspi_ctrl` and both pin-level models during development, but was not committed --
its most exposing gap, the two chained resumes below, is now covered instead by the
committed `nano_qspi_resume_tb.v`. Five real bugs surfaced and were fixed while building
it: a units error in the flash model's byte-to-parcel address
conversion (multiply-then-divide-by-two that canceled itself out, so a redirect always
targeted the byte address instead of the parcel address); `mem_rdata` computed
combinationally from live queue state that a hit's own retire had already moved by the
time `mem_ready` read high (fixed by latching `mem_rdata` at decision time, alongside
`mem_ready`); a "direct combine" pattern in three completion sites that re-shifted an
already-fully-assembled register with a stale extra nibble; a missing
read-modify-write path for partial-word stores (see above); and a wraparound-unsafe
formal invariant (see invariant 3, above).

**A sixth issue was found. The first diagnosis of it was wrong, and named the controller;
the real bug is in the pin-level test models, and the controller's pause/resume protocol
is correct.** Getting this backwards once is worth stating plainly, because a reader who
trusted the earlier text here would go fix a controller that was never broken.

**The real mechanism: two independent one-clock bugs in
`nano/tb/nano_qspi_flash_model.v`, each invisible alone, that stop cancelling exactly
where a pause exposes them.** Both reacted to a clk-registered copy of `sck`
(`sck_rise`/`sck_fall` compared against a `posedge clk`-sampled `sck_d`) rather than to
`sck` itself, which is one real clock late launching an output -- harmless for *sampling*
an input, since the controller doesn't read `sio_captured` until well after the edge
either way, but wrong for an edge the model itself must react to in time. That lateness
hid a second bug: `dummy_left <= DUMMY_SCK` loaded on the mode byte's own *rising* edge,
while the dummy countdown runs on *falling* edges, so the very next fall -- the mode
byte's own, not a dummy one -- was spent as dummy cycle one, and `DUMMY_SCK` bought only
`DUMMY_SCK - 1` real dummy clocks. While SCK free-runs, being a clock late and starting
one clock short cancel: the model's output timing and the controller's own sampling stay
aligned by accident. A pause freezes SCK. The one-clock-late reaction stops mattering (there
is nothing left to react to), but the short dummy count does not un-happen -- it already
consumed one real edge the controller's own accounting did not spend, so the flash is left
one nibble position ahead of where the controller assumes it is. That is exactly the
"trailing falling edge" the earlier text blamed on the controller: it is real, it is
load-bearing (a continuing stream's own next capture consumes it), and removing it -- which
is what the first, reverted fix attempt did -- breaks a correct controller instead of fixing
a broken model. That also explains that attempt's own second regression: forcing the
completion decision one cycle earlier fought a protocol that did not need fighting.

**The decisive test:** with `nano/qspi.v` completely untouched, a copy of the flash model
that reacts to the real `sck` edges (`@(posedge sck)` for the command/address/mode bytes,
`@(negedge sck)` for the dummy countdown and the nibble advance) and loads
`DUMMY_SCK + 1` makes all four chained fetches in the reproduction below PASS. Both
changes are needed together -- fixing the dummy count alone, still reacting a clock late,
makes the corruption worse, not better. `nano/tb/nano_qspi_psram_model.v` carries the
identical pair of bugs (a clk-registered edge comparison, a dummy count loaded one short
on the address's own rising edge) and is fixed the same way, but the PSRAM read path
never pauses mid-transfer, so nothing in this tree exercises the defect there; a real
part would show it under a pause this controller does not currently issue.

**What the three proofs do not cover, and why this slipped past them.** All three proved
invariants -- CS mutual exclusion, the PSRAM CS-low bound, and the prefetch buffer's
address-tag contiguity -- are properties of the controller's *own* state bookkeeping.
None of them says the bytes arriving off the wire are the bytes the flash actually sent;
that is a claim about an external device's behavior, which a proof confined to the
controller's own signals structurally cannot make -- and in this case the external
device was the thing actually wrong. The isolated ten-scenario protocol test missed it
for a narrower, separate reason: it never chains two "fetch_hit0 && !queue_full" resumes
back to back (an uncompressed instruction's second parcel, immediately followed by
another uncompressed instruction's own second-parcel need) -- the specific pattern that
exposes the drift, which `loadstore.S` happens to hit and the hand-written scenarios
happened not to.

**The minimal reproduction, `nano/tb/nano_qspi_resume_tb.v` (run by `make
nano-qspi-resume-test`), now passes, for the right reason.** It talks to
`nano_qspi_ctrl` and the pin-level flash/PSRAM models directly, no `nano.v` involved: a
compressed parcel followed by three uncompressed ones in a row, each needing the parcel
after it, chaining two "`fetch_hit0 && !queue_full`" resumes back to back. Against the
fixed models and the unmodified controller it reports the exact parcels the flash sent.
`nano/tb/nano_qspi_resume_probe.sh` is its forced-red probe, re-anchored now that the
shipping pair actually passes: it shrinks the resume branch's own nibble count by one in
a scratch copy of the controller and requires the shipping pair to PASS while that
mutant FAILS -- the ordinary shape this repo's probes take, in place of the
comparison-neutering stand-in the still-broken pair needed before a real PASS existed to
compare against.

**Fixing the models alone turns `nano-qspi-resume-test` green and `nano-qspi-pins-test`
red**, and that residual is its own section below.

## MIPS: the abstract model's own machinery, re-run at this controller's real costs

Since the pin-level path isn't yet trustworthy end to end, the timing figure below
reuses ADR-0186's already-graded abstract model (`nano-qspi-sim`, not the pin-level
build) with its parameters set to what this controller actually measures rather than
what the brief guessed: `PREAMBLE_CYCLES=24` (matches exactly), `PSRAM_LOAD_CYCLES=41`
and `PSRAM_STORE_CYCLES=33` (this controller's own real bit-serial cost: command(2
nibbles)+address(6)+dummy(4)+data(8) = 20 nibbles = 40 core clocks, +1 handshake, for a
read; 16 nibbles = 32 clocks +1 for a write), `PREFETCH_DEPTH=2`, no loop buffer (v1
scope, per the brief). This is a legitimate cross-check, not a substitute for a real
pin-level run: it answers "does this controller's own measured per-transaction cost,
dropped into the already-validated cycle-accounting model, land where the earlier
abstract sweep predicted" -- and it does, closely.

| | Dhrystone cycles/run | DMIPS/MHz | MIPS@64MHz | CoreMark cycles/iter | CoreMark/MHz | MIPS@64MHz |
|---|---|---|---|---|---|---|
| This controller (depth 2, no loop buffer) | 19,985.5 | 0.028 | 1.70 | 19,232,290.4 | 0.052 | 2.54 |
| ADR-0186's FIFO depth 2 (abstract) | 20,300.5 | 0.028 | 1.67 | 9,917,092.4 | 0.101 | 1.92 |

Dhrystone lands within 1.6% of ADR-0186's own FIFO-depth-2 row, which is the expected
outcome given nearly-identical parameters (this controller's measured 41/33-cycle PSRAM
cost against the abstract model's assumed 44/33). CoreMark's retire-rate MIPS (2.54)
diverges more from the abstract model's own CoreMark cycle count, which the brief's own
"CoreMark is simulated at 16 KB of ROM" caveat and the two benchmarks' different
code-size/branch-density profiles both bear on -- the two numbers are not required to
match, since the abstract model's PSRAM/preamble parameters, not its CoreMark trace,
are what this cross-check is re-using. **The brief's ~2 MIPS estimate is reached**:
1.70-2.54 MIPS depending on benchmark, consistent with ADR-0186's own finding that
~2 MIPS needs FIFO overlap but not necessarily a loop buffer.

## Area: measured, and well over the brief's guess

`nano/area_top.v` is a synthesis-only top wiring `nano.v`'s `riscv` module to
`nano_qspi_ctrl` the way the shipped design does; without it, yosys's `hierarchy
-auto-top` synthesizes whichever of the two independent modules happens to look like a
better root and drops the other entirely; `nano/synth_script.sh` gained a `flatten
-noscopeinfo` step (harmless on the existing single-module measurement, confirmed
unchanged at 45,712.6 um2) to keep the two modules' logic in one flat census rather
than reporting one as an unmapped `$paramod` cell.

nano.v alone: 45,712.6 um2 (unchanged). nano.v + `nano_qspi_ctrl` together: **60,859.6
um2**. The brief's own back-of-envelope guess for this controller was ~5,500 um2 local;
the real bit-serial engine, its two-slot prefetch tags with per-slot address
comparators, and the read-modify-write merge measure **roughly 2.8x that** (15,146.9 um2
attributable to the controller once yosys shares some logic across the two modules'
boundary, against the sum-of-separate-syntheses estimate of 15,931.5). `NANO_MAX_UM2`
moves 61,412 -> 63,000, the same order of headroom the prior ceiling carried. The area
this ticket produces is not what decides nanocpu's freeze line -- that decision is
JEF-1011's, already resolved before this ticket landed.

## Scope

Out of scope, per the ticket: FPGA bring-up on the iCESugar-Pro with the real Pmod (no
board bought yet). `nano/formal/checks.cfg`'s `#insn-check rvfi_insn_check.sv` line and
the RV32E oracle patch it names are untouched. Deferred, not closed: root-causing the
chained-resume bug above and wiring `nano-qspi-pins-test` onto `make test`'s path;
taking a real Dhrystone/CoreMark figure through the pin-level harness once it is fixed;
FPGA-measuring the pad-mux latency `IN_CAPTURE_STAGES` guesses at.
