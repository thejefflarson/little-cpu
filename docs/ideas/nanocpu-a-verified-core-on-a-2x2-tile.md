# nanocpu — a verified core on a 2x2 tile

**Status:** planned · not started · 2026-09-07. Every number below was measured on that date with
yosys 0.68 against the real `sky130_fd_sc_hd`, `ihp-sg13g2` and `gf180mcu` 7-track liberties, or read
from Tiny Tapeout's own shuttle repositories. Nothing here has been built. Where this brief and a
later ADR disagree, the ADR wins.

A second core in this repo, RISC-V, verified to this repo's standard, taped out on Tiny Tapeout in a
2x2 tile. littlecpu cannot go: it is 135,900 um2 on sky130, and every commitment it holds — the
two-word combinational fetch window, the pc published a cycle early, no wrong-path state — presumes
block RAM that answers every cycle. Tiny Tapeout has no RAM and eight bidirectional pins that a QSPI
Pmod consumes entirely. So nanocpu is a new core, and the question this brief answers is what the
best verified core is that fits that box.

## The three constraints, and that all three hold

Tiny Tapeout; verified to this repo's standard; 2x2. **No pair had to be traded.** The recommended
design fits with margin, keeps the formal harness the shape it already has, and every verification
leg transfers except the two that are about FPGA timing.

## Two assumptions that had to die first

**2x2 does not force a bit-serial datapath.** This is the finding that unlocks everything else.

| measured, sky130hd | area | note |
|---|---|---|
| FazyRV core, chunk 1 -> 8 | 9,304 -> 11,462 um2 | datapath width is worth 2.2k across the whole range |
| RV32E register file, 15x32, two read ports | **21,592** | the dominant single cost |
| RV32I register file, 31x32 | 45,011 | |
| 32-bit ALU + barrel shifter + compare + pc adder | **9,086** | the entire execution datapath |
| SERV `serv_rf_top`, 32 regs in flops | 51,825 | 1-bit serial saves nothing once the regfile is flops |
| picorv32 RV32E minimal | 46,447 | a real 32-bit multicycle E core |
| picorv32 RV32E + barrel shifter + C | 51,135 | |

The register file is 35–45% of every design measured. Going serial spends formal depth — FazyRV's own
Makefile sets riscv-formal depth 30/37/61/109 at chunk 8/4/2/1 — to save area the *regfile* is
consuming.

**The 44k um2 budget was too low.** Shipped 2x2 RISC-V projects, TT flow, from the shuttle
repositories' own `stats/`: TinyQV TT06 **53,888** (83.9% utilisation), FazyRV-ExoTiny TT06 **56,448**
(91.0%), TinyQV ttsky25b **66,006**, against 72,565 um2 of core area. Calibration: FazyRV-ExoTiny
synthesises to 61,673 in this flow and shipped at 56,448, so **TT-flow area is about 0.915x a local
`synth; dfflibmap; abc -liberty` number**.

Both TT06 stats files list only `dfxtp_2` — plus `dlxtp_1` where TinyQV instantiates latches by hand.
**The TT flow emits no enable flop**, so an enabled bit is `dfxtp_2 + mux2_1` = 36.3 um2, not
`edfxtp_1`'s 30.03.

## The design

A **32-bit multicycle RV32EC_Zicsr_Zifencei M-mode core**, one state machine, asynchronous-read flop
register file, one 32-bit adder, barrel shifter, executing from QSPI flash in continuous-read mode
with data in PSRAM. Estimated **~49k local flow ≈ 45k TT-flow** against the 56k demonstrated envelope,
with picorv32-E+C at 51,135 as the independent proxy.

1. **ISA: RV32EC_Zicsr_Zifencei, `misa = 0x4000_0014`.** E halves the dominant cost, 45.0k -> 21.6k. C
   is the largest throughput lever on a fetch-bound core — a 16-bit parcel is one fetch, a 32-bit
   instruction two — and riscv-formal has `rv32ic` models. No M: 2.5–3k plus the ALTOPS differential
   oracle machinery, deferred to a 3x2 successor. No A: no shared memory. `fence.i` is a prefetch
   flush and cheap. Zkt is claimed only if the taint grader passes in spirit; a barrel shifter and no
   M make the datapath constant-time by construction.
2. **Datapath: 32-bit, multicycle, barrel shifter.** Serial widths buy 2.2k and cost formal depth. An
   iterative shifter saves ~1.4k and raises the retire gap by 31, which every generated depth pays
   for.
3. **Register file: 15x32 flops, two asynchronous read ports.** One read port is the first fallback
   (−3.5k, +1 cycle per instruction); TinyQV's hand-instantiated latch array the second (~−7k),
   declined for v1 because latches complicate both TT hold-fixing and yosys's formal model on a
   one-way tapeout. DFFRAM's `RAM32` macro is 54.5k for 128 bytes and one port — no.
4. **Memory: the QSPI Pmod on all eight `uio` pins.** Code from flash in continuous-read mode, data in
   PSRAM, no code from PSRAM (the APS6404L's ~8 us CS-low bound is a proved invariant of the
   controller, not a convention). A prefetch FIFO of at least two parcels. **No loop buffer and no
   core pipeline in v1** — see the bandwidth section. Every out-of-window access is a cause-5/7 fault
   arriving with the address, as here.
5. **Pins:** `uio[7:0]` QSPI, `uo_out[0]` UART TX, `uo_out[7:1]` GPIO out, `ui_in[7:0]` GPIO in with
   `ui_in[7]` as MEIP. `mip.MTIP` read-only zero — no `mtime`, whose 128 flops are ~4k this design
   cannot spare.
6. **CSRs: the mandatory M-mode set**, `mcycle`/`minstret` 64-bit and writable (~5k), the performance
   monitor addresses read-zero. Conformance is this repo's line. If the frozen design misses budget by
   less than the counters cost, they are the last cut and the ADR records the non-conformance.
7. **Clock: 64 MHz, SCK = clk/2 = 32 MHz**, inside the 33 MHz output pad rating. STA gate at 64 MHz in
   the TT flow.
8. **PDK: sky130.** Equal capacity to GF180 at 2x2 (below), the flow and the Pmod are proven by two
   shipped RISC-V cores, and the pads are characterised. The RTL is PDK-agnostic; GF becomes the
   target only if a GF shuttle's close lines up with the freeze.
9. **riscv-formal on RV32E:** generate at `isa rv32ic`; a repo-owned copy of `rvfi_insn_check.sv`
   differing from the pin by exactly one `ifdef` block assuming `spec_rs1_addr < 16 && spec_rs2_addr <
   16 && spec_rd_addr < 16`, graded by a diff the way `genchecks-check` grades genchecks. **The
   assumption goes on the spec model's own address wires, never on raw instruction bits** — a blanket
   bit assumption silently removes immediate space from `lui`/`auipc`/`jal`/`csrr*i`/`fence`. Plus a
   hand-written `ill_e` check (an rv32i-legal encoding naming x16–x31 retires with `rvfi_trap`,
   `rd_addr == 0`, no memory write, cause 2) with a forced-red probe and cover goals per encoding
   class. **This patch is the only silent-removal surface in the design.**
10. **Sail co-simulation:** `base.E = true` — verified, `addi x16,x0,1` traces as `illegal 0x100813` —
    with flash/PSRAM/peripheral regions in a second config json. `test/cosim.cc` keeps reading
    `regs_a`.
11. **Shuttle and go/no-go.** Freeze criteria, all graded: TT-flow synthesis ≤ 56k and the GDS action
    clean; STA at 64 MHz; formal green at derived depths; co-sim suite green; the `.S` suite and
    Dhrystone on an FPGA over the real Pmod; gate-level simulation of a suite subset green. Not met
    two weeks before a close means skipping that shuttle, and **the cost of waiting is zero**.
12. **Cost: EUR 395 + EUR 20 Pmod = EUR 415**, about $480 — four tiles at 70, the discounted PCB at
    100 (one per order, "Individual" only, requires a tile in the same order, limited allotment),
    shipping 15. US import duty unverified. A second board is +300.
13. **Repo layout: this repo, `nano/`, with its own included Makefile**, sharing the riscv-formal and
    Sail pins, `genchecks-local.py`, the sanitizer and `cosim.cc`. Forking the grader stack would
    create the "five places" problem twice. `test/march_test.sh`'s single ISA string must not widen;
    nanocpu gets its own.

## Fetch bandwidth, and why there is no pipeline

SCK is an **output** pad, rated 33 MHz; SDR quad moves 4 bits per SCK. So the flash bus tops out at
**132 Mbit/s = 8.25 M parcels/s = 121 ns per 16-bit parcel, whatever the core clock does.** "Eight
clocks per parcel" is that 121 ns expressed at 64 MHz; a 32 MHz core with SCK = clk sees four clocks
per parcel at the same absolute rate. **The pad is the ceiling; the core clock only decides how many
cycles the core gets to spend per parcel.**

**DTR is not available and would not be safe if it were.** The Pmod's KiCad schematic gives the flash
as a **W25Q128JVSIM** — the plain JV. Winbond's DTR read commands (`0Dh`/`BDh`/`EDh`) exist only on
the separate W25Q128JV-DTR part. The PSRAM is an APS6404L-3SQR-SN, SDR only. Even with a DTR part the
data-valid window at 32 MHz DTR is 15.6 ns minus the flash's ~7 ns clock-to-out, and TinyQV's
`qspi_ctrl.v` ships a configurable 0–3 cycle input delay and a falling-edge capture option precisely
because the TT mux path's latency is unknown until silicon. Betting a one-way tapeout on a ~5 ns
untunable window is how nanoV ended at 18 MHz.

**The redirect costs more than the parcel rate.** A taken branch, or any data access — flash CS must
drop to let the PSRAM use the shared lines — re-enters continuous read through a 12-SCK
address/mode/dummy phase: 24 clocks before the first parcel, ~32 including it. A PSRAM word read is
~44 clocks, so a load costs **~68 clocks with the re-preamble, about 1 us**, against ~11 for an
average instruction's fetch. That is where ~2 MIPS at 64 MHz comes from. QPI mode with Set Read
Parameters (`C0h`) at 2 dummy clocks may cut the re-preamble from 12 to 10 SCK — a measured candidate
for the QSPI task, not an assumption.

**Pipelining, at each rate.** With a prefetch FIFO of at least two parcels the fetcher streams the
next instruction while the core executes the current one, so the instruction rate is
max(fetch, execute).

| | fetch per instruction | execute | verdict |
|---|---|---|---|
| 8 clk/parcel (v1) | ~11 clk | 3–4 | fetch is 3x execute; the FIFO already hides execute. **No pipeline** |
| 4 clk/parcel (DTR, or a 32 MHz core) | ~5.6 | 3–4 | still fetch-bound. **No pipeline** |
| with a loop buffer (1 clk/parcel) | ~1.4 | 3–4 | execute-bound; a 2-stage overlap gives up to ~2x **on resident loops only** |

**A loop buffer is the only thing that changes the answer, and it is not in v1.** Eight parcels is 128
flops ≈ 4.2k plus a 22-bit window tag and control ≈ **5.2k TT-flow, 9% of budget**, catching loops of
4–8 instructions and turning ~72 clocks per iteration into ~12 on those. Sixteen parcels ≈ 9.5k
catches most inner loops; thirty-two ≈ 18k is the register file again. It stops paying past 16 because
the next tier of hot code is call chains rather than parcel loops, and it does nothing for the 68-clock
load. The cxxrtl runner charges every cycle to one of `{parcel wait, redirect preamble, PSRAM wait,
execute}` from the first program, the way `make cycles` does here; **if that accounting shows
buffer-resident loops worth more than the budget's spare after the freeze criteria are met, an
8-parcel buffer is the first thing the spare buys.** The trigger is a number.

## The donor: `c55efd6`, not a fresh core

This repo's history contains the multicycle core the current pipeline replaced, and it is the shape
this design independently arrived at. **`c55efd6` — "enable csr checks, swap out handwritten outputs
for RVFI_OUTPUTS"** is the last clean single-file commit: `riscv.v` alone, 704 lines, flat layout,
with `formal/checks.cfg`, `complete.sby`, `imemcheck`, `dmemcheck`, `genchecks.py` and `wrapper.v`
beside it. Its state machine is fetch / ready / decode / execute / finish_load / finish_store /
check_pc / reg_write / multiply / divide — one instruction in flight — over a picorv32 valid/ready
bus.

**Measured against today's riscv-formal pin:** it elaborates clean under yosys 0.68; `genchecks.py`
generates 78 checks and **76 pass** in 4–19 s each at its 2020 depths — all 70 instruction checks
including M and every one of the 24 compressed encodings, plus `reg`, `pc_fwd`, `pc_bwd`, `liveness`,
`unique`, `causal`. **`csrw_mcycle` and `csrw_minstret` fail**: those checks were enabled in that
commit, not passing. `complete` errors only on an engine-line syntax drift and **passes at depth 20
under `btor btormc` in 6 s**. Under iverilog 13 it does not build — three declaration-after-use sites
and one `ivl` segfault. Area 84,291 um2, 1,491 flops, of which 992 are the 31x32 register file.

**It has no CSR state and no trap entry.** No `mstatus`, `mtvec`, `mepc`, `mcause`, `mie`, `mip`,
`mscratch`, `mtval` or `misa` anywhere. Its `trap` is a picorv32-style halt: a terminal `cpu_trap`
state that raises an output, with no vector, no `mepc` save and no `mret`. What CLAUDE.md calls M3 is
entirely absent, and that is exactly the conformance floor decision 6 makes the differentiator.

**So: donor, not revival.** The design is the thirteen decisions above; the old file is verified
material transplanted into it. `git show c55efd6:riscv.v > nano/nano.v` — a file copy at a commit
hash, recorded in the ADR. No branch, no shared history. Where the donor conflicts with a decision —
RV32I against E, trap-as-halt against the CSR floor, the picorv32 bus against the parcel front end —
the decision wins.

What transplants is the expensive half, and it arrives with proof: the decode flags and immediates,
the per-encoding compressed register selection, the execute arms, the RVFI retire block — roughly 350
of 704 lines. What does not: the bus and the trap handling, both being replaced regardless.

**The real asset is a green baseline from the first commit.** Every reshaping step — E, no-M, the
front end, the CSR layer — is graded against 76 generated checks plus `complete` and `liveness` *as it
lands*, instead of building a fresh core and only then discovering whether it is correct. Post-reshape
estimate: −23.4k for E, −10–15k for M, +~6k CSRs and traps, +~5.5k QSPI = **~48–55k local ≈ 44–50k
TT-flow**, which is approach A's number because it is approach A's shape.

Fallback if the donor's fetch/decode coupling fights the parcel front end: a fresh core with the
donor's decode and execute arms transplanted — nearly the same work.

## What was rejected, and why

**A 4-bit serial core (TinyQV's shape), ~40–45k.** Throughput is identical — the core is fetch-bound
either way. It costs riscv-formal depth about 8x an instruction's internal cycles, a rotating register
file whose 420 hold-delay cells (4.2k) exceed the datapath it saves and whose `regs` are only
architecturally readable when the rotation counter is zero — which fights `test/cosim.cc`'s direct
`regs_a` read — and a constant-time claim that becomes an argument about a shift register. It spends
verification to save area the register file is consuming.

**A littlecpu derivative.** 135,900 um2, and stripped of M, A and the wide register file it is still
above 65k before a QSPI controller. Every commitment it holds is about a memory that answers in a
cycle; on a serial fetch the pipeline would idle 8–16 cycles per instruction and the no-wrong-path
property would be paid for and unused.

**GF180 and IHP.** Cells scale 3.06–3.13x on gf180 and 2.0–2.1x on IHP against sky130; tiles are 3.06x
and 1.74x. So **a GF180 2x2 holds the same logic as a sky130 2x2 within 3%, and an IHP 2x2 holds about
84% of it.** GF's 3.3-month close-to-delivery against sky130's 5.6–6.7 and IHP's 9–11 is real and
makes GF the fallback if a shuttle lines up first, but it buys no area. IHP's SRAM macros would change
the design point entirely — TT calls their integration "not trivial" — and are the reason to revisit
this on a later shuttle rather than now.

## Risks

- **The area estimate is ±8k until the skeleton synthesises.** picorv32 is a proxy; 84,291 is the
  donor's first real data point. Build the register file, datapath and sequencer first and run the TT
  flow in week one.
- **The QSPI Pmod is the trust boundary.** Invariants to prove: CS0/CS1/CS2 never low together; no
  PSRAM CS-low interval exceeds 512 clocks at 64 MHz; the parcel buffer holds exactly the parcels at
  [fetch_pc, fetch_pc+N). Failure modes: a flash that never answers, so `hang`/`liveness` must model
  bounded latency by assumption and say so; a mode-bit mismatch after reset leaving the flash in
  continuous read, needing a reset sequence that exits it; pad-mux latency at 64 MHz, which TinyQV
  handles with a registered, configurable input capture.
- **Gate-level versus RTL divergence** — ADR-0163's lesson from the ECP5. sky130 ships behavioural cell
  models, so the gate-level gate is buildable; make it required.
- **The E assumption patches a vendored oracle file.** The diff grader and the `ill_e` probe are the
  whole defence.
- **The interrupt path is unmodelled by riscv-formal**, as it is here; a `components_traps` analogue
  plus a `.S` program over `ui_in[7]`.
- **Nothing is purchasable.** TTSKY26c closed 2026-09-07 and TTIHP26b closes 2026-09-21. The next
  sky130 shuttle is unannounced; by cadence it would close around Dec 2026 – Jan 2027 with silicon
  mid-2027, but that is inference, not a date.
- **TT's flow changes between shuttles** (LibreLane versions), so the area ratchet must be taken with
  the shuttle's pinned flow, and `tt-gds-action` in CI is the instrument.

## Sequence

1. **Instruments before design.** TT template, local LibreLane/GDS action in CI producing
   `synthesis-stats.txt`, graded ratchets on area (≤ 56k) and 64 MHz STA, each with a forced-red probe.
2. **Import the donor** at `c55efd6` into `nano/`, and make its harness run at the current pin — engine
   line, paths, `genchecks-local`'s audit. Re-derive depths by the `remeasure-fg` method; the 2020
   depths were inherited and `insn 10` on a five-cycle instruction is plausible but unproven here.
3. **Reshape under the checks**, re-taking sky130 area after each step with 84,291 as the first point:
   E, then no-M, then the QSPI parcel front end replacing `fetch_instr`/`ready_instr` and the picorv32
   bus.
4. **The QSPI controller** with behavioural flash and PSRAM models, its `mode prove` component task,
   and FPGA bring-up on the iCESugar-Pro with the real Pmod.
5. **CSRs, traps, `mret`, MEIP** — the layer the donor lacks — with a traps proof and the CSR checks.
6. **UART TX, GPIO, the memory map**, the `tt_um_` top with `uio_oe`, and an iverilog X-leg on the pad
   tri-states.
7. **Gate-level simulation gate; Dhrystone over QSPI on the FPGA; mutation-check table; freeze;
   purchase.**

## Deferred

M, on a 3x2 successor with the differential oracle. `mtime` and any interrupt controller. The
one-port and latch register files, both measured fallbacks at −3.5k and −7k. Code from PSRAM. The
GF180 and IHP ports. A loop buffer, gated on the cycle accounting. Cross-core comparison — **nanocpu
is not littlecpu's peer and must never be quoted against it.**
