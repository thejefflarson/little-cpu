# 0139 — Hazard3's iCE40 build joins soc/compare/: clock measured, cycles not yet

Status: Accepted (partial) · 2026-08-26 · vendoring and the clock factor land; CoreMark's cycle
factor is explicitly deferred

## Context

ADR-0136 declined to compare this core's CoreMark figure against Hazard3's published **4.15
CoreMark/MHz**, because that number is Hazard3's RP2350 configuration — Zba/Zbb/Zbs/Zbkb, a fast
multiplier, a branch predictor — and its iCE40 build (`example_soc/fpga/fpga_icebreaker.v` in that
project) has none of that:

```
EXTENSION_C              0     no compressed instructions
EXTENSION_A / M          1 / 1
ZBA/ZBB/ZBC/ZBS/ZBKB     all 0
EXTENSION_ZIFENCEI       0
MULDIV_UNROLL            1     one bit per cycle
MUL_FAST / MULH_FAST     0 / 0
BRANCH_PREDICTOR         0
U_MODE / PMP_REGIONS     0 / 0
CSR_COUNTER              0
CLK_MHZ                  12    the same part and clock this core targets
```

Quoting 4.15 against this core's own number would be the mixed-configuration error ADR-0098 already
named for VexRiscv. The only comparison this repository trusts is one harness: same part, same
memories, same program, same toolchain, same seeds — `soc/compare/` already does this for VexRiscv
(ADR-0086, ADR-0098), and this ADR is that treatment for Hazard3's iCE40 configuration.

**This lands in two pieces, and only the first is finished.** The clock factor — Hazard3's iCE40
build placed beside this core's, in the existing `soc/compare/` harness — is measured below. The
cycle factor needs CoreMark to run on both cores and for Hazard3 to be timed while it does, and
Hazard3's iCE40 build has `CSR_COUNTER = 0`: no `mcycle`, no `minstret`, so it cannot self-time the
way `test/bench/coremark_port.c` and `soc/compare/dhry_port.c` both rely on for this core and for
VexRiscv respectively. **That half is not built yet** — see "What is not done" below — and a wrong
cross-core figure is worse than none, which is the position ADR-0136 already took and the reason this
ADR does not invent one to close the ticket.

## What shipped

**Hazard3 is vendored the way `formal/pin.mk` vendors riscv-formal**: `soc/compare/hazard3_pin.mk`
pins `github.com/Wren6991/Hazard3` at `8af992930f71a69b0e06c38734c1094f41a05ca0`, 40-hex, `override`
so the pin cannot be defeated from the command line, and the clone materialises into
`soc/compare/hazard3/` on demand — gitignored, never committed, fail-closed if a stale clone sits at
the wrong SHA. Hazard3 is Apache-2.0 (its own `LICENSE`, no narrowing `NOTICE`), checked before
vendoring.

**`soc/compare/bench_hazard3.v` instantiates `hazard3_cpu_1port`** — the single-AHB5-port top, the
one Hazard3 top level this harness needs since it drops into the same one-memory-map shape
`bench_littlecpu.v` and `bench_vexriscv.v` already share — with `fpga_icebreaker.v`'s extension,
CSR and muldiv parameters copied verbatim: `EXTENSION_C=0`, `EXTENSION_A=1`, `EXTENSION_M=1`, every
Zb*/Zk* flag `0`, `EXTENSION_ZIFENCEI=0`, `CSR_COUNTER=0`, `U_MODE=0`, `PMP_REGIONS=0`,
`MULDIV_UNROLL=1`, `MUL_FAST=0`, `MUL_FASTER=0`, `MULH_FAST=0`, `FAST_BRANCHCMP=1`,
`BRANCH_PREDICTOR=0`. Three deviations from `fpga_icebreaker.v`'s own SoC integration, all in the
file's own header comment:

- **No JTAG debug module.** `DEBUG_SUPPORT` stays at its default of `0` — `fpga_icebreaker.v` does
  not set it; its own `example_soc.v` hardcodes `1` to build the `hazard3_dm`/JTAG-DTM this harness
  has no debugger to drive, the same limitation `bench_littlecpu.v` and `bench_vexriscv.v` already
  have (no scan chain on either side of this harness).
- **Text at address 0, not `example_soc.v`'s bootloader offset of `0x40`.** That offset exists to
  leave room for a JTAG-loaded program; this harness has no debugger and no bootloader, so it uses
  the same memory map the other two benches do.
- **`CSR_M_MANDATORY` and `CSR_M_TRAP` stay on**, `example_soc.v`'s own values rather than
  `fpga_icebreaker.v`'s list: with them off the core cannot execute at all, since `misa` and the
  trap CSRs are the bare minimum any RISC-V core with CSRs needs. `CSR_COUNTER` stays off exactly as
  `fpga_icebreaker.v` sets it — that is the parameter this whole ADR is about, and it is not
  touched.

**The bus**: `hazard3_cpu_1port` arbitrates fetch and load/store down to one AHB5 master port,
unlike VexRiscv's separate `iBus`/`dBus` in this harness's other bench. `hresp`/`hexokay` are tied
to always-OKAY the way `example_soc.v`'s own comment says ("No global monitor"). Reads answer at
zero wait states — a read's address phase in cycle N is the answer `rtl/memory.v` (reused
unmodified for the RAM half) registers in cycle N+1, the same latency the AHB protocol already
assumes at zero wait states. **Writes are not zero wait states**, and getting this wrong is the
subject of its own section below: `hready` is held low for one cycle after a write's own address
phase, because AHB5 does not present `hwdata` until the following cycle and a single-ported memory
cannot service that write's data phase and a new transfer's address phase at once. ROM and RAM
answer the same `haddr` (or its one-cycle-captured copy, for a pending write), distinguished purely
by which side of `ROM_WORDS*4` it falls on, since AHB has no separate read and write bus the way
this harness's other two benches do. `hwdata` is not shifted to byte 0 for a narrow store —
`hazard3_core.v` replicates it across all four lanes (`MEMOP_SB`/`MEMOP_SH`), the same replication
`bench_vexriscv.v`'s own comment names for VexRiscv — so the byte strobe alone, shifted by the low
address bits, is what picks the right byte out of a lane that already holds it everywhere.

**Anti-fraud, extended to the new core.** `soc/compare/placed_vs_synth.py` is fully generic (a placed
count, a standalone count, a floor) and needed no code change — only a Makefile branch that hands it
Hazard3's own two logs. `soc/compare/bench_tb.v` now instantiates all three cores on
`soc/compare/bench.S` and requires all three to publish the same sequence of stores; Hazard3's own
sequence is read off the same captured address/strobe the adapter performs its write against
(`mem_addr_mux`/`mem_wstrb_mux`, with `hwdata` as the value), since Hazard3 has no separate data bus
to watch the way VexRiscv's `dbus_cmd_address` gives one for free. **This check is why the bus bug
above was ever visible** — see the section below.

`soc/compare/geometry_test.sh`'s six-file cross-check is **not** extended to Hazard3's own file in
this pass — it stays a two-core check between `bench_littlecpu.v` and `bench_vexriscv.v`. Hazard3's
geometry is still enforced the same way the other two are (`COMPARE_ROM_WORDS`/`COMPARE_RAM_WORDS`
reach `bench_hazard3.v` through the same `chparam` the Makefile already uses for both), just not
cross-checked by that particular hermetic script. Left out on purpose to keep this change's footprint
in `test/probe_gates.sh` at zero, since another ticket is running concurrently against that file;
extending `geometry_test.sh` to a third core is a small, separable follow-up.

## The anti-fraud check found a real defect, not a hypothetical one

`make compare-smoke` is not a formality here: the first working version of `bench_hazard3.v`
placed, synthesised at a plausible 0.95× ratio, and **published three all-X words** where
littlecpu's and VexRiscv's own sequences were real numbers. The adapter fed `hwdata` into
`rtl/memory.v` on the SAME cycle as `haddr`/`hwrite`, treating the bus as a zero-wait-state SRAM
port for writes as well as reads. AHB5 does not work that way for a write: `hwdata` is valid one
cycle **later**, in the transfer's data phase, which overlaps the following transfer's own address
phase. A single-ported memory cannot service a write's data phase and a new transfer's address
phase in the same cycle, so a correct AHB-to-simple-RAM bridge needs a wait state exactly there —
and `soc/compare/bench.S` hits that collision on every inner-loop iteration (`sw` immediately
followed by `lw` of the same word). The fix captures a write's address-phase information into a
one-cycle-pending register, holds `hready` low for the cycle its data phase needs, and performs
the write against the captured address once `hwdata` is valid; reads were already correct, since a
read's response never depends on data the core has not presented yet. Verified directly: Hazard3's
six published values now match littlecpu's exactly, where before they read `xxxxxxxx` from the
first unaligned byte/halfword store onward. This is the same shape of defect `CLAUDE.md` already
names for VexRiscv's own harness — a bus adapter that is wrong rather than a core that is slow —
and it is exactly what this gate exists to catch.

## The clock factor

Both cores placed on the existing `hx8k`/`ct256` comparison harness, `soc/compare/bench.S` (RV32I
only — no M, no A, no C exercised, so this is the two cores' shared datapath and not their differing
extensions), default seed plus four explicit seeds, this tree (`5f005c7`, after the adapter fix
above):

| | worst | median | best | placed `ICESTORM_LC` | standalone `SB_LUT4` | ratio |
|---|---|---|---|---|---|---|
| **littlecpu** | 30.90 MHz | 32.01 MHz | 32.69 MHz | 7254 / 7680 (94%) | 6517 | 1.11× |
| **Hazard3 (iCE40 config)** | 32.60 MHz | 33.63 MHz | 33.89 MHz | 3346 / 7680 (43%) | 3505 | 0.95× |

Both ratios clear `COMPARE_MIN_RATIO`'s 0.80× floor, so neither core folded away behind the harness.
**Hazard3's iCE40 configuration clocks 1.06× this core's on the worst placement of five
(1.05× on the median)** — 32.60 MHz against 30.90, both on the same part, memories, program and
seeds. Five seeds is a look, not a verdict; `SOC_MIN_MHZ`'s own convention wants twelve to sixteen
before reading a gap this size as settled, and that sweep has not been run. Placed area and the
placed/standalone ratio are unchanged by the adapter fix (3346 cells, 0.95× either way) — the wait
state the fix adds is one flip-flop and a small mux, evidently inside this placement's churn rather
than outside it — but the fix moved the worst-case clock from 33.08 to 32.60 MHz and the median from
34.48 to 33.63, so the gap is real but smaller than the broken adapter first reported.

**Read the standalone counts as area context, not as the clock explanation.** Hazard3's iCE40 build
is roughly half this core's size here (3505 against 6517 `SB_LUT4`) mostly because it is a smaller
machine on paper — no CSR file beyond the trap minimum, no atomics decode beyond `EXTENSION_A`'s bare
LR/SC and AMOs, `MULDIV_UNROLL=1` instead of a parallel multiplier, no bitmanip, no branch predictor
— not because of anything this harness measured about *why* the clock differs. Neither core's
critical path was walked for this ADR; that is real remaining work if the gap is worth explaining
rather than only quoting.

This reproduces two things this repository already expects rather than surprises: `example_soc/`
targets an iCEBreaker (an iCE40 UP5k board, the same part class this project's own board uses), and
`fpga_icebreaker.v`'s configuration is deliberately minimal — no forwarding network here either
(ADR-0083), so the two cores' shared stall-only-hazard datapath is closer in shape than the
RP2350 comparison would ever have been.

## The decomposition ADR-0136 asked for, so far as it can be stated without the cycle factor

Two things run in this core's favour on Hazard3's iCE40 configuration and are confirmed rather than
merely assumed: this core's multiplier is single-cycle off the ice40's DSP tiles (`MUL_FAST=0` plus
`MULDIV_UNROLL=1` gives Hazard3 roughly 32 cycles per multiply here, the same shape as its own
sequential divider), and this core has the C extension where Hazard3's iCE40 build turns it off
(`EXTENSION_C=0`), so an instruction-count comparison built on a compiled program would need to
separate "fewer cycles per multiply" from "fewer instructions fetched" rather than reading either as
the whole story.

Two things run against this core, both already priced from this side rather than newly measured
here: no forwarding network (ADR-0083 priced forwarding the executor's result to every operand reader
at 12.9% of suite cycles and declined it for missing 12 MHz), and no bitmanip extension at all, where
Zba's own `sh2add` was measured, built and declined for an unrelated reason — a completeness-checker
limitation, not a cost (ADR-0131) — at **+110 packed cells** against `FIT_MAX_LC`'s roughly 117 cells
of headroom on this tree.

**What this ADR cannot state is how much of the gap CoreMark would show is instruction count against
cycles-per-instruction**, because that needs the cycle factor, and the cycle factor is not built.
Hazard3's `--stalls`-shaped accounting has no equivalent here to read against this core's own
(ADR-0136's `--stalls` table), so "how much of a CoreMark gap is missing bitmanip, how much is
missing forwarding, how much is everything else" stays a question this ADR can only name, not
answer.

## What is not done, and why it is not rushed

**CoreMark's cycle factor needs Hazard3 to be timed by something other than itself.** The approach
that makes both this core's and VexRiscv's comparable figures possible — the program reading its own
interval with no runner involved (`mcycle`/`minstret` here, the same for VexRiscv's own CSR-free
configuration handled instead by `soc/compare/dhry_tb.v`'s testbench-side marker count) — has a
precedent this repository already ships for exactly Hazard3's problem: `soc/compare/dhry_port.c`
stores a marker to a fixed RAM word immediately before and after the measured loop, for no reason
other than that VexRiscv's own configuration here has no CSR file either, and
`soc/compare/dhry_tb.v` counts cycles between the two markers on each core's own bus. **The same
shape closes Hazard3's CSR_COUNTER=0 gap too, harness-side, changing neither core** — that is the
preferred approach this ticket named, and it is not merely plausible, it is already built once in
this tree for the identical reason.

It is not built for CoreMark in this pass because CoreMark is EEMBC's much larger benchmark —
ADR-0136 measured 55.2M cycles for 100 iterations on this core alone, over 100× a single Dhrystone
run — and getting a first cross-core placement plus a real anti-fraud gate working in this session's
remaining time left no safe margin to also stand up a `coremark_tb.v`, a matching
`coremark_compare_port.c`, and an RV32IMA-common linker script, run it correctly the first time, and
trust the number that came out. **A wrong cross-core figure is worse than none — that is this
repository's stated reason for declining the raw comparison in ADR-0136, and it applies exactly as
much to a rushed one here.**

The concrete follow-up, so it does not have to be re-derived: a `soc/compare/coremark_compare_port.c`
mirroring `soc/compare/dhry_port.c`'s split (a two-word `.coremarkctl` window instead of `mcycle`,
`start_time()`/`stop_time()` publishing to it rather than reading a CSR); a `coremark_tb.v` mirroring
`dhry_tb.v` (two DUTs — `bench_littlecpu` and `bench_hazard3`, not the three-way `bench_tb.v` uses,
since Hazard3's own three-way smoke agreement is already covered above and CoreMark is a much longer
run to add a third core to for free); a linker script and `COMPARE_COREMARK_CFLAGS` at the two cores'
common ISA (RV32IMA — Hazard3's iCE40 build has no C, unlike the Dhrystone comparison's RV32IC, which
both cores here support); and enlarged ROM/RAM the way `dhry.lds` already enlarges past what
`compare-timing`'s own hx8k placement can hold, since CoreMark's image does not fit either core's 4 KB
comparison-harness ROM. `soc/compare/dhry_fit.py`'s block-RAM arithmetic is the template for whatever
prints the equivalent caveat.

## Consequences

- `make compare-timing COMPARE_CORE=hazard3` is a third working point beside `littlecpu` and
  `vexriscv`; `soc/compare/sweep.sh COMPARE_CORES='littlecpu hazard3'` runs the paired sweep this
  ADR's five-seed look should be replaced with before the clock figure is treated as settled.
- `make compare-smoke` now runs three cores in one simulation and requires all three to agree,
  extending the existing anti-fraud check rather than adding a parallel one -- and it is why the
  write-data-timing defect above is a footnote here rather than a wrong number in this ADR's own
  table.
- No `rtl/` file changed. This is entirely `soc/compare/`, the same boundary ADR-0086 and ADR-0098
  already draw around this kind of measurement.
- **The CoreMark cross-core product ADR-0136 was written to eventually support still does not
  exist.** This ADR is the vendoring and the clock half of it; the cycle half, and the product that
  needs both, are the next ticket's, not a number this one invented to look finished.
