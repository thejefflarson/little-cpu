# ADR-0249: nano's machine timer is a bus device, and MTIP is registered

**Status:** Accepted · 2026-10-04 · amends ADR-0205 (`mip.MTIP` and `mie.MTIE` were read-only zero) and fills the span ADR-0206 reserved at `0x1080_0010` · amended 2026-10-10: `mtime` is `mcycle`

## Context

The owner overturned removing the timer on 2026-09-20. nano's brief had cut `mtime` to save about 128
flip-flops, estimated at roughly 4k µm², when the target was a 2×2 tile, and ADR-0205 followed it:
`mip.MTIP` and `mie.MTIE` read zero. Without a timer nano can be interrupted from outside and cannot
wake itself: no scheduler tick, no timeout, no watchdog unless something off-chip drives `ui_in[7]`.

littlecpu already has this device (`rtl/timer.v`, ADR-0082, ADR-0118). This ADR records what nano's
version copies, what it does differently, and what it costs.

## Amendment, 2026-10-10: `mtime` is `mcycle`

**The owner chose to alias `mtime` to `mcycle`, so nano has one 64-bit counter, not two.** The version
above gave the timer a counter of its own. That tree did not route at 4×2. Hardening run
37228120245 (the timer with its own counter) hit the six-hour limit in detailed routing with about
38,000 violations. Run 38003370567 (the same timer with its registers loaded through one byte-select
mask) hit it too, with violations falling from 52,120 to 43,537 and nowhere near zero. An experiment
branch that also registered `take_trap`'s decode half (run 37372430748) did route, with DRT, Magic
DRC and LVS at zero and one antenna violation (met1 side-area ratio 430.67 against 400, on the net
`core.cfunct3[1] | core.cfunct3[0]`), so it never met the "all zero" bar, and it leaned on an unrelated
change. The owner's rule that `mtime` stays a counter of its own was lifted by the owner after those runs.
`mtime` is still 64-bit; it is no longer separate storage.

What the hardware does now:

- A load from `mtime` or `mtimeh` returns the core's `mcycle` word. A store to either word merges its
  byte lanes into the same half of `mcycle`. A CSR write to `mcycle` or `mcycleh` moves `mtime`
  in the same cycle, since they are one register. A CSR write wins over a bus store in the one cycle
  both could land, which cannot happen: one instruction executes at a time.
- `mtimecmp` is still its own 64-bit register in `nano_timer`. `mtip` still registers the
  comparison, now of `mcycle` against `mtimecmp`, so it posts a cycle late and never early, and
  ADR-0118's rule is unchanged.
- `nano_timer` holds no counter and no incrementer. It takes `mcycle` as an input and raises `mtime_wr`
  for a store to either `mtime` word. The core takes `mtime_wr` as an input and exports `mcycle` as
  `mtime`; it picks the half from `mem_addr[2]` and the lanes from `mem_wstrb`. Every riscv-formal
  harness ties `mtime_wr` low, and the platform's timer is outside the oracle's reach anyway.

**Firmware-visible fact: writing `mcycle` moves `mtime`, and so moves every pending timer
comparison.** Firmware must not write `mcycle` or `mcycleh` while the timer is armed: a write that
raises the count past `mtimecmp` posts MTIP at once, and one that lowers it postpones the
interrupt by the difference. A store to `mtime` has the same effect on `mcycle`, so a profiler that
reads `mcycle` sees it jump. The tick suspension in fact 1 below is unchanged: the cycle of a write
does not increment.

**nano has no `mcountinhibit`, so the counter never pauses.** `mtime` therefore ticks in every
state, including while the core waits on memory, as `mcycle` does.

**Area.** `make nano-area` on this tree reads 75,833.2 µm² (soft logic 60,088.9, sequential
12,719.7, macro 15,744.4), 3,464.6 below the timer with its own counter (79,297.8) and 6,189.6 above
no timer (69,643.6). The timer's 65 remaining flip-flops are `mtimecmp` and `mtip`. The saving is
smaller than the 4,817 the Declined note priced, because the stores into `mcycle` need their own merge
mux. `NANO_MAX_UM2` moves from 81,400 to 77,900, keeping 2,066.8 µm² of headroom (2,102 before).

**This is a clean fit.** Hardening run 38046468060 (head 62543f5) at 4×2, AREA 2, full flow, on the tree this
amendment describes:

| | this tree (run 38046468060) | ADR-0248 baseline, no timer |
|---|---|---|
| detailed-routing DRC, Magic DRC, LVS | 0, 0, 0 | 0, 0, 0 |
| antenna | 0 nets, 0 pins | 0 |
| gate-level simulation (`nano-gl-test`, own routed netlist) | success | success |
| utilization | 63.8% (standard cells 59.6%) | 58.7% |
| wirelength | 440,269 | |
| worst setup slack, tt (nom / min / max) | +0.96 / +1.17 / +0.79 ns | +1.74 ns |
| worst setup slack, ff | +3.98 / +4.14 / +3.88 ns | |
| worst setup slack, ss | −10.09 / −9.51 / −10.62 ns, 509 violations at max_ss | −10.11 ns at max_ss |
| hold, worst | +0.10 ns | |
| max_ss slew / cap violations | 4,331 / 30 | |

Against ADR-0248: max_ss is 0.5 ns worse (−10.62 against −10.11), and the tt worst slack is 0.95 ns
tighter (+0.79 against +1.74). Both sit inside the slow-corner miss ADR-0248 accepted, and nominal and
fast still close.

Two harness decisions the owner accepted with the alias:

- **`mtime_wr` is an allowed constant in the formal tie-off.** Every riscv-formal harness ties it low
  because none has a bus to raise it; the store path into `mcycle` is graded by `nano_timer_tb.v` and
  `mtimealias.S`. `formal/check-interrupt-tie-off.py` names it in `ALLOWED_CONSTANTS`, the only constant
  besides the interrupt a harness may hold, and `docs/manifests/interrupt-tie-off.md` records why.
- **The testbench timer is at `0x0001_3ff0`**, the last 16 bytes of the RAM window. A store to the timer
  now writes `mcycle`, and at the old `0x0001_0010` the CoreMark and Dhrystone data stores landed on it
  and corrupted the cycle counter (`nano-qspi-control-test` failed with verdict 3). `nano.lds` pins
  `.mtimer` there and sets `__stack_top` to it; `dhry.lds` and `coremark.lds` stop their stacks 16 bytes short.

The sections below describe the first version; where they say `mtime` has its own counter, this amendment
replaces them.

## Decision

**Where `mtime` and `mtimecmp` live: on nano's bus, in the span ADR-0206 reserved.** The privileged
spec's section on the machine timer registers makes `mtime` a memory-mapped machine-mode register
with 64-bit precision on every RV32 and RV64 system, and `mtimecmp` a 64-bit memory-mapped
register beside it. Neither is a CSR in the base spec, so reaching them through a CSR address
would put the registers where the spec says they are not. The chip top exists now (`nano/tt/src/tt_um_thejefflarson_nanocpu.v`, `nano/bus.v`) and
the map already held sixteen bytes for them, so nothing about the map moves:

```
0x1080_0010   mtime low        0x1080_0018   mtimecmp low
0x1080_0014   mtime high       0x1080_001c   mtimecmp high
```

`nano/timer.v` (`nano_timer`) is a fourth peripheral beside the UART and GPIO. `mtime` and `mtimecmp`
are 64-bit, as the spec requires of RV32; a narrower counter is a deviation. The core's
`RAM_WORDS` derivation already covered the span (`MAP_TOP` counted the sixteen bytes), so the core's
load/store fault window is unchanged and `nano/memmap_test.sh` now reads the timer's own `BASE`
instead of a reserved constant.

**`mip.MTIP` is the comparison and `mie.MTIE` is writable.** The core takes one new input,
`irq_mtip`, straight from the timer's register: it is already on `clk`, so unlike `irq_meip` it
needs no synchronizer. `interrupt_pending` is `(MEIP && MEIE || MTIP && MTIE) && mstatus.MIE`, and
the entry cause is `0x8000_0007` for the timer. When both are pending the cause is
`0x8000_000b`: the spec ranks the external interrupt above the timer, and the timer, a level, is
taken on the `mret`. The take is the existing one, at `fetch_instr`, so `rvfi_intr` and the
`traps.sv` oracle cover it with no new mechanism.

**Three facts are the platform's to state; firmware cannot derive them.**

1. `mtime` ticks once per clock cycle: 15.625 ns at the shipped 64 MHz. A write to either half
   suspends that cycle's tick so no carry crosses a half-written value. Reading 64 bits takes two
   loads; firmware that cannot tolerate a carry between them reads high, low, high.
2. MTIP is a level, held while `mtime >= mtimecmp`. A handler that returns without moving
   `mtimecmp` is re-entered before the instruction at `mepc` runs (`mtimer.S` proves it with three
   entries).
3. An RV32 `mtimecmp` update is the spec's three stores in the spec's order: low all-ones, high,
   low. Written high first, `mtimecmp` passes through a value at or below `mtime` and posts a
   spurious interrupt. `mtimerorder.S` fires that interrupt on purpose, from `{1, 0x10}` against an
   `mtime` of `{1, 0x50}`, and shows the spec's order does not.

**MTIP posts a cycle after the comparison holds, never before.** `mtip` registers the comparison of
the registered counters, as littlecpu's does, and ADR-0118's rule applies: a change in the comparison
may reach MTIP late and never early. Two consequences are firmware-visible. After a store that moves
`mtimecmp` past `mtime`, MTIP still reads the old level at the very next instruction boundary, so an
interrupt that was legitimately posted the cycle before the store can be taken once after it. And a
`csrr mip` after a store reads the new level, since its read comes at least three cycles after that
boundary. The comparison is not combinational because a 64-bit compare in front of `take_interrupt`
would sit on a path nano already misses at the slow corner (ADR-0248).

**`mtimecmp` resets to zero, so MTIP is up out of reset, and that is harmless.** `mie.MTIE` and
`mstatus.MIE` both reset to zero, so nothing is taken until firmware writes `mtimecmp` and sets both.
This is ADR-0082's property kept, not waived. Firmware reads `mip.MTIP` as 1 until it writes
`mtimecmp`. `mtimer.S` checks MTIP up and MTIE clear first, then that MTIP up with each of the
other two enables clear takes nothing.

**Worst-case interrupt response.** The take happens at `fetch_instr`, so the response is what the
instruction in flight has left, plus the register on MTIP, plus the take cycle. Every instruction
takes four states (`fetch_instr`, `ready_instr`, `fetch_rs1`, `execute_instr`) and a load or
store a fifth; M is cut here, so no divide stretches one. From the cycle `mtime >= mtimecmp` first
holds, MTIP shows the next cycle, the longest wait is an instruction that has just passed its
boundary, and the take lands **at most 5 cycles after the condition at zero wait states**; the
handler's first fetch is issued 2 cycles after that, 7 in all. A sixty-interrupt sweep at varied
phases in a zero-wait simulation measured 1 to 5, so the bound is reached. With the pin-level QSPI
controller the in-flight instruction's memory wait adds to it: the same sweep measured up to 73
cycles with the simulation's 24-cycle redirect preamble and 44-cycle PSRAM load, which is a sample of
sixty phases and not a bound. What sets the worst case is the longest memory transaction, never the
timer.

**F and G do not move.** `make -C nano/formal remeasure-fg` reads F = 11 and G = 9, declared 11 and 9.
Every riscv-formal harness ties both interrupt inputs low, so the timer adds no retire gap to the
traces those depths come from; `traps.sv` leaves both free.

## Area

`make nano-area`, soft logic plus the `rf_top` macro's fixed footprint, the same instrument as
ADR-0225:

| | µm² |
|---|---|
| before | 69,643.6 (soft 53,899.2, sequential 11,315.9, macro 15,744.4) |
| with the timer | 79,297.8 (soft 63,553.5, sequential 14,081.0) |
| delta | +9,654.2 (sequential +2,765.2) |
| `nano_timer` alone | 10,440.0, 1,025 cells, 129 flip-flops |

That is 2.4 times the brief's estimate. The estimate priced the counters' flip-flops; the 129
flip-flops are 2,765 µm² of the delta, and the 64-bit incrementer, the 64-bit comparator, the hold
multiplexer on every register bit and the read multiplexer are the other 6,889. `NANO_MAX_UM2` moves
from 71,700 to 81,400, keeping the 2,102 µm² of headroom the last step left (2,056).

**The first version's fit was not claimed, and did not come.** The tile is about 18,100 µm² (the macro's
15,744 µm² is 87% of one, ADR-0225), so 4×2 is about 145,000 µm². The timer with its own counter added
9,654 µm² locally, and its two hardening runs never finished routing; the amendment above records the
aliased design's clean run.

## Declined

- **Alias `mtime` to `mcycle`.** *Taken by the amendment above, after the routing runs it names.* The largest lever: `nano_timer` without its counter and incrementer
  synthesizes to 5,622.9 µm², a saving of at least 4,817 (writes to `mtime` not built, which would
  add a second source on `mcycle`). Declined on conformance: `mcycle` is software-writable, so any
  firmware that clears it for profiling would move `mtime` and with it every pending `mtimecmp`
  comparison. If the fit fails, this is the first thing to price against the owner's rule that
  `mtime` stays a 64-bit counter of its own.
- **A CSR path to the registers.** The spec says memory-mapped, and the bus had the span.
- **Resetting `mtimecmp` to all ones.** MTIP would be low out of reset, but the platform would differ
  from littlecpu's for nothing, and the enable-reset argument already makes zero harmless.

## What grades it

- `nano/asm/mtimer.S` and `nano/asm/mtimerorder.S`, on both simulator legs and both memory systems
  (flat, and the pin-level QSPI leg), floors 125 and 144 retires. Their retire counts differ between
  memory systems because the programs spin until an interrupt arrives, so the floors sit under the
  smaller. The dual-leg runs now give each program 10,000 cycles instead of 5,000: the pin-level leg
  spends about 45 cycles per retire.
- `nano/asm/mtimealias.S` writes `mcycle` and `mcycleh` and reads `mtime` back, then stores to
  `mtime` and reads `mcycle` back, checking the untouched half each time and the carry across the
  halves; floor 40 retires.
- `nano/tb/nano_mtimer_probe.sh` forces ten mutants red against those three programs: the
  cause code, external-over-timer priority, MTIE gating, MTIE's write bit, `mip.MTIP`, a dead
  comparator, a low-word-only one, a store that never reaches `mcycle`, one that lands in the wrong half,
  and a window that reads the halves swapped.
- `nano/tb/nano_timer_tb.v` grades MTIP against a model: early is an error, late by more than one
  cycle is an error, and it covers the 64-bit carry, the wrong-order transient, byte strobes, the
  window's edges. A stand-in for the core's `mcycle`, written beside the bench's independent model, takes
  CSR writes and bus stores, and the model checks both land. `nano/tb/nano_timer_probe.sh` forces eight
  mutants red, among them MTIP one cycle early and a store that never reaches `mcycle`.
- `nano/tb/asm/tt_gpio_uart.S` takes a timer interrupt through the chip top at the real address,
  and `nano/tb/nano_tt_timer_probe.sh` forces the top's `irq_mtip` wire and the bus's timer read red.
  The flat-memory testbench has no map, so `nano_testbench.v` places the timer in the RAM window at
  the linker script's `.mtimer` block, the last 16 bytes of the RAM window, and `nano.lds` asserts that address. That address moved from `0x0001_0010` with the alias: a store there now writes `mcycle`, and the Dhrystone and CoreMark scripts had data at the old one, so both benchmarks' stacks stop 16 bytes short of the block.
- `nano/formal/traps.sv` asserts that the first retirement after an interrupt reports one of the two
  interrupt causes, with `traps-mcause-probe.py` showing a wrong timer cause reachable and red.
  `formal/check-interrupt-tie-off.py --core nano` now grades `irq_mtip` beside `irq_meip`.
- `nano/memmap_test.sh` requires the timer to abut GPIO, to be 16-byte aligned, and the core's window
  to cover it; `nano/memmap_probe.sh` forces both red.
