# ADR-0249: nano's machine timer is a bus device, and MTIP is registered

**Status:** Accepted · 2026-10-04 · amends ADR-0205 (`mip.MTIP` and `mie.MTIE` were read-only zero) and fills the span ADR-0206 reserved at `0x1080_0010`

## Context

The owner overturned removing the timer on 2026-09-20. nano's brief had cut `mtime` to save about 128
flip-flops, estimated at roughly 4k µm², when the target was a 2×2 tile, and ADR-0205 followed it:
`mip.MTIP` and `mie.MTIE` read zero. Without a timer nano can be interrupted from outside and cannot
wake itself: no scheduler tick, no timeout, no watchdog unless something off-chip drives `ui_in[7]`.

littlecpu already has this device (`rtl/timer.v`, ADR-0082, ADR-0118). This ADR records what nano's
version copies, what it does differently, and what it costs.

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

**This does not say the design fits.** The fit is a routed result and none was run. The tile is about
18,100 µm² (the macro's 15,744 µm² is 87% of one, ADR-0225), so 4×2 is about 145,000 µm², and +9,654
µm² is about 6.7 points of instance utilization on the last hardening run's 58.7%, if the flow
reproduces the local delta, which it has not for an earlier change (ADR-0219: −10k locally and about
−2.5k in the flow). What failed to route before was routing demand at 84.7% and 88.3%, and nano
routed at 54 to 57%. A hardening run at 4×2 on this tree decides, and "done" for it is routed, DRC,
LVS and antenna at zero, timing per corner and `make nano-gl-test` on its own netlist.

## Declined

- **Alias `mtime` to `mcycle`.** The largest lever: `nano_timer` without its counter and incrementer
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
- `nano/tb/nano_mtimer_probe.sh` forces seven timer-interrupt mutants red against those programs: the
  cause code, external-over-timer priority, MTIE gating, MTIE's write bit, `mip.MTIP`, a dead
  comparator and a low-word-only one.
- `nano/tb/nano_timer_tb.v` grades MTIP against a model: early is an error, late by more than one
  cycle is an error, and it covers the 64-bit carry, the wrong-order transient, byte strobes, the
  suspended tick and the window's edges. `nano/tb/nano_timer_probe.sh` forces seven mutants red,
  among them MTIP one cycle early.
- `nano/tb/asm/tt_gpio_uart.S` takes a timer interrupt through the chip top at the real address,
  and `nano/tb/nano_tt_timer_probe.sh` forces the top's `irq_mtip` wire and the bus's timer read red.
  The flat-memory testbench has no map, so `nano_testbench.v` places the timer in the RAM window at
  the linker script's `.mtimer` block, and `nano.lds` asserts that address.
- `nano/formal/traps.sv` asserts that the first retirement after an interrupt reports one of the two
  interrupt causes, with `traps-mcause-probe.py` showing a wrong timer cause reachable and red.
  `formal/check-interrupt-tie-off.py --core nano` now grades `irq_mtip` beside `irq_meip`.
- `nano/memmap_test.sh` requires the timer to abut GPIO, to be 16-byte aligned, and the core's window
  to cover it; `nano/memmap_probe.sh` forces both red.
