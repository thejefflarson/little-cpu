# ADR-0206: nanocpu gets a tt_um top, UART, GPIO and a memory map

**Status:** Accepted · 2026-09-20

## Context

Nothing in `nano/` was a chip yet. `nano/tt/src/tt_um_thejefflarson_nanocpu.v` was a
measurement wrapper that shifted the core's internal bus out over the pins so the
Tiny Tapeout hardening flow had something to run; it drove no peripheral and decoded
no address. This lands the real top: `riscv` (`nano/nano.v`), the QSPI front end
(`nano/qspi.v`) behind a new address router (`nano/bus.v`), a transmit-only UART
(`nano/uart.v`) and a GPIO pair (`nano/gpio.v`), wired to Tiny Tapeout's fixed
`ui_in`/`uo_out`/`uio_*` port set.

## Provenance

Every design call below was either put to the owner or was not; this ADR names
which, because a prior ticket's "owner has pre-decided" line that no owner had
actually said had to be reverted out of the placed SoC.

- **MEIP on `ui_in[7]`**: an assistant's choice, raised and confirmed by the owner
  on 2026-09-21.
- **No `mtime`/`mtimecmp` in this change**: the owner overturned removing the timer
  on 2026-09-20; a sibling ticket restores it. `mip.MTIP` is left exactly as the
  CSR layer already has it (read-only zero, ADR-0205) — not touched here — and this
  memory map reserves the sibling ticket's address space (below) so it lands without
  a reshuffle.
- **Everything else — QSPI on `uio[7:0]`, UART on `uo_out[0]`, GPIO on the rest, the
  map's shape and every address in it — is this assistant's choice, from the
  brief, and has not been put to the owner.** Flagged for review below.

## The clock is a parameter

nano does not close timing at any corner yet, so `CLOCK_HZ` is declared once, as a
`localparam` in `tt_um_thejefflarson_nanocpu`, and the UART's divisor
(`nano/uart.v`'s `DIVISOR = (CLOCK_HZ + BAUD/2) / BAUD`) is the only thing that
derives from it. The value used to measure area and to run the simulated tests
below is 64 MHz, carried over from the wrapper this replaces; it is not a claim
about an achievable rate on real silicon; that gets re-stated once a corner closes.
The area figure below is real at any clock — an untimed synthesis run — and is
quoted without that qualification.

## The pin map

*DECISION NEEDED — none of this section has been put to the owner.*

- `uio[0]` = QSPI `sck`, `uio[1]` = flash chip select, `uio[2]` = PSRAM chip
  select, `uio[3]` = spare chip select (unused, tied high inside
  `nano_qspi_ctrl`), `uio[7:4]` = the four QSPI data lines. This is exactly
  `nano_qspi_ctrl`'s eight signals, one per bit, in the order the controller
  already declares them.
- `uio_oe[3:0]` is constant 1: `sck` and the three selects are always outputs.
  `uio_oe[7:4]` equals `nano_qspi_ctrl`'s own `sio_oe`, turned around per QSPI
  phase (address/command vs. data) by the controller's existing state machine —
  nothing new was built for this, `nano_bus` only routes the signal that already
  existed to a pin.
- `uo_out[0]` = UART tx. `uo_out[7:1]` = GPIO out, 7 bits.
- `ui_in[7:0]` = GPIO in, 8 bits, and `ui_in[7]` is wired a second time straight
  into the core's `irq_meip` pin. Reading GPIO in bit 7 back is a harmless
  redundant view of the same physical pin, not a second input.

## Reusing littlecpu's UART

`nano/uart.v` starts from `rtl/uart.v` (8N1, transmit-only, a byte written while
busy is dropped) and is copied rather than instantiated directly, per CLAUDE.md's
note that nanocpu is never quoted against littlecpu on NUMBERS but sharing a
well-graded module is a choice, not a violation. The one substantive change: the
status register reads `busy` **combinationally** instead of registered one cycle
late. littlecpu's bus can absorb that lag; nano's picorv32-style valid/ready bus
cannot — asserting `mem_ready` the same cycle `mem_valid` is presented (the zero-
wait-state shape every peripheral here uses) with a registered status word would
hand the CPU last cycle's busy bit instead of this one's.

## The memory map

`riscv`'s own load/store region check (`RAM_BASE`/`RAM_WORDS`) is the only thing
that ever refuses an out-of-window access, and it refuses in decode, with the
address in `mtval` and cause 5 (load) or 7 (store) — the same refusal shape as
`nano_qspi_ctrl`'s own range test and littlecpu's memory map. That check tests one
contiguous span, so the chip's whole non-instruction address space has to be one
span, and `nano_bus` decides which sub-region inside it answers.

```
0x0000_0000 - 0x00ff_ffff   flash (instruction fetch only; no region check --
                              see "no cause 1" below)
0x1000_0000 - 0x107f_ffff   PSRAM, 8 MiB (nano_qspi_ctrl, load/store)
0x1080_0000 - 0x1080_0007   UART (nano/uart.v)
0x1080_0008 - 0x1080_000f   GPIO (nano/gpio.v)
0x1080_0010 - 0x1080_001f   reserved for mtime/mtimecmp
```

`riscv` is instantiated with `RAM_BASE = 0x1000_0000` and `RAM_WORDS` derived from
the same arithmetic that built this table (`(MAP_TOP - PSRAM_BASE) / 4`), so the
core's fault window and `nano_bus`'s routed window are the same span by
construction rather than by two numbers that happen to agree today.
`nano/memmap_test.sh` reads both back out of the RTL and refuses a region landing
inside another's span, or the core's window drifting from the union of the four;
`nano/memmap_probe.sh` is its forced-red prerequisite. An address inside the
window but outside every named region — the reserved span above — reads zero and
drops a write, the same way an unimplemented CSR does; that is deliberate room for
the sibling mtime/mtimecmp ticket, not a hole nothing decided.

**No cause 1.** `riscv` has never had an instruction-fetch region check (ADR-0205
already records this for the core alone): `nano_qspi_ctrl` always routes a fetch
to flash regardless of address, so an out-of-window fetch cannot land in `.text`
on this front end either. This ADR does not add one; the 16 MiB flash window above
is what the controller's own 23-bit parcel address can reach, not an enforced
bound.

## Area

`make nano-area`, re-taken over the real top in place of `nano/area_top.v`'s
synthesis-only pairing (now deleted — the real top has a fixed port list, so
`hierarchy -auto-top` needs no help picking it): **78,566.6 um2**, against the
CSR/trap layer's own 76,982.6 (ADR-0205). `NANO_MAX_UM2` steps 79,000 → 80,500,
keeping the same order of headroom this ratchet has carried at every step.

## Verification

`nano/tb/nano_tt_tb.v` drives the top by its pins alone — QSPI to pin-level flash
and PSRAM models (`nano/tb/nano_qspi_flash_model.v`,
`nano/tb/nano_qspi_psram_model.v`), a fixed GPIO-in pattern, and a bit-banged UART
receiver against the known divisor — never the core's internal bus, so a wiring
mistake in `nano/bus.v`, `nano/uart.v` or `nano/gpio.v` is visible here even where
the core-only suite is blind to it. `nano/tb/asm/tt_gpio_uart.S` writes a UART
byte, waits for it to drain, then echoes GPIO in to GPIO out and reports pass/fail
over the usual `tohost` word (rebased into the PSRAM model's own array by
`--adjust-vma`, since `nano_bus` subtracts `PSRAM_BASE` before addressing it).
`make nano-tt-test` runs it; `nano/tb/nano_uio_oe_probe.sh` is its forced-red
prerequisite, an X-leg check iverilog is the only leg that can run: the same
testbench flags `uio_oe` reading unknown on any cycle out of reset, and the probe
confirms that fires against a mutant top with one `uio_oe` bit left undriven
before trusting it on the shipping one.

## Out of scope

Hardening through the TT flow, Sail co-simulation of the new regions, gate-level
simulation and FPGA bring-up — all named out of scope by the ticket this lands.
