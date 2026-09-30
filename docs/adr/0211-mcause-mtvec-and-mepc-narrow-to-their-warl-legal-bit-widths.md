# ADR-0211: mcause, mtvec and mepc narrow to their WARL-legal bit widths

**Status:** Accepted · 2026-09-25

## Context

`nano/nano.v` stored `mtvec`, `mepc` and `mcause` as full 32-bit flip-flop registers.
The privileged spec makes each of them a WARL field: an implementation is free to hold
fewer bits than 32 as long as it can represent every value the core actually needs to
report, so a value outside the implemented set is legalized (mapped to some supported
value) rather than stored verbatim. This is the same technique `nano/nano.v` already
uses for `mstatus` and `mie` — a handful of state bits plus a combinational read-side
reconstruction (`mstatus_value`, `mie_value`) — extended to the three trap-entry CSRs.
`mcycle`/`minstret` are out of scope: whether they can be read-only zero is a separate,
open question for the owner.

## Per-CSR table

| CSR | Bits before | Bits after | Spec sentence permitting the cut | `make nano-area` delta |
|---|---|---|---|---|
| `mtvec` | 32 (full register) | 30 (`BASE`, bits `[31:2]`); `MODE` read-only zero | "The BASE field in `mtvec` is a WARL field that can hold any valid address... In direct mode, all traps into machine mode cause the pc to be set to the address in the BASE field." Nano implements only direct mode, so `MODE` (bits `[1:0]`) needs no storage at all — it is legalized to `0b00` on every write, matching the WARL rule that an implementation need not implement a mode it does not support. | included below (measured together) |
| `mepc` | 32 | 31 (`[31:1]`); bit 0 read-only zero | "The low bit of `mepc` (`mepc[0]`) is always zero... If IALIGN=16, `mepc[1]` is writable." Nano has C (IALIGN=16), so only bit 0 is hardwired; bit 1 stays a real stored bit, unchanged from before this ticket. | included below |
| `mcause` | 32 | 5 (1 `Interrupt` bit + 4-bit `Exception Code`) | "When a trap is taken... `mcause` is written with a code indicating the event that caused the trap... The Interrupt bit... is set if the trap was caused by an interrupt." `mcause`'s `Exception Code` field is WARL and "need only be able to hold the supported exception codes." Nano can raise codes 2, 3, 4, 5, 6, 7 and 11 (the last also as the one interrupt, machine external, with the `Interrupt` bit set) — every one fits in four bits, so bits `[30:4]` are legalized to zero on any write and never stored. Reads reconstruct the full-width value (`mcause_value = {mcause_interrupt, 27'b0, mcause_code}`), which is exactly what a write of any implemented cause reads back as. | included below |
| `mscratch`, `mtval` | 32 | 32, unchanged | Both are plain read/write registers with no WARL field in the spec; nothing to narrow. | n/a |

`make nano-area` (the real `tt_um` top: `nano.v`, `qspi.v`, `uart.v`, `gpio.v`, `bus.v`),
measured on top of the QSPI controller's own slimming (ADR-0210, `NANO_MAX_UM2` stepped
to 77,000 there): **75,067.0 → 74,574.0 µm², −493.0 µm² (−0.66%)** against
`NANO_MAX_UM2` (`nano/nano.mk`), now stepped again to 76,500. The three CSRs together
drop 59 bits of flip-flop state (2 from `mtvec`, 1 from `mepc`, 27 from `mcause`); the
area move is smaller than a flop-count estimate alone predicts, matching the standing
finding that `dfflibmap`/ABC already prune a flip-flop whose D input is a
written-then-read constant, so some of this cut was already free before the RTL said
so explicitly — the win here is the narrower storage plus whatever ABC could not infer
through the CSR write mux on its own.

## Read-back after a write

Each CSR still reads back the full 32-bit value the spec requires: `mtvec_value`,
`mepc_value` and `mcause_value` are combinational reconstructions (`{mtvec_base,
2'b00}`, `{mepc_msbs, 1'b0}`, `{mcause_interrupt, 27'b0, mcause_code}`) read at every
site that used to read the bare register — `csr_rdata`'s mux, both `next_pc <= mtvec`
trap-entry arms, `next_pc <= mepc` on `mret`, and the `RISCV_FORMAL` debug outputs. A
software write to `mcause` naming an unimplemented code (anything above 11, or 8, 9,
10) reads back with the same `Interrupt`:`Exception Code` pair truncated to its low
four bits — a legal WARL legalization, since the field is defined as holding only the
codes an implementation supports.

## Verification

- `make -C nano/formal components_traps`: passes, with `traps-region-probe.py` and
  `traps-tval-probe.py` both still red for their own reason and the shipping core
  passing. This is the harness that checks a real trap's `mcause` against the exact
  expected code (`formal/traps.sv`'s `expected_cause`/`expected_tval` comparisons) — the
  narrowing is exercised on every one of its assertions, not just a subset.
- `make -C nano/formal check`: all 76 generated riscv-formal checks still pass against
  an empty `EXPECTED_FAIL`, matching `EXPECTED_CHECKS`. `mcause`/`mtvec`/`mepc` have no
  generated `csrw_*` check of their own — `nano/formal/checks.cfg`'s `[csrs]` section
  only names `mcycle`/`minstret` — so their WARL legalization is graded by
  `components_traps` above, not by a genchecks round-trip.
- `make nano-test`: 8/8 on both the cxxrtl and iverilog legs, agreeing program by
  program, with the same retire counts as before this change — `csrimm.S`, `meip.S`,
  `mul.S` and `divide.S` all exercise `mcause`/`mepc` directly (trap handlers reading
  `mcause`/`mepc` back after a real trap, `meip.S`'s external interrupt reading back
  `0x8000000b`).
- `make nano-littlecpu-test`: `csrset.S`, `zicsr.S` and `hpm.S` (the three programs the
  ticket named) still pass with unchanged retire counts (76, 241, 127).
- `make nano-timing`: runs to completion; no ratchet on this instrument.

## What was left alone

`mscratch` and `mtval` have no WARL field in the spec and are untouched. `mstatus` and
`mie` were already narrowed to individual bits before this ticket (ADR-0205). The
counters (`mcycle`/`minstret`) are explicitly out of scope — the ticket that opened
this one reserves that decision for the owner.
