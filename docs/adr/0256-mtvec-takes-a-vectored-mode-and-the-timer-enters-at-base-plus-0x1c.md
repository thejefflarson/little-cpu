# ADR-0256: `mtvec` takes a vectored mode, and the timer interrupt enters at BASE + 0x1C

**Status:** Accepted · 2026-10-25. Measured on `main` at 0e402a7e plus this tree, yosys 0.68+48 and
nextpnr 0.11-1-g62e659ed (OSS CAD Suite 20260811), one Apple-silicon host. Builds on ADR-0214 (the
interrupt take lives in X) and ADR-0255 (the board top's cell headroom).

## Decision

`mtvec[0]` is MODE. MODE 0 sends every trap to BASE, as before. MODE 1 sends a synchronous exception
to BASE and an interrupt to BASE + 4 × cause, which for the machine timer (cause 7) is BASE + 0x1C.
MODE 2 and 3 are reserved: bit 1 reads zero, so a write naming them reads back as MODE 0 or 1, a legal
WARL value.

A MODE 1 write clears BASE[5:2], a 64-byte alignment the privileged spec leaves to the
implementation. With BASE aligned, `rtl/executor.v` forms the target by ORing a four-bit offset into
BASE[5:2], not by adding, and the same four bits reach causes 0 to 15 when a second interrupt arrives.
The offset is `interrupt_pending && mtvec[0]` and not `take_interrupt`: `redirect_target` is read only
with `in_valid` and `!x_busy` high, and there a pending interrupt is the one X takes.

## What was priced

Cells (`ICESTORM_LC`, local toolchain): `make fit` 4,419 against the Makefile's 4,422 (the `fit` job's
instrument) and a budget of 4,462; `make soc-timing` 5,166 against 5,177 on `main`.
The board top packs to the following at the six instance names of ADR-0255 (`soc`, `core`, `cpu`,
`u_soc`, `dut`, `s0`), `--pack-only`:

| tree | soc | core | cpu | u_soc | dut | s0 | worst |
|---|---|---|---|---|---|---|---|
| `main` | 5,164 | 5,168 | 5,168 | 5,179 | 5,168 | 5,165 | 5,179 |
| first spelling (MODE 2/3 write dropped by gating the CSR write enable) | 5,230 | 5,275 | 5,275 | 5,198 | 5,275 | 5,172 | 5,275 |
| shipped (bit 1 forced to zero) | 5,186 | 5,176 | 5,176 | 5,158 | 5,176 | 5,203 | 5,203 |

The first spelling dropped a reserved-MODE write as the reference model does, and left 5 cells at the
worst name; gating the write enable on `wdata[1]` was the cost, so the shipped spelling forces bit 1
to zero in the write mask instead. It leaves 77 cells at the worst name, against 101 on `main` here.

Timing, sixteen paired seeds (`default`, 1 to 15) on `soc/paired_sweep.sh`, `main` against this tree,
worst / median / best ns: `main` 79.39 / 74.78 / 73.62 (12.60 / 13.37 / 13.58 MHz); this tree 81.40 /
77.30 / 75.13 (12.29 / 12.94 / 13.31 MHz), +2.5% / +3.4% / +2.1%, inside `soc/bands.py`'s 3.6% churn
band. The first spelling read +3.1% worst and +4.1% median on the same seeds. A spelling that used
`take_interrupt` in the offset read +2.8% / +4.6%.

Both pins went PIN STALE on the source change and were re-taken at the 5% margin rule: `soc/pin.json`
holds seed 67306537 at 13.32 MHz (twelve draws 12.81 to 13.32, median 13.05), `soc/board_pin.json` seed
20740127 at 12.84 MHz (twelve draws 11.71 to 12.84, median 12.30; two draws are under 12.0, as before).

## Graders

`test/asm/mtvecmode.S` runs exceptions under both modes and agrees with Sail
(`test/sail/rv32imac_zicsr.json` now declares vectored mode supported with a 64-byte base).
`test/asm/mtvecirq.S` takes the timer under MODE 1 at BASE + 0x1C, with every other table slot a
jump to `fail`, and under MODE 0 at BASE; the model's timer is a CLINT at another address, so it is
`INCONCLUSIVE SAIL-LIMIT` and compares nothing, and it grades a reserved-MODE write for legality only
because the model keeps the old MODE where this core clears bit 1. `formal/traps.sv` asserts the
target after an interrupt entry, and `formal/traps-vector-probe.py` forces the interrupt to BASE and
requires that assertion's own line to fail (the basecase does not reach a vectored interrupt, so the
verdict is UNKNOWN, with induction failing at that line). `test/csr_tb.v` and `test/executor_tb.v` hold
the register and the target.
