# ADR-0257: A machine software interrupt is an `msip` word per hart, and it outranks the timer

**Status:** Accepted · 2026-10-26. Measured on ADR-0256's tree plus this one, yosys 0.68+48 and
nextpnr 0.11-1-g62e659ed (OSS CAD Suite 20260811), one Apple-silicon host. Builds on ADR-0256 (the
vectored target) and ADR-0255 (the board top's cell headroom).

## Decision

`mie.MSIE` (bit 3) is writable and `mip.MSIP` (bit 3) reads the platform's `msip` line for the hart;
the cause is `0x8000_0003`. The software interrupt outranks the timer when both are enabled and
pending, the spec's order, and under a vectored `mtvec` it enters at BASE + 0xC.

`msip` lives in `rtl/timer.v`: bit 0 of the word at `0x0002_0018 + 4 * hart`. The timer's window was
four words at one hart and eight at two, and the map already reserved eight; it is now eight words at
both (`mtime` at 0 and 1, one `mtimecmp` per hart from 2, one `msip` per hart from 6), so the UART
still starts at `0x0002_0020`. Only bit 0 is stored, a store without byte 0 does not write, and a
read right behind a store sees the new value. The words are not a CLINT's. On a single-hart SoC a hart
interrupts itself, which is what `test/asm/msip.S` drives; `test/dual/ipi.S` has each hart interrupt
the other under `make dual-smoke`.

`rtl/csrs.v` gains the `irq_software` input and an `interrupt_software` output, `irq_software &&
mie.MSIE`, which `rtl/executor.v` reads to pick the cause and the vector offset (3 against 7, ORed
into BASE[5:2]). The formal harnesses tie `irq_software` low beside `irq_timer`.

## What was priced

Board top at the six instance names of ADR-0255, `--pack-only`, same toolchain, side by side:

| tree | soc | core | cpu | u_soc | dut | s0 | worst |
|---|---|---|---|---|---|---|---|
| ADR-0256's tree | 5,186 | 5,176 | 5,176 | 5,158 | 5,176 | 5,203 | 5,203 |
| this tree | 5,058 | 5,178 | 5,178 | 5,080 | 5,178 | 5,078 | 5,178 |

The worst name leaves 102 cells, against 77. The feature did not save cells: the per-name counts are a
mapping hash (ADR-0255), and this tree's spread is wider (120 against 45). `make fit` reads 4,403
against the budget of 4,462; `make soc-timing` packs 5,100 cells at 14.48 MHz and `make board-timing`
5,058 at 12.73 MHz, both above 12.0 at their stale pins.

Timing, sixteen paired seeds (`default`, 1 to 15), `soc/paired_sweep.sh`, worst / median / best ns:
ADR-0256's tree 81.40 / 77.30 / 75.13 (12.29 / 12.94 / 13.31 MHz); this tree 79.55 / 74.66 / 71.20
(12.57 / 13.39 / 14.04 MHz), -2.3% / -3.4% / -5.2%, inside the churn band, so a null and not a speedup.

## Graders

`test/asm/msip.S` (set, take, clear, the vectored entry), `test/asm/msipmask.S` (both masks and the
priority over the timer), `test/timer_tb.v` (the word), `test/csr_tb.v` (`mie`, `mip` and the source
choice), `test/executor_tb.v` (cause 3 and the BASE + 0xC offset), and `formal/traps.sv` (cause and
target after a software entry, with the new `mie`/`mip` bits), whose forced-red direction is the
`software-as-timer` case of `formal/traps-vector-probe.py`. `software-below-timer` in
`test/MUTATION_DETECTORS` pairs the priority with `csr_tb` and `msipmask.S`.

Sail's CLINT keeps `msip` elsewhere and its `mip.MTIP` is already up, so both programs disagree at their
first `mip` read (`DISAGREE`, not the timer programs' `INCONCLUSIVE SAIL-LIMIT`); the manifest records why.
