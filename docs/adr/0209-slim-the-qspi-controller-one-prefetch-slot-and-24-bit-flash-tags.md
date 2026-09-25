# ADR-0209: slim the QSPI controller -- one prefetch slot and 23-bit flash tags

**Status:** Accepted · 2026-09-24

## Context

The full nano chip does not fit a 4x2 Tiny Tapeout tile. `nano_qspi_ctrl` is 15,731 um2
local (unflattened `stat -liberty`, 2026-09-25) against the brief's ~5,500 um2 estimate
-- the largest block after the core -- and two candidate cuts were cheap enough to
measure in parallel with the tile-size question: a two-slot prefetch queue down to one
slot, and 31-bit flash parcel tags down to the 23 bits the flash window actually needs.

## Baseline, re-taken on this tree rather than trusted from an earlier session

`make nano-area`: 78,382.7 of 80,500.0 um2. `make nano-timing`: flops 83,145.99 um2 /
13,459.41 ps, latches 72,269.31 um2 / 12,170.71 ps. The pin-level harness
(`NANO_DHRY_RUNS=5 NANO_DHRY_CYCLES=100000000 make nano-qspi-pins-dhrystone`,
`NANO_COREMARK_ITERATIONS=1 NANO_COREMARK_CYCLES=40000000 make nano-qspi-pins-coremark`):
25,507.6 cycles/Dhrystone, 25,263,609 cycles/CoreMark iteration -- matching the numbers
already on record, so nothing had drifted.

## Cut 1: one prefetch slot instead of two

The shipped queue held two parcels, each with its own address tag, comparator and
16-bit buffer, so a sequential fetch that outran what was cached still found its next
parcel waiting. One slot can hold only the parcel just fetched: a hit still answers in
one cycle when that parcel is enough (a compressed instruction), but a hit needing a
second parcel now streams it directly into the completing transaction (a new
`second_parcel_pending`/`second_parcel_first_data` pair) rather than finding it already
cached, and the next sequential parcel is never fetched ahead of being asked for.
Invariant 3 (the prefetch buffer's contiguity) is restated for one slot: `slot0_valid`
implies `stream_next_addr - slot0_addr == 1`, dropping the paired slot1 equalities
entirely rather than weakening them.

| | area (`nano-area`) | flops delay/area (`nano-timing`) | latches delay/area | Dhrystone cyc | CoreMark cyc |
|---|---|---|---|---|---|
| baseline | 78,382.7 | 13,459.41 ps / 83,145.99 | 12,170.71 ps / 72,269.31 | 25,507.6 | 25,263,609 |
| one slot | 76,101.7 | 14,816.33 ps / 79,092.11 | 17,647.18 ps / 71,427.25 | 24,757.0 | 24,187,279 |
| delta | -2,281.0 (-2.91%) | +1,356.92 ps / -4,053.88 | +5,476.47 ps / -842.06 | -750.6 (-2.94%) | -1,076,330 (-4.26%) |

Retire counts are unchanged (Dhrystone 13,944, CoreMark 777,270): this is a real cycle
count, not a different program. The two-slot queue always fetched one parcel further
ahead than a request needed, in case the next request was sequential; losing that on a
straight run of 4-byte instructions costs an extra idle-to-stream bounce per
instruction, but Dhrystone and CoreMark's actual mix -- compiled RVC code, heavy on
compressed instructions and redirects -- pays more for the two-slot queue's wasted
look-ahead parcel than it saves on the runs where the look-ahead hits, and nets
**fewer** cycles with one slot, not more. `nano-timing`'s pre-layout ABC delay estimate
disagrees, reading worse by 10-45% depending on register-file build; that figure is
explicitly "a ranking proxy... never placed, routed or signed off" and not a gate, and
it moves in the opposite direction once combined with cut 2 (below) -- read as evidence
that this instrument's delay figure does not track this kind of edit reliably, not as a
reason to decline a cut that wins on both of the real, gated measurements.

`make -C nano/formal components_qspi` re-proves clean: `qspi-probe.py`'s `queue-addressing`
mutation is re-anchored on the one remaining slot's own address assignment (tagging it
one parcel ahead of what it received), and still fails the mutant while passing the
shipping controller. `nano/tb/nano_qspi_resume_probe.sh` is re-anchored on the new
second-parcel resume branch (shrinking its own nibble count by one) for the same
reason. `nano-qspi-pins-test` passes both legs, program by program, with the exact
retire counts `nano/asm/OBSERVED_FLOOR` already states.

**Landed.** The area and cycle savings are both real and both in the same direction;
nothing about the design gets slower on a measurement this repo gates.

## Cut 2: 23-bit flash parcel tags

The flash window is 0x0000_0000-0x00ff_ffff (16 MiB); a parcel is 2 bytes, so a parcel
tag needs 23 bits, and the shipped 31-bit tags carried eight bits that could never
differ. `FLASH_WINDOW_BYTES` is a new parameter (default the 16 MiB window) guarded by
an elaboration check -- `nano_qspi_ctrl`'s own `FLASH_TAG_BITS` localparam is fixed at
23, and a window too large for that tag width is refused at elaboration, the same shape
as `rtl/imemory.v`'s `ROM_WORDS` check. `nano/tb/nano_qspi_window_probe.sh` is the
forced-red probe: the shipping 16 MiB window elaborates in both iverilog and yosys, a
32 MiB override is refused in both, and it is wired as `nano-qspi-window-test` on
`make test`'s path.

| | area (`nano-area`) | flops delay/area | latches delay/area | Dhrystone cyc | CoreMark cyc |
|---|---|---|---|---|---|
| baseline | 78,382.7 | 13,459.41 ps / 83,145.99 | 12,170.71 ps / 72,269.31 | 25,507.6 | 25,263,609 |
| 23-bit tags | 77,793.4 | 13,800.05 ps / 81,440.61 | 13,290.67 ps / 69,462.87 | 25,507.6 | 25,263,609 |
| delta | -589.3 (-0.75%) | +340.64 ps / -1,705.38 | +1,119.96 ps / -2,806.44 | 0 | 0 |

Narrower tags touch only bit width, not control flow, so this cut is exactly free in
cycles -- both benchmarks read identical cycle counts to the baseline. The pre-layout
delay estimate again reads worse in isolation and again reverses once combined with
cut 1. `make -C nano/formal components_qspi` re-proves clean; invariant 3's statement
is unaffected by the tag width, only narrower.

**Landed.** A pure area win at zero cycle cost, gated by a new elaboration check with
its own forced-red probe.

## Combined

| | area | flops delay/area | latches delay/area | Dhrystone cyc | CoreMark cyc |
|---|---|---|---|---|---|
| baseline | 78,382.7 | 13,459.41 ps / 83,145.99 | 12,170.71 ps / 72,269.31 | 25,507.6 | 25,263,609 |
| both cuts | 75,067.0 | 12,814.26 ps / 79,725.21 | 11,522.86 ps / 67,672.40 | 24,757.0 | 24,187,279 |
| delta | -3,315.7 (-4.23%) | -645.15 ps / -3,420.78 | -647.85 ps / -4,596.91 | -750.6 (-2.94%) | -1,076,330 (-4.26%) |

The combined area saving (-3,315.7) is larger than the two cuts' deltas summed
(-2,870.3): yosys shares some of the narrowed-tag and single-slot logic across the two
edits that it could not share when either shipped alone. The pre-layout delay estimate,
worse for each cut taken individually, reads better than baseline once both land --
consistent with this repo's standing note that redundant source text is not redundant
hardware and predicts nothing about the period on its own. `make -C nano/formal
components_qspi`, `nano-qspi-resume-test`, `nano-qspi-latency-test` and
`nano-qspi-pins-test` all stay green on the combined tree, both legs agreeing program
by program with the retire counts unchanged from before either cut.

`NANO_MAX_UM2` moves 80,500 -> 77,000 in the same commit, the same ~1,933 um2 headroom
style the prior step used, over the fresh 75,067.0 um2 measurement rather than the
figure this cut started from.

## Scope

Nothing here touches `nano/nano.v`, `nano/formal/`, `nano/nano.mk`'s NANO_LATCH_RF
machinery, or `test/probe_gates.sh` -- a concurrent ticket removes the latch register
file and owns those files. `psram_addr_pending` is left at its existing 31-bit width:
the brief scoped this cut to the flash tags, and PSRAM's own address handling is a
separate, unmeasured question.
