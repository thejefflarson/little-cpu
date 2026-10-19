# ADR-0255: The UPduino board top places only on a second ABC pass, and `make board-timing` gates it

**Status:** Accepted · 2026-10-18. Measured on `main` at b995cb32 plus this tree, yosys 0.68+48 and
nextpnr 0.11-1-g62e659ed (OSS CAD Suite 20260811), one Apple-silicon host. Builds on ADR-0054 (the
timing report), ADR-0066 (12 MHz is a requirement), ADR-0121 (the churn band) and ADR-0171 (the
pinned placement).

## Context

`make bitstream` stopped placing `upduino_top` (`soc/board_upduino.v`): nextpnr packed 5,306
`ICESTORM_LC` against the part's 5,280 and stopped with `Failed to expand region`. Nothing noticed,
because `make soc-timing` places `littlesoc` alone (5,265 on the same tree, 15 cells under the part)
and no job placed the board top.

**The bisect.** Placing `upduino_top` at each commit: 5,222 at 62c85b38; 5,300 at 06d70ee6, the commit
that claimed Zkr (the four health tests, `mseccfg`, the ISA string), which is +78; 5,306 after the
8,192-cycle edge timeout. The wrapper itself adds about 40 over `littlesoc`: it ties `btn_n` and
`spi_miso` high, and a respelling of either tie moves the count by 44 to 76 cells.

## What the measurements say

**The count is a hash, with a spread near the whole deficit.** Renaming the one instance
`littlesoc soc` in the wrapper, which changes no logic, packs 5,238 to 5,306 cells over eight names
(mean of six: 5,279). So a respelling that saves "a few cells" is a draw, and the ticket's 26-cell
target is inside the noise. `rtl/trng.v` offered no saving to spend: widening `ticks` to 13 bits in place of `late` (and reading
`samples[0]` for `have`) synthesised alone to 96 `SB_LUT4` against 95, and `have` alone was 19 cells
worse on the board top at one name.

**A second ABC pass is a real saving.** `synth_ice40 -abc2` (see the amendment below for how it is spelled) packs the board top at 5,190 to 5,225 over
six instance names (mean 5,208), against 5,279 for one pass: 71 cells, with the whole spread under
the part. It removes no logic the design needs. Placed, `upduino_top` reads 5,207 cells at the default
seed and 12.57 MHz.

**It costs Fmax on `littlesoc`.** The same flag on the SoC packs 5,194 against 5,265 and, on the
twelve seeds of `soc/pin.json` against that file's recorded draws for the same sources and toolchain,
reads median 12.30 MHz against 13.04, worst 11.93 against 12.44 (one seed under 12.0). So `littlesoc`
stays on one pass: `SOC_SYNTH` is unchanged, its netlist digest is unmoved, no sweep is owed and
`soc/pin.json` stands. The cost of a second pass is area bought with period, and only the board top
needs the area.

## Decision

1. `$(BUILD)/board.json` synthesises with a second ABC pass. `littlesoc` does not.
2. `make board-timing` places the board top (`BOARD_PNR`, default seed) and grades the `icetime`
   report against `SOC_MIN_MHZ` through `soc/timing_split.py`, the reader `make soc-timing` uses. A
   placement failure removes the stale `.asc` and report and fails with "NOTHING was measured".
3. The `soc-timing` CI job runs it after `make soc-timing`, so branch protection's required set
   covers it unchanged; the job's limit goes from 10 to 15 minutes.
4. `test/probe_gates.sh` forces both directions red: a nextpnr that cannot place (the failure this
   record starts from), and a placed top whose report is under 12 MHz.

## Measured

| | cells (`ICESTORM_LC`) | MHz |
|---|---|---|
| `upduino_top`, one pass | 5,306 (fails to place) | none |
| `upduino_top`, `-abc2`, default seed | 5,207 | 12.57 |
| `upduino_top`, `-abc2`, twelve hashed seeds and default | 5,207 at every seed | median 12.62, worst 12.05, best 12.92 |
| `littlesoc`, one pass (unchanged) | 5,265 | pinned 13.5 |

The worst seed of thirteen reads 12.05 MHz, 0.4% over the requirement, so the board top has less tail
margin than `littlesoc`'s 3.7%. The gate places one seed and the placement it reads moves with any
edit under it, the same condition ADR-0171 records for the SoC.

## Consequences

- Any RTL edit re-rolls both counts; a board top near 5,280 can fail again with no change of its own.
  `make board-timing` makes that a red job rather than a bitstream nobody built.
- The board bitstream is synthesised on a different line from the SoC that `make soc-timing` grades.
  The two are one design under two mapper settings, and the board's own gate now measures the one that
  ships.
- The toolchain floats: `-abc2` was measured on one yosys build. A build that maps differently can
  move the count by tens of cells, and the gate reports it.
- Not taken: removing or weakening any Zkr health test, `mseccfg`, or any other feature. The pure
  respellings tried did not move the count outside its own noise.

## Amendment: the flag is gone in yosys 0.70, so the passes are spelled out

CI's runner has yosys 0.70+157, whose `synth_ice40` has no `-abc2`: the first CI run of
`make board-timing` printed the usage text and failed in synthesis. The recipe now runs the script
in two halves, `synth_ice40 ... -run :map_luts`, then `abc; ice40_opt`, then `synth_ice40 ...
-run map_luts:`, which are exactly the two commands 0.68 ran under `-abc2` ahead of `map_luts`. On
0.68 the spelled-out form gives a netlist identical to the flag's (same JSON), so 5,207 cells and
the 12.57 MHz default-seed reading stand. The count on 0.70 is whatever the CI job log prints.

## Amendment 2026-10-19: the board top maps with a longer ABC9 script, not the classic `abc` pass

Yosys removed classic ABC mapping from the `synth_*` flows (commit 44bab117d, 2026-08-19) and
intends to remove more of it, so the spelled-out `abc; ice40_opt` of the previous amendment is on a
deprecation path. `$(BUILD)/board.json` is now one `synth_ice40 -device u -dsp -spram -top upduino_top
-json` with `scratchpad -set abc9.script soc/board_abc9.script` set first; the script is a Makefile
prerequisite. `littlesoc`'s flows are untouched.

**What differs from the default.** The default ABC9 script on 0.68 and 0.70 is
`&scorr; &sweep; &dc2; &dch -f -r; &ps; &if -W 750 -v; &mfs`. The shipped script is
`&scorr; &sweep; &dc2; &dch -f -r; &syn2; &dc2; &dch -f -r; &ps; &if -W 750 -v; &mfs; &ps -l`: a
second `&syn2; &dc2; &dch -f -r` round before the mapper, and a closing `&ps -l`.

**`-W 750` is hard-coded, and has to be.** It equals `synth_ice40.abc9.u.W`, the wire-delay constant
`synth_ice40` passes for `-device u`. Yosys substitutes `{W}`, `{D}` and `{R}` only into an inline
script (`+...`); a script file is handed to ABC as `source <file>` and the placeholders arrive
literally. A first try with `&if {W} {D} {R} -v` packed 5,098 cells because ABC ignored the unparsed
arguments and mapped with no delay constraint, which is not the default's mapping. 0.70's `abc9`
reads `abc9.script` from the scratchpad (`passes/techmap/abc9_exe.cc`), the same as 0.68.

**Which change carries the saving** (pack count after `nextpnr-ice40 --pack-only`, yosys 0.68+48,
`ICESTORM_LC` of 5,280 available):

| script | cells |
|---|---|
| default script, no extra pass | 5,306 |
| default with `-r` dropped from `&dch` | 5,304 |
| classic `abc; ice40_opt`, then the default script (previous amendment) | 5,207 |
| second round, `&dch -f` without `-r` | 5,163 |
| second round, `&dch -f -r` (shipped) | 5,163 |

Dropping `-r` alone moves nothing (5,304 against 5,306). The second `&syn2; &dc2; &dch` round carries
the whole saving, and it reads the same with or without `-r`, so the shipped script keeps the
default's `-r` and differs from it by the added round only.

**Sixteen paired seeds** (seeds 1 to 16, board top placed with `BOARD_PNR` plus `--seed`, timed with
`icetime` as `make board-timing` does; 1 to 16 are not an independent sample, ADR-0171):

| | cells | worst MHz | median MHz | best MHz | spread |
|---|---|---|---|---|---|
| classic `abc; ice40_opt` (previous amendment) | 5,207 | 12.23 | 12.63 | 13.36 | 9.2% |
| shipped ABC9 script | 5,163 | 12.34 | 12.82 | 13.16 | 6.6% |

The new spelling is smaller by 44 cells and no worse at the worst seed (12.34 against 12.23), the
median and the spread. The default-seed reading is 12.79 MHz (`make board-timing`).
`-dff` (5,167 cells, 12.56 worst) was declined: it adds sequential optimisation to a netlist no
simulation or proof runs, and with the longer script it packed 5,080 cells and read 12.19 worst.

Yosys 0.70's count on CI is recorded in the pull request that landed this amendment.
