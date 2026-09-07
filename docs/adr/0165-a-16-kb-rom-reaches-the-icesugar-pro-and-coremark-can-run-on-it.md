# ADR-0165: A 16 KB ROM reaches the iCESugar-Pro, and CoreMark can run on it

**Status:** Accepted (the plumbing and the simulated match; the board run is
still owed) · 2026-09-06

## Context

Every published CoreMark figure describes hardware this project does not
ship: `make coremark` links against `test/testbench.v`'s simulated 16 KB ROM,
double `rtl/imemory.v`'s shipping 2048 words, because CoreMark does not fit
the smaller one. Dhrystone has no such gap — `make dhrystone-board` and
`make icesugar-dhrystone` both run it on real parts, and ADR-0163 got the
iCESugar-Pro's Dhrystone to read cycle-identical to cxxrtl.

`littlesoc` uses 36 of the LFE5U-25F's 56 `DP16KD` blocks and has no SPRAM on
that part, so the data RAM already costs block RAM there. 20 spare blocks and
no up5k-style 8 KB ceiling (ADR-0135's is a fetch-loop and block-RAM-count
argument specific to the up5k) made the ECP5 the one part where a 16 KB ROM
looked reachable without touching `COREMARK_CFLAGS`.

## The blocker

**`littlesoc` could not be built at any ROM size but 2048 words.**
`SOC_ROM_WORDS` reached `--rom-words` for hex generation and nowhere else:
`rtl/littlesoc.v` hard-coded `2048` twice, once into `littlecpu`'s
`LS_TEXT_WORDS` and once into `imemory`'s `ROM_WORDS`, and no Makefile recipe
`chparam`'d either. `make ecp5-timing SOC_ROM_WORDS=4096` generated a 16 KB
hex and then synthesised the unchanged 8 KB design — same 36 `DP16KD`,
nothing measured.

## Decision

**One parameter, `ROM_WORDS`, threads through `littlesoc.v`** to both
`littlecpu`'s `LS_TEXT_WORDS` and `imemory`'s `ROM_WORDS`, replacing the two
literals a change could disagree on. `SOC_SYNTH` (the ice40 SoC synthesis
used by `make fit`/`make soc-timing`), `ecp5.json`'s recipe and
`icesugar.json`'s recipe each `chparam -set ROM_WORDS $(SOC_ROM_WORDS)
littlesoc` before `hierarchy`/`synth_*`, the same order `COMPARE_READ`'s
callers already use for `COMPARE_ROM_WORDS`. **Each of those scripts stays on
one line.** Wrapping the two recipe-level ones over a backslash-newline inside
their single quotes put a literal `\` and a newline into the yosys script —
not a shell continuation there — and every ECP5 flow stopped with
`ERROR: No such command: \`. `make soc-timing` was unaffected, because
`SOC_SYNTH` is a make variable and make joins those, so the ice40 half stayed
green while both ECP5 halves were dead. **The chparam is emitted only when a caller overrode `SOC_ROM_WORDS`**, which
`$(origin SOC_ROM_WORDS)` answers without a second copy of the number, because
setting it at the unchanged value is NOT free. Measured against merged `main`,
same ROM image (byte-identical `soc/rom_even.hex`), same tools: a chparam that
re-states 2048 moves the up5k SoC netlist **from 6289 to 6319 mapped cells**,
the growth in the two combinational types, because yosys derives a parameterised
copy of `littlesoc` and stops sharing across the hierarchy the way it does for
the plain module. Nothing in the tree would have caught that: `FIT_MAX_LC`
ratchets the core alone, `SOC_EXPECT_EBR`/`SOC_EXPECT_SPRAM` count memories
and not logic, and `SOC_MIN_MHZ` is graded at one seed on CI. This is
ADR-0088's own rule arriving in a new place — a parameter tied off to today's
value is not free of the mapper. Emitted only on override, the shipping
netlist is **cell-identical to `main` in all eleven types**; the digest
differs on net names alone, and that gate is sound in one direction only.
`make window-test` (44/44) and `make memmap-test` confirm the rest.

`icesugar.json` needed its own `chparam` even though `icesugar_pro_top` never
sets `ROM_WORDS` itself: the parameter lives on `littlesoc`, the submodule
`icesugar_pro_top` instantiates at its own default, so `chparam` on
`littlesoc` before `hierarchy` is what that instantiation inherits. Missing
this on the first attempt built a 16 KB ROM image against an 8 KB-synthesised
design silently — the exact defect class this file exists to close — caught
only by reading `soc/bram_reset_check.py`'s printed cell count back against
the intended geometry rather than trusting the build to fail.

**`make coremark-rom-ecp5`** builds CoreMark against `test/bench/coremark.lds`
(the 16 KB region `test/testbench.v`'s simulated ROM already uses) with
`COREMARK_CFLAGS` unchanged, mirroring `dhrystone-rom`'s shape.
`-DCOREMARK_UART` compiles in `coremark_port.c`'s UART path (already
conditional there, the same way `DHRY_UART` gates Dhrystone's) so the board
can print its own report.

**The name carries the geometry, because make would not have.** A second
board route for the up5k's 8 KB ROM landed in the same sprint under the same
name `coremark-rom`, at `-Os -flto` against `test/bench/bench.lds`. Make
resolves a redefined recipe **last-wins**, with only a
`warning: overriding recipe for target`, while a `?=` default beside it is
**first-wins** — so the merged file would have paired one route's flags and
iteration count with the other's linker script and ROM budget, and the flags
string is compiled INTO the image for EEMBC's disclosure line, making the
report misstate its own build. The two routes are therefore
`coremark-rom-ecp5` and `coremark-rom-up5k`, with `COREMARK_ECP5_*` and
`COREMARK_UP5K_*` variables. `test/makefile_target_test.sh` grades it, reading
make's own `overriding recipe` warning rather than a second parser, and runs
inside `make test` with five forced-red probes.

**The pin check is one script, `test/bench/coremark_pin_check.sh`, and both
board routes depend on it.** CoreMark's trademark terms permit the name only
for an unmodified copy, and the route that flashes hardware must not be the
one whose check was forgotten — the up5k route as proposed compiled the
vendored sources with no check at all. Membership before hashes, because
`shasum -c` cannot see a file the manifest never named.
`test/bench/run_coremark.sh` and `soc/compare/run_coremark_compare.sh` still
carry their own copies; collapsing those two onto this script is a follow-up.

**`make icesugar-coremark`** ties it together: `coremark-rom-ecp5` at
`SOC_ROM_WORDS=4096`, `icesugar.bit` at the same override, `icesugar-prog`,
then read the UART for the self-check line — the same shape
`icesugar-dhrystone` already has. It needs the board and is off `make test`
and CI for the same reason.

## What it measures

`make ecp5-timing SOC_ROM_WORDS=4096 ECP5_EXPECT_DP16KD=40`:

| | 8 KB ROM (shipping) | 16 KB ROM |
|---|---|---|
| `DP16KD` | 36 / 56 (64.3%) | **40 / 56 (71.4%)** |
| `TRELLIS_DPR16X4` | 32 | 32 (unchanged — `rtl/regfile.v`, not the ROM) |
| `MULT18X18D` | 4 | 4 (unchanged) |
| Fmax (nextpnr's own estimate, 200 MHz constraint) | 35.11 MHz | **34.78 MHz** |

Four more `DP16KD` (one bank's worth of the doubled window) and a 0.94% Fmax
move — inside the churn a single seed can show, not a finding. Against the
board's real 25 MHz: `icesugar.bit` at `SOC_ROM_WORDS=4096` places and PASSes
nextpnr's own 25 MHz check with margin (26.14 MHz / 33.58 MHz, two clock
domains reported), the same shape the unmodified 8 KB build already showed
(28.82 MHz / 35.08 MHz). **16 KB is comfortably clear of 25 MHz** — the ROM
does not spend the board's margin the way ADR-0145's 12 KB two-window attempt
spent the up5k's.

CoreMark fits: the built image is 14,236 bytes of `.text` + `.data`, inside
the 16,384-byte region and comfortably clear of the 8,192-byte one it cannot
fit (`make coremark-rom` without an override reports exactly that region
overflow, the same diagnostic `test/bench/run_coremark.sh` gives for the
simulated build). **16 KB fits on this part where it does not on the up5k**
(ADR-0135: 32 block RAMs against 26 free there) — the answer this ticket
asked for, in the affirmative, on the one part with the spare blocks to
answer it.

## What matched, and what is still owed

**Verified against cxxrtl on the exact board binary**, the way ADR-0163
verified Dhrystone: `make sim`, then
`test/bench/run_coremark.sh ./sim 100 200000000 "$(COREMARK_CFLAGS)
-DCOREMARK_UART=0x00020020"` — the identical flags `coremark-rom` compiles
the board image with. PASS, self-check clean against EEMBC's published CRCs,
**2.203 CoreMark/MHz**, 45,389,189 cycles, matching the already-published
simulated figure (ADR-0154) to three decimals. `stop_time()` runs before the
UART's own busy-wait tail, so the define changes nothing timed.

**Not done, and this is the honest half:** no iCESugar-Pro was available in
this environment. `make icesugar-coremark`'s `icesugar-prog` and the UART
read that follows it need the physical board — `openFPGALoader` against a
real CMSIS-DAP device — and were not run. Every step that does not need one
(`coremark-rom`, `icesugar.bit`, `make ecp5-timing` at both geometries) was
built and its output read back, matching ADR-0163's own division of labour
between what a development machine can confirm and what only the part can.
**The board run — the actual DMIPS-style self-check line off real silicon —
is a follow-up this ADR does not close.**
