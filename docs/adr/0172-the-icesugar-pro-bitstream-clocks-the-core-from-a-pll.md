# ADR-0172: The iCESugar-Pro bitstream clocks the core from a PLL

**Status:** Accepted · 2026-09-09

## Context

ADR-0163 got Dhrystone printing off the iCESugar-Pro at **0.775 DMIPS/MHz, 19.4
DMIPS at 25 MHz**. The 25 came from `soc/board_icesugar_pro.v` wiring the board's
oscillator pad straight into `littlesoc`, and the design was reported at 33.28
MHz against a 25 MHz constraint in the same run. Nothing in the tree instantiated
an `EHXPLLL`; the only three mentions of one were prose.

On the up5k that gap would not be spendable. The board clock there is a step
function — a 12 MHz crystal, or `SB_HFOSC`'s 48/24/12/6 — so a design that places
at 13 MHz and one that places at 23 MHz both run at 12, and Fmax above the
requirement is margin rather than speed (ADR-0089). **ECP5 synthesises its clock**,
so the same margin is a frequency the part will actually run at.

## Decision

**Instantiate a PLL, drive the core at 30 MHz, and take `CLOCK_HZ` with it.**

`soc/icesugar_pro_pll.v` is `ecppll -i 25 -o 30`'s output, kept in the tree so its
dividers are reviewable rather than regenerated at build time: `CLKI_DIV` 5,
`CLKFB_DIV` 6, `CLKOP_DIV` 20, feedback on `CLKOP`. That is 25 / 5 × 6 = **30.000
MHz exactly**, with the VCO at 600 MHz, mid-range for the 400–800 MHz the part
locks over. Two lines of the generated file were changed and nothing else: explicit
`wire` on the ports, and the `default_nettype` guards this repo's other modules
carry. The four comment lines `ecppll` writes about Lattice Diamond versions were
dropped.

**The frequency is chosen against the worst placement, not the median.** Twelve
paired seeds of the shipping top — `icesugar_pro_top`, caBGA256, the P6 pad, the
PLL in place, constrained at 30 MHz — read

| | MHz |
|---|---|
| worst | 33.45 |
| median | 34.12 |
| best | 34.95 |
| spread | 4.48% |

and `make ecp5-timing`'s own twelve, on the other design (bare `littlesoc`,
caBGA381, the P3 pad), read 32.01 worst against a 33.70 median. **30 MHz is below
both**: 10.3% under the shipping design's own worst seed and 6.3% under the
different design's. The margin is deliberately larger than one placement spread,
because nextpnr's model does not carry the PLL's own output jitter and neither
sweep is a bound on a thirteenth seed.

**`CLOCK_HZ` is not left behind, and the mechanism is a check rather than a
convention.** `rtl/uart.v` derives its 115200 8N1 divisor from `littlesoc`'s
`CLOCK_HZ`, so that parameter *is* the baud rate: moving the clock and leaving the
parameter would scale the baud by the same 1.2× and every board figure would come
back as garbage — which, on a board whose only output is that UART, looks exactly
like a dead core. The frequency is now stated in four places, three of them silent
when wrong:

| Where | What it says |
|---|---|
| `soc/icesugar_pro.lpf` | `FREQUENCY PORT "clk_pin" 25 MHZ` — what nextpnr believes the pad is |
| `soc/icesugar_pro_pll.v` | `CLKI_DIV` / `CLKFB_DIV` — what the silicon multiplies by |
| `soc/icesugar_pro_pll.v` | `FREQUENCY_PIN_CLKOP` — what nextpnr constrains against |
| `soc/board_icesugar_pro.v` | `CORE_HZ` — what `rtl/uart.v` divides by |

`test/pll_clock_test.py` recomputes the third and fourth from the first and second
and requires all four to agree, requires the VCO to be in range, requires the PLL
to be fed by the pad the constraint names, and requires `littlesoc` to be clocked
from the PLL's output rather than the pad. It is hermetic — three file reads and
integer arithmetic — so it runs inside `make test` anywhere, and its fifteen red
directions are in `make probe-gates`.

**The constraint is derived, not declared.** `--freq` is gone from the
`icesugar.config` recipe. With the pad's frequency in the LPF, nextpnr does the
divider arithmetic itself and constrains the core domain at 30 MHz; a `--freq`
alongside it overwrites that derived figure with a flat one, which is exactly the
failure the first attempt here hit — the PLL was in place, `FREQUENCY_PIN_CLKOP`
said 30, and the log still read `PASS at 25.00 MHz`. Missing the derived
constraint is an `ERROR` from nextpnr and a nonzero exit, so the recipe is now the
bitstream's timing gate; `.DELETE_ON_ERROR` removes the configuration nextpnr
still writes on the way out.

**The PLL's lock gates reset, through a port that already existed.** `littlesoc`
takes `btn_n`, a released-button input that holds `reset` asserted while low, and
the board file now drives it from the PLL's `LOCK`. Before lock the output is not
the frequency anything was placed for; after it, the SoC's existing two-flop
synchroniser releases reset two cycles later. No RTL changed to get that.

## What did not move

- **The three ECP5 mapping censuses.** They gate `make ecp5-timing`, which
  synthesises bare `littlesoc` and reads no board file, so an `EHXPLLL` in
  `icesugar_pro_top` is invisible to them. `soc/bram_reset_check.py` runs on the
  iCESugar flow and still reports 36 `DP16KD` with none reset from logic.
- **`ECP5_TARGET_MHZ`.** It is still 200.0 and still unreachable. That constant
  belongs to the measuring flow, whose constraint has to be missed or the run
  reports the target rather than the design; this bitstream's constraint has the
  opposite job and is a different file's.
- **The up5k.** Nothing here reaches `soc/board_upduino.v`, `make fit`,
  `make soc-timing` or any number taken on that part.
- **`soc/compare/`.** Its ECP5 arm measures each core's own Fmax with no PLL and
  scores every core at its own worst placement, which is what makes the ratio a
  comparison. A PLL there would measure the PLL.

## What this is worth, and what is still owed

`19.4 → 23.3 DMIPS` at the same 0.775 DMIPS/MHz — a 20% throughput gain with no
RTL change and no cycle spent — **predicted, not measured**. The board is the only
thing that can confirm it, and no board ran this. What a run must produce:

| | Predicted |
|---|---|
| `make icesugar-dhrystone` DMIPS/MHz | 0.775, unchanged — this is a clock change, not a cycle one |
| Dhrystone at 30 MHz | 23.3 DMIPS |
| Wall-clock time for the same run count | 0.833× ADR-0163's |
| UART | clean 115200, because `CLOCK_HZ` moved with the clock |

A DMIPS/MHz that moved would mean the cycle count changed, which nothing here
should do. Garbled UART output would mean `CLOCK_HZ` and the real clock have come
apart in a way the four-file check does not see — the only candidate left being
the board's oscillator not being 25 MHz, which is MuseLab's figure and has never
been measured here.

## Alternatives

**Ship at 32 MHz.** It is under `make ecp5-timing`'s worst-of-twelve by 0.03%,
which is not a margin. A frequency the median placement makes and the worst does
not is a bitstream that works until it is rebuilt.

**Ship at the median, 34 MHz.** Four of the twelve seeds miss it. With the timing
gate now in place that is a build failure rather than a broken board, but a build
that fails on a third of its seeds is not a shipping configuration.

**Leave `--freq` and constrain by hand.** It works and it is one number in one
place — but it is a *fourth* independent statement of the same frequency, and the
whole difficulty here is that this frequency is already stated three times too
many. Deriving it from the dividers means the placer and the silicon cannot
disagree.

**Hold reset with a counter instead of `LOCK`.** `littlesoc`'s power-on counter is
already that counter, and it counts the PLL's own edges — of which there are none
before lock and unreliable ones during it. Reading `LOCK` costs a wire.
