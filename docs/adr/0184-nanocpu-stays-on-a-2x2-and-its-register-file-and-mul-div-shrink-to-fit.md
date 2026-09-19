# ADR-0184: nanocpu stays on a 2×2, and its register file and mul/div shrink to fit

**Status:** Accepted · 2026-09-14

## Context

ADR-0177 put nanocpu on a 2×2 Tiny Tapeout tile with RV32E as its one cut, and ADR-0179 set its freeze line at ≤ 66,006 µm². Both converted `make nano-area`'s local figure into Tiny Tapeout flow units with a 0.915× factor borrowed from FazyRV-ExoTiny, a different design on an older flow. The self-hosted `nano-tt-area-selfhosted` workflow now runs Tiny Tapeout's own flow on this design, and the borrowed factor does not hold.

**The conversion is 1.256×, not 0.915×.** On one tree, Tiny Tapeout's synthesis (tag ttsky26c, LibreLane 3.0.5) read 79,862.84 µm² against `make nano-area`'s 63,565.96. Today's wrapped core synthesizes to 81,879.78 µm², more than the 2×2's whole 72,564.6 µm² core area. `nano/nano.v` doesn't have its CSR/trap layer, 64-bit counters or QSPI front end yet.

**The tiles, measured on LibreLane 3.0.5 at `AREA 0`, 2026-09-14:**

| Tile | Placement utilization | Result |
| -- | -- | -- |
| 3×2 | 85.723% | detailed placement fails (`DPL-0036`) |
| 4×2 | 63.638% | global routing overflows: 105.77% demand, met2 at 115.92% |
| 6×2 | 41.988% | routes to 0 DRC violations |
| 3×4 | 37.322% | routes to 0 DRC violations |

The limit is wiring, not cell area. The 4×2 fails at a utilization most designs route at, and its congestion covers the whole die rather than a hot spot.

**What makes it wire-heavy.** One block was removed at a time, on 4×2, one run each, LibreLane 3.0.14, 2026-09-14:

| Variant | Synthesis µm² | Routing demand |
| -- | -- | -- |
| Baseline | 81,879.78 | 101.05% |
| Register file cut to 2 entries | 58,682.53 | 54.26% |
| Mul/div replaced by a stand-in | 64,285.40 | 56.76% |
| Enable flip-flop allowed | 79,675.16 | 84.82% |
| Clock relaxed to 25 MHz | 81,879.78 | 105.31% |
| Register file and mul/div both cut, **on a 2×2** | 41,498.55 | 69.17%, no overflow |

What each block costs:
- **The register file.** Its two read ports are selector trees, and each of their four low read-address bits drives 128 `mux4` selects.
- **Mul/div.** It keeps three 64-bit shift registers and a 64-bit add, subtract and compare.
- **The enable flip-flops.** The PDK's excluded-cell list turns every enabled flip-flop into a plain flop plus a `mux2_1`. That also makes the write path fast enough to need most of the 941 hold buffers.

The timing-repair buffers are driven by hold and fanout, not by the clock target.

**The one complete signoff so far** is 3×4 on LibreLane 3.0.14:
- Layout: DRC, LVS and antenna are all clean.
- Hold: met at every corner.
- Setup at 64 MHz: met at tt (+4.69 ns) and ff, missed at ss (−3.82 ns, TNS −1,112.7 ns).

## Decision

**nanocpu stays on a 2×2.** Twelve tiles route today's core, and they are declined on cost.

**The register file and mul/div are rebuilt to fit, in this order:**
1. A latch-array register file.
2. Mul/div built around one shared 64-bit register and a 32-bit adder, keeping M.
3. A one-read-port register file.

Each is measured in Tiny Tapeout's flow on 4×2 and 2×2 before the next begins. Whether Tiny Tapeout accepts overriding its enable-flip-flop exclusion is not pursued.

**M may be cut if those three are not enough.** This amends ADR-0177: RV32E is no longer the only permitted cut. M goes only after the cheaper mul/div has been measured and the core still does not fit a 2×2.

**ADR-0179's freeze line is void.** Its 66,006 µm² was derived in flow units with the 0.915× factor. The 2×2 line is re-derived from Tiny Tapeout flow runs. Until a routed finished core replaces it, the budget to beat is the 2×2 run above that fit: 41,498.55 µm² of synthesized logic at 69.17% routing demand.

**LibreLane 3.0.14 is the instrument's version.** The runner image moved from 3.0.5 to 3.0.14 on 2026-09-14, and it stays there. Every flow number is quoted with its LibreLane version. The 4×2 baseline read 105.77% on 3.0.5 and 101.05% on 3.0.14, one run each, so the two are not interchangeable.

## Consequences

- Area and routing are quoted in Tiny Tapeout flow units from `nano-tt-area-selfhosted`, with the LibreLane version, next to any `make nano-area` figure. The two instruments are never merged.
- `NANO_MAX_UM2` stays a regression bound on the local instrument only. No flow-unit line gates nanocpu until a finished core routes on a 2×2.
- The CSR/trap layer, the counters and the QSPI front end must fit the same budget. Each block rebuilt here is measured against the 2×2, not against whichever tile it happens to fit.
- ADR-0177 and ADR-0179 carry amendments pointing here.

## Amendment · 2026-09-18

This ADR permitted M as a second cut "if those three are not enough" — ADR-0195 confirmed they were not, and ADR-0197 spends the permission: `nano/nano.v`'s multiply/divide unit is deleted outright. **This ADR's own "2×2 stays the target" is retired by that ticket's measurement, not reaffirmed by it**: with M gone the 2×2 places but fails routing, and the 4×2 both places and routes to a clean signoff — see ADR-0197 for the full measurement. ADR-0197 leaves the tile choice to the owner rather than continuing to treat 2×2 as fixed; a future tile decision belongs in its own ADR.
