# ADR-0184: nanocpu goes on a 3×2 tile and keeps every feature

**Status:** Accepted · 2026-09-13

## Context

ADR-0177 put nanocpu on a 2×2 Tiny Tapeout tile, with RV32E as the one cut, and projected the finished core to about 66,200 µm² of layout-flow area. ADR-0179 set the freeze line on that tile at ≤ 66,006 µm². Both projections used a 0.915× local-to-flow factor borrowed from FazyRV-ExoTiny, a different design on an older flow.

Tiny Tapeout's own flow (tag ttsky26c, LibreLane 3.0.5), run on this design by the self-hosted `nano-tt-area-selfhosted` workflow and stopped after synthesis, measures:

- **The wrapped core on main, after the divider fix: 81,879.78 µm²**, against the 2×2 die's 72,564.6. That is 12.8% over the die before the CSR/trap layer or the QSPI front end exist.
- **The same source through `make nano-area`'s local recipe: 63,565.96 µm²** (before the divider fix, against the flow's 79,862.84 on the same tree). That is a factor of 1.256×, the opposite direction from the borrowed 0.915×.

The levers, measured:

| Lever | Worth | Legal in a submission |
| -- | -- | -- |
| Best `SYNTH_STRATEGY` (`AREA 2`) | −1.01% in the flow | yes; TinyQV's accepted submission sets it |
| Enable flip-flops | −2.6% locally (−1,568 µm²) | no; the sky130 PDK's own excluded-cell list removes `edfxtp` |
| Latch register file | −18.7% locally (−11,381 µm²); not measured in the flow | yes, at hold-fixing and formal-model costs not yet assessed |
| One-port register file | −3,500 µm² locally (the brief's estimate), +1 cycle per instruction | yes |

With the CSR/trap layer and the QSPI front end added (about 11,500 µm² locally in the brief's estimates, roughly 14,400 in the flow at 1.256×), the finished core projects to about 96,300 µm² in the flow. Taking every lever above leaves about 76,800, still over the 2×2 die before any routing margin. No 2×2 answer keeps every feature.

## Decision

**nanocpu targets a 3×2 tile and keeps every feature**: M, the 64-bit counters, the CSR/trap layer and the QSPI front end. RV32E stays, because that reshape is done and verified; restoring RV32I would be a separate decision. The latch register file becomes an optional lever rather than a requirement, so its hold-fixing and formal-model costs only need assessing if a later measurement calls for it.

At the brief's pricing a 3×2 is six tiles against four: two more at €70, about +€140.

**The freeze line is re-derived on the 3×2, not carried over.** ADR-0179's 66,006 µm² was a 2×2 line. The 3×2's core area has not been measured here. The first `workflow_dispatch` of `nano-tt-area-selfhosted` at `tiles=3x2` reports it, and a full-flow run there reports whether the core places. The new line is recorded as an amendment to this ADR from those numbers, not estimated ahead of them.

## Consequences

- The QSPI front end, the CSR/trap layer and the chip top proceed. Each quotes its area in Tiny Tapeout flow units from the self-hosted workflow, next to its `make nano-area` figure. The two instruments disagree by 1.256× on this design and must not be merged.
- `NANO_MAX_UM2` remains a regression bound on the local instrument only. No area line gates nanocpu until the 3×2 has been measured.
- ADR-0177's "RV32E is the one cut, on a 2×2 tile" and ADR-0179's freeze line are amended to point here.
