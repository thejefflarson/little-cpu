# ADR-0179: nanocpu's freeze line is the largest 2×2 RISC-V shipped on sky130, and the core is projected to land on it

**Status:** Accepted · 2026-09-12

## Context

The nanocpu brief's freeze criteria (decision 11) begin "TT-flow synthesis ≤ 56k". The brief calls 56k "the demonstrated envelope": FazyRV-ExoTiny shipped on TT06 at 56,448 µm² of Tiny Tapeout layout-flow synthesis area. The brief's core — RV32EC, no M — was estimated at about 45,000 µm² in that flow, so the line left it about 11,000 µm² of room.

ADR-0177 changed the core: M and the 64-bit counters stay, and RV32E is the one cut. ADR-0178 measured the reshaped core at 60,758.272 µm² local, about 55,594 in the layout flow at the brief's 0.915 calibration. Adding the brief's estimates for the two layers still missing — CSRs and traps, about +6,000 µm² local, and the QSPI front end, about +5,500 — projects the finished core to about 72,258 µm² local, **66,116 µm² in the layout flow: about 10,000 over the 56k line.**

The same table in the brief records a larger shipped design: **TinyQV on ttsky25b, 66,006 µm²**, on a sky130 shuttle more recent than TT06. So 56k was the largest design on TT06 when the brief surveyed it, not the flow's limit.

**The stats files' utilisation figures are not a fraction of the tile.** FazyRV-ExoTiny's stats read 91.0% utilisation, but 56,448 is 77.8% of 72,565; TinyQV's TT06 stats read 83.9% at 53,888, which is 74.3%. The flow's own denominator is not the 2×2 core area. ADR-0177's sentence placing FazyRV-ExoTiny at "91% of the tile" conflated the two; its table's "~66,200 µm², at ~91%" is area ÷ 72,565 and stands. Only area ÷ 72,565 is used below.

## Decision

**The area criterion becomes TT-flow synthesis area ≤ 66,006 µm²** — the largest 2×2 RISC-V design in the brief's survey that shipped, TinyQV on ttsky25b, and 91.0% of 72,565 by area. Every other freeze criterion in decision 11 stands. This ADR supersedes decision 11 on this one number only; the brief stays as its dated record.

**The area line is a proxy. The binding test is the one listed beside it: the GDS action clean on the shuttle's pinned flow.** A shipped design proves the flow can close 66,006 µm² of *that* netlist, not of this one, because routing congestion depends on the netlist. Until nanocpu itself has been hardened the line predicts; once the flow has run on nanocpu, its result is the evidence and replaces the proxy.

**The stand-ins are estimates, named.** No run of the Tiny Tapeout flow on nanocpu existed when this was decided. The 0.915 local-to-layout factor is one calibration point on a different design (FazyRV-ExoTiny: 61,673 local, shipped at 56,448), and +6,000 and +5,500 are the brief's estimates for layers not yet built.

**The projection sits on the line, not under it: 66,116 against 66,006, 110 µm² over.** That is inside the error of either stand-in, so this ADR does not claim the design fits. The next two measurements decide it — the hardened flow, and each layer's area as it lands — and ADR-0177's cut order stays pre-committed for the case where the measured core exceeds the line: **the one-port register file first (−3,500 µm² local, +1 cycle per instruction), then TinyQV's latch array (about −7,000), before any feature.** The QSPI and CSR work does not reorder that under pressure.

## Alternatives not taken

- **Keep 56k and spend both fallbacks now.** Together they are about −10,500 µm², nearly the whole gap, so this commits to latches — declined for v1 over Tiny Tapeout's hold fixing and yosys's formal model of latches — before any QSPI logic exists, against a line a shipped design already exceeds by 10,000 µm².
- **A 3×2 tile.** Holds everything with room, at two more tiles, about +€140. Not needed while the 2×2 projection sits at a demonstrated line; it is the answer if the hardened flow says the 2×2 cannot close.

## Consequences

- Nothing grades this line yet. `NANO_MAX_UM2` (60,759 µm²) bounds regressions on the local instrument and is not this line; the Tiny Tapeout flow in CI is the instrument that will grade it.
- The QSPI and CSR layers each quote their measured area delta against its estimate here (+5,500 and +6,000 µm² local), so a layer that overruns shows up the day it lands rather than at freeze.
