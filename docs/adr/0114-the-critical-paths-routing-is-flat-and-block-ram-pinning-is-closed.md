# ADR-0114: The critical path's routing is flat, and block-RAM pinning is closed

**Status:** Accepted · 2026-08-16 · *Closes a work direction. Answers the tail question ADR-0113
left open with no lever. Amends no commitment, adds no gate and no ratchet.*

## Context

Routing is about seven tenths of this SoC's critical path — 53.16 ns of 80.68 at the worst of the
sixteen placements below — and nothing in this repo attributed any of it. `soc/depth/path_stages.py`
attributes logic LEVELS to the RTL that built them and says so; `soc/timing_split.py` totals the
interconnect by cell class, which is how much and not where.

Two results made that gap expensive rather than merely untidy.

**The atomic region test cost zero logic levels and still moved the worst placement 2.2%**
(ADR-0109). The fetch loop came out a level shallower and the period went the wrong way, so what
moved was routing. Every lever this repo has pulled has been a levels lever.

**ADR-0113 declined the successor-pair table on the tail, not on speed.** Its paired median is
+1.17% with a sign test at p = 0.80 — a null at the centre — and its best placement beats the base's
best. What it costs is variance: the spread went 5.5% to 11.2% and the worst placement landed at
11.93 MHz, under the requirement. About **0.6% of worst-seed recovery** would buy −8.03% of
Dhrystone's cycles and 0.727 → 0.790 DMIPS/MHz. That is a small, priced, otherwise-free win blocked
by a placement tail.

The named candidate for tightening that tail was **pinning the block RAMs**. The fetch loop starts
and ends in sixteen enumerable `SB_RAM40_4K` whose columns nextpnr re-chooses every seed, and a
`nextpnr-ice40` constraint file is attractive on its own terms: it is not RTL, so the ECP5 flow
ADR-0110 added never sees it, and it is therefore immune by construction to the two parts
disagreeing in sign the way ADR-0087 measured them doing.

It was also an untested hunch that could absorb weeks. This is the afternoon that grades it.

## The instrument

`soc/routing_bins.py` reads one `soc/baseline_sweep.sh` sweep and charges every routing hop on every
retained report to what sits at the two ends of it:

- **EBR-adjacent** — an `SB_RAM40_4K` at either end;
- **SPRAM-adjacent** — an `SB_SPRAM256KA` at either end;
- **`riscv.pc`-sourced** — the hop carries a bit of that declared net;
- **neither**.

A run of `Odrv4`/`Span4Mux`/`LocalMux`/`InMux` hops carries one net from the cell that drives it to
the cell that reads it, and icetime names that net above the run, so both ends are derivable from
the synthesis JSON — the driver of the carried net, and the driver of the next different net on the
path. Nothing is read off a placed instance name. `riscv.pc` is tested by bit membership of the
declared net for the same reason: most of the nets on these paths are called
`riscv.decoder.pc_SB_DFF_Q_27_D_...` and are not the pc.

**Two things grade it.** The bins are checked against `soc/timing_split.py`'s own routing figure for
the same report and the script exits non-zero when they disagree — a histogram of half a path reads
exactly like a histogram of a path. And every name icetime resolved has to resolve in the netlist,
because a netlist from another tree fills the `neither` bin silently. Nine probes in
`test/probe_gates.sh` force both directions, the reconciliation by handing a copy of the reader a
`timing_split.py` that charges one hop class differently.

**The scope is narrow and is not assumed away.** `icetime -r` prints the critical path and nothing
else, so this is sixteen samples of ONE path each. It is not a routing census of the design, and it
measures **adjacency, not causation**: work displaced by where the memories sit lands in `neither`
and is invisible here.

## The measurement

Sixteen placements, base `c13ed54`, clean tree, `datainit.c` into 2048 ROM words, up5k/sg48,
`yosys 0.68+48`, `nextpnr-0.11-1-g62e659ed`, `icetime` from `oss-cad-suite 20260811`. Worst 80.68 ns
/ 12.39 MHz, median 78.40, best 76.51, spread 5.5% — the same sweep ADR-0113's base column was taken
from.

| | hops | ns | share of routing |
|---|---|---|---|
| EBR-adjacent | 44 | 30.01 | **3.6%** |
| SPRAM-adjacent | 0 | 0.00 | **0.0%** |
| `riscv.pc`-sourced | 40 | 26.38 | 3.1% |
| neither | 1111 | 786.50 | **93.3%** |
| all routing | 1195 | 842.89 | |

Read where the requirement is read, it is smaller still. **At the worst placement the critical path
does not touch a block RAM at all** — EBR 0.0%, pc 5.2%, neither 94.8% — and at the median it
touches neither memory nor pc, at 100% `neither`.

**The contact does not track the tail.** Split the sixteen at the median and the eight fastest carry
15.35 ns of EBR-adjacent routing between them against the eight slowest carrying 14.67, three
placements a side touching a memory at all. The 2.18 ns that separates the two halves is 2.03 ns of
routing and 0.16 ns of logic, and inside the routing it is 0.78 ns of `neither` and **−0.09 ns of
EBR**. What actually moves is the number of hops: 72.4 per placement in the fast half against 77.0
in the slow one, 67 at the best placement and 80 at the worst, with the nanoseconds per hop going
*down* as the path gets slower — 0.75 to 0.66.

**There is no long hop to attack.** Every hop on every one of these paths is between 0.25 and
1.28 ns, and the largest anywhere is 1.279 ns, which is the block RAM's own clock-to-out: a device
parameter, not routing, and not movable by placement. The largest hop that is really interconnect is
an `Odrv12` at 1.232 ns — 1.5% of the worst period.

**And the sixteen are not sixteen.** Three ROM instances appear at the end of any hop on any
placement — `imem.rom_even.0.4`, `imem.rom_odd.0.1`, `imem.rom_odd.0.0` — and none at all at the
placement the requirement is graded on.

## Decision — block-RAM pinning is closed, and the tail has no known lever

**Do not spend time constraining `SB_RAM40_4K` placement to buy period or to tighten the spread.**
Three facts close it independently: the contact is 3.6% of routing aggregate and 0.0% where the
requirement is read; it is the same size in the fast half and the slow half; and there is no
column for a constraint to act on, because at most three of the sixteen are ever on the path and
none of them is at the tail.

SPRAM adjacency is closed outright at 0.0% on all sixteen. The data RAM is never on this path.

**The consequence for ADR-0113 is the part worth writing down.** The successor-pair table's 0.6% of
worst-seed recovery does not have a lever here. That is a materially different position from having
not looked: the tail is routing, and the routing is flat — seventy to eighty short hops rather than
a few long ones — so what would recover it is fewer hops on the critical arc, which is a levels
argument and a placement-quality argument, not a constraint on sixteen cells. ADR-0111's price list
is unchanged and the table stays deferred.

## What this does not say

**It does not say congestion is innocent.** This instrument sees adjacency. If the memory columns
distort the placement of the logic between the ends, that cost lands in `neither` and is not
separable here. What it does say is that the worst path never reaches a memory, so a memory
constraint would have to act on it indirectly — and the independent measurement pointing the same
way is already recorded: ballasting occupancy from 77% to 95% moved the period not at all
(ADR-0088).

**It does not close the pc arc.** `riscv.pc`-sourced routing is 0.98 ns per placement in the fast
half and 2.32 in the slow one, because the slow placements are more often the ones whose critical
arc starts at the pc rather than at a ROM output — six of eight against three of eight. At n = 16
with a binary outcome that is noise, and it is recorded as an observation with its own weakness
stated rather than as a finding.

**It is not a gate and adds no ratchet.** `SOC_MIN_MHZ` is unmoved at 12.0. Nothing here runs on CI,
nothing here is RTL, and it therefore cannot regress either part.

**It is perishable, like every ceiling here.** It was taken on `c13ed54` with one toolchain. A
datapath change that puts the memories back on the tail's critical path re-opens it, and re-opening
it costs one sweep and one command.

## Consequences

- `soc/routing_bins.py` is the third reader of an `icetime -r` report and writes no parser: the walk
  is `soc/timing_split.py`'s, the netlist resolution and the per-hop walk are
  `soc/depth/path_stages.py`'s — which grew `path_hops()` for it, with `path_nets()` now derived
  from that so the report is still walked once there — and the sweep and its provenance block are
  `soc/baseline_summary.py`'s.
- **A sweep does not keep the netlist its seeds were placed from**, because synthesis does not depend
  on the seed. Re-make `soc.json` at the sweep's base commit before binning it; the script refuses a
  netlist whose names do not match rather than filling `neither` with them.
- **Quote the bins with the scope.** Sixteen samples of one path each, adjacency and not causation.
