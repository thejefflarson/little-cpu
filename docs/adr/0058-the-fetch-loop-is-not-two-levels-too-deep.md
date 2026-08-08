# ADR-0058: The fetch loop is not two levels too deep

**Status:** Accepted · 2026-08-02 · *A measured null. Two restructurings of the fetch loop were
proposed against
[ADR-0054](0054-the-memory-system-and-the-first-real-timing-number.md)'s 11.30 MHz measurement and
[ADR-0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md)'s finding that
writable text costs a further 8.5%. Both were built and measured; both are slower than what is
checked in. Corrects how ADR-0054's "41 logic levels" should be read. Its closing recommendation —
target the decode head next — is withdrawn by
[ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md), which measured it.*

## What was proposed, and what it rested on

ADR-0054 named the critical path `imem.in_range → decode → next PC → imem.in_range2` and reported
88.51 ns over **41 logic levels**, 38.7% logic and 61.3% routing, with `LocalMux` + `InMux` at
41.2 ns of the 54.3 ns of routing and all distance routing (the `Span4Mux` family) at 5.5 ns. The
reading taken from that was: the cost is depth, per-level interconnect is paid 41 times, and so
floorplanning cannot help but fewer levels can. Two candidates followed.

1. **Compute the ROM's range flag in parallel with the `next_pc` mux** instead of after it, so the
   comparator stops hanging off the tail of whichever adder wins. Carry-select in shape.
2. **Flatten the `next_pc` priority chain.** Eight arms in priority order is eight levels; a flat
   mux with a one-hot select computed in parallel is about two.

Neither needs a flush or a forwarding network — both are re-expressions of logic that already
exists — so neither is blocked by an invariant. They are blocked by the measurements below.

## The premise is wrong: 41 levels are not 41 interconnect hops

The 41 levels are **25 LUT/setup hops and 17 carry hops**, and the two cost an order of magnitude
apart. A `carryin -> carryout` hop stays inside the carry chain and is entered through no
interconnect at all; every other LogicCell40 hop arrives through a `LocalMux` and an `InMux`. On the
baseline path:

| hop kind | count | ns each |
|---|---|---|
| LUT level, with the interconnect into it | 25 | **3.31** |
| carry hop | 17 | **0.34** |

`LocalMux` x23 and `InMux` x24 in the report say the same thing directly: per-level interconnect is
paid about 23 times, not 41. **Every conclusion below follows from that one number.** A change that
trades a carry hop for a LUT level makes the design shallower by icetime's count and slower in
nanoseconds, and both candidates are that trade.

`soc/timing_split.py` prints the two counts and the two per-hop costs now, so the misreading cannot
be taken from `make soc-timing`'s own output again.

## Method

`make soc-timing` with `SOC_PROG=add.S`, four placements per variant (the default plus
`nextpnr-ice40 --seed 1/2/3`), which is ADR-0057's method. **Homebrew Yosys 0.67+post
(`b8e7da6f`)**, nextpnr-0.10-108-g68c1acd8, icetime from the same install; machine load 8–15
throughout, on 10 cores. nextpnr is seeded, so every figure is reproducible run to run; load moves
wall time and not the numbers.

**The baseline reproduces ADR-0057's four placements exactly** — 87.43 · 87.55 · 87.94 · 88.51 ns
at 4041 logic cells — so the rows below are a comparison rather than four measurements from
different worlds.

`SOC_SEED` was added to the Makefile in this change, because a placement spread is now something two
ADRs have needed and neither could take with the tracked flow.

## The measurements

| variant | LC | icetime ns, four placements | MHz |
|---|---|---|---|
| baseline (`main`) | 4041 | 87.43 · 87.55 · 87.94 · 88.51 | 11.30 – 11.44 |
| candidate 1, **at its ceiling** | 3978 | 88.98 · 89.32 · 89.39 · 89.41 | 11.18 – 11.24 |
| candidate 2, flat one-hot mux | 4059 | 87.43 · 90.03 · 92.24 · 94.10 | 10.63 – 11.44 |
| both together | 3990 | 84.44 · 86.30 · 87.55 · 89.00 | 11.24 – 11.84 |

- **Candidate 1 is a regression, and it was not even implemented — its ceiling was.** Rather than
  replicate the comparator per source, the flags were forced high, which deletes the range check
  entirely. No implementation can beat that, and it measures 88.98–89.41 against 87.43–88.51: four
  placements, no overlap with the baseline, **1.8% slower at the median**. The loop re-terminates at
  the ROM's data register (`imem.even_data`) instead of its range flag, and that ending is longer
  than the comparator's carry chain was. The 17 carry hops the ceiling deletes were worth 5.8 ns of
  88.51.
- **Candidate 2 is inside the noise at best and 6% worse at worst.** Its spread straddles the
  baseline's and is three times as wide. At seed 3 it does what it was supposed to do — the critical
  path leaves the fetch loop — and the design measures 87.43 ns, which is the number the baseline
  already reaches at its own best placement.
- **Together they are 0.9% faster at the median, on distributions that overlap heavily** (84.44–89.00
  against 87.43–88.51). That is inside the 1–2% placement band before ADR-0054's 3.6% edit band is
  considered, and the row contains candidate 1's ceiling rather than candidate 1 — so the number
  describes a design that cannot ship and is not evidence of anything even so.

Area: candidate 2 costs 18 cells and candidate 1's ceiling *saves* 63, both against ADR-0038's ±50
churn floor, so only the second is outside it and only because it is a deletion. `make fit` is
unchanged and was not re-run for candidate 1: its top is `littlecpu` with the memories external
(ADR-0038 decision 1), and `rtl/imemory.v` is not in `FIT_SRCS` at all — a candidate-1 change is
invisible to that number by construction.

## Why candidate 2 cannot win, whatever it is spelled like

Four spellings were built and each measured at two placements. The flat mux is the one the proposal
names, so it is the one that went on to four placements in the table above:

| spelling | ns (two placements) | against baseline |
|---|---|---|
| hold/issue split, priority arms kept, three adders | 88.76 · 86.04 | neutral |
| the same split, plus `(* parallel_case *)` — a flat one-hot mux | 94.10 · 90.03 | **+4%** |
| the same split, with the JALR target sharing the effective-address adder | 92.96 · 94.45 | **+6%** |
| one adder, addends muxed instead of sums | 93.79 · 91.18 | **+4%** |

Three separate mechanisms, all the same shape:

- **A flat one-hot mux routes worse than a priority chain here.** Routing went 54.49 → 59.73 ns while
  logic stayed flat. Six 32-bit sources fanning into a wide OR is more interconnect than a chain abc
  already maps well, and interconnect is 61% of this path.
- **Sharing an adder puts an unrelated cone in series.** Writing the JALR target as
  `mem_addr_calc & ~1` — the same sum a load or a store computes, and the same hardware — costs 5–7%.
  yosys was not sharing them, and making it share puts the load/store address logic on the fetch
  loop. The RTL reads better and the design is measurably worse.
- **Muxing the addends instead of the sums is the wrong way round on this fabric.** A LUT in front of
  a 32-bit adder puts a whole carry chain *after* that LUT; three parallel adders let three chains
  run while decode settles and pay LUT levels only at the mux. The textbook advice assumes a
  balanced-tree adder, and this part does not have one.

The checked-in structure is already the one this fabric wants, and it is a coincidence of the
ADR-0054 refactor rather than something anybody tuned.

## The cap, which is the part worth keeping

**The fetch loop is not uniquely critical. There is a second path within 1.7% of it.** Deleting the
stall term from `next_pc` — throwaway RTL, function-breaking — gives 87.04 ns with the critical path
on `por_done → riscv.csrs.minstret[56]`: decode → `stall` → `instret` → the 64-bit counter's carry
chain. Candidate 2 at seed 3 lands on the same path at 87.43 ns.

So **any change that shortens only the fetch loop is capped at about 87 ns / 11.5 MHz**, and the
baseline already reaches 87.43 at one of its four placements. A 1.7% ceiling is half the edit band;
there is no way to demonstrate such a change even if it works.

Removing the redirect mux entirely — the other throwaway ceiling — gives 77.02 · 78.15 ns, and its
critical path is `pc → riscv.csrs.mscratch[7]`. That 12% is real but it is **not the mux's depth**:
what leaves the loop with the mux is `immediate`, `stall` and `trap_pending`, the whole decode cone
between the fetched instruction and the adders. Those three feed the second and third paths too,
which is why they cluster.

**The head is shared and the tails are not.** Shortening a tail uncovers the next path; the only
change that moves this design's Fmax is one that shortens `imem.in_range → instr → {rs1/rs2,
immediate, hazard}`, and none of that is in either candidate.

## Decision

**No RTL change.** Both candidates are declined on measurement, not on judgement, and the
alternative spellings are declined with them.

**Nothing in `rtl/` or `formal/` ships from this ADR.** The scratch variants live in this pull
request's history. What ships is the tables above, the corrected depth reporting in
`soc/timing_split.py` with its probes, and `SOC_SEED`.

The readability question the ticket raised is answered the same way: the flat mux does not read
better than what is there. The hold/issue split — reset and stall as machine overrides, the five
instruction redirects below them — is arguable and was built; it is a wash on reading and a wash on
timing, so it is not worth the diff. ADR-0054's chain says the priority order plainly and the order
is the thing a reader needs.

## Consequences

- **ADR-0054's "41 logic levels" should be quoted as 25 LUT levels and 17 carry hops.** The
  measurement is unchanged; the reading is corrected, and `make soc-timing` prints both numbers now.
- **The next attempt at this should target the decode head, not the fetch loop's tail**, and should
  expect to fix two paths rather than one. `immediate`'s eighteen-way mux and the `rs1`/`rs2`
  selection feeding `hazard` are where the LUT levels are; both are on more than one near-critical
  path.
- **A single `make soc-timing` run is not a measurement.** `SOC_SEED` exists so a spread can be taken
  with the tracked flow; ADR-0057 needed one and ADR-0054's churn figure is a different axis again.
  A delta has to clear the placement band (1–2%) and the edit band (3.6%), and this design's own
  baseline spans 1.2% across four placements.
- **ADR-0038's 12 MHz intent is untouched.** Nothing here moves the measurement in either direction,
  and ADR-0057 condition 3's question — what to do about the intent when writable text takes the
  design to about 15% short — is still open and still belongs to the change that lands it.
- ADR-0057's own attribution stands and this ADR agrees with it in shape: the expensive things on
  this part are interconnect and cones, not gate counts or level counts.
