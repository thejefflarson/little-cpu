# ADR-0112: the per-module census is not a ceiling, the packed cell is not the LUT, and four more blocks are closed

**Status:** Accepted · 2026-08-16 · *A measurement. No RTL changes, no test changes, no baseline
moves. Twenty-four deleted-whole variants were built in a scratch copy of `rtl/` and thrown away.
Four ceilings are positive — deleting the block **costs** cells — two are exactly zero, and one reads
−17 `SB_LUT4` and **+45 packed cells**, in opposite directions on the same netlist.*

## Context

[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md) settled where synthesis is weak, and
[ADR-0094](0094-the-compressed-decode-is-already-mapped-and-the-block-is-closed.md) and
[ADR-0096](0096-the-csr-file-the-soc-glue-and-the-register-file-are-already-mapped.md) settled how to
find out whether a block is worth asking: **read its deleted-whole ceiling first.** A ceiling read is
one synthesis run, discarded, and it is the only thing that says whether a sub-block is worth two
days or zero. ADR-0094 found three edits in the compressed decode that each state a real fact from
outside their own expression and are together worth nothing — because the block's ceilings, 101 LUTs
for the immediate mux and 253 for the whole compressed decode, said so before anyone spent the days.

Four blocks had never been asked. `rtl/accessor.v` and `rtl/timer.v` were read for ADR-0088's
question and never had a ceiling taken at all. `rtl/memory.v`'s gating and the SoC's read-back
*structure* — as opposed to the three-source OR alone, which ADR-0096 priced at its two-LUT floor —
had not either. And the A extension's rows in `rtl/decoder.v` are new enough
([ADR-0106](0106-the-a-extension-is-built-and-the-board-still-closes.md),
[ADR-0109](0109-an-atomic-outside-the-ram-faults-and-the-declined-measurement-did-not-transfer.md))
that nothing had been measured about them beyond the `fit` job's +452 cells for the extension whole.

The estimates in circulation for those blocks came from `synth_ice40 -noflatten` per-module LUT4
counts. **That is a census, not a ceiling**, and the first table is why the distinction is the point
of this record.

## Method

Every number here was taken on base `fe618f4` with the pinned OSS CAD Suite on this machine —
yosys 0.68+48 (`ff5817c34`), nextpnr-0.11-1-g62e659ed, `icetime` from the same suite — in one
sitting. `SOC_PROG` is the default `datainit.c`. Quote them with that tree and that toolchain:
ADR-0097 measured 44 LUTs separating two texts stating one fact, and CI's number will differ from a
local one.

Each row is that base with one sub-block deleted whole — its output tied to a constant, or its arm
dropped — synthesised for the SoC top, placed, and where the block is core-side, synthesised and
packed for `make fit`'s `littlecpu` top as well. The variants lived in a scratch copy of `rtl/`;
nothing was written into the checkout, and nothing is committed.

**A ceiling bounds every spelling of the block it names, and ceilings do not stack** — the rule
[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md) states, because
stacking zeros is zero. Two rows that overlap cannot be added. A row marked *cascading* deletes logic
in other modules that existed only to feed the one named — a bound generous by construction, which is
what a bound is for.

**These are up5k numbers only.**
[ADR-0110](0110-the-dual-core-soc-targets-ecp5-and-one-source-serves-both-parts.md) makes ECP5 a
second part one source has to serve, and ADR-0087 measured the two parts disagreeing in sign. A
closed row here is closed against the part that binds; a candidate taken off this table still owes
the two-part measurement ADR-0110 requires.

**The periods are not readings.** A variant with a block missing is not a design, and its critical
path is not this design's, so they are omitted. The base's own is 80.22 ns / 12.47 MHz at the default
seed.

## The census does not add up, and it is not a bound either

`synth_ice40 -dsp -spram -noflatten -top littlesoc`. The `SB_DFFE*` split is carried here because a
flip-flop with a clock-enable pin is a different animal from a plain one, and CLAUDE.md already
records why.

| module | `SB_LUT4` | `SB_CARRY` | DFF | of which `SB_DFFE*` |
|---|---|---|---|---|
| `regfile` | 2109 | 0 | 1098 | 1024 |
| `executor` | 1182 | 268 | 147 | 147 |
| `decoder` | 1051 | 124 | 186 | 143 |
| `csrs` | 588 | 127 | 259 | 259 |
| **`accessor`** | **534** | **32** | **151** | **31** |
| **`timer`** | **322** | **126** | **161** | **64** |
| `fetcher` | 177 | 28 | 0 | 0 |
| `regsel` | 58 | 0 | 0 | 0 |
| `memory` | 51 | 0 | 1 | 1 |
| `littlesoc` | 45 | 2 | 10 | 6 |
| `writeback` | 40 | 0 | 0 | 0 |
| `littlecpu` | 1 | 0 | 0 | 0 |
| **sum** | **6158** | 579 | 2013 | 1575 |

The same design flattened — which is what `make soc-timing` synthesises, and what places — is **4315
`SB_LUT4` and 4754 packed `ICESTORM_LC`**. The census overstates the netlist by 43%, and unevenly:
`regfile`'s 2109 is four block RAMs that only get inferred once the hierarchy is flat, and `timer`'s
322 understates its real cost by nearly a hundred cells for the reason the next section gives. **A
per-module count is a statement about a module compiled alone.** Do not derive a harvest estimate
from it.

## Packed `ICESTORM_LC` is the number, and it is deterministic

The Makefile's `make fit` header already says to count logic cells from nextpnr and never cell counts
from yosys, because a flip-flop that cannot share a cell with the LUT feeding it takes a whole cell by
itself, and two planning estimates went wrong in opposite directions on that. This round makes the
warning concrete for a ceiling: **one row moves −17 `SB_LUT4` and +45 packed cells.**

That is not placement noise. Four netlists were placed at four seeds each — `default`, 1, 2, 3 — and
every one packed to the identical `ICESTORM_LC` at all four (base 4754 ×4; the whole timer removed,
4337 ×4; its comparator removed, 4634 ×4; `mtime`'s write path removed, 4799 ×4). The period moved
across those seeds, 76.92–80.22 ns on the base alone; **the packing did not move at all.** So the
packed count is a property of the netlist here, and where it disagrees with `SB_LUT4` the packed
count is the one that describes the part.

The ±50 churn band CLAUDE.md records is a band in **cells**, so every "clears the band?" verdict
below is taken on the packed column.

## `rtl/accessor.v`

| deleted whole | `SB_LUT4` | Δ | Δ carry | Δ DFF | placed LC | **Δ LC** | `fit` LC | Δ | clears ±50? |
|---|---|---|---|---|---|---|---|---|---|
| **base** | 4315 | — | — | — | 4754 | — | 3969 | — | — |
| the whole stage, `out` tied off too *(cascading)* | 1635 | −2680 | −484 | −640 | 1759 | −2995 | 1691 | −2278 | — |
| all its own work, `out.rd_data <= in_rd_data` *(cascading)* | 3186 | −1129 | −158 | −410 | 3404 | −1350 | 3305 | −664 | — |
| the request block's three outputs *(cascading)* | 3295 | −1020 | −158 | −390 | 3484 | −1270 | 3525 | −444 | — |
| the `take_*` registers *(cascading)* | 3736 | −579 | −32 | −117 | 4137 | −617 | 3509 | −460 | — |
| the AMO result mux and its 33-bit adder/subtractor | 4077 | −238 | −32 | −9 | 4513 | **−241** | 3703 | −266 | **yes** |
| the load lane shifter and sign extension | 4256 | −59 | 0 | −2 | 4693 | −61 | 3887 | −82 | at the edge |
| the reservation (`rsrv_word`/`rsrv_held`/`rsrv_hit`) | 4326 | **+11** | 0 | −31 | 4731 | −23 | 3862 | −107 | no |

The `launch_is_*` fan-out has no row of its own because it has no logic in it: those nineteen
continuous assigns are struct field reads, and tying them off *is* the request-block row above. The
three cascading rows are what a store's write path costs the whole SoC — with `mem_wstrb` gone,
`rtl/memory.v` becomes a write-only array and yosys deletes both `SB_SPRAM256KA` with it.

**The reservation is a surprise of ADR-0094's shape.** Deleting a 30-bit register, a 1-bit register
and the 30-bit equality reading them **costs 11 `SB_LUT4` on the top that ships**, while saving 76 on
`fit`: opposite signs on one edit, which is what ADR-0094 found and ADR-0096 reproduced twice. The 31
flip-flops do come out on both tops, and on the SoC they are worth 23 packed cells — inside the band.
There is nothing in it.

**The AMO arithmetic is the one accessor row that clears the band alone**, on both tops and by a
factor of four. It is not deletable, being the A extension; 241 cells and 32 carry bits is the budget
for anyone proposing to share `rtl/executor.v`'s merged subtractor with it.

## `rtl/timer.v`

| deleted whole | `SB_LUT4` | Δ | Δ carry | Δ DFF | placed LC | **Δ LC** | clears ±50? |
|---|---|---|---|---|---|---|---|
| **base** | 4315 | — | — | — | 4754 | — | — |
| the whole module, `mem_rdata` and `mtip` tied off | 3969 | −346 | −126 | −161 | 4337 | **−417** | — |
| `mtimecmp`'s byte-write path *(cascading: the register goes with it)* | 4134 | −181 | −64 | −64 | 4507 | −247 | — |
| the magnitude compare (`mtip <= mtime_next >= mtimecmp`) | 4198 | −117 | −67 | 0 | 4634 | **−120** | **yes** |
| the read mux (four words to one) | 4245 | −70 | −3 | 0 | 4713 | **−41** | **no** |
| `mtime`'s byte-write path | 4298 | −17 | 0 | 0 | 4799 | **+45** | no |
| the read mux's zero-gate | 4342 | **+27** | 0 | 0 | 4777 | +23 | no |

### The `SB_DFFE*` census, before any write-path recommendation

CLAUDE.md's standing warning is that a conditional increment on this fabric is a clock enable and not
a mux, so a deleted flop whose LUT was free saves nothing. The timer's 161 flip-flops split **64
`SB_DFFE*` and 97 plain**, and the whole-module row removes exactly those two counts.

- **`mtimecmp`'s 64 bits are the enabled ones.** They hold except on a write, so yosys has already
  spent the entire byte-write mux on 64 clock-enable pins plus four strobe terms. There is no mux
  there to collect — which is why the `mtimecmp` write row's −247 is a cascade: it deletes the
  *register*, and against a constant-zero `mtimecmp` the comparator folds away too.
- **`mtime`'s 64 bits are plain**, because a free-running counter changes every cycle, so its write
  path really is a datapath mux. It is worth **−17 `SB_LUT4` and +45 packed cells** — deleting it
  *costs* the part 45 logic cells, because the mux LUT the write path contributes is a LUT those
  flip-flops were sharing a cell with, and taking it away leaves 62 more flops with nothing to pair
  against.

**So no write-path edit in this file is worth attempting.** One is a clock enable that costs nothing,
and the other is negative on the instrument that counts.

### The packed-cell verdict on the timer

An `ICESTORM_LC` carries one LUT4, one carry bit and one flip-flop, so a carry bit sharing a cell
with the LUT computing its sum is free. The timer's 126 `SB_CARRY` looked like it might therefore be
free, and **it is**: deleting the magnitude compare removes 117 LUTs and 67 carry bits for 120 cells,
so those 67 carry bits are worth **three cells between them**. Read a carry chain here as costing
nothing above the LUTs already beside it.

**But the module is dearer than its 322-LUT census, not cheaper**: 417 packed cells, 8.8% of the
SoC's 4754, against a flattened LUT delta of 346. The 71-cell excess is flip-flops, not carry — 161
of them, and that many could not all find a LUT to share with. The same effect runs the other way
inside the module, and that is the verdict that matters: **the read mux reads −70 `SB_LUT4` and only
−41 cells**, so it does not clear the band after all and is closed, where the LUT count alone would
have sent someone after it.

## `rtl/memory.v` and the SoC read-back structure

| deleted whole | `SB_LUT4` | Δ | placed LC | **Δ LC** | clears ±50? |
|---|---|---|---|---|---|
| **base** | 4315 | — | 4754 | — | — |
| `rtl/littlesoc.v`'s whole read-back path (`mem_rdata = 0`) *(cascading)* | 3931 | −384 | 4392 | −362 | — |
| the three memories' read zero-gates together, the OR left in place | 4302 | −13 | 4737 | −17 | no |
| `rtl/memory.v`'s two range tests (`in_range`, `atomic_supported`) | 4318 | **+3** | 4754 | **0** | no |
| `rtl/memory.v`'s read zero-gate | 4323 | **+8** | 4760 | +6 | no |
| `rtl/memory.v`'s atomic range test alone | 4323 | **+8** | 4758 | +4 | no |

**Three of the five cost `SB_LUT4` to delete; on the packed column two cost cells and one is exactly
zero.** The zero-gates that let the three memories join with an OR instead of a mux are not merely
cheap — deleting them costs, because ABC folds a two-input AND into the LUT that reads it and removing
it only denies it the folding. That is ADR-0094's fetch-window mask again, on a different block.

This closes what ADR-0096 left open. It priced the three-source OR alone at its floor and never
priced the structure feeding it. **The structure as a whole is that OR plus 17 cells of gating, and
the gating cannot be removed for a profit.** The `mem_rdata = 0` row's −362 is the three memories'
entire read logic, not the combine.

`rtl/memory.v`'s two range tests deserve their line: both are reductions over the address bits above
a power-of-two window, held by the `$fatal` checks ADR-0088 put there, and **their combined
deleted-whole ceiling is zero packed cells**. The atomic one ADR-0109 had just added is +4 on its
own. It cost nothing to add and there is nothing to get back.

## The A extension's rows in `rtl/decoder.v`

| deleted whole | `SB_LUT4` | Δ | placed LC | **Δ LC** | `fit` LC | Δ | clears ±50? |
|---|---|---|---|---|---|---|---|
| **base** | 4315 | — | 4754 | — | 3969 | — | — |
| the eleven `instr_amo*`/`instr_lr`/`instr_sc` recognizers *(cascading)* | 3923 | −392 | 4265 | −489 | 3495 | −474 | — |
| the eleven `out.is_*` publish rows *(cascading)* | 3923 | −392 | 4266 | −488 | 3468 | −501 | — |
| the `atomic_stall` term | 4285 | −30 | 4722 | −32 | 3962 | −7 | no |
| the `uses_rs2` atomic terms | 4314 | −1 | 4753 | −1 | 3943 | −26 | no |
| the `instr_amo_op` immediate arm | 4315 | **0** | 4754 | **0** | 3969 | **0** | no |

The first two rows synthesise to the same census to the cell — 3923 LUT4, 656 carry, 934 flops —
because they delete the same cone: with no atomic flags reaching `decoder_out`, the accessor's
reservation, its AMO arithmetic and its held operands all go with them. That is the A extension's
decode ceiling, and it sits beside the accessor's own share rather than adding to it.

**The `instr_amo_op` immediate arm is exactly zero on both tops and in the packing.** Not "inside the
band" — zero, three times over. It is one arm of an eighteen-arm one-hot mux producing a constant,
and yosys folded it before ABC saw it. That is the cleanest demonstration in this tree of ADR-0088's
rule: an arm restating something already derivable is not merely a null, it is a no-op.

## Decision

**No RTL, no test and no baseline changes.** This record is the deliverable. Two candidates are named
with a budget and everything else is closed:

| attempt | ceiling | what it is |
|---|---|---|
| `rtl/accessor.v`'s AMO result mux and 33-bit adder/subtractor | **241 cells, 32 carry** | required function; the budget for sharing `rtl/executor.v`'s merged subtractor with it |
| `rtl/timer.v`'s 64-bit magnitude compare | **120 cells, 67 carry** | required function; the budget for any incremental `mtip` |

**Closed. Nobody needs to read these again for this reason:**

- `rtl/accessor.v`'s reservation — +11 `SB_LUT4` and −23 cells on the SoC against −107 on `fit`.
- `rtl/accessor.v`'s load lane shifter and sign extension — −61 cells, at the edge of the band, and
  every LUT in it is a byte select the ISA requires.
- `rtl/accessor.v`'s `launch_is_*` fan-out — nineteen struct field reads, no logic.
- `rtl/timer.v`'s `mtime` byte-write path — **+45 cells**; its `mtimecmp` byte-write path — a clock
  enable and not a mux; its read mux — **−41 cells**; its read zero-gate — +23.
- `rtl/memory.v`'s two range tests — **0 cells**; its read zero-gate — +6; the atomic range test
  alone — +4.
- The SoC read-back structure — the OR is at ADR-0096's floor and the three zero-gates it needs are
  17 cells together, which two of the three cost more to remove than they save.
- `rtl/decoder.v`'s `instr_amo_op` immediate arm — **0**; its `uses_rs2` atomic terms — −1; its
  `atomic_stall` term — −32.

`SOC_MIN_MHZ`, `FIT_MAX_LC`, the BMC depths, the exclusion sets and every baseline are untouched, and
`git status` is clean of `rtl/`.

## Consequences

- **A `-noflatten` per-module count is not a harvest estimate.** It sums to 6158 against a flattened
  4315, the error is not uniform, and for `rtl/timer.v` it points the wrong way — 322 LUTs against
  417 packed cells. CLAUDE.md carries this beside the two instruments it already warns about.
- **Grade a ceiling on packed cells, not on `SB_LUT4`.** The packing is seed-independent here, four
  netlists at four seeds each, so it is not a sample; and it disagrees with the LUT count in
  magnitude on the timer's read mux (−70 against −41, which changes that row's verdict) and in
  **sign** on `mtime`'s write path (−17 against +45). A LUT the flops beside it were sharing a cell
  with is not a LUT you can spend.
- **Six more ceilings that are zero or a cost.** ADR-0094 had one; this adds `mtime`'s write path at
  +45 cells, `rtl/timer.v`'s read zero-gate at +23, `rtl/memory.v`'s read zero-gate at +6 and its
  atomic range test at +4, and two exact zeros — `rtl/memory.v`'s two range tests together, and the
  decoder's `instr_amo_op` immediate arm. A gate ABC can fold into the LUT reading it is free, and
  hand-removing it is a cost. That is a pattern with seven instances now, not a curiosity with one.
- **A ceiling can disagree with itself across the two tops.** The reservation reads +11 on the SoC and
  −76 on `fit`. ADR-0094's rule stands: grade on `soc-timing`, and when the two disagree at the band
  the answer is churn.
- **A ceiling is perishable.** These were taken on `fe618f4` with the pinned OSS CAD Suite; re-take
  one in the tree you mean to spend it in, the way ADR-0111 already says and CLAUDE.md already says of
  a declining margin. And they are up5k ceilings: ADR-0110 now asks for a two-part measurement of
  anything structural.
- **This is the ceiling read the harvest was waiting on**, in ADR-0111's sense: the queue it prices is
  a queue of measured wins, and this table says which of these four blocks can supply one. Two can,
  with a budget each; the rest cannot, and are named so that nobody re-derives the same zero.
