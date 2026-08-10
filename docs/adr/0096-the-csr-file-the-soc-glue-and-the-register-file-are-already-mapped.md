# ADR-0096: the CSR file, the SoC glue and the register file are already mapped, and all three blocks are closed

**Status:** Accepted · 2026-08-09 · *A measured null. No RTL changes. Six ceilings say what the three
blocks are worth at all; the one group built is −14 SoC LUTs and misses 12 MHz at every one of six
placements, and the reason is that yosys had already spent the mux the edit came to delete.*

## Context

[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md) settled where synthesis is weak.
yosys and ABC already do everything derivable from the text of an expression — dead bits, common
subexpressions, duplicate adders, constant folding. What they cannot use is a fact that lives
*outside* the expression: a parameter is a power of two, a window is aligned, a reversal is wiring,
an address bit is provably zero because a trap guarantees it. That question has since been asked of
`rtl/imemory.v`, `rtl/memory.v`, `rtl/timer.v`, `rtl/accessor.v`, `rtl/executor.v` and
`rtl/decoder.v` for about −490 SoC cells together (ADR-0088,
[ADR-0090](0090-the-executor-carries-half-the-registers-it-had.md),
[ADR-0094](0094-the-compressed-decode-is-already-mapped-and-the-block-is-closed.md)).

Three blocks had never been asked it: `rtl/csrs.v`, `rtl/regfile.v`, and the SoC glue in
`rtl/littlesoc.v` — the three-source read-back bus, whose range decodes were fixed by ADR-0088 but
whose *consumer* was not. They are read here together, because the facts a mapper is least able to
derive are the ones that cross a module boundary, and reading one block at a time cannot find those.

ADR-0094 added the rule this round is graded by: **grade on `soc-timing`, not on `make fit`.** The
same decode edit measured −50 `SB_LUT4` synthesising `littlecpu` and −1 synthesising `littlesoc`,
both counts deterministic, because ABC maps a cone for what surrounds it. `make fit` also builds the
core with the memories external, so it cannot see `littlesoc.v` at all.

## Six ceilings, which is what makes this a number instead of an opinion

`make soc-timing`, base `d24e94e`, local Homebrew yosys 0.68+post. Each row is that base with one
thing deleted whole. The synthesised `SB_LUT4` count is deterministic for a netlist, and the placed
`ICESTORM_LC` came out identical at all six seeds for every variant that was swept, so neither number
is a placement sample. The churn band is about ±50.

| deleted whole | `SB_LUT4` | Δ LUT | placed LC | Δ LC |
|---|---|---|---|---|
| **base** | 3914 | — | 4331 | — |
| **`rtl/csrs.v`**, every output tied off | 3187 | **−727** | 3532 | −799 |
| — its two mandated 64-bit counters | 3574 | **−340** | 4005 | −326 |
| — its address-selected read mux, every register still observed | 3826 | **−88** | 4238 | −93 |
| — its WARL write mux and all four write masks | 3913 | **−1** | 4333 | +2 |
| **`rtl/littlesoc.v`**'s whole read-back path (`mem_rdata = 0`) | 3634 | **−280** | 4067 | −264 |
| **`rtl/regfile.v`**'s write-through bypass and x0 test | 3883 | **−31** | 4292 | −39 |
| — every scrap of its fabric logic, the write-first term too | 3781 | **−133** | 4221 | −110 |

Two rows cascade outside the block they name and are generous by construction: tying `csrs.v`'s
outputs off deletes the decoder's CSR datapath and its trap targets with it, and `mem_rdata = 0`
deletes the three memories' own read logic. Both are still bounds, which is what a ceiling is for.
The other four are clean.

**What the numbers are made of, which the table cannot say.** The CSR file's cells are the two
counters and almost nothing else: 128 flip-flops and two 64-bit incrementers that RV32 M-mode
mandates and conformance will not trade away. The 88 is the gap between its address-selected read mux
and the cheapest combination of the same twelve sources, not what a restructure would collect. The
WARL row is the one to remember — masks that are wiring, and a mux ABC folded into the register
enables long ago. The register file's fabric is what surrounds four block RAMs, and both halves of it
are correctness-load-bearing for reasons CLAUDE.md already gives. `rtl/littlesoc.v`'s own logic is
the three-source OR, a reset generator and two LED flops.

## The three sources are combined at the floor, and any other function costs

`mem_rdata` is `imem | dmem | timer`. Per output bit that is one `SB_LUT4` with an input to spare,
and the imemory term costs one more inside `rtl/imemory.v`. Six inputs decide a bit —
`data_hit`, `data_hit_odd`, the two ROM bank outputs, the RAM's and the timer's — so **no arrangement
of them fits in fewer than two four-input LUTs**, and the shipping one is two. Moving the imemory's
zero-gate into the SoC as a select (`data_hit ? window : dmem | timer`) is the same two.

The control for that is measured rather than argued. Replacing the OR with an XOR — equally correct
on three sources that are pairwise zero, equally one layer deep — **costs 19 LUTs**: 3933 against
3914, 4353 placed against 4331. ABC absorbs an OR into the gates feeding it in ways it cannot absorb
an XOR. A combine that is already below a neutral function of the same three inputs has nothing in
it.

## The three candidates, each stating a fact from outside its own expression

Built and measured together the way ADR-0088 requires, because measured alone every one of that
ADR's eleven was inside the band and the divider's borrow was +24 alone and −66 in the group.

1. **The counter tick rides the adder's carry-in.** `mcycle_tick ? mcycle + 64'd1 : mcycle` reads as
   a 64-bit incrementer and a 64-bit mux; this fabric gives a carry chain its carry-in for nothing,
   so a conditional increment is `mcycle + {63'b0, mcycle_tick}` and the mux is not built at all.
   That is a fabric fact, the same shape as "a reversal is wiring".
2. **`warl`'s default arm is never observed.** Every reader of `warl` gates on `wen` and on an
   address the case already names — the six register writes, the four counter overrides, the RVFI
   `mscratch` report — so the value it takes at any other address cannot be seen, and the read mux
   need not feed the write mux. That is an observability fact, which no pass in this flow computes.
3. **The privileged spec's CSR numbers put a group's members a couple of address bits apart.** The
   four counter halves are `0xB00`, `0xB02`, `0xB80` and `0xB82` — one group test and two address
   bits, not four twelve-bit comparisons; `mscratch`/`mepc`/`mcause`/`mtval` are `0x340`–`0x343` the
   same way, and so are `mstatus`/`misa` and `mie`/`mtvec`. The grouping is exact: each group's
   membership is precisely the implemented addresses it covers, so `rdata` and `implemented` are
   preserved address for address.

### And two of the three measure nothing, while the third costs

| variant | soc `SB_LUT4` | Δ | soc placed LC | fit `SB_LUT4` | fit LC |
|---|---|---|---|---|---|
| base | 3914 | — | 4331 | 3311 | 3575 |
| 1 alone — the carry-in | 3911 | −3 | 4330 | 3320 | 3583 |
| 2 alone — `warl`'s default | 3907 | −7 | 4326 | 3291 | 3555 |
| 3 alone — the grouped read select | 3963 | **+49** | 4382 | 3351 | 3615 |
| 3, counter halves only | 3936 | +22 | 4353 | — | — |
| **the group, 1 + 2** | 3900 | **−14** | 4318 | 3300 | 3563 |
| the group, 1 + 2 + 3 | 3931 | +17 | 4349 | — | — |

Candidate 3 is ADR-0088's counter-evidence again, on a block that had not had it: the grouping is a
real fact, ABC had already found it, and hand-writing it only denied the mapper the sharing it was
doing. It costs on **both** tops this time, +49 on the SoC and +40 on `fit`, and it still costs +31
inside the group — so the "measure the group, not the part" rule does not rescue it.

Candidates 1 and 2 together are inside the ±50 band on both tops. That is a null on area. It is not a
null on the period.

## The carry-in is the finding, and it is the sharpest one here

Six placements a side, `SOC_SEEDS='default 1 2 3 4 5'`, icetime nanoseconds sorted:

| | sorted, ns | worst | median | best | worst MHz |
|---|---|---|---|---|---|
| base | 76.76 77.34 77.94 78.00 78.07 78.31 | 78.31 | 77.97 | 76.76 | 12.77 |
| 2 alone — `warl`'s default | 74.44 76.04 76.57 76.91 77.42 79.67 | 79.67 | 76.74 | 74.44 | 12.55 |
| 1 alone — the carry-in | 85.16 86.49 86.97 87.42 87.94 87.98 | 87.98 | 87.20 | 85.16 | **11.37** |
| the group, 1 + 2 | 93.22 94.08 94.69 95.53 95.69 97.05 | 97.05 | 95.11 | 93.22 | **10.30** |

**The carry-in edit misses 12 MHz at six placements out of six, for three cells.** So does the group
that carries it. This is not the ±3.6% edit-churn band and it is not placement spread: +11.9% of
median alone and +22.0% in the group, reproduced at every seed, with the same critical path endpoints
as base — `imem.in_range` to the ROM's read data, the fetch loop, three LUT levels deeper at the same
seed (26 against 23).

The census names the one structural difference. Base synthesises 687 `SB_DFFESR` and 154 `SB_DFFSR`;
with the carry-in it is 559 and 282 — **exactly 128 flip-flops, the two counters, moved from enabled
to plain.** `tick ? x + 1 : x` was never an incrementer and a mux. yosys reads it as "this register
holds unless `tick`", puts `tick` on 128 clock-enable pins, and the mux costs nothing at all. Putting
the tick on the carry-in takes that away, hands ABC 128 more datapath inputs to place, and the fetch
loop pays for it.

That is the same lesson as ADR-0088's `mul_div_store` from the other side. **Hand-narrowing a
datapath is arguing with a pass that has already run; so is hand-building a conditional increment.**
The difference is that narrowing `mul_div_store` merely bought nothing, and this buys three cells and
the board clock.

## Decision

**Nothing is taken, and all three blocks are closed to ADR-0088's question.** Candidate 3 costs on
both instruments. Candidate 1 costs the 12 MHz requirement at every placement measured. Candidate 2
is −7 LUTs, well inside the band, and it spends a property the file states — `warl` is today defined
for every address, which is what lets the RVFI report say what landed rather than echo an operand a
mask threw away — so CLAUDE.md's standing rule against a tidier spelling that moves neither
instrument declines it. `SOC_MIN_MHZ` and `FIT_MAX_LC` do not move; neither does anything in `rtl/`.

## What was verified

The group (candidates 1 and 2) was built and run before it was reverted, so that the null is a null
about working hardware and not about a change that never ran: `make test` 62/62 with
`test/EXPECTED_FAIL` exact, and `make cycles` **28 555 cycles, 15 654 retires, CPI 1.82 and the same
split across the six stall reasons — byte-identical to base**, which is the check that both edits are
behaviour-preserving. Candidate 3 was measured but is a read-path restructure that costs on both
tops, so it was declined on the instrument rather than carried through the suite.

The gates below are the tree that ships, which is base plus this file.

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | all benches pass |
| `make probe-gates` | clean (via `make test`) |
| `make window-test` | clean (via `make test`) |
| `make waves` | 79 retires, 79 spec-checked |
| `make cycles` | 28 555 cycles, CPI 1.82 |
| `make fit` | 3575 of 3700 cells |
| `make soc-timing` | 12.77–13.03 MHz over six placements, `SOC_MIN_MHZ` 12.0 held at all six |
| `make -C formal check` | 85 checks, 85 pass, `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact |
| `components_decoder` | successful proof by k-induction |
| `components_executor` | successful proof by k-induction |
| `components_pcloop` | successful proof by k-induction |
| `pcloop_cover` | PASS, both cover goals reached at step 4 |
| `components_traps` | successful proof by k-induction |
| `make cosim-suite` | 60/62 agreed, `test/COSIM_EXPECTED_FAIL` exact |
| `make lint` | svlint clean in both passes |
| `make elaborate-strict` | clean |

`rtl/regfile.v` was edited to take its two ceilings, so its standing liveness probe was run: deleting
the rs2 write-through bypass takes `reg_ch0` **SAT at k = 22**, which is the bound CLAUDE.md records
for it. The check is live and the 31- and 133-LUT ceilings were taken against a design it can see.
(The probe run then exits non-zero on a missing `btorsim`, which writes the waveform after the
verdict; the verdict is in the log.)

`make -C formal complete` does not return a verdict on this machine and not because of anything here
— `formal/check-abc-engine.sh` says so in the target's own words, and CI's pinned OSS CAD Suite is
where it runs.

## Consequences

- **Nobody needs to read these three blocks again for this reason.** `rtl/csrs.v` is 727 LUTs whole
  and 340 of that is the counters the spec mandates; its read mux is 88 from the floor and its WARL
  logic is 1. `rtl/regfile.v` is 133 LUTs of fabric whole and 31 of that is the bypass. The SoC's
  read-back combine is two four-input LUTs per bit against a six-input cone, which is the floor, and
  a neutral function of the same three inputs costs 19 more. A candidate in any of them has to argue
  against a ceiling now.
- **A conditional increment on this fabric is a clock enable, not a mux.** Anywhere `en ? x + k : x`
  appears, yosys has already spent it on the flops' clock-enable pins for no logic at all; rewriting
  it to ride the carry-in is a measured 12 MHz failure at six placements of six. The rule generalises to any
  hand-written "the fabric gives me this for free" edit over a register's next-state expression:
  check the `SB_DFFE*` census before believing the LUT count.
- **ADR-0088's rule survives a second clean null and gains a sign.** All three candidates state a
  fact from outside their expression and all three are worth nothing or less. The question remains
  necessary and not sufficient — but this round adds that an edit inside the band on area can still be
  outside it on period, in the direction that loses the board clock, so an area candidate is not
  exempt from a seed sweep.
- **Grade on `soc-timing`.** Candidate 2 reads −20 on `fit` and −7 on the SoC; candidate 1 reads +9 on
  `fit` and −3 on the SoC, opposite signs on the same edit. ADR-0094's finding reproduces here twice.
- Nothing in `rtl/` changed, so no baseline, ratchet, depth or exclusion moves.
