# ADR-0097: the decode stack pays only in the fetch loop, and that is where it cannot

**Status:** Accepted · 2026-08-10 · *Amends
[ADR-0094](0094-the-compressed-decode-is-already-mapped-and-the-block-is-closed.md). Seven decode
candidates built and measured as a stack: **−117 SoC LUTs, and 103 of them are one edit that misses
12 MHz at every one of six placements.** The other six are a second null. No RTL changes; new decode
vectors, each with a demonstrated red direction.*

## Context

ADR-0094 measured three decode candidates as a group and read `make fit` −50 `SB_LUT4` against
`make soc-timing` −1 as "the churn band caught at synthesis". It then declined four more candidates
on reasoning rather than on an instrument, one of them — a single adder for both `fetcher_pc` arms
of `next_pc` — on a logic-levels argument.

Two things made that unsafe to leave. Both numbers are deterministic and they describe **different
designs**: `make fit` builds `littlecpu` with the memories external, `make soc-timing` builds
`littlesoc` with the memories, the timer and the glue, and ABC maps globally, so the same edit lands
in a different netlist. And this tree's largest area result was a stack of individually sub-band
edits — [ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md)'s eleven, each inside ±50
alone and −169 together, with the divider borrow reading **+24 alone and −66 in the group**. A
group of three declined at the edge of the band is exactly the shape that has been wrong here before.

So all seven were built, and measured as a stack and one at a time, in the design that ships.

## The seven

Rebuilt from ADR-0094's list. 1 to 3 are its group; 4 to 7 are the four it declined without building,
minus the cheaper successor-pair mapping, which stays declined on merit — it trades the cycles
[ADR-0093](0093-the-compressed-successor-is-decoded-and-the-compiled-workload-is-what-moves.md)
bought and re-creates the two copies `rtl/regsel.v` exists to prevent.

1. a compressed shift's amount carried as an immediate, collapsing the operand mux to one select;
2. `out.rs2`'s outer select deleted, because `instr_math_immediate` implies `instr_math`;
3. `instr_error` reading the raw `rs1`/`rd` fields instead of the decoded numbers;
4. one adder for both `fetcher_pc` arms of `next_pc`, muxing the operand instead of the two sums;
5. `clui_immediate != 0` as a reduction over the six bits that carry it;
6. the byte-identical `cli` and `caddi` immediates merged, and their mux arms with them;
7. the compressed MISC-ALU quadrant and funct3 factored out of the `cfunct4` and `cfunct6` tests
   that contain it.

None was believed. Each was asserted in `rtl/decoder.v`'s `FORMAL` block against what it replaced —
the implication in 2, the shift amount against `instr[24:20]` or `instr[6:2]` in 1, `rs1 == rs1_field`
with `rd == rd_field` under the SYSTEM condition in 3, the reduction against the comparison in 5, the
disjointness the merged arm in 6 stopped stating, and the funct4 and funct6 spellings in 7 — and all
of them were discharged by `components_decoder` under `mode prove`.

## What the stack measures

`SB_LUT4` is the mapper's own count and is deterministic for a given netlist; the placed
`ICESTORM_LC` is seed-stable for one netlist, which is what makes it a netlist number rather than a
placement sample. Base is `d24e94e`.

| | SoC `SB_LUT4` | Δ | SoC placed LC | Δ | `fit` `SB_LUT4` | Δ |
|---|---|---|---|---|---|---|
| **base** | 3914 | — | 4331 | — | 3311 | — |
| 1 compressed shift amount | 3900 | −14 | 4320 | −11 | 3281 | −30 |
| 2 `out.rs2` outer select | 3918 | **+4** | 4337 | +6 | 3352 | **+41** |
| 3 `instr_error` raw fields | 3897 | −17 | 4315 | −16 | 3275 | −36 |
| 4 **one `next_pc` adder** | 3811 | **−103** | 4221 | **−110** | 3236 | **−75** |
| 5 `clui` reduction | 3928 | **+14** | 4347 | +16 | 3293 | −18 |
| 6 `cli`/`caddi` merged | 3894 | −20 | 4313 | −18 | 3300 | −11 |
| 7 MISC-ALU factored | 3909 | −5 | 4327 | −4 | 3293 | −18 |
| **all seven** | 3797 | **−117** | 4209 | **−122** | 3232 | −79 |
| the six without 4 | 3886 | −28 | 4304 | −27 | 3268 | −43 |

**The stack clears the band and the reason is one edit.** Take 4 out and −117 becomes −28, which is
inside ±50 and is a null. Two of the seven are positive alone and the sum of the seven solo deltas is
−141 against a stack of −117, so they are sub-additive rather than additive — but nothing here
reproduces ADR-0088's shape, where the group was worth more than its parts. Here the group is worth
what its one real member is worth.

**ADR-0094's own group does not reproduce at its own number.** Rebuilt as candidates 1 to 3 it reads
`fit` −61 against its recorded −50, which agrees, and SoC **−45** against its recorded −1, which does
not. Same three facts, different text, 44 SoC LUTs apart. That does not overturn ADR-0094's reading —
it strengthens the half of it that matters, that a delta at the edge of the band is not evidence of
anything — but the −1 was a sample of a spelling and should not be quoted as the value of those three
facts.

## And the one edit that pays cannot be taken

Six placements a side, `SOC_SEEDS='default 1 2 3 4 5'`:

| | sorted, ns | worst | median | best | worst MHz |
|---|---|---|---|---|---|
| base | 76.76 77.34 77.94 78.00 78.07 78.31 | 78.31 | 77.97 | 76.76 | 12.77 |
| the six without 4 | 74.49 75.09 75.55 76.38 77.50 77.67 | 77.67 | **75.97** | 74.49 | 12.87 |
| **all seven** | 83.91 83.92 84.93 85.20 86.23 86.35 | 86.35 | **85.07** | 83.91 | **11.58** |
| **4 alone** | 84.32 84.83 85.02 85.05 85.20 86.14 | 86.14 | **85.04** | 84.32 | **11.61** |

**The stack misses `SOC_MIN_MHZ` at six placements out of six, and candidate 4 alone misses it at six
out of six with the same distribution.** +9.1% of median period either way; the other six move −2.6%
at the median, inside the ~3.6% churn band and a null in both directions. Nothing is left for the
other six to explain and nothing is an interaction.

ADR-0094 declined candidate 4 by reading logic levels: it moves a LUT level *ahead* of a 32-bit
carry chain, behind `branch_taken`, which is the late input in the fetch loop, and a LUT level is
~3.3 ns against ~11 for thirty-two carry hops. **icetime says exactly that.** The base critical path
is 26 levels, 23 LUT/setup and 4 carry by hop; with candidate 4 it is 25 levels and **no carry hops
at all**, at 3.40 ns each. The carry chain came out of the path and a LUT level went in, which is the
trade CLAUDE.md already names and this is the cleanest instance of it in the tree.

[ADR-0096](0096-the-csr-file-the-soc-glue-and-the-register-file-are-already-mapped.md) measured the
same species from the other end in the same round, and the two are worth reading together. There, a
counter tick moved onto the adder's carry-in frees three cells and misses 12 MHz at six placements of
six, because the mux it deleted was already 128 clock enables rather than logic. Here, an adder
deleted for −103 cells misses 12 MHz at six placements of six, because the mux that replaced it lands
in the fetch loop. **Two edits, both nulls or better on area, both fatal to the requirement, and
neither visible in a LUT count** — what they share is that the cell count answered a different
question from the one being asked. Sweep seeds before believing an area result, including one that
looks free.

## Decision

**Nothing ships from the seven.** Candidate 4 is declined on measurement rather than on reasoning:
it is worth −103 SoC LUTs and it costs the board clock, and 12 MHz is a requirement, not a floor
that slides. The other six are a null on both instruments in both directions, and the standing rule
is not to take a tidier spelling that moves neither — candidate 1 also spends a property for it, as
ADR-0094 said, since after it an `srai`'s second operand carries its `funct7` above the shift amount
instead of zeroes.

What does ship is the graders. Seven decode vectors went in for paths that had none, each
demonstrated red for the reason it was written by mutating `rtl/decoder.v`: the two `next_pc` arms
that add to the fetched pc, `c.lui`'s reserved zero-immediate encoding, an `ecall` encoding with a
register number in a field that must be zero, the RV64-only row above `c.sub`, a compressed shift
with `shamt[5]` set, and a shift's amount reaching the executor with nothing above it — which is the
vector that would notice candidate 1 arriving without the rest of its evidence.

## What was verified

On the branch that carried all seven, before it was reverted, so that this is a result about
hardware that ran and not about a change that was never built. Everything below except the period
sweep was run against the full stack.

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | 9/9 |
| `make probe-gates` | clean (via `make test`) |
| `make waves` | 79 retires, 79 spec-checked |
| `make cycles` | 28 555 cycles, 15 654 retires, CPI 1.82 — byte-identical to base, row for row |
| `make -C formal check` | 85 checks, 85 pass, `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact |
| `components_decoder` | successful proof by k-induction, with the seven new assertions |
| `components_executor` | successful proof by k-induction |
| `components_pcloop` | successful proof by k-induction |
| `pcloop_cover` | PASS, both cover goals reached |
| `components_traps` | successful proof by k-induction |
| `complete_cover` | PASS |
| `nonperturbation` | PASS |
| `make cosim-suite` | 60/62 agreed, `test/COSIM_EXPECTED_FAIL` exact |
| `make lint` | svlint clean in both passes |
| `make elaborate-strict` | clean |

`make -C formal complete` returns no verdict on this machine, for the reason
`formal/check-abc-engine.sh` states in the target's own words; it runs on CI.

Nothing in `rtl/` changed, so no baseline, ratchet, depth or exclusion moves. `FIT_MAX_LC` is
untouched at 3700 — for the record the stack packs to 3496 against base's 3575, so it was never near
it, and the ratchet was not what declined anything here.

## Consequences

- **ADR-0094's null holds, over twice as many edits.** Six behaviour-preserving decode edits, each
  stating a fact from outside its own expression, are worth −28 SoC LUTs together. That block stays
  closed, and it is now closed on two independent measurements instead of one.
- **The fetch loop will not take a LUT level, at any area price.** −103 LUTs is the largest single
  decode edit measured here and it is unaffordable. Read that beside ADR-0088's flattened `next_pc`
  chain, which bought cells inside the band and cost 3–9% of period: two edits, both in this chain,
  both cheaper by cell count and worse by clock.
- **A candidate declined on reasoning is still a candidate.** The reasoning here was right and the
  measurement agrees with it, which is the outcome that costs the least to have checked — but it
  cost four placements' worth of instrument to find out, and the number it produced (+9.1% of median
  period, −103 cells) is what a future reader needs and no argument would have supplied.
- **`make soc-timing`'s `SB_LUT4` is not stable across spellings of one fact**, by 44 LUTs on the
  same three edits. Quote a group's number with the tree and the text it was measured on.
