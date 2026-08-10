# ADR-0094: the compressed decode and the immediate generator are already mapped, and the block is closed

**Status:** Accepted · 2026-08-09 · *A measured null. No RTL changes. Three ceilings say what the
block is worth at all; the one group that looked like an answer reads −50 LUTs on one synthesis top
and −1 on the other, which is the churn band caught in the act.*

## Context

[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md) settled where synthesis is weak.
yosys and ABC already do everything derivable from the text of an expression — dead bits, common
subexpressions, duplicate adders, constant folding. What they cannot use is a fact that lives
*outside* the expression: a parameter is a power of two, a window is aligned, a reversal is wiring,
an address bit is provably zero because a trap guarantees it. That question has since been asked of
`rtl/imemory.v`, `rtl/memory.v`, `rtl/timer.v`, `rtl/accessor.v`, `rtl/executor.v` and
`rtl/decoder.v`'s comparators and address arithmetic, for about −490 SoC cells together
(ADR-0088, [ADR-0090](0090-the-executor-carries-half-the-registers-it-had.md)).

`rtl/decoder.v`'s compressed-instruction expansion and immediate generation is the last large block
that had never been asked it: roughly fifteen immediate forms, each a wiring permutation of `instr`,
muxed into one 32-bit result, plus the quadrant and funct decode that selects among them.

**The honest prior was a null, and it is worth saying why, because it held.** Per output bit an
immediate mux selects among a handful of `instr` bits and ABC maps that shape well. The RVC encoding
does keep a field in the same instruction position across formats — C.J and C.B agree at six of their
low bits, every uncompressed form puts `instr[31]` at bit 31 — but two arms that agree at a bit agree
by having *the same net* on both. That is a common subexpression, not a fact from outside, and the
mapper has it already.

## What was read

- the fifteen immediate forms — five base (I, S, B, U, J) and ten compressed — bit by bit, against
  the `instr` bit each arm contributes at each output position;
- the eighteen-arm one-hot mux over them, and the `$onehot0` assertion that pays for its marking;
- the ~30 compressed flags and the quadrant / `cfunct3` / `cfunct4` / `cfunct6` decode behind them;
- `rtl/regsel.v`, now instantiated twice — over the issuing word and over the fetch window's
  successor
  ([ADR-0093](0093-the-compressed-successor-is-decoded-and-the-compiled-workload-is-what-moves.md));
- every consumer of `immediate`: the effective-address adder, the two-bit misalignment sum, both
  `next_pc` arms that read it, `math_arg`, and the two operand overrides in the publish block.

## Three ceilings, which is what makes this a number instead of an opinion

`make fit` — `littlecpu`, memories external, so this block is in the netlist. The `SB_LUT4` count is
deterministic for a given netlist and is quoted beside the packed cells, because packing adds noise
the mapper's own count does not have. Base is `2f12d64`, and each row is that base with one thing
deleted whole.

| deleted whole | packed LC | `SB_LUT4` | delta LUT |
|---|---|---|---|
| **base** | 3575 | 3311 | — |
| all compressed decode (`quadrant` tied to `2'b11` in `decoder.v` and `regsel.v`) | 3323 | 3058 | −253 (−7.6%) |
| the whole immediate mux (`immediate = i_immediate`) | 3472 | 3210 | −101 (−3.1%) |
| the fetch window's upper-half mask | 3589 | 3324 | **+13** |

Read them as bounds. **Nothing inside the immediate generator can be worth more than 101 LUTs**
against a churn band of about ±50, and the whole C extension is 253 — which is not on the table,
because code density is a product constraint on the up5k
([ADR-0002](0002-isa-target-rv32imc-zicsr.md), [ADR-0003](0003-dual-word-combinational-fetch-window.md)).

**The third row is the one worth keeping.** `instr`'s upper half is masked to zero for a compressed
word, and the obvious reading of ADR-0088's rule says that mask is redundant in `decoder.v`: every
line there that reads `instr[31:16]` is ANDed, directly or through a flag, with `uncompressed`, and
that mutual exclusion is exactly the kind of fact the mapper cannot derive. Deleting it **costs 13
LUTs**. ABC had already folded those sixteen AND gates into the LUTs that read them, and taking them
out only denied it the folding. The mask is load-bearing anyway — `rtl/regsel.v` needs it, because a
compressed encoding with no register falls to the uncompressed field positions and must read `x0`,
and `rvfi_insn` must report a compressed word zero-extended — so it was never going to come out. What
is worth recording is that even its *ceiling* is negative.

## The one group that looked like an answer

Three edits, each stating a fact from outside its own expression, built and measured together the way
ADR-0088 requires — because measured alone every one of that ADR's eleven was inside the band:

1. **A compressed shift's amount carried as an immediate.** Its five bits sit at `instr[6:2]`, which
   the other compressed immediates already read at output bits 0–4, so the mux gains an arm and no
   new data; and the executor reads five bits of a shift's second operand, with nothing else reading
   it at all. Together those collapse
   `out.rs2 <= instr_math ? (instr_math_immediate ? (instr_shift ? {27'b0, rs2} : immediate) : reg_rs2) : reg_rs2`
   to a single select.
2. **`instr_math_immediate` implies `instr_math`** — every immediate form is a term of its register
   form's flag — so the outer select above could never have been the deciding one.
3. **`instr_error` reading the raw `rs1`/`rd` fields instead of the decoded numbers.** A SYSTEM
   instruction is uncompressed, so both register-number muxes fall to their default arms, and the
   illegal-instruction test — which chooses the next pc — need not wait on either.

None of the three was believed. Each was asserted against what it replaced and discharged by
`components_decoder` under `mode prove`: `instr_math_immediate` implying `instr_math`, the shift
amount against `instr[24:20]` or `instr[6:2]` under `uncompressed`, and `rs1 == rs1_field` with
`rd == rd_field` under the SYSTEM condition.

**And the group measures nothing.**

| instrument | base | the group | delta |
|---|---|---|---|
| `make fit` synthesised `SB_LUT4` (`littlecpu`) | 3311 | 3261 | −50 |
| `make fit` packed `ICESTORM_LC` | 3575 | 3526 | −49 |
| `make soc-timing` synthesised `SB_LUT4` (`littlesoc`) | 3914 | **3913** | **−1** |
| `make soc-timing` placed `ICESTORM_LC` | 4331 | 4329 | −2 |

Both `SB_LUT4` counts are deterministic — same yosys, same script, no placement in either — and **the
same edit to the same decoder reads −50 LUTs on one top and −1 on the other.** That is the ±50 churn
band observed at synthesis rather than inferred from placement, and it is the clearest demonstration
in this tree of why a delta at the edge of the band is not evidence of anything: nothing about the
decode cone differs between the two runs, only what surrounds it, and ABC maps it fifty LUTs apart on
that alone.

The period says the same thing from the other side. Six placements a side, `SOC_SEEDS='default 1 2 3 4 5'`:

| | sorted, ns | worst | median | best |
|---|---|---|---|---|
| base | 76.76 77.34 77.94 78.00 78.07 78.31 | 78.31 | 77.97 | 76.76 |
| the group | 75.87 78.30 78.69 79.13 79.54 81.95 | 81.95 | 78.91 | 75.87 |

+1.2% at the median, +4.7% at the worst, −1.2% at the best. Every placement clears `SOC_MIN_MHZ`, the
worst at 12.20 MHz. **Do not read the +4.7% as a cost of the edit** — a netlist one LUT different
cannot cost 4.7% of period. Read it as the ~3.6% edit-churn band plus placement spread, which is what
CLAUDE.md already says those numbers are, and as one more reason a single placement is a sample.

`make cycles` is unchanged to the cycle: 28 555 cycles, 15 654 retires, CPI 1.82, and the same split
across the six stall reasons. Nothing here touches a stall reason, a stage length or the scoreboard,
so F and G are not re-measured and the BMC depth table does not move.

## Three more candidates, and why they were not built

- **One adder for both `next_pc` arms that read `fetcher_pc`.** `fetcher_pc + immediate` and
  `fetcher_pc + pc_inc` are two 32-bit adders muxed after the fact; muxing the operand first would
  delete one of them, worth about −32 cells. It also moves a LUT level *ahead* of a 32-bit carry
  chain, behind `branch_taken`, which is the late input in the fetch loop — today the two adders run
  beside the comparator and only the output mux is after it. A LUT level is ~3.3 ns and 32 carry hops
  are ~11; CLAUDE.md's reading of levels answers this without an instrument, and ADR-0088's own
  counter-evidence is the same shape.
- **A cheaper mapping for the successor's register pair**, since that one is only a guess and a wrong
  guess costs a cycle rather than an answer. That trades area against the cycles ADR-0093 just
  bought, and it would make two copies of one mapping that could disagree — which is the thing
  `rtl/regsel.v` exists to prevent.
- **`clui_immediate != 0` as a reduction over `{instr[12], instr[6:2]}`**, the byte-identical
  `cli_immediate` and `caddi_immediate` merged, and `quadrant == 2'b01 && cfunct3 == 3'b100`
  factored out of the `cfunct4` and `cfunct6` tests that contain it. All three are the same
  expression in the same terms — which is precisely what constant folding, structural hashing and
  CSE are for.

## Decision

**The group is not taken, and this block is closed to ADR-0088's question.** There is no measured win
to weigh, the edit spends a property for it — after it, an `srai`'s second operand carries its
`funct7` above the shift amount instead of zeroes — and CLAUDE.md's standing rule is not to take a
tidier spelling that moves neither instrument. It was built, proved and measured rather than
overlooked, which is why it is written down here.

## What was verified

On the branch that carried the group, before it was reverted — so that the null is a null about
working hardware and not about a change that never ran.

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | all benches pass |
| `make probe-gates` | clean (via `make test`) |
| `make waves` | 79 retires, 79 spec-checked |
| `make cycles` | 28 555 cycles, CPI 1.82, byte-identical to base |
| `make -C formal check` | all checks pass, `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact |
| `components_decoder` | successful proof by k-induction, with the three new assertions |
| `components_executor` | successful proof by k-induction |
| `components_pcloop` | successful proof by k-induction |
| `pcloop_cover` | PASS, both cover goals reached |
| `components_traps` | successful proof by k-induction |
| `make cosim-suite` | 60/62 agreed, `test/COSIM_EXPECTED_FAIL` exact |
| `make lint` | svlint clean in both passes |
| `make elaborate-strict` | clean |

`make -C formal complete` does not return a verdict on this machine and not because of anything here.
`formal/check-abc-engine.sh` now says so in the target's own words — the separately-installed yosys
and sby on this PATH disagree about the `abc -g AND -fast` call sby writes for the `abc bmc3` engine
— and names CI's OSS CAD Suite as where it runs. Nothing in the block this ADR is about reaches that
target's verdict anyway; it is recorded because a missing leg should be named.

## Consequences

- **Nobody needs to read this block again for this reason.** The immediate generator is 101 LUTs
  whole and the compressed decode 253; the band is ±50. A candidate inside it has to argue against a
  ceiling now, not against an intuition.
- **When the two tops disagree at the band, the answer is churn.** `make fit` is a sanity check and
  `make soc-timing` is the design that ships; neither number alone would have said so here. CLAUDE.md
  carries this beside the band it belongs to.
- **ADR-0088's rule survives its first clean null, and is sharper for it.** "Does this state a fact
  from outside the expression?" is necessary and not sufficient — all three edits in the group do
  state one, and all three are worth nothing. The rule tells you what *cannot* pay; only an
  instrument tells you what does.
- Nothing in `rtl/` changed, so no baseline, ratchet, depth or exclusion moves.
