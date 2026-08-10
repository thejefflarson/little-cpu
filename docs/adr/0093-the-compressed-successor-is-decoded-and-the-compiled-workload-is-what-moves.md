# ADR-0093: The compressed successor is decoded, and the compiled workload is what moves

**Status:** Accepted · 2026-08-09 · *Reverses
[ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md)'s decline of
its first variant, on a re-measurement of the placement that declined it. Builds on
[ADR-0089](0089-the-operand-fetch-guess-ships-and-one-workload-feels-it.md), which shipped the
substrate and named this as the version that would move compiled code. Graded against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement, which this clears at ten
placements of ten. Read with
[ADR-0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md)'s rule
about which workload a share belongs to — this time the rule cuts the other way.*

## Context

ADR-0074 built two early-register-read variants and declined both. The cheaper one guesses the
uncompressed field positions flat, so a compressed successor is asked for as x0/x0; ADR-0089 shipped
it and measured what that costs: **9.7% of suite cycles and 1.1% of Dhrystone's.** The entire gap is
that one masking clause. Compiled code is dense with compressed instructions, x0/x0 is right only
when the successor really does read `x0`, and hand-written assembly is mostly 32-bit where the flat
guess is exact.

The version that decodes the compressed successor properly is ADR-0074's first variant. It was
declined because `make soc-timing SOC_SEED=5` exited nonzero at **11.56 MHz**, and ADR-0074
attributed that to "the window's extra fanout and the placement it forces" — a congestion effect
rather than a depth one. Congestion is set by how full the part is, and the part has emptied since:

| | ADR-0074's tree | today |
|---|---|---|
| SoC placed cells | 4769 | 4280 |
| occupancy | 90.3% | **81%** |
| `make fit` | 3958 | 3496 |

**The honest counter-argument is that the period itself did not come back.** `main`'s worst of ten
placements here is 79.30 ns against the 79.77 ns the variant was declined on — 0.6%, which is
nothing. So "the tree got faster" is not available as an explanation, and the two readings could
not be separated by argument. That is why this is a measurement.

## Decision

**Take it.** `rtl/regsel.v` is the register-number mapping for any 32-bit word, and
`rtl/decoder.v` instantiates it twice: once over the instruction being issued, once over the fetch
window's successor word. On a cycle that issues, the register file is asked for the successor's
decoded pair.

```systemverilog
regsel current_regs (.word(in.instr),      .rs1(rs1),      .rs2(rs2));
regsel next_regs    (.word(in.next_instr), .rs1(next_rs1), .rs2(next_rs2));

assign read_rs1 = stall ? rs1 : next_rs1;
assign read_rs2 = stall ? rs2 : next_rs2;
```

**One module instantiated twice, not the mapping written out twice.** Two copies could disagree, and
then the early read would miss because the same bytes decoded differently rather than because the
flow went somewhere else — a lost cycle no oracle in this tree reports. The two marked one-hot case
statements move into `regsel` with the `$onehot0` assertions that make those markings legal, so both
instances carry them and `components_decoder` proves them over an arbitrary word.

**`operand_stall` is unchanged, to the character**, and everything ADR-0089 wrote about why that is
what makes the guess architecturally inert still reads the same. The check compares what the
register file was asked for against what the instruction now being decided about reads; it never
cared where the request came from, and it still does not care that the request is now a better
guess.

**`rtl/fetcher.v` now exports the successor word raw.** The compressed masking moved into `regsel`,
which is the module that decides which fields are register numbers and is therefore the place that
has to know a compressed word's upper half belongs to its neighbour. `regsel` is total: hand it any
32 bits and it answers, and both instances are handed a raw fetched word. Nothing has to be masked
before calling it, which is one fewer precondition than an input contract would have been.

## What it measures

Homebrew Yosys 0.68+post (`c12172fb`), nextpnr-0.10-108-g68c1acd8, `icetime` from the same install,
everything against merged `main` at `9fbee42`. Both sides re-measured on this tree in one sitting;
the base sweep reproduces ADR-0089's ten placements to the hundredth of a nanosecond.

| | cycles | CPI | operand column | scoreboard column |
|---|---|---|---|---|
| `.S` suite, today | 29516 | 1.89 | 1712 (5.8%) | 10027 (34.0%) |
| **`.S` suite, decoded** | **28555** | **1.82** | **753 (2.6%)** | 10027 (35.1%) |
| Dhrystone, today | 2178740 | 2.29 | 641154 (29.4%) | 358928 (16.5%) |
| **Dhrystone, decoded** | **1823126** | **1.92** | **283545 (15.6%)** | 360940 (19.8%) |

**−3.3% of cycles on the suite and −16.3% on Dhrystone.** ADR-0089's table with the two workloads
swapped, and for the reason ADR-0089 gave: the flat guess had already collected the suite's
successors, and what is left over there is 5.8% of its cycles. What is left on compiled code is
29.4%, and both workloads lose the same **56%** of their remaining operand column. Every stalled
cycle is still charged to one of the six reasons; the unattributed column is zero on both.

| | ns, ten placements | median | worst | shipped placement |
|---|---|---|---|---|
| `main` at `9fbee42` | 76.61 · 77.51 · 77.94 · 78.28 · 78.61 · 78.64 · 78.71 · 79.25 · 79.29 · 79.30 | 78.63 ns | 79.30 ns / 12.61 MHz | 79.30 ns / 12.61 MHz |
| decoded successor | 76.76 · 76.98 · 77.34 · 77.49 · 77.94 · 78.00 · 78.07 · 78.07 · 78.31 · 80.47 | 78.04 ns | **80.47 ns / 12.43 MHz** | **77.34 ns / 12.93 MHz** |

**Ten of ten clear 12.0.** The median moves **−0.75%** and the worst placement **+1.5%**, both well
inside the 3.6% churn band and pointing in opposite directions, which is what a null looks like on
this instrument. Nine of the ten sit in a 2% band and the tenth is 2.7% above it; margin over the
requirement is **3.6%** at the worst placement and 7.7% at the one that would be flashed —
`make soc-timing` with no `SOC_SEED`, which is what CI grades and what the bitstream comes from.

`make fit` is **3496 → 3575, +79 cells.** That is outside the ±50 band and is the honest price: a
second copy of the compressed register-number decode, plus whatever a new file's position in ten
source lists costs, since yosys names cells in read order. `FIT_MAX_LC` stays at 3700 with 125 cells
of headroom.

**A spelling that changes no behaviour moved worst-of-ten by 1.3%**, which is worth recording next
to the numbers it could have been quoted as. `regsel` masks a compressed word itself, so passing it
decode's already-masked `instr` and passing it the raw fetched word compute the same function; the
two builds measure 3557 and 3575 cells and worst-of-ten 81.49 ns and 80.47 ns. Same cycles, same
proofs, 1.3% of the number a requirement is graded on. ADR-0089's observation that **worst-of-N is
not a property of the artifact** is the one to read that against.

## Was congestion the constraint? Partly, and not provably

The same design that lost a placement outright at 11.56 MHz now loses none of ten and moves the
median the helpful way by three quarters of a percent. Occupancy fell nine points between the two
measurements and the period did not, so occupancy is the variable that changed in the direction the
outcome changed.

**That is not a controlled experiment and should not be quoted as one.** Three other things moved
between ADR-0074 and here: the datapath itself
([ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md)'s eleven edits and
[ADR-0090](0090-the-executor-carries-half-the-registers-it-had.md)'s executor), the substrate — this
is built on ADR-0089's exported successor word and its `read_rs*`/`rs*` split, not on ADR-0074's
scaffolding — and Yosys 0.67 → 0.68. What the measurement supports is the narrow claim, which is the
one that matters: **the decline did not survive re-measurement, and the reason it was written down —
a placement under the requirement — does not reproduce.** A margin that declines a change is a
measurement with a date on it, and this is the second time in three merges that rule has been
collected.

## Throughput is the product, and the clock term is still a constant

The board runs at 12 MHz. Fmax above the requirement is margin, not speed, so wall time is
cycles × 83.33 ns and the period term does not vary:

| workload | today | decoded | throughput |
|---|---|---|---|
| `.S` suite | 29516 cy = 2.460 ms | 28555 cy = 2.380 ms | **×1.034** |
| Dhrystone (2000 runs) | 2178740 cy = 181.6 ms | 1823126 cy = 151.9 ms | **×1.195** |

Priced instead at each side's own median placement — the reading to use if the clock were free — the
suite is ×1.042 and Dhrystone is ×1.204, because the median moves the helpful way by less than a
percent. The two readings agree to within a percent, and there is no version of this measurement in
which the period eats the win.

`make dhrystone`: **0.535 → 0.640 DMIPS/MHz**, and in absolute terms at the board clock
**6.42 → 7.68 DMIPS**. Same compiler, same flags, same string routines, 3568 bytes of the SoC's 8 KB
ROM in both. 1063 cycles per Dhrystone becomes 889.

## What this bought, said plainly

**A 3.3% cycle win on hand-written assembly and a 16.3% win on compiled C**, which is ADR-0089's
sentence with the workloads exchanged. Read together, the two ADRs price the whole idea: the
operand-fetch cycle was 16.4% of the suite when ADR-0074 measured it and 30.5% of Dhrystone, and
between the flat guess and this one it is down to 2.6% and 15.6%. The remainder is a redirect,
where no guess taken from the fetch window can be right.

ADR-0084's rule is why the suite number is not the headline. On this change it is the small one.

## The correctness argument, which gains nothing and loses nothing

`operand_stall` is the check on the guess and is untouched, so the argument
[ADR-0064](0064-the-write-through-bypass-is-addressed-from-the-held-pair.md) makes as amended by
ADR-0089 stands word for word: the write-through bypass selects on the held pair and is correct
because nothing issues until the held pair is the pair the issuing instruction reads. A better guess
changes how often that test passes and nothing about what it tests.

**`reg_ch0`'s standing liveness probe was run before any of this was believed.** Delete the rs2
write-through bypass and it reports `bad state property 1 reachable at bound k = 22 SATISFIABLE`. A
change to how the register file is addressed is exactly the class that can make that check go green
by stopping to ask.

**`components_pcloop`'s anti-vacuity cover was the one at risk.** Its second goal,
`increment_reached_on_moved_pair`, is design-specific to the guess: it needs a trace reaching the
increment assertion on a cycle where the pair presented had moved. Both goals are still reached at
step 4, unchanged, so the excuse `f_operand_fetch` still compares what was presented against the
decoded pair rather than a guess against itself.

## F and G were re-derived, and neither moved

Nothing here adds a stall reason, lengthens a stage or widens the scoreboard, so ADR-0046's trigger
is not met — but the depths are thin enough that the argument is worth a measurement. Same recipe as
ADR-0089's: a copy of `checks.cfg` with one line in `[depth]`, regenerated, swept.

- **F = 6.** `hang` reports a counterexample at depth 5 and 6, PASS at 7 and 8.
- **G = 6.** `liveness` red at gap 4 and 5, PASS at gap 6 and 7, at trig 10 and again at trig 15.

Both flip points reproduce exactly, so `insn 19` and `reg 15 22` still clear `F + 2G = 18` by the
same one cycle and no depth in the table moves. The reason they cannot move is the one ADR-0074
wrote down: a wrong guess raises `operand_stall` for exactly one cycle, because the next cycle
presents the current instruction's own pair — the longest run of consecutive operand-fetch cycles is
one, whatever the guess is made of.

## Consequences

- **Verified, not just timed.** 62/62 on `make test` with `test/EXPECTED_FAIL` matching, 8/8 on
  `make test-units`, `make probe-gates` green over 188 graded comparisons, **85/85 on
  `make -C formal check`** with both set equalities matching in both directions, and
  `components_decoder`, `components_executor`, `components_pcloop` and `components_traps` each a
  successful proof by k-induction. `pcloop_cover`, `complete_cover`, `nonperturbation`, `imemcheck`
  and `dmemcheck` pass. **`make cosim-suite` is 60/62 before and after**, matching
  `test/COSIM_EXPECTED_FAIL` — the leg positioned to see a stale operand rather than a wrong one.
  `make waves` retires 79 with 20 writes, which is the four-state leg and the only one that could
  have seen the X-propagation defect ADR-0089 found; it is green here because the harness fix that
  ADR shipped is what makes a fetched successor word defined.
- **`make -C formal complete` does not run on this machine** and fails identically on unmodified
  `main`: the local Homebrew yosys rejects abc's `-g AND -fast`. It is covered on CI.
- **`rtl/regsel.v` joins ten source lists**, which is the real cost of a second module in this tree:
  the Makefile's three, six `.sby` files, `formal/checks.cfg`, `formal/Makefile`,
  `formal/check-nonperturbation.py` and `soc/depth/sweep.sh`. Every one of them is exercised by a
  gate above, which is why the list is knowable rather than a hazard.
- **`test/decoder_tb.v` gains three vectors and the red direction was demonstrated.** With
  `read_rs*` put back to the flat slice they report `got=0000001f expected=00000003`,
  `got=0000001f expected=00000002` and a compressed successor paying an operand-fetch cycle it
  should not. The successor word is driven with its upper half set to `0xffff`, so the vector fails
  if `regsel`'s masking is removed as well as if the decode is.
- **The decoder's own `$onehot0` list is three statements now, not five.** The two that pick
  register numbers moved with the case statements they describe; leaving them behind would have
  asserted one-hotness of flags no marked statement reads.
- **The suite's scoreboard column did not move at all** (10027 both sides), and Dhrystone's moved up
  2012 cycles — the same effect ADR-0074 recorded, at a twentieth of the size. A RAW hazard that
  used to hide behind an operand bubble gets charged for itself.
- **What is still on the table is the redirect**, which is now most of what remains of the operand
  column. Nothing in the fetch window can predict it, so that is a different idea and not a tuning
  of this one.
