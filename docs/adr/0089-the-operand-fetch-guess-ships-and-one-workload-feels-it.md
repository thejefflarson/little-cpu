# ADR-0089: The operand-fetch guess ships, and only one workload feels it

**Status:** Accepted · 2026-08-09 · *Reverses
[ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md)'s decline of
its second variant, on a re-measurement of the margin that declined it. Amends
[ADR-0064](0064-the-write-through-bypass-is-addressed-from-the-held-pair.md)'s statement of why the
write-through bypass is correct, without changing the bypass. Read against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement, which this leaves
exactly as it stands, and [ADR-0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md),
whose warning about the suite's share is what this ADR spends most of its length on.*

## Context

ADR-0074 built two versions of an early register-file read and declined both. The one that does what
the idea asks for — decode the guessed successor's register numbers through a second copy of the
compressed register-number mapping — dropped to 11.56 MHz at one placement and is declined by the
requirement. The cheaper one guesses the uncompressed field positions flat, holds 12 MHz everywhere
it was placed, and was declined on **margin**: 0.83% over the board clock against 4.5% for the tree
it was measured on.

That decline was a conservatism about future edits, not a verdict on the trade. Three things have
moved since, and all three had to be re-measured rather than assumed:

- `4a59607` landed eleven cell reductions. `make fit` is 3874 here against ADR-0074's 3880.
- **The tree it was compared against has itself lost margin.** Over ten placements of merged `main`,
  the worst is **81.61 ns — 12.25 MHz, 2.08% over the requirement**. The 4.5% figure in ADR-0074 was
  eight placements of a different tree.
- Nothing in `rtl/` was left behind, so the spelling had to be rebuilt from ADR-0074's own
  description of it. `git log --all -S` finds the ADR and nothing else; there is no branch, no
  stash and no dangling object carrying the build.

## Decision

**Take it.** On a cycle that issues, decode asks the register file for the *next* instruction's
register numbers instead of the pair it is issuing, so the operands arrive with that instruction
rather than a cycle behind it.

`rtl/fetcher.v` exports the 32 bits after the instruction it is presenting — a second slice of the
64-bit shift it already does, not a second read — with the upper half zeroed when those bits are a
compressed instruction, exactly as decode masks its own word. `rtl/decoder.v` reads `[19:15]` and
`[24:20]` out of it:

```systemverilog
assign read_rs1 = stall ? rs1 : in.next_instr[19:15];
assign read_rs2 = stall ? rs2 : in.next_instr[24:20];
```

`operand_stall` is unchanged, to the character. It compares what the register file was *asked for*
last cycle against what the instruction now being decided about *reads*, and that comparison never
cared where the request came from — so the request can be a guess and the existing test is the check
on it. Right, and the instruction issues with no bubble; wrong, and it stalls one cycle and asks
again, which is the cycle every instruction used to pay.

Two rules make it terminate and keep the contract, both ADR-0074's:

- **A stalled cycle asks for the current instruction's own pair.** The pc holds while stalled, so the
  same instruction comes back; guess there too and the two take turns forever. `stall` rather than
  `operand_stall` alone, because during a hazard stall the guess is one instruction ahead and would
  cost an extra cycle after every hazard cleared.
- **The register file does not move.** Its answer still belongs to the pair presented the previous
  cycle, and the write-through bypass still selects on its own registered copy of that pair. What
  changes is the wording of why that is right — below.

## The masking is the whole difference between working and not

Built first without it — the raw successor word, register fields sliced straight out — this is a
**null on the suite and a small loss on several programs**: 32624 cycles to 32560, with the operand
column going *up* (5287 → 5579) and `beq.S` costing 74 cycles more than it does today.

The reason is that the design being replaced is not "every instruction pays a cycle". It pays a
cycle unless the pair it reads is the pair presented last cycle, which for straight-line code is the
*previous instruction's* pair — and in this suite two thirds of instructions already hit that way,
because so many of them read `x0`. A raw slice of a compressed successor is two plausible register
numbers taken from the instruction *after* it. It misses, and it throws away the hit the old
behaviour would have had.

Zeroed, a compressed successor is guessed as x0/x0, which is right often enough to keep most of
those hits and is exactly what ADR-0074 recorded about this variant. With the mask the operand
column goes **5287 → 1650**, which reproduces ADR-0074's 4684 → 1188 as closely as two different
trees can.

## What it measures

Homebrew Yosys 0.68+post (`c12172fb`), nextpnr-0.10-108-g68c1acd8, `icetime` from the same install,
everything against merged `main` at `4a59607`. `soc/timing_sweep.sh` over ten seeds a side.

| | cycles | CPI | operand column | scoreboard column |
|---|---|---|---|---|
| `.S` suite, today | 32624 | 2.08 | 5287 (16.2%) | 9561 (29.3%) |
| **`.S` suite, with the guess** | **29454** | **1.88** | **1650 (5.6%)** | 10027 (34.0%) |
| Dhrystone, today | 2203492 | 2.32 | 671373 (30.5%) | 352874 (16.0%) |
| **Dhrystone, with the guess** | **2178739** | **2.29** | **641153 (29.4%)** | 358928 (16.5%) |

**−9.7% of cycles on the suite and −1.1% on Dhrystone.** Every stalled cycle is still charged to one
of the six reasons; the unattributed column is zero on both.

| | ns, ten placements | median | worst |
|---|---|---|---|
| `main` at `4a59607` | 75.12 · 75.67 · 76.08 · 77.17 · 77.53 · 78.34 · 78.38 · 78.59 · 79.85 · 81.61 | 77.94 ns | **81.61 ns / 12.25 MHz** |
| with the guess | 75.71 · 76.06 · 76.75 · 77.08 · 78.89 · 79.05 · 79.08 · 79.22 · 79.47 · 79.58 | 78.97 ns | **79.58 ns / 12.57 MHz** |

The median moves +1.3% and the worst placement moves −2.5%. Both sit inside the 3.6% edit-churn band
ADR-0054 measured, which makes this **a null in both directions**: not a cost paid and not a saving
banked. `make fit` is 3874 → 3859 cells, inside its own ±50 band and a null on the same reading.

## Throughput is the product, and the clock term is a constant

**The board runs at 12 MHz.** `SB_HFOSC`'s next step up is 24 and the crystal is 12, so a placement
that closes at 13.2 MHz does not run 10% faster than one at 12.0 — it runs at 12.0 with more margin.
Fmax above the requirement is margin, not speed. So wall time is cycles × 83.33 ns and the period
term does not vary:

| workload | today | with the guess | throughput |
|---|---|---|---|
| `.S` suite | 32624 cy = 2.719 ms | 29454 cy = 2.454 ms | **×1.108** |
| Dhrystone (2000 runs) | 2203492 cy = 183.6 ms | 2178739 cy = 181.6 ms | **×1.011** |

Priced instead at each side's own median placement — the reading to use if the clock were free — the
suite is ×1.093 and **Dhrystone is ×0.998**, a wash. Neither reading makes Dhrystone move.

`make dhrystone`: **0.529 → 0.535 DMIPS/MHz**, and in absolute terms at the board clock
**6.35 → 6.42 DMIPS**. Same compiler, same flags, same string routines, 3568 bytes of the SoC's 8 KB
ROM. That is the number this repo publishes, and it moved 1.1%.

## What this really bought, said plainly

**A 9.7% cycle win on hand-written assembly and a 1.1% win on compiled C.** The gap is not noise and
it is not a property of Dhrystone in particular — it is the C extension. Compiled code is dense with
compressed instructions, a guess that reads uncompressed field positions is x0/x0 for every one of
them, and x0/x0 is right only when the successor really does read `x0`. The suite is hand-written
assembly where most instructions are 32-bit and the guess is exact.

So the version that would move compiled code is precisely the one ADR-0074 measured at 11.56 MHz:
the guess that decodes a compressed successor properly. **It is not reopened here.** Today's `main`
is *slower* at its worst placement than the tree that variant was declined on (81.61 ns against
79.77), so there is no reason to think the clock has come back, and finding out costs four
placements and a second copy of the register-number mapping.

ADR-0084's rule earns its keep here in the direction that hurts: read 9.7% as a property of the
suite, never as the prize. The prize is 1.1%.

## Why the SoC_MIN_MHZ policy split was proposed and is not made

The ticket behind this ADR carried a second half: split `SOC_MIN_MHZ`'s two jobs, keeping
worst-of-N as a regression control and grading the requirement on the single placement that actually
ships, because the project flashes one bitstream at one seed and worst-of-N is an order statistic
that gets worse the more seeds you run.

**It is not made, because this change does not need it.** All ten placements clear 12.0, the worst
by 4.75%, and the shipped placement — `make soc-timing` with no `SOC_SEED`, which is what CI grades
and what the bitstream comes from — is **79.05 ns, 12.65 MHz, 5.4% of margin**. That is more margin
at the worst placement than merged `main` has today (2.08%). Changing the grading policy to admit a
design that does not need admitting would be spending a rule to buy nothing.

The observation underneath is still worth writing down, because it will come back: **worst-of-N is
not a property of the artifact.** Run four seeds and this tree's `main` reports 12.72 MHz; run ten
and it reports 12.25. Nothing about the design changed between those two sentences. When a future
change really does sit near the floor, the argument to have is which number the requirement is
about — and the honest answer is the placement that gets flashed, with worst-of-N kept as the
control that catches a bad edit. That argument should be made against a design that needs it.

## What the correctness argument gains, and what it does not

ADR-0064 says the write-through bypass is correct because `operand_stall` guarantees the held pair is
the presented pair on every issuing cycle. **That sentence is now false, and the bypass is still
correct.** The true statement is the one ADR-0074 already wrote out:

> `operand_stall` lets nothing issue until the held pair is the pair the issuing instruction reads.

The two were the same sentence only while decode asked for the instruction in front of it. On an
issuing cycle the presented pair is deliberately something else — the guess for the next instruction
— while the held pair, which is what the bypass selects on, is the issuing instruction's own. The
coupling ADR-0064 named is untouched and if anything more load-bearing: narrowing `operand_stall`
still breaks the register file with nothing to say so but two `test/regfile_tb.v` vectors and
`reg_ch0`.

Those two vectors stop being a curiosity. They point a port somewhere else during the use cycle and
check that the array still answers the fetched address; before this change nothing in the core did
that, and now every issuing cycle does.

**`reg_ch0`'s standing liveness probe was run before any of this was believed.** Delete the rs2
write-through bypass and it reports `bad state property 1 reachable at bound k = 22 SATISFIABLE`. A
change to how the register file is addressed is exactly the class that can make that check go green
by stopping to ask.

## F and G were re-derived, and neither moved

ADR-0074 argued from the shape of the design that the depths were safe and said a version that ships
still owes the measurement. It is paid here, by checks.cfg's own recipe — a copy of the config with
one line in `[depth]`, regenerated, swept:

- **F = 6.** `hang` red at depth 5 and 6, PASS at 7 and 8.
- **G = 6.** `liveness` red at gap 4 and 5, PASS at gap 6 and 7, at trig 10 and again at trig 15.

Both flip points reproduce exactly, so `insn 19` and `reg 15 22` still clear `F + 2G = 18` by the
same one cycle and no depth in the table moves. The reason they do not move is worth keeping: a
wrong guess raises `operand_stall` for exactly one cycle and then the current pair is presented, so
the longest run of consecutive operand-fetch cycles is one, which is what it was before.

## Consequences

- **Verified, not just timed.** 62/62 on `make test` with `test/EXPECTED_FAIL` matching, 8/8 on
  `make test-units`, `make probe-gates` green over 188 graded comparisons, **85/85 on
  `make -C formal check`** with both set equalities matching in both directions, and
  `components_decoder`, `components_executor`, `components_pcloop` and `components_traps` each a
  successful proof by k-induction. `nonperturbation` and `complete_cover` pass.
  **`make cosim-suite` is 60/62 before and after**, matching `test/COSIM_EXPECTED_FAIL` — the leg
  positioned to see a stale operand rather than a wrong one, which ADR-0074 could not run and asked
  a future attempt to carry.
- **`make -C formal complete` does not run on this machine** and fails identically on unmodified
  `main`: the local Homebrew yosys rejects abc's `-g AND -fast`. It is covered on CI.
- **The decoder now publishes both pairs.** `rs1`/`rs2` are what the instruction's encoding names
  and nothing in the datapath reads them; `read_rs1`/`read_rs2` are what the register file is asked
  for. `formal/pcloop.sv` needs both — it registers what was asked for and compares it against what
  is decoded — and building a second copy of the register-number mapping over there is the thing
  that file exists to avoid. Register the guess on both sides of that comparison and the harness
  excuses the pc from advancing on nearly every issuing cycle, which is a proof that has stopped
  asking rather than one that fails.
- **The six stall reasons are still six.** Nothing was added to `stall`, `make cycles` attributes
  every stalled cycle, and `test/decoder_tb.v`'s identity check is unchanged.
- **`test/decoder_tb.v` leaves `in.next_instr` at zero except in the two vectors written for it.**
  Drive it anywhere else and the vector after it issues a cycle earlier than every check expects.
