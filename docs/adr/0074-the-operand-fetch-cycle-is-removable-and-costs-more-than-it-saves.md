# ADR-0074: The operand-fetch cycle is removable, and it costs more clock than it saves

**Status:** Accepted · 2026-08-03 · *Declines a change to
[ADR-0042](0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md)'s
operand-fetch cycle. Uses [ADR-0070](0070-the-suites-cycles-are-charged-to-a-named-stall-reason.md)'s
per-reason cycle accounting as the before/after instrument. Graded against
[ADR-0066](0066-twelve-megahertz-is-a-requirement.md)'s 12 MHz requirement, which is what declines
it. Its reopening condition is closed by
[ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md): the decode head does not hold the
headroom this ADR defers to.*

## Context

ADR-0042 made the register file read synchronous, which fits the core on the part, and paid a
measured +18.0% in cycles for it: decode presents the address pair, bubbles one cycle, then issues.
That bubble is `operand_stall`, and it has looked like the obvious thing to remove ever since.

The idea is straightforward and does not need a forwarding network. `rs1` and `rs2` sit at fixed
places in every uncompressed encoding, the fetch window already holds the bytes after the
instruction being issued (ADR-0003 reads two adjacent words every cycle), and ADR-0054 publishes the
fetch address a cycle early. So on a cycle that issues, decode can ask the register file for the
*next* instruction's pair instead of the one it is issuing, and the answer arrives with the
instruction rather than a cycle behind it.

ADR-0070 is what made the size of the prize measurable rather than assumed. On 59 programs at
`0314890`: 28632 cycles, 13853 retires, **CPI 2.07**, of which `operand_stall` is charged **4684
cycles, 16.4%**. The decode scoreboard is charged 8381, **1.8× as much**. So removing the
operand-fetch cycle entirely and for free is a 16.4% improvement, not the halving it is easy to
assume from "every instruction costs two cycles".

## Decision

**Not built.** The operand-fetch cycle stays. `rtl/decoder.v` keeps presenting the pair belonging to
the instruction it is deciding about, and `operand_stall` keeps its bubble.

Two variants were built and measured end to end. Both work. The one that does what the idea asks for
misses the board clock, and the one that holds the board clock spends four fifths of the margin for
a benchmark improvement on a suite that is not representative of anything.

## How it was made safe, which is worth keeping

`operand_stall` is `!read_taken || (uses_rs1 && prev_rs1 != rs1) || (uses_rs2 && prev_rs2 != rs2)` —
it compares what the register file was *asked for* last cycle against what the instruction now being
decided about *reads*. That comparison does not care where the request came from. So the request can
be a guess, and the existing test is exactly the check on the guess: right, and the instruction
issues with no bubble; wrong, and it stalls one cycle and asks again, which is the cycle every
instruction pays today.

Two rules make it terminate and keep the contract:

- **A stalled cycle asks for the current instruction's own pair.** `read_rs1 = stall ? rs1 :
  next_rs1`. The pc holds on a stalled cycle, so the same instruction comes back; ask for the guess
  there too and the two take turns forever. `stall` rather than `operand_stall` alone, because
  during a hazard stall the guess is one instruction ahead and would cost an extra cycle after every
  hazard clears — and the hazard column is the biggest one.
- **The register file's contract does not move.** Its answer still belongs to the pair presented the
  previous cycle, and its write-through bypass still selects on its own registered copy of that pair
  (ADR-0064). What changes is the wording of why that is right: it is not that the held pair is the
  presented pair on an issuing cycle — with a guess it is deliberately not — but that
  `operand_stall` lets nothing issue until the held pair is the pair the issuing instruction reads.
  That is what ADR-0064's argument needed all along; the presented-pair phrasing was a coincidence
  of there being no guess.

Both variants keep the register-address mapping in one place: a new `regsel` module taking a 32-bit
word and returning `rs1`/`rs2`, built twice — once for the instruction being issued and once for the
word after it. Written out twice instead, the two copies could disagree, and then a guess would miss
because the decode of the same bytes differed rather than because the flow went elsewhere — a lost
cycle no oracle in the tree would report. `formal/pcloop.sv` builds a third one, because its
`f_may_stall`
over-approximation compares against the *decoded* pair and comparing two guesses against each other
would excuse the pc from advancing on almost every cycle.

## What it measures

Homebrew Yosys 0.67+post (`b8e7da6f`), nextpnr-0.10-108-g68c1acd8, `icetime` from the same install,
everything measured on this tree against `9b1202b`. `soc/timing_sweep.sh` with `SOC_SEEDS` widened;
one placement is a sample.

| variant | CPI | cycles | `make fit` LC | worst placement | MHz range |
|---|---|---|---|---|---|
| baseline `9b1202b` | 2.07 | 28632 | 3880 | 79.77 ns (8 seeds) | 12.54 – 13.45 |
| **guess with compressed fields** | **1.79** | 24736 | 4033 | **FAILS (5 seeds)** | **11.56** – 12.26 |
| guess, uncompressed fields only | 1.85 | 25593 | 3985 | 82.65 ns (12 seeds) | 12.10 – 12.68 |

The first variant reads the guessed instruction's register numbers through a second `regsel`, so a
compressed successor is guessed as accurately as an uncompressed one. It removes 93% of the
operand-fetch cycles: the column goes 4684 → 333. Five placements have recorded numbers — 81.59 ·
81.71 · 83.18 · 83.29 ns and then **`make soc-timing SOC_SEED=5` exiting nonzero at 11.56 MHz**,
which stopped the sweep and is the whole verdict. ADR-0066 makes 12 MHz a requirement, and the step
below it is 6.

The second drops the compressed handling and guesses `instr[19:15]` / `instr[24:20]` flat, so a
compressed successor is guessed as x0/x0 and usually misses. The column goes 4684 → 1188, and it
holds 12 MHz at all twelve placements measured (78.87 · 79.06 · 79.28 · 79.71 · 79.95 · 79.95 ·
80.19 · 80.24 · 80.36 · 80.48 · 80.97 · 82.65 ns).

A third build isolates where the clock goes: the same mux, with the guess wired to a constant x0 so
only the address path changes. Four placements give 78.00 · 78.11 · 78.43 · 79.18 ns, which sits
**inside** the baseline's eight-placement range. So putting the register file's block-RAM address
behind `stall` is not by itself distinguishable from churn. What costs is reading the guess out of
the fetch window, and it costs more than its own logic depth explains — the flat version adds two
five-bit slices off a mux and still moves the median 3.6%, which points at the window's extra fanout
and the placement it forces rather than at the levels. That is the part the idea needs and cannot
drop.

## Why the second variant is not shipped either

It meets the requirement, so the argument has to be made rather than assumed.

**It leaves 0.83% of margin over the board clock, against 4.5% today.** That is inside its own
placement spread (78.87 – 82.65 ns is 4.5% wide) and inside the 3.6% edit-churn band ADR-0054
measured. `make soc-timing` on CI places one seed; it would read 12.55 MHz while seed 2 sat at
12.10. The next unrelated change would trip a requirement for reasons that have nothing to do with
it, and the person making it would be told to fix a design that had already spent the headroom.

**What it buys is smaller than it looks.** 10.6% fewer cycles against 3.6% more period is about 6%
of wall time on the `.S` suite — and ADR-0070's own table says to read that CPI as a property of
this suite, which is small hand-written assembly with dense back-to-back dependencies and almost no
loop structure.

**It is not the change the idea describes.** The compressed field extraction is the hard part and
the part that fails; a version with it removed guesses wrong on every compressed successor and is a
different, weaker thing.

Against that: `make fit` +105 cells (twice the churn floor), a new module, a fetch window that
exports a successor word, and a register-file contract whose correctness argument gains a step. The
rule for amending a design commitment is that a change improves one goal *and the other three still
hold, measured*. Faster in cycles and slower in clock, with the clock being the hard constraint and
the cycles measured on an unrepresentative suite, is not that.

## The finding underneath, which outlives the nanoseconds

**The operand-fetch cycle is not where this suite's time goes, and the two big stall reasons
overlap.** Removing 93% of the operand cycles moved the decode-scoreboard column *up*, 8381 → 8837,
because a RAW hazard that used to be hidden behind an operand bubble now gets charged for itself.
Total non-issuing cycles went 14602 → 10706 and the scoreboard's share went 29.3% → 35.7%. A core
with no operand-fetch cycle at all would still spend 43% of this suite's cycles not issuing.

So the ordering is the other way round from the way the idea is usually stated: the scoreboard is
the expensive reason, the operand cycle is second, and the clock that would pay for either has to
come from the decode head (`imem.in_range → instr → {rs1/rs2, immediate, hazard}`) first. ADR-0058
already names that as the shared lever on every near-critical path, and it is untouched.

## Consequences

- **The idea is not dead, it is blocked on the clock.** Reopen it after a change that buys back
  headroom in the decode head. The size of the prize is written down above (16.4% of cycles, of
  which 93% is reachable) and the mechanism is written down above; neither has to be rediscovered.
  What made this expensive is the fetch window feeding a second register-field decode, so anything
  that shortens the decode head before the window fans out is the enabler.
- **Nothing in `rtl/` changed, and nothing was left behind.** `regsel` has no consumer without the
  guess, so it is not checked in; structure with no reader is what this repo keeps deleting.
- **The variants were verified, not just timed.** The compressed-field variant is 59/59 on `make
  test` with `test/EXPECTED_FAIL` matching, 8/8 on `make test-units`, **85/85 on `make -C formal
  check`** with both set equalities matching in both directions, and `components_decoder`,
  `components_pcloop` and `components_traps` all pass by k-induction. `reg_ch0`'s liveness probe was
  re-run on it — delete the rs2 write-through bypass and it reports `bad state property 1 reachable
  at bound k = 22 SATISFIABLE` — because a change to how the register file is addressed is exactly
  the class that can make that check go green by stopping to ask. The decline is about the clock and
  about nothing else.
- **`make cosim-suite` was not run on either variant.** `tools/sail` is gitignored, so the worktree
  these measurements were taken in had no Sail binary, though the checkout it branched from does.
  It is the leg positioned to see a stale operand rather than a wrong one, so a future attempt at
  this should carry its output, run from a tree that has had `make sail-setup`.
- **F and G were not re-derived**, and the ladder's `[depth]` lines did not move. The reason it was
  safe to run at the shipping depths: a wrong guess raises `operand_stall` for exactly one cycle and
  then the current pair is presented, so the longest run of consecutive operand-fetch cycles is one,
  which is what it is today. A version of this that ships still owes the measurement.
- **ADR-0064's coupling gets a sharper statement even though nothing ships.** The write-through
  bypass is correct because `operand_stall` will not let an instruction issue until the held pair is
  the pair it reads — not because the held pair equals the presented pair. The two are the same
  sentence only while decode asks for the instruction in front of it.
