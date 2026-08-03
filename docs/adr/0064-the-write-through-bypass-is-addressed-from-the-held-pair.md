# ADR-0064: The write-through bypass is addressed from the held pair, and invariant 6 now leans on invariant 9

**Status:** Accepted · 2026-08-02 · *Implements
[ADR-0062](0062-twelve-megahertz-is-reachable-and-the-bypass-select-is-the-cost.md)'s single
recommendation. Records the coupling that ADR-0062's consequences section asked the implementing
change to write down. Moves `SOC_MIN_MHZ`, last set by
[ADR-0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md).*

## Context

ADR-0062 probed the SoC's critical path level by level and found it: `rtl/regfile.v`'s
write-through bypass selects on `rs1`/`rs2`, which are instruction bits out of the word the fetch
just returned. That puts the bypass mux — and the four branch magnitude comparators behind it —
*after* the instruction rather than beside it, and every branch pays for the whole chain before the
next PC can be chosen. Deleting the bypass outright reaches 20 LUT levels against a baseline of 29,
which bounds what any safe spelling can buy.

Invariant 9 already says the registered read answers the address pair presented in the *previous*
cycle, and `operand_stall` holds the PC until the pair the issuing instruction needs is the pair
that was presented. So a registered copy of that pair selects the same value with a select that
comes out of flip-flops.

## Decision

`rtl/regfile.v` registers `rs1`/`rs2` into `held_rs1`/`held_rs2` on the same edge that captures
`read_a`/`read_b`, and the read mux — both the bypass comparison and the x0 test — reads the held
pair. The write-first term inside the `always_ff` keeps using the presented `rs1`, because that is
the address being presented on that edge; it does not move.

Four new lines of RTL and two changed ones. No new stall reason, no flush, no forwarding network:
the two forwarding points are the two that were already there, re-addressed.

The decoder keeps its own `prev_rs1`/`prev_rs2` for `operand_stall` and the regfile registers the
pair again rather than reading them. Two five-bit flops are cheaper than two more ports, and a
module that answers its own read contract without a signal from another file is the one worth
having.

## What it measures

`soc/timing_sweep.sh`, four placements each, **Homebrew Yosys 0.67+post (`b8e7da6f`)**,
nextpnr-0.10-108-g68c1acd8, `icetime` from the same install. Machine load 5–9 throughout with a
sibling agent building; `icetime` is a static analysis and nextpnr is seeded, so load moves the wall
time and not the numbers. Both rows are measured on **this** tree (`a28536b`), so this is a
same-tree comparison rather than one against ADR-0062's older base.

| variant | SoC LC | `icetime` ns, four placements | MHz |
|---|---|---|---|
| baseline `a28536b` | 4314 | 95.74 · 96.67 · 98.06 · 99.47 | 10.05 – 10.44 |
| **held-address bypass** | **4383** | **74.34 · 75.81 · 76.88 · 78.80** | **12.69 – 13.45** |

Median −21.6%. The distributions do not overlap and the move is six times the 3.6% edit band
(ADR-0054), so it is the change rather than churn. **Every placement clears 12.0 MHz**, which is the
board crystal and the part oscillator's third divider step — ADR-0062 is where that stops being a
round number.

This tree measures better than ADR-0062's spike did (12.32 – 12.91 MHz there). Same toolchain,
different base: the spike was taken on the ADR-0059 tree, before `fence.i` serialization landed.
Two four-placement samples 3% apart is what the churn bands describe.

`make fit` — the core-only top, a different design from the SoC — goes **3899 → 3880**, which is
inside the ±50-cell churn floor and means nothing in either direction. The SoC top's +69 is the
number that moved, and it is well inside its 5280.

## `SOC_MIN_MHZ` goes to 10.9

The floor sits **11.5% below the worst of four placements**, which is the margin the 10.0 floor was
derived with (ADR-0059) and is wider than the 3.6% edit band by enough to survive a resynthesis.

It is derived from **ADR-0062's 12.32 MHz worst placement, not this tree's 12.69** — 10.90 rather
than 11.23. Two samples exist for the same change and the floor takes the lower one, so it does not
depend on having got the better of the two. That leaves 14.1% of headroom against what this tree
actually measures, which is also the slack for CI's pinned OSS CAD Suite being a different
synthesiser build (ADR-0052 measured 21 cells between two yosys builds on identical RTL; the same
axis has never been measured for timing).

The floor is still a regression catcher and still not the 12 MHz target. What changed is that the
target is now met, so a red here means something broke rather than that the old shortfall is still
there.

## The coupling, which is the part that outlives the nanoseconds

**Invariant 6 now depends on invariant 9.** Before this change the write-through bypass was correct
for *any* address pair — it compared `waddr` against the address being asked about right now. After
it, the bypass is correct **because** `operand_stall` guarantees that on an issuing cycle the held
pair is the presented pair.

A later change that narrowed `operand_stall` — skipping the bubble for some instruction class, say,
or reusing a read across two instructions — would leave the bypass answering with the previous
instruction's operand. Nothing would say so. It is not visible from reading `rtl/regfile.v`, it is
not visible from reading `rtl/decoder.v`, and it is exactly the shape of the non-local rules
invariant 8 already carries.

Three things record it:

- **CLAUDE.md invariant 6** states the dependency and invariant 9 carries the warning, next to the
  existing statement that `operand_stall` is what makes invariant 9 true.
- **`rtl/regfile.v` says it at the site**, in the two sentences the comment rule allows for a
  tripwire: what the held pair buys, and what narrowing `operand_stall` would do to it.
- **`test/regfile_tb.v` pins the spelling by mutation.** Two vectors drive the presented address
  away from the held one *with a write in flight to the held one*, which is the one situation the
  two spellings disagree about. Reverting either select to `rs1`/`rs2` makes both red. Every other
  vector in that bench holds the pair and cannot tell them apart, which is why the vectors had to be
  added rather than assumed.

## The bench assertion that had to change, and why it fired

ADR-0062 predicted `make test-units` would catch exactly one assertion. It did:
`x0 reads 0 in the fetch cycle (rs1)`, returning `0x55555555`.

That is not a defect and it is not a workaround to rewrite it. The read mux forces x0 to zero off
the *held* address now, like every other read, so the cycle that first presents x0 still answers the
address held from the vector above it — x6, whose value is `0x55555555`. The zero arrives one cycle
later, and the bench's own `x0 reads 0 in the use cycle` assertion is unchanged and still passes.

The general statement is that after this change `reg_rs1`/`reg_rs2` are **never** keyed to the
address presented this cycle, in any cycle. That is invariant 9's claim about the use cycle,
arriving one cycle earlier. Nothing in the core reads an operand in a fetch cycle — `operand_stall`
is high through one, so no instruction issues and `next_pc` holds — so `test/regfile_tb.v` is the
only consumer in the tree that can observe it. The assertion is rewritten to say what it now pins
rather than deleted.

## Consequences

- **`reg_ch0` is still asking.** ADR-0040's liveness probe — delete the rs2 write-through bypass —
  was re-run against this change and still produces its counterexample at k = 20. A change that
  touches the regfile's forwarding is exactly the class that could make that check go green by
  stopping to ask, so the probe is re-run rather than the PASS being quoted.
- **The formal ladder needed no change.** `operand_stall` and the fetch/decode lockstep are
  unchanged, so `F` and `G` are unchanged and no `[depth]` line moved. The held pair is state the
  ladder sees like any other register.
- **`make cosim-suite` is the leg that owed this one**, and ADR-0062 could not run it. It reads the
  real `regs_a` array rather than the core's self-report, which makes it the one oracle positioned
  to see an operand that is stale rather than wrong. 56/56 agree.
- **ADR-0038's 12 MHz intent is met and this ADR does not re-declare it.** ADR-0062 measured that it
  was reachable; this one lands the change. Whether the declaration should now become a requirement
  is a separate question and nothing here answers it.
- **The next path is not this one.** ADR-0062's second-path probe puts the remaining critical path
  around 78–82 ns on the tree it measured, ending in the CSR writes and `minstret` rather than in
  the fetch loop, and found the counter split and the registered steal to be measured nulls. There
  is no obvious next change, and the design has budget it did not have.
