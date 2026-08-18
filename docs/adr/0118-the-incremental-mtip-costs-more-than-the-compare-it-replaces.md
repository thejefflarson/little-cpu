# ADR-0118: the incremental `mtip` costs more than the compare it replaces, and the timer is interrogated

**Status:** Accepted · 2026-08-17 · *A measurement. `rtl/` does not change. `test/timer_tb.v` gains a
crossing vector and a check that `mtip` is never early, and `test/mutations/` gains the mutation that
forces both red. Closes the last candidate on
[ADR-0112](0112-the-per-module-census-is-not-a-ceiling-and-four-more-blocks-are-closed.md)'s table.*

## Context

ADR-0112 read twenty-four deleted-whole ceilings and left exactly one SoC-side row worth attempting:
`rtl/timer.v`'s 64-bit magnitude compare, **120 packed cells and 67 carry bits** on `fe618f4`. It is
required function, so that number is a budget and not a saving — the question it poses is whether
`mtip` can be produced *incrementally*, from the increment that moves `mtime` one step, instead of by
a fresh 64-bit compare every cycle.

Two constraints came with the question and neither moved. `mtip` stays **registered**: it reaches the
core as an input of the decoder's trap term, which is on the fetch loop, and an unregistered compare
would put a carry chain there. And a change in the comparison may reach `mtip` **late but never
early** — the privileged spec guarantees only that it is reflected eventually — so a scheme that
fires a tick before the deadline software armed is wrong however cheap it is.

## Method

Base `16c5cad`, the cached OSS CAD Suite on this machine — yosys 0.68+48 (`ff5817c34`),
nextpnr-0.11-1-g62e659ed, `icetime` from the same suite — `SOC_PROG=datainit.c`, in one sitting.
Every variant was built in a scratch clone and thrown away; nothing of either is committed.

Area is nextpnr's `ICESTORM_LC` out of the placement, which is the unit the ±50 band is in, with the
`SB_LUT4` and `SB_CARRY` counts beside it because the two disagree. The placed count is identical at
all sixteen placements of each side, so it is a property of the netlist and not of the seed.

Timing is `soc/baseline_sweep.sh` at **sixteen seeds a side, paired per seed**. The two sweeps differ
in their base commit — a before-and-after of an RTL change always does, since both sides were
committed to stamp a clean tree — so `soc/baseline_summary.py` was passed `--allow-mismatch` with
that as the only recorded difference.

## The budget is 88 cells on this tree, not 120

The ceiling was re-taken rather than quoted: `mtip` tied low places at **4605** against the base's
**4693**, −87 `SB_LUT4` and −64 carry bits. A ceiling is perishable, and this one lost a quarter of
itself in five merges.

## The shape the brief sketched is not expensive, it is wrong

A sticky `mtip` that sets on the crossing and is recomputed only after a store cannot see a level
that is **already true when nothing crosses it**. `mtime` and `mtimecmp` both reset to zero, so the
level holds from the first cycle and no crossing ever follows; the bit stays low until software
writes one of the two registers. `test/timer_tb.v`'s first `mtip` vector and the new level check both
go red at cycle one, in the same run. **A level is not an edge**, and the repair for that is the
ability to answer "is the level true" without a crossing — which is the compare. Priced anyway,
with a wrap clear it also needs: **+56 `SB_LUT4`, +57 placed cells**.

## What was built

Three registered partial compares, exact for the values the two registers hold: `lo_ge`, `hi_gt`,
`hi_eq`. A tick moves them with **equality only** — `mtime + 1 >= mtimecmp` differs from
`mtime >= mtimecmp` exactly when the two are equal, and a carry into the high half raises `hi_gt`
exactly when the halves were equal. A store is the one event that needs magnitude, because it can
install any value with no crossing behind it, and it is repaired on the cycle *after* the store by a
single 32-bit compare whose two operands are muxed by a registered word select.

That repair is free of lateness rather than merely permitted to be late: both sides of the shipping
compare already come out of flip-flops, so the present spelling does not see a write until the cycle
after it lands either. `mtip` is **bit-identical to the shipping design, cycle for cycle**, and every
vector in `test/timer_tb.v` passes unchanged against both.

## The measurement

| variant | `SB_LUT4` | `SB_CARRY` | placed LC | median Δ period | sign test | worst of 16 |
|---|---|---|---|---|---|---|
| base `16c5cad` | 4254 | 688 | **4693** | — | — | **12.21 MHz** |
| the compare deleted whole (not a design) | 4167 | 624 | 4605 | — | — | — |
| sticky on the crossing | 4310 | 688 | 4750 | not swept | — | wrong at reset |
| **partial compares, the candidate** | 4359 | 656 | **4831** | **−1.01%** | 9 faster / 7 slower, p = 0.80 | **12.42 MHz** |

**The period is a null in both directions.** The median moved −1.01%, inside the ~3.6% churn band,
with the sign test at nine of sixteen; both sides hold 12 MHz at every placement, and the worst
placement moving 12.21 → 12.42 MHz is inside the 1–2% placement spread. The candidate's worst path
is *named* after `mtip` at ten of sixteen placements, but a generated cell's prefix is ancestry and
not ownership — both sides end at the ROM address at the same 21–23 LUT levels, so it is the same
fetch loop with a remapped start cell.

**The area is not.** +138 placed cells, spent to save at most the 88 the whole compare is worth: the
candidate costs **1.6×** the budget it was drawn against.

## Why every incremental spelling loses this way

A magnitude compare on this fabric **is a carry chain**, and ADR-0112 already measured what that
means: 67 carry bits are worth about three cells between them. Producing `mtip` incrementally
replaces the chain with three things that are all LUTs — equality over both halves every cycle, which
is carry-free but no cheaper per bit; a magnitude compare that a store still needs; and 64 LUTs of
operand mux to share that compare between the halves. Not sharing it costs a second 32-bit chain,
which is the shipping design with registers bolted on.

The trade is **carry for LUTs**, and it is the area form of the rule this repo already reads periods
by: a LUT level costs ~3.3 ns and a carry hop ~0.34.

## Decision

**`rtl/timer.v` keeps `mtip <= mtime_next >= mtimecmp`.** Nothing in the timer moves.

**The timer is interrogated.** Every sub-block of the module now carries a measurement: ADR-0112's
read mux (−41 cells), `mtime` write path (**+45**), read zero-gate (+23) and 64-bit compare (−120
there, −88 here), and this record for the compare's only remaining alternative. `mtimecmp`'s byte
writes were answered by the `SB_DFFE*` census — 64 clock-enable pins with no mux to collect.
Reopening this module for area needs a new fact about the fabric, not a new spelling.

## What ships instead

`mtip`'s earliness had no grader; that is what this ticket found that is worth keeping.

- **A crossing vector.** `mtimecmp` parked four ticks ahead of `mtime` with no store near it — the
  wait software actually performs — and `mtip` checked low on each of the three ticks before the one
  that reaches it. The tick before is the one an early compare fires on.
- **A level window check**, running over every quiet cycle of the whole bench. It reads the level off
  the two architectural registers, skips the cycle after a store and the cycles the level itself
  moved across, and requires `mtip` to match in both directions — with a coverage guard, so a bench
  that stopped reaching one of the two arms is red rather than quietly half a check.
  A scheme that produced `mtip` a cycle late still passes it, which is what makes it a check on
  earliness rather than on one spelling.
- **`test/mutations/mtip-fires-a-tick-early.patch`**, paired with `timer_tb` and nothing else. That
  pairing is the measurement: the whole `.S` suite stays green under an interrupt that arrives one
  tick early, because arming a timer and waiting for it cannot tell one tick from the next.

## Consequences

- **The compare's budget is 88 cells on this tree and it is not spendable.** ADR-0112's list of
  SoC-side candidates is now empty.
- **`mtip` is graded for earliness for the first time.** Both `test/timer_tb.v` and
  `test/asm/mtimer.S` still fire a spurious interrupt with the wrong `mtimecmp` write order on
  purpose; what was missing was a check on the tick a *correct* sequence fires on.
- **No ratchet moves.** `rtl/` is unchanged, so `FIT_MAX_LC`, `SOC_MIN_MHZ`, `SOC_EXPECT_SPRAM` and
  `SOC_EXPECT_EBR` are untouched, and the riscv-formal legs and co-simulation cannot move — they were
  run anyway, green.
- **A second worked example of a decline that is not about the period.** The candidate holds 12 MHz
  at sixteen of sixteen and is declined on cells, which is the opposite shape to
  [ADR-0117](0117-the-group-is-not-the-edit-and-two-of-six-do-not-ship.md)'s `wen` masks — cells
  bought at the clock's expense. Read both before assuming which instrument will settle a candidate.
