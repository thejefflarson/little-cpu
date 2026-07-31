# ADR-0027: `minstret` counts non-trapping issues; serialization buys exactness, not hazard safety

**Status:** Accepted · 2026-07-31 · *Amends ADR-0005*

## Context

ADR-0005 made two claims about CSR handling that were correct when written and are not correct
after M3 lands traps in decode.

## Amendment 1 — the `minstret` rule

ADR-0005 says `minstret` "increments at issue (equivalent to retire because post-decode nothing
faults)."

The parenthetical was the justification, and **M3 invalidates it: decode itself now faults.** A
trapping instruction issues, and retires in RVFI with `rvfi_trap = 1`, but per the RISC-V spec it
must not increment `minstret` — a trapped instruction did not retire.

**The rule becomes: increment at issue, for non-trapping issues only.**

This is a one-line change to the implementation and a silent correctness bug if missed. It is also
invisible to the riscv-formal ladder as that ladder stands: `rvfi_csrw_check.sv` checks that a CSR
*write* lands, not that a counter's *increment* semantics are right.

**Correction, from ADR-0031.** The sentence that stood here said the `csrc_*` checks that would
catch it were "unreachable at the current pin", because upstream had split `rvfi_csrc_check.sv`
into six files after the pinned SHA. That was wrong in the way that matters: all six models are
present at the pin, and what blocked them was this repo's five-year-stale vendored `genchecks`
copy. Re-vendoring made them generatable. `csrc_inc_minstret` is the direct formal statement of the
rule above and is the check to reach for once `minstret` exists — it needs a `[csrs]` test spelling,
a derived depth (ADR-0025) and a `formal/EXPECTED_FAIL` entry (ADR-0022), and nothing more. Until
then this rule is checked only by `test/asm/minstret.S`, which asserts on values the test's own
handler recorded rather than against any oracle.

## Amendment 2 — what serialization actually buys

ADR-0005 justifies serializing CSR instructions and `mret` on the grounds that it "eliminates every
CSR/pipeline interaction corner."

On inspection there are no such corners to eliminate. CSRs are read and written entirely in decode;
nothing downstream touches them; and the CSR read result reaches `rd` through the existing
`is_add` pass-through, where the ordinary ADR-0004 scoreboard already covers RAW. Remove the
serialization and no hazard appears.

**What serialization actually buys is `minstret` exactness.** Without it, `csrr minstret` reads a
count inflated by the two or three instructions in flight behind it, and the value a program reads
depends on pipeline occupancy rather than on architectural state.

That is still worth the ≤3 cycles, on a core that optimises for readability over throughput, and it
gives the sprint its sharpest test — a `csrr`/`nop`×3/`csrr`/`sub` sequence whose difference is
exactly 4 only because the stall exists.

**Keep the serialization; fix the stated reason.** The reason matters because a future reader
looking to reclaim CPI will read "eliminates hazard corners," verify there are no hazard corners,
and delete a stall that was load-bearing for something else entirely.

## Consequences

- ADR-0005's CSR section is amended in both particulars; the design it describes is unchanged.
- The `minstret` rule needs a test that would fail if it regressed, and the ladder cannot provide
  one — so it belongs in a component proof and in a `.S` test that traps deliberately and checks
  the counter did not advance.
