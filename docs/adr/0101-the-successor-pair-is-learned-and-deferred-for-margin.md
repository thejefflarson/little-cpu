# ADR-0101: The successor pair is learned from history, holds 12 MHz, and is deferred for margin

**Status:** Accepted (the mechanism is deferred, not declined) · 2026-08-11 · *Extends
[ADR-0089](0089-the-operand-fetch-guess-ships-and-one-workload-feels-it.md) and
[ADR-0093](0093-the-compressed-successor-is-decoded-and-the-compiled-workload-is-what-moves.md),
which built the operand-fetch guess and then made it decode a compressed successor. Both took the
guess from the fetch window; this takes it from history instead, and covers the case neither could.
Measured on top of [ADR-0099](0099-the-memory-transaction-launches-from-the-execute-slot.md).*

## Context

Decode presents a register-number pair a cycle before it needs the operands and checks what it
presented against what the issuing instruction reads; a miss costs one `operand_stall` and a hit
costs nothing. ADR-0089 made that presented pair a guess at the *next* instruction's pair, read out
of the fetch window's successor word, and ADR-0093 ran that word through the same compressed mapping
the issuing instruction goes through.

What neither can reach is a redirect. The successor word is the instruction **physically** next in
memory, and after a taken branch, a jump, a `jalr`, a trap or an `mret` that is not the instruction
that runs. ADR-0087 measured redirects at 7.15% of the suite's issues and **16.92% of Dhrystone's**.

On ADR-0099's tree the operand column is 295 540 Dhrystone cycles, 18.4%.

## Decision

**Built and measured; deferred rather than shipped.** What was built: `rtl/pairtable.v`, 256 tagged
entries in one block RAM. The entry for an address holds the register pair read by whatever
instruction really ran after it last time, so a branch that goes the same way twice costs no
operand-fetch cycle on the second pass. Three things about the shape were the design, and they are
what a later attempt should start from rather than re-derive:

- **It is read off `next_pc`, on the same edge the instruction memory latches `imem_addr_next`.** So
  the entry arrives with the instruction it describes, and everything downstream of it is a
  comparison of two registers under a mux whose select already dominates that path. Nothing there
  reaches the fetch loop by inspection: `read_rs*` ends at the register file's address port and at
  `operand_stall`'s held copy, both registers.
- **It is tagged.** Six tag bits over an eight-bit halfword index separates 32 KB of text, four
  times the SoC's whole ROM, so no two addresses a program can execute share an entry and a
  collision is a miss rather than another instruction's pair.
- **The entry is stored against the PREVIOUS issue's address**, not this one's. Storing it against
  this one answers "what does this instruction read", which decode already knows.

**Nothing it says can be wrong in a way that matters**, which is why there was no flush, no
invalidation and nothing to do on a store into text. `operand_stall` is the check that already
exists; a miss costs the cycle that was being paid anyway. The only correctness obligation is that
the table produce a *defined* value, so the array is zeroed in an `initial` block: an EBR with no
INIT comes out of the bitstream as zeros, and a simulation whose array started undefined would push
an X through the guess into the register file's address port — green under the two-state runner and
red only under iverilog, which is the exact failure mode CLAUDE.md records about the ROM.

The RTL is on the branch this ADR lands on (PR #164) and is taken back out before it merges, the way
[ADR-0100](0100-the-early-register-write-is-built-and-costs-the-clock.md)'s is. The measurement is
what survives.

## Consequences

### The two workloads disagree, and this is the clearest case of it in the tree

On top of ADR-0099:

| | before | after |
|---|---|---|
| suite | 28 092 cycles, CPI 1.80 | 28 224, CPI 1.80 (**+0.5%**) |
| Dhrystone | 1 607 619 cycles, CPI 1.69, 0.727 DMIPS/MHz | 1 478 440, CPI 1.56, **0.790** (**−8.0%**) |

Dhrystone's operand column goes 295 540 → 166 374, and about 12 000 of those cycles reappear as
scoreboard stalls the guess had been paying for — the same bookkeeping ADR-0089 and ADR-0093
recorded. **The suite gets worse.** Its programs are short, straight-line and hand-written: an entry
is cold or stale more often than it is right, so the mispredicts outnumber the redirects saved. Read
that as the strongest available statement of ADR-0084's warning — the suite is not a proxy here, it
has the opposite sign.

Measured with the early register write also in place (ADR-0100, declined), this mechanism is worth
−120 269 Dhrystone cycles rather than −129 179, so the two are very slightly sub-additive and
neither depends on the other.

### Area and period

`make fit` 3473 → 3550, +77 cells, and one more block RAM: `SOC_EXPECT_EBR` would move 20 → 21,
which is exactly the 256×16 the table asks for.

Six placements a side:

| | ns, sorted | worst | margin over 12 MHz |
|---|---|---|---|
| ADR-0099's tree | 73.34 73.93 76.16 77.17 77.21 78.40 | 78.40 | 5.9% |
| with the table | 79.51 79.73 80.26 80.63 80.68 80.89 | 80.89 | **3.0%** |

**+5.0% of median period, outside the churn band**, so the table is not free after all — the entry's
mux and its tag compare do land somewhere that matters, even though neither is in the fetch loop by
inspection. It holds 12 MHz at every placement of six.

### Why it is deferred, and what would change that

Nothing here is a correctness or a clock failure. Every gate was green on it and the requirement was
met at every placement. It is deferred on **margin arithmetic and on where the win lands**:

- **The win is one workload's.** −8.0% of Dhrystone's cycles against **+0.5% of the suite's**. The
  compiled workload is the one that resembles code anybody would run, so that is not nothing — but
  it is a single program, and a mechanism that makes the other measurement worse has to be worth
  more than this one to spend margin on.
- **It costs most of the margin the next changes need.** 5.9% over the worst of six placements
  becomes 3.0%, which is *inside* the 3.6% edit-churn band — so the next change to land on top of it
  cannot tell its own effect from churn, and 12 MHz is a requirement whose next step down is 6. That
  pair was measured before the rebase; re-swept on the tree that merges the base is 4.8%, so the
  table's +5.0% of median period would leave less again. Two
  queued changes both add logic to the fetch loop. Starting them at 3.0% makes their own go/no-go
  sweeps unattributable, which is a worse outcome than not having the cycles.
- **It is a new module carrying learned state**, +77 cells, for a property the rest of the design is
  deliberately free of. Against *simple* and *readable* that is a real cost, and it is only worth
  paying for a win the other three goals can see.

**What would change the verdict**: the fetch-loop work ahead of it landing with margin to spare, a
second compiled workload agreeing with Dhrystone, or a spelling whose period cost is inside the
churn band. Re-measure the period on the tree of the day before re-taking it — a ceiling is
perishable, and this one is a placement distribution rather than a property.

### Against the four goals

**Fast** — 8.0% of Dhrystone's cycles at a clock that still meets its requirement, and +0.5% of the
suite's. **Simple** — one module, 100 lines, no interaction with anything: it has no failure mode,
only a hit rate. That is the unusual property here and it is worth stating plainly — the mechanism
cannot be wrong, so no part of the rest of the design has to know it exists. It is still a module and
still state. **Readable** — the alternative it replaces is nothing; decode's two guesses sit on
adjacent lines. **Formally verified** — `pair_hit`/`pair_rs*` were free inputs in `formal/pcloop.sv`
and `formal/traps.sv`, which is *wider* than the table can be, so those proofs covered every entry it
could hold and every one it could not; the generated riscv-formal checks built the real table. All 85
generated checks, all five component proofs, `pcloop_cover`, `dmemcheck`, `imemcheck`, `cover`,
`nonperturbation` and `cosim-suite` were green on it.

F and G were re-derived on that tree and reproduce ADR-0099's exactly: `hang` red at 5 and 6, PASS at
7 (F = 6); `liveness` red at gap 4, PASS at gap 5, at trig 10 and again at trig 15 (G = 5). The
table adds no stall reason — the guess is checked by a comparison that already existed.

### What was not claimed

`test/pairtable_tb.v` graded **when** each answer is available and never whether it is right, because
"right" is not a property this has. Four things it graded, each of which would otherwise be a silent
lost cycle: the entry arriving with the instruction rather than one behind it, the pair being the
successor's rather than the address's own, the tag turning a collision into a miss, and an unwritten
entry reading as a value at all. It also recorded a corner rather than leaving it to be rediscovered:
an entry written on the cycle it is read is not seen, because both EBR ports are synchronous and the
read is read-first, so a two-instruction loop misses on its second pass and hits on its third.

The hit rate is not a ratchet and nothing grades it. A change that made every lookup miss would cost
8% of Dhrystone and pass every check in the tree.
