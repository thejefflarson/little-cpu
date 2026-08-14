# ADR-0108: The A extension is claimed, and one bit is the whole claim

**Status:** Accepted · 2026-08-14 · *Closes decision 5 of
[ADR-0106](0106-the-a-extension-is-built-and-the-board-still-closes.md), which shipped the datapath
and deliberately left `misa` behind. Spends the reference-model configuration
[ADR-0102](0102-sails-reservation-survives-a-trap-and-the-misa-bit-is-the-divergence.md) set up and
predicted the divergence for. Amends
[ADR-0002](0002-isa-target-rv32imc-zicsr.md)'s ISA target from RV32IMC_Zicsr_Zifencei to
RV32IMAC_Zicsr_Zifencei. Carries forward, unchanged in extent, the deviation
[ADR-0104](0104-the-fetch-bus-refuses-and-the-data-bus-cannot-afford-to.md) measured and declined.
`FIT_MAX_LC` and `SOC_MIN_MHZ` both stay where they are.*

## What changed

`rtl/csrs.v`'s `MISA_VALUE` goes `0x4000_1104` → `0x4000_1105`. Bit 0 is A. That is the whole
functional diff: no datapath, no decode, no stall reason, no depth.

Four things move with it because they restate it. `test/asm/csr.S` pinned the old value as a literal
at two cases and now pins the new one; `test/csr_tb.v` does the same at its read and its
ignored-write checks; `test/sail/rv32imc_zicsr.json`'s `A` key flips from `false` to `true`; and
CLAUDE.md's ISA target becomes RV32IMAC_Zicsr_Zifencei.

`-march` did not move and did not need to. `test/march_test.sh` landed it at
`rv32imac_zicsr_zifencei` across all six sites one commit earlier, `DHRY_CFLAGS` and its verbatim
copy in `soc/depth/cycles.py` included, and grades them both ways on every `make test`. **An ISA
string and `misa` are two different claims** — one is what the assembler will accept, the other is
what the hardware says about itself at run time — and this ADR is about the second. The comment in
`march_test.sh` that promised this change has been rewritten to say that rather than to predict it.

## Decision 1 — the model's `A` key flips in this commit, and the transition was demonstrated first

`A` in the Sail configuration is the `misa.A` bit alone. It is **not** an umbrella over `Zaamo` and
`Zalrsc`, which have been `true` since ADR-0102 and are what make the model execute the eleven
instructions. ADR-0102 measured all four combinations and left `A` at `false` on purpose, so that
the commit claiming the bit would be falsifiable: with the model already claiming A, a core that
started claiming it would go green either way and the gate would stop checking at the exact moment
it began to matter.

So the transition was run in both directions rather than the end state confirmed. RTL at
`0x4000_1105`, model key still `false`:

```
DIVERGENCE at architectural change #17
  sail instruction #24  pc=0x0000004e  csrrs x10, misa, x0     test_5+0
  sail : x10=0x40001104
  core : x10=0x40001105   (cycle 62, decode pc=0x00000056)
COSIM-STATUS DISAGREE AT 17
```

Key flipped, nothing else touched:

```
AGREE csr.S: 70 architectural register-file changes, identical in order and value
             (0 not comparable by value); both ran to PASS.
```

`csr.S` gains a third case for the form that would actually clear the bit if anything could.
`csrrs a0, misa, x0` suppresses its write, so the two existing cases prove only that a read-only CSR
can be read; `csrw misa, x0` is a CSRRW, which writes unconditionally even with rs1 = `x0`. `misa`
is `0x301`, outside the read-only encoding range, so the instruction retires rather than trapping
and the WARL fallback swallows it — which the privileged spec permits for a read-only `misa`, and
which the model agrees with because its configuration says `writable_misa: false`.

## Decision 2 — the claim is paired with a mutation, because nothing else in the design can see it

A wrong `misa` has no other consequence. The eleven instructions still decode, still retire, still
agree with the reference model; every proof, every other program and every other bench is blind to
the bit by construction. So the graders that hold the claim were measured rather than asserted:
`test/mutations/misa-drops-the-a-bit.patch` gives bit 0 back, and what goes red is exactly

| leg | what it reports |
|---|---|
| `test/asm/csr.S` | `FAIL 5`, the first of its two misa reads |
| `test/csr_tb.v` | two mismatches — the read *and* the write it is supposed to swallow |

and nothing else: the other 69 programs and the other eight benches stay green under it. That is
the entry in `test/MUTATION_DETECTORS`, graded under set equality in both directions like the six
already there, and `make mutation-check` now runs seven.

The finding worth stating plainly is the one that made the pairing worth adding: **one asm case and
two bench reads are the entire distance between a wrong `misa` and a fully green tree.** There is no
formal check, no co-simulation property and no other program behind it. That is not a defect — a
self-description register has nothing else to be checked against — but it is a thin place, and it is
now a measured thin place rather than an assumed one.

## Decision 3 — what the platform's PMAs actually are, in the specification's vocabulary

The claim `misa.A` makes is that the eleven instructions are implemented. It says nothing about
where they work, and this platform's answer is uneven enough to be worth writing down once, in the
terms the unprivileged spec's memory-attribute chapter uses. Four regions, from `rtl/littlesoc.v`
and `test/testbench.v`:

| region | what answers it | atomicity | reservability |
|---|---|---|---|
| `0x0001_0000` + 64 KB, data RAM | `rtl/memory.v` | `AMOArithmetic` — all nine functions | `RsrvEventual` |
| the text window at `0x0000_0000` | `rtl/imemory.v`'s data port | `AMOArithmetic` | **`RsrvNone`** |
| `0x0002_0000`, four words, machine timer | `rtl/timer.v` | **not `AMONone`, and it should be** | `RsrvNone` |
| everything else | nothing | reads zero, writes nowhere | `RsrvNone` |

`rtl/memory.v` exports the range test it already computes as `reservable`; `rtl/littlecpu.v` takes
it as `mem_reservable`; `rtl/accessor.v` sets no reservation on an `lr.w` where it is low. The data
RAM is the only region that raises it, which is what puts `RsrvNone` on the other three for free.
That is the platform decoding its own regions and handing the core one bit, the same shape
`imem_fault` already had, and it costs no fetch-loop depth.

**The row that is wrong on purpose is the timer's.** An AMO to a device word executes: it reads,
computes, and writes back, in two bus transactions rather than one. A device's atomicity PMA should
say `AMONone` and this platform cannot say it, for the reason decision 4 gives.

## Decision 4 — A is claimed while causes 5 and 7 are unimplemented, and that is the honest caveat

**An atomic outside RAM does not fault.** Load access fault (5) and store/AMO access fault (7) are
not implemented, so an `lr.w` to an unmapped address reads zero, an `sc.w` there fails without
faulting, and an AMO to the timer window executes as described above. None of that is a choice made
to make this ticket easier. It is ADR-0104's measurement: the decode-time region test that would
raise those causes has to read the top of `immediate + reg_rs1` and hand the answer to `next_pc`,
which is four more logic levels in the fetch loop and **10.57–11.00 MHz over four seeds against a
12.00 MHz requirement**. ADR-0106 re-asked the question for the eleven and got the same answer.

What the A extension could not leave alone is narrower, and it is the one outcome the specification
has no room for: a store-conditional must not report success for a store that went nowhere. Hence
the reservation refusal above. **So this core does a third thing that the specification describes
and the reference model cannot be configured into**: it refuses the reservation and raises nothing.
Measured on sail-riscv 0.13.1 with the region split at the data RAM's bounds, the model has exactly
two settings and neither is this machine —

| `reservability` | what the model does at `0x0004_0000` |
|---|---|
| `RsrvEventual` | the `lr.w` reserves and the `sc.w` **succeeds** |
| `RsrvNone` | the `lr.w` raises a **load access fault, cause 5** |

— so configuring `RsrvNone` would trade a register-value divergence for a control-flow one, on a
fault this core is recorded as not raising. The region split is therefore deliberately absent from
`test/sail/rv32imc_zicsr.json`, and `test/asm/amoregion.S` carries `DISAGREE AT 18` in
`test/COSIM_EXPECTED_FAIL` with that reasoning written out at the entry.

**A reader should be able to find this in the record that makes the claim**, not two ADRs away,
which is why it is restated here rather than cited. `misa.A` is still correct: the bit says the
instructions are implemented, and they are. It has never been a statement about the memory map.

## Decision 5 — the reservation policy, and the `.aq`/`.rl` argument, recorded with what would break them

**The reservation is cleared by two events and survives two others.** `rtl/accessor.v` clears it on
any `sc.w`, whether or not that `sc.w` stored, and on a store of this hart's own — `sb`, `sh`, `sw`
or an AMO — to the reserved word. It is **not** cleared on trap entry and **not** cleared on `mret`.
That is what lets a constrained LR/SC sequence keep the eventual-success guarantee with the machine
timer running: a handler that never touches the lock word cannot make the store-conditional fail.
ADR-0102 measured the reference model under Sail alone, with no core in the loop, and found it
clears on neither event either, so there is nothing to mirror and no baseline entry. The reservation
set is the aligned word the instruction names, which is the spec's minimum; a wider one only fails
more store-conditionals.

**`.aq` and `.rl` are decoded and ignored, and the argument is a property of this configuration
rather than of the design.** One hart, in order, no store buffer, no cache, one outstanding access,
every access completing before the next issues: memory order is program order, which is sequentially
consistent and strictly stronger than anything those bits request. Four things invalidate it, named
so the next person does not have to re-derive them:

- **a second agent on memory** — a second core, or DMA;
- **a store buffer**;
- **a cache**;
- **posted MMIO writes**, where the write is acknowledged before it lands.

And one requirement follows for the work that would introduce the first of those: **a two-core
arbiter must keep per-access completion in program order**, or this reopens as a design question
rather than as a comment. This is the `fence.i` lesson restated — "correct and for free" was a
property of the configuration, and it stopped being true the moment text became writable.

## What it cost

**Nothing on the datapath, and the two instruments say so in different ways.**

`make fit`, all four figures local, same machine and same Homebrew yosys, and the count is
deterministic because `fit` has no placement in it:

| `MISA_VALUE` | `fit` | vs. base |
|---|---|---|
| `0x4000_1104` (base) | 3935 | — |
| **`0x4000_1105` (this change)** | **3988** | **+53** |
| `0x4000_1106`, one bit elsewhere | 3979 | +44 |
| `0x4000_1144`, one bit elsewhere again | 3920 | **−15** |

**The two probe rows are the measurement.** Three edits that each add exactly one set bit to one
read-only constant span 68 cells, from −15 to +53. Nothing in that spread is area the A bit costs;
it is ABC re-mapping the cone around a constant, which is the ±50 churn band CLAUDE.md already
records, exercised here about as cheaply as it can be. Reporting +53 as the price of claiming A
would have been reading noise as a bill. `FIT_MAX_LC` stays 4000 — the `fit` job reads about 32
cells below a local Homebrew yosys on identical RTL, so the job's number is expected near 3956, and
the ratchet is what grades it. **12 cells of local headroom is thinner than it sounds and thinner
than it looks**: the OSS CAD Suite is not pinned, CI installs the latest release, so the offset
between the two toolchains is itself free to move. The right response if the job trips is the same
as ever — find the cells, or find the suite bump — and not a ratchet edit, because these four rows
say there are no cells here to find.

**`soc/timing_sweep.sh`, eight seeds a side, `SOC_PROG=datainit.c`, both taken on this tree and this
machine rather than inherited.**

| tree | periods, sorted (ns) | median | worst | vs. 12.00 MHz |
|---|---|---|---|---|
| base (`64759da`) | 76.48 · 77.55 · 78.18 · 78.30 · 78.42 · 78.56 · 78.80 · 79.86 | 78.36 | 12.52 MHz | +4.3% |
| **this change** | 76.10 · 76.93 · 77.99 · 79.34 · 80.60 · 81.00 · 82.03 · 82.49 | 79.97 | **12.12 MHz** | **+1.0%** |

The median moves **+2.1%**, inside the ~3.6% edit-churn band, which is a null in both directions.
**Sixteen placements of sixteen meet the requirement**, and `SOC_MIN_MHZ` does not move.

Two things about that table are worth more than the headline. **The first four seeds alone said
+4.35%**, and the second four brought it back inside the band — which is `soc/timing_sweep.sh`'s own
instruction ("compare distributions, not single runs") producing a different verdict on eight
samples than on four, in the direction that matters. It is also why the baseline was re-taken here
rather than quoted: the base's four-seed row reproduced ADR-0106's shipped row to within a hair
(12.75 MHz worst, 77.9 ns median), which is what says the machine and toolchain are the same ones
and makes the comparison a comparison.

**The second is the tail.** The changed tree's spread is 8.4% against the base's 4.4%, and its worst
placement clears the requirement by 1.0% where the base clears by 4.3%. That is a wider tail on a
design whose only edit is a constant, so it is the same re-mapping the `fit` probe rows show, seen
on the clock instead of on cells — the spelling-dependence ADR-0097 recorded for area and ADR-0106
already observed once on the period. **It is not a regression to fix, because 12 MHz is a
requirement and not a regression floor and every placement meets it**; it is a margin to know about
before the next change goes looking for a percent.

**Dhrystone is unchanged and re-quoted whole**, because nothing executes an atomic in it and nothing
should have moved: **0.727 DMIPS/MHz, 8.72 DMIPS at the board's 12 MHz, 3568 of the SoC's 8192 ROM
bytes.** CPI 1.69, and the `atomic` column of `make cycles` is 0 of 1607636. The flags travel with
the number: `riscv64-elf-gcc 16.2.0`, `-march=rv32imac_zicsr_zifencei -mabi=ilp32 -O2 -std=c11
-ffreestanding -fno-tree-loop-distribute-patterns`, this port's own byte loops rather than a libc's,
2000 runs. It is not a gate and adds no ratchet.

## What ran

| gate | result |
|---|---|
| `make test` | **70/70**, `EXPECTED_FAIL` and the suite shape exact both ways |
| `make test-units` | 9 benches, all PASS |
| `make probe-gates` | 251 graded comparisons, every failure path executed |
| `make mutation-check` | **7 mutations**, each caught by exactly its declared detectors |
| `make cosim-suite` | **66/70 agree**, `COSIM_EXPECTED_FAIL` exact both ways |
| `make -C formal check` | **86 checks, 86 pass**; `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact both ways |
| `make lint` | svlint clean in both passes |
| `make waves` | PASS, `RETIRES 87 SPEC-CHECKED 87` |
| `make dhrystone` | the figures above |
| `make fit` | 3988 of 4000 budgeted |
| `soc/timing_sweep.sh` | the table above, eight seeds a side |

`make -C formal check` was re-run rather than reasoned about even though the change cannot reach it:
`misa` is not in any generated check's cone, F and G are untouched because no stall reason moved,
and no depth was re-derived. A green run is the evidence that all three of those are true.

## Consequences

- **The core no longer executes instructions its `misa` does not claim.** ADR-0106's first
  consequence is discharged, and the temporary state it described lasted two merges.
- **The claim rests on one asm case, two bench reads and one mutation.** Anything that changes how
  `rtl/csrs.v` answers `0x301` has to go through them, and they are graded both ways.
- **The reference model and the register move together from here.** `A` and `MISA_VALUE` are one
  fact stated in two files; `test/asm/csr.S` is what fails if they part, and it fails on the
  co-simulation leg rather than in band, which is the leg that reads the model.
- **The A extension's PMAs are recorded in one table**, and the row that is wrong is marked wrong.
  The next attempt at causes 5 and 7 starts from ADR-0106's observation that an atomic's effective
  address is `rs1` verbatim, with no 32-bit carry chain in front of a region test — so ADR-0104's
  four-level measurement does not transfer to it, and that question is open rather than closed.
- **The margin over 12 MHz is +1.0% at the worst of eight placements**, down from +4.3%, with the
  median a null. The next change in the fetch loop should re-take a sweep before believing a
  headroom figure, which is the perishability rule this repository already applies to ceilings.
