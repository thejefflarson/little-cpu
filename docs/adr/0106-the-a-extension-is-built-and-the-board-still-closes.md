# ADR-0106: The A extension is built, and the board still closes

**Status:** Accepted · 2026-08-12 · *Builds the design brief
[`docs/ideas/the-a-extension-lands-single-hart.md`](../ideas/the-a-extension-lands-single-hart.md)
in RTL. Adds the sixth stall reason and amends design commitment 8's reason list with it (the
commitment itself is untouched). Re-measures F and G, which
[ADR-0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md) requires of any
change that adds a stall reason, and moves G. Extends
[ADR-0104](0104-the-fetch-bus-refuses-and-the-data-bus-cannot-afford-to.md)'s decision 2 rather than
reopening it: the region question the A extension had to ask for itself is answered in the accessor,
where it costs no fetch-loop depth. Confirms
[ADR-0102](0102-sails-reservation-survives-a-trap-and-the-misa-bit-is-the-divergence.md)'s
reservation policy in hardware. Raises `FIT_MAX_LC`.*

## What was built

Opcode `0101111` decoded as an illegal instruction. All eleven `.w` encodings are real now: the
nine AMOs, `lr.w` and `sc.w`. `.aq` and `.rl` are decoded and ignored, for the argument the brief
records and this ADR does not repeat — one hart, in order, one outstanding access, every access
completing before the next issues.

Three pieces, in the three places the pipeline already put them.

**Decode.** Eleven flags on `decoder_output`, one per function rather than the encoding's `funct5`,
because `rtl/accessor.v`'s result mux is a `(* parallel_case *)` over them and a marking is spent
against a `$onehot0` check of the exact arm list. The immediate mux gains an explicit arm forcing
zero: an A encoding puts `funct5`, `aq`, `rl` and rs2 exactly where the I-immediate is read, so the
effective address is rs1 and nothing else, and `test/decoder_tb.v` proves it against the 0x06b those
bits would otherwise arrive as. `uses_rs1` is true for all eleven; `uses_rs2` is true for the AMOs
and `sc.w` and **false for `lr.w`**, whose rs2 field is an encoding constant — a non-zero one is a
different encoding and decodes as illegal. Misalignment is detected in decode with everything else:
cause 4 for `lr.w`, cause 6 for the ten that write.

**The accessor.** One 33-bit adder/subtractor serves the add and all four min/max compares — borrow
is unsigned less-than, a sign-disagreement correction gives signed, the identity `rtl/decoder.v`'s
branch tests and `rtl/executor.v`'s `slt` already prove against reference operators, and the
accessor now proves it there too. The subtracting operand is inverted across all 33 bits and not
just the low 32; narrower, the borrow comes out of the wrong end and every min/max answer inverts.
Swap, and, or and xor fall out of the result mux. A 30-bit word-address reservation sits beside it.

**What did not transfer, and the brief said so first.** `rtl/executor.v`'s merged subtractor works on
rs1 and rs2 at issue. An AMO's operands are the memory word and rs2, and the word does not exist
until two cycles later. Reusing it would be a cross-stage operand loop, which is the forwarding
network ADR-0083 priced and declined. The precedent transferred; the hardware did not.

## Decision 1 — the sixth stall reason bubbles, and that is the opposite of the divider's ruling

An AMO owns the data bus for two cycles: the read goes out while the executor takes it, and the
read-modify-write goes back out on the cycle after. Nothing else may present a transaction on that
second cycle, so decode spends it. That is a sixth reason in `stall`, declared in the three places
commitment 8 names — the OR identity in `test/decoder_tb.v`, the bucket in `make cycles`, and the
enumeration in `CLAUDE.md`.

**Whether it holds or bubbles was a live question and the answer is bubble.** The divider holds
because its cycle is raised while `decoder_out` carries an instruction nothing has consumed; zeroing
it there loses an instruction. The atomic wait is raised *after* the executor has taken the AMO. A
hold would re-present it, and `requesting` would fire again: a second read on the bus beside the
write it was waiting for, and one instruction retiring twice. So the two rulings are opposite for
the same reason, which is whether anything has consumed the instruction yet.

That is only arm order in the publish block, and nothing else would say it was wrong. It is
asserted in `rtl/decoder.v`'s `FORMAL` block — the cycle after the wait carries a bubble — and
vectored in `test/decoder_tb.v` both ways, including the case that separates the two rulings: a
*held* AMO, under a divide, must raise no wait at all, because no read has gone out and no write
cycle is next. **The other half is deliberately not asserted.** "The wait is raised exactly when a
taken AMO sits in `decoder_out`" and "the wait is a term of `stall`" each restate an assign a few
lines above them, so neither can fail for anything the RTL could do. `test/decoder_tb.v`'s
OR-identity check, which reads the signals on both clock edges, is what carries them.

**The wait is computed in decode from `out`, not routed back from the accessor.** `out` is what the
accessor is looking at, one register earlier and one module nearer the pc, and routing it through
the accessor and back would add a module hop carrying no information. The accessor assumes the
property instead and names its discharge: `rtl/decoder.v`'s assertion above.

**One read and one write per AMO, however many cycles decode is held.** The transaction count in
`test/accessor_tb.v` is what says so, taken across a whole AMO and again with the AMO held under a
five-cycle divide: two, both times. Two writes are one write for RAM and are not one for a device,
which is why this is a correctness statement rather than a tidiness one — the same reason ADR-0099
gave for gating the request block on `launch_taken`.

**The scoreboard needed nothing.** An AMO's `rd` and a store-conditional's `rd` are visible in
`decoder_out`, then in `executor_out`, then through the register file's write-through bypass, with
no gap — the same three cycles a load's `rd` occupies, because the wait costs decode a cycle and not
the accessor one. `sc.w` is a store that writes a register, which is a new category, and what makes
it work is that `rtl/decoder.v`'s `rd` mux never listed it with the other stores. So no slot was
added and F did not have to move for that reason. `test/decoder_tb.v` carries SC- and AMO-shaped
vectors that interlock the instruction behind them, because a gap here is invisible to everything
else.

## Decision 2 — G moves 5 → 6, and the thinnest depths clear their floor by one again

Re-measured by the procedure `formal/checks.cfg`'s own header prescribes — a copy of the file with
one line in `[depth]`, regenerated, and the last column swept — not inferred from the change:

| figure | sweep | result |
|---|---|---|
| F | `hang` check cycle 4, 5, 6, 7, 8 | FAIL, FAIL, FAIL, **PASS**, PASS → F = **6**, unchanged |
| G | `liveness` at trig 10, depth 12…16 | FAIL to 15, **PASS at 16** → gap **6** |
| G | `liveness` at trig 15, depth 18…21 | FAIL to 20, **PASS at 21** → gap **6**, the same |

So `F + G` is 12 and `F + 2G` is 18. `insn 19` and `fault 19` clear 18 by one where they cleared 16
by three, and `reg 15 22`'s window of 7 clears G = 6 by one. **No depth moves.** A depth at or above
its derived floor is asking the whole question, and raising one spends the `formal` job's
twenty-minute budget for nothing measured — the brief named that job as the most likely CI casualty
and it is not one. What did change is the margin, from three to one, and that is recorded where it
is enforced rather than discovered later.

G had been 6 before ADR-0099 gave a cycle back by launching the bus transaction from the execute
slot. It is 6 again for a different cycle. The two are not the same cycle and the coincidence is
worth naming so nobody reads this as that change being undone.

## Decision 3 — the region attribute is answered in the accessor, because ADR-0104 measured the other place

The brief and the ticket both wanted an AMO or a store-conditional outside RAM to fault with cause 7
and an `lr.w` there with cause 5. **That is the decode-time region decode ADR-0104 built, measured
and declined**: it must read the top of `immediate + reg_rs1` and hand the answer to `next_pc`, four
extra logic levels in the fetch loop, 10.57–11.00 MHz over four seeds against a 12.00 MHz
requirement. Nothing about the A extension makes that cheaper. So causes 5 and 7 are still not
implemented, and the deviation ADR-0104 recorded still stands, unchanged in extent.

What the A extension cannot leave alone is narrower than that, and it is the one outcome the
specification has no room for: **a store-conditional must not report success for a store that went
nowhere.** Every other out-of-region access already reads zero and writes nowhere silently, which is
the recorded deviation; an `sc.w` reporting 0 is a positive claim that the write landed.

So the reservation refuses to be taken outside the region the data memory answers. `rtl/memory.v`
exports the range test it already computes as `reservable`, `rtl/littlecpu.v` takes it as
`mem_reservable`, and `rtl/accessor.v` sets no reservation on an `lr.w` where it is low. **This is
the fetch bus's shape exactly** — the platform decodes its own regions and hands the core one bit,
the way `rtl/imemory.v` hands it `imem_fault` — and it costs no fetch-loop depth, because the
accessor is not in the fetch loop and the bit accompanies the request rather than the response.

The consequences, stated rather than left to be found:

- **A store-conditional outside RAM always fails**, writes 1 to `rd` and issues no transaction. A
  spurious failure is architecturally permitted anywhere, and the spec's eventual-success guarantee
  attaches only to a region whose reservability PMA declares it, which is the one region here that
  does.
- **The machine timer's four words at `0x0002_0000` get `RsrvNone` for free** and do not get
  `AMONone`. An AMO there executes: it reads the device word, computes and writes it back, in two
  transactions rather than one. That is not what the timer's PMA should say and it is not a fault
  either. It is the same deviation as a plain load or store to an unmapped address, one shape wider,
  and it is bounded by ADR-0104's four-level measurement rather than by an argument.
- **The next attempt starts from a fact this change found**: an atomic's effective address is `rs1`
  verbatim, with no immediate and therefore no 32-bit carry chain in front of the range test. The
  four levels ADR-0104 measured were charged to the top of a sum. A region test for the eleven alone
  reads a register output directly, so that measurement does not transfer to it and the question is
  open rather than closed. It was not attempted here: this ticket's go/no-go was a timing sweep with
  2.8% of margin in it, and adding an untried term to the fetch loop is not what to spend that on.

## Decision 4 — no spec model at the pin, so A gets SYSTEM and MISC-MEM's treatment

Checked in the clone rather than assumed: `insns/generate.py` carries an `insn_amo` generator with
every call site commented out, it covers neither the min/max family nor LR/SC at all, and there is
no `isa_rv32ia*.txt`. `formal/check-complete-exclusions.py` re-derives that from the clone and
accepted all eleven mnemonics, which is the direction that means something — it refuses a mnemonic
that *has* a model.

`formal/COMPLETE_EXCLUSIONS` gains one line and `formal/complete.sv` the matching encoding-keyed
predicate. `formal/checks.cfg` stays `isa rv32imc`, because there is nothing else to generate, so
the generated check set is 86 and unchanged.

**The exclusion carries an anti-vacuity control of its own**, and it is the one cover goal in that
file that deliberately does not read `complete_live`: an excluded class is excluded from the
assertion, so a goal carrying `!insn_excluded` would be unreachable by construction and would say
nothing. It asks instead that an AMO really retires without trapping, so the exclusion is a live
restriction rather than a line about instructions nobody reaches.

What checks the eleven instead:

| leg | what it says |
|---|---|
| `components_accessor` | the nine functions against the operators they replaced, the merged adder/subtractor's borrow against `<` signed and unsigned, one-hot over the result mux, a failed `sc.w` leaving the bus alone, and no reservation held where the platform said none may be |
| `components_decoder` | the eleven in the `$onehot` over `instr_valid`, the atomic wait's arm and its timing |
| `dmemcheck` | the two-cycle memory report against the pinned `rvfi_dmem_check` |
| `test/accessor_tb.v` | the datapath at operands where a wrong compare shows, the reservation's five events, and the transaction count |
| `test/decoder_tb.v` | the eleven encodings, the reserved rows either side of each field, all four ordering suffixes, the three misalignment causes, and the scoreboard's view |

**RVFI for an AMO is assembled across two accessor cycles**, and the write-cycle capture had no
precedent in the file: the address and the write data are read off the live bus on the write cycle
while the read data comes from the read. Report the registered pair instead and the retire says the
core wrote back the word it read, unchanged. Its red direction was forced rather than argued —
swapping old and new data makes `make -C formal dmemcheck` go SAT at step 7, and restoring it passes
again.

## Decision 5 — `misa` and `-march` do not move here

`rtl/csrs.v` still returns `0x4000_1104` and all three of the suite's build sites still say
`-march=rv32imc_zicsr_zifencei`. So no program in `test/asm` executes an atomic, `make cycles`
charges zero cycles to the new bucket, and `make cosim-suite`'s verdicts are unchanged. The
reference model was already configured for this in ADR-0102: `Zaamo` and `Zalrsc` on, `A` off, so
the model executes the instructions and does not claim the bit the core does not claim. Turning both
on is one later change and it has to be one, or the ISA string and the register disagree.

**Say plainly what that means about this ADR's evidence.** The `.S` suite and co-simulation — two of
the three legs that usually carry a change here — say nothing at all about the eleven instructions.
What is behind them is the component proofs, the two unit benches, and `dmemcheck`. That is thinner
than this repository normally accepts, it is a consequence of splitting the ISA-string change out,
and the ticket that moves `misa` is where the suite and the reference model start covering this.

## What it cost

**`make fit`: 3464 → 3935, +471 cells**, both on one machine and one toolchain with the same tree
either side. The brief estimated 300–380 and it is 23% over that. Where it goes, in descending
order: two 32-bit held registers for the write cycle's address and rs2, the two 32-bit muxes that
give the read-modify-write the bus, the 33-bit adder/subtractor and the nine-way result mux, the
30-bit reservation and its comparator, and eleven flags through the decoder's publish register.
`FIT_MAX_LC` goes 3625 → 4000, which is 3935 plus the same ±50 churn band and the same toolchain
term the old figure was derived with. **That is about a tenth of the core spent on instructions no
current workload executes**, and the brief's answer to why is the one this ADR adopts: every
verification leg this project owns is single-hart, so this is where the eleven can be checked, and
the multi-core milestone is then left verifying exactly one new thing.

**`soc/timing_sweep.sh`, four seeds, `SOC_PROG=datainit.c`, one machine and one toolchain. This is
the go/no-go and it is GO.**

| tree | default | seed 1 | seed 2 | seed 3 | worst | vs. requirement |
|---|---|---|---|---|---|---|
| `origin/main` (`d8ecfa2`) | 12.78 | 12.54 | 12.77 | 12.73 | **12.54** | +4.5% |
| this change, first spelling | 12.47 | 12.57 | 12.34 | 12.50 | **12.34** | **+2.8%** |
| this change, **as shipped** | 13.08 | 12.89 | 12.75 | 12.77 | **12.75** | **+6.3%** |

The baseline row reproduces ADR-0104's to the nanosecond — 78.26 / 79.75 / 78.32 / 78.55 ns — which
is what a re-run over changes that touch no RTL should give and is the reason to re-take it rather
than quote it.

**Both of this change's rows are the same design, and that is the finding worth carrying.** The
second was taken after a cleanup pass that states one fact differently in two places: the result
mux's compare arm reads the `amo_compare` net it already had rather than the four-term OR spelled
out again, and the two misalignment lines drop a conjunction each operand already implies. Two cells
on `fit`, 3933 to 3935, deep inside the ±50 churn band. **The worst placement moves 12.34 → 12.75
MHz, 3.3%**, and the median 80.10 → 77.94 ns, which puts the shipped spelling a hair *faster* than
the base it was measured against. Neither number is evidence about the A extension; both are inside
the bands this repository already measured, which is exactly the spelling-dependence ADR-0097
recorded for cells, seen here on the clock. The reason to print both is that quoting only the second
would read as A being free, and quoting only the first would read as it costing 1.7% of period. It
is neither: the requirement is met at every seed of both, and the period cost of A is a null in both
directions.

**The pre-approved fallback was not needed.** Registering the AMO ALU and taking a fourth cycle — a
latency no measured workload pays, since nothing in the suite or Dhrystone executes an atomic —
stays designed and unbuilt. `SOC_MIN_MHZ` did not move and `SOC_EXPECT_SPRAM` and `SOC_EXPECT_EBR`
are unchanged at 2 and 20 — `make soc-timing` checks both exactly on every one of the twelve
placements above.

## What ran

| gate | result |
|---|---|
| `make test` | **64/64**, `EXPECTED_FAIL` exact both ways |
| `make test-units` | 9 benches, all PASS |
| `make probe-gates` | all probes green (it runs inside `make test`) |
| `make mutation-check` | 4 mutations, each caught by exactly its declared detectors |
| `make cycles` | columns add up, `unattributed` 0, the new `atomic` column 0 of 28495 |
| `make waves` | PASS, and the same 16 allowlisted `sorry:` notes as the base — none added |
| `make lint` | clean in both passes |
| `make -C formal check` | **86 checks, 86 pass**; `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact both ways |
| `make -C formal components_{decoder,executor,accessor,pcloop,traps}` | all PASS (`pcloop_cover` PASS first) |
| `make -C formal complete` / `complete_cover` | PASS / PASS |
| `make -C formal complete-exclusions` / `interrupt-tie-off` | PASS / PASS |
| `make -C formal dmemcheck` / `imemcheck` / `nonperturbation` | PASS / PASS / PASS |
| `make fit` | 3935 of 4000 budgeted |
| `soc/timing_sweep.sh`, four seeds, both trees | the table above |

`formal/pcloop.sv`'s `f_may_stall` gained the atomic wait, and it had to: that list is the harness's
own enumeration of every reason decode may hold the pc, and a missing one makes the increment
assertion cover a cycle where holding is legal and fail there. It failed exactly that way before the
term was added — `pcloop.sv:270` at step 5 — which is the file's own comment working as written.

Six red directions were forced rather than argued, three in each bench:

- Hold `decoder_out` on the atomic wait instead of bubbling: five `test/decoder_tb.v` vectors go
  red, including the AMO failing to reach `decoder_out` at all.
- Widen `uses_rs2` to `lr.w`; delete the A arm of the immediate mux (the effective address becomes
  `rs1 + 0x6b`); report a misaligned `lr.w` as a store misalignment. One, four and two vectors red.
- Invert the min/max keep term (8 red); read all four compares unsigned (amomin/amomax red at
  operands straddling zero); let a failed `sc.w` store anyway; ignore `mem_reservable`; never raise
  `take_amo`, so the write half never goes out.

## Consequences

- **The core executes eleven instructions its `misa` does not claim.** That is deliberate and
  temporary, and CLAUDE.md's ISA-target section says so, because the alternative is a reader finding
  the flags in `rtl/decoder.v` and concluding the register is wrong.
- **A sixth stall reason exists again**, and it is the first one whose hold-or-bubble arm was a real
  question rather than an inherited one. The pattern to carry forward: a wait raised before the
  executor consumes the instruction holds, one raised after it bubbles.
- **G is 6 and the thinnest depths clear their floor by one.** The next change that adds a stall
  reason will have to raise a depth or find the cycle back somewhere, and it will find that out from
  `formal/checks.cfg`'s arithmetic rather than from a red check.
- **`FIT_MAX_LC` is 4000 against 3935** — 67 cells of headroom where there had been 161. The up5k is
  at 74% for the core alone.
- **The multi-core milestone now verifies one new thing.** The clear-on-matching-write rule is
  already the shape that extends to snooping a second hart's writes, and the `.aq`/`.rl` argument is
  recorded with the four things that invalidate it named: a second agent on memory, a store buffer,
  a cache, or a posted MMIO write.
