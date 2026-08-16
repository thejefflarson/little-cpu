# ADR-0109: An atomic outside the RAM faults, and the declined measurement did not transfer

**Status:** Accepted · 2026-08-16 · *Closes the half of
[ADR-0104](0104-the-fetch-bus-refuses-and-the-data-bus-cannot-afford-to.md)'s decision 2 that
[ADR-0106](0106-the-a-extension-is-built-and-the-board-still-closes.md)'s decision 3 named as open,
and takes the measurement ADR-0106 asked for. Causes 5 and 7 exist now, for the eleven A encodings
only. **Closes [ADR-0108](0108-the-a-extension-is-claimed-and-one-bit-is-the-whole-claim.md)'s
decision 4** — the caveat `misa.A` was claimed under — and **corrects the one row its decision 3
marked wrong**: the machine timer is `AMONone` now. Retires `test/asm/amoregion.S`'s
`test/COSIM_EXPECTED_FAIL` entry and puts back the Sail region split ADR-0108 recorded as
deliberately absent. Adds a `test/MUTATION_DETECTORS` entry and removes a detector from another. No
commitment is amended.*

## Context — one deviation, two prices, and only one of them was measured

ADR-0104 built the decode-time region test for loads and stores, measured it at **four extra logic
levels in the fetch loop and 10.57–11.00 MHz over four seeds** against a 12.00 MHz requirement, and
declined it. So an access the memory map does not cover was answered silently: a load read zero, a
store was dropped, and no cause 5 or 7 existed anywhere.

ADR-0106 built the A extension on top of that and closed the one outcome the specification has no
room for — an `sc.w` reporting success for a write that went nowhere — by refusing the reservation
outside the region `rtl/memory.v` answers. It left two gaps open by name, and named the reason the
second was worth reopening:

> An AMO there executes: it reads the device word, computes and writes it back, in two transactions
> rather than one. That is not what the timer's PMA should say and it is not a fault either.

> **The next attempt starts from a fact this change found**: an atomic's effective address is `rs1`
> verbatim, with no immediate and therefore no 32-bit carry chain in front of the range test. The
> four levels ADR-0104 measured were charged to the top of a sum. […] so that measurement does not
> transfer to it and the question is open rather than closed.

This ADR answers that. The prediction was right about depth and wrong about being free.

ADR-0108 then claimed `misa.A` on top of both, and wrote the caveat down rather than leaving it to be
found: "A is claimed while causes 5 and 7 are unimplemented". Its decision 3 tabulated this
platform's PMAs and marked one row **wrong on purpose** — the machine timer's `AMOArithmetic`, where
`AMONone` is what a device's atomicity attribute should say. **This change makes that row right**,
and the table it belongs to is ADR-0108's rather than restated here: what moves is the timer's
atomicity to `AMONone` and the text window's to `AMONone`, which is what a region test that asks
"is this the RAM" rather than "is this mapped" gives.

## Decision 1 — the platform answers a second address, and decode faults on it

`rtl/memory.v` computes one range test. It exported it once already, as `reservable`, about the
address on `mem_addr`. It exports it again about a **second** address — `atomic_addr`, the one an
atomic's rs1 names — because decode has to know before it chooses the next pc and cannot wait for
the request to go out. Both answers come out of one window, so the reservation the accessor refuses
and the fault decode raises cannot come to describe different memories.

`rtl/littlecpu.v` publishes `atomic_addr` and takes `atomic_supported`. **This is the fetch bus's
shape on the data side**: the platform decodes its own map and hands the core one bit, and the bit
accompanies the *address* rather than a *response*, which is the distinction ADR-0104's decision 3
said the next peripheral would live or die on. A platform whose memory answers atomics everywhere
ties it high and the core takes neither cause; `formal/{cover,complete,dmemcheck,imemcheck}.sv` are
exactly such platforms and do.

`rtl/decoder.v` raises **cause 5** for an `lr.w` and **cause 7** for the nine AMOs and `sc.w`,
committed on the same `next_pc` override the jumps use. The existing trap suppression does the
rest: every one of the eleven flags is already cleared on a trapping issue, so a refused atomic
issues no transaction, takes no reservation and writes no register, and `components_traps` asserts
all three.

**The address the platform is asked about is `reg_rs1` and deliberately not `mem_addr_calc`**, whose
value is identical for these encodings. The whole point of the term is that it is a register output
rather than an adder's result, so writing it the other way would have been a null that looked like
the same change. `rtl/decoder.v`'s `FORMAL` block asserts `mem_addr_calc == atomic_addr` for every
atomic, which is what stops a later edit to the immediate mux leaving the fault talking about one
address and the transaction about another.

**Alignment outranks the region.** An atomic that is both unaligned and out of region reports the
misalignment, which is what the reference model does and what keeps the eight synchronous causes
disjoint — the decoder's `$onehot0` over them is stated over eight now, and the trap-cause case
still carries no `(* parallel_case *)`.

### Which regions this makes the timer, and text

The bit is `rtl/memory.v`'s window and nothing else, so **the machine timer's four words and the
whole text window are `AMONone` and `RsrvNone`** — an atomic at either faults. That is the answer
ADR-0106 said the timer's PMA should give, and text gets the same answer for the same reason: the
data bus reaches it, it is not main memory that supports atomics, and "is this the RAM" is a
narrower question than "is this mapped". `test/asm/amoregion.S` reaches all three kinds of address.

## Decision 2 — the go/no-go, taken twice, on two bases and two toolchains

**Read the two tables below as two experiments, not one.** The first decided the *spelling* and was
taken on the base this work branched from with a Homebrew toolchain. The second is the *go/no-go*
against the tree that merges, re-taken after ADR-0108 landed underneath — and on the pinned OSS CAD
Suite, because the local `nextpnr-ice40` stopped loading mid-ticket after a `boost` 1.92 upgrade
(`dyld: Symbol not found: boost::program_options::arg`). Both sides of each table share a base and a
toolchain, so each comparison is internal; **the two tables' numbers do not compare to each other**
and are not merged here.

### The spelling, on `64759da` with Homebrew yosys/nextpnr

Two texts of one design, eight seeds each, `SOC_PROG=datainit.c`. They differ in whether the term
that reaches `next_pc` is **one** or **two**:

```
  two terms          one term
  load_access_fault  = instr_lr           && !atomic_supported && !word_misaligned
  store_access_fault = instr_atomic_write && !atomic_supported && !word_misaligned
                       atomic_fault       = instr_atomic && !atomic_supported && !word_misaligned
```

The one-term version answers *whether* it faults in the fetch loop and *which cause* off it —
`trap_cause` and the RVFI fault masks are the only readers of the split, and neither is on the pc's
path.

| tree | worst of 8 | median period | vs. requirement |
|---|---|---|---|
| `origin/main` (`64759da`) | **12.52 MHz** | 78.36 ns | +4.3% |
| two terms | **11.82 MHz** | 81.03 ns | **−1.5%, and under at two seeds of eight** |
| one term — **shipped** | **12.24 MHz** | 78.60 ns | **+2.0%** |

Full rows, MHz, `default 1 2 3 4 5 6 7`:

- baseline — 13.08 12.89 12.75 12.77 12.79 12.73 12.52 12.69
- two terms — 12.74 11.98 12.62 11.82 12.16 12.77 12.41 12.28
- one term — 12.63 12.24 13.09 12.36 12.99 12.82 12.58 13.20

**The median moved 0.31%**, which is a null inside a ~3.6% churn band, and the worst placement moved
2.2%. The two spellings are 3.0% apart at the median and 3.5% apart at the worst seed, and only one
of them meets the requirement — the same finding ADR-0106 recorded on this instrument and ADR-0097
recorded on `fit`. **Quote the distribution of the text that ships.** That is what settled the
spelling, and nothing below reopens it.

### The go/no-go, on `2007d9d` with the pinned OSS CAD Suite. **GO.**

| tree | worst of 8 | median period | vs. requirement |
|---|---|---|---|
| `origin/main` (`2007d9d`) | **12.12 MHz** | 79.97 ns | +1.0% |
| this change, **as shipped** | **12.39 MHz** | 78.75 ns | **+3.3%** |

Full rows, MHz, `default 1 2 3 4 5 6 7`:

- baseline — 12.12 12.41 12.19 13.14 12.35 12.60 12.82 13.00
- shipped — 12.47 13.00 12.50 12.99 12.75 12.39 12.76 12.65

**The change is FASTER than its base here** — +2.2% at the worst placement and 1.5% at the median —
where on the earlier base and toolchain it was 2.2% slower at the worst placement. **Two toolchains
disagree about the sign of this change's effect on the clock, measured on the same two trees'
worth of RTL.** CLAUDE.md already records `fit` as toolchain-dependent, at ~21 cells; this is the
same warning observed on `soc-timing`, and it is the reason to state the verdict and not a
percentage: **the requirement is met at every seed of both distributions, and the period cost of
this change is a null whose sign the instrument does not agree on.** Do not quote either delta as
the price.

Two things moved at once between the tables — the base gained ADR-0108, and the toolchain changed —
so neither is attributable and neither is claimed. What is claimed is only what each internal
comparison supports.

`SOC_MIN_MHZ` does not move and `SOC_EXPECT_SPRAM`/`SOC_EXPECT_EBR` are unchanged at 2 and 20, on
every one of the forty placements above.

## Decision 3 — the depth attribution, which is what this ticket existed to produce

`soc/depth/path_stages.py`, on the placement each tree is graded at and on the default placement,
against ADR-0104's four levels for the general case:

| tree, seed | period | levels | decode's charge | logic | routing |
|---|---|---|---|---|---|
| baseline, default | 76.48 ns | 22 | 8 | 24.36 ns | 52.11 ns |
| shipped, default | 79.20 ns | **20** | 8 | 23.07 ns | 56.11 ns |
| baseline, seed 1 | 77.55 ns | 23 | 10 | 26.98 ns | 50.57 ns |
| shipped, seed 1 (its worst) | 81.69 ns | **22** | 7 | 25.66 ns | 56.01 ns |

**The atomic-only region test costs zero extra logic levels, against the four the general one cost,
and the fetch loop is one level shallower at both placements measured.** ADR-0106's prediction holds
exactly: the four levels were charged to the top of `immediate + reg_rs1`, and there is no such sum
here.

**What moved is routing, not depth** — +7.7% and +10.8% of routing at the two placements while logic
went *down* 5.3% and 4.9%. The `fit` job reads 3934 → 3966 (+32) and the placed SoC 4728 → 4751 (+23),
both inside the ±50 churn band, on a part the SoC already occupies at 89–90%. Read that as what it
is: **the period moved for a reason the level count cannot see, at an occupancy where it would not
have to be this change's 33 cells that did it.** ADR-0088 measured occupancy not setting the period
by ballasting 77% to 95%, so this is not a claim that congestion is the cause — it is a claim that
depth is not.

The lesson for the next attempt at causes 5 and 7 for loads and stores: **the four-level number
still stands and is still the number to beat**, and beating it on levels is now demonstrated not to
be sufficient on its own.

## Decision 4 — co-simulation gets the A extension's region attribute back

`test/COSIM_EXPECTED_FAIL` carried `amoregion.S DISAGREE AT 18`, recorded because the model has
exactly two settings for a region's reservability and this core was neither:

```
  RsrvEventual   the lr.w takes a reservation and the sc.w SUCCEEDS
  RsrvNone       the lr.w raises a LOAD ACCESS FAULT, cause 5
```

The core did a third thing — answered the `lr.w` with zero, took no reservation and raised nothing.
**`RsrvNone` is what this platform is now**, and `test/sail/rv32imc_zicsr.json` says so: four
regions rather than two, the 64 KB data RAM `RsrvEventual` and `AMOArithmetic`, the three around it
`RsrvNone` and `AMONone`. `amoregion.S` **AGREEs**, and the entry comes out with the reason it
closed written where the entry was.

**That entry was expensive while it was open**, which is the reason to record how it closed rather
than just deleting a line: co-simulation is the *only* value-checking oracle the A extension has —
the pin ships no spec model for any of the eleven encodings, so `test/monitor.sim.v` grades pc
continuity on an A retire and nothing else. A baselined program is one that oracle says nothing
about at all.

`atomic_support` was measured rather than read off the schema. Its values are ordered by strength;
at `AMOSwap` the model faults `amoadd.w` and `amo.S`, `amominmax.S`, `amotrap.S` and `lrsc.S` all
stop agreeing. `AMOArithmetic` is the weakest value that admits every encoding this core implements.

**One half of the deviation is still open and is deliberately not asserted.** A plain store to an
address no memory answers is still dropped silently, and the model has memory there, so a program
that wrote and read back would be a divergence again. `amoregion.S` asserts only the load half —
which reads zero on both — and says so where it does it.

**Re-run against the model that claims A, not carried forward.** ADR-0108 flipped Sail's `A` key to
`true` after this work was measured, and the claim being made here is about the tree that merges, so
`amoregion.S` was co-simulated again on the rebased branch: **AGREE**, 382 architectural changes
identical in order and value. `make cosim-suite`: **67/70 agreed**,
matching the baseline exactly in both directions, where it was 66/70 with four entries.

## Decision 5 — `fault_ch0` becomes an oracle for these two causes, having been one for neither

`checks/rvfi_fault_check.sv` at the pin already asserts the two causes this change adds, keyed on
the fault masks: any write mask means `mcause` was written in full with 7, a read mask alone means
5, and both clear means 1. `rtl/writeback.v` held both at zero because a fetch was the only refusal
this core had. They are driven now, from decode, as the access the instruction *would* have made —
an `lr.w` sets the read mask, an `sc.w` the write mask, and an AMO both, which lands it on the store
cause the way the check's own arm order says it should.

`formal/wrapper.v` makes `atomic_supported` a free input, which is what makes those two arms
reachable at all: tied high, the check would still be asking only about fetches. Nothing is assumed
about it, for the same reason nothing is assumed about `mem_reservable` — an atomic decode either
faults with it or issues without it, and both are this core's behaviour.

**No depth moves and F and G do not have to be re-measured.** This adds no stall reason, lengthens
no stage and does not widen the scoreboard: a faulting atomic clears every flag in decode, so it
raises no atomic wait and occupies the pipeline exactly as a trapping instruction already did.
`make -C formal check` is 86 checks and 86 pass, `EXPECTED_CHECKS` unchanged.

## What ran

| gate | result |
|---|---|
| `make test` | **70/70**, `EXPECTED_FAIL` exact both ways; `amoregion.S` PASS at 604 retires / 514 spec-checked |
| `make test-units` | 9 benches, all PASS |
| `make probe-gates` | all probes green (it runs inside `make test`) |
| `make mutation-check` | **8 mutations**, each caught by exactly its declared detectors |
| `make window-test` | 14 elaborations, each rejected or accepted as required |
| `make cycles` | columns add up, `unattributed` 0, the `atomic` bucket unmoved — a refused atomic clears every flag, so it raises no wait |
| `make lint` | clean in both passes |
| `make waves` | PASS, and no `sorry:` note added |
| `make -C formal check` | **86 checks, 86 pass**; `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact both ways |
| `make -C formal components_{decoder,executor,accessor,pcloop,traps}` | all PASS (`pcloop_cover` PASS first) |
| `make -C formal complete` / `complete_cover` | PASS / PASS |
| `make -C formal complete-exclusions` / `interrupt-tie-off` | PASS / PASS |
| `make -C formal dmemcheck` / `imemcheck` / `nonperturbation` | PASS / PASS / PASS |
| `make fit` | **3966** of 4000 budgeted, from **3934** — both the `fit` job's, on this branch and on `2007d9d` |
| `make cosim-suite` | **67/70** agreed, baseline exact both ways |
| eight-seed sweeps, five trees over two bases and two toolchains | the two tables above; **GO**, every seed of both distributions over 12.00 MHz |

Both new graded comparisons have a demonstrated red direction, forced rather than argued:

- `test/mutations/atomic-region-ignored.patch` sets `atomic_fault` to zero — every atomic executes
  wherever it is pointed, which is the behaviour these two causes replaced. `amoregion.S` goes
  `FAIL 4` and `decoder_tb` goes red; `amo.S`, `amominmax.S`, `lrsc.S` and `lrsclock.S` do **not**,
  and that is right, because every atomic in them is inside the data RAM.
- `test/mem_tb.v`'s `check_atomic` drives `atomic_addr` while `mem_addr` is held inside the window,
  so a module answering the question about the wrong wire reports every address supported.

**`sc-reports-success` loses a detector to this change, and that is a real consequence rather than a
tidy-up.** `amoregion.S` caught it at an `sc.w` outside the region the platform will reserve; that
`sc.w` faults in decode now and never reaches the accessor, so **the reservation's refusal is no
longer reachable from software at all**. It stays in `rtl/accessor.v` as the second half of one
statement — a platform that tied `atomic_supported` high would still not let an `sc.w` claim a write
that went nowhere — and `test/accessor_tb.v` and `components_accessor` are what grade it now.
`test/MUTATION_DETECTORS` carries that paragraph where the line was.

`formal/traps.sv` gained the two causes as an oracle written from the ISA rather than transcribed
from the decoder: the eleven encodings off the table, `expected_cause` extended with both, and
`must_not_trap` extended with an aligned atomic at an address the platform **does** answer — without
that half the file would pass on a core that faulted all eleven.

## Consequences

- **Causes 5 and 7 exist, for eleven encodings and no others.** CLAUDE.md's ISA-target section says
  which and why, because the asymmetry is otherwise unreadable: the same fault for a plain load or
  store waits on the top of a 32-bit sum and is still declined on ADR-0104's number.
- **The deviation ADR-0104 recorded is narrower and still open.** An out-of-region load reads zero
  and an out-of-region store is dropped, silently. `amoregion.S` asserts the load half in band.
- **`mtval` stays zero**, and ADR-0104's decision 5 named this change as the one that should revisit
  it — the faulting address is not recoverable from `mepc` here the way an instruction fault's is.
  It is not revisited: the address is `rs1` and the handler can read it out of the register file,
  which is a weaker version of the same argument and is worth writing down as the reason rather
  than leaving the ADR's suggestion looking unaddressed. A handler that needs it without decoding
  the instruction is what would change this.
- **`FIT_MAX_LC` does not move**, and this ADR deliberately does not raise it. The `fit` job reads
  **3966 of 4000**, from 3934 — +32, inside the ±50 churn band, leaving **34 cells of headroom**,
  which is *inside* that band. So the ratchet no longer sits outside the churn CLAUDE.md says it
  must sit outside, and the change that re-derives it should be the one that owns that value rather
  than this one raising it in passing.
- **There is no local-to-CI offset to convert with, and this ADR quotes CI for that reason.** The
  same day this landed, one bit of a read-only constant moved the local count +53 and the job's −4.
  Whatever relationship the two instruments once had, it is not additive. Quote the `fit` job.
  (The local instrument was unavailable here in any case: `nextpnr-ice40` from Homebrew stopped
  loading after a `boost` 1.92 upgrade, which is also why decision 2's second table is on the
  pinned OSS CAD Suite.)
- **The multi-core milestone is unaffected.** The bit is a PMA and a second hart does not change what
  region answers an atomic.
- **`make cycles`'s hazard share reads 37.1% here against CLAUDE.md's 35.7%, and that is the SUITE
  moving, not the core.** `amoregion.S` grew from 121 retires to 604 and now takes seventeen traps.
  The 35.7% is ADR-0084's, re-derived by ADR-0099 on the suite as it was then; it is not re-derived
  here, because a number that travels with a commitment should be re-taken by the change that means
  to spend it, not adjusted in passing by one that changed a test program.
