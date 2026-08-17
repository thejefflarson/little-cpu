# ADR-0113: The load/store region test is priced three ways, and every price is the sum

**Status:** Accepted · 2026-08-16 · *Re-takes
[ADR-0104](0104-the-fetch-bus-refuses-and-the-data-bus-cannot-afford-to.md) decision 2's price on a
tree seventeen commits later and a different toolchain, and re-prices the
[ADR-0111](0111-margin-over-twelve-megahertz-is-a-currency-with-a-price-list.md) row that quotes it.
Locates the cost, which had been assumed rather than measured. Declines two further spellings and
records a fourth that meets the clock and is still not shipped, with the reason. No `rtl/` change,
no commitment amended, `SOC_MIN_MHZ` and `FIT_MAX_LC` untouched.*

## Context — what was open, and what was assumed about it

ADR-0104 built the decode-time region test for plain loads and stores, measured it at four extra
logic levels and 10.57–11.00 MHz over four seeds, and declined it. ADR-0109 then shipped the same
test for the eleven A encodings for **zero** extra logic levels, because an atomic's address is
`rs1` verbatim, and left the lesson that beating four levels is necessary and not sufficient — the
atomic version was a level *shallower* and still moved the worst placement 2.2%, in routing.

So the standing account of the load/store half was: **four logic levels, somewhere in decode.** Where
in decode was never measured. Two readings were live and they imply opposite work:

- the levels are the **compares** — three windows, a union, a cause split — and the **merge** into
  `trap_pending` and the `next_pc` override, all of which are shared with causes that already exist;
- the levels are the **dependency on the top of `immediate + reg_rs1`**, and the compares are free.

This ADR settles that first, because the answer decides which spellings are worth building.

## Decision 1 — the attribution, and it is not where the levels were assumed to be

The instrument is one line. Build the whole region test — `rtl/imemory.v`, `rtl/memory.v` and
`rtl/timer.v` each answering about a second address, the SoC uniting the three answers,
`rtl/decoder.v` raising cause 5 for the five load encodings and cause 7 for the three store
encodings under the existing misalignment order, and the fault joining `trap_pending` — and then
change **only what address the platform is asked about**:

```
  assign data_addr = mem_addr_calc;   // the effective address: immediate + reg_rs1
  assign data_addr = reg_rs1;         // the same test, one adder earlier
```

The second is semantically wrong and can never ship; it tests the base register's region rather than
the access's. It is a depth probe, and it is decisive. Six placements a side, paired per seed
against the same base:

| tree | paired median Δ period | seeds over 12.00 MHz |
|---|---|---|
| the whole test, on `mem_addr_calc` | **+16.45%** | 0 of 6 |
| the whole test, on `reg_rs1` | **+0.52%** | 6 of 6 |

**The compares, the union of three windows, the cause split, the RVFI fault masks and the merge into
`trap_pending` and `next_pc` are together a null.** Three of the six per-seed deltas are negative;
the median sits an order of magnitude inside the ~3.6% churn band. Every nanosecond of the decline is
the sum.

That refutes the more attractive of the two readings, and it is worth stating why it was attractive:
the trap-cause priority chain and the `next_pc` override *look* expensive, they are on the fetch
loop, and ADR-0109 had just shown that a change there could move the period without moving the level
count. None of that made them the cost here. `soc/depth/path_stages.py`'s per-stage buckets do not
show this either — they charge `decode` 8 levels on the base and 9 with the test on the sum, while
the period moves 16% — which is the same warning ADR-0098 attaches to that tool, arriving from the
other direction: **the buckets are an attribution of a path, not of a decision.** The one-line
substitution is the attribution that answered the question.

### The cost is monotone in how far up the carry chain the answer reaches

With the levels located, the natural question is whether the answer has to wait for *all* of the
sum. It does not have to, and it is still not affordable. Three depths, measured:

| what the answer reads | paired median Δ period | worst of N | seeds over 12.00 |
|---|---|---|---|
| `mem_addr_calc[31:16]` — three exact windows | **+17.30%** (16 seeds) | 10.43 MHz | 0 of 16 |
| `mem_addr_calc[31:18]` — one coarse 256 KB window | **+15.95%** (6 seeds) | 10.55 MHz | 0 of 6 |
| `mem_addr_calc[12]` — one carry, and nothing above it | **+9.10%** (16 seeds) | 10.96 MHz | 1 of 16 |
| nothing from the sum at all | **+0.52%** (6 seeds) | 12.18 MHz | 6 of 6 |

The middle two are decisions 2 and 3. Read the column as one statement: **the price is set by which
bit of the effective-address adder the fetch loop has to wait for, and by nothing else.** Bit 31
costs about 17%, bit 12 about 9%, no bit at all is a null. The width and exactness of the comparison
behind that bit are not measurable next to it.

## Decision 2 — angle 3, the coarse check, buys nothing and is declined

The cheapest exact statement is three equalities on aligned power-of-two windows. The obvious way to
undercut it is to stop being exact: every region on this platform lies below 256 KB, so
`~|mem_addr_calc[31:18]` is one fourteen-bit reduction that catches a wild pointer and misses an
access one word past the end of a region.

It measures **+15.95%** of median period over six placements, worst 10.55 MHz, and misses the
requirement at every one. Against the exact test's +16.45% on the same six seeds that is a null — the
two are inside each other's churn band.

**So the exactness of the region decode was never the price**, and an approximation that still reads
the top of the sum inherits the whole of it. This is the specific form ADR-0088's rule takes here:
yosys and ABC had already done everything derivable from the expression, and a narrower expression
over the same late input is the same circuit. Angle 3 is declined on its own measurement rather than
on the exact test's.

## Decision 3 — angle 1, breaking the dependency on the sum, gets halfway and is declined

The adder is not needed in full. A plain load or store adds a **sign-extended 12-bit immediate** to
`rs1`, so its effective address lies in the 4 KB page `rs1` names or in one either side, and which of
the three is decided by the carry into bit 12. The adder publishes no carry of its own, but it does
not have to:

```
  // bit 12 of a sum is the operands' bit 12 exclusive-ored with the carry into it,
  // and a load or store immediate is sign-extended, so its bit 12 is its bit 11
  assign addr_carry12 = mem_addr_calc[12] ^ immediate[11] ^ reg_rs1[12];
```

The platform then answers about **three pages at once** — all functions of `rs1`, which is a register
output — and decode selects one on `{immediate[11], addr_carry12}`. Neither neighbouring page is
computed by adding: each window is a whole number of pages on a multiple of its own size, so the page
above it is in range exactly when this page is in range and is not the window's last, or when this
page is the one below the window. Two equalities and no carry chain, which is the point — an
increment there would hang a chain off the register file's output, on the loop that already ends at
the fetch address.

Sixteen placements, paired: **+9.10%** of median period, worst **10.96 MHz**, and **one seed of
sixteen** clears 12.00. Declined.

It is declined having worked exactly as designed, and that is the useful part. It removes nineteen
bits of the effective-address carry chain from the answer and recovers a little under half the cost —
+9.10% against +17.30% — which is what fixes the shape of the price as monotone in the carry rather
than a threshold. **Halving the wait halves the bill, and half the bill is still four times the
margin available.**

A control was run rather than assumed, because the three-page answer and the carry select are two
changes in one text. Tying `addr_carry12` to a constant — the same three-page answer, the same
platform round trip, the same select, no dependency on the adder at all — measures **+1.66%** of
median period over six placements and holds 12.00 MHz at every one. So the platform round trip, the
three-page decode and the select are not the cost either: **the single exclusive-or that reads bit 12
of the adder is.**

## Decision 4 — the spelling that meets the clock is not a region test, and is deliberately not shipped

The control above is one step from a design that is sound. Ask the platform whether `rs1`'s page **or
either page beside it** is memory it answers, and fault when none of the three is. That is one bit,
one equality per memory plus two boundary equalities, and no arithmetic anywhere:

```
  assign data_page_mapped = data_page[19:PAGE_BITS] == BASE_PAGE[19:PAGE_BITS] ||
                            data_page == BASE_PAGE - 20'd1 ||
                            data_page == BASE_PAGE + 20'(1 << PAGE_BITS);
```

It is **one-sided by construction**: a 12-bit immediate cannot reach past the neighbouring pages, so
the bit is never low for an address the platform does answer, and no correct program is ever faulted.

Sixteen placements, paired against the same base: **+3.00% of median period, 14 of 16 deltas
positive** (sign test p ≈ 0.004, so the sign is real while the magnitude sits inside the churn band),
worst placement **12.21 MHz — over the requirement at 16 of 16.** `make fit` reads 3999 against the
base's 3969 on the same local toolchain, +30 cells, inside the ±50 band. It meets every gate this
repo would put in front of it.

**It is not shipped, and the reason is what it would make `mcause` mean.** The fault is a function of
the base register's page, not of the access, so two instructions computing the *same* effective
address behave differently:

```
  li a0, 0x30000                 li a0, 0x2fff8
  lw a1, 0(a0)     faults        lw a1, 8(a0)     does not fault
```

Both name 0x30000, which no memory on this platform answers. A platform attribute that depends on how
software chose to compute an address is not a PMA, and the privileged spec's access-fault causes are
defined against the access. Shipping this would replace one recorded deviation — an out-of-region
load reads zero and an out-of-region store is dropped, silently — with a *second* deviation that is
harder to describe, cannot be expressed in `test/sail/rv32imac_zicsr.json`'s address-based region map
at all, and would therefore ship two new causes with no oracle outside this repo's own assertions.

It also spends more than half the margin there is. The worst placement goes 12.39 → 12.21 MHz,
leaving **+1.75%** over the requirement, which is under one churn band — so the next change's
go/no-go would stop being attributable, which is the property ADR-0111 exists to protect.

**The design is recorded here in full, priced, with its distribution, so that a later decision to
take it is a decision and not a rediscovery.** What would justify it is a reason to value catching a
wild pointer above keeping `mcause` a function of the access. That is a product call, not a timing
one, and this ADR does not make it.

## Decision 5 — ADR-0111's row is re-priced, and the requirement is unmoved

ADR-0111's price list carries `causes 5 and 7, loads and stores (ADR-0104) | ~13–16% | conformance |
takeable at ~17–20%`. Re-swept here at sixteen seeds on `9e26cfd` with the pinned OSS CAD Suite, the
price is **+17.30% of median period and 18.8% at the worst placement**, and the cheapest spelling
that keeps the fault a function of the access is **+9.10%**. So the row's price stands at the top of
its range for the exact spelling and gains a second, cheaper entry that is still four times today's
margin. Both are worst-seed costs on the tree they were measured on and both are perishable, per that
ADR's own rule.

`SOC_MIN_MHZ` stays at 12.0. `FIT_MAX_LC` is untouched and deliberately not raised — nothing here
ships, and the ratchet is being re-derived elsewhere.

## What ran, and its provenance

Every number above is `soc/timing_sweep.sh` with `SOC_PROG` at its default, on **base `9e26cfd`**,
one machine, and the **pinned OSS CAD Suite** — yosys 0.68+48 (`ff5817c34`), nextpnr-ice40
0.11-1-g62e659ed — because the Homebrew `nextpnr-ice40` on this machine stopped loading after a
`boost` 1.92 upgrade, the same failure ADR-0109 recorded. Both sides of every comparison share that
base and that toolchain, so each comparison is internal; **none of these numbers compares to
ADR-0104's**, which were taken on a Homebrew toolchain seventeen commits earlier.

Sixteen seeds, not four. Four seeds have twice given a different verdict than eight in this repo, the
placement spread on the base measured here is **76.51–80.68 ns, 5.5%**, well over the 1–2% CLAUDE.md
quotes, and the analysis is the **median of per-seed paired deltas plus a sign test**. Worst-of-N is
reported because it is what `SOC_MIN_MHZ` grades, and is not the analysis.

Full rows, ns, `default 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15`:

- base — 80.22 76.92 79.99 77.01 78.44 80.68 78.36 79.06 79.42 76.51 76.52 77.22 78.23 79.89 78.14 78.65
- exact, on the sum — 92.75 91.43 91.00 92.40 91.99 89.36 90.64 95.88 91.37 93.86 92.60 95.14 91.79 89.05 93.21 95.22
- three pages, carry into bit 12 — 83.33 86.60 83.78 91.28 91.24 86.65 84.84 89.10 85.00 82.84 90.28 84.88 87.88 85.83 87.91 83.71
- one-sided page test — 80.84 79.46 81.09 80.15 81.74 79.91 80.43 80.97 79.81 80.11 80.14 79.34 80.97 77.40 80.68 81.90

Six seeds each, `default 1 2 3 4 5`:

- coarse 256 KB window, on the sum — 93.70 94.83 90.07 90.90 89.79 92.86
- probe, the whole test on `reg_rs1` — 81.08 82.12 79.96 81.26 77.54 77.25
- probe, three pages with the carry tied off — 79.43 80.88 79.78 83.30 80.80 80.93

`SOC_EXPECT_SPRAM` and `SOC_EXPECT_EBR` came out at 2 and 20 on every synthesis behind the
eighty-two placements. Nothing under `rtl/` is changed by this ADR, so the suite, the unit benches, the
component proofs, the generated checks and co-simulation are the base's and were not re-run for a
documentation-only change — which is stated rather than left to be assumed.

**Levels are reported and deliberately not used as the verdict**, which is this measurement's other
finding. At the default placement: the base is 22 levels (21 LUT, 1 carry); the exact test on the sum
is 27 (28 LUT, 0 carry) and 15.6% slower; the three-page version is also **27** (23 LUT, 4 carry) and
3.9% slower; the probe on `reg_rs1` is 25 (23 LUT, 3 carry) and 1.1% slower. Two trees at the same
level count are 11% apart in period. ADR-0109 found levels falling while the period rose; this finds
the level count flat across a 4:1 range of cost. **On this part the level count orders nothing** —
quote the distribution.

## Consequences

- **The deviation ADR-0104 recorded stays open and unchanged.** An out-of-region load reads zero and
  an out-of-region store is dropped, silently, for the eight non-atomic encodings. Causes 5 and 7
  remain implemented for the eleven A encodings and nothing else.
- **The four-levels-in-decode account is retired and replaced by a located one.** The cost is the
  fetch loop waiting on a bit of the effective-address adder, and it scales with which bit. A future
  candidate is a null before it is placed unless it moves that dependency, and moving it *down* the
  chain is worth a measurable but insufficient fraction.
- **Three angles are priced rather than one.** Exact-on-the-sum, coarse-on-the-sum and
  carry-select-at-bit-12 are all declined on their own sixteen- or six-seed distributions, and the
  first two are within a churn band of each other, which is the finding that closes the "make it
  cheaper by making it coarser" direction.
- **A fourth spelling meets the clock and is recorded unspent**, with the argument against it stated
  as a claim about `mcause` rather than about timing, so that taking it later is a decision about
  what the causes are allowed to mean.
- **What would have to change for a fifth attempt**, and there is no cheap technical one left. The
  fetch address stops depending on this cycle's decode, which is the no-wrong-path-state commitment,
  priced and declined twice; or the trap stops being committed in decode, which is the
  traps-in-decode commitment; or roughly 17% of worst-seed margin appears from elsewhere, which
  ADR-0111 prices and no item on its list supplies; or the project decides that a cause raised on
  the base register is worth having, which is decision 4 at +3.00% and is the only one of the four
  that is affordable today. **No change to the memory map helps.** Decision 4's test is exact only
  if no unmapped page ever abuts a mapped one, which no finite map satisfies — every region has a
  first page past its end — so widening or aligning the regions moves the inexactness without
  removing it.
