# ADR-0103: The performance monitor reads zero, and the reference model was the wrong instrument

**Status:** Accepted · 2026-08-11 · *Corrects finding 3 of
[ADR-0048](0048-what-an-independent-read-of-the-no-oracle-rtl-found.md). Amends
[ADR-0043](0043-the-reference-model-is-configured-as-this-core.md)'s extension audit with one more
enabled extension and its residual. Supplements
[ADR-0005](0005-traps-and-csrs-commit-in-decode.md).*

## Context

`rtl/csrs.v` raised illegal instruction on 87 addresses: `mhpmcounter3`–`mhpmcounter31`
(0xB03–0xB1F), `mhpmcounter3h`–`mhpmcounter31h` (0xB83–0xB9F) and `mhpmevent3`–`mhpmevent31`
(0x323–0x33F). The RV32 privileged specification's machine-mode performance-monitor section says:

> The hardware performance monitor includes 29 additional 64-bit event counters,
> `mhpmcounter3`–`mhpmcounter31`. The event selector CSRs, `mhpmevent3`–`mhpmevent31`, are 64-bit
> WARL registers that control which event causes the corresponding counter to increment. […]
> **All counters should be implemented, but a legal implementation is to make both the counter and
> its corresponding event selector be read-only 0.**

Trapping on all 87 is neither of the two things that sentence describes. `misa` is untouched by the
question: the machine-level counters belong to M-mode's own facility, and `Zihpm` — which has no
`misa` bit — is the *unprivileged* shadow extension, so implementing these does not widen the ISA
string.

## The part worth keeping is how the question was closed the first time

ADR-0048 was an independent read of the no-oracle RTL against the specification, and it looked
directly at three of these addresses. It retired them:

> **`mhpmcounter3` (0xB03), `mhpmcounter3h` (0xB83) and `mhpmevent3` (0x323)**: the reference model
> raises illegal instruction on all three, exactly as this core does — there is nothing to close.

**That is a measurement of the wrong thing, and it is circular.** Two failures stacked:

1. **A reference model is authoritative about semantics, not about which permitted choice a machine
   makes.** The spec expressly allows read-only zero *and* is expressly asking for the counters;
   whatever Sail does, it is one machine's answer, and the question was what the sentence requires.
2. **Sail trapped because this repo told it to.** `test/sail/rv32imc_zicsr.json` set
   `Zihpm: { supported: false }`, whose comment read *"No Zihpm: mhpmcounter3..31 and
   mhpmevent3..31 do not exist here."* The model was echoing the core's own scope statement back.
   Agreement with a model configured to match us is not evidence about the specification — it is
   the same correlated-author error ADR-0048 was written to defeat, arriving through the one oracle
   that was supposed to be independent.

The generalisation, which is not about CSRs: **an independent oracle stops being independent
exactly where we configured it.** Every knob in that config file is a claim this repo made, so any
question whose answer is a knob has to be settled against the specification first and the model
second. ADR-0043 already says a config override "is where you can talk the oracle into agreeing
with you"; this is what that looks like when it happens.

## Decision

Implement all 87 as read-only zero. No counter state, no event logic, no `mcountinhibit`, no
`mhpmevent`*n*`h` (those exist only with Sscofpmf). The read mux's default arm already answers
zero, so the whole change is a counter-number compare and three window compares feeding
`implemented`. Writes needed nothing at all: none of the 87 addresses is read-only by encoding, and
the WARL fallback already returns the value the register held before the write, so a write to one
is the legal no-op it has to be.

## What it costs, measured

All numbers below are one tree with a local Homebrew yosys — quote CI's `fit` job for the ratchet,
not this. Area:

| | `fit` (littlecpu) | SoC `SB_LUT4` (synth) | SoC `ICESTORM_LC` (placed) |
|---|---|---|---|
| base | 3482 | 3878 | 4219 |
| shipped | 3464 | 3931 | 4274 |

`fit` moved **−18**, inside its ±50 churn band and therefore evidence of nothing in either
direction. The SoC moved **+53 synthesised LUTs and +55 placed cells**, 1.3% of its logic. Both SoC
numbers are deterministic: the synthesis count has no placement in it, and the placed count came out
identical at all four seeds on every text measured here, so nothing about them is a sample.

Period, `soc/timing_sweep.sh`, four seeds, ns:

| | placements | median | worst |
|---|---|---|---|
| base | 74.28 76.23 76.67 77.86 | 76.45 | 77.86 (12.84 MHz) |
| shipped | 76.41 76.47 77.30 78.22 | 76.89 | 78.22 (12.78 MHz) |

The median moved **+0.6%**, inside the ~3.6% churn band, which is a null in both directions. The
worst of four placements holds 12 MHz with 6.5% of margin against the base's 7.0%. `SOC_MIN_MHZ`
does not move and did not need to.

**Against ADR-0096's prior, +53 LUTs is the surprise worth stating, and it is a scope error in
reading that ADR rather than a contradiction of it.** ADR-0096 measured the CSR file's entire WARL
write mux and every legal-value mask together at **one LUT** — but everything it priced was the
*write* side, which this change does not touch at all. What moved is `implemented`, which is address
decode, and address decode is the one part of a read-only-zero CSR that is not free. The right prior
for "what does another CSR cost" is therefore not one LUT; it is however many address bits have to
be compared, and 87 addresses in three ranges is 53 of them.

## The two spellings: the cheaper one was declined, and readability is the reason

The three ranges can be written as three window compares, one per spec range, or as two — the low
and high halves of one counter differ only in `addr[7]`, so one compare covers both counter windows.
Measured the same way, four seeds each:

| | SoC `SB_LUT4` | placed `ICESTORM_LC` | placements (ns) | median | worst |
|---|---|---|---|---|---|
| three windows (shipped) | 3931 | 4274 | 76.41 76.47 77.30 78.22 | 76.89 | 78.22 (12.78 MHz) |
| two windows | 3899 | 4235 | 74.31 76.80 78.42 81.82 | 77.61 | 81.82 (12.22 MHz) |

**The area difference is real and is not churn**: 32 synthesised LUTs and 39 placed cells, with the
placed count identical at every seed on both texts. That is ADR-0097's spelling-dependence measured
again — two texts stating one fact, 39 cells apart — and it is worth knowing that it reproduces on a
block as small as this one.

**It is not what decided the spelling, and the period is not either.** The two medians differ by
0.9%, inside the churn band, so nothing separates them there; the two-window text did produce the
single worst placement of the twelve taken here, 81.82 ns for 1.8% of margin against the shipped
text's 6.5%, but four samples a side make that a weak signal and not a cause, and it is recorded as
an observation rather than leaned on.

What decided it is that **three windows read the way the specification reads** — one named constant
per range, each with its address range beside it — and 39 cells is inside `fit`'s churn band and 0.9%
of the SoC. Readable is one of the four goals, not a tiebreak, and this is the case where it is the
only goal that separates two texts. The declined spelling is named in `rtl/csrs.v` so that nobody
re-derives it as an unclaimed saving.

## Co-simulation: the model is configured, not baselined

Sail traps on all 87 with `Zihpm: false`, so `test/asm/hpm.S` first ran `DISAGREE AT 2` — the model
entered its trap handler at the first `csrr`. `test/COSIM_EXPECTED_FAIL`'s decision procedure asks
first whether the reference model can be **configured** to be this machine, and here it can:
`Zihpm: { supported: true }` with `writable_hpm_counters` left at all-zero is a model that reads all
87 as zero and swallows the writes. `hpm.S` then agrees on all 50 architectural changes, and the
baseline keeps exactly the two timer entries it had. **No divergence is being baselined by this
change.**

Two things about that knob are recorded rather than hidden:

- **It is not a claim that the ISA string grows.** `Zihpm` has no `misa` bit, and it names the
  unprivileged shadows; the model happens to gate the machine-level counters on it too.
- **RESIDUAL, the same shape as `Zicntr`'s.** Enabling it also gives the model the unprivileged
  `hpmcounter3`–`hpmcounter31` shadows (0xC03–0xC1F), which this core traps on. No knob separates
  them, and no program in `test/asm` reads them — the same standing exposure `Zicntr` already
  carries for 0xC00–0xC02, and the same thing will catch it: a suite growing one instruction.

## Consequences

- **`test/asm/hpm.S` joins the suite**, which is 63 programs. It reads both ends of all three ranges
  and a sample in the middle, exercises all six accessing instructions including the immediate
  forms, and **ends with an access one past the top of the counter range that must trap** — without
  which every "no trap" assertion in the file is a trap counter that a broken handler would also
  leave at zero. `test/EXPECTED_FAIL` and `test/COSIM_EXPECTED_FAIL` are unchanged.
- **`test/csr_tb.v` carries the negative half, and it is the half that matters.** A range compare
  that is too wide is the failure mode, and no `.S` program can see it: the addresses just past the
  top of each range, the counter numbers 0–2 inside each window, and number 3 of the window *above*
  each range must all stay illegal, while `minstret` (0xB02) and `mscratch` (0x340) must still
  answer with their own registers rather than the ranges' zero. That last set is the only one that
  catches a compare taking one address bit too few — the addresses immediately past each range are
  number 0 of the next window and stay illegal on the counter-number test alone, so they cannot see
  it. Six mutations of the shipped compare were forced and every one is caught: the ranges deleted,
  the counter-number test dropped, the counter-number test off by one, and each of the three windows
  doubled.
- **The formal side is untouched and was re-run rather than reasoned about.** The whole SYSTEM
  opcode is already in `formal/COMPLETE_EXCLUSIONS`, so widening the legal CSR set cannot move
  `complete`; no harness under `formal/` asserts anything about which addresses are implemented; and
  no stall reason, stage length or scoreboard width changed, so the derived BMC depths stand. 85
  generated checks pass, the four component proofs pass, `complete`, `complete_cover` and
  `nonperturbation` pass.
- **ADR-0048's finding 3 is corrected in place, not deleted**, with the retired paragraph left
  standing and the correction quoted underneath it. The same is done to `test/asm/csrset.S`'s
  header, which repeated the retired suspicion next to the two CSRs it did fix.
- CLAUDE.md's conformance sentence said every mandated CSR was implemented. It now says so truly,
  and adds the rule this cost: **a register the spec merely recommends is still owed a decision.**
