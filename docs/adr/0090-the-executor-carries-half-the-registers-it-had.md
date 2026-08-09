# ADR-0090: the executor carries half the registers it had, one negator and an unsigned multiplier

**Status:** Accepted · 2026-08-09 · *Three behaviour-preserving edits to `rtl/executor.v`. This is
an AREA result, like the one before it. The period moved less than the churn band and is a null in
both directions.*

## Context

[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md) left a rule: yosys and ABC already
do everything derivable from the text of an expression, so an edit that restates the same arithmetic
in the same terms is a null, and the win is in stating a fact that lives outside the expression.
Eleven such edits were worth −84 on `make fit` and −169 placed cells on the SoC, and **each one
alone was inside the ±50 churn band.**

Three facts about the executor were still unstated, and unlike the eleven, each of these clears the
band on its own.

## Decision

### 1. The divider shifts the remainder through the dividend register

The divider carried three 64-bit registers to do a 32-bit division, plus a 64-bit compare and a
64-bit subtract. Its own loop invariant implies `mul_div_x <= div_x < 2**32`, so the top halves were
dead — but that fact lives in the invariant, not in the expression, and the mapper cannot see it.

Hold the divisor still and shift the remainder through the dividend register instead, folding the
quotient register into the dividend one: a quotient bit enters at the bottom on the same edge the
dividend's top bit leaves. Three 32-bit registers, and one 33-bit subtract whose borrow is the
quotient bit inverted.

The widths are exact rather than generous. `div_rem < div_divisor <= 2**32-1` holds every iteration,
so the shifted remainder needs 33 bits; bit 32 of the 33-bit difference is a correct
"shifted remainder is below the divisor" in both cases, because above `2**32` the shifted remainder
already exceeds any divisor and the difference stays below `2**32`; and a borrow can only fire below
`2**32`, so the write back to 32 bits cannot lose a bit. Divide-by-zero never enters the loop. Same
32 iterations, same 33-cycle latency.

### 2. One negator at completion

`-mul_div_store[31:0]` and `-mul_div_x[31:0]` sat in two arms of a four-arm one-hot mux, which is
two independent 32-bit negators that the mapper keeps both of. Select the magnitude first — the
quotient for DIV/DIVU, the remainder otherwise — and negate once, when DIV's operand signs disagree
or REM's dividend was negative.

### 3. The multiplier is unsigned, with a correction on the high half

`$signed({sx, rs1}) * $signed({sy, rs2})` is a 33x33 product. Four `SB_MAC16` do a 32x32 and the
33rd partial-product row lands in soft logic. A negative two's-complement operand contributes
exactly one subtraction of the *other* operand weighted at bit 32, so
`signed_hi = unsigned_hi - (a31 ? b : 0) - (b31 ? a : 0)` is two conditional subtracts on the high
half, with nothing borrowing into it and the low half untouched. The two enables are the ones the
old spelling already computed.

## The measurement

`make fit` is the core alone; `make soc-timing` is the SoC that places. Base is `4a59607`, measured
in this tree with the same toolchain in the same sitting, and reproducing that commit's recorded
numbers exactly.

| after | `make fit` | delta | SoC placed `ICESTORM_LC` | delta |
|---|---|---|---|---|
| base `4a59607` | 3874 | — | 4600 | — |
| 1. shift-remainder divider | 3725 | **−149** | 4459 | **−141** |
| 2. one negator | 3521 | **−204** | 4244 | **−215** |
| 3. unsigned multiply | 3470 | −51 | 4200 | −44 |
| **total** | **3470** | **−404 (−10.4%)** | **4200** | **−400 (−8.7%)** |

`ICESTORM_DSP` stays at 4 throughout, which is the check that change 3 moved the 33rd row out of
soft logic rather than moving the whole product there.

The first two clear the ±50 band by three and four times over — the first two edits in this
direction that would have survived being proposed on their own. The third does not, and is quoted as
what it is: **−51 is a band-edge number and means nothing by itself.** It is here because the DSP
count says the mechanism is the one claimed, and because it is free once the other two are taken.

`FIT_MAX_LC` moves with each step: 4100 → 3950 → 3750 → 3700, keeping the ~200-cell headroom it had
for churn and for CI's yosys reading about 21 higher than a local one.

### The period does not move

up5k, four seeds each, `soc/timing_sweep.sh`:

| | sorted, ns | worst | median | best |
|---|---|---|---|---|
| base `4a59607` | 75.67 77.53 78.34 78.59 | 78.59 | 77.94 | 75.67 |
| this | 74.71 75.44 75.62 76.68 | **76.68** | **75.53** | 74.71 |

**−2.4% on the worst, −3.1% at the median, −1.3% on the best** — against a ~3.6% edit-churn band and
a 1–2% placement spread. Every number moved the same way and every one of them is inside the band,
so this is a null and must be read as one: **it is not a speed result and must not be quoted as
one.** ADR-0088 already measured that occupancy does not set the period on this part, by ballasting
77% to 95% for no change at all, and nothing here disturbs that. The worst of the four placements is
13.04 MHz against `SOC_MIN_MHZ`'s 12.0 — 8.0% of period margin — and `SOC_MIN_MHZ` is untouched.

`make cycles` is byte-identical to the base run: 32 624 cycles, 15 654 retires, CPI 2.08, and the
same split across the six stall reasons. Nothing here touches a stall reason, a stage length or the
scoreboard, so F and G are not re-measured and the BMC depth table does not move.

## The proof got smaller, and the magnitude restriction widened

The old invariant reasoned about a divisor scaled by the iterations left, and that scaling reached
`2**64`: the equation was over 64-bit modular arithmetic, so a wraparound solution existed for
k-induction to start from, and ruling it out took both an extra assertion and the recorded
restriction that the loaded magnitudes are at most 15.

Restated over the two registers that now move, the scaled divisor and its shift are gone. With
`n` the counter and `k = 32 - n` the iterations run:

- `(div_quot & (2**k - 1)) * div_divisor + div_rem == div_mag_x >> n`
- `div_rem < div_divisor`
- `div_quot >> k == div_mag_x & (2**n - 1)`

The product is 32 bits by 32 into 64, and `(2**32-1)**2` is below `2**64`, so it cannot wrap at all:
the equation over 64-bit modular arithmetic *is* the equation over the integers, and the
wraparound-excluding assertion is deleted rather than restated.

What the restriction still buys is solver time on one symbolic product, and the width was measured
rather than guessed:

| cap on the loaded magnitudes | verdict |
|---|---|
| 15 (the old one) | passes |
| **255 — shipped** | **successful proof by k-induction, 88 s** |
| 4095 | no verdict in ten minutes |
| 65535 | no verdict in ten minutes |
| none | no verdict in ten minutes |

So: **widened from 15 to 255, not dropped.** It remains a recorded restriction on the proof rather
than a claim about the design, and `test/exec_tb.v`'s randomized vectors remain what covers full
width. The ceiling is between 255 and 4095 and was not narrowed further, because nothing reads the
exact number.

## The FORMAL block was proving a sequence the pipeline never produces

`rtl/executor.v`'s `FORMAL` block assumed `in` holds steady through a multi-cycle divide — `in.rs1`,
`in.rs2` and the four `is_div*` flags — and then asserted `op_is_div == in.is_div` while dividing,
and computed the four completion references from `in.rs1`/`in.rs2`.

**The pipeline does not hold `in`. It bubbles it.** The divider stall is low on the cycle a divide
issues, so decode issues normally that cycle; the next instruction's operand-fetch cycle publishes a
bubble; and from the following cycle the stall holds that bubble. `in.is_div` really does go 1 → 0
during a divide, and `in.rs1`/`in.rs2` are zero by the time the completion assertions read them. The
proof was checking an input sequence the hardware never produces, and `test/exec_tb.v` plus the `.S`
suite were carrying that case alone — `exec_tb` drove the same hold.

The assume is **deleted, not replaced by a bubble assume.** The RTL reads `in` only in `init`, and
the restated invariant reads proof-only copies of the operands taken on the edge the divider loads
on, so `in` is now completely free while dividing. That covers the bubble the pipeline really
presents and every other sequence too, and is weaker than either assumption it replaces. The six
assertions tying the latches to `in` are deleted with it; what replaces them is `$onehot` over the
four latched op flags — which the completion arm's magnitude select and its negate term both read as
disjoint, and which is also what the deleted `parallel_case` marking had been spent against.

## What can fail

Every graded comparison added or changed here has a demonstrated red direction, forced and observed
rather than argued:

| mutation | result |
|---|---|
| divider's subtract reads `in`'s divisor instead of the latched one | the twelve new `exec_tb` bubble vectors fail; **every held-input vector still passes**, which is what says the old bench could not have caught it |
| quotient bit taken as the borrow rather than its inverse | `components_executor` FAIL, invariant 1, basecase step 3 |
| the restoring step dropped (remainder always takes the difference) | `components_executor` FAIL, `div_rem < div_divisor`, basecase step 3 |
| REM negates on the divisor's sign instead of the dividend's | `components_executor` UNKNOWN (rc=4) — basecase pass, induction FAIL, on the REM completion assertion, which is the shape that file already records for completion mutations |
| multiply correction's rs2 term dropped | `components_executor` FAIL on the new `rs1 == 1` lemma |
| multiply correction's rs1 term dropped | `components_executor` FAIL on the `rs2 == 1` lemma |

The fourth lemma is why change 3 has two red directions rather than one: at `rs2 == 1` only the rs1
correction can fire and at `rs1 == 1` only the rs2 one, so neither term can hide behind the other.

Asserting the new multiplier against a signed 33x33 reference is **not** attempted: that is exactly
the two-`bvmul` miter this file already records as returning no verdict in two minutes.

## What was verified

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | 9/9, including twelve new divide vectors driven with `in` bubbled after issue |
| `make probe-gates` | clean (via `make test`) |
| `make window-test` | clean (via `make test`) |
| `make cycles` | 32 624 cycles, CPI 2.08, byte-identical to base |
| `make -C formal check` | 85 checks, 85 pass, `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact |
| `components_executor` | successful proof by k-induction |
| `components_decoder` | successful proof by k-induction |
| `components_pcloop` | successful proof by k-induction |
| `components_traps` | successful proof by k-induction |
| `complete_cover` | PASS |
| `nonperturbation` | PASS |
| `make cosim-suite` | 60/62 agreed, `test/COSIM_EXPECTED_FAIL` exact, identical to the same run on base |
| `make lint` | svlint clean in both passes |
| `make elaborate-strict` | clean, no warnings |
| `soc/timing_sweep.sh` | four placements, all above `SOC_MIN_MHZ` |

`components_executor` is the named oracle for everything here. The generated checks run under
`RISCV_FORMAL_ALTOPS`, which replaces both the multiplier and the divider with a stub, so **the
whole of this change is invisible to that leg** and an all-green `make -C formal check` says nothing
about it. `make cosim-suite` is carried for the same reason ADR-0032 keeps it: it reads the core's
real `regs_a` and no `rvfi_*` signal, and rewritten arithmetic is the case it was kept for.

`make -C formal complete` did not return a verdict here and did not on base either: the local
Homebrew yosys reports `ERROR: Command syntax error` on `abc -g AND -fast`, identically on an
unmodified export. The `formal` CI job runs the pinned OSS CAD Suite. Recorded rather than worked
around, exactly as ADR-0088 recorded it.

## Consequences

- **ADR-0088's rule survives contact with three edits that each clear the band.** Its measurements
  were all inside ±50 and only cleared it as a group, which left open whether the rule only ever
  finds small things. It does not: the same question — what fact does the expression not state? —
  found a dead register width, a duplicated negator and a partial-product row worth 404 cells
  between them, more than four times the eleven.
- **This is an area result and must not be quoted as a speed one.** −404 on `fit` and −400 placed
  cells are outside their bands; −3.1% of median period over four seeds is inside its, and is a null
  in both directions.
- **The up5k SoC is at 79% of its logic cells**, from 87% two commits ago. That is headroom for the
  things still deferred, not a route to Fmax.
- **A FORMAL block's assumptions are a claim about the rest of the design, and this one was wrong
  for the whole life of the divider proof.** The general lesson is narrow and worth stating: an
  assume that describes a neighbouring module's protocol has to be re-derived when that protocol is
  described anywhere else, and here `CLAUDE.md` already said in plain words that a divider stall
  *holds* `decoder_out` while every other reason *bubbles* — which is precisely why the cycle a
  divide issues is a bubble and not a hold.
- **The divider's proof-side restriction is 255, not 15, and the number is a solver budget.** Anyone
  widening it further should re-run the table above rather than assume the ceiling moved with the
  toolchain.
