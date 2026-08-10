# ADR-0092: the writeback slot costs more than the bypass it replaces, and the two loops do not pay together

**Status:** Accepted · 2026-08-09 · *Built, measured over eight placements a side, and DECLINED.
19.5% of suite cycles and 8.6% of Dhrystone's for a period inside the churn band — a product 5.1%
worse on the number this project quotes. The pairing claim it was filed on is measured here and does
not hold.*

## Context

There are two loops of nearly the same length around the fetch address, and the second one was hidden
under the first:

```
loop 1  ROM RDATA -> instr -> rs1/uses_rs1 -> scoreboard -> stall -> next_pc -> ROM address
loop 2  accessor_out.rd_data -> writeback -> regfile write-through bypass
                             -> branch comparator -> next_pc -> ROM address
```

Loop 2 exists because `rtl/regfile.v` forwards `wdata` at its output, so an operand a branch compares
can be produced by the load in writeback this very cycle. That bypass is what makes the decode
scoreboard's three slots enough: the fourth place an in-flight `rd` can be — writeback — is covered
by forwarding rather than by stalling. It selects from the *held* address pair rather than the
presented one, which is [ADR-0064](0064-the-write-through-bypass-is-addressed-from-the-held-pair.md)
as amended by [ADR-0089](0089-the-operand-fetch-guess-ships-and-one-workload-feels-it.md): since the
operand-fetch guess landed, the pair presented on an issuing cycle is deliberately a *different*
instruction's, and what makes the bypass right is that nothing issues until the held pair is the pair
the issuing instruction reads.

The proposal: delete the bypass and cover writeback with a fourth scoreboard slot on
`accessor_out.rd`. Loop 2 then starts at a register instead of at the accessor's load data. The
stated cost is one cycle per RAW that currently resolves at writeback.

This amendment and the read-enable one in
[ADR-0091](0091-the-stall-costs-the-same-at-a-memory-pin.md) were filed as one ticket on the claim
that the loops are balanced, so neither pays alone. **Both were built; all three configurations were
measured; the claim is answered below.**

## What was built

- `rtl/regfile.v`: `reg_rs1 = (held_rs1 == 0) ? 0 : read_a`, and the same for rs2. The write-first
  term into the read register stays and becomes the only forwarding in the file.
- `rtl/decoder.v`: a fourth `live_rsN` term, `accessor_out_valid && accessor_out_rd == rsN`.
  `rtl/writeback.v` is combinational on the accessor's output, so that pair *is* `wen`/`waddr` at the
  register file: the slot is high exactly on the cycles the write is committing.
- One stalled cycle is enough because the write-first term captures the value on the edge that ends
  it, and because a stalled cycle presents the instruction's own pair rather than the guess.
  `test/regfile_tb.v` now asserts the absence of output forwarding and that the next fetch/use pair
  has the value; `test/decoder_tb.v` covers the slot, its x0 exclusion and its valid gate.

`formal/pcloop.sv`'s over-approximated stall list takes the fourth term too, or its increment
assertion fails on a cycle the pc is now allowed to hold.

## The measurement

`soc/timing_sweep.sh`, up5k, eight seeds a side, same toolchain and sitting. Base is `c2fa29b`.

| | sorted, ns | worst | median | best |
|---|---|---|---|---|
| base `c2fa29b` | 76.61 77.51 78.61 78.64 78.71 79.25 79.29 79.30 | 79.30 (12.61 MHz) | 78.68 | 76.61 |
| fourth slot | 73.50 75.06 76.08 76.22 76.65 77.05 77.96 78.14 | 78.14 (12.80 MHz) | **76.44** | 73.50 |
| both amendments | 75.90 76.72 77.00 77.28 77.71 78.15 78.67 80.15 | 80.15 (12.48 MHz) | 77.50 | 75.90 |

**−2.8% at the median for the fourth slot, −1.5% for the pair**, against a ~3.6% edit-churn band and
a 1–2% placement spread. The whole distribution moves the same way — the slot's worst placement beats
base's median — so it is worth saying plainly that this is the *favourable* half of the pair on the
period. It is still inside the band, and the band is what this repository has decided such a number
means. 12 MHz holds at all 24 of these placements.

`make fit` 3496 → 3418 (−78) for the fourth slot and 3554 (+58) for the pair; the placed SoC 4175 →
4280 (+105) and 4253 (+78). **The two instruments disagree in sign on the fourth slot**, which is
what they are entitled to do — they are different designs, and both numbers are near their bands.
Area is not the story here in either direction.

### Where the work moves

`soc/depth/path_stages.py` on the default placement, with its own caveat in force — it charges a
level to the module owning the state that level newly folds in, and every decode LUT folds in more
instruction bits, which *are* `rom_*_RDATA`, so `imem` is an upper bound and not a measurement:

| | levels (icetime) | ends at | charged |
|---|---|---|---|
| base | 25 | `imem.rom_even.0.4_RDATA_1` | decode 13, imem 4, csrs 2, access 1 |
| fourth slot | 23 | `imem.rom_even.0.0_RDATA_1` | **decode 11**, imem 4, access 3, csrs 2 |
| both | 20 | `imem.bank_re` | imem 7, decode 4, access 1 |

**Loop 1 is critical before and after.** The fourth slot takes the accessor's load data out of loop 2
and puts two more 5-bit comparators into `hazard`, which is loop 1; the reported path stays in loop 1
and the decode bucket stays the largest thing in it. That is the balance the ticket described, read
from the inside: two loops within 2 ns of each other means work moved out of one arrives in the other
at whatever it costs there, and the visible period barely moves.

## Cycles, on both workloads

`make cycles` (the `.S` and `.c` suite) and `make dhrystone`, which disagree by nearly 2× on stall
accounting and so must both be quoted:

| | suite cycles | CPI | Dhrystone cycles | CPI | DMIPS/MHz |
|---|---|---|---|---|---|
| base `c2fa29b` | 29 454 | 1.881 | 2 178 739 | 2.292 | 0.535 |
| fourth slot | 35 195 (**+19.5%**) | 2.248 | 2 365 790 (**+8.6%**) | 2.489 | **0.493** |
| both | 35 256 (+19.7%) | 2.252 | 2 365 775 (+8.6%) | 2.489 | 0.493 |

Retires are identical on both workloads, so this is cycles and nothing else. The suite's scoreboard
column goes from 10 027 cycles to 15 895 and Dhrystone's from 358 928 to 549 151; the operand-fetch
columns barely move (1650 → 1573 and 641 153 → 637 981), so this is new stalling and not
re-attribution. `make waves`'s 200-cycle iverilog run drops from 79 retires to 66 against its floor
of 60.

**The cost is larger here than it was one commit ago.** On `a8d6f46` the same amendment cost 15.8% of
suite cycles; since ADR-0089 removed most of the operand-fetch column, fewer of these RAW cycles were
already being paid for another reason, and the slot is charged for all of them.

### Throughput, as a product

At the median placement of each distribution:

| | period | clock | DMIPS/MHz | **DMIPS** | suite retires/s |
|---|---|---|---|---|---|
| base | 78.68 ns | 12.71 MHz | 0.535 | **6.80** | 6.76 M |
| fourth slot | 76.44 ns | 13.08 MHz | 0.493 | **6.45 (−5.1%)** | 5.82 M (−13.9%) |
| both | 77.50 ns | 12.90 MHz | 0.493 | **6.36 (−6.4%)** | 5.73 M (−15.2%) |

**The clock this buys does not pay for the cycles it costs**, on either workload, and the gap is
larger than the instrument's whole spread. That is decidable without a fourth distribution: 8.6% of
Dhrystone cycles against a ~3.6% churn band means the period would have to move further than this
instrument can resolve before the product turned around.

## The pairing claim

The ticket's case was that neither amendment is worth landing alone because each attacks one of two
balanced loops, so only the pair can move the period. Measured on `c2fa29b`, at the median of eight
placements each:

- the read-enable half alone: **−0.1%** of period, +0.2% of suite cycles — a null in both directions;
- the fourth slot alone: **−2.8%** of period, **+19.5%** of suite cycles;
- the two together: **−1.5%** of period, +19.7% of suite cycles.

**The pair is not better than either half.** It is worse on the period than the fourth slot alone and
worse on cycles than both. Nothing here is evidence that the loops must be attacked together; what
the measurement shows is that the cheap half buys nothing and the expensive half does not buy enough.
The pairing claim was a reason to measure all three, and measuring all three is what answers it.

## The standing probe was run, in both directions

`CLAUDE.md` names one: delete the rs2 write-through bypass and `reg_ch0` must go SAT. On a clean
export of `c2fa29b`, with only that deletion, `btormc` reports **bad state property 1 reachable at
bound k = 22, SATISFIABLE** — a real counterexample at the configured depth, and the same verdict the
probe gives on `a8d6f46`. The probe is alive on today's main, and for the right reason: it fires on
exactly the cycle the bypass covers. (sby reports `ERROR 16` around it because `btorsim` is not
installed here to render the trace; the solver's verdict is the result, not the exit code.)

With the fourth slot present and the bypass gone, `reg_ch0` passes at the same depth. So the slot
carries the property the bypass carried — the amendment is correct, and it is declined on cost.

**The probe as written stops applying to a design with no bypass to delete.** Its replacement would
be the same shape: delete the fourth slot's rs2 term and `reg_ch0` must go SAT. Nothing in this
repository needs that today, because the bypass stays.

## F and G were re-measured from scratch

Widening the scoreboard is one of the three things `CLAUDE.md` says must re-measure the two figures
the BMC depth table is built on, so they were re-measured on the built pair rather than argued about,
by the procedure `formal/checks.cfg` documents — a copy of that file with one line in `[depth]`, run
through `genchecks-local.py`, and the last column swept:

- **F = 6.** `hang` red at 5 and 6 (`bad state property 0 reachable at bound k = 6`), PASS at 7 and 8.
  The check asserts on a registered flag, so the flip point is F + 1.
- **G = 6.** `liveness` red at gap 4 and gap 5, PASS at gap 6 and 7 at trig 10; red at gap 5 and PASS
  at gap 6 again at trig 15.

Both reproduce the recorded numbers exactly, so the depth table would not have moved — `insn 19` and
`reg 15 22` keep the one cycle of margin they have over F + 2G = 18. That is worth knowing for the
next attempt in this direction: **an extra stalled cycle on a RAW does not lengthen the worst-case
retire gap**, because the worst gap is already set by something longer.

## What was verified

On the built pair, before it was reverted:

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | 9/9, with `test/regfile_tb.v` rewritten around the removed bypass |
| `make probe-gates`, `make window-test` | clean (via `make test`) |
| `make waves` (iverilog) | 66 retires against a floor of 60 |
| `make cycles` | 35 256 cycles, CPI 2.25 |
| `make -C formal check` | 85 checks, 85 pass, `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact |
| `components_decoder` / `components_executor` / `components_pcloop` / `components_traps` | successful proof by k-induction, all four |
| `complete_cover` | PASS |
| `nonperturbation` | PASS |
| `make cosim-suite` | 60/62 agreed, `test/COSIM_EXPECTED_FAIL` exact, identical to the same run on `c2fa29b` |
| `make lint` | svlint clean in both passes |
| `make elaborate-strict` | clean, no warnings |
| `make fit` | 3554 against `FIT_MAX_LC` 3700 |

`make -C formal complete` did not return a verdict here and does not on an unmodified export of
`c2fa29b` either: the local Homebrew yosys reports `ERROR: Command syntax error` on `abc -g AND
-fast`. The `formal` CI job runs the pinned OSS CAD Suite.

## Consequences

- **The write-through bypass stays, and the reason is now measured rather than assumed.** It is worth
  19.5% of suite cycles and 8.6% of Dhrystone's, against a fourth scoreboard slot that returns a
  period inside the churn band. The commitment it belongs to — the regfile answer belonging to the
  pair the issuing instruction reads, via two fabric forwarding points — is unchanged, and this is
  the evidence for the second of those two points rather than an amendment to it.
- **Work moved out of loop 2 lands in loop 1 at full price.** Two loops within 2 ns is a statement
  about both, and a structure that shortens one by lengthening the other returns a null. Check which
  loop a candidate's logic *joins*, not only which one it leaves.
- **A CPI cost larger than the instrument's spread does not need a fourth distribution.** The sweep
  was run anyway because the claim under test was about the period, and it agrees.
- **This amendment got more expensive when ADR-0089 landed, not less.** Removing the operand-fetch
  cycle removed the thing that was already paying for part of these stalls. A CPI cost measured
  against an older tree is as perishable as a period ceiling.
- **Neither amendment is landing, and the pair is not a route to 24 MHz.** With ADR-0091's ceiling not
  reproducing on either base, the fetch-loop direction ADR-0087 ranked has no unpriced candidate left
  in it that this repository has identified.
