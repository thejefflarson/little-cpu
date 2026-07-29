# ADR-0014: `test/EXPECTED_FAIL` is the M1 regression baseline

**Status:** Accepted · 2026-07-28 · *Supplements ADR-0007 and ADR-0008*

## Context

`9cd0c67` landed the ADR-0007 cxxrtl runner, `test/asm/riscv_test.h`, ADR-0008's two-region memory
map, and a working `make test`. The harness is correct to spec. The core is not: `rtl/regfile.v` is
a registered-read register file with no write-through bypass, which is a direct violation of
CLAUDE.md invariant 6 and of ADR-0004. Consequently **all 46 tests time out**, `simple.S` included.

That left a choice about what `make test` should mean on the day it first exists:

1. Hold the harness until the core can pass something. Rejected — it strands a correct, reviewed
   harness behind an unrelated RTL fix, and leaves M1 with no way to measure progress.
2. Bend ADR-0008's `tohost` protocol into something the broken core can drive (an `x0`-only
   encoding). Rejected — ADR-0008 is Accepted and settled, the workaround does not actually work
   (the *value* half of a store has no `x0` escape hatch), and it would have to be ripped out the
   moment the real fix lands.
3. Ship the harness to spec and record the current failure set as a baseline the gate enforces.

We took (3). `test/EXPECTED_FAIL` lists the tests that are *known not to pass*, and
`test/run_tests.sh` exits 0 only when the actual failure set matches that file **exactly** — in both
directions. A test that starts failing is caught; a test that starts passing is also caught, because
the baseline is a set equality, not a ceiling.

The obvious hazard is that a baseline in which *everything* fails cannot self-validate. A harness
structurally incapable of ever reporting a pass produces a byte-identical table. That has to be
ruled out by something other than the baseline itself.

## Decision

**`test/EXPECTED_FAIL` is a regression gate with a burn-down contract, and it is only meaningful
alongside a positive control.**

### The burn-down contract

- Lines are removed **one at a time, by the change that makes that test pass**, in the same commit
  as the fix. The deletion is the evidence the fix worked.
- The file is **never regenerated from a run**. Rewriting it to match observed output launders a
  regression into the baseline and destroys the only thing it is for. Edit it by hand.
- Removing a line is a claim. Adding one back is a **regression** and needs to be justified in the
  PR that does it, not absorbed silently.
- The file is expected to shrink to empty by M1. It is a debt ledger, not a permanent fixture; when
  it is empty, `make test` becomes a plain all-pass gate and the file stays (empty) so that the
  set-equality check keeps working.

### The positive-control requirement

**An all-fail baseline is not accepted on its own.** Whenever `test/EXPECTED_FAIL` lists every test
in the suite, the reviewer must independently demonstrate that the harness *can* report a pass.
Verified for this baseline, and the form future checks should take:

- **Runner exit ladder, all four paths**, by seeding `tohost` directly in the `--ram` image:
  `1` → `PASS`/exit 0; `9` → `FAIL 4`/exit 1 (confirming the `testnum = tohost >> 1` decode);
  `0` → `TIMEOUT`/exit 2; missing argument → usage/exit 3.
- **A pass driven through the RTL**, not merely seeded — assemble, `objcopy`, load, run, `tohost`
  write observed, exit 0. Today this requires a program written around the operand-skew bug (a
  store repeated three times, after enough NOPs for writeback to land, exploiting the fact that the
  address port recovers after one repeat and the store-data port after two). Such a program is a
  **diagnostic and is deliberately not checked in**: it passes *because* of the bug and would break
  when the bug is fixed.
- **Mutation of the gate itself**: delete a line from `test/EXPECTED_FAIL` and confirm the run exits
  non-zero with an expected-vs-actual diff; flip an expectation in a unit bench and confirm
  `make test-units` fails.

The reason for the second bullet is specific. The skew is **not uniform across operand ports** —
`rs1` and `rs2` recover at different depths — so a plausible-sounding one-cycle fix in the decoder
would correct addresses while leaving store data wrong, turning a loud all-timeout baseline into a
quiet wrong-results one. A baseline that only ever observes `TIMEOUT` cannot distinguish those two
worlds. The positive control is what makes the all-fail table mean "the core is broken" rather than
"the harness is blind".

### The `-march` deviation

`test/run_tests.sh` assembles with **`-march=rv32im_zicsr`**, not the project's stated
`rv32imc_zicsr` target (ADR-0002).

This is correct for now. ADR-0003's dual-word combinational fetch window is not implemented, so the
core cannot fetch a 16-bit instruction. Assembling with `c` would let the toolchain emit compressed
encodings the core cannot execute, and every test would fail for a reason unrelated to what it is
checking — noise that would swamp the baseline exactly when it is least able to absorb it.

It is recorded here because it was otherwise **silent**. Nothing in the suite announces that the C
extension is never exercised, and ADR-0002 is explicit that C is a product constraint on the up5k
rather than a preference, so a suite that quietly proves RV32IM forever is a real gap.

**Sunset condition:** flip `test/run_tests.sh` to `-march=rv32imc_zicsr` when ADR-0003's fetch
window lands. Expect a batch of `EXPECTED_FAIL` churn at that point — compressed encodings appearing
throughout the suite for the first time is new coverage, and failures it exposes are discoveries,
not regressions. That is the one sanctioned exception to the one-line-at-a-time rule, and it should
be its own commit, doing nothing else.

## Consequences

- `make test` is a working regression gate from day one, against a core that passes nothing. M1
  progress is measurable as lines leaving `test/EXPECTED_FAIL`.
- The gate catches unexpected passes as well as unexpected failures, so a fix cannot land without
  its baseline update, and an accidental improvement cannot pass unnoticed.
- Reviewers carry an obligation the file itself cannot enforce: while the baseline is all-fail,
  demanding a positive control. This ADR is what that obligation is written in.
- The suite proves RV32IM, not RV32IMC, until ADR-0003 lands. Anyone reading a green `make test`
  before then should not conclude the C extension works — it is not tested at all.
- `test/EXPECTED_FAIL` becoming empty is the M1 exit criterion in file form.
