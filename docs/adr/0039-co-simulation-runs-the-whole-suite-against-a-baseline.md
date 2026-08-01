# ADR-0039: Co-simulation runs the whole suite against a baseline, and `tohost` becomes a doubleword

**Status:** Accepted · 2026-07-31 · *Completes items (1) and (2) of ADR-0032's integration list.
Amends ADR-0008 on the width of `tohost`. Applies ADR-0014's set-equality contract and ADR-0035's
name-and-status format to a second gate.*

## Context

ADR-0032 built the co-simulation harness as a time-boxed spike and left suite-wide integration as
future work. Two things have changed since, and both make finishing it worth doing now.

**It is the only oracle in this repo that reads the real register array.** `test/monitor.v` and every
`insn_*` check on the riscv-formal ladder evaluate a spec model on `rvfi_insn` / `rvfi_rs1_rdata` /
`rvfi_rs2_rdata` and compare the result against `rvfi_rd_wdata` — the core's own account of itself.
`reg_ch0` is the single check that ties that account back to `rtl/regfile.v`, and it is one `mode
bmc` query at depth 21 (ADR-0024, ADR-0025), which ADR-0023 records as inconclusive. ADR-0032's
mutation experiment measured the hole exactly: an extra architectural write outside the retiring
instruction's `rd`, gated to fire after cycle 40, was missed by the whole `.S` suite with the
per-retire monitor live **and** by the entire 78-check ladder, and caught by co-simulation in 0.6s.

**The regfile is about to move to block RAM**, and that change cannot be gated on anything weaker.
`make cosim-run` takes one program at a time and has no baseline, so today it cannot be a gate at
all.

There is a second, independent reason, measured while planning this: deleting the rs2 write-through
bypass — a direct violation of CLAUDE.md invariant 6 — **passes `reg_ch0`**, while the `.S` suite
catches it instantly (52 × `MONITOR-ERROR`). The formal ladder is not the oracle for that property.
The simulation legs are, and this is one of them.

## Decision

**A per-program suite runner, graded by set equality in both directions against a baseline — and
`tohost` widened to the doubleword the protocol it borrows always specified.**

### 1. `tohost` is an 8-byte-aligned `.dword`, and the verdict macros write both halves

`test/asm/riscv_test.h`'s `RVTEST_DATA_BEGIN` emitted `tohost` as a 32-bit `.word`. ADR-0008 places
it at the base of RAM with `.data` immediately after, so every load/store test's `TEST_DATA` began
four bytes *inside* the doubleword IO window any HTIF consumer claims at the symbol. Sail answered
every `lw` from `RAM_BASE+4` with zero and eight programs "diverged" for a reason that was entirely
this repo's fault. The spike worked around it by stripping the symbol from a throwaway ELF copy.

Both workarounds are now gone. `tohost` is `.align 3` + `.dword 0`; the IO window covers only
`tohost` itself; `test/cosim.py` hands the reference model **the same ELF the other two legs run**,
unmodified.

`RVTEST_PASS` and `RVTEST_FAIL` write the upper word (always zero) first and the verdict last.
Sail's HTIF fires on a half-write once it has seen the other half, so this order makes the verdict
store the single event that terminates the reference model — the same store `test/cxxrtl.cc` and
`test/cosim.cc` already stop on when RAM's low word goes non-zero. Both sides end on the same
instruction. Neither store writes a register, so the sequence of architectural register-file states
is unchanged by the pair.

**`--inst-limit` therefore goes back to being a runaway bound.** The spike used it as a convergence
*criterion*, inferring "the reference reached a verdict" from its last eight traced PCs being a
self-loop. Now `SUCCESS` / `FAILURE: <n>` on Sail's own output is the criterion, and a run that hits
the limit without one is `INCONCLUSIVE` and is not compared. There is deliberately no fallback to
Sail's process exit status: it is 0 both for `SUCCESS` and for hitting `--inst-limit`, so reading it
would silently turn "never reached a verdict" into "the program passed".

The verdicts themselves are compared as well, so a run in which the two ran the program to different
outcomes is a divergence even in the case — unreachable if the register comparison is working — that
the register traces matched.

### 2. `test/run_cosim.sh` + `test/COSIM_EXPECTED_FAIL`, driven by `make cosim-suite`

Shaped after `test/run_tests.sh`, and for the same reason: the failure mode that matters in a gate is
a **false green**. The properties that exist only for that are the baseline being read and
format-checked before the suite runs, the program list being checked for emptiness (a glob matching
nothing would otherwise print `0/0 agreed` and match an empty baseline), and the per-program status
coming from `test/cosim.py`'s own `COSIM-STATUS` line cross-checked against its exit code — a run
that produced no status line is `COSIM-ERROR`, not a verdict about the core.

The baseline is name-and-status per ADR-0035, and the statuses pin **how** the two disagreed:
`DISAGREE AT <n>` (the index of the first differing architectural change), `DISAGREE LENGTH`,
`DISAGREE VERDICT`, `INCONCLUSIVE SAIL-LIMIT`, `INCONCLUSIVE CORE-TIMEOUT`. A baselined program that
starts diverging at a different point is diverging for a new reason and goes red.

### 3. It stays opt-in — that part of ADR-0032 is not amended

`make test`, `make test-units` and CI's required set reach none of it, and `make test` keeps working
on a machine with no Sail installed. **Adding this to branch protection's required set is what
ADR-0032 forbids**, and this ADR does not do it. The regfile change gates on co-simulation by
carrying its pre/post output in the pull request, reviewed by a human — which is why criterion 5
below is not decoration.

## Rationale — measured, not argued

**The suite is green and the baseline has two entries.** 52 programs, 50 agree, 10s wall (against
7.3s for `make test`; the spike measured 24s, and the difference is HTIF termination replacing a
run to the instruction limit).

Both baselined entries are read-only CSR **value** divergences that no change to this core could
close:

- **`csr.S` — `DISAGREE AT 17`, `csrr a0, misa`.** Sail reports `0x4034112f` against this core's
  `0x40001104` (ADR-0002). `misa` is not a configuration knob in the Sail model; it is derived from
  the extension set. Bringing the two into agreement was attempted and measured: it needs A, B, D,
  F, S, U and V disabled, and each disable pulls in a further cascade the model's own validator
  demands — `Ssccptr`, `Svade`, `Svadu`, `Svvptc`, then the `Zve*`/`Zvfh*` family, then
  `memory.physaddr_bits` and two `base.mstatus.*_legal_states` keys. That is a large,
  version-fragile config edit to make one CSR read match, on a schema ADR-0032 already flags as
  something a version bump moves. Recorded rather than chased.
- **`minstret.S` — `DISAGREE AT 7`, `csrr a0, mcycle`.** `mcycle` counts cycles and the reference
  model has no pipeline, so it counts something else by construction (6 against this core's 30 at
  the same point). The rate is implementation-defined; the test asserts only monotonicity and a
  bound, which is why it passes on both sides. No configuration makes a cycle-accurate reference
  model out of an ISA model.

**`trap.S` agrees, 214/214.** ADR-0030's causes against the reference model's own trap semantics,
with `test/sail/memory-map.json`'s global `memory.misaligned.exceptions.load_store` set to
`{"Some": "AlignmentException"}` to match ADR-0005's causes 4 and 6. No divergence in this suite is
attributable to the memory-map configuration.

**A gate that compares nothing is worse than no gate**, so the mutation was re-run against the
integrated leg rather than inherited from ADR-0032. One extra architectural write (`regs[31] <=
wdata` alongside `regs[waddr]`, gated to fire only after cycle 40 — ADR-0032's mutation B, reverted
before commit):

| Leg | Result under the mutation |
|---|---|
| `make test` — 52 `.S` under cxxrtl, per-retire RVFI monitor live in both sim legs | **52/52 PASS — misses it** |
| `make cosim-suite` | **3/52 agreed — catches it in 49 programs**, and the gate exits 1 |

The three that still agree (`jal.S`, `simple.S`, `straddle.S`) are the three too short to reach
cycle 40, which is the mutation's own gate rather than a hole in the leg. `add.S` reproduces
ADR-0032's measurement exactly:

```
DIVERGENCE at architectural change #18
  sail instruction #27  pc=0x0000004a  add x14, x1, x2
  sail : x14=0x80000000
  core : x14=0x80000000 x31=0x80000000   (cycle 44, decode pc=0x00000054)
```

Note also that `csr.S`'s status moved from `DISAGREE AT 17` to `DISAGREE AT 11` under the mutation.
That is the second field of the baseline doing its job: a baselined program that starts diverging
earlier does not launder a new defect into a match.

## Consequences

- **`test/asm/riscv_test.h` is shared infrastructure and this changed it.** Every program's `.data`
  moves four bytes and every program executes one extra store. `make test` (52/52) and
  `make test-units` (six benches) were re-run green in the same change. `test/cxxrtl.cc`,
  `test/cosim.cc` and `test/testbench.v` are untouched: `tohost` is still the first word at
  `RAM_BASE`, and the upper word stays zero, so reading the low word alone remains a faithful read
  of the riscv-tests encoding on a little-endian machine.
- **This leg inherits the `.S` suite's coverage exactly**, as ADR-0032 already said. It says nothing
  about instructions or operand patterns those 52 programs never reach — including the real
  multiplier and divider on untested operands (ADR-0010). It makes the programs already being run
  check vastly more per run; it is not a proof and does not become one.
- **Nothing runs it by default, so nothing notices if nobody runs it.** That is the accepted cost of
  staying off CI's required set, and it is why the regfile change carries pasted output rather than
  a green check. A nightly job — ADR-0032's integration item (3), with the formal nightly as
  precedent — is still unbuilt, and ADR-0037's warning applies to it in advance: never put the
  graded command in a pipeline in a `run:` block.
- **`test/cosim.py`'s exit codes changed**: 0 agree, 1 disagree, **2 inconclusive**, 3 setup error.
  Inconclusive used to share 3 with a broken setup, which is exactly the conflation ADR-0035 spent
  a whole decision separating. `make cosim-run` is otherwise unchanged.
- **`test/cosim.cc` still reads no `rvfi_*` signal**, and must not start. ADR-0032's consequence
  stands unamended: the moment it samples one to align its trace it stops being independent of the
  oracle it exists to cross-check, and the mutation table above stops holding. Nothing in this
  change goes near it — `test/cosim.cc:183`'s `"uut regfile regs"` bind and `:191`'s `depth == 32`
  assertion are deliberately left as they are for the regfile change to repoint.
- **ADR-0032's integration list is now items (3) and (4).** The nightly, and extending the
  comparison to memory (`--dump-memory` on one side, the RAM array already in hand on the other),
  which would cover stores — checked today only by `dmemcheck`, bounded, and by whatever the test
  program asserts.
