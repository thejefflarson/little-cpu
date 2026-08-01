# ADR-0043: The reference model is configured as *this* core, and what is left over is exempted by name

**Status:** Accepted · 2026-08-01 · *Amends [ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md)
and [ADR-0039](0039-co-simulation-runs-the-whole-suite-against-a-baseline.md) on how the Sail model
is configured. Takes `test/COSIM_EXPECTED_FAIL` to empty. Applies
[ADR-0002](0002-isa-target-rv32imc-zicsr.md)'s ISA target to a fourth artefact.*

## Context

`make cosim-suite` landed at 50/52 with two baselined divergences (ADR-0039). They looked alike in
the table and were completely different in kind.

**`csr.S` — `csrr misa`: Sail `0x4034112f`, core `0x40001104`.** Both MXL=01. The extension bits
were not close:

| | extensions |
|---|---|
| core | **I, M, C** — RV32IMC_Zicsr, ADR-0002 |
| Sail | I, M, C **plus A, B, D, F, S, U, V** |

The core was right. `test/sail/memory-map.json` was a `--config-override` on the model's **default
RV32 machine**, and an override inherits everything it does not mention. Nobody chose atomics,
bit-manipulation, double and single float, supervisor mode, user mode or vectors; they arrived by
default and stayed because nothing in the suite happened to notice.

**`misa` was the symptom, not the problem.** A misconfigured reference model is wrong about
everything the misconfiguration touches. A model that believes S-mode exists has different trap
delegation, different address translation and different CSR legality — and M3's trap work is exactly
what starts to reach those. This is the same failure class as ADR-0033's
`memory.misaligned.exceptions.load_store` finding, which sat undetected until traps landed: a config
knob quietly making the oracle describe a different machine. That one was caught before it produced
a false divergence. This one was already producing one.

**`minstret.S` — `csrr mcycle`: Sail 6, core 30.** Neither value is wrong. `mcycle` counts cycles;
an ISA model has no pipeline. No configuration makes these agree, and one that did would mean this
core retires one instruction per cycle — the opposite of what it is. `minstret.S` already knew this:
its assertion is a deliberate *bound* (`x1 < 16`) so it does not become a change-detector for
pipeline timing. The test is correct. What diverged is that the co-simulation compares the whole
register file, and `csrr mcycle` parks an implementation-defined value in an architectural register
at a comparison point.

## Decision

**Three mechanisms, applied in a fixed order, and baselining is not one of them.**

### 1. A complete `--config`, not a `--config-override`

`test/sail/rv32imc_zicsr.json` replaces `test/sail/memory-map.json`. `test/cosim.py` runs
`sail_riscv_sim --config <file>` and no longer passes `--rv32`.

The reason is not tidiness. **A `--config` file must be complete** — the model's schema check
rejects one with a missing key and names every one it wants (measured: a file containing only
`{"base": {"xlen": 32}}` produces seventeen `Required property ... not found` lines). So nothing is
inherited silently, and a version bump that introduces a knob fails loudly in this repo instead of
picking a default nobody read. That is the opposite of the property that made vectors and supervisor
mode our reference machine, and it is worth ~590 lines of JSON.

Every knob in it traces to a decision that predates the divergence it resolves — ADR-0002 (the ISA
target), ADR-0005 (traps and CSRs), ADR-0008 (the memory map) — and the file says which, at each
knob. **A config override is where you can talk the oracle into agreeing with you**: Sail is
authoritative about RISC-V semantics but not about which implementation-defined choice this core
makes. The rule ADR-0033 wrote for the misalignment key now covers the whole file.

### 2. `NONCOMPARABLE_CSRS` in `test/cosim.py`, for values only

`mcycle` and `mcycleh`. A read of one into a register makes **that one register's value** at
**that one change** not comparable. Which register, where the change falls in the sequence, and
everything computed from it afterwards are compared exactly as before. The exemption and its count
are printed on every run that uses it — `NOT COMPARED BY VALUE (4, see NONCOMPARABLE_CSRS ...)` —
so it cannot quietly become "compared nothing".

Detection is from the **instruction encoding** in Sail's trace, not from its disassembly text:
SYSTEM opcode, a CSR `funct3`, a listed CSR address, and a non-`x0` `rd`. `csrw`-style forms that
write the CSR and leave no register holding its value are not exempted, because there is nothing to
exempt.

The comparison walks two cursors rather than one index, because such a read may produce an
architectural change on one side and not the other: the two sides read different values, and one of
those can be the value the destination register already held, which a distinct-state reduction does
not see. `run_sail` therefore emits the event whether or not it changed a register on the Sail side,
and `compare` consumes the core-side record only if it is a change to exactly that register.

### 3. An assertion the reference model is entitled to fail belongs in a bench with no reference model

This is the rule that did the most work here, and it is not in the ticket that provoked this ADR.
Fixing `misa` unmasked two divergences the old baseline had been hiding behind it — `mip` and `mie`
— and **neither is a value problem.**

- **`mip`.** This core has no timer, so `mip.MTIP` is read-only zero (ADR-0002). The model's mtime
  keeps advancing with `platform.clint.supported: false` — that key unmaps the CLINT's MMIO window
  and nothing more — and raises MTIP against an `mtimecmp` that resets to zero, on the first tick of
  every program (`CSR mip (0x344) <- 0x00000080` under `--trace-reg`). There is no knob: the schema
  gives `platform.clint` exactly `base`, `size` and `supported`, and names neither `mtimecmp` nor
  MTIP anywhere.
- **`mie`.** Which interrupt-enable bits are writable is implementation-defined. ADR-0002 has no
  interrupt sources, so all of `mie` is read-only zero here; the model hardwires MSIE/MTIE/MEIE
  writable (`csrw mie, -1` then `csrr` reads back `0x888`) and the schema names no knob for it.
- **`mtval` writes.** Same shape, found the same way: this core hardwires `mtval` to 0, the model
  makes it writable, and `xtval_nonzero` only controls what a *trap* writes into it.

`csr.S` asserted all three were zero. On the reference machine those assertions are **false**, so
the program does not merely read a different value — it **takes a different branch** and runs to
`fail` where the core runs to `pass`. No value exemption can paper over that, and none should: two
runs of one program taking different paths is precisely what this leg exists to report.

So those assertions came out of `test/asm/csr.S`. They are not dropped — `test/csr_tb.v` already
asserted every one of them directly against `rtl/csrs.v` ("mie reads 0", "mip reads 0", "mie ignores
a write", "mip ignores a write", "mtval ignores a write"), together with `implemented` being high
for all three addresses, which is the half that says a `csrr` of any of them retires rather than
trapping. `make test-units` runs it. `csr.S` keeps `csrr a0, mtval` reading zero, which is true of
both machines, and carries a comment at the site saying where the rest went and why.

**The order matters and is written into `test/COSIM_EXPECTED_FAIL`'s header.** Can the model be
*configured* to be this machine? Is only the *value* incomparable, with both sides still taking the
same branches? Or does the program assert something implementation-defined the model is entitled to
answer differently? Only if none of the three fit is a baseline entry the answer.

## Rationale — measured, not argued

**`csrr misa` agrees.** Both sides now report `0x40001104`; Sail's ISA string is
`rv32imc_zicntr_zicsr_zifencei_zca_zvl32b_zvl64b_zvl128b_zvl256b`.

**`test/COSIM_EXPECTED_FAIL` is empty and the suite is 52/52**, 7.3s wall (against 10s at 50/52 —
the difference is `csr.S` and `minstret.S` no longer running to a `fail` path on one side).

**The audit found more than `misa`.** The cascade ADR-0039 predicted — `Ssccptr`, `Svade`, `Svadu`,
`Svvptc`, the `Zve*`/`Zvfh*` family, `physaddr_bits`, two `mstatus.*_legal_states` — largely
collapsed once every extension was disabled at once rather than one at a time; what the validator
actually demanded was `mstatus.fs_legal_states`/`vs_legal_states` set to `ExtContext_Off` and
`memory.physaddr_bits` reduced from 34 to 32 (34 requires Sv32). Beyond `misa`'s seven bits, the
audit disabled every `Zi*`, `Za*`, `Zf*`/`*inx`, `Zc*` beyond `Zca`, `Zb*`/`Zk*`, `Zv*` and
`Ss*`/`Sm*`/`Sv*`/`St*` entry; set `writable_misa`, `writable_fiom`, the three counter-enable masks,
`medeleg`/`mideleg` and every `xtval_nonzero` field to their read-only-zero forms; turned off
vectored `mtvec`, PMP (`count: 0`) and the CLINT and interrupt generator; and set `wfi_is_nop`.

**Three extensions are enabled for a reason rather than because the target names them, and each was
measured:**

- **`Zca`** is C for this target. `misa.C` comes from `Zca` alone once F and D are off — `Zcf` and
  `Zcd` would otherwise be required too — and that is what brings `misa` to `0x4000_1104`.
- **`Zicntr` is what makes `mcycle`/`minstret` exist at all.** With it off, `csrr a0, minstret`
  raises an illegal-instruction trap in the model: `minstret.S` reported `SUCCESS` while every
  `csrr` vectored to the handler (its fatal handler runs with `TESTNUM` still 0 and writes the
  riscv-tests *pass* encoding — a separate trap for a future reader), and `trap.S` failed at test 19
  with two spurious handler entries. This core implements the machine counters and riscv-formal's
  `[csrs] mcycle minstret` requires them, so it is on. **Recorded residual:** `Zicntr` also gives
  the model the unprivileged shadows `cycle`/`time`/`instret` (0xC00–0xC02), which this core does
  not implement and would trap on. No program in `test/asm` reads them and there is no knob
  separating the machine counters from the shadows.
- **`Zifencei`**, because `rtl/decoder.v` decodes `fence.i` and retires it as a NOP (ADR-0005). With
  it off the model would raise an illegal instruction where this core does not.

**`platform.instructions_per_tick` was tried as a way to silence `mip.MTIP` and is rejected.**
Stretching the tick period past the run does keep mtime — and therefore MTIP — at zero. It also
freezes `mcycle`: in sail-riscv 0.13.1 `mcycle` **is** that tick counter (with the default of 2, a
program 13 instructions in reads `mcycle == 6`). At `1_000_000` the model failed `minstret.S` test
3, which asserts only that `mcycle` advances between two reads — turning a real property into a
false divergence. The knob stays at the model's default and `mip` is handled by mechanism 3 instead.
This is the general shape of the trap: a knob that makes one divergence disappear is not thereby the
right knob.

**The gate still fails in both directions with an empty baseline**, which is ADR-0035's contract and
the thing driving a baseline to empty is most likely to break:

- *Unexpected divergence.* ADR-0032's mutation B (`regs[31] <= wdata` alongside `regs[waddr]`, gated
  to fire after cycle 40; reverted before commit) gives `3/52 agreed` and `make: *** [cosim-suite]
  Error 1`. `make test` reports `52/52 passed` against the same mutation, reproducing ADR-0039's
  table exactly.
- *Unexpected agreement.* A single fabricated line `add.S DISAGREE AT 3` added to the empty baseline
  gives `52/52 agreed` followed by `Divergence list does NOT match test/COSIM_EXPECTED_FAIL` and
  exit 1. Driving the baseline to empty did not disable the direction that catches a program
  agreeing when it was not supposed to.

**`make test` still works with no Sail installed** (ADR-0032's standing constraint): with
`tools/sail` moved aside, `make test` is `52/52 passed` and `./test/cosim.py add.S` exits 3 with the
message naming `make sail-setup`.

`make test-units` passes (six benches), `make lint` is clean in both passes, and the formal ladder is
untouched — no `rtl/` file changed (`git diff origin/main -- rtl/` is empty).

## Consequences

- **`test/sail/memory-map.json` is gone.** ADR-0032's description of it and ADR-0039's reference to
  it are historical; `test/sail/rv32imc_zicsr.json` is the file. `docs/THREAT_MODEL.md`'s
  false-positive note about the executable memory region is repointed at it — the note itself is
  unchanged and still correct.
- **The config is now a thing a Sail version bump can break loudly**, which is the point and also
  the cost. `--config` demands completeness, so a release that adds an extension or a platform knob
  will fail the schema check until this file names it. Regenerate the skeleton with
  `sail_riscv_sim --rv32 --print-default-config`, re-apply the decisions (they are individually
  commented), and re-run the suite. This is strictly better than the old failure mode, where the
  same release would have silently enabled something.
- **`test/asm/csr.S` no longer has a test 16**, and the numbering has a hole there on purpose.
  `TESTNUM` values are labels for failure reporting, not an ordering; renumbering 17–20 would churn
  a file for nothing and lose the correspondence with the git history that explains the gap.
- **`NONCOMPARABLE_CSRS` is a list that can only shrink safely.** Adding an entry is two claims, not
  one: that no configuration can close the gap, *and* that every program reading the CSR still takes
  the same branches on both machines. The second is the one that gets forgotten; `mie` and `mip` are
  the worked example of it failing, and the dict's own comment says so.
- **`mcycleh` is exempted while it currently agrees.** It reads zero on both sides for every program
  in this suite today. It is on the list so that stops being load-bearing, at the cost of one
  register's value at one change per read — an accepted, stated weakening rather than an accident
  waiting for a program long enough to roll the counter.
- **This changes nothing about what the leg is for or where it runs.** ADR-0032's opt-in decision
  and ADR-0039's §3 both stand: `make test`, `make test-units` and CI's required set reach none of
  it, and adding it to branch protection still needs a new ADR. `test/cosim.cc` still reads no
  `rvfi_*` signal and must not start.
- **What this does *not* establish.** The co-simulation leg still inherits the `.S` suite's coverage
  exactly (ADR-0032). A correctly configured reference model makes the programs already being run
  check more per run; it does not make them cover more. The residual differences named above —
  `mie`, `mip`, `mtval` writability, `mcycle`'s rate, the unprivileged counter shadows — are
  differences between two legal machines, and this leg is now silent about all of them by
  construction. Every one of them is checked in `test/csr_tb.v` instead, against assertions this
  repo wrote rather than against an oracle, which is exactly the caveat ADR-0037 attaches to
  everything M3 added.
