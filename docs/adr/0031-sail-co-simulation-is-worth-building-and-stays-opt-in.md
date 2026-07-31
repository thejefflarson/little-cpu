# ADR-0031: Sail co-simulation is worth building, and stays opt-in

**Status:** Accepted · 2026-07-30 · *Replaces the "Spike or Sail co-simulation" deferred item in
[`docs/adr/README.md`](README.md)*

## Context

The deferred list said "riscv-formal is the oracle. Revisit only if formal and simulation ever
disagree." That test is the wrong one, because it can only fire on a bug **both** legs can see. The
question worth asking is what neither leg can see at all.

Both existing oracles read the core's **self-report**. `test/monitor.v` and every `insn_*` check on
the riscv-formal ladder evaluate a spec model on `rvfi_insn` / `rvfi_rs1_rdata` / `rvfi_rs2_rdata`
and compare the result to `rvfi_rd_wdata`. Nothing in that loop touches `rtl/regfile.v`. A core that
mis-reports a value and computes with that same mis-reported value tells an internally consistent
story and satisfies both. `reg_ch0` is the one check that ties the report back to the register file,
and it is a single `mode bmc` query at depth 21 (`formal/checks/reg_ch0.sby`, ADR-0024, ADR-0025).

There is also a documented history of hand-maintained models being wrong in ways that cost real
time: ADR-0019's `monitor_insn_div`/`monitor_insn_rem` signedness defect, ADR-0024's hardcoded
engine, ADR-0023's inconclusive `reg`. Every one of those was found by checking rather than
assuming. RISC-V International's Sail model is the spec's executable definition and is maintained by
the people who write the spec, which is a different and better failure mode than a generated
Verilog monitor.

This ADR records a **time-boxed spike**, not an integration: build the thing, get a verdict on one
program, try to break it, and decide.

## Decision

**Build the co-simulation harness, keep it entirely off `make test` and off CI, and record the
integration work as a follow-up rather than doing it here.**

Three files and three Makefile targets, none of them on any existing path:

- `test/sail/memory-map.json` — a `--config-override` placing Sail's MainMemory at `0x0` so
  ADR-0008's two-region link map is executable and loadable, and restating the CLINT window the
  model's config validator demands.
- `test/cosim.cc` — a **second** cxxrtl runner beside `test/cxxrtl.cc`, sharing the same
  `test/rtl.cc`. It reads `uut regfile regs` through `debug_items` and **no `rvfi_*` signal at
  all**, printing a `CS ` line every time the real 32×32 register file changes.
- `test/cosim.py` — assembles one `test/asm/*.S`, runs both, reduces each side to the sequence of
  **distinct architectural register-file states**, and reports the first divergence with
  instruction number, PC, disassembly, and both values.
- `make sail-setup` / `make cosim` / `make cosim-run`.

`make sail-setup` fetches upstream's **prebuilt release tarball** (pinned, `SAIL_RISCV_VERSION`,
0.13.1) rather than building the model. There is no `brew install sail-riscv`; homebrew's `sail`
formula is an unrelated WordPress deploy tool, and a source build needs opam + OCaml + the Sail
compiler. Upstream ships macOS-arm64 and two Linux tarballs, which covers every machine this repo is
developed or CI'd on. The pin is a plain variable, not an enforced control like `formal/pin.mk` —
nothing executes out of the tarball at build time and nothing on `make test`'s path depends on it.

**Comparing states rather than write events** is a deliberate reduction. A write of a value a
register already holds is architecturally invisible; Sail traces it and a state snapshot does not.
Reducing both sides to distinct states makes them agree on what counts as an event without either
having to special-case it, and makes writes to `x0` a non-issue on both sides.

## Rationale — the spike's actual results

**It agrees, on the whole suite, not just the one program the spike promised.** 49/49 programs
AGREE, 24s wall for the lot on an M-series laptop:

```
AGREE=49 NOT-AGREE=0 wall=24s over 49 programs
```

`add.S` — the named program — matches on all **218** architectural register-file changes, in order
and in value. That is the first time anything in this repo has compared the core's real register
file against an external spec.

**A deliberately-introduced defect is caught, and the measurement is more interesting than a bare
"yes."** The mutation is an extra architectural write outside the retiring instruction's `rd`
(`rtl/regfile.v`, reverted before commit): `regs[31] <= wdata` alongside `regs[waddr]`. RVFI's
per-retire contract is exactly one `rd`, so anything else the core writes is outside what the
monitor and the ladder can describe, by construction; the `.S` suite sees it only if a test program
happens to read that register back, and none of these 49 read `x31`.

| Leg | Mutation A: clobber on every write | Mutation B: same, gated to fire only after cycle 40 |
|---|---|---|
| `make test` — 49 `.S` under cxxrtl, per-retire RVFI monitor live in both sim legs, plus `regfile_tb` | **49/49 PASS — misses it** | **49/49 PASS — misses it** |
| riscv-formal ladder, 78 checks | `reg_ch0` **catches it** (`bad state property 0 reachable at bound k = 20 SATISFIABLE`) | **67 pass / 11 fail — exactly `formal/EXPECTED_FAIL`. Misses it.** |
| Sail co-sim | **catches it** at architectural change #0 | **catches it** at architectural change #18 |

Mutation B is the one that settles the question. It is the same defect, made to fire later than
`reg_ch0`'s 21-cycle window — the shape of any defect whose trigger is a counter rollover, a rarely
taken FSM path, or a specific instruction history. The full ladder run against it is
indistinguishable from a clean one:

```
78 checks: 67 pass, 11 fail
Failure list matches formal/EXPECTED_FAIL exactly.
```

while the co-sim reports:

```
DIVERGENCE at architectural change #18
  sail instruction #27  pc=0x0000004a  add x14, x1, x2
  sail : x14=0x80000000
  core : x14=0x80000000 x31=0x80000000   (cycle 44, decode pc=0x00000054)
```

**So the class this leg catches that the others structurally cannot is: architectural state that no
retiring instruction names, corrupted beyond the bound of a BMC query.** Not "a wrong answer" — the
existing legs are good at wrong answers. Wrong *state*, at a depth the ladder cannot reach and in a
register the test programs never read.

**It also found a defect in this repo's own test infrastructure**, which is the ADR-0019 pattern
repeating. Sail auto-detects HTIF from the ELF's `tohost` symbol and claims the whole **doubleword**
at that address as an IO window. ADR-0008 puts `tohost` at the base of RAM and `test/asm/riscv_test.h`
emits it as a 32-bit `.word`, so every load/store test's `TEST_DATA` starts four bytes later, *inside*
that window. Sail answered every `lw` from `0x10004` with zero (`--trace-mem`:
`htif[0x000010004] -> 0x0`) and eight programs "diverged" for a reason that was entirely the
harness's fault. The spike works around it by stripping `tohost` from a throwaway ELF copy. The real
fix — padding `tohost` to a full doubleword, as upstream riscv-tests does, which would also buy a
clean HTIF exit instead of `--inst-limit` plus a spin-loop check — changes shared test infrastructure
both existing legs read, and belongs in the integration ticket.

## Consequences

- **The "Spike or Sail co-simulation" deferred item is resolved as *yes, eventually*, not *now*.**
  The `docs/adr/README.md` bullet is replaced by a pointer here.
- **This does not become a merge gate, and `make test` does not learn about it.** That is the
  constraint the spike was run under and it holds afterwards: a time-boxed experiment that quietly
  becomes a required check is unplanned maintenance. `make test` works unchanged on a machine with
  no Sail installed; `test/cosim.py` fails with a message naming `make sail-setup` rather than
  breaking a build.
- **Cost, measured, on this machine.** `sail_riscv_sim` on `add.S` with full instruction+GPR
  tracing: **0.17s**. The `cosim` binary: **0.01s**, same as `sim`. One end-to-end `cosim.py`
  invocation: **0.6s**, dominated by `riscv64-elf-gcc` and Sail, not by simulation. Whole suite:
  **24s**, against **7.3s** for `make test`. So a whole-suite co-sim leg costs roughly 3–4× `make
  test` — real, but nowhere near the formal ladder's 75s, and it parallelises per-program trivially.
- **What integrating it across the suite would take**, in rough order of cost: (1) fix `tohost` to a
  doubleword in `riscv_test.h`/`sections.lds` so the HTIF workaround and the `--inst-limit` +
  spin-loop convergence check both go away — this is the only change that touches what the existing
  legs read, and it needs the whole suite re-run on both; (2) a `run_cosim.sh` in the shape of
  `test/run_tests.sh`, with a `test/COSIM_EXPECTED_FAIL` baseline per ADR-0014's set-equality
  contract; (3) a nightly job, not a PR gate, with `sail-setup` cached — the ladder's nightly
  (ADR-0022) is the precedent; (4) extending the comparison to **memory**, which this spike does
  not do at all: Sail has `--dump-memory` and the runner already holds the RAM array, so an
  end-of-run image diff is cheap and would cover stores, which today are checked only by
  `dmemcheck` (bounded) and by whatever the test program asserts.
- **What it will still not catch.** Co-sim runs the programs it is given. It inherits the `.S`
  suite's coverage exactly, so it says nothing about instructions or operand patterns those 49
  programs never reach — including the real multiplier and divider on untested operands, which
  ADR-0010 already flags as covered by nothing else either (the ladder runs under
  `RISCV_FORMAL_ALTOPS`). It is not a proof and does not become one; it makes the programs already
  being run check vastly more per run.
- **It does not read `rvfi_*`, and that must stay true.** The moment `test/cosim.cc` samples an
  RVFI signal to align its trace, it stops being independent of the oracle it exists to
  cross-check, and the mutation table above stops holding. The retire *sequencing* is derived from
  the register file changing, not from `rvfi_valid`.
- **The prebuilt-binary route is what makes this affordable**, and it is the fragile part. If
  upstream stops shipping a macOS arm64 tarball, this leg costs an opam/OCaml toolchain to keep. The
  `SAIL_RISCV_VERSION` pin is what makes that a scheduled problem rather than a surprise.
- Sail's version, the model's own config schema, and `--config-override`'s merge semantics
  (`memory.regions` is **replaced**, not merged) are all things a version bump can move.
  `test/sail/memory-map.json` says so at the point it matters.
