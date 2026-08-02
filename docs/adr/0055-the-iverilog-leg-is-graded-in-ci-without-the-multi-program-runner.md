# ADR-0055: The iverilog leg is graded in CI, without building the multi-program runner

**Status:** Accepted · 2026-08-02 · *Closes one clause of the "what does not work" bullet in
CLAUDE.md that named `make waves` — the iverilog leg still has no image loader, and that stays
deferred.*

## Context

`test/testbench.v` builds under iverilog on every PR (`.github/workflows/ci.yml`'s `elaborate`
job) and is executed by nothing. `make waves` is the only thing that ran `testbench.vvp`, and it
was not in any workflow; even run by hand, the per-retire monitor only `$display`s on a mismatch —
`ch0_handle_error` sets `errcode` and there is no `$fatal`, `$finish` or `$stop` anywhere in
`test/monitor.v` or `test/monitor.sim.v` — so a mismatch exited 0.

ADR-0037 already measured what this leaves uncaught: with the hazard scoreboard's two continuous
assigns written as a `function automatic live_producer(r)` (iverilog under-sensitizes the call,
CLAUDE.md's engineering rules), the leg froze at the first RAW hazard for the whole of M1 and
nothing noticed, because nothing ran it. "A leg that cannot fail is not a leg."

The design brief defers a graded multi-program iverilog runner (an image loader for
`test/testbench.v`, mirroring `test/cxxrtl.cc`'s `--rom`/`--ram`). That stays deferred here. The
only artifact this ADR grades is the one that already exists: `testbench.vvp`'s baked-in 200-cycle
program (the six-instruction increment loop `test/testbench.v` writes into `rtl/imemory.v`'s two
banks under `ifdef ICARUS`).

## Decision

### 1. Two failure paths, both under `ifdef ICARUS`

`rvfi_monitor_errcode` is a one-cycle **pulse** — `test/monitor.sim.v`'s outer `always` block
resets it to 0 every cycle before conditionally setting it — so it has to be checked every cycle,
the same way `test/cxxrtl.cc`'s runner loop already does. A continuous `always @(posedge clk)`
block right after the monitor instantiation does that and `$fatal(1)`s on a nonzero value,
matching `test/cxxrtl.cc`'s exit 4.

A second block adds an end-of-run floor: fewer than 15 memory writes or fewer than 60 RVFI retires
over the 200-cycle run is also `$fatal(1)`. Both counters are new (`mem_write_count`) or already
present for `test/cxxrtl.cc`'s debug-item reads (`rvfi_retires`, declared in the existing
`ifdef RISCV_FORMAL` block for the observation counters — ADR-0053's sibling change to
`test/testbench.v`).

**Measured on this exact program at `238a066`: 20 writes, 79 retires, all 79 spec-checked.** The
floors (15, 60) sit 25% and 24% under those, enough margin for a legitimate change that costs a few
extra stall cycles, and nowhere near what the named regression produces: reverting `live_rs1`/
`live_rs2` to the `function automatic live_producer(r)` form gives **0 writes, 1 retire** on the
same program, confirmed by building and running it.

### 2. Why the grading code moved, not just grew

The existing `ifdef ICARUS` `initial` block that waits 200 cycles and calls `$finish` sits right
after the clock/reset declarations, near the top of the file — *before* `rvfi_monitor_errcode`/
`rvfi_retires`/`mem_write_count` are declared (all three live inside or need the
`ifdef RISCV_FORMAL` block that follows). iverilog does not bind a forward reference to a `logic`
declared later in the same module — confirmed with a two-line repro (`$display` of a variable
declared after the referencing `initial` block fails to elaborate: `error: Unable to bind
wire/reg/memory`). `clk`/`reset` themselves stay declared where they were, since the memory/CPU
instantiations that follow reference them by name; the whole `$dumpfile`/`$dumpvars`/reset/wait/
grade/`$finish` sequence moved, as one `initial` block, to sit after those three declarations
instead of splitting into two blocks that would need to agree on a cycle count (`repeat(1)` here,
`repeat(200)` there) without either saying so.
`testbench.vvp` always compiles with both `ICARUS` and the `RISCV_FORMAL_MACROS` list together (one
Makefile recipe, `RISCV_FORMAL_MACROS` unconditional), so nothing here changes what the build
requires.

### 3. `ci.yml`'s `elaborate` job runs and grades it

The step that builds `testbench.vvp` gains `vvp testbench.vvp`, checked directly (no pipe — the
existing `run:` block already avoids one, per ADR-0037 §4's rule). `vvp`'s exit status is `$fatal`
→ 1, `$finish` → 0, so the step is graded with no new tooling and no new dependency.

**`elaborate` was already a required status check.** This strengthens a required job in place; it
does not add anything to branch protection, which stays a human action (ADR-0036). A side effect,
not a target of this change: `make waves`'s recipe (`vvp $< && mv testbench.vcd $@`) now also fails
on the same three conditions, because `make` aborts a recipe on the first nonzero line. `make waves`
was not made to grade anything on purpose here — the multi-program runner it would need to mean
something beyond this one baked-in program is still the deferred work — but it no longer silently
swallows a mismatch either.

### 4. What stays out of scope

No image loader for `test/testbench.v`, so this is **one fixed program, not the `.S` suite** — the
gap CLAUDE.md's "what does not work" bullet already names stays open for that half. Record the
outcome as *single-program graded, multi-program still absent*, not as the coverage gap closed.

## Evidence

Three red directions demonstrated on real `vvp` runs of `testbench.vvp`, each reverted after:

| Mutation | Result |
|---|---|
| `rtl/decoder.v`'s `live_rs1`/`live_rs2` reverted to `function automatic live_producer(r)` (ADR-0037's defect) | `FLOOR VIOLATION: writes=0 (need >= 15) retires=1 (need >= 60)`, exit 1 |
| `rtl/accessor.v`'s `out.rvfi_mem_wdata` shadow offset by `+1` on stores (real memory unaffected) | `RVFI Monitor error 120` every store, `RVFI MONITOR ERROR 120`, exit 1 |
| `rtl/executor.v`'s `is_add` arm computes `rs1 + rs2 + 1` | existing `TRAP TO ZERO` `$fatal` fires (mtvec == 0, no handler installed), exit 1 |

The unmutated tree exits 0 and prints `RETIRES 79 SPEC-CHECKED 79 WRITES 20`.

## Consequences

- CI's `elaborate` job now actually exercises the iverilog leg instead of only proving it
  compiles — CLAUDE.md's verification table calls this leg "the microscope"; it now has something
  to look through.
- The floor is a fixed pair of numbers pinned to one fixed program, in the ADR-0038/ADR-0040
  measurement idiom: cite the commit it was measured at, not "current"; re-measure and restate
  (with margin) if the baked-in program or the pipeline's stall behaviour changes enough to move it.
- `test/testbench.v` gains two `ifdef ICARUS` regions: the per-cycle errcode check, placed right
  after the monitor instantiation it reads, and the floor check folded into the existing
  wait/`$finish` block, placed after the counters it reads. `test/rtl.cc` (the cxxrtl leg's
  elaboration of this same file, `make sim`'s dependency) and `make elaborate-strict` both build
  `test/testbench.v` through yosys with `ICARUS` never defined, so neither region exists in either
  of those builds — only iverilog defines `ICARUS`.
