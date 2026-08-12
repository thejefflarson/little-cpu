# ADR-0104: The fetch bus refuses, and the data bus cannot afford to

**Status:** Accepted · 2026-08-11 · *Replaces
[ADR-0067](0067-the-bus-never-refuses-and-the-eleven-are-ruled.md) decision 1 **in part**, and the
split is the whole content of this ADR. That decision made one claim about two buses and adopted it
as a permanent commitment; **the fetch half is overturned — the fetch bus refuses now — and the data
half stands, no longer as a commitment but as a deviation with a measured price.** ADR-0067's
decisions 2, 3 and 4 are re-read below rather than inherited. Re-rules five checks in
`formal/checks.cfg` by name and puts `fault_ch0` on the generated check set. Splits the Sail region
map.*

## Context — what the spec asks for, and what this SoC did

The privileged spec retired *vacant* as a region category: "What previous versions of this
specification termed *vacant* regions are no longer a distinct category; they are now described as
I/O regions that are not accessible (i.e. lacking read, write, and execute permissions)." An access
to one fails its PMA check, and the spec enumerates three ways a platform may surface that: a precise
access-fault exception, an imprecise bus-error interrupt, or a legacy bus error response or timeout.
Of the three it names the first as preferred — "we **strongly recommend** that, where possible,
RISC-V processors precisely trap physical memory accesses that fail PMA checks. Precisely trapped PMA
violations manifest as instruction, load, or store access-fault exceptions."

This SoC did none of the three. `rtl/memory.v` drives `32'b0` when `in_range` is false and every write
arm in both memories is gated on its own range test, so an out-of-region load read zero and an
out-of-region store was dropped, **silently**. An out-of-region fetch read zero too, which decodes as
an illegal instruction, so a PC that ran off the end of text reported cause 2 for an address the
memory had never supplied.

**The hart is conformant and that is not the point.** It implements RV32IMC_Zicsr_Zifencei correctly
and `misa` claims nothing false; the gap is in the platform's memory system, which is why the fix is a
region decode and a trap cause rather than anything about instruction semantics. That distinction
explains where the change goes. It does not soften the finding: a strong recommendation in a spec is
what integrators build on, and silence at the point of failure plants a bug class with no symptom —
a driver writing to a mistyped peripheral address gets no fault, no bus error and no timeout, just a
store that vanishes.

### ADR-0044 option 1, quoted beside what shipped

ADR-0044 recorded three options and picked none. Its option 1 reads:

> **No faulting bus at all.** An address-decoded system where every access in range is answered
> **and nothing out of range is reachable**.

ADR-0067 adopted that label as a permanent commitment. **The shipped design does not satisfy it.**
Nothing stops a program computing an out-of-range address: `test/asm/sections.lds` links `.text` at
zero, the data bus reaches every address the ALU can produce, and there is no protection of any kind.
Out of range is reachable and is answered silently — which is neither of option 1's two clauses. The
second clause was the load-bearing one and it was never true.

That mislabel is the mechanism by which the deviation stopped reading as one. Called "option 1", the
behaviour reads as a design with a stated property; described plainly — *an access the map does not
cover is dropped without telling anyone* — it reads as the defect it is. Both ADRs are on the record
and this is the correction.

## Decision 1 — the fetch bus refuses, and half of ADR-0067's decision 1 goes with it

ADR-0067 decision 1 read the two buses off the RTL and then generalised: "There is no fault line, no
handshake and no way to signal refusal on either bus", adopted "as a commitment, not as a description
of what happens to be built". **The generalisation is what fails.** The two buses were never the same
question — one refusal arrives with the request and the other would arrive with the response — and
binding them into one permanent commitment is what made the cheap half look as expensive as the
dear half for a year. Only the fetch clause is overturned here; decision 2 below is what re-states
what the data clause now is.

`rtl/imemory.v` already computed `in_range` and did not export it. It exports one bit now:

```
assign imem_fault = !in_range;
```

`rtl/decoder.v` takes it as an input and raises **cause 1, instruction access fault**, on the same
`next_pc` override the jumps use. Nothing about the pipeline changes: the fault arrives *with* the
fetched word, in the cycle decode is looking at that word, so it is detected and committed in decode
like every other trap. ADR-0044 had already isolated this as the one of `fault_ch0`'s three arms that
could be committed in decode with the commitment untouched, and that is what it is.

**The commitment is obeyed and strengthened, not amended.** A previously unreachable fault is now
reachable, and it still resolves in decode. It is not a stall reason, adds no bucket to
`make cycles`, and costs no cycle: a faulting fetch issues on the cycle it would have issued anyway
and retires having only redirected the pc.

**Arm order is the whole mechanism, and it is deliberate.** The `imem_fault` arm sits above every
cause the word could produce, because a word the memory never supplied is not an instruction — it is
zero, which decodes as illegal, and whose register fields could name any address at all. So
`instr_illegal` is still high underneath and the cause is 1 anyway. `test/decoder_tb.v` pins exactly
that, and the decoder's `$onehot0` over the six synchronous causes is stated under `!imem_fault` for
the same reason the interrupt arm sits above them.

## Decision 2 — the load/store half is built, measured and declined

It was built first, in full: `rtl/regions.v` decoding text/RAM/timer from one shared `rtl/memmap.vh`
that `rtl/memory.v` and `rtl/timer.v` took their parameter defaults from, three region hits rather
than one "mapped" bit so an atomic could later ask "is this RAM" separately, causes 5 and 7 disjoint
from the misalignment causes, and the whole RVFI fault report with exact store strobes. Every test
passed. It misses the board clock.

Four seeds of `soc/timing_sweep.sh`, `SOC_PROG=datainit.c`, one machine and one toolchain
(Homebrew yosys + nextpnr-ice40; CI's numbers differ, so read the columns against each other and
not against CI). **All four rows are one base**, `e348cb8`, so the comparison is internal; the
shipped design was re-swept after rebasing onto the performance-monitor change and reads
12.54 / 12.73 / 12.77 / 12.78, worst **12.54 MHz**, which is the number to quote:

| tree | default | seed 1 | seed 2 | seed 3 | worst | vs. requirement |
|---|---|---|---|---|---|---|
| `origin/main` (baseline) | 13.46 | 12.84 | 13.04 | 13.12 | **12.84** | +7.0% |
| fetch half only — **shipped** | 12.65 | 13.23 | 13.01 | 12.48 | **12.48** | **+4.0%** |
| load/store half only | 10.57 | 10.99 | 10.88 | 11.00 | **10.57** | **−11.9%** |
| both halves | 11.16 | 11.33 | 11.32 | 10.98 | **10.98** | −8.5% |

The load/store half is 12–18% of period below the baseline distribution at every seed. `make
soc-timing`'s edit-churn band is ~3.6% and its placement spread 1–2%, so this is four to five times
the noise and the two distributions do not overlap at all. **`SOC_MIN_MHZ` stays at 12.0.**

`soc/depth/path_stages.py` attributes it. The critical path is the fetch loop in both trees — ROM
`RDATA` → the window → decode → `next_pc` → ROM address — and it goes from **23 logic levels on the
baseline to 27 with the load/store half**, four extra LUT levels and no extra carry hops, charged to
`decode`. At ~3.3 ns a LUT level that is ~13 ns, which is the ~16 ns measured. Read the buckets with
ADR-0098's caveat: `path_stages.py` charges a level to the module owning the state it newly folds in,
so `decode`'s share is understated and `imem`'s is an upper bound.

**The four levels are not a spelling problem.** The test is an equality on the address bits above each
aligned power-of-two window, which is the cheap form and the one the memories already use — a
magnitude compare would be worse, and `rtl/imemory.v` has already measured a comparison against a
non-power-of-two constant at a quarter of the whole period. The depth is inherent: the fault must be
known before `next_pc` is chosen, and it depends on the **top** of `immediate + reg_rs1`. That adder
already reaches `next_pc` through the `jalr` arm; what is new is a ≤28-bit equality behind it and the
OR into the trap term, and there is no shorter exact statement about the high bits of a 32-bit sum.
`rtl/decoder.v`'s misalignment test is the standing precedent in the other direction: it adds two bits
rather than reading `mem_addr_calc[1:0]` precisely so the carry chain stays out of the fetch loop, and
the region test cannot do the same trick because it needs the far end of that chain.

**So `rtl/regions.v` and `rtl/memmap.vh` are not in this change.** With no consumer they would be
dead code, and shipping the map as unused RTL would make the design look like it enforced something it
does not. The shape is on the record here instead.

**What is still wrong, stated plainly.** A load or a store to an address no memory answers reads zero
and writes nowhere, with no fault, no bus error and no timeout. That is outside the three outcomes the
spec enumerates. It is the deviation this ADR could not close, it blocks nothing about the A extension
except the "is this RAM" question the A extension will have to ask for itself, and the next attempt
should start from the four-level measurement above rather than from the idea.

## Decision 3 — ADR-0067's decisions 2, 3 and 4, re-read

**Decision 2 said "nothing faults after decode, because nothing after decode can fault."** The first
clause holds and the second no longer does: `imem_fault` is a refusal, and a refusal exists on this
core now. What makes the first clause true is not that refusal is impossible but that this one arrives
*before* decode has committed anything — with the word, in the cycle decode reads it. So the ruling
is re-stated: **nothing faults after decode, because the only thing that can refuse says so in time.**
The distinction matters for the next peripheral: a refusal that arrives with the request is free, and
one that arrives with the *response* is not, and no amount of decode-side work fixes the second.

**Decision 3's table is superseded by the five re-rulings below.** Its other six entries are unchanged
and are not restated here; read `formal/checks.cfg`.

**Decision 4 survives untouched.** It is about `formal/imemcheck.sv` holding the fetch-bus property
against the real ports, and nothing here changes either the task or `checks/rvfi_bus_imem_check.sv` at
the pin. `imemcheck` and `dmemcheck` tie `imem_fault` low, because their memory models answer every
address and have no window for a fetch to fall outside of; both still pass.

## Decision 4 — the five, by name

| check | was | is | why |
|---|---|---|---|
| `fault_ch0` | `[DESIGN]` "no access fault can be raised" | **generated**, `[depth] fault 19` | one can be raised: an instruction access fault, which is this check's `ifault` arm |
| `bus_imem_fault_ch0` | `[DESIGN]` "no fault line on the fetch bus" | `[BLOCKED]` | there is a fault line on the fetch bus now. What is left is `RISCV_FORMAL_BUS` — nine `rvfi_bus_*` outputs to drive and to re-prove unread — so this is a burn-down item, not a decline |
| `bus_dmem_fault_ch0` | `[DESIGN]` "no fault line on the data bus" | `[DESIGN]` | unchanged and now measured rather than asserted: the data bus still cannot refuse, and decision 2 above prices what it would cost |
| `bus_dmem_io_read_fault_ch0` | `[DESIGN]` | `[DESIGN]` | the data-bus half of the row above, plus `rvformal_addr_io`, which no region here distinguishes |
| `bus_dmem_io_write_fault_ch0` | `[DESIGN]` | `[DESIGN]` | the same |

**One of the five moved in the direction ADR-0067 said none of them ever would**, and that is the
finding: it called all five "permanently unmeetable". Two of the three conditions on
`bus_imem_fault_ch0` are met now, and the third is plumbing.

### `fault_ch0`'s depth, derived

F = 6 (worst-case first retire) and G = 5 (worst-case retire gap), both re-derived by ADR-0099 and
unchanged here — this adds no stall reason, lengthens no stage and does not widen the scoreboard.
`fault_ch0` has `insn`'s shape exactly: one channel, `assume(valid)` at the check cycle, every
assertion about that one retire. So it takes `insn`'s floor, **F + 2G = 16**, and `insn`'s number, 19,
which clears it by three. Sharing the number means the two move together when F or G moves.

**Its own counterexamples are reachable far shallower, and the depth is the derivation rather than a
measured need.** Probe: report the read half of the fault mask on a fetch fault
(`rvfi_mem_fault_rmask = {4{mem_fault}}` in `rtl/writeback.v`), which makes the check read an
instruction fault as a load fault and assert `insn != 0` against a word that is zero. Swept 19 → 1, it
is `FAIL` at **every** depth: a fetch fault is reachable on the first retire out of reset. The number
stays at `insn`'s because a shallower one would stop asking about later retires for no measured
reason.

### The third condition ADR-0044 named, and it was real

ADR-0044: "Reopening `fault_ch0` is therefore three things, not one: a bus that can refuse,
`rvfi_mem_fault*` driven from `rtl/`, and `mcause` on the RVFI CSR set (which is blocked on its own,
unrelated grounds)."

All three were needed, and the third bites harder than it reads. `checks/rvfi_fault_check.sv` at the
pin puts its whole `mcause` block behind `` `ifdef RISCV_FORMAL_CSR_MCAUSE `` and leaves a dangling
`else` behind when the macro is absent, so **the check does not parse at all without it** — measured,
`ERROR rc=16`, `syntax error, unexpected TOK_ELSE` at line 96. That is an upstream defect at the
pinned SHA, not a configuration choice.

It is resolved without putting `mcause` in `[csrs]`, which is what was blocked: a name there generates
`csrw_mcause_ch0`, and `rvfi_csrw_check.sv` has no WARL model and would fail a correct core.
`` `define RISCV_FORMAL_CSR_MCAUSE `` in `[defines]` declares the four `rvfi_csr_mcause_*` signals
without generating that check. `rtl/csrs.v` exports the shadow under the same macro, so it exists
exactly where the generated checks ask for it and nowhere else — the sim legs do not carry it.
`make -C formal nonperturbation` was re-run for the new instrumentation and passes.

The check now asserts, on every retire that reports a fault, that `mcause` was written in full with 1.
Nothing else in the tree said that; `components_traps` says it too as of this change.

### Both red directions of the `#omit` region, forced

The region is a graded comparison, so both were run rather than assumed, the way ADR-0067 ran them:

| forced | result |
|---|---|
| delete `#omit bus_dmem_fault_ch0` | exit 1, `dropped checks vs checks.cfg #omit declarations: MISMATCH` / `in dropped but not #omit: bus_dmem_fault_ch0` |
| add `#omit insn_add_ch0`, a check that *is* generated | exit 1, `in #omit but not dropped: insn_add_ch0` |

`checks.cfg` was restored and re-run green after each. `formal/EXPECTED_CHECKS` gained `fault_ch0`
and the audit reports `86 generated, 13 declined`, both set equalities exact in both directions —
and the first attempt without that line failed exactly as designed
(`in generated but not EXPECTED_CHECKS: fault_ch0`), which is that file's whole purpose.

## Decision 5 — `mtval` stays zero, and this ADR says so rather than leaving it to be found

`rtl/csrs.v` returns `mtval` as read-only zero. The privileged spec permits that unconditionally —
"if `mtval` is written with a nonzero value when a breakpoint, address-misaligned, access-fault or
page-fault exception occurs" is the conditional form; a hart may always write zero. It **stays zero**
for this cause too.

The reason is that the faulting address is already recoverable and costs nothing to recover: for an
instruction access fault it is `mepc`, which the handler reads anyway. Writing it into `mtval` as well
would duplicate a value the handler has, in a 32-bit register with a write port and a WARL mask, on a
core that has just declined four logic levels for the load/store half of this same feature. When
causes 5 and 7 land, `mtval` is worth revisiting — *those* addresses are not recoverable from `mepc`
— and that is the change that should carry it.

## Decision 6 — Sail's region map is split, and the old one was hiding the same defect

`test/sail/rv32imc_zicsr.json` carried **one 1 MB region** at 0x0 with `executable: true` across the
whole span. So the reference model would execute anywhere the linker script could reach, and
`test/asm/ifault.S` diverges against it — measured, before the split:

```
DIVERGENCE at architectural change #7
  sail instruction #13  pc=0x000000cc  csrrs x6, mcause, x0
  sail : x6=0x00000002
```

Sail reported **cause 2** — the same wrong answer the core used to give, for the same reason: it
fetched a zero word from a region it believed was executable. A reference model that reproduces the
defect cannot report it.

The split is two regions, at the address `rtl/imemory.v`'s decode splits at: `[0x0, 0x4000)`
executable (the 16 KB `rom` of `test/asm/sections.lds`, which is the *simulated* machine's ROM), and
`[0x4000, 0x100000)` readable and writable but **not** executable. Everything else about the second
region matches the first, deliberately: the core does not distinguish them for loads and stores, so a
Sail region that refused a data access would diverge from the core on any program that made one. That
is decision 2's gap, visible in the reference model as well.

`make cosim-suite` graded before and after: **61/63 agreed**, matching `test/COSIM_EXPECTED_FAIL`
exactly in both directions, unchanged either side. `ifault.S` `AGREE`s after the split and `DISAGREE`s
before it. The two baselined `INCONCLUSIVE SAIL-LIMIT` entries are `mtimer.S` and `mtimermask.S`, for
the reason that file records — the model has no machine timer — and are untouched.

## What ran

Machine: 10 cores, one sibling agent live. `make -C formal complete` was run under
`~/.cache/little-cpu/oss-cad-suite`, which is what it requires.

Re-run in full after the rebase onto `be293ff`, not carried forward. The four-seed sweep
reproduces to the nanosecond across that rebase — 78.26 / 78.32 / 78.55 / 79.75 ns — which is
what a rebase over changes that touch no RTL should give, and is the reason to re-run rather
than to assume.

| gate | result |
|---|---|
| `make test` | **64/64** (the suite gained `hpm.S` on the base and `ifault.S` here), `EXPECTED_FAIL` exact both ways; `ifault.S` PASS at 65 retires / 59 spec-checked |
| `make test-units` | 9 benches, all PASS |
| `make probe-gates` | all probes green (it runs inside `make test`) |
| `make window-test` | 14 elaborations, each rejected or accepted as required |
| `make -C formal check` | **86 checks, 86 pass**; `EXPECTED_FAIL` and `EXPECTED_CHECKS` exact both ways |
| `make -C formal components_{decoder,executor,accessor,pcloop,traps}` | all PASS (`pcloop_cover` PASS first) |
| `make -C formal complete` / `complete_cover` | PASS / PASS |
| `make -C formal complete-exclusions` / `interrupt-tie-off` | PASS / PASS |
| `make -C formal dmemcheck` / `imemcheck` / `nonperturbation` | PASS / PASS / PASS |
| `make fit` | **3464** of 3625 budgeted — `FIT_MAX_LC` untouched |
| `soc/timing_sweep.sh`, four seeds | 12.54 / 12.73 / 12.77 / 12.78 MHz — worst **12.54**, +4.5% |
| `make cosim-suite` | **62/64** agreed, baseline exact both ways |

Two graded comparisons are new and both have a demonstrated red direction, forced by mutation rather
than argued:

- `test/imem_tb.v`'s `check_fault`. Set `assign imem_fault = 1'b0;` and three vectors go red
  (`one past the end faults`, `far out of range faults`, `the top of the address space faults`).
- `test/decoder_tb.v`'s cause-1 vectors. Delete the `imem_fault` arm from the trap-cause case and four
  go red, including `...as an instruction access fault, not an illegal instruction: got=00000002`,
  which is the exact behaviour this change replaces.

Neither is a new *script*, so `test/probe_gates.sh` gains nothing: it probes the shell and Python
graders, and a unit bench's red direction is demonstrated the way `test/exec_tb.v`'s is.

`components_traps` needed two assertions widened, and both are the same fact: `imem_fault` is free in
that harness, so a word the memory never supplied can accompany any encoding. `must_not_trap` and the
`mcause`-equals-the-expected-cause assertion each gained `!imem_fault`, for the reason they already
carried `!interrupt_pending` — a redirect the instruction had no part in. A third assertion was added
in exchange: on a fetch fault, `mcause` reads 1.

## Consequences

- **A PC that runs off the end of text now reports the cause the spec names.** `test/asm/ifault.S`
  jumps to 0x8000, past both machines' ROM, and asserts cause 1, `mepc` at the unfetchable address,
  no register written, and that fetching resumes afterwards — with the other direction (a jump that
  stays inside text does not fault) so the file cannot pass on a core that faults every jump.
- **A handler for this cause cannot use `riscv_test.h`'s shared one.** That handler resumes at
  `mepc + 4`, which is four bytes further into the same unmapped window, so it would fault forever.
  `ifault.S` carries its own, which redirects to a label — which is what a real handler has to do too.
- **`test/asm/trap.S` is untouched.** Its eight-traps-and-eight-entries assertion counts entries in
  one program, and adding a cause there would have renumbered every case after it for no coverage the
  new file does not give.
- **The decoder gained an input and the SoC gained a wire.** A platform whose memory answers
  everywhere ties `imem_fault` low and the core never takes the cause; `formal/imemcheck.sv` and
  `formal/dmemcheck.sv` are exactly such platforms and do.
- **`formal/EXPECTED_CHECKS` is 86 and the declined set is 13.** One of the thirteen is now a
  burn-down item rather than a decline, which is the first time that list has shrunk in that
  direction.
- **The next change in this area has a number to beat, not an argument.** Four logic levels in the
  fetch loop, 10.57–11.00 MHz over four seeds. Any spelling of the load/store region decode is
  measured against that, and a candidate that does not move the level count is a null before it is
  placed.
