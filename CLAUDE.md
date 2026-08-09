# Little CPU

A hobby RISC-V core in SystemVerilog on the open toolchain (Yosys / iverilog / SymbiYosys — no
vendor EDA). Target **RV32IMC_Zicsr_Zifencei**, machine mode only; home is an ice40 up5k running at
the board's 12 MHz crystal.

**Four goals: fast, simple, readable, formally verified.** They are not in tension by default — the
clearest spelling of a thing is often the fastest and the smallest, and the formal harness is what
makes aggressive simplification safe. Every rule in this file serves one of the four; a rule that
stops serving them gets deleted. This file is a rulebook, not a changelog — history lives in git
and `docs/adr/`, and the README is deliberately left as-is; ground truth is here.

Three habits carry the goals:

- **Measure a conflict; never assume one.** `make fit` and `make soc-timing` are the instruments.
  Do not discard a measured win because it sounds like an optimisation, and do not take
  a tidier spelling that costs measured speed or area without recording the trade. Three standing
  precedents, two declined and one taken on the same kind of evidence: `rtl/memory.v` ships the flat
  spelling of its write/read arms (the nested one costs 3.6% of Fmax), `rtl/executor.v`'s
  `mul_div_counter` stays `[6:0]` (narrowing it costs 37 cells), and `rtl/decoder.v` now asks the
  register file for the *next* instruction's pair on a cycle that issues — 9.7% of suite cycles,
  **1.1% of Dhrystone's**, and no measured clock or area (ADR-0089). A margin that declines a change
  is a measurement with a date on it: the 0.83% that declined this one was re-measured against a
  tree whose own worst placement had since fallen to 2.08%. The version that guesses a *compressed*
  successor too is the one that misses 12 MHz, and it stays declined.
- **Prove the property, then spend it.** Find a place the design pays for a property it already
  proves — a priority chain over proven-disjoint flags, a comparator that cannot differ — simplify
  it, and let the riscv-formal checks, the component proofs and the `.S` suite say whether the
  property still holds. A marking is spent against an assertion, never against belief:
  `(* parallel_case *)` is legal only where a `$onehot`/`$onehot0` check covers the exact flags in
  that arm list (ADR-0068).
- **A grader that cannot fail is not a grader.** Every graded comparison must have a demonstrated red
  direction that fails for the reason it was written; `make probe-gates` forces all of them and
  runs as a prerequisite of `make test`. Five of this repo's recorded defects were comparisons
  whose failure path had never once run.

## Design commitments

**These can and should change when a change moves the four goals forward together.** The goals are
what the project is for; a commitment is only a means to them, and none is held harder than the
others. How to change one: measure the improvement, show the other three goals still hold —
measured where they are measurable (`make fit`, `make soc-timing`, the riscv-formal checks, the
`.S` suite) —
and record the amendment as an ADR. The evidence is required because breaking a commitment is
silent — tests can stay green while the design rots — which is exactly why these are written down
and measured rather than left to judgement. Older ADRs cite these as `invariant N`; the numbers
are kept in parentheses so those references still resolve.

- **No wrong-path state** (1). No state may exist that a later cycle must un-commit — no flush
  logic, no kill signal. The decoder owns the PC and is its only driver; the fetch address is
  published a cycle early (`next_pc` → `imem_addr_next`), and a stalled cycle re-presents the same
  words. This keeps the required BMC depths small and derivable, retire unfiltered, and `pcloop`'s
  induction free of speculative state. Enforced by `formal/pcloop.sv`, `rtl/decoder.v`'s `FORMAL`
  block and `test/decoder_tb.v`.
- **All traps are detected and committed in decode** (2). Nothing faults after decode; a trap is a
  branch to `mtvec` on the same override the jumps use. This is what makes CSR commit precise with
  no reorder buffer. It stands on ADR-0067's ruling that the bus never refuses a transaction.
- **Every inter-stage struct carries a `valid` bit** (3). A bubble is `valid = 0`; retire is
  `valid` reaching writeback, which gates `wen` and drives `rvfi_valid`.
- **Hazards are stall-only** (4). No forwarding network, and 34.0% of suite cycles is what that
  costs — **16.5% on Dhrystone, and the two are not separable from the operand-fetch cycle in either**
  (ADR-0084): a cycle that is both is charged to the scoreboard, which is why removing two thirds of
  the suite's operand column moved the scoreboard column *up* by 466 cycles (ADR-0089). Read 34.0%
  as an upper bound on one workload, never as the prize.
  Both spellings were built and measured (ADR-0083): forwarding the executor's result to every
  operand reader buys 12.9% of cycles and misses 12 MHz outright at 9.49, and confining it to the
  executor's operands buys 7.5% and holds 12 MHz on 0.48% of margin against today's 3.35%. Deleting
  the scoreboard outright buys **0% of period** at its ceiling, so nothing in this direction pays for
  the muxes, and only one of the three in-flight slots can be forwarded from at all. That is the
  evidence to beat.
- **CSR instructions, `mret` and `fence.i` serialize** (5) — held in decode until execute, access
  and writeback are empty. Two distinct reasons share the mechanism: the first two so a one-cycle
  architectural update cannot interleave with older instructions; `fence.i` because text is
  writable and the fetch address publishes early, so an older store's write edge must pass first
  (ADR-0061). Do not collapse them. The emptiness check reads **four** slots —
  `accessor_out.valid` is routed in separately because a store in flight is invisible to the
  scoreboard's three (ADR-0026).
- **The regfile read is synchronous, and the answer belongs to the address pair presented the
  previous cycle** (6, 9). Decode presents a pair, bubbles a cycle (`operand_stall`) whenever what
  was presented is not what the instruction reads, then issues — and in the issue cycle observes the
  architectural value of rs1/rs2 *including a writeback committed that same cycle*, via two fabric
  forwarding points (write-first into the read register, then the write-through bypass). What it
  presents on an issuing cycle is a **guess at the next instruction's pair**, read flat out of the
  fetch window's successor word (ADR-0089), so the pair presented and the pair being read are
  deliberately different signals there. The bypass selects on a **registered copy** of the address
  pair and is correct because `operand_stall` lets nothing issue until the held pair is the pair the
  issuing instruction reads (ADR-0064 as amended by ADR-0089) — narrowing `operand_stall` breaks it
  with nothing to say so except two `test/regfile_tb.v` vectors and `reg_ch0`. Touching
  `operand_stall` is an amendment, not a tuning change. The standing liveness probe: delete the rs2
  write-through bypass and `reg_ch0` must go SAT — run it before believing any `reg_ch0` result
  under a changed configuration.
- **Stalls are one global broadcast over two mechanisms** (8): a divider or accessor stall
  **holds** `decoder_out` unchanged (an issued instruction nothing has consumed); every other
  reason **bubbles** (nothing issued). A `fetch_stall` coinciding with a freeze HOLDS — bubbling
  would drop an issued instruction — and that ruling is only arm order in the publish block, so it
  is asserted in `rtl/decoder.v`'s `FORMAL` block and vectored in `test/decoder_tb.v`. Every
  in-flight non-`x0` `rd` must be visible to the scoreboard on every cycle between issue and the
  regfile write-through, with no gap. Six reasons raise `stall`, and it is exactly their OR: the
  divider, the accessor's load turnaround, the decode scoreboard, serialization, the operand-fetch
  cycle, the stolen fetch window. `test/decoder_tb.v` checks that identity and `make cycles`
  charges every stalled cycle to the first reason that is true, so a seventh has to be added in
  both places as well as here.

Retired numbers, never reused: 7 (the generated-but-tracked monitor) lives under Verification; 9 is
folded into 6.

## ISA target

RV32IMC_Zicsr_Zifencei, M-mode only, `misa = 0x4000_1104` (neither Z-extension has a `misa` bit;
the ISA string is the only place they are claimed). Traps implemented: illegal instruction = 2,
breakpoint = 3, load misaligned = 4, store misaligned = 6, ecall from M = 11.
Instruction-address-misaligned is unreachable (C makes 2-byte targets legal) and not implemented.
C stays because code density is a product constraint on the up5k (ADR-0002/0003). `fence.i` costs a
pipeline drain — see the serialization commitment.

**One interrupt: the machine timer, cause `0x8000_0007`.** `mie.MTIE` is the only writable bit of
`mie`; `mip.MTIP` is `rtl/timer.v`'s line and read-only; `mip.MSIP`/`mip.MEIP` stay read-only zero,
which is conformant WARL on a platform with neither source. `mtime`/`mtimecmp` are four words at
`0x0002_0000`, in `rtl/littlesoc.v` and `test/testbench.v` alike. **It is taken on a cycle that
would otherwise have issued**, because `stall` outranks the trap arm of `next_pc` — so it waits out
a divide, a load turnaround and a serialization with no logic of its own, the displaced instruction
has not issued, and nothing is un-committed. It is therefore **not** a stall reason and adds no
seventh bucket to `make cycles`. Measured worst-case response: 33 cycles, 2.75 µs at 12 MHz, set by
the divider. `mtimecmp` resets to zero, so `mtip` is asserted out of reset and both enables resetting
to zero is what makes that harmless (ADR-0082).

Three things about it are the platform's to state, and firmware cannot derive any of them:
**`mtime` ticks once per clock cycle**, so 83.33 ns at 12 MHz — the spec asks only for a constant
frequency and a published period, and blesses the cycle counter for a fixed-frequency system.
**MTIP is a level**, posted until `mtimecmp` exceeds `mtime`; taking the trap does not lower it, so
a handler that returns without moving `mtimecmp` is re-entered before the instruction at `mepc`
runs. **An RV32 `mtimecmp` update is the spec's three stores in the spec's order** — low all-ones,
high, low; high-first is unsafe and `test/timer_tb.v` and `test/asm/mtimer.S` each fire a spurious
interrupt that way on purpose before doing it correctly.

**Conformance is not negotiable against minimality.** Every CSR the privileged spec mandates for
RV32 M-mode is implemented — most legally read zero, so the cost is near nothing. The CSR set is a
floor, not a closed list; "exact" once made a conformance gap look like a design choice (ADR-0048).

## Verification — three legs, each load-bearing

| Leg | Role | Catches what the others can't |
|---|---|---|
| **cxxrtl** | primary runner | real mul/div arithmetic, long/randomized runs |
| **iverilog** | microscope | waveforms, `$display`, second elaboration frontend |
| **riscv-formal** | oracle | exhaustive per-instruction semantics, pipeline corners |

What a green result does and does not mean:

- **The riscv-formal checks run under `RISCV_FORMAL_ALTOPS` and never check the real multiplier or
  divider.**
  The named oracles for that arithmetic are `test/exec_tb.v` and `components_executor`, both
  mutation-checked (ADR-0051).
- **Every generated riscv-formal check is `mode bmc`**: PASS means no counterexample within that
  check's depth, not
  that the property holds. Depths are derived from two measured numbers — F (worst-case first
  retire, from `hang`) and G (worst-case retire gap, from `liveness`). **Any change that adds a
  stall reason, lengthens a stage, or widens the scoreboard must re-measure F and G before it
  lands** (ADR-0046); some depths exceed their measured minimum by only one cycle.
- **riscv-formal ships no spec model for SYSTEM or MISC-MEM at the pinned SHA**, so trap and CSR
  behaviour
  is checked against assertions this repo wrote (`test/asm/trap.S`, `test/csr_tb.v`, the decoder
  and `traps` proofs), not an oracle. `formal/COMPLETE_EXCLUSIONS` mechanises that boundary: a pin
  bump that adds a spec model goes red until the exclusion comes out. The generated instruction
  check also drops every value comparison once an instruction traps, keeping only the trap flag,
  and its two pc checks accept whatever target the core reports — so `components_traps` is the only
  thing that says a trap lands on `mtvec` and saves the right state.
- **It ships no model of an INTERRUPT either**, so the core's timer input is tied off in all five
  harnesses under `formal/` and the generated checks run with no interrupt in the trace.
  `formal/INTERRUPT_TIE_OFF` mechanises that the same way, in both directions and re-derived from
  the clone. F and G were re-measured under the tie-off and both flip points reproduce exactly, so
  the depths are unaffected. `components_traps` is the oracle for entry — `test/asm/mtimer.S` and
  `test/asm/mtimermask.S` for the whole path. `rvfi_intr` is now driven and is **not** optional:
  both sim legs' monitor checks pc continuity across retires and stops only for a retire carrying
  it, so an undriven `rvfi_intr` makes every interrupt a monitor error.
- **Both sim legs read the sanitized `test/monitor.sim.v` as their per-retire oracle**, so
  `test/sanitize_monitor.py` is a change to the oracle, not to plumbing. `test/monitor.v` is
  generated but tracked: regenerate it (`make monitor-check`), never hand-edit it.
- **iverilog derives a continuous assign's sensitivity from the call's arguments.** A function
  called from a continuous assign whose body reads module state silently under-evaluates — no
  diagnostic — and once left this leg dead for a whole milestone while every grader stayed green.
  Write such logic out. Treat a green iverilog run as evidence only if it could have failed.
- **cxxrtl is two-state, so this leg is the only one that can see an X.** Decode reads register
  numbers out of the word *after* the instruction it is issuing, so an undefined ROM word reaches
  the register file's address port and turns the whole pipeline X — green under cxxrtl, green under
  every formal check because they drive that word as a free two-state input, red only here.
  `test/testbench.v` zeroes both ROM banks before poking its program for that reason, the way
  `soc/compare/rom_flat.py` zero-pads its image to full depth. A simulated memory defined only where
  a program was written is not a model of a block RAM, whose every word comes out of the bitstream.
- **Sail co-simulation is deliberately not a leg and stays off CI's required checks** (ADR-0032).
  `test/cosim.cc` reads the core's real `regs_a` and no `rvfi_*` signal — the property that lets it
  catch architectural writes the self-reporting oracles structurally miss (measured: an extra
  `regs[31] <=` write enabled only past the BMC bound, invisible to every riscv-formal check and
  the whole `.S` suite, reported by co-sim in 0.6s). A change that needs its verdict carries
  pre/post `make cosim-suite` output in the PR. Do not "align" it against `rvfi_valid`.

## Reference models

**Signed arithmetic in a reference model must be a self-determined statement of its own, never an
arm of a conditional expression.** IEEE 1800 sign-context rules silently evaluate the whole
expression unsigned, for negative operands only; this has produced wrong oracles here twice (the
generated monitor's DIV/REM models, `exec_tb`'s SRA reference). Every reference pins itself against
hand-computed literals before any RTL vector runs and stops with `ORACLE BROKEN` if degraded — an
oracle that is wrong is worse than none, because it fails correct hardware and teaches the reader
to distrust the bench.

## Measurements and ratchets

Two instruments, two designs — never merge their numbers. `make fit` is the core alone (its top
never places: 231 `SB_IO` against sg48's 39, expected); `make soc-timing` is the SoC, which places
and times.

- **`make fit` has a churn band of about ±50 cells**: functionally identical edits move the count
  that much from ABC/nextpnr re-mapping alone. A delta inside the band is not evidence of
  anything, and a ratchet (`FIT_MAX_LC`) must sit outside it. **The number is also
  toolchain-dependent** — CI's pinned OSS CAD Suite and a local Homebrew yosys differ by ~21 cells
  on identical RTL — so quote the `fit` job's number; a local run is a sanity check.
- **`make soc-timing` has a ~3.6% edit-churn band and a 1–2% placement spread.** One placement is a
  sample: `soc/timing_sweep.sh` runs four seeds; compare distributions, not single runs. A delta of
  a couple of percent is not evidence of anything.
- **12 MHz is a requirement, not a regression floor** (ADR-0066). `SOC_MIN_MHZ` is 12.0 — the board
  clock, whose next divider step down is 6 — and it does not slide. When it trips, fix the design,
  not the floor. The margin over the worst placement is deliberately tighter than the churn band.
- **The divider step above today's 12 is 24.** `SB_HFOSC` divides 48 MHz by 1, 2, 4 or 8 —
  48 / 24 / 12 / 6. An `SB_PLL40` fed by the 12 MHz crystal can synthesize an intermediate clock
  if a change ever earns one. The target for Fmax work is 24 MHz — 41.67 ns, about half of today's
  period — and 12 is already met. A few-percent idea can be read against 41.67 ns and declined in a
  minute, instead of after four placements (ADR-0078).
- **`make dhrystone` is the only figure comparable to another project's**, and it is quoted in
  DMIPS/MHz because that is what the field publishes. 0.535 at `-O2`, 3568 bytes of the SoC's 8 KB
  ROM (ADR-0084). Dhrystone is string-dominated and the optimiser can delete part of the work, so
  **the flags, the compiler and the string library travel with the number** — the program prints all
  three and will not compile without them. It is not a gate and adds no ratchet. **Quote the
  absolute figure with it**: 6.42 DMIPS at the board's 12 MHz, because Fmax above the requirement is
  margin and not speed, so a CPI win converts to throughput and a placement that closes higher does
  not (ADR-0089).
- **The only cross-core comparison that means anything is one harness**, and `soc/compare/` is it:
  same part, memories, program, toolchain and seeds, this core against the VexRiscv Verilog in the
  pinned riscv-formal clone. Measured that way the gap is **1.6×, not the 3× two separately-published
  numbers suggested**, both critical paths are the fetch loop, and their 92 MHz does not reproduce
  here (ADR-0086). Neither side is a shipped design — theirs is RV32IC with a branch predictor and no
  privileged architecture, ours has no timer in the harness and 4 KB of ROM — so quote it with what
  it is. **A harness whose outputs do not depend on the datapath measures nothing**: an all-NOP ROM
  placed 449 cells of a 1711-cell core and reported a critical path, so
  `soc/compare/placed_vs_synth.py` grades the placed count against the core's own synthesis and
  `make compare-smoke` requires both cores to publish the same values.
- **Taking the instruction memory out of that fetch loop is priced and declined** (ADR-0087). A
  synchronous memory leaves a combinational loop only behind a register, and a register in the fetch
  loop is a fetch stage — so "the memory out" and "the depth curve" are one experiment. Measured over
  four seeds on both parts: the tail comes out for 3–4 levels, the **head does not come out at all**
  (registering it *adds* three to four levels, because a bank output mux is one `SB_LUT4` that ABC
  folds into the decode reading it, and a register there forbids the sharing), and the best product
  of clock against cycles anywhere is +0.4%. The two parts disagree in sign — −7.2% on hx8k, +1.2% on
  up5k — so do not quote one. Redirects are 7.15% of the suite's issues and **16.92% of
  Dhrystone's**, the opposite ordering from the RAW share, so no depth argument stands on the suite
  alone.
- **yosys and ABC already do everything derivable from the expression** — dead bits, common
  subexpressions, duplicate adders. An edit that restates the same arithmetic in the same terms is a
  null, and two are measured: narrowing `mul_div_store` from 64 bits to 32 where only `[31:0]` is
  read moves `fit` by 11 cells because DCE had already removed them, and flattening the `next_pc`
  priority chain into a parallel mux buys 53 cells and costs 3–9% of period. **What they cannot use
  is a fact from outside the expression**: a parameter is a power of two, a window is aligned, a
  reversal is wiring, an address bit is provably zero because a trap guarantees it. Eleven such edits
  together are worth −169 placed cells on the SoC and −84 on `fit`; **each one alone is inside the
  ±50 band**, and over eight seeds a side the period moved +0.3% at the median — a null in both
  directions (ADR-0088). It is an
  area lever, not an Fmax one — **occupancy does not set the period on this part**, measured by
  ballasting 77% to 95% for no change at all. Where a fact like that is now load-bearing it is an
  elaboration `$fatal`, not a comment: `rtl/{imemory,memory,timer}.v` refuse to build at a
  non-power-of-two depth or an unaligned `BASE`, and `make window-test` forces all three red.
  **The rule does not only find small things.** The same question asked of `rtl/executor.v` found
  three that each clear the band alone — a divider carrying 64-bit registers for a 32-bit division,
  a duplicated negator inside a one-hot mux, and the multiplier's 33rd partial-product row in soft
  logic — worth −404 on `fit` and −400 placed cells together, with the period a null again and
  `ICESTORM_DSP` unmoved at 4 (ADR-0090).
- **Read logic levels apart**: a LUT level costs ~3.3 ns (delay plus interconnect), a carry hop
  ~0.34 ns and no interconnect. A change that trades a carry hop for a LUT level gets shallower by
  icetime's count and slower in nanoseconds. The SoC is routing-dominated; wide flat muxes route
  worse than chains. **There is no single lever.** The decode head
  (`imem.in_range → instr → {rs1/rs2, immediate, hazard}`) was named as one and measures 3.3%
  deleted whole, inside the churn band. The period is in the fetch loop instead: no single input to
  `next_pc` is worth more than 5%, all of them deleted together are worth 21%, and collecting that
  means the pc stops depending on this cycle's decode — which is the no-wrong-path-state commitment
  (ADR-0076). **Measure the whole set.** A ceiling over one term bounds only that term, and a median
  inside the churn band is a null in both directions — neither a candidate declined nor one
  admitted.

Baselines and grading:

- Failure baselines (`test/EXPECTED_FAIL`, `test/COSIM_EXPECTED_FAIL`, `formal/EXPECTED_FAIL`)
  are **name-and-status pairs under set equality in both directions** — an unexpected pass is as
  red as an unexpected failure, and a baselined test failing a *different* way is red too.
  `test/OBSERVED_FLOOR`'s name set doubles as the suite manifest, checked both ways before anything
  runs, so a suite that shrank is red rather than a smaller table that still "passes". Before
  adding a co-sim baseline entry, read that file's header — it is the decision procedure.
- **A `.c` entry's floor numbers are a silence bound, not an observation.** An assembly program's
  retire count is fixed by its instruction sequence; a C program's is whatever that gcc inlined,
  and the two toolchains here differ by about 1% on identical source — enough to trip a `>=` floor
  copied from the table, for a reason the floor cannot tell apart from the monitor going blind. So
  a `.c` line is 16, chosen because the recorded blindness defects produced 0 retires (already
  exit 6) or 1. `test/run_tests.sh` rejects a `.c` floor above 64 rather than trusting the header
  (ADR-0081).
- **Never put a graded command in a pipeline in a CI `run:` block**: the default shell is errexit
  without pipefail, so the step's status becomes `tee`'s. This held the formal CI job accidentally
  green for its entire life.

## Commands

```sh
make setup          # install the RISC-V toolchain (brew on macOS)
make test           # the test/asm suite (.S and .c) under cxxrtl + unit benches +
                    # probe-gates, graded against EXPECTED_FAIL / OBSERVED_FLOOR
make test-units     # the unit benches alone; the bench list is checked against
                    # test/*_tb.v in both directions
make probe-gates    # force every graded comparison red for its own reason; hermetic
make window-test    # force the elaboration checks in rtl/{imemory,memory,timer}.v
                    # red, in both frontends. Runs inside `make test`
make cycles         # the suite again, every cycle charged to an issuing cycle or
                    # one of the six stall reasons; nonzero on a stalled cycle none
                    # of them explains. Not on CI -- there is no CPI ratchet
make dhrystone      # Dhrystone 2.1 (test/bench, NOT the graded suite) -> DMIPS/MHz,
                    # the ROM image against the SoC's 8 KB, and the same accounting
                    # on compiled code. DHRY_RUNS picks the iteration count.
                    # Not on CI, and it adds no ratchet either
make waves          # iverilog leg -> waves.vcd; one baked-in program, graded,
                    # not the test/asm suite (test/testbench.v has no image loader)
make monitor-check  # regenerate test/monitor.v at the pin and diff
make fit            # the core's area number; ratchet on FIT_MAX_LC
make soc-timing     # the SoC place-and-time flow; requirement on SOC_MIN_MHZ.
                    # SOC_SEED picks a placement; soc/timing_sweep.sh runs four
make compare-timing # this core and VexRiscv in ONE hx8k harness; COMPARE_CORE
                    # picks the side, soc/compare/sweep.sh runs both over seeds.
                    # A measurement, not a gate -- but the placed-vs-synthesised
                    # check inside it is graded
make compare-smoke  # both harnesses run the one image in iverilog and must
                    # publish the same values; says the netlist RUNS

make -C formal check                # the generated riscv-formal checks; always a fresh run.
                                    # interrupt-tie-off is a prerequisite
make -C formal check-baseline       # re-grade a finished run without re-running
make -C formal components_decoder   # component proofs by k-induction (mode prove):
make -C formal components_executor  #   read the sby summaries, not the job colour
make -C formal components_pcloop
make -C formal components_traps     #   traps is the only proof over real mtvec/mepc/mcause/mstatus
make -C formal complete             # depth-50 whole-ISA walk minus COMPLETE_EXCLUSIONS
make -C formal complete_cover       # its anti-vacuity control
make -C formal nonperturbation      # RVFI instrumentation is unread by the core;
                                    # structural, NOT sequential equivalence

make sail-setup     # once: fetch the pinned sail-riscv release
make cosim-run      # co-sim one program (PROG=add.S)
make cosim-suite    # the whole suite, graded against COSIM_EXPECTED_FAIL
```

`make sail-setup` and `make lint-setup` unpack into `~/.cache/little-cpu`
(`XDG_CACHE_HOME` moves it), **outside the checkout**. A git worktree is given tracked files only
and a downloaded tool is gitignored, so an install inside the checkout is invisible from every
worktree — which is why it does not live there. `make test` enforces it.

Toolchain: macOS `brew install riscv64-elf-gcc`; Linux `apt install gcc-riscv64-unknown-elf`.
Tests are freestanding assembly, so no multilib or newlib. Formal needs the pinned YosysHQ OSS CAD
Suite. CI runs on every PR (`.github/workflows/ci.yml`); read the required set live from
`gh api repos/thejefflarson/little-cpu/branches/main/protection`, not from comments.

## Engineering rules

- **Compiler and elaboration warnings are errors.** Two allowlisted exceptions, both documented
  where they are allowlisted: iverilog's `sorry: constant selects in always_* processes` for
  `rtl/writeback.v`'s `always_comb` struct reads (over-sensitivity — provably safe; only
  *under*-sensitivity is a bug; do not add new ones outside that file, prefer a continuous
  assign), and yosys's `Deep recursion in AST simplifier` notice on the `elaborate` CI job.
- **Comments earn their place or go.** A comment must say something the code does not, in one or
  two plain sentences readable without leaving the file. Delete restatement, history (git has it),
  section banners and emphasis furniture. **No ADR numbers and no invariant numbers in comments** —
  state the mechanism instead; `git blame` finds the record. One exception: tripwires — a two-
  sentence warning that stops a fixed defect from being silently reintroduced. This discipline now
  applies to this file too.
- **Every non-trivial change adds or updates tests and runs the full suite** before being declared
  done. Elaboration succeeding is not a substitute.
- **Never commit build artifacts** (`test/rtl.cc`, `sim`, `*.vvp`, `*.vcd`, `rvfi_macros.vh`,
  `formal/` output dirs). `test/monitor.v` is the one deliberate exception.
- **riscv-formal is SHA-pinned.** A pin bump regenerates `test/monitor.v`, re-runs the generated
  checks, and
  re-derives the sanitizer's site counts and `COMPLETE_EXCLUSIONS` rather than editing them to
  silence a failure.
- **No ticket IDs in code, comments, ADRs, docs, or commit messages.** Cite the ADR, the commit
  SHA, or just say the reason. PR titles and descriptions are the exception — a ticket ID there
  buys automatic closing and rots nothing in the repo.
- Prefer verified/first-party GitHub Actions; simplest approach unless asked otherwise.

## State

M1 (the pipeline runs the RV32IM suite), M3 (CSRs and machine-mode traps) and M2 — parity with the
serialized core this rewrite tore down, which was formally verified before the teardown — are all
reached. M2's six-term conjunction was built by ADR-0037 and its successors and every term was
re-measured against merged main in one sitting before the milestone was declared (ADR-0079); that
audit, not any single green run, is the record.

**Declaring M2 is a claim about those six terms, not a claim that the core is correct.** Three
residuals outlive it. The multiplier is checked *differentially, not exhaustively* — operands, the
retired slice and three points of the product function, with `test/exec_tb.v`'s randomized vectors
covering the rest. The divider is proved under a *recorded magnitude restriction* and its completion
assertions are basecase-unreachable at the configured depth, so that result is inductive only.
`complete` passes over a *recorded exclusion set*, not over the ISA. The two caveats under
Verification — bounded BMC, and no spec model for what M3 added — survive M2 by design and are not
burn-down items. `csrc_upcnt` is the narrow case worth knowing by name: it says `minstret` strictly
increases, **not** that it advances by exactly the non-trapping issues, so ADR-0027's rule is
narrowed rather than closed and `test/asm/minstret.S`, `test/csr_tb.v` and `components_traps` carry
that half.

**So do not read empty baselines or an all-green `make -C formal check` as "the core is correct"** —
an empty `formal/EXPECTED_FAIL` is necessary, not sufficient.

The SoC is 8 KB of ROM in block RAM plus 64 KB of data RAM, which yosys maps to two of the part's
four `SB_SPRAM256KA` at 256 kbit each. **128 KB is the up5k's whole SPRAM, not the SoC's.**
`SOC_EXPECT_SPRAM` and `SOC_EXPECT_EBR` hold both counts exactly. It places, meets 12 MHz, and runs
a program that reads its own `.data`. SPRAM still cannot be initialised, so `.data` rides in the
ROM at a load address `test/asm/boot.lds` puts there and `test/crt0.S` copies into RAM before
`main`. That runtime costs 82 bytes and `test/asm/datainit.c`'s whole ROM image is 284 of 8192 — a whole
Dhrystone is 3568 of it — so SPI-flash boot stays deferred, alongside the radix-4 divider. Two things have come off that list:
the machine timer is built (ADR-0082), and the forwarding network is priced and declined rather than
pending (ADR-0083). An interrupt controller, more sources and a vectored `mtvec` are still on it.

The suite is `test/asm/*.S` **and** `test/asm/*.c`, and `test/OBSERVED_FLOOR` names both. Anything
under `test/bench/` is deliberately outside it: both legs glob `test/asm`, and a benchmark that
needs two million cycles would time out against the runner's 5000 and be graded as a failure. The two
shapes differ only in how `.data` reaches RAM: an assembly program's is poked in by the harness,
which is the thing the hardware cannot do, and a C program's is copied by the startup, which is the
thing it can. A change to one program shape's build is a change in three places —
`test/run_tests.sh`, `test/cosim.py`'s `assemble()` and the Makefile's `soc-rom`.

## Pointers

- Decisions: [`docs/adr/`](docs/adr/) — re-derive the count by listing the directory; this file
  has been behind on it three times by quoting a number.
- Briefs: [`docs/ideas/`](docs/ideas/) — the rewrite plan, the fit brief, the four-goals brief, the
  one-address-space brief. Where a brief and an ADR disagree, the ADR wins.
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic).
- Work is tracked in Linear, project **Little CPU** (team JEF) — named so you know where the queue
  is; nothing in this repo depends on it.
