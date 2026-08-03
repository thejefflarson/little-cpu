# Little CPU

A hobby RISC-V core in SystemVerilog on the open toolchain (Yosys / iverilog / SymbiYosys —
no vendor EDA). Target **RV32IMC_Zicsr**, machine mode only; eventual home is an ice40 up5k.

**This project optimizes for readability. That is not a trade against speed, and the two are not
opposed by default.** The clearest version of a thing is often the fastest and the smallest:
ADR-0054 replaced twelve scattered `pc <=` writes with one priority chain and got a more readable
decoder, **312 fewer logic cells** and no time cost, all from the same edit. Reach for that shape
first.

Where they genuinely conflict, readability wins — but **measure the conflict, do not assume it**,
and do not leave a measured win on the floor because it sounds like an optimisation. What is
actually forbidden is named and narrow: **no flush logic, ever** (invariant 1) and **no forwarding
network without a new ADR** (invariant 4). Everything else is open to a change that reads at least
as well and is measured against `make fit` and `make soc-timing`.

This line used to say CPI was deliberately sacrificed and that the design must not be "improved" by
adding machinery. That framing made every speed question look like a violation, which is how the
operand-fetch cycle went a long time without anyone asking whether it had to cost what it costs.

## Current state — read this before believing anything else

**This is a half-finished rewrite. As of `a4662a2` it is a rewrite that computes correct
results — but it is still unverified.** A serialized FSM core (`rtl/riscv.v` + `rtl/alu.v`) once
passed riscv-formal sans CSRs; it was torn down in two waves (2021: `9758a39`→`1709433`; 2023:
`49b317a`→`4fbd650`→`13fec44`) into today's staged design, and the rewrite stalled partway. The
project went from *formally verified* to *unverified*, and getting back is the whole plan. **M1 is
green; M2's blocker (RVFI) is cleared but M2 itself is not reached.** Do not read "47/47" as "the
core is correct."

The README is the project's voice and is deliberately left as-is. **Ground truth lives here.**

**M1 is reached** (`a4662a2`). `rtl/regfile.v` was combinational-read with write-through
bypass (it is synchronous-read as of ADR-0042 — see below), every inter-stage struct carries a `valid` bit, and decode runs a stall-only hazard
scoreboard (ADR-0004 / ADR-0009 / ADR-0015). `make test` was **47/47** at that commit with
`test/EXPECTED_FAIL` empty (it is seeded with M3 debt now — see below); ADR-0014's set-equality
check runs in both directions, so an unexpected *pass* is caught too. The same change fixed three
datapath defects found on the way:
AUIPC computed `reg_rs1 + reg_rs2` instead of `pc + immediate`, `mem_wdata` was registered a cycle
behind `mem_addr`/`mem_wstrb`, and SLL/SRL/SRA did not mask the shift amount to `rs2[4:0]`.

**`b2dafcc` lands RVFI and makes the monitor live in both sim legs.** `rtl/structs.v` carries an
`ifdef RISCV_FORMAL` shadow payload (insn, pc, rs1/rs2 addr+rdata) captured in decode and forwarded
stage to stage riding the existing valid-bit protocol; `rtl/accessor.v` adds the mem
addr/rmask/wmask/rdata/wdata capture (the one stage that actually knows those values, including the
one-cycle load-response latch); `rtl/writeback.v` drives `rvfi_valid`/`rvfi_order`/etc. from the
retiring `accessor_output`, exactly where invariant 3 below says retire happens. `trap`/`halt`/
`intr`/`mode`/`ixl`/the CSR fields were hardwired constants in `rtl/littlecpu.v` at that commit,
because no traps and no CSRs existed yet; only `halt`/`intr`/`mode`/`ixl` still are (the CSR fields
came alive with `rtl/csrs.v`, `rvfi_trap` with trap entry — both below). `test/monitor.v` stays pristine and tracked; a gitignored, build-time-derived
`test/monitor.sim.v` (`test/sanitize_monitor.py`) carries the build-time repairs it needs —
originally two, stripping the `$time`-in-`$display` yosys can't elaborate and ADR-0019's DIV/REM
signedness fix, now three with the `!spec_trap` gate below — and **both** legs read it, so they
cannot drift into checking different specs. `make test` is now per-retire
self-checking in both legs, not merely end-state-checking via `tohost`.

**The generated monitor's DIV/REM spec model is defective, and the sanitizer fixes it**
(ADR-0019). `monitor_insn_div`/`monitor_insn_rem` compute signed division as a conditional whose
other branches are unsigned; per IEEE 1800 sign-context rules that propagates down and evaluates the
division **unsigned**, silently, for negative operands only. Confirmed by driving the two modules
directly with known-correct RVFI values (`-7 / 2` → `0x7ffffffc`, not `0xfffffffd`) and by the
`A_SIGNED 0` parameter yosys puts on the `$div`/`$mod` cell — not a simulator quirk, and not this
core: with the sanitizer rule in place `div.S`/`rem.S` pass. `test/monitor.sim.v` rewrites the two
sites to make the arithmetic self-determined. **`test/EXPECTED_FAIL` is not the place to park a
monitor defect** — read ADR-0019 before adding a line back for one.

**That defect is a repo-wide hazard, not a monitor bug, and it has now caught work here twice** —
the generated monitor's DIV and REM spec models above, and then `test/exec_tb.v`'s new SRA
reference, whose natural one-line form
(`cond ? ... : $signed(a) >>> sh`) reported six mismatches against RTL that was correct. **Any
signed arithmetic written into a *reference model* in this repo must be a self-determined statement
of its own, never an arm of a conditional expression.** `exec_tb`'s shift references are written
that way with the reasoning in place, and a `ref_selftest` block pins all three against
hand-computed literals *before* a single RTL vector runs, so a reference that degraded would say
`ORACLE BROKEN` and stop rather than blaming the core. An oracle that is wrong is worse than no
oracle: it fails correct hardware and teaches the reader to distrust the bench.

**ADR-0003's dual-word combinational fetch window lands, closing the gap ADR-0021 found.**
`rtl/fetcher.v` now reads two adjacent words every cycle (`imem_addr`/`imem_addr2 = imem_addr + 4`)
and windows the 32 bits starting at `pc` out of them — stateless, so a 32-bit instruction straddling
`pc % 4 == 2` costs nothing (no aligner FSM, no stall, no flush; invariant 1 below stays intact).
`rtl/decoder.v` also now masks `instr[31:16]` to zero whenever `quadrant != 2'b11`, the defence
behind the immediate-mux fix ADR-0021 already landed. `test/run_tests.sh` assembles
`-march=rv32imc_zicsr` (ADR-0014's sunset condition, now met): the assembler freely compresses
eligible instructions throughout the whole 49-file suite — the C.JR/C.JALR fix and this window are
no longer verified only by the formal ladder — and two new files target the window directly:
`test/asm/straddle.S` (a straight-line and a branch-redirected straddle at `pc % 4 == 2`) and
`test/asm/rvc.S` (riscv-tests' own RVC corner-case suite). All 49 pass, `test/EXPECTED_FAIL` stays
empty, and the per-retire monitor confirms real compressed retires in both sim legs (`rvfi_insn` a
zero-extended 16-bit value, `rvfi_pc_wdata` stepping by 2) rather than merely that the assembler
happened not to compress anything. `formal/imemcheck.sv` — the check that models fetch at 16-bit
granularity, per ADR-0003's own consequences section — is re-pointed at the split
`imem_addr`/`imem_addr2` interface and still passes. `misa`'s C bit is no longer an untested claim.

**The harness can trap and recover, and the monitor no longer lies about trapping retires.**
Three things landed together, all of them test-side — no `rtl/` file changed. (1)
`test/asm/riscv_test.h` gains **opt-in** trap macros: a `.align 2` handler that records
`mcause`/`mepc`/a trap counter into RAM and resumes at `mepc+4`, a fatal variant that fails the
test from inside the handler, and an installer for `mtvec`. `RVTEST_CODE_BEGIN` is byte-identical
(verified with `gcc -E`), so the 49 existing tests do not start executing CSR instructions before
the CSR RTL exists. The handler **cannot read the faulting instruction** — Harvard buses,
ADR-0008 — so it cannot tell a 2-byte fault from a 4-byte one, and every trapping instruction in a
resuming test must be wrapped in `.option norvc`. (2) The sanitizer moved from an inline `sed`
chain to `test/sanitize_monitor.py` and gained a **third rule**: gate the monitor's spec-value
checks on `!ch0_spec_trap`, mirroring riscv-formal's own `checks/rvfi_insn_check.sv`, which the
monitor generator never emits. Without it a misaligned `lw` — where the spec model reports the
loaded value and `pc+4` while a correct core writes nothing and redirects to `mtvec` (ADR-0028) —
reports errors 104/105/106/111-113 in both sim legs **on correct hardware**. Same situation as
ADR-0019's DIV/REM defect and the same remedy; the script now also asserts a site count per rule,
so a pin bump that changes the generator fails loudly instead of silently shipping an unapplied
sanitizer. (3) `test/monitor_tb.v` joins `make test-units`, driving the sanitized monitor directly
with hand-built trapping and non-trapping RVFI vectors — including a deliberately wrong one, so a
gate that disabled all spec checking could not pass.

**A site count proves a sanitizer rule fired, not what it swallowed.** Rule 3 selects its ~45-line
span by *position* — first anchor (`rs1_addr`) to last (`mem_addr`) — so a pin bump that moved the
trap comparison (`handle_error(101, "mismatch in trap")`) between those anchors would keep the count
at 1, keep the diff at eight lines, and silently gate off the one assertion riscv-formal's
`checks/rvfi_insn_check.sv` deliberately keeps live under `spec_trap`. A core that failed to trap on
a misaligned load would then retire clean in **both** sim legs. So rule 3 now also asserts on
*contents*: two forbidden literals in the span, the exact multiset of `handle_error` codes it
encloses (derived from the current output — re-derive it after a pin bump, never edit it to silence
a failure), and a post-check that error 101 survives into the sanitized output at all. All three
were confirmed by mutation; the pre-change sanitizer accepted every one of them at exit 0.
`test/monitor_tb.v` carries the simulation-side half (vector 7: a retire claiming `rvfi_trap = 0`
where the spec model says it must trap — the existing wrong-retire control is non-trapping and
passes happily without error 101). `trap.S` sits in `test/EXPECTED_FAIL` as `MONITOR-ERROR 101`
today, which is that check doing its job against the missing M3 trap logic.

**`make -C formal genchecks-check` is the drift control for the repo's other vendored generated
file.** ADR-0031 permits `formal/genchecks-local.py` to differ from the pin's
`checks/genchecks.py` by this repo's header and `basedir` and nothing else — but that rule lived
only in a hand-run `cp` + `diff -u` recipe in the script's own header, which nothing enforced, and
unenforced drift there is exactly what produced the five-year-stale fork ADR-0031 exists to end.
`formal/check-genchecks.py` undoes those two documented edits and then requires byte equality with
the clone, so any residual diff is drift by construction and is printed. It runs in CI in the
`monitor-freshness` job, which has already paid for the from-scratch clone at the pin.

`test/EXPECTED_FAIL` was seeded with three M3 tests — `csr.S`, `minstret.S` and `trap.S` — written
ahead of the RTL that pays them off, which is the direction ADR-0014's burn-down contract was
designed for. All three are paid off now (the third by the trap change below), so it is **empty
again** and `make test` is **52 pass / 0 expected-fail**, exiting 0 only on set equality. That
equality is over
**name-and-status pairs** (ADR-0035) — the baselined entry is `trap.S      MONITOR-ERROR 101`, so a
baselined test that starts failing some *other* way (a broken assembly, a `TIMEOUT`, an unstartable
runner) is a red gate rather than a match. The same change made every build step's exit status
checked and `mktemp` fatal: an unchecked `objcopy` that emitted an **empty** RAM image used to make
`tohost` read zero and every data-independent test still report `PASS`.
`test/run_tests.sh` also grew a `MONITOR-ERROR` label for runner exit 4, which used to fall into
`RUNNER-ERROR` and read as "the sim would not start" when it actually means the per-retire oracle
disagreed with the core mid-run, and a `TRAP-TO-ZERO` label for exit 5 (ADR-0029, below).
**Nothing here checks M3 semantics against an oracle**:
riscv-formal ships no spec model for `csrr*`/`ecall`/`ebreak`/`mret` at the pinned SHA, so
`spec_valid` is 0 for every one of them and the monitor's whole semantic block is skipped. The
trap and CSR tests assert on values their own handler recorded. Error 133 ("expected intr after
trap") stays unreachable in this single-channel config — `shadow_pc_valid <= !rvfi_trap` gates 130
and 133 off for the retire after a trap — so `rvfi_intr` hardwired to 0 does not break anything.

**`make test` now measures that its oracle fired, instead of inferring it.** The monitor is the
gate's per-retire oracle and **nothing counted whether it ever looked**: `test/cxxrtl.cc` sampled
the errcode and counted nothing, so a monitor whose `rvfi_valid` never asserted — ADR-0037's
under-sensitivity class, an `ifdef` that dropped the shadow payload, a `write_cxxrtl` that optimised
the instance away — left all 52 programs PASSing off `tohost` alone, with an empty
`test/EXPECTED_FAIL` and so no red entry whose disappearance would say otherwise. `test/testbench.v`
counts retires (`rvfi_valid`) and spec-checked retires (`spec_valid`, from a **second
`monitor_isa_spec`** instantiated in the bench — `test/monitor.v` is generated-but-tracked and a
yosys hierarchical reference silently *implicitly declares* the name rather than resolving it, both
measured); the runner prints `RETIRES <n> SPEC-CHECKED <m>` and adds **exit 6** for zero of either;
`test/run_tests.sh` labels that `MONITOR-SILENT`, carries both counts as a third table column, and
grades them against **`test/OBSERVED_FLOOR`** — names by set equality both ways, numbers by `>=`,
because instruction counts legitimately move and a 52-number ratchet is one nobody would keep.
**The probe is one line and both directions are measured**: `assign rvfi_valid_observed = 1'b0;` in
`test/testbench.v` gives 52 × `MONITOR-SILENT` (`retires=0 spec-checked=0`) and exit 1 here, and the
same blinding on the pre-change tree gave **52/52 and exit 0** — every program still reaches
`tohost` and still passes its own assertions, which is the whole defect. `rvfi_valid_observed` is
the single wire the monitor, the probe and the counters all read, so they cannot go blind
independently. **A low spec-checked count is the pin's coverage boundary, not a bug** — no spec
model exists for `ecall`/`ebreak`/`mret`/`csrr*`, so `csr.S` is 108/82, `minstret.S` 42/31 and
`trap.S` 303/259; that is written at `test/OBSERVED_FLOOR` so nobody has to rediscover it.

**Both sim gates now assert what the suite CONTAINS, before either runs a program.** `make test`
verified that every program it *found* passed; it had no idea how many it should find, and with
`test/EXPECTED_FAIL` and `test/COSIM_EXPECTED_FAIL` both empty there was no red entry whose
disappearance would say the suite had shrunk — a bad rebase or a glob that stopped matching would
print `12/12 passed`, match the baseline exactly and exit 0. **No new file was added**:
`test/OBSERVED_FLOOR` already names every program, so it *is* the manifest, and
`test/check_suite_shape.sh` enforces its name set in both directions from both runners. Measured on
a throwaway branch: deleting `add.S` and `lw.S` made the **pre-change** `run_cosim.sh` report
`50/50 agreed` and **exit 0**, and the pre-change `run_tests.sh` catch it only after printing
`50/50 passed`; both now stop naming the two programs with nothing run. Adding an unlisted `.S` is
red in both, and the pre-change co-sim leg reported `53/53 agreed`, exit 0. `make test-units` gets
the same treatment: the six bench invocations were spelled out with nothing tying them to
`test/*_tb.v`, so a seventh landed unrun and the gate reported six green — `UNIT_BENCHES` is now
compared against the tree in both directions **and drives the recipe**, so a declared bench with no
`UNIT_BENCH_SRC_*` is red too rather than building with no design under test.

**The SoC places, and this project has a real Fmax for the first time** (ADR-0054). `rtl/imemory.v`
and `rtl/memory.v` were placeholders ADR-0044 recorded as unbuildable in four ways, reachable from no
simulation at all. They are real now: the ROM is two **word-interleaved** banks in block RAM (banking
replaces the ROM duplication `littlesoc.v` used for ADR-0003's second fetch port — ADR-0044 names the
technique at halfword granularity, which is wrong for a fetch interface that asks for two adjacent
*words* and windows them itself), and the data RAM is shaped so yosys infers `SB_SPRAM256KA`. **The
load-bearing line in `rtl/memory.v` is that its read port is NO-CHANGE on a write cycle**: the
read-first spelling maps the same array to 148 `SB_RAM40_4K` — five times the part's block RAM —
silently, failing later in nextpnr with a message about BELs. Measured both ways.
`make soc-timing` reports, at Homebrew yosys 0.67+post with the machine at load ~3:
**4041/5280 LC (76%), 20/30 EBR, 2/4 SPRAM, 4/39 IO, 8/8 GB**, and
**88.51 ns = 11.30 MHz — 34.20 ns logic (38.7%) + 54.29 ns routing (61.3%), 41 logic levels**, on
`imem.in_range → decode → next PC → imem.in_range2`. That is the loop invariant 1 puts in one cycle,
measured rather than argued. **The design did NOT meet ADR-0038's 12 MHz at that commit**, and the
12 MHz declaration was deliberately left where it stood. It is met now, and 12 MHz is a
**requirement** rather than an intent — see the bypass change below and ADR-0066.
**Two findings came out of the first run and are fixed**:
the path started at the `btn_n` *pad* (the button is synchronised now) and ended in a 32-bit
incrementer feeding the ROM's range check (both range tests compare a word index against a constant
now). 9.60 → 11.30 MHz for those two. **The SoC would still not RUN a program in `test/asm`**: SPRAM
cannot be initialised, so `.data` is not there at power-on and a copy stub or ADR-0044's flash boot
path is unbuilt.

**Fetch is still combinational and there is still no flush; the address is published a cycle early**
(ADR-0054, adopting ADR-0044's recommendation). `rtl/decoder.v`'s scattered `pc <= ...` writes are
one combinational priority chain, `next_pc`, in the same order last-write-wins gave them; the memory
latches `imem_addr_next = {next_pc[31:2], 2'b00}` on the same edge `pc` takes it. **No extra cycle,
no stall reason, no speculation** — a stalled cycle sets `next_pc = pc` and the memory re-presents the
same words. Two side effects worth knowing. `out.rvfi.pc_wdata` is assigned from `next_pc` once
instead of being a second hand-maintained copy of every redirect. And **`make fit` went 4187 → 3875**
(312 cells, six times the ±50 churn floor, both measured under the same local yosys): six independent
`cond ? pc+imm : pc+inc` writes to `pc` became one `branch_taken` and one priority mux. `FIT_MAX_LC`
is 4100 now. **Nothing on the riscv-formal ladder is in contact with the new port** — `wrapper.v`
answers `imem_data` combinationally against `imem_addr`, unchanged — so the lockstep is asserted in
`formal/pcloop.sv` (on the composed loop), in the decoder's own `FORMAL` block, and in
`test/decoder_tb.v`, each probed by mutation.

**The memories are on the simulator's dependency graph now, and that is most of what makes the above
real** (ADR-0054). `test/testbench.v` instantiates `rtl/imemory.v` and `rtl/memory.v` rather than its
own inline arrays, so all 56 `.S` programs and the whole Sail co-simulation run through the
synchronous banked ROM and the SPRAM-shaped RAM — a broken fetch lockstep would have had every
program executing the wrong instructions, and nothing in the tree would have noticed. The runners
de-interleave the `--rom` image across the two banks. `test/imem_tb.v` joins `make test-units`
(**seven** benches) and walks every word index at both parities against a **flat** reference, kept
deliberately independent of the bank split.

**Text is writable, the steal reaches decode, and `fence.i` stopped being free** (ADR-0059,
ADR-0060, ADR-0061). `rtl/imemory.v` owns the range decode, the write port and the arbitration;
`rtl/accessor.v` exports `mem_ren` (`!reset && in_valid && is_load`) and `rtl/decoder.v` takes
`fetch_stall` as the **sixth stall reason** on the existing bubble mechanism — `pc` holds, `out <=
'0`, no flush, nothing issued. `mem_ren` is not cosmetic: the idle bus presents address 0, which is
inside the text range, so without it every idle cycle would steal a fetch and the core would never
run. **`fence.i` now serializes** on invariant 5's drain predicate, because the fetch address is
published a cycle early (ADR-0054) — a store at cycle D writes the array at edge D+2 while the
instruction after a `fence.i` at D+1 was fetched at edge D+1, so it would execute stale text after
`fence.i` retired. All four in-flight slots have to be empty for that argument to hold, and
`accessor_out.valid` is the slot that carries a store (ADR-0026). **Three ladder depths moved with
it** — `insn 19`, `ill 19`, `reg 15 22` — off a re-measured **F = 6, G = 4 → 6**, and the ladder's
wall time roughly doubled. Two things about the change are worth knowing before touching it. The
**coincidence of `fetch_stall` with a divider or accessor freeze is a ruling, not statement order**:
holding wins, and it is asserted in two places because reordering two `else if` arms is otherwise
silent. `formal/arbiter.v` is the steal equation written once, and all five harnesses instantiate it — so
`fetch_stall` is a free input nowhere and a constant nowhere. A free stall input starves `hang` and
`liveness`, which was measured, not argued.

**Three programs exercise all of that end to end now, and the suite is 59** (ADR-0064).
`test/asm/selfmod.S` stores into `.text`, fences and runs the stored word; `test/asm/textload.S`
carries a handler that reads the faulting instruction out of `.text`, works out two bytes or four
and resumes past it — so none of its four trapping instructions is wrapped in `.option norvc` and
two of them are `c.ebreak`, which is the discipline that hid ADR-0048's defect no longer being the
only way to write a resuming trap test; `test/asm/contend.S` puts loads and stores in `.text` at
both word-address parities with byte and halfword strobes. **Twenty-six stolen fetch windows across
the three (6 / 4 / 16), against zero anywhere in the tree before**, counted with a throwaway
`fetch_stall` counter. `make test` is 59/59 and `make cosim-suite` is 59/59, both baselines empty,
and `test/sail/rv32imc_zicsr.json` needed no change — its one `MainMemory` region is already
executable, readable and writable across the megabyte. **The one thing to know before editing
`selfmod.S`**: test 2's store has to be `sw t1, %lo(patch_adjacent)(x0)`, because `fence.i` waits
for its own rs1 field to have been presented and so can only issue one cycle behind a store that
also presented `rs1 = x0`. Written the way anyone would write it (`la t0, site; sw t1, 0(t0)`) the
program **passes with the serialization term deleted** — measured, both ways. `.data`'s load address
and the crt0 copy loop are deliberately not in that change; the SoC still cannot run a program that
reads its own `.data`.

**The design reaches 12 MHz, and the whole change is six lines in `rtl/regfile.v`** (ADR-0064,
implementing ADR-0062). The write-through bypass compared `waddr` against `rs1`/`rs2` — instruction
bits out of the word the fetch just returned — so the bypass mux and the four branch magnitude
comparators behind it sat **after** the instruction rather than beside it, and every branch paid for
the whole chain before the next PC could be chosen. It compares against a **registered copy** of the
address pair now. Measured on this tree, four placements each: **95.74 – 99.47 ns (10.05 – 10.44
MHz) → 74.34 – 78.80 ns (12.69 – 13.45 MHz)**, 29 LUT levels → 23, distributions not overlapping and
the move six times the 3.6% edit band. `SOC_MIN_MHZ` went **10.0 → 10.9** there and is **12.0** now
(ADR-0066). `make fit` is 3899 → **3880**, inside the ±50 churn floor and meaning nothing; the SoC
top's LC is 4314 → 4383. **12 is the board crystal, not a round number** — the part's oscillator
divides 48 to 48/24/12/6, so the step below 12 is 6, and missing it costs half the clock rather than
a percentage.

**So 12 MHz is a REQUIREMENT, and `SOC_MIN_MHZ` stopped being a regression floor** (ADR-0066). A
regression floor sits under the current measurement and slides every time the design moves; a
requirement sits at the number the hardware asks for and does not move at all. 12.0 leaves **5.75%**
against the worst local placement (12.69 MHz), which is tighter than any other ratchet here and
against a 3.6% edit-churn band — accepted deliberately, because a design at 11.9 MHz has stopped
meeting its requirement and the honest report of that is a red gate. **When it trips, the fix is the
design, not the floor.** `soc-timing` is a **required** check on `main` as of the same decision, and
CI measured 12.72 MHz on its own hardware.

**What that change costs is a coupling, and it is the part to remember.** The bypass used to be
correct for *any* address pair; it is now correct **because** `operand_stall` guarantees that on an
issuing cycle the held pair is the presented pair. Invariant 6 therefore depends on invariant 9, and
narrowing `operand_stall` would leave the bypass answering with the previous instruction's operand
with nothing to say so. Two `test/regfile_tb.v` vectors drive the presented address away from the
held one with a write in flight to the held one — the only situation the two spellings disagree
about — and both go red if either select is reverted; every other vector in that bench holds the
pair and cannot tell them apart. **`make test-units` caught exactly one existing assertion**, `x0
reads 0 in the fetch cycle (rs1)`, and rewriting it was part of the change rather than a workaround:
the x0 test keys off the held address like every other read, so the cycle that first presents x0
still answers the address held from the vector above it. Nothing in the core reads an operand in a
fetch cycle, so that bench is the only consumer in the tree that can see it. `reg_ch0`'s liveness
probe was re-run — delete the rs2 write-through bypass, `bad state property 1 reachable at bound
k = 22 SATISFIABLE` — because a change to the regfile's forwarding is exactly the class that could
make that check go green by stopping to ask.

**Half the suite's cycles issue nothing, and the decode scoreboard is 57% of them** (ADR-0070).
`make cycles` charges every simulated cycle of the same 59 programs to an issuing cycle or to one of
the decoder's six stall reasons, read as the named signals `rtl/decoder.v` drives; **no `rtl/` file
changed**, because those signals survive `write_cxxrtl` as debug items. Measured at `0314890`:
**28632 cycles, 13853 retires, CPI 2.07**; issue 49.0%, decode scoreboard **29.3%**, operand fetch
**16.4%**, divider 2.8%, serialization 1.5%, load turnaround 1.1%, stolen fetch window 0.03%,
**unattributed 0**. ADR-0042's accepted +18.0% is *located* rather than inferred, and the three
reasons that get argued about most — `fence.i`/CSR serialization, the load turnaround, the stolen
window — are 5.4% between them. **A column is cycles CHARGED, not cycles the signal was high**:
coincident reasons go to the first one the decoder itself would try, so `fetch_stall` is high on 26
cycles across the three writable-text programs and is charged 8. Read ADR-0070 before quoting any of
this; `test/decoder_tb.v` carries the identity that keeps the six-reason list from falling behind
`stall`.

**Every graded comparison in the grading layer now has a probe of its own red direction, and two of
them had never been executed** (ADR-0053). Five of this repo's recorded defects were in that layer
and every one was in a *script*; the class is the comparison whose failure path was never run, and
reading the script is what missed all five. `test/probe_gates.sh` forces **108** of them to fail and
requires each to fail *for the reason it was written for* — exit status **and** a fragment of the
diagnostic, with a control per group that must exit 0, so a grader degenerated into `exit 1` cannot
pass. It is hermetic (no toolchain, no Sail, no yosys, no sby — the failing `objcopy`, the
unstartable runner and the diverging reference model are stubs) and hangs off `make test`, which
puts it in CI's required `test` job with no workflow change. Two findings came out of it, both
fixed with the red demonstrated: **`formal/check-baseline.sh` read an *unreadable* baseline as an
empty one** — `chmod 000 formal/EXPECTED_FAIL` printed "Failure list matches ... exactly" and exited
0, swallowing an unexpected pass as well (ADR-0035 item 4's fix, never carried to the formal side);
and **`test/cxxrtl.cc`'s exit 5 was unreachable in its own scenario** — traps commit in decode while
retires happen in writeback, so a fault in the first instructions of `_start` raised `trap_to_zero`
with `RETIRES 0` and the silence gate turned it into `MONITOR-SILENT`, which is exactly the
misattribution ADR-0029 added `TRAP-TO-ZERO` to prevent.

**ADR-0053 reached every script; four graded ratchets living in the Makefile itself were still
untouched, and one of them had never been run on CI at all.** `make fit`'s `FIT_MAX_LC` ratchet,
`make soc-timing`'s `SOC_MIN_MHZ` ratchet, the SPRAM/EBR census `soc.json`'s recipe runs, and
`check-unit-benches`' list equality are each a graded comparison in the same sense ADR-0053's
audit covers, and none had its red direction exercised. Two of the four were `awk`/`grep` embedded
in a recipe rather than a script, so probing them meant extracting them first: `soc/fit_report.py`
now carries `make fit`'s table-presence check and ratchet, and `soc/cell_census.py` replaces the
`soc_expect_cells` `define` — both callable against a fixture log, both wired back into the
Makefile so there is one copy of the logic. `check-unit-benches` needed no extraction: its three
branches are probed by invoking the real target against the real `test/*_tb.v` tree with
`UNIT_BENCHES`/`UNIT_BENCH_SRC_*` overridden on the command line, which is what its own comment
already commits to being possible. `test/probe_gates.sh` is **137** probes now, up from 108
(ADR-0058 added three for `soc/timing_split.py`'s LUT-versus-carry depth split; ADR-0070 twelve for
`test/stall_report.py` and for `run_tests.sh` driving it).
**`make soc-timing` joins CI in the `soc-timing` job**, mirroring `fit`: non-required at the time,
for the same reason (ADR-0036) — it publishes the census and the logic/routing split to the step
summary whether it passes or fails. **It is required now** (ADR-0066): the floor it grades is the
board clock, so a red there means the design does not run on the part. It is the first job needing
both the RISC-V cross compiler and the OSS CAD
Suite, so the toolchain assertion moved into `.github/actions/verify-toolchain` rather than being
copied from the `test` job — the duplication shape that broke `elaborate` once already.

**M3 opens: `rtl/csrs.v` lands the CSR file and the Zicsr access path.** ADR-0005's set, exactly —
RW `mstatus` (MIE/MPIE, MPP WARL→`2'b11`), `mtvec` (direct mode, 4-byte-aligned base, resets to 0
per ADR-0029), `mepc` (bit 0 only, because C makes 2-byte targets legal), `mcause`, `mscratch`,
`mcycle`/`mcycleh`, `minstret`/`minstreth`; RO `mtval`/`mie`/`mip` = 0, `misa` = `0x4000_1104`,
`mvendorid`/`marchid`/`mimpid`/`mhartid` = 0. It is a **sibling of the decoder, not a pipeline
stage**: every access is read and committed in decode on the edge the accessing instruction issues,
so no CSR state exists downstream and the read result rides the `is_add` pass-through `lui`/`jal`
already use. `rtl/decoder.v` separates `csrrwi`/`csrrsi`/`csrrci` from the register forms (they were
folded together, losing the zimm-vs-rs1 distinction), excludes the immediate forms from `uses_rs1`
and from `rvfi_rs1_valid`, and folds serialization into `hazard` per ADR-0026 — with
`accessor_out.valid` newly routed in, because a store in flight is invisible to the other three
slots. `minstret` increments at issue, gated on `instr_valid`, which is ADR-0027's non-trapping rule
written now rather than retrofitted. **Nothing reads `mtvec`/`mepc`/`mcause` yet** — trap entry is
deliberately the next step (ADR-0011) — and an unimplemented CSR stays an *unrecognised
instruction* via the existing `instr_valid` path rather than raising a trap, which is exactly what
it did before this file existed.

`csr.S` and `minstret.S` came out of `test/EXPECTED_FAIL`; `test/csr_tb.v` joins `make test-units`.
On the ladder, **`csrw_mcycle_ch0` and `csrw_minstret_ch0` went FAIL → PASS** and came out of
`formal/EXPECTED_FAIL` in the same commit, and `formal/checks.cfg`'s `[csrs]` gained **`mscratch`
only** — 78 checks became 79, `csrw_mscratch_ch0` passes. `mtvec`/`mepc`/`mcause`/`mstatus` are
deliberately **not** on that list and must not be added: `rvfi_csrw_check.sv` has no WARL model, so
a correctly masked WARL CSR fails there **on a correct core** (the reasoning is written out next to
the `[csrs]` list). **That WARL bar binds `csrw` only** — the `csrc` models all take a
`RISCV_FORMAL_CSRC_MASK`, so a WARL CSR *can* be stated to one. What keeps those four (and the
read-only CSRs) off the list is different and is not a `checks.cfg` question: `rtl/csrs.v` exports
RVFI shadow payloads for exactly three CSRs, and a name here generates its `csrw_*` check whether
asked for or not. Those four are checked field-by-field in `test/csr_tb.v` instead. Note also that
`genchecks` defines `RISCV_FORMAL_CSRWH` for `mcycle`/`minstret` by itself, so the `h` halves are
exercised whether or not `checks.cfg` asks for it.

**The regfile read is synchronous now, the core's logic fits the up5k, and the ladder gained an
assumption** (ADR-0042). `rtl/regfile.v` is two block-RAM arrays (an ice40 EBR has one read port, so
a second read port is a second copy — `yosys` infers `4 x SB_RAM40_4K` with no attribute), posedge
write, **posedge registered read**, with two forwarding points left in fabric: write-first into the
read register, and the existing write-through bypass. `rtl/decoder.v` gains `operand_stall` — present
the address pair, bubble, issue — which is a fifth stall *reason* on ADR-0009's existing bubble
mechanism and adds **no flush** (the PC simply holds, so invariant 1 holds by construction).
**`make fit` goes 6971/132% to 4236/80%**, a reduction of 2735 logic cells, and becomes a ratchet.
The price is **+18.0% cycles across the 52-program suite**, measured — CLAUDE.md's fourth line being
cashed. The stall is gated on `uses_rs1`/`uses_rs2`, the scoreboard's own predicates: ungated it
costs +27.8%, so the gate recovers 35% of the penalty for one extra term.

**Two things about that change are non-obvious and are why it has an ADR.** First, `formal/wrapper.v`
now assumes **instruction memory is a function of its address** (same address as last cycle ⇒ same
data). Without it `hang` and `liveness_ch0` produce real counterexamples at k=30, because decode
decides whether the fetched operand belongs to the instruction it is issuing by comparing
`rs1`/`rs2`, which are combinational out of `imem_data` — so a free `imem_data` lets the environment
starve forward progress forever. That port did not exist before this change, and `wrapper.v`'s
`RISCV_FAIRNESS` comment (which said no such port existed) is corrected in place. `imemcheck.sv`
already relied on the same fact in a stronger form. Second, **the negedge alternative was rejected
despite being cheaper and free in cycles**: sby's `prep` fails closed on mixed clock polarity, so the
generated ladder cannot run against it *at all* — not one red check, all 82 — and `clk2fflogic` stays
rejected (ADR-0040). Serialising the two read ports onto one array was also built and rejected: 44/52,
and it needs a second bypass level, which is the first step toward the forwarding network invariant 4
forbids.

What does not work right now — **one live entry, then six resolved ones kept in place.** Five of the
six are struck through, and every one of those outlived its own fix here by several commits. That is
the failure this section exists to prevent, so they stay as markers rather than being deleted.

- **Three things this repo treated as gates were reached by no automation, and one of them computes
  a verdict nothing reads. Two of the three are now fixed; the other one stands.** Found by reading
  `Makefile`, `formal/Makefile`, both workflows and
  `test/run_tests.sh` end to end; the four-column inventory that came out of it lives in the pull
  request that added this bullet and is destined for the coverage-map ADR. (1) ~~**`make fit` is a
  ratchet nothing pulls**~~ — **fixed: it is a job now** (ADR-0052). `ci.yml`'s `fit` job runs it on
  every PR, **non-required on purpose** — area is a design constraint, not a correctness one, and
  branch protection is human-only (ADR-0036). Probes, both run: `make fit FIT_MAX_LC=4100` exits
  **2** ("logic cells is over the 4100-cell budget"), and a nextpnr that prints no utilisation table
  exits **1** rather than reporting a 0% fit.
  **Putting it on CI found that the number is toolchain-dependent, which nobody had measured.**
  Same commit, same RTL: **4208 cells / 79% under CI's pinned OSS CAD Suite** (Yosys 0.66+179,
  `e74db6dea`) and **4187 under a local Homebrew Yosys 0.67+post** — 21 cells apart on nothing but
  the synthesiser build. The churn floor under Pointers was characterised as instability across
  *edits*; this is a second axis. **Quote the pinned number, 4208** — the `fit` job is where it is
  measured, and the pin is the reference for the same reason `formal/pin.mk` exists. The 4236 this
  file carried from ADR-0042 was a local measurement too, so the apparent 28-cell "reduction" is
  two toolchains being compared, not cells being saved.
  (2) ~~**`make waves` grades nothing.**~~ — **fixed, on the artifact rather than the target**
  (ADR-0055). `ch0_handle_error` still only `$display`s — there is still no `$fatal`, `$finish` or
  `$stop` anywhere in `test/monitor.v` or `test/monitor.sim.v` — but `test/testbench.v` now checks
  `rvfi_monitor_errcode` itself, every cycle (it is a one-cycle pulse, so a once-at-the-end read
  would miss it), and adds an end-of-run floor on memory writes and RVFI retires: fewer than 15
  writes or 60 retires over the baked-in 200-cycle program is `$fatal(1)`, matching
  `test/cxxrtl.cc`'s exit 4. **`ci.yml`'s `elaborate` job now runs `vvp testbench.vvp` and grades
  its exit status** — the same step that used to build it and stop. `make waves`'s recipe
  (`vvp $< && mv testbench.vcd $@`) inherits the same three failure paths as a side effect of
  sharing the binary, not because it was made to grade anything on purpose.
  **This is one fixed program, not the deferred multi-program `.S` runner** — `test/testbench.v`
  still has no image loader, so the coverage gap the design brief defers stays open; only the
  baked-in loop is graded. All three red directions were demonstrated on real runs and are tabled in
  ADR-0055: reverting the hazard scoreboard's two continuous assigns to ADR-0037's
  `function automatic live_producer(r)` form gives 0 writes / 1 retire against the 15/60 floor; a
  `+1` offset on the accessor's `rvfi_mem_wdata` shadow (real memory unaffected) trips the errcode
  check; and a broken ADD trips the pre-existing `trap_to_zero` `$fatal`.
  (3) **`make -C formal all` is dead** — no workflow names it. That one is
  redundancy rather than a hole: every target it lists (`complete`, `check`, `dmemcheck`,
  `imemcheck`, the three `components_*`) is invoked separately by `ci.yml` (ADR-0050 deleted the
  nightly and folded its checks into the `formal` job).
  A target that names seven gates and is reached by nothing still reads like coverage, which is why
  it is written down here rather than left to be rediscovered.
- ~~**The ALTOPS divide branch reads stale operands**~~ — **fixed, and this bullet outlived it by
  several commits.** The ALTOPS issue arm latches its operands (`rtl/executor.v:52-60`,
  `div_alt_rs1`/`div_alt_rs2` off `mul_div_x`/`mul_div_y`) exactly as the non-ALTOPS branch does, so
  the completion arm no longer reads an `in` that decode has bubbled. `insn_div_ch0`,
  `insn_divu_ch0`, `insn_rem_ch0` and `insn_remu_ch0` all PASS and none of them is in
  `formal/EXPECTED_FAIL`, which is empty. What is still true, and is the part worth keeping: under
  `` `RISCV_FORMAL_ALTOPS `` those four checks say nothing about the real divider's arithmetic
  (ADR-0010), and ADR-0045 decided that gap closes by naming `components_executor` +
  `test/exec_tb.v` as the oracle rather than by dropping ALTOPS.
- ~~**`formal/equiv.sh` runs but does not converge**~~ — **the file is deleted and the guarantee is
  proved another way** (ADR-0047). Measured: `equiv_make` leaves **459 of 495** `$equiv` cells
  unproven — `mem_addr`, `accessor.pending_*`, `executor.mul_div_*`, `regfile.wdata`, essentially the
  whole datapath — because it matches **by name** and the gate build optimises to a differently-named
  netlist; `equiv_induct` then diverges at **~660k clauses per step** (1.20M → 1.86M → 2.53M → 3.19M)
  and never returns. So ADR-0020's own remedy (blackbox the divider, or bound the miter) cannot work:
  the failure is matching, not the divider. It sat in the nightly behind `continue-on-error` +
  `timeout 3600`, which made a check that had **never produced a verdict** read as coverage.
  `make -C formal nonperturbation` replaces it and runs on the PR gate in ~9s: build with
  `-D RISCV_FORMAL`, `delete -port littlecpu/rvfi_*`, sweep, and require the result to be
  **structurally identical** to the plain build — cell histogram plus a name-independent connectivity
  fingerprint. **It is strictly weaker than sequential equivalence and must not be quoted as if it
  were**: it proves the instrumentation is *unread*, not that two designs behave alike. Both failure
  directions are demonstrated on real mutations (a shadow value ORed into decode's `stall`, +193
  cells; one gating the accessor's `out.valid`, **+11** cells).
- ~~**`reg` is inconclusive**~~ — **false since ADR-0024, and this bullet is where ADR-0037 caught
  itself repeating it.** ADR-0023 wrote it under `smtbmc yices`; the ladder moved to `btor btormc`
  and `reg_ch0` returns in seconds. ADR-0042 §3 re-measured it against the synchronous regfile at
  `PASS 0 31`, and ADR-0046 re-ran its probe against the five-reason pipeline: deleting the rs2
  write-through bypass gives `bad state property 1 reachable at bound k = 20 SATISFIABLE` at the
  shipping `reg 15 20`. It returns a verdict, and the verdict means something. What ADR-0023's
  sentence was reaching for is still true and lives in ADR-0032 instead: `reg_ch0` is the *only*
  check tying RVFI's self-report back to `rtl/regfile.v`, and an architectural write past its bound
  is invisible to the whole ladder.
- ~~**The formal nightly cannot go red**~~ — **fixed, and it was never the `|| true`** (ADR-0037).
  This bullet described `formal-nightly.yml` as `make -C formal check || true` with no `-k`; both
  were fixed earlier, and the bullet outlived them. The real defect survived both fixes: the graded
  `check-baseline.sh` call was piped into `tee`, and a `run:` block without an explicit `shell:` key
  is `bash -e {0}` — errexit but **not** pipefail — so the step's exit status was `tee`'s and was
  always 0. **ADR-0022's central guarantee had never held.** `1961234` fixed both copies of the step
  (nightly and the new PR-gate `formal` job) and demonstrated both failure directions on real runs.
- ~~**`components_executor` is green and does not check the multiplier's sign logic, its high half,
  or the signed divider at all**~~ — **fixed** (ADR-0051 closing ADR-0049 F1/F5). The divide
  invariant's two *unguarded* `always_comb assume(in.rs1 <= 32'h0000000f)` statements were
  proof-global, so with operands in 0..15 every high half and every sign bit was zero and six
  named defects PASSED the task. The cap is guarded to the divide family now and bounds `div_x`/
  `div_y` — the ADR-0012 magnitudes — so DIV/REM operands range over -15..15 with the **sign free**.
  The full-width multiply miter is gone and is not coming back: **a standalone miter of the
  multiplier against its reference, with no divider, no pipeline state, no cap and `mode bmc depth
  1`, returns no verdict in two minutes** — the obstacle is two structurally distinct `bvmul` terms,
  and neither bit width nor engine moves it. What replaces it is a decomposition (the 33-bit
  operands, the retired slice of one shared product term, three constant-multiplication lemmas over
  that term) plus signed DIV/REM completion assertions. **Eleven mutations are caught by both
  oracles**, tabled in ADR-0051 — and the two things worth carrying out of that table are that
  lemma 3 is the only thing that catches a masked high half, and that a broken *completion*
  assertion reports `UNKNOWN rc=4`, not `FAIL`, because the guard is basecase-unreachable at depth
  20 against a 33-cycle divider (ADR-0049 F3). The task went 6.5s → 50.3s.
- ~~**Three of the five component-proof tasks are vacuous**~~ — **they were deleted; there are
  three tasks now and all three assert something.** `formal/components.sby` carries `decoder`,
  `executor` and `pcloop`; `fetcher`, `accessor` and `writeback` are gone, and `formal/Makefile`
  says so where the targets used to be. ADR-0006's slate is discharged. The rule the bullet was
  protecting stands and is worth keeping: a green run of a task with no assertions is not a result.
- **`components_pcloop` was failing on `main`, and nothing ran it** (ADR-0046). It has failed since
  `e4f5250` — `failed assertion ... at pcloop.sv:273 step 3`, the sequential-advance assertion —
  because its `f_may_stall` over-approximation predates ADR-0042's fifth stall reason and therefore
  covered a cycle on which the decoder legitimately holds the pc. Attributed by mutation: forcing
  `operand_stall = 1'b0` makes the task pass by k-induction. Fixed by transcribing
  `rtl/decoder.v`'s `prev_rs1`/`prev_rs2`/`read_taken` register into the harness, and the task is on
  CI now — ADR-0017 puts the fetcher↔decoder pc loop in M2's scope, and **an M2-scope proof nothing
  runs is a prose-only guard**.

**The ladder now asserts its own shape, so a green ladder can no longer shrink quietly**
(ADR-0033's gap 1). `formal/checks.cfg`'s `[depth]` table is the list of checks that *exist*, not a
tuning table — `genchecks` returns early on any check with no depth line, silently — and
`formal/check-baseline.sh` used to glob the run directories `sby` creates only for checks it
actually started, so a never-generated check was missing from the results and from
`formal/EXPECTED_FAIL` at once and set equality called that a clean match. Two files close it, both
set equalities in **both** directions per ADR-0014: `formal/EXPECTED_CHECKS` names every check that
must be generated, and `checks.cfg`'s `#omit` lines name every check `genchecks` considered and
skipped, with a reason each. `formal/genchecks-audit.py` derives both sets by tracing `genchecks`'
own `get_depth_cfg` calls — it does not re-implement the naming, and it cross-checks its trace
against `genchecks`' own `consistency_checks`/`instruction_checks` sets and against the `.sby` files
on disk, so a wrong inventory fails rather than being reported. `check-baseline.sh` now enumerates
`checks/*.sby` rather than directories, so a generated-but-never-scheduled check resolves to
NO-STATUS and counts non-PASS, and `make -C formal check` ends in that comparison rather than in
`sby`'s exit code, which `expect pass,fail` makes meaningless. Deleting any one `[depth]` line —
including a **passing** check's — now fails at generation in a second and again at the post-run
gate.

Three checks joined the ladder in the same change, taking it from 79 to **82**: `ill_ch0` (red — see
`formal/EXPECTED_FAIL`), `causal_mem_ch0` and `hang` (both pass). `ill_ch0` is deliberately landed
**red**: ADR-0033's rule is that a known-red check on the ladder is the system working and a
known-red check off the ladder is the system lying. Fourteen upstream checks remain declined, each
with a `#omit` line; the count is derived from the generator, not asserted in prose.

**The ladder is 85 now, and the three that joined are the first `csrc_*` counter checks.**
`formal/checks.cfg`'s `[csrs]` carries per-CSR **test lists** — `mcycle upcnt`, `minstret upcnt`,
`mscratch any` — which is the only way upstream's six models
(`rvfi_csrc_{any,const,hpm,inc,upcnt,zero}_check.sv`) are reachable at all. The **bare** `csrc`
spelling emits a `.sby` reading `rvfi_csrc_check.sv`, and this file said that file "does not exist
at the pin"; the fact was right and the phrasing implied a pin bump would fix it. **It has never
existed upstream** — the pin is upstream `main`'s HEAD, the add/delete log for the path is empty,
and the string appears in no branch or tag. `genchecks`' bare-`csrc` branch is dead code.
**`csrc_inc` is declined because it is red on a CORRECT core here**: the model clears `csr_written`
every non-check cycle, so its post-write fallback lives one cycle, and CSR serialization means two
CSR retires are never adjacent — measured red at `1 15`, PASS with writes to `0xB02` assumed away,
and swept red at 13-16 with no depth escaping it. `upcnt` states the surviving half and is stricter
there. **What that does not close**: `upcnt` says `minstret` strictly increases, not that it
advances by exactly the non-trapping issues, so **ADR-0027's rule is narrowed, not closed** —
`test/asm/minstret.S` and `test/csr_tb.v` still carry it. `csrc_any_mscratch_ch0` is the first
check on this ladder that asserts a CSR write *sticks*: deleting `MSCRATCH: mscratch <= warl;`
makes it red and leaves `csrw_mscratch_ch0` PASSing.

**`make -C formal check` could not re-run the ladder, and had been re-grading the previous run**
(ADR-0040). `formal/Makefile`'s `checks` was a plain file target naming a **directory**, and a
directory's mtime bumps whenever an entry is created inside it — `sby` creates `checks/<name>/` for
every check it runs, so after one ladder run `checks/` was newer than `checks.cfg`,
`genchecks-local.py` and `EXPECTED_CHECKS` at once and make called it up to date forever. Editing
`checks.cfg` between two runs did not regenerate. Compounding it, `genchecks-local.py:72` is
`sbycmd = "sby"` with no `-f` and the makefile it emits hardcodes that string, so every
re-invocation aborted inside sby and left the prior `status` file in place; `make -BC checks` forces
the **rule**, not sby. Measured at `317b86c`: run 1 took 310.4s and wrote 82 status files, run 2 took
22.0s, printed "Directory already exists" for all 82, wrote **none**, and still reported "82 checks:
82 pass, 0 fail" — a verdict it had not computed, about RTL that may have changed underneath it.
`checks` is now `.PHONY` and deletes the run directories before regenerating (~1s), and `-B` is off
the sub-make. **`make -C formal check` is now always a fresh run**; `make -C formal check-baseline`
is still how you re-grade a finished one without re-running it.

**The ladder will not silently mis-model a negedge regfile — it refuses to run at all** (ADR-0040).
This matters because ADR-0038 now *recommends* a negedge-BRAM regfile on area grounds. A raw
`yosys ... prep -flatten -nordff; write_btor` does discard clock polarity silently (`clk` becomes an
unused input; the read register and the storage array advance on the same step) — but the ladder
never takes that path. sby runs its own `prep` model step after the `.sby`'s `[script]`, and
`formalff -clk2ff` there fails closed: *"CLK clock ... also used with opposite polarity, run
clk2fflogic instead"*, `DONE (ERROR, rc=16)`, which `check-baseline.sh` counts as red. **The silent
loss is scoped to bespoke yosys scripts that end in `write_btor`/`write_smt2` without going through
sby** — any standalone regfile proof written that way would be one. (`formal/equiv.sh` was the
example this line used to give; it is deleted as of ADR-0047, and it was never exposed anyway — it
had no backend, a correction `cdb53cb` already made.)
`clk2fflogic` is **rejected** as the remedy: at genchecks' own depths it makes `reg_ch0` PASS on a
regfile with the rs2 write-through bypass deleted, which the stock ladder catches in 13.9s, because
`clk2fflogic` makes a clock cycle two BMC steps while genchecks ties `skip`/`depth`/
`RISCV_FORMAL_CHECK_CYCLE` to one `[depth]` column — and no `checks.cfg` expression can decouple
them. **Deleting the rs2 write-through bypass is this repo's liveness probe for `reg_ch0`**: one
line, a direct invariant-6 violation, counterexample on bad state property 1 at k=20. Reach for it
before believing any `reg_ch0` result taken under a changed configuration.

**M3's keystone: every trap is now detected AND committed in decode, and the formal baseline is
empty.** `rtl/decoder.v` computes misalignment from the same `$signed(immediate) +
$signed(reg_rs1)` it already had, decides illegal-instruction (including ADR-0005's two illegal-CSR
rules), and commits `ecall`/`ebreak`/illegal/load-misaligned/store-misaligned in the *same*
non-stalled `else` arm that issues an instruction — which is what buys, for free, no trap on a
stalled cycle, no trap from a bubble, no double commit, and a `reg_rs1` the scoreboard has already
made hazard-clear. **A trap is a branch**: `pc <= mtvec` is the same override `jal` and the
branches use, so there is still no flush anywhere (invariant 1). `rtl/csrs.v` gains a second write
port (`trap_entry`/`trap_cause`/`trap_epc`/`mret_entry`) and hands `mtvec`/`mepc` back to decode;
`mret` gets its own decode, restores MIE from MPIE, and serializes on the existing drain
predicate; `wfi`, `fence` and `fence.i` become the NOPs ADR-0005 always said they were, which was
optional right up until "unrecognised" started meaning "illegal instruction". `rtl/accessor.v`'s
`mem_misaligned` and `rtl/littlecpu.v`'s `mem_misaligned_trap` are **gone** — ADR-0011's deferred
debt, discharged — and the top-level `trap` port is redefined per ADR-0028 as a one-cycle "trap
entry committed" pulse that something finally consumes. `rvfi_trap` is driven from the retiring
instruction's shadow instead of being hardwired 0. **`formal/EXPECTED_FAIL` reaches EMPTY: 82
checks, 82 pass** — the nine misalignment entries and `ill_ch0` all closed at once, exactly as the
attribution predicted, and `insn_lb`/`insn_lbu`/`insn_sb` (the byte accesses, which cannot be
misaligned) never moved. `test/EXPECTED_FAIL` reaches empty too, at 52/52. **This does not by
itself mean M2** — see the milestone table.

ADR-0034's sunset condition **fired**: `decoder_output`'s `csr_addr`/`is_csr_imm` were kept as
scaffolding on the explicit condition that they be struck in the trap change if neither acquired a
downstream consumer, and neither did. They are gone. ADR-0029's harness half landed with the RTL
that makes it reachable: `test/testbench.v` raises a `(* keep *) trap_to_zero` flag when a trap
redirects to address 0, `test/cxxrtl.cc` turns that into exit code 5 and `test/run_tests.sh` labels
it `TRAP-TO-ZERO`, so a test that faults before installing `mtvec` gets a named failure instead of
a timeout. And the Sail config (`test/sail/rv32imc_zicsr.json` since ADR-0043;
`test/sail/memory-map.json` at the time) sets `memory.misaligned.exceptions.load_store` to
`{"Some": "AlignmentException"}` — the **global** key, which is what actually governs; the
per-region `misaligned_exceptions` attribute is checked after address translation and with no MMU
is never consulted. Without it the reference model completes misaligned accesses and reports a
false divergence on every one; verified both ways (`trap.S` agrees 214/214 with the key, diverges
at architectural change #6 without it).

**The iverilog leg was dead, and this change is what noticed.** `rtl/decoder.v`'s hazard
scoreboard called a `function automatic live_producer(r)` from two continuous assigns. iverilog
builds such an assign's sensitivity list from the *call's arguments*, so a function body that also
reads module-level signals (`out`, `executor_out`, `accessor_pending_*`) never re-evaluated when
those changed — **under**-sensitivity, the one direction CLAUDE.md's `sorry:` exception says is a
real bug, and iverilog emits no diagnostic for it. Measured: `hazard_rs1` latched high at the first
RAW hazard of every program and never fell, so `make waves`' own baked-in loop executed two
instructions and then span (0 memory writes in 200 cycles, on `main`, before this change), and no
`.S` program could reach `tohost`. yosys evaluates the function correctly, so cxxrtl and the whole
formal ladder were unaffected — which is precisely why it went unnoticed: the leg the verification
table calls the microscope was reporting nothing, and nothing it reported was wrong. The two
assigns are now written out. All 52 `.S` programs run to `PASS` under `vvp` with the per-retire
monitor live, the first time that has ever been true. **There is still no tracked runner for it**:
`test/testbench.v` has a baked-in program and no image loader, so the 52/52 above was measured with
a scratch harness. `make waves` is the tracked iverilog exercise and it now executes real
instructions rather than deadlocking.

What does work: `yosys ... write_cxxrtl` elaborates the current RTL cleanly — **one** warning,
`Deep recursion in AST simplifier`, which is a recursion-depth notice rather than a correctness
signal and is the single entry on the `elaborate` gate's allowlist; everything else yosys prefixes
`Warning:` is promoted to an error there (`.github/workflows/ci.yml`, which cross-references this
note). It is not zero, and this line said zero until ADR-0037. The
cxxrtl binary builds and runs, the whole `.S` suite passes under it with `test/EXPECTED_FAIL`
empty, `make test-units` passes (**eight** benches since ADR-0060 added `accessor_tb`: `exec_tb` —
10,000 randomized differential vectors
per op across `mul`/`mulh`/`mulhu`/`mulhsu`/`div`/`divu`/`rem`/`remu`/`sll`/`srl`/`sra` **and**
`add`/`sub` — the latter pair closing the bench's own blind spot on the simplest ALU op — plus 384
directed shift vectors per shift op sweeping every amount 0-31 with a
clean and a dirty rs2 so the `rs2[4:0]` mask is checked rather than coincided with, and — because
ADR-0045 names it M2's mul/div oracle and ADR-0033's rule is that a named gate must be unable to
stop checking quietly — it now asserts **its own shape** first: every mul and div reference is
pinned against hand-computed literals like the shift ones already were (a degraded one says
`ORACLE BROKEN` and stops), the per-op vector count is pinned against ADR-0010's `>= 10,000`
written as its own literal rather than a second copy of the loop bound, and each required directed
corner vector is witnessed against a manifest kept apart from its call sites — so a deleted
directed vector, a shrunken loop and a reference degraded into a conditional arm are each red,
where the first two used to print `PASSED` and exit 0 — `mem_tb`,
`decoder_tb`,
`regfile_tb` — which covers the write-through bypass and x0 semantics — `csr_tb`, which covers
`rtl/csrs.v`'s read mux, its `implemented` address set, its WARL masks and its trap-entry/`mret`
port, `imem_tb`, which walks rtl/imemory.v's bank select at every word index and both parities
against a flat reference plus its range decode (ADR-0054), `accessor_tb`, which exists for the one
signal nothing else can see stuck low — `mem_ren`, the read enable the memory's arbiter keys on
(ADR-0060) — and `monitor_tb`, which checks the oracle
itself rather than the core), and **all three**
component proofs — decoder, executor and `pcloop` — pass by k-induction (`mode prove`; re-run and
read off each `sby` summary, not inferred from a green job. See ADR-0017 for what the decoder proof
does and does not establish, and ADR-0046 for what `pcloop` discharges). `make waves`
now runs the iverilog leg (`testbench.vvp`) instead of the cxxrtl runner, matching the verification
table below, and produces a real `waves.vcd` — and, as of ADR-0055, grades the run it produces one
too: `ci.yml`'s `elaborate` job runs and checks this same `testbench.vvp` directly, on one fixed
baked-in program, not the `.S` suite (see the second bullet under "what does not work" above for
what stays open). CI (`.github/workflows/ci.yml`) runs **nine** jobs on every PR:
elaborate, test, components, lint, fit, soc-timing, nonperturbation, monitor-freshness and formal —
the last of which also carries `complete` and `complete_cover` as hard gates (M2 term 5). **Seven of
the nine are required** — `elaborate`, `test`, `components`, `monitor-freshness`, `lint`, `formal`
and, since ADR-0066, `soc-timing`, read live from
`gh api repos/thejefflarson/little-cpu/branches/main/protection -q
'.required_status_checks.contexts'` rather than from any comment in the workflow — and
`nonperturbation` and `fit` are the two that run without gating, each deliberately
(ADR-0047, ADR-0052). **`soc-timing` was one of them until 12 MHz became a requirement**: area is
still a design constraint, but a clock the board cannot supply is not one. **There is no
`continue-on-error` anywhere in the
file**, in any job: the `formal` job carried the last one until ADR-0052 struck it. This line named
four jobs and stopped there until the gate inventory; `lint` and `formal` had both been promoted to
required in the meantime,
and `ci.yml`'s own comments still said otherwise.

**A Sail co-simulation spike measured what the existing oracles structurally cannot see**
(ADR-0032). Both the RVFI monitor and every `insn_*` ladder check read the core's *self-report*;
`reg_ch0` is the one check that ties that report back to `rtl/regfile.v`, and it is a single
`mode bmc` query at depth 21. An injected extra architectural write outside the retiring
instruction's `rd` (`regs[31] <= wdata` alongside `regs[waddr]`) was **missed by all 49 `.S` tests
with the per-retire monitor live**, caught by `reg_ch0` — and then, gated to fire only after cycle
40, **missed by the entire 78-check ladder** (67 pass / 11 fail, matching `formal/EXPECTED_FAIL`
exactly) while the co-sim reported it in 0.6s with instruction number, PC and both values. The
class is *architectural state no retiring instruction names, corrupted past the BMC bound*. The
mutation was reverted; `rtl/` is untouched by that change.

**Co-simulation now runs the whole suite against a baseline** (ADR-0039). `make cosim-suite` is
`test/run_cosim.sh` in the shape of `test/run_tests.sh`, graded by ADR-0014's set equality in both
directions against `test/COSIM_EXPECTED_FAIL` in ADR-0035's name-and-status format: **52 programs,
50 agree, 10s wall.** The two baselined entries are read-only-CSR *value* divergences no change to
this core could close — `csr.S` on `misa` (Sail derives it from its extension set; matching it needs
seven extensions disabled and a cascade of dependent keys, measured and recorded rather than chased)
and `minstret.S` on `mcycle` (an ISA model has no pipeline, so it cannot count cycles). `trap.S`
agrees 214/214. **The mutation was re-run against the integrated leg rather than inherited**: the
same post-cycle-40 `regs[31] <= wdata` is missed by `make test` at 52/52 and caught by
`make cosim-suite` in 49 of 52 programs (the three that still agree are the three too short to reach
cycle 40), and `csr.S`'s baselined status moves from `DISAGREE AT 17` to `DISAGREE AT 11` — the
second baseline field doing its job.

**`test/COSIM_EXPECTED_FAIL` is now EMPTY and the suite is 52/52** (ADR-0043), and no `rtl/` file
changed to get there. Both entries above were artefacts of how the reference model was configured
or compared, not of this core. The `misa` one was the more important: the model was a
`--config-override` on Sail's **default RV32 machine**, and an override inherits everything it does
not mention — so the thing this repo cross-checks against had A, B, D, F, S, U and V, none of them
implemented here and none of them chosen. `misa` was simply the one register a test read.
`test/sail/rv32imc_zicsr.json` replaces it with a **complete `--config`** describing ADR-0002's
RV32IMC_Zicsr: `--config` is rejected outright if a key is missing, so a Sail version bump that
adds a knob now fails loudly here instead of picking a default nobody read. `mcycle` is handled by
`test/cosim.py`'s `NONCOMPARABLE_CSRS`, which exempts **one register's value at one change** and
prints every exemption it takes. **Fixing `misa` unmasked two divergences it had been hiding**, and
those are the ones worth remembering: `mip.MTIP` and `mie`'s writable bits are read-only zero here
(ADR-0002) and neither is configurable in sail-riscv 0.13.1, so `csr.S`'s assertions about them made
the *reference model* run the program to `fail` — a different branch, not a different value, which
no value exemption can or should paper over. Those assertions moved to `test/csr_tb.v`, which
already made every one of them. **Before adding a line to that baseline, read its header**: the
three questions in it are the decision procedure, and a baseline entry is the last of them.

The same change made `tohost` an 8-byte-aligned `.dword` in `test/asm/riscv_test.h`, which is what
the HTIF protocol its encoding is borrowed from always specified (ADR-0039 amends ADR-0008). It was
a 32-bit `.word` at the base of RAM, so `.data` began four bytes inside the doubleword IO window any
HTIF consumer claims at the symbol; Sail answered every `lw` from `RAM_BASE+4` with zero. **This is
shared test infrastructure both existing sim legs read** — every program's `.data` moves four bytes
and every program executes one extra store — and `make test` (52/52) and `make test-units` were
re-run green in the same change. The verdict macros write the upper word first and the verdict last,
so the reference model terminates on the same store the cxxrtl runners stop on; the `objcopy
--strip-symbol=tohost` workaround and the spin-loop convergence heuristic are both gone.

**`86e2721` ports the riscv-formal ladder to `littlecpu`** — the first time any riscv-formal
check has run against the pipelined core since the 2021 teardown. `formal/wrapper.v` speaks the
real bus (no handshake: `imem_data`/`mem_rdata` are free every cycle, per invariant 1 and
ADR-0015). Of 78 generated checks: **55 of 70 `insn_*` pass**; `pc_fwd`, `pc_bwd`, `liveness`,
`unique`, `causal`, `imemcheck` and `dmemcheck` pass; `cover` reaches all five goals, including
≥2 loads and ≥2 stores and ≥2 uncompressed and ≥2 compressed retires in one trace — which is what
rules out a vacuous harness. **Read ADR-0023 before quoting any of these numbers.** Every one of
the 15 failures is accounted for (9 = the M3 trap gap, 2 = the C.JR/C.JALR defect, 4 = the ALTOPS
divide defect — all three since fixed, and the last two no longer have bullets of their own above),
`reg` was inconclusive *under the `smtbmc yices` engine that run used* (ADR-0024 replaced it; see
the struck bullet above), and everything ran under `RISCV_FORMAL_ALTOPS`, so
a green `insn_mul` says nothing whatever about multiplication (ADR-0010). **M2 is not reached.**

**Four of `rtl/decoder.v`'s six `(* parallel_case *)` markings were spent against nobody's proof,
and the two statements that lack the marking must keep lacking it** (ADR-0068). The immediate,
`rd`, `rs1` and `rs2` muxes select on opcode-group and **compressed** flags — `instr_load_op`,
`instr_clw`, `instr_cswsp` — and the `$onehot` assertion in that file covers the 45 *architectural*
flags (`instr_lw`, `instr_add`) and names none of them. A compressed flag widened to overlap a
sibling leaves every architectural flag one-hot, so the existing check stays green while two arms of
a marked mux match at once and synthesis picks either; **measured** — widening `instr_clw` to
quadrant 2 passed `components_decoder` before this change. Five `$onehot0` assertions, one per arm
list, close it, and the same mutation now fails on exactly the two of them it should. The two
**unmarked** statements were the ticket's actual ask and both are declined: `next_pc` selects on
`stall` and `trap_pending`, which are both true whenever a trapping instruction is held, and
`trap_cause` is ADR-0030's recorded priority order, whose first stated failure mode is someone
flattening it. There are **eight** such statements in the file, not six — two are written
`case(1'b1)` with no space and a space-sensitive grep is what missed them. The change is
`ifdef FORMAL`-only and that is proven, not argued: the two `fit.json` netlists differ in one line,
the module's recorded end line.

## Invariants — do not break these

These are the design. Violating one is a bug even if tests pass.

1. **Fetch is combinational and the decoder owns the PC.** There are never wrong-path instructions.
   **No flush logic may be introduced, ever.** If a change appears to need one, something upstream
   has gone wrong — stop and reconsider rather than adding a kill signal.
   **On real hardware this is bought by publishing the fetch address a cycle early** (ADR-0054):
   there is no combinational-read memory on the target part, so `rtl/decoder.v` names the next PC
   combinationally (`next_pc`) and the memory latches `imem_addr_next` on the same edge `pc` takes
   it. Decode still sees the instruction at `pc` in the cycle it decides the next one; nothing
   speculative rides on it, and a stalled cycle sets `next_pc = pc` so the same words are
   re-presented. **`pc` must have exactly one driver** — `formal/pcloop.sv`, `rtl/decoder.v`'s own
   `FORMAL` block and `test/decoder_tb.v` each assert that, because a second one puts the memory a
   cycle out of step with decode and the ladder cannot see it.
2. **All traps are detected and committed in decode.** Nothing faults after decode. This is what
   makes CSR commit precise without a reorder buffer. **The memory system is what will test this**:
   an access fault is raised by memory refusing a transaction, i.e. after decode, so a bus that can
   refuse forces a ruling on this invariant — which is why five of the fourteen checks
   `formal/checks.cfg` declines are declined against it, and why ADR-0044 records the three options
   without picking one.
3. **Every inter-stage struct carries a `valid` bit.** A bubble is `valid = 0`; retire is `valid`
   reaching writeback, which gates `wen` and drives `rvfi_valid`.
4. **Hazards are handled by stall-only interlock in decode.** No forwarding network. Adding one is
   a CPI-only optimization and requires a new ADR.
5. **CSR instructions, `mret` and `fence.i` serialize** — held in decode until execute/access/
   writeback drain. The first two serialize so a one-cycle-wide architectural update cannot
   interleave with instructions that issued before it. `fence.i` serializes for a different reason
   (ADR-0061): text is writable and the fetch address is published a cycle early, so ordering a
   store against the fetches behind it needs the older store's write edge to have passed before
   `fence.i` issues. Same mechanism, two reasons; do not collapse them.
6. **The regfile read is synchronous and takes one cycle** (ADR-0042). Decode observes, in the
   cycle it issues, the architectural value of rs1/rs2 **including a writeback committed in that
   same cycle** — two forwarding points in fabric (write-first into the read register, then the
   write-through bypass) are what make that true. The second sentence is the load-bearing one and is
   unchanged: it is what ADR-0004's stall-only scoreboard depends on, and why decode needs no
   forwarding path for the writeback slot. The flip-flop array that used to make the read
   combinational was one implementation of that contract, not the contract.
   **This invariant now DEPENDS on invariant 9, and that dependency is new** (ADR-0064). The
   write-through bypass compares `waddr` against a **registered copy** of the address pair, not
   against the `rs1`/`rs2` decode is presenting — instruction bits out of the word the fetch just
   returned, which put the bypass mux and the branch comparators behind it *after* the instruction
   and cost this design 19% of its clock period. It selects the same value only because
   `operand_stall` guarantees that on an issuing cycle the held pair **is** the presented pair. The
   bypass used to be correct for any address pair; it is now correct conditionally, and the
   condition lives in another file. See invariant 9 for what breaks it.
7. **`test/monitor.v` is generated but tracked.** Regenerate it; never hand-edit it.
8. **Stalls are a single global broadcast: six reasons over two mechanisms** (ADR-0026 amending
   ADR-0009; ADR-0042 adding the fifth, ADR-0060 the sixth). The reasons are the decode scoreboard,
   the divider, the accessor's one-cycle load-response turnaround (ADR-0015), serialization
   (invariant 5), the operand-fetch cycle (invariant 9), and the instruction memory's stolen fetch
   window (`fetch_stall`, ADR-0059) — the last two bubble, exactly like a RAW hazard, and add no
   third mechanism. The **mechanisms** are
   what actually matters, and three non-local rules hold them together, each breakable silently:
   (a) while `divider_stall` or `accessor_stall` is asserted, decode **holds** `decoder_out`
   unchanged rather than bubbling it, and nothing downstream may consume it that cycle; a RAW
   hazard **bubbles** instead, and so do serialization and `fetch_stall` — neither instruction has
   issued, so they fold into the existing `hazard`/bubble arm rather than becoming new machinery.
   **A `fetch_stall` coinciding with either freeze HOLDS**: the freeze is about an instruction
   already in `decoder_out` that nothing has consumed, and bubbling would drop it. That is a ruling
   (ADR-0060), enforced only by the publish block's arm order, so it is asserted in
   `rtl/decoder.v`'s `FORMAL` block and driven as a vector in `test/decoder_tb.v` — reordering two
   `else if` arms is otherwise silent. (b) Every
   in-flight non-`x0` `rd` is visible to the scoreboard on every cycle between issue and the
   regfile write-through, with no gap: `decoder_out` → `executor_out` → `accessor_pending` (loads
   only) → write-through. (c) The serialization drain predicate reads **four** slots, not the
   scoreboard's three: `accessor_pending_valid` covers loads only, so `accessor_out.valid` is
   routed into the decoder separately (`rtl/littlecpu.v`) — without it a *store* in flight is
   invisible and a CSR instruction issues early, giving a `minstret` that is wrong only when a
   store happens to be in flight.
9. **The regfile's answer belongs to the address pair presented in the PREVIOUS cycle** (ADR-0042).
   Decode holds `pc` across the pair and bubbles — that is `operand_stall`, and it is why invariant 6
   costs a cycle. Nothing may consume `reg_rs1`/`reg_rs2` in a cycle whose address pair differs from
   the one presented last cycle. Same shape as invariant 8's rules: true today only because
   `operand_stall` enforces it, breakable silently by a later change, and not visible from reading
   `rtl/regfile.v` alone. `test/regfile_tb.v` pins it directly — it fetches x5, points `rs1` at x6 in
   the use cycle, and asserts that **x5's** value is what comes back.
   **Narrowing `operand_stall` now breaks invariant 6 as well, silently** (ADR-0064). Skip the
   bubble for some instruction class and the write-through bypass — which selects on the held pair —
   starts answering with the previous instruction's operand. Nothing in the tree would say so except
   the two `test/regfile_tb.v` vectors that drive the presented address away from the held one with
   a write in flight to the held one; every other vector in that bench holds the pair and cannot
   tell the two spellings apart. **A change that touches `operand_stall` must re-read invariant 6**,
   and it is a new-ADR change, not a tuning one.

## ISA target

RV32IMC_Zicsr_Zifencei, M-mode only. `misa = 0x4000_1104` — neither Zicsr nor Zifencei has a
`misa` bit, so the ISA string is the only place either is claimed. **Zifencei costs a pipeline
drain** (ADR-0061): `fence.i` serializes on invariant 5's predicate, because text is writable
(ADR-0059) and the fetch address is published a cycle early (ADR-0054), so a store at cycle D writes
the array at edge D+2 while the instruction after a `fence.i` at D+1 was fetched at edge D+1. This
section said it was correct **and free** — one hart, no icache, so a conformant `fence.i` is a NOP —
and that was a property of ADR-0008's split buses rather than of the core. The cache half is still
free; the ordering half never was, it was merely unreachable. Before ADR-0002 was amended the string
did NOT claim Zifencei while the decoder accepted it anyway — silently permissive, and the one
combination that was wrong.

**Conformance is not negotiable against minimality.** Every CSR the privileged spec lists
unconditionally for RV32 machine mode is implemented; a core that traps on a mandatory register is
non-conformant however small its set is, and the spec lets almost all of them read zero, so the
cost of getting this right is close to nothing. ADR-0005 headed its list "CSR set (exact)" until
ADR-0048, and that word turned a conformance gap into an apparent design choice — `mstatush` and
`mconfigptr` were missing, and the question it produced was whether the exact set could widen
rather than why the core was non-conformant. The list is a **floor**, not a closed set.

Traps: illegal instruction = 2, breakpoint = 3, load address misaligned = 4, store address
misaligned = 6, environment call from M-mode = 11. **Instruction-address-misaligned (0) is
unreachable** — C makes 2-byte targets legal — and is not implemented. No interrupts: `mie` and
`mip` are read-only zero.

The C extension stays because **code density is a product constraint** on the up5k, not a
preference. See ADR-0002 and ADR-0003.

## Commands

```sh
make setup          # install the RISC-V toolchain (brew on macOS)
make test           # assemble test/asm/*.S, run under cxxrtl, pass/fail table with
                    # per-program retire / spec-checked-retire counts, graded against
                    # test/EXPECTED_FAIL (set equality) and test/OBSERVED_FLOOR (>=).
                    # OBSERVED_FLOOR's NAME set is also the suite manifest, checked
                    # both ways before a single program is assembled -- so a suite
                    # that SHRANK is red, not a smaller table that still "passes".
                    # Also runs `make probe-gates` (ADR-0053).
make cycles         # THE CPI COUNTERPART OF `make fit` (ADR-0070). The same 59
                    # programs, per-program and whole-suite, with every cycle
                    # charged to an issuing cycle or one of the six stall
                    # reasons. 0314890: 28632 cycles, 13853 retires, CPI 2.07,
                    # 51.0% of cycles issuing nothing -- 29.3% decode
                    # scoreboard, 16.4% operand fetch, 2.8% divider, 1.5%
                    # serialization, 1.1% load turnaround, 0.03% stolen fetch.
                    # A COLUMN IS CYCLES CHARGED, NOT CYCLES THE SIGNAL WAS
                    # HIGH. NOT on `make test` and not on CI: no CPI ratchet
                    # exists and this adds none. It does grade its own
                    # arithmetic -- a stalled cycle none of the six explains
                    # exits nonzero. THE CPI DESCRIBES THE SUITE, NOT THE CORE.
make probe-gates    # forces all 137 graded comparisons in the grading scripts to
                    # FAIL and requires each to fail for its own reason. Hermetic
                    # (stubs, no toolchain); a prerequisite of `test` on purpose,
                    # so it is in CI's required job. All fork, no work: ~68-90s of
                    # wall for ~4s of user time, and that ratio is the host's
make waves          # iverilog + VCD (testbench.vvp's baked-in program) -> waves.vcd.
                    # testbench.vvp now checks the monitor's errcode every cycle and a
                    # 15-write/60-retire floor itself (ADR-0055), so `vvp` -- and this
                    # recipe, which aborts before the `mv` on vvp's nonzero exit -- fails
                    # on those too. ONE FIXED PROGRAM, NOT THE `.S` SUITE: there is still
                    # no image loader for test/testbench.v, so this is not the deferred
                    # multi-program runner. `.github/workflows/ci.yml`'s `elaborate` job
                    # runs and grades this same testbench.vvp directly, on every PR.
make monitor-check  # regenerate test/monitor.v at the pin and diff
make fit            # the ONE area number: nextpnr logic cells on up5k/sg48 (ADR-0038).
                    # Placement always fails (231 SB_IO vs 39) and that is expected --
                    # the utilisation table printed before placement is the measurement.
                    # A RATCHET as of ADR-0042: over FIT_MAX_LC (4400) exits nonzero.
                    # ON CI as of ADR-0052, in the `fit` job, NON-REQUIRED on purpose
                    # -- area is a design constraint, not a correctness one, and
                    # adding it to branch protection is a human action.
                    # 3875 cells / 73% under a local Homebrew yosys as of
                    # ADR-0054, down from 4187 on the same toolchain -- the
                    # `next_pc` refactor, 312 cells, six times the churn floor.
                    # THE MEASUREMENT IS TOOLCHAIN-DEPENDENT: the pre-ADR-0054
                    # tree measured 4208 under CI's pinned OSS CAD Suite against
                    # 4187 locally, 21 cells apart on the synthesiser build alone
                    # (ADR-0052), so expect roughly 3896 on the `fit` job. A local
                    # run is a sanity check; the job is the measurement.
                    # Probe: `make fit FIT_MAX_LC=3800` exits 2.
make soc-timing     # THE SoC PLACE-AND-TIME FLOW (ADR-0054), and NOT `make fit`:
                    # different top, different design, numbers that must not be
                    # merged. littlesoc -- core + BRAM ROM + SPRAM RAM + 4 pins --
                    # PLACES, and icetime reports the critical path with its
                    # LOGIC/ROUTING SPLIT, which is the finding. 4383/5280 LC,
                    # 2/4 SPRAM; 74.34 - 78.80 ns = 12.69 - 13.45 MHz over four
                    # placements as of ADR-0064, which addressed the regfile's
                    # write-through bypass from a registered copy of rs1/rs2.
                    # THAT MEETS 12 MHz at every placement, and 12 is the board
                    # crystal rather than a round number -- the step below it
                    # is 6 (ADR-0062). It was 88.51 ns = 11.30 MHz, 38.7% logic /
                    # 61.3% routing, over
                    # imem.in_range -> decode -> next PC -> imem.in_range2
                    # at ADR-0054, whose 41 levels were 25 LUT + 17 CARRY at
                    # 3.31 ns and 0.34 ns each; read those apart (ADR-0058).
                    # Grades SOC_MIN_MHZ, which is 12.0 as of ADR-0066: the
                    # board clock itself, NOT a margin under the measurement.
                    # It was 10.9 (ADR-0064) and 10.0 before that, both
                    # regression floors that slid as the design moved. This one
                    # does not slide, and the margin against the worst local
                    # placement is 5.75% -- tighter than any other ratchet
                    # here, accepted on purpose. WHEN IT TRIPS, FIX THE DESIGN.
                    # soc/timing_sweep.sh runs four placements and prints the
                    # spread. One placement is a sample; compare distributions.
                    # Needs the RISC-V toolchain (it builds a real ROM image).
                    # ON CI in the `soc-timing` job, REQUIRED as of ADR-0066
                    # (it was non-required, mirroring `fit`). The
                    # census (soc/cell_census.py) and the ratchet
                    # (soc/timing_split.py) are both probed hermetically in
                    # test/probe_gates.sh. `SOC_PROG=lw.S` picks the program,
                    # `SOC_SEED=3` places it differently -- one placement is a
                    # sample, and main's own spread is 1.2% (ADR-0058).
                    # Probe: `make soc-timing SOC_MIN_MHZ=99` exits 2 (the
                    # script exits 1 and make reports its own status).

make -C formal components_decoder   # component proofs. THREE tasks, all with real assertions:
make -C formal components_executor  # decoder, executor, pcloop -- the assertion-free fetcher /
make -C formal components_pcloop    # accessor / writeback tasks were deleted, not left unrun.
                                    # pcloop is the composed fetcher+decoder proof that discharges
                                    # ADR-0017's assume(in.pc == pc); it needs `smtbmc boolector`
                                    # (see formal/components.sby) and is on CI as of ADR-0046
make -C formal check                # the riscv-formal ladder (85 checks; see ADR-0023). ALWAYS a
                                    # fresh run -- it deletes checks/ first (ADR-0040)
make -C formal check-baseline       # re-grade a finished ladder: EXPECTED_CHECKS + EXPECTED_FAIL
make -C formal complete             # the depth-50 whole-ISA BMC walk: every non-trapping retire is
                                    # an instruction riscv-formal's spec model recognises. 16s, on
                                    # the PR gate, ungated. Excludes MISC-MEM and SYSTEM by
                                    # declaration (formal/COMPLETE_EXCLUSIONS) -- so NOT whole-ISA
                                    # coverage, and `abc bmc3` is a third engine in this repo
make -C formal complete_cover       # its anti-vacuity control: the twelve opcode classes the
                                    # exclusion set does NOT name must each retire in a real trace
make -C formal complete-exclusions  # set equality both ways between complete.sv's declared
                                    # exclusions and their baseline, plus "no spec model at the
                                    # pin" re-derived from the clone. A prerequisite of both above
make -C formal nonperturbation      # ADR-0047: the RVFI instrumentation is UNREAD by the core.
                                    # yosys + python3 only, ~9s, on the PR gate. NOT sequential
                                    # equivalence -- it replaced equiv.sh, which never converged

make sail-setup     # once: fetch the pinned sail-riscv release into tools/sail/
make cosim-run      # Sail co-simulation on ONE program (PROG=add.S). Opt-in; see ADR-0032
make cosim-suite    # the whole .S suite under co-sim, graded against COSIM_EXPECTED_FAIL
```

Toolchain: macOS `brew install riscv64-elf-gcc`; Linux `apt install gcc-riscv64-unknown-elf`.
Tests are freestanding assembly (`-march=rv32imc_zicsr -mabi=ilp32 -nostdlib -T sections.lds`), so
no multilib or newlib is needed. Formal needs a pinned YosysHQ OSS CAD Suite.

## Verification — three legs, each load-bearing

| Leg | Role | Catches what the others can't |
|---|---|---|
| **cxxrtl** | primary runner | real mul/div arithmetic (formal runs under ALTOPS), long/randomized runs |
| **iverilog** | microscope | waveforms, `$display`, second elaboration frontend |
| **riscv-formal** | oracle | exhaustive per-instruction semantics, pipeline corners |

**The iverilog leg was inert for the whole of M1 — `a4662a2` to `6309b3e` — and this table did not
know** (ADR-0037). `rtl/decoder.v`'s hazard scoreboard called a `function automatic
live_producer(r)` from two continuous assigns, and iverilog derives such an assign's sensitivity
list from the **call's arguments**, so a body that also read `out`/`executor_out`/
`accessor_pending_*` never re-evaluated when those changed. Under-sensitivity — the one direction
the `sorry:` exception below says is a real bug — and iverilog emits no diagnostic for it at all.
Measured on `main` before the fix: `hazard_rs1` latched high at the first RAW hazard and never
fell, the PC advanced `0x0`→`0x4` and froze, and `make waves` did **0 memory writes in 200 cycles**
(201 reads, all to address 0). After the fix the same program does 22 writes. yosys evaluates the
function correctly, so cxxrtl and the whole ladder were unaffected — which is exactly why nothing
caught it. **A leg that cannot fail is not a leg**: treat a green iverilog run as evidence only if
you can point at something it would have failed on.

**riscv-formal runs with `RISCV_FORMAL_ALTOPS`, so it never checks the real multiplier or divider.**
That arithmetic is covered only by the `.S` suite and the executor component proof. Do not assume a
green formal ladder means the ALU is correct.

**There is a fourth thing you can run, and it is deliberately not a leg** (ADR-0032, ADR-0039).
`make cosim-suite` diffs the core's *real* `regs_a` array — read through cxxrtl `debug_items` by
`test/cosim.cc`, which touches no `rvfi_*` signal at all — against the Sail RISC-V model, over the
whole suite, graded against `test/COSIM_EXPECTED_FAIL`: **52 of 52 agree, 7.3s** (ADR-0043; it was
50 of 52 at ADR-0039), against 7.3s for
`make test`. `make cosim-run PROG=x.S` does one program. It is **not** on `make test`'s path and
**not** in CI's required set, on purpose: `make test` must keep working on a machine with no Sail
installed (demonstrated by moving `tools/sail` aside), and **adding it to branch protection is what
ADR-0032 forbids**. The way a change gates on it is by carrying its pre/post output in the pull
request. `test/cosim.cc` reading no `rvfi_*` signal is the property that makes this leg worth
having; do not "align" it against `rvfi_valid`.

`test/monitor.v` rides along in both sim legs (`b2dafcc`), so every run is self-checking per-retire —
a test that corrupts state transiently but converges to the right final registers fails loudly, not
just end-state assertions passing. Both legs read the sanitized `test/monitor.sim.v`, which is
therefore load-bearing for correctness and not merely for elaboration: a change to it is a change to
the oracle (ADR-0019).

## Engineering rules in force

- **Compiler and elaboration warnings are errors.** Fix them; don't silence them.
  **One documented exception**, and only this one: iverilog emits `sorry: constant selects in
  always_* processes are not fully supported` for every struct-field read inside an
  **`always_comb`** block — 20 of them, all from `rtl/writeback.v`, against
  `in[952:0]` (`accessor_output`, with the `RISCV_FORMAL*` macros on — 953 bits; it was 952 before
  `6309b3e` added `rvfi_shadow.trap`, and the count held at 20 across that change). It cannot build a precise sensitivity entry for a constant
  part-select, so it falls back to whole-struct sensitivity.
  **That fallback is provably safe**: over-sensitivity re-evaluates redundantly and can never
  yield a stale value; only *under*-sensitivity is a correctness bug. Everywhere the select was
  cheap to hoist into a named continuous assign it has been (`rtl/decoder.v`'s register-index
  fields, `rtl/accessor.v`'s payload fields, `test/testbench.v`'s ROM index, and
  `rtl/writeback.v`'s twelve CSR payload reads) — and it reads better besides.
  `rtl/writeback.v` keeps the `in.` form in its two `always_comb` blocks because hoisting all
  30 reads would add more plumbing than it removes. Copying the struct to a local does not help
  (still a constant select); `always @(in)` would, but trades away `always_comb`'s latch
  checking for exactly the sensitivity iverilog already infers.
  **The diagnostic is specific to `always_comb`**, where the sensitivity list is *inferred*.
  `rtl/executor.v` emits **zero** of these despite its `case (1'b1)` over 39 `in.is_*` flags,
  because that body sits in `always_ff @(posedge clk)`, whose sensitivity is written out.
  Attribute them by struct width rather than by the diagnostic, which carries **no filename at
  all** (it prints `:0:`): `$bits` gives `decoder_output` 943, `executor_output` 921,
  `accessor_output` **953**, and only the last matches `in[952:0]`. ADR-0034 printed
  `iverilog -g2012 -s <mod> rtl/structs.v rtl/<mod>.v` for this; **that command does not run** —
  it needs `-I./rtl/`, and even then reports 0 for every module, because the reads in question are
  inside `ifdef RISCV_FORMAL` and it passes no macros (ADR-0037). Use `make testbench.vvp`.
  **Do not add new ones outside `rtl/writeback.v`**, and prefer a continuous
  assign for any new struct-field read in an `always_comb` — that is what holds the count at 20
  (ADR-0034; this bullet named `rtl/executor.v` until then, which was measurably wrong).
- **Every comment must earn its place, and the default is delete.** A comment earns it only by
  telling the reader something the code does not already say, in one or two plain sentences,
  understandable without leaving the file. Otherwise it goes — not shortened, gone. Delete
  restatement of the next line, history (git has it), "this is deliberate" with no *why*, section
  banners, and emphasis furniture: ALL-CAPS, "which is the whole point", rhetorical setup that
  announces a lesson before stating it, arguing with a hypothetical reader. **A shorter essay is
  still an essay.**
  **No ADR numbers and no invariant numbers in comments.** They read as the explanation while
  carrying none — a reader who has not memorised the list learns nothing, and one who has still has
  to go and look. Say the mechanism instead. The decision record is in `docs/adr/` and `git blame`
  finds it.
  **Short sentences, one idea each, no invented terms.** If a word is not a signal, module, target
  or file name in the code, do not coin it. "waits until", not "drains". Read it aloud: if it sounds
  like a specification, rewrite it; if it sounds like one engineer telling another what to watch
  out for, it is right.
  **If you cannot state the mechanism in your own words, delete the comment rather than paraphrase
  the ADR.** A wrong explanatory comment is worse than the citation it replaced, because it reads as
  authoritative and nobody re-checks it. Removing a citation from `formal/pcloop.sv` is what found
  its claim to be the only check on the fetch lockstep to be false.
  **One exception: tripwires.** A warning that stops someone silently reintroducing a defect earns
  its place by preventing a bug — keep it, in two sentences. The test is whether deleting it would
  let the defect back in; if the bug is fixed and guarded, the story about it is history and goes.
  This applies to code, tests, config and workflows. It does **not** apply to this file, which is
  deliberately a running log, or to `docs/adr/`.
- **Every non-trivial change adds or updates tests and runs the full suite** before being declared
  done. Elaboration succeeding is not a substitute for tests passing.
- **Never commit build artifacts.** `test/rtl.cc`, `sim`, `*.vvp`, `*.vcd`, `rvfi_macros.vh`, and
  `formal/` output dirs are all generated. (`test/monitor.v` is the one deliberate exception.)
- **riscv-formal is SHA-pinned.** Bumping the pin requires regenerating `test/monitor.v` and
  rerunning the ladder.
- **Shared repo knowledge must not depend on the tracker.** No ticket IDs in code, comments, ADRs,
  docs, or commit messages — a bare ticket number is meaningless to anyone reading this repo
  without access to the tracker, and it rots the moment the tracker does. Cite the thing that
  lives here instead: the **ADR** that decided it, the **commit SHA** that landed it, or — best —
  just say what the reason *is*. "Held for one cycle because the memory registers `mem_rdata`
  (ADR-0015)" beats a ticket number and always will.
  **PR titles and descriptions are the exception** — those live on GitHub, not in the checked-out
  repo, so a ticket ID there costs nothing and buys automatic issue closing. Naming the tracker
  itself once under Pointers is likewise fine. It is the *scattered* IDs that rot.
- Prefer verified/first-party GitHub Actions. Simplest approach unless asked otherwise.

## Milestone ladder

| | Milestone | Green means |
|---|---|---|
| M0 | Foundation | this file, riscv-formal SHA-pinned, dead references gone |
| M1 | Finish the pipeline | all RV32IM `.S` tests pass under cxxrtl — **reached, `a4662a2`** |
| M2 | **Parity checkpoint** | **all six** conditions below hold. An empty `formal/EXPECTED_FAIL` is *one* of them and on its own means nothing — it was reached at `6309b3e` and M2 was not (ADR-0037) |
| M3 | Past the old core | CSRs + machine-mode traps — **both landed** (`rtl/csrs.v`; trap commit in `rtl/decoder.v`) |
| M4 | Full ladder + CI | the `formal` job green with every check the repo owns on it; tag a release. **There is no nightly** — ADR-0050 deleted it and folded `imemcheck`/`dmemcheck`/`cover` into the required PR gate |

M1 is reached. **M2 is the milestone that erases the verified→unverified regression** — treat it
as the real finish line, not M1. `b2dafcc` cleared M2's blocker (RVFI is driven, the monitor is live
in both sim legs) and `86e2721` landed the ladder port itself (`wrapper.v` / `checks.cfg` /
`imemcheck` / `dmemcheck` / `cover` / `equiv.sh`, ADR-0006) — but **M2 is not reached.** M2's own
wording is "re-proves everything the serialized core proved", and 15 red checks plus an inconclusive
`reg` plus a non-converging `equiv.sh` is not that (ADR-0047 has since deleted `equiv.sh`; see
term 4). ADR-0023 lists what closing it takes. An empty
`formal/EXPECTED_FAIL`, read together with `formal/EXPECTED_CHECKS` per ADR-0033, is a **necessary
condition and not a sufficient one** — this file called it "the signal" until ADR-0037, which is
the language of sufficiency for something that was only ever necessary, and a change nobody
believes completes the milestone duly satisfied it.

`rtl/csrs.v` took that baseline from 11 entries to 9 (both `csrw_*`) on a 79-check
ladder; landing `ill_ch0` red made it 10 on an 82-check ladder; **the trap change took it to zero
on that same 82-check ladder.** All ten were the trap gap — nine misalignment, one
illegal-instruction — and all ten closed together, which is the attribution behind them holding.

**The baseline is empty and M2 is still not reached** (ADR-0037). **M2 requires all six of the
following.** Closing it means deleting terms from this list — a burn-down with the same shape as
`formal/EXPECTED_FAIL` itself. **Term 4 is met as of ADR-0047 and term 5 with `formal/COMPLETE_EXCLUSIONS`**;
ADR-0045 additionally closes terms 2 and 3, which this list has not yet been rewritten for. **Only
term 6 is open.** Read each term's own text, not this sentence:

1. **`formal/EXPECTED_FAIL` empty and `formal/EXPECTED_CHECKS` matching — MET at `6309b3e`**: 82
   generated, 82 pass, both set equalities in both directions. **ADR-0045 reopened this and ADR-0046
   closed it again**, which is the more useful thing to know about it: the depths those 82 checks run
   at were derived against a pipeline two changes old, and a depth below its floor goes green having
   stopped asking. They are re-derived now, and the derivation is *measured* — `hang` gives the
   worst-case first retire (F = 6), `liveness` the worst-case retire gap (G = 6 since ADR-0060; it
   was 4), both in seconds, both re-runnable. **Three depths moved with the sixth stall reason** —
   `insn 15 → 19`, `ill 15 → 19`, `reg 15 20 → 15 22` (ADR-0057 predicted them, ADR-0060 re-measured
   them against the shipping RTL) — and `insn 19` and `reg 15 22` still clear their floors by
   **one cycle**, so **any change that adds a stall reason, lengthens a stage, or widens the
   scoreboard past its three fixed slots must re-measure F and G before it lands.** Two of the last
   four such changes did not; the ladder's wall time roughly doubled paying for this one.
2. **The mul/div checks run without `RISCV_FORMAL_ALTOPS`**, or ADR-0010's gap is closed by a named
   oracle that does. `insn_mul`/`insn_div`/`insn_rem` passing still says nothing whatever about the
   real multiplier or divider. **ADR-0045 closed this on the "or" clause by naming
   `components_executor` + `test/exec_tb.v`, and ADR-0049 then measured that the proof half of that
   pair did not cover what the closure claimed.** ADR-0051 makes the claim true and ships the
   evidence: eleven mutations, both oracles, both tables. **The rule that came out of it is the part
   worth keeping — a term does not close on an assertion that an oracle exists, only on a
   demonstration of what it catches.** ADR-0045 warned that a third move on this term should prompt
   asking whether the criterion describes anything real; it does, and the two prior closures were
   unmeasured.
3. **`reg_ch0` returns a verdict** rather than exhausting its budget (ADR-0023). It is the one
   check tying RVFI's self-report back to `rtl/regfile.v`; ADR-0032 measured exactly that hole by
   injecting an architectural write the entire ladder missed.
4. ~~**`formal/equiv.sh` converges**~~ — **MET at ADR-0047**, on ADR-0037's second clause: the
   guarantee is proven another way and `equiv.sh` is deleted rather than fixed. The mechanism is
   `make -C formal nonperturbation`, a structural check that the `-D RISCV_FORMAL` build with its
   `rvfi_*` ports deleted sweeps to a netlist identical to the plain build. **Amending a criterion is
   not meeting it** (ADR-0045's own closing line): this term closed by taking an "or" clause
   ADR-0037 wrote for the case, and it is the third of the six to close that way.
5. ~~**`formal/complete` passes**, or every check it declines has a recorded reason~~ — **MET**,
   on the second clause, and mechanised rather than written in prose. `complete` is the only thing
   in the tree that walks the ISA as a whole: each of the 70 `insn_*` ladder checks constrains
   `rvfi_insn` to its own encoding, so an encoding none of them names is invisible to all of them.
   It had never once passed — `DONE (FAIL, rc=2)` in seconds, at `assert(spec_valid && !spec_trap)`,
   on FENCE — and it ran behind `continue-on-error` on the nightly, which is the resting place
   ADR-0047 named when it retired `equiv.sh`. **`formal/COMPLETE_EXCLUSIONS`** now declares the two
   opcode classes riscv-formal ships no spec model for at the pin (MISC-MEM and SYSTEM — `fence`,
   `fence.i`, `ecall`, `ebreak`, `mret`, `wfi`, `csrr*`), each with its encoding and its reason, and
   `make -C formal complete` **PASSES at depth 50 in 12-16s** (measured across four runs;
   `complete_cover` is another 14-17s). The exclusion predicate keys on the
   encoding decoded from `rvfi_insn` and on nothing the core decodes, so a core that mis-decodes an
   instruction cannot excuse itself from its own check; `formal/check-complete-exclusions.py` holds
   the set to its baseline by ADR-0014 set equality in both directions **and** re-derives "no spec
   model" from the pinned clone, so a stale exclusion fails rather than silently covering less.
   **The set did not have to widen beyond those two classes to get a pass.** Both the check and its
   anti-vacuity control run on the PR gate with no `continue-on-error`. Fifth of the six to close;
   term 6 is the last one standing.
   **Read what it does not mean**: `mode bmc` at depth 50, nothing about spec *values*, and blind
   by declaration to the two excluded classes — `formal/complete_cover.sby` is what makes "the
   assertion was live, class by class" measured rather than assumed (twelve cover goals, all
   reached).
6. ~~**Every check the repo owns is on a gate that can fail, and that gate is green**~~ — **MET at
   ADR-0052, and it took one code change to get there because the term was not met when it was
   written.** ADR-0050 wrote "with no `continue-on-error` anywhere in it" and the `formal` job
   **had one**, on the step that runs the ladder — surviving on the reasoning that the step's status
   "is not the signal". It is: `formal/Makefile`'s `check` puts a leading `-` on the sby sub-make and
   then ends in `check-baseline`, so that command's exit status already *was* the comparison's. What
   the suppression bought was that generation aborting, or the audit script dying, reached the
   summary as a green step. It is gone; the gate step below it runs on `!cancelled()` so the report
   still lands. **`grep -c continue-on-error .github/workflows/*.yml` is now 0 in every file.**
   The evidence, all on the **gate's own run** rather than a local one:
   the required `formal` job green on merged main (`d2e736e`, run `30724575535`), **per step** —
   ladder 85/85 with *both* set equalities matching in both directions, `imemcheck` PASS,
   `dmemcheck` PASS, `cover` PASS, `complete` PASS, `complete_cover` PASS — **4m22s against a
   20-minute timeout**. The three memory checks were one step until ADR-0052 split them, because
   `make` stops at the first failure and a shared step cannot record three outcomes.
   **What it does NOT cover, stated rather than assumed**: ADR-0050's concrete clause is the formal
   checks, and `make cosim-suite` (deliberately off every gate, ADR-0032) is outside it — term 6 is
   about the formal ladder, not about every gate in the repo. That is not a regression this term
   introduced and is recorded under "what does not work". `make waves` (`testbench.vvp` and its
   baked-in program, elaborated by CI's `elaborate` job) was recorded there too at the time, but is
   now graded, as its own step unrelated to this term — it was never one of term 6's checks either
   way (ADR-0055).
   **The intent never changed** — the ladder's verdict must be observed by something automated that
   can fail — and a required PR check is a strictly stronger instrument than a job ADR-0022 itself
   described as not gating merges. **This is the third move of an M2 criterion**, and ADR-0045 said
   a third should prompt asking whether the criterion is real; ADR-0050 asks it explicitly rather
   than restating the term and moving on. A fourth should be treated as evidence it is not.
   The history is still worth keeping: the `|| true` and the missing `-k` were fixed earlier, and
   what survived both was the graded comparison piped into `tee` — a `run:` block without an
   explicit `shell:` key is `bash -e {0}`, errexit but **not** pipefail, so its exit status was
   `tee`'s, always 0.
   **ADR-0022's "that comparison step's exit status is the job's real signal" had therefore never
   held**, and stayed accidentally true only because the ladder kept matching its baseline
   (ADR-0037). General rule: never put the graded command in a pipeline in a `run:` block.

**That quiet corner was read directly against the spec, and it was hiding a real defect**
(ADR-0048). `rtl/csrs.v`, `rtl/decoder.v`'s trap arm and `rtl/writeback.v` were read against the
privileged specification **with `test/csr_tb.v` and the trap `.S` files closed**, the expectation
written down first and the benches opened only afterwards — because a bench written from the same
reading as the RTL cannot detect a misreading, and the number of assertions is irrelevant to that.
Three findings. **`c.ebreak` (16'h9002) decoded to nothing**, so it raised illegal instruction
(cause 2) instead of breakpoint (cause 3): it is the `rd == 0`, `rs2 == 0` corner of quadrant 2's
`funct4 == 4'b1001` row that `instr_cjalr` and `instr_cadd` each exclude by a different field. It
survived because **the suite's `.option norvc` discipline guarantees the assembler never emits a
compressed trapping encoding** — correct for the resuming handler (ADR-0008), and a blind spot
exactly one instruction wide. **A counter write at the carry boundary moved the other half**:
`csrw mcycle` with `mcycle == 0xffff_ffff` advanced `mcycleh`, because the increment was suppressed
per half rather than per counter; `rvfi_csrw_check` reads only the self-report and never observes
the register, so no ladder check can see it in any configuration. **Two required CSRs were missing** —
`mstatush` and `mconfigptr`, which Sail reads and this core trapped on, measured. All three are
fixed, each with the test that caught it written first, and the suite is **55/55 with both `.S`
baselines empty**. Everything else read as correct, `test/csr_tb.v` included: it disagreed with the
specification nowhere.

**The third finding was briefly landed red instead of fixed, and that was the wrong call.** The
audit declined to widen ADR-0005's CSR set on the grounds that the ADR called it *exact*, making it
an ISA-scope decision rather than an audit's. The reasoning was sound and the premise was not: a
core that traps on a spec-mandatory register is non-conformant, minimality was never a licence for
that, and `mstatush`/`mconfigptr` both read zero legally — the fix is two lines and no new state.
**ADR-0005's "exact" is struck**; the set is a floor. The lesson is not about CSRs: a scope
statement that can make a conformance gap look like a design choice will eventually be believed by
someone with no reason to doubt it.

**Two caveats qualify what any green ladder here means, and neither is closable by burning
down a list.** Every check is `mode bmc` — PASS means "no counterexample within this check's
configured depth", not that the property holds, and there is no `mode prove` anywhere on the
ladder. And riscv-formal ships **no spec model at all** for `ecall`/`ebreak`/`mret`/`csrr*` at the
pin, so the behaviour M3 actually added is checked by `test/asm/trap.S`, `test/csr_tb.v`,
`test/decoder_tb.v` and `rtl/decoder.v`'s own component proof — against assertions this repo wrote,
not against an oracle. An empty baseline is loudest exactly where the ladder is quietest. **That
second caveat is mechanised now** rather than left to prose: `formal/COMPLETE_EXCLUSIONS` is the
list of encodings with no oracle, `formal/check-complete-exclusions.py` re-derives it from the
pinned clone on every run of `complete`, and a pin bump that *added* one of those spec models fails
the gate until the exclusion comes out. The caveat is unchanged; what changed is that it can no
longer quietly widen.

## Pointers

- Design brief: [`docs/ideas/finish-the-rewrite.md`](docs/ideas/finish-the-rewrite.md)
- Area/fit brief: [`docs/ideas/fit-the-core-on-the-up5k.md`](docs/ideas/fit-the-core-on-the-up5k.md) —
  **the core's logic fits: 3875/5280 logic cells, 73%** under a local Homebrew yosys as of ADR-0054,
  down from 6971/132%. The synchronous-read regfile did most of it (ADR-0042); ADR-0054's `next_pc`
  refactor took a further **312 cells, 4187 → 3875, both measured under the same toolchain**.
  **Quote a number with the toolchain it came from** — the pre-ADR-0054 tree measured 4208 on CI's
  pinned OSS CAD Suite against 4187 locally, 21 cells apart on the synthesiser build alone
  (ADR-0052), on top of the ±50 edit churn below.
  `make fit` is a ratchet against `FIT_MAX_LC` (4100 now) and a **non-required CI job**.
  **`make fit`'s top still does not place**, which is expected and unrelated to logic: it presents
  **231 `SB_IO` against sg48's 39**, so nextpnr always fails on a pad. **The SoC does place**, on
  four pins, because both its memories are internal — `make soc-timing`, ADR-0054. Read ADR-0038
  before quoting any area number: yosys cell counts are blind to unpaired flip-flops and have
  produced two wrong estimates already, and the core's number and the SoC's are two different
  designs. **The brief's two `reg_ch0` claims are false and are struck in place** (ADR-0040), and its
  recommendation of a *negedge* read is superseded (ADR-0042).
- **Timing**: `make soc-timing`, and `soc/timing_sweep.sh` for the four-placement spread that is
  the only honest form of the number. **74.34 · 75.81 · 76.88 · 78.80 ns = 12.69 – 13.45 MHz**
  (ADR-0064, local Homebrew yosys 0.67+post), against 95.74 – 99.47 ns on the same tree before it.
  **12 MHz is met at every placement, and it is a REQUIREMENT rather than an intent** (ADR-0066):
  `SOC_MIN_MHZ` is 12.0, `soc-timing` is a required check, and CI's own hardware measures 12.72 MHz.
  ADR-0062 is where 12 stops being a
  round number: the board crystal is 12 MHz and the part oscillator divides 48 to 48/24/12/6, so the
  step below 12 is 6. **A red `soc-timing` is a design problem, not a floor to lower.** The one change that did it addressed `rtl/regfile.v`'s write-through bypass
  from a registered copy of the address pair — read invariant 6 before touching `operand_stall`.
  The earlier figure was 88.51 ns = 11.30 MHz, 38.7% logic / 61.3% routing, on
  `imem.in_range → decode → next PC → imem.in_range2` (ADR-0054); the comparable one before that is
  `rtl/regfile.v` placed alone at 86% routing (ADR-0038), so the whole SoC is still
  routing-dominated, just less extremely than an isolated 32:1 mux.
- **Those 41 logic levels are 25 LUT levels and 17 carry hops, and quoting the single number is how
  this repo talked itself into a sprint** (ADR-0058). A LUT level costs **3.31 ns** — its own delay
  plus the `LocalMux` + `InMux` into it — and a `carryin -> carryout` hop costs **0.34 ns** and no
  interconnect at all. Per-level interconnect is paid about 23 times, not 41, and a change that
  trades a carry hop for a LUT level gets shallower by icetime's count and slower in nanoseconds.
  `make soc-timing` prints both counts and both per-hop costs now.
- **The fetch loop is not uniquely critical: a second path sits within 1.7% of it** (ADR-0058).
  Decode → `stall` → `instret` → `minstret`'s 64-bit counter measures 87.04 ns against the loop's
  88.51, and the baseline already reaches 87.43 at one of its four placements. **That cap was
  written as "about 11.5 MHz" and it was a property of the tree it was measured on, not a bound**:
  ADR-0062 re-measured the second path at 81.91 ns worst, and ADR-0064's change cleared 12.69 MHz at
  four placements because the same bypass select sits on both paths — `trap_pending` tests
  load/store misalignment, which is computed from `reg_rs1`. A cap derived by deleting a term from
  one path stops being a cap when the term is on the other one too. Two
  restructurings were built and measured against this: computing the ROM's range flag in parallel
  with the `next_pc` mux (measured at its ceiling, with the comparator deleted outright — 1.8%
  *slower*, four placements, no overlap) and flattening the `next_pc` priority chain into a one-hot
  mux (inside the noise at best, 6% worse at worst, because a wide flat mux routes worse than a
  chain on a path that is 61% interconnect). Sharing the JALR target with the effective-address
  adder costs 5–7%, and muxing the addends instead of the sums costs 4–6% — a LUT in front of an
  adder puts a whole carry chain after it. **The lever is the decode head** (`imem.in_range → instr
  → {rs1/rs2, immediate, hazard}`), which every one of these paths shares.
- **`SOC_SEED=<n>` places the same netlist differently**, because one placement is a sample. The
  spread on unmodified `main` is 87.43–88.51 ns across four placements (1.2%), and ADR-0057 measured
  1–2% generally. Compare distributions.
- **`make soc-timing` has a churn axis of about 3.6%, three times `make fit`'s, and it is measured.**
  Two logically identical spellings of `rtl/memory.v`'s write/read arms — a module **not on the
  critical path** — give **88.51 ns / 41 logic levels** and **91.67 ns / 53**, on 11 logic cells'
  difference in the netlist; 11 cells anywhere is enough for nextpnr to redistribute placement. Each
  figure is reproducible run to run (nextpnr is seeded); it is the *edit* the number is unstable
  under. So **a `make soc-timing` delta of a couple of percent is not evidence of anything** — but
  `SOC_MIN_MHZ` no longer buys margin against that band. It is 12.0, the board clock, and the worst
  local placement is 12.69, so there is **5.75%** between them against a 3.6% churn band and a 1–2%
  placement band (ADR-0066). That is accepted: a regression floor may sit below the measurement so
  noise does not cry wolf, and a requirement floor may not — an edit that lands at 11.9 MHz has
  stopped meeting the requirement, whatever moved it. `rtl/memory.v` therefore ships the FLAT spelling of its arms and says so at
  the site — the tidier nested one costs 11 cells and 3.6% of the only timing number this project
  has, which is the trade ADR-0038 already made twice in the other direction.
- **`make fit` has a churn floor of roughly ±50 cells, and a ratchet has to sit above it.**
  Measured by sweeping `rtl/executor.v`'s `mul_div_counter` across four widths that are all
  functionally identical — the counter's range is 0..32 and yosys already constant-folds every bit
  above 5, so the **flip-flop count is byte-identical in all four** (1757). The logic cells are not:
  `[6:0]` (what is checked in) → **6971**, `[5:0]` → 7008, `[7:0]` → 7027, `[8:0]` → 7024 —
  non-monotonic, spanning 56 cells, entirely LUT4 churn from ABC re-mapping and nextpnr packing
  (LUT4-only 5159 → 5198, LUT4+DFF 587 → 587, DFF-only 1170 → 1170, CARRY 41 → 39 for the first
  two). Each figure is reproducible run to run; it is the *edit* the number is unstable under, not
  the tool. Two consequences: **a `make fit` delta smaller than ~50 cells is not evidence of
  anything**, and ADR-0038 decision 1a's utilisation ratchet needs a margin wider than this floor
  or it will go red on changes that synthesize to the same hardware.
  This is also why `mul_div_counter` is still `[6:0]` for a value that never exceeds 32. Narrowing
  it is a legibility change that removes no register and **costs 37 cells measured**; ADR-0038
  rejected the shifter merge at 19 cells *saved* on legibility grounds, and accepting a hygiene
  change at 37 cells *spent* would cut against that ruling. Read this before proposing the
  narrowing again — it is measured and declined, not overlooked.
- Decisions: [`docs/adr/`](docs/adr/) — **sixty-nine ADRs, sixty-eight of them accepted**, plus a
  deferred list. Re-derived by counting: `ls docs/adr/*.md | wc -l` is 70, one of which is
  `README.md`, and the status column in that README carries exactly one non-accepted entry
  (ADR-0016, superseded by ADR-0018). This line has now been behind four times — "forty-five
  accepted", then "forty-seven", then "fifty-five" while seven more had landed, then "sixty-three"
  while five had — so **re-derive it with the two commands rather than incrementing it**. The first
  lapse missed ADR-0049 through ADR-0051 in one go, and the third missed ADR-0056 through ADR-0062.
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic)
- Work is tracked in Linear, project **Little CPU** (team JEF). Named here so you know where the
  queue is — but nothing in this repo should depend on it, and no ticket ID belongs in the code.

**Deferred behind future ADRs** — forwarding network, radix-4 divider, interrupts, and the
**bootloader** (SPRAM cannot be initialised, so the SoC ADR-0054 builds cannot get a program's
`.data` into RAM at power-on; that needs a `.text` copy stub or ADR-0044's SPI-flash path). FPGA
timing closure came off this list at ADR-0054, which took the measurement — and the design **meets
12 MHz at every placement** (ADR-0064), which is a **requirement** rather than a declaration as of
ADR-0066. (The negedge-BRAM regfile came off this list in ADR-0042, **decided
against**: it is 99 cells cheaper and costs no cycles, but the generated ladder cannot model a
mixed-polarity design at all, so `reg_ch0` would never again run against `rtl/regfile.v`.) Each trades away simplicity the current design depends on; none are
safe to build while the core is unverified. (Sail co-sim came off this list in ADR-0032: the
harness exists and ADR-0039 runs it over the whole suite behind `make cosim-suite`, still opt-in;
wiring it into `make test` or CI's required set is decided *against* and needs a new ADR to change.)
