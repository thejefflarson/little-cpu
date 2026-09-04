# Little CPU

A hobby RISC-V core in SystemVerilog on the open toolchain (Yosys / iverilog / SymbiYosys — no
vendor EDA). Target **RV32IMAC_Zicsr_Zifencei_Zkt**, machine mode only; home is an ice40 up5k
running at the board's 12 MHz crystal.

**Four goals: fast, simple, readable, formally verified.** They are not in tension by default — the
clearest spelling of a thing is often the fastest and the smallest, and the formal harness is what
makes aggressive simplification safe. Every rule in this file serves one of the four; a rule that
stops serving them gets deleted. This file is a rulebook, not a changelog: it states what is true
and where it is enforced, and cites the ADR under `docs/adr/` that holds the measurement behind a
rule. The measurement's narrative stays in the ADR. The README is a short front door that points
here.

Four habits carry the goals:

- **Measure a conflict; never assume one.** `make fit`, `make soc-timing` and `make ecp5-timing`
  are the instruments, one per design. Do not discard a measured win because it sounds like an
  optimisation, and do not take a tidier spelling that costs measured speed or area without
  recording the trade: `rtl/memory.v`'s flat write/read arms and `rtl/executor.v`'s `[6:0]`
  `mul_div_counter` are tidier spellings declined on measured cost, and `rtl/decoder.v`'s guess at
  the next instruction's register pair was declined on a 0.83% margin and shipped when the margin
  was re-measured on the tree it would land in (ADR-0089, ADR-0093). **A margin that declines a
  change is a measurement with a date on it** — re-take it on the tree you mean to spend it in.
- **An inherited conclusion is not a measurement, and the cheap test outranks it.** A report that
  something cannot work — an upstream issue, a datasheet's worst case, a pin table — is evidence
  about someone else's setup until it has been run here. `sudo iceprog` was reported useless on
  macOS and is what made the board flashable (the `prog` recipe's comment); `SB_HFOSC`'s ±10% trim
  was quoted to argue the UART could never work, and the part measures nominal (ADR-0130); two
  community pin tables give the UPduino's clock as 41 and 44 against the vendor's own 20
  (`soc/upduino.pcf`'s header). The cost of testing one is usually a single command.
- **Prove the property, then spend it.** Find a place the design pays for a property it already
  proves — a priority chain over proven-disjoint flags, a comparator that cannot differ — simplify
  it, and let the riscv-formal checks, the component proofs and the `.S` suite say whether the
  property still holds. A marking is spent against an assertion, never against belief:
  `(* parallel_case *)` is legal only where a `$onehot`/`$onehot0` check covers the exact flags in
  that arm list (ADR-0068).
- **A grader that cannot fail is not a grader.** Every graded comparison must have a demonstrated
  red direction that fails for the reason it was written; `make probe-gates` forces all of them and
  runs as a prerequisite of `make test`. Five of this repo's recorded defects were comparisons
  whose failure path had never once run.

## Design commitments

**These can and should change when a change moves the four goals forward together.** A commitment
is only a means to the goals, and none is held harder than the others. To change one: measure the
improvement, show the other three goals still hold — measured where they are measurable (`make
fit`, `make soc-timing`, the riscv-formal checks, the `.S` suite) — and record the amendment as an
ADR. The evidence is required because breaking a commitment is silent: tests stay green while the
design rots. Older ADRs cite these as `invariant N`; the numbers are kept in parentheses so those
references still resolve.

- **No wrong-path state** (1). No state may exist that a later cycle must un-commit — no flush
  logic, no kill signal. The decoder owns the PC and is its only driver; the fetch address is
  published a cycle early (`next_pc` → `imem_addr_next`), and a stalled cycle re-presents the same
  words. This keeps the BMC depths small and derivable, retire unfiltered, and `pcloop`'s
  induction free of speculative state. Enforced by `formal/pcloop.sv`, `rtl/decoder.v`'s `FORMAL`
  block and `test/decoder_tb.v`.
- **All traps are detected and committed in decode** (2). Nothing faults after decode; a trap is a
  branch to `mtvec` on the same override the jumps use, which is what makes CSR commit precise with
  no reorder buffer. A refusal counts as committed in decode only when it arrives with the
  *address*, in the cycle decode reads the word — never with the response. Three do: the fetch bus
  (`rtl/imemory.v`'s `imem_fault`, cause 1); an atomic, whose effective address is rs1 verbatim so
  `rtl/memory.v`'s range test about it reads a register output with no adder and costs no logic
  level (causes 5 and 7, ADR-0109); and a plain load or store, whose address is a sum — every
  same-cycle spelling put that sum in the fetch loop and cost the board clock (seven priced:
  ADR-0104, ADR-0116, ADR-0128), so the answer arrives a cycle late unless `reg_rs1` is deep inside
  a mapped window (in it, and in neither its first 2 KB block nor its last), in which case raw
  register bits answer whatever the offset. The instruction is held in decode and nothing issues,
  so a deferred *answer* is not a deferred *trap*. The period is a null at sixteen paired
  placements; the price is +13.79% of Dhrystone's cycles (ADR-0129). Enforced by
  `components_traps` over `formal/traps.sv`, `test/decoder_tb.v`'s region vectors and
  `rtl/decoder.v`'s `FORMAL` block.
  **So this core has a layout preference, and the shipping linker scripts pay it** (ADR-0158). A
  64 KB window is 32 blocks and 30 of them are deep, so a program whose stack and `.data` sit one
  2 KB block clear of the edges reaches the fast arm on essentially every access, and one that does
  not pays a cycle per access. Every script under `test/` starts `.data` one block above `ram`'s
  origin and puts `__stack_top` one block below its top, which takes Dhrystone's `REGION` column
  from 212,752 cycles to 2 on RTL nobody touched. **A program that ignores the convention pays what
  Dhrystone used to pay.** It is graded in the linker rather than remembered: two `ASSERT`s per
  script, with `test/probe_gates.sh` planting a bad layout in each and requiring the link to fail.
  `soc/compare/dhry.lds` is left at the conventional layout on purpose, so the stale cross-core
  product is re-taken as a pair rather than one side at a time.
- **Every inter-stage struct carries a `valid` bit** (3). A bubble is `valid = 0`; retire is
  `valid` reaching writeback, which gates `wen` and drives `rvfi_valid`.
- **Hazards are stall-only except where the executor's own slot already holds the answer** (4). A
  RAW hazard stalls in decode against a two-slot scoreboard, and forwarding excuses that stall only
  for `out.rs1`/`out.rs2`, only from `executor_out` (never `out`, whose instruction has not reached
  the executor), and only for encodings with no other decode-side reader of the same register:
  `instr_math` for rs1, the same plus store/AMO/`sc.w` write data for rs2, never a branch's rs2.
  `executor_out.rd_ready` holds back a load's, an AMO's, `lr.w`'s or `sc.w`'s still-unpacked result,
  so those hazards stay stall-only. RVFI's `rs1_rdata`/`rs2_rdata` report the forwarded value, not
  the register file's — the monitor checks `rd_wdata` against exactly those two fields, so the
  register file's answer makes every forwarded retire self-contradictory. A cycle that is both a
  hazard and an operand-fetch cycle is charged to the scoreboard (`test/cxxrtl.cc`'s bucket order),
  so those two columns of `make cycles` are not separable, and the scoreboard's share is an upper
  bound on one workload, never the prize. Measured on the tree this
  lands on: Dhrystone's hazard column falls 357,798 → 317,207 cycles and the run
  1,543,497 → 1,506,943, which is 0.758 → **0.777 DMIPS/MHz**; the `.S` suite goes
  41,052 → 39,096 (ADR-0154).
  Still declined on the clock: forwarding to every operand reader (ADR-0083), a fourth scoreboard
  slot in place of the write-through bypass (ADR-0092) and writing a committed result into the idle
  register port a cycle early (ADR-0100), which is the evidence to beat — every candidate so far
  touched the write-through bypass or the fetch loop it sits in.
- **CSR instructions, `mret` and `fence.i` serialize** (5) — held in decode until execute, access
  and writeback are empty. Two reasons share the mechanism and must not be collapsed: the first two
  so a one-cycle architectural update cannot interleave with older instructions; `fence.i` because
  text is writable and the fetch address publishes early, so an older store's write edge must pass
  first (ADR-0061). The emptiness check reads **three** slots — `accessor_out.valid` is routed in
  separately because a store writes no register and is invisible to the scoreboard's two
  (ADR-0099). The `.S` suite cannot observe the `fence.i` term — a text store's own fetch-port
  steal already holds back the only fetch the wait could cover — so `test/decoder_tb.v` is its
  grader (ADR-0105).
- **The regfile read is synchronous, and the answer belongs to the address pair presented the
  previous cycle** (6, 9). Decode presents a pair, bubbles a cycle (`operand_stall`) whenever what
  was presented is not what the instruction reads, then issues — observing in the issue cycle the
  architectural value of rs1/rs2 *including a writeback committed that same cycle*, via write-first
  into the read register and then the write-through bypass. What it presents on an issuing cycle is
  a **guess at the next instruction's pair**, mapped out of the fetch window's successor word by
  `rtl/regsel.v`, instantiated twice so a compressed successor is decoded rather than masked away
  (ADR-0089, ADR-0093); a pair learned from history, which reaches a redirect, was built twice and
  declined on the placement tail (ADR-0101, ADR-0113). The bypass selects on a **registered copy**
  of the address pair and is correct only because `operand_stall` lets nothing issue until the held
  pair is the pair the issuing instruction reads (ADR-0064): narrowing `operand_stall` breaks it
  with nothing to say so except two `test/regfile_tb.v` vectors and `reg_ch0`, so touching
  `operand_stall` is an amendment, not a tuning change. That is a rule about the guard, not the
  neighbourhood: what is *presented* on a stalled cycle is a separate question. The standing
  liveness probe: delete the rs2 write-through bypass and `reg_ch0` must go SAT — run it before
  believing any `reg_ch0` result under a changed configuration.
- **Stalls are one global broadcast over two mechanisms** (8): a divider stall **holds**
  `decoder_out` unchanged (an issued instruction the executor has not consumed); every other reason
  **bubbles** (nothing issued). A `fetch_stall` coinciding with a freeze HOLDS — bubbling would drop
  an issued instruction. Every in-flight non-`x0` `rd` must be visible to the scoreboard on every
  cycle between issue and the regfile write-through, with no gap. **Eight** reasons raise `stall`,
  and it is exactly their OR: the divider, the atomic write cycle, the decode scoreboard,
  serialization, the operand-fetch cycle, the stolen fetch window, the ungranted bus, the
  load/store region wait. A reason is declared in **six** places: the decoder's signal, its OR, its
  publish arm and its `FORMAL` asserts; `test/decoder_tb.v`'s OR-identity check and its both-ways
  vectors; `test/cxxrtl.cc`'s bucket; `test/stall_report.py`'s `REASONS` and `HEADINGS`;
  `formal/pcloop.sv`'s `f_may_stall`; and this list. Each hold/bubble ruling is only arm order in
  the publish block, so each is asserted in the `FORMAL` block and vectored both ways in
  `test/decoder_tb.v`. Three arms carry their reason with them: the **atomic write cycle** bubbles
  because the executor has already consumed the AMO and a hold would retire it twice (G is 6 with
  it, ADR-0106); the **ungranted bus** bubbles because `rtl/executor.v` publishes `stalled` and
  takes no input that freezes it, so a held `decoder_out` would be consumed twice — it is tied low
  in every single-hart integrator (`formal/MULTIHART_TIE_OFF`), `rtl/littledual.v` alone drives
  it, and the core does not decide its own wait: decode publishes `bus_request` and the platform
  ANDs it against its grant, because a grant term inside the decoder would close the loop through
  the arbiter; the **region wait** bubbles for the scoreboard's reason, is the only reason about an
  answer rather than a resource, and is charged last in `make cycles` so its column counts exactly
  the capture cycles. The memory transaction is presented from `decoder_out` during the executor's
  cycle and the request block is gated on the cycle the executor takes it, because a re-presented
  request is idempotent for RAM and not for a device; `components_accessor` and
  `test/accessor_tb.v`'s transaction count grade that (ADR-0099).

Retired numbers, never reused: 7 (the generated-but-tracked monitor) lives under Verification; 9 is
folded into 6.

## ISA target

RV32IMAC_Zicsr_Zifencei_Zkt, M-mode only, `misa = 0x4000_1105` (none of the three Z-extensions has
a `misa` bit; the `-march` string is the only place they are claimed). Traps implemented:
instruction access fault = 1, illegal instruction = 2, breakpoint = 3, load misaligned = 4, load
access fault = 5, store misaligned = 6, store/AMO access fault = 7, ecall from M = 11.
Instruction-address-misaligned (0) is unreachable — C makes 2-byte targets legal — so not
implementing it costs nothing and closes nothing. C stays because code density is a product
constraint on the up5k (ADR-0002, ADR-0003).

**Every refusal has one shape: the platform decodes its own map and hands the core one bit that
arrives with the address.** `rtl/imemory.v` publishes a fetch outside the text window → cause 1.
`rtl/memory.v` answers its range test about `atomic_addr` → cause 5 for `lr.w` and 7 for the nine
AMOs and `sc.w`, alignment outranking the region, which makes everything outside the data RAM —
text, timer, UART, SPI controller — `AMONone` and `RsrvNone`; the reservation is refused there too,
so a platform that tied the fault bit high still could not let an `sc.w` claim a write that went
nowhere (`test/accessor_tb.v`, `components_accessor`). The twelve plain load and store encodings
raise 5 and 7 a cycle late through the region wait. **The fast arm is one-sided on purpose**: a
miss means "wait for the flip-flop", never "fault", so a window narrower than three 2 KB blocks —
the timer's, the UART's, the SPI controller's — never reaches it; asking about `rs1`'s page instead
holds 12 MHz everywhere and is declined because it makes `mcause` a function of the base register
rather than of the access (ADR-0129). **One term reaches `next_pc`** — "an atomic faults" — with
the cause split answered off the fetch loop; two terms missed 12 MHz (ADR-0109).

**The eleven A instructions are decoded, executed and claimed** (ADR-0106, ADR-0108): Zaamo and
Zalrsc in full, `.aq`/`.rl` decoded and ignored, cause 4 for a misaligned `lr.w` and 6 for the
other ten; `misa` bit 0 is the only runtime statement of that. The suite builds at
`-march=rv32imac_zicsr_zifencei_zkt`, so six programs execute atomics — `amo.S`, `amominmax.S`,
`amotrap.S`, `lrsc.S`, `lrsclock.S`, `amoregion.S` — and five agree with the reference model, the
semantic oracle for the nine functions, LR/SC's five invalidation events and the four causes
(`lrsclock.S` arms the timer the model lacks). The `.S` suite is not an oracle for them:
`test/monitor.sim.v` value-checks nothing in an A retire because the pin ships no spec model for
any of the eleven, so an `OBSERVED_FLOOR` line for one of those programs is a retire count and not
evidence anything was compared. Every claim they make is an in-band assertion that reads its memory
result back into a register, because `test/cosim.cc` compares registers and never memory.

**The ISA string has one source and `make test` grades it**: `test/march_test.sh` declares
`rv32imac_zicsr_zifencei_zkt` and checks all seven sites that state it, four of which are silent
when wrong — `soc-rom`'s `.c` shape, `DHRY_CFLAGS`, `COREMARK_CFLAGS` and the copy of
`DHRY_CFLAGS` in `soc/depth/cycles.py` all build programs with no atomic in them. Two spellings
that look identical must **not** move with it: `formal/checks.cfg`'s `isa rv32imc` and
`MONITOR_GEN -i rv32imc` name what riscv-formal generates a spec model for, and the pin has none
for A or for Zkt, so widening either generates nothing.

**Zkt is claimed, and it adds no instruction and no `misa` bit** (ADR-0134). It promises that a
listed set — RV32I arithmetic, logical and shift, the four multiplies, and the arithmetic C
encodings — executes in time independent of its operands' VALUES; `DIV`/`REM` (32 cycles, or one
when `rs2 == 0` or on `INT_MIN / -1`), loads, stores, branches and jumps are excluded, and the
exclusion is what makes the claim true. Load/store timing here varies with address arithmetic, not
cache state, and the constant-time model treats addresses as non-secret. Three graders carry it:
`test/zkt_isolation_test.py` grades the taint half on the ELABORATED NETLIST (ADR-0137 records why
the source-text version was replaced) — it seeds taint at `reg_rs1`, `reg_rs2` and
`executor_out.rd_data`, follows a flip-flop's D to its Q, and requires that none of the signals
`STALL_TARGETS` names is reachable, with `region_stall` the one reason allowed to read a register
value and blocked as a taint source past its own hop; its header carries the argument.
`rtl/decoder.v`'s `FORMAL` block asserts `region_stall`'s gate and `ls_access`'s exact membership
as single-trace equalities and `components_decoder` proves them, with `formal/decoder-zkt-probe.py`
as the red direction and prerequisite. `rtl/executor.v`'s `FORMAL` block asserts that the four
multiplies resolve in the `init` state with no counter and `components_executor` proves it, with
`formal/executor-zkt-probe.py` as the red direction and prerequisite; its header says why the
mutation is narrowed to `rs2 != 0` and why only the basecase leg is read.

**One interrupt: the machine timer, cause `0x8000_0007`.** `mie.MTIE` is the only writable bit of
`mie`; `mip.MTIP` is `rtl/timer.v`'s line and read-only; `mip.MSIP`/`mip.MEIP` are read-only zero,
which the spec allows for an interrupt that can never become pending. `mtime`/`mtimecmp` are four
words at `0x0002_0000` and **the map reserves eight**, one `mtimecmp` and one `mtip` per hart
(`NHARTS`, ADR-0124); `test/memmap_test.sh` reads every `BASE` under `rtl/` and refuses one inside
the span. The layout is **deliberately not a CLINT's**. The interrupt is taken on a cycle that
would otherwise have issued, because `stall` outranks the trap arm of `next_pc` — so it waits out a
divide, a region wait and a serialization with no logic of its own, and is **not** a stall reason.
Worst-case response: 33 cycles, set by the divider. `mtimecmp` resets to zero, so `mtip` is
asserted out of reset, and both enables resetting to zero makes that harmless (ADR-0082). Three
facts are the platform's to state and firmware cannot derive them: **`mtime` ticks once per clock
cycle**; **MTIP is a level**, posted until `mtimecmp` exceeds `mtime`, so a handler that returns
without moving `mtimecmp` is re-entered before the instruction at `mepc` runs; **an RV32 `mtimecmp`
update is the spec's three stores in the spec's order** — low all-ones, high, low — and
`test/timer_tb.v` and `test/asm/mtimer.S` each fire the spurious interrupt the other order gives on
purpose. A change in the comparison may reach `mtip` late and never early, and `test/timer_tb.v`
is the only grader of that (ADR-0118).

**Conformance is not negotiable against minimality.** Every CSR the privileged spec mandates for
RV32 M-mode is implemented, the 87 performance-monitor addresses included — most legally read
zero, and the whole monitor is four address compares with no state behind them (ADR-0103). What a
read-only-zero CSR costs is its address decode, not its write mux, so ADR-0096's one-LUT figure for
every WARL mask is not the prior for adding one. The CSR set is a floor, not a closed list
(ADR-0048). A register the spec merely recommends is still owed a decision, against the spec
sentence and not the reference model: Sail is only as independent as
`test/sail/rv32imac_zicsr.json` makes it, and a key that says a feature is absent makes the model
agree with the core rather than with the specification (ADR-0103).

## Verification — four legs, each load-bearing

| Leg | Role | Catches what the others can't |
|---|---|---|
| **cxxrtl** | primary runner | real mul/div arithmetic, long/randomized runs |
| **iverilog** | microscope | waveforms, `$display`, four-state X, second elaboration frontend |
| **riscv-formal** | oracle | per-instruction semantics against a spec model, bounded by depth and by what the pin models |
| **Sail co-sim** | independent architectural oracle | register writes no self-reporting oracle sees; required on `main`, off `make test`'s path |

What a green result does and does not mean:

- **The riscv-formal checks run under `RISCV_FORMAL_ALTOPS` and never check the real multiplier or
  divider.** The oracles for that arithmetic are `test/exec_tb.v` and `components_executor`, each
  shown red against a hand-run mutation table when built (ADR-0051); neither has a patch in
  `test/mutations/`, so `make mutation-check` does not re-run that table.
- **Every generated riscv-formal check is `mode bmc`**: PASS means no counterexample within that
  depth, not that the property holds. Depths derive from F (worst-case first retire, from `hang`)
  and G (worst-case retire gap, from `liveness`), both 6, declared in `formal/checks.cfg`'s
  `#derive` lines. **Any change that adds a stall reason, lengthens a stage, or widens the
  scoreboard must re-measure F and G before it lands** (ADR-0046); `make -C formal remeasure-fg` is
  that sweep. `formal/genchecks-audit.py` grades every depth against its family's floor and a depth
  below it fails generation (ADR-0107), because a shallow depth does not go red — it goes green
  having stopped asking.
- **riscv-formal ships no spec model for SYSTEM, MISC-MEM or AMO at the pinned SHA**, so trap, CSR
  and atomic behaviour is checked against assertions this repo wrote, not an oracle;
  `formal/COMPLETE_EXCLUSIONS` mechanises that boundary and a pin bump that adds a spec model goes
  red until the exclusion comes out. The generated instruction check drops every value comparison
  once an instruction traps and its pc checks accept whatever target the core reports, so
  `components_traps` over `formal/traps.sv` is the only thing that says a trap lands on `mtvec` and
  saves the right state, `mtval` included. That harness restates the memory map as parameters
  (`test/memmap_test.sh` compares the copy) so it can require an unanswered access to trap with
  cause 5 or 7, and it has two forced-red prerequisites, `traps-region-probe` and
  `traps-tval-probe`, each requiring a mutated core to fail at its own arm's line. For the twelve
  plain load and store encodings the generated checks grade the refusal itself: decode publishes
  the refused access's word address and exact write strobe on the fault channel and
  `rvfi_insn_check.sv` compares both (write mask exactly, read mask as a superset); the A encodings
  have no spec model, so `components_accessor` and `fault_ch0` are their graders (ADR-0109).
- **It ships no model of an INTERRUPT**, so the timer input is tied off in all five harnesses that
  instantiate `littlecpu` (`formal/INTERRUPT_TIE_OFF` lists them; `formal/traps.sv` builds decoder
  and CSR file without the core and is where the line is free). `components_traps` is the oracle
  for entry; `test/asm/mtimer.S` and `test/asm/mtimermask.S` for the whole path. `rvfi_intr` must
  be driven: both sim legs' monitor checks pc continuity across retires and stops only for a retire
  carrying it.
- **It describes ONE hart**, so the grant wait and the write snoop are tied off in the same five
  harnesses; `formal/MULTIHART_TIE_OFF` mechanises that in both directions and sweeps for an input
  every harness holds constant that no baseline declares. A free `bus_wait` would let the
  environment withhold the grant forever, which is what `hang` and `liveness_ch0` measure. Both
  tie-offs are inside F and G. `mem_lock` is deliberately not in that baseline — an unread output
  cannot weaken a check — and `components_accessor` asserts it covers exactly the AMO write-back
  cycle.
- **Both sim legs read the sanitized `test/monitor.sim.v` as their per-retire oracle**, so
  `test/sanitize_monitor.py` is a change to the oracle. The spec model has no memory map, so a
  refused access disagrees with it and is shown to the monitor all the same — dropping it leaves a
  hole in `rvfi_order` that the reorder buffer reads as a lost instruction. `rvfi_mem_fault` gates
  error 101 and the value comparisons behind it, and a retire carrying that flag must also report a
  trap or the monitor says so. `test/monitor.v` is generated but tracked: `make test/monitor.v`
  regenerates it at the pin, `make monitor-check` diffs a fresh generation against it, and neither
  licenses hand-editing.
- **iverilog derives a continuous assign's sensitivity from the call's arguments.** A function
  called from a continuous assign whose body reads module state silently under-evaluates, with no
  diagnostic (ADR-0037). Write such logic out. Treat a green iverilog run as evidence only if it
  could have failed.
- **A harness cannot reach inside an instance, and both ways of trying are silent.** yosys resolves
  no hierarchical reference — `decoder.ls_answer_valid` parses as an implicitly declared undriven
  wire the solver picks, on a Warning nothing grades — and it parses `bind` and drops it. The
  giveaway shape is passing the property and failing the BASE CASE, because the solver picks the
  wire adversarially in each direction (ADR-0129). So an invariant about a submodule's state is
  **asserted in that submodule**, and the composed task reads it with `-formal -noassume`:
  `formal/components.sby`'s `traps` task does, and `pcloop`'s split is still the coarse one.
- **iverilog is four-state and cxxrtl is not, so the iverilog leg is the only one that can see an
  X.** Decode reads register numbers out of the word *after* the instruction it is issuing, so an
  undefined ROM word turns the whole pipeline X — green under cxxrtl, green under every formal check
  because they drive that word as a free two-state input. `test/testbench.v` zeroes both ROM banks
  before poking its program, the way `soc/compare/rom_flat.py` zero-pads its image: a memory
  defined only where a program was written is not a model of a block RAM.
- **Sail co-simulation is a required check on `main`, in a job of its own** (ADR-0032 as amended by
  ADR-0095). `test/cosim.cc` reads the core's real `regs_a` and no `rvfi_*` signal, which is what
  lets it catch an architectural write the self-reporting oracles structurally miss; do not "align"
  it against `rvfi_valid`. Nothing on `make test`'s path reaches it. It **cannot** cover the
  interrupt path: `mtimer.S`, `mtimermask.S` and `lrsclock.S` are `INCONCLUSIVE SAIL-LIMIT` in
  `test/COSIM_EXPECTED_FAIL` because the model's timer is a CLINT at a different address with a
  different tick, so nothing is compared.

**Signed arithmetic in a reference model must be a self-determined statement of its own, never an
arm of a conditional expression.** IEEE 1800 sign-context rules silently evaluate the whole
expression unsigned, for negative operands only; this produced wrong oracles here twice (the
generated monitor's DIV/REM models, `exec_tb`'s SRA reference). `test/exec_tb.v` pins its
references against hand-computed literals before any RTL vector runs and stops with
`ORACLE BROKEN` if one has degraded; the monitor's models are rewritten by
`test/sanitize_monitor.py` on a graded site count. A new reference model owes the `exec_tb` kind
of self-test: an oracle that is wrong fails correct hardware and teaches the reader to distrust the
bench.

## Measurements and ratchets

Three instruments, four designs — never merge their numbers. `make fit` is the core alone (its top
never places: 231 `SB_IO` against sg48's 39, expected); `make soc-timing` is the SoC, which places
and times; `make ecp5-timing` is that SoC on the other part; `make dual-ecp5-timing` is the dual
top, ECP5 only.

- **`make fit` has a churn band of about ±50 packed cells**: functionally identical edits move the
  count that much from re-mapping alone, and the band measures wider than nominal, so `FIT_MAX_LC`
  is derived from a span measured on the tree and budgets the whole span (the derivation is the
  comment above it in the Makefile). The count is **toolchain-dependent by as much as the band with
  no fixed sign** — quote the `fit` job's number and treat a local run as a sanity check;
  `soc/baseline_sweep.sh` stamps a sweep with its tool versions and `soc/baseline_summary.py`
  refuses to subtract two that disagree. The suite is not pinned, so a suite bump belongs on the
  list of causes when `FIT_MAX_LC` or `SOC_MIN_MHZ` trips. It is **top-dependent** (ADR-0094) and
  **spelling-dependent** (ADR-0097): quote a number with the tree and the text it was measured on.
  **A parameter tied off to today's value is not free of the mapper**: only a widened port and an
  *untaken* `generate` arm measured zero, which is why `rtl/imemory.v` and `rtl/timer.v` spell the
  first window and the first hart on their own (ADR-0124). **Grade a ceiling on packed
  `ICESTORM_LC`, not `SB_LUT4`**: the two disagree in magnitude and sign on the same netlist
  (ADR-0112). **A generated cell's module prefix is ancestry, not ownership**: after flatten yosys
  names a cell after a neighbouring net, so only totals are comparable across builds, and a
  `-noflatten` per-module census is not a harvest estimate (ADR-0112).
- **`make soc-timing` has a ~3.6% edit-churn band and a 4–9% placement spread**, and
  **`soc/bands.py` is the one place either number is stated**; `test/band_source_test.py` grades
  that both ways, with `docs/` exempt because an ADR is a measurement with a date on it. **A
  go/no-go is twelve to sixteen seeds, paired by seed, quoting worst, median and spread** — never
  worst-of-N against worst-of-N; `soc/timing_sweep.sh`'s default four is a look, not a verdict, and
  a short sweep takes a shorter look at the same distribution rather than sampling a tighter one
  (ADR-0121). **Margin is a sample**: quote it with the sweep it came from. The period is
  spelling-dependent by more than `fit` is, so quote the distribution of the text that ships
  (ADR-0106). **A candidate whose cost is a variance needs sixteen seeds, not eight** — the tail is
  what `SOC_MIN_MHZ` grades, and eight seeds passed a candidate sixteen declined (ADR-0113). **A
  median inside the band is a null that does not even reproduce** (ADR-0121).
- **A tied-off PORT is not a tied-off change.** An input held constant folds before mapping; an
  **output the integrator does not read does not fold**, and adding one unread output to
  `rtl/littlecpu.v` moves the SoC +44 `SB_LUT4` on its own. So a ports-only change can still owe
  the sixteen seeds: the tied-off multi-hart surface swept −0.2% at the worst placement and +1.5%
  at the median over sixteen, inside the band (recorded in the commit that landed it, fa10ebd), and
  the dual top's later ports moved the digest again, so that sweep is owed (ADR-0125). Do not read
  `fit` for this class of edit.
- **`make ecp5-timing` is a different CLASS of instrument**: the same `littlesoc` with no fork and
  no `ifdef`, placed where there is no `icetime`, so nextpnr's own engine both places and grades.
  `soc/ecp5_report.py` is the single reader and refuses every shape of "nothing was measured". The
  three mapping censuses **gate** (`DP16KD`, `TRELLIS_DPR16X4`, `MULT18X18D` — each falling back to
  soft logic is silent in a frequency and enormous in area); the frequency **publishes** with no
  ratchet; the constraint handed to the placer is a pinned constant the design must miss, or the
  run measures the target. No ECP5 band has been derived and `soc/bands.py` refuses to answer for
  the part; up5k's figures do not transfer. Pinning `clk` to the module's oscillator pin is not
  cosmetic: the pad decides where the global network is entered, and `soc/littlesoc.lpf`'s header
  records the one placement that read faster unpinned.
- **The DUAL configuration is a FOURTH design, ECP5 only.** Two fetch windows are two copies of the
  banked ROM — 32 block RAMs against the up5k's 30 — so no up5k number describes it.
  `make dual-ecp5-timing` gates three censuses that double where the design does and not where it
  does not, which is what says two windows are two copies of one storage and two harts are two
  whole cores (ADR-0125). `make dual-smoke` is off `make test`'s path and not on CI, and runs one
  program both ways — both harts, and hart 1 held in reset — because a dual harness that measures
  one hart looks exactly like a working one. **A signal a single-initiator design never had to
  drive to zero is not a signal two initiators may OR**: `rtl/accessor.v` publishes rs2 on
  `mem_wdata` for every issuing instruction, so ORed across two harts it lost most of a smoke
  program's counted increments, invisibly to a bus-exclusivity check (ADR-0125). Read the
  producer's idle behaviour before joining two of anything.
- **12 MHz is a requirement, not a regression floor** (ADR-0066). `SOC_MIN_MHZ` is 12.0 — the board
  clock, whose next divider step down is 6 — and it does not slide; when it trips, fix the design.
  **The step above is 24** (`SB_HFOSC` gives 48 / 24 / 12 / 6), so the target for Fmax work is
  41.67 ns, about half of today's period, and a few-percent idea can be declined against it in a
  minute (ADR-0078).
- **`make dhrystone` and `make coremark` are the figures comparable to another project's** —
  Dhrystone to VexRiscv, CoreMark to Hazard3 and most cores published since — and neither is a
  gate. Dhrystone: **0.777 DMIPS/MHz, 9.32 DMIPS at 12 MHz** at `-O2` (ADR-0154), quoted with the
  absolute figure because Fmax above the requirement is margin and not speed (ADR-0089), and with
  the flags, the compiler, the string library and **the linker script** — the program prints the
  first three and will not compile without them, and `test/bench/bench.lds` asserts the fourth at
  link time. **It went 9.10 → 7.97 → 9.10 → 9.32, and the moves differ in kind**: the region wait
  spends 13.79% of Dhrystone's cycles to make an out-of-region access fault (ADR-0129), insetting
  the layout gives those cycles back with no RTL change and the netlist digest unmoved (ADR-0158),
  and executor-only forwarding buys the last step in the datapath (ADR-0154). A CPI regression
  with no conformance behind it is still a regression, and a figure recovered by moving the
  software is the firmware ceasing to pay a cost, never the core getting faster. **CoreMark is
  SIMULATED AT 16 KB OF ROM**, double the part's 8, against `test/testbench.v`'s `ROM_WORDS`, and
  every printed figure says so; the five algorithm files are vendored unmodified and pinned by
  `test/bench/coremark/PINNED.sha256` (ADR-0136). **2.013 CoreMark/MHz** on the shipping layout
  against 1.811 on the conventional one, so it travels with the linker script the way the DMIPS
  figure does (ADR-0158). Hazard3's published 4.15 CoreMark/MHz is its RP2350 build, not its iCE40
  one, and quoting it against an ice40 core is the mixed-configuration error ADR-0098 names.
- **The only cross-core comparison that means anything is one harness**, `soc/compare/`: same part,
memories, program, toolchain and seeds, against the VexRiscv in the pinned riscv-formal clone and
Hazard3's iCE40 build (`soc/compare/hazard3_pin.mk`, ADR-0139). **A product is a measurement only
when both factors were taken on one tree**, and the current one is **1.16× against VexRiscv in its
favour**, twelve seeds a side on one tree (ADR-0098 as amended): 23.14 DMIPS against 26.87 at each
core's worst placement, 1.17× median against median. Its halves pull opposite ways — the clock is
1.60× theirs at the worst placement (30.13 against 48.24 MHz) and the cycles are **1.379× ours**
(741.1 against 1021.9 per Dhrystone, 0.768 against 0.557 DMIPS/MHz). **Read the product, not a
half.** Between ADR-0129 and here the cycle half rose and the clock half fell, and the product did
not move at all, so either factor quoted alone tells you the opposite of the other. Re-take
both halves before quoting the product, with what each side is and with the caveat that Dhrystone's
cycles are simulated at a larger map than the clock is placed at (`make compare-dhrystone` prints
the block arithmetic; ADR-0098 lists the distortions). **Hazard3's iCE40 build is the third core in the
harness, and its DHRYSTONE factors are now both measured** — `soc/compare/dhry_tb.v`'s marker
mechanism, built for VexRiscv's identical CSR-free gap, needed wiring rather than invention. Its
CoreMark half is still not built: `CSR_COUNTER=0` means no `mcycle`, so it cannot self-time that
run (ADR-0139). Read the three-way Dhrystone row only at the one ISA all three cores share, RV32I,
which is what makes every pairwise ratio in it a comparison rather than three configurations. Two graded checks stand in front of every number:
`soc/compare/placed_vs_synth.py` refuses a placed count under `COMPARE_MIN_RATIO` of the core's own
synthesis — an all-NOP image once placed a quarter of this core with a plausible critical path
beside it (ADR-0086) — and `make compare-smoke` requires all three cores to publish the same values,
which caught Hazard3's first bus adapter publishing all-X words (ADR-0139). The harness gives
VexRiscv no data path to its ROM, so keep read-only data out of ROM there.
- **A register in the fetch loop is a fetch stage, and it is priced and declined** (ADR-0087): the
  loop's tail comes out for 3–4 levels, its head not at all (a bank output mux is one `SB_LUT4`
  that ABC folds into the decode reading it, and a register there forbids the sharing), the two
  parts disagree in sign, and the best product of clock against cycles is +0.4%. Redirects are
  7.15% of the suite's issues and 16.92% of Dhrystone's, the opposite ordering from the RAW share,
  so no depth argument stands on the suite alone.
- **yosys and ABC already do everything derivable from the expression** — dead bits, common
  subexpressions, duplicate adders — so an edit that restates the same arithmetic is a null
  (ADR-0088). **Dead logic is free only where ABC has a LUT input to fold it into**:
  `rtl/writeback.v`'s `wen` masks are unread by every consumer and cost +2.83% of median period and
  the requirement at the worst of sixteen seeds to delete, so they stay and say so (ADR-0115,
  ADR-0117); read the consumer before calling a redundant term free. **What the tools cannot use is
  a fact from outside the expression** — a power-of-two parameter, an aligned window, an address
  bit a trap guarantees zero — and that is an area lever, not an Fmax one: eleven such edits are
  −169 SoC cells with the period a null (ADR-0088), and 352 cells of ballast moved the median +2%
  and the worst placement the other way, so **occupancy does not set the tail on this part**
  (ADR-0121). Where such a fact is load-bearing it is an elaboration `$fatal`:
  `rtl/{imemory,memory,timer,uart,spiflash}.v` and `rtl/littlecpu.v`'s copy of the map refuse a
  non-power-of-two depth or an unaligned `BASE`, and `make window-test` forces all six red.
- **Closed for area, by name — read the ceiling in the ADR before reopening a block for cells**:
  `rtl/executor.v` (ADR-0090); `rtl/decoder.v`'s compressed expansion and immediate generation
  (ADR-0094, ADR-0097); `rtl/csrs.v`, `rtl/regfile.v` and the SoC read-back bus (ADR-0096);
  `rtl/accessor.v`, `rtl/timer.v`, `rtl/memory.v`'s gating and the A decode rows (ADR-0112); the
  AMO result mux, now a per-bit truth table (ADR-0119); the timer's compare (ADR-0118). Four fabric
  facts fell out of those ceilings: **sharing an arithmetic unit is not a saving** — a 33-bit adder
  and a 33-bit 2:1 mux place at the same cost (ADR-0119); **a carry chain is nearly free**
  (ADR-0112); **a conditional increment is a clock enable, not a mux** — riding the carry-in
  instead saved three cells and missed 12 MHz at six of six, so read the `SB_DFFE*` census and
  sweep seeds even for an area null (ADR-0096); **a sticky bit set on a crossing is wrong where both
  registers reset to zero** (ADR-0118). **Every ceiling answers "how many cells does deleting this
  save", none answers "what sits on the path"** — no block is closed for period.
- **There are two loops around the fetch address, within 2 ns of each other**: `ROM RDATA → instr →
  rs1 → scoreboard → stall → next_pc → ROM address`, and `accessor_out.rd_data → writeback → the
  regfile's write-through bypass → branch comparator → next_pc → ROM address`. Work moved out of
  one arrives in the other at full price, and every attack on the pair is priced and declined:
  `stall` at the ROM's read-enable pin (a null, ADR-0091), a fourth scoreboard slot for the bypass
  (a product 5.1% worse in DMIPS, ADR-0092), the early register write (ADR-0100). **What did pay is
  a cone that had no business being in either loop**: `instr_error` read the muxed register numbers
  rather than the raw fields, and reading the fields is −2.47% of median with the worst placement
  flat, bought by depth rather than area (ADR-0115, ADR-0117). **A ceiling is perishable** — the
  `stall` arm of `next_pc` was worth −3.8% when filed and a null three merges later (ADR-0091) — and
  so is a CPI cost, so re-take one in the tree you mean to spend it in.
- **Read logic levels apart**: a LUT level costs ~3.3 ns with interconnect, a carry hop ~0.34 ns
  without, so a change that trades a carry hop for a LUT level gets shallower by icetime's count
  and slower in nanoseconds. **The fetch loop will not take a LUT level at any area price**: the
  largest decode edit measured, −103 SoC LUTs for one `next_pc` adder, missed 12 MHz at six of six
  (ADR-0097). `soc/depth/path_stages.py` attributes a path, not a decision, and its level count
  orders nothing (ADR-0116); `soc/routing_bins.py` shows the routing on that path is flat, with no
  long hop and no column to pin (ADR-0114). The SoC is routing-dominated and **there is no single
  lever**: reading `soc.timing.rpt` finds candidates, not wins, the decode head is a plateau at 3.3%
  deleted whole, and the period is in the fetch loop, whose inputs to `next_pc` are worth 21% only
  all together — collecting that means the pc stops depending on this cycle's decode, which is the
  no-wrong-path-state commitment (ADR-0076). **Measure the whole set**: a ceiling over one term
  bounds only that term.

Baselines and grading:

- Failure baselines (`test/EXPECTED_FAIL`, `test/COSIM_EXPECTED_FAIL`, `formal/EXPECTED_FAIL`)
  are **name-and-status pairs under set equality in both directions** — an unexpected pass is as
  red as an unexpected failure, and a baselined test failing a *different* way is red too.
  `test/OBSERVED_FLOOR`'s name set doubles as the suite manifest, `formal/EXPECTED_CHECKS` is the
  same for the generated check set, and `test/PROBES_EXPECTED` for `make probe-gates`'s labels
  (ADR-0140), each checked both ways so a suite that shrank is red rather than a smaller table that
  still "passes". Before adding a co-sim baseline entry, read that file's header — it is the
  decision procedure.
- **A `.c` entry's floor numbers are a silence bound, not an observation.** A C program's retire
  count is whatever that gcc inlined, and the two toolchains here differ by about 1% on identical
  source, so a `.c` line is 16 — the recorded blindness defects produced 0 or 1 retires — and
  `test/run_tests.sh` rejects a `.c` floor above 64 (ADR-0081).
- **Never put a graded command in a pipeline in a CI `run:` block**: the default shell is errexit
  without pipefail, so the step's status becomes `tee`'s. This held the formal CI job accidentally
  green for its entire life.

## Commands

```sh
make setup          # macOS: brew install riscv64-elf-gcc and svlint; Linux: prints the apt line
make lint           # svlint over rtl/, RVFI macros off and on. A required CI check
make lint-setup     # fetch the pinned svlint release into the tool cache
make test           # the test/asm suite (.S and .c) under cxxrtl + unit benches + probe-gates
                    # + every repo-scanning `*-test` target (memmap, march, band-source,
                    # retired-term, adr-numbering, port-connect, compare-geometry,
                    # tracked-ignored, tool-cache, pin-bump, abc-engine, zkt-isolation)
                    # + window-test, imem-share-test, board-elaborate, mutation-probe and
                    # dual-build; graded against EXPECTED_FAIL / OBSERVED_FLOOR
make test-units     # the unit benches alone; the list is checked against test/*_tb.v both ways
make elaborate-strict # yosys elaborates every simulation source through `check`; the
                    # required `elaborate` CI job
make probe-gates    # force every graded comparison red for its own reason. Two groups need a
                    # real tool: yosys for the zkt netlist walk, and the cross linker
                    # for the linker scripts' layout ASSERTs
make mutation-check # delete a term from rtl/ and require exactly the detectors
                    # test/MUTATION_DETECTORS pairs with it to go red, both ways. ~3.5 min,
                    # not on `make test`; `make mutation-probe` forces its graders red and IS
make window-test    # force the elaboration `$fatal`s in rtl/{imemory,memory,timer,uart,
                    # spiflash}.v and rtl/littlecpu.v red, in both frontends
make board-elaborate # read soc/board_upduino.v warning-free and force two breakages red.
                    # Does not read soc/upduino.pcf -- a stale pin there is nextpnr's to catch
make imem-share-test # map rtl/imemory.v at one and two fetch windows on both parts and
                    # require two windows to be two copies of ONE storage
make cycles         # the suite, every cycle charged to an issue or one of the eight stall
                    # reasons; nonzero on a cycle none explains. Prints the two load/store
                    # locality counters. Not on CI -- there is no CPI ratchet
make dhrystone      # Dhrystone 2.1 (test/bench) -> DMIPS/MHz and the same accounting;
                    # DHRY_RUNS picks the count. Not on CI, no ratchet
make coremark       # CoreMark (test/bench) -> CoreMark/MHz, SIMULATED AT 16 KB OF ROM;
                    # COREMARK_ITERATIONS picks the count. Not on CI, no ratchet
make waves          # iverilog leg -> waves.vcd; one baked-in program, not the suite
make monitor-check  # regenerate test/monitor.v at the pin into a temp file and diff it;
                    # `make test/monitor.v` rewrites the tracked copy
make fit            # the core's area number; ratchet on FIT_MAX_LC
make soc-timing     # the SoC place-and-time flow; requirement on SOC_MIN_MHZ. SOC_SEED picks
                    # a placement; soc/timing_sweep.sh runs four
make bitstream      # icepack the board wrapper into board.bin; BOARD_OSC=internal uses
                    # SB_HFOSC instead of the crystal. No board needed
make prog           # iceprog board.bin onto the UPduino; root on macOS
make suite-board    # the .S suite on the part, in batches, read back over the UART; root
make dhrystone-board # Dhrystone built for the board; flash with `make prog`, read the UART
make dual-smoke     # two harts, one text storage, one arbiter, under cxxrtl; one program run
                    # both ways. Off `make test` and CI. `make dual-elaborate` is iverilog's look
make dual-ecp5-timing # the dual top placed, ECP5 only; three censuses GATE, the frequency
                    # PUBLISHES, nothing merges with an up5k number
make ecp5-timing    # the SoC on ECP5 at a declared corner; three censuses GATE, the
                    # frequency PUBLISHES, no ratchet. ECP5_SEED picks a placement
make netlist-digest # the mapped netlist's digest; `make netlist-diff BASE=<ref>` names what
                    # moved. Digest unchanged, NO SWEEP IS OWED; changed, sixteen seeds are.
                    # Sound in one direction only. netlist-determinism is a prerequisite
make compare-timing # this core, VexRiscv and Hazard3 in ONE hx8k harness; COMPARE_CORE
                    # picks one, soc/compare/sweep.sh's COMPARE_CORES sweeps a subset.
                    # The placed-vs-synthesised check inside it is graded
make compare-smoke  # all three harnesses run one image in iverilog and must agree
make compare-dhrystone  # Dhrystone on all THREE cores, one RV32I image, one simulation ->
                    # DMIPS/MHz each, plus a fourth row of this core alone at its native
                    # ISA so the shared subset's cost is a number. COMPARE_DHRY_MHZ adds
                    # the absolute column. Not a gate, not on CI
make compare-coremark # the same for CoreMark, on littlecpu and Hazard3's iCE40 build
                    # only -- VexRiscv has no M in this harness's pinned build, so it
                    # cannot run an RV32IMA image. COMPARE_COREMARK_MHZ adds the
                    # absolute column. Not a gate, not on CI
make compare-product # both factors of every cross-core pair in one run, stamped into
                    # soc/compare/product.json with the commit, seeds and CFLAGS behind
                    # each number. COMPARE_PRODUCT_SEEDS picks the sweep (twelve by
                    # default). Not a gate, not on CI -- a scheduled workflow re-takes
                    # it weekly and opens a PR when it moved

make -C formal check                # the generated riscv-formal checks, always a fresh run;
                                    # both tie-off checks are prerequisites
make -C formal check-baseline       # re-grade a finished run without re-running
make -C formal components_decoder   # component proofs by k-induction (mode prove): read the
make -C formal components_executor  #   sby summaries, not the job colour. decoder and
make -C formal components_accessor  #   executor run their zkt probes first; pcloop runs
make -C formal components_pcloop    #   pcloop_cover; traps runs traps-region-probe and
make -C formal components_traps     #   traps-tval-probe; busarbiter runs busarbiter_cover
make -C formal components_busarbiter #  and busarbiter-probe -- each a forced red direction
make -C formal complete             # depth-50 whole-ISA walk minus COMPLETE_EXCLUSIONS
make -C formal complete_cover       # its anti-vacuity control
make -C formal imemcheck            # the fetch window's and the data bus's memory-interface
make -C formal dmemcheck            #   checks, and the cover goals
make -C formal cover
make -C formal genchecks-check      # the local genchecks copy differs from the pin only by
                                    # header and basedir; the `monitor-freshness` CI job
make -C formal nonperturbation      # RVFI instrumentation is unread by the core; structural
make -C formal remeasure-fg         # re-measure F and G, both flip points both ways, graded
                                    # against checks.cfg. ~20s, not on CI
make -C formal all                  # everything above but remeasure-fg, nonperturbation
                                    # and genchecks-check

make sail-setup     # once: fetch the pinned sail-riscv release
make cosim-run      # co-sim one program (PROG=add.S)
make cosim-suite    # the whole suite, graded against COSIM_EXPECTED_FAIL
make sail-reservation-probe  # ask the MODEL what a trap and an mret do to an LR
                    # reservation. No core runs. Not on CI
```

`make sail-setup` and `make lint-setup` unpack into `~/.cache/little-cpu` (`XDG_CACHE_HOME` moves
it), **outside the checkout**: a git worktree is given tracked files only and a downloaded tool is
gitignored, so an install inside the checkout is invisible from every worktree. `make test`
enforces it.

Toolchain: macOS `brew install riscv64-elf-gcc svlint`; Linux `apt install gcc-riscv64-unknown-elf`
and `make lint-setup`. Tests are freestanding — `.S`, and `.c` built `-nostdlib -ffreestanding`
against `test/crt0.S` — so no multilib or newlib. Formal needs the YosysHQ OSS CAD Suite, which CI
takes at the latest release; it is the one tool that floats. Everything else downloaded or vendored
is pinned and refuses a command-line override: riscv-formal (`formal/pin.mk`), sail-riscv and
svlint (`SAIL_RISCV_VERSION`, `SVLINT_VERSION`), Hazard3 (`soc/compare/hazard3_pin.mk`) and CoreMark
(`COREMARK_PIN`). CI runs on every PR (`.github/workflows/ci.yml`); read the required set live from
`gh api repos/thejefflarson/little-cpu/branches/main/protection`, not from comments.

## Engineering rules

- **Compiler and elaboration warnings are errors.** Two allowlisted exceptions, both documented
  where they are allowlisted: iverilog's `sorry: constant selects in always_* processes` for
  `rtl/writeback.v`'s `always_comb` struct reads (over-sensitivity, provably safe; do not add new
  ones outside that file), and yosys's `Deep recursion in AST simplifier` notice on the
  `elaborate` CI job.
- **Comments earn their place or go.** A comment must say something the code does not, in one or
  two plain sentences readable without leaving the file. Delete restatement, history (git has it),
  section banners and emphasis furniture. **No ADR numbers and no invariant numbers in comments** —
  state the mechanism instead; `git blame` finds the record. One exception: tripwires — a
  two-sentence warning that stops a fixed defect from being silently reintroduced. This discipline
  applies to this file too, with one difference: here an ADR number is the pointer that replaces
  the record. State the rule, cite the ADR that holds the measurement, and leave the narrative
  there.
- **Every non-trivial change adds or updates tests and runs the full suite** before being declared
  done. Elaboration succeeding is not a substitute.
- **Never commit build artifacts** (`test/rtl.cc`, `sim`, `*.vvp`, `*.vcd`, `rvfi_macros.vh`,
  `formal/` output dirs). `test/monitor.v` is the one deliberate exception.
- **riscv-formal is SHA-pinned.** A pin bump regenerates `test/monitor.v`, re-runs the generated
  checks, and re-derives the sanitizer's site counts and `COMPLETE_EXCLUSIONS` rather than editing
  them to silence a failure.
- **No ticket IDs in code, comments, ADRs, docs, or commit messages.** Cite the ADR, the commit
  SHA, or just say the reason. PR titles and descriptions are the exception.
- **`git config --local` in a worktree writes the checkout's one shared `.git/config`**, so a
  change meant to be local to one worktree is live in all of them until it is unset. Use an
  isolated clone for anything that needs its own git config.
- Prefer verified/first-party GitHub Actions; simplest approach unless asked otherwise.

## State

M1 (the pipeline runs the RV32IM suite), M3 (CSRs and machine-mode traps) and M2 — parity with the
serialized core this rewrite tore down, which was formally verified before the teardown — are all
reached; every M2 term was re-measured against merged main before the milestone was declared
(ADR-0079), and that audit is the record. **Declaring M2 is a claim about those six terms, not a
claim that the core is correct.** Three residuals outlive it: the multiplier is checked
differentially, not exhaustively; the divider is proved under a recorded magnitude restriction and
its completion assertions are basecase-unreachable at the configured depth; `complete` passes over
a recorded exclusion set, not over the ISA. `csrc_upcnt` says `minstret` strictly increases, **not**
that it advances by exactly the non-trapping issues; `test/asm/minstret.S`, `test/csr_tb.v` and
`components_traps` carry that half (ADR-0027). **Do not read empty baselines or an all-green
`make -C formal check` as "the core is correct"** — an empty `formal/EXPECTED_FAIL` is necessary,
not sufficient.

The SoC is 8 KB of ROM in block RAM plus 64 KB of data RAM in two of the part's four
`SB_SPRAM256KA`; `SOC_EXPECT_SPRAM` and `SOC_EXPECT_EBR` hold both counts exactly. It places, meets
12 MHz, and has run on the part: `make prog` flashes an UPduino v3.0, the UART's counter decoded
digit-perfect, and Dhrystone on the part matched cxxrtl to the cycle (ADR-0130). `make suite-board`
and `make dhrystone-board` run the suite and Dhrystone there; neither is graded on CI, because a
board is not always plugged in. SPRAM cannot be initialised, so `.data` rides in the ROM at a load
address `test/asm/boot.lds` puts there and `test/crt0.S` copies into RAM before `main`. Still
deferred: the radix-4 divider (a CPI lever that costs area, so never part of an area pass,
ADR-0038), booting a program out of the flash, an interrupt controller, more interrupt sources and
a vectored `mtvec`. The full forwarding network is declined, not deferred (ADR-0083); the
executor-only spelling ships (ADR-0154).

**8 KB of text is the ceiling, and it is the fetch loop's, not the part's.** `rtl/imemory.v`
refuses a `ROM_WORDS` that is not a power of two because both its range tests are reductions on the
address bits above the ROM, and 16 KB is 32 block RAMs against the 26 free (ADR-0135); a 12 KB
window as two power-of-two windows ORed is buildable and declined on period (ADR-0145). The two
free `SB_SPRAM256KA` cannot hold text either: fetch reads two neighbouring words every cycle from
two 32-bit banks, and at 16 bits wide a fetch window is four SPRAMs, the whole part. So
`test/asm/rvc.S`, at 12 256 bytes, still cannot run on silicon, and `soc/run_suite_board.sh`
batches.

**The configuration flash has a read-only controller, `rtl/spiflash.v`, and its own pins are not
wired to this board** (ADR-0135). Eight bytes at `0x0002_0028`: a data register whose read gives
`busy` in bit 8 above the byte the last exchange shifted in, and a write-only control register
whose bit 0 is the chip select; it is a mode-0 shift register that knows no commands. `busy` sits
beside the byte rather than in a register software writes because the reference model has plain
memory here, and polling a stored bit would spin on Sail and compare nothing. What is wired is the
UART's pin, and it is shared: the UPduino's pin 14 is `serial_txd` and `spi_miso` at once, so
`soc/board_upduino.v` drives it only while pin 16, the flash's chip select, reads released —
turn-on through a two-flop synchroniser, turn-off combinational, because a synchronised-only enable
would drive against the flash's own driver for two clock periods (`soc/miso_share_enable.v`). That
predicate answers "is the flash's driver off", not "is a host there" — `iceprog` parks the select
high for most of a session — so it cannot grant the on-chip controller its own pins.
`soc/pin_lockout.v` was built for that, ships bounded and graded standalone, and is not wired;
`sck`, `mosi` and `cs_n` are tied off. Reaching the flash's data path owes a real-board measurement
of that contention window first.

**A transmit-only UART is the only observable output a flashed bitstream has** — eight bytes at
`0x0002_0020`, above the timer's reservation because the four words between belong to a second
hart: a write-only data register and a `busy` bit, 8N1 at 115200 from a divisor of 104. No
receiver, no queue, no interrupt. It is outside `make fit`'s top, so its area is a `soc-timing`
number and never a `fit` one. Co-simulation cannot cover it (`test/asm/uart.S` is `DISAGREE AT 7`)
and `test/uart_tb.v`, which decodes the line with five of its own failures forced, is the only
oracle for the wire.

The suite is `test/asm/*.S` **and** `test/asm/*.c`, and `test/OBSERVED_FLOOR` names both. Anything
under `test/bench/` is deliberately outside it: both legs glob `test/asm`, and a benchmark that
needs two million cycles would time out against the runner's 5000. The two shapes differ only in
how `.data` reaches RAM — poked in by the harness for assembly, copied by the startup for C — and a
change to one shape's build is a change in FIVE places: `test/run_tests.sh`, `test/cosim.py`'s
`assemble()`, the Makefile's `soc-rom`, `test/dual_smoke.sh` and `test/dual_build.sh`.

## Pointers

- Decisions: [`docs/adr/`](docs/adr/) — re-derive the count by listing the directory; this file
  has been behind on it three times by quoting a number.
- Briefs: [`docs/ideas/`](docs/ideas/) — list the directory rather than trusting an enumeration
  here. Where a brief and an ADR disagree, the ADR wins.
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic).
- Work is tracked in Linear, project **Little CPU** (team JEF) — named so you know where the queue
  is; nothing in this repo depends on it.
