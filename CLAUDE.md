# Little CPU

A hobby RISC-V core in SystemVerilog on the open toolchain (Yosys / iverilog / SymbiYosys — no
vendor EDA). Target **RV32IMAC_Zicsr_Zifencei**, machine mode only; home is an ice40 up5k running at
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
  **1.1% of Dhrystone's**, +26 cells and a median period inside the churn band (ADR-0089). **A margin
  that declines a change is a measurement with a date on it**: the 0.83% that declined this one was
  re-derived twice against two later trees whose own worst placements were 2.08% and 4.75%, so
  neither the number nor the verdict travelled. The version that decodes a *compressed* successor
  lost a placement outright at 11.56 MHz and was declined on it; re-measured after ~490 cells came
  out of the SoC it lost none of ten and bought **16.3% of Dhrystone's cycles**, so it ships
  (ADR-0093). Occupancy moved in the direction the outcome moved, but the datapath and the toolchain
  moved too — what is claimed is that the decline did not survive re-measurement, not that
  congestion was the cause.
- **An inherited conclusion is not a measurement, and the cheap test outranks it.**
  A report that something cannot work — an upstream issue, a datasheet's worst case,
  a pin table — is evidence about someone else's setup until it has been run here.
  Three in one evening, each of which closed a question that was not closed:
  YosysHQ's icestorm issue says `sudo iceprog` does not help on macOS, and it is
  what made the board flashable; `SB_HFOSC`'s ±10% trim was quoted to argue the
  UART could never work, and the part measures nominal within error so 115200
  reads clean; and two community pin tables give the UPduino's clock as 41 and 44
  against the vendor's own file, which says 20 (ADR-0130). The rule is not that
  external reports are wrong — it is that **the cost of testing one is usually a
  single command**, and a conclusion adopted without paying it will be defended as
  though it had been.

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
  no reorder buffer. **The fetch bus refuses now and that is why the commitment survives, not an
  exception to it**: the instruction memory reports having nothing at the address it is answering,
  the report arrives with the word in the cycle decode reads it, and the fault is committed there
  with every other cause. A refusal that arrived with the *response* would not be, which is the
  distinction to hold on to. **The data bus refuses an ATOMIC and nothing else**: the data RAM
  answers its range test about a second address — the one an atomic's rs1 names, a register output
  with no adder in front of it — and decode raises causes 5 and 7 from it, for zero extra logic
  levels against the four the same test for a load or a store cost (ADR-0109). **The data bus
  refuses a plain load or store too now, and it costs cycles and no clock** (ADR-0129). Their
  address is a 32-bit sum, and every spelling that answered in the ISSUING cycle put that sum in the
  fetch loop: the price was monotone in which bit the loop waited for — bit 31 is +17.3% of median
  period, bit 12 is +9.1% (ADR-0104, ADR-0116) — and the cheapest exact one of the seven measured
  **+9.40% of median period with 9 of 16 placements under 12.00 MHz and the median at 11.97**
  (ADR-0128). **So the answer arrives a cycle late instead**, and only when it is not already known:
  `reg_rs1` deep inside a window — in it, and in neither its first 2 KB block nor its last — is
  answered whatever the 12-bit offset is, off raw register bits with no adder; anything nearer an
  edge bubbles one cycle and reads a flip-flop. **The trap is still committed in decode**, so the
  commitment holds rather than bending: the instruction is held there, nothing issued, and a
  deferred *answer* is not a deferred *trap*. Sixteen paired placements put the period at
  **−1.23% of median, 5 of 16 seeds slower, sign test p = 0.21 and 0 of 16 under 12.00 MHz**. The
  price is **+13.79% of Dhrystone's cycles**, 9.10 → 7.97 DMIPS.
- **Every inter-stage struct carries a `valid` bit** (3). A bubble is `valid = 0`; retire is
  `valid` reaching writeback, which gates `wen` and drives `rvfi_valid`.
- **Hazards are stall-only** (4). No forwarding network, and 35.7% of suite cycles is what that
  costs — **22.5% on Dhrystone, and the two are not separable from the operand-fetch cycle in either**
  (ADR-0084): a cycle that is both is charged to the scoreboard, which is why removing most of the
  operand column moved the scoreboard column *up* (ADR-0089, ADR-0093), and why deleting the load
  turnaround gave 11 995 of its cycles straight back as operand misses (ADR-0099). Read 35.7%
  as an upper bound on one workload, never as the prize.
  Both spellings were built and measured (ADR-0083): forwarding the executor's result to every
  operand reader buys 12.9% of cycles and misses 12 MHz outright at 9.49, and confining it to the
  executor's operands buys 7.5% and holds 12 MHz on 0.48% of margin against today's 3.35%. Deleting
  the scoreboard outright buys **0% of period** at its ceiling, so nothing in this direction pays for
  the muxes, and only one of the two in-flight slots can be forwarded from at all.
  **Writing a committed executor result into the register file on a cycle the port is idle** needs no
  operand mux at all — the existing write-through bypass carries it — and is the third measurement in
  the same chain: −9.1% of suite cycles, −6.5% of Dhrystone's, **+9.4% of median period and under
  12 MHz at four placements of six** (ADR-0100). That is the evidence to beat, and the pattern in it
  is that every candidate so far touches the bypass or the loop it sits in.
- **CSR instructions, `mret` and `fence.i` serialize** (5) — held in decode until execute, access
  and writeback are empty. Two distinct reasons share the mechanism: the first two so a one-cycle
  architectural update cannot interleave with older instructions; `fence.i` because text is
  writable and the fetch address publishes early, so an older store's write edge must pass first
  (ADR-0061). Do not collapse them. The emptiness check reads **three** slots —
  `accessor_out.valid` is routed in separately because a store writes no register and is therefore
  invisible to the scoreboard's two (ADR-0026 as amended by ADR-0099). **The `.S` suite cannot see
  `fence.i`'s half of this and no program can be written that does** (ADR-0105): a text store takes
  the fetch port for its write cycle, so the earliest fetch the wait could still be covering is one
  the memory already holds back, and deleting `instr_fencei` from `serialize` leaves the whole suite
  green. `test/decoder_tb.v` is the grader; `test/asm/selfmod.S` is red only when both mechanisms
  are deleted, and the wait stays because a memory that answers fetch and data at once has only it.
- **The regfile read is synchronous, and the answer belongs to the address pair presented the
  previous cycle** (6, 9). Decode presents a pair, bubbles a cycle (`operand_stall`) whenever what
  was presented is not what the instruction reads, then issues — and in the issue cycle observes the
  architectural value of rs1/rs2 *including a writeback committed that same cycle*, via two fabric
  forwarding points (write-first into the read register, then the write-through bypass). What it
  presents on an issuing cycle is a **guess at the next instruction's pair**, mapped out of the
  fetch window's successor word by `rtl/regsel.v` — the same mapping the issuing instruction goes
  through, instantiated twice, so a compressed successor is decoded rather than masked away
  (ADR-0089, ADR-0093). A guess taken from what really followed that address last time reaches the
  case a fetch window cannot — a redirect — and is **built twice, measured twice and declined**: it
  buys 8.0% of Dhrystone's cycles on both trees it was built on, costs +90 cells, and its period
  cost is a null at the median with the whole of it in the tail, which puts one placement of sixteen
  under 12 MHz (ADR-0101, ADR-0113). The pair presented and the pair being
  read are deliberately different signals there. The bypass selects on a **registered copy** of the
  address pair and is correct because `operand_stall` lets nothing issue until the held pair is the
  pair the issuing instruction reads (ADR-0064 as amended by ADR-0089) — narrowing `operand_stall`
  breaks it with nothing to say so except two `test/regfile_tb.v` vectors and `reg_ch0`. Touching
  `operand_stall` is an amendment, not a tuning change. **That is a rule about the guard, not about
  the neighbourhood**: what is *presented* on a stalled cycle is a separate question, and
  `operand_stall` remains the exact compare whatever is presented — so a change there is checked by
  the same signal rather than weakening it. The standing liveness probe: delete the rs2
  write-through bypass and `reg_ch0` must go SAT — run it before believing any `reg_ch0` result
  under a changed configuration.
- **Stalls are one global broadcast over two mechanisms** (8): a divider stall **holds**
  `decoder_out` unchanged (an issued instruction the executor has not consumed); every other
  reason **bubbles** (nothing issued). A `fetch_stall` coinciding with a freeze HOLDS — bubbling
  would drop an issued instruction — and that ruling is only arm order in the publish block, so it
  is asserted in `rtl/decoder.v`'s `FORMAL` block and vectored in `test/decoder_tb.v`. Every
  in-flight non-`x0` `rd` must be visible to the scoreboard on every cycle between issue and the
  regfile write-through, with no gap. **Eight** reasons raise `stall`, and it is exactly their OR:
  the divider, the atomic write cycle, the decode scoreboard, serialization, the operand-fetch
  cycle, the stolen fetch window, the ungranted bus, the load/store region wait. A reason is
  declared in **six** places, not
  the three this paragraph used to name: the decoder's signal, its OR, its publish arm and its
  `FORMAL` asserts; `test/decoder_tb.v`'s OR-identity check and its both-ways vectors;
  `test/cxxrtl.cc`'s bucket; `test/stall_report.py`'s `REASONS` and `HEADINGS`;
  `formal/pcloop.sv`'s `f_may_stall`; and this list.
  **The load/store region wait is the eighth and it BUBBLES**, for the scoreboard's reason rather
  than the bus's: nothing has issued, so a held `decoder_out` would hand the executor an
  instruction twice. Its arm is only arm order in the publish block, so it is asserted in
  `rtl/decoder.v`'s `FORMAL` block and vectored in `test/decoder_tb.v` both ways round.
  **It is the only reason that is about an answer rather than about a resource**, which is what
  fixes where it sits in `make cycles`' order: it is raised for every cycle its access spends
  waiting on anything, and charged last, so its column counts exactly the capture cycles — one per
  load or store whose base register sits near a mapped window's edge.
    **The ungranted bus is the seventh and it BUBBLES**, and unlike the sixth its arm was settled by
  construction rather than by argument: `rtl/executor.v` publishes `stalled` as an output and takes
  no input that freezes it, so a divider stall is the executor being busy in its own FSM and not a
  hold the decoder can borrow. A bus-grant wait leaves the executor idle, so a held `decoder_out`
  is consumed a second time. It is tied low in every SINGLE-HART integrator — so `make cycles`
  reports the column at zero, `test/decoder_tb.v` drives the input directly because nothing
  single-hart can otherwise say which arm shipped, and `formal/MULTIHART_TIE_OFF` is where the
  tie-off and its depth consequence are declared. `rtl/littledual.v` is the one platform that drives
  it, and **the core does not decide its own wait**: decode publishes `bus_request` — its stall
  conjunction with `bus_wait` left out, ANDed with the nine encodings that reach the data bus — and
  the platform ANDs that against its grant. A grant term inside the decoder would close the loop
  through the arbiter. G is measured on the tied-off machine, nothing under `formal/` builds two
  cores, and no generated check's depth applies to a configuration that has a wait in it.
  **The atomic write cycle is the sixth, and it BUBBLES** — the only reason to be added since the
  divider and the only one whose arm was a live question. An AMO reads its word while the executor
  takes it and writes the result back on the cycle after, so that cycle is spent to keep anything
  else off the bus; by then the executor has already consumed the instruction, and a hold would
  present it again — a second read beside its own write, and one instruction retiring twice. That is
  the opposite of the divider's ruling for the opposite reason, it is only arm order in the publish
  block, and it is asserted in `rtl/decoder.v`'s `FORMAL` block and vectored in `test/decoder_tb.v`
  both ways round. It moved **G from 5 to 6**, so the depths that cleared `F + 2G` by three now
  clear it by one (ADR-0106).
  **The load turnaround was the sixth and is gone** (ADR-0099): the bus transaction is presented
  from `decoder_out` during the executor's own cycle, so the answer is there when the instruction
  reaches the accessor. A held `decoder_out` would re-present it, which is idempotent for RAM and
  not for a device, so the request block is gated on the cycle the executor takes it —
  `components_accessor` and a transaction count in `test/accessor_tb.v` are what say so.

Retired numbers, never reused: 7 (the generated-but-tracked monitor) lives under Verification; 9 is
folded into 6.

## ISA target

RV32IMAC_Zicsr_Zifencei, M-mode only, `misa = 0x4000_1105` (neither Z-extension has a `misa` bit;
the ISA string is the only place they are claimed). Traps implemented: instruction access fault = 1,
illegal instruction = 2, breakpoint = 3, load misaligned = 4, load access fault = 5,
store misaligned = 6, store/AMO access fault = 7, ecall from M = 11.
**Instruction-address-misaligned (0) is the one remaining cause and it is unreachable** — C makes
2-byte targets legal — so not implementing it costs nothing and closes nothing.

**Causes 5 and 7 are raised for the eleven A encodings and for the twelve plain load and store
encodings**, and what remains asymmetric is the price, not the behaviour (ADR-0104, ADR-0109,
ADR-0128, ADR-0129). All three refusals have the same shape:
the platform decodes its own map and hands the core one bit that arrives with the *address*.
`rtl/imemory.v` publishes a fetch outside the text window on `imem_fault` → cause 1; `rtl/memory.v`
answers its range test about `atomic_addr` → cause 5 for `lr.w` and 7 for the nine AMOs and `sc.w`,
which makes the machine timer's four words and the whole text window `AMONone` and `RsrvNone`.
**An atomic's effective address is rs1 verbatim** — the A encodings put funct5/aq/rl/rs2 where the
I-immediate is read from — so that test reads a register output and measures **zero extra logic
levels**. A plain load or store has to read `immediate + reg_rs1` and hand the answer to `next_pc`,
and **the price is that wait and nothing about the region decode** (ADR-0116): the same three
windows and the same merge asked about `reg_rs1` instead of the sum are a null, and the cost is
monotone in which bit of the sum is waited for — bit 31 is +17.30% of median period, bit 12 is
+9.10%, and a **coarse** check that still reads bit 31 is +15.95%, a null against the exact one. The
cheapest exact spelling waits for bit 12 and it is what ships here: three tests per window on raw
register bits, the immediate's sign picking the pair, and the carry into bit 12 — recovered from the
existing adder as `sum[12] ^ rs1[12] ^ imm[12]`, never a second one — picking between two
precomputed answers one mux deep. **It reproduces the bit-12 figure on a later tree at +9.40% of
median period and puts 9 of 16 placements under 12.00 MHz** (ADR-0128), which is what closed the
same-cycle direction after seven spellings of it. **What ships answers a cycle late** (ADR-0129) and
is a null at sixteen seeds. **The fast arm is one-sided on purpose**: a miss means "wait for the
flip-flop", never "fault", so a window narrower than three 2 KB blocks — the timer's, the UART's —
never reaches it. That is the whole difference between this and asking about `rs1`'s page and its
neighbours, which holds 12 MHz at 16 of 16 placements and is declined because it makes `mcause` a
function of the base register rather than of the access.
**Which spelling reaches `next_pc` decides whether the board closes.** One term that says an atomic
faults, with the cause split answered off the fetch loop, swept a worst of 12.24 MHz over eight
seeds; two terms each carrying their own encoding test swept 11.82 and missed at two seeds of eight.
C stays because code density is a product constraint on the up5k (ADR-0002/0003). `fence.i` costs a
pipeline drain — see the serialization commitment.

**The eleven A instructions are decoded, executed and now claimed** (ADR-0106, ADR-0108). `rtl/`
implements Zaamo and Zalrsc in full — `amoadd.w`, `amoswap.w`, `amoand.w`, `amoor.w`, `amoxor.w`,
`amomin.w`, `amomax.w`, `amominu.w`, `amomaxu.w`, `lr.w` and `sc.w`, with `.aq`/`.rl` decoded and
ignored, cause 4 for a misaligned `lr.w` and cause 6 for the other ten. **`misa` bit 0 is the only
runtime statement of that**, and it moved with the reference model's `A` key in one change so the
transition was falsifiable: at `0x4000_1105` against the model's `A: false`, `csr.S` diverged, and
flipping the key agreed. **The suite builds at `-march=rv32imac_zicsr_zifencei_zkt`**, so six
programs execute atomics — `amo.S`, `amominmax.S`, `amotrap.S`, `lrsc.S`, `lrsclock.S` and
`amoregion.S` — and five of the six **agree with the reference model**, which is the semantic oracle
for the nine functions, LR/SC's five invalidation events, the two misalignment causes and the two
region causes. The `.S`
suite is not one: **`test/monitor.sim.v` value-checks nothing in an A retire**, because the pin
ships no spec model for any of the eleven, so an `OBSERVED_FLOOR` line for one of those programs is
a retire count and not evidence anything was compared. Every claim they make is an in-band
assertion, and every one reads its memory result back into a register because `test/cosim.cc`
compares registers and never compares memory.
**The bit is claimed and causes 5 and 7 are implemented for the eleven**, which is what closes the
caveat the claim shipped with. **An atomic outside the region `rtl/memory.v` answers faults** —
cause 5 for `lr.w`, 7 for the ten that write, alignment outranking the region — so the machine
timer's four words get the `AMONone` its PMA always should have said. The reservation is *also*
refused there, and it is no longer reachable from software because the fault gets there first: it
stays as the second half of one statement, so a platform that tied the fault bit high still could
not let an `sc.w` claim a write that went nowhere. `test/accessor_tb.v` and `components_accessor`
are what grade it now, and `test/MUTATION_DETECTORS` lost `amoregion.S` as a detector of
`sc-reports-success` for exactly that reason. **This is what closed the suite's one A-extension
co-simulation divergence**: the model has only `RsrvEventual` and `RsrvNone`, the core used to be
neither, and `RsrvNone` is what it is now.

**The ISA string has one source and `make test` grades it**: `test/march_test.sh` declares
`rv32imac_zicsr_zifencei_zkt` and checks all seven sites that state it, three of which are silent
when wrong — the Makefile's `soc-rom` builds a program with no atomic in it, and `DHRY_CFLAGS` is
copied verbatim into `soc/depth/cycles.py` with nothing else comparing the two. Two spellings that
look identical must **not** move with it: `formal/checks.cfg`'s `isa rv32imc` and `MONITOR_GEN -i
rv32imc` name what riscv-formal generates a spec model for, and the pin has none for A or for Zkt,
so widening either generates nothing.

**Zkt is claimed too, and it adds no instruction and no `misa` bit** (ADR-0134). It is a promise about
instructions the design already implements: a listed set — RV32I arithmetic, logical and shift,
`MUL`/`MULH`/`MULHSU`/`MULHU`, and the arithmetic C encodings — must execute in time independent of
its operands' VALUES. `DIV`/`REM`, loads, stores, branches and jumps are excluded from the list, and
that exclusion is what makes the claim true here rather than merely convenient: `rtl/decoder.v`'s
`stall` is the OR of eight reasons. **`test/zkt_isolation_test.py` grades the taint half of this on
the ELABORATED NETLIST, not on the source text** (ADR-0137, superseding ADR-0134's RTL-text-and-regex
version, which three rounds of review defeated on facts only elaboration knows: a value laundered
through `out.rs1 <= reg_rs1`'s own register, a procedural `branch_taken` no continuous-assign scan
ever followed, an `assign` inside an un-taken `` generate if (0) `` masking a real driver, and a
parameterised port width a `[N:0]` regex read as 1 bit). It runs `yosys -q -s` the way
`formal/check-nonperturbation.py` does, and reads cells and named nets off the JSON `write_json`
writes: `SEEDS` names `reg_rs1`, `reg_rs2` and `executor_out.rd_data`, every decoder input the
netlist's own measured width finds wider than the 5 bits a register NUMBER ever needs, and a
struct-typed port's field offsets come from a satellite module elaborated against `rtl/structs.v`
alone rather than this script guessing struct layout — a renamed field fails that elaboration
outright, an added or resized one fails a sum-of-widths check. Forward reachability from those
seeds, following a flip-flop's D to its Q the same as any other cell's inputs to its outputs, must
not reach seven of the eight reasons. The eighth, `region_stall`, does read a register value (on
the way to deciding whether a load's or a store's address lands near a mapped window's edge), is
gated on `ls_access`, and is blocked from being used as a taint SOURCE past its own one hop, so
everything that legitimately reads it (`stall` directly, and `out`'s own bubble condition, and so
`live_rs1`/`live_rs2`'s RAW-hazard check by way of `out.rd`/`out.valid`, which are blocked the same
way `region_stall` is for the identical reason: a register NUMBER or a single control flag read
back is not a VALUE, even though which number it holds can itself have been decided by a
value-dependent fault check) is judged as if that one read were invisible, while any OTHER path
still counts. What actually licenses blocking `out.rd`/`out.valid`/`out.is_amo` is that every path
a register VALUE can take to reach one is TRAP-MEDIATED — taking a trap is architecturally visible,
not a covert timing channel — not their width, which a 5-bit field can violate while still being
narrow; the script enforces the 5-bit bound as a tripwire on top of that argument, not in place of
it, so a field growing past the point that argument could even be attempted stops the run.
`region_stall`'s own captured state (`ls_capture`, `ls_answer`, `ls_answer_valid`) gets a second,
independent check: none of the eight reasons may read one of THOSE directly either, seeded from
their own bits rather than from a register.

**`region_stall`'s own gate, and that `ls_access` is exactly the eight base load/store encodings,
are proved by a SOLVER now, not walked structurally** (ADR-0137, second amendment). Three rounds
each defeated a different structural spelling of those same two facts — graded `ls_access` by NAME,
then treated a NOT as polarity-preserving (`region_stall = !ls_access && ...` bottomed out at the
leaf `ls_access`, asserting on every NON-load/store instruction), then let a MUX read as an AND
under odd polarity (`!(ls_settled ? !ls_access : 1'b0)` walked straight through it). Each fix was
correct and each new spelling got through, because both facts are single-trace exact-set-equality
invariants with no VALUE comparison in them at all — the same shape as `assert(out.is_amo ==
(out.is_amoswap || ...))`, which already sits in `rtl/decoder.v`'s own `ifdef FORMAL` block — and a
structural netlist walk is an over-approximation built for the 2-safety question the taint check
above still needs it for, not an exact decision procedure. So `rtl/decoder.v` now states both as
assertions instead: `assert(!region_stall || ls_access)`, and `assert(ls_access == (instr_lb ||
instr_lbu || instr_lh || instr_lhu || instr_lw || instr_sb || instr_sh || instr_sw))` — the eight
base encodings rather than `ls_access`'s own two named intermediates, since those are nothing but
this same OR restated in two pieces, and `instr_lw`/`instr_sw` each fold two further compressed
forms in behind their OWN `assign`. `make -C formal components_decoder` proves both by k-induction;
`formal/decoder-zkt-probe.py` is their demonstrated red direction, one core one line of
`rtl/decoder.v` from the shipping one per assertion, wired as a prerequisite the same way
`traps-region-probe` is of `components_traps`. `rtl/executor.v` is
the other half, and it is proved now too, not merely read (ADR-0134, amendment 1): `MUL`/`MULH`/
`MULHSU`/`MULHU` resolve in decode's `init` state with no counter, so they are constant-time
whatever the operands are, and `rtl/executor.v`'s `ifdef FORMAL` block states that as an assertion
beside the four that already pin a completing multiply's own value — `state == init` one cycle
after any of the four issues, guarded on `$past(state) == init` for the same reason those four
are: `in` is free while a divide runs, so an unguarded assertion could catch a divide's stale `in`
still naming the multiply that issued it. `make -C formal components_executor` proves it by
k-induction, the same task that already proves the multiplier's and divider's own arithmetic.
`formal/executor-zkt-probe.py` is its demonstrated red direction: one core three lines from the
shipping one, diverting `MUL` at `rs1 == 0 && rs2 != 0` into the divider's own arm — latching
`op_is_divu` alongside it so the divide arm's onehot invariant over its four op flags still holds.
**`rs2 != 0` is load-bearing, not incidental**: the divide arm's own first branch is `if (rs2 ==
0)`, which writes `32'hffffffff` while a real `MUL(0, 0)`'s value is `0`, so diverting `rs2 == 0`
too would make it a second, genuine counterexample to a pre-existing MUL-value assertion — reachable
from reset in the same steps as the latency one — and which of the two a basecase run happens to
report would be the solver's own arbitrary choice, not a property of the mutation. Excluding it
leaves only the real divide reachable, which completes at `0 / rs2 == 0` for every `rs2` the
mutation can reach, so the mutation changes only the multiply's LATENCY and the basecase leg —
the one over a reachable-from-reset trace — reports no failure but this one, at its own line.
Both this probe and `decoder-zkt-probe.py` require the failing line to come with an
`engine_N.basecase:` prefix on its log line for exactly this reason: `mode prove`'s INDUCTION leg
starts from an arbitrary, possibly-unreachable state, so a line it reports is not evidence about a
reachable trace at all. Wired as a prerequisite of `components_executor` the same way
`decoder-zkt-probe` is of `components_decoder`. `DIV`/`REM` are the one place this design is NOT
constant-time — 32 cycles ordinarily, one cycle when `rs2 == 0` or on `INT_MIN / -1` — which is
exactly why the list excludes them rather than the claim being false, and they get no such
assertion: asserting constant-time division would be asserting something false. The load/store
address-timing this leaves outside the list is backwards from the usual case: most cores' load
timing varies with CACHE STATE, and this one has none — it varies with ADDRESS ARITHMETIC instead,
and the standard constant-time model treats addresses as non-secret, so it sits inside the model
without needing its own exception.

**One interrupt: the machine timer, cause `0x8000_0007`.** `mie.MTIE` is the only writable bit of
`mie`; `mip.MTIP` is `rtl/timer.v`'s line and read-only; `mip.MSIP`/`mip.MEIP` stay read-only zero,
which the spec allows in any position of `mip` whose interrupt can never become pending, and names
outright for `mip.MSIP` on a single-hart system. `mtime`/`mtimecmp` are four words at
`0x0002_0000`, in `rtl/littlesoc.v` and `test/testbench.v` alike — and **the map reserves eight**,
which is what `rtl/timer.v`'s `NHARTS` widens to for one `mtimecmp` and one `mtip` per hart against
the one `mtime` (ADR-0124). The four reserved words read zero here and a peripheral put in them
would work on this machine and have to move on the day a second hart landed, with nothing to say so,
so `test/memmap_test.sh` reads every `BASE` under `rtl/` and refuses one inside the span. That
layout is **deliberately not a CLINT's** and firmware written against it does not port to one.
**It is taken on a cycle that
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
interrupt that way on purpose before doing it correctly. **A change in the comparison may reach
`mtip` late and never early**, and `test/timer_tb.v` is the only grader of that: it counts the ticks
to a crossing and holds `mtip` to the level on every quiet cycle. No `.S` program sees an interrupt
arriving a tick early, which is measured rather than assumed (ADR-0118).

**Conformance is not negotiable against minimality.** Every CSR the privileged spec mandates for
RV32 M-mode is implemented, the 87 hardware performance monitor addresses included — most legally
read zero, so the cost is near nothing, and the whole performance monitor is four address compares
with no state behind them — 53 SoC LUTs, `fit` and the period both nulls (ADR-0103). **What a
read-only-zero CSR costs is its address decode, not its write mux**, so ADR-0096's one-LUT figure
for every WARL mask in the file is not the prior for adding one. The CSR set is a floor, not a
closed list; "exact" once made a conformance gap look like a design choice (ADR-0048). **A register
the spec merely recommends is still owed a decision** — the monitor's 29 counters are a "should",
and that gap survived an audit because the question was closed against a reference model that
trapped on them rather than against the spec sentence. **Sail stops being an independent oracle
exactly where this repo configured it**: it trapped because `test/sail/rv32imac_zicsr.json` said this
core has no such counters, so it was agreeing with us and not with the specification.

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
  lands** (ADR-0046), and `make -C formal remeasure-fg` is that sweep, both flip points in both
  directions. F is 6 and G is 6 — the atomic write cycle took G back up the cycle the load
  turnaround gave back (ADR-0099, ADR-0106), and the thinnest depth clears its derived floor by one
  again. **The arithmetic is machine-checked now, not commented** (ADR-0107): `checks.cfg` declares
  F and G in lines genchecks' parser drops, every check's depth is graded against its family's floor
  by `formal/genchecks-audit.py`, and a depth below its floor fails generation. It has to be
  checked rather than read, because a shallow depth does not go red — it goes green having stopped
  asking.
- **riscv-formal ships no spec model for SYSTEM, MISC-MEM or AMO at the pinned SHA**, so trap, CSR
  and atomic behaviour
  is checked against assertions this repo wrote (`test/asm/trap.S`, `test/csr_tb.v`, the decoder
  and `traps` proofs), not an oracle. `formal/COMPLETE_EXCLUSIONS` mechanises that boundary: a pin
  bump that adds a spec model goes red until the exclusion comes out. The generated instruction
  check also drops every value comparison once an instruction traps, keeping only the trap flag,
  and its two pc checks accept whatever target the core reports — so `components_traps` is the only
  thing that says a trap lands on `mtvec` and saves the right state.
  **Its model of a load or store access fault states the whole of it now**: an access the map does
  not answer must trap and must report cause 5 or 7, where it used to require only the cause,
  because reading zero was what this platform did and the spec only recommends otherwise. That is
  why the map has to reach `formal/traps.sv`, which no port of the core carries it to, so it
  restates it and `test/memmap_test.sh` compares the copy. An arm no core reaches is worth nothing
  until it has been shown to fail: `make -C formal traps-region-probe` builds two cores one line of
  `rtl/decoder.v` apart, requires each to go red **at its own arm's line**, and is a prerequisite of
  the proof for the reason `pcloop_cover` is one.
  **The generated checks grade the refusal itself, which the atomic half was never graded by**:
  `checks/rvfi_insn_check.sv` has a memory-fault contract and the eleven A encodings have no spec
  model to reach it, so it had never once been evaluated here. The twelve plain load and store
  encodings do reach it, so decode publishes the refused access's own word address and its exact
  write strobe on the fault channel and the check compares both against the spec model's effective
  address and mask. A superset is legal for the read mask and not for the write mask.
- **It ships no model of an INTERRUPT either**, so the core's timer input is tied off in all five
  harnesses under `formal/` and the generated checks run with no interrupt in the trace.
  `formal/INTERRUPT_TIE_OFF` mechanises that the same way, in both directions and re-derived from
  the clone. F and G were re-measured under the tie-off and both flip points reproduce exactly, so
  the depths are unaffected. `components_traps` is the oracle for entry — `test/asm/mtimer.S` and
  `test/asm/mtimermask.S` for the whole path. `rvfi_intr` is now driven and is **not** optional:
  both sim legs' monitor checks pc continuity across retires and stops only for a retire carrying
  it, so an undriven `rvfi_intr` makes every interrupt a monitor error.
- **It describes ONE hart**, so the core's shared-bus inputs — the grant wait and the write snoop —
  are tied off in the same five harnesses and `formal/MULTIHART_TIE_OFF` mechanises that in both
  directions, re-deriving the port half from `rtl/littlecpu.v` through
  `test/port_connect_test.py`'s parser rather than a second one. It also sweeps for the direction
  that rots: an input every harness holds at a constant and no baseline declares is red, because
  that is a restriction on every generated check arriving unrecorded. A free `bus_wait` would let
  the environment withhold the grant forever, which is what `hang` and `liveness_ch0` measure, and
  the depths are derived on the tied-off machine — F and G re-measured under it reproduce exactly.
  `mem_lock` is deliberately not in that baseline: an unread output cannot weaken a check, and
  `components_accessor` is where it is asserted to cover exactly the cycle an AMO writes back.
- **Both sim legs read the sanitized `test/monitor.sim.v` as their per-retire oracle**, so
  `test/sanitize_monitor.py` is a change to the oracle, not to plumbing. **The spec model has no
  memory map, so a refused access disagrees with it**, and the retire is shown to the monitor all
  the same: dropping it leaves a hole in `rvfi_order` that the reorder buffer reads as a lost
  instruction, and every retire after it is then graded against the wrong shadow. What is dropped is
  the model's opinion about that one retire — `rvfi_mem_fault` gates error 101 and the value
  comparisons behind it — and the gate is not free, because a retire carrying that flag must also
  report a trap or the monitor says so itself. `test/monitor.v` is
  generated but tracked: regenerate it (`make monitor-check`), never hand-edit it.
- **iverilog derives a continuous assign's sensitivity from the call's arguments.** A function
  called from a continuous assign whose body reads module state silently under-evaluates — no
  diagnostic — and once left this leg dead for a whole milestone while every grader stayed green.
  Write such logic out. Treat a green iverilog run as evidence only if it could have failed.
- **A HARNESS CANNOT REACH INSIDE AN INSTANCE, and both ways of trying are silent.** yosys resolves
  no hierarchical reference — `decoder.ls_answer_valid` parses as an *implicitly declared undriven
  wire* whose value the solver picks, on a Warning nothing grades — and it parses `bind` and drops
  it, on no diagnostic at all. An invariant written over such a wire has the giveaway shape of
  passing the property it was meant to strengthen and then failing the BASE CASE, because the solver
  picks the wire adversarially in each direction; five spellings were written and read as five
  different modelling mistakes before the wire was the answer. So an invariant about a submodule's
  own state is **asserted in that submodule**, and the composed task reads it with `-formal
  -noassume`: assertions in, standalone environment models out. `formal/components.sby`'s `traps`
  task is the one that does it, and its split is by that flag rather than by `-formal` on the whole
  file — `pcloop`'s is still the coarse one, and the two comments say which and why.
- **cxxrtl is two-state, so this leg is the only one that can see an X.** Decode reads register
  numbers out of the word *after* the instruction it is issuing, so an undefined ROM word reaches
  the register file's address port and turns the whole pipeline X — green under cxxrtl, green under
  every formal check because they drive that word as a free two-state input, red only here.
  `test/testbench.v` zeroes both ROM banks before poking its program for that reason, the way
  `soc/compare/rom_flat.py` zero-pads its image to full depth. A simulated memory defined only where
  a program was written is not a model of a block RAM, whose every word comes out of the bitstream.
- **Sail co-simulation is a required check on `main`, in a job of its own** (ADR-0032 as amended by
  ADR-0095). `test/cosim.cc` reads the core's real `regs_a` and no `rvfi_*` signal — the property
  that lets it catch architectural writes the self-reporting oracles structurally miss (measured: an
  extra `regs[31] <=` write enabled only past the BMC bound, invisible to every riscv-formal check
  and the whole `.S` suite, reported by co-sim in 0.6s). Do not "align" it against `rvfi_valid`.
  Nothing on `make test`'s path reaches it, so a machine without Sail still runs the whole suite and
  a divergence reads as co-sim. What it **cannot** cover is the interrupt path: `test/asm/mtimer.S`
  and `test/asm/mtimermask.S` are baselined `INCONCLUSIVE SAIL-LIMIT` in
  `test/COSIM_EXPECTED_FAIL` because the reference model has no machine timer, so a green job says
  nothing about the one interrupt this core takes.

## Reference models

**Signed arithmetic in a reference model must be a self-determined statement of its own, never an
arm of a conditional expression.** IEEE 1800 sign-context rules silently evaluate the whole
expression unsigned, for negative operands only; this has produced wrong oracles here twice (the
generated monitor's DIV/REM models, `exec_tb`'s SRA reference). Every reference pins itself against
hand-computed literals before any RTL vector runs and stops with `ORACLE BROKEN` if degraded — an
oracle that is wrong is worse than none, because it fails correct hardware and teaches the reader
to distrust the bench.

## Measurements and ratchets

Three instruments, three designs — never merge their numbers. `make fit` is the core alone (its top
never places: 231 `SB_IO` against sg48's 39, expected); `make soc-timing` is the SoC, which places
and times; `make ecp5-timing` is that same SoC on the other part.

- **`make fit` has a churn band of about ±50 cells**: functionally identical edits move the count
  that much from ABC/nextpnr re-mapping alone. A delta inside the band is not evidence of
  anything, and a ratchet (`FIT_MAX_LC`) must sit outside it. **±50 is the nominal figure and the
  band measures wider than it**: setting one further bit of the read-only `misa` constant spans 68
  cells on `64759da` (3988, 3979 and 3920 against a base of 3935) and 63 on `2007d9d` (3958, 3971,
  3987, 3925 and 3980 against 3988), all of them edits that change no logic at all — which is why
  `FIT_MAX_LC` is derived from a span measured on the tree rather than from ±50. Budget the whole
  span rather than its upward half: every probe on the second tree came out *below* its base, so a
  count can sit anywhere in that window including the bottom.
  **The number is also toolchain-dependent, by as much as the band** — the `fit` job reads **3934**
  on `2007d9d` where both a local Homebrew yosys and a cached OSS CAD Suite read **3988**, 54 cells
  apart on one tree; other trees read 32 apart (3543 job, 3575 local on `d3a9556`) and 3 apart
  twice, with the sign not the same either time (3938 against 3935 on `64759da`, 3966 against 3969
  on `421947f`). The sharp form: the one-bit `misa` edit
  between `64759da` and `2007d9d` moved the local count **+53** and the job's **−4**, so the gap is the
  same re-mapping and has no fixed size or sign. Quote the `fit` job's number, budget for the gap's
  size rather than its direction, and treat a local run as a sanity check. The other instrument
  answers this with tooling rather than a convention now, and it is the same argument:
  `soc/baseline_sweep.sh` stamps a sweep with its base commit and its resolved tool versions, and
  `soc/baseline_summary.py` **refuses** to subtract two sweeps whose stamps disagree.
  **The suite is not pinned** — CI installs the latest release — so
  `FIT_MAX_LC` and `SOC_MIN_MHZ` are graded against a toolchain that can move under them, and a
  suite bump belongs on the list of causes when either trips. **And it is top-dependent**: one
  decode edit measured −50 `SB_LUT4` synthesising `littlecpu` and −1 synthesising `littlesoc`, both
  counts deterministic and neither with a placement in it, because ABC maps the same cone
  differently for what surrounds it (ADR-0094). When the two tops disagree at the band, the answer
  is churn. **It is spelling-dependent too**, which is the sharper form of the
  same warning: those three edits rebuilt from their own description read SoC −45 rather than −1, so
  44 LUTs separate two texts stating one fact (ADR-0097). Quote a group's number with the tree and
  the text it was measured on, and do not read either top's count as the value of the idea.
  **A parameter tied off to today's value is not free of the mapper either**, which is the sharp
  form for a digest gate that forgives nothing: at one hart, folding the shipping fetch window into
  the loop that serves the others is +30 SoC cells, naming `|mem_wstrb && text_range` once instead
  of writing it twice is +64, the general form of the timer's read mux is +19 to +25, an in-process
  `for` loop over the harts is 36 in either direction — and **an inert generate loop, zero
  iterations and nothing elaborated, is −18**. Only a widened port and an *untaken* `generate` arm
  measured zero, which is why `rtl/imemory.v` and `rtl/timer.v` spell the first window and the first
  hart on their own (ADR-0124). **Bit-identity is not reachable for a width-one vector port at all**:
  the canonical JSON records the declaration, so a scalar that becomes `[0:0]` leaves either an
  attribute or a net name behind under every spelling tried.
- **A generated cell's module prefix is ancestry, not ownership.** After flatten yosys names a new
  cell after a neighbouring net, so neither `icetime`'s net names along a path nor a per-module cell
  count off the placed netlist attributes anything: a ROM-slicing experiment moved the `imem.`
  bucket by −193 while `SB_LUT4` total moved +17, and `executor` "grew" 181 cells on RTL nobody
  touched. Only totals are comparable across builds. **A `-noflatten` per-module count is not a
  harvest estimate either**: it sums to 6158 `SB_LUT4` against the flattened design's 4315, unevenly
  — `regfile`'s 2109 is four block RAMs that only get inferred flat, and `timer`'s 322 understates a
  real cost of 417 packed cells (ADR-0112). A census is not a ceiling; take the ceiling.
- **`make soc-timing` has a ~3.6% edit-churn band and a 4–9% placement spread**, and
  **`soc/bands.py` is the one place either number is stated** — keyed by part, printed by every
  sweep script, and restated by none of them. `test/band_source_test.py` grades that both ways: a
  percentage written beside "churn" or "spread" anywhere else in the tree is red, and so is this
  file quoting a figure that file no longer states. The copies it replaced sat in six files and were
  four times too narrow for as long as it took one sixteen-seed sweep to say so, with nothing able to
  go red for it. `docs/adr/` is exempt on purpose — an ADR is a measurement with a date on it and
  must **not** move when a later sweep moves the band. It is best-to-worst over an **unchanged**
  netlist — five sweeps of eight to sixteen
  placements read 4.4, 5.5, 6.9, 8.0 and 9.2% — and up to 11% on a netlist whose cost is a variance
  (ADR-0121 supersedes ADR-0057's 1–2%, which was four placements: a short sweep does not sample a
  tighter distribution, it takes a shorter look at the same one). **A go/no-go is twelve to sixteen
  seeds, paired by seed, quoting worst, median and spread** — never worst-of-four, never worst-of-N
  against worst-of-N, which throws the pairing away and has almost no power against a spread this
  wide. Four seeds is what `soc/timing_sweep.sh` runs by default and it is a look, not a verdict.
  **Margin is a sample too**: the shipping design's worst of sixteen read 12.39 MHz on `d737240`,
  12.21 on `16c5cad` and **12.03** on `c51ebba` — three samples and not a trend, every one
  meeting the requirement and none of them a regression — so quote a margin with the sweep it came
  from, the way a price is quoted with its tree. A delta of a couple of percent is not evidence of
  anything. **It is spelling-dependent the way `fit` is**, and by more: two texts of one design, two
  cells apart on `fit`, swept 12.34 and 12.75 MHz at their worst seeds — 3.3% — with one of them a
  hair faster than the base and the other 1.7% slower (ADR-0106). Quote the distribution of the text that ships. **A candidate whose cost is a variance
  needs sixteen seeds, not eight**: the learned pair table's median cost is +1.2% and a null, its
  best placement beats the base's best, and what it really does is take the best-to-worst spread from
  5.5% to 11.2% — so eight seeds passed it at 12.16 MHz and sixteen declined it at 11.93 (ADR-0113).
  A short sweep cannot see the tail, and the tail is what `SOC_MIN_MHZ` grades. **A median inside
  the band is a null that does not even reproduce**: one edit read −2.67% of median on one tree and
  +0.34% on the next, at −54 cells and −37 (ADR-0121).
- **A tied-off PORT is not a tied-off change, and the digest is where the two come apart.** An input
  the integrator holds at a constant folds before mapping and leaves the cell census untouched; an
  **output the integrator does not read does not fold** — it is still a net for ABC to map around,
  and a one-line probe adding any unread output to `rtl/littlecpu.v` moves the SoC **+44 `SB_LUT4`**
  on its own. So a change that only adds ports can still owe the sixteen seeds, and the seeds are
  the answer rather than the alarm: the multi-hart surface digests different at +48 packed cells and
  sweeps −0.2% at the worst placement, +1.5% at the median, inside the churn band with all sixteen
  over 12 MHz. Read a digest difference for the class of edit it is, and do not read `fit` for this
  one — the same change is **−35 cells** on the core's own top.
- **`make ecp5-timing` is the third instrument and a different CLASS of one.** Same `littlesoc`,
  same sources, same ROM image, placed on the ECP5 the multi-core milestone targets — no fork, no
  `ifdef`, no part-specific RTL. There is no `icetime` for ECP5, so **nextpnr's own timing engine
  both drives the placement and grades it**, with nothing behind it; `soc.asc`'s recipe already
  treats that as disqualifying on ice40, so `soc/ecp5_report.py` is the single reader and refuses a
  report that does not name the design's clock, one placed against a constraint other than the
  declared one, one whose configuration names a corner nobody asked for, and one whose path does not
  reconcile with the frequency published from it. **The frequency handed to the placer is a pinned
  constant far above what the design reaches**, because nextpnr stops working a path once the
  constraint is met — a 25 MHz target reported as 25 MHz would be measuring the target. What it
  **gates** is the mapping, on three exact censuses — `DP16KD`, the register file's
  `TRELLIS_DPR16X4` and `MULT18X18D` — because each of those falling back to soft logic is silent in
  a frequency number and enormous in area. What it **publishes** is the frequency, with no ratchet:
  one placement is a sample, and the ECP5 churn band is derived from ECP5's own sweep rather than
  copied from up5k's. The corner is the board's: Colorlight i5, LFE5U-25F-6BG381C, speed grade 6,
  which is also the pessimistic one of the three. **Pinning `clk` to the module's real oscillator pin
  is not cosmetic** — letting nextpnr choose that pad instead read 3.5% faster, because the pad
  decides where the global network is entered.
- **The DUAL configuration is a FOURTH design on the third instrument, and ECP5 only.** Two fetch
  windows are two copies of the banked ROM — 32 block RAMs against the up5k's 30 — so no up5k number
  describes it and none may be subtracted from one of its. `make dual-ecp5-timing` publishes a
  frequency with no ratchet (35.36 MHz at one placement) and **gates three mapping censuses that
  double where the design does and not where it does not**: `DP16KD` 40 — 32 for the one data RAM
  plus 4 for *each* ROM copy — with `TRELLIS_DPR16X4` and `MULT18X18D` at exactly twice the
  single-hart counts. All three were declared before the first run and matched, which is what says
  two windows are two copies of one storage and two harts are two whole cores (ADR-0125).
  **`make dual-smoke` is its own job and is off `make test`'s path**: two monitor instances roughly
  double a 7000-line generated module, and `test/cxxrtl.cc` is a merge gate that must not grow a
  configuration axis or a flag. It runs one program twice and grades both directions — 32 with two
  harts, 16 with hart 1 held in reset — because **a dual harness that measures one hart looks exactly
  like a working one** unless the answer depends on the second hart having run. `test/monitor.sim.v`
  is read UNMODIFIED by both instances and `test/sanitize_monitor.py` is untouched, which was a
  prediction and is now measured: 114 of 115 and 139 of 140 retires spec-checked.
- **A signal a single-initiator design never had to drive to zero is not a signal two initiators
  may OR.** Three of the shared bus's four ports join with an OR the way `mem_rdata`'s three
  sources do; `mem_wdata` does not, because `rtl/accessor.v` publishes rs2 on it for **every
  issuing instruction** and not only for a store — with one initiator `mem_wstrb` is the only gate
  that matters. ORed across two harts it lost **30 of a smoke program's 32 counted increments**, and it is
  invisible to a bus-exclusivity check because the hart doing the damage has neither a read enable
  nor a strobe raised. Read the producer's idle behaviour before joining two of anything (ADR-0125).
- **12 MHz is a requirement, not a regression floor** (ADR-0066). `SOC_MIN_MHZ` is 12.0 — the board
  clock, whose next divider step down is 6 — and it does not slide. When it trips, fix the design,
  not the floor. The margin over the worst placement is deliberately tighter than the churn band.
- **The divider step above today's 12 is 24.** `SB_HFOSC` divides 48 MHz by 1, 2, 4 or 8 —
  48 / 24 / 12 / 6. An `SB_PLL40` fed by the 12 MHz crystal can synthesize an intermediate clock
  if a change ever earns one. The target for Fmax work is 24 MHz — 41.67 ns, about half of today's
  period — and 12 is already met. A few-percent idea can be read against 41.67 ns and declined in a
  minute, instead of after four placements (ADR-0078).
- **`make dhrystone` and `make coremark` are the figures comparable to another project's** — Dhrystone
  to VexRiscv, whose comparison harness is `soc/compare/`, and CoreMark to Hazard3 and most cores
  published since, which quote CoreMark and not Dhrystone. Neither merges with the other's number;
  see ADR-0098 for why a product needs both factors from one tree, and read the rest of this bullet
  before quoting either.
  `make dhrystone` is quoted in
  DMIPS/MHz because that is what the field publishes. **0.664** at `-O2`, 3568 bytes of the SoC's
  8 KB ROM (ADR-0084, ADR-0093, ADR-0099, ADR-0117, ADR-0129). Dhrystone is string-dominated and the
  optimiser can delete part of the work, so **the flags, the compiler and the string library travel
  with the number** — the program prints all three and will not compile without them. It is not a
  gate and adds no ratchet. **Quote the absolute figure with it**: **7.97 DMIPS** at the board's
  12 MHz, because Fmax above the requirement is margin and not speed, so a CPI win converts to
  throughput and a placement that closes higher does not (ADR-0089). **It went 9.08 → 7.97 on
  purpose**: the region wait spends 13.79% of Dhrystone's cycles to make an out-of-region access
  fault, which is the one trade the board clock could not pay for (ADR-0129). A CPI regression with
  no conformance behind it is still a regression.
- **`make coremark` is SIMULATED AT 16 KB OF ROM, a configuration the part does not have.** CoreMark
  is several times Dhrystone's size and does not fit rtl/imemory.v's shipping 8 KB, so it links against
  test/testbench.v's double-sized ROM_WORDS instead — a figure this labels every place it is printed,
  because a number that forgets its own memory configuration is not one EEMBC's own run rules would
  let stand. `test/bench/coremark/` vendors the five algorithm files unmodified (Apache-2.0, checked
  against `test/bench/coremark/PINNED.sha256` on every run) and `test/bench/core_portme.h` /
  `test/bench/coremark_port.c` are this repo's own porting layer, the same split
  `test/bench/dhry_port.c` uses for Dhrystone. Not a gate and adds no ratchet, and not on CI, the same
  as `make dhrystone`. See ADR-0136 for the number, the Hazard3 comparison and its configuration
  caveat — Hazard3's published 4.15 CoreMark/MHz is its RP2350 build (Zba/Zbb/Zbs, a fast multiplier,
  a branch predictor); its iCE40/iCEBreaker build has none of those, and quoting the RP2350 figure
  against an ice40 core is the mixed-configuration error ADR-0098 already has a name for.
- **The only cross-core comparison that means anything is one harness**, and `soc/compare/` is it:
  same part, memories, program, toolchain and seeds, this core against two others — the VexRiscv
  Verilog in the pinned riscv-formal clone, and Hazard3's iCE40 build, pinned separately by
  `soc/compare/hazard3_pin.mk` (ADR-0139). What follows through the DMIPS product below is the
  VexRiscv comparison specifically — **both factors of throughput are measured there, neither is
  quoted from a README, and a product is only a measurement when both of its factors were taken on
  one tree** —
  the cycle half is the one that moves, and it left the last product stale inside a day. Re-taken
  together on `be293ff`, five placements a side: the gap on **clock** is **1.48×, not the 3× two
  separately-published numbers suggested** (32.61 MHz here against 48.19 there, worst of five, and
  1.47× median on median), both critical paths are
  the fetch loop; the gap on **cycles** goes the other way at **0.784 DMIPS/MHz here against 0.557
  there**, 29.0% fewer cycles for the same work; and the **product is 1.05× — 25.56 DMIPS against
  26.84**, 1.04× if both medians are read instead (ADR-0098 as amended, ADR-0086, ADR-0099).
  **That product is STALE and the cycle half is why**: ADR-0129 spent 13.79% of Dhrystone's cycles on the region
wait, so this core's 0.784 no longer describes the tree that number was taken on. Re-take BOTH
halves together before quoting it again — a product whose factors come from two trees is not a
measurement, which is the rule this very paragraph exists to enforce. **Their
  published 0.52 reproduces**, 7.1% low, where the same project's published 92 MHz does not: 52.11
  at its best placement here, 48.19 at its worst. Neither side is a shipped design — theirs is
  RV32IC with a branch predictor and no privileged architecture, ours has no timer in the harness
  and 4 KB of ROM — so quote it with what it is. Dhrystone needs 26 of that part's 32 block RAMs
  before either core's own 4 and 18, so those cycles are **simulated at a larger map than the clock
  was placed at**; that and eight more distortions are listed in ADR-0098, and
  `make compare-dhrystone` prints the block arithmetic every run for all three cores now, littlecpu
  and Hazard3 both fitting the same 32-block budget VexRiscv's branch predictor does not (below).
  **A harness whose outputs do not
  depend on the datapath measures nothing**: an all-NOP ROM placed 449 cells of a 1711-cell core and
  reported a critical path, so `soc/compare/placed_vs_synth.py` grades the placed count against the
  core's own synthesis, and `make compare-smoke` requires all three cores to publish the same
  values — **this is not a formality**: the first version of Hazard3's own bus adapter fed a
  write's data to memory on the same cycle as its address, which AHB5 does not guarantee, so
  Hazard3 published three all-X words where the other two published real ones. Without that
  three-way check the harness would have measured a core whose every store was corrupt and
  reported a perfectly plausible clock number beside it (ADR-0139). The Dhrystone run compares every
  core's whole data RAM against littlecpu's, word for word. **That stimulus is a
  live control on one side only**: a NOP image makes every ROM word identical, which collapses
  VexRiscv's read-only array and most of its core with it — still red at 0.64×, weaker than the
  0.26× that founded the gate — while this harness's instruction memory is written by the design, so
  its contents are never a constant and the datapath survives whatever it holds, at 1.10×. **What
  folds this side makes the datapath dead rather than uniform**: the same NOP image with the text
  write port disconnected is 0.02×, a core handed a constant instruction instead of the memory's
  answer is 0.02×, and a core input left unconnected — which is how the gate last went red for real,
  on a harness an `rtl/` change did not update — is 0.09×, with icetime reporting 50.77 MHz for one
  sixth of the core.
  **That harness gives VexRiscv no data path to its ROM** — a load from a ROM address reads zero
  there — so anything run in it keeps its read-only data out of ROM until that is fixed.
- **Hazard3's iCE40 build is the third core in the harness. Its Dhrystone cycle factor is measured;
  its CoreMark one and the twelve-seed clock re-sweep both of those DMIPS/MHz figures still want are
  not.** `CSR_COUNTER=0` means it has no `mcycle` and cannot self-time a CoreMark run the way
  `mcycle`/`minstret` let this core do — that half is still not built (ADR-0139) — but
  `soc/compare/dhry_tb.v`'s marker mechanism, built for VexRiscv's identical CSR-free gap, needed no
  new mechanism to close Hazard3's Dhrystone half, only wiring: **0.712 DMIPS/MHz for Hazard3's iCE40
  build in the three-way RV32I row, against 0.758 for littlecpu and 0.590 for VexRiscv** — the only
  ISA all three cores share, so every pairwise ratio the ISA permits comes out of one image and one
  simulation (ADR-0098's second amendment). **Hazard3's own row carries a disclosed bias the other
  two do not pay**: its AHB5 adapter's mandatory write-data wait state is 9.01% of its measured
  cycles, counted directly rather than estimated, and printed beside the row rather than folded into
  it silently. No absolute DMIPS product is quoted for any of the three pairs yet — the clock table
  below is ADR-0139's own five-seed one, taken on a tree before ADR-0129's region-wait fault, and
  reusing it against fresh cycle counts would be the exact stale-product mistake named above for the
  VexRiscv pair. Five placements a side, same part, memories, program and seeds as the
  VexRiscv comparison: littlecpu's worst is 30.90 MHz, median 32.01, best 32.69, placed/standalone
  ratio 1.11×; Hazard3's iCE40 configuration is 32.60 MHz worst, 33.63 median, 33.89 best, ratio
  0.95× — **1.06× at the worst placement, 1.05× at the median**. Read the ratio as area context
  (Hazard3's iCE40 build is roughly half this core's cell count, mostly for lacking a bitmanip
  extension, a branch predictor and this core's parallel multiplier), not as the clock's
  explanation — neither core's critical path was walked for this comparison. Five seeds is a
  look, not a verdict, the same qualifier the rest of this
  file holds every comparison to: `SOC_MIN_MHZ`'s own convention wants twelve to sixteen before
  reading a gap this size as settled, and that sweep has not been run. The pre-fix adapter — see
  the AHB bug above — reported a *larger* gap, 33.08 MHz worst, so the defect had been flattering
  this core rather than the other way round.
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
  priority chain into a parallel mux buys 53 cells and costs 3–9% of period.
  **Dead logic is free only where ABC has a LUT input to fold it into**, which is a fact about the
  consumer and not about the expression: `rtl/writeback.v` masked `wdata` with `wen` that every
  consumer in `rtl/regfile.v` already tests, and the mask survived because the bypass mux it feeds
  had already spent all four of its inputs. Read the consumer before calling a redundant term free.
  **What they cannot use
  is a fact from outside the expression**: a parameter is a power of two, a window is aligned, a
  reversal is wiring, an address bit is provably zero because a trap guarantees it. Eleven such edits
  together are worth −169 placed cells on the SoC and −84 on `fit`; **each one alone is inside the
  ±50 band**, and over eight seeds a side the period moved +0.3% at the median — a null in both
  directions (ADR-0088). It is an
  area lever, not an Fmax one — **occupancy does not set the tail on this part**, ballasted 77% to
  95% for no change at one placement a point and re-measured against a registered prediction at
  sixteen paired placements: 352 cells of logic nothing reads, 88.6% to 95.2%, the design's own
  netlist identical cell for cell, is **+2.03% of median with 13 of 16 seeds slower (p = 0.021) and
  the worst placement 1.1% the other way** (ADR-0121). An effect on the median smaller than the churn
  band and none at all on the tail, so freeing cells buys headroom on the 5280 and whatever path it
  takes with it — never margin. Where a fact like that is now load-bearing it is an
  elaboration `$fatal`, not a comment: `rtl/{imemory,memory,timer,uart,spiflash}.v` refuse to build
  at a non-power-of-two depth or an unaligned `BASE`, and `make window-test` forces all five red.
  **The rule does not only find small things.** The same question asked of `rtl/executor.v` found
  three that each clear the band alone — a divider carrying 64-bit registers for a 32-bit division,
  a duplicated negator inside a one-hot mux, and the multiplier's 33rd partial-product row in soft
  logic — worth −404 on `fit` and −400 placed cells together, with the period a null again and
  `ICESTORM_DSP` unmoved at 4 (ADR-0090). **The question is necessary and not sufficient**, and
  `rtl/decoder.v`'s compressed expansion and immediate generation is the block that showed it: three
  edits that each do state such a fact are worth nothing, and the ceilings say why — the immediate
  mux is 101 LUTs deleted whole, the compressed decode 253, and deleting the fetch window's
  upper-half mask *costs* 13 because ABC had already folded it away (ADR-0094). Six such edits
  stacked are −28 SoC LUTs (ADR-0097), so that block is closed **for area** on two measurements;
  read the ceilings before reopening it for cells. So are `rtl/csrs.v`, `rtl/regfile.v` and the SoC's
  read-back bus, on six more ceilings: the CSR file is 727 LUTs whole and 340 of that is the two
  counters RV32 M-mode mandates, its WARL write mux and every legal-value mask together are worth
  **one**, the register file's write-through bypass is 31 and all its fabric 133, and `mem_rdata`'s
  three-source OR is at the two-LUT floor its six inputs set — an XOR in its place *costs* 19
  (ADR-0096). So are `rtl/accessor.v`, `rtl/timer.v`, `rtl/memory.v`'s gating and the A extension's
  decode rows, on twenty-three more ceilings of which **six are zero or a cost**: only two clear the
  band, the AMO result mux with its 33-bit adder/subtractor at 241 cells and the timer's 64-bit
  magnitude compare at 120, and a carry chain here is free — those 67 carry bits are worth three cells
  between them (ADR-0112). The rest are closed by name, including `rtl/memory.v`'s two range tests at
  **zero** and the decoder's `instr_amo_op` immediate arm at zero on both tops. **The AMO row is spent
  and the accessor is closed for area** (ADR-0119): 241 decomposes into 72 of adder and 162 of mux,
  the three bitwise arms in that mux are 88 cells for 96 bit-operations — one LUT per output bit, the
  floor — and what paid was building eight of the nine functions as one four-entry truth table indexed
  per bit, **−54 cells at all sixteen placements and identical on two texts of the idea**, with the
  median period −2.67%. **Sharing an arithmetic unit is not a saving on this fabric**: a 33-bit adder
  and a 33-bit 2:1 mux place at the same cost, so one adder behind two operand muxes and two adders
  behind a result mux are the same 288 cells — any sharing proposal starts at zero and pays routing.
  **The timer's compare is closed too, and the reason generalises**: its ceiling re-takes at 88
  cells on a later tree, and
  producing `mtip` from registered partial compares instead costs **+138 placed cells** for a period
  that is a null at sixteen seeds — equality every cycle, the magnitude a store still needs and the
  mux that shares it are all LUTs where the compare was a carry chain (ADR-0118). A sticky bit set on
  the crossing is wrong rather than dear: both registers reset to zero, so the level is true with
  nothing crossing it.
  **A conditional
  increment on this fabric is a clock enable, not a mux**: `en ? x + 1 : x`
  is 128 `SB_DFFESR` and no logic, and riding the adder's carry-in instead frees three cells, moves
  those flops to `SB_DFFSR` and misses 12 MHz at six placements out of six. `mtimecmp`'s byte-write
  path is the same shape — 64 clock-enable pins and no mux to collect. Read the `SB_DFFE*` census
  before believing a LUT count, and sweep seeds even for a candidate that is a null on area.
  **Every ceiling above answers "how many cells does deleting this save", and none of them answers
  "what sits on the path"** — so none of these blocks is closed for period. `rtl/decoder.v`'s
  `instr_error` is the demonstration: it read the *muxed* register numbers rather than the raw
  fields, putting the whole compressed-decode cone in the trap arm of the fetch loop for no cells at
  all (ADR-0115).
- **Grade a ceiling on packed `ICESTORM_LC`, not on `SB_LUT4`.** The two disagree in magnitude and in
  sign, on netlists whose packing is seed-independent: `rtl/timer.v`'s read mux is −70 `SB_LUT4` and
  −41 cells, which is the difference between clearing the ±50 band and not, and `mtime`'s byte-write
  path is −17 `SB_LUT4` and **+45 cells** (ADR-0112). A LUT the flops beside it were sharing a cell
  with is not a LUT you can spend.
- **There are two loops around the fetch address, within 2 ns of each other**, which is why work
  moved out of one arrives in the other at full price: `ROM RDATA → instr → rs1 → scoreboard → stall
  → next_pc → ROM address`, and `accessor_out.rd_data → writeback → the regfile's write-through
  bypass → branch comparator → next_pc → ROM address`. Both attacks on that pair are priced and
  declined. Routing `stall` to the ROM's read-enable pin instead of into the address mux is a null in
  both directions — −0.1% of median period, and the critical path ends *at* the enable (ADR-0091);
  replacing the bypass with a fourth scoreboard slot costs **19.5% of suite cycles and 8.6% of
  Dhrystone's** for −2.8% of median period, a product 5.1% worse in DMIPS (ADR-0092), and writing a
  committed result into the idle port a cycle early buys 6.5% of Dhrystone's cycles for **+9.4% of
  median period**, under the requirement at four placements of six (ADR-0100). The pair is no
  better than either half. **What did pay is a cone that had no business being in either loop**: the
  trap decode read the muxed register numbers rather than the raw encoding fields, so every SYSTEM
  form dragged the compressed register-select decode into `trap_pending` and so into `next_pc`.
  Reading the fields is **−2.47% of median period, 13 of 16 seeds faster, the worst placement flat**,
  and eleven cells — about 1.9 ns of median, bought by depth rather than by area (ADR-0117).
  **A ceiling is perishable**: deleting the whole `stall` arm of `next_pc`
  was worth −3.8% three merges ago and is a null on both bases measured since, so re-take one in the
  tree you mean to spend it in. So is a CPI cost — the same fourth slot cost 15.8% of suite cycles
  before the operand-fetch guess landed and 19.5% after, because the guess had been paying for part
  of those stalls.
- **`soc/depth/path_stages.py` charges a level to the module owning the state that level newly folds
  in**, and every LUT in the instruction decoder folds in more instruction bits — which are
  `rom_*_RDATA`. So `decode 11 · imem 5` is not five levels of memory: the `imem` bucket is an upper
  bound on the memory's contribution, and decode's share is understated by the same amount.
  **It attributes a path, not a decision, and the level count orders nothing** (ADR-0116): four
  spellings of one region test span 4:1 in cost with two of them at the same 27 levels and 11% apart
  in period, and the tool charged `decode` one extra level for a change worth 16%. What located that
  cost was substituting one line and re-sweeping — sixteen seeds, paired.
- **The routing on that path is flat, and constraining block-RAM placement is closed** (ADR-0114).
  `soc/routing_bins.py` charges every routing hop of a sweep to what sits at its two ends, reconciled
  against `soc/timing_split.py`'s routing total per placement. Over sixteen placements the block RAMs
  are at one end of 3.6% of routing nanoseconds and the SPRAM 0.0% — **0.0% at the worst placement
  and at the median**, so the tail's critical path never reaches a memory — the contact is the same
  size in the fast half and the slow half, and the largest hop anywhere is the block RAM's own
  1.279 ns clock-to-out. What separates the halves is the number of hops, 72.4 against 77.0. So the
  tail is routing and the routing is **distributed**: there is no long hop, and no column to pin.
  It bins ADJACENCY, and `icetime -r` prints one path, so it cannot see work displaced by where the
  memories sit.
- **Read logic levels apart**: a LUT level costs ~3.3 ns (delay plus interconnect), a carry hop
  ~0.34 ns and no interconnect. A change that trades a carry hop for a LUT level gets shallower by
  icetime's count and slower in nanoseconds. **Measured, in the fetch loop, at any area price**:
  muxing the operand of `next_pc`'s two `fetcher_pc` adders instead of their sums deletes one 32-bit
  adder for −103 SoC LUTs — the largest single decode edit measured here — and takes the critical
  path from 23 LUT levels and 4 carry hops to 25 levels and none, +9.1% of median period, **under
  12 MHz at six placements of six** (ADR-0097). Beside ADR-0088's flattened `next_pc` chain, that is
  two edits in this one chain that buy cells and cost the clock. **Deleting dead logic from it is the
  third**: `rtl/writeback.v`'s `wen` masks are unread by every consumer in `rtl/regfile.v` and cost
  +2.83% of median period and 11.89 MHz at the worst of sixteen seeds to remove, so they stay and say
  so (ADR-0117). The SoC is routing-dominated; wide
  flat muxes route worse than chains. **There is no single lever** — which bounds how much one
  *deletion* buys, and says nothing about what a construct already on the path costs for free.
  Two such were found by reading `soc.timing.rpt` against the RTL after this paragraph had been read
  as closing the question, and **the two did not measure alike**: spelling `instr_error` off the raw
  fields is −2.47% of median with its worst seed flat, and deleting those `wen` masks is the anti-win
  above. Reading the report finds candidates, not wins. The decode head
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
make mutation-check # delete a term from rtl/ and require exactly the detectors
                    # test/MUTATION_DETECTORS pairs with it to go red, both ways.
                    # probe-gates asks whether a comparison can fail at all; this
                    # asks whether a program still sees the property it was
                    # written for. ~3.5 min, not on `make test`, no ratchet.
                    # `make mutation-probe` forces its own graders red and IS on it
make window-test    # force the elaboration checks in rtl/{imemory,memory,timer,
                    # uart,spiflash}.v and rtl/littlecpu.v's copy of that map
                    # red, in both frontends. Runs inside `make test`
make board-elaborate # read soc/board_upduino.v, the one file nothing else on
                    # `make test` or on CI touches -- `bitstream` and `prog`
                    # both need a board. Warning-free elaboration, and two ways
                    # of breaking the wrapper forced red: a port the SoC does
                    # not have, which yosys fails on, and a synchroniser input
                    # reading a pin nothing drives, which it only warns about.
                    # It does NOT read soc/upduino.pcf -- nothing here parses
                    # one, so a pin naming a port that no longer exists is
                    # nextpnr's to catch. Inside `make test`
make imem-share-test # map rtl/imemory.v for ice40 and ECP5 at one and at two
                    # fetch windows, and require two windows to be two copies of
                    # ONE storage -- every copy on the same write enable,
                    # address, data and edge. A claim about the MAPPED netlist:
                    # at RTL there is one array and no simulation can fail on
                    # its absence. Forces its own red directions. Inside `make test`
make cycles         # the suite again, every cycle charged to an issuing cycle or
                    # one of the seven stall reasons; nonzero on a stalled cycle none
                    # of them explains. Prints the two load/store locality
                    # counters under the table -- accesses whose base register is
                    # within 2 KB of a region edge, and accesses issuing on a
                    # write-through to it. Not on CI -- there is no CPI ratchet
make dhrystone      # Dhrystone 2.1 (test/bench, NOT the graded suite) -> DMIPS/MHz,
                    # the ROM image against the SoC's 8 KB, and the same accounting
                    # on compiled code. DHRY_RUNS picks the iteration count.
                    # Not on CI, and it adds no ratchet either
make coremark       # CoreMark (test/bench, NOT the graded suite) -> CoreMark/MHz,
                    # SIMULATED AT 16 KB OF ROM -- double the part's 8, because
                    # CoreMark does not fit the smaller one. COREMARK_ITERATIONS
                    # picks the iteration count. Not on CI, and it adds no ratchet
make waves          # iverilog leg -> waves.vcd; one baked-in program, graded,
                    # not the test/asm suite (test/testbench.v has no image loader)
make monitor-check  # regenerate test/monitor.v at the pin and diff
make fit            # the core's area number; ratchet on FIT_MAX_LC
make soc-timing     # the SoC place-and-time flow; requirement on SOC_MIN_MHZ.
                    # SOC_SEED picks a placement; soc/timing_sweep.sh runs four
make dual-smoke     # the dual configuration under cxxrtl: two harts, one text
                    # storage, one arbiter. Builds one smoke program and runs it
                    # BOTH ways -- both harts, and hart 1 held in reset -- and
                    # grades the shared count moving between them. Off `make
                    # test`'s path: two monitor instances double a 7000-line
                    # generated module. `make dual-elaborate` is iverilog's look
make dual-ecp5-timing # the dual top placed. ECP5 only -- 32 block RAMs against
                    # the up5k's 30. Three mapping censuses GATE, the frequency
                    # PUBLISHES, and no number here merges with an up5k one
make ecp5-timing    # the same SoC on ECP5: synth_ecp5 + nextpnr-ecp5 at a
                    # declared corner. Three exact mapping censuses GATE; the
                    # frequency PUBLISHES, with no ratchet. nextpnr's own
                    # estimator both places and grades -- there is no icetime
                    # here -- and the constraint it is handed is a pinned
                    # constant above what the design reaches. ECP5_SEED picks a
                    # placement. Never merge its numbers with an up5k one
make netlist-digest # the mapped netlist's digest -- the shipping synth script
                    # plus `opt_clean -purge`, with the source attributes
                    # dropped. `make netlist-diff BASE=<ref>` compares two trees
                    # and names what moved. Digest unchanged, NO SWEEP IS OWED;
                    # changed, the paired sixteen-seed sweep is, and it says
                    # nothing about the period. Sound in one direction only.
                    # netlist-determinism is a prerequisite of both, the way
                    # pcloop_cover is of pcloop: it places three bitstreams and
                    # compares bytes. Replaces a sweep, never a gate. Not on CI
make compare-timing # this core, VexRiscv and Hazard3's iCE40 build in ONE hx8k
                    # harness; COMPARE_CORE picks among the three,
                    # soc/compare/sweep.sh's COMPARE_CORES sweeps whichever
                    # subset over seeds (littlecpu and vexriscv by default). A
                    # measurement, not a gate -- but the placed-vs-synthesised
                    # check inside it is graded
make compare-smoke  # all three harnesses run the one image in iverilog and
                    # must publish the same values; says the netlist RUNS
make compare-dhrystone  # the other factor: Dhrystone on all THREE cores, one
                    # RV32I image, one simulation -> DMIPS/MHz each, plus a
                    # fourth row -- this core alone, at its native ISA, same
                    # geometry -- so the shared subset's cost is a number.
                    # Simulated at a larger map than compare-timing places;
                    # COMPARE_DHRY_MHZ adds the absolute column from a
                    # placement. Not a gate, not on CI

make -C formal check                # the generated riscv-formal checks; always a fresh run.
                                    # interrupt-tie-off is a prerequisite
make -C formal check-baseline       # re-grade a finished run without re-running
make -C formal components_decoder   # component proofs by k-induction (mode prove):
make -C formal components_executor  #   read the sby summaries, not the job colour
make -C formal components_pcloop    #   pcloop runs pcloop_cover first, its anti-vacuity
                                    #   control, as a prerequisite of the same target
make -C formal components_traps     #   traps is the only proof over real mtvec/mepc/mcause/mstatus,
                                    #   and runs traps-region-probe first the same way -- the red
                                    #   direction for the load/store region cause arm, two sby runs
make -C formal complete             # depth-50 whole-ISA walk minus COMPLETE_EXCLUSIONS
make -C formal complete_cover       # its anti-vacuity control
make -C formal nonperturbation      # RVFI instrumentation is unread by the core;
                                    # structural, NOT sequential equivalence
make -C formal remeasure-fg         # re-measure F and G by sweeping hang and liveness,
                                    # both flip points in both directions, graded
                                    # against the figures checks.cfg declares. Six sby
                                    # runs, ~20s. Not on CI, no ratchet

make sail-setup     # once: fetch the pinned sail-riscv release
make cosim-run      # co-sim one program (PROG=add.S)
make cosim-suite    # the whole suite, graded against COSIM_EXPECTED_FAIL
make sail-reservation-probe  # asks the MODEL what a trap and an mret do to an
                    # LR reservation. No core runs. Not on CI, no ratchet
```

`make sail-setup` and `make lint-setup` unpack into `~/.cache/little-cpu`
(`XDG_CACHE_HOME` moves it), **outside the checkout**. A git worktree is given tracked files only
and a downloaded tool is gitignored, so an install inside the checkout is invisible from every
worktree — which is why it does not live there. `make test` enforces it.

Toolchain: macOS `brew install riscv64-elf-gcc`; Linux `apt install gcc-riscv64-unknown-elf`.
Tests are freestanding assembly, so no multilib or newlib. Formal needs the YosysHQ OSS CAD Suite,
which CI takes at the latest release rather than a pin; `formal/pin.mk`'s riscv-formal SHA is the
only pinned tool. CI runs on every PR (`.github/workflows/ci.yml`); read the required set live from
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
- **`git config --local` in a worktree writes the checkout's one shared `.git/config`, not a
  copy scoped to that worktree**, and every other engineer's concurrent worktree reads the same
  file. With several engineers working in parallel worktrees, a config change meant to be local
  to one is live in all of them until it is unset. Use a fully isolated clone instead of a
  worktree for anything that needs its own git config.
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
Dhrystone is 3568 of it. The radix-4 divider is still deferred, and so is booting a program out of the
flash — the controller for it is built and reachable in simulation, but not yet wired to this board's
pins (see below). Two things have come off that list:
the machine timer is built (ADR-0082), and the forwarding network is priced and declined rather than
pending (ADR-0083). An interrupt controller, more sources and a vectored `mtvec` are still on it.

**8 KB of text is the ceiling and it is a range test's, not the part's** (ADR-0135). `rtl/imemory.v`
refuses a `ROM_WORDS` that is not a power of two, because both its range tests are reductions on the
address bits above the ROM; the next legal size up is 16 KB, which is 32 block RAMs against the 26
free. So there is no ROM size between the one that ships and one that does not fit, and spending the
spare block RAMs is not a thing that can be done. **The two free `SB_SPRAM256KA` cannot hold text
either, and that is arithmetic rather than a timing question**: fetch reads two neighbouring words
every cycle out of two parity banks, `SB_SPRAM256KA` is 16 bits wide with one address port, so a
32-bit bank is two of them and a fetch window is four — the whole part, with none left for the 64 KB
data RAM. ADR-0087's warning about the fetch loop was never reached; the design it would have
measured cannot be built here. `test/asm/rvc.S` is 12 256 bytes and therefore still cannot run on
silicon, and `soc/run_suite_board.sh` still batches.

**A read-only controller for the configuration flash is the ninth pin** (ADR-0135). `rtl/spiflash.v`
is a byte-at-a-time single-lane mode-0 shift register — eight bytes at `0x0002_0028`, a data register
whose read gives `busy` in bit 8 above the byte the last exchange shifted in, and a write-only control
register whose bit 0 is the chip select. It knows no commands, so a JEDEC id read and an `0x03`
sequential read are the same eight clocks a byte. **`busy` sits beside the byte rather than in the
register software writes, and the co-simulation is why**: the model has plain memory here, so a wait
that polled back the chip select it had just stored spun on Sail and the entry read `INCONCLUSIVE
SAIL-LIMIT`, which compares nothing; polling a register nothing stores a set bit to makes it
`DISAGREE AT 9`, with eight changes compared. **The controller itself is not wired to a board pin —
only the UART's own long-standing pin conflict with the flash chip is.** The UPduino's pin 14 is
`serial_txd` and `spi_miso` at once, so the FPGA's only route to the USB serial port is also the
on-board configuration flash's own data-out line, and driving it unconditionally — as
`soc/board_upduino.v` always had — fights that flash chip for the wire while `iceprog` reads it,
because this board's CRESET is not wired to the programmer and the FPGA keeps running throughout.
`soc/board_upduino.v` now reads pin 16, the flash's chip select, and drives pin 14 with the UART
only once that reads released. That predicate is sound at DC where a grant built on `released` alone
would not be: an SPI peripheral's own output stage is a deterministic function of its chip select —
driving low, high-impedance high — regardless of whether a pull-up or `iceprog` itself is what holds
the select high, so the LEVEL answers "is the flash's own driver off" with no ambiguity about who is
holding it. **The edge is a different question, and `soc/miso_share_enable.v` answers the two
directions differently rather than trusting one predicate for both**: turn-on goes through a
two-flop synchroniser, because a fresh transition needs those two cycles to settle before it is
trusted; turn-off is combinational, on the raw read, because a synchronised-only enable would keep
this design driving pin 14 for up to two clock periods after the flash's own driver is already live
on it — two push-pull drivers on one pin, on every chip-select assertion a host makes. **A predicate
good enough for the level is not good enough to grant the on-chip controller its own pins on.** `soc/pin_lockout.v` was built to arbitrate all four shared pins by the same chip select, on the
same kind of read, but there its question is different — "is a host there at all" — and `released`
cannot answer that: a host idle with its own chip select parked high reads exactly like no host being
there, and `iceprog` parks it that way, as a push-pull MPSSE output, for most of a programming
session and not a corner of it. A grant built on that predicate could still drive three pins into the
FTDI's own driver. So `rtl/spiflash.v`'s own pins — `sck`, `mosi`, and its own `cs_n` — are not wired
to this board at all; `soc/board_upduino.v` ties them off (`spi_miso` high, the rest unconnected), and
`soc/pin_lockout.v` still ships, bounded so a hung on-chip request cannot hold pins forever once
something wires a requester to it (below), graded standalone by its own bench. Reaching the flash's
data path from this board is a future ticket's, and it owes a real-board measurement of the
contention window described above before it wires that sharing in.

**A transmit-only UART is the fifth pin and the only observable output a flashed bitstream has** —
eight bytes at `0x0002_0020` — above the timer's reservation rather than abutting the four words it
decodes, because the four between belong to a second hart — a write-only data register and a `busy`
bit, 8N1 at 115200 baud from a divisor of 104 the header states the 0.16% error of. No receiver, no
queue, no interrupt. Its read-back is one flip-flop, so it joins `mem_rdata`'s OR on bit 0 and costs the other 31 nothing, and
it is outside `make fit`'s top entirely — `littlecpu` does not contain it, so the device's area is a
`soc-timing` number and never a `fit` one. **Nothing here produces a bitstream**: `make soc-timing`
stops at the `.asc`, there is no `icepack` and no `iceprog`, so the pin is measured and not yet
flashed. **Co-simulation cannot cover it** — the model has plain memory at that address, so
`test/asm/uart.S` is `DISAGREE AT 7`, at the first read of `busy` after a write, and
`test/uart_tb.v`, which decodes the line at the configured divisor with five of its own failures
forced, is the only oracle for the wire.

The suite is `test/asm/*.S` **and** `test/asm/*.c`, and `test/OBSERVED_FLOOR` names both. Anything
under `test/bench/` is deliberately outside it: both legs glob `test/asm`, and a benchmark that
needs two million cycles would time out against the runner's 5000 and be graded as a failure. The two
shapes differ only in how `.data` reaches RAM: an assembly program's is poked in by the harness,
which is the thing the hardware cannot do, and a C program's is copied by the startup, which is the
thing it can. A change to one program shape's build is a change in FOUR places —
`test/run_tests.sh`, `test/cosim.py`'s `assemble()`, the Makefile's `soc-rom` and
`test/dual_smoke.sh`, which builds the one program that needs two harts to terminate and is
therefore outside `test/asm` for the same reason `test/bench/` is.

## Pointers

- Decisions: [`docs/adr/`](docs/adr/) — re-derive the count by listing the directory; this file
  has been behind on it three times by quoting a number.
- Briefs: [`docs/ideas/`](docs/ideas/) — the rewrite plan, the fit brief, the four-goals brief, the
  one-address-space brief. Where a brief and an ADR disagree, the ADR wins.
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic).
- Work is tracked in Linear, project **Little CPU** (team JEF) — named so you know where the queue
  is; nothing in this repo depends on it.
