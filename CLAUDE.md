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
here; it keeps its original format and is edited only to correct a command or a fact.

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
  macOS and is what made the board flashable (`docs/flashing-the-upduino.md`); `SB_HFOSC`'s ±10% trim
  was quoted to argue the UART could never work, and the part measures nominal (ADR-0130); two
  community pin tables give the UPduino's clock as 41 and 44 against the vendor's own 20
  (`docs/pin-constraints.md`). The cost of testing one is usually a single command.
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
  logic, no kill signal. `rtl/littlecpu.v` owns `fetch_pc`, since it spans F and X, and
  `rtl/fetcher.v` is stateless: the ROM is addressed off `fetch_pc_next` a cycle ahead, so its
  output register already holds the window at `fetch_pc` and there is no skid to keep coherent
  (ADR-0221, amended). D publishes `predicted_pc`, a static BTFN/`jal` guess: a `jal` or a backward
  conditional branch is guessed taken, at `fetcher_pc + immediate` sized to the ROM window, and
  anything else falls through by `+2`/`+4`. **Fetch follows the guess**, so a correct guess is the
  absence of a redirect. X redirects only on a miss, a trap or `mret`, and the redirect reaches
  `fetch_pc_next` a register later (`x_redirect_q`) so no branch-compare or `jalr` result drives it
  combinationally. A miss still commits nothing: D drops the wrong-path word it holds, X drops the
  word issued beside the redirect (`kill`), and the second wrong-path word never issues — no
  counter, no list, the same rule wrong-path register writes already kept. This keeps the BMC
  depths small and derivable, retire unfiltered, and `pcloop`'s induction free of speculative state
  (ADR-0221, ADR-0208). B3 (ADR-0214) deleted the load/store region wait, one of `x_busy`'s two
  reasons to hold X across multiple cycles, and the skid's deletion shortened the redirect path
  again; `make -C formal remeasure-fg` measures **F = 5 and G = 4**, and
  `formal/checks.cfg`'s `#derive` lines and every `[depth]` floor move with it (ADR-0221,
  ADR-0214). Enforced by `formal/pcloop.sv` (the fetcher/D/X/littlecpu topology) and
  `rtl/decoder.v`'s `FORMAL` block; `test/decoder_tb.v` checks D's own `predicted_pc` guess
  directly, but `fetch_pc` itself lives in `rtl/littlecpu.v`, which has no unit bench of its own,
  so the closed `pcloop` proof is the only check of the whole address chain.
- **All traps are detected by D and committed by X, one cycle later** (2). Nothing faults after X
  settles; a trap is a branch to `mtvec` on the same override the jumps use, which is what makes
  CSR commit precise with no reorder buffer. A refusal counts as committed only when it arrives
  with the *address*, in the cycle X resolves the instruction D already captured — never with the
  response. B3 (ADR-0214) made this uniform across every trapping class, with no exception left:
  the fetch bus (`rtl/imemory.v`'s `imem_fault`, cause 1, detected in D and carried on
  `dx_out.imem_fault`); an atomic, whose effective address is rs1 verbatim so `rtl/executor.v`'s
  range test about it reads a register output with no adder and costs no logic level (causes 5 and
  7, ADR-0109); and now a plain load or store too — the region test reads X's own effective-address
  sum (`mem_addr_calc`) directly and combinationally, with no deferred answer and no held cycle.
  Every same-cycle spelling tried on the fused decoder put that sum in the fetch loop and cost the
  board clock (seven spellings priced, among the retired fetch-loop dead ends under Measurements),
  which is why it answered from `reg_rs1` alone where it could and deferred a cycle at an edge
  instead; moving the test into X
  with the D/X split (ADR-0208) carried the deferral over rather than re-measuring whether X, now
  its own stage with no fetch-loop timing to protect, still needed it. **The layout preference this
  used to create is retired**: ADR-0158's convention (start `.data` one block clear of a
  mapped-region edge) is no longer load-bearing, since every access now answers in one cycle
  regardless of where `.data`/`__stack_top` sit. Dhrystone and CoreMark measure no cycle change from
  this deletion, because their own linker scripts already followed the convention and the wait it
  paid was already down to 2 cycles each — measuring a conflict rather than assuming one shows the
  win is a program NO LONGER NEEDING the convention to reach that floor, not a faster number on the
  two that already had it (ADR-0214 has the figures, `make cycles`' hand-written suite included).
  The linker scripts and `test/probe_gates.sh`'s layout `ASSERT`s are left in place as harmless
  structure, not because anything still reads their placement's cost — retiring them is separate,
  unstarted work. Enforced by `components_traps` over
  `formal/traps.sv` (rebuilt on the D/X topology) and `rtl/executor.v`'s `FORMAL` block;
  `test/executor_tb.v` is the region test's and the trap-cause priority chain's own directed bench,
  driving `rtl/executor.v` the way `test/exec_tb.v` drives its arithmetic. **The one cause with no
  instruction word to detect it from is the timer interrupt**, and it too moved: X now reads the
  live `interrupt_pending` line itself and displaces whatever D has handed it, rather than D
  pre-empting a cycle ahead with a bubble of its own (B3, ADR-0214) — so "detected by D" describes
  every synchronous trap here, and the one asynchronous cause is detected and committed by X alone,
  in the same cycle, with nothing for D to carry.
- **Every inter-stage struct carries a `valid` bit** (3). A bubble is `valid = 0`; retire is
  `valid` reaching writeback, which gates `wen` and drives `rvfi_valid`.
- **Hazards are stall-only except where the executor's own slot already holds the answer**
  (4). B1 split decode into D and X with no forwarding at all: D presents its own
  instruction's pair (never a guess, commitment 6) and a RAW hazard against anything still
  in flight — X's own `out` or its already-registered `executor_out` — simply stalled, with
  no exception (measured 44,620 suite cycles and 1,698,022 Dhrystone cycles, 0.670
  DMIPS/MHz, ADR-0208). B2 (ADR-0222) gives X a forwarding mux: `fwd_rs1`/`fwd_rs2`, D's own
  `dx_match_rs1`/`dx_match_rs2` against `out.rd` gated on a same-cycle-result class flag —
  register NUMBERS and class flags, never a VALUE — ride the D/X register into X and select
  `executor_out.rd_data` over `reg_rs1`/`reg_rs2`, excluding a CSR access's own rs1 (which
  reads `reg_rs1` verbatim via `csr_arg`) and x0 (always zero regardless of `out.rd_data`).
  An `ex_match` producer — one instruction further back — needs no mux at all: by the time
  its consumer reaches X the result has moved on to `writeback`, and the regfile's own
  write-through bypass (commitment 6) already delivers it on the issuing cycle. Only a
  load-use dependency still stalls, two cycles (no ready result to forward, then the result
  not yet unpacked); `hazard`'s split (`test/stall_report.py`'s hzA/hzB) must sum to the hazard column, so a
  hazard cycle that is neither a `dx_match` nor an unready `ex_match` fails the report; a ready
  `ex_match` has no counter because B2 gives it no stall. RVFI's
  `rs1_rdata`/`rs2_rdata` report the forwarded value, not the regfile's own answer, so the
  monitor's `rd_wdata` check is self-consistent only against exactly those two fields.
  Measured on B2's own tree (ADR-0222): against B1, the suite's cycles fall 44,620 → 30,893
  and Dhrystone 1,698,022 → 1,394,022 cycles (0.670 → 0.816 DMIPS/MHz); against main
  (pre-B1), Dhrystone 1,613,644 → 1,394,022 cycles and CoreMark 2.155 → 2.414 CoreMark/MHz.
  Every pre-B1 number and declined-candidate history this commitment used to carry
  described the fused decoder's own confined-forwarding scheme and retired with B1's teardown
  (the pointer under Measurements names those ADRs); B2 is a fresh mechanism against
  the D/X split, not a reopening.
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
  previous cycle** (6, 9). D presents the register file its **own** instruction's pair —
  `read_rs1`/`read_rs2`, held at `out`'s pair while `x_busy` extends the wait
  (`x_busy ? out.rs1 : rs1`) — never a guess at the next instruction's pair: B1 deleted the whole
  guessed-pair mechanism, `rtl/regsel.v`'s second instance for a compressed successor included
  (ADR-0089, ADR-0093 described the deleted scheme; ADR-0208 is the deletion). The synchronous
  answer therefore always arrives exactly one cycle after D captures an instruction, precisely when
  X — which owns `reg_rs1`/`reg_rs2` as its own inputs and is the first stage a register value
  exists in at all — needs it, so B1 has no operand-fetch stall either; there is no guess left to
  be wrong about. `rtl/regfile.v` is otherwise unchanged: a write-first into the read register plus
  a write-through bypass on a **registered copy** of the read address (`held_rs1`/`held_rs2`) still
  lets X observe a writeback landing the same cycle it reads its own operands. The standing
  liveness probe is unchanged too: delete the rs2 write-through bypass and `reg_ch0` must go SAT —
  run it before believing any `reg_ch0` result under a changed configuration.
- **Stalls are one global broadcast over two mechanisms** (8): `x_busy` **holds** `out` unchanged
  (X is still working the instruction D already handed it); every other reason **bubbles** (nothing
  issued). Every in-flight non-`x0` `rd` must be visible to the scoreboard on every cycle between
  issue and the regfile write-through, with no gap. **Six** reasons raise `stall` — the divider is
  collapsed into `x_busy`'s one bit at D's level (moved into X with the D/X split, ADR-0208); the
  operand-fetch reason is gone outright, since B1 deleted the guess it existed to cover (commitment
  6); and the load/store region wait is gone outright too, since B3 (ADR-0214) made the region test
  combinational rather than deferred, so there is nothing left for `x_busy` to fold in for that
  reason. `stall_own = hazard || serialize || fetch_stall || atomic_stall || x_busy`, then
  `stall = stall_own || bus_wait`, is exactly their OR — there is no `stall_other` tier anymore. Two
  related vocabularies are graded against each other, not one: D's own composition (the raw signals
  `stall`'s OR is built from -- `hazard_rs1`/`hazard_rs2`, `serialize`, `fetch_stall`,
  `atomic_stall`, `x_busy`, `bus_wait`) declared in the decoder's signal, its OR, its publish arm
  and its `FORMAL` hold-assert, and vectored both ways (hold, bubble) in `test/decoder_tb.v`'s
  OR-identity check; and the CPI-accounting taxonomy (`divider`, `atomic`, `hazard`, `serialize`,
  `fetch`, `bus`) in `test/cxxrtl.cc`'s bucket, split across `uut decoder` and `uut executor` since
  `divider_busy` lives in X, and `test/stall_report.py`'s `REASONS` and `HEADINGS`. `formal/pcloop.sv`
  no longer carries a separate `f_may_stall` over-approximation: the D/X rebuild (ADR-0208) composes
  the real fetcher/decoder/executor instances, so pcloop's induction reads their actual
  `issuing`/`redirect` outputs directly and has nothing left to declare here. `rtl/executor.v`'s own
  `x_busy` is `divider_busy`'s own condition, restated rather than aliased (an alias collapses to
  the same netlist bit, which would break `test/zkt_isolation_test.py`'s one-hop block); the two
  are graded identical, not OR'd, in `test/executor_tb.v`'s identity check. `test/stall_sites_test.py`
  grades all of it; the cycle-accounting identity itself (`test/stall_report.py`'s `unattributed`
  column) still runs on every `make test`, not only under `make cycles`. The **atomic write cycle**
  still bubbles because X has already consumed the AMO and a hold would retire it twice (ADR-0106);
  the **ungranted bus** still bubbles because X publishes `stalled` and takes no input that freezes
  it, so a held `dx_out` would be consumed twice — it is tied low in every single-hart integrator
  (`formal/MULTIHART_TIE_OFF`), `rtl/littledual.v` alone drives it, and the core does not decide
  its own wait: D publishes `bus_request` and the platform ANDs it against its grant, because a
  grant term inside D would close the loop through the arbiter. The memory transaction is presented
  from `dx_out` during X's own cycle and the request block is gated on the cycle X takes it,
  because a re-presented request is idempotent for RAM and not for a device; `components_accessor`
  and `test/accessor_tb.v`'s transaction count grade that (ADR-0099, unaffected by the split).

Retired numbers, never reused: 7 (the generated-but-tracked monitor) lives under Verification; 9 is
folded into 6.

**Stage B of the fetch refactor split decode into D and X** (ADR-0221, ADR-0208; commitments 1, 2,
4, 6 and 8 above are the rewritten prose, not a separate note). `formal/pcloop.sv` and
`formal/traps.sv` are rebuilt on the real fetcher/D/X/csrs topology (no more hand-wiring the fused
decoder), `rtl/executor.v` has its own `` `ifdef FORMAL `` block, and `components_pcloop`,
`components_decoder` and `components_traps` all close by k-induction against it —
`components_traps` needed several cross-module invariants added to `rtl/decoder.v`'s own `FORMAL`
block (onehot0 over `out`'s class flags, and each fact `formal/traps.sv`'s reference model
re-derives from `dx_instr`'s raw bits independently of D's decode) before the composed proof's
k-induction could generalize past what executor.v's own standalone-only assumes used to give it for
free. `test/exec_tb.v` is rebuilt against `rtl/executor.v`'s real ports (operand values ride
`reg_rs1`/`reg_rs2`, not `in.rs1`/`in.rs2`) and passes with full coverage.
`test/zkt_isolation_test.py` is retargeted at `rtl/executor.v`, whose one timing output (`x_busy`)
is gated by the divider's own state register — Zkt's one remaining named exclusion, DIV/REM, since
B3 (ADR-0214) deleted the load/store region wait outright — rather than decoder.v's nine
now-data-blind stall reasons. `test/decoder_tb.v` is rebuilt against
D's real single-cycle present-then-issue shape (decode, the RAW-only scoreboard, serialization, the
atomic write cycle, the `x_busy` hold/bubble split, `x_redirect`'s unconditional kill,
`bus_request`'s over-asking, and the one-cycle interrupt bubble), and `test/executor_tb.v` is new:
branch and jump resolution, the trap-cause priority chain and `trap_tval`, CSR read/write
suppression, atomic address and fault, the region test (B3 rewrote its vectors again when the
deferred-answer protocol it originally tested was deleted, ADR-0214), and the interrupt bubble's
commit all moved there with the RTL they test. `test/stall_sites_test.py` is
rebuilt for the two vocabularies above. `test/MUTATION_DETECTORS`'s five patches are re-keyed to
where their term now lives; `atomic-region-ignored` and `loadstore-region-ignored` are caught by
`executor_tb` now that `decoder_tb` no longer sees the region test.

## ISA target

RV32IMAC_Zicsr_Zifencei_Zkt, M-mode only, `misa = 0x4000_1105` (none of the three Z-extensions has
a `misa` bit; the `-march` string is the only place they are claimed). Traps implemented:
instruction access fault = 1, illegal instruction = 2, breakpoint = 3, load misaligned = 4, load
access fault = 5, store misaligned = 6, store/AMO access fault = 7, ecall from M = 11.
Instruction-address-misaligned (0) is unreachable — C makes 2-byte targets legal — so not
implementing it costs nothing and closes nothing. C stays because code density is a product
constraint on the up5k (ADR-0002, ADR-0003).

**Every refusal now has one shape: X decodes its own copy of the map and answers combinationally,
with no round trip to the platform.** `rtl/imemory.v` publishes a fetch outside the text window →
cause 1. B3 (ADR-0214) moved the twelve plain load and store encodings' region test into
`rtl/executor.v` itself, which carries its own copy of the map and answers the effective address's
range combinationally, the same cycle, with no asymmetry between a wide window and a narrow one —
the old fast-arm/deferred-arm split, and the layout preference it required, are both gone. The
atomic region test followed the same move (ADR-0221 amendment, 2026-09-28): the eleven A
instructions' effective address is `rs1` verbatim, so `rtl/executor.v` runs the identical
combinational test on it that a load or store already runs on `mem_addr_calc` — cause 5 for `lr.w`
and 7 for the nine AMOs and `sc.w`, alignment outranking the region, which makes everything outside
the data RAM — text, timer, UART, SPI controller — refuse in X before the access ever reaches the
bus. `rtl/memory.v` no longer computes an answer about an atomic's address at all; its own write
gate (`in_range && |mem_wstrb`) is unchanged and is the second, independent guard behind X's
refusal, so a platform whose own decode disagreed with X's still could not let an `sc.w` claim a
write that went nowhere (`test/accessor_tb.v`, `components_accessor`). The declined alternative
both moves retire — answering off `rs1`'s page rather than the effective address — was priced
against the fused decoder's fetch-loop budget, which no longer exists on this tree (ADR-0109).

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
encodings — executes in time independent of its operands' VALUES; `DIV`/`REM` (32 iterations, 16
when the dividend's magnitude has a zero top half, or one when `rs2 == 0` or on `INT_MIN / -1`),
loads, stores, branches and jumps are excluded, and the exclusion is what makes the claim true. Load/store timing here varies with address arithmetic, not
cache state, and the constant-time model treats addresses as non-secret. Three graders carry it:
`test/zkt_isolation_test.py` grades the taint half on the ELABORATED NETLIST (ADR-0137 records why
the source-text version was replaced) — it seeds taint at `rtl/executor.v`'s `reg_rs1`/`reg_rs2`,
follows a flip-flop's D to its Q, and requires that `x_busy` (X's one timing output) is not
reachable except through `state`, the divider's own register — Zkt's one remaining named exclusion,
DIV/REM, blocked as a taint source past its own hop; its header carries the argument. D's own nine
former stall reasons dropped out of this check with the D/X split: none of them reads a bit of
register-file data anymore (ADR-0208), and B3 (ADR-0214) deleted the load/store region wait
outright, so `region_stall` is not a second exclusion to name — it does not exist. `rtl/executor.v`'s
`FORMAL` block asserts `ls_access`'s exact membership as a single-trace equality and
`components_executor` proves it, with `formal/decoder-zkt-probe.py` as the red direction and
prerequisite (retargeted at `rtl/executor.v` with the split, not renamed; its own header records
that the sibling `region_stall` assertion it used to probe alongside `ls_access` went with the
deletion). That same block asserts that the four
multiplies resolve in the `init` state with no counter and `components_executor` proves it, with
`formal/executor-zkt-probe.py` as the red direction and prerequisite; its header says why the
mutation is narrowed to `rs2 != 0` and why only the basecase leg is read.

**One interrupt: the machine timer, cause `0x8000_0007`.** `mie.MTIE` is the only writable bit of
`mie`; `mip.MTIP` is `rtl/timer.v`'s line and read-only; `mip.MSIP`/`mip.MEIP` are read-only zero,
which the spec allows for an interrupt that can never become pending. `mtime`/`mtimecmp` are four
words at `0x0002_0000` and **the map reserves eight**, one `mtimecmp` and one `mtip` per hart
(`NHARTS`, ADR-0124); `test/memmap_test.sh` reads every `BASE` under `rtl/` and refuses one inside
the span. The layout is **deliberately not a CLINT's**. **The take is X's own decision, not D's**
(B3, ADR-0214): X reads the live `interrupt_pending` line every cycle it is not mid-divide, and
displaces whatever real instruction D has handed it rather than D pre-empting the decode a cycle
ahead of time — D never sees `interrupt_pending` at all now, and carries no bubble for it.
`stall` still outranks the trap arm of `next_pc`, so the take waits out only a divide and a
serialization, with no logic of its own, and is **not** a stall reason. Worst-case response:
33 cycles, set by the divider. `mtimecmp` resets to zero, so `mtip` is
asserted out of reset, and both enables resetting to zero makes that harmless (ADR-0082). Three
facts are the platform's to state and firmware cannot derive them: **`mtime` ticks once per clock
cycle**; **MTIP is a level**, posted until `mtimecmp` exceeds `mtime`, so a handler that returns
without moving `mtimecmp` is re-entered before the instruction at `mepc` runs; **an RV32 `mtimecmp`
update is the spec's three stores in the spec's order** — low all-ones, high, low — and
`test/timer_tb.v` and `test/asm/mtimer.S` each fire the spurious interrupt the other order gives on
purpose. A change in the comparison may reach `mtip` late and never early, and `test/timer_tb.v`
is the only grader of that (ADR-0118). **Device stores land one cycle late**: the timer, the UART
and the SPI controller latch a store's strobe, word and data and apply it a cycle after X launches
it, because the forwarded operand, the address adder, the device decode and the register write
otherwise share one cycle. `mtip` compares the registered `mtime` and `mtimecmp`, so it can post
one or two cycles later than the store that caused it; the timer bypasses a pending store into its
read data, and the UART and SPI controller report `busy` from `busy || start_pending`, so a read
is never stale (ADR-0221, ADR-0214).

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
  and G (worst-case retire gap, from `liveness`), 5 and 4 (ADR-0221, ADR-0214), declared in
  `formal/checks.cfg`'s `#derive` lines. **Any change that adds a stall reason, lengthens a stage, or widens the
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
  median inside the band is a null that does not even reproduce** (ADR-0121). **Both figures are
  re-confirmed with provenance, not merely inherited**: a fresh sixteen-seed sweep on an unchanged
  netlist read 6.8% spread, inside the existing range above, and two real edits to `rtl/csrs.v`
  (ADR-0170's own comment diff, replayed, and a matched-line-count blank-line-only diff) read
  worst-case churn of 3.6% and 1.7% respectively, so 3.6% stands as the ceiling rather than a
  number nobody re-took
  (ADR-0194). `soc/paired_sweep.sh` is the one-command runner behind that: a named base ref
  against the working tree, both parts, paired by seed, refusing below twelve seeds a side and
  never bypassing `soc/baseline_summary.py`'s own toolchain-mismatch refusal. `soc/bands.py`'s own
  `derived` field carries the tree and both tool versions.
- **With no `SOC_SEED` override, `make soc-timing` grades ONE pinned placement, not a
  sweep's worst** (ADR-0171, amending ADR-0066: the 12.0 requirement is unchanged, only
  what is measured against it). Three re-rolls of one netlist's RTL semantics — yosys's
  generated cell names carry `file:line`, ABC9 sorts by that name string, and a comment
  or a blank line is enough to reorder what it hands the placer — span about 3.7%
  worst-of-sixteen against a 3.5% clearance, so one draw of sixteen can land under 12.0
  while the design that produced it is unchanged. `soc/pin.json` records a digest of the
  files synthesis READS, a placer seed, the measured MHz and the distribution it was chosen
  from; keying that on the netlist instead does not work, because the OSS CAD Suite floats
  and the same sources map to a different netlist on every release. A source change **warns**
  as PIN STALE and does not fail: the gate is Fmax, and the warning's job is that a
  regression can hide behind a pinned seed that still clears. **Seeds 1..16 are not an independent sample**:
  nextpnr's RNG state update is linear over GF(2), so small-integer seeds span a
  low-dimensional subspace of the state — provable, but whether that correlates
  placements is unmeasured, and the decision does not lean on it either way.
  `make soc-seed-search`, off `make test` and CI like `make fit`, draws high-entropy
  seeds (never 1..N) and refuses to write a pin under a 5% margin over `SOC_MIN_MHZ`, so
  a pin has to survive the same toolchain drift `fit`'s own churn band already accounts
  for. An explicit `SOC_SEED=` — `soc/timing_sweep.sh`'s every row included — bypasses
  the pin entirely; `$(origin SOC_SEED)` is what tells that apart from no override at
  all, since both read as an empty string.
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
  run measures the target. **A fourth check is structural and absolute: no block RAM's reset may be
  driven by logic** (`soc/bram_reset_check.py`, over the mapped JSON, on all three ECP5 flows).
  Yosys maps a synchronous constant arm — `mem_rdata <= in_range ? ram[index] : 32'b0` — onto
  `DP16KD`'s output reset, and on the part that read returns zero whatever the array holds, so a
  program's stores land and read back as nothing (ADR-0163). Nothing else sees it: RTL simulation
  passes, the censuses count the same 36 `DP16KD`, nextpnr places and times it, and **yosys ships
  no behavioural model for `DP16KD`**, so the mapped netlist cannot be simulated on any machine.
  Spell such an arm as a mux on the block's OUTPUT. **ECP5 now has a derived band, and it does not
  transfer to or from up5k's**: one sixteen-seed sweep on an unchanged netlist reads 10.3%
  placement spread, wider than up5k's own and not yet a range a second sweep could narrow or
  widen (ADR-0194). Edit-churn reads 0% under the two rtl/csrs.v diffs that moved up5k's netlist,
  each independently confirmed `DIGEST-DIFFERENT` for up5k by `make netlist-diff` and placed
  BYTE-IDENTICAL on ecp5 at all sixteen seeds — a measured null under those fixtures, not a proof
  this part cannot churn. `soc/bands.py`'s `derived` field carries the tree and both tool
  versions; `soc/bands.py ecp5 --require` now exits zero. Pinning `clk` to the module's oscillator pin is not
  cosmetic: the pad decides where the global network is entered, and `docs/pin-constraints.md`
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
  gate. Dhrystone: **0.943 DMIPS/MHz, 11.32 DMIPS at 12 MHz** at `-O2` under xPack gcc 15.2.0 with this port's own byte-loop string routines and `test/bench/bench.lds`, 1,206,025 cycles at 2,000 runs (ADR-0220; 0.722 on `main` before the fetch refactor, ADR-0190), quoted with the
  absolute figure because Fmax above the requirement is margin and not speed (ADR-0089), and with
  the flags, the compiler, the string library and **the linker script** — the program prints the
  first three and will not compile without them, and `test/bench/bench.lds` asserts the fourth at
  link time. **It went 9.10 → 7.97 → 9.10 → 9.32 → 8.66 → 11.32, and the moves differ in kind**: the region
  wait spends 13.79% of Dhrystone's cycles to make an out-of-region access fault (ADR-0129), insetting
  the layout gives those cycles back with no RTL change and the netlist digest unmoved (ADR-0158),
  executor-only forwarding buys the last step in the datapath (ADR-0154), and the last move is the
  compiler alone: pinning xPack's `riscv-none-elf-gcc` 15.2.0-1 in place of whichever of Homebrew's
  16.2.0 or a CI image's 13.2.0 happened to answer to the two names every build used to search for
  changes what the same C compiles to, with no RTL and no linker script touched (ADR-0190). A CPI
  regression with no conformance behind it is still a regression, and a figure recovered by moving the
  software is the firmware ceasing to pay a cost, never the core getting faster; a figure that moves
  with the compiler is neither, and is why the compiler is pinned now. The last step, 8.66 → 11.32, is the fetch refactor: D/X split, executor forwarding, the static predictor with fetch following the guess, and the deleted region wait (ADR-0208, ADR-0222, ADR-0214, ADR-0221 as amended; the cross-core re-take is ADR-0220). **CoreMark is
  SIMULATED AT 16 KB OF ROM**, double the part's 8, against `test/testbench.v`'s `ROM_WORDS`, and
  every printed figure says so; the five algorithm files are vendored unmodified and pinned by
  `test/bench/coremark/PINNED.sha256` (ADR-0136). **2.776 CoreMark/MHz** under the pinned compiler
  (ADR-0220; 2.155 on `main` before the fetch refactor, ADR-0190; 2.203 under Homebrew's prior 16.2.0, ADR-0154), and it travels with the
  linker script the way the DMIPS figure does: the inset layout read 2.013 against 1.811 on the
  conventional one when ADR-0158 measured it, and executor-only forwarding took the inset figure
  to 2.203 afterwards (ADR-0154). Hazard3's published 4.15 CoreMark/MHz is its RP2350 build, not its iCE40
  one, and quoting it against an ice40 core is the mixed-configuration error ADR-0098 names.
  **`make coremark-rom-up5k` links a CoreMark image the part can actually hold** — a link, not the
  score below: the shipping image is 6,996 of the 8 KB budget under the pinned compiler (was 7,076
  under Homebrew's 16.2.0), `-Os -flto` rather than `COREMARK_CFLAGS`'s `-O2`. **1.776 CoreMark/MHz**
  under the pinned compiler (1.780 under 16.2.0, essentially unmoved — `-Os -flto` code is far less
  compiler-version-sensitive than `-O2`'s, ADR-0190), a 17.6% cost against the native-ISA `-O2`
  figure above, legal under EEMBC's own build-option allowance; `COREMARK_UP5K_ITERATIONS`'s 800
  iterations clears `core_main.c`'s own ">=10 secs" self-check for real at 12 MHz (ADR-0165). No
  board has run it yet — `make coremark-board` is off `make test` and CI, the same standing as
  `make dhrystone-board` before ADR-0130.
- **The only cross-core comparison that means anything is one harness**, `soc/compare/`: same part,
memories, program, toolchain and seeds, against the VexRiscv in the pinned riscv-formal clone and
Hazard3's iCE40 build (`soc/compare/hazard3_pin.mk`, ADR-0139). **Every figure below is one
stamp: the weekly workflow's `make compare-product` dispatched on `main` at 11cc506 on
2026-10-01, under xPack gcc 15.2.0, yosys 0.69+158, nextpnr 0.11.1-40, twelve seeds a part, with
the cycle halves re-run locally the same day and digit-identical** (ADR-0232 holds the table, the
tool stamp and the standing flags; the stamp itself is `soc/compare/product.json`, which a
refresh PR carries; `docs/comparison.md` is rendered from it by `make compare-doc`, and `make test` fails on drift and on a littlecpu cycle factor moving off `soc/compare/CYCLE_FLOOR`, ADR-0244). **A product is a measurement only when both factors were taken on one tree
AND one toolchain**, and **A COMPARISON IS ONLY AS GOOD AS ITS LEAST EXAMINED ASSUMPTION** — this
harness has been wrong about the part (ADR-0086/ADR-0160), the opponent's configuration (ADR-0160
as amended: `FormalSimple` had no `MulPlugin`, no `CsrPlugin` and no hazard forwarding, which
flattered VexRiscv on period and this core on cycles at once), and the shared ISA — each
corrected once found, never all at once.
**It places on exactly the two parts this design ships to, and hx8k is gone** (ADR-0171).
`COMPARE_PART` selects `up5k` (the default) or `ecp5` and anything else is a hard error; there is
no third row to add without measuring one. The two arms answer different questions and are never
averaged. **On up5k the clock is a step function** — the board's crystal, or `SB_HFOSC`'s
48/24/12/6 — so `make compare-timing` grades it PASS/FAIL at 12 MHz through
`soc/compare/step_gate.py`, prints the margin above the step as unspendable, and the comparison is
then CYCLES ALONE; a core under the step is out of the comparison rather than slower in it, because
its next clock is 6. **On ECP5 `EHXPLLL` synthesises `ref × M / N / D` on a fine grid**, so Fmax is
a real factor there and both halves of the product vary; the frequency PUBLISHES with no ratchet,
since `soc/bands.py` has no band for that part and up5k's own was derived on `littlesoc` rather than
on this bench. The ECP5 arm carries the same gates the SoC's own ECP5 flow does — `DP16KD`,
`TRELLIS_DPR16X4` and `MULT18X18D` censuses plus `soc/bram_reset_check.py` — and
`placed_vs_synth.py` reads `TRELLIS_COMB` against `LUT4` there rather than the ice40 pair.
**Hazard3's multiplier maps to soft logic on ECP5 and the other two cores' do not**
(`COMPARE_ECP5_EXPECT_DSP_hazard3` is 0 against 4), which is declared rather than rediscovered.
**RV32IM, not RV32I or RV32IMA, is the widest ISA all three cores share**: Hazard3's iCE40 build has
no C, and the generated VexRiscv has no `AtomicPlugin`, so `COMPARE_DHRY_CFLAGS` and
`COMPARE_COREMARK_CFLAGS` both build at `rv32im`, and CoreMark runs all three cores in one
simulation, `soc/compare/coremark_tb.v` reusing `soc/compare/dhry_monitor.v` for its third DUT
(ADR-0146 as amended). Both benchmarks are `-O2` under the pinned compiler (the flags
`make compare-dhrystone` prints), with `soc/compare/dhry_port.c`'s own byte loops for the string
routines, linked by the harness's own `soc/compare/dhry.lds` / `coremark.lds` (the
placed-geometry budget comes from `soc/compare/bench.lds`) and not by `test/bench/bench.lds`. **Hazard3's own two-port top removes the fetch/data contention rather than working around
it** (`hazard3_cpu_2port`, ADR-0146 as amended a third time): a dedicated fetch port and a dedicated
load/store port, the same topology the other two cores have. A same-cycle attempt to also remove
its adapter wait was tried and reverted on up5k Fmax and is not foreclosed generally.
**Cycles** (one image, one simulation, between each program's two `mcycle` marks; Dhrystone 400
runs, CoreMark one iteration): Dhrystone littlecpu **228,825 (0.995 DMIPS/MHz)**, VexRiscv 262,827
(0.866; takes 1.149× littlecpu's cycles), Hazard3 252,026 (0.903; 1.101×); CoreMark littlecpu
**359,507 (2.782 CoreMark/MHz)**, VexRiscv 426,430 (2.345; 1.186×), Hazard3 666,552 (1.500; 1.854×).
Before the fetch refactor this core read 313,627 and 446,995 against the same two opponents'
cycles, which are unchanged to the digit (ADR-0190, ADR-0220): the refactor moved littlecpu from
behind VexRiscv on both benchmarks and behind Hazard3 on Dhrystone to ahead of every opponent on
both, with the opponents' own unchanged cycle counts as the control. **Hazard3's disclosed adapter wait is still counted**: `wait_cycles=28805` of Dhrystone's
252,026 (11.43%) and `wait_cycles=14176` of CoreMark's 666,552 (2.13%). Bounding Hazard3 at its own
account, 252,026 − 28,805 = 223,221 Dhrystone cycles, reads **0.975× littlecpu's**, so removing the
disclosed wait would put it level with, and not clearly ahead of, this core.
**Clocks, worst / median of twelve paired placements** (the stamp's `seeds` are `default` and 1
through 11; nothing is read at one placement). Up5k: littlecpu 12.55 / 12.78 MHz, VexRiscv 21.93 /
22.69, Hazard3 13.85 / 14.26; **all three reach the 12 MHz step and none reaches 24**, so the up5k
product is the cycle ratio at one shared clock: Dhrystone littlecpu **11.94** DMIPS, VexRiscv 10.39
(0.871× littlecpu), Hazard3 10.84 (0.908×, derived from the cycle row above rather than stamped);
CoreMark littlecpu **33.38**, VexRiscv 28.14 (0.843×), Hazard3 18.00 (0.539×). **On up5k this
core is now ahead of both opponents on both benchmarks; before the refactor it was behind both on
Dhrystone and close to VexRiscv on CoreMark.** ECP5 has no quantisation step, so its own
product uses each core's own clock: littlecpu 37.38 / 38.93 MHz (9.59% spread), VexRiscv 53.16 /
55.34 (9.11%), Hazard3 50.35 / 51.89 (8.35%). Dhrystone at each core's worst placement:
littlecpu **37.19 DMIPS**, VexRiscv **46.05 (1.24× littlecpu)**, Hazard3 45.48 (1.22×, derived,
not stamped: `soc/compare/run_product.sh`'s Dhrystone pairs carry only littlecpu and VexRiscv).
CoreMark: littlecpu **103.98**, VexRiscv **124.67 (1.20×)**, Hazard3 **75.54 (0.73×)**. **On ECP5
the opponents' clocks (VexRiscv 1.42×, Hazard3 1.35× this core's at the worst placement) outweigh
its cycle lead, so the order is the reverse of up5k's**: VexRiscv is ahead of littlecpu on both
benchmarks, Hazard3 on Dhrystone and behind it on CoreMark.
**Every Hazard3 ECP5 clock reading carries a standing flag**: this same RTL, byte-checksummed, read
33.26 MHz in an earlier session and 48.50 in a later one before the two-port adapter, and
nextpnr-ecp5 — the one tool this repo does not pin — is the likely, unconfirmed explanation; this
stamp's 50.35 / 51.89 sits with the later reading and inherits the flag rather than resolving it.
**The toolchain is part of the stamp, not a detail**: the same twelve seeds moved VexRiscv 4.5% at
its worst placement between two yosys builds while this core's up5k SoC came out bit-identical, so
halves synthesised by different toolchains do not form a product — re-take both halves together.
**The two benchmarks do not carry the same map caveat** (ADR-0171): on up5k the placed geometry is
a 4 KB ROM and 64 KB of SPRAM, and Dhrystone's image (1,968 bytes of text and 10,572 of RAM at
RV32IM: 4 of the part's 30 block RAMs and 2 of its 4 SPRAMs, 8 blocks with the core's own 4)
FITS it, so nothing in that row is distorted by memory size; CoreMark's 10,648 bytes of text do not
(22 block RAMs, 26 with the core's own 4, against a 4 KB placed ROM), so its cycles are still
simulated at a larger map than the clock they are multiplied by was placed at. `make compare-dhrystone`
and `make compare-coremark` print that block arithmetic every run, and ADR-0098 lists the
distortions. `soc/compare/product.json` carries a `base` that is a commit on `main`, which is why
the stamp is taken by the weekly `.github/workflows/compare-product-schedule.yml` dispatched on
`main` and not on a PR branch: this repo squash-merges and deletes branches, so no checkout could
resolve a PR-branch commit once merged; the workflow pushes the branch that carries the stamp and
opens an issue linking it, and a person opens the PR (ADR-0233). Two graded checks stand in front of every number:
`soc/compare/placed_vs_synth.py` refuses a placed count under `COMPARE_MIN_RATIO` of the core's own
synthesis — an all-NOP image once placed a quarter of this core with a plausible critical path
beside it (ADR-0086) — and `make compare-smoke` requires all three cores to publish the same
values, which caught Hazard3's first bus adapter publishing all-X words (ADR-0139). The harness
gives VexRiscv no data path to its ROM, so keep read-only data out of ROM there.
**A pairwise row at the ISA that ONE pair actually shares beyond RV32IM is a fourth, fifth, sixth
and seventh row, printed alongside the three-way one rather than replacing it** (ADR-0160 as
amended): littlecpu-vs-VexRiscv at rv32imc and littlecpu-vs-Hazard3 at RV32IMA, on both benchmarks,
plus the "littlecpu alone at its native ISA" row for each. Their clocks are the figures above,
cited and not re-measured. **A costs neither core anything**: the pairwise-A cycle counts are
digit-identical to the RV32IM row's. **C now costs littlecpu nothing and VexRiscv a little**:
littlecpu reads 228,825 at rv32im, rv32ima, rv32imc and rv32imac on Dhrystone, and 359,507 on
CoreMark at rv32im and rv32ima against 359,505 at rv32imc and rv32imac, from images that differ in size (Dhrystone's text is
1,968 bytes without C and 1,356 with), so the equality is a result and not one shared binary;
VexRiscv pays +2.59% on Dhrystone (269,629) and +2.61% on CoreMark (437,545). Against the opposite
finding before the refactor (littlecpu +1.10% and +3.09%, VexRiscv +9.76% and +3.80%), the cycle gap
between the two cores at the ISA they share *widens* rather than narrows: VexRiscv takes 1.178× and
1.217× littlecpu's cycles at rv32imc, against 1.149× and 1.186× at rv32im.
- **The fetch-loop dead ends are retired history, not current ceilings.** ADR-0076, 0078, 0083,
  0087, 0091, 0092, 0097, 0100, 0113, 0129 and 0175 priced candidates against a fused decoder whose
  fetch address closed a loop through decode. The D/X split and the stateless fetch replaced that
  pipeline (ADR-0208, ADR-0221), and each of those ADRs carries a pointer amendment saying its
  measurement stands as dated and prices a pipeline that no longer exists. Four still carry a
  rule that outlives it, and are cited where it applies: ADR-0097 (a period is
  spelling-dependent; the compressed decode stays closed for area), ADR-0113 (a cost that is a
  variance needs sixteen seeds), ADR-0129 (a harness cannot reach inside an instance) and ADR-0078
  (the 24 MHz step). **A register-only fetch address is `rtl/fetcher.v`, proven in `pcloop` and
  `traps`** (ADR-0221): the ROM is addressed off `fetch_pc_next` and the fetcher holds no state, so
  the tail leaves the loop and the head is still the block RAM's output. The one-window skid that
  shipped first cost placed cells the up5k did not have, and was deleted in the cell-trim pass
  (ADR-0221, amended); the registered head Stage A saw its clock from (ADR-0201) is a second
  window register and a two-cycle redirect, the four-word queue's price by another name.
- **yosys and ABC already do everything derivable from the expression** — dead bits, common
  subexpressions, duplicate adders — so an edit that restates the same arithmetic is a null
  (ADR-0088). **Redundant SOURCE TEXT is not redundant HARDWARE, and it predicts nothing about the
  period**: it changes how ABC factors the cone and so how nextpnr places it, in either direction.
  `rtl/writeback.v`'s `wen` masks are the worked example and they have measured BOTH WAYS on
  sixteen and twelve paired seeds — +2.83% of median period and the requirement at the worst seed
  to delete when filed, then −3.22% of median, −3.15% at the worst placement and −128 cells to
  delete a month later, which is how they come to be deleted now (ADR-0115, ADR-0117 as amended).
  Read the consumer before calling a redundant term free, and **re-take the measurement before
  citing it**: nothing in this tree expires a ceiling, so a declined edit keeps its authority until
  someone spends the placements. **What the tools cannot use is a fact from outside the
  expression** — a power-of-two parameter, an aligned window, an address
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
- **The period is still set by the ROM-to-ROM loop, and the window select is nearly half
  of the up5k's.** Read off the reports of `make soc-timing` (the pinned placement, 2026-10-01, `main`
  at 11cc506) and `make ecp5-timing` (one placement, the default seed). **Up5k: 75.51 ns
  (13.24 MHz), 21 LUT levels, 67.5% routing**, from a flip-flop to a block RAM's read address;
  `soc/depth/path_stages.py` charges its levels by the registers each folds in, which is what
  survives yosys's cell naming: **9 to `rtl/imemory.v` (the window select over the ROM's own
  output registers), 6 to the core (`dx_out`, then `x_redirect_q` and `x_redirect_target_q`
  feeding `fetch_pc_next`), 1 each to the accessor and `rtl/csrs.v`, 4 unattributed.**
  **ECP5: 26.30 ns (38.02 MHz) on one placement, 11.12 ns logic and 15.18 ns routing**, from the
  ROM's output (5.83 ns of clock-to-output, 22% of the period) to the same ROM's address pin.
  So taking fetch's own state out of the loop (ADR-0221) did not move the end of the worst path
  off the ROM; what limits the period now is the ROM output, the
  window select and decode that read it, and the redirect and `fetch_pc_next` arithmetic that
  address it, in one cycle. **No single term is a lever here either**: a whole-path reading finds
  candidates, never wins, and a cone that has no business on this path is the one thing that has
  paid (ADR-0115, ADR-0117). Re-read `soc.timing.rpt` and `ecp5.pnr.log` on the tree you mean to
  change; a worst path is a sample of one placement, and this bullet is a dated reading and not
  a ceiling.
- **Read logic levels apart**: a LUT level costs ~3.3 ns with interconnect, a carry hop ~0.34 ns
  without, so a change that trades a carry hop for a LUT level gets shallower by icetime's count
  and slower in nanoseconds. `soc/depth/path_stages.py` attributes a path, not a decision, and its
  level count orders nothing (ADR-0116); `soc/routing_bins.py` shows the routing on the fetch
  path was flat, with no long hop and no column to pin (ADR-0114). The SoC is routing-dominated
  and **there is no single lever**: reading `build/soc.timing.rpt` finds candidates, not wins.
  **Measure the whole set**: a ceiling over one term bounds only that term, and a ceiling is as
  perishable as a CPI cost, so re-take either on the tree you mean to spend it in.
- **A machine with spare cores and a CI pod saturated at its quota are different instruments.**
  Time a CI change as a whole shard at the concurrency CI runs it at, never one item alone: a
  solver swap read about 31% faster on a laptop and 8% as a real four-job shard in the four-CPU
  pod. Read pod memory from `/sys/fs/cgroup/memory.peak`, which is cumulative per pod, so compare
  two configurations in separate jobs.

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
make setup          # fetches the pinned RISC-V gcc on both platforms (make riscv-gcc-setup);
                    # macOS also brews svlint, Linux points at make lint-setup for it
make doctor         # resolve and verify the RISC-V compiler, yosys, nextpnr-ice40 and
                    # icetime -- path, version, and icetime's chip database actually
                    # loading -- before a placement or a sweep is spent finding out
make lint           # svlint over rtl/, RVFI macros off and on. A required CI check
make lint-setup     # fetch the pinned svlint release into the tool cache
make riscv-gcc-setup # fetch the pinned xPack riscv-none-elf-gcc release into the tool
                    # cache; make test's riscv-gcc-pin-test grades that PATH resolves it
make test           # the test/asm suite (.S and .c) under cxxrtl + unit benches + probe-gates
                    # + every repo-scanning `*-test` target (memmap, march, band-source,
                    # retired-term, adr-numbering, port-connect, compare-geometry,
                    # vexriscv-path, tracked-ignored, tool-cache, pin-bump, abc-engine,
                    # zkt-isolation, fixture-freshness, makefile-target, lut4-site,
                    # pll-clock, probes-header, dhry-board-parity, macro-register,
                    # compare-product-schedule-publish, stall-sites, pin-help-text,
                    # formal-ci-coverage, yosys-script-oneline, tmp-path)
                    # + window-test, imem-share-test, board-elaborate, mutation-probe,
                    # dual-build, nano-test, nano-startup-test, nano-littlecpu-test and
                    # nano-qspi-loop-test; graded against EXPECTED_FAIL / OBSERVED_FLOOR,
                    # with STALL_REPORT=1 so the cycle-accounting identity runs on every
                    # call, not only `make cycles`
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
make cycles         # the suite, every cycle charged to an issue or one of the six stall
                    # reasons; nonzero on a cycle none explains -- the same check `make test`
                    # now runs by default, but this target also prints the full CPI table and
                    # the two load/store locality counters. Not on CI -- there is no CPI ratchet
make dhrystone      # Dhrystone 2.1 (test/bench) -> DMIPS/MHz and the same accounting;
                    # DHRY_RUNS picks the count. Not on CI, no ratchet
make coremark       # CoreMark (test/bench) -> CoreMark/MHz, SIMULATED AT 16 KB OF ROM;
                    # COREMARK_ITERATIONS picks the count. Not on CI, no ratchet
make waves          # iverilog leg -> waves.vcd; one baked-in program, not the suite
make monitor-check  # regenerate test/monitor.v at the pin into a temp file and diff it;
                    # `make test/monitor.v` rewrites the tracked copy
make fit            # the core's area number; ratchet on FIT_MAX_LC
make soc-timing     # the SoC place-and-time flow; requirement on SOC_MIN_MHZ. With no
                    # SOC_SEED override this grades the pinned placement (soc/pin.json,
                    # ADR-0171), PIN STALE on a source change. An explicit SOC_SEED
                    # (soc/timing_sweep.sh runs four) bypasses the pin
make soc-seed-search # off `make test` and CI, like `make fit`: sweeps high-entropy seeds
                    # and writes soc/pin.json at >=5% margin over SOC_MIN_MHZ.
                    # SOC_SEARCH_SEEDS overrides the seed list, SOC_SEARCH_COUNT the
                    # default draw's size
make bitstream      # icepack the board wrapper into build/board.bin; BOARD_OSC=internal uses
                    # SB_HFOSC instead of the crystal. No board needed
make prog           # iceprog build/board.bin onto the UPduino; root on macOS
make suite-board    # the .S suite on the part, in batches, read back over the UART; root
make dhrystone-board # Dhrystone built for the board; flash with `make prog`, read the UART
make coremark-board # CoreMark built for the up5k at COREMARK_UP5K_CFLAGS (-Os -flto, not
                    # COREMARK_CFLAGS' -O2); flash with `make prog`, read the UART.
                    # `make coremark-rom-up5k` builds that image alone
make icesugar-bitstream # the iCESugar-Pro (ECP5) bitstream; ICESUGAR_PROG picks the program,
                    # ICESUGAR_ROM=noop-rom takes banks another recipe already wrote.
                    # No --freq: the LPF states the pad, nextpnr derives the core
                    # domain from the PLL's dividers, and missing it is an ERROR
make icesugar-prog  # load it into SRAM over JTAG. NOT the flash: a flash write leaves the
                    # part at `@cdone:0`, unconfigured, until it is physically power-cycled
make icesugar-read  # read that board's UART for a bounded window
make icesugar-dhrystone # build Dhrystone for it, load it, read the report it prints itself.
                    # Needs the board, so off `make test` and off CI, like suite-board
make icesugar-coremark # the same for CoreMark, at SOC_ROM_WORDS=4096: 16 KB of ROM, which
                    # this part has the spare block RAM for and the up5k does not.
                    # `make coremark-rom-ecp5` builds that image alone
make dual-smoke     # two harts, one text storage, one arbiter, under cxxrtl; one program run
                    # both ways. Off `make test` and CI. `make dual-elaborate` is iverilog's look
make dual-ecp5-timing # the dual top placed, ECP5 only; three censuses GATE, the frequency
                    # PUBLISHES, nothing merges with an up5k number
make ecp5-timing    # the SoC on ECP5 at a declared corner; three censuses GATE, the
                    # frequency PUBLISHES, no ratchet. ECP5_SEED picks a placement
make netlist-digest # the mapped netlist's digest; `make netlist-diff BASE=<ref>` names what
                    # moved. Digest unchanged, NO SWEEP IS OWED; changed, sixteen seeds are.
                    # Sound in ONE direction: digest-equal implies the placer's input is
                    # unmoved. Digest-different no longer implies a semantic (RTL-meaning)
                    # change -- on this toolchain a comment CAN move the mapped netlist of
                    # a large file, measured on rtl/csrs.v (ADR-0170) -- so it still means
                    # spend the sweep. netlist-determinism is a prerequisite, and its
                    # comment-class case now exercises a large representative file rather
                    # than the small one that could never have caught this
make compare-timing # this core, VexRiscv and Hazard3 in ONE harness, on the two parts
                    # this design ships to. COMPARE_PART picks one -- up5k (default),
                    # where the 12 MHz step GATES pass/fail and the comparison is
                    # cycles alone, or ecp5, where Fmax is a factor and PUBLISHES with
                    # no ratchet. COMPARE_CORE picks a core, soc/compare/sweep.sh's
                    # COMPARE_CORES sweeps a subset. The placed-vs-synthesised check
                    # and, on up5k, the step gate are both graded
make compare-smoke  # all three harnesses run one image in iverilog and must agree
make compare-dhrystone  # Dhrystone on all THREE cores, one RV32I image, one simulation ->
                    # DMIPS/MHz each, plus a fourth row of this core alone at its native
                    # ISA so the shared subset's cost is a number. COMPARE_DHRY_MHZ adds
                    # the absolute column. Not a gate, not on CI
make compare-coremark # CoreMark on all THREE cores, one RV32IM image -- VexRiscv's
                    # generated build has M but no A, the same ceiling Dhrystone
                    # already builds at. COMPARE_COREMARK_MHZ adds the absolute
                    # column. Not a gate, not on CI
make compare-product # both factors of every cross-core pair in one run, stamped into
                    # soc/compare/product.json with the commit, seeds and CFLAGS behind
                    # each number. COMPARE_PRODUCT_SEEDS picks the sweep (twelve by
                    # default). Not a gate, not on CI -- a scheduled workflow re-takes
                    # it weekly and opens an issue when it moved

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
make -C formal complete_cover       # its anti-vacuity control, tied to complete's depth
                                    # by cover-depth-tie.py over the run's own log
make -C formal imemcheck            # the fetch window's and the data bus's memory-interface
make -C formal dmemcheck            #   checks. Depth is graded against F/G by
                                    # check-memcheck-depth.py, a Makefile prerequisite
make -C formal imemcheck_cover      # each memcheck's own anti-vacuity control, at the
make -C formal dmemcheck_cover      #   same depth, behind a forced-red stalled-bus probe
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

make nano-liberty-setup # once: fetch the pinned sky130hd liberty nanocpu's area
                    # instrument reads
make nano-area      # nanocpu's area, local `synth; dfflibmap; abc -liberty`, never
                    # merged with the brief's own TT-flow/LibreLane number; ratchet
                    # on NANO_MAX_UM2. Not on `make test`'s path; no-ops until
                    # nano/nano.v lands
make nano-timing    # area AND delay from one delay-oriented synthesis run, both
                    # register-file builds, typical corner only; no ratchet, a ranking
                    # instrument like `make cycles`. Prints its own limits and the
                    # dated, stale correlation against the last real flow run. Not on
                    # `make test`'s path, the same standing as `make nano-area`
make nano-test      # nano/asm's six hand-written x0-x15 programs under BOTH sim legs --
                    # nano-sim (cxxrtl) and nano/tb/nano_icarus.vvp (iverilog, wrapped by
                    # nano_sim_icarus.sh behind nano-sim's own CLI) -- graded against
                    # nano/asm/EXPECTED_FAIL / OBSERVED_FLOOR and required to agree with
                    # each other program by program (nano_dual_leg_test.sh). A real-tool
                    # prerequisite, nano_x_probe.sh, forces the iverilog leg to catch an X
                    # a skipped memory-zeroing loop leaves behind. On `make test`'s path
make nano-startup-test # the shared nano/bench/start.S initializes gp before any
                    # gp-relative reference runs; PASS/FAIL over tohost. On `make test`'s path
make nano-dhrystone # Dhrystone on nanocpu under nano-sim --bench, core-only, zero-wait-state,
                    # 80 KB flat memory (nano/tb/nano_memory.v). Not on `make test`'s path
make nano-coremark  # CoreMark on nanocpu, same memory model and standing as nano-dhrystone
make nano-qspi-sim  # nano-sim built against nano/tb/nano_qspi_memory.v instead of
                    # nano_memory.v -- a behavioural QSPI-flash/PSRAM timing model on the
                    # same bus, nano.v untouched. NANO_QSPI_PREFETCH_DEPTH, NANO_QSPI_LOOP_KIND,
                    # NANO_QSPI_LOOP_WINDOW and NANO_QSPI_PREAMBLE_CYCLES parameterize it;
                    # always rebuilt, since the generated file's mtime cannot distinguish
                    # one parameter set from another
make nano-qspi-timing # sweeps that model's configurations against Dhrystone and CoreMark,
                    # reporting cycles, DMIPS/MHz or CoreMark/MHz at an assumed 64 MHz,
                    # and the {execute, parcel wait, redirect preamble, loop hit, handshake,
                    # PSRAM wait} bucket split. Reporting only, no ratchet, like `make cycles`.
                    # Not on `make test`'s path
make nano-qspi-loop-test # the loop buffer's three invariants, each able to fail: a branch-free
                    # program costs the same cycles with the loop buffer on or off; a loop
                    # resident in it pays no marginal preamble/wait per iteration once warm;
                    # and a loop with a load and a block-straddling instruction runs to PASS
                    # in both shapes, where every nano-qspi-sim exits 7 on a fetch served from
                    # parcels its flash run never streamed (ADR-0186). nano-qspi-loop-probe
                    # is its forced-red prerequisite. On `make test`'s path
make nano-qspi-pins-sim  # nano.v -> nano/qspi.v (the real bit-serial QSPI controller) ->
                    # pin-level flash/PSRAM behavioural models, sck/cs_n/sio rather than
                    # nano_qspi_memory.v's abstract bus. nano-qspi-pins-test reruns the
                    # suite through it; NOT on `make test`'s path -- a chained-resume bug
                    # in multi-instruction programs, root-caused in docs/adr/0202, fix
                    # in progress
make nano-qspi-resume-test  # the chained-resume bug's minimal reproduction: nano_qspi_ctrl
                    # and the pin-level models directly, no nano.v. Committed red until
                    # the fix lands; nano-qspi-resume-probe is its forced-red prerequisite,
                    # proving the comparisons -- not a crash or a timeout -- drive the FAIL
make -C nano/formal components_qspi  # nano_qspi_ctrl's three invariants (CS0/CS1/CS2
                    # never low together; no PSRAM CS-low interval exceeds
                    # PSRAM_CS_LOW_LIMIT clocks; the prefetch buffer holds exactly the
                    # parcels at [fetch_pc, fetch_pc+N)) by k-induction; qspi-probe is
                    # its forced-red prerequisite
make -C nano/formal components_traps # nano's trap entry and interrupt path: the only harness
                    # that leaves irq_meip free, with traps-region-probe and
                    # traps-tval-probe as forced-red prerequisites
make -C nano/formal ill_e # RV32E's register-naming restriction, read off nano's own RVFI report
make -C nano/formal ill_e_cover # its anti-vacuity control: reachable on the correct core, tied
                    # to ill_e's depth by cover-depth-tie.py, behind ill-e-cover-probe, which
                    # stalls the bus and requires the goal to go unreached
make -C nano/formal nonperturbation # nano's RVFI instrumentation is unread by the core; the same
                    # script as littlecpu's, `check-nonperturbation.py nano`, behind
                    # nonperturbation-probe.py, which leaks an rvfi_* bit into a real output
make -C nano/formal interrupt-tie-off # nano's INTERRUPT_TIE_OFF, both directions
make -C nano/formal all # every target above plus check, the memchecks and components_memreq;
                    # test/formal_ci_coverage_test.py requires each to be run by a ci.yml step,
                    # for both designs' `all` lists
```

`make sail-setup` and `make lint-setup` unpack into `~/.cache/little-cpu` (`XDG_CACHE_HOME` moves
it), **outside the checkout**: a git worktree is given tracked files only and a downloaded tool is
gitignored, so an install inside the checkout is invisible from every worktree. `make test`
enforces it.

Toolchain: `make riscv-gcc-setup` fetches the pinned RISC-V gcc on both platforms (ADR-0190); svlint
is `brew install svlint` on macOS or `make lint-setup` on Linux. Tests are freestanding — `.S`, and
`.c` built `-nostdlib -ffreestanding` against `test/crt0.S` — so no multilib or newlib. Formal needs
the YosysHQ OSS CAD Suite, which CI takes at the latest release; it is the one tool that floats.
Everything else downloaded or vendored is pinned and refuses a command-line override: riscv-formal
(`formal/pin.mk`), sail-riscv and svlint (`SAIL_RISCV_VERSION`, `SVLINT_VERSION`), the RISC-V gcc
(`RISCV_GCC_VERSION`), Hazard3 (`soc/compare/hazard3_pin.mk`) and CoreMark (`COREMARK_PIN`). CI runs
on every PR (`.github/workflows/ci.yml`); read the required set live from
`gh api repos/thejefflarson/little-cpu/branches/main/protection`, not from comments. The runners
are self-hosted ARC pods declared in the sibling `cluster` repo (`argocd/apps/runners.yaml`,
`charts/actions/runners/values.yaml`): read their CPU and memory limits there, and record the date
a decision was measured against them. `nproc` and `free` inside a pod report the host.

## Engineering rules

- **Compiler and elaboration warnings are errors.** Two allowlisted exceptions, each documented
  where it is allowlisted: iverilog's `sorry: constant selects in always_* processes` for
  `rtl/writeback.v`'s `always_comb` struct reads (over-sensitivity, provably safe; do not add new
  ones outside that file); and yosys's `Deep recursion in AST simplifier` notice on the
  `elaborate` CI job.
- **No file may be more than 5% comment lines**, graded per file by
  `test/comment_density_test.py` on `make test`; `docs/comment-budget.md` is the derivation
  and says which comment-shaped lines are code. Prose that outgrows the budget moves to
  `docs/` rather than dying: the manifests' formats live under `docs/manifests/`, the
  constraint files' headers in `docs/pin-constraints.md`.
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
- **Scratch goes in the worktree; downloaded tools go in `~/.cache/little-cpu`.** The question that
  sorts them is whether two worktrees would want the same bytes: a downloaded tool, yes; a build
  product or a log someone reads afterwards, no, and it belongs under the gitignored `build/`
  (`$(BUILD)`), where one worktree cannot overwrite another. `mktemp` is for a file nothing reads
  after the script exits. `test/tmp_path_test.sh` refuses a literal `/tmp/` in a tracked Makefile or
  shell script. It cannot see an ad-hoc command, so `make test > …` and its kin write under the
  worktree or the session's scratch directory, never a fixed `/tmp` name.
- **riscv-formal is SHA-pinned.** A pin bump regenerates `test/monitor.v`, re-runs the generated
  checks, and re-derives the sanitizer's site counts and `COMPLETE_EXCLUSIONS` rather than editing
  them to silence a failure.
- **No ticket IDs in code, comments, ADRs, docs, or commit messages.** Cite the ADR, the commit
  SHA, or just say the reason. PR titles and descriptions are the exception.
- **`git config --local` in a worktree writes the checkout's one shared `.git/config`**, so a
  change meant to be local to one worktree is live in all of them until it is unset. Use an
  isolated clone for anything that needs its own git config.
- **A fresh `git worktree` has no `formal/riscv-formal`**: its absence surfaces as `Current isa
  string 'rv32imc' not supported` plus a list of `insn_*` checks never generated, which reads
  like a regression in the branch. A flat symlink to the main checkout's clone is unsafe too, not
  just an easy fix: a generated check reaches sources through `../..` from inside the clone, and
  the OS resolves `..` AFTER following a symlink, so those reads land in the MAIN CHECKOUT's
  tree, not this worktree's -- silently stale for littlecpu's `rtl/` (the main checkout has its
  own copy) and loudly broken for `nano/` (the main checkout may have none at all). Make
  `formal/riscv-formal` a real directory in the worktree instead, holding one symlink per
  top-level entry of the pinned clone, so `..` from inside it stays in this tree; confirm it by
  reading which source paths yosys's own log reports opening. Confirm a suspected pre-existing
  failure against CI, never against another fresh worktree, which fails the same way.
- **Knowledge about this repo lives in this repo.** A rule goes in this file, a measurement with
  its date in an ADR, and owed work in the tracker. An agent's private memory (Claude Code's
  per-project memory directory, outside the checkout) holds only preferences about how to work
  with the owner, never a fact about the code, the toolchain, CI or a measurement: nothing grades
  it, and no other session or engineer agent can read it. A session that learns such a fact
  records it here, in an ADR or in a ticket before it ends; a private memory found to be about the
  repo moves here and is deleted.
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

On the fetch-refactor tree `make fit` reads 4,347 packed cells locally and 4,332 in the `fit` job, against `FIT_MAX_LC` 4,441 and `make
soc-timing` places at 5,084 of 5,280 `ICESTORM_LC`, with eight seeds at 12.57–13.24 MHz and
`soc/pin.json` holding seed 20382078 at 13.24 MHz (ADR-0220 derives the budget and records the
sweep; the ECP5 and dual figures are not re-taken there).

The SoC is 8 KB of ROM in block RAM plus 64 KB of data RAM in two of the part's four
`SB_SPRAM256KA`; `SOC_EXPECT_SPRAM` and `SOC_EXPECT_EBR` hold both counts exactly. It places, meets
12 MHz, and has run on the part: `make prog` flashes an UPduino v3.0, the UART's counter decoded
digit-perfect, and Dhrystone on the part matched cxxrtl to the cycle (ADR-0130). `make suite-board`
and `make dhrystone-board` run the suite and Dhrystone there; neither is graded on CI, because a
board is not always plugged in. **The same source runs on a second part**: a MuseLab
iCESugar-Pro (ECP5 LFE5U-25F) reports Dhrystone at **0.775 DMIPS/MHz, 19.4 DMIPS at 25 MHz** over
`make icesugar-dhrystone`, **cycle-identical to cxxrtl on the same binary** (ADR-0163). **That
bitstream now clocks the core from an `EHXPLLL` at 30 MHz rather than from the 25 MHz pad**, which
is spendable on this part and would not be on the up5k, whose clock is a step function
(ADR-0172): the frequency is picked under the WORST of twelve paired seeds on the shipping top and
under `make ecp5-timing`'s own worst of twelve, and `--freq` is gone from the recipe so nextpnr
derives the core domain from the PLL's dividers and a missed period is an ERROR rather than a
slow board. **`CLOCK_HZ` is `rtl/uart.v`'s baud divisor, so the clock and that parameter must move
together or the board's only output is garbage**; the frequency is stated in four places and
`test/pll_clock_test.py` recomputes them from the pad and the dividers. Predicted 23.3 DMIPS at an
unchanged 0.775 DMIPS/MHz, and **no board has run it**. Sharing
`DHRY_CFLAGS` does not make a board figure and a simulated one comparable: `dhrystone-rom` also
defines `DHRY_UART` and `make dhrystone` does not, which moves `.text` and costs a cycle a run.
`DHRY_BOARD_EXTRA_DEFINES` names that one difference in one place, and
`test/dhry_board_parity_test.sh` compiles both builds' translation units and requires the two files
`DHRY_UART` never touches to come out byte-identical, so a second, undocumented divergence is
caught rather than described after the fact.
Getting there needed the data RAM's out-of-range arm off the block RAM's reset, and that board is
programmed by loading SRAM over JTAG — a flash write leaves the part unconfigured until it is
power-cycled. SPRAM cannot be initialised, so `.data` rides in
the ROM at a load
address `test/asm/boot.lds` puts there and `test/crt0.S` copies into RAM before `main`. Still
deferred: the radix-4 divider (a CPI lever that costs area, so never part of an area pass,
ADR-0038), booting a program out of the flash, an interrupt controller, more interrupt sources and
a vectored `mtvec`. Forwarding ships as X's own mux (ADR-0222); the fused decoder's full network
is among the retired fetch-loop dead ends under Measurements.

**The ECP5 holds 16 KB of text, and that is where CoreMark reaches a board** (ADR-0165).
`rtl/littlesoc.v` takes `ROM_WORDS` as a parameter and the three `littlesoc` synthesis
flows `chparam` it before `hierarchy`, so a wider ROM is one override rather than an
edit; at 4096 words the LFE5U-25F reads `DP16KD` 36 → 40 of 56 and Fmax 35.11 → 34.78 MHz,
a null. The default is unchanged, so every existing target is a no-op. **Keep every
multi-command `yosys -p` script on ONE line**: a backslash-newline inside the quotes is
stripped by GNU Make 3.81 (macOS) but reaches yosys under 4.x (the runners), which stops
with `No such command: \`. `test/yosys_script_oneline_test.py` refuses the pattern in the
Makefile and `nano/*.mk` on `make test`.

**8 KB of text is the ceiling on the up5k, and it is the fetch loop's, not the part's.** `rtl/imemory.v`
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
  has been behind on it three times by quoting a number. **A new number is checked against every
  open PR branch, not only main**: `test/adr_numbering_test.sh` reads one tree, so two branches
  can each claim the same next number and both pass it, the way #363 and #364 both took ADR-0188
  in the same week. Before claiming one, run
  `gh pr list --state open --json headRefName` and `git ls-tree <branch> docs/adr/` on each result,
  and pick a number none of them already holds. **This is checked mechanically too**, by a CI job
  this manual rule does not make redundant: the "adr collision check" workflow
  (`.github/workflows/adr-collision-check.yml`, `.github/scripts/adr-collision-check.sh`) reads
  every other open PR's added `docs/adr/` files over the GitHub API and fails naming the number
  and the other branch when two collide. It runs only on pull requests and is not part of
  `make test` — the API call is non-hermetic — and it does not replace
  `test/adr_numbering_test.sh`: that check catches a stale or duplicated row within one tree,
  which no cross-branch comparison can see, and the CI job catches two branches claiming one
  number, which no single-tree check can see. Picking a good number stays cheaper than being
  caught claiming a bad one, so the manual rule stays too.
- Briefs: [`docs/ideas/`](docs/ideas/) — list the directory rather than trusting an enumeration
  here. Where a brief and an ADR disagree, the ADR wins.
- Reference text from the old core: `git show 1709433^:rtl/riscv.v` (RVFI retire block),
  `git show e67875c^:rtl/alu.v` (arithmetic).
- Work is tracked in Linear, project **Little CPU** (team JEF) — named so you know where the queue
  is; nothing in this repo depends on it.
