# Finish the rewrite: Little CPU to verified RV32IMC_Zicsr

**Status:** brief settled 2026-07-27 · ladder M0–M4 · target `RV32IMC_Zicsr`, machine mode, ice40 up5k

## Idea

Complete the half-finished serialized→pipelined rewrite of Little CPU: finish the pipeline
control the rewrite never reached, port forward the formerly-green riscv-formal harness, add
the CSR/trap story neither core ever had, and land sim + formal green in CI.

## Problem & context

This is not a broken CPU — it is a **stalled rewrite**.

Git history shows a 491-line serialized FSM core (`rtl/riscv.v` + 159-line `rtl/alu.v`, single
memory port with a picorv32-style `mem_valid`/`mem_instr`/`mem_ready` handshake) that **passed
riscv-formal sans CSRs**. It was torn down in two waves:

| Wave | Commits | Effect |
|---|---|---|
| 1 (2021) | `9758a39` *pipeline refactor wip* → `alu.v` deleted Apr → `riscv.v` deleted May in `1709433` *cleanup* → `9ea584a` *rename* (`pipeline.v`→`littlecpu.v`) | FSM core replaced by a handshake/skidbuffer pipeline |
| 2 (2023) | `49b317a` *remove skidbuffer* → `4fbd650` *structs and testbench* → `13fec44` *slash and burn* | stripped down to today's fused staged design |

Recent commits (`49dc5e9` *single cycle fetch*, `6871818` *single cycle accessor*, `fd36a52`
*fixes*) are the author working forward through wave 2.

**The `formal/` directory is the wave-0 harness that actually passed — not junk.** The plan is
therefore: *finish what was started, restore the verified property that was lost, then extend it
past where the old core ever got (CSRs).*

## Project state, honestly sorted

### (a) Genuine bugs — wrong in any architecture

1. `rtl/executor.v:28-30` — mulh sign bits taken from **bit 0 instead of bit 31**
   (`in.rs1[0] & in.is_mulh`), and the 33×33 product is formed from unsigned concats.
   MULH/MULHSU wrong for negative operands. Needs `$signed({rs1[31] & sign_en, rs1}) * $signed(…)`.
   **Invisible to riscv-formal under ALTOPS** — only sim and the component BMC catch it.
2. `rtl/executor.v:124` — restoring divider subtracts when `mul_div_x <= mul_div_y`; comparison
   is inverted (should be `>=`).
3. `rtl/memory.v:23-27` — on a pure read (`wstrb==0`) the final `mem_rdata <= mem_wdata`
   **overrides** the RAM read; loads through `littlesoc` never return memory contents. The inline
   testbench memory is correct; `memory.v` regressed.
4. `rtl/decoder.v:216-219` — `instr_ebreak` matches *any* SYSTEM funct12≠0 with funct3=0/rs1=0/rd=0,
   so `mret` (0x302) and `wfi` (0x105) decode as EBREAK. Must be funct12==1 exactly.
5. `rtl/fetcher.v:34` — component assertion `assert(out.pc == 32'b0)` after reset asserts a
   property of a free input; wrong as an assert.
6. `rtl/executor.v:102,109` — **divider iteration count is 65; correct value is 32.**
   `mul_div_y` starts at `rs2 << 31` and shifts right once per iteration, so iterations `i = 0..31`
   produce exactly 32 quotient bits. At 65, the divisor shifts to zero around iteration 33 and the
   remaining cycles keep shifting `mul_div_store` — the quotient is destroyed regardless of which
   direction the comparison in bug 2 goes. Plausibly a leftover from when this lived in the
   RV64-flavoured `alu.v`. Fixing it halves the divide stall for free.

Arithmetic reference when fixing 1/2/6: `git show e67875c^:rtl/alu.v`.

### (b) Unfinished rewrite scaffolding — not bugs, just unreached

- Registered-read regfile (`rtl/regfile.v:15-17`) with no compensating pipeline structure →
  operands are one instruction stale, **always**. This, not CSRs, is why "it doesn't work."
- `executor.stalled` computed (`executor.v:23-25`) but wired to nothing; no valid bits, so the
  accessor replays the previous load/store for the whole divide.
- RVFI ports declared (`littlecpu.v:14-45`), never driven. The old core's working hookup
  (`git show 1709433^:rtl/riscv.v`, `ifdef RISCV_FORMAL` block) was never ported.
- `// TODO: csrs` (`writeback.v:25`); CSR address/zimm dropped in decode; no `mret`/`fence` decode.
- `formal/wrapper.v`, `checks.cfg [script-sources]`, `dmemcheck`/`imemcheck` port lists still speak
  wave-0's interface.
- No `make test`; `riscv_test.h` missing entirely despite every `.S` including it; dangling
  `handshake.v`/`skidbuffer.v` references in `Makefile:22` and `components.sby`.

## Was the rewrite's direction right?

**Yes — finish the staged rewrite. Do not resurrect the serialized core as mainline or as a
co-sim oracle.**

1. The staged design is the repo's stated identity, and the author tore the old core down *twice*.
   Going back reverses years of intent to save roughly one milestone.
2. The fused-fetch design retains the serialized core's key verification property — no wrong-path
   instructions, no flush — while pipelining. It is the rare rewrite that keeps the old
   verification story.
3. The old core's green run depended on the single-port handshake memory interface that no longer
   exists anywhere in the SoC. Resurrecting it means maintaining two cores *and* two memory systems.
4. A co-sim oracle against the old core is redundant: **riscv-formal's ISA spec modules are already
   the golden model**, and sequential equivalence between an FSM core and a pipeline is not
   something `equiv_simple`/`equiv_induct` can do.

The old core's role is **reference text**: its RVFI retire block, rs1/rs2-validity masking, and mem
rmask/wmask conventions port near line-for-line into the shadow-payload design.

The real risk is not the rewrite's direction — it is *staying* in the current "was verified, now
unverified" limbo. Hence the ladder's explicit **parity checkpoint at M2**.

## What ports forward from the wave-0 harness

M2 is a **port, not a rewrite**.

| Artifact | Verdict | Delta needed |
|---|---|---|
| `formal/checks.cfg` | **Keep** | ISA/depths/`[csrs]` already right (rv32imc, mcycle/minstret). Update `[script-sources]` to the staged RTL; keep ALTOPS. |
| `formal/complete.sv` | **Keep** | Author already ported it to `littlecpu` in `fd36a52`. Finish: drive imem/dmem inputs rand-stable. |
| `formal/wrapper.v` | **Port** | Only file still instantiating `riscv` with the handshake. Rewrite port list to `littlecpu`; `RVFI_WIRES`/`RVFI_CONN` skeleton and fairness pattern carry over. Fairness becomes "bounded consecutive internal stall" (no `mem_ready` anymore). |
| `formal/imemcheck.sv` | **Port — valuable** | Already models fetch at **16-bit granularity** (`imem_data[15:0]`) — wave-0 anticipated compressed straddle. Re-point at the split `imem_addr`/`imem_data` interface; it directly stresses the new C fetch window. |
| `formal/dmemcheck.sv` | **Port** | Same shape, re-pointed at `mem_addr`/`wstrb`/`wdata`/`rdata` without the handshake fire condition. |
| `formal/cover.sv` | **Port (cheap)** | Counts loads/stores/compressed-vs-long retires — exactly the "do compressed instructions actually retire" sanity we want. Port list only. |
| `formal/equiv.sh` | **Keep — misunderstood gem** | Not serialized-vs-pipelined equivalence. It proves **RVFI instrumentation doesn't change core behaviour** (gold = plain build, gate = `RISCV_FORMAL` build with rvfi ports deleted). With big `ifdef` shadow payloads landing in the structs, this earns its keep. Update file list/top. |
| old `riscv.v` RVFI block | **Reference text** | Retire-edge driving, `rs1_valid`/`rs2_valid` masking (`!is_lui && !is_jal && !is_auipc`…), `rvfi_order` accumulation, mem capture conventions, `rvfi_csr_* <= 0` (exactly why the old run was "sans CSRs"). Port the *logic*, re-keyed from FSM states to the pipeline's valid bits. |
| old `alu.v` | Obsolete | Its mul/div moved into `executor.v` in the rewrite, bringing bugs 1/2/6 with it. |

Delete only the truly dead: `components.sby` handshake/skidbuffer tasks, `Makefile:22` references
to deleted RTL, the vacuous component tasks.

## Goals / non-goals

**Goals**

1. RV32IMC_Zicsr + machine-mode traps, correct and precise.
2. `make test`: `.S` suite assembled and run fast under cxxrtl, RVFI-monitored, exit-code green.
3. riscv-formal parity with the old core (M2), then `csrw`/liveness on top (M4).
4. CI: PR gate + nightly deep formal.
5. `CLAUDE.md` capturing invariants and command surface.

**Non-goals** — FPGA timing closure / nextpnr (deferred; feasibility budget below), interrupts
(`mie`/`mip` RO-zero), S/U-mode, PMP, `mtval` values, Spike/Sail co-sim (riscv-formal is the
oracle; revisit only if formal and sim disagree), README rewrite (the honest state lives in
`CLAUDE.md`), littlesoc SPI flash.

## C-extension design

C stays: ice40 code density is a product constraint (up5k SPRAM is small and `littlesoc` already
targets it).

### Fetch: dual-word combinational window — no state, no stall, no flush

The fetcher presents `imem_addr = {pc[31:2], 2'b00}` **and** `imem_addr2 = imem_addr + 4`; the
instruction handed to decode is the 32-bit window:

```
({imem_data2, imem_data} >> (pc[1] ? 16 : 0))[31:0]
```

This preserves the design's signature invariant completely: fetch stays combinational, the decoder
still owns the PC, branches still resolve with zero wrong-path instructions, and a straddling
32-bit instruction at `pc%4==2` costs *nothing* — no aligner FSM, no halfword buffer, no buffer
invalidation on redirect.

Rejected alternatives:

- **32-bit fetch + halfword buffer** — adds fetch state, straddle stalls, and redirect
  invalidation (a flush by another name).
- **Straddle-stall two-cycle fetch** — adds a state bit and mid-straddle redirect corners.

Cost of the window: a second combinational ROM read. Free in sim/formal (`imemory.v` and the
testbench read the array twice; `littlesoc` wires two reads). On real ice40 BRAM this becomes
even/odd interleaved 16-bit banks or negedge reads — deferred with the rest of FPGA timing. The
formal wrapper models both imem inputs as rand-stable; wave-0's 16-bit-granular `imemcheck.sv` is
the ready-made consistency check.

### Decode: mostly done, two real gaps

Present and plausible: C.LWSP/SWSP/LW/SW, C.J/JAL/JR/JALR, C.BEQZ/BNEZ, C.LI/LUI/ADDI/ADDI16SP/
ADDI4SPN, C.SLLI/SRLI/SRAI/ANDI, C.MV/ADD/AND/OR/XOR/SUB — with RV32-only C.JAL correctly present.

Reserved encodings that must trap already fall through to `!instr_valid` correctly: `16'h0000`
(canonical illegal, caught by the `caddi4spn_immediate != 0` guard), C.LUI/C.ADDI16SP with imm=0,
C.LWSP rd=0, RV32-reserved `shamt[5]=1` shift forms.

**Missing:**

1. **C.EBREAK** (quadrant 10, funct4=1001, rd=0, rs2=0) — currently falls between `instr_cadd`
   (needs rs2≠0) and `instr_cjalr` (needs rd≠0) into illegal.
2. **C-aware trap set**: with C, only 2-byte alignment is required, so
   **instruction-address-misaligned (mcause 0) becomes unreachable** — branch/JAL immediates have
   bit0=0 and JALR clears bit0. Drop it from the trap set; `mepc` hardwires only bit[0].

`pc_inc = uncompressed ? 4 : 2` (`decoder.v:278-279`) and the compressed-immediate muxing are
already correct. C.NOP and the rd=x0 HINT space fall out as harmless x0-writing base ops.

### RVFI with C

Keep `-DRISCV_FORMAL_COMPRESSED` (root Makefile already sets it) and the `-i rv32imc` monitor
(already right). Decode-stage shadow capture stores `rvfi_insn` as the 16-bit value zero-extended
for compressed instructions (mask the window's upper half when `quadrant != 2'b11`);
`rvfi_pc_wdata = pc + pc_inc` (2 or 4). Insn-check count grows to the full `isa_rv32imc.txt` set
(~75 checks) — each a small BMC; nightly tier, with a handful (`insn_c_addi`, `insn_c_lw`,
`insn_c_j`) in the PR smoke set.

### The decoder `$onehot` assertion

`decoder.v:396-406` currently FAILs. The property as stated — "for a valid instruction, exactly one
*merged base-op* signal fires" — is the right property; the vector already uses merged signals, so
compressed forms alias into their base op and contribute one bit. **The FAIL is a decode-hole
detector, not a broken property.** Known reachable holes: the C.EBREAK gap and the funct12-wildcard
`instr_ebreak` (bug 4).

Action: pull the counterexample from `formal/components_decoder/engine_0/`, fix the hole it names,
extend the vector with the new ops (`mret`, `wfi`, `fence`, csr ops), and add `cover` statements so
each op is provably reachable.

## Simulation: three legs, each load-bearing

**cxxrtl — the workhorse.** `test/cxxrtl.cc` grows from 27 lines to a ~150-line runner: parse args
(`--rom <hex> --cycles N [--vcd out.vcd]`), load the program hex directly into ROM via
`debug_items` (more robust than `$readmemh` through yosys), run until a write of the magic
pass/fail value to the `tohost` address, return a real exit code. `make test` = assemble every `.S`
→ `objcopy -O verilog --verilog-data-width=4` hex → run each under the cxxrtl binary.

What this buys: cxxrtl runs one-to-two orders of magnitude faster than iverilog for a core this
size, which matters exactly where this design hurts — even at the corrected 32 iterations, `div.S`
and `rem.S` are thousands of cycles, and a hazard/div torture test or an overnight randomised run
is only practical here. It also keeps the yosys frontend honest (same `write_cxxrtl` elaboration CI
uses). `test/rtl.cc` stays **generated at build** (already gitignored; delete the stale on-disk
copy, it isn't tracked). The RVFI monitor compiles into this leg, so every cxxrtl run is
self-checking per-retire, not just end-state-checking.

**iverilog — the microscope.** `test/testbench.v` stays the debugging bench: VCD waveforms,
`$display` tracing, same monitor instance. Runs one smoke program in CI to keep the
second-frontend elaboration path alive. Not the bulk runner.

**riscv-formal — the oracle.** Exhaustive per-instruction semantics and pipeline corners to
bounded depth, with ALTOPS masking real mul/div arithmetic — precisely the hole the other legs
cover.

### Do the `.S` files provide behavioural accuracy?

**Partially — one leg of a three-legged stool, and today that leg is missing a foot.**

What they uniquely give: end-to-end checks of real programs through the real assembler against
architected state, and critically **the only place the real divider and multiplier arithmetic gets
tested at all**, because riscv-formal runs under ALTOPS. Plus decent hazard coverage via
riscv-tests' built-in bypass-pattern macros.

What they don't give: exhaustiveness (fixed operand vectors, fixed instruction sequences — that's
riscv-formal's job).

Three concrete gaps:

1. **Thirteen files can never run.** `addw.S`, `addiw.S`, `sd.S`, `ld.S`, `lwu.S`, `sllw.S`,
   `slliw.S`, `sraw.S`, `sraiw.S`, `srlw.S`, `srliw.S`, `subw.S` are RV64-only — the suite was
   copied wholesale from riscv-tests, whose canonical sources are RV64; upstream's rv32 target
   simply *excludes* the `*w`/doubleword tests in its Makefile. There is no "RV32 port" of `addw` —
   the instruction does not exist. `fence_i.S` is likewise dead: on this Harvard core (ROM fetch,
   RAM data) self-modifying code is impossible. **Decision: delete them**, mirroring what upstream
   riscv-tests does for rv32. (Alternatives — keeping them inert forces `make test` to carry an
   exclusion list forever and misleads readers about coverage; "porting" them is not possible.)
2. **No compressed tests, and C is in scope.** Pull in riscv-tests' `rv32uc/rvc.S`, plus a small
   straddle-specific test (32-bit instruction at `pc%4==2`; branch into a misaligned target) since
   that's the novel fetch path.
3. **No trap/CSR tests.** Add `csr.S`, `ecall.S`, `ebreak.S`, `illegal.S`, `mret.S` in M3, adapting
   riscv-tests' machine-mode patterns.

Also: `riscv_test.h` is missing entirely. A minimal local version (RVTEST_CODE_BEGIN/PASS/FAIL
writing the `tohost` magic) is M1 work.

## `test/monitor.v` — tracked and useful

Stays tracked. It is deterministic output of
`riscv-formal/monitor/generate.py -i rv32imc -c 1 -a -p monitor`, and that ISA string is **already
correct** for the RV32IMC decision — no immediate regeneration needed.

Keeping it from rotting:

1. The generation command stays a Makefile target (the `test/monitor.v:` rule already exists) —
   add a header comment naming the riscv-formal pin it was generated from.
2. Regeneration is required exactly when the riscv-formal SHA pin or the ISA string changes.
3. **CI verifies freshness** by regenerating and diffing — cheap, catches drift.
4. Merge-conflict risk is near zero in a solo repo; conflicts resolve by regenerating, never by
   hand-editing.

Wired into `test/testbench.v` today (lines 102–126) and compiles into the cxxrtl build the same way
(it lives behind `RISCV_FORMAL`, and `testbench.v` is the cxxrtl top).

What it catches that `.S` pass/fail doesn't: per-retire checking of *every* instruction in *every*
run — decode legality, rd/rs consistency, PC chaining, memory-access consistency. A test that
corrupts state transiently but converges to the right final register values still fails loudly, and
torture/randomised runs become self-checking without expected-output files.

## `CLAUDE.md` contents (M0 deliverable)

- **What this is** — hobby RV32IMC_Zicsr core optimised for *readability*; mid-rewrite from a
  serialized core (pointer to git history + ADRs); README is the project's voice, state-of-the-repo
  truth lives here.
- **Invariants (do not break)** — fetch is combinational and the decoder owns the PC, so there are
  never wrong-path instructions and **no flush logic may be introduced**; all traps are detected and
  committed in decode, nothing faults after decode; every inter-stage struct carries a `valid` bit,
  bubbles are `valid=0`, retire is `valid` reaching writeback and drives `rvfi_valid`; hazards are
  handled by stall-only interlock in decode (no forwarding network — adding one is a CPI
  optimisation requiring an ADR); CSR instructions and `mret` serialize; regfile is
  combinational-read with write-through.
- **ISA target** — RV32IMC_Zicsr, M-mode only, `misa = 0x4000_1104`; traps: illegal=2, ebreak=3,
  load/store-misaligned=4/6, ecall-from-M=11; no interrupts; instruction-address-misaligned
  unreachable (C).
- **Command surface** — `make test` (cxxrtl runner over `test/asm`), `make waves` (iverilog VCD),
  `make -C formal` targets and the PR-vs-nightly split, monitor regeneration command, toolchain
  install lines.
- **Engineering rules in force** — warnings are errors; every non-trivial change adds/updates tests
  and runs the full suite before done; never commit build artifacts (`test/rtl.cc`, `sim`, `*.vvp`,
  `*.vcd`, formal output dirs); `test/monitor.v` is generated-but-tracked, regenerate never
  hand-edit; riscv-formal is SHA-pinned, bumping it requires regenerating the monitor and rerunning
  the ladder.
- **Pointers** — `docs/adr/` index, M-ladder status, known-deferred list.

## Key decisions

See `docs/adr/` for the load-bearing ones.

1. **ISA target: RV32IMC_Zicsr.** C stays for ice40 code density. Fetch becomes a dual-word
   combinational window; decode gains C.EBREAK; instr-addr-misaligned dropped as unreachable;
   `misa = 0x4000_1104`. The existing monitor/`checks.cfg`/`-DRISCV_FORMAL_COMPRESSED` config was
   already right and stays. → ADR-0002, ADR-0003
2. **Pipeline approach: fused fetch/decode, PC-in-decode, comb-read regfile + write-through,
   stall-only decode scoreboard, valid bits everywhere, divider freezes decode, no flush anywhere.**
   → ADR-0004
3. **Frame: complete the rewrite.** Distinguish real bugs (six) from unreached scaffolding; port
   wave-0 assets rather than rewrite. Serialized core = reference text, not a component or oracle.
   → ADR-0001
4. **Traps in decode; CSR file beside the decoder; CSR/`mret` serialize by draining E/A/W.**
   CSR set: RW `mstatus{MIE,MPIE,MPP}`, `mtvec` (direct mode), `mepc`, `mcause` {2,3,4,6,11},
   `mscratch`, `mcycle(h)`, `minstret(h)`; RO-zero `mtval`, `mie`, `mip`,
   `mvendorid`/`marchid`/`mimpid`/`mhartid`; `misa` RO. Zicsr side-effect suppression rules
   (CSRRS/RC with rs1=x0 or zimm=0 suppress the write; CSRRW with rd=x0 suppresses the read),
   illegal on RO-write or unimplemented CSR, counter-write beats increment, `minstret` counted at
   issue (equivalent to retire because post-decode nothing faults). `wfi`/`fence`/`fence.i` = NOP.
   → ADR-0005
5. **RVFI = shadow payloads in the stage structs**, ported from the old core's retire block
   (rs-validity masks, order counter, mem capture) rather than invented. Compressed insns report
   16-bit `rvfi_insn`; `pc_wdata` steps of 2. → ADR-0006
6. **Formal harness: port, don't delete.** Pin riscv-formal to a SHA. → ADR-0006
7. **Component proofs: exactly two.** Decoder onehot/valid (fixed), and an executor arithmetic BMC
   (~depth 70, real `/ % *` vs the divider/multiplier) — the ALTOPS hole-plug. Others deleted as
   vacuous. Note `components_executor` currently "PASSes" with **zero assertions**.
8. **cxxrtl is the primary sim runner**; iverilog kept for waveforms + second-frontend smoke;
   `test/rtl.cc` generated at build. → ADR-0007
9. **`test/monitor.v` stays tracked**, with regeneration rule + CI freshness diff.
10. **README untouched; `CLAUDE.md` is the truth file.**
11. **Toolchain:** brew `riscv64-elf-gcc` locally, `apt gcc-riscv64-unknown-elf` + pinned OSS CAD
    Suite in CI. Freestanding assembly needs no multilib libs;
    `-march=rv32imc_zicsr -mabi=ilp32 -nostdlib`.
12. **CI:** PR gate ≈ elaborate (warnings-as-errors) + full cxxrtl `.S` suite + iverilog smoke +
    2 component proofs + smoke formal (`insn_add`, `insn_c_addi`, `insn_lw`, `insn_c_j`, `reg`) +
    monitor freshness. Nightly = full genchecks ladder (~75 insn checks + reg/pc_fwd/pc_bwd/unique/
    causal, later liveness/csrw) + imem/dmem/cover/equiv. Green PR job = merge.
13. **up5k feasibility: fits, tight.** Rough budget on 5280 LUT4 / 5280 FF — comb-read regfile
    992 FF + ~600 LUT of read mux; decoder incl. C ~500–700 LUT; ALU/shifter ~400; divider datapath
    ~300 LUT / 200 FF; CSR file ~390 FF + ~200 LUT mux; pipeline structs ~300 FF; mul → 4 DSPs
    (Makefile already passes `-dsp`). Total ≈ 2.7–3.5k LUT, ≈ 2k FF — **55–70% of the part**. RVFI
    shadow logic is `ifdef`'d out of synthesis. Escape hatch if it doesn't close: negedge-BRAM
    regfile. FPGA timing itself stays deferred.

## Multiplier / divider performance

**The multiplier needs nothing.** `executor.v:30` is already a combinational 33×33 product
registered on the next edge, and `-dsp` infers 4 DSP blocks on up5k. Its only defect is the
sign-bit bug (#1).

**The divider's 65 is not a speed choice — it is bug #6.** The correct count is 32. Fixing it
halves the stall for free as part of M1 bug-fix work, not as an optimisation.

**Beyond that, do not optimise now.** Radix-4 would reach ~17 cycles but roughly doubles the
comparator/mux logic on a part already at 55–70%, and the payoff is thin from three directions:
formal never sees it (ALTOPS), cxxrtl-as-primary-runner absorbs most of the sim cost, and the one
genuine benefit — tightening the M4 liveness fairness bound — is already halved by the 65→32 fix.
Radix-4 and early-termination-on-leading-zeros are logged as deferred CPI items behind an ADR gate,
same treatment as the forwarding network. Both are safe post-verification additions; neither is
safe to do *while* the core is unverified.

## Risks & unknowns

- **The dual-word fetch window is the one genuinely novel piece.** Wave-0 fetched through the
  handshake; wave-2 never did C straddle. Mitigations already in the plan: `imemcheck` at 16-bit
  granularity, a dedicated straddle `.S` test, and the `insn_c_*` checks. This is where M2's
  surprises should be expected.
- **`csrw`/counter formal checks are new ground for this project** — the old core hardwired
  `rvfi_csr_* = 0`. The serialize-CSRs decision minimises corners; expect iteration in M4.
- **Solver runtimes** at depth 20–30 with ~75 insn checks: nightly-scale (est. 1–3 h wall on a
  runner with `-j`); PR smoke in minutes. Liveness may need a stall-fairness assumption — the
  divider's (corrected) 32 cycles sets the bound.
- **cxxrtl + monitor `$display` path** — recent yosys supports it, but if the monitor's error
  reporting misbehaves under cxxrtl, fallback is a C++-side check of the monitor's error flag.
  Small, contained.
- Genuinely open: nothing that blocks M0/M1.

## Milestone ladder

Ordered so the repo is never in a worse state than it started.

**M0 — Foundation (small).** Write `CLAUDE.md` (contents above, including the rewrite-history
narrative this brief recovered). Pin riscv-formal to a SHA. Delete dead references only
(`Makefile:22` handshake/skidbuffer, `components.sby` stale tasks, vacuous component tasks, the 13
RV64 `.S` files, stale on-disk artifacts). Add a monitor-freshness make target.
*Green = repo is self-describing; nothing works differently yet.*

**M1 — Finish the pipeline; sim green (the big one).** Comb-read regfile + write-through; valid
bits; decode scoreboard stall; divider stall integration; **C fetch window**; C.EBREAK + `mret` /
`wfi` / `fence` decode; fix all six real bugs; local `riscv_test.h`; cxxrtl runner + `make test`;
toolchain installed.
*Green = all RV32IMC `.S` tests (incl. new `rvc.S` + straddle test) pass under cxxrtl, and the
smoke test passes under iverilog.* **This is where "it doesn't work" dies.**

**M2 — Back to green (parity checkpoint).** Port RVFI from the old core's retire block into shadow
payloads; port `wrapper.v` / `checks.cfg` / `imemcheck` / `dmemcheck` / `cover` / `equiv.sh`; fix
the two kept component proofs.
*Green = the pipelined core proves everything the serialized core proved: rv32imc
insn/reg/pc_fwd/pc_bwd/unique/causal, no CSRs.* **This erases the regression from "verified" to
"unverified."**

**M3 — Past the old core: CSRs + machine mode.** CSR module, traps-in-decode commit, serialization;
trap/CSR `.S` tests; RVFI csr signals go from hardwired-zero to real.
*Green = M2 ladder still green + trap tests pass.*

**M4 — Full ladder + CI.** `csrw_mcycle` / `csrw_minstret`, liveness with stall fairness; both
GitHub workflows (PR gate + nightly); monitor freshness check.
*Green = nightly ladder fully green; tag a release.*

## Deferred

FPGA timing / nextpnr closure (budget says feasible; do after M4) · forwarding network (CPI-only,
ADR-gated) · radix-4 divider and early-termination (CPI-only, ADR-gated) · interrupts · `mtval`
values · Spike/Sail co-sim · littlesoc SPI flash · negedge-BRAM regfile (escape hatch, only if
synthesis demands) · the C decode paths are *kept*, not deferred.

## Handoff to plan-sprint

> **Theme:** Finish the serialized→pipelined rewrite and get back to formally green — pipeline
> control and C fetch first, harness port to parity second, CSRs third — with cxxrtl as the fast sim
> leg and the wave-0 formal harness ported, not rewritten.

Plan against M0–M4 with **M1 as critical path** and **M2 as the explicit restore-the-lost-property
checkpoint**. Decisions 1–13 are settled. Panel: **infra** — toolchain, CI, and formal mechanics
dominate the non-RTL work; no product or data questions in play.

## References

**Files:** `rtl/{decoder,executor,regfile,accessor,memory,littlecpu,fetcher,structs}.v` ·
`formal/{checks.cfg,wrapper.v,complete.sv,imemcheck.sv,dmemcheck.sv,cover.sv,equiv.sh,components.sby,Makefile}` ·
`test/{testbench.v,cxxrtl.cc,monitor.v,asm/}` · `Makefile`

**Historical:** `git show 1709433^:rtl/riscv.v` · `git show e67875c^:rtl/alu.v`

**External:** [riscv-formal verification procedure](https://yosyshq.readthedocs.io/projects/riscv-formal/en/latest/procedure.html) ·
[YosysHQ/riscv-formal](https://github.com/YosysHQ/riscv-formal) ·
[riscv-gnu-toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain/blob/HEAD/README.md) ·
[Homebrew riscv64-elf-gcc](https://formulae.brew.sh/formula/riscv64-elf-gcc) ·
[Sentinel riscv-formal setup (prior art)](https://sentinel-cpu.readthedocs.io/en/main/development/testing.html)
