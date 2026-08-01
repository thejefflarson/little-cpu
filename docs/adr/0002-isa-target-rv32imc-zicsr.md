# ADR-0002: ISA target is RV32IMC_Zicsr_Zifencei, machine mode only

**Status:** Accepted · 2026-07-27

## Context

The repo's ISA target was ambiguous. The README says "RISCV 32 base ISA (IM)"; the root Makefile
passes `-DRISCV_FORMAL_COMPRESSED`; `test/monitor.v` is generated with `-i rv32imc`;
`formal/checks.cfg` says `isa rv32imc`; and `rtl/decoder.v` carries substantial but incomplete
C-extension decode. Meanwhile `test/asm/` contains 13 RV64-only tests, and there is no CSR support
at all.

The compressed extension is expensive to verify: it complicates instruction fetch and roughly
doubles the riscv-formal `insn_*` check count. An initial recommendation was to cut it.

## Decision

**Target RV32IMC_Zicsr_Zifencei, machine mode only.** The C extension stays.

- `misa = 0x4000_1104` (MXL=1, C+I+M). Neither Zicsr nor Zifencei has a `misa` bit — they are
  named extensions, so the ISA string is the only place either is claimed.
- **Zifencei is claimed rather than declined, because the core already implements it correctly
  and for free.** `rtl/decoder.v` decodes `fence.i` and retires it as a NOP, which is exactly a
  conformant Zifencei on this design: one hart, no instruction cache, so instruction fetch is
  always coherent with stores and there is nothing to invalidate. **The state before this
  amendment was the one combination that was wrong** — the string did not claim Zifencei while
  the decoder accepted `fence.i` anyway, so the core was silently permissive: an instruction from
  an unclaimed extension retired instead of raising illegal-instruction. Claiming it costs
  nothing and makes the RTL and the string agree; declining it would have meant *removing*
  working, correct logic to raise a trap nobody wants. Added to `-march` in `test/run_tests.sh`
  and `test/cosim.py`, without which the assembler refuses to emit `fence.i` at all — which is
  why `fence.i` had no test before `test/asm/fencenop.S`.
- Traps implemented: illegal instruction (2), breakpoint (3), load address misaligned (4), store
  address misaligned (6), environment call from M-mode (11).
- **Instruction-address-misaligned (mcause 0) is unreachable and not implemented** — with C, only
  2-byte alignment is required; branch and JAL immediates have bit 0 = 0, and JALR clears bit 0.
  `mepc` hardwires bit [0] only.
- No interrupts: `mie` and `mip` are read-only zero (no interrupt sources exist).
- No S-mode, no U-mode, no PMP, no `mtval` values (hardwired 0, which is spec-legal).

The existing `-DRISCV_FORMAL_COMPRESSED`, `-i rv32imc` monitor generation, and
`checks.cfg`'s `isa rv32imc` were already correct and are **kept unchanged**.

Decode gaps to close: **C.EBREAK** (quadrant 10, funct4=1001, rd=0, rs2=0 — currently falls between
`instr_cadd` and `instr_cjalr` into illegal), plus `mret`, `wfi`, `fence`, and `fence.i`.

## Rationale

**C stays because code density is a product constraint, not a preference.** The core targets an
ice40 up5k, whose on-chip memory is small. Compressed instructions buy roughly 25–30% code size on
typical RV32 code, which directly determines what programs fit. That outweighs the verification
cost, provided the cost can be contained — and ADR-0003 shows it can, at zero cost to the design's
central invariant.

Secondary: the decoder's C support is already substantial (all the CI/CR/CB/CJ forms, RV32-only
C.JAL correctly present, and the reserved encodings that must trap already falling through to
`!instr_valid`). Cutting it would discard working code and require touching every config that
already says `rv32imc`.

Machine-mode-only Zicsr is the minimum that makes traps precise and satisfies riscv-formal's
`[csrs] mcycle minstret`. Anything more (S-mode, PMP, interrupts) has no consumer in this design.

## Consequences

- riscv-formal's `insn_*` check set grows to the full `isa_rv32imc.txt` (~75 checks). These are
  individually small BMCs; they land in the nightly tier, with `insn_c_addi`, `insn_c_lw`, and
  `insn_c_j` in the PR smoke set.
- Fetch requires the dual-word window of ADR-0003.
- The 13 RV64-only `.S` tests are deleted (they cannot assemble for RV32 — there is no "RV32 port"
  of `addw`). `fence_i.S` is deleted too: self-modifying code is impossible on this Harvard core.
- The test suite gains compressed coverage it does not currently have at all: riscv-tests'
  `rv32uc/rvc.S` plus a straddle-specific test.
- `rvfi_insn` reports the 16-bit value zero-extended for compressed instructions;
  `rvfi_pc_wdata` steps by 2 or 4.
- up5k budget stays feasible at roughly 55–70% of the part (see the brief's decision 13).
