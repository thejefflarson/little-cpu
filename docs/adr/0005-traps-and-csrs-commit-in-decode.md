# ADR-0005: Traps and CSR accesses are detected and committed in decode

**Status:** Accepted · 2026-07-27

## Context

The core has no CSRs. `rtl/decoder.v:207-214` decodes `csrrw`/`csrrs`/`csrrc` (folding the
immediate variants into the register forms, losing the zimm-vs-rs1 distinction and never carrying
the CSR address from `instr[31:20]`), `rtl/executor.v:116` is an explicit empty statement for all
of them, and `rtl/writeback.v:25` is `// TODO: csrs`. `rtl/littlecpu.v:47-51` synthesises a `trap`
output combinationally with no `mepc`/`mcause`/`mtvec` behind it.

`formal/checks.cfg` already lists `[csrs] mcycle minstret`, so riscv-formal's `csrw` checks are in
scope. CSR reads and writes must be **precise** and correctly ordered against the pipeline.

## Decision

**All traps are detected and committed in decode. The CSR file is a sibling module of the decoder,
read and written in the decode stage. CSR instructions and `mret` serialize.**

- **Traps in decode.** Illegal instruction, `ecall`, `ebreak`, *and* load/store misalignment are all
  detected in decode. Misalignment detection is possible there because `mem_addr = rs1 + imm` is
  already computed in decode. **Nothing can fault after decode**, so no instruction ever needs to be
  killed downstream.
- **Serialization.** CSR instructions and `mret` are held in decode until execute/access/writeback
  drain (≤3 cycles). This makes counter reads trivially exact and eliminates every CSR/pipeline
  interaction corner.
- **CSR read results** ride the pass-through trick the decoder already uses for `lui`/`jal`:
  `out.rs1 <= csr_rdata; out.is_add <= 1; out.rs2 <= 0`.
- **`minstret` increments at issue**, which is equivalent to counting retires because post-decode
  nothing faults.

### CSR set (exact)

**Read/write:** `mstatus` (MIE, MPIE, MPP only; MPP is WARL → `2'b11`), `mtvec` (direct mode only;
base WARL, 4-byte aligned), `mepc` (WARL, bit [0] = 0 — only bit 0, because C makes 2-byte targets
legal), `mcause` (WLRL over the implemented codes), `mscratch`, `mcycle`/`mcycleh`,
`minstret`/`minstreth`.

**Read-only:** `mtval` = 0 (spec-legal), `mie` = 0, `mip` = 0 (no interrupt sources),
`misa` = `0x4000_1104` (RV32IMC), `mvendorid`/`marchid`/`mimpid`/`mhartid` = 0.

**Trap causes:** illegal instruction = 2, breakpoint = 3, load address misaligned = 4, store
address misaligned = 6, environment call from M-mode = 11. Instruction-address-misaligned (0) is
unreachable under C — see ADR-0002.

### Zicsr semantics that must hold

- CSRRS/CSRRC with `rs1 == x0` (or `zimm == 0`) **suppress the write** — no illegal-instruction
  trap on a read-only CSR.
- CSRRW with `rd == x0` **suppresses the read** side effects.
- Access to an unimplemented CSR, or a write to a CSR with `addr[11:10] == 2'b11` (read-only),
  raises illegal instruction.
- An explicit write to a counter **takes precedence** over that cycle's increment.
- `mret` restores `MIE ← MPIE` and `pc ← mepc`.
- `wfi` executes as a NOP (spec-legal). `fence` and `fence.i` are NOPs — no caches, Harvard ROM.

### Decode changes required

`decoder_output` gains `csr_addr` and `is_csr_imm`. Immediate-form CSR instructions must **not**
create an rs1 interlock (the zimm field is not a register). `mret` needs its own decode — currently
`rtl/decoder.v:216-219` matches *any* SYSTEM instruction with funct12 ≠ 0 as `ebreak`, so `mret`
(0x302) and `wfi` (0x105) both decode as EBREAK. `ebreak` must be funct12 == 1 exactly.

## Rationale

Because nothing faults after decode and issue is strictly in-order, decode-stage commit **is**
precise — no reorder buffer, no exception PC tracking down the pipe, no downstream kill. Serializing
CSR instructions costs at most 3 cycles on an instruction class that is rare in real code, and buys
away the entire class of CSR/pipeline hazards, including the hard one (what does `mcycle` read
when there are three instructions in flight?).

## Consequences

- CSR access latency is ~4 cycles. Irrelevant for this core's workload.
- The `csrw_mcycle` / `csrw_minstret` formal checks become tractable, because the counter's value at
  the moment of a CSR read is unambiguous.
- **This is new ground for the project** — the serialized core hardwired `rvfi_csr_* = 0`, which is
  exactly why its green run was "sans CSRs." Expect iteration in M4.
- Invariants for `CLAUDE.md`: nothing faults after decode; CSR instructions and `mret` serialize.
- ~390 FF for the CSR file (two 64-bit counters plus five small CSRs) and ~200 LUT of read mux.
