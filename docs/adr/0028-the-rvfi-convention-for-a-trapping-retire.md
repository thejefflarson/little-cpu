# ADR-0028: The RVFI convention for a trapping retire

**Status:** Accepted · 2026-07-31 · *Supplements ADR-0006*
· **Corrected by [ADR-0037](0037-an-empty-baseline-is-not-m2.md): the `dmemcheck` bullet below is
STRUCK. The two shadows it relies on are not independent — `rtl/accessor.v` builds `rvfi_mem_wmask`/
`rvfi_mem_wdata` from the same bus signals `dmemcheck` samples — so they cannot desynchronise the way
described. Two mutations, including this bullet's literal scenario, left `dmemcheck` PASS. Two of the
three indirect guards remain, and one of those (`reg`) is inconclusive.**

## Context

riscv-formal's `rvfi_insn_check.sv` ends like this:

```systemverilog
if (!spec_trap) begin
    ...   // rd_addr, rd_wdata, pc_wdata, and every mem_* assertion
end
assert(spec_trap == trap);
```

**Under `spec_trap`, the only surviving assertion is the flag comparison.** `pc_wdata`, `rd_addr`,
`rd_wdata` and the memory masks are entirely unconstrained. The ladder cannot distinguish a core
that traps correctly from one that traps *and also* corrupts `rd`, writes memory, and redirects
somewhere arbitrary. It does not care where you trap to.

So the repo has to state what it drives on a trapping retire, and hold itself to it, because the
oracle will not.

## Decision

**On a trapping retire this core drives:**

| field | value |
|---|---|
| `rvfi_trap` | 1 |
| `rvfi_rd_addr` | 0 |
| `rvfi_rd_wdata` | 0 |
| `rvfi_mem_rmask` / `rvfi_mem_wmask` | 0 |
| `rvfi_pc_wdata` | `mtvec` |
| `rvfi_valid` | 1 — a trapping instruction *does* retire |

The instruction retires. It just retires having architecturally done nothing except redirect.

## Which existing checks enforce which parts, indirectly

This is the load-bearing part of the ADR, because none of it is obvious and CI would not notice if
a future change broke the reasoning:

- ~~**`dmemcheck` catches a trapping store that still reaches the bus.**~~ **STRUCK — see ADR-0037.**
  The claim as written was: `formal/dmemcheck.sv:61-68`
  builds its environment shadow from the **real** `mem_wstrb`/`mem_wdata`, while `rvfi_dmem_check`
  builds its shadow purely from `rvfi_mem_*`. A store that is architecturally suppressed but still
  strobes the bus desynchronises the two, and a later load fails at the RVFI level.
- **`reg` catches a trapping instruction that still writes `rd`.** `rvfi_reg_check` runs on the RVFI
  stream, but `rvfi_rs1_rdata`/`rs2_rdata` are captured from the *real* regfile in decode — so a
  corrupted `rd` surfaces as a later instruction's read disagreeing with the shadow model.
- **`pc_fwd`/`pc_bwd` catch a dishonest or untaken redirect.** They assert
  `pc_wdata(N) == pc_rdata(N+1)` with **no trap special case**, so the reported target must be the
  one actually taken. Note they are satisfied by *any* target, including `mtvec == 0`.

**What nothing in the ladder enforces: that the target is `mtvec`, and that `mepc`, `mcause`, and
`mstatus` are correct.** That gap is why the trap commit path gets its own component proof.

## Consequences

- The convention above is a repo commitment, not a spec requirement discovered by tooling. It must
  be restated in `rtl/writeback.v` at the drive site, because a reader of that code has no other way
  to learn that the ladder is silent here.
- Any future change to trap reporting must be checked against the **two** surviving indirect guards
  above by hand (the `dmemcheck` one is struck — ADR-0037 — and `reg` is inconclusive, ADR-0023) — they are not named in any check's title and the connection is not discoverable from CI
  output.
- The top-level `trap` port on `rtl/littlecpu.v` is **redefined, not deleted**: it becomes a
  one-cycle "trap entry committed this cycle" pulse. Deleting it would change the port list in
  `formal/wrapper.v`, `formal/dmemcheck.sv`, `formal/imemcheck.sv`, `formal/cover.sv`,
  `formal/complete.sv`, and `test/testbench.v` for no functional gain, and the harness check in
  ADR-0029 needs the signal anyway.
