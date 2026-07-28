# ADR-0003: Dual-word combinational fetch window for compressed instructions

**Status:** Accepted · 2026-07-27

## Context

The core's most valuable structural property is that **there are never wrong-path instructions**.
`rtl/fetcher.v` is purely combinational (`imem_addr = pc`, output = `imem_data` in the same cycle),
and `rtl/decoder.v` owns the PC and resolves branches in the same cycle the instruction is fetched.
Nothing speculative is ever issued, so **no flush logic exists anywhere in the design** — which is
simultaneously why the core is a good read and why its formal verification surface is small.

The C extension (ADR-0002) requires fetching 32-bit instructions that may straddle a 4-byte
boundary when `pc % 4 == 2`. The conventional answer — a fetch aligner with a halfword buffer —
introduces fetch state and, with it, buffer invalidation on redirect. That is a flush by another
name, and it would cost the invariant above.

## Decision

Fetch presents **two adjacent words combinationally** and selects a 32-bit window from them:

```
imem_addr  = {pc[31:2], 2'b00}
imem_addr2 = imem_addr + 4
instr      = ({imem_data2, imem_data} >> (pc[1] ? 16 : 0))[31:0]
```

The decoder masks the upper half when `quadrant != 2'b11` (compressed instruction).
`pc_inc = uncompressed ? 4 : 2`, which `rtl/decoder.v:278-279` already implements correctly.

## Rationale

The window is **stateless**. Fetch stays combinational, the decoder still owns the PC, branches
still resolve with zero wrong-path instructions, and a straddling 32-bit instruction at
`pc % 4 == 2` costs *nothing*: no aligner FSM, no halfword buffer, no invalidation on redirect, no
straddle stall. The invariant survives the C extension completely intact.

Alternatives rejected:

- **32-bit fetch + halfword buffer.** Adds fetch state, straddle stalls, and redirect invalidation
  — reintroducing exactly the flush semantics the design has never needed.
- **Straddle-stall two-cycle fetch.** Adds a state bit and creates mid-straddle redirect corner
  cases (what happens when a branch resolves while a straddle is half-fetched?).

## Consequences

- **Cost: a second combinational ROM read.** Free in simulation and formal — `rtl/imemory.v` and
  the testbench read the array twice; `rtl/littlesoc.v` wires two reads.
- **On real ice40 BRAM this needs work**: even/odd interleaved 16-bit banks, or negedge reads.
  Deferred with the rest of FPGA timing closure (post-M4). This does not block any milestone.
- The formal wrapper models both `imem_data` inputs as `rvformal_rand_reg` with stability
  assumptions while the PC is stalled, matching real ROM behaviour.
- `formal/imemcheck.sv` already models fetch at **16-bit granularity** (`imem_data[15:0]`) — the
  wave-0 harness anticipated compressed straddle. Re-pointed at the split `imem_addr`/`imem_data`
  interface, it becomes the ready-made consistency check for this window.
- **This is the one genuinely novel piece of the plan.** Wave-0 fetched through a handshake; the
  current design has never done C straddle. Expect M2's surprises here. Mitigations: `imemcheck` at
  16-bit granularity, a dedicated straddle `.S` test (32-bit instruction at `pc % 4 == 2`, branch
  into a misaligned target), and the `insn_c_*` formal checks.
