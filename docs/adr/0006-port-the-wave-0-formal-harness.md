# ADR-0006: Port the wave-0 formal harness; RVFI via per-stage shadow payloads

**Status:** Accepted · 2026-07-27

## Context

`rtl/littlecpu.v:14-45` declares the full RVFI port set, including `rvfi_csr_mcycle_*` and
`rvfi_csr_minstret_*`. **Not one of them is ever assigned** — `grep -rn "rvfi_" rtl/` finds no
drivers. riscv-formal therefore cannot pass.

The `formal/` directory looks stale: `checks.cfg`'s `[script-sources]` reads `rtl/riscv.v` and
`rtl/alu.v`, which no longer exist, and `formal/wrapper.v` instantiates a module named `riscv` with
a picorv32-style `mem_valid`/`mem_instr`/`mem_ready` handshake the current core does not have.

The first instinct was to treat all of this as dead and rewrite from scratch. Git archaeology
showed otherwise: **this is the harness from the era when the core passed riscv-formal sans CSRs**
(ADR-0001). It is proven-good work against a different interface, not junk.

## Decision

**Port the wave-0 harness rather than rewrite it**, and generate RVFI from **per-stage shadow
payloads** rather than a separate retire tracker.

### Harness disposition

| Artifact | Verdict | Delta |
|---|---|---|
| `checks.cfg` | Keep | ISA, depths, and `[csrs]` are already right (rv32imc, mcycle/minstret). Update `[script-sources]` to the staged RTL. Keep `RISCV_FORMAL_ALTOPS`. |
| `complete.sv` | Keep | Already ported to `littlecpu` by the author in `fd36a52`. Finish: drive imem/dmem inputs rand-stable. |
| `wrapper.v` | Port | The only file still instantiating `riscv` with the handshake. Rewrite the port list for `littlecpu`; the `RVFI_WIRES`/`RVFI_CONN` skeleton carries over. Fairness becomes "bounded consecutive internal stall" — there is no `mem_ready` any more. |
| `imemcheck.sv` | Port — valuable | Already models fetch at 16-bit granularity (`imem_data[15:0]`); wave-0 anticipated compressed straddle. Re-pointed, it directly stresses ADR-0003's fetch window. |
| `dmemcheck.sv` | Port | Same shape, re-pointed at `mem_addr`/`wstrb`/`wdata`/`rdata` without the handshake fire condition. |
| `cover.sv` | Port (cheap) | Counts loads/stores and compressed-vs-long retires — exactly the "do compressed instructions actually retire" sanity we want. |
| `equiv.sh` | **Keep** | Not serialized-vs-pipelined equivalence, as first assumed. It proves **RVFI instrumentation does not change core behaviour** (gold = plain build, gate = `RISCV_FORMAL` build with rvfi ports deleted). With large `ifdef` shadow payloads landing in the structs, this earns its keep. |

Delete only the genuinely dead: the `handshake`/`skidbuffer` tasks in `components.sby` (files
removed in `49b317a`) and the matching references at `Makefile:22`.

**Pin riscv-formal to a SHA** (the `formal/Makefile` currently clones unpinned HEAD against a
Jan-2023 vendored `genchecks.py` — a version-skew time bomb).

### RVFI generation

Extend the stage structs in `rtl/structs.v` with an `ifdef RISCV_FORMAL` section carrying `insn`,
`pc_rdata`/`pc_wdata`, rs1/rs2 addr+rdata, rd_addr, mem addr/masks/data, trap, and csr
rdata/wdata/masks. Capture in decode, carry down, drive `rvfi_*` from writeback. `rvfi_order`
counts retires. Trapping instructions retire as valid-with-`rvfi_trap`, `pc_wdata = mtvec`.

The **logic** is ported from the serialized core's retire block (`git show 1709433^:rtl/riscv.v`):
`rs1_valid`/`rs2_valid` masking (`!is_lui && !is_jal && !is_auipc`…), order accumulation, and
memory capture conventions — re-keyed from FSM states to ADR-0004's valid bits.

### Component proofs: exactly two

1. **Decoder** — the `$onehot` / valid-decode assertions, rewritten. The current FAIL at
   `decoder.v:396-406` is a **decode-hole detector, not a broken property**: the vector already
   uses merged base-op signals, so compressed forms alias correctly and contribute one bit. Its
   reachable holes are the missing C.EBREAK and the funct12-wildcard `ebreak`. Fix the holes,
   extend the vector with `mret`/`wfi`/`fence`/csr ops, and add `cover` statements so each op is
   provably reachable.
2. **Executor arithmetic BMC** (new, ~depth 70) — proves the divider and multiplier against the
   `/`, `%`, and `*` operators. **This is load-bearing precisely because ALTOPS means riscv-formal
   never checks the real arithmetic**, and the current code has three arithmetic defects
   (mulh sign bit, inverted divider comparison, wrong iteration count).

Delete the rest. `components_fetcher` asserts a property of a free input (`fetcher.v:34` asserts
`out.pc == 0` after reset, but `out.pc` is combinationally the free `pc` input) — the assertion is
simply wrong. `components_executor` currently **"PASSes" with zero assertions** — `executor.v:159-166`
contains only reset assumptions. `components_accessor` and `components_writeback` are likewise
vacuous.

## Rationale

A separate RVFI retire tracker would re-implement the pipeline registers it watches. The shadow
payload rides the valid-bit protocol that ADR-0004 introduces anyway, so retire detection is free.

Porting beats rewriting because the wave-0 files encode conventions that were *validated by a green
run* — particularly the rs-validity masks and memory mask semantics, which are the fiddly parts of
an RVFI hookup and the easiest to get subtly wrong.

## Consequences

- **M2 is a port, not a rewrite** — materially smaller and lower-risk than writing a wrapper from
  scratch.
- M2 becomes the **parity checkpoint**: the pipelined core proves everything the serialized core
  proved (rv32imc insn/reg/pc_fwd/pc_bwd/unique/causal, no CSRs).
- `equiv.sh` guards against the shadow payloads perturbing synthesis behaviour — a real risk given
  how much `ifdef`'d state the structs are about to carry.
- Solver runtimes: ~75 `insn_*` checks at depth 20–30 is nightly-scale (est. 1–3 h wall with `-j`);
  PR smoke runs in minutes. Liveness may need a stall-fairness assumption bounded by the divider's
  32 cycles.
- Bumping the riscv-formal pin requires regenerating `test/monitor.v` and rerunning the ladder. CI
  enforces monitor freshness by regenerating and diffing.
