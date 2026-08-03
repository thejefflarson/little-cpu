# ADR-0071: The trap commit path is proved by k-induction, and its priority encoder is still vacuous

**Status:** Accepted · 2026-08-02 · *Supplements [ADR-0011](0011-misalignment-detection-stays-in-accessor-until-m3.md),
[ADR-0028](0028-the-rvfi-convention-for-a-trapping-retire.md) and
[ADR-0030](0030-trap-cause-priority-and-why-the-causes-are-disjoint.md). Adds a fourth task to
`formal/components.sby`.*

## Context

riscv-formal's `checks/rvfi_insn_check.sv` wraps every value comparison it makes — `rd_addr`,
`rd_wdata`, `pc_wdata`, all five `mem_*` fields — in `if (!spec_trap)`. Under `spec_trap` the only
surviving assertion is `assert(spec_trap == trap)`: the core has to *say* it trapped, and nothing
more is checked. ADR-0028 recorded which existing checks cover the consequences indirectly —
`dmemcheck` for a store that was suppressed but executed anyway, `reg` for a corrupted `rd`,
`pc_fwd`/`pc_bwd` for the redirect itself.

**Nothing anywhere checked that the trap target is `mtvec`, or that `mepc`, `mcause` and `mstatus`
are right.** `pc_fwd` and `pc_bwd` compare the core's reported `pc_wdata` against the core's own
next `pc_rdata`, so they are satisfied by *any* target the core commits to consistently, `mtvec` of
0 included. And riscv-formal ships no spec model at all for `ecall`, `ebreak`, `mret` or `csrr*` at
the pin, so `complete` excludes the whole SYSTEM class by declaration
(`formal/COMPLETE_EXCLUSIONS`).

`rtl/decoder.v`'s own `ifdef FORMAL` block does assert `pc == mtvec` after a trap. It asserts it
against a **free input**: in `components_decoder` the decoder is alone, and `mtvec`/`mepc` are
whatever the solver picks. That is the right proof for that task and it is not a proof about the
CSR file.

The ladder is also `mode bmc` throughout. A trap either redirects correctly or it does not, forever;
that deserves an unbounded answer.

## Decision

**A fourth task, `traps`, in `formal/components.sby`**, `mode prove` like the other three, over
`formal/traps.sv` — the fetcher, the decoder and the CSR file wired together the way
`rtl/littlecpu.v` wires them. `make -C formal components_traps`, and a step in CI's required
`components` job.

**No RTL changed.** The proof found no defect in the trap path.

### What it asserts, and how it observes state it cannot reach

`mtvec_value` and `mepc_value` are output ports of `rtl/csrs.v`, so they are read directly.
`mcause`, `mstatus` and the two counters are not: they are internal registers.

They are read **the way software reads them**. `csr_addr` is `instr[31:20]` on every cycle, whatever
the instruction is, and `csr_rdata` answers it combinationally in the same cycle. The instruction
word is a free input to the harness, so the solver can point that address at any register on any
cycle — including the cycle after a trap, and including the trapping cycle itself, because a
reserved-opcode word carries twelve bits the decoder hands straight to the CSR address port. No
reference model of `rtl/csrs.v` was needed and none was written.

The assertions, in groups:

- **Interlock.** The three stall reasons that arrive as inputs (`divider_stall`, `accessor_stall`,
  `fetch_stall`) each stop the decoder issuing. Nothing raises a CSR enable, a retire, a trap or an
  `mret` on a cycle that did not issue. A cycle that did not issue leaves `mtvec`, `mepc` and every
  register the read mux answers for exactly as it found them.
- **One general no-side-effects assertion**, over every address the read mux serves: a CSR reads
  back what it read last cycle unless `csr_wen`, `trap_entry` or `mret_entry` named it. The two
  counters are excluded on the cycles they advance, and only then — `mcycle` on every cycle,
  `minstret` only when `instret` is high.
- **Trap commit.** `pc` takes the `mtvec` the CSR file held when the trapping instruction issued.
  `mepc` takes the faulting instruction's own address with bit 0 cleared and bit 1 preserved.
  `mcause` takes the cause an independent oracle in the harness says it should. MIE moves into MPIE
  and MIE goes to zero. `decoder_out.rd` is zero and no load or store flag survives.
- **`mret`.** `pc` takes `mepc`, MPIE comes back as MIE, MPIE is set. The MIE half reaches one cycle
  further back than the other assertions, because an `mret`'s own instruction word puts `0x302` on
  the CSR address port and so cannot read `mstatus`; the harness reads `mstatus` the cycle before
  and holds it still across the edge in between.
- **`minstret` does not count a trapping issue.** Asserted twice — on the count enable, and on the
  register through a held address — because a counter that ticked from somewhere else would satisfy
  the first alone.
- **WARL.** `mtvec[1:0]` is zero, `mepc[0]` is zero, `mstatus.MPP` reads `2'b11`. All three are
  inductive and none needs a guard.

### The cause oracle is written from the ISA, not transcribed from the decoder

Working out whether an arbitrary 32-bit word is legal is most of decode, and a copy of decode makes
a poor oracle for decode. So the harness names a **small set of encodings the ISA fixes** and states
what each must do: `ecall` (11), `ebreak` and `c.ebreak` (3), a misaligned uncompressed `lw`/`lh`/
`lhu` (4), a misaligned `sw`/`sh` (6), and two encodings illegal in every conforming RV32
implementation — opcode `7'b1111111`, which is reserved for instructions longer than 32 bits, and
the all-zero halfword (2). It also names three encodings that **must not** trap, because a core that
trapped on everything would otherwise satisfy most of the file.

The effective address is computed in the harness from the I- and S-format immediates and `reg_rs1`,
each sum its own statement with both operands marked `$signed`. A signed sum written as an arm of a
conditional takes its signedness from the other arms, and that has produced silently unsigned
arithmetic in this repo twice.

### `fetch_stall` is deliberately left free

`formal/arbiter.v` exists so that `fetch_stall` is a free input in none of the whole-core harnesses:
an environment free to choose it can hold the core still forever, which starves `hang` and
`liveness_ch0`. **This task has no liveness assertion**, and for a safety proof a free stall input
is an over-approximation — strictly more cycles to check, never fewer. `formal/pcloop.sv` leaves it
free for the same reason. Instantiating the arbiter here would constrain the input and weaken the
result.

### Assumptions

Three, all of them the same one: reset is high before the first clock edge and low forever after.
It stands for the harness convention every task in this tree uses and for `rtl/littlesoc.v`'s
power-on counter. It carries more weight here than in `pcloop`: `mtvec`, `mepc`, `mcause` and
`mstatus` have no initial value in the RTL, so without it the base case starts them anywhere and the
two WARL assertions fail on registers nothing had written yet.

**No other assume.** In particular the RTL is read **without** `-formal`, so `rtl/decoder.v`'s own
`assume(in.pc == pc)` is not compiled into the instance — the `fetcher` instance answers that
question here instead. Confirmed structurally: the prepared model carries three `$assume` cells and
all three are `traps.sv:140-142`.

## Evidence

`PASS 0 2` — `successful proof by k-induction`, 2-3 seconds of wall time on an idle machine under
the default `smtbmc` (yices) engine, against `components_executor`'s 28 and `components_decoder`'s
6. **25 `$assert` cells** in the prepared netlist, read off the yosys log rather than inferred from
the word PASS, so this is not a fourth green task with nothing in it.

Eleven mutations. Ten produce a counterexample; one does not, and that one is a finding. A second
finding came out of a mutation that passed against the first version of the harness.

| mutation | result |
|---|---|
| `next_pc = mtvec` → `32'b0` on trap | FAIL, `traps.sv:334` step 4 |
| `trap_entry = issuing && trap_pending` → `!reset && trap_pending` | FAIL, `traps.sv:312` step 1 |
| swap the illegal and load-misaligned **arms** of the cause encoder | **PASS — see below** |
| swap the two cause **values** | FAIL, `traps.sv:343` step 3 |
| `instret = committing` → `issuing` | FAIL, `traps.sv:377` step 2 |
| trap does not push MIE into MPIE | FAIL, `traps.sv:351` step 4 |
| `mret` does not set MPIE | FAIL, `traps.sv:359` step 3 |
| drop `mtvec`'s WARL alignment mask | FAIL, `traps.sv:400` step 3 |
| trap entry clears `mepc[1]` as well as `mepc[0]` | FAIL, `traps.sv:340` step 4 |
| `csr_wen = committing && ...` → `issuing && ...` | FAIL, `traps.sv:389` step 2 — **see below** |
| make `load_misaligned` also raise illegal, so two causes overlap | FAIL, `traps.sv:343` step 3 |

### Finding 1: the arm swap is unfalsifiable, and that is ADR-0030 being right

Swapping two arms of a `case (1'b1)` whose guards are provably disjoint is a no-op. ADR-0030 already
says so in prose: an illegal instruction has no memory operand, so it cannot also be misaligned, and
the priority encoder is vacuous today. The task passes the swap, and it should.

What makes the order observable is an **overlap**, so that is the mutation to reach for. Widening
`instr_illegal` to include `load_misaligned` makes a misaligned `lw` commit cause 2 where the ISA
oracle says 4, and the task fails in under a second. The disjointness itself stays asserted where it
already was — `rtl/decoder.v`'s `$onehot0` over the five cause terms, checked by
`components_decoder`.

### Finding 2: the trap gate on `csr_wen` is redundant with the address decode

`csr_wen = committing && csr_write_op` waits for a non-trapping commit. Remove that wait and
**nothing about the architectural state changes**, which is why the first version of this task
passed the mutation. The reason is worth writing down: a CSR access can only trap by being illegal,
and it is illegal only when its address is unimplemented or read-only (`addr[11:10] == 2'b11`).
`rtl/csrs.v` writes neither — `warl` falls back to `rdata` and the write `case` takes its empty
`default` — so a spuriously enabled write on a trapping cycle lands nowhere.

The gate is still the rule the design states, so the task now asserts it structurally: a trapping
instruction commits no CSR access. That assertion is the only thing in the tree that would notice
the gate going away, and it makes the mutation red at `traps.sv:389` step 2.

## Consequences

- `make -C formal components_traps` is a step in CI's required `components` job. It adds about three
  seconds to a job whose 5-minute timeout is currently spent mostly on `components_executor`.
- **The task must be re-read whenever a trap cause is added.** The harness's cause oracle is a list
  of encodings and their causes; a new cause that overlaps an existing one changes what the oracle
  should say, and ADR-0030's priority order is what decides it.
- The observation technique — read internal CSR state through the architectural read port, with the
  instruction word free — is available to any future task over `rtl/csrs.v` and does not need a
  reference model.
- What this does **not** cover: `mtval` (read-only zero here, so there is nothing to commit), the
  counters' arithmetic beyond "they do not move when nothing asked" (`csrc_upcnt_*` on the ladder
  and `test/csr_tb.v` carry that), and anything downstream of decode — a trapping instruction's
  passage through the executor, accessor and writeback is `dmemcheck` and `reg`'s business, as
  ADR-0028 recorded.
