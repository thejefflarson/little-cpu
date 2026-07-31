# ADR-0034: What the CSR ladder checks cannot see, and the decisions the CSR file forced

**Status:** Accepted · 2026-07-31 · *Recorded at integration. Amends ADR-0005's decode-side field
list and ADR-0027's note on the `h` halves; supplements ADR-0028 and ADR-0033 on what a green
ladder does not establish; corrects a `CLAUDE.md` engineering rule.*

## Context

The CSR file (`rtl/csrs.v`) landed as a sibling of the decoder, with `csrw_mcycle_ch0`,
`csrw_minstret_ch0` and `csrw_mscratch_ch0` all passing and `formal/EXPECTED_FAIL` matching
exactly at 79 checks / 70 pass / 9 fail. Four questions came up during that work that the
existing ADRs did not answer, and one measurement contradicted a rule `CLAUDE.md` states as a
documented exception. All five are recorded here rather than left in a review thread, which is
the treatment ADR-0033 established for this shape of finding.

## Decision

### 1. `decoder_output` loses the three Zicsr op flags; `csr_addr`/`is_csr_imm` stay, with a sunset condition

`is_csrrw`/`is_csrrs`/`is_csrrc` are **gone, and their removal is forced rather than cosmetic.**
ADR-0005 prescribes that the CSR read result rides the `is_add` pass-through. `rtl/executor.v`'s
op select is a single `(* parallel_case *) case (1'b1)`, so an instruction that set both `is_add`
and a CSR flag would make two arms match at once — under `parallel_case` that is an assertion to
synthesis that cannot happen, i.e. a lie, not mere redundancy. The executor's own `$onehot0` over
the flag set says the same thing. They could not coexist with ADR-0005's own prescription.

`csr_addr` and `is_csr_imm` were added as ADR-0005 specifies and **have no downstream consumer
today** — a CSR access is read, computed and committed entirely in decode, so nothing after decode
needs either. They are **kept**, for one reason: without `RISCV_FORMAL` there is no `rvfi.insn` in
the struct, so they are the only place a waveform reader on the iverilog leg can tell a CSR access
apart from the add it is disguised as. That leg is load-bearing (`CLAUDE.md`'s verification table).

**The sunset condition is explicit, so this cannot rot quietly: if the trap-entry change lands
without either field acquiring a consumer, strike them both in that change.** Thirteen bits of
`decoder_output` is not worth carrying indefinitely on a debuggability argument alone, and a
struct field nothing reads is exactly the incidental machinery this project's stated goal warns
against.

### 2. ADR-0027's trapping-`minstret` rule belongs in `trap.S`, not `minstret.S`

ADR-0027 states two counter rules. They have different preconditions and the partition follows
that, rather than following the file names:

- **exactness** (`csrr`/`nop`×3/`csrr`/`sub` differs by exactly 4 only because the stall exists)
  needs no trap, and stays in `test/asm/minstret.S`, which passes today;
- **a trapping instruction does not increment `minstret`** needs a *takeable* trap, which does not
  exist until trap entry lands. It is written and waiting as `test/asm/trap.S` tests 14 and 15,
  and `trap.S` stays baselined in `test/EXPECTED_FAIL`.

**Confirmed as correct.** Splitting a single ADR's rules across two files is the right call when
one rule's precondition is a feature that does not exist yet: the alternative is either baselining
`minstret.S` wholesale — losing the exactness check that *does* pass — or writing the assertion
nowhere and rediscovering it later. Test 15 re-reads `trap_count` so test 14 cannot pass against a
sequence that never entered the handler.

### 3. `genchecks` defines `RISCV_FORMAL_CSRWH` itself, so the `h` halves are exercised

The assumption going in was that the ladder would not reach `mcycleh`/`minstreth`. **That was
wrong, and the correction matters.** `formal/genchecks-local.py` does

```python
    if csr_mode and insn in ("mcycle", "minstret"):
        print("`define RISCV_FORMAL_CSRWH", file=sby_file)
```

unconditionally for exactly those two. With it defined, `checks/rvfi_csrw_check.sv` admits the
`h` address in its `assume`, and asserts `csr_insn_changed_full[31:0] == 0` on a high-half write
and `[63:32] == 0` on a low-half one.

So `rtl/csrs.v` reporting **per-half masks** (`{{32{wr_mcycleh}}, {32{wr_mcycle}}}`) is not merely
a tidy choice — it is **required**. A uniform 64-bit mask across both halves would fail that
assertion on an otherwise correct core. ADR-0027's remark that the `h` halves are out of reach is
amended accordingly.

### 4. `csrw_mscratch_ch0` cannot see the CSRRS/CSRRC `rs1 == x0` write suppression

Measured, not assumed. Deleting that suppression from `rtl/decoder.v` leaves `csrw_mscratch_ch0`
**PASSing**. `rvfi_csrw_check` reasons entirely over reported masks and data, and a
suppressed-write-that-fires computes `rdata | 0` — the same value, with `smask` and `cmask` both
zero — so no assertion in the check can distinguish it. The rule only becomes architecturally
observable once an illegal write to a read-only CSR raises a trap.

`test/decoder_tb.v` **does** fail on that mutation, which is the layered defence working as
designed rather than a defect in either layer.

For the record, a mutation the ladder check **does** catch: making CSRRC compute `rdata | arg`
instead of `rdata & ~arg` gives `bad state property 11 reachable at bound k = 30 SATISFIABLE`.
Both mutations were reverted; the finding is recorded beside the `[csrs]` list in
`formal/checks.cfg`.

**This is the same species as ADR-0028's "which existing checks enforce which parts, indirectly"
and ADR-0033's audit of the machinery M2 is measured by.** A green `csrw_*` is a statement about
the reported CSR interface, not about the decode-side suppression rules that feed it.

Relatedly, and already recorded beside that list: `mtvec`, `mepc`, `mcause` and `mstatus` are
deliberately **off** `[csrs]`, because `rvfi_csrw_check.sv` has no WARL model and would fail them
on a *correct* core. Those are checked field-by-field in `test/csr_tb.v`.

### 5. `CLAUDE.md`'s `sorry:` attribution was wrong, and the rule it carries pointed at the wrong file

The engineering-rules bullet said the 20 iverilog `sorry: constant selects in always_* processes`
notes were "all from `rtl/executor.v`'s `in.is_*` flags", and closed with "do not add new ones
outside `executor.v`".

**Measured: `rtl/executor.v` emits zero. All 20 come from `rtl/writeback.v`**, against
`in[311:0]` (`accessor_output`):

```
iverilog -g2012 -s executor  rtl/structs.v rtl/executor.v    ->  0
iverilog -g2012 -s writeback rtl/structs.v rtl/writeback.v   -> 20
```

The reason is principled rather than a toolchain accident: the diagnostic is about an **inferred**
sensitivity list, so it is specific to `always_comb`. `rtl/executor.v`'s `case (1'b1)` over 39
flags sits inside `always_ff @(posedge clk)`, whose sensitivity is written out and needs no
inference. `rtl/writeback.v`'s two `always_comb` blocks are the real source.

This is load-bearing misinformation in a clause whose whole purpose is to bound an accepted
warning: as written it forbade adding notes in the one file that has none, and licensed them in
the file that has all of them. `CLAUDE.md` is corrected, and now states the `always_comb`
mechanism and the one-line command that attributes them, so the next reader can check rather than
inherit the claim.

## Consequences

- The count stays **exactly 20**. The CSR change kept it there by driving `rtl/writeback.v`'s
  twelve new CSR payload reads as continuous assigns rather than adding them to the `always_comb`
  block — which is now the documented pattern for any new struct-field read.
- `fence` and `wfi` are **still unrecognised instructions** rather than the NOPs ADR-0005
  specifies. `instr_ecall`/`instr_ebreak` require `funct12` of exactly 0 and 1, and `fence`'s
  opcode is not in `instr_valid` at all. This is harmless today — an unrecognised instruction has
  its execution flags suppressed and no trap is taken — but it becomes **wrong the moment trap
  entry makes "unrecognised" mean illegal-instruction**, at which point a legal `fence` faults.
  Recorded here because that is the change that must fix it.
- `formal/complete` was already failing for retiring `ecall`/`ebreak`/unrecognised instructions;
  CSR instructions join that set. It is not on any gate, and ADR-0022/ADR-0023 already record it.
- Nothing here changes the M2 signal. Every ladder check is `mode bmc`, so PASS means "no
  counterexample within the configured depth", and everything still runs under
  `RISCV_FORMAL_ALTOPS` (ADR-0010).
