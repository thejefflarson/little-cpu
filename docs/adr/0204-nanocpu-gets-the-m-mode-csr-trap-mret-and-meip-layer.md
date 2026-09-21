# ADR-0204: nanocpu gets the M-mode CSR, trap, mret and MEIP layer

**Status:** Accepted · 2026-09-20

## Context

nanocpu (`nano/nano.v`) had no CSR file, no trap entry, no `mret` and no interrupt
input; every SYSTEM-opcode encoding decoded as illegal and halted the core. The donor
core this rewrite is built from never carried machine-mode state at all. Bringing
nano to M2-equivalent conformance — the same floor littlecpu already holds — needs the
mandatory RV32E M-mode CSR set, the traps nano can actually raise, `mret`, and one
external interrupt line for the board pin Tiny Tapeout scaffolding reserves. Wiring
that TT pin itself is a sibling ticket's scope; this one ends at a new `irq_meip`
core input.

## The CSR set

`misa` is `32'h4000_0014` — RV32EC, matching ADR-0197's decision to drop M from nano
rather than the brief's stale RV32IMAC-shaped proposal. Implemented, at the addresses
the privileged spec gives them: `mstatus` (`MIE`/`MPIE` only — no `S`/`U` bits, no
`MPP` beyond the M-mode-only field), `misa`, `mie` (`MTIE` read-only zero, `MEIE`
writable), `mip` (`MTIP` read-only zero — no `mtime`; `MEIP` reflects the synchronized
`irq_meip` pin), `mtvec`, `mscratch`, `mepc`, `mcause`, `mtval`, `mcycle`/`mcycleh` and
`minstret`/`minstreth` (64-bit, **writable**, unlike a read-only-by-convention
counter), `mhartid` (zero), `mvendorid`/`marchid`/`mimpid`/`mconfigptr` (zero), and the
performance-monitor address window (read zero, matching littlecpu's own floor — see
`CLAUDE.md`'s "conformance is not negotiable against minimality"). A CSR whose address
falls outside every named case and outside the perf-monitor window is unimplemented
and traps.

## Traps nanocpu can raise

Illegal instruction (2), breakpoint (3), load address misaligned (4), load access
fault (5), store address misaligned (6), store/AMO access fault (7, no AMO on nano so
this is store-only), ecall from M (11). **No cause 1**: nanocpu's fetch path has no
region check of its own to refuse against, matching littlecpu's own unreachable-cause
precedent. A load or store outside `RAM_BASE`..`RAM_BASE+RAM_WORDS*4` is a real fault
here, not a simulation artifact — nano's QSPI front end always routes fetches to flash
and loads/stores to PSRAM (`nano/qspi.v`), so an out-of-window access genuinely cannot
land in `.text`. Alignment is checked first: a misaligned access never also claims to
be out of range, the same ordering littlecpu uses.

`mret` restores `mstatus.MPIE` into `MIE` and sets `MPIE`, then redirects to `mepc`.
CSR instructions and `mret` hold the core in `execute_instr` until the transaction
resolves — nano has no reorder buffer, so this is the same one-instruction-at-a-time
serialization littlecpu's CSR/`mret`/`fence.i` commitment states, just free here
because nano never has more than one instruction in flight at all.

## One interrupt: MEIP

`irq_meip` is a new core input (`ui_in[7]` on the board — the pin itself is out of
scope, owned by a sibling ticket). It is synchronized two flops deep before it can set
`mip.MEIP` or raise `interrupt_pending`, matching the metastability discipline an
async board pin needs. The interrupt is taken only at a genuine instruction boundary
(`cpu_state == fetch_instr`), reported to RVFI as `rvfi_intr`, and drives `mcause =
0x8000_000b`. `mip.MTIP` stays zero: nano has no `mtime`.

## Three real bugs the formal harness found

Building `nano/formal/traps.sv` (nanocpu's own `formal/traps.sv`-shaped harness,
`riscv` instantiated directly, no core-level tie-offs) and the generated
riscv-formal checks against the new logic found three genuine correctness bugs, not
harness artifacts:

- **`mret` took the CSR write path.** `is_csr`'s original check was a bare
  `is_system_op` (the raw SYSTEM opcode plus "uncompressed"), which also matches
  `ecall`/`ebreak`/`mret` (funct3 000 under the same opcode). Since the CSR-write
  `always_ff` block gave `csr_wen` priority over the `is_mret` arm, `mret` silently
  never restored `mstatus.MPIE`. Found by `traps.sby`'s own BMC at shallow depth.
  Fixed by deriving `is_csrrw`/`rs`/`rc`/`wi`/`si`/`ci` from `is_system_op &&
  funct3 == ...` and defining `is_csr` as their OR, never the bare opcode.
- **An E-illegal CSR write still committed.** `csr_wen` did not check `take_trap`, so
  a CSR instruction naming a register x16–x31 (E-illegal, since nanocpu is RV32E)
  wrote through to the CSR before the trap it should also raise. Fixed:
  `csr_wen = is_csr && csr_write_op && cpu_state == execute_instr && !take_trap`.
- **`csrrsi`/`csrrci` with a zero immediate did not suppress the write.** The spec
  gives `csrrs`/`csrrc` (register form) and `csrrsi`/`csrrci` (immediate form) the
  same no-op-write rule when their source is zero; nano's `csr_write_op` only checked
  the register-form pair. A zero-immediate `csrrci`/`csrrsi` therefore still fired
  `csr_wen`, and since its computed new value equals the CSR's old value, it silently
  overrode `minstret`'s per-cycle auto-increment for that cycle instead of leaving it
  alone. Found by `csrc_upcnt_minstret_ch0` (a `csrrci x0, minstret, 0` between two
  reads broke strict monotonicity — `mcycle`'s own per-cycle tick masked the same bug
  in `csrc_upcnt_mcycle_ch0`, which is why only the `minstret` check went red).
  `nano/asm/csrimm.S` is the direct regression: two back-to-back reads bracketing a
  zero-immediate `csrrci`/`csrrsi` of `minstret`, asserting the gap is exactly 2.

Two more bugs were in the RVFI reporting nano.v feeds the checks, not in the core's
architectural behavior:

- **`rvfi_rd_addr`/`rvfi_rd_wdata` were not zeroed on a trapping retirement.** RVFI
  requires both to read zero when `rvfi_trap` is set; nanocpu reported the raw decoded
  `rd` field regardless. This was latent — no per-instruction check could reach the
  trap branch before region faults existed, since the `RISCV_FORMAL_E` assumption
  already excludes every E-illegal encoding from the general checks. Fixed by gating
  both on the captured `take_trap` at report time.
- **The fault channel's reported address was unaligned.** `nano/formal/checks.cfg`
  defines `RISCV_FORMAL_ALIGNED_MEM`, under which the generic spec model reports
  `spec_mem_addr` word-aligned (`addr & ~3`) even for a byte or halfword access.
  nanocpu's fault-channel override reported the raw, unaligned `load_store_address`
  instead, which is why every sub-word load and store fault (`lb`/`lbu`/`lh`/`lhu`
  and `sb`/`sh`) disagreed while the naturally-aligned `lw`/`sw` happened to match.
  Fixed by masking the captured fault address the same way the non-faulting path
  already does: `{load_store_address[31:2], 2'b00}`.

## The generic CSR checks have no RV32E model either

`rvfi_csrw_check.sv` and `rvfi_csrc_upcnt_check.sv` are the pinned, unforked
riscv-formal oracles — unlike `rvfi_insn_check.sv`, which `nano/formal/rvfi_insn_check.sv`
already forks specifically to add the `RISCV_FORMAL_E` register-range assumption.
Neither CSR checker has an RV32E model, so both would otherwise let x16–x31 name `rd`
(or `rs1` on a register-form CSR instruction) — an encoding nanocpu correctly traps
instead of committing. `nano/formal/checks.cfg`'s `[assume !csr[wc]_.*]` section
restricts exactly those two check families' own instruction word to RV32E's register
range, using the generic `assume_stmts.vh` mechanism genchecks already has rather than
forking two more files. (The pattern-negation is genchecks' own convention: a bare
pattern in `[assume ...]` *excludes* the matching checks from that assumption, so
including only the CSR family needs the leading `!`.)

## Verification

All 76 generated riscv-formal checks pass; `formal/EXPECTED_FAIL` and
`nano/formal/EXPECTED_FAIL` are both empty — `csrw_mcycle_ch0` and
`csrw_minstret_ch0` flip from the ticket's baselined FAIL to PASS, the acceptance
criterion asked for directly. `components_traps` over the new `formal/traps.sv`
passes, with `traps-region-probe.py` and `traps-tval-probe.py` as its forced-red
prerequisites (each mutant fails for its own reason; the shipping core passes both).
`ill_e`, `complete` (including its `COMPLETE_EXCLUSIONS` check — SYSTEM opcodes are
excluded the way littlecpu excludes them, since riscv-formal ships no spec model for
SYSTEM), `dmemcheck` and `imemcheck` all still pass. F and G re-measure to 12 and 10,
unchanged, on `make -C nano/formal remeasure-fg`.

`nano/asm/mul.S` and `divide.S` no longer just halt on the M-extension encodings they
can't execute (nanocpu has no M): each installs a trap handler, executes the encoding,
asserts `mcause == 2` and that `mepc` points at the offending word, advances `mepc`
past it, `mret`s, and finishes normally. `nano/asm/meip.S` (new) exercises the
interrupt path over `ui_in[7]`'s core-side signal directly: it arms `mie.MEIE` and
`mstatus.MIE`, busy-loops bounded at 100 iterations polling a handler-incremented
counter, and asserts exactly one interrupt lands with `mcause == 0x8000000b`.
`nano/asm/csrimm.S` (new) is the zero-immediate suppression regression described
above. The suite is 8/8 on both the cxxrtl and iverilog legs, and both legs agree
program by program.

Portable subset run against nanocpu (`make nano-littlecpu-test`, `LITTLECPU_FLOOR`):
`csrset.S` and `hpm.S` are un-excluded now that nano has CSRs (76 and 127 retires,
measured directly). `contend.S` is newly excluded — its own reason is architectural,
not a regression: nanocpu's QSPI front end genuinely cannot route a load/store to the
text-resident data `contend.S` expects to reach, the same one-sided fast/fault split
littlecpu's own region wait documents for its shared bus. Every other program in the
portable subset is unaffected.

## Area

`make nano-area`: **76,982.6 um2** against the stepped `NANO_MAX_UM2` budget of
79,000 (`nano/nano.mk`), up from the QSPI-front-end baseline's 60,859.6. The CSR file,
traps, `mret` and MEIP together cost roughly 16,100 um2 — well past the brief's ~5k
estimate, which did not anticipate 64-bit `mcycle`/`minstret`, the full mandated CSR
address decode, or the RVFI fault-channel plumbing the region-fault work needed to
stay checkable at all. `nano-timing`'s own standing caveat applies unchanged: this is
a ranking instrument, not the brief's TT-flow number, and the two are never merged.

## Scope note

`nano/formal/traps-region-probe.py` and `traps-tval-probe.py` follow
`ill-e-probe.py`'s established shipping-plus-mutants pattern directly and are run for
real (both confirmed: shipping PASS, every mutant FAIL, for its own reason). They do
not carry `ill-e-probe.py`'s further self-test section (a `test/probe_gates.sh`-style
meta-test that plants a broken probe script itself and requires *that* to fail); this
is a deliberate, time-boxed cut rather than a gap nobody noticed, left for a follow-up
if the pattern is judged worth the extra machinery.
