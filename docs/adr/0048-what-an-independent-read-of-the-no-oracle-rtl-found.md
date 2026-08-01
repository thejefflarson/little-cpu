# ADR-0048: What an independent read of the no-oracle RTL found

**Status:** Accepted · 2026-08-01 · *Supplements
[ADR-0005](0005-traps-and-csrs-commit-in-decode.md),
[ADR-0027](0027-minstret-counts-non-trapping-issues.md),
[ADR-0030](0030-trap-cause-priority-and-why-the-causes-are-disjoint.md). Records one RTL defect,
one spec-scope divergence left open, and the audit method that found them.*

## Context

riscv-formal ships no spec model for `ecall`/`ebreak`/`mret`/`csrr*` at the pinned SHA. `spec_valid`
is 0 for every one of them, so the per-retire monitor's whole semantic block is skipped and every
`insn_*` ladder check is silent about the behaviour M3 added. What is left is `test/csr_tb.v`,
`test/asm/trap.S`, `test/asm/csr.S` and `test/decoder_tb.v` — **assertions this repo wrote**.

CLAUDE.md's own summary of the situation is *"an empty baseline is loudest exactly where the ladder
is quietest."* The specific hazard is **correlated-author error**: a bench written from the same
reading of the spec as the RTL cannot detect a misreading, because the misreading is in both. The
number of assertions is irrelevant to that; they are not independent samples.

## The method, and why the ordering is the whole thing

`rtl/csrs.v` (in full), `rtl/decoder.v`'s trap arm and `rtl/writeback.v` were read against the
privileged specification **with `test/csr_tb.v`, `test/asm/trap.S` and `test/asm/csr.S` closed**.
An expectation was written down first — WARL mask by WARL mask, cause by cause, decode arm by decode
arm — and only then were the benches opened and compared. A disagreement counts as a finding
regardless of which side turns out to be wrong.

Read the benches first and you inherit their reading. That is not a stylistic preference; it is the
only thing that makes the exercise able to fail.

## Finding 1 — `c.ebreak` raised the wrong trap cause. RTL was wrong

**`c.ebreak` (16'h9002) raised illegal instruction (mcause 2) instead of breakpoint (mcause 3).**

The C extension defines C.EBREAK as expanding to EBREAK, so it raises the same exception with the
same cause. `rtl/decoder.v`'s `instr_error` requires `uncompressed`, and no compressed decode
covered the encoding: it is the `rd == 0`, `rs2 == 0` corner of quadrant 2's `funct4 == 4'b1001`
row, and its two neighbours each exclude it by a different field — `instr_cjalr` needs
`instr[11:7] != 0`, `instr_cadd` needs `instr[6:2] != 0`. With no decode at all, `instr_valid` was
low and `instr_illegal` produced cause 2.

**The failure mode is why it survived.** The instruction still traps, still records the correct
`mepc`, and the handler still resumes — only the cause value is wrong, and a wrong-but-plausible
answer is exactly what nothing in this repo was positioned to notice. `test/asm/trap.S`'s breakpoint
case is `.option norvc`, as is every trapping instruction in that file, because the handler resumes
at `mepc+4` unconditionally (`riscv_test.h` constraint 1, ADR-0008's Harvard buses). So the
assembler never had a reason to emit C.EBREAK anywhere in the suite, and `test/decoder_tb.v` drives
the 32-bit form. **The compressed breakpoint was the one trap cause with an encoding no test could
reach.**

Fixed here, in two lines, with `test/asm/cebreak.S` written and seen to fail first
(`cebreak.S FAIL 2`) at both alignments.

## Finding 2 — a counter write at the carry boundary moved the other half. RTL was wrong

"Any CSR write takes precedence over the automatic increment" (privileged spec 20211203 §3.1.11,
and ADR-0005's own wording) is a statement about the 64-bit counter, not about the half the address
happens to name: `mcycle` and `mcycleh` are two views of one register.

`rtl/csrs.v` suppressed the increment **per half**. With `mcycle == 32'hffff_ffff`, the increment's
carry landed in the high half while an explicit write replaced the low half — so `csrw mcycle`
advanced `mcycleh` by one. Same shape in `minstret`. That contradicts the file's own comment, which
states the invariant it was built to satisfy.

**Three oracles are structurally blind to it, and that is the part worth keeping.**
`checks/rvfi_csrw_check.sv` reads only the self-reported `rmask`/`wmask`/`rdata`/`wdata` and never
observes the register, so `csrw_mcycle_ch0` cannot see it in any configuration; every ladder check
is `mode bmc` from reset and could not reach 2^32 cycles if it could; and a `.S` program can land a
write on that one cycle only by calibrating the instruction spacing first, which is a test that
stops testing the moment the spacing changes. **A test that can silently stop testing is worse than
no test** — this repo's own `ref_selftest` doctrine. Driving the CSR port directly in
`test/csr_tb.v` is the one place it is deterministic, and that is where the check lives.

## Finding 3 — two required CSRs are missing. Landed red, left open

**`mstatush` (0x310, §3.1.6.4, "For RV32 only, mstatush is a 32-bit read/write register") and
`mconfigptr` (0xF15, §3.1.4) are listed unconditionally in the privileged spec's machine-mode
register tables and are absent from `rtl/csrs.v`'s `implemented` set**, so an access to either
raises illegal instruction where the spec says it should read zero.

**The divergence is measured, not argued**, and that is what promotes it from "one reading of a
specification sentence" to a finding. sail-riscv 0.13.1 reads both and continues; this core traps on
both.

The same measurement retired two suspicions the read had raised, and they are recorded so nobody
re-opens them. **`mhpmcounter3` (0xB03), `mhpmcounter3h` (0xB83) and `mhpmevent3` (0x323)**: the
reference model raises illegal instruction on all three, exactly as this core does — there is
nothing to close. **`mcountinhibit` (0x320)**: the spec makes it explicitly optional ("if not
implemented, the implementation behaves as though the register were set to zero"), so declining it
is legal; Sail implementing it is Sail's choice. (`mcounteren` (0x306) is likewise not required
without S- or U-mode, and the unprivileged `cycle`/`instret` aliases belong to Zicntr, which
ADR-0002 does not claim.)

**This is not fixed here and must not be fixed casually.** ADR-0005 states its CSR set as *exact*,
and widening it is an ISA-scope decision that belongs to an amendment of ADR-0002/ADR-0005 with its
own reasoning about what "RV32IMC_Zicsr, machine mode only" commits to — not to the audit that
noticed. It is owed to a follow-up, and naming it here is what stops it depending on anyone's
memory (the shape ADR-0041 decision 2 used for the co-simulation nightly).

`test/asm/csrset.S` lands it **red in both baselines** — `csrset.S FAIL 2` in `test/EXPECTED_FAIL`
and `csrset.S DISAGREE AT 2` in `test/COSIM_EXPECTED_FAIL`. That is ADR-0033's rule applied to the
`.S` suite: a known-red test in the suite is the system working, and a known-red property with no
test at all is the system lying. Because the status pins the first failing case (ADR-0035), fixing
`mstatush` without `mconfigptr` turns the entry into `FAIL 3` and goes red rather than matching —
the burn-down catching a partial fix rather than absorbing it.

## Everything else read as correct

Recorded because a negative result from an independent read is the point of doing one, and because
the next person should not have to redo it:

- **WARL, field by field.** `mstatus` (MPP hardwired 11, MIE/MPIE only), `mtvec` (MODE forced to
  direct), `mepc` (bit 0 only — bit 1 must survive, because C makes 2-byte targets legal),
  `mcause` (WLRL, fully writable), `mtval`/`mie`/`mip` (hardwired zero, writes ignored rather than
  faulting, because their addresses are writable), `misa` (WARL, unsupported write leaves it
  unchanged), the four identification CSRs (read-only *by address*, so a write faults).
- **Trap cause priority** matches ADR-0030, and its disjointness argument survives the `c.ebreak`
  fix: `instr_cebreak` feeds `instr_ebreak`, which the `$onehot0` assertion already covers.
- **`mepc` bit 0**, on both the explicit-write path and the trap-entry path, through one mask.
- **`mstatus` MIE/MPIE/MPP** on trap entry (MPIE←MIE, MIE←0, MPP←M) and on `mret` (MIE←MPIE,
  MPIE←1, MPP←least-privileged supported = M).
- **Counter increment semantics** against ADR-0027: a trapping instruction does not count; a CSR
  write beats the increment; `csrr minstret` excludes itself.
- **The `mret`/`ecall`/`ebreak` decode arms**, including that SYSTEM funct3 == 0 with a non-zero
  rs1 or rd is illegal, and that `sret` is.
- **Zicsr's suppression rules are encoding tests, not value tests**, in all six instructions — the
  sharp case being a CSRRS whose rs1 is a real register that happens to hold zero, which must still
  fault against a read-only CSR where the same instruction written with x0 must not.
- **`rtl/writeback.v` holds no architectural decision at all.** `wen = in.valid && in.rd != 0`, and
  everything else in the file is `ifdef RISCV_FORMAL` reporting.

**`test/csr_tb.v` disagreed with the specification nowhere.** Every assertion in it that the
independent read reached an expectation about matched that expectation. That is a real result and
the ticket's acceptance criterion 5 asked for it either way; it is recorded so that "the bench was
never checked" stops being true.

## Consequences

- `test/asm/cebreak.S`, `test/asm/zicsr.S` and `test/asm/csrset.S` join the suite; it is 55
  programs, graded 54 pass / 1 expected-fail by `make test` and 54 agree / 1 expected-divergence by
  `make cosim-suite`. **Both baselines stop being empty**, for the one entry above and for no other
  reason. The formal ladder is untouched at 82 checks / 82 pass with `formal/EXPECTED_FAIL` still
  empty, re-run against the `c.ebreak` fix rather than argued about.
- **The suite's `.option norvc` discipline has a blind spot, and it is now named.** Wrapping every
  trapping instruction keeps the resuming handler correct (ADR-0008) and simultaneously guarantees
  the assembler never emits a *compressed* trapping encoding. `c.ebreak` is the only such encoding
  this core implements, so the hole is closed rather than open — but any future compressed
  instruction that can fault inherits it, and closing it means padding with a `c.nop` so `mepc+4`
  lands on a boundary, which is what `cebreak.S` does.
- Finding 3 is owed to a future ADR, not dropped. Naming it here is what stops it depending on
  anyone's memory (the shape ADR-0041 decision 2 used for the co-simulation nightly).
- The method generalises and is cheap: read the RTL against the spec with the bench closed, write
  the expectation down, then compare. Two of the three files came back clean; the one defect it
  found was in the third and was invisible to all four existing oracles at once.
