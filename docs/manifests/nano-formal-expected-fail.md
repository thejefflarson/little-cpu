# `nano/formal/EXPECTED_FAIL`

Nanocpu's riscv-formal regression baseline. Same format and grading as
`formal/EXPECTED_FAIL` (see `docs/manifests/formal-expected-fail.md` for the
two-field format and the vocabulary `check-baseline.sh` accepts) — a
name-and-status pair, graded both ways by `../../formal/check-baseline.sh`, so
an unexpected PASS is as red as an unexpected FAIL. This file stays terse on
purpose, the same reason `test/COSIM_EXPECTED_FAIL` does: the baseline names
what is red, and the *why* lives here so a two-line reason does not blow the
5% comment budget `test/comment_density_test.py` grades the baseline file
against.

## `csrw_mcycle_ch0` and `csrw_minstret_ch0`

Both were enabled in the donor commit and have never passed (ADR-0167). The
mechanism is identical for both, and it is a property of `nano.v` as it
stands today, not a modelling gap in the check:

`checks/rvfi_csrw_check.sv` (the generated `csrw_<csr>_ch0` check) assumes the
retiring instruction at the check cycle is a CSRRW/CSRRS/CSRRC/CSRRWI/
CSRRSI/CSRRCI naming the CSR under test, and asserts `!rvfi.trap` whenever
the access is not `csr_illacc` — a machine-mode CSR read from machine mode is
never `csr_illacc`, and `nano.v` hard-codes `rvfi_mode <= 3`, so this
assumption holds for both `mcycle` (`0xB00`) and `minstret` (`0xB02`), both
machine read/write addresses. The assertion then requires the core not to
trap.

`nano.v`'s `is_valid` — the OR that decides whether an encoding is legal —
never lists `is_csrrw`/`is_csrrs`/`is_csrrc`/`is_csrrwi`/`is_csrrsi`/
`is_csrrci`, the flags it computes for exactly this opcode. Every CSR
instruction is therefore illegal on this core, not just one targeting
`mcycle` or `minstret`, and it traps at the first retire the check tries.
`assert(!rvfi.trap)` fails immediately and BMC finds the counterexample at
the shallowest depth tried — this is not a margin or a depth-tuning question.

This is the same absence the `#omit` lines for `csrc_inc_mcycle_ch0`,
`csrc_inc_minstret_ch0`, `csrc_upcnt_mcycle_ch0` and `csrc_upcnt_minstret_ch0`
already record next to `[depth]`: `nano.v` has no CSR read/write mechanism at
all — "no CSR entry mechanism ... beyond the `mcycle`/`minstret` counters
riscv-formal's own checks exercise" (ADR-0167) — not a narrower gap in these
two CSRs alone. Nothing in this tree grades whether a reshaped nanocpu ought
to make `mcycle`/`minstret` CSR-readable at all; that is bound up in which
CSR/trap layer the RV32E reshape adds (`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md`,
brief steps 3-5), an architectural question this baseline entry does not
settle and is not trying to. Until that reshape lands a CSR mechanism, these
two checks stay red for the reason stated above, and the reason is the entry.
