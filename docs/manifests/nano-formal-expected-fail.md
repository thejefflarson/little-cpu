# `nano/formal/EXPECTED_FAIL`

Nanocpu's riscv-formal regression baseline. Same format and grading as
`formal/EXPECTED_FAIL` (see `docs/manifests/formal-expected-fail.md` for the
two-field format and the vocabulary `check-baseline.sh` accepts) — a
name-and-status pair, graded both ways by `../../formal/check-baseline.sh`, so
an unexpected PASS is as red as an unexpected FAIL.

## Empty

`csrw_mcycle_ch0` and `csrw_minstret_ch0` were the only two entries, both red
because `nano.v` had no CSR read/write mechanism at all — every CSR
instruction traded as illegal, so the generated `csrw_<csr>_ch0` check's
`assert(!rvfi.trap)` failed at the shallowest depth tried. `nano.v` now
implements the mandatory M-mode CSR set, `mcycle`/`minstret` writable among
it, so both checks pass and this file states no exceptions. It stays tracked,
empty but for its own header, because `check-baseline.sh` reads it
unconditionally.
