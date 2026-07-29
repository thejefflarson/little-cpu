# ADR-0022: The formal nightly reports against an explicit baseline, not `|| true`

**Status:** Accepted · 2026-07-29 · *Supplements ADR-0006, ADR-0013, ADR-0014 · Follow-up required*

## Context

`.github/workflows/formal-nightly.yml` landed with the ladder step written as:

```yaml
- name: riscv-formal ladder (make -C formal check)
  run: make -C formal check || true
```

The stated reason is sound as far as it goes: the ladder has known failures that are not
regressions (CSRs are M3; see ADR-0011 and ADR-0021), so a non-zero exit is expected and the job
should not go red for them. This is a reporting job, not a gate — there is no required-status-check
entry for this workflow, and there must not be one while known failures exist.

Two problems, both discovered by re-running the ladder at integration rather than by reading:

1. **`|| true` makes 15 failures and 30 failures look identical.** The step summary lists every
   check's status, but nothing compares that list to anything. A reader has to know the expected set
   by heart to notice a new failure. That is precisely the hole ADR-0014 closed for the `.S` suite
   with `test/EXPECTED_FAIL` and a **set-equality** check in both directions — a newly-*passing*
   check is as much a signal as a newly-failing one, and neither is visible today.

2. **The ladder does not actually finish.** `formal/Makefile`'s `check` target runs
   `$(MAKE) -BC checks -j$(JOBS)` with no `-k`. GNU make stops scheduling new jobs at the first
   failure. With the current 17 failures out of 78 checks, a nightly run completes roughly the
   first parallel wave and abandons the rest — and `|| true` then reports that partial run as
   success. The summary would show a majority of checks as "no status", which reads like an
   infrastructure hiccup rather than the design working as written. Confirmed at integration: the
   full sweep only ran here because `-k` was passed by hand.

There is a third, smaller version of the same failure to bound work: `reg_ch0.sby` carries no
`timeout`, and `reg` does not terminate in any practical budget (ADR-0023). It will sit in the
solver until `timeout-minutes: 300` kills the whole job, taking the `complete` and `equiv.sh` steps
with it.

## Decision

**Report-only is the right posture; `|| true` is the wrong mechanism.** The nightly keeps its
non-gating role, and gains an explicit baseline.

Concretely, the follow-up work is:

- Add `-k` to `formal/Makefile`'s `check` recipe so the whole ladder runs. A ladder that stops at
  the first failure cannot produce a baseline to compare against.
- Add a tracked `formal/EXPECTED_FAIL` listing the checks known to fail, each with the reason —
  applying ADR-0014's pattern, including its set-equality property so an unexpected *pass* also
  trips. Today's contents, verified at integration:
  - `csrw_mcycle`, `csrw_minstret` — CSRs are M3.
  - `insn_lh`, `insn_lhu`, `insn_lw`, `insn_sh`, `insn_sw`, `insn_c_lw`, `insn_c_lwsp`,
    `insn_c_sw`, `insn_c_swsp` — all fail on `assert(spec_trap == trap)`; `rvfi_trap` is hardwired
    0 until misalignment traps land (M3, ADR-0011). Every byte-granularity access passes, which is
    what makes this attribution checkable rather than asserted.
  - `insn_c_jr`, `insn_c_jalr` — the real decode defect, ADR-0021.
  - `insn_div`, `insn_divu`, `insn_rem`, `insn_remu` — the ALTOPS operand-latching defect,
    ADR-0023.
  - `reg` — inconclusive, ADR-0023.
- Replace `|| true` with the baseline comparison as the step's exit status.
- Give `reg` a bounded `timeout` in `checks.cfg` so an unsolvable check is *reported* as
  inconclusive rather than eating the job budget.
- The `complete` and `equiv.sh` steps are **not** `|| true` today and will therefore fail the job
  every night for reasons ADR-0023 documents as expected. Decide per step whether it belongs in the
  baseline or should be allowed to report; do not paper over either with `|| true`.

**This did not hold the ladder port.** The port is the most valuable change in the project's
history and the nightly is a new, non-required, trivially-revertible reporting job. Blocking the
former on the latter would have been the wrong trade. Recording it is the price of merging it.

## Consequences

- Until the baseline lands, **the nightly's green is not evidence of anything** and its summary must
  be read by a human against this ADR's list. Do not cite a green nightly as a result.
- No status check from this workflow may be added to branch protection while the expected-failure
  set is non-empty. The gate is `ci.yml`; this is a report.
- Every entry in the baseline is a debt with a named owner ADR. The file shrinking to empty is the
  M2 completion signal, and is a better one than any single check passing.
