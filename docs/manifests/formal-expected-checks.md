# `formal/EXPECTED_CHECKS`

The riscv-formal check set's EXPECTED SHAPE: every check `formal/checks.cfg`'s
`[depth]` table causes genchecks to generate, one per line.

## Why this file exists

`formal/EXPECTED_FAIL` answers "did any check's verdict move?" It cannot
answer "did a check stop existing?", and those are different questions with
the same green. genchecks' two call sites both end with
`if depth_cfg is None: return`, so deleting or mistyping one `[depth]` line
removes a check silently — no `.sby`, no directory, no status file, no
warning, exit 0. A never-generated check is then absent from the results AND
absent from `EXPECTED_FAIL` at once, so that file's set-equality reports a
clean match on a smaller set. The worst single line to lose is `reg`:
`reg_ch0` is the check that ties RVFI's self-report back to the actual
register file.

TWO mechanisms read this file, and they catch different things:

- `formal/genchecks-audit.py` set-equalities the GENERATED set against it at
  generation time, so a lost depth line fails in a second rather than after
  a three-minute run.
- `formal/check-baseline.sh` set-equalities the `.sby` files on disk against
  it after the run, and treats a name here with no status file as non-PASS.
  That is what makes "sby never scheduled it" and "its directory vanished"
  both count as failures rather than as a quiet, matching baseline.

Set equality in BOTH directions: an unexpected ADDITION fails this as loudly
as a disappearance. That is the case worth having after a pin bump — upstream
growing a check nobody here has ruled on should stop the gate, not join the
set unexamined.

Edit by hand, never regenerate from a run; regenerating launders exactly the
regression this file exists to catch. Adding a line is a claim that the same
commit adds a `[depth]` entry and (if it is red) a `formal/EXPECTED_FAIL`
entry with a reason. Removing one is a claim that the corresponding check
should no longer exist, which needs a `#omit` line in `checks.cfg` saying
why.

Nothing here is a verdict. Every check in this set is `mode bmc`, so a PASS
means "no counterexample found within the check's configured depth", not
that the property holds; and everything runs under `RISCV_FORMAL_ALTOPS`, so
`insn_mul`/`insn_div`/`insn_rem` passing says nothing whatever about the real
multiplier or divider. This file asserts only that the checks EXIST.

Every name here is generated; every check genchecks considered and dropped
has a `#omit` line in `checks.cfg`, and `formal/genchecks-audit.py` grades
both sets both ways.
