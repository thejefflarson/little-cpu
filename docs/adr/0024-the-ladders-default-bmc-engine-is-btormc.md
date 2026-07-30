# ADR-0024: The ladder's default BMC engine is `btor btormc`, not `smtbmc yices`

**Status:** Accepted · 2026-07-29 · *Closes one of ADR-0023's three named holes*

## Context

ADR-0023 named `reg_ch0` as the ladder's most important unresolved check: it ties RVFI's
self-report back to the actual register file, and without it the 55+ passing `insn_*` checks
establish only that the core's story about itself is spec-consistent, not that the story is true.
`reg_ch0` is a single BMC query at depth 21 (`skip 20`). Under `smtbmc yices` — `formal/checks.cfg`'s
only configurable solver, itself only reachable behind `smtbmc`, since `formal/genchecks-local.py`
gave `checks.cfg` no way to name a different **engine** at all — it did not return in over 20
minutes, independently reproduced twice. ADR-0022 gave it a 1800s bound so a non-converging run is
*reported* as `TIMEOUT` instead of consuming the whole nightly job budget, and `formal/EXPECTED_FAIL`
carried `reg_ch0` as a known-non-PASS entry on that basis.

`btor btormc` is a different BMC engine entirely — AIG/BDD-based bounded model checking via
`btormc`, not an SMT solver invoked incrementally through `smtbmc`. Run against the exact same
`reg_ch0.sv` and depth, it returns `PASS` in single-digit seconds.

**Full 78-check ladder, both engines, from-scratch, same machine:**

| | `smtbmc yices` (prior default) | `btor btormc` (new default) |
|---|---|---|
| generated | 78 | 78 |
| pass | 67 | 67 |
| non-pass | 11 (`formal/EXPECTED_FAIL`, exact match) | 11 (`formal/EXPECTED_FAIL`, exact match) |
| `reg_ch0` | inconclusive (`TIMEOUT` at 1800s, per prior integration runs) | **PASS** |

No check's verdict differs between the two engines other than `reg_ch0` — the two 11-entry non-pass
sets are identical, checked by `formal/check-baseline.sh` against the same `formal/EXPECTED_FAIL`.
That was checked deliberately, not assumed: switching the engine that answers a bounded query can in
principle also flip a query that was previously *wrong* rather than merely slow (an under-constrained
harness reads as vacuously fast under one engine and correctly bounded under another), and the
identical non-pass set is the evidence that didn't happen here.

**A caveat on reproducing the timing, in the interest of not repeating ADR-0023's own lesson.** The
`>20 minute` non-convergence for `reg_ch0` under `smtbmc yices` was measured, and independently
reproduced, against the pinned OSS CAD Suite build (`formal-nightly.yml`'s
`OSS_CAD_SUITE_TAG`) — the same environment the nightly and the architect's own re-verification use.
Re-measuring locally on a Homebrew-installed toolchain (`yosys 0.67+post`, `yices 2.6.4`, no OSS CAD
Suite) for this change, `smtbmc yices` also converged `reg_ch0` quickly (single-digit seconds) and
the full 78-check `smtbmc yices` sweep completed in under a minute — it did **not** reproduce the
non-convergence on that machine. SMT solvers are known to be highly version- and build-sensitive on
hard queries near a difficulty cliff; a Homebrew build and a pinned OSS CAD Suite build are not the
same yices binary. This local run does not retract the original finding — it doesn't have access to
the reference environment that produced it — but it means the magnitude of the win is confirmed on
the reference (OSS CAD Suite / nightly) environment and not independently reconfirmed on every
environment. `btor btormc`'s result, by contrast, was fast in every measurement taken, on both
environments.

## Decision

**`formal/checks.cfg` gains an `engine` option** (a verbatim `sby` `[engines]` line — e.g.
`btor btormc` or `smtbmc yices`) that sets the default engine for every check
`formal/genchecks-local.py` generates, and **a `[engine]` section** for a per-check override by
regex against the full check name (e.g. `reg_ch0 smtbmc yices`), checked first-match-wins. Both are
local-fork additions to `genchecks-local.py` — documented at the point they're read in the script's
own fork-provenance header, not only here — because upstream's `solver` option only ever selects an
SMT solver behind `smtbmc` (the two special-cased spellings `bmc3`/`btormc` are upstream's, and the
only way upstream lets a single string pick a different engine; nothing there lets one check differ
from the rest).

`checks.cfg` sets `engine btor btormc`. `solver yices` stays, as the `smtbmc` fallback if `engine` is
ever removed and as what a `[engine]` override naming `smtbmc` without a solver falls back to.
`reg_ch0`'s 1800s `timeout` (ADR-0022) stays too, as a safety net — a bounded engine switch does not
retire the need to bound a query that might not converge under some future check or future engine.

**`formal/EXPECTED_FAIL` drops `reg_ch0`.** It is no longer inconclusive; it is a `PASS`, like every
other check on this ladder, `mode bmc` — a statement that no counterexample exists within the
check's configured depth, not an unbounded proof. Switching engines changed how fast the answer
arrived, not what the answer means.

## Rationale

ADR-0023 asked for exactly this shape of decision if `reg` did not converge on its own: "a different
depth split, a different engine, or a different bound." A different engine is what this is, chosen
because it is the cheapest of the three (no depth or property change) and because the measurement
supports it without qualification on the reference environment.

Making the engine configurable rather than hardcoding `btor btormc` is what keeps `smtbmc yices`
reachable per ADR-0023's own caution against overfitting a bounded result to one tool: if a future
check needs `smtbmc`-specific machinery (`--dumpsmt2`, a solver quirk, an unsat core), the `[engine]`
override is a one-line `checks.cfg` change, not a `genchecks-local.py` edit.

## Consequences

- **One of ADR-0023's three named holes is closed.** `reg` is no longer inconclusive. The other two
  — ALTOPS never checking the real multiplier/divider (ADR-0010), and `equiv.sh` not converging
  (ADR-0020) — are untouched by this change and stay open.
- **M2 is still not reached.** `formal/EXPECTED_FAIL` is down to 11 entries (CSRs are M3; the
  misalignment-trap set is M3, ADR-0011) but is not empty, which is ADR-0023's own signal for when
  M2 closes.
- Every check on the ladder is still `mode bmc`. A green `reg_ch0` under `btor btormc` means "no
  counterexample within 21 cycles," exactly as it would have meant under `smtbmc yices` had that
  engine converged. Nothing about what a PASS asserts changed; only the tool that found it did.
- The measured gap between engines on this specific query, on the pinned OSS CAD Suite build, is
  worth reporting upstream to YosysHQ/riscv-formal as a data point about `smtbmc`'s solver-mode
  scaling versus `btormc` on this class of pipelined-core BMC query. Filing that is outward-facing
  and not this repo's call to make unilaterally; noted here so it isn't lost.
