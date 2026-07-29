# ADR-0013: The riscv-formal pin is an enforced control, not a comment

**Status:** Accepted · 2026-07-27 · *Implements ADR-0006's pinning clause*

## Context

ADR-0006 called the unpinned `git clone` of riscv-formal "a version-skew time bomb" and required a
SHA pin. `bcbf88b` added one. Reviewing that PR showed the pin **did not work**, in a way worth
recording so it is not reintroduced.

The clone target was a *directory*, and the checkout was a separate recipe line. Make does not
delete a target on recipe failure unless `.DELETE_ON_ERROR:` is set — and even then it does not
remove directories. So:

1. If `git clone` succeeded and `git checkout` failed, the directory survived. Make then considered
   the target satisfied **forever**, and every subsequent build ran against unpinned upstream HEAD.
2. Bumping `RISCV_FORMAL_SHA` never re-checked-out an existing clone. The pin silently stopped
   applying at exactly the moment someone believed they had changed it.
3. Nothing ever compared the clone's actual `HEAD` to the pin, so none of this was observable.

This is not a cosmetic gap. This repo **executes Python straight out of that clone**
(`checks/rvfi_macros.py`, `monitor/generate.py`) and commits its stdout as tracked, compiled source
(`test/monitor.v`, 7008 lines, instantiated in both simulation legs). `formal/genchecks-local.py`
additionally turns third-party `isa_*.txt` lines into generated makefile recipes that `make -BC`
then executes. The pin is the only thing standing between an upstream repository and code execution
here. A control that reads as protection while providing none is worse than no control, because it
stops anyone from looking.

## Decision

**The pin is enforced mechanically, at both call sites, from one definition.**

- `formal/pin.mk` owns the clone recipe and the pin. Both the root `Makefile` and `formal/Makefile`
  include it. There is exactly one recipe, so the two cannot drift apart again — which is how they
  came to differ in the first place.
- The clone is **atomic**: clone to `$@.tmp`, `checkout --detach`, assert `rev-parse HEAD` equals
  the pin, and only then `mv` into place. A failure at any step leaves no directory behind, so make
  retries the whole thing rather than treating a half-built unpinned clone as done.
- A **parse-time guard** hard-fails both Makefiles if a clone already on disk is at anything other
  than the pin, naming the exact re-checkout command. `clean` is exempt, since that is the escape
  hatch. This covers the pre-existing-unpinned-clone and bumped-pin cases, which the atomic clone
  alone does not.
- `RISCV_FORMAL_SHA` is `override` (not settable from the command line) and must be 40 hex digits,
  so it cannot be redirected from a script or pointed at a moving branch or tag.

**Regenerating `test/monitor.v` when the pin moves is required, not optional.** `bcbf88b`'s
regeneration produced a 45-line delta, all of it upstream changing `rvfi_insn >> 32` to
`rvfi_insn >> 16 >> 16` — a real portability fix, since a 32-bit-wide shift by 32 is not
well-defined across frontends. Independently reproduced byte-for-byte from a fresh clone at the pin,
so it is an upstream change and not an artifact of how the generator was invoked. This is exactly
`CLAUDE.md` invariant 7 and ADR-0007 working as intended: regenerate, never hand-edit.

**`test/monitor.v` depends on `generate.py` and `pin.mk`, with the clone order-only.** Depending on
the clone *directory* would make a tracked file regenerate whenever anything is written anywhere
inside the clone — silently overwriting it during an ordinary build and defeating `monitor-check`.

## Rationale

Assume the dependency will betray you. The mitigation for executing third-party code is to keep the
version you execute pinned, verifiable, and hard to change by accident — not to trust that a
variable named `RISCV_FORMAL_SHA` is doing something.

Fail closed. Every mode above failed *open*, and each one failed open quietly. The guard is a few
lines of make and turns three silent conditions into one loud one with the fix in the message.

## Consequences

- Bumping the pin is now a two-step, visible operation: edit `formal/pin.mk`, then re-checkout (the
  guard tells you how) and regenerate `test/monitor.v`. That friction is the point.
- `formal/genchecks-local.py` is a fork of a **much older** upstream and diffs ~450 lines against
  the pinned SHA — it is missing upstream's `csr_spec`/`buslen`/`nbus`/`abspath` options and its
  isa-string parser. Only the `basedir` change is deliberate. The pin now stops that skew growing,
  but does not remove it; re-vendoring from the pin and re-applying only `basedir` is open work.
- The pin is the *only* thing gating `genchecks-local.py`'s isa-line-to-makefile-recipe path. That
  path should also be hardened on its own merits — it should not be the pin's job alone. Open work.
- CI (`c66527d`) should run `make monitor-check` and a from-scratch clone, which is what actually
  proves the guard keeps working. Until CI exists, nothing runs this automatically.
