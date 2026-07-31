# ADR-0033: What a green ladder does not cover — three assurance gaps, named

**Status:** Accepted · 2026-07-30 · *Recorded at integration. Constrains ADR-0022's M2 signal;
amends ADR-0031's `timeout` consequence; scopes ADR-0032's opt-in fetch.*

## Context

`formal/EXPECTED_FAIL` reaching empty is the signal CLAUDE.md declares for M2 — the milestone that
erases the verified→unverified regression, and the one the whole project is aimed at. A signal that
load-bearing is worth auditing as machinery in its own right, separately from the properties it
reports on.

Three gaps turned up while integrating the trap harness, the `genchecks` re-vendor and the Sail
co-simulation spike. **None is exploitable** — this repo has no service, no secrets, no user data,
and read-only CI — so none of them held a change. All three are the same shape: *a check that can
stop checking without anything going red*. That is the shape this project cares about, so they are
recorded here rather than left in a review thread.

## Decision

### 1. The `[depth]` table is an undeclared coverage gate, and the count must become an assertion

`formal/checks.cfg`'s `[depth]` section reads as a tuning table. It is not: it is the list of checks
that exist. `genchecks-local.py`'s `check_cons` and `insn_cons` both do

```python
    if depth_cfg is None: return
```

so a check with no matching depth line is **not generated at all** — no `.sby`, no directory, no
status file, no mention anywhere. Counted against upstream's own call sites at the pin, **fourteen
of them are dropped today**: `causal_mem`, `causal_io`, `ill`, `hang`, `fault`, and the nine
`bus_*`. `formal/checks.cfg` documents ten of those (`fault` and the nine `bus_*`) and concludes
"0 of 10 are applicable as the core stands". That reasoning is sound for the ten it covers. It does
not cover the other four, and **`ill` is applicable**: `checks/rvfi_ill_check.sv` reads only the
ordinary RVFI ports this core already drives, assumes `insn == 0`, and asserts `trap`, `rd_addr ==
0`, `rd_wdata == 0`, `mem_wmask == 0`. Today `rvfi_trap` is hardwired to zero, so `ill_ch0` would
**fail** — it is a known-red property kept off the ladder and out of the baseline, which is exactly
what `formal/EXPECTED_FAIL` exists to prevent (ADR-0022, and ADR-0014's contract before it).

Nothing downstream can notice. `formal/check-baseline.sh` enumerates `"$CHECKS_DIR"/*/` — directories
`sby` creates only for checks that actually ran. A never-generated check is absent from the results
*and* absent from `EXPECTED_FAIL`, so set equality reports a clean match. The script prints `total`
but never compares it to anything; the number 78 lives only in prose (CLAUDE.md, ADR-0023, ADR-0031).
And every generated `.sby` carries `expect pass,fail`, so `sby`'s exit code carries no verdict either.

**Decision: the expected check count becomes a mechanical assertion, and every upstream check
either has a depth line or an explicit, reasoned omission recorded next to `[depth]`.** Concretely:
`genchecks-local.py`'s final `Generated N checks.` is compared against a number the repo commits to,
and `check-baseline.sh` gains the same check against the directory count, so "the ladder shrank"
fails as loudly as "a check regressed". The four undocumented omissions get the treatment
`formal/checks.cfg` already gives the other ten — with `ill` expected to end up **on** the ladder
and in `EXPECTED_FAIL` until M3 lands traps, not off it.

**Consequence for the milestone, stated plainly: until that assertion exists,
`formal/EXPECTED_FAIL` reaching empty does not by itself mean M2.** It means the checks that were
generated agree with the baseline. Read it with the generated count beside it.

### 2. One pin bump can silence both oracles at once, and the sanitizer must say what it swallowed

`test/sanitize_monitor.py` is the oracle both sim legs read (ADR-0019). Its third rule wraps a
~45-line span in `if (!spec_trap)`, selecting that span by first anchor (the `rs1_addr` comparison)
and last anchor (the `mem_addr` comparison) with a non-greedy `.*?` across it. It asserts that the
rule matched **once** — which is a real and welcome control, and catches the failure mode where a
rule silently stops matching — but it does not assert *what the span contained*.

The partition matters. Upstream's `checks/rvfi_insn_check.sv` keeps `assert(spec_trap == trap)`
**outside** the gate; everything else goes inside. If a future generator relocates the
`handle_error(101, "mismatch in trap")` comparison into the anchored span, the sanitizer would wrap
the trap comparison in the very guard it exists to add, the per-retire oracle would stop checking
trapping retires in **both** sim legs, the site count would still be 1, and every gate would stay
green. `test/monitor_tb.v`'s negative control drives a wrong *non-trapping* retire, so it would not
catch this either.

The same pin bump regenerates `test/monitor.v` and is the moment `formal/genchecks-local.py` needs
re-syncing (ADR-0031) — and that re-sync has no mechanical drift check, only a hand-run `cp`+`diff`
recipe in the script's header. So one bump touches the simulation oracle and the formal ladder's
generator together.

**Decision: the trap-gate rule must assert its span's contents, not just its arity** — minimally,
that the matched text contains no `handle_error(101`, which is the one comparison that must stay
outside the gate. `test/monitor_tb.v` gains a wrong **trapping** retire as a second negative
control, so "the gate swallowed the trap check" is a bench failure rather than an argument. And the
`genchecks` re-sync gets a mechanical diff check rather than a documented habit. A pin bump is
already an ADR-0013 event; these make it a *checked* one.

### 3. `reg_ch0`'s per-check bound is restored by wrapping `sby`, not by re-forking `genchecks`

ADR-0031 removed `reg_ch0`'s `timeout 1800` and recorded the loss honestly: the `timeout=` parameter
rode this repo's stale fork, upstream has no equivalent, and keeping it means keeping a fork — which
is the exact debt the re-vendor paid down. The condition it guarded is also gone, since ADR-0024's
engine switch takes that check from non-convergence to seconds.

What remains is that `.github/workflows/formal-nightly.yml`'s `timeout-minutes` is a **shared**
budget: one hanging check now starves every check scheduled after it, and `-k` does not help,
because the job dies rather than the check. That is a real reduction, and the ladder is precisely
the machinery M2 is defined against.

**Decision: restore a per-check wall-clock bound by wrapping the `sby` invocations that
`formal/Makefile`'s `check` target drives. Do not re-fork `genchecks`.** The property ADR-0031
bought — "differs from upstream by only `basedir`", verifiable with one `diff` — is worth more than
a config knob, and re-forking to regain the knob would spend it immediately.

**Not implemented in this change, deliberately.** Three things make it bigger than it looks, and
each belongs in the follow-up rather than in an integration commit: the generated `checks/makefile`
hardcodes `sby <check>.sby` with no variable to override, so the wrapper has to post-process it;
`timeout(1)` is not on macOS by default (`gtimeout` from coreutils is), so it needs a portability
shim or it breaks local runs; and because every `.sby` carries `expect pass,fail`, a killed check
must be surfaced through its `status` file to reach `check-baseline.sh` at all — a wrapper that only
kills the process makes a hang look like a `NO-STATUS`, which is the right verdict but for an
unexamined reason. It also wants a full-ladder run to validate, and the ladder is not in the PR
gate — it is nightly (ADR-0022). Landing it blind on a green PR check would be theatre.

### 4. The Sail fetch's weak pin is bounded by scope, and that scope is the precondition

`make sail-setup` pipes a GitHub release tarball into `tar xz` with no checksum and no signature,
then executes the unpacked binary immediately and on every `cosim-run`. Release assets are mutable,
so `SAIL_RISCV_VERSION` pins a name, not bytes. This repo demonstrably knows better: `formal/pin.mk`
pins a 40-hex SHA with `override` so it cannot be defeated from the command line, clones atomically
via `.tmp`, verifies `HEAD` after checkout and fails closed; `ci.yml` SHA256-verifies the
oss-cad-suite tarball.

**Decision: accepted as-is while co-simulation stays opt-in, with the precondition recorded at the
point a reader would need it.** Nothing on `make test`'s path and no CI workflow reaches the target;
it runs only when a maintainer types `make sail-setup`. **Wiring co-simulation into `make test` or
CI — item (3) of ADR-0032's integration list — requires giving this fetch `formal/pin.mk`'s
treatment first.** The Makefile comment that justified the weak pin with "no code is executed out of
the tarball at build time", six lines above a recipe that executes it, was corrected when ADR-0032
merged; a prose-only guard contradicted by its own code is worse than no guard, because it is read
as one.

Two smaller notes on that target, for whoever picks it up: bumping `SAIL_RISCV_VERSION` does **not**
re-fetch, because the rule keys on the binary existing — contradicting the `clean` target's own
comment, which offers a bump as the way to re-fetch. And `tools/sail` is gitignored, so a stale or
substituted tree never shows up in `git status`.

## Consequences

- **`formal/EXPECTED_FAIL` empty is a necessary, not sufficient, M2 signal** until gap 1's count
  assertion exists. ADR-0023's list of what M2 needs gains one entry that is about the ladder rather
  than about the core.
- `ill_ch0` is expected to join the ladder **red**, and to leave `formal/EXPECTED_FAIL` when M3
  lands trap commit in decode (invariant 2, ADR-0030). A known-red check on the ladder is the
  system working; a known-red check off the ladder is the system lying.
- The three follow-ups above are filed separately. None of them blocked a merge, and none of them
  should: they are gaps in *assurance*, in a repo whose entire subject is assurance, which is why
  they are written down here instead of being fixed quietly or forgotten quietly.
- One more thing to check before co-simulation is worth anything under M3, found while reading
  `test/sail/memory-map.json`: it declares `misaligned_exceptions.load_store: None` for the
  MainMemory region, i.e. the reference model **completes** misaligned accesses. The core does too
  today, so the spike's 49/49 agreement is real. The moment M3 lands ADR-0030's causes 4 and 6, the
  model and the core will disagree on every misaligned access *by configuration* — false
  divergences, not findings. That config line is part of the trap work, not a co-simulation detail.
