# ADR-0036: Three gate-hardening decisions ratified at integration, and a correction to ADR-0031

**Status:** Accepted · 2026-07-31 · *Recorded at integration. Corrects ADR-0031's `csrc_*`
framing; scopes the `case` family disabled in `.svlint.toml`; notes a gap in ADR-0022's
enforcement.*

## Context

Three changes landed together because all three are about the same thing: the gates that decide
whether work merges. `0355193` added a structural lint gate, `b585b17` hardened the simulation
gate so an assembler or `objcopy` failure cannot report PASS (ADR-0035), and `961e359` made the
formal ladder assert its own shape rather than only its verdicts (ADR-0033's gap 1).

Each carried a judgement the ticket that asked for it did not anticipate, and two of them
contradicted a claim written down elsewhere in this repo. A wrong claim left standing in an ADR
is worse than one never written, because the next reader has no way to tell it apart from a
measured one. So the corrections are recorded here rather than only in the reviews.

## Decision

### 1. `hang` belongs on the ladder. `liveness` does not subsume it.

The ticket asked for `hang` to be declined as "subsumed by the passing `liveness`". **That is
false**, and the two models say so directly at the pin (`c992aa6`).

`checks/rvfi_liveness_check.sv` opens its trigger cycle with

```verilog
if (trig) begin
    assume(rvfi_valid[`RISCV_FORMAL_CHANNEL_IDX]);
```

so the entire property is conditioned on a retire existing at the trigger cycle. It bounds the gap
from one retire to the *next*, and it is **vacuous on a core that never retires at all**.
`checks/rvfi_hang_check.sv` carries no such assumption: `okay` starts at 0, is set only by
`rvfi_valid`, and is asserted at the check cycle. "A retire happened by cycle N" is a strictly
different — and strictly stronger, on the never-retires case — property than "retires are not more
than N apart".

`formal/cover.sby` exhibits retires, but a cover witness is not a proof of an assertion. No sound
omission reason exists, so `hang` is on the ladder. It passes in ~3s.

The general rule this instance confirms: **an omission whose stated reason evaporates on inspection
is not an omission, it is a missing check.** `formal/checks.cfg`'s `#omit` lines are load-bearing
precisely because each one must survive being read against the model it declines.

### 2. ADR-0031 is corrected on the `csrc_*` family

ADR-0031 says the counter-CSR checks "are now generatable and are still not on the ladder", and
that adding one is "a follow-up needing its own derived depth (ADR-0025) and `formal/EXPECTED_FAIL`
entry (ADR-0022)". **That framing is incomplete in a way that will mislead whoever acts on it.**

`[csrs]` holds bare names. `genchecks` calls `check_cons` with `csr_test=None` for each, which sets
`check_name = "csrc"`, and the `.sby` template emits `rvfi_@check@_check.sv` — that is,
`rvfi_csrc_check.sv`. **No such file exists at the pin.** The six real models are
`rvfi_csrc_{any,const,hpm,inc,upcnt,zero}_check.sv`, and they are reachable *only* through a
per-CSR **test list** in `[csrs]` (`minstret inc`, `mvendorid const`, …).

So adding a bare `csrc` depth line does not produce three checks that fail; it produces three
checks that **cannot elaborate**. What ADR-0031 got right is that the six models are present and
that `csrc_inc_minstret` is the one to reach for first (ADR-0027). What it got wrong is the
implication that a `[depth]` line is the missing piece. The `[csrs]` test list is.

This is the one omission in `formal/checks.cfg` that is a trap for a future reader rather than a
judgement call, and it is now declared there with that reason attached, alongside its three
`#omit` lines.

### 3. The `case` family stays disabled, and the `rtl/accessor.v` follow-up is declined

`.svlint.toml` disables `case_default`, `explicit_case_default` and `implicit_case_default`. The
ticket's stated reason — that they do not see the assign-defaults-before-`case` idiom — was wrong,
and the engineer corrected it with a measurement: on a two-module file differing only in whether
`y = 0;` precedes the `case`, **all three rules fire identically on both**. Only an explicit
`default:` arm satisfies them. They are "write a `default:` arm" rules, not latch detectors.
Re-verified here against svlint 0.9.5: four findings per module, same positions.

That correction stands. The decision it was offered in support of also stands, but **for a
different reason than either the ticket or `.svlint.toml` first gave**, and the reason is the
second measurement in the same change: yosys does *not* accept a defaultless `always_comb` `case`.
It errors

```
ERROR: Latch inferred for signal `\m.\y' from always_comb process
```

and exits 1 — re-verified here under Yosys 0.67. The `elaborate` job already runs `proc; opt_clean;
check`, and the `test/rtl.cc` recipe already runs `write_cxxrtl`, so **the latch class is already a
hard gate in two places.**

Therefore the proposed one-line follow-up — turning `rtl/accessor.v`'s commented-out default into a
`default: ;` arm so `case_default` could be enabled — **is declined, not deferred.** Enabling the
rule would buy a style requirement, not a defect class: the only thing it catches that yosys does
not is a `case` that is already safe. Paying an `rtl/` edit for that, in the stage whose legibility
CLAUDE.md singles out, is the wrong trade. `.svlint.toml`'s comment on the rule is corrected in the
same commit as this ADR, because as written it claimed a value the same file's own measurement
refutes.

The engineer's ticket discipline in *not* making that `rtl/` edit was correct regardless. A lint
change that edits what the lint finds is not a lint change.

## Consequences

- `hang` is on the ladder and in `formal/EXPECTED_CHECKS`. If it ever goes red, the core stopped
  retiring; that is a different and louder failure than `liveness` going red.
- ADR-0031's `csrc_*` paragraph should be read together with this one. Nothing in the RTL or the
  ladder changes as a result — the family was not on the ladder before and is not now — but the
  *route* to putting it there is a `[csrs]` test list, not a `[depth]` line.
- `case_default` is closed as a question rather than left as a standing invitation. Reopening it
  needs a defect class yosys does not already error on.
- **`make lint` is a gate that nothing enforces.** The `lint` CI job runs on every PR, but
  `main`'s branch protection requires only `elaborate`, `test`, `components` and
  `monitor-freshness`. A red `lint` therefore does not block a merge today. That is a repository
  settings change, not a code change, and it is left for a human to make deliberately — the same
  care ADR-0018 took over which checks gate an automerge. Until it is made, `make lint` is
  advisory in CI and binding only by convention.

## A gap this integration surfaced but did not close

**`formal/EXPECTED_FAIL` records names only, and ADR-0035 has just shown why that is not enough.**
The simulation gate learned this lesson in `b585b17`: matching on the test name alone left the set
equality blind to *why* a test failed, so a broken harness and a broken test were both laundered
into a green gate, and `test/EXPECTED_FAIL` grew a status field. The formal gate has the identical
shape and did not.

This is not hypothetical. The post-merge verification run of `make -C formal check` on a machine
**without `btorsim`** produced:

```
82 checks: 72 pass, 10 fail
Failure list matches EXPECTED_FAIL exactly.
Generated check set matches EXPECTED_CHECKS exactly (82 checks).
```

exit 0 — while **all ten** non-PASS checks reported `ERROR 16 2`, not `FAIL`. `btormc` found the
counterexamples correctly (`ill_ch0`: `bad state property 0 reachable at bound k = 15
SATISFIABLE`); `sby` then failed to render the traces, because the step that does so shells out to
`btorsim`, which was not on `PATH`. `check-baseline.sh` buckets everything that is not `PASS`
together, so "a real counterexample at the configured depth" and "the trace renderer is missing"
are the same result to the gate.

The consequence that matters is the inverse case: if `btorsim` ever vanished from the nightly's
pinned OSS CAD Suite, every red check would flip `FAIL` → `ERROR`, the set equality would still
match exactly, and the ladder would stay green having stopped distinguishing a proof failure from a
tooling failure. That is the same shape as ADR-0033's gaps — *a check that can stop checking
without anything going red* — one level down, in the status field rather than the check list.

It is recorded and **not** closed here, because closing it is a change to the baseline format and
its script, which is its own ticket and wants the same treatment ADR-0035 gave the simulation side:
a second field pinning the status, with `ERROR` never a legitimate baselined value. `961e359` is
strictly an improvement over what preceded it and this gap predates it; nothing above is a reason
to hold that work.
