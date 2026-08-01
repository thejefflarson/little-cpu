# ADR-0041: Integration decisions from the fit / co-simulation / negedge wave

**Status:** Accepted · 2026-08-01 · *Supplements ADR-0008, ADR-0032, ADR-0038, ADR-0039, ADR-0040;
records two things the three merged changes decided but did not write down*

## Context

Three changes landed together: `make fit` (ADR-0038's measurement), suite-wide Sail co-simulation
with a baseline (ADR-0039), and the negedge-regfile spike plus the `formal/Makefile` stale-run fix
(ADR-0040). Reviewing them raised five judgement calls. Three are already recorded where they
belong — the two baselined co-simulation divergences in `test/COSIM_EXPECTED_FAIL`'s header, the
`formal/wrapper.v` substitution seam in ADR-0040 decision 2, and `make fit`'s tolerance of a failed
placement in ADR-0038 decision 1a. Two are not recorded anywhere, and this ADR is where they go.

## Decision 1: the `tohost` doubleword changes 382 register *values* and zero register *events*

`test/asm/riscv_test.h` is included by every program in the suite, so ADR-0039's change to it is the
widest-blast-radius change in the wave. Its own comment says the sequence of architectural
register-file states "is unchanged by this pair" of stores. That is true and is the load-bearing
claim — but it is easy to read as the broader "nothing changed", which is false, and a future reader
diffing traces across that commit will find 51 of 52 programs differing and needs to know why before
they go looking for a bug.

Measured at integration by replaying all 52 programs through `test/cosim.cc`'s architectural-register
tracer on both sides of the change (`rtl/` is byte-identical across it, so one `cosim` binary serves
both and only the assembled images differ):

- **7076 register-change events before, 7076 after**, and **no program's event count differs.** The
  two added `sw`s contribute no architectural register change at all. The precise claim holds.
- **Every** field that does differ is a pure address relabeling: `.data` +4 (348 register values,
  200 PCs) and `.text` +8 (34 register values, 105 PCs). `.text` grows by exactly the two 4-byte
  stores; `.data` by the four bytes of `tohost` padding. **Zero differences outside those two
  deltas.**

Both sides of the co-simulation read the same ELF, so the relabeling is invisible to it by
construction. Recorded here as a number rather than an argument, because "one extra store, no
register writes" is the kind of claim that is cheap to assert and was worth ten minutes to measure.

## Decision 2: the co-simulation nightly is owed work, not a dropped item

ADR-0032's integration list had three items. ADR-0039 landed two — the `tohost` doubleword and the
`COSIM_EXPECTED_FAIL` baseline — and deliberately omitted the third, a nightly job, on the grounds
that ADR-0037's `tee`/pipefail trap makes an untestable workflow a poor thing to add blind. **That
reasoning is ratified.** A nightly whose graded step cannot go red is worse than no nightly: it is
ADR-0022's failure exactly, and this repo has already paid for it once.

It is **owed, not dropped**, and it is recorded here so that it stops depending on anyone's memory.
The preconditions, all of which are now met or nearly so:

1. The graded command must not sit in a pipeline in a `run:` block, and both failure directions must
   be demonstrated on real runs before the job is called done (ADR-0037's general rule).
2. It must grade against `test/COSIM_EXPECTED_FAIL` by the same set equality in both directions that
   `make cosim-suite` already applies, so an unexpected *agreement* is caught too.
3. It must not join the required check set. Putting co-simulation on branch protection is what
   ADR-0032 forbids and ADR-0039 restates; a nightly is the only shape this leg gets.

Until it exists, the way a change gates on this leg is by carrying `make cosim-suite`'s pre- and
post- output in its pull request. That is the mechanism the regfile change is expected to use.

## Consequences

- **The suite has a second baseline file to keep honest.** `test/COSIM_EXPECTED_FAIL` joins
  `test/EXPECTED_FAIL` and `formal/EXPECTED_FAIL`/`EXPECTED_CHECKS`. All four are set equalities in
  both directions (ADR-0014) over name-and-status pairs (ADR-0035); none is regenerated wholesale
  from a run. Three of the four are enforced by a required check. The co-simulation one is not, by
  decision 2 above, and that asymmetry is the price of the leg staying opt-in.
- **`make fit` is the fourth thing that is not a leg**, alongside co-simulation. It gates nothing and
  is not in CI. Its number moves whenever the RTL does, and nothing notices; ADR-0038 accepted that.
- **Nothing in this wave touched `rtl/`.** `git diff` over the three merges is 0 lines under `rtl/`
  and `test/monitor.v` is byte-identical, so invariants 1–8 are untouched by construction rather
  than by inspection. The one change to shared behaviour is `test/asm/riscv_test.h`, measured in
  decision 1.
