# ADR-0056: The Makefile-embedded ratchets get probes, and `soc-timing` gets a job

**Status:** Accepted · 2026-08-02 · *Extends [ADR-0053](0053-every-graded-comparison-carries-a-probe-of-its-red-direction.md)
to the four graded comparisons that live in the Makefile itself. Follows [ADR-0052](0052-m2-term-6-is-verified-and-the-fit-ratchet-gets-a-job.md)'s
job for `make soc-timing`.*

## Context

ADR-0053's audit reached `test/*.sh`, `test/*.py`, `formal/*.sh` and `formal/*.py`. It did not reach
the Makefile, and four graded, red-capable comparisons live there: `make fit`'s `FIT_MAX_LC` ratchet,
`make soc-timing`'s `SOC_MIN_MHZ` ratchet (inside `soc/timing_split.py`, which the Makefile calls but
`test/probe_gates.sh` never drove), the SPRAM/EBR census `soc.json`'s recipe runs, and
`check-unit-benches`' list equality. None had its red direction executed by anything. This is the
same defect class ADR-0053 named, one level up from where the audit looked.

Separately, `make soc-timing` was hand-run only. ADR-0054 built the SoC and took the first real
timing number; nothing kept a regression from landing unnoticed the way `make fit` did before
ADR-0052.

## Decision

**1. `soc_expect_cells` and `make fit`'s table/ratchet check are extracted into scripts.** Neither
was a target — one is a `define`, the other is inline `grep`/`sed`/`awk` in the `fit` recipe — so
neither could be driven against a fixture without either a real synthesis run or a second parser of
the same log. `soc/cell_census.py` replaces `soc_expect_cells`; `soc/fit_report.py` replaces the
`fit` recipe's table-presence check, utilisation dump and ratchet. Both are called from the Makefile
exactly where the inline logic used to sit, so there is one copy of each check — the same reason
`soc/timing_split.py` already carries `make soc-timing`'s ratchet instead of a `python3 -c` in the
recipe. Verified byte-for-byte against real `make fit` output before and after the extraction, and
against real `soc.synth.log` cell-table formatting.

**2. `check-unit-benches` needed no extraction.** It is already a `.PHONY` target with no side
effects, comparing `$(UNIT_BENCHES)`/`$(UNIT_BENCH_SRC_*)` against the real `test/*_tb.v` tree. Its
own comment says the declaration drives the recipe; the probes take that literally and invoke
`make -C $REPO check-unit-benches` with those variables overridden on the command line, against the
real tree, hermetically (no compiler, no simulator — the target itself uses only `mktemp`, `ls`,
`sort`, `comm`, `cmp`).

**3. `test/probe_gates.sh` gains four groups, fourteen probes, 108 → 122.** One control plus its red
directions per group, matching ADR-0053's discipline exactly:

- `soc/timing_split.py --min-mhz`: a report that clears the floor (control), one that misses it, one
  with no critical path (`reported_total is None`), one whose hop sum does not reconcile with the
  reported total.
- `soc/cell_census.py`: a matching count (control), a wrong count, and a cell type the log never
  mentions (`got=${got:-0}`).
- `check-unit-benches`: a bench present in `test/` but missing from `UNIT_BENCHES`, a declared bench
  with no file, and a declared bench with no `UNIT_BENCH_SRC_*` — the three branches the recipe
  already has.
- `soc/fit_report.py`: a measurement within budget (control), over budget, and no utilisation table.

`PROBES_EXPECTED` is bumped to the literal count, per the same rule that pins it in the first place.

**4. `make soc-timing` gets a CI job, mirroring `make fit`.** Same non-piped graded-command pattern
(a `run:` block is `bash -e {0}`, not pipefail — ADR-0037 §4), same non-required status: area and
timing are design constraints, not correctness ones (ADR-0036), and adding either to branch
protection is a human action taken deliberately or not at all. It publishes the census and the
logic/routing split to the step summary whether the step passes or fails, by writing the summary
before checking the captured exit status — the same shape `fit` and `nonperturbation` already use.

**5. The toolchain assertion is a composite action, not a second copy.** `soc-timing` is the first
job needing both the RISC-V cross compiler (to build a real ROM image) and the OSS CAD Suite.
Hand-copying the `test` job's inline assertion step would be the exact duplication shape that broke
the `elaborate` job once already: a workflow re-stating something another job (or the Makefile)
already owns, under a comment claiming it could not diverge. `.github/actions/verify-toolchain`
takes a `tools` input and is used by both `test` and `soc-timing`, so the tool list and the failure
message live once.

## Consequences

- `grep -c continue-on-error .github/workflows/*.yml` is unchanged by this ADR: no directive is
  added. A plain substring `grep` over `ci.yml` returns non-zero because of prose in the `formal`
  job's own comments describing continue-on-error's removal (ADR-0050, ADR-0052) — pre-existing,
  and not the YAML key.
- Nine CI jobs run on every PR now, not eight; the required set is unchanged (`elaborate`, `test`,
  `components`, `monitor-freshness`, `lint`, `formal`) and `soc-timing` joins `fit` and
  `nonperturbation` as non-required.
- `soc/fit_report.py` and `soc/cell_census.py` are load-bearing for correctness now, not just for
  `test/probe_gates.sh`'s coverage: `make fit` and `make soc-timing` both read them, so a change to
  either script changes the ratchet.
- This does not make `make fit` or `make soc-timing` grade anything they did not grade before —
  same checks, same thresholds, extracted rather than rewritten.
