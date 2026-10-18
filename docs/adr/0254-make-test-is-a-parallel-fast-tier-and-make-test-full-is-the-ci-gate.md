# ADR-0254: `make test` is a parallel, change-aware fast tier, and `make test-full` is the CI gate

**Status:** Accepted · 2026-10-18 · *Builds on ADR-0140 (the probe manifest). No `rtl/` change ships
from this ADR.*

## Context

`make test` ran about 70 prerequisites one after another, with no `-j`, and CI's `test` job took
23 minutes 29 seconds for the step (1,409 s). Locally it took 28 minutes on a lightly loaded 10-core machine
and about 67 under heavy load. It was meant to be the quick check before full CI and had become
the slowest thing a change waits on.

The owner decided, on 2026-10-18: `make test` finishes in under about five minutes on an idle
machine, in two tiers; and `make probe-gates` "feels like a CI thing", so the whole forced-red
sweep leaves the fast tier for the tier CI runs rather than a subset of it staying behind. The
fast tier keeps the actual checks.

## Measurements, before

One CI run on `main` (23 minutes 29 seconds for `make test`, a 4-CPU pod), with each
target's span read from the log's timestamps, and one local serial pass per target (10 cores, load
average 5 to 9 at the start, so the local figures carry that load):

| target | CI | local |
|---|---|---|
| `probe-gates` (1,446 probes, one process) | 11 m 41 s | 1,068 s |
| `nano-qspi-loop-test` (24 s of test, 14 forced-red mutants) | about 7 m 35 s | 328 s |
| `nano-qspi-pins-test` | about 1 m 30 s | 64 s |
| `nano-test` | about 55 s | 59 s |
| `sim` | 54 s | 49 s |
| the other 61 targets together | about 1 m | about 130 s |

Two targets are 80% of the wall time, and both are forced-red sweeps. The nano loop buffer's test is
24 seconds; the other 300 are the sweep that proves the test can fail. Nothing else in the list
takes more than a minute.

## Decision

**Two tiers.** `make test` is the fast tier and `make test-full` runs every check; CI's `test` job
runs `make test-full`, so the job name branch protection requires is unchanged and every probe still
runs on every PR. The forced-red sweeps are the full tier only: `probe-gates` and
`nano-qspi-loop-probe`, which is split from `nano-qspi-loop-test` into a `nano-qspi-loop-run` that
stays in the fast tier. The smaller `*-probe` prerequisites inside other targets (`nano-test`'s
seven, `nano-timer-test`'s, `mutation-probe`) stay where they are: each is under a minute, and
removing them would cut a check from its own red direction without buying a second of the five
minutes.

**One schedule.** `test/CHECKS` has a row per check (tier, lane, what it needs, the files it reads);
`test/checks.py` runs the rows, `TEST_JOBS` at a time (default: the CPU count), prints a failure's
log and a one-line pass, and ends with the slowest eight. The Makefile's `TEST_CHECKS` is the list
of what exists and `test/checks_manifest_test.py` holds the two equal in both directions, so a
check dropped from `make test-full` or added and never scheduled is red. `gen-inputs` and
`nano-sim` are build rows, not checks: they generate the files `sim` and the nano simulators read
once, so no two checks regenerate them at the same moment.

**`probe-gates` runs in shards.** `probe_gates.sh --parallel` starts `PROBE_SHARDS` processes of the
same script (default 8). Every shard runs every line of setup and only its own groups' probes: a
group belongs to the shard its position modulo the count names, and a group marked `spread`, whose
probes are slow and independent (the iCELink eject probes sleep 31 seconds each), deals its probes
round-robin. Each shard writes the labels it ran and `--merge` compares their union to
`test/PROBES_EXPECTED` under the same set equality, so a group no shard owned and a group two shards
ran are both a mismatch.

**Change-aware skipping, local only.** `make test` skips a check whose stamp, in
`build/test-state/stamps`, equals the hash of the files its row declares, its row text and the first
line of `--version` from yosys, iverilog, the RISC-V gcc, clang++ and python3. Three things send it
back to running everything: no green run of the tier yet; a changed file no row claims; a new
toolchain. A scanner that reads the whole repository is marked `@all` (keyed on every file, claiming
only the paths it names), and the build rows are `!always` because make's own timestamps already
decide them. `make test-full` never skips, which is also why CI, starting from a clean checkout,
never could.

**It is graded where it can fail.** `test/checks_test.py` runs the driver on a throwaway repository
and counts how often each check's command ran: a cold run, a warm one, an edited declared input, an
edited file under a directory pattern, a file no row claims, `--no-skip`, the full tier, a failing
check, a new tool version, lane exclusion, `needs` ordering and four refusals. Eight forced-red
probes in `probe_gates.sh` each break one rule of the driver and require that grader to name it.
`test/checks_manifest_test.py` also requires every tracked file a check's recipe names, and every
file those scripts name in their own text, to be claimed by that check's row, which is how a
declared input that is narrower than what the script reads goes red.

## Measurements, after

(Local figures: 10 cores, load average 5 to 14 throughout, so none is an idle-machine figure.)

| run | wall |
|---|---|
| `make test`, cold (69 checks) | 86 s |
| `make test`, nothing changed | 1.2 s |
| `make test`, one line of `rtl/executor.v` changed (50 of 69 ran) | 64 s |
| `make test-full`, cold (73 checks, after rebasing on `main`; `TEST_JOBS=4 PROBE_SHARDS=4 NANO_PROBE_JOBS=2`; load 3.5 at the start) | 573 s |
| **CI** `make test-full`, this PR's run (`TEST_JOBS=3 PROBE_SHARDS=3 NANO_PROBE_JOBS=1`) | 1,057 s against 1,409 s before |

The CI run shared its node with the PR's other twenty jobs, and `sim` took 134 s there against 54 s
in the baseline run: the pod is limited by what the node has free, not by its four-CPU ceiling, so
parallelism bought 25% on CI where the same suite went from 1,700 s serial to 573 s on the local machine. Its slowest checks were
`nano-qspi-loop-probe` (827 s), `probe-gates` (601 s), `nano-qspi-pins-test` (186 s) and `sim`
(134 s). The first two are the forced-red sweeps that were already the first two in the baseline;
they are cheaper to shorten (fewer mutants per run, or a smaller sweep) than to schedule better.

The cold fast tier's long poles are `nano-qspi-pins-test` (84 s), `sim` (71 s) and `nano-test`
(45 s), run side by side. In the one-line case `sim` (47 s) is the pole, and 13 of the 50 reruns
are the `@all` scanners at 1 to 3 seconds each.

## Consequences

- **The five-minute target is met on this machine with room**, but the machine was never idle, so
  the figure is an upper bound on an idle machine's.
- **A fast-tier pass no longer includes the forced-red sweep.** A change to a grader that
  `probe-gates` forces red is checked by `make test` only for the green direction; the red one is
  found by `make test-full`, which CI runs on every PR. `make probe-gates` is still one command.
- **Parallel safety is a property of each check.** The profile of every target's writes found no
  two sharing a path except through the build rows; a cold `make test-full` ran all 73 green. A new
  check keeps its scratch to `mktemp` or a path only it writes, and CLAUDE.md says so.
- **CI memory.** A cxxrtl build holds about 600 MB. The runner's configuration file says 3 GiB,
  but the pod's `memory.max` reads 6 GiB and the run's `memory.peak` was 2.6 GB with the settings
  above, which `.github/workflows/ci.yml` prints after the step. They stay below the CPU count
  because CPU on a shared node, not memory, set the wall time.
- **The suite's python launcher leaks `PYTHONHOME`.** The OSS CAD Suite's `python3` exports it for
  its own interpreter, and a check that runs a different `python3` under a minimal `PATH` failed
  to start under it; the driver drops `PYTHONHOME` and `PYTHONEXECUTABLE` from every child.

## Declined

- **A subset of the probes in the fast tier.** The owner's direction was the whole sweep, and a
  subset needs a second manifest to say which.
- **Keying every check on a hand-written file list.** The twenty-odd scanners that read the whole
  repository and finish in a second or two are keyed on everything (`@all`); a precise list for
  them is a chance to be wrong in exchange for a second saved.
