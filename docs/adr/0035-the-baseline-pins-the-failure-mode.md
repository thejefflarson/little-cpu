# ADR-0035: `test/EXPECTED_FAIL` pins the failure mode, not just the file name

**Status:** Accepted · 2026-07-31 · *Amends ADR-0014. Same shape as ADR-0033's "a check that can
stop checking without anything going red."*

## Context

`test/run_tests.sh` is the merge gate. `make test` calls it, CI's `test` job calls `make test`, and
its exit status is what decides whether a change lands. A gate that reports success without having
tested anything is therefore the failure mode that matters here — more than any bug it might miss,
because a false green disables every other check downstream of it at once.

A review of the script found three ways it could do exactly that, plus a fourth smaller one. None
is exploitable — this repo has no service, no secrets, no untrusted input, and read-only CI — so
this is ordinary robustness, recorded because the machinery is load-bearing.

**1. The objcopy exit status was discarded.** The script set no error-exit option, and the next
thing to read an exit code was the `case` over the simulator's status. Both `objcopy` calls (ADR-0008's
two images) ran unchecked. A failing objcopy — a binutils without `--verilog-data-width`, a full
disk, a `--only-section` filter matching nothing — left no image or a truncated one, and the
simulator ran against it anyway, with its verdict reported as if it were about the CPU.

The worst case is specific and quiet: **an empty RAM image**. The hex parser accepts an empty file,
the `tohost` word reads zero, and every test with no data dependency still executes to
`RVTEST_PASS` and exits 0. The gate prints PASS for a test that never received its data image. The
objcopy error text landed in the build log, but the log was printed only for tests that *failed* —
so on a false PASS the diagnostic was discarded too.

**2. The failure set recorded only the test name.** ADR-0014's contract is a set equality against
`test/EXPECTED_FAIL`, in both directions, so an unexpected pass is caught as well as an unexpected
failure. But the set's elements were bare filenames, so the comparison was blind to *why* a test
failed. `trap.S` is baselined for a precise reason — `MONITOR-ERROR 101`, "mismatch in trap", the
one spec-value check ADR-0033 gap 2 deliberately keeps live under `spec_trap`. If it instead began
reporting `ASSEMBLE-ERROR` (someone breaks the assembly, or the assembler regresses),
`TIMEOUT`, or `RUNNER-ERROR` (the sim binary will not start), the name was still in the failure set
and the gate exited 0. A broken test and a broken *harness* were both laundered into a green merge
gate. An assembler error passing silently also defeats the project's warnings-are-errors rule,
which the `ASSEMBLE-WARNING` label exists to enforce.

**3. The temp directory creation was unchecked.** With no error-exit option, a failed `mktemp` left
`$tmp` empty and every artifact path collapsed to the filesystem root (`/add.elf`, `/add.rom.hex`).
The cleanup trap then removed nothing. On a CI container running as root those writes succeed, so a
later run whose objcopy also failed could pick up the *previous* run's stale image and report PASS
from an image the current build never produced. It also moved the images out of `mktemp`'s private
0700 directory onto fixed, predictable paths. The same trap-before-`set -e` ordering was in the
Makefile's `test-units` recipe.

**4.** The baseline file itself was read with no existence check. A missing or mistyped path yielded
an empty expected set, and against an all-passing suite the gate printed "Failure list matches
`test/EXPECTED_FAIL` exactly" having compared nothing.

## Decision

**The gate checks every step that produces the thing it tests, and the baseline pins the failure
mode.**

### The baseline format grows a second field — this is the ADR-0014 amendment

`test/EXPECTED_FAIL` entries are now `<test>.S <STATUS>`:

```
trap.S      MONITOR-ERROR 101
```

The status is the exact label the table prints, including its numeric suffix where it has one.
Whitespace between the fields is free (both sides are normalised before comparison). A line with
only a name — the old format — is **rejected with an error naming the format**, not half-matched:
silently accepting it would make every baselined entry unmatchable in a way that reads like a
regression.

This is the natural extension of ADR-0014's contract rather than a departure from it. That contract
already catches an unexpected *pass*, because it is a set equality and not a ceiling; it should
equally catch a test that fails for a new reason. Everything else in ADR-0014 stands unchanged: the
file is still edited by hand and never regenerated from a run, lines still come out one at a time in
the commit that makes that test pass, and adding one back is still a regression that needs
justifying in the PR. **Changing an existing line's status is the same kind of claim as removing the
line**, and needs the same justification — that is the whole point of the second field.

### The build steps are checked, and the simulator is gated on them

- `set -euo pipefail` is on. Every place a nonzero status is *expected* handles it at its own call
  site (`if ! cmd`), so error-exit stays live everywhere else. The simulator's status is captured
  explicitly rather than read from `$?` at a distance, and it is the **one** site where error-exit
  is lifted (`set +e` … `set -e`) rather than guarded with `|| sim_status=$?`. That is not a style
  choice: **bash 3.2 — macOS `/bin/bash`, the interpreter in this script's shebang — rewrites a
  127 "command not found" to 1 while `set -e` is in force.** An unstartable runner would then land
  in the `1` arm and be reported as `FAIL`, a verdict about the CPU. Measured on the way in, not
  assumed; it now reports `RUNNER-ERROR 127`. Adding `set -e` to a script is not free, and this is
  the shape of what it costs.
- Each objcopy is checked separately and carries its own status label with the region that failed:
  **`OBJCOPY-ERROR rom|ram`** for a nonzero exit, **`OBJCOPY-EMPTY rom|ram`** for the zero-exit
  empty-output case. The empty case gets its own label because it is the one that used to produce a
  false PASS rather than a visible error; the two are diagnosed differently and should read
  differently. All 52 programs in the suite produce non-empty images in both regions today (the
  smallest RAM image is 21 bytes), so the emptiness check has no legitimate case to reject.
- The simulator runs only when the assemble step and both objcopy steps succeeded.
- `RUNNER-ERROR` carries the unexpected exit status, so "the sim crashed" and "the sim exited 5"
  are distinguishable in the table.
- The failure log is a single per-test build log covering the assembler and both objcopy calls.

### The setup probes and the temp directory are fatal

- `objcopy` is probed with `command -v` alongside the compiler probe, rather than derived blind from
  the compiler's path and assumed to exist. A half-installed toolchain says so once, up front,
  instead of appearing as 52 identical per-test build failures.
- `mktemp -d` failing is fatal, and its output is additionally checked for emptiness and
  directory-ness before the trap is installed.
- The baseline path is checked for existence and readability **before the suite runs**, so a
  mistyped path fails in a second rather than after a full run.
- The Makefile's `test-units` recipe enables `set -e` **before** `mktemp` and before installing its
  trap, and asserts the directory exists.

## Consequences

- `test/EXPECTED_FAIL`'s format changed. Any future line must carry a status; the script says so if
  one does not. Nothing else in the repo reads this file.
- The gate is strictly harder to satisfy than it was: every path added here can only turn a PASS
  into a failure, never the reverse. Each was demonstrated failing — a forced objcopy failure, a
  baselined test changed to a different failure mode, an unwritable `TMPDIR`, a missing baseline —
  rather than argued.
- A toolchain or environment problem now stops the run instead of being scored as a CPU verdict.
  That is a behaviour change for anyone running on a broken setup: they get an error, where before
  they got a table.
- `OBJCOPY-ERROR` / `OBJCOPY-EMPTY` join the label set that `test/EXPECTED_FAIL` may contain. Nothing
  should ever be baselined under them — they describe a broken build, not a known-red property —
  but the format admits them, and a line under either is a review flag.
- This does not make the gate *correct*, only harder to fool. The classes ADR-0032 measured — state
  no retiring instruction names — are untouched by any of it, and so is everything in ADR-0023.
