# test/CHECKS

`test/CHECKS` is the schedule `make test` and `make test-full` run: one row per check, read by
`test/checks.py`. The Makefile's `TEST_CHECKS` is the list of what exists, and
`test/checks_manifest_test.py` holds the two equal in both directions, so a check added to one and
not the other is red.

```
name | tier | lane | needs | inputs | command
```

| column | meaning |
|---|---|
| `name` | The Makefile target the check runs (`make -s <name>`), unless `command` is given. Unique. |
| `tier` | `fast` runs under `make test` and `make test-full`; `full` runs only under `make test-full`, which is what CI's `test` job runs. |
| `lane` | Checks that share a lane run one at a time, in manifest order. `-` is a lane of its own. A lane is how two checks that write the same path stay out of each other's way. |
| `needs` | Comma-separated rows that must have passed first, or `-`. A `fast` row cannot need a `full` one. |
| `inputs` | Space-separated patterns over `git ls-files --cached --others --exclude-standard`. A pattern ending in `/` claims everything under it; anything else is an `fnmatch` glob in which `*` crosses `/`. A lone `*` means the check always runs and claims nothing. |
| `command` | Optional shell command replacing `make -s <name>`. |

## How skipping decides

`make test` skips a check only when every one of these holds; any doubt runs it.

1. The tier has a green snapshot, `build/test-state/files-<tier>.json`: the hash of every
   non-ignored file at the end of the tier's last fully green run. With none, every check runs.
2. Every file changed since that snapshot is claimed by some row's `inputs`. A changed file no row
   claims runs every check in the tier, and the run names the file.
3. The check's stamp, `build/test-state/stamps/<name>`, equals the hash of the files its own
   `inputs` match, its row text, and the first line of `--version` from yosys, iverilog, the
   RISC-V gcc, clang++ and python3. A stamp is written only when the check passes.
4. If a change is unclaimed, the stamp was also taken in that same epoch (the hash of the
   unclaimed files). A run that failed halfway can therefore resume without repeating the checks
   that passed, and a further edit to the unclaimed file runs them all again.

`make test-full` never skips (`--no-skip`), and CI runs it from a clean checkout. `TEST_NO_SKIP=1
make test` is the same override locally. A tool that is not on `PATH` records as `absent`.

A pattern that matches no file is an error, and every tracked file a check's recipe names must be
claimed by that check's row: `test/checks_manifest_test.py` reads the recipe from `make -n`. It
cannot see a file a script opens by itself, so a row whose script reads further than its recipe
names declares the wider directory.

## Why lanes

Checks run in parallel at `TEST_JOBS` (default: the CPU count). Two checks that write one fixed
path under `build/`, or that rebuild one artifact, share a lane. `needs` orders a check after the
build that produces an artifact it reads.
