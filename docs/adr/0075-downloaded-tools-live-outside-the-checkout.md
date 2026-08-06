# ADR-0075: Downloaded tools live outside the checkout

**Status:** Accepted · 2026-08-06 · *Moves the install location named by
[ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md),
[ADR-0033](0033-what-the-green-ladder-does-not-cover.md) and
[ADR-0043](0043-the-reference-model-is-configured-as-this-core.md), all of which say `tools/sail`.
No `rtl/` change ships from this ADR, and co-simulation stays off `make test` and off CI's required
set.*

## Context

`make sail-setup` unpacked the pinned sail-riscv release into `tools/sail`, and `make lint-setup`
unpacked svlint into `tools/svlint`. Both directories are gitignored, which is correct — they hold
downloaded binaries.

A git worktree is given the repository's **tracked** files. Nothing else crosses. So an install made
in the main checkout exists there and in no worktree, and the only thing a worktree gets is
`sail_riscv_sim not found. Run 'make sail-setup'` — a message that is true about that directory and
says nothing about the copy sitting one directory up.

This matters more than a missing binary because of what co-simulation is. It is the only oracle here
that reads the core's real `regs_a` array instead of the core's own report of what it retired; the
worked case is an extra `regs[31] <=` write enabled only past the BMC bound, invisible to every
riscv-formal check and to all 59 `.S` programs, and reported by co-simulation in 0.6s. Every
engineer working in a worktree was working without that instrument and had nothing telling them so.

## What was measured

- Six of six live worktrees lacked `tools/sail/bin/sail_riscv_sim`. The main checkout had it, and
  `make cosim-suite` there was 59/59 agreed against an empty `test/COSIM_EXPECTED_FAIL`.
- It has already cost a verdict.
  [ADR-0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md) records that
  `make cosim-suite` was run on neither operand-fetch variant, because the worktree those
  measurements were taken in had no Sail binary — while the checkout it branched from did. That ADR
  named the cause correctly, which is the better outcome and not the one to plan for: the message
  the worktree gives says only that the binary is absent from that directory, so reading it as
  "Sail is not installed on this machine" is the expected reading, not a careless one.
- `make lint` has the same shape and does not bite the same way. `SVLINT` resolves from `PATH`
  first and only falls back to `tools/svlint`, so a `brew install svlint` or a
  `cargo install svlint` satisfies every checkout at once; CI's lint job runs `make lint-setup`
  into a fresh full checkout, never a worktree. The hole is real but latent — it appears only for
  someone whose only svlint is the one `lint-setup` unpacked, working in a worktree — so it is
  fixed here for the same reason and by the same mechanism rather than left as an asymmetry
  somebody has to rediscover.

## Decision

**Both setups unpack into `${XDG_CACHE_HOME:-$HOME/.cache}/little-cpu`.** One install serves every
checkout, every worktree and every clone on the machine. `tools/` is no longer written by anything
and stays gitignored so an install an older checkout left there can never be committed.

The pinned-release integrity check is untouched and still gates the unpack: `sail-setup` refuses to
extract a tarball whose SHA-256 is not the digest in the Makefile, refuses to run at all when no
digest is pinned for the host asset, rejects archive members outside the top directory or containing
`..`, records the unpacked binary's own digest in `.sail-pin`, and `test/cosim.py` re-checks that
digest before executing it. `lint-setup` verifies its archive the same way. Moving a directory
changes none of that; the verification was never a property of the path.

### Rejected: symlink `tools/sail` into each worktree at creation

Cheapest, and it reopens silently. It needs a hook or a documented step wherever worktrees are
made, and a worktree created without that step is back to the state this ADR is about — with the
same message, which is what made the original defect invisible.

### Rejected: have `sail-setup` detect a worktree and link to the main checkout's copy

Fixes the symptom for `sail-setup` only, and buys a dependency on `git worktree` layout — the main
checkout's path, resolved out of `.git`, in a Makefile. A plain clone with no worktree still gets
nothing shared, and the "which checkout is primary" question has no good answer for a machine with
two clones.

## Evidence

Both installs verified from a **fresh worktree**, not inferred from the main checkout, because
inferring is the mistake that hid this:

- `make sail-setup` in a worktree: `sha256 ok: 53d0c6f…0737`, then `0.13.1`.
- `make cosim-suite` in a *second*, freshly created worktree that ran no setup at all:
  `59/59 agreed`, matching the empty `test/COSIM_EXPECTED_FAIL`.
- With `~/.cache/little-cpu` moved aside, `make test` is unaffected — 59/59 passed, both baselines
  empty — and `make cosim-suite` stops before running anything with `co-simulation setup is
  incomplete`. Co-simulation degrading to a loud refusal on a machine that never ran the setup is
  the property that keeps it off the merge gate, and it is unchanged.

## Consequences

- **Two files now compute one path**, and a drift between them would make `make cosim-suite` report
  a binary `make sail-setup` had just written as missing — the same failure, from the other end.
  `test/tool_cache_test.sh` compares the Makefile's `SAIL_RISCV_DIR` against `test/cosim.py`'s
  `SAIL_DIR`, and rejects an install directory inside the checkout or named relatively. It hangs off
  `make test`, and its four red directions are forced by `make probe-gates`.
- **An install left in `tools/` by a checkout from before this is now dead weight.** Nothing reads
  it. Delete it; `rm -rf tools`.
- **The staging directory is `mktemp -d`, not a fixed `.tmp` name.** Two checkouts each had their
  own; one cache means two concurrent setups would have shared one staging path. The final
  `rm -rf` / `mv` swap is still not atomic, so two `*-setup` runs racing on one `$HOME` can leave a
  mangled install — a case with no caller today (co-simulation is not in CI, and the lint job runs
  `lint-setup` once), and one the digest re-check in `test/cosim.py` turns into a loud refusal
  rather than a wrong answer. CI cannot reach it either: `little-cpu-runners` is a scale set that
  gives every job its own ephemeral runner, so no two jobs share a `$HOME` — three jobs of one run
  were checked and landed on three distinct runners. The exposure is a workstation running two
  setups at once, by hand.
- `XDG_CACHE_HOME` is honoured by the Makefile and by `test/cosim.py` under the same rule, which is
  also what lets the probes decide the cache location without installing anything.
