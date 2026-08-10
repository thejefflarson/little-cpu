# ADR-0095: Co-simulation is a required check, and the precondition for that was met two changes ago

**Status:** Accepted · 2026-08-10 · *Amends
[ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md)'s "stays opt-in" decision
and [ADR-0033](0033-what-the-green-ladder-does-not-cover.md)'s decision 4, both of which named the
same precondition. Adds a required check to `main`, which is a repository setting a human applies.*

## Context

[ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md) built the co-simulation
harness and deliberately kept it off `make test` and off CI. It gave one reason, and it named
exactly what would lift it:

> The fetch has no checksum and no signature, and a GitHub release asset is mutable, so
> `SAIL_RISCV_VERSION` pins a name rather than bytes. That is an accepted gap with a named
> precondition: **integrating co-sim into `make test` or CI means giving this fetch `formal/pin.mk`'s
> treatment first** — a checksum verified before anything out of the tarball runs.

[ADR-0033](0033-what-the-green-ladder-does-not-cover.md)'s decision 4 restated it as the same
condition and added two smaller notes about the target: that bumping the version did not re-fetch,
and that a substituted install never showed up in `git status`.

**That precondition has been met, and the ADRs were never amended to say so.** Reading the recipe
today, `make sail-setup`:

- refuses a `SAIL_RISCV_VERSION` set from the command line or the environment, `override`, in the
  shape `formal/pin.mk` uses for the riscv-formal SHA;
- rejects a version that is not three numeric parts, so a branch, a moving tag or a range cannot be
  pinned;
- carries a SHA-256 per platform asset, and refuses outright to fetch an asset it has no digest for;
- refuses to unpack at all when neither `shasum` nor `sha256sum` is on PATH, rather than skipping
  the comparison on a machine that cannot make it;
- compares the digest **before extraction**, so nothing out of the tarball exists on disk, let
  alone runs, unless the bytes are the pinned ones;
- rejects a member outside the asset's top-level directory, a `..` component, or an entry that is
  neither a file nor a directory;
- records the pin and the unpacked binary's own digest in a stamp, and re-checks both on every
  `cosim-run` and `cosim-suite`, so a stale or substituted install is an error instead of a silent
  result.

What remained was enforcement. Co-simulation ran when a person remembered: eight pull requests in
one session carried its output because each engineer was asked for it, and nothing in the repository
would have noticed if one had not been.

**That matters more than for any other leg here, because of what only this one reads.** Every other
oracle evaluates a spec model on the core's self-report. `test/monitor.v` and every generated
`insn_*` check compare `rvfi_rd_wdata` against a model of `rvfi_insn` / `rvfi_rs1_rdata` /
`rvfi_rs2_rdata`; a core that mis-reports a value and computes with that same mis-reported value
tells all of them an internally consistent story. `test/cosim.cc` reads `uut regfile regs_a` through
cxxrtl `debug_items` and no `rvfi_*` signal at all. ADR-0032 demonstrated the difference with an
extra `regs[31] <=` write enabled only past the BMC bound: the whole `.S` suite passed, the
generated riscv-formal checks matched `formal/EXPECTED_FAIL` exactly, and co-simulation reported it
in 0.6 s.

**The remaining objection was availability, and it does not survive inspection.** The asset comes
from `github.com/riscv/sail-riscv/releases`; CI's checkout, its Actions and its runners already come
from that same origin unconditionally. If GitHub is unreachable there is no job to block, so the
fetch adds no failure mode this workflow does not already carry. That argument was about a
dependency CI has had all along.

## Decision

**`make cosim-suite` runs as a required check on `main`, in a job of its own, and the fetch it needs
is cached as bytes that still meet the pin in the run that executes them.**

### The job

`.github/workflows/ci.yml` gains `cosim`, on the same self-hosted runners as `test`: verify the
cross compiler and `clang++` are in the image, put the OSS CAD Suite on PATH, restore the cached
tarball, `make sail-setup`, `make cosim-suite`. What is graded is `test/run_cosim.sh`'s exit status
— the set of programs that do not agree, against `test/COSIM_EXPECTED_FAIL`, under set equality in
both directions, so an unexpected *agreement* is as red as an unexpected divergence and a baselined
program diverging a different way is red too. The command is redirected to a file and its status
re-raised, never piped: the default step shell is errexit without pipefail, and a graded command in
a pipeline takes `tee`'s status, which is how the formal job stayed accidentally green for its whole
life.

**A job rather than a step of `test`, and it gates nothing else.** It needs a network fetch no other
job here does, and a red result has to read as "co-simulation", not as a suite failure. Nothing on
`make test`'s path reaches any of it, so the merge gate still runs whole on a machine with no Sail
install.

### The cache holds the tarball, not the unpacked tree

`make sail-setup` now keeps the verified tarball under `~/.cache/little-cpu/download` instead of
deleting it after extraction, and `actions/cache` is pointed at that directory with a key of
`version + asset + digest`. `make sail-pin` prints the key and the path, so the workflow reads them
out of the Makefile rather than restating them; a key cannot go on naming bytes the pin no longer
has.

**Caching the unpacked tree was the obvious thing and is the weaker one.** A restored tree is put on
PATH and executed having met no checksum in that run — the stamp it would be checked against comes
out of the same cache entry. The precedent next door,
`.github/actions/setup-oss-cad-suite`, keys its cache on the published digest for exactly this
reason and says so, and that mitigation is real but indirect. Caching the tarball needs no such
argument: the cache holds an inert file, the pinned SHA-256 is compared on every run whether the
entry hit or missed, and the binary that runs was produced in that job from bytes that had just met
it. It costs 1.8 MB of cache and about 0.4 s of extraction.

## Evidence

Measured on this repository's runners, at `f144fc9` and its two deliberately-broken descendants.

**Cold, cache miss, green:** `Cache not found for input keys:
sail-0.13.1-sail-riscv-Linux-x86_64-ee052f…`, then `fetching …`, then `sha256 ok: ee052f…`, then
`60/62 agreed` and `Divergence list matches test/COSIM_EXPECTED_FAIL exactly`. The cache saved under
that key in the post step.

**Warm, cache hit, green:** `Cache restored from key: sail-0.13.1-sail-riscv-Linux-x86_64-ee052f…`,
`using the tarball already in /home/runner/.cache/little-cpu/download`, `sha256 ok: ee052f…`. The
digest comparison ran on the restored bytes, which is the property the tarball cache exists for.

**Wall time, four runs:**

| run | fetch/verify/extract | `make cosim-suite` | job |
|---|---|---|---|
| cold, cache miss | 1 s | 58 s | 69 s |
| warm, cache hit (the mutation below) | <1 s | 88 s | 100 s |
| warm, cache hit | <1 s | 118 s | 131 s |

**Read the fetch column, not the job column.** The cache is worth about a second, and the 69–131 s
spread is contention: two self-hosted runners take the workflow's ten jobs, so `make cosim-suite`
itself measures 58 s to 118 s for the same 62 programs. Locally it is 41 s. The cold run being the
fastest of the three is that spread, not a result about caching.

**The job can fail, for the reason it exists.** ADR-0032's canonical defect — an extra architectural
write no retiring instruction names — reintroduced in `rtl/regfile.v` as `regs_a[31] <= wdata`
alongside `regs_a[waddr]`, and the same for `regs_b`:

| job | result |
|---|---|
| `test` (62 programs, per-retire RVFI monitor live, plus `regfile_tb`) | **success — misses it** |
| `elaborate`, `lint`, `components`, `monitor-freshness`, `nonperturbation` | success |
| `formal` | failure (`reg_ch0`, which is the one check that ties the report back to the register file) |
| `cosim` | **failure**, `0/62 agreed` |

```
--- add.S ---
DIVERGENCE at architectural change #0
  sail instruction #5  pc=0x0000000c  c.li x3, 0x2
  sail : x3=0x00000002
  core : x3=0x00000002 x31=0x00000002   (cycle 12, decode pc=0x00000014)
COSIM-STATUS DISAGREE AT 0
```

`fit` and `soc-timing` went red on the same commit as well, since two extra write ports cost area
and period. This is the unconditional mutation, which `reg_ch0` reaches inside its bound; ADR-0032's
gated variant is the one every check but this leg misses, and that measurement is not re-taken here.

**The digest comparison fires in CI, and refuses rather than executing.** Pointing
`SAIL_SHA256_sail-riscv-Linux-x86_64` at 64 zeroes:

```
Cache not found for input keys: sail-0.13.1-sail-riscv-Linux-x86_64-0000…
fetching https://github.com/riscv/sail-riscv/releases/download/0.13.1/sail-riscv-Linux-x86_64.tar.gz
sail-riscv tarball SHA-256 MISMATCH -- refusing to extract:
  expected : 0000000000000000000000000000000000000000000000000000000000000000
  actual   : ee052f64494a2f5f071afd9c2cb4aa5eaae4ba84753e4f77e442b4f83f2e9469
  tarball  : /home/runner/.cache/little-cpu/download/sail-riscv-Linux-x86_64-0.13.1.tar.gz (removed)
```

The `make cosim-suite` step never ran. Note the cache missed too — a wrong digest is a different
key, so a substituted pin cannot be served an entry fetched under the right one.

**And the comparison is forced red without a network, in `make probe-gates`.** Three probes drive
`make sail-setup` with a stub `curl` that substitutes the asset: the mismatch is refused before
extraction; a refused download leaves nothing unpacked and does not survive to be served to the next
run; and a tarball already in the cache is reused and still has to meet the digest. The second
reports `refused=yes` alongside what it found on disk, so it cannot read clean because `make` died
before reaching the comparison at all. `SAIL_ASSET` is fixed in those probes rather than taken from
`uname`, so the fixture is the same on every host and `make test` does not start requiring a machine
upstream ships a tarball for.

## What a green `cosim` job does and does not mean

- **It covers the programs it is given, and nothing else.** Co-simulation inherits the `.S` and `.c`
  suite's coverage exactly. It says nothing about instructions or operand patterns those programs
  never reach — including the real multiplier and divider on untested operands.
- **It structurally cannot cover the interrupt path.** `test/COSIM_EXPECTED_FAIL`'s two entries,
  `mtimer.S` and `mtimermask.S`, are `INCONCLUSIVE SAIL-LIMIT`: the reference model's timer is at a
  different address with a different tick period, both conformant, so the model never takes the
  interrupt and nothing is compared. That is weaker than a divergence and is recorded as such. The
  entries stay exactly as they are; **a green `cosim` job is not evidence about the one interrupt
  this core takes.** What covers that is `formal/traps.sv`, `test/decoder_tb.v`, `test/csr_tb.v` and
  `test/timer_tb.v`, as `test/COSIM_EXPECTED_FAIL` already sets out at length.
- **It is not a proof.** It compares two executions of the same program.

## Consequences

- **`main`'s required set gains `cosim`**, alongside `elaborate`, `test`, `components`,
  `monitor-freshness`, `lint`, `formal`, `soc-timing` and `nonperturbation`. Which jobs are required
  is a repository setting; read it live with
  `gh api repos/thejefflarson/little-cpu/branches/main/protection`, never from a comment.
- **The merge gate grows a tenth job on a two-runner pool, and that is what the contention above
  is.** `components` was killed by its own `timeout-minutes: 5` in two of the four runs — after its
  fourth proof had reported `DONE (PASS)`, so the gate read red for a busy pool. On `main` that job
  measures 3:51 to 4:05, which is under a minute of headroom, and the tenth job is what pushed it
  over. Its budget goes to 10, matching `test`. Nothing else in the set came close.
- **CI now depends on `github.com/riscv/sail-riscv/releases` being reachable.** Stated plainly
  because it is new, and bounded by the argument above: the same origin already serves the checkout
  and the runners.
- **A pinned release that upstream deletes or rewrites turns the gate red rather than quiet**, which
  is the intended direction. Recovering means bumping `SAIL_RISCV_VERSION` and its three digests
  together — the fetch refuses an asset it has no digest for, so a half-done bump cannot fetch.
- **ADR-0033's two smaller notes are closed by the same recipe**: the stamp makes a version bump
  re-fetch on its own, and it makes a substituted install an error rather than something only
  `git status` could have shown, which it never could.
- **The nightly job ADR-0032's integration list proposed is not built and is not wanted.** Nightly
  was the right shape for a leg nothing verified the fetch of; a per-PR gate is the right shape for
  one where every run checks the bytes it executes.
- **What ADR-0032 said must stay true, stays true.** `test/cosim.cc` reads no `rvfi_*` signal, and
  the retire sequencing is derived from the register file changing. The moment it samples one, the
  mutation table above stops holding and this job stops being worth its runner.
