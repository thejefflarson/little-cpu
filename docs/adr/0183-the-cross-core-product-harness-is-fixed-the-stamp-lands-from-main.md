# ADR-0183: The cross-core product harness is fixed; the stamp itself lands from main

**Status:** Accepted · 2026-09-13

## Context

`soc/compare/product.json` was stamped at `122ef7bdd2b0`, `dirty: yes`, against
hx8k-era `ram_words` (512, since 16384) and `rv32i` Dhrystone CFLAGS (since
`rv32im`). Every `make compare-dhrystone` run printed its own `*** STALE:`
warning naming the drift. Separately, `product_check.py`'s `moved_paths()`
asked git for every path under `rtl/` or `soc/compare/` that differed from the
stamped base, and `soc/compare/product.json` is itself under `soc/compare/`:
a run that wrote the file then read its own write as evidence of staleness,
so `make compare-product` could never exit 0. That defect closed separately
(`stale_reasons()`/`moved_paths()` take the artifact's own path and exclude
it, both directions probed in `test/probe_gates.sh`).

## The stamp does not land from this PR

A first version of this change re-took the measurement and committed
`product.json` with `base` set to a commit on this PR's own branch. That does
not survive review: this repo squash-merges and deletes the source branch on
every merge, so the instant this PR merges, `base` names a commit no checkout
can resolve any more (`git cat-file -e <base>^{commit}` fails). Every later
`product_check.py`/`product_diff.py` call against that stamp would either
refuse outright or (after this ADR's own fix below) call it permanently
"news," and the weekly schedule workflow would open a refresh PR on its very
first run for a reason that has nothing to do with the numbers moving.

**The stamp instead lands from `main`.** `.github/workflows/compare-product-schedule.yml`
already exists to re-measure and open a PR on a schedule; once this PR merges,
the main loop dispatches it against `main`, where the commit it measures from
is one every future checkout can resolve. This is also this ticket's
`workflow_dispatch` acceptance criterion: dispatching it from an unmerged PR
branch would repeat the exact mistake this section describes, so it is a
post-merge step, not something this PR does itself.

## What the harness fixes, so the eventual stamp is correct rather than merely fresh

A stamp with a resolvable base is not enough on its own; the first attempt at
this measurement surfaced four more defects in the harness that would have
been stamped right alongside it:

- **`product_diff.py`'s `is_news()` had the identical self-watching bug**
  `product_check.py` already fixed, one level up: it called `stale_reasons()`
  without the artifact-exclusion parameter, so the freshly-written
  `product.json` always counted as its own evidence of staleness and
  `--require-news` could never report "no news." Fixed by threading the same
  `artifact` parameter through from `product_diff.py`'s own `args.after`.
- **An orphaned `base` made `--require-news` refuse instead of recover.**
  The squash-merge problem above is not hypothetical for the *older* snapshot
  a scheduled re-take diffs against, either -- once one stamp has been through
  this cycle, `stale_reasons()` would refuse the whole question rather than
  answer it. `is_news()` now checks `base_resolvable()` first and treats an
  unresolvable base as news outright, so the job recovers by re-measuring
  instead of failing forever.
- **up5k's product used each core's own placed Fmax, not the 12 MHz step.**
  CLAUDE.md's own rule is that up5k's clock is a step function and the
  comparison there is cycles alone once every core clears it; `product_write.py`
  multiplied by each core's own worst/median MHz regardless, so a stamped
  up5k ratio (VexRiscv/littlecpu, 1.990× worst) overstated the real,
  same-clock answer (1.14×) by treating unspendable placement margin as
  throughput. `--step-mhz` now lets the caller supply the fixed clock every
  core in an up5k pair has already cleared (`make compare-timing`'s own
  `step_gate.py` refuses a placement that has not), and the product is
  computed from cycle factors alone at that clock; each core's own Fmax stays
  recorded in `clock_mhz` as margin. ECP5 has no step and omits the flag,
  keeping the real Fmax product.
- **The scheduled workflow's pair-derivation could fail open.** It read
  `product.json`'s pair names from a `python3 -c` call inside a process
  substitution, whose exit status `set -e` does not see -- a broken call
  there would silently check zero pairs rather than fail the step. Captured
  into a plain variable instead (command substitution failures *do* trip
  `set -e`), refused if empty, and a pair name the step does not recognise is
  now refused rather than silently skipped.

Also closed in the harness, none of them changing what gets measured: both
`product_check.py` and `product_write.py` now validate `base` as a 40-character
commit SHA (a malformed stamp is refused as malformed, not graded stale or
written at all) and `moved_paths()`'s `git diff` takes `--end-of-options`
before it; both `product_check.py` and `product_diff.py` wrap `main()` so an
uncaught bug exits with the "refused" status rather than colliding with
"stale"/"no news"; `soc/compare/run_product.sh`'s `measure_coremark()` now
propagates a write failure with `return 1` (`set -e` is suspended for the
whole function, since it is the left side of `measure_coremark && COREMARK_OK=1`
at the call site) and the caller exits rather than falling back to
`not_yet_measured`, which could otherwise overwrite a part's pair that had
already measured correctly; the CoreMark `product_check` calls now pass the
same `--current cflags/rom_words/ram_words` the Dhrystone ones already did;
and an ECP5 pair now stamps its own `ecp5_part`/`ecp5_target_mhz` fields
(via a new generic `--field NAME=VALUE`) and the Trellis database's digest
(via `soc/print_toolchain.sh trellis-db`, asked for only when ECP5 is being
placed), checked with `--current` the same generic way `cflags` already is.
`.github/workflows/compare-product-schedule.yml` gets `persist-credentials:
false` on its checkout, `gh auth setup-git` plus `GH_TOKEN` confined to the
one step that pushes a branch and opens a PR, and `if: github.ref ==
'refs/heads/main'` on the job, so a `workflow_dispatch` cannot push from
anywhere else; `test/compare_product_schedule_token_test.py` grades all three,
mirroring `test/pin_bump_token_test.py`'s shape for the pin-bump workflow.

## The measurement taken on this branch, and what it says about CLAUDE.md's numbers

Toolchain: Yosys 0.68+48, nextpnr-ice40/nextpnr-ecp5 0.11-1-g62e659ed, icetime
(oss-cad-suite 20260811), Icarus Verilog 14.0, riscv64-elf-gcc 16.2.0. Base
`ef9dacc42773`, `dirty: no`, twelve seeds a side (`default 1 2 3 4 5 6 7 8 9
10 11`), paired by seed.

| pair | core | worst MHz | median MHz |
|---|---|---|---|
| dhrystone (up5k) | littlecpu | 12.61 | 12.98 |
| dhrystone (up5k) | vexriscv | 21.92 | 22.78 |
| coremark (up5k) | hazard3 | 14.00 | 14.39 |
| dhrystone_ecp5 | littlecpu | 32.01 | 33.70 |
| dhrystone_ecp5 | vexriscv | 52.91 | 54.91 |
| coremark_ecp5 | hazard3 | 48.88 | 50.39 |

**These do not reproduce CLAUDE.md's own up5k figures, and that is the
correct thing to say about them, not "within rounding."** CLAUDE.md's
existing paragraph quotes littlecpu at 12.40/12.85/13.23 MHz and Hazard3 at
14.30/14.57/14.95 from an earlier session; this one reads a real, placement's
worth different (about 1.7% on littlecpu's worst), which is inside the
~4-9% placement spread CLAUDE.md's own measurement section already documents
for this class of sweep, not a discrepancy that needs chasing. The ECP5 rows
above do reproduce CLAUDE.md's already-quoted 32.01/33.70, 52.91/54.91 and
48.88/50.39 exactly, because both sessions read the same shipping RTL through
the same pinned-enough toolchain on a part with no quantisation step to
launder small placement differences the way up5k's does.

**Six of CLAUDE.md's absolute DMIPS/CoreMark figures were off by a rounding
methodology, independent of which session's clocks were used**, and this ADR
is where that correction is recorded: multiplying `cycle_factor` by the clock
at full precision and rounding once, rather than rounding an intermediate
value first, gives littlecpu 9.39 DMIPS and Hazard3 18.03 CoreMark at up5k's
fixed 12 MHz step (CLAUDE.md had 9.40/18.04), and VexRiscv 47.42 DMIPS,
littlecpu 73.89 CoreMark, VexRiscv 123.91 CoreMark and Hazard3 73.45 CoreMark
at each core's own ECP5 worst placement (CLAUDE.md had 47.41/73.88/123.92/73.46).
CLAUDE.md's cross-core paragraph now carries these six corrected figures.

**Hazard3's Dhrystone row is not stamped by `run_product.sh` and this
measurement says nothing new about it.** The `dhrystone`/`dhrystone_ecp5`
pairs carry only littlecpu and VexRiscv -- Hazard3 is CoreMark's second
column, not Dhrystone's, matching what the ticket that added the ECP5 sweep
scoped. Hazard3's Dhrystone figures in CLAUDE.md (10.80 DMIPS at up5k, 43.99
at ECP5, its ratios) stay sourced from the manual `make compare-dhrystone`
sweep ADR-0146/ADR-0160 already cite.

## Consequence

`soc/compare/product.json` is unchanged by this PR. CLAUDE.md's cross-core
paragraph cites this ADR for the rounding correction and states plainly that
the real stamp lands from `main` after merge, via the scheduled workflow's
post-merge dispatch.
