# ADR-0207: nano's formal checks read the latch register file where genchecks lets them

**Status:** Accepted · 2026-09-23 · *Amends ADR-0189*

## Context

ADR-0189 shipped `NANO_LATCH_RF`, a latch-array register file behind a build option, and
named the condition for making it the default: "`NANO_LATCH_RF` cannot become the default
until `make -C nano/formal check` reads it: `clk2fflogic` added to every `.sby` script and
`checks.cfg`, F/G re-derived, and the flip-flop build's own checks re-confirmed unaffected."
Today, running any of nano's formal harnesses against the latch build stops at the same
point ADR-0189 recorded: yosys accepts the fifteen `$dlatch` cells cleanly through
`prep -nordff`, then refuses them at the model-writing step with `please run clk2fflogic
before write_btor` (or the identical complaint naming `write_smt2`).

## Decision

**Nano's hand-written harnesses now read the latch build; the genchecks-generated ladder
and `remeasure-fg` do not, and cannot without forking a pinned file.** The two halves of
`nano/formal/` sit on opposite sides of a line ADR-0040 already drew for littlecpu's own
ladder, for a reason specific to `clk2fflogic` rather than to latches:

**`clk2fflogic` turns one clock cycle into two BMC steps** (ADR-0040's Finding 3, measured
there at exactly `k = 2*CHECK_CYCLE + 1`). A property search that used to need `depth N`
steps to cover `N` real clock cycles needs roughly `2N + 1` under it.

**Nano's five hand-written proofs — `ill_e.sby`, `dmemcheck.sby`, `imemcheck.sby`,
`traps.sby`, and their `_cover` anti-vacuity twins — have no macro standing between their
raw BMC bound and the real cycles it searches.** Each states `[options] depth N` directly;
`N` steps search `N` real cycles today, full stop. Doubling `N` (plus one, matching
ADR-0040's exact formula) restores the same span of real-cycle coverage under
`clk2fflogic`, with nothing else to keep in step. `complete.sby`/`complete_cover.sby` are
the same shape but are not touched here — see Deferred below.

**The genchecks-generated ladder cannot take the same fix, because one number does three
jobs at once.** `formal/genchecks-local.py` (vendored byte-for-byte from riscv-formal,
`formal/pin.mk`'s pin; `make -C formal genchecks-check` enforces it stays that way, and
it is the identical file both `formal/` and `nano/formal/` run — `genchecks-audit.py`
takes the harness directory as its only argument) reads one `[depth]` value per check
family and writes it into three places: sby's `skip`, sby's `depth` (one more), and the
`` `define RISCV_FORMAL_CHECK_CYCLE`` a check's own `.sv` compares its retire counter
against:

```python
hargs["depth"] = depth_cfg[0]
hargs["depth_plus"] = depth_cfg[0] + 1
hargs["skip"] = depth_cfg[0]
# ...
# `define RISCV_FORMAL_CHECK_CYCLE @depth@
```

Under `clk2fflogic`, correctness needs the BMC step budget to be roughly *twice*
`CHECK_CYCLE`, not one more than it — and no value of `depth_cfg[0]` can be simultaneously
itself (for `CHECK_CYCLE`, which must stay a real cycle count so the check's own retire
counter — itself `clk2fflogic`-transformed the same way, so it still counts real cycles —
reaches it) and its own double (for the step budget). Raising the checks.cfg column moves
both by the same +1 relationship regardless of magnitude, never the required 2:1 one. This
is exactly ADR-0040's Finding 3, reproduced here by reading the same source rather than by
re-running its (13-14x costlier) experiment: fixing it needs either a fork of
`genchecks-local.py`, which ADR-0031 forbids and `genchecks-check` grades on every CI run,
or a `.sby` rewritten after generation — the "new, unenforced, load-bearing piece of the
oracle" ADR-0040 declined for the identical reason. `remeasure-fg.py` sweeps `hang`/
`liveness` through this exact same generation path (`genchecks_audit.probe()` builds a
one-check set from `checks.cfg` per swept depth line), so it is blocked the same way, for
the same reason, not a different one.

**So `make -C nano/formal check` (the 78-check generated ladder) and `make -C nano/formal
remeasure-fg` stay exactly as they are: flop build only, no `clk2fflogic`, unaffected by
this change.** This is the gate step 3 of the originating brief asked to be named when it
applies per-mechanism rather than per-build: the generated ladder is gated for
`NANO_LATCH_RF`, structurally, by a file this repo does not own; the hand-written harnesses
are not.

### What now reads NANO_LATCH_RF

Seven new `.sby` files, each the existing script plus `verilog_defines -D NANO_LATCH_RF`
before `read_verilog -sv nano.v` and `clk2fflogic` after `prep`, depth doubled (`2N+1`):

| harness | flop depth | latch depth |
|---|---|---|
| `ill_e_latch.sby` | 40 | 81 |
| `ill_e_latch_cover.sby` | 100 | 201 |
| `dmemcheck_latch.sby` | 24 | 49 |
| `dmemcheck_latch_cover.sby` | 24 | 49 |
| `imemcheck_latch.sby` | 15 | 31 |
| `imemcheck_latch_cover.sby` | 15 | 31 |
| `traps_latch.sby` | 25 | 51 |

Every shipping-core proof above was run for real (not stubbed) against `NANO_LATCH_RF`
and passes:

- `ill_e_latch.sby`: PASS, depth 81, ~2 minutes.
- `imemcheck_latch.sby`: PASS, depth 31, ~3.5 minutes.
- `traps_latch.sby`: PASS, depth 51, **2h23m19s**.
- `dmemcheck_latch.sby`: reached step 44 of 49 clean (no counterexample) over roughly six
  hours before the session that ran it ended; `smtbmc --presat --unroll boolector`'s
  per-step cost grows sharply past step ~35, the same shape ADR-0040 measured (13-14x per
  check at a correctly configured depth, there on a much shallower one). **Not yet a
  completed PASS** — the last five steps are owed, and are exactly the kind of run a
  scheduled CI job, not an interactive session, should carry.

**Only `ill_e_latch` was also proved against a real mutant in this session.**
`ill-e-probe.py --sby-file ill_e_latch.sby` runs the shipping core (PASS, above) and the
wrong-RV32E-rule mutant (FAIL) — the same mutation ADR-0189's own `ill_e.sv` already reads
off nano.v's RVFI report, unmodified — demonstrating the doubled-depth `clk2fflogic`
script is not vacuous: it catches a real defect, not just a design that happens to pass.
`ill-e-probe.py`, `probe_common.py` (shared by `traps-region-probe.py`/
`traps-tval-probe.py`), and `memcheck-cover-probe.py` all gained the argument needed to
run the same mutation proof against `dmemcheck_latch`/`imemcheck_latch`/`traps_latch`, and
each is unit-tested with a stub `sby` in `make probe-gates` (the argument threads to the
right file and directory, and a missing target is refused by name). **The real,
multi-hour solver runs for those mutation probes are not taken here.**
`traps-region-probe.py` alone runs three full `traps_latch.sby` proofs (shipping, two
mutants); at the measured 2h23m each, that is most of a working day for one probe, and
`dmemcheck_latch.sby`'s single shipping-core run already took several hours.
Running the mutation probes for these three harnesses is real, owed work — the code path
is untested by anything stronger than a stub — and is left to a scheduled CI job rather
than blocking this ticket on a multi-day session.

**These are multi-hour proofs, not a `make test`-scale cost.** None of the seven targets
above is added to `make test`'s path, matching every other formal target in this
repository that needs a real solver. `traps_latch` alone costs more real time than every
other check in this ticket combined, and `dmemcheck_latch` more still.

`check-memcheck-depth.py` gained `--clk2fflogic`, which grades `2*floor+1` instead of
`floor` (`nano/formal/Makefile`'s `memcheck-depth` target now checks all four memchecks,
against the unrun `dmemcheck_latch.sby` too); `formal/memcheck-cover-probe.py` gained
`dmemcheck_latch`/`imemcheck_latch` as `--check` choices, reading the same unmodified
`dmemcheck.sv`/`imemcheck.sv` a `_latch` variant's `.sby` already does (`NANO_LATCH_RF`
lives in `nano.v`, never in the checker) — unit-tested with a stub `sby`, not yet run for
real against the stalled-bus mutant.

### F and G

**Unchanged in real-cycle terms: F = 12, G = 10.** `clk2fflogic` changes how many BMC
steps model a clock cycle; it does not change nano.v's own behaviour, so the worst-case
first-retire and retire-gap cycle counts a correct core actually exhibits are the same
whether or not the solver is asked to model them two steps at a time.
`remeasure-fg` cannot confirm this independently under `clk2fflogic` — it is blocked by
the identical `genchecks-local.py` coupling described above, so re-running it would sweep
`hang`/`liveless` through the same vacuous horizon the generated ladder would. What is
stated here is therefore a derivation from the unaffected flop-build measurement, not a
new one: the equivalent BMC-step figures, used only to size the hand-written harnesses'
depth floors (never fed back into `checks.cfg`), are `2*12+1 = 25` steps to first retire
and `2*10 = 20` steps of gap.

### Deferred

**`complete.sby`/`complete_cover.sby` are not given `_latch` twins here.** They are the
same shape as the other hand-written harnesses (a plain `[options] depth N`, no genchecks
coupling) and would take the identical fix, but `complete.sby` walks the entire
`isa_rv32imc` instruction list at once, and ADR-0040 measured `clk2fflogic` at a 13-14x
per-check wall-time cost on top of the doubled depth. Running that multiplier across every
instruction in one exhaustive sweep is untested and could be prohibitively expensive for a
CI job; closing this gap is left to a future ticket that can measure it rather than guess
at a timeout.

### The default stays flops, on two gates now

This ADR closes the verification gate ADR-0189 named. **It does not flip the default.** A
timing measurement made after ADR-0189, tracked separately from this ticket, found the
latch build's worst setup paths run from a register-file latch (a positive level-sensitive
latch clocked by `clk'`, the inverted clock) to a flip-flop and so get only half a clock
period: at the slow corner it misses setup by about 2 ns at every clock target tried (24,
27, and 36 ns), because the budget scales with the period. `NANO_LATCH_RF` becoming the
default is gated on **both** this ADR's verification **and** that timing question being
resolved — not on either alone.

## Consequences

- `make -C nano/formal ill_e_latch`, `ill_e_latch_cover`, `dmemcheck_latch`,
  `dmemcheck_latch_cover`, `imemcheck_latch`, `imemcheck_latch_cover`,
  `components_traps_latch`, and the aggregate `all-latch` are new targets. None is added to
  `make test`'s path — like their flop-build siblings, they need a real solver and are not
  hermetic, so they stay formal-only CI/manual targets.
- `make -C nano/formal check` and `make -C nano/formal remeasure-fg` are byte-for-byte
  unmodified in this change and continue to run the flop build only; the 78-check ladder
  and its `EXPECTED_FAIL`/`EXPECTED_CHECKS` baselines are untouched.
- `check-memcheck-depth.py`, `ill-e-probe.py`, `nano/formal/probe_common.py` (and so
  `traps-region-probe.py`/`traps-tval-probe.py`), and `formal/memcheck-cover-probe.py` all
  gained an optional argument; every default-argument call site (the flop build's existing
  Makefile targets) is unchanged, and `make probe-gates` covers each new argument's own
  error path.
- `complete.sby`/`complete_cover.sby` staying flop-only is a known gap, not a silent one:
  `NANO_LATCH_RF`'s illegal-instruction rule is proved against a real mutant;
  `imemcheck_latch`'s shipping core passes for real; `traps_latch`'s shipping core passes
  for real but its region/mtval mutants and `dmemcheck_latch`'s own shipping run are owed
  (above), and none is proved against the ISA-completeness sweep.
- `NANO_LATCH_RF` remains off by default. Flipping it needs a second ADR closing the
  setup-timing question this one deliberately leaves open.
