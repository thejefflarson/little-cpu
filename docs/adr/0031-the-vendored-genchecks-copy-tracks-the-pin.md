# ADR-0031: The vendored `genchecks` copy tracks the pin, and only `basedir` differs

**Status:** Accepted · 2026-07-30 · *Amends the mechanism half of ADR-0024; leaves its measurement
intact. Also retires the per-check `timeout` ADR-0022 introduced.*

## Context

`formal/genchecks-local.py` is a vendored copy of riscv-formal's `checks/genchecks.py`. The header
it carried claimed one deliberate structural difference — `basedir` resolved relative to the script,
so the harness can live in `formal/` rather than adopting upstream's `cores/<name>/` layout
(ADR-0006) — and then conceded that "the rest of the difference is NOT deliberate."

It was not a small remainder. The fork was 654 lines against upstream's 870 at the pinned SHA, and
the skew had started costing real work:

- **We reimplemented engine selection upstream already had.** ADR-0024 measured `reg_ch0` at ~8–12s
  under `btor btormc` versus non-convergence under `smtbmc yices`, and to act on that finding it
  added an `engine` option and an `[engine]` per-check override section to the fork. Upstream's
  existing `solver` line already selects between `abc bmc3`, `btor btormc`, and `smtbmc <name>`
  from the same three spellings. The fork was simply too old to contain it.
- **Ten checks could not be generated at all** — `fault` and nine `bus_*` checks, upstream work
  from late 2025.
- **The counter-CSR checks were unreachable.** The fork emitted a single `csrc_{name}` pointing at
  `rvfi_csrc_check.sv`; upstream split that into six models (`rvfi_csrc_{any,const,hpm,inc,upcnt,
  zero}_check.sv`), all six present at the pin. `csrc_inc_minstret` is the direct formal statement
  of ADR-0027's minstret rule, and the *fork* was what blocked it — not upstream, and not the pin.

The pin itself was never the problem. `formal/pin.mk` is at upstream HEAD; it was the fork that was
five years stale. This is the failure mode ADR-0006 named as the "version-skew time bomb", arriving.

## Decision

**Re-vendor `formal/genchecks-local.py` verbatim from the SHA in `formal/pin.mk`, re-applying only
the `basedir` change.** A `diff -u` against `formal/riscv-formal/checks/genchecks.py` must show the
header and the two `basedir` hunks and nothing else. The second hunk is the isa-table `open()`,
which spells the same `../..` inline; it is the same one change, not a second one.

Three consequences follow, and each is a decision in its own right:

**1. The local `engine` option and `[engine]` section are gone; `checks.cfg` says `solver btormc`.**
ADR-0024's measurement stands unchanged and is not being relitigated — `btor btormc` is still the
ladder's engine, still for the reason ADR-0024 gives, and the generated `[engines]` line is still
`btor btormc`, verified before the fork's mechanism was deleted. What changes is only *how* the
config asks for it. ADR-0024's decision section promised a one-line per-check override for a future
check needing `smtbmc`-specific machinery; that promise is withdrawn. Upstream has no per-check
override, and re-forking `genchecks` to reinstate one costs more than it buys — sby's own
`[engines]` section remains the lever for a check run by hand.

**2. `reg_ch0`'s 1800s `timeout` is gone.** ADR-0022 added it, and ADR-0024 explicitly kept it "as a
safety net". Upstream `check_cons` has no `timeout` parameter and no way to emit the sby line, so
keeping it means keeping a fork. The condition it existed for no longer holds: it bounded a query
that did not converge under `smtbmc yices`, and ADR-0024 replaced that engine with one under which
the same check at the same depth returns in ~8–12s. The nightly's job-level `timeout-minutes`
remains the backstop. **This is the one place this ADR accepts a real reduction in protection**, and
it is recorded as such rather than argued away: if some future check hangs, the nightly loses the
whole job rather than one check's status, exactly as it did before ADR-0022. Restoring a per-check
bound means either forking `genchecks` again or teaching `formal/Makefile`'s `check` target to wrap
each `sby` invocation — the latter is the cheaper answer and the one to reach for. **ADR-0033
decides it that way**, and records the three things that make the wrapper less trivial than it
looks.

**3. The ten newly-generatable `fault`/`bus_*` checks are evaluated and deliberately not adopted.**
None appears in `checks.cfg`'s `[depth]`, so `genchecks` skips all ten. **0 of 10 are applicable as
the core stands**, for three distinct reasons, recorded in full at the point a reader would add them
(`formal/checks.cfg`, above `[depth]`):

- All nine `bus_*` need `RISCV_FORMAL_BUS` — a second RVFI interface of nine signals
  (`bus_valid`/`insn`/`data`/`fault`/`addr`/`rmask`/`wmask`/`rdata`/`wdata`). The core drives none.
- Every `*_fault` variant models a bus that can refuse a transaction. This core's bus cannot:
  `imem_data`/`imem_data2`/`mem_rdata` are free every cycle with no fault line and no handshake
  (invariant 1, ADR-0015, `formal/wrapper.v`). More decisively, a bus fault is by construction
  detected *after* decode, and **invariant 2 says nothing faults after decode**. These check a trap
  model this design has ruled out; they need an ADR before they need a depth.
- The three `io` ones additionally need `rvformal_addr_io`, a distinguished MMIO region with
  ordering semantics. ADR-0008's map has a `tohost` word, not an IO region the RTL treats
  differently, and there is no ordering to check with one outstanding access at a time.

`bus_imem` and `bus_dmem` are the only two whose property is meaningful here — and
`formal/imemcheck.sv` and `formal/dmemcheck.sv` already check it against the real split
`imem_addr`/`imem_addr2` interface (ADR-0003), and both pass. Adopting the `bus_*` forms would mean
plumbing nine RVFI outputs to re-derive a result already held, and would put an `` `ifdef
RISCV_FORMAL `` value on a path the core reads — precisely what ADR-0020's non-perturbation argument
depends on nothing doing.

**The `csrc_*` checks are now generatable and are still not on the ladder.** Adding one is a
follow-up needing its own derived depth (ADR-0025) and `formal/EXPECTED_FAIL` entry (ADR-0022), and
`mcycle`/`minstret` are not implemented (M3). `csrc_inc_minstret` is the one to reach for first.

## Consequences

The ladder is unchanged where it counts. The re-vendor reached the generated `.sby` files in exactly
two ways, and this was measured by diffing all 79 generated files against a from-scratch baseline
taken immediately before the change:

- every check's script line reads `read -sv X.sv` where it read `read_verilog -sv X.sv` — upstream's
  frontend-agnostic spelling, which without Verific dispatches to `read_verilog -sv`;
- `reg_ch0.sby` lost its `timeout 1800` line.

**Same 78 checks, same names, same verdicts: 67 pass, 11 non-PASS matching `formal/EXPECTED_FAIL`
exactly.** As always, every check is `mode bmc`: a PASS means *no counterexample was found within
that check's configured depth*, not that the property holds. The whole ladder still runs under
`RISCV_FORMAL_ALTOPS`, so a green `insn_mul` still says nothing whatever about the real multiplier
(ADR-0010). This ADR moves M2 no closer; it removes an obstacle in front of the checks that would.

Re-syncing after a future pin bump (ADR-0013) is now a documented three-line recipe in the script's
own header, and the acceptance test for it is mechanical: the diff against the clone must show only
the header and the two `basedir` hunks. Anything else is drift, and drift here is what produced the
situation this ADR exists to end.

The cost is that this repo now carries upstream code paths it does not exercise — `csr_spec`,
`buslen`, `nbus`, `abspath`, `custom_csrs`, `illegal_csrs`, `verilog-files`/`vhdl-files`. That is
the point: keeping them unedited is what makes the file byte-comparable with upstream, and what
makes the next re-sync a copy rather than an archaeology exercise.
