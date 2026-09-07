# ADR-0169: A local yosys area instrument stands up for nanocpu

**Status:** Accepted · 2026-09-07

## Context

`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md` (the brief) puts
"instruments before design" first in its sequence: nothing about a 2x2-tile
core can be measured until an area number exists to compare against. The
brief's own donor figure is **84,291 um2** at `synth; dfflibmap -liberty;
abc -liberty` against the real `sky130_fd_sc_hd__tt_025C_1v80` liberty,
yosys 0.68 — above the shuttle's demonstrated **56,448 um2** 2x2 envelope
(FazyRV-ExoTiny, TT06) on purpose, because the reshaping steps the brief
lays out (RV32I → RV32E, drop M, the QSPI front end) are what are expected
to close that gap. `nano/nano.v`, the donor import, is a separate ticket
running in parallel and does not exist in this tree yet.

## Decision

**A local instrument, not the TT flow itself.** `make nano-area` runs
`synth; dfflibmap -liberty; abc -liberty; stat -liberty -json` against the
pinned sky130hd liberty and grades the result — no placement, no STA,
nothing LibreLane's own flow later does. This is the same relationship
`make fit` has to a real ice40 place-and-route: a fast, hermetic, toolchain
this repo already runs everywhere proxy for a flow that is slower, needs a
separate install (LibreLane, OpenROAD, a full PDK), and is out of scope for
an instrument ticket. The brief's own calibration — FazyRV-ExoTiny reads
61,673 um2 in this shape of flow and shipped at 56,448 through LibreLane,
about 0.915x — is how the two numbers are meant to be read against each
other: never merged, the same rule that keeps `make fit`, `make soc-timing`
and `make ecp5-timing` apart. Standing up the real TT-flow/LibreLane action
is follow-up work, not this ticket's.

**The liberty is a pin.** `nano/nano.mk` fetches
`sky130_fd_sc_hd__tt_025C_1v80.lib` from a 40-hex commit on
OpenROAD-flow-scripts, SHA-256-verified before anything reads it, the same
shape as `formal/pin.mk`'s riscv-formal pin and the Makefile's Sail/svlint
pins. `make nano-liberty-setup` writes it under `$(TOOL_CACHE)/sky130`,
outside the checkout, and `test/tool_cache_test.sh` now checks that
location the same way it checks the Sail and svlint installs.

**The ratchet starts at the donor's own measured figure, honestly.** The
brief's 84,291 um2 is a real number from a real yosys run, but it was not
taken on this tree — `nano/nano.v` does not exist here to re-measure a
churn band from the way `FIT_MAX_LC`'s derives one from six functionally
identical spellings of `rtl/littlecpu.v`. Fabricating a churn band with no
run behind it would be exactly the "inherited conclusion" CLAUDE.md warns
against, so `NANO_MAX_UM2 := 84291` is the brief's figure with no invented
margin. It starts the design legitimately over the 56,000 um2 shuttle
envelope and is expected to stay red against that final number through
several reshaping steps — this is the option the ticket calls "a ratchet
that starts at the current measured value and steps down as the reshaping
lands," not "set the ceiling to 56k and leave it red." Each reshaping step
re-takes `NANO_MAX_UM2` the way `FIT_MAX_LC` gets re-taken, starting from a
real `make nano-area` run rather than the brief's number, until it reaches
the 56,000 um2 target the brief and CLAUDE.md's eventual ratchet cite.
`area_report.py` takes an optional `--previous` to print a trend line once a
step actually has a distinct prior figure to compare against; nothing passes
it yet, since `NANO_MAX_UM2` has no history of its own before this ADR.

**Not wired into `make test`.** The same standing `make fit` and
`make soc-timing` have — a real instrument, run and graded on its own, but
not a prerequisite of the suite. `make nano-liberty-setup` needs a network
fetch the way `make sail-setup` does, and `nano/nano.v` does not exist in
this tree at all, so making `nano-area` part of `make test`'s dependency
graph would either force every contributor to fetch a PDK-adjacent liberty
file for an unrelated change, or hard-fail the whole suite on a file a
parallel ticket has not landed yet. Neither is acceptable; CLAUDE.md's own
instrument list keeps `fit`, `soc-timing` and `ecp5-timing` off `test` for
the same reason.

**The missing-donor case is a coordination point, not an error.** `nano-area`
opens with a `test -e $(NANO_SRCS)` guard, the same shape `noop-rom` uses for
its own "something should have written this first" check, rather than a
parse-time `$(wildcard)` split into two competing target bodies. Absent, it
prints why and exits 0; present, the same recipe runs `synth` straight
through. Once `nano/nano.v` lands, in a separate ticket, this file needs no
edit.

## What this does not settle

**No GDS, no STA, no LibreLane run.** This instrument measures synthesis
area only, the way `make fit` measures logic cells only; the brief's own
freeze criteria (TT-flow synthesis ≤ 56k, the GDS action clean, STA at
64 MHz) still need a real LibreLane/GDS CI action, deferred to when the
design is closer to freeze. **No `nano/nano.v` exists yet** — `make
nano-area` was smoke-tested against a throwaway one-flop scratch module in
this session (not committed) to confirm the full path — liberty fetch,
`synth`, the ratchet's pass and fail arms, and the missing-source guard —
runs green and red for real, not only against `test/probe_gates.sh`'s
fixtures. The donor import is a separate ticket's.

## Amendment · 2026-09-07 · the donor landed the same day, and the instrument was run against it

The donor import this ADR defers to merged first, so `nano/nano.v` now exists
and the ratchet has been run against the real thing for the first time, on
yosys 0.68+post (git sha1 c12172f) against the pinned liberty:

    RATCHET: 84290.8 of 84291.0 um2 budgeted -- OK

The instrument composes — liberty fetch, `synth`, `dfflibmap`, `abc -liberty`,
`stat -liberty -json`, and the ratchet's pass arm all run end to end on the
donor, not on a scratch module — and the brief's 84,291 um2 reproduces to
within 0.2 um2 on a different machine and toolchain than the brief's.
Reproducing that closely is the finding: it says the brief's figure and this
recipe measure the same thing.

**It also leaves 0.2 um2 of margin, and that is not a band.** This ADR declined
to invent a churn band because there was no donor here to measure one from.
There is now, and the number above is a single run — the first this repo has
taken. `FIT_MAX_LC` budgets a whole measured span for exactly this reason
(ADR-0142), and a budget set to one run's value trips on the next
functionally-neutral respelling or toolchain bump with nothing wrong with the
design. Deriving that span — six functionally identical spellings of
`nano/nano.v`, the way `FIT_MAX_LC`'s derivation was taken — and re-setting
`NANO_MAX_UM2` to cover it is a follow-up, not this ADR's. Until it is taken,
read a `nano-area` failure as "re-derive the budget," not "the design grew."
Nothing is gated on it: `nano-area` is off `make test` and off CI.
