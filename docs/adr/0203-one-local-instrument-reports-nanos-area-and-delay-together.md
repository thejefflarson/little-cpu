# ADR-0203: one local instrument reports nano's area and delay together

**Status:** Accepted · 2026-09-20

## Context

`make nano-area` answers "how big" in about four seconds; nothing local answered "how
fast" at all. The only timing figure that existed anywhere was `nano-tt-area-selfhosted`,
LibreLane's real synthesis-and-place-and-route flow on a self-hosted runner, which takes
on the order of an hour. Ranking an RTL change on speed meant either skipping the
question or spending that hour, so it went unasked most of the time.

A three-second ad-hoc command, run once outside the tree, showed a proxy was cheap:
`abc -liberty <lib> -script +strash;dch,-f;map,-B,0.2;topo;stime,-c` prints ABC's own
mapped critical-path delay from the same kind of synthesis `nano-area` already runs. That
command's own `Delay =` line is a different quantity from `nano-area`'s ratchet-grade
area — the two never merge into one figure, the standing rule for every instrument pair in
this repo.

## Decision

**`make nano-timing`, a second local instrument beside `nano-area`, not a replacement for
it.** One yosys run per register-file build (`nano/timing_script.sh`) does
`synth; dfflibmap -liberty; techmap -map <latch stub>; abc -liberty <lib> -script
+strash;dch,-f;map,-B,0.2;topo;stime,-c; stat -liberty -json` — area and delay both come
out of that one run, so ranking two RTL versions costs one synthesis rather than two.
`nano/timing_report.py` reads the ABC `stime -c` line and the `stat -liberty -json`
output and prints both, per build, with the tool version, the liberty path and the exact
recipe next to the numbers. No ratchet: `NANO_MAX_UM2` stays the only gated area figure,
and this target can never fail on a value, only on a run that measured nothing.

**The area printed here is not `nano-area`'s area.** `nano-timing`'s ABC script
(`dch;map -B 0.2;topo`, aimed at delay) maps the same source to a different netlist than
`nano-area`'s plain `abc -liberty` does — on this tree, 46,756.09 µm² against 45,712.6 µm²
for the flops build. The report says so next to the number rather than leaving a reader to
assume the two are the same instrument twice.

**A latch left generic is priced at zero, and this instrument is the first thing that
measured the latch build closely enough to notice.** `dfflibmap` maps flip-flops only;
nothing in this repo had ever run `NANO_LATCH_RF` through a local yosys synthesis before,
so nobody had hit the fact that a `$_DLATCH_P_` yosys hands to `abc -liberty` unmapped
comes back priced at zero by `stat -liberty` rather than refused, the same failure mode
`nano/area_report.py`'s own cell-type check exists to catch for flip-flops. Read live on
this tree: the latch build's `stat` area was **30,251.51 µm²** with 480 `$_DLATCH_P_`
cells silently priced at nothing, and **37,458.43 µm²** once they were mapped —
a 7,207 µm² gap that would have shipped as the register file's real "local timing" area if
`nano-timing`'s own unknown-cell-type check had not caught it (it now does, sharing
`nano/area_report.py`'s `check_liberty` with a caller name it can quote correctly).

The fix is `flow/platforms/sky130hd/cells_latch_hd.v`, the same OpenROAD-flow-scripts
commit's own two-line techmap stub mapping `$_DLATCH_P_`/`$_DLATCH_N_` to
`sky130_fd_sc_hd__dlxtp_1`/`dlxtn_1`. It is pinned the same SHA-256-verified way the
liberty already was, from the same commit, into the same cache directory, and
`nano/nano.mk`'s `nano-liberty-setup` fetches both — `techmap -map` on that stub is a
measured no-op on the flops build (46,756.0928 µm² either way), so both register-file
builds run the identical script.

**One corner, not two, and the report says so where a reader will see it.** The ticket
asked for `sky130_fd_sc_hd__ss_100C_1v60.lib` pinned the same way as the typical corner.
It cannot be: the pinned OpenROAD-flow-scripts commit's `sky130hd` platform, checked at
that commit and at the project's current `master`, carries only the typical-corner
liberty at `flow/platforms/sky130hd/lib/` — no slow or fast corner ships there as a
standalone file. Every other trace of `sky130_fd_sc_hd__ss_100C_1v60.lib` found across a
broad public code search (LibreLane's own test fixtures and docs, `hammer`, `DFFRAM`)
names a path under a full `sky130A` PDK install — the multi-hundred-megabyte bundle
`nano-tt-area-selfhosted` fetches via `librelane --pdk-root`, which is exactly the class
of dependency ADR-0169 already declined for a local instrument ("no GDS, no STA, no
LibreLane run... a full PDK... out of scope"). Fetching a PDK bundle to read one file out
of it is not a fast local instrument; it is the flow this instrument exists to stand in
for. `nano-timing` reports the typical corner only, and prints that limit beside every
number rather than only here. **Closing the 4×2's slow-corner setup violations ADR-0197
already measured still needs the real flow** — this instrument cannot rank that work,
only the typical-corner area/delay trade this pair of numbers already covers.

**The flow correlation is recorded, not re-taken.** `nano/timing_flow_correlation.json`
carries the one pair this tree has: `make nano-area` read 63,565.96 µm² against
LibreLane 3.0.5's 79,862.84 µm² (`ttsky26c`), a 1.256× factor, on the RV32IM tree before M
was dropped and before the register file moved to latches or a single read port.
`make nano-timing` prints it every run, marked STALE, because `make nano-area` no longer
reads 63,565.96 µm² on this tree and the two halves of a correlation are only a
measurement together, on one tree. Re-taking it needs a fresh `nano-tt-area-selfhosted`
run, on the order of an hour on a self-hosted runner neither this change nor this
environment can dispatch; that re-take is follow-up work, not fabricated here.

## What this does not settle

No slow corner, no placement, no routing, no sign-off, no `NANO_ONE_PORT_RF` coverage
(that lever is already declined, ADR-0195). The flow-correlation pair is a citation with
a date, not a fresh measurement. `make nano-timing` is off `make test`'s path, the same
standing `nano-area` has, since it needs the same liberty fetch and the same donor-not-
landed guard.
