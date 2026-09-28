# ADR-0215: nano's hardened netlist gets a gate-level simulation

**Status:** Accepted · 2026-09-27

## Context

ADR-0213 turned on clock gating in nano's LibreLane flow: `sky130_fd_sc_hd__dlclkp_1`
cells exist only in the netlist that flow produces. RTL simulation on both legs and every
riscv-formal check read `nano/nano.v`, which never mentions a gating cell, so nothing in
the tree could see whether gating actually worked, worked with the right polarity, or
existed at all. ADR-0163 is this repo's own precedent for the gap: a block RAM whose
reset was driven by logic passed every RTL simulation and every mapped-netlist census, and
only a real program run on the mapped netlist found that every read came back zero.

## Decision

**A hardened netlist is gate-level simulated through the existing pins-only test.**
`make nano-gl-test NETLIST=<path>` builds `nano/tb/nano_tt_tb.v` (already single-clock
throughout, so a netlist full of independently gated clocks runs under iverilog the same
way the RTL does) against the given netlist, `nano/tb/nano_qspi_flash_model.v` and
`nano_qspi_psram_model.v`, and the sky130_fd_sc_hd behavioral cell models, then runs the
same `tt_gpio_uart.S` program `run_nano_tt_test.sh` already runs against the RTL.
`nano/gl_census.py` first parses the netlist's own text for cell instantiations and
refuses one with zero `dlclkp` instances -- the silent case where
`SYNTH_CLOCKGATE_MIN_WIDTH` is set but the pass never ran or never matched anything.

**The cell models are fetched and verified, not read from the workflow's own PDK cache.**
`nano/nano.mk` pins a commit of `google/skywater-pdk-libs-sky130_fd_sc_hd` (the library's
own source, main branch static since 2020) and a SHA-256 of its tarball, the same two-pin
shape `NANO_LIBERTY_COMMIT`/`NANO_LIBERTY_SHA256` already uses; `nano/sky130_verilog_setup.sh`
fetches it, verifies it, and flattens `cells/**/*.v` and `models/**/*.v` (every basename is
unique across the whole library) into one directory, rewriting their `` `include ``
directives to match, since Icarus resolves an include by search path rather than by the
including file's own directory. The workflow's `PDK_ROOT` cache is a LibreLane/volare
artifact keyed by LibreLane's own version string, not by a hash of its contents, and it
does not exist outside that one job -- `make nano-gl-test` has to work on a laptop with no
PDK installed at all, so it needs its own pin regardless of what the workflow restores.
Building against `` `define FUNCTIONAL `` (no `USE_POWER_PINS`) is required: sky130's
default `.behavioral.v` variant leaves two internal delay wires undriven without a
`specify` block this flow never generates, which reads as a permanently unknown `GCLK` on
every gating cell; `.functional.v` wires the same gates directly to the real ports and is
the standard no-SDF gate-level simulation model.

**Two forced-red probes, both prerequisites of `nano-gl-test`.** `nano-gl-gate-probe`
builds `nano_gl_gate_probe_fixture.v` -- one real `sky130_fd_sc_hd__dlclkp_1` gating a
counter -- runs it once as shipped (must PASS, the control) and once with its `GATE`
connection mutated to a constant `1'b0` (must FAIL, since a real dlclkp's output clock
does not toggle while gated). `nano-gl-census-probe` feeds `gl_census.py` two synthetic
text fixtures, one with a `dlclkp` instantiation and one without, requiring the first to
pass and the second to be refused for exactly that reason. Both probes need a real tool
(iverilog and the pinned cell library) the way `formal/decoder-zkt-probe.py` needs real
yosys, so they stay Makefile prerequisites of their own target rather than entries in
`test/probe_gates.sh`: that script's `test/PROBES_EXPECTED` manifest is for probes that
run with no network fetch on every `make test`, and `nano-gl-test` is deliberately off
that path (below).

**`nano-gl-test` is off `make test`'s path, the same standing as `nano-area`.** It needs a
network fetch (the sky130 verilog tarball) `make test` does not otherwise require, and,
unlike `nano-area`, it also needs a real hardened netlist that only exists after a
LibreLane run -- there is nothing to point it at in ordinary CI or on a fresh checkout.

**The self-hosted workflow gates on it, rather than only publishing a result.** After
hardening, a new step looks for `nano/tt/runs/wokwi/final/nl/*.nl.v` (`always()`, since a
routing run the job's timeout cancelled produces no `final/` view at all, and that is a fact to check rather than assume from the harden
step's own outcome). When one exists, `make nano-gl-test` runs and a failure fails the
job: an unverified hardened netlist reaching tapeout is exactly the risk this whole change
exists to close, and a report nobody is required to act on is not a check. When none
exists, a summary line says so plainly rather than staying silent. The step needs
`riscv-none-elf-gcc` and `iverilog`, so the workflow gains the same two composite-action
steps (`setup-riscv-gcc`, `setup-oss-cad-suite`) the ordinary `test` job in `ci.yml`
already uses on the same runner pool.

## Measurement

The mechanism is proved against real tooling short of a full chip run: `nano-gl-gate-probe`
and `nano-gl-census-probe` both pass, each demonstrating its control and its forced-red
mutant for the reason the probe names. `make test` and `make probe-gates` are unaffected
(`nano-gl-test` is off both paths, as stated above).

The first real netlist found a defect the probes could not. A `6x2` synthesis-only
dispatch (run 36410116279) wrote `final/nl/tt_um_thejefflarson_nanocpu.nl.v` with 45
`dlclkp_1`, and `nano-gl-test` failed to elaborate it: 6,419 `Unknown module type` errors,
one per cell instance. The run passed `-I` alone, which only resolves `` `include ``, and
the probe's fixture `` `include ``d its one cell, so the probe elaborated and the netlist,
which includes nothing, could not. Both now pass `-y "$CELL_DIR" -Y .v`, which resolves
each cell as a library module, and the fixture no longer includes its cell, so the probe
exercises the path a netlist uses. Both also pass `-D UNIT_DELAY=`, which the models'
flip-flops read and iverilog warned about when left undefined.

Against a local clock-gated netlist (yosys `synth`, then the flow's `clockgate` pass, with
`dfflibmap` and `abc` told not to use `edfx*` the way LibreLane's cell exclusions do, 27
`dlclkp_1`), `nano-gl-test` elaborates warning-free and prints `PASS`; `vvp` takes 1.4 s
and 72 MB peak resident on an M-series Mac. The same netlist with every `dlclkp_1`'s `GATE`
tied to `1'b0` fails with `uio_oe is X`, so the check can go red on a real netlist and not
only on the fixture. No routed netlist has been tested yet: a 4×2 `AREA 2` full-flow
dispatch on this branch (run 36377874205) reached the job's 360-minute limit in detailed
routing with 46,828 violations after seven iterations and wrote no netlist.

## Consequences

- `nano-gl-gate-probe`'s fixture is intentionally not a slice of the real chip: forcing one
  specific gate low inside hundreds of auto-named cells in the real netlist is not a stable
  text mutation across synthesis runs, so the probe proves the mechanism (a stuck-low real
  dlclkp cell is caught) rather than mutating the shipping netlist itself.
- `make nano-area`'s local instrument runs the same `clockgate` pass, so its netlist has
  the gates this check exists for, but it is never simulated: this check verifies the
  flow's own output, not `nano-area`'s.
- iverilog is the only leg that can run this: cxxrtl cannot fire an `always` block on a
  clock a synthesis pass derived, and riscv-formal has no model of the netlist at all.
