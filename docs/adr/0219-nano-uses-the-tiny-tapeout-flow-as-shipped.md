# ADR-0219: nano uses the Tiny Tapeout flow as shipped

**Status:** Accepted · 2026-09-30 · supersedes ADR-0213

## Context

ADR-0213 turned on LibreLane's clock gating (`SYNTH_CLOCKGATE_MIN_WIDTH` 8, a `dlclkp_1` ICG) to
take nano's routing demand on a 4×2 tile from 145.5% to 88.3%. Tiers 1 and 2 then took it to 58.5%,
and run 36568965575 on main at 7b111c6 routed with zero DRC, LVS and antenna errors.

That run's netlist did not work. `nano-gl-test` (ADR-0215) found `uio_oe` unknown 635 ns after reset.
Traced through the routed netlist (run 36619040652, which uploads it):

- `core.next_pc`'s 32 flops share one clock gate whose enable holds a known 0 for all of reset, and
  `rst_n` appears nowhere in that enable's combinational cone. The RTL's `next_pc <= 0` never gets
  a clock edge, and the first fetch is `mem_addr <= next_pc`, so on silicon the core fetches its
  first instruction from whatever `next_pc` powers up holding.
- 13 of the flow's 35 clock gates have an enable that reset does not reach.

The cause is in yosys, not in nano. A synchronously reset flop with an enable (`$_SDFFE_*`, reset
over enable) may only be gated on enable OR reset. yosys 0.62's `clockgate` gates it on the enable
alone. Commit `f4a10a4808` ("clockgate: reject $sdffe for correct priority handling") fixes that
by not gating such flops at all, and first shipped in 0.65. LibreLane 3.0.14, the latest stable
release and the default in Tiny Tapeout's `tt-gds-action`, bundles 0.62 (sha 7326bb7d6). Before
the flow's `CLOCK_GATE` pass, 461 of the 1,127 flops it gated were `$_SDFFE_*`. With yosys 0.68
the same RTL leaves them ungated, which is why local synthesis never showed the defect.

Nothing but a gate-level simulation of the flow's own netlist could see it. The formal checks and
both simulation legs read the RTL, the local area instrument ran a different yosys, and LVS
compares the layout with the netlist, never the netlist with the RTL.

The local instrument had a second, independent gap. It let `dfflibmap` and `abc` use every cell in
the liberty file, while LibreLane excludes the union of open_pdks' `no_synth.cells` and
`drc_exclude.cells`: every `_1` drive strength, `edfxtp_1` among them. On main 5,962 of the local
netlist's 7,298 cells were ones the flow never uses, 674 of them `edfxtp_1`. That is why ADR-0216
measured −10k µm² locally and about −2.5k in the flow.

## Decision

**nano is hardened with the Tiny Tapeout flow as shipped**: LibreLane 3.0.14 through
`tt-support-tools`' `main`, the PDK's own cell lists, and no clock gating. `config.json` loses
ADR-0213's two keys and gains no replacement. A change to what the flow does, as opposed to a
change to the RTL it is given, needs a defect the default flow has and a check that sees what the
change does.

**Size comes from the design.** The tile count is whatever a default-flow run of the RTL on main
routes with DRC, LVS and antenna clean, meets timing at the target clock, and passes
`nano-gl-test` on its own routed netlist. Nothing short of all four is reported as a fit.

**The local instrument measures the cells the flow allows.** `nano/synth_script.sh` and
`nano/timing_script.sh` drop `clockgate` and take the excluded-cell list, which `make
nano-liberty-setup` builds from open_pdks' two files at commit `8afc8346`, the commit LibreLane
resolves, each pinned by SHA-256. Both scripts refuse a missing list rather than measure without
it. The same RTL now reads 78,965.7 µm² where it read 58,790.1 with clock gating and every cell
allowed, and `NANO_MAX_UM2` moves to 81,000. It still runs yosys 0.68 and yosys's generic `synth`
rather than LibreLane's pass sequence and its `AREA 2` ABC script, so it ranks one RTL version
against another and never says whether a design fits.

**`nano-gl-test` no longer requires a clock gate.** `gl_census.py` loses `--require` and keeps
refusing a file with no sky130 cells, and its probe now requires RTL to be refused. The gate
probe's fixture becomes an enabled `dfrtp_1` behind a `mux2_1`, which is how every enabled
register reaches a default-flow netlist, and still clocks through `buf_1` and `buf_2`.

## Consequences

- The 4×2 fit ADR-0213 and ADR-0216/0217 reported was a fit of a netlist that did not work, and
  it is withdrawn. A default-flow run at 4×2 and at 6×2 on the main that carries this change
  decides the tile count.
- Clock gating becomes available again when Tiny Tapeout's action installs a LibreLane whose yosys
  includes `f4a10a4808`. Even then, flops that yosys rejects stay ungated, so the gain is smaller
  than ADR-0213 measured, and `nano-gl-test` has to pass on that run's netlist before a fit is
  claimed.
- Overriding the PDK's cell lists to allow `edfxtp_1` was considered and not tried: it is another
  change to the flow rather than to the design, and the lists' reason for excluding `_1` cells is
  a drive-strength policy this repo has not measured.

## Amendment 1 (2026-09-30): the flip-flop exclusions did not take effect

The instrument as first merged passed each excluded cell to `dfflibmap` and `abc` as
`-dont_use "name"`. `abc` strips the quotes; `dfflibmap` matches the name literally, quotes
included, so no flip-flop was ever excluded. On Tier 3's tree the result still held 577
`edfxtp_1` and 467 `dfxtp_1`, both of which the flow forbids. The probes graded only the
script's text, so none could see it.

Both scripts now pass the names bare, and refuse a list holding anything but a
`sky130_fd_sc_hd__` cell name. `nano/area_report.py` and `nano/timing_report.py` now read the
list too and refuse a report that uses any cell on it. That grades the outcome, and a probe
forces it red.

With the exclusions in effect, the same RTL (main after Tier 3) reads 75,082.0 µm² against
73,137.6, with every enabled flop now a `dfxtp_2` behind a `mux2_1`, which is what the flow
builds. `NANO_MAX_UM2` moves to 77,100. The figures in this ADR's decision and in ADR-0218's
table were taken with the flops unexcluded. Their ranking holds, since every row used the
same instrument, but their absolute values do not.

