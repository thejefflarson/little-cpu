# ADR-0224: nano instantiates its own clock gates rather than asking the flow to

**Status:** Accepted · 2026-10-01 · supersedes ADR-0219's "no clock gating" decision; the rest of ADR-0219 stands

## Context

The 4×2 Tiny Tapeout tile is a hard requirement for nano, and the register file is what stops it fitting. After Tier 3, with the register file cut to one register, the flow routed a 4×2 clean (run 36766659352 at placement density 60: 56,089 µm² of synthesis, met1 77.4% and total 56.7% routing demand, 0 violations, 0 DRC/LVS/antenna; run 36766667951 at density 84 also 0). The same RTL with the full register file (run 36753853585, density 84) had 76,054 µm² of synthesis, met1 92.8% and total 84.7%, and stayed near 68,000 violations until the six-hour timeout. The register file's 480 enabled, unreset flip-flops are what a clock gate unburdens: each bit otherwise needs a multiplexer feeding its own Q back, 480 cells and 960 short nets on the most congested layer.

ADR-0213 got there by turning on LibreLane's `clockgate` pass, and ADR-0219 turned it off because yosys 0.62, the version LibreLane 3.0.14 bundles, gates a flop whose synchronous reset outranks its enable (`$_SDFFE_*`) on the enable alone, so reset never lands while the enable is low. Upstream fixed that in `f4a10a4808`, first released in 0.65. No Tiny Tapeout project uses the flow's `SYNTH_CLOCKGATE_*` settings.

Hand-instantiated `sky130_fd_sc_hd__dlclkp` cells are in shipped, working sky130 silicon: TinyQV on ttsky25a has 39 `dlclkp_1` and runs CoreMark and MicroPython at 64 and 90 MHz; FemtoRV on ttsky25b has one `dlclkp_4` and works at 80 MHz; TinyQV 3x2 on ttsky25b has 37. They instantiate the cell in `latch_reg.v` and model it for simulation.

## Decision

**nano instantiates its clock gates in the RTL, and the flow's own pass stays off.** `nano/tt/src/config.json` is the template's, with no `SYNTH_CLOCKGATE_*` key.

`nano/nano.v` carries two small modules. `nano_gated_reg` is a register that loads `d` on the clock edges where `enable` is high; `nano_gated_reg_r` adds a synchronous reset by making reset part of the enable and the reset value part of `d`. Under the flow's `SCL_sky130_fd_sc_hd` define (LibreLane's `synthesize.py` passes it, as TinyQV's `latch_reg.v` keys on it) a gated register is one `sky130_fd_sc_hd__dlclkp_1` with GATE = `enable` and CLK = `clk`, driving plain flops on GCLK. In every other build, which means cxxrtl, iverilog and every formal proof, it is an ordinary `always_ff @(posedge clk) if (enable) q <= d`. The two arms read the one `enable` port, so what a simulator or a proof checks about the enable is the signal the gate sees. This is the answer to cxxrtl's inability to fire an `always` block on a derived clock (`nano/tb`, ADR-0215): the sim legs stay on `clk`, and the real gate is covered by gate-level simulation (`make nano-gl-test`), which reads the PDK's own `dlclkp_1` model.

A gated register with a reset needs reset in its enable, because a gated clock does not tick while the enable is low; `nano_gated_reg_r` does that for every caller. This is the property yosys 0.62's pass got wrong for `$_SDFFE_*`, and a hand-placed gate makes it a line in one module rather than a property of a synthesis pass's version.

### What is gated

One gate per register for the register file (15), and one gate for each other group below. Measured on `make nano-area` (local yosys, the PDK's excluded cells removed, `dlclkp_1` allowed, µm²), each row on top of the one above:

| group | gates | area | change |
| -- | -- | -- | -- |
| none (main) | 0 | 75,082.0 | |
| register file, x1-x15 | 15 | 69,595.5 | −5,486.5 |
| `op_rs1` | 1 | 69,275.2 | −320.3 |
| CSRs: `mcycle`/`minstret` halves, `mscratch`, `mtvec`, `mepc`, `mcause`, `mstatus`, `mie` | 10 | 68,132.8 | −1,142.4 |
| `pc` and `instr` together | 1 | 66,998.0 | −1,134.8 |

27 gates, −8,084.0 µm² (−10.8%), 1,044 flip-flops unchanged. Tried and left out because the local figure went the wrong way: `mem_addr` and `mem_valid` (the enable has to be a separate next-address block, +929), and the QSPI controller's `tx_shift`, `stream_next_addr`, `rx_shift` and `psram_rmw_result` (their loads are spread through the state machine, so a gate means splitting it into next-value and load signals, +303 for the split and +1,150 more for the gates). The local instrument is a ranking: it does not place, route or time. Whether 27 gates reach the 4×2 is the flow's to say.

### The instrument and the netlist check

`nano/synth_script.sh` and `nano/timing_script.sh` define `SCL_sky130_fd_sc_hd` and read the liberty as a blackbox library so the gates survive synthesis, and keep the excluded-cell list. The PDK lists `dlclkp_1` as excluded from synthesis, but a hand instantiation bypasses `dfflibmap` and `abc`, as the shipped designs show, so `nano/area_report.py` accepts exactly that one cell and still refuses every other excluded cell, `dlclkp_2` included.

`nano-gl-test` requires clock gates again: `gl_census.py --require dlclkp`, so a flow that stopped defining the SCL macro would not pass an ungated netlist. `nano_gl_gate_probe.sh` forces a real `dlclkp_1`'s GATE low in a fixture and requires its counter to stop, and `nano_gl_stuck_gate_probe.sh` ties the 15 register-file gates of the real netlist low and requires the gate-level test to fail. `make nano-gl-local` builds a netlist with the local yosys through `flatten`, `synth`, `dfflibmap` and `abc` (`nano/flow_netlist_script.sh`) and runs the test on it with no flow run.

## Consequences

- A fit still needs all four of ADR-0219's conditions: the flow routes clean at 4×2, meets timing, and `nano-gl-test` passes on that run's own routed netlist. This ADR does not claim one.
- Hold into a gated register is the named risk: the gate's output arrives later than `clk`, and if clock-tree synthesis does not balance through it the hold repair eats the saving. Read `RSZ-0046` and the delay-cell census on every run.
- A register whose enable is not a simple load, and whose RTL then has to be split to name the enable, costs more than the gate saves on this instrument; the rule for adding a group is to measure it here first.
- The decision to keep the flow's pass off stands until Tiny Tapeout's action installs a LibreLane whose yosys includes `f4a10a4808`, and even then hand placement keeps the enable readable at the register.
