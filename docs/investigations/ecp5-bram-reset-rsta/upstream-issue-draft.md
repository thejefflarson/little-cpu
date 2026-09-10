# Draft issue -- NOT FILED

Target: YosysHQ/yosys. A human should read this, decide whether to send it,
and file it themselves; nothing in this repo files it automatically.

---

## Title

ecp5: does DP16KD's synchronous read-port reset (RESETMODE=SYNC) actually
clear the read data every cycle under REGMODE=NOREG, or only intermittently?

## Body

We hit a defect on a MuseLab iCESugar-Pro (LFE5U-25F) where a block-RAM-backed
data memory returned zero from every address on real silicon, while RTL
simulation, the `synth_ecp5` cell census, and `nextpnr-ecp5` placement and
timing all looked correct. The narrative and the fix are here:
<https://github.com/thejefflarson/little-cpu/blob/main/docs/adr/0163-a-block-ram-read-through-its-own-reset-returns-zero.md>

The RTL was

```verilog
always @(posedge clk)
  if (in_range && write) ram[index] <= wdata;
  else if (!write)       rdata <= in_range ? ram[index] : 32'b0;
```

which `synth_ecp5` maps onto `DP16KD`'s own read-port reset: `RESETMODE=SYNC`,
`REGMODE_A=REGMODE_B=NOREG`, and the reset pin (`RSTA` or `RSTB`, whichever
port the pass picks) driven from `~in_range` rather than tied to a constant.
The fix was to stop writing the idiom that way -- register the read and the
range check separately, and select between them with an explicit mux on the
block's output -- which is a **correctness fix that costs LUTs**, not a
preference.

We have not been able to test on a board again since (this report is written
from a machine with no board attached), but we did narrow the two
open-source tools against each other as far as we could without one. A
four-variant reproducer sits at
<https://github.com/thejefflarson/little-cpu/tree/main/docs/investigations/ecp5-bram-reset-rsta>
(each one `always` block over a single 512x32 array, exactly one `DP16KD`).
Its finding: **`nextpnr-ecp5`'s placed `--textcfg` output is a faithful,
unmodified copy of every relevant field in `synth_ecp5`'s JSON netlist** --
`REGMODE_A`/`REGMODE_B`, `RESETMODE`, and whether `RSTAMUX`/`RSTBMUX` route a
real net or tie off -- and this holds identically whether the `DP16KD`/
`PDPW16KD` cell is produced by `memory_bram`'s inference or by a hand-written
primitive instantiation that never runs that pass at all. We could not find
anything in `nextpnr-ecp5` that treats `RESETMODE=SYNC` combined with
`REGMODE=NOREG` and a logic-driven reset pin as unusual -- it places it the
same way it places every other legal-looking parameter combination.

So the two remaining questions, which need either Lattice's own EBR
configuration guide or a board, and we have neither on this machine:

1. **Does `RESETMODE=SYNC` actually clear the read output on every cycle it
   is asserted, under `REGMODE=NOREG`** -- the mode `synth_ecp5`'s own
   default parameterization reaches for whenever the extra output-register
   pipeline stage is not otherwise requested -- or is the synchronous
   per-cycle behaviour `cells_sim.v`'s comment-free blackbox declaration
   implies only real under `REGMODE=OUTREG`?
2. If it should work under `NOREG`, is there a documented interaction with
   `CE` (clock enable) that a data-dependent reset asserted only on some
   cycles, alongside a `CE` that is also only sometimes asserted (our
   `else if (!write)` guard drives both from logic), needs to route
   differently than `nextpnr-ecp5` does today?

We are not asserting this is a yosys bug -- we could not confirm it is a bug
at all without a board, and we are reporting what we found rather than what
we suspect. If `ecp5/cells_sim.v` (or the underlying memory-inference pass)
already carries an internal note about this restriction that we missed, a
pointer to it would resolve this immediately.

Reproducer attached / linked above:

- `mem_sync.v` -- the idiom that maps onto the block's own reset.
- `mem_mux.v` -- the fixed idiom, for comparison.
- `mem_mux_rst_driven.v` -- isolates "reset driven by logic" from "output
  mux downstream," to rule out the mux placement as what matters.
- `prim_raw.v` -- the same `RESETMODE=SYNC`/`REGMODE=NOREG` configuration,
  hand-instantiated, bypassing memory inference.

Toolchain: `oss-cad-suite`, yosys 0.68+48/ff5817c34, nextpnr-0.11-1-g62e659ed.
