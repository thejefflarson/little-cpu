# Investigation: is the ECP5 block RAM reset yosys's fault or nextpnr/Trellis's?

ADR-0163 found that yosys maps `rtl/memory.v`'s pre-fix out-of-range arm --
`mem_rdata <= in_range ? ram[index] : 32'b0` -- onto `DP16KD`'s own
synchronous reset (`RSTA`/`RSTB` driven by logic, `REGMODE_A`/`REGMODE_B` at
`NOREG`), and that on the iCESugar-Pro that read returns zero whatever the
array holds. It fixed the defect and recorded that it had not established
*which* tool is wrong. This is that follow-up, run on this machine only --
no board is attached here, so nothing below is a claim about what the part
actually does at runtime. It reads yosys's own JSON netlist and nextpnr's own
`--textcfg` output, which is all that is checkable without one.

## Method

Four minimal designs, each one `always @(posedge clk)` block over a single
512x32 array (16,384 bits, exactly one `DP16KD`/`PDPW16KD`), built through
this repo's own `synth_ecp5` invocation and `soc/bram_reset_check.py`, then
placed with `nextpnr-ecp5 --25k --package CABGA381 --speed 6 --textcfg`
(the project's own ECP5 corner; unconstrained I/O, since only the EBR tile's
configuration is being read). All four builds are reproduced below; every
number is from this session, on `oss-cad-suite` yosys 0.68+48/ff5817c34 and
nextpnr-0.11-1-g62e659ed.

| Variant | File | Reset pin | Output mux | `bram_reset_check.py` |
|---|---|---|---|---|
| 1 | `mem_sync.v` | driven by logic | none (folded into the register) | **flags it** |
| 2 | `mem_mux.v` | tied to a constant | a separate LUT mux, post-register | passes |
| 3 | `mem_mux_rst_driven.v` | driven by logic | a separate LUT mux, post-register | **flags it** |
| 4 | `prim_raw.v` | driven by logic, via a hand-written `PDPW16KD` instance | n/a -- no inference pass runs at all | n/a (not a memory-inferred design) |

Variant 1 is the pre-fix idiom ADR-0163 diagnosed, reconstructed at minimum
size. Variant 2 is the shipped fix's idiom, same reduction. Variant 3 and 4
are new to this investigation.

## What yosys's own JSON netlist says

Both variant 1 and variant 2 map to `REGMODE_A = REGMODE_B = NOREG` --
**identical in both the broken and the fixed spelling.** `REGMODE` is not
what ADR-0163's defect turns on, despite its text naming that parameter.

What differs between the two, in the JSON `synth_ecp5 -json` writes:

| | variant 1 (bad) | variant 2 (fixed) |
|---|---|---|
| `RESETMODE` | `SYNC` | `ASYNC` |
| `RSTB` connection | a real net (wire index) | the constant `'0'` |

`RESETMODE=SYNC` is that primitive's own *default* parameter value in
`ecp5/cells_sim.v` -- yosys is not reaching for an obscure corner of the
parameter space here, it is the ordinary spelling of "clear this read port
synchronously." Variant 2's `RESETMODE=ASYNC` is not a meaningful choice on
yosys's part; it is what yosys writes when a reset port is unused and tied
off, and it is immaterial once the connected value is a constant that never
changes.

## What nextpnr-ecp5 does with it

`nextpnr-ecp5 --textcfg` writes the placed EBR tile's configuration as a
short block of `enum:`/`word:` lines. For variant 1:

```
enum: EBR0.MODE PDPW16KD
enum: EBR0.REGMODE_A NOREG
enum: EBR0.REGMODE_B NOREG
enum: EBR0.RESETMODE SYNC
enum: EBR0.RSTAMUX INV
enum: EBR0.RSTBMUX RSTB
```

and for variant 2:

```
enum: EBR2.MODE PDPW16KD
enum: EBR2.REGMODE_A NOREG
enum: EBR2.REGMODE_B NOREG
enum: EBR2.RESETMODE ASYNC
enum: EBR2.RSTAMUX INV
enum: EBR2.RSTBMUX INV
```

`RSTBMUX RSTB` means the tile's reset pin is fed from the routed net yosys
asked for; `RSTBMUX INV` means it is tied through the fixed value both
unused ports (`RSTAMUX INV` in both cases) already use. **Every field in the
placed configuration is a direct, unmodified copy of the corresponding field
in yosys's JSON netlist** -- `RESETMODE` and the choice between `RSTB` and
`INV` both carry straight through with no reinterpretation.

**Variant 4 settles whether this is an artifact of yosys's memory-inference
pass specifically.** `prim_raw.v` hand-instantiates `PDPW16KD` directly --
`memory_bram` never runs, because there is no `$mem` cell to fold -- with
the same parameters (`REGMODE=NOREG`, `RESETMODE=SYNC`) and its `RST` pin
wired to an arbitrary combinational signal. Placed, its tile reads:

```
enum: EBR0.MODE PDPW16KD
enum: EBR0.REGMODE_A NOREG
enum: EBR0.REGMODE_B NOREG
enum: EBR0.RESETMODE SYNC
enum: EBR0.RSTAMUX RSTA
enum: EBR0.RSTBMUX RSTB
```

The same shape (`RESETMODE SYNC`, the reset pin routed rather than tied)
appears whether the cell arrives through yosys's automatic inference or
through a hand-written instantiation that bypasses it entirely. nextpnr's
encoding does not depend on how the cell was produced.

**`nextpnr-ecp5`'s own binary carries no validation for this combination.**
`strings` over `libexec/nextpnr-ecp5` turns up parameter checks only for
`REGMODE_A`/`REGMODE_B` (`"DP16KD %s has invalid REGMODE_A configuration
'%s'"`, checked against the two-entry enum `{NOREG, OUTREG}`); there is no
string, and so no code path visible this way, that validates `RESETMODE`
against `REGMODE`, or that treats a routed `RSTAMUX`/`RSTBMUX` under
`REGMODE=NOREG` as anything unusual. nextpnr accepts and places it exactly
as asked, with nothing suggesting it has an opinion about whether the
combination is sound.

## What this settles

**Nothing found here shows nextpnr-ecp5 or Trellis mis-encoding, dropping,
or reinterpreting what yosys's netlist requests.** Every field checked --
`REGMODE_A`, `REGMODE_B`, `RESETMODE`, and which net (if any) reaches
`RSTAMUX`/`RSTBMUX` -- reads back in the placed `--textcfg` output exactly
as yosys's JSON declared it, and does so identically whether the cell came
from `memory_bram`'s inference or from writing the primitive by hand. That
rules out the specific failure mode of "nextpnr silently drops or
miscodes the reset connection" -- the evidence this investigation can gather
is squarely against it.

**This narrows, but does not resolve, which tool is at fault.** Ruling out a
silent nextpnr/Trellis encoding error leaves two possibilities this
machine cannot distinguish between: that yosys's `ecp5/cells_sim.v` model
offers a feature (a synchronous, per-cycle output clear under
`REGMODE=NOREG`) that real `DP16KD` silicon does not implement the way the
model assumes, or that the feature is real and something else in this
reduction (this investigation never wired `CE` and `RST` to conflicting
values, for instance) is what differs on the part. Answering that needs
either Lattice's own EBR configuration guide, which is not available on
this machine, or a board run of variant 1 and variant 4 side by side, which
needs hardware this machine does not have. Recorded here as an honest
**could-not-fully-distinguish**, with the encoding-fidelity half resolved
and the silicon-semantics half not.

## What this settles about the "also unresolved" question

ADR-0163's fix ties the reset low **and** moves the mux after the register,
and never isolated which one the defect needed. Variant 3 isolates them: a
genuine logic-driven synchronous clear (`extra_rst`), folded into the
block's own reset exactly as variant 1's is, **with** the downstream output
mux variant 2 uses. `soc/bram_reset_check.py` still flags it, and its placed
tile carries the identical hazardous signature (`RESETMODE SYNC`, `RSTBMUX
RSTB`) variant 1's does. **The downstream mux does not by itself neutralize
a logic-driven block RAM reset.** What variant 2 actually needed was tying
the reset pin to a constant; the mux is a necessary consequence of
implementing the zero arm without delegating it to the block's own reset
feature, not an independently load-bearing part of the fix.
`soc/bram_reset_check.py`'s criterion -- any block RAM reset port connected
to a non-constant net -- is checking the invariant that matters, not a
side effect of how the fix happened to be spelled.

## Reproducing this

```sh
# from the repo root
yosys -p 'read_verilog docs/investigations/ecp5-bram-reset-rsta/mem_sync.v; synth_ecp5 -top mem_sync -json /tmp/a.json'
python3 soc/bram_reset_check.py /tmp/a.json      # flags it

yosys -p 'read_verilog docs/investigations/ecp5-bram-reset-rsta/mem_mux.v; synth_ecp5 -top mem_mux -json /tmp/b.json'
python3 soc/bram_reset_check.py /tmp/b.json      # passes

nextpnr-ecp5 --25k --package CABGA381 --speed 6 --json /tmp/a.json \
  --lpf-allow-unconstrained --textcfg /tmp/a.config
grep -A16 tile_group /tmp/a.config               # RESETMODE SYNC, RSTBMUX RSTB
```

`upstream-issue-draft.md` in this directory is a draft report against yosys,
built from variant 1 and variant 4. It has not been filed; a human sends it.
