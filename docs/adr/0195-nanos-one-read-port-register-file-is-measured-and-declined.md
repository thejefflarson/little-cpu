# ADR-0195: nano's one-read-port register file is measured and declined

**Status:** Accepted (the lever is declined; `NANO_ONE_PORT_RF` ships off by default) · 2026-09-18

## Context

ADR-0184's work order for fitting nanocpu onto a 2×2 Tiny Tapeout tile is: (1) a
latch-array register file (ADR-0189), (2) mul/div rebuilt around one shared 64-bit
register (ADR-0192), (3) a one-read-port register file, with M itself a second permitted
cut only if the three together are not enough. Steps 1+2 together read 65,026.1152 µm² /
66.49% routing demand on 4×2 (`AREA 2`, `disallow_congestion=true`) and `[GPL-0301]`
105.422% utilization on 2×2 — closer, but not closed, and ADR-0192 named the
read-address fan-out ADR-0184 originally flagged as the plausible remainder, sizing
step 3's target at roughly 61,700 µm² or below. This is step 3, measured.

## Decision

`nano/nano.v` gains a `NANO_ONE_PORT_RF`-selected read side: rs1 is read into a held
register (`op_rs1`) in one cycle, rs2 into a second held register (`op_rs2`) in the
next cycle unless the instruction doesn't read rs2 (`rs2_valid`, an existing signal),
and `rf_raddr` — `rs1[3:0]` or `rs2[3:0]` depending on which of two new FSM states,
`fetch_rs1`/`fetch_rs2`, is active — is the only expression that ever indexes `regs[]`
for a read anywhere in the file. Every other read site goes through `` `RF_RS1``/`` `RF_RS2``
macros that expand to `op_rs1`/`op_rs2` under the define and to the original
`regs[rs1[3:0]]`/`regs[rs2[3:0]]` text otherwise, so the default build's preprocessed
source is unchanged (`iverilog -E` diff against `origin/main` is clean modulo
line-padding blanks, and `make nano-area` reproduces 57,704.0928 µm² exactly). It
combines orthogonally with `NANO_LATCH_RF`, so all four register-file builds are
measurable; new Makefile targets `nano-oneport-{test,startup-test}` and
`nano-oneport-latch-{test,startup-test}` join `make test` alongside the existing pair.
The CI workflow's `ports` choice input (`two`/`one`) reaches `VERILOG_DEFINES` only
through `env:`, mirroring how `regfile` was added.

**The measurement is negative on both counts it was taken for, and the register file
ships with `NANO_ONE_PORT_RF` off.** Halving the read ports buys 1.7% of synthesis
area, costs a fifth of Dhrystone's throughput, and makes routing measurably worse
rather than better. It does not close the 2×2 gap it was built to close. It stays in
the tree only as a measured, off-by-default lever — the way `NANO_LATCH_RF` did before
it, and for the same reason: the number is worth having on record so nobody re-spends
the placements finding this out twice.

## Correctness

All four register-file builds pass both simulator legs and their startup checks
(`nano-test`, `nano-latch-test`, `nano-oneport-test`, `nano-oneport-latch-test`, plus
each `-startup-test`), retiring the same instruction counts as the pre-existing builds.
`nano/tb/nano_exec_probe.sh`'s `mulhsu-does-not-negate-rs1` mutation is rewritten
against the new `` `RF_RS1`` text (the other three mutations are untouched, since they
never named a `regs[]` read directly). `make -C nano/formal check`/`all` reproduce the
existing baseline exactly (82 checks, 80 pass) and `remeasure-fg` reproduces F=12, G=10
unchanged, because `nano/formal`'s harnesses never build with `NANO_ONE_PORT_RF`
defined — the same standing `NANO_LATCH_RF` already has, so no proof depended on
single-cycle operand fetch and none needed re-deriving.

## Measurement

**Cycle cost, pinned compiler (xPack `riscv-none-elf-gcc` 15.2.0-1, ADR-0190), one
session, `make nano-dhrystone`/`make nano-coremark`, flops on both sides:**

| Metric | Two ports (baseline) | One port | Change |
|---|---|---|---|
| Dhrystone cycles | 503,300 | 632,327 | **+25.6%** |
| Dhrystone DMIPS/MHz | 0.226 | 0.180 | −20.3% |
| CoreMark cycles | 9,301,580 | 11,470,408 | **+23.3%** |
| CoreMark CoreMark/MHz | 0.538 | 0.436 | −18.9% |

Retire counts are identical both sides on both benchmarks (105,361 and 1,498,831), so
every added cycle is the one-port read schedule and nothing else moved. A lever that
costs a fifth of the core's speed is worth recording precisely so nobody re-tries it
expecting it to be free.

**Tiny Tapeout flow** (`nano-tt-area-selfhosted.yml`, LibreLane 3.0.14, `AREA 2`,
`disallow_congestion=true`), against ADR-0192's two-port-plus-latches pair on the same
tile, strategy and flow:

| | Two ports, 4×2 ([35393696793](https://github.com/thejefflarson/little-cpu/actions/runs/35393696793)) | One port, 4×2 ([35405902371](https://github.com/thejefflarson/little-cpu/actions/runs/35405902371)) | Change |
|---|---|---|---|
| Synthesis area | 65,026.1152 µm² | 63,907.5424 µm² | **−1,118.57 µm², −1.72%** |
| GPL-0019 utilization | 51.064% | 50.182% | −0.88 pt |
| GRT-0096 total demand | 66.49% | **75.86%** | **+9.37 pt** |
| GRT-0096 total overflow | 59 (met1 9, met2 1, met3 48, met4 1) | **1,414** (met1 323, met2 354, met3 556, met4 181) | **×24** |
| GRT-0018 wirelength | 593,048 µm | **685,225 µm** | **+15.5%** |
| Routed nets | 8,146 | 7,377 | −9.4% |
| RSZ-0038 buffers / nets | 924 / 299 | 861 / 283 | −6.8% / −5.4% |
| RSZ-0046 hold endpoints | 815 | 774 | −4.8% |
| Result | `GRT-0116` congestion | `GRT-0116` congestion | still refused |
| Wall time | 2,873s | 1,203s | |

Both runs fail `disallow_congestion=true` on `GRT-0116`, but at a different scale in
the wrong direction: overflow rose 24× and wirelength rose 15.5% for a 1.7% area cut.
**One read port buys area and spends routing, and the routing loss is much larger in
proportion than the area gain.**

**2×2, real, not projected**
([35407575556](https://github.com/thejefflarson/little-cpu/actions/runs/35407575556),
245s): the same 63,907.5424 µm² synthesis as the 4×2 row above, then
`[GPL-0019] Utilization: 103.601 %`, `[GPL-0301] Utilization 103.601 % exceeds 100%` —
refused before routing is ever attempted. Arithmetic projected from ADR-0192's own
method (63,907.5424 / 65,026.1152 × 105.422%) read 103.6% before this run landed; the
real number, 103.601%, confirms it to three significant figures. Against the two-port
pair's 105.422%, one port buys 1.8 points of utilization — real, in the right
direction, and not remotely enough to cross 100%.

**Concrete cell-census deltas, not speculation, for what actually moved** (`stat.rpt`,
same liberty, same flow, both builds carry `NANO_LATCH_RF`):

| Cell | Two ports | One port | Δ |
|---|---|---|---|
| Total cells | 6,121 | 5,893 | −228 (−3.7%) |
| `sky130_fd_sc_hd__dfxtp_2` (flops) | 511 | 577 | **+66** |
| `sky130_fd_sc_hd__dlxtp_1` (latches) | 480 | 480 | 0 (write side untouched, as designed) |
| `sky130_fd_sc_hd__mux2_1` | 254 | 384 | **+130** |
| `sky130_fd_sc_hd__mux4_2` | 0 | **16** | new |

+66 flops is close to the 64 bits `op_rs1`/`op_rs2` add (two 32-bit registers that did
not exist in the two-port build); the register-file itself loses nothing from removing
a read port, since `NANO_LATCH_RF`'s write side is unchanged and reads were always
`regs[]` array accesses rather than a distinct hardware structure with its own cell
type. The read-select tree ADR-0184 named does shrink — total cells fall 228 and area
falls 1,118.57 µm² — so the mechanism does remove what it was built to remove. What it
adds back, and by more than it saves in area even though not in cell count, is a
`mux2_1`/`mux4_2` population (+130, +16) that did not exist before.

**Everything past the cell census — why the router in particular got worse — is
inference, clearly labelled as such, not a further measurement.** ADR-0184's own
finding was that the register file's two read ports were the dominant source of
routing congestion: "52% of net crossings in overflowing routing cells were
register-file nets." **That attribution does not survive this measurement.** Cutting
the read ports in half took overflow from 59 to 1,414 and wirelength up 15.5%, the
opposite of what removing the dominant congestion source should do. The plausible
mechanism, offered as inference only: `op_rs1`/`op_rs2` are read at roughly twenty
sites scattered across the ALU, load/store address, branch comparator, and multiply/
divide operand paths — the same set of consumers the old `regs[rs1[3:0]]`/
`regs[rs2[3:0]]` reads already fanned out to — but they are now driven by two
flip-flops the placer must locate somewhere, rather than by a register-file read port
whose physical position the array's own layout already fixed. Two new FSM states
(`fetch_rs1`, `fetch_rs2`) also widen `cpu_state`'s case-arm fan-out by two more
comparisons, each reaching every consumer that already read `cpu_state`. Neither claim
is measured directly here; a placement-density or routing-congestion map of the two
builds would be needed to confirm either, and none was taken.

## Consequences

- `NANO_ONE_PORT_RF` ships off by default, exactly like `NANO_LATCH_RF` before it: an
  additive, orthogonal build option, measured and kept as a lever, not adopted.
- ADR-0184's work order (latch register file, shared-register mul/div, one-read-port
  register file) is now complete, and **none of the three alone or in combination
  (latches+mul/div: ADR-0192; latches+mul/div+one-port: this ADR) closes 2×2 under
  `disallow_congestion=true`.** The best result on record stays ADR-0192's
  latches-plus-mul/div pair: 65,026.1152 µm² / 66.49% demand / 59 overflow on 4×2,
  ~5.1% too much area to place on 2×2 at all.
- The read-address fan-out ADR-0184 identified as the dominant remaining congestion
  cause is not resolved by removing a read port; whatever is actually driving the
  residual overflow on the latches-plus-mul/div pair remains unidentified. M as a
  second permitted cut (ADR-0184) is the next lever the work order names, now that all
  three measured, cheaper ones have been spent.
- The local `NANO_MAX_UM2` ratchet is unaffected (61412, unchanged); no flow-unit line
  gates nanocpu until a finished, routed core exists (ADR-0184).
- `nano/tb/nano_exec_probe.sh`'s `mulhsu-does-not-negate-rs1` mutation now targets
  `` `RF_RS1`` text; a future rewrite of the read side owes it the same care ADR-0192's
  mul/div rewrite gave its own four mutations.
