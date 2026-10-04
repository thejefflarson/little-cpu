# ADR-0248: nano's slow-corner misses at 64 MHz are diagnosed and accepted

**Status:** Accepted · 2026-10-04 · amends ADR-0197's timing paragraph; builds on ADR-0219 and ADR-0225

## Context

ADR-0197 recorded that nano on a 4×2 routes with DRC, LVS and antenna clean while every slow
(`ss`) corner shows setup violations, and left the question open. It named three problems: the
reset input path under the shell's 20% input delay, max-slew and max-cap violations, and a latch
register file's half-cycle path. The tree has since changed under that record. ADR-0225 retired
both flop and latch register files for the `rf_top` macro, so the third problem no longer exists
and no latch variant is built. This ADR re-diagnoses the first two, and names the path behind the
worst slack, on the tree as it stands.

The tree is `main` at 9bc0fc8 (RTL unchanged since the macro landed). Run 37182760963 hardened it
at 4×2 with the default flow and `AREA 2`, full flow, 64 MHz (15.625 ns). It routes with Magic DRC
0, LVS 0 differences, antenna 0 violating nets and pins, and `nano-gl-test` on its own routed
netlist passes. Instance utilization is 58.7% (53.9% standard cell), routed wirelength
358,618 µm. The run reproduces the earlier one digit for digit (run 36841107809: max_ss worst
slack −10.109 ns), so the flow is deterministic on this tree.

The earlier runs' artifacts did not carry the stage that holds the slow-corner paths. LibreLane
writes the post-route STA one directory per corner under `55-openroad-stapostpnr`, and the
workflow's collector only walked directories named `reports`. The collector now copies those
`.rpt` and `.log` files, and `test/nano_tt_area_workflow_test.py` grades that, with a forced-red
probe.

## Measurements

Setup worst slack (ns) and counts per corner, `timing__setup__ws` and the violation counts from
`55-openroad-stapostpnr/summary.rpt`. Hold is clean everywhere, worst +0.11 ns (ff).

| corner | setup ws | setup violations | max slew | max cap |
|---|---|---|---|---|
| nom / min / max tt | +2.02 / +2.17 / +1.74 | 0 / 0 / 0 | 483 / 280 / 602 | 0 / 0 / 0 |
| nom / min / max ff | +4.66 / +4.76 / +4.58 | 0 / 0 / 0 | 90 / 32 / 116 | 0 / 0 / 0 |
| nom / min / max **ss** | **−9.24 / −8.35 / −10.11** | 405 / 401 / 407 | 3,028 / 2,472 / 3,443 | 11 / 4 / 13 |

Max fanout reads 44 at every corner. Every setup violation at every corner is register to
register; none starts at an input port.

## Diagnosis

**1. Reset is not a problem on this tree, and the default constraint stays.** The worst path from
`rst_n` has +5.62 ns of slack at max_ss and +6.20 ns at max_tt, against the shell's 3.125 ns input
delay. The flop register file's reset fanout that made it the worst path there is gone with that
register file. No constraint is added, no exception is declared, and no synchronizer is added: a
synchronizer would delay reset by two cycles to fix a path with 5.6 ns to spare. The default
constraint asserts that `rst_n` changes at most 3.125 ns after a clock edge at the pin, which is
all this ADR relies on; it does not claim more about the carrier board than that.

**2. The −10.11 ns path is instruction decode feeding a global enable.** Startpoint is the
instruction register (the flop's net is named `core.caddi16sp_immediate[8]`, a bit of `instr`),
endpoint `bus.ctrl.mem_addr[5]`, 74 cells deep, passing `core.rd[1]` and an `or4`/`or4`/`nor3`
legality-shaped cone. The violators agree with that reading. 378 of the 407 max_ss violators start
at that register, and their endpoints are every register `nano/nano.v` updates under `!take_trap`
or `instret`: `minstret` and `mcycle` (64 each), the register file write (33), `mscratch` (32),
`mepc` (31), `mtvec` (30), `tx_shift` (36), and the address registers. The decode-to-trap cone
fans out to all of them, so its depth is paid once and charged everywhere. I did not trace each
cell to a source line; the claim is the cone, not the cells.

Each corner's own worst path has 12.6 ns of data delay at max_tt against 28.2 ns at max_ss, a
ratio of 2.2, so a path with +1.74 ns at tt cannot close at ss at this period without about a
third less logic on it.

**3. Slew and capacitance are concentrated, and they are the resizer's choice of buffer.** Of
3,420 max_ss slew violations matched to a driver, 2,634 (77%) sit on nets driven by delay cells
`repair_design` inserted as fanout buffers (`clkdlybuf4s25_1` 2,439, `dlymetal6s4s_1` 107,
`dlymetal6s2s_1` 88), 391 nets in all; 10 of the 13 cap violations are on the same cells. At
nominal tt the share is 94% (555 of 589). The clock tree does not appear. The netlist holds 826
delay cells of 26,667, 542 of the buffers named `fanout*`, and 18 of the 74 cells on the worst
path, which cost 10.4 ns of its 28.2 ns at ss and 2.1 ns at tt. The resizer repairs at one
corner (nominal tt), so the ss edges are about twice as slow as anything it saw.

## Counterfactuals measured and not shipped

Both are runs on throwaway branches, one dispatch each, full flow at 4×2.

- **Exclude the delay cells from the buffer pool** (run 37187046270; `EXTRA_EXCLUDED_CELLS` on
  `clkdlybuf*` and `dlymetal*`, `dlygate*` kept for hold). Routes, DRC/LVS/antenna 0, gate-level
  simulation passes. max_ss moves −10.11 → −8.21 ns, nom_ss −9.24 → −7.68, min_ss −8.35 → −7.05;
  slew violations do not move (3,443 → 3,292 at max_ss) because the resizer picks `buf_1`, the
  next weakest buffer, which now drives 2,488 of 3,269. Cap violations rise 13 → 32. A cell-list
  override that buys 1.9 ns, leaves the slow corner 8 ns short and the slew count unchanged is
  not shipped, which is also the owner's standing position on flow knobs (ADR-0219).
- **Synthesis strategies `DELAY 0` and `DELAY 4`** (runs 37192092123 and 37192093780). Neither
  finishes: both were cancelled at the job's six-hour limit still in detailed routing, the last
  logged pass reading 542 violations for `DELAY 0` and 1,151 for `DELAY 4`, where the `AREA 2`
  run reaches zero in 1 h 29 min. Not a candidate at 4×2.

## Decision

**The slow-corner setup misses at 64 MHz are accepted on the record, and no flow knob or
constraint changes.** The 64 MHz target and the 4×2 tile stay. What is accepted:

- max_ss setup worst slack −10.11 ns, 407 violating endpoints (−9.24 nom_ss, −8.35 min_ss). The
  cause is decode depth in front of the global trap/retire enable, a design property, and the
  nominal and fast corners close with +1.74 ns and +4.58 ns at worst. The owner has accepted that shipped
  Tiny Tapeout cores such as TinyQV miss `ss` by a similar margin; that supports accepting it
  and does not prove it safe.
- Max-slew violations (3,443 at max_ss, 483 at nom_tt) and 13 max-cap violations, 77% of the
  slew count and 10 of 13 caps on resizer-inserted delay cells. These are electrical limits in
  the library, already counted in the setup numbers above, and the flow's own checker does not
  gate on them.
- No constraint is changed, so none needs its assertion stated.

**The fix that would remove the miss is a design change, and it is not made here.** `nano/nano.v`
has a spare cycle (`fetch_rs1`, while the macro read settles) in which the instruction register is
stable. Registering the instruction-decode half of `take_trap` (illegal, `ecall`, `ebreak`, CSR
legality) there would take the legality cone out of the execute cycle's global enable. That is a
re-timing of verified RTL: it moves `take_trap`'s cause chain, RVFI's captured copy and the
formal harnesses' depth derivation, and a re-run of this flow decides whether the next cone
closes. It is the next ticket's measurement, not this one's.

## Consequences

- ADR-0197's statement that the slow-corner violations are "the resizer's own known limit on this
  design" is amended: on the macro tree they are decode depth plus the resizer's buffer choice,
  measured above.
- `nano-gl-test` passes on the baseline and on the delay-cell experiment. The accepted state is
  not a fit claim stronger than what ran: routes, DRC/LVS/antenna 0, gate-level simulation
  passes, nominal and fast corners close, and the slow corner misses.
- The workflow now uploads the per-corner post-route STA, so the next diagnosis starts from paths
  rather than counts.
