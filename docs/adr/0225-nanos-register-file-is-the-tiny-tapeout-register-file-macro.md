# ADR-0225: nano's register file is the Tiny Tapeout register-file macro

**Status:** Proposed · 2026-10-01 · an experiment: the default-flow run at 4×2 on this branch decides whether it ships

## Context

Measurement-only runs 36766659352 and 36766667951 reduced nano's register file to one register.
Flow synthesis fell from 76,054 to 56,089 µm² and nano routed cleanly on a 4×2 tile at 54–57%
routing demand. With the full 15×32 flop register file it never routes (84.7%). The register
file's logic is what blocks 4×2: 480 flops, their write enables, and two read multiplexers.

Sylvain Munaut's `rf_top` is a hard 32×32 SRAM block built for Tiny Tapeout, with one write port
and two read ports. FemtoRV's register-file test (MichaelBell/ttsky25b-femtorv-soc, 2×2) and
KianV's (TinyTapeout/ttsky25b-kianv-linux-soc-with-regfile, 6×2) both ship it, and the first's
author reports it working in silicon at 80 MHz (not re-verified here). A hard block is not routing demand: the router sees a
rectangle that obstructs `li1`, `met1`, `met2` and `met3`, and the logic that was in it is gone.
A separate branch (#424) tries RTL clock gates on the flop register file instead. This ADR is
the other alternative, and the owner keeps whichever passes every check on 4×2.

## Decision

**nano's `regs[]` array is replaced by one `rf_top` instance, and nothing else about the core's
cycle behaviour changes.** `nano/nano.v` reads and writes through the macro's three ports:

- Both read ports are presented together in `fetch_rs1`, from the same decode of `instr` that
  already fed the old read mux. Both operands are registered inside the macro by the edge that
  ends `fetch_rs1`, so `execute_instr` finds `rf_ra_data` and `rf_rb_data` as the old `op_rs1`
  register and the old live rs2 read did. `op_rs1` is no longer a register, and the shared
  four-bit read-address mux is gone.
- Word 0 is real storage in the macro, so x0 is masked on read (`|rs1[3:0] ? data : 0`, 64
  AND gates) and never written, since `wb_en` already requires `|rd[3:0]`. Words 16 to 31 are
  unused; address bit 4 is tied low.
- `rf_top` is defined in `nano/nano.v` itself, as a behavioural model. Every simulator and every
  proof reads that text, and in the Tiny Tapeout flow the liberty cell replaces the module of
  that name, which is how both shipped designs do it. A second file would have meant touching
  every harness and every probe fixture that copies `nano.v`.
- RVFI's `rd_wdata` read `regs[rd]` combinationally at fetch entry. The macro cannot be read
  back by name, so `rvfi_wb_data` (RVFI builds only) holds the last value written instead.

**The contract nano relies on, and no more.** The liberty says: reads are clocked on `clk` and
leave 4.0 ns after the edge; inputs need 2.0 ns of setup and 0.2 ns of hold; the minimum period
is 10 ns. It says nothing about a read of the word being written on the same edge, and neither
does the author's validation write-up. The two shipped behavioural models bypass (they return
the written word), and KianV's atomics broke on `rd == rs2` because a held read port re-reads
after a write (a KianV fix, reported in this ADR's research and not re-verified here). **Nobody has shown what the silicon does, so nano does not depend on it.** The
ordering that makes that true:

- Writes land on the edge ending `execute_instr` (ALU, jump, CSR) or `finish_load`. Stores and
  branches write nothing.
- Operand addresses are presented in `fetch_rs1`, at least three edges after the last write
  (`fetch_instr`, `ready_instr`, `fetch_rs1`). The edge that captures them never carries a write.
- `instr` is held through `execute_instr`, `finish_load` and `finish_store`, so the addresses
  hold and the macro keeps answering with the same word. No write falls between the capture and
  the last consumer; a write lands only on the edge that ends the instruction.
- After that final edge, a port whose address equals the write address is unspecified. Nothing
  consumes it: the next instruction presents new addresses before it reads.

**The model enforces this by poisoning the unspecified case.** A read of the word being written
on the same edge returns the old word inverted, which is neither answer a design could mistake
for the macro's. Every leg then fails if nano ever consumed such a read. It is a single value,
not a free one: `anyseq` would need a formal-only define that nano's `.sby` files do not carry,
so a proof under the poison shows the retired trace is right when it is read, not for every
value it could have held.

## Provenance and licence

The three files come from TinyTapeout/ttsky25b-kianv-linux-soc-with-regfile at commit
`58bb3d5cfd123a339afffb0126d4d094cf68d154`, `regfile_macro/`, and are byte-identical to the copies in
MichaelBell/ttsky25b-femtorv-soc at `50aa37fdb7befa4fe2fac69f71a10d64d102df3c`, `macro/`. `nano/nano.mk`
pins each by SHA-256 and `make nano-rf-macro-setup` fetches them into the tool cache, refusing a
download that does not match, the way `nano-liberty-setup` does:

| file | SHA-256 |
|---|---|
| `rf_top.lef` | `b8d2dfd70500fc79bf921dba9e643da7ecb95ca1d1198f6581ab498571d11dcb` |
| `rf_top.lib` | `4c85936bb8a29385b9953ab8797041570082bdd860f0a46a28644be7c8e8ccb8` |
| `rf_top.gds.gz` | `525fb010215ba3400689c0c40a4bf97415910f40498f6882d7cd027cad1d01e2` |

**No macro file is committed, because the licence is not clear enough to redistribute.** The
author's own repository, smunaut/ttsky25b-rf-validation, has no LICENSE file; its `src/project.v`
carries `Copyright (c) 2025 Sylvain Munaut` and `SPDX-License-Identifier: Apache-2.0`, and its
`gds/` and `lef/` are the validation chip, not the macro. The macro's own LEF, liberty and GDS exist
only in the two downstream repositories above, each carrying Tiny Tapeout's template Apache-2.0
LICENSE for the whole repository and no notice on the macro files. The author's other tapeout
repository, smunaut/ttcad25a-rf-yolo-test, is Apache-2.0. Apache-2.0 is compatible with this
repository's MIT licence, so redistribution with the notice kept is probably fine. But the author's
grant for these particular artefacts is inferred from a template and a sibling repository, not read
from a file, so it is not stated here as fact.

The flow reads the files from `nano/tt/macro/`, which `make nano-rf-macro-install` fills and
`.gitignore` excludes; the self-hosted workflow runs it before hardening. A real submission through
`tt-gds-action` has no such step, so **shipping needs the owner's call: ask the author for a
licence statement, or commit the three files with Apache-2.0 attribution.**

## How each tool sees the macro

| tool | what it reads | what it can say |
|---|---|---|
| cxxrtl and iverilog | `rf_top` in `nano/nano.v`, two-state and four-state | registered timing, held reads, the poisoned same-edge read, an X from a word never written |
| riscv-formal and the component proofs | the same module | the same, for every retire within each check's depth; F stays 11 and G stays 9 |
| `nano-gl-test` | the same module, cut out of `nano.v` by `gl_census.py` and read with `GL_TEST` | the logic around the macro, in the flow's own netlist |
| `nano-area` and `nano-timing` | a black box, priced from the LEF | soft logic and the macro's footprint, separately |
| the Tiny Tapeout flow | the LEF, liberty and GDS | placement, routing, STA against 4.0 ns clock-to-out and 2.0 ns setup, DRC, LVS |

**Gate-level simulation sees only the behavioural model of the macro, not its layout.** Nothing in
this repository simulates the macro's transistors, and yosys ships no model for it. What the macro
does as silicon is the author's validation chip's claim, not a check here.

`gl_census.py` takes `--macro rf_top`: the netlist must instantiate it exactly once, and any
instantiated module that is neither a `sky130_fd_sc_hd` cell, a named macro nor defined in the
same file is refused. `nano/gl_census_probe.sh` forces each direction red: a netlist missing the
macro, one with it twice, one with another module, one run without naming the macro, and a macro
source that defines no such module.

## Measurements

Cycles do not move, since `fetch_rs1` already existed and now presents both addresses.

| harness | benchmark | before | after |
|---|---|---|---|
| `nano-sim` (zero wait) | `make nano-dhrystone` | 415,887 cycles | 415,887 |
| `nano-sim` (zero wait) | `make nano-coremark` | 15,696,013 cycles | 15,696,013 |
| `nano-qspi-pins-sim` | `make nano-qspi-pins-dhrystone` | verdict 0, 4,294,544,416 cycles, 13,926 writes | verdict 0, same |
| `nano-qspi-pins-sim` | `make nano-qspi-pins-coremark` | not taken on main | verdict 0, 4,294,262,627 cycles |

The pins harness fails on main already, as `CLAUDE.md` records for its chained-resume bug, so those
rows say the macro changed nothing about a failure that was there. The `nano-qspi-pins-coremark`
baseline was not run.

`make nano-area`, the local instrument, with the macro black-boxed and priced from the LEF's
`SIZE 132.640 BY 118.700`:

| | µm² |
|---|---|
| soft logic | 53,899.2 (sequential 11,315.9) |
| `rf_top`, fixed | 15,744.4 |
| total | 69,643.6 |
| flop register file, same instrument, before | 75,082.0 |

That is −5,438 µm² on this instrument. It is a ranking and not a fit: the soft logic falls by
21,183 and the macro puts 15,744 back, and what decides 4×2 is routing, which only the flow
measures. `NANO_MAX_UM2` moves to 71,700 and grades soft logic plus the macro, so a second hard
block cannot hide.

Formal: `make -C nano/formal all` is green, and `remeasure-fg` reads F = 11, G = 9, declared 11 and 9.

## Consequences

- **Open until the flow runs:** that 4×2 routes with the macro at `[0.1, 45.0]` (the position
  FemtoRV ships), with the halo and power-pin settings copied from its `config.json`; that the
  macro's 4.0 ns clock-to-out and 2.0 ns setup fit nano's 15.625 ns period around the decode and
  mask logic it now sits between; DRC, LVS and antenna with the macro's GDS; and
  `nano-gl-test` on that run's netlist. Nothing here reports a fit.
- The fixed 15,744 µm² is 87% of one Tiny Tapeout tile. nano cannot go below it.
- 16 of the macro's 32 words are unused. The macro has no 16-word variant.
- The macro has no reset and holds no known value at power-up, as the flops did not.
- `nano/rf_macro_setup.sh`, `nano/area_report.py`, `nano/gl_census.py` and the workflow each grade
  the macro's presence, digest or count, with forced-red probes in `test/probe_gates.sh`,
  `nano/gl_census_probe.sh`, `nano/tb/nano_rf_model_probe.sh` and `nano/tb/nano_rf_timing_probe.sh`.
- No nano mutant consumes the same-edge read, because nano has no such path. The model's own
  bench grades the poison, and the timing probe grades the registered-read assumption: addresses
  presented a cycle late, and the two ports swapped, each fail the suite on the iverilog leg.
