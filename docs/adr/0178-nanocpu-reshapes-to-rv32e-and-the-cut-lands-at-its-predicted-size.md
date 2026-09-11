# ADR-0178: nanocpu reshapes to RV32E, and the cut lands at its predicted size

**Status:** Accepted · 2026-09-11

## Context

ADR-0177 chose RV32E as nanocpu's one area cut: 16 registers instead of 32, M and the
`mcycle`/`minstret` counters kept, `misa` out of scope (`nano/nano.v` implements no CSR
register at all). It also named the `-23,400 µm²` figure it was decided against for what
it is: an upper bound, measured by synthesizing a 31x32 and a 15x32 register file in
isolation (45,011 against 21,592 µm²), which the real core's own logic could realize less
of once yosys folds the register file's read multiplexers into the datapath around them.
This ADR is the reshape ADR-0177 called for, and its job is to say whether that shortfall
happened.

The reverted `ill_e` (ADR-0174's amendment) also had to be rebuilt to the standard the
first one failed: wired to the real core rather than a hand-written reference, and able to
tell a right RV32E rule from a wrong one.

## The reshape

`nano/nano.v`'s register file shrinks from `regs[0:31]` to `regs[0:15]`; every read and
write site indexes it as `regs[rs1[3:0]]`/`regs[rs2[3:0]]`/`regs[rd[3:0]]`, truncating
explicitly rather than relying on an implicit narrow-address wrap a solver and a mapper
might resolve differently. `rd`, `rs1` and `rs2` stay 5-bit decoded fields -- the decode
`case` statements are untouched -- so the raw register number named by an encoding is
still available for the E-illegality test.

`is_e_illegal` is one line: `rd[4] || (rs1_valid && rs1[4]) || (rs2_valid && rs2[4])`.
Every decode arm that is not the raw `instr[11:7]`/`instr[19:15]`/`instr[24:20]` field
already produces a value under 16 by construction (0, 1, or a compressed
`{2'b01, x}` register), so checking bit 4 needs no per-class list of which arm ran.
`rs1_valid`/`rs2_valid` (moved out of the `RISCV_FORMAL` block, since illegality now
needs them unconditionally) gate rs1/rs2 to the encodings that actually read a register
there -- lui/auipc/jal have no rs1, and jalr/load/math_immediate's would-be rs2 field is
immediate or shamt bits, not a register name. `is_valid` now ANDs in `!is_e_illegal`, so
a register field naming x16-x31 traps the same way any other illegal encoding does.

`nano/formal/checks.cfg` keeps `isa rv32imc` and `RISCV_FORMAL_ALTOPS` -- nothing narrows
the generated check set's ISA, and `nano/formal/EXPECTED_CHECKS` is unchanged. What
changes is `[defines]`, which now also states `` `define RISCV_FORMAL_E ``. That macro
only does anything inside `nano/formal/rvfi_insn_check.sv` (ADR-0174's fork), which the
pinned `checks/rvfi_insn_check.sv` upstream does not carry -- so `formal/genchecks-audit.py`
gained a small, generic step, `patch_local_insn_check`: when a harness directory carries
its own `rvfi_insn_check.sv` beside `checks.cfg`, every generated `insn_*` check's
`[files]` entry for the pinned clone's copy is rewritten to the local fork instead. This
only ever fires for `nano/formal` -- `formal/` (littlecpu's own harness) has no such file,
so its checks keep reading the pin unmodified, and the shared `formal/riscv-formal` clone
itself is never edited in place.

## `ill_e`, rebuilt against the real core

`nano/formal/ill_e.sv` asks the real core's own RVFI report one question, with no separate
reference model to disagree with itself: `nano.v` already reports 0 for a register field
that is not decode-significant on the retiring encoding (the same `rs1_valid`/`rs2_valid`
gating above), so bit 4 of any *reported* `rvfi_rs1_addr`/`rvfi_rs2_addr`/`rvfi_rd_addr`
is exactly the E-illegal condition -- `assert(rvfi_trap)` whenever it is set, with no
per-class membership list for a second out-of-range field to hide behind. A `cover
property` proves a LOAD naming x16 as rd and x17 as rs1 at once is reachable and correctly
trapped -- the exact multi-field trace the reverted `ill_e`'s per-term isolation
conditions structurally could not see. `nano/formal/ill_e.sby` (mode bmc, depth 40) and
`nano/formal/ill_e_cover.sby` (mode cover, depth 100) both pass on the shipping core; the
cover goal is reached at step 7.

`nano/formal/ill-e-probe.py` is the forced-red control: it mutates `is_e_illegal` itself
in `nano.v`, bit 4 to bit 3 -- the mutation the reverted `ill_e`'s two `.sby` files both
survived -- and requires `ill_e.sby` to fail against it, after confirming the shipping
core passes first. Because `ill_e.sv`'s property is read entirely off RVFI and never
touches `nano.v`'s internals, this mutation is caught with no change to `ill_e.sv` at all:
a core that thinks x8 (bit 3 set, bit 4 clear) is illegal and x16 (bit 4 set, bit 3 clear)
is not retires x16 as ordinary arithmetic, and the checker's independent bit-4 read of
`rvfi_rd_addr` catches exactly that. Measured on this tree: shipping PASS, wrong-rule
mutant FAIL. `test/probe_gates.sh` exercises the script's own parsing and control logic
against a stub `sby`, the same standing `complete-cover-probe.py` has; its control case is
labelled `ill_e catches a wrong RV32E rule, and the shipping core still passes`, which
`test/ill_e_wiring_test.py` requires `test/PROBES_EXPECTED` to carry before
`RISCV_FORMAL_E` may appear in `checks.cfg` at all.

## The two area points

Both from `make nano-area`, on this tree, against the pinned liberty
(`ec0e1067a35c8bf20b11e58d1e8ac53326067e4dac84a125cc1b917a3518d0d9`, `sha256sum` of
`sky130_fd_sc_hd__tt_025C_1v80.lib`) and confirmed deterministic (an identical `area.json`
on a second run):

| Configuration | Local µm² |
| -- | -- |
| Donor (32 registers, unreshaped) | 84,290.8416 |
| Donor + RV32E (16 registers) | 60,758.272 |
| Difference | −23,532.5696 (−27.93%) |

`edfxtp_1` (the enabled-DFF cell the register file maps to) falls by exactly 512 -- 16
registers times 32 bits -- and every other moved cell type is addressing/muxing logic
around it (`mux4_2` 422→270, `nand2_1` 870→659, `nor2_1` 965→856, `o21ai_0` 828→595); no
unrelated cell type moved, which is the check that the saving is the register file and
not an accidental simplification elsewhere.

**RV32E did not come in short of its predicted upper bound -- it landed almost exactly on
it.** ADR-0177's −23,400 µm² was measured on register files synthesized alone, and the
concern it recorded was that fusing the read multiplexers into the surrounding datapath
could only shrink that number. What the real core shows is the opposite in size, if not
in direction: narrowing the register address from 5 bits to 4 shrinks not just the
register file's own flip-flops but every read-address decode tree fed by `rs1`/`rs2`/`rd`
throughout the design, wherever `regs[]` is read -- a saving that was never confined to
one module for the isolated measurement to have captured in full. The realized saving,
−23,532.6 µm², is 100.6% of the −23,400 µm² upper bound. Because the shortfall this ADR
was watching for did not happen, ADR-0177's cut order -- the one-port register file next,
then TinyQV's latch array -- is not reopened by this measurement.

**Projected fit, CSR/traps and the QSPI front end still unbuilt:** ADR-0177's brief
figure for those two together was about `+11,500 µm²` local. Applied to this reshape's own
measured base rather than the donor's, that projects to `84,290.8 + 11,500 - 23,532.6 ≈
72,258 µm²` local, `× 0.915 ≈ 66,116 µm²` in the layout flow -- against a 2×2 tile's
72,565 µm², essentially the same ~91% margin ADR-0177 projected from the upper-bound
figure, fractionally better. This is still a projection: the CSR/trap layer and the QSPI
front end are later tickets, and their own `make nano-area` figures are what will confirm
or move it.

## `NANO_MAX_UM2`

Steps down from `84291` to `60759` -- `nano/nano.mk`'s own margin convention (round the
measured figure up to the next whole µm², the same as the `84290.8416 → 84291` it
replaces; ADR-0169 declined to invent a churn band for this instrument for want of a
second data point, and this ADR does not add one either, since the two points it has are
different designs, not two measurements of one).

## F and G

`make -C nano/formal remeasure-fg` reproduces the declared `F = 12, G = 10` against the
reshaped core, both flip points found where `checks.cfg` says they are. Shrinking the
register file changes no path length in the state machine that generates fetch, decode,
execute or writeback cycles, so this is a null result rather than a surprise -- recorded
because CLAUDE.md requires the sweep rather than the argument for it.

## Consequence

Two graded surfaces move: `nano/nano.mk`'s `NANO_MAX_UM2` ratchets down to `60759`, and
`nano/formal/checks.cfg` turns `RISCV_FORMAL_E` on for the generated `insn_*` checks,
which is safe only because `ill_e` now stands behind it with a real forced-red control.
Nothing about `misa`, a CSR file, or the counters changes here -- those stay ADR-0177's
explicit non-cuts, owed to a later ticket.
