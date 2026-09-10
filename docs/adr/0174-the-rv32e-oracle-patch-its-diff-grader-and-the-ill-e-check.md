# ADR-0174: The RV32E oracle patch, its diff-grader, and `ill_e`

**Status:** Accepted · 2026-09-10

## Context

riscv-formal ships no RV32E spec model -- the pin has `isa_rv32i/ic/im/imc` and Zb* and
nothing narrower. `docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md` (decision 9) names
the fix as a repo-owned copy of `checks/rvfi_insn_check.sv`, differing from the pin by
exactly one `ifdef` block, graded by a diff the way `formal/check-genchecks.py` grades
`formal/genchecks-local.py`, plus a hand-written `ill_e` check for the restriction itself.
The brief calls this patch "the only silent-removal surface in the design": every other
graded comparison in this tree states a property against the real core; this one edits the
FILE THAT STATES THE PROPERTIES, and a patch that assumes too much makes a broken E core
read as correct with nothing going red to say so.

`nano/nano.v` is still the plain file copy of the donor commit `c55efd6` (ADR-0167) -- a
32-register RV32IMC core with no register-count restriction and no CSR/`mcause` layer. The
reshape to RV32E (the brief's sequence step 3) has not landed. This ADR builds the
infrastructure the reshape will need -- the patch, its grader, and `ill_e` -- ahead of it,
the same order ADR-0167/ADR-0168 built nanocpu's whole riscv-formal harness ahead of the
donor being correct in any ISA this repo ships.

## The patch: one `ifdef` block, on the spec model's own decoded addresses

`nano/formal/rvfi_insn_check.sv` is the pinned `checks/rvfi_insn_check.sv`
(`c992aa61fdfe0846c5ed90324c596202a1c69b76`, the SHA `formal/pin.mk` names) plus one block,
right after the existing `assume(spec_valid);`:

```systemverilog
`ifdef RISCV_FORMAL_E
    assume(spec_rs1_addr < 16 && spec_rs2_addr < 16 && spec_rd_addr < 16);
`endif
```

**Why it has to sit there and nowhere else.** A generated `insn_add` check, say, asks
whether the DUT's `add` retire agrees with the reference model's on every reachable
encoding -- and the reference model does not know E exists, so it is free to claim
`spec_valid` for `add x20, x5, x3`, a register RV32E does not have. A correct E core traps
that encoding as illegal (`nano/formal/ill_e.sv`, below), which makes
`assert(spec_trap == trap)` fail for a core that did exactly what it should: the encoding
was never invalid arithmetic, only an invalid *register*, a question `insn_add`'s own model
has no opinion on. The assumption defers that question to `ill_e` by keeping BMC inside the
register range E actually has for THIS check family, rather than asking `insn_add` to also
be an illegal-instruction check.

**Why the bound is `spec_rs1_addr`/`spec_rs2_addr`/`spec_rd_addr` and never `rvfi_insn`'s
raw bits.** `lui`, `auipc` and `jal` have no source register at all -- bits [19:15] and
[24:20] are immediate bits on those encodings, not a register name -- and `csrr*i`/`fence`
are similar. An assumption spelled on those raw bit ranges would silently narrow the
immediate space BMC explores on encodings the check appears to still cover in full, which
is exactly the brief's "silently removes immediate space" failure mode: nothing goes red,
the check set is unchanged, and a whole corner of `lui`'s encoding space is simply never
tried again. `spec_rs1_addr` and `spec_rs2_addr` are already 0 on every encoding that does
not read a register -- `rvfi_insn_check.sv`'s own `rs1_rdata_or_zero`/`rs2_rdata_or_zero`
convention, lines 68-69, and every `rvfi_isa_*` model's convention behind them -- so
bounding the DECODED address touches only encodings that genuinely name a register, and
touches all of them.

## The diff-grader

`nano/formal/check-rvfi-insn-check.py` is modelled directly on
`formal/check-genchecks.py`, for the reason that file gives for not diffing and eyeballing
the result: it UNDOES the one documented edit -- the `ifdef` block, matched as one
contiguous string -- and the fork's own added header paragraph, then requires byte
equality with the pinned clone. A residual diff is drift by construction, and it is
printed. `make -C nano/formal check-rvfi-insn-check` is the target; it runs in CI as a new
step in the `monitor-freshness` job, beside `make -C formal genchecks-check`, since neither
needs the OSS CAD Suite, only the riscv-formal clone and Python.

Tested against the real files in this tree: the grader reports a match at the current pin,
and reports the actual unified diff (undoing only the documented edits first) when either
file is hand-drifted from the other.

## `ill_e`: the restriction itself, checked against a reference, not against nano.v yet

`nano/formal/ill_e.sv` states the property decision 9 names: an rv32i-legal, uncompressed
encoding that names a register above x15 in a decode-significant `rd`/`rs1`/`rs2` field is
not a legal RV32E instruction and must retire as illegal -- trapped, `rd_addr == 0`, no
memory write.

**It is not wired to `nano.v`.** `nano.v` is the unreshaped donor: it has 32 registers, so
nothing in it makes x16-x31 illegal, and it has no CSR/`mcause` register to compare a
specific cause against (ADR-0167: "no `mstatus`, no `mtvec`, ... no CSR entry mechanism at
all"). Wiring `ill_e` to `nano.v` today would either always fail (a permanent, uninformative
red, since a 32-register core legitimately retires every one of these encodings) or say
nothing about cause 2, which no signal in `nano.v` states. So `ill_e.sv` checks the property
against `ill_e_top`, a reference implementation of exactly the restriction -- correct by
construction, a handful of lines of combinational decode over the nine base-ISA opcode
classes (`LOAD`, `OP-IMM`, `AUIPC`, `STORE`, `OP`, `LUI`, `BRANCH`, `JALR`, `JAL`, the same
list `complete.sv`'s own cover goals already use) -- with a cover property per class, so the
check is shown reachable through every shape of encoding that can name a high register, not
merely passing because BMC never tried one. `nano/formal/ill_e.sby` (`mode bmc`, depth 2)
grades the assertion; `nano/formal/ill_e_cover.sby` (`mode cover`, depth 2) grades the nine
cover goals, the same `complete.sby`/`complete_cover.sby` split this directory already uses.
Both are wired into CI as new `formal-extra` steps.

**This is a residual, stated rather than hidden.** Once the reshape gives `nano.v` real
x16-x31 trapping and the CSR/trap layer (brief sequence steps 3 and 5), swapping `ill_e_top`
for the real core -- the way `complete.sv` and `dmemcheck.sv` already wire `nano.v` in -- and
adding the cause-2 comparison this file does not yet make, is the natural next step. Until
then, a green `ill_e` says the PROPERTY is sound and non-vacuous, not that `nano.v` has it.

**The forced-red direction.** `nano/formal/ill-e-probe.py` mutates `ill_e.sv` one line at a
time, mirroring `nano/formal/complete-cover-probe.py`'s own shape. Three mutations target the
assertion the property states (`trap`, `rd_addr`, `mem_write`), each independently, so no clause
of it is checked only by construction:

- `no-trap` ties `trap` to a constant regardless of `e_illegal` -- "lets an x16-31 encoding
  retire normally", the literal defect class the check exists to catch -- and requires
  `ill_e.sby` to fail at the pinned `assert (trap);` line, not at some other line.
- `no-rd-clear` ties `rd_addr` to the raw field regardless of `e_illegal` -- an E-illegal
  retire that still writes a register -- and requires `ill_e.sby` to fail at
  `assert (rd_addr == 5'd0);`.
- `no-mem-clear` ties `mem_write` to the raw store decode regardless of `e_illegal` -- an
  E-illegal retire that still writes memory -- and requires `ill_e.sby` to fail at
  `assert (!mem_write);`.
- `no-illegal` ties `e_illegal` to a constant, so no encoding is ever E-illegal, and
  requires `ill_e_cover.sby` to leave at least one of the nine cover goals the shipping
  file reaches unreached.

Every mutant is built alongside the unmutated file, and the unmutated file is required to
pass all three assertions and reach every cover goal first -- a mutant that fails proves
nothing about a control that was never shown to pass. Measured on this tree: the shipping
reference passes every assertion and reaches all nine cover goals; `no-trap`, `no-rd-clear`
and `no-mem-clear` each fail at their own pinned line, found dynamically rather than
hardcoded, so a future edit to the file cannot make the probe pin the wrong one silently;
`no-illegal` fails and names an unreached site. `ill-e-probe.py` is wired as a real prerequisite of
`make -C nano/formal ill_e_cover` only -- not of `ill_e` alone, the same asymmetry
`complete_cover`'s own probe has against `complete`, so the two CI steps together spend the
solver once rather than twice -- and `test/probe_gates.sh` exercises its own parsing and
mutation logic against a stub `sby`, sixteen labels, all in `test/PROBES_EXPECTED`.

## The pin-bump path re-syncs two vendored files, not one

`formal/bump-riscv-formal-pin.sh` already regenerates `test/monitor.v` against a pin bump.
`nano/formal/rvfi_insn_check.sv` is a second vendored file with no generator -- it is a
hand-maintained fork, not a copy -- so the bump script now also runs
`nano/formal/check-rvfi-insn-check.py` against the freshly-cloned upstream file at the new
SHA and reports the result as its own section of the issue body: in sync, or **STALE** with
the grader's own diff attached, so a human re-applying the `RISCV_FORMAL_E` block by hand
has the exact text to re-apply rather than a blank prompt to go find it.
`formal/propose-pin-bump.sh` (a generic issue-opening wrapper, unchanged in behaviour) and
`.github/workflows/riscv-formal-pin-bump.yml` both gained a comment naming the second file,
so a future editor of either does not rediscover the coupling by reading
`bump-riscv-formal-pin.sh` cold. `monitor-freshness` -- which runs on every PR, the pin-bump
PR included -- is what makes a bump that leaves the fork stale go red on its own, whether or
not a human reads the issue's STALE section first.

## Consequence

**This patch remains the one place in nanocpu where a green result can mean "stopped
asking"**, exactly as the brief warns: every OTHER graded comparison in this tree states a
property against the design under test, and a bug in the grader is caught by the same kind
of forced-red probe that catches a bug in the design. Here, the grader IS partly the design
of the oracle itself, and the only thing standing between "assumes correctly" and "assumes
away the property" is the diff-grader's byte equality against the pin, plus `ill_e`'s own
proof that the property it states is non-vacuous. Both are built and green on this tree
today, at the current pin, with `nano.v` unreshaped; wiring `RISCV_FORMAL_E` into a real
generated E check set, and swapping `ill_e_top` for the reshaped `nano.v`, are the reshape's
to do, not this ticket's.
