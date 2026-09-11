# ADR-0174: The RV32E oracle patch, its diff-grader, and `ill_e`

**Status:** Accepted · 2026-09-10; `ill_e` reverted 2026-09-11, oracle patch kept

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

**The assumption is global, and `ill_e` has to cover everything it reaches.** The `ifdef`
block above applies to every generated `insn_*` check, not just the nine base-opcode
classes the first draft of `ill_e` covered: of the 70 checks the pin's `isa_rv32imc.txt`
generates, nine are compressed encodings that read a FULL 5-bit register field --
`c_add`, `c_addi`, `c_jalr`, `c_jr`, `c_li`, `c_lui`, `c_lwsp`, `c_mv`, `c_slli` -- and the
assumption would exempt those too the day it is turned on, whether or not `ill_e` says
anything about them. `ill_e.sv` now decodes those (see below); SYSTEM's CSR opcode is
similarly widened even though no `insn_*` check reaches it today (riscv-formal's own CSR
write checks go through `rvfi_csrw_check.sv`, a different file the assumption never
touches) -- covering it anyway means a future check family routed through
`rvfi_insn_check.sv` does not reopen this exact gap silently.

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
file is hand-drifted from the other. `test/probe_gates.sh` carries both cases -- the
diff-grader's own forced-red direction, since a grader with none is exactly the defect
class CLAUDE.md names.

## `ill_e`: the restriction itself, checked against a reference, not against nano.v yet

`nano/formal/ill_e.sv` states the property decision 9 names: an rv32i-legal encoding that
names a register above x15 in a decode-significant register field is not a legal RV32E
instruction and must retire as illegal -- trapped, `rd_addr == 0`, no memory write.

**It is not wired to `nano.v`.** `nano.v` is the unreshaped donor: it has 32 registers, so
nothing in it makes x16-x31 illegal, and it has no CSR/`mcause` register to compare a
specific cause against (ADR-0167: "no `mstatus`, no `mtvec`, ... no CSR entry mechanism at
all"). Wiring `ill_e` to `nano.v` today would either always fail (a permanent, uninformative
red, since a 32-register core legitimately retires every one of these encodings) or say
nothing about cause 2, which no signal in `nano.v` states. So `ill_e.sv` checks the property
against `ill_e_top`, a reference implementation of exactly the restriction -- correct by
construction.

**Twelve classes, not nine.** The nine base 32-bit opcode classes `complete.sv`'s own cover
goals also name (`LOAD`, `OP-IMM`, `AUIPC`, `STORE`, `OP`, `LUI`, `BRANCH`, `JALR`, `JAL`) --
a coincidence of overlap with that file's list, not a shared source, since `complete.sv`
also covers the three RVC quadrants generically and `ill_e.sv` covers no compressed opcode
class at that granularity at all. `ill_e.sv` adds: `MISC-MEM` (`fence`/`fence.i`), decoded
and deliberately kept out of the OR-list -- its `rd`/`rs1` bit positions are reserved, not a
register name, the same as `lui`/`auipc`/`jal`'s immediate bits -- with its own cover goal
showing the class is reachable at all rather than silently absent; `SYSTEM`'s two CSR
sub-shapes, `class_sys_reg` (`csrrw`/`csrrs`/`csrrc`, funct3 `001`/`010`/`011`: both `rd`
and `rs1` are registers) and `class_sys_imm` (`csrrwi`/`csrrsi`/`csrrci`, funct3
`101`/`110`/`111`: `rd` only -- the `rs1` field position holds a 5-bit immediate on these,
never a register); and three compressed classes reading a FULL 5-bit field rather than the
3-bit `x8`-`x15` one most RVC encodings use -- `class_c_cr` (`c.add`/`c.mv`/`c.jr`/`c.jalr`,
CR format), `class_c_ci` (`c.addi`/`c.li`/`c.lui`/`c.slli`/`c.lwsp`, CI format) and
`class_c_css` (`c.swsp`, CSS format), each gate matched field-for-field against the pinned
`insns/insn_c_*.v` spec models rather than derived from the spec text, since those models
are what the generated checks actually build against. `c.jalr`'s `rd` is hardwired to `x1`
and `c.lwsp`/`c.swsp`'s `rs1` to `x2` in those models -- neither is a decoded field, so
neither is checked; only bits the instruction word actually carries are.

**23 (class, field) memberships, each its own named term, each with its own cover goal.**
`e_illegal` is not `uses_rd`/`uses_rs1`/`uses_rs2` ORed with a class list behind each --
it is a flat OR of 23 terms named `ill_<class>_<field>` (`ill_op_rs2` is "OP reads rs2" and
nothing else), each ISOLATED against its siblings in the same class (the other field held
low), so `ill_op_rs2`'s own cover goal cannot be satisfied by `rd` or `rs1` going high
instead. This is the fix for a real gap an earlier draft had: covering only
`e_illegal && class_op`, say, would still pass if `class_op` were silently dropped from
`rs2`'s membership list, because `rd` or `rs1` going high on the same OP-class encoding
would still reach the shared goal. `nano/formal/ill_e.sby` (`mode bmc`, depth 2) grades the
assertion (three clauses: `trap`, `rd_addr == 0`, no memory write) against all 23 terms at
once; `nano/formal/ill_e_cover.sby` (`mode cover`, depth 2) grades the resulting 24 cover
goals (23 isolated memberships plus `MISC-MEM`'s structural one), the same
`complete.sby`/`complete_cover.sby` split this directory already uses. Both are wired into
CI as one `formal-extra` step, `make -C nano/formal ill_e ill_e_cover`.

**What a green `ill_e` does and does not mean.** The assign block that computes `trap`,
`rd_addr` and `mem_write` from `e_illegal` is the same expression the assert then checks --
`assign trap = e_illegal; ... assert(trap);` -- so those three clauses are TAUTOLOGIES by
construction, not properties a bug in `ill_e_top` could ever violate; `ill-e-probe.py`'s
mutations against them (below) demonstrate the PROBE'S OWN mechanism can go red, not that
the assertion is falsifiable against `ill_e_top` itself. What is a real, checkable claim is
`e_illegal`'s own definition: whether the 23 (class, field) memberships correctly enumerate
the restriction, which the drop-`<term>` mutations below test directly by removing one and
requiring exactly its own goal to be lost. **A green `ill_e` says the restriction as
enumerated is internally consistent and every membership is independently reachable --
never that `nano.v`, or any real core, has the restriction.** `test/ill_e_wiring_test.py`
(below) is the standing, mechanical statement of that boundary.

**This is a residual, stated rather than hidden.** Once the reshape gives `nano.v` real
x16-x31 trapping and the CSR/trap layer (brief sequence steps 3 and 5), swapping `ill_e_top`
for the real core -- the way `complete.sv` and `dmemcheck.sv` already wire `nano.v` in -- and
adding the cause-2 comparison this file does not yet make, is the natural next step.

**The forced-red direction.** `nano/formal/ill-e-probe.py` mutates `ill_e.sv` one text span
at a time, mirroring `nano/formal/complete-cover-probe.py`'s own shape, in two families:

- Three mutations target the assertion's three clauses (`trap`, `rd_addr`, `mem_write`)
  independently, tying each one to the value a core that got the restriction wrong would
  report -- "lets an x16-31 encoding retire normally" -- and require `ill_e.sby` to fail at
  that clause's own pinned line, found dynamically by text search rather than hardcoded.
- `e_illegal`'s OR-list is read out of the file by regex (`illegal_terms()`), never
  hardcoded, so the probe cannot itself drift from what the file states. `tie-low` ties the
  whole expression to a constant, losing every membership at once, as a cheap smoke test;
  `drop-<term>`, run once per term (23 runs), removes exactly one and requires exactly its
  own isolated cover goal -- found the same way, by reading the term's own
  `cover property (live && e_illegal && <term>);` line -- to go unreached, and no other.
  Both mutation functions preserve the exact line count of what they replace (padding
  `tie-low`'s single-line replacement back out, and never matching a newline in the
  targeted-removal regex `drop-<term>` uses), because every cover goal after that
  statement needs to keep its own line number for the drop-`<term>` check to mean anything.

**`tie-low`'s check is honest about what sby's cover engine actually reports.** Measured on
this tree, at both this file's `depth 2` and `complete_cover.sby`'s own `depth 100`: unlike
`complete-cover-probe.py`'s stalled-bus mutant, which reports all twelve of `complete.sv`'s
cover goals unreached at once (a single shared gate, `rvfi_valid`, forced permanently
false), `ill_e_cover.sby`'s `tie-low` mutant reports only ONE of the 23 e_illegal-gated
goals unreached before the job declares FAIL and stops -- because these 23 are 23
INDEPENDENTLY gated conditions, not one shared gate, and smtbmc's cover engine does not
keep searching for more once the overall verdict is already decided. `tie-low`'s own check
is written to that measurement: "at least one, and only a real e_illegal-gated one" -- a
cheap sanity check, not a precise one. The drop-`<term>` loop is what is precise, and it is
precise BECAUSE each run isolates exactly one membership rather than asking the same
engine to resolve 23 independent negatives in a single job.

Every mutant is built alongside the unmutated file, and the unmutated file is required to
pass all three assertions and reach every cover goal first -- a mutant that fails proves
nothing about a control that was never shown to pass. Measured on this tree: the shipping
reference passes every assertion and reaches all 24 cover goals; the three assertion
mutants and all 23 drop-`<term>` mutants each fail at exactly their own pinned line;
`tie-low` fails and names one real membership goal. `ill-e-probe.py` is wired as a real
prerequisite of BOTH `make -C nano/formal ill_e` and `ill_e_cover` -- `ill_e` alone proves
the assertion only against a hand-written reference with no design under it, so unlike
`complete`, which proves something real about `nano.v` without its own probe, `ill_e`
without the probe proves nothing on its own. CI still spends the solver once, not twice:
the two targets are one `make` invocation (`make -C nano/formal ill_e ill_e_cover`), and a
PHONY prerequisite two goals share in ONE invocation runs once, not once per goal.
`test/probe_gates.sh` exercises the script's own parsing and mutation logic against a stub
`sby` that reads the same 23 term names out of the fixture's `ill_e.sv`, so it can name
which single membership a `drop-<term>` mutant is missing generically rather than
special-casing one.

## The pin-bump path re-syncs two vendored files, not one

`formal/bump-riscv-formal-pin.sh` already regenerates `test/monitor.v` against a pin bump.
`nano/formal/rvfi_insn_check.sv` is a second vendored file with no generator -- it is a
hand-maintained fork, not a copy -- so the bump script now also runs
`nano/formal/check-rvfi-insn-check.py` against the upstream file AT THE PINNED SHA and
reports the result as its own section of the issue body: in sync, or **STALE** with the
grader's own diff attached (capped at 300 lines and fenced with tildes rather than
backticks, since the diffed file is SystemVerilog and its own preprocessor directives are
backtick-prefixed -- a triple-backtick run is not something a backtick fence can promise
never appears), so a human re-applying the `RISCV_FORMAL_E` block by hand has the exact
text to re-apply rather than a blank prompt to go find it. The upstream file is read with
`git show "$UPSTREAM_SHA:checks/rvfi_insn_check.sv"` against the clone, not from the
clone's own working tree: that tree sits at whatever upstream's default branch HEAD was AT
CLONE TIME, which can have moved past `$UPSTREAM_SHA` in the gap between the `ls-remote`
that resolved it and the `clone` moments later, and reading the working tree would then
silently grade the fork against a commit that is not the pin about to be written.
`formal/propose-pin-bump.sh` (a generic issue-opening wrapper, unchanged in behaviour) and
`.github/workflows/riscv-formal-pin-bump.yml` both gained a comment naming the second file,
so a future editor of either does not rediscover the coupling by reading
`bump-riscv-formal-pin.sh` cold. `monitor-freshness` -- which runs on every PR, the pin-bump
PR included -- is what makes a bump that leaves the fork stale go red on its own, whether or
not a human reads the issue's STALE section first.

## The tautology tripwire

`e_illegal`'s assign/assert pair is provably self-consistent, which is exactly why it
cannot be the only thing standing between "the assumption is safe" and "the assumption is
live and nothing checks it": the day `RISCV_FORMAL_E` is wired into
`nano/formal/checks.cfg` for real (the reshape's job, not this ticket's), the restriction
it relies on is checked by nothing unless `ill_e.sv` has ALSO been swapped from `ill_e_top`
to the real core by then. `test/ill_e_wiring_test.py` is a standing, hermetic, `make
test`-path check for exactly that pairing: red if `RISCV_FORMAL_E` appears anywhere in
`checks.cfg` while `ill_e.sv` does not instantiate the real core (`riscv wrapper (`, the
same instantiation `complete.sv` and `dmemcheck.sv` already use). It is a no-op today,
since neither half is true yet, and says so. It checks wiring, not class coverage,
because the widening above already makes coverage the settled question rather than a
moving one: `ill_e` decodes every register-field shape the pin's 70 generated checks and
`rvfi_insn_check.sv`'s own routing can reach today, so there is no narrower-than-the-
assumption state left to fail closed against -- only a future pin bump adding a check
family this file has not yet been taught to decode, the same residual a pin bump always
owes `formal/COMPLETE_EXCLUSIONS` and the sanitizer's site counts.

## Consequence

**This patch remains the one place in nanocpu where a green result can mean "stopped
asking"**, exactly as the brief warns: every OTHER graded comparison in this tree states a
property against the design under test, and a bug in the grader is caught by the same kind
of forced-red probe that catches a bug in the design. Here, the grader IS partly the design
of the oracle itself, and the only things standing between "assumes correctly" and "assumes
away the property" are the diff-grader's byte equality against the pin, `ill_e`'s own proof
that its 23 memberships are each independently reachable and none is redundant with
another, and the wiring tripwire that keeps the day the assumption goes live from being the
day nothing checks it. None of the three is a claim about `nano.v`; all three are built and
green on this tree today, at the current pin, with `nano.v` unreshaped. Wiring
`RISCV_FORMAL_E` into a real generated E check set, and swapping `ill_e_top` for the
reshaped `nano.v`, are the reshape's to do, not this ticket's.

## Amendment · 2026-09-11 · `ill_e` reverted, oracle patch kept

**`ill_e` is reverted.** It could not tell a right RV32E rule from a wrong one: changing
every "register at x16 or above" test in `ill_e.sv` to test bit 3 instead of bit 4, 23
sites, left both `ill_e.sby` and `ill_e_cover.sby` passing. Only breaking one of the
file's own `assign` lines against the `assert` below it made the check fail, which is
this ADR's own tautology admission read the other way round: the three assertion clauses
compared three `assign` lines with three `assert` lines restating them, and the
mutation probes that were supposed to carry the safety case only ever showed that the
*probe* could go red, never that the *property* could be falsified against `ill_e_top`.
The reference model made the same mistake at the term level: each of the 23 (class,
field) memberships carried an isolation condition so it could own its own cover goal
(`ill_load_rd` required `!rs1[4]`), so a load naming x16 *and* x17 matched no term at all
and was not flagged illegal, even though the stated property covers it. Nothing noticed,
because nothing compared the model against anything but itself. `nano/formal/ill_e.sv`,
`ill_e.sby`, `ill_e_cover.sby` and `ill-e-probe.py` are removed, along with their
Makefile targets, CI step and probe group.

**The oracle patch and its diff-grader are kept, because nanocpu is now going RV32E.**
The repo owner decided the cut on area: keeping every planned feature costs about
87,700 um2 in Tiny Tapeout's layout flow, against a 2x2 tile's 72,565 um2 of core area,
so RV32E -- 16 registers instead of 32 -- is the one cut taken, with M and the
mcycle/minstret counters kept. That reverses the premise this ADR shipped under, that
nanocpu would stay RV32I and the patch would have nothing to grade. The patch itself was
never the problem: `nano/formal/rvfi_insn_check.sv`'s one `ifdef RISCV_FORMAL_E` block,
`check-rvfi-insn-check.py`'s byte-equality grader, and the pin-bump path's re-sync are
untouched and still needed the day the generated checks turn the assumption on.

**The wiring tripwire now enforces the wrong-rule direction mechanically, not in
prose.** `test/ill_e_wiring_test.py` still refuses to let `RISCV_FORMAL_E` go live in
`nano/formal/checks.cfg` without a real-core `ill_e.sv` -- the `riscv wrapper (`
instantiation `complete.sv` already uses -- wired in. It now also refuses unless
`test/PROBES_EXPECTED` names a forced-red probe containing the phrase "a wrong RV32E
rule": a probe proving the future `ill_e` catches a mutated rule, not just its own
`assign`/`assert` pair. Stating that requirement in an error message was tried and
rejected: this ADR already stated in writing that its assertions were tautologies, and
shipped anyway, so a sentence is not the bar. A label in `PROBES_EXPECTED` is, because
`make probe-gates` requires every listed label to run and go red for its own reason --
requiring the label requires a working probe.

**`nano/nano.v` never changed while any of this was built.** `make nano-area` reads
84,290.8 um2 on `0f66638`, the donor's own figure to the decimal. The general lesson is
that a grader built ahead of the thing it grades is only worth something once that thing
is decided -- true whether the decision lands on "not now" or, as here, on "yes, cut
this".
