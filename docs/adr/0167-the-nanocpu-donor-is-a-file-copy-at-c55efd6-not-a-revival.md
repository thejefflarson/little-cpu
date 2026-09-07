# ADR-0167: The nanocpu donor is a file copy at `c55efd6`, not a revival

**Status:** Accepted · 2026-09-07

## Context

`docs/ideas/nanocpu-a-verified-core-on-a-2x2-tile.md` (decision 13, and the
"donor" section) names `c55efd6` -- this repo's own history, the multicycle
RV32IMC core the current pipeline replaced -- as the shape nanocpu independently
arrived at, and directs that it be imported as a plain file copy rather than
resurrected on a branch. This ADR records the import: what came in, what did
not, and what changed on the way in.

## What was imported

`git show c55efd6:riscv.v > nano/nano.v` -- one 704-line file, byte-identical to
that commit (`git show c55efd6:riscv.v | diff - nano/nano.v` is empty), no
branch, no shared git history with the commit it came from. `nano/formal/` is a
new harness that shares this repo's `formal/pin.mk` (so the SHA-pinned
riscv-formal clone is the one clone, not a second one) and
`formal/genchecks-local.py` (so the vendored, header-diff-only copy of upstream
`genchecks.py` stays singular). Everything else under `nano/formal/` --
`checks.cfg`, `wrapper.v`, `dmemcheck.sv`/`dmemcheck.sby`,
`imemcheck.sv`/`imemcheck.sby`, `complete.sv`/`complete.sby`,
`remeasure-fg.py`, `EXPECTED_FAIL`, `EXPECTED_CHECKS` -- is nanocpu's own,
adapted from the donor commit's `formal/` tree and from this repo's current
`formal/` conventions. `nano/formal/checks.cfg` reaches the donor file the same
way `formal/checks.cfg` reaches `rtl/*.v`: `@basedir@` in the shared
`genchecks-local.py` always names the absolute path of the shared
`formal/riscv-formal` clone, and a plain relative path off it --
`@basedir@/../../nano/formal/wrapper.v`, `@basedir@/../../nano/nano.v` --
reaches into `nano/` the same way `formal/checks.cfg`'s own
`@basedir@/../../rtl/decoder.v` reaches `rtl/`. No template, no generated file,
no path substitution: `checks.cfg` is tracked directly.

## Where the donor conflicts with the brief, the brief wins

The donor is RV32IMC with a picorv32-style valid/ready bus and a `trap` output
that is a terminal halt state -- no `mstatus`, no `mtvec`, no `mepc`, no
`mret`, no CSR entry mechanism at all beyond the `mcycle`/`minstret` counters
riscv-formal's own checks exercise. The brief's design is RV32EC with the QSPI
parcel front end replacing the bus and the CSR/trap layer the donor lacks.
Both of those replacements are future work (brief steps 3-5); this ticket only
imports the donor and gets its OWN harness green at the current pin, unchanged
in ISA or bus shape. Where they conflict, the decision -- not the donor text --
is what ships once the reshape starts.

## The file is unmodified, not edited on the way in

The donor's declaration order -- `regs`, `pc`, `rd`, `rs1`, `rs2` are declared
at line 242, well after `regs[rs1]` is read inside a jump-address mux 135
lines earlier, the same declare-after-use pattern recurring at every decode
flag in the file -- makes iverilog 13 segfault while elaborating it (`ivl:
Segmentation fault`, no diagnostic). An earlier pass at this ticket reordered
five declarations to turn that crash into a clean `declaration after use`
compile error, which is progress on its own but not a fix, since the same
pattern recurs throughout the rest of the file and fixing all of it is the
rewrite the brief's reshape steps do, not this import. Reordering even five
declarations already makes the file diverge from `c55efd6:riscv.v`, and this
ticket's own instructions are explicit that a donor import that does not match
its source commit exactly takes the pristine copy instead -- so `nano/nano.v`
ships as the unmodified file, and **nanocpu has no four-state (iverilog) leg
yet**, the same conclusion ADR-0168 already reaches for other reasons.
cxxrtl and riscv-formal, this repo's other two legs, do not see the ordering
at all -- they build a signal graph, not a top-to-bottom program -- so neither
the formal checks below nor a future cxxrtl runner are affected; only the
X-visibility CLAUDE.md documents iverilog as uniquely providing is absent
until a later reshape pass reorders the file for real. This is a known Icarus
Verilog limitation (SystemVerilog `logic` does not require
declaration-before-use the way Verilog-2001 `reg`/`wire` conventionally do),
not a nanocpu defect; it has not been filed upstream from this change, since
doing so needs a minimal reduction independent of this repo's tree, which is
follow-up work this ADR records rather than closes.

**No line in the imported file is edited.** Comment density measures 2.65%
(17 of 642 graded lines), under `test/comment_density_test.py`'s 5% budget, so
`nano/nano.v` needs no exemption and the donor's 2020-era comments stay as
they were rather than being rewritten to look native.

## Verified against the current riscv-formal pin, on a complete run

Measured on this tree, yosys 0.68, riscv-formal
`c992aa61fdfe0846c5ed90324c596202a1c69b76` (the pin `formal/pin.mk` already
names):

- `make -C nano/formal checks` generates **79 checks** (one more than the
  brief's 78: this harness adds a `hang` family the donor's 2020 config never
  requested, needed to measure F -- see ADR-0168) and, run to completion,
  **77 pass, 2 fail**. `csrw_mcycle_ch0` and `csrw_minstret_ch0` fail,
  matching the brief's finding that those two were enabled in the donor
  commit and never passing; nothing else regressed. `nano/formal/EXPECTED_FAIL`
  and `EXPECTED_CHECKS` baseline exactly that, graded both ways by
  `../../formal/check-baseline.sh`, the same script `formal/Makefile` uses --
  confirmed by running it: "79 checks: 77 pass, 2 fail... Failure list matches
  EXPECTED_FAIL exactly... Generated check set matches EXPECTED_CHECKS
  exactly."
- **An earlier pass at this ticket reported 71 pass / 2 fail / 6 no-status,
  and that number was never a measurement of the design.** Six checks
  (`insn_sb_ch0`, `insn_sh_ch0`, `insn_sll_ch0`, `insn_slli_ch0`,
  `insn_slt_ch0`, `insn_slti_ch0` -- alphabetically consecutive, i.e. the run's
  own next in-flight batch) show `engine_0: finished (returncode=-9)` in their
  logs: `btormc` was killed by SIGKILL mid-BMC, not refuted or exhausted.
  That is a process that died with the session, not a property that failed --
  `check-baseline.sh` itself refuses to let ERROR or NO-STATUS be baselined as
  a known failure for exactly this reason (`baselineable_status` in
  `formal/check-baseline.sh`, ADR-0036). Re-running the full generate-and-run
  step to completion in this worktree reproduces 77 PASS / 2 FAIL / 0 other
  of 79 every time, matching the brief's own 76-of-78 figure once the one
  added `hang` check is accounted for (76 + 1 = 77 of 78 + 1 = 79). The
  number this ADR baselines is the completed run, not the interrupted one.
- `make -C nano/formal complete` -- the whole-ISA BMC walk -- needed its engine
  line fixed: the donor's `mode bmc` / `aigsmt z3` / `abc bmc3` combination
  errors on today's pin with an engine-line syntax drift the brief already
  named. `btor btormc` at `depth 20` is what the brief measured working, and
  it passes here too (kmax 19, 4 s on this machine).
- `make -C nano/formal dmemcheck` and `make -C nano/formal imemcheck` both
  pass unchanged from the donor's own `.sby` shape, only with paths rewritten
  to the shared clone and to `nano/nano.v`.

The sky130 area figure the brief quotes (84,291 um2, 1,491 flops) is not
re-measured by this ticket: no OpenLane/sky130 flow exists in this repo yet
(brief sequence step 1, "instruments before design", is still open), so there
is nothing here to run it against. It is reported as the brief's own number,
inherited rather than re-taken, exactly the distinction CLAUDE.md asks every
other inherited figure to carry.

## Consequence

`nano/nano.v` and `nano/formal/` exist as a green, current-pin baseline: every
later reshape step (E, no-M, the QSPI front end, CSRs and traps) is graded
against this check set and `complete` as it lands, rather than a fresh core
being built and only then checked. `nano/formal/` targets are wired into this
repo's `formal-checks`/`formal-extra` CI jobs as new steps, never `make
test`'s path -- see ADR-0168 for which legs nanocpu joins and when.
