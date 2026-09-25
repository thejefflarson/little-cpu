# ADR-0209: The latch register file is removed; nano ships flip-flops only

**Status:** Accepted · 2026-09-25 · *Supersedes ADR-0189 and ADR-0207*

## Context

**This is the owner's decision, made 2026-09-25: remove `NANO_LATCH_RF` entirely and keep
the flip-flop register file that already ships.** Flops are the default build today, and
the fully verified one: all 76 generated riscv-formal checks pass against them with an
empty `nano/formal/EXPECTED_FAIL`.

`NANO_LATCH_RF` (ADR-0189) existed to buy area on a small Tiny Tapeout tile. Measured
against the local ranking instrument (`synth; dfflibmap; abc`, an ad hoc techmap standing
in for the real flow's `SYNTH_LATCH_MAP` step), the latch build read 50,870.04 µm² against
the flop build's 61,411.40 (−17.17%); a real LibreLane 4×2 harden read 70,070.95 µm²
against 81,879.78 (−14.4%). Re-measured on this tree with the current `make nano-timing`
recipe immediately before this change: 72,269.31 µm² (latches) against 83,145.99 µm²
(flops), a 10,876.68 µm² (13.1%) difference — the same order of magnitude ADR-0189
recorded, never merged with `make nano-area`'s own ratchet figure, which reads
78,382.6752 µm² (8,044 cells) for the shipping flop build regardless of what the latch
build measured.

That area win never became the default, because it never cleared three independent
problems, each recorded in ADR-0189 or ADR-0207 and none resolved since:

1. **Setup timing.** A timing measurement taken after ADR-0189 found the latch build's
   worst setup paths run from a register-file latch (a positive level-sensitive latch
   clocked by the inverted clock) to a flip-flop and so get only half a clock period: at
   the slow corner it misses setup by about 2 ns at every clock target tried (24, 27, and
   36 ns), because the budget scales with the period (ADR-0207, "The default stays flops,
   on two gates now").
2. **Formal tools cannot read `$dlatch`, and the workaround cannot be made sound for the
   generated checks.** yosys accepts the fifteen `$dlatch` cells cleanly through
   `prep -nordff`, then refuses them at the model-writing step (`please run clk2fflogic
   before write_btor`, or the identical complaint naming `write_smt2`) — ADR-0189's own
   finding. `clk2fflogic` fixes that but turns one clock cycle into two BMC steps
   (ADR-0040's Finding 3), at a measured 13-14x per-check wall-time cost. ADR-0207 gave
   nano's five hand-written harnesses that fix, but proved the generated riscv-formal
   ladder structurally cannot take it: `formal/genchecks-local.py` (vendored byte-for-byte
   from the pin, ADR-0031) writes one `[depth]` value into three places at once — sby's
   `skip`, sby's `depth`, and `` `define RISCV_FORMAL_CHECK_CYCLE`` — and no value can be
   simultaneously itself (for the real-cycle retire count) and its own double (for the
   `clk2fflogic` step budget). Forking the pinned file to fix it is exactly what ADR-0031
   forbids.
3. **A clock-LEVEL-enabled latch is a special case for every downstream tool.** Beyond the
   two points above: `dfflibmap` alone leaves every inferred `$_DLATCH_P_` unmapped and
   needs its own `techmap` step before `stat -liberty` will price it at all (`nano/nano.mk`
   carried `NANO_LATCHMAP_URL`/`NANO_LATCHMAP_SHA256` and a second pinned fetch for exactly
   this), and reading `!clk && sel_q` in an `always_latch` needed its own working-around of
   iverilog's four-state constant-select limitation, the same class of warning
   `rtl/writeback.v` is allowlisted for.

None of the three is a matter of more engineering time on the current design: (1) is a
physical fact about a latch-to-flop path at this part's corner, (2) is a structural
property of a file this repo does not own, and (3) is the recurring cost of carrying a
second storage element through every tool in the chain. The owner's call is to stop paying
that cost and keep the flip-flop build, which already meets 12 MHz-equivalent timing and is
formally verified end to end.

## Decision

**Remove every trace of `NANO_LATCH_RF` and keep the flip-flop register file exactly as it
ships today.**

- `nano/nano.v`: both `ifndef NANO_LATCH_RF` arms (the `regs[0] <= 0` reset and the
  `regs[rd[3:0]] <= reg_wdata` write) become unconditional; the `ifdef NANO_LATCH_RF`
  latch-array `generate` block and the RVFI `rvfi_rd_wdata_q` arm that read `reg_wdata`
  instead of `regs[rd[3:0]]` under the macro are deleted. Preprocessing the result with no
  macros set is identical to before this change (nothing inside an `ifndef` arm moved).
- `nano/formal/`: the seven `_latch`/`_latch_cover` `.sby` files ADR-0207 added
  (`ill_e_latch[.sby|_cover.sby]`, `dmemcheck_latch[.sby|_cover.sby]`,
  `imemcheck_latch[.sby|_cover.sby]`, `traps_latch.sby`) are deleted, along with the
  Makefile targets that built, cleaned or probed them (`all-latch`,
  `components_traps_latch`, every `*-latch-probe`/`*-latch-cover-probe` target) and the
  `--sby-file`/latch arguments `ill-e-probe.py` and `probe_common.py` carried only to
  select one of those files.
- `formal/check-memcheck-depth.py` and `formal/memcheck-cover-probe.py` lose the
  `--clk2fflogic` flag and the `imemcheck_latch`/`dmemcheck_latch` `--check` choices —
  the only call sites for either were nano's latch Makefile targets; littlecpu's own
  `formal/Makefile` never passed them, so this is pure removal, not a behavior change for
  littlecpu.
- `nano/nano.mk`, `nano/synth_script.sh`, `nano/timing_script.sh`, `nano/timing_report.py`:
  `make nano-timing` now runs one `yosys` synthesis (the flop build) instead of two in
  parallel, so the `techmap -map`/latchmap-quoting step `timing_script.sh` carried leaves
  with it. `NANO_LATCHMAP_URL`/`NANO_LATCHMAP_SHA256` and the `cells_latch_hd.v` fetch in
  `nano-liberty-setup` are deleted — that pin existed only to give the latch build's
  `$_DLATCH_P_` cells a price.
- `nano/tb.mk` and the top-level `Makefile`: `nano-latch-test`, `nano-latch-startup-test`,
  `nano-oneport-latch-test`, `nano-oneport-latch-startup-test` and their `.cc`/`.vvp` build
  rules are deleted, along with their entries in `make test`'s target list.
  `NANO_ONE_PORT_RF`'s own `nano-oneport-test`/`nano-oneport-startup-test` are untouched —
  a separate, still-live design question.
- `.github/workflows/nano-tt-area-selfhosted.yml`: the `regfile` workflow-dispatch input
  and the `NANO_LATCH_RF` `VERILOG_DEFINES` branch it fed are deleted; the workflow always
  hardens the flop build now.
- `test/probe_gates.sh`/`test/PROBES_EXPECTED`: every probe whose only job was grading a
  now-deleted latch code path is deleted; two probes that tested the (now-gone)
  `--sby-file`/`--clk2fflogic`/latchmap arguments are replaced with equivalents that grade
  the same guard logic (a missing `.sby`, a quoted path) through the arguments that remain.

## Consequences

- The flop build is unchanged in every measurable way: `make nano-area` reads the same
  8,044 cells / 78,382.6752 µm² against the same 80,500 µm² ratchet, and `make nano-timing`
  reads the same 83,145.99 µm² / 13,459.41 ps for the one build it now reports (both taken
  on origin/main immediately before this change and re-confirmed after). `make -C
  nano/formal check` still passes all 76 generated checks against an empty
  `EXPECTED_FAIL` — that harness was always flop-only (ADR-0207) and this change does not
  touch it.
- `make nano-timing` reports one build instead of two; its output no longer carries a
  `latches` row.
- The pinned `cells_latch_hd.v` fetch is gone from `nano-liberty-setup`, so `make
  nano-liberty-setup` and `make nano-timing` fetch and verify one file, not two.
- ADR-0189 and ADR-0207 are superseded: their decisions (ship a latch-array build behind
  `NANO_LATCH_RF`; extend nano's hand-written formal harnesses to read it) are reversed.
  Their measurements stay in place as the record of what was tried and why it did not
  clear the bar to become the default.
- ADR-0184's work order — latch register file, then shared-register mul/div, then a
  one-read-port register file, as steps toward fitting a small Tiny Tapeout tile — loses
  its first step. `NANO_ONE_PORT_RF` (ADR-0195, measured and shipped off by default) is
  unaffected by this ADR; it is a separate question this ADR does not reopen.
