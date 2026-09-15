# ADR-0188: nano's register file becomes a latch array under `NANO_LATCH_RF`

**Status:** Accepted · 2026-09-14

## Context

Today's flip-flop `nano/nano.v` does not fit a small Tiny Tapeout tile in the real
`nano-tt-area-selfhosted.yml` flow (LibreLane 3.0.14 on the self-hosted runner image, `AREA 0`,
`disallow_congestion=true`): a 3×2 fails detailed placement (DPL-0036, 85.723%
utilisation) and a 4×2 (8 tiles) fails global routing. **A separate wiring-ablation study
on this same tree, on this same flow, found the register file is the largest single
contributor to that 4×2 routing demand**: cutting it from sixteen entries to two took
demand from 101.05% to 54% and cut synthesis area by 23,197 µm², and traced two specific
structures behind it — the four low read-address bits (`rs1[3:0]`/`rs2[3:0]`) each drive
128 `mux4` selects, and 515 of the flip-flop build's 941 post-CTS hold-fixing buffers
terminate at a register-file flop, 408 of those through the per-bit write-enable mux2 fed
by `reg_wdata`. ADR-0179 already named a latch array as the second fallback if the
flip-flop design could not close, "declined for v1 over Tiny Tapeout's hold fixing and
yosys's formal model of latches." A local (non-TT-flow) `make nano-area` measurement on
an earlier tree read a latch register file at 49,377.36 µm² against 60,758.27 for the
flip-flop one, −18.7%, but had never been through the real flow this ADR was meant to run
it through.

**The runner pool (`little-cpu-runners`) was down for this ADR's whole working session**
(`gh run list --status in_progress`/`--status queued` read empty and a same-day,
unrelated PR's CI job sat `queued` for three hours) — the flow numbers below are therefore
the flip-flop 4×2 baseline already on file (LibreLane 3.0.14, run 34848744663) plus this
session's local structural analysis, and the latch variant's own routed numbers on 4×2,
2×2 and 3×4 are follow-up work, dispatched the moment the pool is moving again.

## Decision

`nano/nano.v` gains a `NANO_LATCH_RF`-selected register file, kept inline behind an
`ifdef` rather than split into a second file. Every one of nano's mutation and probe
scripts (`test/probe_gates.sh`, `formal/memcheck-cover-probe.py`,
`nano/formal/complete-cover-probe.py`, `nano/formal/ill-e-probe.py`,
`nano/tb/nano_exec_probe.sh`, `nano/tb/nano_x_probe.sh`) and every `nano/formal/*.sby`
file's `[files]`/`[script]` section assume `nano/nano.v` is the one and only RTL source
they copy or read; a second file would need touching every one of them for no benefit
the `ifdef` does not already give, and "its own file... if that reads best" in the
brief's own wording left that call open.

**The design**: sixteen transparent-high latches, matching the scheme TinyQV (the
brief's own cited example) and lowRISC ibex's `RegFileLatch` both use. `cpu_state`,
`rd` and `reg_wdata` are flops clocked by the same `clk` a latch here is transparent
under, so each is stable for the whole high phase after settling — the same split an
ordinary flip-flop already makes internally between its two internal latches, transparent on opposite clock phases.
Register 0's latch is gated on `cpu_state == fetch_instr` (zeroing it, matching the
flip-flop build's own `regs[0] <= 0` every fetch); every other register's is gated on
`cpu_state == reg_write && rd[3:0] == <its index>`. Select and data are read from plain
per-generate-instance wires rather than a bit-select inside the `always_latch` process
itself — iverilog's four-state frontend cannot fully evaluate a constant select inside
an `always_*` process (the same class of warning `rtl/writeback.v` is already
allowlisted for in CLAUDE.md) and over-widens the sensitivity list without one.

**The default build is unchanged.** Every line this ADR adds sits either inside an
`ifndef NANO_LATCH_RF` wrapping an existing statement, or inside a new
`ifdef NANO_LATCH_RF` block; preprocessing `nano/nano.v` with no macros set reproduces
origin/main's file verbatim (`iverilog -E`, diffed modulo the blank lines it pads in for
line-number parity), and `synth; dfflibmap; abc` against sky130hd lands on the identical
61,411.3984 µm² both before and after.

**Tests.** `nano-latch-test` and `nano-latch-startup-test` build the latch variant
through both of nano's simulator legs (`nano-latch-sim`/cxxrtl,
`nano/tb/nano_icarus_latch.vvp`/iverilog) and grade it with the exact same graders as
`nano-test`/`nano-startup-test` — `nano/asm/EXPECTED_FAIL`/`OBSERVED_FLOOR`, dual-leg
agreement, the startup PASS/FAIL — introducing no new comparison logic, so no new
`test/probe_gates.sh` entry is owed beyond the ones that already force those graders
red. Both join `make test`; the added cost is nano-test's own (about 20s locally),
since it is the same suite built twice. Both variants retire the identical instruction
counts on all six suite programs and the startup check (18 retires), on both simulator
legs.

**Formal cannot read the latch variant as-is.** Probed directly (not fixed, per the
brief): elaborating `nano/nano.v` under `NANO_LATCH_RF` through the exact script
`nano/formal/ill_e.sby` and `nano/formal/dmemcheck.sby` run (`prep -nordff`, `flatten`)
produces fifteen `$dlatch` cells cleanly through `check`, but both backends nano's
checks depend on refuse them at the write step: `ERROR: Unsupported cell type $dlatch
for cell ... -- please run clk2fflogic before write_btor` (the `btor btormc` engine
`ill_e.sby`/`complete.sby` use) and the identical error naming `write_smt2` (the
`smtbmc` engine `dmemcheck.sby`/`imemcheck.sby` use, and what the generated
`checks.cfg` family also resolves to). Making the latch variant checkable would mean
adding `clk2fflogic` (yosys's own suggestion) to the shared script every `.sby` file and
`nano/formal/checks.cfg`'s `[script-sources]` build from, verifying it does not also
widen what those checks can prove about the *flip-flop* build's flops (`clk2fflogic`
changes how every clocked element is modelled, not only latches), and re-deriving F/G
since the technique changes how the solver sees a cycle boundary. None of that is done
here.

## What the latch array structurally changes, against the wiring ablation's two findings

Neither of these is a routed-flow number (the runner pool was down); both are read off
a local `synth; dfflibmap; techmap <dlxtp map>; abc` run against the same sky130hd
liberty `make nano-area` uses, comparing the flip-flop and `NANO_LATCH_RF` builds cell
for cell. `dfflibmap` alone leaves every inferred `$_DLATCH_P_` unmapped (it targets
`$dff`-family cells only), so an ad hoc `techmap` mapping `$_DLATCH_P_` straight to
`sky130_fd_sc_hd__dlxtp_1` stands in for the real flow's `SYNTH_LATCH_MAP` step here;
it is not that step, and the resulting 50,203.15 µm² (−18.25% against the flip-flop
build's 61,411.40) is offered only as a sanity check against the historical 49,377.36
vs 60,758.27 (−18.7%) figure, not as this ADR's area answer.

- **The write-enable mux2 the ablation traced 408 of 941 hold buffers through has no
  equivalent in the latch build.** `sky130_fd_sc_hd__mux4_2` goes 268 → 0 and
  `sky130_fd_sc_hd__edfxtp_1` (the enable-flop `edfxtp` being excluded from the PDK's
  usable-cell list forces every gated flop to synthesise as, plain flop plus mux) goes
  792 → 280, a fall of 512 cells, replaced by 480 `sky130_fd_sc_hd__dlxtp_1` latches
  (sixteen registers × 32 bits is 512 bits; the missing 32 are register 0's, whose
  latch data input is the literal constant `regs[0] <= 0` and optimises to a tied-low
  net in both builds once the mapper sees it, which is why plain `dfxtp_1` barely moves,
  185 → 177). A latch holds its value with no logic at all when its own select is low,
  which is the mechanism the write-side hold-vs-load mux existed to provide for a plain
  flop; removing the mux removes what those 408 buffers were fixing hold on. This is an
  inference from the cell census, not a measured hold-buffer count for the latch build —
  that number is owed from the real flow.
- **The read-address fan-out is untouched by this change, on purpose.** `nano/nano.v`'s
  roughly twenty `regs[rs1[3:0]]`/`regs[rs2[3:0]]`/`regs[rd[3:0]]` read sites are
  byte-identical text under both builds — `NANO_LATCH_RF` touches only the two write
  statements and adds the latch generate block, never a read expression. So the same
  four address bits drive the same set of reads either way; the local census shows no
  surviving `mux4_2` cell in either build's read logic (both land on 0 for that
  specific cell type), meaning ABC maps that 16-way read select through AOI/OAI gates
  rather than a discrete mux primitive in this liberty regardless of the register
  file's storage element, so this census cannot separate "unchanged" from "remapped
  differently." The routing-demand share the ablation attributed to read fan-out is
  expected to persist into the latch build; only a routed run says by how much.

## Measurement

Dispatched on this branch via `nano-tt-area-selfhosted.yml`'s `regfile` input
(`AREA 0`, `disallow_congestion=true`, `stop_after_synthesis=false`), one run at a time
(the workflow's own concurrency group cancels a second dispatch on the same ref), on
LibreLane 3.0.14 throughout.

| Tiles | Regfile | Chip area (µm²) | Placement util. | GRT demand | DRT violations | Wall time | Run |
|---|---|---|---|---|---|---|---|
| 4×2 | flops | 81,879.78 | 63.638% | 101.05% | — (routing failed) | — | [34848744663](https://github.com/thejefflarson/little-cpu/actions/runs/34848744663) |
| 4×2 | latches | *not yet measured — runner pool down this session* | | | | | |
| 2×2 | flops | *no baseline yet* | | | | | |
| 2×2 | latches | *not yet measured* | | | | | |
| 3×4 | flops | see ADR-0179/main's 12-tile measurement | | | | | |
| 3×4 | latches | *not yet measured* | | | | | |

**DECISION NEEDED**: the central question this ADR was opened to answer — does the
latch array alone bring nanocpu's routed 4×2 (or 2×2) area/demand down to a closing
tile — is not yet settled by a routed number. The local structural evidence above
(the write-side mux and its associated hold buffers are gone; the read-side fan-out the
ablation found dominant is not addressed by this change alone) predicts a real but
partial improvement, not a guarantee of closing 4×2 on its own. Dispatch
`nano-tt-area-selfhosted.yml` with `regfile=latches` at `tiles=4x2`, `2x2`, then `3x4`
once `gh run list --status in_progress` shows the self-hosted pool moving again, and
record the routed numbers here.

## Consequences

- `NANO_LATCH_RF` is additive and off by default; nothing about the shipping (flops)
  configuration changes area, timing, or behaviour.
- Formal verification of the latch variant is a follow-up, not covered by
  `make -C nano/formal check` today.
- ADR-0179's fallback order (one-port register file, then this latch array) is the
  standing plan if the flip-flop build still cannot close a small tile; this ADR
  ships the mechanism and the local structural evidence, and the routed measurement
  that plan was waiting on is still owed.
