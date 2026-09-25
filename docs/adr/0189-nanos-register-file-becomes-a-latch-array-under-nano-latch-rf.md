# ADR-0189: nano's register file becomes a latch array under `NANO_LATCH_RF`

**Status:** Superseded by [ADR-0209](0209-the-latch-register-file-is-removed-flip-flops-only.md) · 2026-09-15

> **Superseded 2026-09-25.** The owner decided to remove `NANO_LATCH_RF` and keep the
> flip-flop register file — see ADR-0209. The analysis and measurements here remain the
> record of what the latch build bought and why it never became the default.

## Context

Today's flip-flop `nano/nano.v` does not fit a small Tiny Tapeout tile in the real
`nano-tt-area-selfhosted.yml` flow (LibreLane 3.0.14 on the self-hosted runner image, `AREA 0`,
`disallow_congestion=true`): a 3×2 fails detailed placement (DPL-0036, 85.723% utilisation) and a
4×2 (8 tiles) fails global routing. ADR-0184 ranked the register file and mul/div as the two
largest contributors to that 4×2 congestion by removing each in turn on this same flow: cutting
the register file to two entries took synthesis area 81,879.78 → 58,682.53 µm² and routing demand
101.05% → 54.26%; its two read ports are selector trees whose four low read-address bits each
drive 128 `mux4` selects, and the PDK's excluded-cell list turns every enabled flip-flop into a
plain flop plus a `mux2_1`, which also makes the write path fast enough to need most of the
design's 941 post-CTS hold buffers. ADR-0184's work order is: (1) a latch-array register file,
(2) mul/div rebuilt around one shared 64-bit register and a 32-bit adder, (3) a one-read-port
register file — each measured in the flow on 4×2 and 2×2 before the next begins, with M a second
permitted cut if the three together are still not enough. This ADR is step 1.

## Decision

`nano/nano.v` gains a `NANO_LATCH_RF`-selected register file, kept inline behind an `ifdef` rather
than split into a second file. Every one of nano's mutation and probe scripts
(`test/probe_gates.sh`, `formal/memcheck-cover-probe.py`, `nano/formal/complete-cover-probe.py`,
`nano/formal/ill-e-probe.py`, `nano/tb/nano_exec_probe.sh`, `nano/tb/nano_x_probe.sh`) and every
`nano/formal/*.sby` file's `[files]`/`[script]` section assume `nano/nano.v` is the one and only
RTL source they copy or read; a second file would need touching every one of them for no benefit
the `ifdef` does not already give, and "its own file... if that reads best" in the brief's own
wording left that call open.

**The design is transparent-LOW latches with a registered select and data, not a same-edge
decode.** A first cut gated each latch on `clk && sel`, `sel`/`din` decoded straight from
`cpu_state`/`rd`/`reg_wdata` — a real hazard: those three are flops on the very `clk` edge that
opens the latch, so `sel`/`din` and the latch's own transparent window begin at the identical edge
with no designed margin between them, and `clk && sel` is a combinational AND on the clock with no
guarantee of a glitch-free result STA can check. The shipped design instead:

1. Decodes `we[gi]` (one-hot: `cpu_state == fetch_instr` for register 0, `cpu_state == reg_write
   && rd[3:0] == gi` for the rest) combinationally, exactly as before.
2. Registers it: `we_q <= we; wdata_q <= reg_wdata;`, on the same `posedge clk` the FSM already
   runs on.
3. Opens latch `gi` on `!clk && we_q[gi]`.

`we_q`/`wdata_q` are flip-flop outputs, so they change only at a rising edge and are then stable
for the *entire* following period — both phases — until the next one. The falling edge that opens
a latch therefore always finds `we_q`/`wdata_q` already a half period settled, never racing it;
the coincident rising edge that closes the latch and updates `we_q`/`wdata_q` for the *next* write
doesn't touch what was already latched, since a transparent latch's stored value is whatever its
D input held through the whole window that just ended, not a function of what happens exactly as
it closes. This is the same standard "gate a latch's enable from a registered, same-domain
control signal" construction OpenSTA's ordinary latch/time-borrowing timing checks are built to
verify — no `create_generated_clock`, no dedicated `sky130_fd_sc_hd__dlclkp` clock-gate cell,
because the enable never leaves the data path into a clock tree; it drives one latch's own native
GATE pin, the same way an unregistered `sel` would have, just now provably settled before the
window that reads it opens. A `dlclkp`-based scheme (a real ICG cell, registered enable) would be
no less sound but is the one the brief itself flagged as conditional ("if that maps cleanly
through the flow"); the registered-AND scheme needs no cell beyond what `dfflibmap`/`abc` already
map for the rest of the design, so it is the one that ships. Select and data are still read from
plain per-generate-instance wires rather than a bit-select inside the `always_latch`/`always_ff`
process itself — iverilog's four-state frontend cannot fully evaluate a constant select inside an
`always_*` process (the same class of warning `rtl/writeback.v` is already allowlisted for in
CLAUDE.md) and over-widens the sensitivity list without one; this applied to `we_q[gi]` exactly as
it did to the original `sel`/`din`, and is worked around the same way.

**Read-after-write.** A retiring write's own latch is still closed (opaque) at the moment of
retire — `is_fetch = cpu_state == fetch_instr` and `we_q`/`wdata_q` are set at the edge *entering*
that same fetch_instr cycle, so the latch opens only in that cycle's low phase and closes again
exactly at the edge ending it. Concretely: nothing in `nano.v`'s own execution ever reads
`regs[rd[3:0]]` for a *value* during `fetch_instr`, `ready_instr` or `decode_instr` — the next
real read of the register file is `execute_instr`, reached only after `fetch_instr` → `ready_instr`
(at least one more cycle, more if the memory answer is not immediate) → `decode_instr`, several
cycles clear of any write's own retiring cycle either way this design ever committed a write. The
one thing that *is* sampled synchronously at the exact edge closing the write's own retiring
cycle is RVFI's own `rvfi_rd_wdata_q <= |rd ? regs[rd[3:0]] : 0;` (for the monitor's benefit, not
the core's own execution) — captured at the same edge the latch closes, from a value the latch has
already tracked correctly through its whole low-phase window, so it is not actually stale.
`NANO_LATCH_RF` still reads `reg_wdata` there directly rather than `regs[rd[3:0]]`, ifdef-gated so
the default build's text is untouched: `reg_wdata` is the same value the latch is committing, read
with no dependence on that same-edge coincidence holding on the real, physically-delayed silicon
this reasoning is idealised over. `nano-latch-test`/`nano-latch-startup-test` are the check this
claim has to pass, since a stale `rd_wdata` is exactly what makes the monitor's independent
reference disagree with the core's own report.

**The default build is unchanged.** Every line this ADR adds sits either inside an
`ifndef NANO_LATCH_RF` wrapping an existing statement, or inside a new `ifdef NANO_LATCH_RF`
block; preprocessing `nano/nano.v` with no macros set reproduces origin/main's file verbatim
(`iverilog -E`, diffed modulo the blank lines it pads in for line-number parity), and
`synth; dfflibmap; abc` against sky130hd lands on the identical 61,411.3984 µm² both before and
after.

**Tests.** `nano-latch-test` and `nano-latch-startup-test` build the latch variant through both of
nano's simulator legs (`nano-latch-sim`/cxxrtl, `nano/tb/nano_icarus_latch.vvp`/iverilog) and grade
it with the exact same graders as `nano-test`/`nano-startup-test` — `nano/asm/EXPECTED_FAIL`/
`OBSERVED_FLOOR`, dual-leg agreement, the startup PASS/FAIL — introducing no new comparison logic,
so no new `test/probe_gates.sh` entry is owed beyond the ones that already force those graders
red. Both join `make test`; the added cost is nano-test's own (about 20s locally), since it is the
same suite built twice. Both variants retire the identical instruction counts on all six suite
programs and the startup check (18 retires), on both simulator legs, under the registered-enable
scheme.

**Formal cannot read the latch variant as-is, and it cannot become the default until it can.**
Probed directly (not fixed, per the brief): elaborating `nano/nano.v` under `NANO_LATCH_RF`
through the exact script `nano/formal/ill_e.sby` and `nano/formal/dmemcheck.sby` run
(`prep -nordff`, `flatten`) produces fifteen `$dlatch` cells cleanly through `check`, but both
backends nano's checks depend on refuse them at the write step: `ERROR: Unsupported cell type
$dlatch for cell ... -- please run clk2fflogic before write_btor` (the `btor btormc` engine
`ill_e.sby`/`complete.sby` use) and the identical error naming `write_smt2` (the `smtbmc` engine
`dmemcheck.sby`/`imemcheck.sby` use, and what the generated `checks.cfg` family also resolves to).
Making the latch variant checkable means adding `clk2fflogic` (yosys's own suggestion) to the
shared script every `.sby` file and `nano/formal/checks.cfg`'s `[script-sources]` build from,
verifying it does not also widen what those checks can prove about the *flip-flop* build's own
flops (`clk2fflogic` changes how every clocked element is modelled, not only latches), and
re-deriving F/G since the technique changes how the solver sees a cycle boundary. None of that is
done here, and it is the gate: `NANO_LATCH_RF` stays a build option, never the default, until
`make -C nano/formal check` reads it too.

## What the latch array structurally changes, against ADR-0184's two findings

Neither of these is a routed-flow number; both are read off a local
`synth; dfflibmap; techmap <dlxtp map>; abc` run against the same sky130hd liberty `make nano-area`
uses, comparing the flip-flop and `NANO_LATCH_RF` builds cell for cell. `dfflibmap` alone leaves
every inferred `$_DLATCH_P_` unmapped (it targets `$dff`-family cells only), so an ad hoc
`techmap` mapping `$_DLATCH_P_` straight to `sky130_fd_sc_hd__dlxtp_1` stands in for the real
flow's `SYNTH_LATCH_MAP` step here; it is not that step, and the resulting 50,870.04 µm²
(−17.17% against the flip-flop build's 61,411.40) is offered only as a sanity check against the
historical 49,377.36 vs 60,758.27 (−18.7%) figure, not as this ADR's area answer.

- **The write-enable mux2 ADR-0184 named has no equivalent in the latch build.**
  `sky130_fd_sc_hd__mux4_2` goes 268 → 0 and `sky130_fd_sc_hd__edfxtp_1` (the enable-flop
  `edfxtp` being excluded from the PDK's usable-cell list forces every gated flop to synthesise as
  plain flop plus mux) goes 792 → 280, a fall of 512 cells, replaced by 480
  `sky130_fd_sc_hd__dlxtp_1` latches and a modest rise in plain `dfxtp_1` (185 → 224) for the new
  `we_q`/`wdata_q` registers this ADR's fix adds. A latch holds its value with no logic at all when
  its own select is low, which is the mechanism the write-side hold-vs-load mux existed to provide
  for a plain flop; removing the mux removes what those 408 buffers were fixing hold on. This is an
  inference from the cell census, not a measured hold-buffer count for the latch build — that
  number, and the flow's clock-gating check on the new registered enables, are owed from the real
  flow.
- **The read-address fan-out is untouched by this change, on purpose.** `nano/nano.v`'s roughly
  twenty `regs[rs1[3:0]]`/`regs[rs2[3:0]]`/`regs[rd[3:0]]` read sites are byte-identical text under
  both builds — `NANO_LATCH_RF` touches only the two write statements and adds the latch write
  logic, never a read expression. So the same four address bits drive the same set of reads either
  way; the local census shows no surviving `mux4_2` cell in either build's read logic (both land on
  0 for that specific cell type), meaning ABC maps that 16-way read select through AOI/OAI gates
  rather than a discrete mux primitive in this liberty regardless of the register file's storage
  element, so this census cannot separate "unchanged" from "remapped differently." The routing
  demand ADR-0184 attributed to read fan-out is expected to persist into the latch build; only a
  routed run says by how much.

## Measurement

Dispatched on this branch via `nano-tt-area-selfhosted.yml`'s `regfile` input (`AREA 0`,
`disallow_congestion=true`, `stop_after_synthesis=false`), one run at a time (the workflow's own
concurrency group cancels a second dispatch on the same ref), on LibreLane 3.0.14 throughout, per
ADR-0184's own work order (4×2 then 2×2 before mul/div's turn begins).

**Two earlier dispatches on this branch were cancelled, not trusted for numbers.** The first 4×2
latch dispatch (run 34849107473) sat with the self-hosted runner pool not moving for over two
hours and was cancelled rather than trusted further. A second (34923698630) was cancelled
deliberately: it started against the racy `clk && sel` scheme, before the fix this ADR now ships.
**The measurement below is from a clean dispatch (34925911447) against the corrected,
registered-enable RTL.**

| Tiles | Regfile | Chip area (µm²) | Placement util. | GRT total demand | Result | Wall time | Memory | Run |
|---|---|---|---|---|---|---|---|---|
| 4×2 | flops | 81,879.78 | 63.638% | 101.05% | GRT-0116 congestion | — | — | [34848744663](https://github.com/thejefflarson/little-cpu/actions/runs/34848744663) |
| 4×2 | latches | **70,070.95** | **54.980%** | **70.55%** | GRT-0116 congestion (closer, not closed) | 3107s (~51.8 min) | 3,221,188,608 B (~3.00 GiB, at the pod's 3 GiB ceiling) | [34925911447](https://github.com/thejefflarson/little-cpu/actions/runs/34925911447) |
| 2×2 | flops | *no baseline yet* | | | | | | |
| 2×2 | latches | 70,070.95 | **113.506%** | not reached | GPL-0301: placement utilization exceeds 100% | 294s | 3,221,188,608 B (at the pod's 3 GiB ceiling) | [34929665602](https://github.com/thejefflarson/little-cpu/actions/runs/34929665602) |

**4×2 latches: real, substantial, and still not enough alone.** Global routing's own per-layer
report (`GRT-0096`): met1 80.77%, met2 81.43%, met3 62.09%, met4 36.37%, **total 70.55%** against
the flip-flop build's 101.05% -- a 30.5-point drop. Total wirelength 653,002 µm against the
flip-flop build's 874,050 (−25.3%), 8,146 routed nets. It still fails
`disallow_congestion=true`: the aggregate usage is under 100%, but `Total Overflow` is nonzero on
every layer (met1 96, met2 46, met3 157, met4 18, 317 total) -- localized hotspots exceed capacity
even though the average dropped far below it, and `disallow_congestion=true` fails on any
remaining overflow, not just an aggregate over 100%. **The hold and clock-gating story the review
asked for**: OpenSTA's own mid-PnR timing (`nom_tt_025C_1v80`, after CTS, before the routing that
never finished) names each latch by its actual mechanism -- `Endpoint: _nnnnn_ (positive
level-sensitive latch clocked by clk')` -- and finds it fully analyzable: hold is clean, WNS 0 /
TNS 0.0, after `repair_timing -hold` found 885 endpoints with hold violations and inserted **395**
hold buffers, against the flip-flop build's 941 -- more than half gone, consistent with the
write-side mux this ADR already traced out of the design. Setup at this same intermediate,
pre-final-routing snapshot reads WNS −0.274 ns / TNS −2.042 ns, a small residual the flow's later
steps (never reached, since global routing itself failed first) would ordinarily continue closing.

**Answered for both tiles: the latch array alone reaches neither a routable 4×2 nor a placeable
2×2.** On 4×2 it is **closer, not closed**. It makes real cuts to area (−14.4%), placement
utilization, wirelength and GRT demand across every layer, and it more than halves the hold-buffer
population the write-side mux was responsible for. But disallowed-congestion routing still fails on
localized overflow from the read-address fan-out, which ADR-0184 already flagged as untouched by
this change. On 2×2 (run 34929665602) the same 70,070.95 µm² needs 113.506% of the placeable area,
and global placement stops (`GPL-0301`) before routing is reached. The next step is ADR-0184's
step 2, the shared-register mul/div. The read-address fan-out this change leaves in place is
step 3's to address.

## Consequences

- `NANO_LATCH_RF` is additive and off by default; nothing about the shipping (flops) configuration
  changes area, timing, or behaviour.
- `NANO_LATCH_RF` cannot become the default until `make -C nano/formal check` reads it: `clk2fflogic`
  added to every `.sby` script and `checks.cfg`, F/G re-derived, and the flip-flop build's own
  checks re-confirmed unaffected.
- ADR-0184's work order (latch register file, then shared-register mul/div, then a one-read-port
  register file, M a second permitted cut) is the standing plan. Step 1 measures real: 4×2's GRT
  total demand 101.05% → 70.55%, area 81,879.78 → 70,070.95 µm², hold buffers 941 → 395 -- but 4×2
  still does not route clean under `disallow_congestion=true`, so step 2 (mul/div) is not optional
  for closing it. On 2×2 the latch build does not even place (113.506% utilization), which
  completes the pair ADR-0184's work order asks for before step 2 begins.
