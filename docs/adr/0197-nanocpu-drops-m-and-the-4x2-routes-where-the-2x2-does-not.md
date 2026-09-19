# ADR-0197: nanocpu drops M, and the 4×2 routes where the 2×2 does not

**Status:** Accepted (M is cut) · the tile choice is open

## Context

ADR-0184's work order for fitting nanocpu onto a small Tiny Tapeout tile was: (1) a
latch-array register file, (2) mul/div rebuilt around one shared 64-bit register, (3) a
one-read-port register file, with M itself a second permitted cut only if those three
together were not enough. ADR-0189, ADR-0192 and ADR-0195 measured all three: the best
combination (latches plus the rebuilt mul/div) reads 65,026.1152 µm² / 66.49% routing
demand / 59 overflow on 4×2, and 105.422% placement utilization on 2×2 — closer than the
untouched baseline, closed on neither tile. ADR-0195 named M as the next, and last,
lever the work order has.

## Decision

**Delete the multiply/divide unit outright.** `nano/nano.v` had it rebuilt once already
(ADR-0192); this ticket removes it rather than shrinking it again. The eight M encodings
(`mul`, `mulh`, `mulhsu`, `mulhu`, `div`, `divu`, `rem`, `remu`) name no decode logic of
their own any more — `is_math_op`'s `funct7 == 0000001` case simply matches nothing in
`is_valid`'s list, so they fall through to the same illegal-instruction trap every other
unimplemented opcode already takes. Nothing new was written to make them illegal; an
existing mechanism now reaches them.

Build moves from `-march=rv32emc -mabi=ilp32e` to `-march=rv32ec -mabi=ilp32e`
everywhere it is declared: `nano/tb.mk` (the one source), `test/march_test.sh`'s
exception-list entry that names it, and `test/probe_gates.sh`'s QSPI accounting probe.
`nano/asm/mul.S` and `divide.S` become illegal-trap tests rather than arithmetic ones —
the assembler itself now refuses the mnemonics at this ISA, so each places the raw
R-type encoding via `.word` and requires a `TRAP` exit, with the destination register
pre-written so RVFI's `regs[rd]` read on the trap path is not X under iverilog's
four-state leg (a real gap the first version of this test hit: `rd` names `x4`, which a
trapping instruction never writes, and the retiring instruction's RVFI report still
reads it). Nine of littlecpu's own `test/asm` programs that name a real M mnemonic —
`div.S`, `divu.S`, `hazard.S` (whose hazard-forcing instruction is `div`), `mul.S`,
`mulh.S`, `mulhsu.S`, `mulhu.S`, `rem.S`, `remu.S` — move from a real retire floor to
`EXCLUDED M extension` in nano's borrowed-subset manifest, the same idiom the
A-extension programs already use.

The mul/div differential oracle ADR-0181 built and ADR-0192 rewrote
(`nano/tb/nano_exec_cxxrtl.cc`, `nano_exec_probe.sh`, `nano_exec_run.sh`,
`nano_exec_tb.v`, and the `nano-exec-test`/`nano-exec-probe` targets) is deleted with
the unit it existed to differentially test. `nano/formal/complete.sv`'s `OP-M` cover,
unreachable now that an OP-M retire always traps (`complete_live` requires `!rvfi_trap`),
is replaced by a cover that the trap itself is reached
(`rvfi_valid && rvfi_trap && insn_is_m`) — re-derived, not weakened; `complete_cover`
still reaches 13 of 13 goals. `nano/formal/checks.cfg`'s `isa` narrows from `rv32imc` to
`rv32ic`, so genchecks stops generating per-instruction spec-model checks for encodings
the core no longer implements (confirmed against the pin's own `isa_rv32ic.txt`, whose
diff against `isa_rv32imc.txt` is exactly the eight M mnemonics), and
`nano/formal/EXPECTED_CHECKS` drops the corresponding eight `insn_*_ch0` entries.
`make -C nano/formal remeasure-fg` reproduces **F = 12, G = 10** unchanged — the real
mul/div loop's cycle count never entered F or G under `RISCV_FORMAL_ALTOPS`, the same
finding ADR-0192 already recorded when the unit was rebuilt rather than removed.

## Correctness

`make -C nano/formal check`: 74 generated checks (down from 82, exactly the eight
dropped), exact match against `EXPECTED_CHECKS` and `EXPECTED_FAIL` (72 pass, 2
known-fail: `csrw_mcycle_ch0`/`csrw_minstret_ch0`, unrelated to this change). `make -C
nano/formal all` (`dmemcheck`, `imemcheck`, both `_cover` variants, `complete`,
`complete_cover`, `ill_e`, `ill_e_cover`, `check-rvfi-insn-check`): all pass.
`nano-test`, `nano-latch-test`, `nano-oneport-test`, `nano-oneport-latch-test` (each
both simulator legs), all four `*-startup-test` variants, and `nano-littlecpu-test`: all
pass, both legs agreeing program by program. `make test` and `make probe-gates`: both
green.

## Measurement

**Local instrument**, `make nano-area` (`synth; dfflibmap; abc -liberty`, sky130hd,
never merged with the Tiny Tapeout numbers below): not separately re-run here — the
Tiny Tapeout flow's own synthesis figure is quoted throughout, since it is the number
both tiles below are measured against.

**Benchmarks, both sides under the pinned compiler in one session** (xPack
`riscv-none-elf-gcc` 15.2.0-1, ADR-0190), `make nano-dhrystone`/`make nano-coremark`,
core-only zero-wait-state model, against today's 0.226 DMIPS/MHz and 0.538
CoreMark/MHz:

| Metric | With M | Without M | Change |
|---|---|---|---|
| Dhrystone cycles | 503,300 | 499,700 | −0.72% |
| Dhrystone DMIPS/MHz | 0.226 | 0.228 | +0.88% |
| CoreMark cycles | 9,301,580 | 19,496,420 | **+109.6%** |
| CoreMark CoreMark/MHz | 0.538 | 0.256 | **−52.4%** |

Dhrystone is a wash: GCC strength-reduces most of its multiplies to compile-time
constant shift-adds regardless of whether real M exists underneath, so removing the
hardware unit barely registers — the slight improvement is noise from which
instructions the remaining libgcc calls land on, not a real speedup. CoreMark is the
honest price of the cut: it leans on real multiply and divide, and more than doubles
its cycle count with M gone. This is reported plainly as the cost of the tile, not
softened or excused.

**Tiny Tapeout flow** (`nano-tt-area-selfhosted.yml`, LibreLane 3.0.14, `regfile=latches`,
`SYNTH_STRATEGY = AREA 2`), synthesis is tile-independent and reads **48,240.016 µm²**
on every run below — **−16,786.10 µm², −25.8%**, against ADR-0195's two-port-plus-latches
pair's 65,026.1152 µm² (M still present). The pre-ABC `Chip area` line some of these
logs print earlier (15,863.9648 µm² in one) is not this number and is not quoted as one,
per ADR-0192's own warning about reading a `stat` print from before ABC has mapped
anything.

**2×2, `disallow_congestion=true`** (run
[35412456941](https://github.com/thejefflarson/little-cpu/actions/runs/35412456941)):
placement succeeds — `[GPL-0019]` utilization 78.331%, core area 329.36 × 220.32 µm
(matching the 2×2's known ~72,565 µm² core area). Global routing fails badly:
`GRT-0096` met1 169.77%, met2 165.54%, met3 135.86%, met4 124.08%, **total demand
154.09%**, `GRT-0116` overflow **26,406**. Wirelength 619,192 µm. `RSZ-0038`: 581
buffers in 210 nets. `RSZ-0046`: 665 hold endpoints. Wall time 1,482s. The tile offers
43,394 routing units against a demand of 66,867 — the design would have to shed at
least a third of its wiring just to reach 100% demand, before overflow is even
considered. **It places. It does not route, and the gap is not close.**

**4×2, `disallow_congestion=true`** (run
[35414209212](https://github.com/thejefflarson/little-cpu/actions/runs/35414209212)):
same 48,240.016 µm² synthesis, `[GPL-0019]` utilization 37.942%. `RSZ-0038`: 635
buffers in 224 nets. `RSZ-0046`: 760 hold endpoints. Global routing still fails
`GRT-0116`, but at a wholly different scale: `GRT-0096` total demand **52.93%** (met1
67.73%, met2 56.44%, met3 49.74%, met4 14.76%), overflow **1/0/2** by layer (met1/met2/
met3), essentially the last few congested cells rather than a structural shortfall.
`GRT-0704` suggested reducing the global router's own layer adjustment from 30% to 28%
— an unspent lever, since the next run closes it without touching the router's
settings at all. Wirelength 461,037 µm.

**4×2, `disallow_congestion=false`, the same inputs otherwise** (run
[35415688944](https://github.com/thejefflarson/little-cpu/actions/runs/35415688944)):
**the flow completes all 80 stages**, wall time 3,603s. Detailed routing converges:
`DRT-0199` violation count falls 257 → 101 → 62 → 26 → 1 → 0 across its iterations.
Final routed wirelength 334,941 µm (down from global routing's own 461,037 µm estimate,
the same figure the failed run above reported — the flag changes whether the flow stops
on overflow, not what global routing itself computes). Post-route instance area
149,183 µm² against a 154,113 µm² die (`design__instance__utilization__stdcell`
44.667%, once 82,547.9 µm² of fill cells pad the rest) — a different, later-stage number
than the 48,240.016 µm² synthesis figure, not a discrepancy with it.

Signoff, read from `runs/wokwi/final/metrics.csv` and the stage reports directly rather
than estimated: **Magic DRC clean** (`COUNT: 0`). **LVS passed** (netgen: "Circuits
match uniquely"). **Antenna passed** — the first check (stage 40) lists dozens of
violations up to 6.62× the required ratio, all on met1–met3, but the repair stage fixes
every one: the final re-check (stage 46) reports an empty violation table, and
`antenna__violating__nets`/`antenna__violating__pins` are both 0 in the final metrics.
KLayout DRC and XOR were skipped by this flow's own gating (not independently verified
here). **Hold is clean at every corner without qualification**: `timing__hold_vio__count`
is 0 and `timing__hold__wns`/`timing__hold__tns` are both 0 at all nine PVT corners
(nom/min/max × tt/ss/ff) the metrics file reports. **Setup is not clean everywhere, and
this is worth stating precisely rather than repeating the flow's own summary verdict
unexamined**: the flow's checker step, which runs at the nominal `tt` corner, correctly
reports zero setup and zero hold violations there
(`timing__setup_vio__count__corner:nom_tt_025C_1v80` = 0), and the resizer's own logged
"Unable to repair all setup/hold within margin" is explained by what that nominal-corner
check does not cover: every slow (`ss`) corner shows real setup violations —
`nom_ss_100C_1v60` 171 violations (WNS −7.384 ns, TNS −633.87 ns), `min_ss_100C_1v60`
169 (WNS −6.695 ns, TNS −555.34 ns), `max_ss_100C_1v60` 172, the worst of the three
(WNS **−7.966 ns**, TNS **−709.06 ns**) — while every `tt` and `ff` corner, fast or
nominal, stays at zero. The aggregate (corner-unqualified) line in the same file matches
the `max_ss` corner exactly, confirming it as the reported worst case. Hold's full
cleanliness and setup's slow-corner-only violation are both real, measured facts from
the same file; neither is estimated.

## Consequences

- **The 2×2 places but does not route, and by a wide margin (154.09% demand, 26,406
  overflow); the 4×2 routes and signs off clean on DRC/LVS/antenna, with hold clean at
  every corner and setup clean everywhere except the slow corner (worst −7.97 ns WNS,
  −709.06 ns TNS at `max_ss_100C_1v60`).** Both are real, measured results, recorded
  here as what each tile does under this design today — **this ADR does not set a tile
  as nanocpu's ceiling.** The 2×2 was described in this ticket's brief as a hard budget
  limit; the owner has since asked about the 4×2's own fit and decided to merge the
  M cut regardless of which tile ships. The tile choice rests with the owner, to be
  recorded in a future ADR once made, not assumed here from either result.
- `nano/nano.v`'s mul/div is gone outright, not a define-selected variant: there is no
  path back to any version of the unit short of `git revert`.
- ADR-0184's three-lever work order (latch register file, rebuilt mul/div, one-read-port
  register file) plus this ADR's M cut are now the full, exhausted set of levers that
  ADR-0184 and ADR-0195 named. No further lever is pending from that plan.
- `NANO_MAX_UM2`, the local-instrument ratchet, is unaffected by this change (it never
  gated on the Tiny Tapeout flow's own numbers, per ADR-0184).
- The setup violations at the slow corner are the resizer's own known limit on this
  design at this tile, not a hidden defect this ADR is discovering for the first time
  and leaving unrecorded — a future engineer closing them (or deciding they are
  acceptable for the tile the owner picks) should start from the per-corner numbers
  above rather than re-deriving them.

## Amendments

- ADR-0177 ("RV32E is the one cut") is superseded on that point: M is a second cut, now
  exercised, not merely permitted. See its own amendment.
- ADR-0184 (which permitted M as a second cut) is amended to record that the cut was
  taken and what each tile measures with it. See its own amendment.
- ADR-0180's own text calling `test/march_test.sh`'s nano declaration a "non-required
  site" is corrected in place: that exception-list entry is graded both ways by
  `test/march_test.sh`, which is on `make test`'s required path, so it is not a
  nano-local, unenforced detail.
