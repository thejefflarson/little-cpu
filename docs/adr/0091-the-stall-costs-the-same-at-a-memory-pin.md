# ADR-0091: the stall costs the same at a memory pin, and the arm it would replace is worth nothing

**Status:** Accepted · 2026-08-09 · *Built, measured over eight placements a side on two consecutive
bases, and DECLINED. It is a null in both directions, and the ceiling that motivated it is a null on
both bases too.*

## Context

`rtl/decoder.v`'s `next_pc` chain begins `reset: 0; stall: pc;`, so the whole stall cone — the decode
scoreboard, the operand-fetch check, the divider, the accessor, serialization, the stolen window —
sits in front of a 30-bit mux whose output is the address the instruction ROM is asked for on the
next edge. That is the closing hop of the fetch loop
([ADR-0076](0076-the-decode-head-is-a-plateau-not-a-lever.md),
[ADR-0087](0087-the-instruction-memory-does-not-come-out-of-the-fetch-loop.md)).

`SB_RAM40_4K` has an `RE` pin that holds `RDATA`. The proposal: route the stall there instead. The
address then stops depending on this cycle's stall, the memory re-presents the window it already
holds, and the stall reaches one pin per bank rather than travelling into the address path.

**The measurement that motivated it was a ceiling: deleting the entire `stall` arm — functionally
wrong, a pure upper bound — was recorded at 78.92 ns, −3.8%.** That number predates
[ADR-0088](0088-the-win-is-in-what-the-expression-cannot-say.md),
[ADR-0090](0090-the-executor-carries-half-the-registers-it-had.md) and
[ADR-0089](0089-the-operand-fetch-guess-ships-and-one-workload-feels-it.md), which between them took
569 cells out of the placed SoC and rebuilt the front of that same loop.

## The ceiling does not reproduce, on either base

Same experiment, eight seeds a side through `soc/timing_sweep.sh`, on `c2fa29b` and again on the
`a8d6f46` it was branched from:

| base | | sorted, ns | worst | median | best |
|---|---|---|---|---|---|
| `c2fa29b` | base | 76.61 77.51 78.61 78.64 78.71 79.25 79.29 79.30 | 79.30 | 78.68 | 76.61 |
| `c2fa29b` | `stall` arm deleted | 77.05 77.79 77.99 78.35 78.42 80.08 80.51 81.45 | 81.45 | **78.39** | 77.05 |
| `a8d6f46` | base | 74.42 74.71 75.44 75.62 76.63 76.68 76.68 77.25 | 77.25 | 76.13 | 74.42 |
| `a8d6f46` | `stall` arm deleted | 74.51 74.72 76.12 76.63 76.85 77.63 77.69 79.96 | 79.96 | **76.74** | 74.51 |

**−0.4% at the median on `c2fa29b`, +0.8% on `a8d6f46`, and the worst placement is 2–3% slower on
both.** Deleting the term outright buys nothing measurable, twice, on two trees whose fetch loops
differ. The −3.8% was real when it was taken; three merges later the term it bounded is not worth
anything, which is the whole reason a ceiling is re-taken before it is spent.

**That is the finding, and it closes the direction independent of any spelling.** No implementation
of "get the stall off the address path" can beat deleting the dependency altogether.

## What was built anyway, and what it measured

A ceiling bounds the *term*, not the *rewrite*: this one also shortens the address path by an arm and
moves the enable onto a primitive pin, so it could in principle have come out somewhere the ceiling
does not describe. It did not.

- `rtl/decoder.v` publishes `imem_ren = reset || !stall || fetch_stall`, and `pc` takes it as a clock
  enable, so the pc and the window it names move together. The `next_pc` chain keeps one stall arm,
  `fetch_stall`, because a stolen window carries the data bus's answer and has to be fetched again at
  `pc` — no other arm could name the right address, and that one arrives as a flip-flop.
- `rtl/imemory.v` takes `imem_ren` and reads its banks under `imem_ren || text_access`, holding
  `even_data`, `odd_data`, `odd_first`, `in_range` and `in_range2` together: they describe one
  window, not the address being presented.
- The new port is connected in `rtl/littlesoc.v`, `test/testbench.v`, `soc/compare/bench_littlecpu.v`
  and all five harnesses under `formal/`. The four with a combinational fetch model leave it unread,
  as they already do `imem_addr_next`.

Eight seeds a side on `c2fa29b`:

| | sorted, ns | worst | median | best |
|---|---|---|---|---|
| base `c2fa29b` | 76.61 77.51 78.61 78.64 78.71 79.25 79.29 79.30 | 79.30 (12.61 MHz) | 78.68 | 76.61 |
| read enable | 77.63 77.87 77.94 78.23 79.00 79.58 79.67 80.71 | 80.71 (12.39 MHz) | **78.62** | 77.63 |

**−0.1% at the median, +1.8% at the worst, +1.3% at the best** — against a ~3.6% edit-churn band and
a 1–2% placement spread. A null in both directions, which per this repository's own rule is neither a
candidate admitted nor one declined *on the period*. What declines it is that a null is all it buys,
for a new core output port, a new memory input, five harness edits and a new requirement on the
platform (below).

**On `a8d6f46` the same amendment measured 78.97 ns at the median (+3.7%) and 83.75 ns at its worst,
which is 11.94 MHz — under `SOC_MIN_MHZ`.** Same RTL, a different tree around it, and the difference
between "a null" and "misses the board clock" is one re-base. That is worth recording as a property
of the *instrument*, not of the amendment: one placement is a sample, and so, it turns out, is one
tree.

`make fit` 3496 → 3542 (+46) and the placed SoC 4175 → 4169 (−6); `SB_RAM40_4K` stays at 20 and
`SB_SPRAM256KA` at 2, which is what says the enable mapped onto the primitive's pin rather than into
fabric muxes on the output register.

## Which loop is critical afterwards

`soc/depth/path_stages.py` on the default placement, reading its own caveat — the `imem` bucket is an
upper bound on the memory's contribution, because every decode level folds in more instruction bits
and those are `rom_*_RDATA`:

| | levels | ends at | charged |
|---|---|---|---|
| base `c2fa29b` | 25 | `imem.rom_even.0.4_RDATA_1` | decode 13, imem 4, csrs 2, access 1 |
| read enable | 22 | **`riscv.csrs.mscratch[1]`** | imem 9, decode 7, access 1, csrs 1 |
| read enable + the fourth slot | 20 | **`imem.bank_re`** | imem 7, decode 4, access 1 |

The fetch loop does come off the top of the report at that placement — and a CSR path of the same
length is underneath it, which is the plateau `CLAUDE.md` already describes. With both amendments in,
the path is the fetch loop again and it **ends at the enable**: the same cone that used to arrive at
the ROM's address now arrives at its read enable, and has to reach sixteen `SB_RAM40_4K` `RE` pins
and 32 `pc` flop enables instead of one 30-bit mux. On a routing-dominated fabric that charges
`LocalMux` + `InMux` per LUT input wherever the LUT sits, moving a signal from a mux to a pin does
not shorten the cone in front of it.

## Cycles

`make cycles` and `make dhrystone`, cxxrtl:

| | suite cycles | CPI | Dhrystone cycles | CPI | DMIPS/MHz |
|---|---|---|---|---|---|
| base `c2fa29b` | 29 454 | 1.88 | 2 178 739 | 2.29 | 0.535 |
| read enable | 29 515 | 1.89 | 2 178 724 | 2.29 | 0.535 |

+61 cycles on 62 programs: one per reset, and nothing else. Retires are identical at 15 654, and
`make waves` retires 79 either way.

**Throughput is a product and both terms travel together.** At the median placement, Dhrystone:
0.535 × 12.71 MHz = 6.80 DMIPS on base against 0.535 × 12.72 MHz = 6.81 here, **+0.1%** — which is
the same null the period is.

## The one defect this found, and it outlives the amendment

**A synchronous fetch that holds its answer needs its first window filled under reset, and the cxxrtl
runner never clocked a reset edge.** `test/cxxrtl.cc` sets `p_reset` true, calls `step()` with the
clock unchanged, then clears it before the first rising edge of cycle 0. The design survives that
today only because a stalled cycle re-reads: the first cycle after reset stalls for its operands,
which re-presents address 0, and the window is right by the time anything issues. Take the re-read
away and the core issues whatever the memory powered up with — measured, as 62 programs reporting
TRAP-TO-ZERO with zero retires.

`test/testbench.v`'s iverilog side holds reset over one posedge and `rtl/littlesoc.v` holds it over a
16-cycle power-on counter, so both of those are correct; `rtl/decoder.v`'s own `FORMAL` block already
*assumes* reset precedes the first edge. The cxxrtl runner was the only harness that did not, and
nothing said so. **It is left alone here** — with the amendment reverted it changes no behaviour, and
landing it on its own would move every program's cycle count by one for no reason. It is written down
so the next change that makes the fetch hold does not spend a day finding it again.

## What was verified

On this amendment alone, before it was reverted, so that the decline is a decline of something that
worked. The formal legs — the 85 generated checks, the four component proofs, `complete_cover`,
`nonperturbation` and `make cosim-suite` — were run on the pair and are listed in
[ADR-0092](0092-the-writeback-slot-costs-more-than-the-bypass-it-replaces.md), measured in the same
sitting.

| leg | verdict |
|---|---|
| `make test` | 62/62, `test/EXPECTED_FAIL` exact |
| `make test-units` | 9/9, including new `test/imem_tb.v` hold vectors and `test/decoder_tb.v` enable vectors |
| `make waves` (iverilog) | 79 retires, unchanged from base |
| `make cycles` | 29 515 cycles, CPI 1.89 |
| `make fit` | 3542 against `FIT_MAX_LC` 3700 |
| `soc/timing_sweep.sh` | eight placements, all above `SOC_MIN_MHZ` on `c2fa29b`; one below it on `a8d6f46` |

`make -C formal complete` did not return a verdict, and not because of this change: the local
Homebrew yosys reports `ERROR: Command syntax error` on `abc -g AND -fast`, identically on an
unmodified export. The `formal` CI job runs the pinned OSS CAD Suite. Recorded rather than worked
around, exactly as ADR-0088 and ADR-0090 recorded it.

## Consequences

- **The `stall`-to-address term is priced at zero and the direction is closed.** Not "small" — zero,
  bounded by a functionally wrong ceiling measured over eight placements on each of two consecutive
  bases. Anyone reaching for it again should re-take that ceiling first; it moved once already.
- **A ceiling is perishable.** −3.8% was a true measurement of a design that no longer exists, and it
  was the whole case for this work. Re-take a ceiling in the tree you intend to spend it in, in the
  same sitting, on the same instrument.
- **Moving a signal from a mux to a primitive pin is not a shortening.** The cone in front of it is
  unchanged and the fan-out is worse; with both amendments in, the critical path ends at that pin.
- **The same RTL was a null on one base and missed 12 MHz on the previous one.** A single tree is a
  sample the way a single placement is. `SOC_MIN_MHZ` is untouched, and it is what caught the second
  case.
- **A design whose fetch holds its output register requires a reset that spans a clock edge.** Three
  of this repo's four harnesses already provide one. `test/cxxrtl.cc` does not, and that is now
  written down.
