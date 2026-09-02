# 0146 — CoreMark's cycle factor lands for Hazard3; VexRiscv cannot run it

Status: Accepted · 2026-08-29 · closes the half of ADR-0139 explicitly left deferred, and settles
a question that ADR ticket text assumed was open: whether a three-way CoreMark comparison is
possible at all

## Context

ADR-0139 put Hazard3's iCE40 build in `soc/compare/` and measured its **clock** against this core's:
1.06× at the worst placement of five, 1.05× at the median. It named its own gap in its own title —
"clock measured, cycles not yet" — because Hazard3's iCE40 configuration sets `CSR_COUNTER=0`, so it
has no `mcycle` to self-time a CoreMark run with, the same problem `soc/compare/dhry_port.c` and
`soc/compare/dhry_tb.v` already solve for VexRiscv's identical gap on Dhrystone: a marker stored to a
fixed RAM word, counted on the bus by the testbench instead of read back by the program. ADR-0139
named that exact mechanism as the concrete follow-up rather than inventing a number under time
pressure. This ADR builds it.

`make coremark` already reports **1.811 CoreMark/MHz** for this core alone (ADR-0136), and Hazard3
publishes **4.15** for its RP2350 build — Zba/Zbb/Zbs, a fast multiplier, a branch predictor, none of
which its iCE40 configuration has. Neither number is comparable to the other, which is the
mixed-configuration error ADR-0098 already has a name for. The only number worth having is one
harness, one image, one simulation, both factors from one tree — which is also the exact rule
ADR-0098's own product went stale on days after it was written, so this ADR is careful to re-take
the clock half in the same session as the cycle half rather than reuse ADR-0139's.

## DECISION NEEDED, resolved: VexRiscv is not a third DUT

The ticket that asked for this measurement suggested VexRiscv "too, since it is already in the
harness and the marginal cost is one more `COMPARE_CORE`." That assumption does not survive reading
`soc/compare/bench_vexriscv.v`'s own header: **that configuration is RV32IC. No M extension at
all** — not a slow one, an absent one, with no CSR file and no trap machinery to survive an illegal
instruction either. CoreMark's `core_bench_matrix` and its state-machine benchmark both need real
multiply and divide; there is no way to hand that core a CoreMark image without one of two changes,
and both were considered and declined:

- **Compile the shared image at `-march=rv32ic`, the way `COMPARE_DHRY_CFLAGS` already does for
  Dhrystone**, and let libgcc's software routines carry the multiply on all three cores. Rejected: it
  would erase littlecpu's and Hazard3's own *hardware* multipliers — the exact difference this
  measurement exists to show — for the sake of a third data point that would then need its own
  caveat explaining why its multiply is nothing like either of the other two.
- **Give VexRiscv real M-extension hardware.** Rejected outright: it is not this repository's core to
  edit, and the whole point of `soc/compare/` is running the *pinned* Verilog unmodified.

So this measurement is a pair, not a trio: littlecpu and Hazard3's iCE40 configuration, sharing
**RV32IMA** — the ISA both implement in hardware, with the C extension dropped because Hazard3's
iCE40 build does not have it (`EXTENSION_C=0`). `soc/compare/coremark_tb.v`'s own header states this
reasoning so a future reader does not have to re-derive it.

## What shipped

Six new files under `soc/compare/`, mirroring the Dhrystone comparison's own split file for file:

- **`coremark_compare_port.c`** implements `test/bench/core_portme.h` — reused unmodified, it declares
  no core-specific behaviour — the way `dhry_port.c` implements `dhry_port.h`. `start_time()` and
  `stop_time()` publish to a `.coremarkctl` window instead of reading a CSR, littlecpu included, for
  the reason `dhry_port.c` gives for timing littlecpu the same way it times VexRiscv: one measured
  window has to mean the same thing on every core in the harness. `ee_printf` is a no-op — it never
  dereferences its format string — which is not a shortcut but load-bearing: see below.
- **`coremark_start.S`** is `test/crt0.S` with the `.data` copy removed, mirroring `dhry_start.S`.
- **`coremark.lds`** is the enlarged, simulation-only memory map, mirroring `dhry.lds`: 16 KB of ROM
  (measured `.text` is 10768 bytes at `-march=rv32ima`), 16 KB of RAM. `.rodata` is folded into the
  poked RAM region with no ROM load copy, the same move `dhry.lds` makes for VexRiscv's missing ROM
  data path — but for a *stronger* reason here. `core_state.c`'s `intpat`/`floatpat`/`scipat`/
  `errpat` string tables are read **algorithmically** by the state-machine benchmark, not only handed
  to a printf, so this is not optional even for the pair that ships here (both of which, unlike
  VexRiscv, *can* read their own ROM as data) — a rodata split by core would have made the RAM
  comparison below untrustworthy for a reason that had nothing to do with the ISA.
- **`coremark_tb.v`** mirrors `dhry_tb.v`: two DUTs, `bench_littlecpu` and `bench_hazard3`, watching
  each core's own write bus for the two markers and the verdict word, comparing the two 16 KB data
  RAMs word for word at the end. Hazard3's publications are read off `mem_addr_mux`/`mem_wstrb_mux`/
  `hwdata` — the same captured signals `soc/compare/bench_tb.v` already reads for the identical
  reason (no separate data bus on a single AHB5 port).
- **`coremark_fit.py`** and **`coremark_dmips.py`** mirror `dhry_fit.py` and `dhry_dmips.py` exactly
  in shape — parsed facts, every comparison that can call a run bad, a demonstrated red direction for
  each — with `CoreMark/MHz = iterations * 1e6 / cycles` (frequency cancels the same way DMIPS/MHz's
  does) in place of the VAX-referenced DMIPS formula.
- **`run_coremark_compare.sh`** mirrors `run_dhrystone.sh`'s build-report-run split.

`Makefile` gains `compare-coremark` beside `compare-dhrystone`, `COMPARE_COREMARK_ITERATIONS`/
`_CYCLES`/`_CFLAGS`, and `compare.coremark.vvp`. `test/probe_gates.sh` gains a probe group for each
new Python script, and `test/PROBES_EXPECTED` is updated — the same "a grader that cannot fail is not
a grader" discipline `dhry_fit.py`/`dhry_dmips.py` are already held to.

## A real defect the RAM comparison would have hidden without a fix

Hazard3's `hazard3_regfile_1w2r.v` is built with `RESET_REGFILE=0` in this harness (the same default
`fpga_icebreaker.v` ships) — the FPGA-inference-friendly form with **no reset branch at all**:

```
end else begin: real_dualport_noreset
  reg [W_DATA-1:0] mem [0:N_REGS-1];
  always @ (posedge clk) begin
    if (wen) mem[waddr_masked] <= wdata;
    ...
```

and `wen` is itself gated on `|xm_rd` — a write to `x0` never happens, in hardware, by construction.
So `mem[0]` is never written by anything and stays **X in simulation for the life of the run** unless
something else clears it. `soc/compare/dhry_tb.v` already zeros littlecpu's and VexRiscv's register
files at time 0 for exactly the reason `test/testbench.v` zeros its ROM banks (an X reaching the
register file's address port turns the whole pipeline X, invisible to every formal check and to
`cxxrtl`'s two-state model, visible only here); `coremark_tb.v` does the same for Hazard3's regfile,
reaching `dut_haz.core.core.regs.real_dualport_noreset.mem[i]` — the generate-block-qualified path,
not merely the instance name, because the block the RTL takes at `RESET_REGFILE=0` is the one that
declares `mem` at all. Left unzeroed this would not have failed loudly: an X reaching an unused
register stays silent, and one reaching `x0`'s read fails only once something actually reads a
register nobody wrote, which for CoreMark's own working set was likely but not certain to happen
early. This is the same category of catch ADR-0139's AHB write-timing bug was — the anti-fraud check
existing to be forced, not merely present.

## Both factors, one tree

Everything below is `be293ff`'s tree with the files in this ADR added, one session, no gap between
the two measurements the way ADR-0098's own product had one.

### The cycle factor

1 iteration, RV32IMA, `-O2`:

```
COREMARK core=littlecpu marks=2 cycles=479420 verdict=1 writes=15701
COREMARK core=hazard3   marks=2 cycles=714984 verdict=1 writes=15701
COREMARK core=hazard3 wait_cycles=14176
COREMARK ramdiff=0 of=4096 words
```

Both verdicts PASS — CoreMark's own list/matrix/state CRCs matched EEMBC's published 2K-performance
values on **both** cores — and the two 16 KB data RAMs are bit-identical afterward. **Hazard3 takes
1.491× littlecpu's cycles for the same work.** CoreMark/MHz: **2.086** for littlecpu, **1.399** for
Hazard3's iCE40 configuration.

**The harness itself charges Hazard3 a cost littlecpu never pays, and it is now counted rather than
folded silently into that ratio.** `soc/compare/bench_hazard3.v`'s AHB5 adapter holds `hready` low
for one cycle after every write's address phase — correctly, and for the reason its own
`wr_pending_q` comment gives (a single-ported synchronous memory needs it for a store immediately
followed by a load of the same word); `bench_littlecpu.v` drives `.bus_wait(1'b0)` and pays nothing
equivalent. `coremark_tb.v` now counts those cycles directly off `wr_pending_q` — an independent
measurement on the Dhrystone side of this same adapter found 9.01% there, so this figure was not
assumed to travel from that one: **14,176 of Hazard3's 714,984 measured cycles, 1.98%.** Subtracting
them bounds what the harness itself contributes: 714,984 −
14,176 = 700,808 cycles the core accounts for, which is **1.462×** littlecpu's rather than 1.491× —
a 2.0% correction, not the 9.01% the Dhrystone harness pays for the identical adapter, because
CoreMark's own write mix differs from Dhrystone's. **Both figures are reported, not one substituted
for the other**: 1.491× is what the harness measured, 1.462× is a lower bound on what the two cores'
own execution accounts for, and `soc/compare/coremark_dmips.py` prints the wait-cycle count and its
share alongside the ratio so a reader can do this arithmetic without re-running the simulation.

**Fixing this also fixed a second, independent undercount.** `coremark_tb.v` used to count a Hazard3
write toward `haz_writes` only when `mem_wstrb_mux == 4'b1111` — a full word — where littlecpu's own
counter (`|dut_ours.mem_wstrb`) counts any strobe. Every sub-word store CoreMark makes on Hazard3's
side was invisible to the printed `writes=` figure, which under-counted at 13350 against littlecpu's
15701 for identical C source. The two now agree exactly (15701 each), which is what the fix predicts
for one image compiled once and run on two cores that compute the same thing: the `4'b1111` filter
also made marker/verdict detection depend on `.coremarkctl` staying a naturally-aligned 32-bit type,
which nothing enforced, so the fix moves those compares out from under the width test entirely rather
than merely widening it. Neither cycle count moved — CoreMark's markers and verdict word are always
full-word stores by construction, so this was a reporting defect in `writes=`, not in `cycles=`.

**1 iteration is not an undersized sample; it is exact.** `core_main.c`'s own `iterate()` is
`for (i = 0; i < iterations; i++) { core_bench_list(res, 1); core_bench_list(res, -1); }` — the
identical, seed-determined pair of calls every time, with no RNG reseed and no cache state (this core
and Hazard3 both have none) to warm up or cool down between iterations. So the per-iteration cost is
constant by construction, not merely observed to be stable the way ADR-0136 found for real-clock
jitter at 10/40/100 iterations. Measured directly rather than only argued: at 2 iterations, littlecpu
is 958879 cycles (479439.5/iteration against 479420.0 at 1) and Hazard3 is 1429994 (714997.0 against
714984.0), and the ratio reproduces to three decimals both times — 1.491×. The residual few cycles of
difference are one-time loop-entry cost amortising over more iterations, not drift in the per-iteration
figure. At roughly 4000 simulated cycles per wall-clock second for two real RTL cores under iverilog,
1 iteration is ~184 s and each additional one buys no new information at ~184 s more; `compare-timing`'s
`COMPARE_DHRY_RUNS=400` is chosen the same way, for the same reason (measured flat to 0.26%).
`COMPARE_COREMARK_ITERATIONS` is overridable for a reader who wants to reproduce the linearity check.

### The image against the harness's geometry

```
rom (.text):                       10768 bytes; placed budget 4096
ram (.coremarkctl + .data + .bss): 2104 bytes; placed budget 2048
the image needs 22 + 6 = 28 SB_RAM40_4K, and hx8k has 32 in total
  littlecpu   4 of its own + 28 for the image =  32 blocks: fits
  hazard3     4 of its own + 28 for the image =  32 blocks: fits
THE IMAGE DOES NOT FIT THE PLACED GEOMETRY (4096 rom, 2048 ram)
```

**This is a simulation and cannot be a placement, the same standing arrangement `dhry.lds` already
states for Dhrystone** — the 4 KB/2 KB harness `bench.lds` places neither benchmark, and nothing here
shrinks CoreMark to change that. The block-RAM arithmetic above is printed for its own sake, not
because either core's own census threatens the part: at this image's size both cores' own registered
census plus the image happens to land exactly at 32 of 32, which is a property of `SB_RAM40_4K`'s
256-word granularity and this image's particular byte count, not a claim that a slightly larger
CoreMark build would still fit. `soc/compare/coremark_fit.py` is the same script that would say so if
it did not.

### The clock factor, re-taken on this tree

`COMPARE_CORES="littlecpu hazard3" COMPARE_SEEDS="default 1 2 3 4" soc/compare/sweep.sh`, five
placements a side, the same hx8k/ct256 harness and `bench.S` ADR-0139 used:

|          | worst    | median   | best     |
|----------|----------|----------|----------|
| littlecpu | 30.90 MHz | 32.01 MHz | 32.69 MHz |
| hazard3   | 32.07 MHz | 32.45 MHz | 33.38 MHz |

littlecpu's figures reproduce ADR-0139's exactly (30.90/32.01). Hazard3's do not — 32.07/32.45 here
against 32.60/33.63 there — which is the expected shape of a five-seed sample rather than a change in
either design: **five seeds is a look, not a verdict**, the qualifier ADR-0139 already carried and
this ADR inherits rather than resolves. Both this sweep and ADR-0139's sit inside the part's
undetermined churn band (`soc/compare/sweep.sh` prints, correctly, that hx8k has none derived).

### The product

`soc/compare/coremark_dmips.py --mhz littlecpu=<worst|median> --mhz hazard3=<worst|median>`, on the
cycle counts above:

|          | worst-placement CoreMark | median-placement CoreMark |
|----------|---------------------------|-----------------------------|
| littlecpu | 64.45                     | 66.77                       |
| hazard3   | 44.85                     | 45.39                       |

**littlecpu's CoreMark score is 1.437× Hazard3's iCE40 configuration's at the worst placement, 1.471×
at the median** — the first CoreMark product this repository has been able to quote for this pair,
both factors taken in one session on one tree. Compare against the **clock** gap alone (1.06×/1.05×,
Hazard3 ahead) to see the cycle factor doing essentially all of the work: Hazard3 is faster clocked
and slower per iteration, and the second effect outweighs the first by roughly 40%.

**The wait-state bias above moves this product too, in Hazard3's favour**: correcting its cycle count
for the 1.98% the harness's write adapter spends puts Hazard3 at 45.76 CoreMark at the worst
placement and 46.30 at the median, which narrows the gap to **1.408× at the worst placement, 1.442×
at the median** rather than 1.437×/1.471×. Quote either pair with what it is — 1.437×/1.471× is what
the harness measured, 1.408×/1.442× is a lower bound with the adapter's own cost taken out — and do
not average them.

## The decomposition

**Hazard, region and operand stalls are already priced for this core alone**, on its own
`rv32imac_zicsr_zifencei_zkt` CoreMark build at 16 KB of ROM under `cxxrtl` (ADR-0136): hazard 28.9%
of cycles, region 10.2%, operand 9.3%. That figure describes this core's *own* stall profile on its
*own* build, not a component of the 1.491× cycle gap above — the compare harness's RV32IMA build runs
under `iverilog`, at a different geometry, with none of `make cycles`' stall-reason instrumentation
available for either core (`--stalls` is `test/cxxrtl.cc`'s and Hazard3 implements nothing like it).
Reading it here is context for *how this core spends its own cycles*, which the reader can weigh
against the 1.491× figure, not a term that sums to it.

**Bitmanip's contribution stays undecomposed**, the same statement ADR-0139 and ADR-0136 both already
make: no Zba/Zbb/Zbs build of this core exists to measure the delta against, on either side of this
pair — Hazard3's iCE40 build has none either (`EXTENSION_ZBA`/`ZBB`/`ZBC`/`ZBS`/`ZBKB` all 0), so this
pair does not even raise the question CoreMark's own README credits bitmanip for on the RP2350
comparison ADR-0136 already declined.

**The missing C extension's own contribution is the number the ticket that requested this
measurement called out as the most interesting one available, and it is measured — on littlecpu
alone, since Hazard3 cannot be asked the question at all.** `coremark_tb.v` runs both cores on one
image, and Hazard3's iCE40 build cannot decode a compressed encoding (`EXTENSION_C=0`) with no trap
handler installed to survive trying — handed an `-march=rv32imac` build, it never reaches its own
verdict marker and the run silently spends its whole cycle budget instead of failing loudly, the same
shape `soc/compare/placed_vs_synth.py` and `compare-smoke`'s three-way check exist to catch in a
placement, here showing up in a simulation. **But littlecpu's own half of that same run is a clean,
complete, isolated measurement** — same harness, same geometry, same port, the only thing that moved
is the compiler's ISA target — because `coremark_tb.v`'s always-running capture logic records
littlecpu's markers and verdict the moment they occur, independent of whether Hazard3 ever reaches
its own:

```
COMPARE_COREMARK_CFLAGS='-march=rv32imac ...'
COREMARK core=littlecpu marks=2 cycles=490781 verdict=1 writes=15701
COREMARK core=hazard3 marks=0 cycles=0 verdict=0 writes=230768
```

**littlecpu takes 2.37% MORE cycles with the C extension on** — 490781 against 479420 at
`-march=rv32ima` — while its own `.text` shrinks 29.4% (7600 bytes against 10768). Both directions
are real and neither cancels the other: fewer, denser instructions is not the same claim as fewer
cycles on this pipeline, and CLAUDE.md's own fetch-loop measurements already establish why a change
in code shape can cost cycles independent of instruction count (redirects, and where a load or store
lands relative to a 2 KB region boundary, are both address-dependent and both move when the compiler
re-lays out the same source at a different encoding). This ADR does not decompose *why* — that needs
walking the actual instruction stream RVC changed, which is out of scope here — only reports that the
net direction is the opposite of the naive "C makes code smaller so it must be faster" reading, on
this core, on this benchmark. `rtl/decoder.v`'s own commitment to keeping C is a density argument for
an 8 KB ROM (ADR-0002/0003), not a cycle-count one, and this is the first measurement stating that
distinction as a number rather than an assumption.

## Anti-fraud

`make compare-smoke` is unchanged by this ADR and does not need to be: it already runs all three
cores on `bench.S`, and this measurement adds no fourth harness top, only a new testbench
(`coremark_tb.v`) built from the same `bench_littlecpu.v`/`bench_hazard3.v` those three already share
— the same relationship `dhry_tb.v` already has to `compare-smoke`, off its path for the identical
reason (a much longer run, graded by its own RAM comparison instead). `soc/compare/coremark_fit.py`
and `soc/compare/coremark_dmips.py` are both probed in `test/probe_gates.sh`, mirroring
`dhry_fit.py`/`dhry_dmips.py`'s own probe groups line for line — including the one that matters most:
two cores whose data RAMs disagree are red, never a plausible-looking ratio. The wait-cycle
disclosure is probed too — a fixture with the real measured `wait_cycles=14176` line pins the exact
sentence `coremark_dmips.py` prints, so a later edit that silently drops or miscomputes the
percentage is caught the same way a dropped RAM comparison would be. `soc/compare/run_coremark_compare.sh`'s
own manifest check — the two-way match against `PINNED.sha256` that catches a file dropped in beside
the vendored tree rather than only a mutated one — is probed the same way `test/bench/run_coremark.sh`'s
identical check already is, including the concrete `core_portme.h`-shadowing case.

## Consequences

- `make compare-coremark` is a new, ungated, non-CI target, the same standing `compare-dhrystone`
  already has.
- `soc/compare/sweep.sh COMPARE_CORES='littlecpu hazard3'` is how the clock half above was taken; nothing
  about `sweep.sh` itself changed.
- CLAUDE.md's own Hazard3 bullet is updated in the same commit — see below — since it explicitly said
  "no product to quote for this pair, in either direction," which this ADR closes.
- No `rtl/` file changed.
- VexRiscv is excluded from this comparison, by construction, for as long as the pinned riscv-formal
  clone's Verilog has no M extension; that is a fact about the pinned clone rather than about this
  repository's own core, and is worth restating if the pin ever moves.
- Quote the ratio with its bias: 1.491× and 1.437×/1.471× are what the harness measured, 1.462× and
  1.408×/1.442× are lower bounds with the write-adapter's own cost taken out. Both belong in the same
  sentence, the way `soc/compare/coremark_dmips.py` now prints them in the same run.
- `run_coremark_compare.sh`'s vendor-tree check is a two-way match now, not `shasum -c` alone, closing
  the same gap `test/bench/run_coremark.sh`'s own check already closed for the single-core build.
