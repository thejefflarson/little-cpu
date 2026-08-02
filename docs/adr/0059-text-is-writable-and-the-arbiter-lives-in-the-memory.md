# ADR-0059: Text is writable, and the arbiter lives in the memory

**Status:** Accepted · 2026-08-02 · *Builds step 2 of
[`docs/ideas/one-address-space-over-two-memories.md`](../ideas/one-address-space-over-two-memories.md).
Re-measures [ADR-0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md)'s
synthesis half against the real steal select, which is that ADR's condition 2, and answers its
condition 3. Reads its logic levels through
[ADR-0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md)'s model.*

## Context

`rtl/imemory.v` was a ROM with an idle write port. `rtl/littlesoc.v` and `test/testbench.v` decoded
two disjoint buses, so nothing on the data bus could reach the text region: no program could be
loaded at runtime, no trap handler could read the instruction that faulted, and ADR-0054's `.data`
gap had no path to close.

ADR-0057 built scratch RTL for the two unknowns, measured them and threw it away. This change is the
first of the sprint's shipping steps: the write port, the data read and the steal, with the core
still untouched.

## Decision 1 — the range decode and the steal live inside `rtl/imemory.v`

There is one implementation of both, and both consumers get it by instantiating the module.

The alternative — a separate arbiter module, or the decode written into each consumer — was
declined on the parameter. `test/testbench.v` runs `ROM_WORDS = 4096` against the SoC's 2048, so a
range test written outside the module would have to be given the size twice and would put the two
sim legs on different maps the first time one of them changed. A separate module would need the
same parameter and would add a second file to keep in step, for no coverage `test/imem_tb.v` does
not already give.

## Decision 2 — the two buses join with an OR, and it is verified across a write

`rtl/memory.v` answers zero outside its range and `rtl/imemory.v` answers zero outside the text
range, so `mem_rdata` is `imem_mem_rdata | dmem_mem_rdata` — a wired join rather than a select, one
less level, and it sits on the data path rather than the fetch loop.

The case worth checking is the one ADR-0054 decision 4 creates: `rtl/memory.v`'s read port is
**no-change on a write cycle**, so on a store its registered output holds whatever the last load
left there. Two live values on the wire would corrupt a text load. They cannot meet:

- On a **text load**, `mem_addr` is outside the RAM's range and `mem_wstrb` is zero, so
  `rtl/memory.v` takes its read arm and drives zero on exactly that cycle. The OR sees the text
  word and a zero.
- On a **store**, `rtl/imemory.v` answers zero — `data_hit` gates on `mem_ren`, not on the write —
  and the RAM holds its stale value. The OR sees a stale value and a zero, on a cycle no load reads
  `mem_rdata` (ADR-0015).

## Decision 3 — `mem_ren` is a port and both consumers tie it low

The idle bus presents `mem_addr = 0`, which is inside the text range, so without a read enable every
idle cycle would steal a fetch and the core would never run. The port lands here so the next change
drives it; `fetch_stall` lands unconnected for the same reason. `test/imem_tb.v` drives both, so
neither is untested — it is untested *by the suite*, which is a different thing and is why the bench
grew rather than the suite.

While `mem_ren` is low, a store to text still writes and still steals, with nothing consuming the
steal. No program in `test/asm` does that. One thing did: `test/testbench.v`'s baked-in `make waves`
program counted at address 1020, inside the text range. It counts at `0x10000` now, which is where
the comment above it always said the data was. Retires and writes are unchanged at 79 and 20.

## The measurement

**Toolchain, quoted with the number:** Homebrew Yosys 0.67+post (`b8e7da6f`),
nextpnr-0.10-108-g68c1acd8, `icetime` from the same install — the same box and build ADR-0057 and
ADR-0058 measured on. Machine load 8–26 throughout, with a sibling build live; `icetime` is a static
analysis, so load moves the wall time and not the numbers. `SOC_PROG=add.S`.

| design | LC | `icetime` ns, four placements (default, seeds 1–3) | MHz |
|---|---|---|---|
| baseline (`85e7113`) | 4041 | 88.51 · 87.94 · 87.55 · 87.43 | 11.30 – 11.44 |
| **this change** | **4304** | **98.08 · 97.75 · 98.19 · 96.92** | **10.18 – 10.32** |

**+263 logic cells and about +10.5% on the critical path.** The distributions do not overlap and the
gap is outside both bands that could explain it — 1–2% across placements of one netlist and
ADR-0054's 3.6% across edits — so it is the change, not churn. ADR-0057 predicted 4336 cells and
94.89–96.77 ns with a free pad driving the mux select; the real select is a combinational cone off
the accessor, and its condition 2 said the spike's figure was therefore a lower bound. It was, by
about two points.

**The cell census does not move: 20 `SB_RAM40_4K`, 2 `SB_SPRAM256KA`, 4 `SB_IO`.** Adding a
byte-enabled write port to the initialised banks does not degrade block-RAM inference, which is what
ADR-0057 measured structurally and `make soc-timing`'s declared census now re-checks on every run.

### Read the levels apart, or this change looks like an improvement

| | levels | LUT/setup | carry | per LUT | per carry | path |
|---|---|---|---|---|---|---|
| baseline | 41 | 25 | 17 | 3.31 ns | 0.34 ns | `imem.in_range → imem.in_range2` |
| this change | **30** | **29** | **1** | 3.35 ns | 0.83 ns | `imem.rom_odd…RDATA → imem.rom_even…RDATA` |

It is the same loop — bank read data, fetch window, decode, next PC, back into the banks — and it
still fits in one cycle with no flush (invariant 1). What moved is where it ends: it used to end at
the registered range flag, and it now ends at the bank read address, with the steal mux in front of
it. That trades **16 carry hops for 4 LUT levels**, so `icetime`'s single level count falls by
eleven while the path gets 10.8% slower. ADR-0058 wrote that hazard down as a warning; this is it
happening on a real change, and the reason `make soc-timing` prints the two counts separately.

### The ruling on ADR-0038's 12 MHz intent (ADR-0057 condition 3)

The design was 6% short of the declared 12 MHz. It is now about 15% short. **The declaration is not
moved, and this is the reasoning rather than a shrug:**

- The gate is `SOC_MIN_MHZ`, and it holds at every placement measured — 10.18 MHz worst against a
  10.0 floor.
- The alternatives are priced and none of them is cheap. ADR-0057 measured the read mux as free in
  area and the write port as all of it, so dropping text loads does not buy the time back. ADR-0058
  measured a second path within 1.7% of the fetch loop, which caps fetch-loop tuning near 11.5 MHz.
- What would buy it back is ADR-0057's own fallback — decode the steal from the accessor's
  registered inputs a cycle early, taking the select's cone off the mux. It costs a register and
  changes the shape of everything downstream, so it is a change of its own, not a tweak to this one.

The number to reconcile the intent against is now 10.2 MHz rather than 11.3.

## Consequences

- **`SOC_MIN_MHZ`'s margin is thin: 1.8% at the worst placement, against a 3.6% edit-churn band.**
  It is left at 10.0 deliberately — lowering a ratchet in the same change that consumed its headroom
  is how a floor stops meaning anything — but the next change to this design either buys time back
  or moves the floor with a reason. This is the first time the floor has been within the churn band
  of the measurement, and it should not be discovered by a red gate on an unrelated edit.
- **Nothing in `formal/` builds `rtl/imemory.v`**, so the ladder is out of contact with this change.
  It was run rather than assumed: 85 checks, 85 pass, both set equalities exact.
- **`mem_ren` and `fetch_stall` are dark**, so the suite's retire counts do not move — measured
  identical program by program, not merely above `test/OBSERVED_FLOOR`. The next change drives them
  and lands ADR-0057's three `[depth]` lines with them; the F and G that justify those were measured
  against a modelled arbiter, and this is the arbiter they modelled.
- **The bus stays non-faulting.** Out-of-range loads read zero, out-of-range writes are dropped
  without aliasing a mapped word, out-of-range fetch reads zero. Nothing here can refuse a
  transaction, so nothing faults after decode and invariant 2 is untouched. That is ADR-0044
  option 1, adopted for the text region as the brief proposes.
- **`test/imem_tb.v` covers the data port against a flat reference**, per ADR-0054 decision 5's
  rule: the reference is a word array updated byte by byte from the strobe, never derived from the
  bank split. Seven mutations of `rtl/imemory.v` were run against it — swapped write bank, no steal,
  strobes ignored, range test open, `fetch_stall` stuck low, read answer unmasked, and the steal
  ungated by `mem_ren` — and each is red with a diagnostic naming its own defect.
