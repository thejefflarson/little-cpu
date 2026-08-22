# 0124 — Two fetch windows out of one storage, and what a tied-off parameter costs

Status: accepted

## Context

The path to a dual-core ECP5 SoC needs two things from the platform before it needs an arbiter or a
second core: text that two harts fetch from **one** storage, and a machine timer with one `mtime`
and one `mtimecmp` per hart, which the privileged specification mandates. Both have to be free at
the single-hart defaults the up5k SoC ships with, because that build is a fraction of a nanosecond
over its 12 MHz requirement and there is nowhere to put cells.

A probe had already answered the interesting synthesis question: neither part infers a true
dual-port primitive from a second read port. Both replicate the bank and drive every copy from the
same write — one architectural storage, each copy keeping the one-read-one-write shape the design
already handles. That was a remembered result from a tree that no longer exists. It is re-taken
here, and mechanised.

## Decision

**`rtl/imemory.v` takes `NHARTS`, default 1, and publishes one fetch window per hart.** The
address, the word pair, the stall bit and the fault bit are packed low window first, so at one hart
every width is the scalar it has always been and no integrator changes.

**A load steals the window that answers it and no other; a write stalls every window.** A load has
to come out of some copy's read port, and they all hold the same words, so it comes out of window
0's — the other windows keep fetching. A write lands in every copy on one edge, and both parts
define a same-address write-during-read as returning *invalid* data on the reading port rather than
the old word. Zifencei permits fetching the old or the new instruction, never garbage, so every
window is told. `fetch_stall` is already a per-core input and already a stall reason with settled
hold rules, so this adds no stall reason and no new arm.

**`rtl/timer.v` takes `NHARTS`, default 1.** The window is two words of `mtime` plus two per hart,
rounded up to a power of two: four words at one hart, eight at two, with the top two reading zero.
`mtip` becomes one line per hart. The elaboration check that the base is aligned to the whole
window moves with it.

**The layout is deliberately not a CLINT's, and that costs portability.** A CLINT gives `mtimecmp`
a base of its own with room for 4095 harts and puts `mtime` far above them. Firmware written
against the map here does not port to a CLINT one. What the choice buys is one aligned window with
one equality in front of it, on a machine whose whole peripheral map is a handful of windows.

**The map reserves the timer's widest window — 32 bytes — at every `NHARTS`.** The shipping build
decodes 16 of them and the other 16 read zero. Without the reservation a peripheral placed at
`BASE+0x10` would work perfectly on the single-hart machine and would have to move on the day the
second hart landed, with nothing to report the collision in between: at one hart those addresses
read zero from every memory on the bus. `test/memmap_test.sh` reads every `BASE` parameter declared
under `rtl/` and refuses any that falls in the span, naming the address to move to.

## What was measured

### The mapping, on both parts

`rtl/imemory.v` alone, 2048 words:

| | one window | two windows |
|---|---|---|
| ice40 `SB_RAM40_4K` | 16 | **32** |
| ECP5 `DP16KD` | 4 | **8** |
| ECP5 `LUT4` / `TRELLIS_FF` | 352 / 96 | 517 / 108 |

The one-window ECP5 figures reproduce the remembered probe exactly. **The dual configuration can
never be built on the up5k**, and the 32 against that part's 30 block RAMs states it as a
measurement rather than an occupancy argument.

The write really is common. Grouping the mapped block RAM cells by the nets on their write port
gives 16 groups of 2 on ice40 and 4 groups of 2 on ECP5 — every copy on the same enable, address,
data and edge — and the cells within a group differ on their read port, which is what says they
serve different windows rather than being one window counted twice. `test/rom_replication.py` is
that check and `test/imem_share_test.sh` runs it on both parts, forces it red against a mutant
whose second window reads a pair of banks the write does not reach, and forces its other two
refusals against a netlist that passes.

### The single-hart netlist, and why it is not bit-identical

The landing gate for a tied-off change is the mapped-netlist digest: equal means no seeds are owed.
**It is not equal, and the reachable bar turned out to be lower than the criterion.** Every spelling
was measured against the base tree's canonical netlist, one variable at a time, on the shipping
`littlesoc`:

| edit, at `NHARTS = 1` | SoC `SB_LUT4` |
|---|---|
| the fetch window folded into a loop that includes window 0 | **+30** |
| naming `\|mem_wstrb && text_range` once and using it twice | **+64** |
| the timer's read mux written once, with hart 0 an arm of it | **+19 to +25** |
| an in-process `for` loop over the harts, either half | **+36 / −20** |
| **an inert generate loop — zero iterations, nothing elaborated** | **−18** |
| the port widths, the `NHARTS` parameter and the bit selects | 0 |
| the read mux inside a taken `generate if` arm | 0 |

So the shipping windows and hart 0 are spelled exactly as they were, and everything above the first
lives in a generate the single-hart build does not take. What is left is **−18 `SB_LUT4`**, 4217 to
4199, and the last row of that table is the whole of it: a generate loop that elaborates to nothing
at all moves the same count by the same −18. Nothing else moved — same modules, same ports, same
`SB_CARRY`, `SB_DFF`, `SB_DFFSR`, `SB_DFFESR`, `SB_RAM40_4K` at 20 and `SB_SPRAM256KA` at 2.

**Bit-identity is not reachable for a parameterised module here, and that is worth writing down.**
A width-one *vector* port cannot digest equal to a scalar one: the canonical JSON records the
declaration, so the choice is a `single_bit_vector` attribute on the net or the net keeping the
submodule's name instead of the top's. One or the other survives every spelling tried. Six further
differences are the names of anonymous cells inside `$__ABC9_DELAY` and the `SB_RAM40_4K`
simulation model, an id counter off by one.

## Consequences

The digest moved, so **the paired sixteen-seed sweep is owed** and has not been spent: `nextpnr` on
the machine this was written on cannot start. `make fit` and `make soc-timing` in CI are the
measurement, and the delta is 18 cells fewer inside a band measured at ±50 to ±68, with no cell type
changed — but that is an argument, not the sweep.

`SOC_EXPECT_EBR` stays 20 and `SOC_EXPECT_SPRAM` stays 2, both re-measured. The core is untouched,
so `formal/` is untouched: `formal/arbiter.v` transcribes window 0's behaviour and window 0 did not
move. `make window-test` gains the pair that says the timer's alignment check moved with `NHARTS` —
one base, accepted at one hart and refused at two — which is stronger evidence than the diagnostic
text, because iverilog's elaboration tasks take one string literal and the number cannot be in it.

Still open, and deliberately: the dual top, the arbiter, `HART_ID`, and any program that runs on two
harts. Nothing here instantiates more than one window or more than one hart outside a bench.
