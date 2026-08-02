# ADR-0057: What writable text costs, in ladder depth and in nanoseconds

**Status:** Accepted · 2026-08-02 · *Measures the two unknowns
[`docs/ideas/one-address-space-over-two-memories.md`](../ideas/one-address-space-over-two-memories.md)
put at the front of its own sequence. Supplies the empirical evidence
[ADR-0025](0025-formal-ladder-depths-are-derived-not-inherited.md) requires before a `[depth]` line
moves, re-runs
[ADR-0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md)'s recipe against
a sixth stall reason, and re-measures
[ADR-0054](0054-the-memory-system-and-the-first-real-timing-number.md)'s critical path against the
steal mux. Corrects ADR-0054's `SOC_MIN_MHZ` figure.*

## Why this was a spike

Both numbers need hardware that does not exist. F and G are measured by sweeping `hang` and
`liveness`, which needs a core that already has `fetch_stall` and a `formal/wrapper.v` that drives
it; the mux's cost needs a `rtl/imemory.v` with the mux in it. There is no way to measure first and
build second, so scratch RTL was built, measured, and thrown away. **Nothing in `rtl/` or `formal/`
ships from this ADR.** What ships is the tables below.

The scratch RTL was:

- `rtl/decoder.v`: `fetch_stall`, a sixth stall reason on the existing bubble mechanism, folded into
  `stall` and into the `hazard || operand_stall` arm.
- `rtl/accessor.v`: `mem_ren`, `!reset && in_valid && is_load`.
- `formal/wrapper.v`: `fetch_stall` registered from `(mem_ren || |mem_wstrb) && mem_addr < 0x2000`,
  and the same-address stability assumption weakened (below).
- `rtl/imemory.v`: a 2:1 mux on both bank read addresses from a free input, plus a byte-enabled
  write port.
- `rtl/littlesoc.v` and `soc/littlesoc.pcf`: the arbiter, a `mem_rdata` mux, and a pad for the mux
  select.

## The part the brief got wrong, and why it changes the answer

The brief proposes weakening `formal/wrapper.v`'s "instruction memory is a function of its address"
assumption for the cycle after a text store. Measuring F and G with that assumption at full strength
would measure a machine nobody is going to build, so the spike weakened it first and measured
second.

It has to be weakened in two places, not one:

1. **A text access of either kind** takes the bank read port for that edge, so the fetch window
   presented in the next cycle is not the ROM's answer for `imem_addr` at all. That is one cycle of
   free data after a load as well as after a store.
2. **A text store** additionally changes what the ROM answers from the cycle after that onward. The
   array is written on the same edge and the fetch address is published a cycle early (ADR-0054), so
   the first fetch that can see the new value is two cycles after the store.

The spike therefore drops the compare on the cycle after any text access and, for a store, on the
cycle after that as well. That is the weaker of the two readings, chosen deliberately: an assumption
can only make checks easier, so erring wide is the safe direction for a depth floor.

## The measuring rig, checked against ADR-0046's published numbers first

`fetch_stall` forced low and the assumption restored to full strength is main's environment,
expressed on the spike tree. Under it:

| sweep | result | ADR-0046 |
|---|---|---|
| `hang`, check cycle | red at 6, PASS at 7 | red at 6, PASS at 7 |
| `liveness`, trig 10 | red at gap 3, PASS at gap 4 | red at gap 3, PASS at gap 4 |

F = 6, G = 4. The rig reproduces both published figures exactly, so the numbers below are a
difference and not a re-derivation from scratch.

## F, from `hang`

`rvfi_hang_check.sv` asserts on a registered flag, so the flip point is F + 1.

| `hang` check cycle | verdict |
|---|---|
| 4 | `bad state property 0 reachable at bound k = 4` |
| 5 | `bad state property 0 reachable at bound k = 5` |
| 6 | `bad state property 0 reachable at bound k = 6` |
| **7** | **PASS** |
| 8, 9, 10, 11, 12 | PASS |

**F = 6, unchanged.**

That surprised me, so here is why it holds. A steal needs a data access, a data access needs an
instruction that has already reached the accessor, and that instruction retires on the next edge
regardless. The sixth stall reason cannot delay the *first* retire — only a later one.

## G, from `liveness`

The smallest passing `check − trig` is the worst-case gap.

| trig | gap 3 | gap 4 | gap 5 | gap 6 | gap 7 |
|---|---|---|---|---|---|
| 10 | red k=13 | red k=14 | red k=15 | **PASS** | PASS |
| 15 | — | red k=19 | red k=20 | **PASS** | PASS |
| 20 | — | — | red k=25 | **PASS** | PASS |

(Trig 10 was also swept at gap 2, red at k=12, and gap 8, PASS.)

**G = 6**, stable at three trigger cycles rather than an artefact of one. The gap-5 counterexample is
the non-vacuity witness ADR-0046 describes: `liveness` opens with `assume(rvfi_valid)` at its trig
cycle, so an unreachable trigger would PASS at every gap instead of failing below 6.

## Attribution: the stall reason is not what costs the two cycles

Four configurations, one `liveness` sweep each at trig 10, nothing else changed:

| `fetch_stall` | imem stability assumption | G |
|---|---|---|
| forced low | in force | **4** (reproduces main) |
| live | in force | **5** |
| forced low | weakened | **6** |
| live | weakened | **6** (the proposal) |

- **The sixth stall reason costs +1 on its own.** That is the steal bubble, and it is also the
  witness that the modelled arbiter is reachable: if `fetch_stall` could never assert, this row would
  read 4.
- **The weakened assumption costs +2 on its own**, and it dominates. With no `fetch_stall` the core
  *consumes* the free window, decode issues from an instruction word that changed underneath it, and
  the operand-fetch cycle pays for the recovery. That is one lost cycle for the garbage window and
  one for the refetch — the two cycles the brief attributes to a text access, arriving through a
  mechanism the brief does not describe.
- **Together they are still 6**, because they are the same two cycles counted two ways.

**The consequence for the sprint is that `fetch_stall` is not the expensive part of this proposal.**
Removing it would not buy a cycle of ladder depth back; it would cost correctness and leave G at 6.

## The control, so the weakening is known to be a weakening

ADR-0042 decision 2 and ADR-0046 both record that disabling the stability assumption outright makes
`hang` produce a real counterexample. Re-run here:

```
imem_unstable forced to 1 (assumption fully off), hang 1 30:
  bad state property 0 reachable at bound k = 30 SATISFIABLE
```

So the spike's edit really did loosen the environment, and the version it ships is strictly between
"in force" and "off" rather than being a no-op that happened to move a number.

## Every `[depth]` line, re-checked

F + G goes 10 → **12**. F + 2G goes 14 → **18**.

| line | floor | configured | margin | verdict |
|---|---|---|---|---|
| `insn 15` | two-hop **18** | 15 | **−3** | **must move** |
| `ill 15` | same configuration, same floor 18 | 15 | **−3** | **must move** |
| `csrw 30` | 18 | 30 | 12 | stays |
| `reg 15 20` | window ≥ **6** | window 5 | **−1** | **must move** |
| `pc_fwd 10 30`, `pc_bwd 10 30` | window ≥ 6 | 20 | 14 | stays |
| `causal 10 20`, `causal_mem 10 20` | window ≥ 6 | 10 | 4 | stays |
| `liveness 1 10 30`, `unique 1 10 30` | trig-to-check ≥ 6 | 20 | 14 | stays |
| `hang 1 30` | F + 1 = 7 | 30 | 23 | stays |
| `csrc_upcnt 1 15`, `csrc_any 1 15` | measured **9**, unchanged | 15 | 6 | stays |

**The `csrc_*` floor was re-measured rather than derived**, because ADR-0046 is explicit that F and G
do not bound it: neither figure is about two retires of a particular kind. Its own mutation probe —
delete `MSCRATCH: mscratch <= warl;` from `rtl/csrs.v` and sweep for where the counterexample
disappears — gives PASS at 8 and red at 9, 10, 11, 12, 13 and 15. Same flip point as ADR-0046, so the
floor is still 9 and 15 still clears it by six. Following the table's *margin convention* instead
(one past the two-hop figure) would say 19; the measured floor governs, and 15 stays.

The three lines that move go to **`insn 19`**, **`ill 19`** and **`reg 15 22`** — the floor plus the
one cycle of margin the current table carries on exactly these two families.

## The ladder run, and why green at the shipping depths is not reassurance

The ladder was run at the **shipping** depths under the proposal's environment:

```
85 checks: 85 pass, 0 fail
Failure list matches EXPECTED_FAIL exactly (name and status).
Generated check set matches EXPECTED_CHECKS exactly (85 checks).
```

`insn`, `ill` and `reg` are below their floors in that run, so those checks go green having stopped
asking — the failure mode ADR-0046 exhibits with the shift-mask probe and ADR-0037 names in general.
A change that added `fetch_stall` without moving the depths would produce this exact output, so it
must not be quoted as evidence of anything.

At the re-derived depths the ladder is green with the same two set equalities exact, so the move is
affordable as well as necessary. **It is not free in wall time**:
`make -C formal check JOBS=4` went **7m14s → 13m42s** on this box at load 8–11. The `formal` job's
own measurement on CI is 4m22s against a `timeout-minutes: 20`, so a similar factor there lands
around 8–9 minutes and the margin narrows from about 5× to about 2×. Worth knowing before the
implementing change, not a blocker.

## The synthesis half

Toolchain, quoted with the number as ADR-0052 requires: **Homebrew Yosys 0.67+post
(`b8e7da6f`)**, nextpnr-0.10-108-g68c1acd8, `icetime` from the same install. Machine load 5–7
throughout; every run is `make soc-timing` with `SOC_PROG=add.S`.

**The baseline reproduces ADR-0054 byte for byte** on this box — 4041/5280 LC, 20 EBR, 2 SPRAM,
4 IO, 8 GB, **88.51 ns = 11.30 MHz**, 34.20 ns logic / 54.29 ns routing, 41 logic levels,
`imem.in_range → imem.in_range2`. So the deltas below are a comparison and not two measurements from
different worlds.

### Placement noise, measured rather than inherited

ADR-0054 characterised `make soc-timing`'s instability across **edits** at 3.6%. A delta between two
different designs also needs the instability across **placements of one netlist**, which nobody had
taken. Each row is the default placement plus `nextpnr-ice40 --seed 1/2/3` on the same `soc.json`:

| design | LC | `icetime` ns, four placements | MHz |
|---|---|---|---|
| baseline (main) | 4041 | 87.43 · 87.55 · 87.94 · 88.51 | 11.30 – 11.44 |
| read mux only, no write port | 4029 | 91.57 · 91.79 · 94.88 · 97.00 | 10.31 – 10.92 |
| write port only, no read mux | 4313 | 96.98 · 98.33 · 99.37 · 100.24 | 9.98 – 10.31 |
| **both (the proposal)** | **4336** | 94.89 · 95.78 · 96.72 · 96.77 | **10.33 – 10.54** |

Placement noise on one netlist is about 1–2% for the baseline and the proposal and about 6% for the
read-mux variant. The baseline and proposal distributions **do not overlap**: 87.4–88.5 ns against
94.9–96.8 ns, a median move of about **+8.5%**. That is outside the placement band and outside
ADR-0054's 3.6% edit band, so it is a real slowdown and not churn.

### The attribution is the reverse of the brief's

The brief names the mux as the risk and offers a load-free fallback on the grounds that "a store uses
the EBR's separate write port and needs only a read-enable gate."

- **The read mux costs no area at all.** 4029 cells against the baseline's 4041 — twelve fewer,
  which is inside `make fit`'s ±50 churn floor and means nothing in either direction.
- **The write port costs 285 cells** and is where all of the area goes. This ADR does not attribute
  those 285 further; the histogram comparison was not taken.
- **The write-port-only variant is the one that trips the gate.** It measures 9.98 MHz at the default
  placement and `make soc-timing` exits 2. Its distribution straddles 10.0 MHz.

The write-only and both variants are 96.98–100.24 and 94.89–96.77 — close enough that they cannot be
ordered at this resolution, and the honest statement is that they are the same design point in
timing terms. The separation that *is* solid is baseline < read-mux-only < {write port, both}.

**So the brief's fallback is the slower half, not the cheaper one.** If timing has to be bought back,
dropping text loads is not where to buy it.

### The cell census does not move, and byte enables are real

**20 `SB_RAM40_4K` and 2 `SB_SPRAM256KA` in every variant**, which is the declared count
`make soc-timing` already enforces. Adding a write port to the initialised banks does not degrade
block-RAM inference.

The mechanism is not the one `ice40/brams.txt` suggests, which is worth recording because the obvious
place to look is the wrong one. yosys did not reach for that file's `byte 1` / `wrbe_separate` at
width 16. It mapped both banks at
`READ_MODE = WRITE_MODE = 2` — 1024 × 4 — so each 32-bit bank is eight EBRs of one nibble lane each,
`MASK` is tied to zero and unused, and byte granularity comes from **four distinct `WCLKE` nets per
bank, each shared by exactly the two EBRs that carry one byte**. Read straight off the netlist:

```
imem.rom_even.0.0  WCLKE=[525]   imem.rom_even.0.4  WCLKE=[643]
imem.rom_even.0.1  WCLKE=[525]   imem.rom_even.0.5  WCLKE=[643]
imem.rom_even.0.2  WCLKE=[585]   imem.rom_even.0.6  WCLKE=[708]
imem.rom_even.0.3  WCLKE=[585]   imem.rom_even.0.7  WCLKE=[708]
```

Confirmed behaviourally as well, because writing the whole word instead of the strobed bytes would be
silent corruption and a structural reading of write enables is not enough to rule it out. A throwaway
bench drove the spike memory with `sb` into byte 0, `sb` into byte 2 and `sh` into the upper half,
over a word already written, checking the other three bytes and the other bank each time. Eight
assertions, all pass.

### `SOC_MIN_MHZ` is 10.0, and two documents say otherwise

The Makefile carries `SOC_MIN_MHZ := 10.0`, and its own comment derives it ("10.0 MHz is 11.5% under
the measurement" — 11.30 × 0.885 = 10.00). CLAUDE.md's Pointers section agrees. **ADR-0054's decision
section and CLAUDE.md's Commands block both say 10.5**, and the brief inherited that figure.

The Makefile is the gate and 10.0 is the number, demonstrated rather than read off the file: the
write-port-only variant measured **9.98 MHz and exited 2**, and the proposal measured **10.44 MHz and
exited 0**. Under a 10.5 floor the proposal would have failed too. ADR-0054's 10.5 and CLAUDE.md's
Commands block are corrected in this change.

## Decision

**GO, with three conditions.** Neither unknown kills the sprint's shape. The proposal clears
`SOC_MIN_MHZ` at every placement measured, the cell census does not move, byte-granular writes to
text are real, and the ladder move is three lines with the evidence for each.

1. **`insn 19`, `ill 19` and `reg 15 22` land in the same change as `fetch_stall`**, not as a
   follow-up. ADR-0046's sunset condition says any change that adds a stall reason must re-measure F
   and G before it lands; this is that measurement, and a change that adds the stall without the
   depths would leave M2 term 1 green having stopped asking.
2. **The steal mux's select was a free pad in this spike, so 8.5% is a lower bound.** The real select
   is `(mem_ren || |mem_wstrb) && text_range(mem_addr)`, a combinational cone off the accessor, and
   putting it on the mux adds a path this measurement excluded on purpose — isolating the mux's cost
   on the fetch loop was the point. `make soc-timing` must be re-run on the real change, and quoted
   with its toolchain.
3. **The 8.5% needs a ruling against ADR-0038's 12 MHz intent, not a shrug.** The design was 6%
   short; it would be about 15% short. ADR-0054 deliberately declined to reconcile the measurement
   with the intent, and this change makes the gap large enough that declining again is itself a
   decision. It is a design call and it belongs in the implementing change's ADR.

**If condition 2 comes back over the line, decode the steal from the accessor's registered inputs a
cycle early**, which takes the select's cone off the mux entirely. It costs a register and it changes
the shape of everything downstream, so it is not free. Dropping text loads to keep stores is the
other fallback on offer and it does not help: measured, the store side is the expensive half and the
mux is free in area.

## Consequences

- **`formal/checks.cfg`'s `[depth]` table has an amendment waiting for it, with evidence.** ADR-0025's
  rule is that depths move only on empirical evidence in either direction; the sweeps above are that
  evidence, and re-running them is the recipe in ADR-0046 §"TO RE-MEASURE", unchanged.
- **`insn` and `reg` still clear their floors by exactly one cycle at the new numbers.** ADR-0046
  called that thin and it stays thin. A seventh stall reason re-opens all of this again.
- **The `csrc_*` pair is the one family whose floor is measured rather than derived, and it did not
  move.** Anyone re-deriving depths after a pipeline change has to run its mutation probe separately;
  F and G will not tell them.
- **ADR-0054's `SOC_MIN_MHZ = 10.5` is corrected to 10.0**, in that ADR and in CLAUDE.md's Commands
  block. The value was never 10.5 in the Makefile, so nothing was mis-gated — but a floor quoted
  wrongly in the two places a reader looks first is a floor that will eventually be raised to the
  quoted number by someone tidying up.
- **`make soc-timing` now has a placement-noise figure as well as an edit-churn figure**: about 1–2%
  across `nextpnr --seed` on one netlist, against ADR-0054's 3.6% across edits. The two are different
  axes and a delta has to clear both.
- **The spike RTL lives in this pull request's history and nowhere else, and it is not a starting
  point for the implementing change.** It has no `pcloop` update, no decoder `FORMAL` block update,
  no `imemcheck` write path, and its SoC `mem_rdata` mux is a sketch.
