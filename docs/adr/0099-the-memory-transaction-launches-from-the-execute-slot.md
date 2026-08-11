# ADR-0099: The memory transaction launches from the execute slot, and the load turnaround stops existing

**Status:** Accepted · 2026-08-11 · *Amends design commitment 8 — six stall reasons become five —
and the emptiness check of commitment 5, which now reads three slots rather than four, retiring
[ADR-0026](0026-stalls-are-four-reasons-over-two-mechanisms.md)'s fourth. Re-derives the F and G of
[ADR-0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md): G moves from 6
to 5 and F does not move. Read against
[ADR-0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md), which is
the reason this was found at all.*

## Context

`make cycles` on the hand-written suite charged the accessor's load turnaround **1.4%** of cycles.
On Dhrystone the same column is **12.5%** — 227 522 cycles of 1 823 126. The suite is about 6%
loads and Dhrystone about 24%, and the turnaround is paid by *every* load whether or not anything
depends on it. That is the largest single gap between the two profiles in the tree, and it is
exactly the shape ADR-0084 warned the suite would hide.

The cost was structural, not accidental. `decoder_output.mem_addr` and `.rs2` are computed in decode
— the effective address is already there, because a misaligned access has to trap before it issues.
They then rode through the executor untouched, 64 bits of pass-through, and only reached the bus
from `rtl/accessor.v` a stage later. The memory needs a cycle to answer, so the answer arrived a
cycle after the accessor's own cycle, and that cycle had to be bought: `accessor_stall` froze the
executor, held `decoder_out`, and a `pending_*` slot carried the load's `rd` across the gap so the
decode scoreboard could still see it.

## Decision

**Present the transaction during the executor's cycle, from `decoder_out`.** A synchronous memory
latches it on that edge and answers for the whole of the next cycle, which is the cycle the same
instruction reaches `rtl/accessor.v` — where the lane select and the sign extension already lived. A
load now takes exactly the stages an ALU op takes.

Seven things follow, and none of them is a trade:

- `accessor_stall` ceases to exist. `stall` is five reasons, not six.
- `pending_valid`/`pending_rd` cease to exist, so two 5-bit comparators leave the hazard chain and
  the load-use shadow is one cycle shorter.
- `executor_output` loses `mem_addr`, `mem_data` and all eight `is_l*`/`is_s*` flags — 72 bits of
  pass-through. What the response cycle needs (two address bits and five width flags) is registered
  inside the accessor at launch, where it was already being registered under another name.
- The executor's freeze arm goes, and with it twelve `$past` assertions that existed only to
  describe it.
- Back-to-back loads stream at one per cycle.
- A store's write edge lands one cycle earlier, which strengthens rather than weakens the pipeline
  emptying that commitment 5 holds `fence.i` for.
- `make fit` reads **3473** against 3579, −106 cells, outside the ±50 churn band.

### The one thing that is not free: a held launch

Decode holds `decoder_out` unchanged for every cycle of a divide — an issued instruction the
executor has not taken. A request block reading it without a guard would drive the bus on all
thirty-three of them. **For RAM two identical writes are one write. For a device they are two**, and
the platform this core ships on has `mtime`/`mtimecmp` at `0x0002_0000`.

The guard is not a new state bit. The executor consumes `in` on exactly the cycles `state == init`,
which is exactly `!divider_stall`, so `littlecpu` hands the accessor that signal as `launch_taken`
and the whole request block is gated on it. A registered "already issued" bit would be a second copy
of a fact the pipeline already publishes, and keeping two copies in step is the kind of state
commitment 1 exists to refuse. **Launching only on the taken cycle is also the only correct choice
for loads**: a request fired early during a divide would have its answer on the bus thirty-two
cycles before the instruction reached the stage that reads it.

Two graders, each with a demonstrated red direction:

- **`components_accessor`**, a new `mode prove` task in `formal/components.sby` — the accessor had
  none, having previously carried no assertions. It states that the bus is driven exactly when the
  executor takes a memory instruction, and on no other cycle. Deleting `launch_taken` from
  `requesting` fails it at `accessor.v:238` in step 1.
- **`test/accessor_tb.v` counts transactions across a held store to `mtimecmp`.** Seven held cycles
  then one taken cycle must produce **one**. Under the same deletion it reports `got=8 expected=1`.
  The count is the only observable: every MMIO register on this platform is a plain register, so a
  repeated write differs in no value anywhere in the machine, only in how many times it happened.

## Consequences

### Cycles

| | suite | Dhrystone |
|---|---|---|
| before | 28 555 cycles, CPI 1.82 | 1 823 126 cycles, CPI 1.92, 0.640 DMIPS/MHz |
| after | 28 092 cycles, CPI 1.80 | 1 607 619 cycles, CPI 1.69, 0.727 DMIPS/MHz |
| delta | **−1.62%** | **−11.82%, +13.6% DMIPS/MHz** |

The Dhrystone accessor column goes from 227 522 to zero; 11 995 of those cycles reappear as operand
misses, which is why the win is 215 507 and not 227 522. **The suite is off by a factor of seven on
this change** — read the two columns as ADR-0084 says to.

### Period

Six placements a side, `soc/timing_sweep.sh` with `SOC_SEEDS='default 1 2 3 4 5'`:

| | ns, sorted | worst | margin over 12 MHz |
|---|---|---|---|
| before | 76.26 76.71 77.38 77.89 78.36 82.65 | 82.65 | 0.83% |
| after | 73.34 73.93 76.16 77.17 77.21 78.40 | 78.40 | **5.9%** |

The median moves −1.2%, inside the churn band and therefore a null. **The worst placement is what
moved**: −5.1%, and the requirement's margin goes from under one percent to nearly six. That is the
number to quote, because `SOC_MIN_MHZ` is a requirement over every placement and not a median.

### F and G

Re-derived from scratch by the procedure `formal/checks.cfg` documents, one check per config:

- `hang` red at 4, 5 and 6; PASS at 7. **F = 6, unmoved** — F is the first retire out of reset and
  no load is part of it.
- `liveness` red at gap 3 and 4; PASS at gap 5, at trig 10 and again at trig 15. **G = 5, down from
  6** — the cycle that leaves is the load turnaround.

Every depth in the table stays. The derived floors drop from F + G = 12 and F + 2G = 18 to 11 and
16, so `insn 19` now clears by three where it cleared by one, and `reg 15 22`'s seven-cycle window
clears G = 5 by two. A shorter floor is not evidence for cutting a depth.

### The suite's retire counts drop by one on 27 of 62 programs

`test/OBSERVED_FLOOR`'s `.S` numbers are observations of how much the monitor saw before the runner
stopped, and the runner stops when the `tohost` store reaches RAM. That store now lands one cycle
earlier, so on the 27 programs whose final cycle happened to retire something, one fewer retire is
observed; the other 35 are unmoved. The floors were lowered by exactly one, per that file's own
documented procedure for a legitimate drop. Nothing about coverage changed — the lost retire is a
tail-loop instruction after the program has already declared its result — and every affected floor
is still in the hundreds except `simple.S`, whose 4 still clears the recorded blindness signature of
0 or 1.

### Against the four goals

**Fast** — 11.82% of Dhrystone's cycles at a clock that did not move, which at a fixed 12 MHz is
11.82% of throughput. **Simple** — one stall reason, one executor arm, one pipeline slot and 72
struct bits deleted; nothing added but a six-bit register that replaces one of the same width.
**Readable** — a load and an add now have the same shape, and the file that owns the bus owns all of
it. **Formally verified** — one new proof task where there were none, F and G re-derived rather than
inherited, and all 85 generated checks, all five component proofs, `dmemcheck`, `imemcheck`,
`cover`, `pcloop_cover`, `nonperturbation` and `cosim-suite` green.

### What this does not claim

The divider still holds `decoder_out`, so the hold mechanism of commitment 8 is intact and only one
of its two reasons is gone. And the guard's proof is over the accessor alone: it says the bus is
driven only on a taken cycle. That it is *one* transaction per instruction rests on the executor
taking each instruction once, which is the pipeline property `pcloop` and the generated checks carry
and this task does not restate.
