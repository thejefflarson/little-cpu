# Second part first, second hart second

The path from today's single-core up5k SoC to a dual-core ECP5 SoC. ADR-0110 settled the part, the
no-fork rule and which constraint binds; it deferred bring-up, the memory map, the clock, the
arbiter and every verification question. This brief resolves those and sequences them.

## The problem

The A extension cost about 453 logic cells for instructions no measured workload executes, justified
entirely as multi-hart groundwork: ADR-0106's argument was that the multi-core milestone should then
verify **exactly one new thing** rather than bringing up an instruction class, an arbiter and a
second hart through one symptom stream. Declining multi-core now strands that.

Meanwhile the milestone's load-bearing premise — *the same RTL places on ECP5 at 35.33 MHz, 20%
logic, 36 of 56 block RAMs* — is a remembered number from a tree that no longer exists, taken with a
toolchain that has since moved. Nothing in the repo builds for ECP5: no `nextpnr-ecp5` invocation,
no constraint file, no job. In a repo whose method is that a decision is a measurement with a date
on it, the milestone rests on the only undated number in it.

That is why the first step is an instrument and not a design.

## Goals and non-goals

**Goals.** A reproducible ECP5 measurement of today's single-core SoC, in CI. A dual-hart top that
places on ECP5 and meets a board-clock requirement, with the up5k single-core build measurably
unchanged. Cross-hart visibility through one shared storage verified, in its two graded faces:
atomicity (AMO indivisible across harts, LR/SC reservations invalidated by the other hart's writes,
`sc.w` never claiming a store that lost) and instruction-fetch coherence (the spec's `fence.i`
protocol, coordinated in software). The `.aq`/`.rl` argument re-derived for the two-hart
configuration, which ADR-0110 requires and ADR-0108 names the invalidators for.

**Non-goals.** An SMP software platform. Inter-processor interrupts and a CLINT-shaped map. Caches.
Store buffers. DMA. A second clock domain. Hardware bring-up on a physical board. Any change to the
core's pipeline structure or its CPI.

## What the measurements settled

**Text is shared storage, and that was a spec decision before it was a design one.** The first draft
of this brief gave each hart a private text memory, which deletes fetch arbitration cheaply. It was
withdrawn. RISC-V's multi-hart architecture is written assuming harts share a physical address
space, and the spec *supplies* the mechanism for cross-hart instruction-memory coherence — the
writing hart executes a data fence, then every remote hart executes its own `fence.i`, with the
coordination left to software. Private text does not relax that rule; it makes the rule
inexpressible, and makes the same address denote different storage per hart, which no software can
discover. Shared text without IPIs merely leaves an optional software protocol unimplemented, which
the spec explicitly permits. Legality was never the question; the difference in kind decided it.

**Yosys replicates rather than inferring a true dual-port RAM, and replication is the better
answer.** A probe extended `rtl/imemory.v` with a second fetch window in the same spelling and
synthesised it standalone for both parts. Neither part infers a TDP primitive: each bank is
replicated into two copies with a **common write**. That is one architectural storage — every write
lands in both copies on one edge — so text is shared in exactly the spec's sense, while each copy
keeps the one-read-one-write shape the shipping design already handles. ECP5 maps 8 DP16KD against
the real file's 4; ice40 maps 32 `SB_RAM40_4K` against 16.

**The dual configuration can never be built on up5k**, which the 32-against-30 EBR count now states
as a measurement rather than an occupancy argument. ADR-0110 had already ruled the part out.

**The tie-off folds to today's mapping.** With the second window unread, ECP5 is identical to the
real file (4 DP16KD, 352 LUT4, 96 FF) and ice40 reads 16 EBR and 251 LUT against 249 — two LUTs,
attributable to the probe wrapper. The single-hart build does not pay for the second hart's surface.

**That produces a better landing gate than a sweep.** For any tied-off landing, diff the mapped
netlist first: if the single-hart netlist is bit-identical, the placement distribution is provably
unmoved and no seeds need spending. Only a difference earns the paired sixteen-seed sweep. With the
worst placement at 12.03 MHz on current main and a measured placement spread of 4–9% on an unchanged
netlist, not spending seeds when a hash can answer is the right economy.

**A text write steals both harts' fetch windows.** The datasheets say a same-address
write-during-read returns *invalid* data on the reading port while storing the written word
correctly. Zifencei permits fetching the old **or new** instruction, not garbage, so the design does
not lean on "an invalid read is like a stale fetch". `fetch_stall` is already a per-core input and
already a stall reason with settled hold rules; the shared memory raises both outputs on a write.
No new stall reason and no new arm. The other collision path — one hart data-reading text while the
other writes it — is closed by the arbiter, because every data access requests the grant and the
writer owns it that cycle.

**Cross-hart `fence.i` comes off the deferred list.** With one storage, no store buffers and
per-hart completion in program order, the spec's protocol is implementable with polling alone: A
stores the patch and then a flag, B polls the flag through the same serialization point, executes
its own `fence.i`, and fetches from an array that already holds the store. IPIs optimise the
coordination rather than enabling it. This becomes a torture program, and it is exactly the program
private text could never have passed.

## The design

**Bring-up is boardless.** `synth_ecp5` plus `nextpnr-ecp5 --25k` on the existing `littlesoc`,
graded on nextpnr's own timing report — there is no `icetime` for ECP5, so that estimator is the
instrument, quoted as such and never merged with icetime numbers. `--25k` because the remembered
figures are 25F-die numbers and because keeping the smaller part is the same discipline that keeps
up5k binding. Censuses gate the mapping questions: expected DP16KD count, and a distributed-RAM
count for the register file. The ECP5 churn band is **derived from ECP5's own sweep**, never copied
from up5k's. Board, when a hardware step exists: Colorlight i5, same die class as the gate.

**The map.** Text at 0 is one shared 8 KB image with two fetch windows. Data RAM at `0x0001_0000`
and the timer are shared; the timer's window widens to eight words, with `mtime` shared and
`mtimecmp` per hart, which the privileged spec mandates. `rtl/timer.v` gains `NHARTS`, default 1, so
the up5k build synthesises exactly today's timer. `mhartid` already exists hardwired zero in the
read-only-zero row and becomes a `HART_ID` parameter; hart 0 keeps ID 0 as the spec requires. Both
harts boot at 0 and `crt0` branches on `mhartid`; hart 1 parks polling a RAM mailbox, since `wfi` is
a NOP here. ECP5 dual budget: ROM 8 plus RAM 32 of 56 DP16KD.

**The arbiter** is one module with one job: at most one hart drives the shared bus per cycle, an
AMO's read and write cycles are one indivisible grant, and no requester waits unboundedly.
Round-robin, because fixed priority starves. The request is a decode-level flag — *this hart
launches a memory access this cycle* — and deliberately **not** an address-range test, because a
range test reads the top of `immediate + rs1` and that whole family is priced at 9–17% of period.
An ungranted hart **holds** `decoder_out` exactly as a divider stall does, which is the seventh stall
reason and takes ADR-0106's recorded ruling for free: a wait raised before the executor consumes the
instruction holds. It is declared in all three places commitment 8 names and tied low in every
single-hart integrator. The core exports its atomic write cycle as `mem_lock` so the grant spans an
AMO's two cycles.

**LR/SC across harts** is the per-hart reservation plus one snoop input — the granted write address
and a write-valid, broadcast to both harts, ORed into the existing clear term. ADR-0106 predicted
this shape. Forward progress falls out structurally: a failed `sc.w` issues no transaction, so a
hart's SC can only fail because the other hart's write succeeded; failures cannot cascade and
constrained LR/SC loops cannot livelock. `.aq` and `.rl` stay decoded and ignored, on an argument
re-derived for two harts rather than on hardware: two in-order harts, one outstanding access each,
one serialization point, no buffers, so every execution is a sequentially consistent interleaving,
which dominates anything those bits request.

## Verification, leg by leg

Extending the existing legs to two harts is not expensive, it is **impossible at the pinned tools**,
and the design says so rather than pretending otherwise. RVFI describes one hart and ships no model
of an interrupt or of AMO; sail-riscv's emulator is single-hart; `test/cosim.cc` reads one core's
`regs_a` and never compares memory, so a load observing the other hart's store diverges
structurally. So every existing leg stays single-hart with mechanised tie-offs, and all new coverage
sits on the new surface.

| Leg | With two harts | Work |
|---|---|---|
| Generated riscv-formal checks | Describe one hart exactly as today; wait and snoop inputs tied off in all harnesses, mechanised both directions like `INTERRUPT_TIE_OFF`. F and G re-measured under the tie-off. | Cheap |
| Component proofs | `components_accessor` gains the snoop-clear property with a driven input; the decoder gains the seventh reason's hold arm; `pcloop`'s `f_may_stall` gains the term; **new `components_arbiter`** — grant `$onehot0`, AMO lock indivisible, bounded wait — the only genuinely new proof. | Real, bounded |
| `.S` suite, cxxrtl | Unchanged and still graded; structurally silent on cross-hart. | None |
| Dual harness (new) | Two monitor instances, one per hart's RVFI. Torture programs live outside `test/asm` with their own runner and floor file, the `test/bench` precedent: two harts AMO a shared counter N times each and the final word must be 2N; an LR/SC lock; a snoop cross-invalidation; and the cross-patch `fence.i` program. | The real new work |
| Sail co-simulation | Stays on the single-hart suite. The dual programs are outside its scope, stated plainly. | None |

**Cross-hart behaviour will have no independent oracle.** Its graders are the arbiter proof and this
repo's own assertions — the same epistemic status trap, CSR and AMO semantics already have, and the
same discipline applies: every grader with a demonstrated red direction. Named mutations:
grant-during-AMO-write-cycle, snoop term deleted, arbiter token frozen, cross-steal deleted, and a
write landing in only the writing hart's copy.

## Sequence

Each step is independently shippable on green CI and produces its own measurement.

1. **The ECP5 instrument.** `make ecp5-timing`, censuses, sixteen-seed sweep, band derivation, an ADR
   dating the numbers. Existing RTL only, no dual-core anything.
2. **`HART_ID` / `mhartid`.** A parameter reaching a row that already exists, plus `csr_tb` vectors
   and one mutation.
3. **The core's tied-off multi-hart surface.** Snoop port and clear term, the seventh stall reason
   with its three declarations, `mem_lock` out. Gate: mapped-netlist diff first, paired sixteen-seed
   up5k sweep only if it differs, `make fit` regardless. F and G re-measured under the tie-off.
4. **The shared-storage platform pieces.** The dual-window instruction memory and the timer's
   `NHARTS`, together, both tied off everywhere single-hart. Gates: the netlist diff,
   `SOC_EXPECT_EBR` exact, and the DP16KD census on the ECP5 target.
5. **The dual top.** Arbiter and its proof, the dual harness, the torture programs and their
   mutations, ECP5 dual place-and-route at `DUAL_MIN_MHZ := 25.0` graded worst-of-sixteen. The
   `.aq`/`.rl` ADR lands here, recording the shared-text coherence argument with it.

## Risks

The grant path is combinational and lands in the fetch loop on the dual part; it is unpriced until
built, and the fallback is a registered grant at one wasted cycle per contended access. ECP5's own
seed spread is unknown and is derived, not assumed. The toolchain is unpinned under a new gate,
which is the standing risk `FIT_MAX_LC` and `SOC_MIN_MHZ` already carry, with the stamped-sweep
refusal machinery as the response. The dual harness must zero all four ROM banks, the existing
two-state X rule doubled.

## Deferred

`msip` and IPIs, now with a cleaner reason: they optimise a protocol that is already implementable
rather than enabling one that is not. A CLINT-shaped map. SPI-flash boot. Hardware bring-up. The
85F. Any CPI ratchet on the dual harness. Extending co-simulation or riscv-formal to two harts,
which is impossible at the pins and revisits only on a pin bump that changes that.
