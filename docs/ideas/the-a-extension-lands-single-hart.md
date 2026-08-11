# The A extension lands single-hart

**Add Zaamo and Zalrsc to reach `RV32IMAC_Zicsr_Zifencei`, on one core, now — because that is
where the oracles work. The multi-core SoC it enables is deferred.**

## Why now, when it buys this core nothing

One hart already gets mutual exclusion by disabling interrupts, which is what the ESP32-C3
(RV32IMC, no A) relies on. A is worth roughly nothing to a single-core design as a *feature*.
It is worth landing anyway because **every verification leg this project owns is single-hart**,
and A is a large pile of per-instruction semantics that those legs can check today.

Nine AMO functions, signed and unsigned min/max boundaries, store-conditional success and
failure, three misalignment causes — all of it is verifiable now against the component proofs,
the `.S` suite and Sail co-simulation. Deferring A to the multi-core milestone means bringing up
a new instruction class, a memory arbiter and a second hart through one symptom stream, on a
board this project has never shipped on.

**Land A now and the multi-core milestone verifies exactly one new thing: cross-hart atomicity.**

Say this plainly in the ADR, so nobody later reads the area bill as a mistake. It is about eight
to nine percent of the core's cells spent on instructions no current workload executes.

## What the target is

`misa` moves `0x4000_1104` → `0x4000_1105` — bit 0 is A. The string becomes
`RV32IMAC_Zicsr_Zifencei`, and both Z-extensions stay named: `rv32imac` expands to
`rv32i2p1_m2p0_a2p1_c2p0_zmmul1p0_zaamo1p0_zalrsc1p0_zca1p0`, with no `zicsr` and no `zifencei`.

Full A rather than `Zalrsc` alone. A is the only form with a `misa` bit, so `Zalrsc` can never be
discovered at runtime; GCC emits one `amoswap.w.aq` for a C `__atomic_exchange_n` under
`-march=…imac` against a retry loop under `zalrsc`; every profile and RTOS assumes A. `Zalrsc`
alone is essentially theoretical — the factoring exists so implementations *can* claim partial
support, not because anyone shipped it.

## riscv-formal has no A models at the pin

Checked in the clone rather than assumed. `insns/generate.py` carries an `insn_amo` generator and
**every call site is commented out** — and only swap, add, xor, and, or, with no min/max family
and no LR/SC generator at all. There is no `isa_rv32ia*.txt`. `RISCV_FORMAL_EXTAMO` exists only as
unread channel plumbing.

So A gets the same treatment SYSTEM and MISC-MEM already get: **a recorded exclusion plus oracles
this repo writes**. `COMPLETE_EXCLUSIONS` gains one line, `complete.sv` gains the matching
predicate *and* an AMO cover goal so the exclusion cannot go vacuous, and `checks.cfg` stays
`isa rv32imc` because there is nothing else to generate.

What checks A instead: accessor `FORMAL` assertions for the read-modify-write function and the
reservation, a `dmemcheck`-shaped bus harness for the read-then-write sequence, the `.S` vectors,
and co-simulation as the semantic authority. The pinned `rvfi_dmem_check` already handles a retire
carrying both `rmask` and `wmask`, so an AMO retire reporting old data read and new data written
is checked at the monitored address for free.

## The shape: a sequenced unit in the accessor

AMOs pass through the executor untouched, exactly as loads and stores do. A small ALU and a state
extension of the existing pending mechanism in `rtl/accessor.v` do read, compute, write, with the
reservation register beside them. `lr.w` is a load that sets the reservation; `sc.w` is a
one-cycle conditional store that writes 0 or 1 to `rd`.

**One 33-bit adder/subtractor serves add and all four min/max compares** — borrow is unsigned
less-than, and a sign-disagreement correction gives signed, the same identity the executor and
decoder already prove against reference operators. Swap, and, or and xor fall out of the result mux.

Note what this corrects: the merged subtractor in `rtl/executor.v` operates on rs1 and rs2 **at
issue**, while an AMO's operands are the memory word and rs2, and `mem_rdata` does not exist until
two cycles later. Reusing it would need a cross-stage operand loop — forwarding-network-shaped,
already priced and declined. The *precedent* transfers; the hardware does not.

**No new stall reason.** The accessor stall lengthens, exactly as the divider stall already models
multi-cycle occupancy, so the six buckets and commitment 8's OR-identity are untouched.

Rejected: cracking AMOs into micro-ops in decode (breaks one-retire-per-instruction, which every
verification leg is built on, to save ~150 cells in the most proof-laden module), and serializing
them through the CSR emptiness check (pays 4–6 cycles for exclusivity the global stall already
provides, and widens a carefully-argued two-reason list to a third reason that is not one).

## The reservation survives traps

Cleared only by a store or SC whose word address matches, by any SC, or replaced by a new LR.
**Not cleared on trap entry or `mret`.**

That is conformant — single-hart SC failure is only *required* for intervening writes to the
reservation set, and surviving a trap is strictly more generous toward forward progress. And it
makes interrupt-induced livelock structurally impossible: a timer handler that never touches the
lock word cannot fail the store-conditional. The spec's constrained LR/SC sequences must
eventually succeed, and clearing on every trap with a timer running is exactly how an
implementation fails that while every individual instruction behaves.

A bonus falls out of the no-wrong-path-state commitment: `stall` outranks the trap arm of
`next_pc`, so an interrupt can never split an in-flight AMO. Atomicity against interrupts is free.

**Sail's observable policy must be checked against this before the co-sim baseline lands** — the
SC result is a register value co-simulation compares. If Sail clears on trap entry and the
difference is reachable from a suite program, mirror Sail and record which and why.

## `.aq` and `.rl` are decoded and ignored

One hart, in order, no store buffer, no cache, one outstanding access, every access completing
before the next issues: memory order is program order, which is sequentially consistent and
strictly stronger than anything those bits request.

**Record the argument; do not inherit it.** What invalidates it is named: a second agent on memory
— the second core, or DMA — a store buffer, a cache, or posted MMIO writes. The two-core arbiter
must keep per-access completion in program order or this reopens. This is the `fence.i` lesson:
"correct and for free" was a property of the configuration, and it stopped being true when text
became writable.

## What it costs, and the up5k question

Estimated **300–380 logic cells**: decode, eleven flags through two pipeline registers, a 31-bit
reservation, one 33-bit adder/subtractor, logic operations, result and write-data muxes. That
takes `make fit` from 3575 past the ratchet — a raise with a measured reason, which is what the
ratchet's own rules allow — and SoC occupancy from 82% to roughly 88–89%.

**Timing risk is routing pressure only.** The AMO datapath is registered on both sides and nowhere
near the fetch loop. The worst of six seeds today clears 12 MHz by 6.4% against a 3.6% churn band.

**The four-seed sweep is the go/no-go for whether A ships on up5k**, and the fallback is designed
in: register the AMO ALU and take a fourth cycle, a latency no measured workload pays because
nothing in the suite or Dhrystone executes an AMO.

**No conditional-generate split.** It would double every verification leg's configuration space —
which configuration do the proofs, the suite, the baselines and the depth table describe? — to
save cells the up5k can probably afford. If the sweep proves otherwise, that is the moment to
revisit, with numbers. Moving the project's home to ECP5 is a larger decision than A and must not
ride in on it.

## F and G will move

The AMO occupies the accessor one cycle longer than a load, and `RISCV_FORMAL_ALTOPS` does not
touch it. Expect **G 6 → 7** with F unchanged, pushing `insn 19` to about 21 and `reg 22` to about
24 — the two entries that today clear `F + 2G` by exactly one cycle.

Budget the BMC runtime against the formal job's twenty-minute ceiling. **This is the most likely CI
casualty.**

## Risks worth carrying into the tickets

- **Routing at ~89% occupancy** nudging the fetch loop under 12 MHz on some seeds. Bounded by the
  fallback; settled only by a sweep on real RTL.
- **A Sail reservation-policy mismatch**, surfacing as a co-sim failure on an SC result.
- **Formal job time** at the new depths.
- **The scoreboard's view of SC's `rd`** — a store that writes a register is a new category, and
  commitment 8 says in-flight `rd` visibility must hold on *every* cycle with no gap. It needs
  SC-shaped vectors in the decoder testbench before it is believed.
- **RVFI memory reporting for an AMO** must be assembled across two accessor cycles — address and
  write data at the write cycle, read data from the read. A plumbing bug here fails a
  `dmem_ch0`-shaped check on a correct core, and the write-cycle capture has no precedent in the
  file.

## Deferred

The shared-memory arbiter and cross-hart reservation snooping — the actual multi-core work, and
the clear-on-matching-write rule is already the shape that extends to it. ECP5 bring-up and any
question of moving the project's home. `Zacas`. `Zb*`. A conditional-generate RV32IMC build. A
liveness *proof* over forward progress: the reservation policy makes livelock structurally
impossible, and a harness for it is not worth building until a second hart exists to contend.
