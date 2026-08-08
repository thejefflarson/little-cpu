# ADR-0082: The machine timer interrupt is taken at a decode boundary

**Status:** Accepted · 2026-08-08 · *Amends `CLAUDE.md`'s ISA-target claim that `mie`/`mip` are
read-only zero and that there are no interrupts. Rests on
[ADR-0005](0005-traps-and-csrs-commit-in-decode.md)'s decode-commit rule and
[ADR-0026](0026-stalls-are-four-reasons-over-two-mechanisms.md)'s stall protocol without changing
either. Restricts the generated riscv-formal checks in the sense
[ADR-0010](0010-muldiv-verification-under-altops.md) admits, and mechanises the restriction the way
[ADR-0014](0014-expected-fail-is-the-m1-regression-baseline.md) requires. Defers Sail
co-simulation of the two new programs under
[ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md)'s pattern.*

## The gap

`mie` and `mip` read zero and are not writable. Not "interrupts are disabled" — **absent**. A core
in that state can run exactly one polled loop: no preemption, no periodic tick, no RTOS, nothing
that reacts to time. It is the largest functional gap this design has against every comparable
core, and unlike the two questions asked alongside it — 24 MHz on this part
([ADR-0080](0080-twenty-four-megahertz-is-not-reachable-on-the-up5k.md)) and an external-memory
project — it is reachable.

## Why an asynchronous event does not break the decode-commit rule

Every trap in this core is detected and committed in decode, and no state exists that a later cycle
has to un-commit. An interrupt is asynchronous. That reads like a conflict and is not, for one
reason:

`rtl/decoder.v`'s `next_pc` priority chain is `reset > stall > trap > mret > jalr > jal > branch >
sequential`, and `trap_entry` is `issuing && trap`. **An interrupt ORed into the trap arm as a
registered level is therefore taken only on a cycle that would otherwise have issued an
instruction.** The instruction it displaces has not issued: nothing downstream has heard of it,
nothing has to be taken back, `mepc` is its own address and it re-executes after `mret`. The
privileged spec permits taking an interrupt between two instructions, and this is literally that.

Two consequences fall out of the arm ORDER rather than out of new logic:

* `stall` is ABOVE the trap arm, so an interrupt automatically waits out a divide, a load
  turnaround, a hazard, the operand-fetch cycle, a stolen fetch window and a serialization. A
  half-completed `mret` is impossible for the same reason a half-completed divide is.
* The interrupt is NOT a stall reason. It costs no stalled cycle, so `make cycles` needs no seventh
  bucket and its six-reason identity is unchanged. Confirmed: `unattributed` is 0 over the whole
  suite, including the two new programs.

The one place the interrupt is not simply "the trap arm with another input" is the publish block.
An exception rides out on its own instruction — `out.valid` stays high and the execution flags are
cleared — because the instruction issued. An interrupt takes the BUBBLE arm instead, because its
instruction did not. So `issuing` now guards the last TWO arms of that block rather than the last
one, and `test/decoder_tb.v` asserts `out.valid` is low on an interrupt entry.

## What it costs

**One gate in the core.** `rtl/csrs.v` computes `interrupt_pending` as the AND of three flip-flops
— the platform's line, `mie.MTIE`, `mstatus.MIE` — and `rtl/decoder.v` ORs it into a trap term that
was already a six-way OR. The 64-bit counter and comparator live in `rtl/timer.v`, on the data bus,
outside the core: `make fit` does not see them and `make soc-timing` does.

Measured over four placements of the SoC, graded on the worst, against `SOC_MIN_MHZ = 12.0`: see
the pull request that lands this. The requirement holds at every seed. `make fit` moves within its
±50-cell churn band, which is a null in both directions rather than a result.

## Scope: the smallest conformant increment

* **Machine timer only.** `mtime` and `mtimecmp` memory-mapped at `0x0002_0000`, four words, in
  both `rtl/littlesoc.v` and `test/testbench.v` so the same `.S` program runs on both.
* `mip.MTIP` is the platform line, read-only. `mie.MTIE` is the one writable bit.
* **`mip.MSIP` and `mip.MEIP` stay read-only zero.** That is conformant WARL on a platform with no
  software-interrupt register and no external controller. Adding either means adding its hardware
  first.
* **No interrupt controller, no vectored `mtvec`.** Both are the same mechanism with more sources
  and can follow.
* `mcause` is `0x8000_0007`. An interrupt outranks the interrupted instruction's own fault, because
  that instruction did not execute; it faults when it re-executes after `mret`.

`mtimecmp` **resets to zero**, so `mtip` is asserted from the first cycle. That is what the
comparison says, and it is harmless because `mstatus.MIE` and `mie.MTIE` both reset to zero —
software must set `mtimecmp` before enabling anything, which every RISC-V platform requires anyway.
It is also forced: the cxxrtl runner deasserts reset before the first rising edge, so the whole `.S`
suite runs on cxxrtl's zero-initialised state rather than on the RTL's reset branch, and a non-zero
reset value would be invisible on the primary simulator and present on the board.

**The tick period is one clock cycle**, so 83.33 ns on the 12 MHz crystal. The spec requires only
that `mtime` advance at a constant frequency and that the platform provide a way to determine the
period; it names a fixed-frequency system with no frequency scaling as the case where driving
`mtime` from the cycle counter is the right answer, which is this board exactly. Firmware cannot
derive the period from anywhere else, so it is written at the top of `rtl/timer.v` and in
`CLAUDE.md`.

**`mtip` is a level.** It stays posted until `mtimecmp` becomes greater than `mtime`, and taking
the trap does not lower it — so a handler that returns without moving `mtimecmp` is re-entered
before the instruction at `mepc` runs. `test/asm/mtimer.S` asserts exactly that, with three
entries from one arming and a limit in RAM so the correct behaviour is not a livelock. It is the
easiest thing in this feature to get wrong: a core that cleared the pending bit on entry, or
latched an edge, passes every other case in the file.

**The RV32 update sequence is the spec's own**, and it is `low = all ones`, `high`, `low` — not
high-first. The first store makes the pair no smaller than the OLD value and the second no smaller
than the NEW one, so nothing fires in between. High-first is unsafe whenever the new high half is
smaller than the old one and the old low half is small: the pair passes through `{new high, old
low}`. **Both `test/timer_tb.v` and `test/asm/mtimer.S` fire a spurious interrupt that way on
purpose** before doing it correctly over the same values — a sequence whose failure path has never
run is not a check.

## The real cost is the proof

riscv-formal ships **no spec model for an interrupt** at `formal/pin.mk`'s SHA. It carries
`rvfi_intr` and four checks read it, but only to SUPPRESS an expectation: the two pc checks stop
requiring continuity across a retire that carries it. No `rvfi_*_check.sv` names `mie`, `mip` or
`mstatus`. So whatever this repo asserts is the only oracle, and the boundary has to be declared
rather than described.

**The input is tied off in all five riscv-formal harnesses**, and `formal/INTERRUPT_TIE_OFF` plus
`formal/check-interrupt-tie-off.py` are the mechanised form of that — set equality in both
directions on the harness set, a required `.irq_timer(1'b0)` on each, set equality in both
directions on the upstream files that mention `rvfi_intr`, and a re-derivation from the pinned clone
that no check models an interrupt CSR. A pin bump that adds a model goes red until the tie-off is
argued again. It is a prerequisite of `make -C formal check`, and
`test/probe_gates.sh` forces all ten of its failure paths.

Leaving the input free would not weaken those checks so much as point them at a machine their spec
does not describe: an interrupt makes a retire disappear, and `hang`, `liveness`, `unique` and every
per-instruction check are written about a stream of retires. It would also move the DEPTHS, which
are derived from F and G.

**`formal/traps.sv` grows the properties that then ARE the oracle**, with the timer line free and
`rtl/csrs.v` real, proved by k-induction. Its assertion count read off the model the proof ran on
rather than inferred from the word PASS: **34**, up from 25. What they say:

1. **Entry commits nothing.** `interrupt_pending` implies no `instret`, no `csr_wen`, no `csr_ren`
   and no `mret_entry`; and one cycle after an interrupt entry, `decoder_out.valid` is low. That is
   the no-wrong-path-state claim, stated about the interrupt.
2. **`mepc` is the un-issued instruction's own address**, and `mcause` is `0x8000_0007`. MIE moves
   into MPIE and MIE clears — that half was already asserted for exceptions and now covers this too.
3. **No entry without all three terms.** Each is read the way software reads it: no source, `mie`
   bit 7 clear, or `mstatus` bit 3 clear, each on its own forbids arming. `mip` is asserted to be
   exactly the platform line in bit 7 and zero elsewhere.
4. **The response is bounded, and the bound is a composition of two proofs, not one.** Entry clears
   MIE on the same edge it redirects, so the cycle after cannot take another one and nothing re-arms
   until an `mret` — asserted here. The decoder's own proof supplies the other half: an armed
   interrupt with no stall raises `trap_entry` on that cycle. Together: one entry per arming, on the
   first cycle that is not stalled.

**What is NOT proved, and why.** A literal "irq held ⇒ entry within N" cannot be stated in
`formal/traps.sv` without new assumptions: `divider_stall`, `accessor_stall`, `fetch_stall` and
`executor_out` are all free inputs there, and an environment that holds any of them forever
starves any bound. Rather than add four assumptions this repo would then have to discharge, the
bound is **measured**: see below. Say so plainly rather than quoting a proof that was not run.

**F and G were re-measured under the tie-off, and the flip points reproduce today's exactly** —
`hang` red at 6 and PASS at 7; `liveness` red at gap 5 and PASS at gap 6, at trig 10 and again at
trig 15. So F = 6 and G = 6 are unchanged and no `[depth]` line moves. That equivalence is the
evidence that the tie-off preserves the measurement, not an assumption about it.

## The measured worst-case latency

**33 cycles from `interrupt_pending` rising to `trap_entry`, which is 2.75 µs at 12 MHz.** Measured,
not estimated: a sweep of the arming delay from 1 to 80 cycles against a program of back-to-back
divides, loads, CSR reads and a `fence.i`, reading the two signals out of the elaborated design each
cycle. The distribution is flat from 0 to 33 with the maximum at exactly 33, which is the 32-cycle
divide plus the operand-fetch cycle behind it — the divider is the longest stall the core has, and
it is what sets the bound.

## What checks what

| Claim | Where |
|---|---|
| the timer's map, its 64-bit level compare, the torn write, byte strobes, the counter | `test/timer_tb.v` |
| `mie`/`mip` WARL, the three-term enable, entry disarming, `mret` re-arming | `test/csr_tb.v` |
| the decode-boundary commit, the cause priority, waiting out all five stalls | `test/decoder_tb.v` |
| entry's architectural effect, under k-induction | `formal/traps.sv` |
| the whole path, end to end, in a running program | `test/asm/mtimer.S`, `test/asm/mtimermask.S` |
| that the generated checks are unaffected | `formal/INTERRUPT_TIE_OFF` |

## Sail co-simulation: the spec permits both machines

`mtimer.S` and `mtimermask.S` are baselined `INCONCLUSIVE SAIL-LIMIT` in
`test/COSIM_EXPECTED_FAIL`, with the reasoning written at the entry as that file's header requires.
**This is not work left undone.** Everything that differs between the two machines is a quantity the
privileged spec explicitly leaves to the platform: where `mtime` and `mtimecmp` are mapped; the tick
period, required only to be *constant* and to be published; and how promptly a change in the
comparison is reflected in MTIP — "eventually, but not necessarily immediately", with the spec
itself warning software that a spurious timer interrupt can follow from that. This platform ticks
once per clock cycle; the model ticks once per two instructions. Both are conformant.

A value exemption was considered first, because `test/cosim.py`'s `NONCOMPARABLE_CSRS` is the right
tool when only a VALUE is incomparable — that is what closed `mcycle`, and `mip` is already on that
list. It cannot close this one, for a structural reason rather than an effort one: with different
tick periods the interrupt lands between a **different pair of instructions** on the two machines,
so what differs is the POSITION of the architectural change in the sequence — which is exactly what
that mechanism goes on comparing. An exemption wide enough to cover it would compare nothing.

Configuration does not reach it either: `platform.clint` has only `base`, `size` and `supported`,
and nothing in the schema names `mtimecmp`, MTIP or the CLINT's layout. Reconciling would mean
adopting the model's address map and its tick period — choosing this platform's timer to suit a
reference model. That is a product decision, not a test fix, and it is not made here. Co-simulation
keeps its role on the other 59 programs, which is where the property it was built for lives.

## Consequences

* `CLAUDE.md`'s ISA-target section no longer says there are no interrupts. `mie.MTIE` is writable
  and `mip.MTIP` is real; `misa` is unchanged, since neither is a `misa` bit.
* `rvfi_intr` is now driven rather than tied low. It is required, not optional: both sim legs'
  monitor checks pc continuity across retires and stops only for a retire carrying `rvfi_intr`, so
  without it every interrupt is a monitor error.
* The SoC's memory map gains a third range. The read buses still join with an OR because the three
  ranges do not overlap and each answers zero outside its own.
* A seventh stall reason was NOT added, and must not be: an interrupt entry is a committing trap on
  an issuing cycle.
