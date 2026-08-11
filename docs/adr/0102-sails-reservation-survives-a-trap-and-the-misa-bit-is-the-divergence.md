# ADR-0102: Sail's reservation survives a trap, and `misa` — not the reservation — is what would have gone red

**Status:** Accepted · 2026-08-11 · *Stands up the reference model for the A extension, ahead of any
RTL. Amends [ADR-0043](0043-the-reference-model-is-configured-as-this-core.md)'s configuration with
the atomics knobs it left off, and narrows the question
[ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md) and
[ADR-0095](0095-co-simulation-is-required-and-its-fetch-is-verified.md) made load-bearing: what a
required check can be red about.*

## Context

The A extension is coming — nine AMOs plus `lr.w`/`sc.w`, reaching `RV32IMAC_Zicsr_Zifencei`. Its
design brief settles this core's reservation policy: **cleared by a store or SC whose word address
matches, by any SC, or replaced by a new LR, and not cleared on trap entry or `mret`.** That is
deliberate. It makes interrupt-induced livelock structurally impossible — a timer handler that never
touches the lock word cannot fail the store-conditional — and the spec's constrained LR/SC sequences
must eventually succeed.

The reservation policy is the one part of LR/SC the specification leaves to the implementation, so
it is the one part a reference model cannot be authoritative about. And co-simulation compares
architectural register values: **an SC's result *is* a register value**, 0 for success and nonzero
for failure. If Sail drops its reservation where this core keeps it, and a suite program can reach
the difference, every LR/SC program diverges on a correct core. Co-simulation is a required check on
`main`, so that is a red merge gate earned by the reference model — discovered in the middle of the
hardest ticket of the extension rather than before it.

The brief flagged it and declined to guess:

> **Sail's observable policy must be checked against this before the co-sim baseline lands** — the
> SC result is a register value co-simulation compares. If Sail clears on trap entry and the
> difference is reachable from a suite program, mirror Sail and record which and why.

Reading the model's source would answer it. It would not be evidence, and this repository does not
accept an inference where an instrument exists.

## What was measured

`test/sail/reservation_probe.S` runs under the Sail model **alone**. No core is involved and none
could be: this core does not decode `lr.w`/`sc.w` yet, so a comparison would have had nothing to
compare. Six cases, one architectural event each, every one leaving a bit that is 1 when the SC
succeeded. Bit 0 is an anti-vacuity control — an SC that nothing could have invalidated — and the
runner exits nonzero if it fails, because with it clear no other line means anything. The handler
touches no memory, which is what keeps the trap cases about trap entry rather than about the
handler's own stores.

```
$ make sail-reservation-probe
sail:   ~/.cache/little-cpu/sail/bin/sail_riscv_sim
config: test/sail/rv32imc_zicsr.json
isa:    rv32imc_zicntr_zicsr_zifencei_za128rs_za64rs_zaamo_zalrsc_zca_zvl32b_zvl64b_zvl128b_zvl256b
command: sail_riscv_sim --config rv32imc_zicsr.json --inst-limit 5000 probe.elf

HTIF located at 0x10000
Entry point: 0x0
FAILURE: 79 (0x0000004f)

sc.w back to back with the lr.w (control)      SUCCEEDED
sc.w inside the trap handler, no mret          SUCCEEDED
sc.w after trap entry and mret                 SUCCEEDED
sc.w after an mret with no trap behind it      SUCCEEDED
sc.w after a same-hart sw to the reserved word failed
sc.w one word up from the reservation          failed
```

(Paths abbreviated; the run prints them absolute, and prints the command it is
about to run rather than a restatement of it.)

**Sail does not clear the reservation on trap entry, and does not clear it on `mret`.** The two are
separated rather than inferred from each other: case 2 runs the `sc.w` *inside* the `ecall` handler,
so no `mret` has executed when it is graded, and case 4 executes an `mret` with no trap behind it —
`mepc` set by hand, which is legal in M-mode and is the only way to isolate that half.

That is this core's policy exactly. **The model mirrors the design, so there is no divergence to
mirror, no baseline entry to add, and nothing to redesign.** The risk this was written to retire is
retired, in the direction that costs nothing.

The two remaining lines are the knobs, confirming they mean what the configuration now claims. Both
were measured by flipping them, not read:

| `reservation_set_size_exp` | `require_exact_reservation_addr` | `sc.w` one word up |
|---|---|---|
| 2 | true  | fails |
| 2 | false | fails |
| 3 | true  | fails |
| 3 | false | **succeeds** |

So the two overlap: at a one-word set the exact-match knob is unobservable, and with exact matching
on the set size is. The shipped pair is `2` and `true` — two independent reasons for one answer,
both set to what this core does rather than only whichever one is currently load-bearing, because
the other is what a later widening lands on. `invalidate_on_same_hart_store` is the one knob here
that carries its line alone: at `false` the same-hart-store case succeeds, which is an SC succeeding
where this core fails it.

**The direction matters and only one of the two needs an argument.** Spurious SC failure is
architecturally permitted, so a core that fails an SC where Sail succeeds is conformant. The reverse
— succeeding where Sail fails — is the direction that has to be defended, and it is the direction
`invalidate_on_same_hart_store: false` would have put the model in. Also relevant, and the reason
the region attributes below are not a loose end: the spec's eventual-success guarantee attaches only
to regions whose reservability PMA is `RsrvEventual`, which the one main-memory region here already
declares.

## What would actually have gone red

Turning the extension on surfaced a different divergence, in a register nobody was watching. **`A`
is not an umbrella over `Zaamo` and `Zalrsc` in this model. It is the `misa.A` bit and nothing
else.** All four combinations, measured with a `csrr` of `misa` and with the probe:

| `A` | `Zaamo` / `Zalrsc` | `misa` | `lr.w` / `sc.w` |
|---|---|---|---|
| true  | true  | `0x40001105` | execute |
| true  | false | `0x40001105` | execute |
| false | true  | `0x40001104` | execute |
| false | false | `0x40001104` | illegal instruction |

`rtl/csrs.v` returns `misa` as a constant `0x4000_1104`. With `A: true` the model reads
`0x40001105`, and `test/asm/csr.S` parks that value in an architectural register twice:

```
$ ./test/cosim.py --quiet csr.S
DISAGREE csr.S
DIVERGENCE at architectural change #17
  sail instruction #24  pc=0x0000004e  csrrs x10, misa, x0    test_5+0
  sail : x10=0x40001105
  core : x10=0x40001104   (cycle 62, decode pc=0x00000056)
COSIM-STATUS DISAGREE AT 17
```

That is the same failure mode this configuration was built to end.
[ADR-0043](0043-the-reference-model-is-configured-as-this-core.md) exists because a
`--config-override` on the model's default RV32 machine inherited A, B, D, F, S, U and V, and the
symptom was one wrong `misa` read. Turning `A` on here would have re-created that exact symptom from
the opposite direction — deliberately this time, which is worse.

So `Zaamo` and `Zalrsc` are on and `A` stays off. The model executes the instructions; it does not
claim the bit the core does not claim. **`A` flips in the same change that makes `rtl/csrs.v` return
`0x4000_1105`**, which is the brief's stated target and belongs to the ticket that writes the RTL.
Flipping it early would also hide that transition: with the model already claiming A, the change
that makes the core claim it would be green either way, and a step that cannot fail is not a check.

This is the finding the ticket was worth having, even though it is not the finding it went looking
for. The reservation question came back clean; the question nobody asked came back red.

## Decision

1. **`Zaamo` and `Zalrsc` are enabled in `test/sail/rv32imc_zicsr.json`; `A` is not.** The first two
   are what make the instructions executable, the third is `misa.A`, and it moves with
   `rtl/csrs.v` and not before. `Zabha`, `Zacas`, `Zama16b` and `Zawrs` stay off with the reason at
   the site, as the rest of that file does.
2. **This core's reservation policy stands as the brief settled it, unchanged.** Sail's observed
   policy is identical on both events, so there is nothing to mirror and nothing to record as a
   divergence. `test/COSIM_EXPECTED_FAIL` is untouched.
3. **`platform.reservation` describes this core**: a one-word set, exact address matching, and
   invalidation on a same-hart store. Every value is measured against the probe rather than argued
   from the schema.
4. **`memory.misaligned.exceptions.amo` and `.lrsc` become `AlignmentException`**, the causes this
   core raises (4 and 6). Both were upstream defaults picked for a machine with A disabled, and the
   `amo` default let a misaligned AMO complete without faulting at all. Neither is reachable from
   `test/asm` today; they are set now because the alternative is meeting the model's default the
   first time the suite grows a misaligned AMO.
5. **The probe is committed and runnable**, as `make sail-reservation-probe`. A finding about a
   pinned external model is perishable — a pin bump can move it — and a claim that cannot be
   re-derived in the tree that relies on it is not a measurement.
6. **The probe is graded, not merely printed.** It exits nonzero when the control case fails, when
   the model reaches no HTIF verdict, and when the verdict carries no marker bit. Its own red
   direction is demonstrated: with `Zaamo`/`Zalrsc` off, `lr.w` is an illegal instruction and the
   run reports `FAILURE: possible trap loop detected with MEPC=0x30`, which the runner rejects as
   "nothing was observed" rather than decoding as a mask of zero. That last part is the guard worth
   having — the model prints prose under the same `FAILURE:` prefix it prints numbers under, and a
   pattern loose enough to match both would have read a run that executed no atomic at all as six
   failed cases.

   **It is not in `make probe-gates`, and that is a constraint rather than an omission.**
   `probe-gates` is hermetic and runs as a prerequisite of `make test`, which has to keep working on
   a machine with no Sail install. A gate that executes the pinned model cannot be either. The red
   direction above is therefore demonstrated by hand and recorded here, which is the weaker of the
   two mechanisms this repository uses and is the one available.

## Consequences

The A-extension RTL ticket starts with a reference model that already executes atomics and already
agrees about reservations, and its remaining co-simulation work is one boolean and the `misa`
constant, changed together.

**The probe is not in `test/asm` and must not move there.** Both sim legs glob that directory, this
core does not decode these instructions, and the graded suite's three build sites —
`test/run_tests.sh`, `test/cosim.py`'s `assemble()` and the Makefile's `soc-rom` — stay at
`-march=rv32imc_zicsr_zifencei`. The probe is the only thing here built at `rv32imac_zicsr_zifencei`.
It is also the only executable in this repository that asks the reference model a question instead
of grading something against it, which is why it reports a table rather than a verdict.

**What this does not settle, and is deliberately left to other tickets.** The memory map is still a
single 1 MB `MainMemory` region carrying `atomic_support: AMOCASQ` and `reservability: RsrvEventual`
over its whole extent. The machine timer's four words at `0x0002_0000` sit inside it and should be
`AMONone` + `RsrvNone` — a reservation must be refused outside RAM — and out-of-region accesses will
fault. Both are region-attribute work with their own tickets, and both are visible in this
configuration as things that are true of the model and not yet true of the platform.

**A green co-simulation run still says nothing about atomics.** No program in `test/asm` executes
one, so the 62-program suite exercises none of what was turned on here. The verdicts before and
after this change are identical for exactly that reason, which is the acceptance criterion and also
the limit of what it demonstrates: this configures the oracle, it does not test the core.
