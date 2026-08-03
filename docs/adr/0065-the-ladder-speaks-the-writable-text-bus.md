# ADR-0065: The ladder speaks the writable-text bus, and `imemcheck` stops at the first store

**Status:** Accepted · 2026-08-02 · *Builds step 4 of
[`docs/ideas/one-address-space-over-two-memories.md`](../ideas/one-address-space-over-two-memories.md)
on top of [ADR-0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md) and
[ADR-0060](0060-the-steal-reaches-decode-as-the-sixth-stall-reason.md). Pays off ADR-0060 decision 3,
which tied `fetch_stall` low in four tasks and recorded that as a placeholder. Supplies
[ADR-0049](0049-every-formal-assume-names-its-scope-and-its-discharge.md)'s three clauses for both
assumptions it changes.*

## Context

ADR-0060 drove `fetch_stall` in `formal/wrapper.v` from a transcription of `rtl/imemory.v`'s arbiter
and left the four hand-authored tasks — `imemcheck`, `dmemcheck`, `cover`, `complete` — tying it
low. It named `imemcheck` as the one where the bill would come due: riscv-formal's
`rvfi_imem_check` pins one halfword for the whole trace, and a store to text can change it.

Three things needed settling. Whether the four tasks can model the arbiter at all. Whether the
wrapper's weakened stability assumption survives a program that stores to text in a loop. And what
`imemcheck` should do about a halfword that moves.

## Decision 1 — one transcription of the arbiter, instantiated everywhere

`formal/arbiter.v` is `rtl/imemory.v`'s equation written once —
`(mem_ren || |mem_wstrb) && mem_addr < 0x2000`, registered — plus a `text_write` output for the two
consumers that also need the store's own cycle. Every harness in `formal/` instantiates it:
`wrapper.v`, `imemcheck.sv`, `dmemcheck.sv`, `cover.sv`, `complete.sv`. **`fetch_stall` is a free
input nowhere and a constant nowhere.**

This mirrors ADR-0059 decision 1 one level up. Before this change there were two behaviours across
five files — one transcription and four ties to zero — and the difference between them was the whole
of ADR-0060's open item.

Two alternatives were declined:

- **Leaving the four tied low**, on ADR-0060's own reasoning: the tie is an assumption, an
  assumption can only make checks easier, and this one was written to be temporary.
- **Moving the transcription into `rtl/` so there is literally one implementation.** ADR-0059
  deliberately put the arbitration *inside* `rtl/imemory.v`, and pulling it out into a module the
  harnesses could share would put a module boundary on the loop `make soc-timing` reports as the
  critical path. No timing was measured for this change, so that is a reason to leave the shipping
  RTL alone rather than a number. A transcription that says it is one is the cheaper honesty.

## Decision 2 — the wrapper's exception stays the weakest form, and the loop case was the risk

The same-address stability assumption lapses on exactly two cycles: the one whose fetch window a
data access took, and — after a text *store* — the one after that. Both are dropped compares rather
than modelled values.

The brief warned that a program storing to text in a loop can re-open the exception indefinitely and
starve `hang` and `liveness_ch0`, which is the failure ADR-0042 found when `imem_data` was left
free. **It does not: both PASS, at `PASS 0 3` and `PASS 0 6`.**

The reason is that the exception is bounded by the core's own bus rather than by the environment.
Re-opening it needs a store to text, a store needs an instruction that issued, and that instruction
was fetched through a window the assumption held still for. The environment cannot hold the
exception open on its own, which is exactly the property a free `fetch_stall` would have destroyed.

## Decision 3 — `imemcheck` is this repo's own check, and it stops at the first store

`rvfi_imem_check` is dropped. `formal/imemcheck.sv` carries a shadow halfword of its own: the
address is `rand_const` as before, the contents start free, and a store inside the text range that
covers it moves the shadow. The fetch ports are assumed to answer with what the shadow holds, and
that assume is dropped on a stolen window — which is also the one cycle on which the shadow can be
ahead of the array, since a store lands on the same edge it steals.

The ticket's fallback — gating the upstream checker's `enable` low from the first store — was not
taken. `enable` is the only lever upstream offers and it is all-or-nothing for the rest of the
trace, and `formal/cover.sv` and `formal/complete.sv` are already bespoke, so this is not a new
precedent. Writing the check here is what makes the drop below expressible at all.

**The retire assertion stops at the first store to the watched halfword and does not resume.** That
is not a convenience. An instruction fetched before the store can retire after it and correctly
report the old encoding — that window is exactly what `fence.i` closes, and the brief says such
instructions may legally see either value. An assertion against the post-store shadow would fail
correct hardware.

### What the check no longer sees

**Any retire at the watched halfword after something stored to it.** A core that fetched correctly
before a store and incorrectly after it is invisible here.

Two measurements say how much that is worth, and they point in opposite directions, so both belong:

- **The gate is reachable, and cheap for the solver.** Replacing the assertion with
  `assert(!shadow_stored)` gives a counterexample at step 2 — the first instruction can be a store
  to the watched word. On an adversarial trace the assertion can go quiet almost immediately, so
  this is a real narrowing rather than a dead branch.
- **The class the check was written for is still caught.** With `{imem_data2, imem_data}` changed to
  `{imem_data, imem_data}` in `rtl/fetcher.v` — the second fetch port disagreeing with the first,
  which is what this task exists to catch — `imemcheck` reports a counterexample at step 3 on the
  high-halfword arm. A counterexample survives on store-free traces, and the solver finds it.

The honest summary: the property this task already had is unweakened, and the property writable text
creates — that a store becomes visible to a later fetch — is asserted **nowhere** on this ladder. It
is an assumption here, because this task models no memory: the fetch data is a free input the shadow
constrains. Closing it needs `selfmod.S` and `textload.S`, which are the brief's step 5.

## Decision 4 — `dmemcheck`, `cover` and `complete` take the live arbiter and are unaffected

Verified rather than assumed, each re-run on this tree. None of the three reads a fetch port's value
against a memory model, so a steal costs stall cycles and nothing else. `cover` still reaches all
five goals and `complete_cover` all twelve, so the added stall did not push either past its bound.

## The measurements

Machine: 10 cores, two sibling agents live, load 5–49 across the runs. `JOBS=4` throughout, so the
wall times below are not comparable with a quiet box.

| gate | result |
|---|---|
| `make -C formal check JOBS=4` | **85 checks, 85 pass**, 10m25s; both set equalities exact in both directions |
| `formal/EXPECTED_FAIL` | empty |
| `make -C formal imemcheck` | PASS, 65s |
| `make -C formal dmemcheck` | PASS, 24s |
| `make -C formal cover` | PASS, 5s, all five goals |
| `make -C formal complete` | PASS |
| `make -C formal complete_cover` | PASS, twelve goals, 13s |
| `make -C formal nonperturbation` | PASS, 9s |
| `make test` | 56/56, baseline exact |
| `make test-units` | eight benches |
| `make lint` | clean in both passes |

**No `[depth]` line moved.** F and G were re-measured in ADR-0060 against the shipping RTL, and
nothing here adds a stall reason, lengthens a stage or widens the scoreboard. What this change moves
is the environment the four hand-authored tasks run in, and none of those reads `checks.cfg`.

### Both liveness probes, re-run under the new configuration

A weakened assumption that also stopped the ladder catching things is the failure this change is
most able to cause, so the two probes `formal/checks.cfg` names are re-run rather than quoted.

| probe | verdict |
|---|---|
| delete `rtl/executor.v`'s `rs2[4:0]` shift masking | `insn_sll_ch0`, `insn_srl_ch0`, `insn_sra_ch0` each `bad state property 9 reachable at bound k = 19 SATISFIABLE` |
| delete `rtl/regfile.v`'s rs2 write-through bypass | `reg_ch0` `bad state property 1 reachable at bound k = 22 SATISFIABLE` |

**A red check reports `ERROR, rc=16` rather than `FAIL` on a box with no `btorsim`.** The engine
finds the counterexample and sby then fails writing the VCD (`btorsim: command not found`).
`check-baseline.sh` counts anything that is not PASS, so the ladder still grades it red — but
reading the summary line alone would say "the check errored" where it means "the check found a
counterexample". Read the engine's own `reachable at bound` line before believing a rc=16.

## Consequences

- **`fetch_stall` is driven from one equation in five harnesses.** A change to `rtl/imemory.v`'s
  arbitration is a change to `formal/arbiter.v` and to nothing else under `formal/`.
- **`imemcheck` no longer reads any riscv-formal check model.** It is the third bespoke task here,
  after `cover` and `complete`. A pin bump can no longer change what it asserts, which cuts both
  ways: an upstream improvement to `rvfi_imem_check` will not arrive here either.
- **The ladder's wall time did not move for this change.** The doubling ADR-0060 recorded came from
  its three depth lines, and the `formal` job's margin against `timeout-minutes: 20` is what it was.
- **The suite still does not exercise any of this end to end.** No program in `test/asm` does a
  text-region data access, so `make test` is a control here rather than evidence.
- **The `text_write` output is unread by three of the five harnesses**, connected empty at those
  sites. It exists because the wrapper's second exception and `imemcheck`'s shadow both need the
  store's own cycle, and deriving it at those two sites would put the 8 KB bound in three places.
