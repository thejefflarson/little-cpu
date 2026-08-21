# ADR-0123: The bus arbiter is two bits, and its wait bound is conditional on an assumption nothing discharges yet

**Status:** Accepted · 2026-08-18 · *Builds `rtl/busarbiter.v` and proves it — grant `$onehot0`, a
lock indivisible, a lock worth exactly one extra cycle, and a stated wait bound per hart, by
k-induction over the module's whole reachable state. Nothing instantiates it, so no `fit` and no
`soc-timing` number moves and none is quoted. The wait bound is proved under an environment
assumption about `mem_lock` that no property in this tree discharges; the core surface that will owe
it is not built.*

## Context

The dual-hart SoC needs three things of one shared data bus that the single-hart design has never
had to state: at most one driver per cycle, an AMO's read and write cycles indivisible, and no
requester waiting forever. The first two are what a bus arbiter is; the third is the one no program
can test. A fixed-priority arbiter passes every torture program ever written for it and starves the
losing hart anyway, and no amount of `.S` suite catches that — which is why this module ships with
its proof rather than after it.

## Decision 1 — the grant is the round-robin pointer, so there are two bits of state and no hidden ones

A round-robin arbiter is usually a grant register plus a pointer saying whose turn is next. Here the
pointer is redundant: the tie goes to whichever hart did not hold the bus last cycle, which `grant`
already says. So the whole module is

```systemverilog
assign winner = (request == 2'b11) ? (grant[0] ? 2'b10 : 2'b01) : request;
assign held   = |(grant & mem_lock);
always_ff @(posedge clk) grant <= reset ? 2'b00 : (held ? grant : winner);
```

two flops and one mux, and **every bit of its state is on a port**. That is not only smaller — it is
what lets the proof's strengthening invariants be written over the module's outputs. A pointer
register would be state the harness cannot see, `yosys` does not let a harness reach into an
instance for it (`formal/pcloop.sv` records what happens when one tries), and the invariants would
have to be split into the RTL's own `FORMAL` block or the pointer exposed as a port nothing else
reads.

**The price is an asymmetric bound, and it is priced rather than hidden.** An idle bus reads `2'b00`
and `2'b00`'s tie goes to hart 0, so hart 1 can be made to lose one tie hart 0 cannot: hart 0's worst
wait is **2** cycles and hart 1's is **3**. Both are bounded, which is the property; each hart's
harness carries its own constant, and each constant is covered as well as asserted, so a bound
loosened by a later edit goes red as an unreachable cover goal rather than passing wide.

## Decision 2 — `mem_lock` is read on the granted cycle, and it means "I need the next cycle too"

The grant is registered, which the design brief settled for a reason this module cannot revisit: a
combinational grant forces a hart to *hold* a decoded instruction rather than bubble, and holding
one needs a freeze input on `rtl/executor.v`, which is the named oracle for real mul/div arithmetic.
Registered means the arbiter must be told about an AMO's second cycle **before** that cycle. So
`mem_lock[i]` is read on a cycle hart `i` holds the grant and asks to keep it one cycle longer; it is
the AMO's issue cycle that raises it, not the write cycle that follows.

That is an interface contract, and the core-surface work owes it exactly: `mem_lock` high on the
cycle an AMO issues, low on the write cycle, never two cycles running.

## Decision 3 — the wait bound is a safety property with a constant, proved under an assumption nothing discharges yet

`formal/busarbiter.sv` counts, per hart, the cycles it has asked for the bus without getting it, and
asserts that count never passes the hart's bound. A safety property with a number holds at every
depth; a liveness one would hold "eventually" and say nothing about a board. Two invariants make it
inductive — the other hart must be holding a lock, and that lock must have run out — and both are
measured rather than believed: at `depth 2` the proof closes with them and returns UNKNOWN without
either. The task runs at sby's default depth, which reaches the same verdict by unrolling, so what
the invariants buy is the argument written down and a proof whose depth does not grow with it.

**The bound is false without an assumption: no hart raises `mem_lock` on two cycles running.** A lock
held forever holds the bus forever and no arbiter can take it back. **Nothing in this tree discharges
that assumption today** — the core publishes no `mem_lock` at all, and the duration property that
will owe it lands with that output. Until then every PASS on the wait bound reads as "granted within
the bound, *provided* the core really does lock for one cycle", and the assumption says so where it
is assumed. An assumption nothing discharges is a hole, not a proof, and naming the hole is the
whole of the honesty here.

Bounding the lock inside the arbiter instead — a bit refusing a second extension — would delete the
assumption and is declined. It would cut an AMO's bus tenure short rather than report anything,
turning a fairness bug this proof catches into a torn atomic nothing in the machine records.

## Decision 4 — the control and the two red directions are prerequisites of the proof, not neighbours of it

Every assertion in the harness is guarded, and an arbiter that grants nobody satisfies all of them.
So `formal/busarbiter_cover.sby` asks the opposite question of the same file — both harts really get
granted, a lock really spans two cycles *while the other hart is asking*, and each hart's bound is
really reached — and `formal/busarbiter-probe.py` builds two arbiters one line from the shipping one
and requires each arm to go red as itself:

| core | proof | cover |
|---|---|---|
| shipping | PASS | PASS |
| tie always to hart 0 | FAIL, and the wait bound is among the red lines while the lock arm is not | PASS |
| grant re-arbitrated mid-lock | FAIL at the lock arm's line and nowhere else | FAIL, lock goal unreached |

The third row is the anti-vacuity control's own red direction: the one mutation that makes the cover
blind is the one that makes the lock arm false, so the control cannot quietly stop seeing the class
it exists for. Both are `make` prerequisites of `components_busarbiter` rather than targets beside
it, for the reason `pcloop_cover` and `traps-region-probe` are: a control that can be run separately
from the thing it controls eventually is not run at all.

The probe's own comparisons are graded in turn by `test/probe_gates.sh` against a stub solver — 17
probes, including a shipping arbiter that fails to prove, a mutation that proves, a mutation red at
the other arm's line, and each of the three pinned line parses respelled.

## What it cost, and what was not measured

`rtl/busarbiter.v` is in neither `FIT_SRCS` nor `SOC_SRCS` and nothing instantiates it, so neither
instrument reads it and **neither number can have moved**. That is a fact about the source lists, not
a measurement, and it is the reason no cell count or period appears above. The arbiter's real cost —
a grant in the dual top's fetch loop, or a registered one at a cycle per contended access — is the
dual top's measurement to take, on a part this repo does not yet place for.

The proof, its cover and the probe together run in about **four seconds** and add one step to the
`components` CI job.

## Consequences

- The core surface owes this module a `mem_lock` that is high on an AMO's issue cycle and never on
  two cycles running, and a property proving it. Until that property exists, the wait bound is
  conditional and is quoted that way.
- The bound is per hart — 2 and 3 — and both numbers are asserted and covered. Changing the
  tie-break for an idle bus changes them; the covers are what make that loud.
- `grant` is `$onehot0` and not `$onehot`: the arbiter parks on nobody, so an idle bus has no
  driver and a hart pays one cycle to be handed a bus nobody was using. Parking on the last owner
  would buy that cycle back and is deliberately not decided here — it is a CPI question, and CPI is
  measurable only in the dual top.
- Nothing in `rtl/` instantiates the module yet. It is elaborated by yosys under the proof and
  parsed by svlint under `make lint`; the iverilog leg does not see it until something does.
