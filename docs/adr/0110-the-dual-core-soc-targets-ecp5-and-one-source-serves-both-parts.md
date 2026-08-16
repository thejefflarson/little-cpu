# ADR-0110: The dual-core SoC targets ECP5, and one source serves both parts

**Status:** Accepted · 2026-08-16 · *Settles the part question ADR-0106 left open when it justified
the A extension as multi-hart groundwork. Amends no commitment. Constrains every future
optimisation.*

## Context

ADR-0106 landed the A extension on a single hart, and its whole justification was that the
multi-core milestone should then verify exactly one new thing — cross-hart atomicity — rather than
bringing up a new instruction class, a memory arbiter and a second hart through one symptom stream.
It cost about 453 logic cells, eight to nine percent of the core, for instructions no measured
workload executes.

It did not say which part the second hart lands on, and the arithmetic had become hard to avoid.
`littlesoc` places at **4712 of the up5k's 5280 logic cells — 89% occupancy, 568 free.** A second
hart is most of a core again. It does not fit, and on this part nothing else will either.

The alternative was already measured. Earlier in this project the same RTL was placed on ECP5:
**35.33 MHz, 20% logic utilisation, 36 of 56 block RAMs**, with the register file mapping to 32
distributed RAMs and no block RAM at all.

## Decision 1 — the dual-core SoC targets ECP5

The second hart goes on ECP5. The up5k is not where multi-core happens.

This is a decision about deployment, not about the design: ADR-0106's rationale for landing A
single-hart is unchanged and does not depend on which part the second hart runs on.

## Decision 2 — one source serves both parts

**There is no fork.** No part-specific RTL, no `ifdef` split, no parameterised datapath. The same
source places and closes on both, and the single-core design stays a first-class up5k target.

Three consequences follow, and they bind future work harder than the first decision does.

**The up5k stays the binding constraint.** 89% and 12.12 MHz worst-seed against ECP5's 20% and
35.33 MHz. So every area and period argument is an up5k argument, and ECP5 is where a candidate must
**not regress** rather than where it must fit. That is much the cheaper test.

**A structural change needs a two-part measurement.** ADR-0087 measured the two parts disagreeing
in *sign* — registering the fetch loop's tail was −7.2% on hx8k and +1.2% on up5k. "Measured on
up5k" is therefore no longer sufficient evidence on its own for a structural edit. `soc/compare/`
already places on hx8k, so the machinery exists.

**The operand-fetch cycle is paid on both parts.** ECP5's distributed-RAM register file offers an
asynchronous read, and it is tempting to read that as deleting the operand-fetch stall — 2.7% of
suite cycles but **18.4% of Dhrystone's**, the largest column after hazard. It does not.
`rtl/regfile.v`'s read is a clocked assignment and decode's present-bubble-issue protocol is written
into `rtl/decoder.v`; it is commitment 6, not a property of the memory. **Technology mapping cannot
delete a cycle the RTL takes.** Exploiting the asynchronous read would mean changing the read
protocol, which is a commitment amendment plus exactly the fork decision 2 forbids.

## What was considered and rejected

**Making multi-core fit on up5k.** A full census puts the never-interrogated blocks at roughly 1078
logic cells and the realistic yield of asking all of them the fact-from-outside-the-expression
question at −150 to −400 placed cells. That is an order of magnitude short of a second hart, and it
would spend the entire remaining area budget of the design on one milestone.

**Forking the source per part.** It would double the configuration space every verification leg
describes — which configuration do the proofs, the suite, the baselines and the depth table
describe? — for a benefit available more cheaply by letting up5k bind.

**Moving the project's home to ECP5 outright.** The up5k is the board this project has, the
constraint that has produced every measurement in `docs/adr/`, and the harder target. Keeping it
first-class keeps the design honest; a part with 3× the headroom would retire the discipline along
with the constraint.

## Consequences

- **Area work on up5k is judged on up5k's own terms.** It is no longer the gate on the multi-core
  milestone. What remains is `FIT_MAX_LC` hygiene against an instrument whose local-to-CI offset has
  no fixed size and no fixed sign, and whatever placement tail occupancy buys — which is measured,
  not assumed.
- **The two-part check is a standing cost on structural edits.** Cheap for constraints, which the
  other flow never sees; real for RTL.
- **The ECP5 port is not designed here.** Bring-up, its memory map, its clock, and what a second hart
  and an arbiter actually need are all open and belong to their own decisions.
- **`.aq` and `.rl` stay decoded and ignored only until a second agent touches memory.** ADR-0108
  names the invalidators; the second hart is the first of them, and the arbiter owes the core
  per-access completion in program order or that argument reopens as a design question.
