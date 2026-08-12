# ADR-0105: The `fence.i` wait is unobservable in the suite, and the memory's port arbitration is why

**Status:** Accepted · 2026-08-11 · *Records what
[ADR-0099](0099-the-memory-transaction-launches-from-the-execute-slot.md) cost the behavioural leg,
and narrows what a green `.S` run says about design commitment 5. It does not amend the commitment:
the wait stays.*

## Context

`test/asm/selfmod.S` carried a written claim that its test 2 is the case `fence.i`'s wait has to
cover. It is not, and has not been since the merge before last. Delete `instr_fencei` from
`serialize` in `rtl/decoder.v` and the whole suite is green:

| tree | `selfmod.S` under the deletion | suite |
|---|---|---|
| `b20c33c` — before the execute-slot launch | **FAIL 2**, retires 17 | 61/62 |
| `3d183fe` — after it | **PASS**, retires 47, spec-checked 44 | 62/62, failure list matches the baseline |
| `d749a74` — the tree this merges onto | **PASS**, retires 47, spec-checked 44 | 63/63, failure list matches the baseline |

The same deletion is loud on the microscope leg either way: `test/decoder_tb.v` reports five
mismatches — "fence.i serializes while the pipe is busy", "...and that is a stall", "...so nothing
issues", "...and it bubbles decoder_out rather than holding it", "the drained pipe releases it".

### How wide the window actually is

Four probe programs, `gap0..gap3`: a word stored into `.text` at a site *k* instructions further on,
**no `fence.i` anywhere**, and a check of whether the stored instruction or the one it replaced ran.

| tree | gap 0 | gap 1 | gap 2 | gap 3 |
|---|---|---|---|---|
| `b20c33c` | old word | **old word** | new word | new word |
| `3d183fe` | old word | new word | new word | new word |
| `d749a74` | old word | new word | new word | new word |

Cycle by cycle, with *S* the cycle decode publishes the store. The executor takes it in *S+1* and
`rtl/accessor.v` drives the bus from `decoder_out` that same cycle, so the write edge ends *S+1*.
`rtl/imemory.v` gives fetch and data one read port: the write raises `text_access` in *S+1*, so the
fetch published that cycle is thrown away and `fetch_stall` holds decode through *S+2*, which
re-presents the same address and reads it again after the write.

An instruction executing in cycle *C* was read on the edge ending *C−1*, so it runs the pre-write
word only if *C−1 ≤ S+1*. *C = S+2* is exactly the fetch the write's own port steal discards. That
leaves *C = S+1* — the instruction immediately behind the store — and nothing else.

**A `fence.i` cannot occupy a slot it has to follow.** The earliest cycle it can execute is *S+1*,
so the instruction it is covering executes at *S+2* or later, which the port steal already holds
back. Whatever the wait does, it moves a fetch that was after the write already.

Before the execute-slot launch the write edge was a cycle later, at the end of *S+2*, and the steal
was in *S+2* covering *C = S+3*. Two slots were exposed, a `fence.i` in the first left the patched
word in the second, and the deletion was caught. That slot is what stopped existing.

### The note had already rotted once, for a different reason

`selfmod.S` said its test 2 store must use `x0` as its base, or "the test passes with the wait taken
out of the decoder". Rewritten as `la t2, patch_adjacent; sw t1, 0(t2)` and measured under the same
deletion, test 2 still fails on `b20c33c` (FAIL 2, retires 19) and still fails on `3d183fe` under
the pair of deletions below (FAIL 2, retires 19). Decode takes the next instruction's register pair
out of the fetch window on a cycle that issues, so the fence's `rs1 = 0` is presented behind any
store, whatever that store's own base register is. The stated reason had stopped being the reason
before its subject disappeared.

## Decision

**1. The wait is what makes the core correct independent of the memory, and it stays for that
reason.** The observable ordering today comes from port arbitration: a text write takes the fetch
port for its own cycle, and that is a property of the memory this core is wired to, not of the core.
`rtl/littlecpu.v` already states the other case — a memory that keeps its two buses separate ties
`fetch_stall` low, and then nothing but `serialize` orders the store against the fetch. Deleting the
term would leave the core correct **only on the memory it happens to ship with**.

This is the `fence.i` lesson this repo has already learned once. "Correct and for free" was a
property of the configuration, and it stopped being true when the configuration moved. Read the
measurement above as that same lesson arriving from the other side: the configuration moved again,
and this time it moved *towards* correctness and took the oracle with it.

**2. `selfmod.S` grades the two mechanisms as a disjunction, and its note now says so.** One mutation
at a time on `d749a74`, reproducing what `3d183fe` read:

| mutation | `selfmod.S` |
|---|---|
| `serialize` loses `instr_fencei` | PASS |
| `text_access` loses the store (`mem_ren && text_range`) | PASS |
| both | **FAIL 2**, retires 17 |

So the program is a live grader for the pair and for neither term alone. The note that claimed
otherwise is replaced by the table above, stated as mechanism.

**3. The suite is given no red direction for the serialize term, because there is none to give.**
The argument above is structural, not a failure to find one. `test/decoder_tb.v` is the grader for
that term, and it fails for the reason it was written.

**4. Declined: pinning the one-slot behaviour.** A program with no `fence.i` that requires a store to
be visible two instructions later would be red under a single mutation — and it would write this
memory's arbitration into the suite as though it were architecture, and go red on the separate-bus
memory for a reason that is not a defect. The exposed slot stays a measurement in this file.

## Consequences

- A green `.S` run says text writes reach the instruction stream. It says nothing about *what*
  ordered them. Both of this repo's checks on `fence.i`'s wait — `test/decoder_tb.v` and the decoder
  proof — are outside the behavioural leg.
- The load half of the same shared port is still watched by four programs: with `text_access` reduced
  to the store side, `textload.S` (FAIL 3), `contend.S`, `datainit.c` and `selfmod.S` all go red,
  59/63. `textload.S` never had this hole — it writes no text and contains no `fence.i`.
- The retire counts in `test/OBSERVED_FLOOR` do not move; nothing in the suite changed.
- **This oracle was disarmed twice by merges that were green both times**, and neither merge was
  wrong: one made the base register irrelevant, the other closed the slot. A comment claiming a red
  direction is a claim with a date on it, and the only thing that keeps one true is forcing it.
- **For whoever builds the general mutation harness: pair the `fence.i` mutation against
  `test/decoder_tb.v` and against nothing under `test/asm/`.** That harness was scoped expecting a
  `.S` detector for this mutation. There is none, and the argument above says there cannot be one
  while the instruction memory shares a port — so a harness that pairs it against the suite will
  record a permanent unpaired mutation and read it as a hole in the programs rather than as a fact
  about the memory.
- If the behavioural leg is ever wanted for this property, it needs a memory model that answers fetch
  and data in the same cycle, wired into `test/testbench.v` as a second configuration. That is an RTL
  change and it is not made here.
