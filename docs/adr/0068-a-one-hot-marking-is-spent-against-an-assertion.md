# ADR-0068: A one-hot marking is spent against an assertion, not against inspection

**Status:** Accepted · 2026-08-02 · *Supplements
[ADR-0030](0030-trap-cause-priority-and-why-the-causes-are-disjoint.md), which is what declines one
of the two statements this audit was asked to mark.*

## Context

`(* parallel_case *)` tells synthesis that at most one arm of a `case (1'b1)` can match, turning a
priority chain roughly N deep into a flat mux roughly log N deep. On flags that really can overlap
it is a synthesis/simulation mismatch: the simulator takes the first matching arm and the gates take
whichever the optimiser preferred. `rtl/structs.v` already records one instance of that hazard —
three CSR flags were removed from `decoder_output` because they would have made two arms of the
executor's op select match at once alongside `is_add`.

The audit that produced this ADR started from a count: six `case (1'b1)` statements in
`rtl/decoder.v`, four marked and two not, with the two unmarked ones described as paying for a
priority the decoder's `$onehot` assertion proves cannot happen. Both halves of that count turned
out to be wrong, and the second half is the useful finding.

## What the audit found

**There are eight such statements, not six.** The count came from a grep for `case (1'b1)`; two of
them are written `case(1'b1)` with no space, and both of those already carry the attribute. Six of
the eight are marked.

**The two unmarked ones must stay unmarked**, and each already says so at the site.

- The `next_pc` chain selects on `reset`, `stall`, `trap_pending`, `instr_mret`, `instr_jalr`,
  `instr_jal` and `branch_taken`. Four of those are not instruction flags at all, and `stall` and
  `trap_pending` are both true on any cycle where a trapping instruction is held — which is a normal
  cycle, not a corner. Marking it would be the mismatch above, on the one signal that owns the PC.
- The `trap_cause` chain selects on `instr_illegal`, `instr_ebreak`, `instr_ecall`,
  `load_misaligned` and `store_misaligned`. Three of those five are outside the instruction one-hot
  set, and mutual exclusivity comes from a separate `$onehot0` assertion instead. That assertion
  would license the attribute — but ADR-0030 records the priority order deliberately, and its whole
  first reason is that someone reading the encoder concludes the order is dead and simplifies it
  away. Flattening it is that simplification. The five arms are constants feeding `mcause`, so there
  is nothing to win.

**The interesting half is the four that were already marked.** The immediate, `rd`, `rs1` and `rs2`
muxes select on opcode-group flags (`instr_load_op`, `instr_branch_op`) and on compressed flags
(`instr_clw`, `instr_cswsp`, `instr_candi`). The `$onehot` assertion in `rtl/decoder.v` covers 45
*architectural* flags — `instr_lw`, `instr_add`, `instr_jal` — and names none of those. So four
attributes were spent against reading the encodings and finding them disjoint, which is true and was
proved by nobody. A compressed flag that widened to overlap a sibling would keep every architectural
flag one-hot, so the existing assertion would stay green while two arms of a marked mux matched at
once.

## Decision

**Add the missing proof rather than the missing attribute.** `rtl/decoder.v`'s `FORMAL` block gains
five `$onehot0` assertions, one per marked statement whose arms the instruction check does not
reach — the four muxes above plus the operand overrides in the publish block. Each assertion is that
statement's arm list, so it holds exactly the condition the attribute claims.

Neither statement the finding named gets the attribute.

## Evidence

`make -C formal components_decoder` passes by k-induction with the five assertions in, 6s.

The red direction was measured. Widening `instr_clw` to match quadrant 2 as well as quadrant 0 makes
it overlap `instr_clwsp` — both feed `instr_lw`, so the architectural flags stay one-hot — and the
task fails on exactly two of the new assertions, the immediate mux and the `rs1` mux, and on nothing
else. Before this change that mutation passed.

**The change is not visible to synthesis, and that is proven rather than argued.** Neither
`make fit` nor `make soc-timing` defines `FORMAL`, and the two `fit.json` netlists differ in one
line: the module's recorded end line, `rtl/decoder.v:4.1-845.10` against `4.1-882.10`. No cell, net
or connection moves. `make fit` is 3880 of 4100 either way, and the four-placement sweep reproduces
the shipping numbers to the digit: 74.34 / 75.81 / 76.88 / 78.80 ns, 12.69 – 13.45 MHz, against a
`SOC_MIN_MHZ` of 10.9.

## Consequences

- The six marked statements in `rtl/decoder.v` are each licensed by an assertion the required
  `components` job proves. A future one has to bring its own.
- The rule generalises past this file: an optimisation attribute is a claim about the design, and
  the place to put a claim is next to the thing that checks it.
- Nothing here changes what the core computes, so there is no simulation-side test. The proof is the
  test, and the mutation above is how to re-run its red direction.
- **Each assertion transcribes its arm list rather than sharing it**, which is the known weakness:
  an arm added to a marked statement and not added here leaves that arm unlicensed, and the
  assertion stays green. Sharing the lists through named wires would close it and would move code
  that is `FORMAL`-only today into the synthesized part of the file, so the transcription is
  deliberate. The site says so.
- Two statements are now declined in three places at once — at the site, here, and in ADR-0030 for
  the trap causes. That is deliberate; the finding has been raised once already from a reading of
  the file alone.
