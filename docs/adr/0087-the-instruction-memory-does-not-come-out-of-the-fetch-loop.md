# 0087 — The instruction memory half comes out of the fetch loop, and depth does not pay for it

Status: Accepted

## Context

[ADR-0086](0086-both-cores-in-one-harness-and-the-gap-is-the-fetch-loop.md) put both cores in one
harness and measured the gap at 1.59× rather than 3×, then decomposed it: 22 logic levels against
17, and 1.43 ns per level against 1.17. It also named a mechanism nothing here had noticed.
**Our fetch loop contains the instruction memory at both ends** — it leaves a block RAM's data
output, crosses decode, reaches `next_pc`, and closes inside the same block RAM's address decode,
because the fetch address is published a cycle early. Theirs starts and ends at ordinary flip-flops.

That left two questions in cost order: whether the instruction memory can come out of the loop, and
what registered boundaries buy against what the bubbles cost. They turn out to be one question.

## Decision

**The tail of the loop comes out for 3–4 levels; the head does not come out at all. On the board it
buys nothing, on hx8k it buys 5–7% of period, and one fetch stage costs more cycles than either
buys. Take the three levers already priced instead.**

Everything below is a spike: `soc/depth/` places and times a memory that is functionally wrong on
purpose, nothing in `rtl/` changed, and no gate, ratchet or commitment moved.

## Where the 22 levels sit

`soc/timing_split.py` names the two ends of the path and counts the levels between them. It cannot
say where the middle sits, and the report's own net names must not be read for it: yosys names a
generated cell after a neighbouring net, so `imem.in_range2_SB_LUT4_I3_O_...` is ancestry in
whichever direction the namer walked, and it appears on levels that have nothing to do with
`in_range2`. Reading those prefixes as a per-stage split produces something that looks like a
measurement.

What is derivable exactly: registers and memories keep their RTL names through synthesis even though
the combinational wires between them do not, so walking back from each level's **side** inputs —
every input except the one the path arrives on — to the nearest register says what that level folds
in. `soc/depth/path_stages.py` does that walk against the synthesised JSON.

The shipping up5k SoC, 82.00 ns over 21 LUT levels and 1 carry hop, in loop order:

| levels | where on the loop | what those levels fold in |
|---|---|---|
| 7 | the memory's read-out | eight banks of `SB_RAM40_4K` read data, the parity window mux, `in_range`/`in_range2` |
| 6 | decode and the stall reasons | `accessor.in`, `accessor.pending_rd` (the scoreboard), `prev_rs1` (the operand-fetch cycle), `fetch_stall` |
| 5 | `next_pc` | `mtvec`, `mepc`, the pc adder — the one carry hop is here |
| 4 | the memory's address decode | `text_access` off `accessor.in`, the new `pc`, closing at the ROM's depth-select register |

By module rather than by position that is decode 10, `imem` 5, access 3, `csrs` 2, and 2 levels that
fold in nothing new and are reducing fan-in on what the path already carries. The hx8k geometry —
ADR-0086's 22 levels — splits decode 9, `imem` 7, access 1, `csrs` 1, unattributed 4. Across
placements of one netlist the `imem` charge moves by two, so read the split at that resolution and
not finer.

**Eleven of the twenty-two levels are the instruction memory's own read-out and address decode.**
That is the case for getting it out. Half of it transfers.

## The two candidates are one candidate, and the depth curve is the same experiment

A synchronous memory leaves a combinational loop only behind a register — there is no other
mechanism, because the read is synchronous and its output either feeds combinational logic in the
same cycle or does not. So there are exactly two places a register can go, and each takes the memory
out of one end:

- **`addr`** — a register between `imem_addr_next` and the ROM's address decode. The loop closes at
  a flip-flop; the memory is off the tail.
- **`data`** — a register between the ROM's read data and `imem_data`. The loop starts at a
  flip-flop; the memory is off the head.
- **`both`** — a loop that touches no memory at all, which is the shape VexRiscv has.

And a register in the fetch loop is a fetch stage. So "get the memory out of the loop" and "price the
depth curve" are not two questions with an order between them: they are one experiment with three
points past today's zero. `soc/depth/variants.py` writes the memory by editing `rtl/imemory.v` at
three checked anchors rather than copying it, so the `base` control cannot drift from the shipping
file.

## The measurement

Four seeds a variant, two parts, `soc/depth/sweep.sh`, summarised by `soc/depth/summary.py`. up5k is
the board and carries `SOC_MIN_MHZ`; hx8k is `soc/compare/`'s geometry, where a level count is
comparable with VexRiscv's 17. The two designs differ — 8 KB of ROM, 64 KB of SPRAM and a timer
against 4 KB, 2 KB of block RAM and none — so their nanoseconds are never merged.

**up5k, the board:**

| variant | worst ns | MHz | best ns | LUT levels | carry | LC | critical path of the worst |
|---|---|---|---|---|---|---|---|
| base | 80.96 | 12.35 | 76.60 | 22–23 | 0–3 | 4790 | ROM read data → the ROM's address decode |
| `addr` | 79.92 | 12.51 | 77.36 | 22–23 | 0 | 4681 | ROM read data → `csrs.mcause` |
| `data` | 86.45 | 11.57 | 82.18 | 25–26 | 2–3 | 4705 | `imem_data` → the ROM's address decode |
| `both` | 85.45 | 11.70 | 78.28 | 25 | 0 | 4684 | the timer's `mtip` → `minstret` |

**hx8k, the comparison geometry:**

| variant | worst ns | MHz | best ns | LUT levels | carry | LC | critical path of the worst |
|---|---|---|---|---|---|---|---|
| base | 31.02 | 32.24 | 30.19 | 21–23 | 0–1 | 7415 | ROM read data → the ROM's address decode |
| `addr` | 29.44 | 33.97 | 27.57 | **18–19** | 0 | 7336 | ROM read data → `csrs.minstret` |
| `data` | 29.60 | 33.78 | 28.93 | 24–25 | 0 | 7337 | `pc` → the ROM's address decode |
| `both` | 29.92 | 33.42 | 27.69 | 23–25 | 0 | 7277 | `accessor_out.valid` → `csrs.minstret` |

Against base, read against the bands — ~3.6% edit churn, 1–2% placement spread:

| | up5k worst | up5k median | hx8k worst | hx8k median |
|---|---|---|---|---|
| `addr` | −1.3% | +1.2% | **−5.1%** | **−7.2%** |
| `data` | +6.8% | +6.6% | −4.6% | −4.5% |
| `both` | +5.5% | +1.6% | −3.5% | −4.0% |

The control holds: `base` is the generated memory with both registers unread and deleted, and it
places at 4790 logic cells against the shipping SoC's 4769 — 21 apart, inside the ±50 churn band —
with the same critical path endpoints. On hx8k it reproduces ADR-0086 to within the placement
spread: 30.19–31.02 ns over 21–23 levels at 7415 cells, against 30.56–31.70 ns over 22 levels at
7400.

## What the two ends are worth, and why they differ

**The tail comes out.** `addr` removes the ROM's address decode from the loop and, on hx8k, removes
3–4 levels with it, at every seed — a level count is a property of the mapped netlist and does not
move with placement, so that is not churn. On up5k it removes the same logic and the period does not
move, because a second path of the same length was waiting behind it: the critical path relocates to
`ROM read data → decode → csrs.mcause` and is just as long. The board's SoC has a timer and a 64 KB
SPRAM the comparison geometry does not, and more paths at the top of the distribution is what that
buys.

The one other structural change `addr` makes is that the pc adder's carry hops leave the critical
path entirely — 0 at every seed on both parts, against 0–3 at base — and that buys nothing, which is
the carry hop being worth ~0.34 ns against a LUT level's ~3.3 ns all over again.

**The head does not come out.** `data` puts a register between the memory and decode and the level
count *rises* by three to four, at every seed on both parts. The eleven levels attributed to the
memory are not eleven levels of memory: on a four-input LUT fabric
`in_range ? (odd_first ? odd_data : even_data) : 0` is four inputs and maps to **one** `SB_LUT4`, and
ABC is free to fold that LUT together with the first layer of decode reading it — the output-mux LUTs
have spare inputs and decode has terms to put in them. A register there does not delete those levels;
it **forbids the sharing**, and decode is then mapped on its own from a flip-flop boundary. The
mechanism is offered as the reading; the level counts stand without it.

That `data` still gains 4.6% on hx8k and loses 6.8% on up5k is the same fixed cost read against two
different periods. `SB_RAM40_4K`'s clock-to-output is 2.246 ns on hx8k — 7.2% of a 31 ns period, and
worth more than the levels it costs — and 1.279 ns on up5k, 1.6% of 82 ns, and worth less.

**So the best case is `both` being neither.** With the memory out of the loop entirely the loop is no
longer the critical path — on up5k it runs from the timer's `mtip` line to `minstret` — and the
design is not faster on the board and is 3.5% faster on hx8k, less than `addr` alone. The 1.29× of
level count ADR-0086 measured is not collectable this way: the best level count reached here is
18–19 against VexRiscv's 17, at 29.44 ns worst against their 19.91.

## What a fetch stage costs in cycles

`soc/depth/cycles.py` counts **redirects**: cycles the decoder issues an instruction whose `next_pc`
is not `pc + pc_inc`, which is the five arms of the next-PC chain below the stall arm. Nothing here
had counted them. It patches the shipping runner at three checked anchors rather than forking it.

| workload | cycles | issues | CPI | redirects | of issues |
|---|---|---|---|---|---|
| the `.S` and `.c` suite | 32 624 | 15 851 | 2.058 | 1 133 | 7.15% |
| Dhrystone, 2000 runs | 2 205 494 | 951 448 | 2.318 | 160 952 | 16.92% |

**The two workloads disagree the other way round from the stall accounting.** ADR-0084 recorded the
suite's RAW share at 29.3% against Dhrystone's 16.0%; the redirect share goes 7.15% against 16.92%.
The suite overstates what a hazard costs and understates what a redirect costs, so a depth proposal
graded on the suite alone is graded on the workload that flatters it.

What a fetch stage costs depends entirely on what refills it, and the two policies are an order of
magnitude apart:

| | +1 stage | +2 stages |
|---|---|---|
| **one bubble per issue** — the next address is known only after decode, so every instruction waits | suite ×1.486, Dhrystone ×1.431 | ×1.972, ×1.863 |
| **one bubble per redirect** — fetch runs ahead sequentially and something discards the wrong guess | suite ×1.035, Dhrystone ×1.073 | ×1.069, ×1.146 |

The per-redirect row is an upper bound: a redirect landing in a cycle already stalled for a divide, a
load turnaround or a serialization refills for free, and this model charges it anyway. The
per-issue row is the one that needs no new mechanism, and it is the honest price of a deeper fetch
that does not run ahead.

## The product, which is the answer

Clock against cycles, on the refill policy that needs no amendment and on the one that does:

| | one bubble per issue | one bubble per redirect |
|---|---|---|
| up5k `addr`, worst / median | ×0.708 / ×0.690 | ×0.944 / ×0.921 |
| up5k `both`, worst / median | ×0.509 / ×0.529 | ×0.827 / ×0.859 |
| hx8k `addr`, worst / median | ×0.736 / ×0.753 | ×0.982 / **×1.004** |
| hx8k `both`, worst / median | ×0.557 / ×0.559 | ×0.905 / ×0.909 |

Dhrystone's redirect rate, because it is the workload that does not flatter the answer; on the suite
the same best cell reads ×1.041.

**Every cell is a loss except one, and that one is a rounding error.** The best result across two
parts, two ends of the placement distribution, three depths and both workloads is +0.4% — on the part
this project does not ship, at the optimistic end of its spread, under the refill policy that
requires amending a design commitment. On the board it is −5.6% to −7.9% under that policy and −29%
to −49% without it.

## The discarded fetch buffer

The boundary case: a fetch buffer holding a word fetched ahead of a redirect and discarded before its
valid bit is ever set. Strictly a kill signal; substantively nothing was committed.

**It is on the allowed side of the commitment's first clause and the forbidden side of its second,
and the second is the one that is enforced.**

Nothing architectural has to be un-committed. No register write, no CSR write, no store, no retire —
the buffer holds a copy of memory that was read and not used, so discarding it drops a value rather
than reversing a decision, which is what a bubble already does to the fetch window of a stalled
cycle. On "no state a later cycle must un-commit", a fetch buffer passes.

The commitment's second clause is a mechanism ban — "no flush logic, no kill signal" — and the
discard is literally a kill signal, one bit wide. That clause is not decoration: it is what
`formal/pcloop.sv`, `rtl/decoder.v`'s `FORMAL` block and `test/decoder_tb.v` actually assert.
`pc == $past(next_pc)`, and `past_pc + 4 == pc` unless the previous cycle branched. A prefetch buffer
means the memory is answering an address that is not `pc`, and those assertions are false as written.
This repository's operative definition of "kill signal" is those assertions, and a discarded fetch
buffer fails them. **Adopting one is an amendment, not a tuning change.**

The useful half of the ruling is what the amendment would cost, because it is much less than a real
flush. ADR-0047 retired `formal/equiv.sh` after it diverged at ~660k clauses per step and never
returned a verdict; that was sequential equivalence over the whole core. This is one bit of state and
one strengthening invariant relating buffer validity to `pc`, with no architectural state to reverse,
so `pcloop`'s induction is re-provable rather than abandoned. **Cheap to amend, and not worth
amending on these numbers** — the amendment's whole yield is the +0.4% cell above.

## What re-measuring F and G would cost

Not re-derived here; this is the exposure. Every generated riscv-formal check is `mode bmc` at a
depth built from two measured numbers: F = 6, the worst-case first retire, and G = 6, the worst-case
retire gap. The table is checked against the single hop F + G = 12 and the two hops F + 2G = 18, and
`insn 19` and `reg 15 22` clear 18 **by one cycle** — the thinnest margin in the file.

A fetch stage lengthens the stage that produces a retire, so it moves both numbers rather than one: F
grows by the stage count and so does the worst-case G. F + 2G therefore grows by three cycles per
stage, and the two checks with a one-cycle margin are under water immediately. A single added fetch
stage means:

- re-measuring both flip points by the procedure in `formal/checks.cfg`'s `[depth]` — copy the file
  with one line in the table, sweep, twice;
- moving every line of that table, not the two that trip, because they are all derived from the same
  two numbers;
- re-running all 85 generated checks at the new depths, against the `formal` job's 20-minute wall,
  with BMC cost growing faster than the depth does;
- and, for the prefetch variant, rewriting the `pcloop` and decoder pc assertions the ruling above
  names, which are the things that would otherwise fail.

None of that is speculative work — ADR-0046 already requires it of any change that adds a stall
reason or lengthens a stage — but it is the invoice, and it is attached to a change measured to buy
0.4% at best.

## The ranking

Against the levers already priced, on their own measured numbers:

| lever | what it buys | status |
|---|---|---|
| everything off `next_pc` together (ADR-0076) | 21% of period | ceiling; collecting it means the pc stops depending on this cycle's decode |
| the cheaper operand-fetch spelling | net 9%, holds 12 MHz on 0.83% margin | declined on the margin |
| executor-only forwarding (ADR-0083) | net 4.5%, holds 12 MHz on 0.48% margin | declined on the margin |
| **the memory out of the fetch loop, this ADR** | **+0.4% at its single best cell, −5.6% on the board** | **declined on the measurement** |

It ranks below all three, and below doing nothing. ADR-0086's finding survives — the critical path is
the fetch loop on both sides — but the part of it that is the memory is not the part that costs
nanoseconds, and the 1.29× of level count stays where ADR-0076 left it: in decode and in `next_pc`,
where no single input is worth more than 5% and all of them together are worth 21%.

## Consequences

- **Question 1 is answered by halves, and question 2 does not need asking.** The tail of the loop
  comes out for 3–4 levels and the head does not come out at all, and a register in either place is a
  fetch stage whose cycles cost more than the levels buy. The depth curve was priced anyway because
  it is the same experiment, and it is flat to negative at every point.
- **A negative result with a mechanism.** A synchronous memory's read-out is nearly free at a
  combinational boundary on a four-input LUT fabric, because its output mux shares LUTs with the
  logic reading it. Registering that boundary does not remove levels; it forbids the sharing and adds
  them. Anyone reading ADR-0086's "the memory is at both ends of it" as an 11-level target should read
  this ADR first.
- **The two parts disagree, and both readings are real.** The same edit is worth −7.2% on hx8k and
  +1.2% on up5k, because the board's SoC has more paths at the top of its distribution and because a
  block RAM's fixed clock-to-output is 7.2% of one period and 1.6% of the other. A single-part
  measurement of this would have been quotable and wrong in either direction.
- **Redirect density is now measured, and the suite is the wrong workload for it.** 7.15% of issues
  on the `.S` suite against 16.92% on Dhrystone — the opposite ordering from the RAW share, so
  neither workload can carry a depth argument alone.
- **The discarded fetch buffer is ruled forbidden as written and cheap to amend**, and the ruling is
  recorded now so the next proposal starts from it rather than re-litigating it. Nothing about the
  no-wrong-path-state commitment changed.
- **`soc/depth/` is a spike with no gate.** Nothing in it grades the shipping design and nothing in
  it runs on `make test`. Its two rot-prone pieces do not copy what they modify: the memory is
  generated from `rtl/imemory.v` and the redirect counter is patched into `test/cxxrtl.cc`, both at
  anchors that stop the script when they move. `soc/depth/path_stages.py` outlives the spike — it is
  the only thing here that answers "where do the levels sit", which every Fmax proposal so far has
  had to guess at.
- **`soc/timing_split.py` now has one walk and two readers.** The split was factored into
  `summarise()` so `soc/depth/row.py` reuses it; a second parser of a file that one already
  reconciles is the shape that once left a ratchet in the copy nobody maintained. Its printed output
  and its `--min-mhz` grading are unchanged, and its probes still cover both.
