# ADR-0070: The suite's cycles are charged to a named stall reason

**Status:** Accepted · 2026-08-02 · *The CPI counterpart of `make fit`. Adds `make cycles`; no
`rtl/` file changes. Gives
[ADR-0042](0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md)'s accepted +18.0%
an invoice, and puts a number on the six stall reasons
[ADR-0026](0026-stalls-are-four-reasons-over-two-mechanisms.md) and
[ADR-0060](0060-the-steal-reaches-decode-as-the-sixth-stall-reason.md) enumerate.*

## Context

The `.S` suite's cycle count was the only CPI number this project had, and nothing decomposed it.
Every argument about where the cycles went was made from structure: the operand-fetch cycle "costs a
cycle per instruction", `fence.i` "drains the pipeline", the divider "is 33 cycles". None of those
was weighed against the others, because nothing weighed anything.

Area was in that position before `make fit`, and it produced four wrong estimates inside a week,
each corrected only by building the thing and looking. ADR-0042 then accepted a **measured +18.0%**
cycle cost for 2735 logic cells, deliberately and correctly — but with no way to ask which part of
the pipeline that 18% now sits in, or what it would take back.

## Decision

**`make cycles` runs the same 59 programs `make test` runs and charges every simulated cycle to
either an issuing cycle or one of the decoder's six stall reasons.** Three parts:

1. `test/cxxrtl.cc` gains `--stalls`. It reads `uut decoder stall` and the seven signals that make
   it up — `divider_stall`, `accessor_stall`, `hazard_rs1`, `hazard_rs2`, `serialize`,
   `operand_stall`, `fetch_stall` — through cxxrtl `debug_items`, once per cycle, and prints one
   `STALLS cycles=... issue=... unattributed=...` line.
2. `test/stall_report.py` turns those lines into a per-program and whole-suite table with CPI.
3. `STALL_REPORT=1 test/run_tests.sh` wires the two together; the Makefile calls that `make cycles`.

### The signals are read, never rebuilt

Every reason is the named wire `rtl/decoder.v` drives. Nothing in the runner re-derives
`hazard` from `uses_rs1` and the scoreboard slots, and nothing re-derives `stall` from the seven
terms. An attribution that drifts from the RTL is worse than no attribution: it would keep printing
a table that adds up while describing a pipeline that no longer exists. Reading `stall` itself
rather than OR-ing the seven probes is what makes the `unattributed` column mean something — see
below.

**No `rtl/` file changed, and none needed to.** The decoder's stall signals survive
`write_cxxrtl` as plain named debug items, so the `(* keep *)` this change was budgeted for was
never spent.

### A cycle goes to the first reason the decoder itself would try

Several reasons are true at once often — `operand_stall` is high on 31 of `selfmod.S`'s cycles and
is charged 12 of them — so the count needs an order. The order is `rtl/decoder.v`'s own: the publish
block holds `decoder_out` for the divider and the accessor before it bubbles for anything else, and
`stall` ORs the rest left to right.

    divider · accessor · hazard (rs1, rs2) · serialize · operand · fetch

`hazard_rs1` and `hazard_rs2` share a column because they are one reason, the decode scoreboard
finding a producer still in flight, and they are adjacent in the order, so merging them cannot move
a cycle past anything else.

**A column is therefore cycles CHARGED, not cycles the signal was high**, and the difference is
large where reasons coincide. `fetch_stall` is high on 26 cycles across the three writable-text
programs — 6 / 4 / 16, reproducing ADR-0064's independently taken count exactly — and is charged 8.
The report says so where the number is printed.

### It is a sibling target, not part of `make test`

`make test` is unchanged: same flags to the runner, same table, same set equality against
`test/EXPECTED_FAIL`, same exit status. Two reasons.

- A CPI figure is a measurement to compare against the last one, not a merge criterion. There is no
  CPI ratchet here and this ADR does not propose one; the number moves for legitimate reasons on
  every change to the suite.
- The counting costs a `debug_eval()` per cycle. Measured over the suite: **1.80–1.84 s of user CPU
  without it, 1.94–1.96 s with**, three runs each; wall time is 4.0–4.6 s either way and the two
  distributions overlap, because the suite is fork-dominated. That is small, and it is still not
  something to spend on the required gate for a number the gate does not read.

### What it does grade

`make cycles` exits nonzero on **unattributed cycles**: a cycle where `rtl/decoder.v` raised
`stall` and none of the six named reasons was high. That is a stall reason nobody has written down,
and it is the one wrong answer this measurement can give — the cycles would otherwise be silently
correct in the total and attributed to nothing. It also exits nonzero when the columns stop adding
up to the cycle count, which is a field name that has drifted between the runner and the report.

## Evidence

Measured at `0314890` with Homebrew yosys 0.67+post (`b8e7da6f`), riscv64-elf-gcc, `CYCLES=5000`.
**59 programs, 28632 cycles, 13853 instructions retired, CPI 2.07.**

| | cycles | share |
|---|---|---|
| issue | 14030 | 49.0% |
| hazard (decode scoreboard) | **8381** | **29.3%** |
| operand (operand-fetch cycle) | 4684 | 16.4% |
| divider | 792 | 2.8% |
| serialize | 419 | 1.5% |
| accessor (load turnaround) | 318 | 1.1% |
| fetch (stolen window) | 8 | 0.0% |
| unattributed | 0 | 0.0% |

**Half the suite's cycles issue nothing, and the RAW scoreboard is 57.4% of the ones that do not.**
The operand-fetch cycle is 16.4% of all cycles — ADR-0042's +18.0%, now located rather than
inferred. The three reasons that get argued about most (`fence.i` and CSR serialization, the load
turnaround, the stolen fetch window) come to 5.4% between them.

Three runs of `make cycles` are byte-identical.

### Both red directions were executed

- A seventh term ORed into `rtl/decoder.v`'s `stall` (`|| (rs1 == 5'd2)`, thrown away afterwards)
  gives **259 unattributed cycles** and a nonzero `make cycles`, and trips `test/decoder_tb.v`'s new
  identity check with 3 mismatches. The same mutation ORed with `instr_ecall` did **not** trip the
  bench check, which is how its sampling was found to be clock-edge only and widened to both edges.
- `test/probe_gates.sh` goes from 125 probes to **137**: eight against `test/stall_report.py`
  directly (sum mismatch, unattributed, a missing field, a non-numeric count, a malformed line, an
  empty input, and two controls) and four against `run_tests.sh` driving it through the sim stub.

### What the number is not

It is the suite's CPI, not the core's. These are small hand-written assembly programs with dense
back-to-back dependencies and almost no loop structure; their instruction mix is not real code's.
The report says that where the number is printed, because that is where it will be quoted from.

## Consequences

- There is now a per-reason cost to compare any pipeline change against, on the same suite, with the
  same runner. The next argument about forwarding, about the divider's radix, or about narrowing
  `operand_stall` starts from a number.
- `test/decoder_tb.v` carries the identity `stall == the OR of the six named reasons`, checked on
  both clock edges. A seventh stall reason is red in a required gate rather than the next time
  somebody asks for the table.
- **This ADR proposes no optimisation and reopens no ruling.** Forwarding still needs its own ADR
  (invariant 4), CSR serialization still buys `minstret` exactness (ADR-0027), the load turnaround
  is still structural (ADR-0015) and the radix-4 divider is still rejected on area (ADR-0038). The
  measurement exists so the next conversation about any of them starts from data.
- `make cycles` is not on CI. It grades nothing about the design, and adding a job for it would be a
  gate on a number this ADR deliberately declines to ratchet.
- The attribution priority is a ruling and is transcribed in three places — the runner's
  `kStallReasons`, the report's `REASONS`, and this ADR. Reordering one without the others changes
  which column a coincident cycle lands in, and every column would still add up.
