# ADR-0131: One opcode is measured, and the exclusion set is the wall, not the clock

**Status:** Accepted (declined) · 2026-08-24

## Context

Every proposal to add instructions here -- bitmanip, the crypto extensions, anything -- runs into a
number this repo did not have: what does one more opcode cost in the fetch loop? ADR-0115 measured a
decode cone that had drifted into `trap_pending` at −2.47% of median period for eleven cells, and
ADR-0094 measured the opposite: edits that state a real fact about the compressed decode are worth
nothing at all because ABC had already folded them. So the decode block was closed **for area** on
two measurements and closed for period on **none**. Adding an opcode is the direction nobody had
measured.

Zba's `sh2add` was built as the cheapest plausible probe: a constant shift into the adder
`rtl/executor.v`'s `is_add` arm already has (`{rs1[29:0], 2'b00} + rs2`), one decode row in
`rtl/decoder.v` (opcode `0110011`, funct7 `0010000`, funct3 `100`, the OP major opcode's fourth
funct7 value beside `math_low`, `math_high` and `instr_m`), no new arithmetic, no forwarding-network
touch. It was decoded, executed, covered by `test/decoder_tb.v` and a self-checking
`test/asm/sh2add.S` under `.option arch, +zba` (the suite's own `-march` was left untouched), and
`test/OBSERVED_FLOOR` gained a line.

## Decision

**Built, measured on both axes, and declined -- not on cost, on a formal gate the RTL cannot pass at
all without a change out of this ticket's scope.** The RTL, the decoder_tb vector, the `.S` program
and the floor line come back out before this lands, the way ADR-0100's and ADR-0101's did. What
survives is the measurement, on `41f48e4` (this branch, one commit past `9944aad`).

### The formal walk is what declines it

`make -C formal complete` asserts, for every retire whose opcode class is not in
`formal/COMPLETE_EXCLUSIONS`, that riscv-formal's `rv32imc` spec model recognises the instruction and
says it does not trap. `sh2add` retires non-trapping under the OP major opcode (`0110011`), which is
**not excluded** -- ADD, SUB and the M extension all live there and all have spec models, so the
class as a whole is checked. The spec model has no arm for funct7 `0010000`, so `spec_valid` is 0 on
every `sh2add` retire and the assertion fails:

```
engine_0: Output 0 of miter "model/design_aiger" was asserted in frame 5. Time = 0.06 sec
...
Assert failed in rvfi_testbench: complete.sv:212.7-212.39
  assert(spec_valid && !spec_trap);
```

`formal/check-complete-exclusions.py`'s `WIRE_RE` accepts exactly one predicate shape per exclusion:
`insn_uncompressed && insn_opcode == 7'bXXXXXXX` -- a whole seven-bit opcode class, nothing narrower.
Excluding `0110011` outright is not the fix either: `check_no_spec_model` reads `insns/isa_rv32imc.txt`
from the pinned clone, finds `add`/`sub`/the M mnemonics named there, and refuses the exclusion by
design -- the check exists precisely so a mnemonic that *does* have a model cannot be waved off with
its neighbours. There is no way, inside the mechanism as it stands, to say "one funct7 row of an
otherwise-modelled opcode has no spec model" without either wrongly excluding the modelled rows
beside it or narrowing the predicate shape `check-complete-exclusions.py` deliberately restricts to a
whole class -- which the script's own comment calls out as "a design change, [that] belongs in an
ADR, not in a regex."

This is not a defect in `sh2add`'s decode, and it is not particular to `sh2add`: **every candidate
extension named in this repo's own ISA-target section shares a major opcode with an already-modelled
instruction.** Zba's other two shifts sit in the same `0110011` row; Zbb and Zbs mostly extend
OP and OP-IMM; the SHA-256/512/word rotations of Zknh live under OP-IMM (`0010011`), which ADDI, the
shifts and the compare-immediates already occupy. A probe of any of them hits the identical wall.
That makes this a **structural finding about the exclusion mechanism**, not a per-instruction one --
recorded here rather than worked around, because working around it silently is exactly the kind of
weakened check ADR-0010 requires a record for.

**DECISION NEEDED, for the architect:** whether `formal/complete.sv`'s exclusion predicate should be
widened to key on the full encoding (opcode **and** funct3/funct7) rather than only the opcode class,
which would let a future extension exclude the one row it actually adds without excusing the modelled
rows beside it. That is a change to a hand-written formal assertion and its checker script, is worth
its own ADR, and is out of scope here: this ticket's job was to measure the cost of one opcode, not
to redesign the mechanism that decides whether the ISA can claim it.

### The period cost is a null at sixteen seeds

`make netlist-diff BASE=origin/main` confirmed the digest moved (sweep owed), so the paired sweep was
taken with `soc/baseline_sweep.sh`, sixteen seeds a side, on `9944aad` (base) and `41f48e4` (with
`sh2add`), one toolchain (OSS CAD Suite `20260811`, yosys `0.68+48`, nextpnr-ice40 `0.11-1-g62e659ed`,
`SOC_PROG=datainit.c`):

| | base | with `sh2add` | Δ |
|---|---|---|---|
| worst | 80.30 ns, 12.45 MHz | 82.51 ns, 12.12 MHz | **+2.8%** |
| median | 78.31 ns, 12.77 MHz | 78.69 ns, 12.71 MHz | **+0.5%** |
| best | 76.12 ns, 13.14 MHz | 76.96 ns, 12.99 MHz | **+1.1%** |
| spread | 5.5% | 7.2% | -- |

11 of 16 seeds slower, sign test p = 0.21 -- a null, and every one of the +0.5% median / +2.8% worst
is inside `soc/bands.py`'s own ~3.6% edit-churn band for this part. **Every placement of sixteen
holds the 12 MHz requirement**, worst seed 12.12 MHz, margin down from base's 3.75% to 1.0%. Reading
`soc.timing.rpt` with `soc/depth/path_stages.py` finds the critical path running
`decode → decode/imem → decode/csrs/access/core/top → csrs → decode/csrs`, the same fetch-loop/CSR
cone every prior period investigation in this tree has named -- no `executor` stage appears on it, so
the new arm does not look like it sits on the reported path. That locates a candidate, not a
decision (ADR-0116): the sweep, which read a null, is the verdict.

### The area cost is real, on the one instrument that is not noisy here

`make fit` (`littlecpu`, single deterministic placement each): **4102 → 4151, +49 packed
`ICESTORM_LC`** -- inside the ~50-cell churn band on its own, so this instrument alone cannot tell
the opcode's cost apart from re-mapping noise. Against `FIT_MAX_LC = 4219`, that is 68 cells of
headroom left where there were about 117.

`make soc-timing` (`littlesoc`): packed `ICESTORM_LC` was **4875 on every one of sixteen base
placements and 4985 on every one of sixteen `sh2add` placements** -- deterministic on both sides, so
the **+110 cells (+2.3%)** it reports is not seed noise the way the timing column is. Grading the
ceiling on packed cells rather than `SB_LUT4` matters here the way it did in ADR-0112: the pre-place
canonical netlist (`make netlist-diff`) reported **SB_LUT4 4397 → 4510 (+113), SB_CARRY 695 → 724
(+29), SB_DFFESR 690 → 691 (+1), 6224 → 6367 total (+143)** across the whole design, a different
number from the placed-and-packed +110 on the same top, in the same direction but not the same
magnitude -- exactly ADR-0112's warning that a LUT the flops beside it were sharing a cell with is
not a LUT anyone can spend.

## What the numbers say about flat versus linear

The period is flat: a one-mux, no-new-arithmetic opcode moved the median 0.5% and held 12 MHz at
every placement, which is the null direction ADR-0117's "measure the whole set" rule expects for one
term. **The area is not flat.** The cheapest possible instruction spent 49 of `FIT_MAX_LC`'s roughly
117 cells of headroom on `littlecpu`'s own count, leaving 68, and the SoC top's cleaner +110 agrees
on direction if not on magnitude. Read linearly against that one data point: Zba's other two shifts
(`sh1add`, `sh3add`) at a similar cost would clear the ratchet outright before the register-write
path or the immediate encoding of anything bigger is even touched, and the ten-opcode `Zknh` or the
roughly forty of full `Zbb`+`Zbs` are not close. **This is one point, not a curve** -- a second probe
of a different shape (`sha256sig0`,
rotations and a 3-input XOR with no adder contact) was in scope to build if `sh2add` came back cheap,
and its period does read as cheap; it was not built, because the formal-completeness finding above
declines any next opcode in the same major opcode space regardless of what a second area number would
say, and spending another sixteen-seed sweep on a question the first finding already closes is not
this ticket's job.

## Recommendation

**The probe is reverted. The ISA string does not gain `_zba`.** Not because the instruction is
expensive in the fetch loop -- it measurably is not -- but because `make -C formal complete` cannot
be made to pass for it without a change to the exclusion mechanism that this ticket did not scope and
should not make unreviewed. Building that mechanism -- a per-encoding exclusion predicate in
`formal/complete.sv`, checked the way `check-complete-exclusions.py` checks today's whole-class one --
is the prerequisite for any future extension that shares a major opcode with an already-modelled
instruction, which by the ISA-target section's own account is every candidate this repo has named.
That is a DECISION NEEDED for the architect, not a call this probe makes for it.
