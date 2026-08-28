# 0141 — Zknh's SHA-256 half is measured, and the marginal opcode costs more, not less

Status: accepted (declined) · continues ADR-0131's `sh2add` probe, and is the first real user of
ADR-0132's / ADR-0138's per-encoding completeness exclusion predicate.

## Context

ADR-0131 measured Zba's `sh2add` — a constant shift into the adder `rtl/executor.v`'s add arm
already has — at +110 packed `ICESTORM_LC` on `littlesoc` and a null period. Its planned second
probe was `sha256sig0`: `ROR(x,7) ^ ROR(x,18) ^ (x >> 3)`, no adder, no shifter, three constant
rotates (wiring) XORed together. That probe was never built, because `make -C formal complete`
could not exclude one funct7 row of an otherwise-modelled opcode without either excusing the
modelled rows beside it or narrowing the exclusion predicate past what the mechanism supported —
recorded as ADR-0132's DECISION NEEDED and closed by ADR-0138's build.

The question this ticket answers is not "what does SHA-256 cost" — it is whether `sh2add`'s
+110 cells is a **flat per-opcode tax** or **tracks datapath contact**. `sh2add` touches the adder
`rtl/executor.v` already has; the SHA-256 half touches nothing arithmetic at all. If the cost is
flat, ten-opcode Zknh needs roughly 1100 cells against the ~110 of headroom `FIT_MAX_LC` left after
`sh2add`, and the crypto direction is dead on this part. If it tracks shape, an XOR-only instruction
may be a fraction of `sh2add`'s cost and the direction reopens on real numbers.

## The four encodings, checked against the ratified spec and not from memory

`sha256sig0`, `sha256sig1`, `sha256sum0` and `sha256sum1` are RV32 Zknh's SHA-256 half. Fetched from
`riscv/riscv-crypto`'s scalar-crypto spec (`doc/scalar/insns/sha256sig{0,1}.adoc`,
`sha256sum{0,1}.adoc`) rather than assumed: all four are OP-IMM (`0010011`), funct3 `001` — the same
funct3 SLLI's shift-immediate family already occupies — and funct7 `0001000`, told apart from SLLI
(funct7 fixed at `0000000`) by the funct7 value itself and from each other by the field a normal
R-type reads as `rs2`:

| mnemonic | rs2-position selector | operation |
|---|---|---|
| `sha256sum0` | `00000` | `ROR(rs1,2) ^ ROR(rs1,13) ^ ROR(rs1,22)` |
| `sha256sum1` | `00001` | `ROR(rs1,6) ^ ROR(rs1,11) ^ ROR(rs1,25)` |
| `sha256sig0` | `00010` | `ROR(rs1,7) ^ ROR(rs1,18) ^ (rs1 >> 3)` |
| `sha256sig1` | `00011` | `ROR(rs1,17) ^ ROR(rs1,19) ^ (rs1 >> 10)` |

Cross-checked against `riscv64-elf-as`, which has native support for the mnemonics under
`.option arch, +zknh`: `sha256sig0 x3, x1` assembles to `0x10209193`, which decodes to exactly this
table (opcode `0010011`, funct3 `001`, funct7 `0001000`, selector `00010`). All four instructions
read only `rs1`; the field a normal R-type would read as `rs2` is a function selector with no
register in it, the same shape `lr.w`'s own rs2-position encoding constant already has in this core
(`rtl/decoder.v`'s `instr_lr`).

## What was built and where each half was checked

`rtl/decoder.v`: one `instr_sha256op` gate (`instr_math_immediate_op && funct3 == 3'b001 &&
funct7 == 7'b0001000`) and four `instr_sha256{sig0,sig1,sum0,sum1}` flags off the raw `instr[24:20]`
selector, the same shape `instr_amoadd`/`instr_lr` already use for a funct5- or rs2-position-encoded
row. Folded into `instr_valid`, the publish block's `out.is_*` assignments and zeroing, and the
decoder's `$onehot` completeness assertion the same way every other op flag is.

`rtl/executor.v`: four combinational wires, each a three-term XOR of a constant rotate
(`{rs1[n-1:0], rs1[31:n]}`) and a shift, and four arms in the `rd_data` result mux. No adder, no
barrel shifter — the constant rotates are pure wiring, unlike `sh2add`'s shift into the existing
adder.

`test/decoder_tb.v`: one vector per instruction, checking the flag decodes, the sibling flags do
not, and the issuing `out.is_*` flag with `rs1` passed through unshifted.

`test/asm/sha256.S`: self-checking, the same shape as `test/asm/sh2add.S` and `test/asm/amo.S` —
riscv-formal ships no spec model for Zknh, so every case compares a register against a value computed
from the ratified spec's own `ror32`/xor definition, with nothing routed through the per-retire RVFI
monitor. `TEST_R_OP`, `TEST_R_SRC1_EQ_DEST`, `TEST_R_DEST_BYPASS` and four zero-destination cases per
instruction; `test/OBSERVED_FLOOR` gained a line, `retires=360 spec-checked=312` observed.

## ADR-0132's predicate: the first real user, and what it found

**One exclusion line covers all four instructions, not four lines.** `formal/complete.sv`'s
predicate narrows on opcode, funct3 and funct7 — three fields — and all four SHA-256 encodings share
the identical triple (`0010011`/`001`/`0001000`); what tells them apart is the field a normal R-type
reads as `rs2`, which the mechanism has no field for and does not need one for, because riscv-formal's
`rv32imc` model has no row anywhere under that opcode/funct3/funct7 triple to wrongly excuse.
`formal/check-complete-exclusions.py` accepted the single four-mnemonic line on the first attempt: it
re-derived SLLI's own model encoding as `(opcode 0010011, funct3 001, funct7 000000?)` — the RV32
shift-immediate family's funct6 wildcard on bit 25 — compared it against the declared
`(0010011, 001, 0001000)`, found the compared prefix disagreeing at bit 25 (`0001000`'s bit is `1`
where SLLI's pattern fixes `0`), and refused nothing.

**`make -C formal complete` passes at depth 50 with the exclusion in, and fails at the assertion
ADR-0131 measured without it** — same shape as ADR-0138's `sh2add` acceptance test: `assert
(spec_valid && !spec_trap)` fires on a non-trapping `sha256sig0` retire when `insn_excluded` is not
extended, and does not when it is. `make -C formal complete_cover` still passes at thirteen goals —
no fourteenth goal was needed, because OP-IMM's own anti-vacuity is already covered by every
non-excluded OP-IMM instruction (ADDI, SLLI, ...) reaching `complete_live`; a fourth exclusion-only
cover goal is what AMO needed because AMO's whole opcode class is excluded and nothing else reaches
it, which is not this exclusion's situation.

**Verdict on the mechanism: it worked exactly as ADR-0132 designed it, on the first case built for
it.** The "one row, four instructions" shape this probe surfaced did not need the checker to accept
or refuse anything it had not been built for — narrowing stops at funct7, and no modelled instruction
lives inside this funct7 at all, so the overlap clause had nothing to say no to.

## Cost, on both tops

`make fit` (`littlecpu`, single deterministic placement, matching ADR-0131's methodology):

| | `ICESTORM_LC` | Δ vs base |
|---|---|---|
| base | 4109 | — |
| + `sha256sig0` only | 4132 | +23 |
| + `sha256sig0` + `sig1` | 4168 | +36 (cum +59) |
| + `sig0` + `sig1` + `sum0` | 4231 | +63 (cum +122) |
| all four | 4300 | +69 (cum +191) |

`make soc-timing` (`littlesoc`, packed `ICESTORM_LC` — deterministic across every one of sixteen
seeds swept below, base and after alike, matching ADR-0131's own finding):

| | packed `ICESTORM_LC` |
|---|---|
| base | 4943 |
| after | 5183 |
| Δ | +240 |

Pre-place canonical netlist (`make netlist-diff BASE=origin/main`), the whole-design figure ADR-0131
also quoted alongside the packed one — DIGEST-DIFFERENT, confirming the sweep below was owed:

| | `SB_LUT4` | `SB_DFFESR` | total |
|---|---|---|---|
| `origin/main` | 4450 | 712 | 6310 |
| this tree | 4692 | 716 | 6556 |
| Δ | +242 | +4 | +246 |

**`FIT_MAX_LC` (4219) trips**: `littlecpu`'s `fit` reads 4300 against a budget of 4219, 81 cells
over. This is the finding the ticket asked for, not a defect — the ratchet is not raised and the RTL
does not ship.

## Cost per instruction and the marginal cost of the fourth

Measured incrementally on `littlecpu`'s `fit`, adding one instruction at a time (`sig0`, then `sig1`,
then `sum0`, then `sum1`, each held live while the rest were tied to a constant `1'b0` decode so ABC
could fold the unreached ones away):

| instruction added | marginal `ICESTORM_LC` | running total |
|---|---|---|
| `sha256sig0` (1st) | +23 | 23 |
| `sha256sig1` (2nd) | +36 | 59 |
| `sha256sum0` (3rd) | +63 | 122 |
| `sha256sum1` (4th) | +69 | 191 |

Average: 47.75 cells/instruction on `littlecpu`; 60 cells/instruction on `littlesoc`'s packed count
(+240 ÷ 4).

**The marginal cost does not fall as more instructions are added — it grows, and the fourth is three
times the first.** All four functions are the identical shape (three constant rotates XORed
together, one selector comparison, no arithmetic), so nothing about the *arithmetic* explains this —
what grows is the cost of extending a decode cone and an op-select mux that already carry roughly
forty other flags. The first addition finds slack ABC can still fold into; each later one competes
for the same slack with everything added before it, and the fourth finds the least of it left. This
is the same shape ADR-0117 names for `instr_error`: a cone that already sits on a hot path is not
free to extend, whatever the extension itself computes.

## Period, at sixteen seeds

`soc/baseline_sweep.sh`, sixteen up5k placements a side, one toolchain (OSS CAD Suite `20260811`,
yosys `0.68+48`, nextpnr-ice40 `0.11-1-g62e659ed`, `SOC_PROG=datainit.c`). Run with `SOC_MIN_MHZ=0`
so a placement under the board clock is recorded rather than aborting the sweep — the sub-12 MHz
count below is read off the raw `mhz` column, not from `make soc-timing`'s own ratchet:

| | base | with the four SHA-256 instructions | Δ (ns) |
|---|---|---|---|
| worst | 82.89 ns, 12.06 MHz | 83.43 ns, 11.99 MHz | +0.7% |
| median | 77.00 ns, 12.99 MHz | 79.30 ns, 12.61 MHz | +3.0% |
| best | 74.79 ns, 13.37 MHz | 75.93 ns, 13.17 MHz | +1.5% |
| spread | 10.8% | 9.9% | — |
| seeds under 12.00 MHz | 0/16 | 1/16 | — |

**13 of 16 seeds slower, sign test p = 0.021** — not the null `sh2add` was. The median move (+3.0%)
sits inside `soc/bands.py`'s ~3.6% edit-churn band for this part on its own, but the sign test says
the direction is not noise, and **one placement of sixteen misses the board's 12 MHz clock outright**
(seed 1, 11.99 MHz) where the base held all sixteen. `sh2add`'s adder sat in the fetch loop's own
data path and still measured a period null; four instructions that touch no adder and no shifter at
all cost more period than that, which says the period price here is not about the arithmetic either
— it is the same decode-cone-and-op-select-mux cost the area numbers above already point at, showing
up a second way.

## Verdict: flat, not shape-dependent

`sh2add` — an adder-contact instruction — cost +110 packed cells on `littlesoc` and was a period
null. The SHA-256 half — four XOR-of-rotates instructions with no arithmetic at all — cost +240
packed cells for four, 60 cells per instruction on average, and it is not a period null: 13 of 16
seeds slower (p = 0.021) and one placement of sixteen misses 12 MHz where the base held all sixteen.
**The per-instruction area cost is not a fraction of `sh2add`'s; it is comparable to it, the marginal
instruction is more expensive than the first, not less, and it costs period `sh2add` did not.** The
datapath contact this probe was built to test for — no adder, no shifter, "just" a three-input XOR
per output bit — did not buy a materially cheaper or faster opcode on either axis. What dominates is
not the arithmetic, which really is close to free (`rtl/executor.v`'s own diff is four `assign`
wires with no cells of their own beyond the XOR/rotate wiring), but the fixed cost of adding a fourth
funct7 row and a fourth op-select flag to a decode cone and an op-select mux that already carry
roughly forty other flags — and that cone sits in the fetch loop, which is where a period cost with
no arithmetic behind it has to be coming from.

**For Zknh (ten RV32 instructions): still dead on this part.** Four of the cheapest-shape
instructions available — no adder, no shifter, no multiplexed operand — spent 81 cells more than
`FIT_MAX_LC`'s whole 4219-cell budget allows, on one-quarter of the extension's ten opcodes. Ten at
this observed, *growing* marginal rate is not close to fitting, and this probe used the cheapest four
of the ten — the SHA-512 word-rotate half this ticket did not build adds nothing structurally
different.

**For rotations specifically, apart from Zknh: the wiring itself is free; the opcode is not.** A
constant rotate is exactly what ADR-0087's "a reversal is wiring" argument already established, and
nothing here contradicts it — the arithmetic costs nothing measurable, which is visible in
`rtl/executor.v`'s diff having no cells of its own. What costs is the decode row and the op-select
slot, a property of how many things this core already decodes, not of what a rotation itself
requires. **"Not on this part" is the right sentence for Zknh as a whole; it is not the right
sentence for "does a rotate cost anything" in isolation**, which this probe answers separately and in
the other direction.

## Recommendation

**The probe is reverted. The ISA string does not gain `_zknh`, and Zknh is not claimed.** The RTL,
the decoder_tb vectors, the `.S` program, the `OBSERVED_FLOOR` line and the formal exclusion all come
back out, per the ADR-0100/0101/0113/0131 convention that a probe is a measurement and not a feature.
What survives is this record.

## Amendment 1 — the cost is the functions' own LUT floor, and the period was the spelling (2026-08-28)

At integration the four-flag spelling was challenged: the four encodings share one
opcode/funct3/funct7 triple and differ only in a 2-bit field, so if the cost above is really the
decode row and op-select slot, a one-flag spelling should flatten it. It was built — one
`is_sha256` row flag gated on `instr[24:22] == 0`, a 2-bit `sha_sel <= instr[21:20]` carried
beside it, the executor picking among the same four XOR networks off the select — on the same
toolchain, after first reproducing both endpoints above exactly (base `fit` 4109, four-flag 4300).
The whole suite passed with it, 75/75, `test/asm/sha256.S`'s spec-derived vectors included, and it
was reverted after measuring, per the same convention.

**Area does not flatten: 4296 against the four-flag 4300, a null at the ±50 band.** The four op
flags were never the cost. What is: synthesised standalone for ice40 — no decoder, no fetch loop,
nothing else in the module — one σ/Σ function is exactly **32 `SB_LUT4`, one LUT per output bit**,
the same floor ADR-0119 measured for the AMO's bitwise arms; the four functions plus their
four-way select are **187**; the two Σs plus a two-way select are **96**. Against the +191 `fit`
cells and +242 pre-place `SB_LUT4` the probe measured in the whole core, the XOR datapath is the
whole bill. A constant rotate is wiring; a three-input XOR is not. The sentence above attributing
the cost to the decode row and op-select slot in a cone carrying roughly forty other flags does not
survive this measurement — and neither does the growing-marginal reading, or this record's own
title. The marginals (+23, +36, +63, +69) are differences of single `fit` runs on an instrument
whose churn band is about ±50 cells, and their mean, 47.75 per instruction, is the flat floor
187/4 ≈ 47. Four points drifting monotonically inside the band are churn, by this repo's own rule
that a delta inside the band is not evidence of anything.

**The period cost was the text, not the instructions.** Sixteen paired seeds of the one-flag
spelling, against a base sweep that reproduces this record's own base column seed for seed: worst
**−2.61%** (80.73 ns, 12.39 MHz — faster than the base's own worst), median +0.39%, best +1.07%,
9 of 16 seeds slower, sign test p = 0.804, and **0 of 16 placements under 12.00 MHz** where the
four-flag text lost one. That is ADR-0106's spelling-dependence showing up at the sweep level: two
texts of the same four instructions, one a decline signature and one a null. "One placement misses
the board clock" above is a fact about the four-flag text. (`littlesoc`'s packed count reads the
other way from `fit` between the two spellings — 5259 against 5183 — the two tops disagreeing at
the band, which ADR-0094 says to read as churn.)

**The sum0/sum1 subset does not fit either: `fit` 4225 against the 4219 budget.** The compression
loop's two Σ functions, spelled the one-flag way, consume the entire remaining headroom and land
within re-mapping churn of the ratchet line — not affordable without raising `FIT_MAX_LC` for
cause, and the area ceilings say the named blocks are already closed for cells. No separate sweep
was taken for it; the all-four null above bounds a strictly smaller change of the same spelling,
which is an inference and labelled as one.

**What stands: Zknh stays dead on this part, on harder ground than the Decision gives.** An
implementation artifact could have been re-spelled away; a one-LUT-per-output-bit floor cannot.
Ten opcodes of this shape are roughly 320 cells of function alone against the ~110 the budget
leaves, whatever the decoder says — and the SHA-512 six are wider still. What falls is the
mechanism paragraph, the growing-marginal claim, and the period half of the verdict: a
period-clean spelling of all four exists, so the wall is area, and only area.
