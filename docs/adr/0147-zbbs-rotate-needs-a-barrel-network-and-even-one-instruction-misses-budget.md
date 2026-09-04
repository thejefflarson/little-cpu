# 0147 — Zbb's rotate needs a barrel network, and even one instruction misses budget

Status: accepted (declined) · continues ADR-0141's Zknh SHA-256 probe with the extension the
workload actually names, on ADR-0141 Amendment 1's standalone-synthesis method.

## Context

ADR-0141 priced Zknh's SHA-256 half — four instructions, each three constant rotates XORed
together, no adder, no shifter — at 187 `SB_LUT4` standalone (its Amendment 1's number, the
functions' own floor) against a headroom of about 110 cells (`FIT_MAX_LC` 4219 minus this tree's
base, 4109). Amendment 1's closing line named the real problem: Ed25519 hashes with SHA-512, so
SHA-256's four instructions never execute on this stack, and its actual cost is field arithmetic the
M extension already serves. **Rotation is the operation this stack's own workload names** —
ChaCha20's quarter-round is add-xor-rotate, and software SHA-256/SHA-512 both use it — and
`ror`/`rol`/`rori` is a real, three-mnemonic slice of Zbb, not the ambiguous four-way selector
`sha256sig0` and its siblings shared one funct7 row over.

## What was built and where it was checked

`rtl/decoder.v`: `instr_rori` is a fifth row of OP-IMM's funct3 101 — one funct7 (`0110000`) below
`srai`'s (`0100000`), the same distance `srai` sits from `srli` — and its amount reaches `math_arg`
through the exact field `slli`/`srli`/`srai` already route it through, so `instr_shift` gains one
term and nothing else about that path changes. `instr_rol`/`instr_ror` are OP's own `0110000` row at
funct3 001/101; `ror` unifies its register and immediate forms into one flag the way `instr_srl`
already unifies `srl`/`srli`. **There is no `roli`**: a left rotate by n is a right rotate by 32−n,
so Zbb does not encode one, and this decoder does not invent one either. All three encodings were
cross-checked against `riscv64-elf-as -march=rv32i_zbb`'s own disassembly
(`rori x1,x2,13` → `0x60d15093`, `ror x1,x2,x3` → `0x603150b3`, `rol x1,x2,x3` → `0x603110b3`)
before any vector was written against them, the ADR-0141 convention.

`rtl/executor.v`: **the existing shifter cannot serve a rotate, and this is a structural fact, not a
missed sharing opportunity.** `shift_wide` is 33 bits — 32 bits of operand plus one fill bit, enough
for a logical shift's zero or an arithmetic shift's sign bit — and a rotate's vacated bits are filled
from the word's own other end, up to 31 of them. So rotate gets its own barrel network, built the
plain way (`{x,x} >> amt`, truncated to 32 bits, which is exactly `ROR(x,amt)` for `amt` in 0..31).
`rol` still reuses the reversal trick `sll` does: `ROL(x,n) = reverse(ROR(reverse(x),n))`, checked
algebraically before it was built (`ROR(rev(x),n)[i] = x[31-((31-i+n) mod 32)]`, and reversing that
result lands on `x[(i-n) mod 32]`, which is `ROL(x,n)[i]` by definition) and then checked by machine
below. Two independent formal reference assertions were added beside the existing shift ones —
`rotr_res == (rs1 >> amt) | (rs1 << (32-amt))`, the funnel-shift identity, not the reversal-trick
expression the implementation itself uses, so a shared bug is unlikely to hide behind both — and both
proved under `make -C formal components_executor`'s k-induction (basecase and induction, ~3.5 min),
the same task that already proves the multiplier's and divider's own arithmetic. A throwaway
directed bench (not committed) cross-checked five concrete vectors against hand computation, all
passing: `ror(0x12345678,8)=0x78123456`, `rol` of the same by 8 is the byte-reverse of that shift,
amount 0 is the identity both directions, and `ror(1,31) == rol(1,1) == 2`.

`test/decoder_tb.v` gained three vectors — decode, sibling-flag exclusion, `math_arg` routing, and
the issuing `out.is_ror`/`out.is_rol` flag — on the assembler-verified encodings above.
`make -C formal components_decoder` and `components_executor` both pass by k-induction with the two
new op flags folded into their `$onehot0` lists, including both tasks' own `-zkt-probe`
prerequisites. `test/zkt_isolation_test.py` still passes: `reg_rs1`/`reg_rs2`/`executor_out.rd_data`
reach only `region_stall` among the eight stall reasons with the two new flags in the netlist, so
rotate's amount and result never reach `stall` — **rotate would stay Zkt-clean if it ever shipped**,
checked on the elaborated netlist rather than assumed from the fact that it is "just" a shifter.
No completeness exclusion was added and `make -C formal complete` was not run: the acceptance
criteria for this probe are the two `fit` numbers, the standalone floor, and the sixteen-seed sweep,
and ADR-0141's own four-instruction probe already established that this repo's completeness
mechanism (ADR-0132/0138) has nothing new to teach about a rotate encoding sharing OP/OP-IMM's major
opcode with already-modelled rows.

## Cost, on both tops

`make fit` (`littlecpu`), same tree, same toolchain (OSS CAD Suite `20260811`, yosys `0.68+48`,
nextpnr-ice40 `0.11-1-g62e659ed`) ADR-0141's Amendment 1 used, reproducing its base exactly:

| | `ICESTORM_LC` | Δ vs base |
|---|---|---|
| base | 4109 | — |
| + `rori`/`ror`/`rol` | 4347 | +238 |

`make soc-timing` (`littlesoc`), packed `ICESTORM_LC`, one placement, and the pre-place canonical
netlist (`make netlist-diff BASE=origin/main`) ADR-0141 also quoted:

| | packed `ICESTORM_LC` | `SB_LUT4` (pre-place) | `SB_DFFESR` | total |
|---|---|---|---|---|
| base | 4943 | 4450 | 712 | 6310 |
| + rotate | 5236 | 4741 | 714 | 6603 |
| Δ | +293 | +291 | +2 | +293 |

`FIT_MAX_LC` is 4219; `littlecpu`'s `fit` reads 4347, **128 cells over budget**. Both tops move by
almost exactly the same amount for once (+238 packed on `littlecpu`, +293 packed on `littlesoc`,
+291 `SB_LUT4` pre-place) — a much tighter agreement than ADR-0094's usual top-dependent spread,
consistent with a cost that is nearly all one self-contained combinational function rather than a
decode cone that packs differently against each top's neighbourhood.

## Standalone synthesis: the function is the whole bill, and reuse does not pay off

Following ADR-0141 Amendment 1's method exactly — `yosys -p "synth_ice40; stat"` on the function
alone, no decoder, no fetch loop, nothing else in the module:

| module | what it contains | `SB_LUT4` |
|---|---|---|
| `rotr_only` | `ROR(x,amt)` alone, `{x,x} >> amt` truncated — the network `rori` and register `ror` both share | 192 |
| `rol_only` | `ROL(x,amt)` alone, via the reversal trick, same network wired backwards | 192 |
| `rotr_rotl_select` | both directions, runtime-selected by `is_rol` — the shape this probe actually ships | 280 |
| `rotr_twoshift` | the "obvious" reuse: `(x >> amt) \| (x << (32-amt))`, both operators already in `rtl/executor.v` for `srl`/`sll` | 328 `SB_LUT4` + 4 `SB_CARRY` = 332 |

**`rol_only` costing exactly 192 — identical to `rotr_only`, not merely close — is the reversal
trick confirmed empirically**: ADR-0087's "a reversal is wiring" holds for a runtime-variable-amount
rotate exactly as it does for the shift amount `sll`/`srl` already share. **The two-shift-OR
spelling, which superficially "reuses the existing shifter" `srl`/`sll` already pay for, measures
worse, not better: +140 cells (73%) against the dedicated funnel network.** Two independent barrel
networks (one per direction) plus a 6-bit subtractor for the complementary amount plus a 32-bit OR
cost more than one 64-bit-wide funnel shift, because the two shifts cannot be time-multiplexed the
way `sll`/`srl` share `shift_wide` — both operands of the OR are needed on the same cycle. **This is
the ticket's "may reuse cheaply, or may not" question, answered by building both and measuring, not
by assuming the existing shifter's presence in the file implies a discount.**

`rotr_rotl_select` (280) is what ships. Against it, `rori` costs **nothing measurable**: it is
`ror`'s existing `is_ror` arm reading `rs2[4:0]` from the immediate-routed `math_arg` instead of a
register — the identical hardware, the same way `srli` costs nothing beyond `srl`'s. The 280-cell
floor is what `rori` alone would already cost, because there is no fixed-shift-amount rotate in Zbb
the way `sha256sig0`'s constant rotates let that probe build a strictly cheaper single-instruction
subset; a variable amount is the whole reason a barrel network is needed at all. **Rotate's three
mnemonics genuinely share one execution unit** — unlike Zknh's four independent XOR functions, whose
combined floor (187) is close to four times any one of them — so if a rotate were ever affordable,
claiming all three would cost almost exactly what claiming `rori` alone would.

Against the whole-core numbers above, the 280-cell floor accounts for nearly all of the +291
`SB_LUT4` pre-place delta — about 11 cells of decode/plumbing overhead, the same small residual
ADR-0141 Amendment 1 found for SHA-256 (191 whole-core against 187 standalone). **The function is
the whole bill here too**; what differs from SHA-256 is the function's own price, not where the
price is paid.

## Period, at sixteen seeds

`soc/baseline_sweep.sh`, sixteen up5k placements a side, `SOC_PROG=datainit.c`, one toolchain (the
same OSS CAD Suite build quoted above), paired by seed against a base sweep on the same tree
(`ab5af01`):

| | base | + rotate | Δ |
|---|---|---|---|
| worst | 82.89 ns, 12.06 MHz | 81.01 ns, 12.34 MHz | −2.3% |
| median | 77.00 ns, 12.99 MHz | 78.13 ns, 12.80 MHz | +1.5% |
| best | 74.79 ns, 13.37 MHz | 74.30 ns, 13.46 MHz | −0.7% |
| spread | 10.8% | 9.0% | — |
| seeds under 12.00 MHz | 0/16 | 0/16 | — |

**12 of 16 seeds slower, sign test p = 0.077** — short of the conventional 0.05 significance
threshold, and the direction the sign test leans is not the direction the tail moved: the
*worst* placement is 2.3% *faster* with rotate than the base's own worst, so whatever the median's
+1.5% is measuring, it is not building toward a placement that misses the board clock. **0 of 16
placements land under 12.00 MHz, against 0 of 16 for the base** — every seed that met the
requirement without rotate still meets it with it. The median move sits inside `soc/bands.py`'s
~3.6% edit-churn band for this part on its own. Read together — a borderline sign test, a null
tail, and a median inside the churn band — this is the same shape ADR-0141 Amendment 1's one-flag
SHA-256 spelling found (9/16 slower, p = 0.804, 0/16 under 12 MHz): **the period is a null.**
Unlike SHA-256's four-flag spelling, which did lose one placement outright, rotate's period cost
was never the obstacle here at either spelling.

## Verdict: the barrel network is the wall, not the decode row, and no subset clears it

**Full Zbb rotate — `ror`/`rol`/`rori` together — is not affordable: `fit` reads 4347 against a
4219 budget, 128 cells over.** Unlike ADR-0141's SHA-256 probe, where the two cheapest of four
functions (`sum0`/`sum1`) landed at 4225 against the same 4219 — six cells short, close enough that
the amendment called it "not affordable without raising `FIT_MAX_LC` for cause" rather than
categorically dead — **rotate has no such near-miss subset, because it has no such thing as a
partial rotate.** The cheapest possible slice is `rori` alone, and `rori` needs the entire 192-cell
`rotr_only` network, standalone, before a single cell of decode overhead — already 82 cells over the
whole ~110-cell headroom on its own. Adding `ror`/`rol` on top costs only 88 more (280 total) because
they share that network; the wall is the barrel shifter's existence, not its instruction count.

**For rotation specifically: the datapath genuinely reuses across all three mnemonics, and the
existing shifter genuinely does not help.** Both are measured facts this probe was built to check
rather than assume, per the ticket's instruction, and both came out the way a reader who has not
built a rotator might not expect — reuse across ROR/ROL is real (the reversal trick), reuse against
the existing SLL/SRL/SRA shifter is a net loss (+140 cells for the naive spelling). Neither changes
the verdict: 192 cells is the floor for offering variable-amount rotation *at all*, in any subset,
and that floor alone exceeds the entire headroom this tree has.

**Zkt would survive rotate intact, if it were ever affordable.** `test/zkt_isolation_test.py` passes
with both new flags in the netlist; the amount and the result never reach any of the eight stall
reasons, the same way `sll`/`srl`/`sra` already do not. This is not what closes the question —
`fit` does — but it means area is the only obstacle recorded here, not a Zkt regression that would
need its own decision.

**Stated plainly, for the record: the full three-mnemonic set is not affordable, no smaller subset
is either, and so nothing here is claimable as an extension.** SHA-256's probe left a real, if
narrow, question — whether a two-of-four subset could be claimed instead — because its four
functions were separable and one pair nearly cleared the budget. Rotate has no such question to
leave open: `ror`, `rol` and `rori` are one function with three names for it, the function's floor
(192 cells) alone is 1.75× the entire headroom, and claiming any one of the three costs
approximately what claiming all three would. There is no ISA-legal single-instruction Zbb subset
cheaper than the whole barrel network, so "claim just `rori`" is not a real alternative the way
"claim just `sum0`/`sum1`" briefly was.

**Rotate stays unclaimed. The RTL, the `test/decoder_tb.v` vectors and the two formal reference
assertions all come back out**, per the ADR-0100/0101/0113/0138/0141 convention that a probe is a
measurement and not a feature; this record is what survives.
