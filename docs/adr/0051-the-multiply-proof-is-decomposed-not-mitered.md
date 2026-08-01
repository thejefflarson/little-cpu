# ADR-0051: The multiply proof is decomposed, not mitered — and the signed divide path gets its first assertions

**Status:** Accepted · 2026-08-01 · *Closes ADR-0049 F1 and F5. Supplements ADR-0010, ADR-0012.
Re-closes M2 term 2 (ADR-0045), this time with a mutation table.*

## Context

ADR-0045 closed M2's second term by naming `components_executor` + `test/exec_tb.v` as the oracle for
real mul/div arithmetic, and wrote that `rtl/executor.v`'s `` `ifdef FORMAL `` block "proves
MUL/MULH/MULHU/MULHSU at full width against SystemVerilog `*`, unrestricted, by `mode prove`, on CI
today."

**That sentence was written without a single mutation being run against it, and ADR-0049 measured it
false in an afternoon.** The divide invariant's operand cap was two *unguarded*
`always_comb assume(in.rs1 <= 32'h0000000f)` statements, and an unguarded assume is proof-global — so
the four multiply assertions forty lines above it ran on operands in 0..15, where every product's
high half is zero and every operand's bit 31 is zero. Three of the four multiplier defects ADR-0010
names by hand **passed** the proof written to catch them (ADR-0049 F1), and the same cap held
`op_sign_x`/`op_sign_y` at constant zero, so both sign tie-backs read `assert(0 == 0)` and ADR-0012's
magnitude wrapper had no formal coverage of any kind (F5).

This ADR is the fix, and it ships the table ADR-0045 did not.

## The wall is the miter, not the operand width

The obvious repair is to narrow the cap and let the existing miter — an independently computed
`mul_ref_*` product asserted equal to the RTL's `multiply` — run at full width. **It does not run at
full width, and the reason rules out the whole family of fixes that adjust the bound.**

| configuration | verdict |
|---|---|
| cap narrowed to rs1 only, `MUL_RS1_BITS` = 8 / 6 / 4, `smtbmc yices` | no verdict at 900 s |
| the same, `smtbmc boolector` | no verdict at 900 s |
| a standalone miter: multiplier + reference only, no divider, no pipeline state, no cap, full 32-bit operands, `mode bmc depth 1` | **no verdict in 2 minutes** |

The last row is the one that settles it. With the divider gone, the pipeline gone and the depth at 1,
the query still does not return — so the obstacle is not the operand bound, not the surrounding
state, and not the engine. It is the miter itself: **two structurally distinct `bvmul` terms**, which
is exactly the shape ADR-0045 measured from the other direction when a non-ALTOPS `insn_mul_ch0` sat
for 15 m 26 s under z3.

Any ceiling table produced by sweeping the operand cap is therefore measuring the wrong variable, and
a ceiling derived in the *vacuous* configuration measures nothing at all.

## Decision

**Do not miter two multipliers. Prove the wrapper around one shared product term** — the same move
ADR-0012 already made for the divider, where the loop invariant reasons over `div_x`/`div_y` rather
than restating division.

`rtl/executor.v`'s `` `ifdef FORMAL `` block is rewritten as one coherent proof:

1. **The operand cap is guarded to the divide family** — `is_div || is_divu || is_rem || is_remu ||
   state == divide` — so every multiply assertion runs against free 32-bit operands.
2. **The multiply proof is decomposed into three obligations:**
   - **(a) the 33-bit operands.** `{mul_sign_x, in.rs1}` and `{mul_sign_y, in.rs2}` must equal
     extensions derived by width-extending assignment from a self-determined right-hand side
     (`assign rs1_sext33 = $signed(in.rs1);`), selected per variant. MUL/MULHU unsigned both,
     MULH signed both, MULHSU signed × unsigned.
   - **(b) the retired slice.** The correct half of *that same* `multiply` term reaches
     `out.rd_data` for each of the four variants.
   - **(c) three constant-multiplication lemmas** over the product term itself: `rs1 == 0 →
     prod == 0`, `rs2 == 0 → prod == 0`, and `rs2 == 1 && !mul_sign_y → prod == {{32{mul_sign_x}},
     rs1}`. Each multiplies by a **constant**, so the solver sees shifts and adds rather than a
     second `bvmul`.
3. **The divide cap becomes a magnitude cap** on `div_x`/`div_y` — the ADR-0012 magnitudes the
   divider actually loads — so DIV/REM operands range over −15..15 with the **sign free**, and
   `op_sign_x`/`op_sign_y` stop being constant zero.
4. **Signed DIV/REM completion assertions are added**, tying `out.rd_data` to `$signed(rs1) /
   $signed(rs2)` and `%`. Until now the only completion assertions in the task were the unsigned
   pair.

(a) and (b) read the same product term the RTL does, so neither can see a product that is simply
wrong. **(c) is therefore not optional** — it is the only part of the multiply section that
constrains the term's own value, and it is what catches a masked high half.

### The cap must bound `div_x`/`div_y`, not `in.rs1`/`in.rs2`

Capping `$signed(in.rs1)` to −15..15 directly was built first and **does not work**. For DIVU/REMU no
magnitude conversion happens, so a "small" negative `in.rs2` is a divisor near 2³²;
`mul_div_store * div_scaled_divisor` then wraps mod 2⁶⁴ and induction fails on the
`mul_div_store <= div_x` bound — the very invariant that exists to rule wraparound solutions out.
Measured: **induction FAIL at 4 s** (`executor.v`'s quotient-bound assertion), basecase still crawling
at step 17 after 1 m 45 s.

Bounding `div_x`/`div_y` instead gives the intended reading in both families: for DIV/REM it is a
magnitude cap that leaves `in.rs1[31]` free; for DIVU/REMU, where `div_x` *is* `in.rs1`, it is the
same 0..15 restriction as before, which is correct — those operands have no sign to free.

### The `|| state == divide` term is required

Guarding the cap on the op flags alone fails induction: a start state can sit in `divide` with every
`is_div*` low, and the cap lapses there. Measured; the term stays.

### `rs2 == 32'hffffffff` is not a fourth lemma

Multiply-by-−1 is the obvious next lemma and is measured **not** to work: no verdict in two minutes.
It is named in the source so it is not proposed again without re-measuring.

## The mutation tables

Both oracles, same eleven mutations, each applied to a scratch copy of `rtl/executor.v` (the tree is
untouched). `M1`–`M9` are the required set; `C1` and `C2` are controls.

### `components_executor` — `sby -f components.sby executor`

**Read the verdict column with ADR-0049 F3 in hand.** `components.sby` sets no depth, so `mode
prove`'s basecase runs 20 steps, and the real divider needs 33 cycles from issue — so any assertion
guarded on `$past(state) == divide && state == init` is **basecase-unreachable**, and breaking it
gives `UNKNOWN rc=4` (basecase pass, induction FAIL) rather than `FAIL`. That is this assertion
class's normal detection signal, not a weakness of the new assertions, which is what **C1** is in the
table to demonstrate: it mutates the *already-accepted, pre-existing* DIVU completion and reports the
same `UNKNOWN rc=4`.

| # | mutation | verdict | caught by | wall |
|---|---|---|---|---|
| M1 | MULHSU treats rs2 as signed | **FAIL rc=2** | (a) `mul_op_y_ref` | 1.2 s |
| M2 | `mul_sign_x = 1'b0` | **FAIL rc=2** | (a) `mul_op_x_ref` | 1.0 s |
| M3 | `mul_sign_y = 1'b0` | **FAIL rc=2** | (a) `mul_op_y_ref` | 0.8 s |
| M4 | sign taken from bit 0 | **FAIL rc=2** | (a) `mul_op_x_ref` | 0.7 s |
| M5 | high half masked to zero | **FAIL rc=2** | (c) lemma 3 | 0.7 s |
| M6 | `multiply` forced constant | **FAIL rc=2** | (c) lemma 3 | 0.7 s |
| M7 | MULH takes the low slice | **FAIL rc=2** | (b) MULH slice | 1.3 s |
| M8 | delete the `op_is_div` sign restore | **UNKNOWN rc=4** | `div_ref` completion | 38.2 s |
| M9 | delete the `op_is_rem` sign restore | **UNKNOWN rc=4** | `rem_ref` completion | 30.7 s |
| C1 | `op_is_divu` completion + 1 (**control**) | **UNKNOWN rc=4** | `divu_ref` completion | 23.7 s |
| C2 | MUL low half + 1 (**control**) | **FAIL rc=2** | (b) MUL slice | 1.5 s |

**Nothing in the required set is uncaught.** Six of these are mutations ADR-0049 measured as
**PASSING** the pre-change task — M1, M2, M3 (the sign enables), M5 (the high half), and M8/M9 (F5's
sign restores). M4 was caught before and still is. M6, M7 and the two controls are new here.

### `test/exec_tb.v` — `iverilog` + `vvp`, ADR-0010's primary oracle

M2 term 2's closure names **both** oracles, so both are shown to catch. Every row exits 1 and prints
the offending vector.

| # | exit | first mismatch line |
|---|---|---|
| M1 | 1 | `MISMATCH mulhsu rs1=12153524 rs2=c0895e81 got=fb8466bd expected=0d999be1` |
| M2 | 1 | `MISMATCH mulh rs1=ffffffff rs2=ffffffff got=ffffffff expected=00000000` |
| M3 | 1 | `MISMATCH mulh rs1=ffffffff rs2=ffffffff got=ffffffff expected=00000000` |
| M4 | 1 | `MISMATCH mulh rs1=06b97b0d rs2=46df998d got=bafcfdb3 expected=01dc9740` |
| M5 | 1 | `MISMATCH mulhsu rs1=ffffffff rs2=00000001 got=00000000 expected=ffffffff` |
| M6 | 1 | `MISMATCH mulhsu rs1=ffffffff rs2=00000001 got=00000000 expected=ffffffff` |
| M7 | 1 | `MISMATCH mulh rs1=ffffffff rs2=ffffffff got=00000001 expected=00000000` |
| M8 | 1 | `MISMATCH div rs1=7cfde9f9 rs2=e33724c6 got=00000004 expected=fffffffc` |
| M9 | 1 | `MISMATCH rem rs1=8484d609 rs2=b1f05663 got=2d6b805a expected=d2947fa6` |
| C1 | 1 | `MISMATCH divu rs1=12153524 rs2=c0895e81 got=00000001 expected=00000000` |
| C2 | 1 | `MISMATCH mul rs1=12153524 rs2=c0895e81 got=5676ff25 expected=5676ff24` |

Note which rows the two oracles catch *differently*. M1–M7 are caught by the bench in the directed
vectors ADR-0010 mandated (`MULH(-1,-1)`, `MULHSU(-1,1)`, `MULHU(-1,-1)`) or within the first random
pair; M8/M9/C1 need a negative operand, which the bench reaches by randomization and the proof
reaches by construction under the magnitude cap. Neither oracle is redundant with the other.

## The residual, stated plainly

**The multiplication operator itself is checked differentially, not exhaustively.** There is no miter
here. What the proof establishes is the operands, the retired slice, and three points of the product
function; what covers the rest is `test/exec_tb.v`'s ≥10,000 randomized vectors per op. That
sentence is written at the top of the `` `ifdef FORMAL `` block, not only here, because the place a
reader forms a belief about this proof is the source.

Two further caveats carry over unchanged and are not closed by this work:

- **ADR-0049 F3.** The four completion assertions are basecase-unreachable at depth 20. The divide
  result is proved inductively and never simulated inside this task. Raising the basecase past 33 is
  a depth change needing its own evidence (ADR-0025).
- **ADR-0010's ALTOPS caveat.** The generated ladder still never runs this arithmetic.

## Rationale

The alternative was to keep the miter and buy convergence with a tighter bound. Rejected on the
measurement above: at `depth 1` with nothing else in the module the miter still does not return, so
there is no bound that buys it. Decomposition is not a weaker proof of the same thing — it is a
proof of different things, and the honest move is to say which ones and to demonstrate what each
catches.

The other alternative was to leave the cap unguarded and record the scope, which is exactly what
ADR-0049 did as an interim. That was correct as an audit finding and is not a resting place: clause 3
of ADR-0049 offers "narrow the assume" as the preferred route wherever it costs nothing, and here it
costs 44 s of solver time.

## Consequences

- `components_executor` goes from **6.5 s to 50.7 s** wall (`sby` elapsed process time 37 s), by
  k-induction, on the same engine. It stays on CI's `components` job.
- **ADR-0049 F1 and F5 are closed.** The census row for `executor.v`'s operand cap changes from
  scope "the whole task" to scope "the divide family", and it becomes the **second** guarded assume
  in `rtl/`.
- **M2 term 2 is re-closed, and this is its third move.** ADR-0045's closing line says a third should
  prompt asking whether the criterion describes anything real. It does: the criterion is "the real
  multiplier and divider have an oracle", and the two prior closures were unmeasured claims about
  oracles that existed. The difference here is the tables above. **A term that closes without its
  mutation table is not closed.**
- The independently computed `mul_ref_uu`/`mul_ref_ss`/`mul_ref_su` products are deleted. Anyone
  restoring them should read the wall-table above first.
- No `rtl/` file changes outside the `` `ifdef FORMAL `` block, so `make fit` does not move and the
  shipping core is untouched.
