# ADR-0145: A 12 KB text window is priced and declined — the period, not the area

**Status:** Declined (the measurement) · 2026-08-29

## Context

ADR-0135 found that 8 KB of text is a range test's ceiling, not the part's: `rtl/imemory.v` refuses
a `ROM_WORDS` that is not a power of two, because both its range tests are reductions on the address
bits above the ROM, and the next power of two up is 16 KB — 32 `SB_RAM40_4K` against the 26 free once
`rtl/regfile.v`'s 4 are set aside. There is no legal size between the one that ships and one that does
not fit, so `test/asm/rvc.S` (12256 bytes) could not run on silicon and `soc/run_suite_board.sh` had
to batch the suite across reflashes.

12 KB — 3072 words, 24 `SB_RAM40_4K` — fits with two block RAMs to spare. The proposal was two window
shapes to choose between: (a) two power-of-two windows ORed, 3072 = 2048 + 1024, keeping both range
tests reductions at the cost of one OR; (b) a magnitude comparison against the non-power-of-two
constant directly. Shape (a) was expected to be the cheaper of the two.

## What building shape (a) actually touched

The range test is not the only place `ROM_WORDS`'s power-of-two shape is load-bearing, and three more
places needed the identical two-window treatment before the design was even correct, let alone fast:

- **`rtl/imemory.v`'s bank arrays.** `BANK_WORDS = ROM_WORDS / 2` is 1536 at this depth, not a power
  of two, so the existing address computation — a plain truncation that stays in range for a
  power-of-two `BANK_WORDS` by construction — can land between `BANK_WORDS` and `2**BANK_BITS - 1`:
  outside the declared array, undefined in Verilog simulation and an address the physical bank RAM
  does not have. The index is never a *valid* fetch address (`imem_fault` already covers it), but the
  array read still needs an in-bounds address to stay defined. A clamp folds the one bit that would
  carry a raw index past `BANK_WORDS` back to zero before the array read reaches it.
- **`rtl/decoder.v`'s `ls_supported` and `ls_text_deep`.** A plain load or store into the text region
  (self-modifying code, or a load reading data placed in `.text`) is classified by the *decoder's own*
  copy of the map, not `rtl/imemory.v`'s, and both are mask-based tests with the identical
  power-of-two assumption. This one is a correctness question, not a performance one: a load from the
  new 4 KB extension would otherwise fail the mask test and fault on an address the physical memory
  answers.
- **`rtl/littlecpu.v`'s elaboration gate**, because `test/memmap_test.sh` requires its `LS_TEXT_WORDS`
  *default* to equal `rtl/littlesoc.v`'s actual `ROM_WORDS` — `formal/wrapper.v` instantiates
  `littlecpu` with no override, so widening the shipping ROM moves every riscv-formal check's default
  configuration, not only the SoC's.

None of this was visible from the range test alone. It is still cheap in the sense that no one piece
is large, but it is cheap in four places instead of one, three of which are correctness rather than
area or period — and the fourth, decoder's own region test, sits in the same fetch loop the range
test does.

## Correctness, measured before timing

`ROM_EXTRA == 0` reproduces every one of the four changed files' old logic exactly — no new wires, no
new gates on that path — so nothing about the *default* configuration (2048, 4096, or any other pure
power of two) moved. **All 86 generated riscv-formal checks pass** at the new 3072-word default
(`make -C formal check`: `86 checks: 86 pass, 0 fail`, matching `EXPECTED_FAIL` and `EXPECTED_CHECKS`
exactly) — the check suite `formal/wrapper.v` builds now exercises the real two-window shape rather
than the old single-window one. `components_traps`, the only proof over real
`mtvec`/`mepc`/`mcause`/`mstatus`, passes by k-induction with `traps-region-probe`'s own red
directions still firing at their own lines. `test/zkt_isolation_test.py` passes unchanged — the taint
graph's structure (which signals feed `region_stall`, which are blocked as sources past their own
hop) is untouched by widening the computation inside one existing edge. `make soc.json`'s cell census
confirms **`SOC_EXPECT_EBR` is 28** exactly, measured off the SoC's own synthesis log: 24
`SB_RAM40_4K` for the two 1536-word banks (a clean 6×256-word depth cascade each, no padding) plus the
4 `rtl/regfile.v` already took. `test/asm/rvc.S` links to 12256 bytes against the new 12288-byte ROM —
32 bytes to spare — confirmed by building it directly (`make soc-rom SOC_PROG=rvc.S`: 3058 of 3072
words used). Every one of these would have shipped correct.

## The period does not

**`make fit` (the core alone) is a null**: 4094 packed cells against the base tree's 4109, both well
inside `FIT_MAX_LC`'s churn band and, if anything, slightly lower. **`make soc-timing`'s own packed
cell count is not** — 5188 against the base's 4943, +245 cells on the whole SoC that `fit`'s
core-alone build does not see, the same top-dependent split ADR-0094 already measured for a different
edit. But area was never the risk here; the fetch loop was.

**Sixteen paired placements, `datainit.c`, same tree (`e7ee25c`), base with `ROM_WORDS`/`LS_TEXT_WORDS`
reverted to 2048 against the candidate at 3072:**

| | worst | median | best | spread (best-to-worst) |
|---|---|---|---|---|
| base (2048, one window) | 12.06 MHz | 12.99 MHz | 13.37 MHz | 9.8% |
| candidate (3072, two windows) | **11.86 MHz** | 12.23 MHz | 12.56 MHz | 5.6% |

**16 of 16 seeds are slower on the candidate, none faster** — a two-sided sign test puts that at
p ≈ 3×10⁻⁵, the same order of significance ADR-0128's own paired sweep reported for a change with a
real mechanism behind it, not noise. The median moved **−5.89%** and the worst placement **−1.66%**.
**5 of 16 candidate placements are under the 12.00 MHz floor** (`default`, seed 2, seed 3, seed 4,
seed 8 — 11.86, 11.90, 11.98, 11.93, 11.92 MHz) against 0 of 16 on the base. Every one of the sixteen
candidate placements' critical path starts and ends inside `rtl/imemory.v` — `imem.rom_even`/
`imem.rom_odd` read-data bits at one end, the same or a `region_stall`-adjacent net at the other —
which is squarely the fetch loop CLAUDE.md's own ADR history keeps finding to be the expensive one:
the two loops around the fetch address are within 2 ns of each other, and work added to the range
test or the bank-index clamp lands in the loop that already has the least margin.

**By the ticket's own rule — `SOC_MIN_MHZ` 12.0 is a requirement, not a floor that slides, and 0 of 16
under 12.00 MHz is what ships — this misses it at 5 of 16.**

## Shape (b) is declined without a fresh sweep

Shape (a) already fails, and `rtl/imemory.v`'s own header comment states a directly applicable prior
measurement against shape (b): a magnitude comparison against a non-power-of-two constant in this
exact position "yosys can only build as a carry chain longer than a tile column — so it becomes carry
segments with general routing between them, and it measured a quarter of the whole period." That
measurement was taken for `next_is_last`'s own `< ROM_WORDS - 1` arm, not for `ROM_WORDS` itself, but
it is the same test in the same fetch loop against the same class of constant, and shape (a)'s OR of
two reductions already costs more than the board clock's margin. Building and sweeping shape (b)
sixteen seeds a side would be spending a second sixteen-seed sweep to confirm a result the codebase's
own prior evidence already predicts in the wrong direction. It is not measured here; it is declined on
the existing measurement plus the engineering argument, and a future ticket that wants the number
should take it rather than infer it from this one.

## Decision

**Declined. The shipping ROM stays 2048 words / 8 KB.** All RTL, Makefile and test changes made while
pricing shape (a) are reverted in the same commit that adds this ADR — a two-window mechanism that
never ships is not a lower-risk thing to carry in the tree than no mechanism at all, and every one of
the four files it touched sits in or beside the fetch loop. `test/asm/rvc.S` still does not run on
silicon, and `soc/run_suite_board.sh` still batches. The measurement stands: raising the text window
past 8 KB, by either window shape this ticket considered, costs the board clock at a fifth of
placements, and there is no design here that would not.

## What would change the answer

Nothing in this measurement rules out a *cheaper* two-window spelling — the OR itself is a null-sized
cost on paper, and the loss is concentrated in `rtl/imemory.v`'s bank-index clamp and
`rtl/decoder.v`'s second-window mask, both new gates in the fetch loop's own two loops. A future
attempt that moves either off that path (the way ADR-0128's same-cycle region test was later moved a
cycle late in ADR-0129, recovering the clock at the cost of CPI) is the shape most likely to reopen
this. Absent that, the ceiling this ADR measures is ADR-0135's own: 8 KB is what a range test in the
fetch loop can afford, not what the part has room for.
