# 0207 — A register-only fetch address over a one-window skid ships as the fetch front end

Status: Accepted. 2026-09-21, amended 2026-09-23 and 2026-09-28. The RTL shipped first as a tracked patch
(`soc/fetch_ahead/skid.patch`) while its up5k fit was in question; it now ships in `rtl/` as the
real fetch front end on `thejefflarson/fetch-refactor`, the integration branch the rest of the
fetch refactor stacks on. The shape is correct, proven, and faster than `main` in cycles. It still
misses the up5k by five placed cells with the flash controller restored — **that overrun is not
this ADR's to close**: the owner's call is to finish the refactor first and trim cells once, on
the tree Stage B leaves, rather than shave five cells off a shape that is about to move again.
`make soc-timing` is expected red on the branch that carries this commit until that trim pass
lands; every other gate is green.

## What this is

ADR-0205 priced Stage A's fetch path on the placed SoC and found the four-word queue alone +585
cells against a part with about 360 free, and that no cut to the predictor changed that. This is
the re-spike it asked for, from a clean base: three shapes built end to end, each measured as a
placed SoC, on Dhrystone, and on ECP5, under a budget of about +250 placed cells, no worse than
+4.1% of Dhrystone's cycles, and a real share of the ECP5 clock Stage A saw. None of the three
meets all three; the cheapest one that keeps the cycles is +365 to +418 cells, and no shape moves
the ECP5 clock at one placement, because the clock Stage A bought came from registering the head
of the fetch loop and not from decoupling its tail. Both halves of that are the finding.

The amendment below moves that shape from a tracked patch into `rtl/` itself, on the branch the
rest of the fetch refactor stacks on, and runs the verification the original write-up deferred
until the shape shipped somewhere: the generated riscv-formal set, `imemcheck`, `make
cosim-suite`, `make mutation-check` and `make dual-smoke`.

## The mechanism

`rtl/fetcher.v` presents an address built from registers alone. The ROM answers a word address a
cycle after it is presented, so decode's window at `pc` — the two words `{w[p], w[p+1]}` that
`rtl/fetcher.v` has always windowed an instruction out of — is read off one of two places: the
ROM's own output register, when the address it holds is `pc`'s word, or `skid`, a 65-bit register
holding the one window fetch has moved past.

- **The fetch address**: `word + hit`, where `word` is `pc[31:2]` and `hit` says the window at `pc`
  is available this cycle; a live guess overrides it with the guessed target's word. A miss re-reads
  decode's own word; a hit reads the word after it. Nothing here reads `next_pc`, `stall`, or the
  word arriving from the ROM.
- **The skid** captures the ROM's window whenever the ROM holds decode's word and the skid is empty;
  its valid bit is set only if decode did not leave that word (`pop`, from `next_pc`) and clears
  when it does. A valid skid therefore always holds decode's current word, so `hit` needs no
  address compare against it. Decode consumes at most one word per cycle and the ROM delivers one
  new word per cycle, so one window of skid is all a decoupled address needs: a deeper buffer
  cannot reduce cycles, only absorb a faster front end this design does not have. That is why
  Stage A's queue was four words for a launch rule that only ever needed one word of slack.
- **The straddle is answered by construction.** A window is two words, and popping one word moves
  to the next window, so an instruction at `2 mod 4` always has both halves in whatever source
  answers `hit`. The depth-2 skid ADR-0196 built was word-granular with no output mux; the 64-bit
  window is the output mux, spent on purpose.
- **The guess** is formed a cycle early, off `next_instr` — the raw word after the instruction at
  decode, which decode already windows for the register-pair guess. A `jal`, `c.j`, `c.jal`, or a
  backward `branch`/`c.beqz`/`c.bnez` next in line registers its target (`pc + pc_inc + imm`) and a
  valid bit; the bit clears the cycle decode issues, so it steers the ROM exactly while that
  instruction sits in decode. A word-straddling instruction leaves `next_instr`'s upper half
  empty, so no guess. No mispredict signal exists: decode computes `next_pc` as it always has, and
  a wrong guess is simply a miss the next cycle.
- **Every miss costs exactly one cycle** — an unguessed redirect, a mispredict, and a stolen ROM
  read alike — and un-commits nothing: the decoder's `fetch_stall` input now means "the window at
  `pc` is not here", and the eight stall reasons, their six declared sites and the decoder itself
  are unchanged. A stolen read while the skid answers decode costs nothing, which is where the
  cycle win below comes from.

The decoder gains two outputs, `issuing` and `redirect`, read only by the guess register's enable.
`rtl/fetcher.v`'s `FORMAL` block asserts that the ROM's address is last cycle's fetch address,
that a valid skid holds decode's word, and that an issuing cycle reads decode's word from one of
the two; `formal/pcloop.sv` reads it with `-formal -noassume` and proves all three by k-induction
alongside its own pc properties, with three covers reaching the skid, a miss followed by a hit, and
a taken guess. `formal/traps.sv` is rewired the same way and `components_traps` passes with both
probes. `test/fetcher_tb.v` runs the fetcher over a scripted decode and a one-cycle ROM through a
sequential run, a compressed pair, a straddle, guessed and unguessed branches, a mispredict, three
held cycles, two stolen reads and a faulting word, checking the window's source and the miss
count at each step. `make -C formal remeasure-fg` reads F = 6, G = 6, unchanged.

## The curve

Every row is one configuration built as a throwaway edit of the patched tree, placed by
`make soc-timing` at the pinned seed (20740127) on one machine and one toolchain — Yosys 0.68+48,
nextpnr 0.11-1-g62e659ed, icetime oss-cad-suite 20260811 — with the packed `ICESTORM_LC` read off
nextpnr's utilisation table whether or not the placement then succeeded. Dhrystone is
`make dhrystone` at `DHRY_RUNS=2000`; CoreMark `make coremark` at `COREMARK_ITERATIONS=20`, the same
image both sides. ECP5 is `make ecp5-timing` at one placement (the part's own spread is 10.3%,
ADR-0194). `main` is a74544d, placed before #395 restored the flash controller; the last row
places the shipping shape against 474bb03, the main that carries it, since that is the part the
shape has to fit. Both instruments carry a churn band of about ±50 cells.

| shape | SoC LC | places | icetime MHz | cycles / 2000 Dhrystones | vs main | ECP5 MHz |
|---|---|---|---|---|---|---|
| `main` (a74544d), no skid, no guess | 4,836 | yes | 13.18 | 1,613,644 (0.722 DMIPS/MHz) | — | 35.00 |
| control alone: register address, no skid, no guess | 4,971 | yes | 13.19 | 2,062,027 on Dhrystone's own timed loop against 1,576,021; the run's report never finished inside the 4,000,000-cycle limit | +30.8% | — |
| skid, no guess | 5,129 | yes | 12.15 | 1,713,400 (0.680) | +6.2% | 33.00 |
| skid, guess from decode's own target adder (a stalled branch only) | 5,157 | yes | 12.38 | 1,671,122 (0.698) | +3.6% | 34.03 |
| guess, no skid | 5,192 | yes | 12.68 | — | | — |
| **skid and the one-cycle-early guess, as the patch ships** | **5,254** | yes | 12.78 | **1,587,014 (0.734)** | **−1.65%** | 33.03 |
| the same, with `pop` in the skid's data enables (the first spelling) | 5,287 | no | — | 1,587,014 | −1.65% | 32.51 |
| the same, ROM-hit tracked as a flag rather than an address compare | 5,273 | yes | 12.55 | — | | — |
| **the shipping shape on 474bb03, the main with the flash controller back** | **5,285** | **no** | — | 1,587,014 | −1.65% | — |
| `main` 474bb03 | 4,920 | yes | 12.61 | 1,613,644 | — | — |

CoreMark at 20 iterations, cycles for the timed run: `main` 9,279,683, the shipping shape 9,535,010,
**+2.75%** — CoreMark's branch mix is forward-heavier and `jalr`-heavier than Dhrystone's, and
neither is guessed, so the one-cycle misses outweigh the stolen reads the skid hides there.
(`run_coremark.sh` declines to print CoreMark/MHz below its own iteration floor, so the cycle
counts are what this row carries.)

What the rows say:

- **The shape is faster than `main`, not slower.** The guess covers Dhrystone's loop back-edges at
  zero cost, and the skid absorbs the ROM reads Dhrystone's `.rodata` loads steal — `main` pays a
  bubble for every one of those. Read off a per-issue diff of two `--vcd` traces of the same
  fifty-run image: the two pcs after a text load lose 1,612 and 1,068 stall cycles, and the
  sixteen loop pcs behind a redirect gain 50 each, one miss per unguessed or mispredicted turn.
  With no guess the misses win, +6.2%; with the guess only where decode's own `pc + immediate`
  adder can supply it — a branch that stalls at decode for a cycle anyway — +3.6%.
- **The cells are the control, the skid and the guess in roughly equal thirds**, +135, +158 and
  +125 by the rows above, with the flag-tracked hit (−14) and the first fetch-mux spelling (+45)
  inside the band. The skid's 65 flops do not pack: their D is the block RAM's output, so each
  costs a cell of its own, and the 64-bit window mux is another 64 — 129 cells is the price of
  reading the ROM's output register and a copy of it through one mux. The guess is a four-encoding
  immediate mux, one adder and a 30-bit register that packs into it. The control is a 30-bit
  incrementer, a 30-bit compare and the fetch mux.
- **The ECP5 clock does not move.** At one placement every shape reads 32.5 to 34.0 MHz against
  `main`'s 35.0, inside the part's 10.3% spread. The critical path says why: `main`'s runs
  `ROM DOB → decode → next_pc → ROM address`, the shipping shape's runs `ROM DOB → skid mux →
  decode → next_pc → pc`, and the first spelling's ended at the skid's clock enables instead
  (`next_pc → pop → capture → sixty-five CE pins`), which is why `pop` was moved off the data
  enables. The loop's tail is gone and its head is still the block RAM's output, one mux deeper.
  ADR-0087 measured exactly this on `addr` — the tail comes out, the head does not — and Stage
  A1's 41.76 MHz against 34.90 came from decode reading the queue's flops rather than the ROM: a
  registered head. A registered head needs the ROM two words ahead of decode, which needs a second
  window register, which is the four-word queue's cost by another spelling, and pays two cycles
  per unguessed redirect rather than one. `soc/paired_sweep.sh 474bb03 ecp5`, twelve seeds paired by seed, on the shipping shape: base worst
  33.01 MHz / median 34.39, candidate worst 31.80 / median 34.53; per seed the candidate reads
  −9.0% at the worst pairing, +1.2% at the median and +5.9% at the best, slower at five of twelve.
  That is inside the part's 10.3% placement spread in both directions: a null, not a loss. (The
  summary refused the pair on its own tree-mismatch rule because the candidate tree carried this
  ADR's uncommitted text beside the RTL; the per-seed deltas above are read off its two CSVs.)
- **The up5k does not hold it.** 5,254 on the main this was cut from is 26 cells under the part;
  on the main that carries the restored flash controller it is 5,285, five over, and nextpnr
  refuses to place. A `make soc-seed-search` pin needs 12.60 MHz with a 5% margin under it, and a
  placement at the part's last cells does not find one. Against the +250 budget the shape is
  +365 to +418, and the two shapes inside +330 — the skid without a guess, and the skid with only
  a stalled branch guessed — miss the cycle ceiling and the budget both.

## The decision

**The RTL ships**, in `rtl/fetcher.v`, the two decoder outputs, `rtl/littlecpu.v`'s wiring, both
formal harnesses, their `.sby` tasks and `test/fetcher_tb.v`, on `thejefflarson/fetch-refactor` —
the integration branch the rest of the fetch refactor stacks on, not `main` directly. The patch
this shape first shipped as, and the script that staged and graded it
(`soc/fetch_ahead/skid.patch`, `soc/fetch_ahead/patches_check.sh`), are deleted along with the
`make test` target and the two probes that graded them: once the shape lives in `rtl/`, every
gate that already reads `rtl/` — `make test`, `make lint`, `make elaborate-strict`, the formal
component proofs — grades it directly, and a second grading path over a patch would just be a
second place the same claim could drift from the code.

**On the up5k the fit is still short, and closing it is out of scope here.** The cheapest
decoupled fetch that keeps `main`'s cycles costs +365 placed cells after churn; the part had
about 360 free before the flash controller and has five fewer with it back, so this shape does
not place at 5,285 against 5,280. The owner's direction is to finish the fetch refactor on this
branch first and spend a single trim pass against the tree Stage B leaves, rather than shave five
cells from a shape only Stage B's own deletions (about 200 cells, ADR-0205) will make room for
anyway. `make soc-timing` on this commit, applied to current `main` (474bb03) alone, is the
number that trim pass starts from — see Verification below. Until it lands, `soc-timing` stays
red on every branch carrying this commit; that is expected, not a regression to chase here.

**On ECP5 the direction as measured buys nothing**, so there is no clock reason to carry the
shape on that part either: the decoupling that moves the ECP5 period is the registered head, and
its price is a second window register and a second cycle per unguessed redirect — the shape
Stage A built.

Not built, and named: a 48-bit skid (`w[p]` and `w[p+1]`'s low half, with `w[p+1]`'s upper half
read off the ROM when it is at `p+1`) saves about 32 cells at the cost of a bubble when a guess
has moved the ROM and decode's `next_instr` guess reads garbage; a guess restricted to
uncompressed encodings saves about 25; neither reaches the budget on its own, and neither is
needed once Stage B's deletions are in hand.

## Verification

The first pass (2026-09-21, over the patched tree) ran `make test-units`, `make cycles` (the
suite), `components_pcloop`, `components_traps`, `make -C formal remeasure-fg`, `make lint`,
`test/stall_sites_test.py`, `test/comment_density_test.py` and `make dhrystone`, and deferred the
generated riscv-formal set, `imemcheck`, `make cosim-suite`, `make mutation-check` and
`make dual-smoke` until the shape shipped somewhere those checks read. This amendment
(2026-09-23) moved the shape into `rtl/` and ran the deferred five, plus every gate `make test`
already carries, on the tree that now ships it:

| gate | result |
|---|---|
| `make -C formal check` | 86/86 generated riscv-formal checks pass; matches `EXPECTED_FAIL` (empty) and `EXPECTED_CHECKS` (86) |
| `make -C formal imemcheck` `imemcheck_cover` | both PASS; depths 15 (imemcheck, ≥ F+2=8) and 20 (dmemcheck, ≥ F+G+2=14) clear `check-memcheck-depth.py`; cover reaches its statement at step 6 |
| `make cosim-suite` | 69/75 agree; the divergence list matches `test/COSIM_EXPECTED_FAIL` exactly |
| `make mutation-check` | PASS; 11 mutations, each caught by exactly the detectors `test/MUTATION_DETECTORS` pairs with it |
| `make dual-smoke` | OK; both harts retire together, the held-hart-1 mutant correctly reports nothing observed on hart 1 |
| `make test` | PASS; suite 75/75 (see uart.S note below), every repo-scan target, `make probe-gates` |
| `make lint` | clean, both RVFI passes |
| `make elaborate-strict` | clean |

**One floor moved, and it is the same one this ADR already characterized before landing in
`rtl/`**: `uart.S` retires 1,336 against its prior floor of 1,381 (`test/OBSERVED_FLOOR`, updated
in this change) — the poll loop's exit branch is a guessed-taken backward branch, a mispredict
costs one cycle every frame, and fewer polls complete in the fixed window. Every other floor
holds.

`make soc-timing` on this branch, which is current `main` (474bb03) plus this shape and no other
RTL change: **`ICESTORM_LC: 5285/5280` (100%), placement fails** ("Failed to expand region").
That is the number the owner's later trim pass starts from — five cells, not the ADR's earlier
+365-to-+418-against-`main` estimate restated, since it is now a direct placement rather than a
throwaway measurement off a patched tree.

## Amendment: the guess was timed for Stage A, and Stage B never re-timed it

**A predictor that only ever costs cycles is not a predictor with a bug in its target
computation — it is one instrumented, resolved, and torn out**, and the trace below is what
proves the mechanism, not the effect: the loop test on the unfixed guess drives `fetch=200`
against the disabled guess's own `fetch=100`, a doubled cost measured with the guess itself
switched off as the control.

**Bisection on the guess's own toggle, `make cycles` (SUITE column), off vs on, at each stage
of the D/X split this ADR's shape stacks under:**

| commit | shape | guess off | guess on | delta |
|---|---|---|---|---|
| 28f929f | Stage A (this ADR, fused decoder) | 39,976 | 39,827 | **-149, guess helps** |
| 1009c0f | B1: split decode into D and X | 44,138 | 44,620 | **+482, guess costs** |
| 1b95010 | B2: forward X/M into X | 30,572 | 31,061 | +489, guess costs |
| b587659 | B3: delete the region wait | 30,264 | 30,753 | +489, guess costs |
| 25cd444 | this branch, unchanged fetcher.v | 30,264 | 30,753 | +489, guess costs |

The regression starts exactly at B1 (ADR-0208), the commit that moved branch resolution from D
into X, one cycle later than where this ADR's guess was timed for.

**The mechanism.** The guess is formed one cycle before the guessed branch is even decoded (off
`next_instr`) and applied to `fetch_word` during the single cycle that branch is at decode
(`guess_valid <= candidate && ... on issuing`, cleared the very next cycle regardless of
anything else). In Stage A's fused decoder, decode resolved the branch in that same cycle, so
the guessed word landed in the ROM exactly one cycle before `pc` needed it — the ROM's own
latency. B1 moved resolution into X, one stage later: `pc` still advances on D's own naive
`predicted_pc` (`pc + 2`/`+4`) in the interim, and the fetcher's `fetch_word` recomputes fresh
from `word + hit` every cycle with no memory of an earlier guess. The guessed prefetch is
consumed by nothing, sits in the ROM for exactly one cycle, and is overwritten by that plain
recomputation the cycle before `pc` ever reaches the guessed target — a `redirect` that would
have cost one miss unguessed now costs two, because the guess also evicts whatever the ordinary
sequential run-ahead would have had ready for the intervening cycle.

Proved on a hand-traced waveform (`$display` added and removed, not committed) of a 100-iteration
counted backward branch (`test/asm/btfnloop.S`, aligned so the branch's own fall-through crosses
a fetch window): `redirect` and `guess_valid` both fire as designed, `rom_addr` is steered to the
target for exactly one cycle, and is stomped by the plain `word + hit` recomputation the very
next cycle, before `pc` ever redirects there — reproducing the miss twice, at `fetch_stall`, in
the RTL itself, not merely inferred from the aggregate.

**The fix.** X already computes `redirect`/`redirect_target` — resolved exactly one cycle before
`fetch_pc_next` selects it, the same lag `rtl/littlecpu.v` (and every formal harness composing
the real topology) already wires between the decoder's `out` and the executor's `redirect` — so
the fetcher needs no guess at what X will decide; it already knows, one cycle ahead, which is
precisely the ROM's own latency. `rtl/fetcher.v` takes a new `redirect_target` input and
`fetch_word` steers off `redirect`/`redirect_target` directly, unconditionally (no `hit` gate: a
pending redirect is never wrong to act on). This deletes the whole `next_instr`-based heuristic —
candidate detection (`n_jal`/`n_branch`/`n_cj`/`n_cb`), four immediate decoders, and the guess
register — and, being wired to the resolved value rather than a heuristic, is unconditionally
correct for every redirect (forward branches, `jalr`, traps, `mret`), not just backward branches
and `jal`. `formal/pcloop.sv` and `formal/traps.sv` wire the new port the same way
`rtl/littlecpu.v` does; the composed proof reaches a new cover, `fetcher.redirect_served`
(a redirect immediately followed by a hit), in place of the deleted `guess_taken`.

**Measured, fixed vs the two rows this ADR already carried:**

| build | suite (`make cycles`, 77 comparable programs) | Dhrystone (2000 runs) | DMIPS/MHz | CoreMark (100 iter, 16 KB ROM) |
|---|---|---|---|---|
| predictor off | 30,264 | 1,392,021 | 0.777 | 2.425 |
| predictor on (broken) | 30,753 | 1,394,022 | 0.777 | 2.414 |
| **fixed** | **29,018** (30,338 with the new regression counted in) | **1,234,021** | **0.922** | **2.661** |

Dhrystone's own `fetch` stall column reads **0** for the entire run — every redirect the
benchmark takes is served with no miss. CoreMark still carries a residual, `fetch=37,219` of
37,580,102 cycles (0.10%), small next to `hazard` (5,011,241) and `lsissue` (6,893,500) and not
chased here.

**Area moved the same direction, unasked.** Deleting the heuristic rather than re-timing it
shrinks the design: `make fit` (core alone) reads **4,497** `ICESTORM_LC` against this branch's
prior 4,617–4,636 (`FIT_MAX_LC` is 4,219; both numbers are over it, a standing, expected-red
state this ticket's own trim pass is for — not something this change closes by itself, and not a
regression this change introduces). `make soc-timing`'s utilisation line moves from
**5,463/5,280 (over capacity, does not place)** to **5,277/5,280 (99%, places)** — the SoC now
fits the part's logic cells; its Fmax still reads under the 12 MHz requirement (9.53 MHz),
unrelated to this change and left for the trim pass.

**Regression.** `test/asm/btfnloop.S`: a 100-iteration counted backward branch, deliberately
aligned so the branch's own fall-through instruction crosses a fetch window — the exact
condition the broken guess doubled a miss under. `test/OBSERVED_FLOOR` and
`nano/asm/LITTLECPU_FLOOR` both carry its retire floor (nano decodes the same program at 1,214
retires against littlecpu's 1,213 — one extra, not investigated, unrelated to this fix).
`test/fetcher_tb.v` is rewritten for the new interface and timing (X's resolution now lags
`issuing` by a register, mirrored in the testbench's own scripted decode) and cannot even build
against the pre-fix `fetcher.v`, which had no `redirect_target` port — the interface change is
itself the forced-red boundary.

**Verification, this amendment.** `make test` (full, including the rebuilt `test/fetcher_tb.v`
and the two new manifest lines), `make lint`, `make -C formal components_decoder
components_executor components_traps components_pcloop` (all four k-induction proofs pass, the
new `fetcher.redirect_served` cover reached at step 2), `make -C formal remeasure-fg` (F = 5,
G = 5, both reproduce — unchanged, since the fix deletes logic rather than adding a pipeline
stage).

## Amendment: a D-stage guess was tried again, on top of the fix above, and declined

The trim pass this ADR still owes was approached from two directions, in one working session, on
`main` (474bb03) plus the fix above: a D-stage BTFN/jal predictor feeding `rtl/fetcher.v`'s
`fetch_word`, and a register on the timer interrupt's own path into X. Neither survived measurement
against the shape this ADR already ships.

**The D-stage guess.** The premise — that the fix above left `rtl/fetcher.v` with no guess at all
and needed one back — misreads what the fix above did: `redirect`/`redirect_target` already give
`fetch_word` the ROM's own one-cycle lead with **zero** mispredict cost, correct for every redirect
class. A second, heuristic guess (`predicted_taken`/`predicted_target_low`, D's own class-flag and
immediate decode, narrowed to 8 bits to place at all) was built beside it anyway, at real cost: it
took several widths and a mispredict-handling pass to reach parity with this ADR's own placed
count (5231–5300 against 5277), the 8-bit field's zero-extension made most of its guesses land
inside the first 256 bytes of the 8 KB ROM only, and Dhrystone read no better than "guess off" —
the field's real-world benefit was statistically indistinguishable from having no D-stage guess at
all. It added a struct field, a second immediate decode, and formal/test surface
(`formal/pcloop.sv`, `formal/traps.sv`, `test/fetcher_tb.v`, `test/decoder_tb.v`,
`test/zkt_isolation_test.py`) for no measured win over the fix this ADR already ships. Declined;
the tree reverted to this ADR's own committed shape (`git checkout 7382201 -- <files>`, confirmed
by an empty `git diff --stat` against it) rather than carrying a second predictor that duplicates
the first's job at a higher cost.

**The interrupt register.** B3 (ADR-0214) moved the timer interrupt's take into X, reading
`interrupt_pending` live and combinationally from `csrs.v`'s `irq_timer && mie_mtie && mstatus_mie`
— one hop from `mtip`'s own flip-flop to X's `redirect`, itself one hop from `fetch_pc`'s flip-flop.
A registered `interrupt_pending` (`interrupt_pending_reg`, one flop in `rtl/littlecpu.v` between
`csrs`'s output and `executor`'s input) was proposed as a fix for exactly this hop, since
`test/timer_tb.v`'s own record already allows the take a cycle late. The register is spec-legal and
was formally re-verified: `formal/traps.sv`'s reference model (`dx_is_interrupt`) needed the
identical one-cycle delay to stay in step with the DUT, and once matched `components_traps` closes
by k-induction the same as without it. But **measured against this ADR's own committed baseline,
not against the pre-B1 fused decoder or any other tree**, the register does not pay for itself:

| build | `make fit` | `make soc-timing` (ICESTORM_LC) | icetime | critical path |
|---|---|---|---|---|
| this ADR's shape (no register) | 4,497 | 5,277/5,280 (99%) | 9.53 MHz | `por_done → imem.rom_even.RDATA[2]` |
| + `interrupt_pending_reg` | — | 5,300/5,280 (100%) | 9.48 MHz | `por_done → riscv.decoder.out[1]` |

The register costs +23 placed cells and a cycle of interrupt latency, and Fmax moves the wrong way
by 0.05 MHz — inside the churn band, so not even a clear loss, but not a measured win either. Both
rows' critical paths start at `por_done` (the power-on-reset release) and end deep in decoder
logic; neither runs anywhere near `mtip`, `interrupt_pending`, or `csrs.v`. **The hypothesis this
register was built to test — that B3's move put the timer interrupt on the SoC's critical path —
does not hold at this pinned seed.** A single pinned-seed placement at 99–100% occupancy is not a
reliable read of which hop is slow (this file's own curve shows the same design's critical path
moving between unrelated cells on a one-cell area change), but the register had nothing to show for
itself even so: no Fmax gain, real area and latency cost. Declined for the same reason as the
D-stage guess — reverted alongside it, restoring this ADR's own committed shape exactly.

**What both declines leave standing.** `make fit` (4,497) and `make soc-timing` (5,277/5,280,
9.53 MHz) are unchanged from this ADR's own prior amendment — the trim pass this ADR still owes is
still owed, and neither direction tried here closed it. `make -C formal remeasure-fg` still reads
F = 5, G = 5. `make test`, `make lint`, and all four component proofs
(`components_decoder`/`components_executor`/`components_traps`/`components_pcloop`) pass on the
exact tree this ADR ships — reconfirmed fresh rather than inherited, since two directions were
tried and backed out in the same session. `make dhrystone`/`make coremark` are not re-taken here:
the RTL is byte-identical to what this ADR's prior amendment already measured (0.922 DMIPS/MHz,
2.661 CoreMark/MHz), confirmed by an empty diff against 7382201 on every file but one test
program's comment.
