# 0207 — A register-only fetch address over a one-window skid ships as the fetch front end

Status: Accepted. 2026-09-21, amended 2026-09-23. The RTL shipped first as a tracked patch
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
