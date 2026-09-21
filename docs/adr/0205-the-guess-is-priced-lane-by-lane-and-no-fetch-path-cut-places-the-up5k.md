# 0205 — The guess is priced lane by lane, and no fetch-path cut places the up5k

Status: Accepted. 2026-09-20.

## What this is

ADR-0201 shipped the fetch-side guess at about +577 `ICESTORM_LC` against a brief that budgeted
+45, and left `make fit` tripped at 5,274 of the part's 5,280. This ADR is the trade curve behind
that number — every lane, the straddle, the one-turn bookkeeping and the queue depth, each built and
measured rather than counted — and the decision it forces: two spellings that cost no cycle ship,
everything with a cycle price stays, and the up5k does not place under any of them, because the
decoupled fetch itself does not fit the part with the guess deleted outright.

## How it was measured

Every row is one configuration built as a throwaway edit and taken on one machine, one toolchain
(Yosys 0.68+48, nextpnr 0.11-1-g62e659ed), one session: `make fit` for the core alone, and the SoC
synthesised by `make soc.json`'s own script and handed to nextpnr at seed 1 with
`soc/littlesoc.pcf`, reading the packed `ICESTORM_LC` off its utilisation table whether or not the
placement then succeeds — it never did above 5,280, and nextpnr prints the table before it says so.
Dhrystone is `make dhrystone` at `DHRY_RUNS=2000`, CoreMark `make coremark` at 100 iterations, both
from the same `STALLS` line that gives `guess=` and `mispredict=`. The SoC column is the one that
has to fit; `fit` is kept beside it because the two disagree by up to a hundred cells on the same
edit, in both directions, and a per-module census attributes nothing after flatten. Both instruments
carry a churn band of about ±50 cells, so a row inside that band of its neighbour is a null, not a
saving.

Two reference trees were placed the same way, so the budget is a measurement on this toolchain and
not an inherited number: `origin/main` reads **4,836** and places at 13.24 MHz; the Stage A1 head
reads **5,421**. The SPI flash controller is being restored to `main`'s SoC on a separate branch, at
about +85, so the free space the fetch path is working into is about 360 cells, not 444.

## The curve

Cells are placed SoC `ICESTORM_LC` at seed 1 (with `make fit` beside it); the part holds 5,280.
Cycles are per Dhrystone; CoreMark/MHz is at 16 KB of ROM as always. Mispredict is
`mispredict / guess` on each benchmark.

| configuration | SoC LC | fit LC | cycles/Dhry | CoreMark/MHz | mispredict Dhry / CM |
|---|---|---|---|---|---|
| `origin/main`, no queue, no guess | 4,836 | — | ~788 | 2.155 | — |
| A1 head, queue only | 5,421 | — | — | — | — |
| guess off (`predict_found = 0`) | 5,489 | 4,734 | 1,001 | 1.712 | — |
| **all four lanes, as shipped by ADR-0201** | **6,069** | **5,274** | **820** | **1.955** | **3.0% / 7.8%** |
| lane 0 dropped (`sel_l0`, `sel_w0`) | 6,012 | 5,230 | 825 | 1.884 | 3.1% / 6.9% |
| lane 1 dropped (`sel_l1`, `sel_s1`) | 6,067 | 5,204 | 838 | 1.909 | 3.4% / 7.5% |
| lane 2 dropped (`sel_l2`, `sel_w1`) | 6,089 | 5,209 | 913 | 1.838 | 3.0% / 8.9% |
| lane 3 dropped (`sel_l3`) | 6,124 | 5,232 | 828 | 1.934 | 3.2% / 8.4% |
| straddle dropped (`sel_s0`, `sel_s1`), `pop2` kept | 6,011 | 5,166 | 880 | 1.907 | 2.3% / 6.9% |
| straddle dropped and `pop2` tied low | 5,887 | 5,128 | 880 | 1.907 | 2.3% / 6.9% |
| `pair_base` off `stolen_pc`, `fetch_addr_d1` deleted | 6,003 | 5,200 | 820 | 1.955 | 3.0% / 7.8% |
| resolve compare on `pc[3:1]` alone | 6,008 | 5,164 | 820 | 1.955 | 3.0% / 7.8% |
| both of the above, as throwaways | 6,004 | 5,135 | 820 | 1.955 | 3.0% / 7.8% |
| **both, as the text that ships** | **5,963** | **5,154** | **820** | **1.955** | **3.0% / 7.8%** |
| both, plus `redirect_target_reg` deleted (reads decode's `pc`) | 6,026 | 5,136 | 820 | 1.955 | 3.0% / 7.8% |
| both, plus mispredict from `jal`/`branch_taken` rather than the target compare | 5,927 | 5,050 | 820 | 1.955 | 3.0% / 7.8% |
| both, plus straddle dropped and `pop2` tied low | 5,779 | 4,983 | 880 | 1.907 | 2.3% / 6.9% |
| `redirect_target_reg` deleted alone | 6,118 | 5,194 | 820 | 1.955 | 3.0% / 7.8% |
| redirect/mispredict compared before the mux rather than after | 6,158 | 5,265 | 820 | 1.955 | 3.0% / 7.8% |
| `pop2` from decode's own sequential compare, no second incrementer | 6,052 | 5,273 | 820 | 1.955 | 3.0% / 7.8% |
| three-word queue, `room` at one committed word | 6,006 | 5,150 | 1,177 | 1.443 | 3.0% / 7.8% |
| guess off, `redirect_target_reg` deleted | 5,532 | 4,691 | 1,001 | 1.712 | — |

What the rows say:

- **The lanes are not where the cells are.** One immediate mux and one pair of adders serve every
  position, so a lane is its own decode and a mux arm: dropping any one moves the SoC by −57 to
  +55, inside the churn band, and `fit` by 40 to 70. Lane 2 is the only lane whose cycles matter
  (+93 per Dhrystone without it, the step ADR-0201 measured as 108 on the way in); the other three
  cost 5 to 18. There is no per-lane trade to make: every lane is cheaper than the band it is
  measured in and buys real cycles.
- **The straddle is 60 cycles per Dhrystone for about 60 cells, and the double pop behind it is a
  further 120 in the SoC.** Dropping both is the largest cut on the curve that keeps a guess at all
  (5,887) and the only one that pays in cycles (880, the every-lane step of ADR-0201's own table).
  `pop2`'s cost is not its incrementer — re-spelling that from decode's sequential compare is a null
  at 6,052 — but the queue's second-word pop path and `fetcher.v`'s straddle test together.
- **Two spellings cost no cycle and ship.** `fetch_addr_d1` duplicated `stolen_pc` on every cycle a
  response is accepted, and `stolen_pc` was already there; the resolve compare needs three bits,
  not thirty-two. Together −65 SoC and −139 `fit` as throwaways, and **−106 SoC (5,963) and −120
  `fit` (5,154)** as the text that ships — the dead register gone from the source and the comment
  lines around it reorder what ABC is handed, which is the churn band doing what it does — with the
  retire stream identical on both benchmarks.
- **Three re-spellings are nulls and stay out.** Deleting `redirect_target_reg` reads +49 alone and
  +22 in combination: on this part a register whose D is a LUT output packs into that LUT's cell,
  so a deleted register frees nothing unless its input was another register. Comparing before the
  mux is +89. The `pop2` re-spelling is −17.
- **The three-word queue is declined on cycles, not cells.** Under the same `room` rule as the
  four-word one (`committed <= DEPTH - 2`) it launches a pair only when the queue holds at most one
  word and nothing is due, so a pair lands every third cycle and decode starves on uncompressed
  code: 1,177 cycles per Dhrystone, +44%, for −63 cells. A `room` rule that counts the cycle's own
  pop might recover part of that; it is a redesign of the launch condition, not a depth change, and
  was not built.

## The decision

`rtl/fetchctrl.v` reads `pair_base` off `stolen_pc` and no longer keeps `fetch_addr_d1`;
`rtl/decoder.v` resolves a guess on `fetcher_pc[3:1] == predicted_src_pc[3:1]`. Everything else on
the curve stays as ADR-0201 shipped it: four lanes, the straddle, `pop2`, the 32-bit target compare
and the four-word queue. The tree has one behaviour and no configuration knob — every other row
above was a throwaway edit, measured and reverted.

**Why `stolen_pc` is `pair_base`.** On every response a guess may commit off, `stolen_pc` holds
the address `fetch_pc` had the cycle before: the three arms that write it anything else each make
the next cycle's response one nothing reads `pair_base` for. After a steal `req_valid` is low for
two cycles; after a commit `waiting` is cleared, so it is low the next; after a redirect the
abandoned pair's response is still accepted and flushed, and on that one cycle — `redirect_apply_d1`
— `stolen_pc` already holds the new target, which is why `predict_commit` excludes it (the eighth
bug of ADR-0201) and why the fourth bug, fixed there by adding `fetch_addr_d1`, no longer needs the
register. `rtl/fetchctrl.v`'s `FORMAL` block asserts
`!req_valid || flush || stolen_pc == $past(fetch_pc)` — gated on the same named `flush` window
`predict_commit` itself excludes, so the property and the arm it certifies share one source of
truth — and `components_pcloop` proves it by k-induction; deleting the `stolen_pc <= fetch_pc` arm
makes it fail at once, which is its red direction.

**Why three bits resolve a guess.** While a record is outstanding decode is at or behind its source:
the candidate's pair is pushed behind at most the two words the queue held (`req_valid` requires
`queue_count <= 2`), the candidate sits at most six bytes into its pair, and decode's `pc` names the
queue's head word or a half of it — so `src - pc <= 14` in 32-bit modular arithmetic on every cycle
the record is live, except the one after a redirect, when `pc` has already moved and the record
clears on the next edge. Within fourteen bytes no two addresses share `pc[3:1]`. `formal/pcloop.sv`
states that bound as the modular difference — the first spelling, `src <= pc + 14`, went red on a
pair at the top of the address space, where `pc + 14` wraps to 2, which is arithmetic and not a
defect. It is not inductive over free queue contents, so `formal/pcloop.sby`'s `bmc` task checks it bounded
at depth 12 as a prerequisite of `components_pcloop`. Twelve, because every shape the check found
on the way here (both ends of the bound, the wrap, the odd `mtvec`) appeared by step 7, F is 10,
and the composed harness doubles its solve time per step past ten under either engine — `smtbmc
boolector` reached step 12 at 1.5 minutes, 14 at 6 and 15 at 12, and `abc bmc3` was slower still
(step 12 at 4.4 minutes) — against the pcloop CI job's twenty-minute wall, which the k-induction
that this task prerequisites also has to fit. `pcloop_cover` covers both ends of the bound (the
source reached at step 4, a source exactly fourteen bytes ahead at step 5). The
harness also assumes `mtvec[1:0] == 0` and `mepc[0] == 0`, which restates `rtl/csrs.v`'s own WARL
masks: with those two inputs free a trap could land `pc` on an odd byte, a state no CSR write
reaches, and the bound read the source one byte behind it. The full-width `predicted_src_pc` port
stays, so the harness can state the bound; the synthesised register keeps only the bits decode
reads.

## The up5k

**No row places, and the shortfall is not the predictor's.** The shipping tree reads 6,069 against
5,280, 789 over before the flash controller's 85 comes back. With the guess deleted outright the
SoC reads 5,489, still 209 over; with the flash controller back that is about 294, and Stage B's
planned deletions are about 200. So the decoupled fetch with no guess at all is still about a
hundred cells past the part after Stage B, before any margin — and `make soc-seed-search` refuses a
pin under 12.60 MHz, which a placement at the part's last hundred cells will not reach. The
predictor at its cheapest cycle-neutral spelling is +515 on top of that.

What would fit is a fetch queue smaller than the four-word one by at least a few hundred cells, with
no prediction — the cell count `main` places with, 4,836, leaves room for a fetch path of at most
about 360 plus Stage B's 200, and A1's queue alone is +585. The three-word shape measured here costs
44% of Dhrystone's cycles; the two-word skid ADR-0196 built was wrong on the first straddle; a
two-word array with a top-up push is unbuilt. On the up5k the clock this restructure buys is a step
function — 12 MHz is met on `main` and 24 is not reachable — so every configuration on this curve
is slower than `main` there (820 against 788 at best, 1,001 with no guess) for cells the part does
not have. That is the finding, stated as one: the up5k needs a different fetch path than the one
Stage A built, and no cut to the predictor changes it.

## Verification

Both shipped spellings are retire-identical to ADR-0201's tree on Dhrystone (820 cycles, 4,081
mispredicts of 134,731 guesses) and CoreMark (51,135,805 cycles, 232,061 of 2,960,656).

`components_pcloop` on the shipping tree: `pcloop_cover` reaches all four covers (the two new ones
at steps 4 and 5), `pcloop_bmc` passes depth 12 in 85 s with no trace, and the k-induction passes
basecase and induction in 16 s with the `stolen_pc` assertion in it. Both graders were then forced
red by hand and restored: with the `stolen_pc <= fetch_pc` arm replaced by a hold, the k-induction
fails at that assertion at step 3; with the bound narrowed to six bytes, `pcloop_bmc` finds the
fourteen-byte case at step 4.

The rest, on the shipping tree, from a real clone of riscv-formal at the pin inside the worktree:

| gate | result |
|---|---|
| `make lint`, `make elaborate-strict` | clean, both passes; no warning |
| `make -C formal imemcheck` / `imemcheck_cover` | PASS at full depth with the guess live, no trace; cover reached |
| `make -C formal components_decoder`, `components_traps` | PASS, basecase and induction, with their probes |
| `make -C formal remeasure-fg` | **not re-taken**: this change adds no stall reason and no stage and the retire stream is cycle-identical, so F = 10 / G = 8 stand as ADR-0201 measured them; the sweep was started and stopped after its `liveness_ch0` probe at bound 28 had not answered in thirty minutes on a machine at load 54 |
| `make test` | exit 0: 81/81, 43,575 cycles, `unattributed` 0, every `*-test` target and `probe-gates` green, the Zkt walk unchanged |
| `make cosim-suite` | 75/81 agree; the six divergences match `test/COSIM_EXPECTED_FAIL` exactly |
| `make mutation-check` | 11 mutations, each caught by exactly its paired detectors; tree restored |
| `make dual-smoke` | OK — two harts counted 32, one hart counted 16 |
| `make dhrystone` / `make coremark` | 820 cycles/Dhrystone, 0.694 DMIPS/MHz; 1.955 CoreMark/MHz — every `STALLS` field identical to ADR-0201's run |
| `make fit` | **RED: 5,154** against the 4,802 budget, −120 from 5,274, left tripped for ADR-0201's reason |
| SoC, seed 1 | **5,963 `ICESTORM_LC`, does not place**; `make soc-timing` says so and PIN STALE |
| `make ecp5-timing` | `DP16KD` 36, `TRELLIS_DPR16X4` 32, `MULT18X18D` 4, all as declared; no block-RAM reset driven by logic; 38.81 MHz at one placement against 39.49 on ADR-0201's text — inside the part's 10.3% placement spread, so a null at one seed |

The generated riscv-formal set (`make -C formal check`) is left to the stack's CI job, as ADR-0201
did: four of its checks did not terminate with the guess live on that tree and nothing here changes
what they see.
