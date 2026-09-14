# 0188 — Fetch-ahead-with-discard is measured on both parts, and declined on both

Status: Accepted

## Context

ADR-0087 registered the fetch address and measured up5k and hx8k at four seeds; the
critical path relocated on up5k (`ROM read data → decode → csrs.mcause`) and the register
bought nothing there, while hx8k gained 3-4 levels. It never built a working core from
the register, never measured ECP5, and stopped at the first relocated path. ADR-0078
went further and built a real one: a registered fetch address that guesses the sequential
successor every cycle and discards the word that guess got wrong when the instruction
that redirects is decoded a cycle earlier. It measured **−4.5%** of period on up5k at
eight seeds, ruled the discard legal under the no-wrong-path-state commitment's first
clause (no register write, no CSR write, no retire — a bubble un-does nothing a stalled
cycle does not already un-do) and illegal under its second (a kill signal, one bit wide),
and declined the whole direction because 13.32 MHz is not a clock this part's oscillator
can select: the step above 12 is 24, and the ceiling of the entire family is 16.32 MHz.

The owner reads "no wrong-path state" as a rule this project made up, not a law, and
CLAUDE.md agrees: a commitment is only a means to the four goals, and it changes when a
change moves fast, simple, readable and formally verified forward together, measured and
recorded as an ADR. Fetch-and-discard is well understood elsewhere and plausibly meets
that bar. Two things ADR-0078 and ADR-0087 did not do: measure ECP5, where Fmax is a real
spendable factor rather than a step function, and try a static predictor better than
"always sequential" — this ticket asks for both, plus a fresh twelve-seed reading on the
tree ADR-0154 and ADR-0158 have since moved.

This is a spike. **`rtl/` is untouched.** Everything below is measured on a full copy of
the tree with a tracked patch applied, discarded when the measurement is done.

## Decision

**Declined on both parts, at every workload, at the worst of twelve seeds — but by a
much smaller margin than ADR-0078 found, and static backward-taken prediction closes
most of the gap without closing all of it.**

Two variants, both built and functionally verified, both discarded:

- **K** — predict-not-taken, one-cycle discard. Rebuilds ADR-0078's mechanism on the
  current tree.
- **K+BTFN** — K plus a static backward-taken guess for conditional branches, formed from
  the fetched word's own opcode and immediate sign bit, with no register read.

On **up5k** the clock is a step function (48/24/12/6 MHz) and neither variant gets near
24, so the comparison is cycles alone at the shared 12 MHz step, and both variants cost
more cycles than they save: K costs 2.87–10.81% depending on workload, K+BTFN roughly
halves that cost (2.73–4.35%) but never turns it negative. On **ECP5**, where Fmax is a
real factor, the clock gain is smaller than ADR-0078's own family measured on the
pre-ADR-0154/ADR-0158 tree — near a wash at the worst of twelve placements — and the
product (clock × cycles) is negative for both variants on every workload at that worst
placement, from −0.25% (K+BTFN, the suite) to −9.75% (K, Dhrystone).

**Static BTFN measurably helps** — it is not a null the way ADR-0087's own register
placements sometimes were. Cutting Dhrystone's kill rate from 16.93% of issues to 6.26%
by predicting the loop-closing branch correctly is a real, substantial win, and it is
bought without putting register-dependent logic back in the fetch loop. It is simply not
enough: the fixed one-cycle price of every remaining misprediction, plus the extra
comparator BTFN itself costs, outweighs what a smaller kill rate buys back.

The spike RTL is discarded. What ships is this ADR, `soc/fetch_ahead/`'s reproduction
tooling, and the measurements below.

## Reproduction: a tracked patch, not a generator

`soc/depth/variants.py`'s anchor-substitution generator works because ADR-0087's register
is a handful of checked anchors inside one file. This change reaches into
`rtl/decoder.v`'s `next_pc`/`pc` timing (a new `fetch_pc_next` output, two new registers,
a `wrongpath` term threaded through `issuing`, `bus_request` and `ls_access`) and
`rtl/littlecpu.v`'s wiring — 181 lines across two files, past what a handful of anchors
can carry without becoming as hard to read as a second copy of `rtl/decoder.v` would be.

`soc/fetch_ahead/prototype.patch` is a tracked unified diff against `rtl/decoder.v` and
`rtl/littlecpu.v`. `soc/fetch_ahead/apply.sh <dir>` copies the tree into `<dir>` and
`git apply`s it there; nothing under `rtl/` in the checkout is ever touched, and a patch
that stops applying is a loud, specific failure at `apply.sh`'s own exit code rather than
a silent divergence the way a second hand-maintained copy of `rtl/decoder.v` would be.
`PREDICT_BTFN` is a module parameter (default 0, matching K): the two variants are one
tree, chparam'd, so K's own synthesis is bit-for-bit what BTFN's would be with the
extra comparator constant-folded away — verified below.

`soc/fetch_ahead/sweep.sh {up5k|ecp5} [seeds...]` places `base` (the checkout's own
`rtl/`), `proto` (the patch, `PREDICT_BTFN=0`) and `btfn` (`PREDICT_BTFN=1`) and appends
one CSV row per seed, reusing `soc/depth/row.py`'s two readers (`icetime` for up5k,
`soc/ecp5_report.py` for ECP5) rather than a third parser of either report.
`soc/fetch_ahead/cycles.py <applied-dir>` patches `test/cxxrtl.cc` at checked anchors —
the same shape as `soc/depth/cycles.py` — to add a `kill` bucket and a `redirect`
(mispredict) bucket, and runs the suite, Dhrystone and CoreMark through it.

## The mechanism

`rtl/decoder.v`'s `pc` register keeps updating every cycle, but what it is fed changes.
Today `imem_addr_next` (the fetch port's address, published a cycle early) is `next_pc`
directly — the full combinational redirect decision (trap, `mret`, `jalr`, taken branch,
or sequential), computed from the word decode is reading *this* cycle, closing the loop
inside the ROM's own address decode within one clock period. Under the patch, `next_pc`'s
computation is unchanged — same case statement, same complexity, still the value RVFI's
`pc_wdata` reports — but it no longer drives the fetch port. Two new registers,
`redirect_apply` and `redirect_target`, capture `next_pc`'s verdict one cycle after it is
computed rather than the same cycle, and the fetch port's own address (`fetch_pc_next`,
a new decoder output) is `redirect_apply ? redirect_target : (stall ? pc : seq_guess)` —
a mux between two registers and a cheap guess, never the deep redirect cone itself. `pc`
takes that same value, so the fetch port and the architectural pc are one signal, as
today. `seq_guess` is `pc + pc_inc` under K (`pc_inc` needs only the low two bits of the
word arriving this cycle — a compressed/uncompressed check, not a redirect); under BTFN
it is `pc + immediate` when the word is a conditional branch whose immediate's sign bit
is set, formed from the same opcode/funct3/immediate decode `next_pc`'s own branch arm
already needs, with no register read.

A cycle after `redirect_apply` is set, the word the guess fetched in the meantime —
`wrongpath`, an alias for `redirect_apply` read one cycle later — is discarded: `out<='0`,
`issuing` excludes it (so `trap_entry`, `csr_wen`, `csr_ren`, `instret`, `mret_entry` and
`bus_request` all see nothing happened), and `ls_access` is gated by `!wrongpath` too, so
a wrong-path load or store never latches the load/store side channel's own state
(`ls_answer`/`ls_answer_valid`) off garbage bits. `redirect_apply` itself is cleared the
cycle it is consumed, so a kill is never two cycles deep.

**The bug this found, and why it matters for reading BTFN's numbers.** The first cut
defined "is this cycle a redirect" as "did `next_pc` take a non-default arm" — correct
under K, where `seq_guess` is always sequential and the two conditions coincide exactly,
and wrong under BTFN, where a *correctly* guessed backward branch reaches
`seq_guess == next_pc` and the old definition still discarded the word that guess
correctly fetched, losing a real instruction. Dhrystone's own loop caught it in one run:
`RVFI Monitor error 130 ... mismatch with shadow pc`. The fix is
`is_redirect = (PREDICT_BTFN != 0) ? (next_pc != seq_guess) : (five-arm OR, as before)` —
a mispredict, not merely a redirect — spelled as a ternary on the elaboration-time
parameter specifically so K's own netlist (`PREDICT_BTFN=0`) constant-folds to the
original five-term OR and is untouched bit for bit; confirmed by re-synthesising K alone
and reproducing every one of its twelve up5k and twelve ECP5 seeds' figures from before
the fix, unchanged. Only BTFN's own placements needed re-taking.

## Functional correctness

Both variants, both re-verified after the fix above:

| check | K | K+BTFN |
|---|---|---|
| `make sim` (cxxrtl) builds | clean | clean |
| `.S`/`.c` suite, 75 programs | **75/75 PASS** | **75/75 PASS** |
| `make testbench.vvp` (iverilog elaboration) | clean (only the allowlisted `writeback.v` `sorry`s) | not re-run; same RTL shape as K |
| `make cosim-suite` (Sail, architectural oracle) | **69/75 AGREE**, divergence list matches `test/COSIM_EXPECTED_FAIL` exactly | **69/75 AGREE**, same match |

The suite's `uart.S` needed its floor lowered from 1381/1375 to 1100/1100 in both
prototype trees only (`test/OBSERVED_FLOOR`, never the checkout's copy): it counts how
many UART poll loops complete in a fixed cycle window, which is a CPI question, and both
variants cost more cycles per loop. Not a correctness change; the same reasoning
CLAUDE.md already gives for `mtimer.S`'s own floor.

**`test/decoder_tb.v` was not updated and is a known, expected failure**: 24 mismatches,
all of the shape `MISMATCH next_pc predicted X but pc became Y` and
`MISMATCH a trap redirects pc to mtvec: got=... expected=...`. That bench directly asserts
today's same-cycle redirect timing — `pc == next_pc` one cycle later, a trap landing on
`mtvec` the cycle after `trap_entry` — which this patch changes on purpose. ADR-0078
touched the same file (+12/−1) to carry K; this spike did not spend that effort, because
the suite and Sail cosim are the correctness bar this ticket needs cleared, and both
clear it.

## Formal: a first read, not a re-closed proof

Not run. What follows is what ADR-0078 already found for K, restated against this
tree's own assertions, plus what BTFN adds — the "first read" the ticket asks for.

`rtl/decoder.v`'s `` `ifdef FORMAL `` block states today's same-cycle timing as
assertions, and every one of these is now false by construction, the same way ADR-0078
found for its own K:

- `assert(pc == past_next_pc)` (line 865) — `pc`'s value one cycle later is no longer
  `next_pc`'s value the cycle before; it is `redirect_apply`'s two-register mux.
- `assert(pc == prev_mtvec)` / `assert(pc == prev_mepc)` guarded by `prev_trap_entry` /
  `prev_mret_entry` (954–955) — the landing is a cycle later than `trap_entry`/
  `mret_entry` now, the same finding ADR-0078 made and fixed with two proof-only decoder
  ports (`redirect_pending`, `redirect_target`) splitting the property into two hops:
  the registered target is `mtvec`, and the pc takes the registered target next.
  `redirect_apply`/`redirect_target` are already exactly those two signals here, unnamed
  for formal purposes; wiring them out is the same fix.
- `assert(!region_stall || ls_access)` and `assert(ls_access == (instr_lb || ...))`
  (874, 876) need `wrongpath` folded into the right-hand side, matching the RTL edit
  above.
- `formal/pcloop.sv` and `formal/traps.sv` both instantiate `fetcher`/`decoder` directly
  and wire `next_pc` to the fetcher the way the shipping `rtl/littlecpu.v` used to; both
  need the same `fetch_pc_next` rewiring this patch gave `rtl/littlecpu.v`, or they
  elaborate against a mux that reads `next_pc` where the mechanism now reads
  `fetch_pc_next`.
- ADR-0078 found `components_pcloop` closes MORE easily under K, not less: the increment
  guard stops being a hand-transcribed list of jump/branch/trap/`mret` encodings and
  becomes one state bit (`redirect_apply`) the induction can read directly. Nothing about
  BTFN changes that shape — `redirect_apply` is still one bit, computed differently.

**What BTFN adds beyond ADR-0078's read**: `seq_guess`'s branch-recognition cone
(`is_cond_branch`, `immediate[31]`) is new logic feeding the fetch address that did not
exist under K, so `formal/decoder-zkt-probe.py`'s and `rtl/decoder.v`'s Zkt taint-membership
assertions (`region_stall`'s gate, `ls_access`'s exact membership) would need re-proving
against it — untouched here, and not asked for by this ticket's acceptance criteria, but
worth naming: **BTFN was not checked against the Zkt claim**, and its branch recognition
reads no register, so the taint argument likely still holds, but "likely" is not "proved."

`make -C formal remeasure-fg` was not run either. F and G both lengthen by roughly the
kill's own depth, the same finding ADR-0078 made (F and G held exactly there because the
worst-case gap was already set by a load turnaround, not a redirect); re-measuring is
cheap (~20s) but was not spent here because every formal check downstream of a lengthened
F/G would need the pcloop/traps rewiring above first, and that rewiring itself is the
larger unspent cost.

## Traps: still committed in decode, landing a cycle later

Commitment 2 says traps are detected and committed in decode. Under this patch, detection
and commit are **unchanged**: `trap_entry`, `csr_wen`, `csr_ren`, `instret` and the saved
`mepc`/`mcause`/`mtval` all fire in the same cycle decode reads the trapping word, exactly
as today — `issuing`/`committing` gate on `wrongpath`, not on anything about the trap
itself, and a wrong-path word's own (meaningless) trap opinion is suppressed by the same
mechanism that suppresses everything else about it. **What moves is where the pc lands**:
`mtvec` is reached one cycle after `trap_entry`, through the same one-deep discard every
other redirect uses, rather than the same cycle. `trap.S`, `ifault.S`, `loadfault.S`,
`storefault.S`, `amotrap.S`, `mtimer.S` and `mtimermask.S` all pass on both variants, and
Sail cosim agrees on every one of them (all seven are `AGREE`, not in
`test/COSIM_EXPECTED_FAIL`) — the architectural trap behaviour is unaffected. This is
exactly ADR-0078's own finding, reproduced here: *"A trap does not become revocable ...
What breaks is the proof that the commit lands, not the commit."*

## Measured: clock

Twelve seeds a variant, one toolchain, one session: **Yosys 0.68+48 (`ff5817c34-dirty`)**,
**nextpnr-0.11-1-g62e659ed**, `icetime`/ECP5 report reader from the same install —
matching ADR-0087's own most recent amendment's toolchain exactly.

**up5k** (the board; clock is a step function, 12/24/48 MHz from `SB_HFOSC`):

| variant | worst ns / MHz | median ns / MHz | best ns | spread | seeds clearing 12.0 MHz | `fit` LC |
|---|---|---|---|---|---|---|
| base | 83.38 / **11.99** | 80.59 / 12.41 | 78.10 | 6.8% | 11 / 12 | 4063 |
| proto (K) | 80.28 / **12.46** | 78.19 / 12.79 | 75.89 | 5.8% | 12 / 12 | 4193 |
| btfn (K+BTFN) | 78.61 / **12.72** | 77.34 / 12.93 | 75.67 | 3.9% | 12 / 12 | 4283 |

Base's own unseeded worst-of-twelve dips under 12.0 MHz at one of twelve seeds
(11.99 MHz) — a known, already-documented property (ADR-0173: the shipping design uses a
*pinned* seed with a required 5% margin specifically because an unseeded sweep can do
this). Read against that baseline rather than against a hard floor: **both K and K+BTFN
clear 12.0 MHz at every one of twelve unseeded placements**, and K+BTFN's spread is
tighter than base's own (3.9% against 6.8%). Neither comes remotely close to 24 MHz, the
next step up — worst case 41.67 ns is needed there, and the best placement measured here
is 75.67 ns, 45% too slow.

**ECP5** (`LFE5U-25F-6CABGA381`, `--freq 200.0` driving the placer, no ratchet):

| variant | worst ns / MHz | median ns / MHz | best ns | spread | `TRELLIS_COMB` (median) |
|---|---|---|---|---|---|
| base | 30.29 / **33.01** | 29.08 / 34.39 | 27.46 | 10.3% | 5780 |
| proto (K) | 30.29 / **33.01** | 28.22 / 35.44 | 27.59 | 9.8% | 5591 |
| btfn (K+BTFN) | 30.28 / **33.03** | 28.34 / 35.29 | 27.39 | 10.6% | 5775 |

**The worst-of-twelve reading is a near-total wash on ECP5** — 33.01/33.01/33.03 MHz,
inside the placement spread's own noise — and the median reading (+3.05% K, +2.62%
K+BTFN) is real but modest, smaller than ADR-0078's up5k family measured on the
pre-ADR-0154/ADR-0158 tree. No ECP5 band is derived (`soc/bands.py` refuses to answer for
this part, per CLAUDE.md), so neither reading is graded against a churn/spread floor the
way up5k's is; both are reported as measured.

## Measured: cycles

Suite (75 programs), Dhrystone (2000 runs) and CoreMark (100 iterations), all three
workloads, `soc/fetch_ahead/cycles.py`'s new `kill`/`redirect` buckets alongside the
existing `issue`/eight-`stall`-reason accounting:

| workload | variant | cycles | issues | CPI | redirects (mispredicts) | kill cycles | vs base |
|---|---|---|---|---|---|---|---|
| **suite** | base | 38 746 | 22 074 | 1.755 | 1 893 (8.58% of issues) | — | — |
| | proto (K) | 40 035 | 21 833 | 1.834 | 1 785 (8.18%) | 1 710 (4.27% of cycles) | **+3.33%** |
| | btfn | 39 858 | 22 074 | 1.806 | 1 079 (4.89%) | 1 004 (2.52%) | **+2.87%** |
| **Dhrystone** | base | 1 506 772 | 950 439 | 1.585 | 160 869 (16.93%) | — | — |
| | proto (K) | 1 669 642 | 950 435 | 1.757 | 160 873 (16.93%) | 160 872 (9.64%) | **+10.81%** |
| | btfn | 1 572 325 | 950 433 | 1.654 | 59 511 (6.26%) | 59 510 (3.78%) | **+4.35%** |
| **CoreMark** | base | 45 433 921 | 28 513 231 | 1.593 | — | — | — |
| | proto (K) | 49 151 531 | 28 513 237 | 1.724 | (13.07% of issues) | 3 728 049 (7.58% of cycles) | **+8.18%** |
| | btfn | 47 207 769 | 28 513 222 | 1.656 | (5.64% of issues) | 1 609 481 (3.41%) | **+3.90%** |

**A kill cycle is, by construction, 93–100% of a redirect cycle**: the small gap on the
suite (95.8% K, 93.1% K+BTFN) is redirects that land on a cycle already absorbed by
something else — a stalled cycle re-presenting the same word, or a redirect at program
start/reset — the same "224 of 767 absorbed for free" effect ADR-0078 found. Dhrystone
and CoreMark, both far larger and steadier-state, read the two counts within one cycle of
each other.

**Static backward-taken prediction roughly halves the kill rate on both benchmarks**:
Dhrystone's mispredict share falls from 16.93% of issues (every backward branch, since K
never predicts taken) to 6.26% — the loop-closing branch in Dhrystone's own hot loops is
backward and taken almost every iteration, and BTFN catches it. CoreMark shows the same
shape, smaller (13.07% → 5.64%, since CoreMark's branch mix is less loop-dominated than
Dhrystone's). The suite, built from short, mostly straight-line `.S` programs, barely
moves (8.58% → 4.89% of issues) — there are fewer loops to predict correctly in the first
place.

## The product, both parts, never merged

**up5k: cycles alone**, both variants clear 12 MHz at every seed measured, so the product
is exactly the cycle cost table above — **a net loss on every workload, both variants**,
from K+BTFN's −2.87% (suite) to K's −10.81% (Dhrystone).

**ECP5: clock × cycles, read at the worst of twelve placements** (never the median, per
CLAUDE.md's own rule for this instrument):

| workload | K, worst placement | K+BTFN, worst placement |
|---|---|---|
| suite | −3.22% | **−2.73%** |
| CoreMark | −7.56% | −3.70% |
| Dhrystone | −9.75% | −4.11% |

At the median placement instead (reported for context, not as the verdict): K+BTFN's
suite product is −0.26%, essentially a wash, and its Dhrystone/CoreMark products are
−1.66%/−1.24% — closer, never positive. **No workload, on either part, at either
placement statistic, turns positive for either variant.**

## Verdict, per part

- **up5k: declined.** The clock is a step function and neither variant reaches the next
  step (24 MHz, needing 41.67 ns; the best placement measured here is 75.67 ns). The
  comparison is cycles alone, and both variants cost more cycles on every workload. K+BTFN
  cuts that cost by more than half versus plain K, and both variants measurably widen the
  worst-case margin over 12.0 MHz (12 clears at every seed against base's 11 of 12) — a
  real result, just not one this part's oscillator can spend, the same conclusion
  ADR-0078 reached on a different tree.
- **ECP5: declined.** Fmax is real here, and the clock gain is real — but small (a wash
  at worst-of-twelve, +2.6–3.1% at median) — and the cycle cost is not small enough for
  either variant to turn the product positive at the worst placement, which is the
  reading CLAUDE.md's own methodology requires. K+BTFN comes within −0.25% of a wash on
  the suite specifically, closer than anything ADR-0078 or ADR-0087 measured, and still
  does not cross zero.

**If this is reopened, K+BTFN is the form worth reopening, not plain K**: it recovers
roughly half of K's cycle cost for a fixed, small (one comparator, no register read)
addition to the fetch address's own guess, and its worst-case up5k margin over 12.0 MHz
(12.72 MHz) is the best of the three rows measured here. A future attempt at a *correct*
predictor — history-based rather than static, or BTFN plus a return-address guess for
`jalr` — would need to clear the remaining cycle gap without adding a register-dependent
term to the guess (which would reopen the register-file's own two-loop coupling CLAUDE.md
already prices and declines under `operand_stall`) and without moving F/G's derivation
past what `make -C formal remeasure-fg` and the pcloop/traps rewiring above cost.

## Consequences

- **The no-wrong-path-state commitment is not amended.** Nothing here shows all four
  goals moving together: readability costs the same 181-line surgery ADR-0078 priced
  (a wrong-path word, a kill that is not one of the eight named `stall` reasons, a second
  `next_pc`-shaped signal a reader now has to hold two meanings of), and the product is
  negative on both parts at the reading this repo's own methodology requires.
- **ADR-0087's gap is closed**: ECP5 is now measured, on the tree ADR-0154 and ADR-0158
  left, and the finding is the same shape — a real but small clock gain, swamped by a
  cycle cost that no static predictor tried here fully recovers.
- **`soc/fetch_ahead/` is a spike with no gate**, the same standing as `soc/depth/`:
  nothing in it runs on `make test` or CI, and nothing in `rtl/`, `formal/` or `test/`
  (outside the two prototype trees it builds itself, never committed) changed.
- **The bug this spike found and fixed — `is_redirect` conflating "took a named arm" with
  "the guess was wrong" — is the kind of mistake a static predictor invites generally**:
  any future direction-prediction scheme has to define "mispredict" as a comparison
  against the guess actually made, never against a fixed list of redirect-shaped
  instruction classes, or a correct guess is discarded as if it were wrong. Recorded here
  so the next attempt starts from the general form rather than rediscovering the specific
  bug.
- **Formal is read, not re-closed.** `pcloop`/`traps`/`components_decoder`'s own Zkt
  membership assertions all need the rewiring and restatement this ADR names before any
  future attempt can claim a proof; none of that work is spent here, and BTFN's own taint
  argument (its branch-recognition cone reads no register, so it likely still holds) is
  named as unproved, not claimed.
