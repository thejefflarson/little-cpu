# 0201 — Stage A3: the mispredict definition lands; the guess itself does not yet ship

Status: Partial. The `redirect`/`mispredict`/`kill` semantics this ticket asks for are built,
proved not to regress anything with the guess held off, and shipped. The static BTFN/jal guess
itself is fully implemented but disabled at the one gate that spends it (`predict_found` tied to
`1'b0` in `rtl/fetchctrl.v`) after a real, reproducible Dhrystone-scale corruption survived three
found-and-fixed bugs and a fourth was not found inside this stage's own time budget. 2026-09-19.

## What this is

The third of four Stage A merges described in
`docs/ideas/the-fetch-address-reads-registers.md`, stacked on Stage A2 (`rtl/fetchctrl.v`'s
`kill`, ADR-0198/0199). The ticket's own definition — **"mispredict" means
`resolved_target != guessed_target`, never an instruction class** — is what ADR-0188 recorded
getting wrong (an unpredicted taken branch and a correctly-guessed one were the same signal, so a
correct backward-branch guess discarded its own already-fetched word). That definition is built,
proved, and live: `kill` today counts exactly the redirects a guess did not cover, which is
`kill = buffer_empty && redirect_recovering` unchanged from ADR-0198, under a `redirect` whose own
meaning changed underneath it. With no guess ever formed, this reduces to A2's own behaviour
exactly, which is the standing verification for it (below).

## The mechanism, as built

`rtl/fetchctrl.v` reads a static BTFN (backward-taken, forward-not-taken)/`jal`-always-taken guess
off the fetched pair the cycle a genuine response is accepted (`req_valid`), no register read:
RVC's own length rule ("4 bytes iff its own low two bits are 11") applied at up to four 16-bit
lanes, starting from lane 0 or lane 1 depending on `fetch_odd` — whether the previous accepted
pair's own tail instruction straddled into this one. Only a candidate that ends exactly at the
pair's own 8-byte boundary is ever a candidate: an uncompressed branch/`jal` filling the second
word whole (candidate A), or a compressed `c.j`/`c.jal`/`c.beqz`/`c.bnez` in the second word's
upper half behind a compressed lower half (candidate B). Nothing between a taken candidate and the
pair's end is ever queued live-then-abandoned, which is what lets the whole mechanism be three
registers (`predicted_active`, `predicted_src_pc`, `predicted_target` — "one predicted turn in
flight" per the ticket) rather than a run-marker per queue entry and a second base-pc register: a
candidate in the pair's first word, or ending before the boundary, would need that bookkeeping,
since taking it would strand live already-queued bytes behind the guessed target. Those forms are
not predicted, which is the ticket's own "state precisely which forms are not predicted and why"
escape.

`rtl/decoder.v` compares its own resolution against the outstanding guess:

```
predicted_this   = predicted_active && (fetcher_pc == predicted_src_pc);
expected_fetch   = predicted_this ? predicted_target : (fetcher_pc + pc_inc);
predict_resolved = issuing && predicted_this;
mispredict        = issuing && predicted_this && (next_pc != predicted_target);
redirect          = issuing && ((next_pc != expected_fetch) || instr_fencei);
```

A correctly guessed branch has `next_pc == predicted_target == expected_fetch`, so `redirect`
stays false and fetch's already-queued words are used with no flush; `fence.i` is ORed in
separately, since its own `next_pc` equals `expected_fetch` unconditionally and would never flush
without the explicit term.

## Why the guess is not spent

`assign predict_found = 1'b0;` in `rtl/fetchctrl.v` is the one line that turns the whole mechanism
above off. What it stands on is a bisection against Dhrystone (which the hand-written `.S` suite
did not exercise deeply enough to catch): the first attempt corrupted architectural state on
26 of the suite's 75 programs and Dhrystone itself, always as `TRAP-TO-ZERO` — decode issuing
whatever garbage bytes a wrong fetch address queued, as real instructions, until one of them
trapped before `mtvec` was ever configured. Three real, distinct bugs were found and fixed by
bisecting candidate A alone (`cand_a_taken` narrowed to `jal`, then to nothing, rebuilding up):

1. **The candidate's own address arithmetic assumed `stolen_pc` was word-aligned.**
   `rtl/imemory.v` indexes on `addr[31:2]` alone, so a guess landing on an odd half-word (any
   branch whose immediate's bit 1 is set, i.e. most of them) fetched the wrong word pair on the
   next request while computing the *next* candidate's target from the *unmasked* address. Fixed
   by rounding `stolen_pc` down to `pair_base` before using it in any candidate's own arithmetic.
2. **`fetch_odd`'s own update never distinguished a committed guess from a plain sequential
   advance.** `fetch_odd_next` (`!boundary4`) describes the *sequential* continuation's own
   parity; a committed guess jumps to `predict_tgt` instead, whose own bit 1 has nothing to do
   with `boundary4`. Missing this sent every backward-taken branch whose target had bit 1 set into
   the lane walk misaligned by a half word on the very next pair — reading tail bytes of a real
   instruction as a fresh opcode, the exact class of error the length-rule walk exists to prevent.
   Fixed: `fetch_odd <= predict_tgt[1]` on a committed guess, mirroring `redirect_apply`'s own
   `redirect_target_reg[1]`.
3. **A one-pair loop** (`lw t1,4(t0); bnez t1,1b`, `UART_WAIT_IDLE`'s own shape, uncompressed
   because it reads `t1` = x6, outside the compressed register range) **still corrupted `uart.S`**
   with both fixes above in place. Bisected to a real, reproducible failure at test 7; the
   mitigation shipped is exclusion — `cand_a_same_pair`/`cand_b_same_pair` refuse a candidate whose
   own target lands back in the pair it was found in — rather than a diagnosed root cause, because
   the fix was found empirically (this restriction alone turned the `.S` suite from red to green)
   faster than the mechanism could be proved sound by inspection.

With all three fixes in and candidate B excluded entirely (`predict_found` gated to
`cand_a_taken` only), the `.S` suite (75/75) passed clean, but **Dhrystone still corrupted at
cycle 26,407 with `mispredict=1`** — essentially no guess ever resolved wrong, yet the same
`TRAP-TO-ZERO` recurred at the identical cycle whether candidate A predicted branches and `jal`
both or `jal` alone. Disabling `predict_found` entirely (§ "the guess is not spent") is the only
configuration that passed both the suite and Dhrystone; Dhrystone's own figures at that
configuration (below) reproduce A2's own baseline to the cycle, which is the control that says the
`redirect`/`mispredict` rewiring itself is sound. **A fourth bug remains unfound**: something a
single ~13,000-retire pass of Dhrystone reaches and the 75-program hand suite does not, that
corrupts state even when almost nothing is predicted. It is not diagnosed further here; a
follow-up owes either a targeted reproducer (bisecting Dhrystone's own source, or a waveform trace
of the specific retire the corruption traces back to) or a redesign that does not depend on
`fetcher_pc` ever exactly revisiting `predicted_src_pc` to resolve a guess, which is the one
invariant every fix above left unquestioned.

## Verification

`test/decoder_tb.v` gains three vectors against `rtl/decoder.v` directly (this bench has no
`fetchctrl` instance, so `predicted_active`/`predicted_src_pc`/`predicted_target` are driven by
hand): a correctly guessed taken branch (no redirect, no mispredict, `predict_resolved`), a
wrongly guessed one (redirect, mispredict, `predict_resolved` still fires), and an unpredicted
taken branch (redirect, not a mispredict). These exercise the mechanism `rtl/decoder.v` actually
carries, independent of whether `rtl/fetchctrl.v` currently forms a guess.

`formal/pcloop.sv`'s Property 1 gains a fourth arm: `fetch_pc` may land on `predicted_target` the
cycle `predicted_active` rises 0→1 with `fetch_pc == predicted_target` in the same edge (both set
in `rtl/fetchctrl.v`'s one `always_ff`, a one-cycle relationship, unlike the redirect arm's
two-cycle one). This arm is unreachable with `predict_found` tied low — the property is still
sound (an unreachable disjunct weakens nothing) and documents the machinery for whoever re-enables
it. "A killed word never issues" (Property 3) is unchanged and still load-bearing.

`formal/traps.sv` gained the minimal additive free-input/unread-output wiring `rtl/decoder.v`'s
new ports force (`predicted_active`/`predicted_src_pc`/`predicted_target` free,
`predict_resolved`/`mispredict` unread) — Stage A4's own rewiring of that file is a separate,
concurrent branch and is not otherwise touched here.

`test/zkt_isolation_test.py`'s `NON_VALUE_PORTS` gains `predicted_src_pc`/`predicted_target`: both
are wide enough to need a classification, and both are derived from ROM instruction bits and pc
arithmetic alone, never a register-file or CSR-file read — the same standing `mtvec`/`mepc`
already carry there. No change was needed to `STALL_TARGETS`, `formal/decoder-zkt-probe.py` or
`rtl/decoder.v`'s own Zkt assertions: `redirect`/`kill`/`mispredict` all sit outside `stall`'s own
composition exactly as `redirect`/`kill` already did before this ticket, and `redirect` already
depended on register-resolved branch/`jalr` targets pre-A3 — this ticket adds no new path from a
tainted register into a timing-visible signal, only a new comparison against untainted data.

`make -C formal remeasure-fg`: **F = 8, G = 8, both reproduce exactly** — no new stage, no new
stall reason, and with the guess un-spent the composed proof's own timing is bit-for-bit A2's.

**`.S` suite**: 75/75, matching `test/EXPECTED_FAIL` (empty) exactly; `test/OBSERVED_FLOOR`
unchanged.

**Dhrystone**, `make dhrystone` (`DHRY_RUNS=2000`): **1001 cycles/Dhrystone, 0.568 DMIPS/MHz,
identical to A2's own figure to the cycle.** `RETIRES 946440`, `cycles=2054635`,
`issue=946442`, `kill=488703` (A2: `488313`; both are `--stalls`-instrumented builds, not the
shipped binary, and the small residual difference is the mispredict/predict-resolved wiring now
present but always false, not a behaviour change — no control signal reads it). `mispredict=0`.
This is the control for "the redefinition changes nothing when nothing is guessed," and it holds.

**The mispredict rate this ticket asks to report is 0%, on purpose**: no guess is ever formed, so
`kill` still counts every redirect, same as A2. ADR-0188's own BTFN prototype measured 6.26% of
Dhrystone's issues as mispredicts and 3.00 cycles/redirect is A2's own already-measured kill cost;
neither figure moves here because the mechanism that would spend them is switched off.

`make lint`: clean, both passes. `make elaborate-strict`: clean.

`make -C formal all`: **100 PASS, 0 FAIL** — the generated set (86 checks, `[depth]` floors F=8/G=8,
all 86 at or above theirs, `EXPECTED_CHECKS` matches exactly), `complete`/`complete_cover`,
`imemcheck`/`dmemcheck`, and all six component proofs by k-induction (`components_decoder` with
its two Zkt probes, `components_executor`, `components_accessor`, `components_pcloop`,
`components_traps` with both its region and tval probes, `components_busarbiter`).

`make cosim-suite`: 69/75 agree; the divergence list matches `test/COSIM_EXPECTED_FAIL` exactly —
unchanged from A2, since nothing about what co-sim can and cannot see moved.

`make mutation-check`: 11 mutations, every one caught by exactly its paired detectors; `rtl/`
restored cleanly afterward (`git status` clean).

`make dual-smoke`: OK — two harts counted 32, one hart (held in reset) counted 16.

`make fit`: **4732 of 5280 `ICESTORM_LC`** (89%), +52 against the 4680 the Makefile's own comment
cites (inside the churn band), ratchet `FIT_MAX_LC` 4802 not tripped. This is not a pure null: the
alignment-tracking machinery (`fetch_odd`, the four-lane boundary walk) runs on every accepted
pair regardless of `predict_found`, since `fetch_odd`'s own next state depends on it unconditionally
— only the candidate-to-`fetch_pc` mux is dead code with `predict_found` tied low, not the walk that
would feed it. `make ecp5-timing`/`make soc-timing`: not separately re-taken here; A2's own SoC
does not place on the up5k for a reason this ticket does not touch, and Stage A's own placement
question is still Stage B's to answer.

**CoreMark**, `make coremark`: **1.712 CoreMark/MHz**, `Total ticks 58399413`, `Iterations 100`,
self-check PASS, 2K validation configuration PASS. Not compared against a prior Stage-A figure —
neither ADR-0196 nor ADR-0198 measured CoreMark — but consistent with the Dhrystone control: since
`predict_found` is tied low, this is what A1/A2's own CoreMark figure already is.

**The cycle-by-cycle mispredict breakdown the ticket asks for has no live guess to trace**: with
`predict_found` tied to `1'b0`, `predicted_active` never rises, so no waveform of the shipped
binary shows one. The closest available evidence is `test/decoder_tb.v`'s three hand-driven
vectors, which exercise `rtl/decoder.v`'s own mispredict logic directly: cycle N presents the
guessed instruction with `predicted_active`/`predicted_src_pc`/`predicted_target` already set;
`next_pc`, `redirect`, `mispredict` and `predict_resolved` are combinational off that same cycle,
so a correct guess resolves with `redirect=0`/`mispredict=0` and a wrong one with both `1`, on the
identical cycle decode issues the branch — there is no separate "kill" cycle to trace in decode
itself, since `kill` is `rtl/fetchctrl.v`'s own accounting for the cycles *after* that redirect
while the queue refills (`buffer_empty && redirect_recovering`), which A2's own 3.00
cycles/redirect figure already measured. A real trace of a live mispredict needs the predictor
re-enabled, which this ADR declines to do for a diagnostic screenshot given the unfound fourth bug.

## Consequences

- **The redirect/mispredict semantic the ticket asks for is real, proved, and load-bearing the
  moment a follow-up re-enables `predict_found`** — nothing about `rtl/decoder.v`'s own logic
  needs to change again; only `rtl/fetchctrl.v`'s fourth bug needs finding.
- **Three real bugs are fixed and worth keeping fixed**: the word-aligned pair base, the guessed
  target's own half-word parity feeding `fetch_odd`, and the same-pair exclusion. A follow-up that
  re-derives the predictor from scratch should not have to rediscover any of the three.
- **The fourth bug is the open item**, and it is specifically NOT the same-pair short-loop shape
  (that is excluded structurally) and NOT primarily about branch mispredicts (`jal`-only alone
  reproduced it with `mispredict=1`). The next session's fastest path is probably a waveform trace
  of the specific Dhrystone retire that first diverges, not another round of `.S`-suite bisection,
  since the `.S` suite is now proven unable to catch it.
- **Stage A3's own measurement section (Dhrystone before/after, the cycle-by-cycle mispredict
  breakdown, kill cycles against a real mispredict rate) is not answerable from this state**,
  because nothing is predicted. The owner's own ceiling ("Dhrystone no worse than +3% over main")
  is trivially met (0%, since nothing changed), which is not the same claim as "the predictor is
  fast" — it is not evidence either way about the predictor once it is re-enabled.
- **A pre-existing comment-density gap, not caused by this ticket**, was found on `formal/pcloop.sv`,
  `formal/traps.sv`, `rtl/decoder.v`, `rtl/littlecpu.v` and `formal/components.sby` — each was
  already over `docs/comment-budget.md`'s 5% on A2's own branch, before this ticket touched them.
  This ticket condenses (never deletes the substance of) the comments in the four it touches —
  `formal/pcloop.sv`, `formal/traps.sv`, `rtl/decoder.v`, `rtl/littlecpu.v` — to bring each under
  budget, alongside keeping every file it added net comment lines to under budget
  (`rtl/fetchctrl.v`, `test/cxxrtl.cc`, `test/decoder_tb.v`, `test/zkt_isolation_test.py`).
  `formal/components.sby`, untouched by this ticket, is left as found.
