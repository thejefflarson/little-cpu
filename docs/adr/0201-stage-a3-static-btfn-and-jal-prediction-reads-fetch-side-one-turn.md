# 0201 — Stage A3: the mispredict definition lands, and the guess ships from every lane

Status: Accepted, superseding the Partial status the five earlier passes below left it at. The
final update at the end of this file is the record that matters now: `predict_found` is live in
`rtl/fetchctrl.v` for both candidate classes — a `jal` or backward branch, compressed or not, at
any lane of the fetched pair, straddling ones included — after five more real defects (the ninth
through the thirteenth) were root-caused and fixed, each with a regression program that goes red
under exactly its own mutation, and `make -C formal imemcheck` passes at its full depth with the
guess live. Dhrystone reads **820 cycles/Dhrystone, 0.694 DMIPS/MHz**, against 1001/0.568 with the
guess off and about 788/0.722 on `main`. The earlier passes are kept below as written, because
their bug numbering, their counterexamples and their measurements are what the final update is
read against. 2026-09-19, updated five times; the final update is 2026-09-20.

## Update: the fourth and fifth bugs, found by a retire-stream differential

The original session's own bisection (below, in "Why the guess is not spent") found three bugs and
then declined the whole mechanism after a fourth, undiagnosed corruption survived all three fixes.
A follow-up session found it by building the exact tool the ADR recommended: `test/cxxrtl.cc` grew
`--retire-trace N`, a bounded ring of the last N retires (cycle, pc, insn) and N cycles of
fetch-side control state (`pc`, `next_pc`, `fetch_pc`, `redirect`, `kill`, `mispredict`,
`predicted_active`, `predicted_src_pc`, `predicted_target`, `buffer_empty`, the fetched `instr`,
`instr_illegal`, `trap_cause`), dumped on any fatal exit. Diffing a jal-only-predicted Dhrystone run
against the predictor-off baseline at the same cycle window found the first divergence immediately:
a real `jal` correctly predicted and taken with no flush, followed one cycle later by decode reading
a wrong word at the jump target.

**Fourth bug: `pair_base` and `fetch_odd` read a cycle earlier than `imem_data`/`imem_data2` catch
up.** `stolen_pc`, reused as `pair_base`'s source, is not a fetch address one cycle behind
`fetch_pc` — its `redirect_apply` arm sets it to the *new* `fetch_pc` immediately, a full cycle
before that address's own ROM response has arrived. For one cycle after any redirect (a real one, or
a guess committing, which is the same kind of discontinuity), candidate detection reads bits that
belong to a different, unrelated pair than the one `pair_base` claims — reproduced live as
`predicted_target` latching `0xfffffaca` out of nothing. Separately, once a guess *is* correctly
detected, its own pair's naive successor (the address `fetch_pc` would have advanced to sequentially)
is already in flight by construction: `fetch_pc[T]` is committed to the ROM one cycle before the
candidate in `fetch_pc[T-1]`'s own pair is even evaluated, so the moment a guess commits, that
in-flight response would otherwise still reach the queue behind the guessed target's own words.
Fixed with three changes in `rtl/fetchctrl.v`:
- `fetch_addr_d1`, an unconditional one-cycle copy of `fetch_pc` with no redirect or retry
  exception, feeds `pair_base` instead of `stolen_pc` (which keeps its original retry-address role
  unchanged).
- `predict_trusted = req_valid && predict_found` and `predict_commit = predict_trusted && room &&
  !redirect_apply && !fetch_stall` gate every consumer that used to read `predict_found` alone:
  the `fetch_pc` mux, `fetch_odd`'s update, and `predicted_active`'s own latch (below). `waiting`
  additionally clears on `predict_commit` (`launch && !predict_commit`), which is what drops the
  abandoned in-flight response rather than pushing it into the queue.
- `fetch_odd`'s redirect/guess-driven update moves from `redirect_apply` to `redirect_apply_d1` (and
  the new `predict_commit_d1`), so it settles on the same cycle `fetch_addr_d1` does; by then
  `fetch_pc` already holds the settled target, so both paths can simply read `fetch_pc[1]`.

**Fifth bug, found the same way against `test/asm/rvc.S`'s own jal-over-embedded-data pattern (a
`j` skipping 8 KB of literal test data placed right after it, landing on a real instruction at a
non-word-aligned half of a 4-byte word): `predicted_active` latched on `predict_trusted` alone, not
`predict_commit`.** `room` can be false on the exact cycle a candidate is confirmed, which blocks
`fetch_pc` from actually taking the guessed jump (it falls back to the sequential `+8` instead) —
but the bookkeeping latched anyway, recording a guess that was never speculatively fetched. When
decode later reached the real `jal`, `predicted_src_pc` still matched, and its own independently
computed `next_pc` happened to equal `predicted_target` (the same real jump, correctly decoded) —
so `mispredict` and `redirect` both read false, and decode used the already-queued words from the
sequential path that never actually detoured, instead of flushing and fetching the real target.
Fixed by gating the `predicted_active`/`predicted_src_pc`/`predicted_target` latch on `predict_commit`
rather than `predict_trusted`, so the bookkeeping only ever claims a guess is active when `fetch_pc`
genuinely took it.

With both fixes, `test/asm/rvc.S` — already a standing, un-excepted member of the suite, whose
`test/OBSERVED_FLOOR` line (`rvc.S 184 184`) is unchanged — is the regression: it reproducibly
`TRAP-TO-ZERO`s under jal-only prediction with either bug present, and passes, retiring exactly its
recorded floor, with both fixed. That is the red-then-green proof for the fourth and fifth bugs; no
new program was added, because a real one already exists and already exercises the class of bug.
With jal-only prediction *live* (`predict_found = cand_a_jal && !cand_a_same_pair`), `make test`'s
own full run (75/75 `.S`/`.c` programs, every repo-scanning check) was green, and Dhrystone
`PASS`ed at 993 cycles/Dhrystone, 0.573 DMIPS/MHz. That configuration is not what ships — see the
sixth bug, found only after both of the above, which is why `predict_found` is back to `1'b0` below.

## Branches and candidate B: a bug that made jal-only look closer to shippable than it was

Setting `predict_found` to `cand_a_taken` (branches too) or `cand_a_taken || cand_b_taken`
reproduces a different failure from jal-only's own: Dhrystone never reaches a `PASS` verdict,
instead retiring far more instructions than the workload needs (5.45M retired against a
10,000,000-cycle budget, where the whole benchmark needs under 2,000,000) with `mispredict=0` for
the entire run. This is a livelock, not the corruption the fourth and fifth bugs were — the machine
keeps making forward progress, just never the progress that reaches Dhrystone's own completion
check. It was not root-caused in this session's own time budget: the leading candidate is that a
*mispredicted* backward branch (the one case a static BTFN guess is wrong about, and the one case
candidate A's own jal-only gate never exercises, since `jal` has no operand and is trivially always
correctly guessed) drives some loop's exit condition into a state neither the guess-correct nor the
guess-wrong path recovers from cleanly. This is recorded as a real, distinct, still-open finding —
widening `predict_found` to branches needs it root-caused first, on top of the sixth bug below,
which blocks jal alone in the meantime anyway.

## The sixth bug: a guess is a text read with no invalidation path, and self-modifying code needs one

Both the fourth and fifth bugs, and the jal-only figures above, were found and measured believing
jal-only prediction was ready to ship. `make -C formal all`, run once with jal-only prediction live
end to end (not merely the targeted checks the fourth and fifth bugs' own fixes were verified
against), found a sixth problem, one `formal/imemcheck.sv` already exists to catch: a `shadow_addr`
memory-coherence check, which watches one arbitrary text half-word, assumes the ROM's own output
always reflects the most recent write to whatever address it is reading, and asserts that whatever
`rvfi_insn` a later retire reports at that address matches. It failed at step 12 with jal-only
prediction live; it does not fail — this check predates this stage — with `predict_found` tied to
`1'b0`.

**The property this check states, restated in prose: any word already sitting in the fetch queue
must not be retired if a text write has since landed on its own address, because the queued content
is now stale.** Sequential-only fetch (the design as shipped before this ticket) is bounded to
looking two pairs (8 words) ahead of decode by the fetch queue's own four-word depth — a small,
fixed window a write would need to land inside, and inside a window that short before this stage
apparently never happens in a way the check's own bounded depth can construct. A guess breaks that
bound: the moment a candidate's own pair is fetched, fetch can jump to the guessed target and start
queuing its words immediately, *independent of how many instructions decode still has to retire
before it reaches the guessing instruction itself* — in the `rvc.S` trace this ADR's own
"cycle-by-cycle breakdown" section quotes, the guess formed at cycle 4 and decode did not retire the
guessing `jal` until cycle 8, a four-cycle window with no bound tying it to queue depth at all. A
text write landing on the guessed target's own address inside that window is invisible to the
guess: nothing in `rtl/fetchctrl.v` snoops `mem_addr`/`mem_wstrb`, so nothing invalidates
`predicted_active` or the queue's own already-buffered words, and decode retires stale bytes.

This is not a corruption `test/asm/rvc.S`, `make cosim-suite`, or Dhrystone can reach — none of them
write to a jal's own target address in the handful of cycles between a guess forming and that jal
retiring — which is exactly why a targeted `.S` regression did not catch it and a formal check,
built to search adversarially rather than run one fixed program, did. **No fix is shipped for it.**
A real one needs `rtl/fetchctrl.v` to observe the data bus's own write address and strobe (plumbed
in from `rtl/littlecpu.v`, where `mem_addr`/`mem_wstrb` already exist) and invalidate
`predicted_active` — which, as the fifth bug's own fix already established, is sufficient on its
own to force a real flush-and-refetch through the existing redirect path — whenever a text write's
address falls within whatever the guess has already queued. That is real, scoped design work, not
a one-line gate change, and is why `predict_found` ships tied to `1'b0` rather than at this
half-finished state.

## A seventh, independent finding: CoreMark corrupts under jal-only prediction too

CoreMark does not write to its own text, so it cannot be exercising the sixth bug above — and it
still corrupts. With `predict_found = cand_a_jal && !cand_a_same_pair` (the same jal-only
configuration the fourth and fifth bugs' fixes were verified against, on the exact tree those fixes
landed on), `make coremark` reports `TRAP TO ZERO` after `RETIRES 2481` of the run's eventual
tens of millions, `mispredict=0` throughout — the same signature as the original session's own
undiagnosed Dhrystone corruption at cycle 26,407 (a trap taken with no guess ever resolved wrong),
but on a different program and, since the fourth and fifth bugs are fixed, evidently a different
cause. **The control confirms the attribution**: reverting `predict_found` to `1'b0` on the
identical tree — no other change — runs the same CoreMark image to its own `PASS`, self-check
`PASS`, `2K validation configuration: PASS`, 1.712 CoreMark/MHz, `mispredict=0` (nothing is guessed,
so the field reads its own idle value). Whatever this is, it is jal prediction's, not CoreMark's,
and it is a *third* correctness gap distinct from both the fourth/fifth bugs (fixed) and the sixth
(self-modifying code, above) — this repo's own `make coremark` and `make dhrystone` share no
`.S`-suite-style baselining, so nothing short of running the actual workload would have caught it.
**Not root-caused, on purpose**: with two independent, already-proven reasons jal-only cannot ship
(the sixth bug's formal counterexample and this one), spending further session time isolating a
third would not change the shipping decision, so `predict_found` returns to `1'b0` immediately
after this control run and this finding is left for whoever next reopens `rtl/fetchctrl.v`,
alongside the sixth bug and the branch livelock below.

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
two-cycle one). This arm was reachable and proved, by `components_pcloop`, over the fixed
`predict_commit`-gated logic during the session's jal-only-live investigation; with `predict_found`
back to `1'b0` for the reason below, it returns to being a sound but unreachable disjunct, same as
the original pass left it — the difference is that it is now proved *correct when reachable*, not
merely unreachable. "A killed word never issues" (Property 3) is unchanged and still load-bearing.

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
tainted register into a timing-visible signal, only a new comparison against untainted data. This
was re-run against the elaborated netlist with jal prediction live (not merely declared sound
against dead logic): `test/zkt_isolation_test.py` reported the decoder reaching `region_stall`
only, and `formal/decoder-zkt-probe.py`'s own forced-red prerequisite still required a mutation at
`region_stall`'s gate or `ls_access`'s membership specifically — the predictor's own new registers
(`fetch_addr_d1`, `predict_commit_d1`, `predicted_active` and friends) are all in
`rtl/fetchctrl.v`, entirely outside the netlist this walk traces from `reg_rs1`/`reg_rs2`/
`executor_out.rd_data`, so neither the taint graph nor the probe's own mutation site needed to
change. With `predict_found` back to `1'b0`, `make test`'s own run of this check (below) is what is
authoritative for the shipped netlist.

`make -C formal remeasure-fg`, re-run against jal prediction live: **F = 8, G = 8, both reproduce
exactly** against the declared values — no new stage, no new stall reason. `predict_commit`'s own
`room` gate means a guess never lengthens the pipeline by a cycle beyond what a plain redirect
already costs; it only sometimes avoids paying that cost at all. Re-run again with `predict_found`
back to `1'b0` (the shipped configuration): unchanged, F = 8, G = 8.

**`.S` suite**: 75/75 with `predict_found` at `1'b0` (shipped), matching `test/EXPECTED_FAIL`
(empty) exactly; `test/OBSERVED_FLOOR` unchanged. It was also 75/75 with jal-only prediction live —
`rvc.S`'s own floor (`184 184`) is the regression the fourth and fifth bugs were graded against,
described above, and stayed green through the session's own configuration changes.

**Dhrystone, jal-only prediction live** (not shipped; measured before the sixth and seventh
findings, and reconfirmed bit-for-bit on this session's own final tree before `predict_found`
went back to `1'b0`): `make
dhrystone` (`DHRY_RUNS=2000`) read **993 cycles/Dhrystone, 0.573 DMIPS/MHz, PASS.** `RETIRES
940450`, `SPEC-CHECKED 940445`, `cycles=2038643`, `issue=940452`, `kill=484697`, `mispredict=0`.
Against the shipped, no-predictor configuration (1001 cycles/Dhrystone, 0.568 DMIPS/MHz,
`kill=488703`, identical to this ADR's own original figures): **−0.8% cycles**, from `kill` falling
by 4,006 cycles — every one of them a `jal` whose guess avoided a flush entirely, at zero cost when
right (jal has no operand, so a `jal`'s own guess is trivially always correct: `mispredict=0` is not
a control, it is what predicting a register-independent jump always reads). Against **main** (0.722
DMIPS/MHz, ADR-0190, about 788 cycles/Dhrystone): jal-only would have read **+26% cycles, missing
the owner's own +3% ceiling by a wide margin** even before the sixth bug is counted. Reported
honestly rather than chased: jal alone is a small fraction of Dhrystone's own redirects (`kill` fell
0.8%, not the double-digit percentage closing the gap to main would need), and the branch half that
would close most of the remaining gap is the one this session could not ship (see "Branches and
candidate B" above) — so jal-only was never going to meet the ceiling even had the sixth bug not
existed. **The shipped configuration's own Dhrystone figures are unchanged from this ADR's
original pass**: 1001 cycles/Dhrystone, 0.568 DMIPS/MHz, `mispredict=0` because nothing is guessed.

`make lint`: clean, both passes. `make elaborate-strict`: clean.

`make -C formal all`, jal-only prediction live: 88 of the 86-generated-check-plus set passed with
**no failures until `imemcheck`**, which failed — the sixth bug, described above, with a
counterexample at step 12 (`shadow_addr = 0xffff0000`) — stopping the run there. Every other
generated check (`insn_*`, `csrw_mcycle`, and the rest of the 86) passed with jal prediction live;
`imemcheck` is the one this ADR's own sixth-bug section is about. Re-run with `predict_found` back
to `1'b0` (the shipped configuration): **`imemcheck` alone confirmed PASS first**, then the full
`make -C formal all` — the generated set (86 checks, `[depth]` floors F=8/G=8, all 86 at or above
theirs, `EXPECTED_CHECKS` matches exactly), `complete`/`complete_cover`, `imemcheck`/`dmemcheck`,
and all six component proofs by k-induction (`components_decoder` with its two Zkt probes,
`components_executor`, `components_accessor`, `components_pcloop`, `components_traps` with both its
region and tval probes, `components_busarbiter`) — see the final tally below.

`make cosim-suite`, `make mutation-check`, `make dual-smoke`, `make fit`, `make ecp5-timing`,
`make coremark`, against the shipped (`predict_found = 1'b0`) configuration, run sequentially after
the formal re-run above finished (never concurrently with a formal pass — resource contention reads
as a tool failure, not a design one, and already cost this session one wasted `remeasure-fg`
attempt): **all clean.** `make cosim-suite` matches `test/COSIM_EXPECTED_FAIL` exactly (69/75
agree, the same six divergences the baseline already names). `make mutation-check`: 11 mutations,
each caught by exactly its paired detector. `make dual-smoke`: `OK — two harts counted 32, one hart
counted 16`. `make fit`: 4654 of the 4802-cell budget (the ~50-cell churn band, no ratchet trip).
`make ecp5-timing`: all three mapping censuses gate clean (`DP16KD` 36, `TRELLIS_DPR16X4` 32,
`MULT18X18D` 4, all "as declared"), no block-RAM reset driven by logic, Fmax 41.76 MHz (publishes,
no ratchet). `make coremark`: `PASS`, self-check `PASS`, **1.712 CoreMark/MHz**, `mispredict=0`
(nothing is guessed). This is also where the seventh finding above was found: the same CoreMark
image, same tree, with jal-only prediction live instead, does not reach `PASS`.

**The cycle-by-cycle breakdown of one correctly predicted jal**, from the jal-only-live
configuration this ADR does not ship, read off `--retire-trace`'s own capture of `test/asm/rvc.S`
(columns: cycle, `pc`, `next_pc`, `fetch_pc`, `predicted_active`, `predicted_src_pc`,
`predicted_target`, `redirect`). This is the same `jal` and the same four-cycle gap the sixth bug's
own section cites as the exposure window a text write could land in undetected:
```
4  pc=0x0000004 next_pc=0x0000008 fetch_pc=0x0000010                 pred_active=1 src=0x0c tgt=0x1ffe
8  pc=0x000000c next_pc=0x0001ffe fetch_pc=0x0000018 redirect=0      pred_active=1 src=0x0c tgt=0x1ffe
9  pc=0x0001ffe next_pc=0x0000000 fetch_pc=0x0000020 redirect=0(trap elsewhere, unrelated)
```
The guess forms at cycle 4, well before decode reaches the jal at cycle 8 — fetch is racing ahead
of decode by several pairs, exactly as intended. At cycle 8 decode issues the real `jal`,
`predicted_this` matches (`fetcher_pc == predicted_src_pc`), `next_pc` computed independently of
the guess equals `predicted_target`, `redirect=0`: **no flush.** At cycle 9 decode is already
issuing the real instruction at the jump target — one cycle later, the same as a plain
non-branching instruction would cost. **Zero cycles, not three**: the 3.00 cycles/redirect figure
(ADR-0198) is what an *unpredicted* redirect pays (resolve → `redirect_apply` registers the target
→ the target's own pair is requested → its response is pushed into the queue → decode can finally
issue), and remains exactly that cost, unchanged, for anything jal-only prediction does not cover
(every branch, every `jalr`, every trap). That three-cycle structure is a property of the ROM's own
one-cycle latency plus the two-cycle `flush` window (`redirect_apply || redirect_apply_d1`); this
ticket touches none of it. A live *mispredicted* guess has no trace to show, since jal-only
prediction cannot mispredict by construction (above) and branches are not shipped.

## Consequences

- **The guess still does not ship, and the reason moved from "one unknown" to "two known, real
  problems, plus a third not yet diagnosed"**: `predict_found` is `1'b0` in the shipped tree, same
  as the original pass, but this session traded three unknowns (a fourth bug, a fifth bug, and "is
  jal-only shippable at all") for two scoped, formally-or-empirically-demonstrated real problems
  (the sixth bug's missing text-write invalidation, and CoreMark's own corruption, confirmed
  attributable to the predictor but not root-caused) — a guess needs to invalidate itself against a
  text write to whatever it has already speculatively queued, and something distinct from that,
  reachable by CoreMark alone among everything this session ran, also needs finding and fixing.
- **Five real bugs are now fixed** (word-aligned pair base, guessed-target half-word parity, the
  same-pair exclusion — the original session's three — plus the fourth and fifth this update
  found: `pair_base`/`fetch_odd` reading a cycle before `imem_data`/`imem_data2` catch up after any
  discontinuity, and `predicted_active` latching on `predict_trusted` instead of the room-gated
  `predict_commit`), all inert but present in the shipped tree (`predict_found`'s own `1'b0` gates
  every consumer of the mechanism they fix). A follow-up that re-derives the predictor from scratch
  should not have to rediscover any of the five, and does not need to reach `rtl/fetchctrl.v` at
  all to un-ship them — flipping `predict_found` back to `cand_a_jal && !cand_a_same_pair` is
  sufficient to reach the jal-only-live state this ADR measured, once the sixth bug has a fix.
- **A retire-stream differential, not another round of `.S`-suite bisection, is what found the
  fourth and fifth bugs; the full formal suite, run against the live guess rather than only the
  targeted checks the fourth and fifth bugs' own fixes were verified against, is what found the
  sixth.** `test/cxxrtl.cc --retire-trace N` is now a standing tool, not a one-off script, for
  exactly the reason the original session's own bisection stalled — a corruption the hand suite
  cannot reach needs the specific retire and the cycles before it, not a pass/fail table. The sixth
  bug's own lesson is sharper: **a mechanism is not verified until the full formal suite has run
  against it live** — every targeted check (`test/decoder_tb.v`, `pcloop`'s new arm, Zkt, F/G) can
  pass while a check nobody thought to re-target (`imemcheck`, unrelated on its face to a static
  branch guess) finds a real, adversarially-constructible violation.
- **Branches and candidate B are a separate, still-open livelock**, found and left unexplained
  before the sixth bug was found; widening `predict_found` past jal needs both this and the sixth
  bug resolved, and neither is a "flip a gate" change.
- **A third, independent correctness gap — CoreMark's own corruption under jal-only prediction — is
  also open**, confirmed real (a same-tree control run with `predict_found` back to `1'b0` passes
  clean) but not root-caused, deliberately: the shipping decision (disable) was already forced by
  the sixth bug alone, so this session did not spend further time isolating a third cause it did
  not need to isolate to reach that decision. A future session re-enabling jal-only owes finding
  this one too, not only the sixth bug's fix.
- **The owner's own +3% ceiling was never going to be met by jal alone even had the sixth bug not
  existed**: jal-only prediction, live, read 993 cycles/Dhrystone against main's ~788, +26%. That
  number is not shipped and is recorded for whoever picks up the sixth bug next, so the next
  session knows what ceiling jal alone reaches even once corrected, and that branches (blocked on
  their own livelock) are what closing the real gap needs.
- **A pre-existing comment-density gap, not caused by this ticket**, was found on `formal/pcloop.sv`
  and `rtl/decoder.v` after the Stage A1 merge picked up both sides' own prose on the same ports and
  properties, pushing files A3's own first pass had already brought under budget back over it.
  Condensed again here, alongside `rtl/fetchctrl.v`'s own comments for the fourth and fifth bugs'
  fix, without losing the mechanism each states.

## Update: an eighth bug (`predict_commit`'s own redirect window), and a ninth counterexample

A follow-up session took the sixth bug's own suggestion — snoop the write bus and invalidate a
guess that a store has outrun — and found a *different* bug first, by reproducing `imemcheck`'s
step-12 counterexample and reading its own waveform rather than trusting the shadow-address value
alone. **The counterexample this session first reproduced had no store in it at all**:
`mem_wstrb` stayed zero the entire fifteen-step trace, and `shadow_stored` never left `0` — so the
sixth bug's own "a write with no invalidation path" story does not describe it. Reading `pair_base`,
`fetch_odd`, `predicted_src_pc`/`predicted_target` and `redirect_apply`/`redirect_apply_d1` cycle by
cycle against the trace found this instead:

**`predict_commit` only excluded `redirect_apply`, not `redirect_apply_d1`.** `flush` itself is
`redirect_apply || redirect_apply_d1` — two cycles — because the pair a redirect is abandoning
still arrives one cycle *after* `redirect_apply` itself has already dropped back to zero (the ROM's
own one-cycle latency for the request the abandoned path made the cycle before). `predict_commit`
read only the first of those two cycles, so a candidate detected in that stale, being-discarded
pair could still commit a guess — `predicted_src_pc`/`predicted_target` built from a `pair_base`
that has nothing to do with the redirect's own real target. Fixed by widening the exclusion to
match `flush`'s own window: `predict_commit = !redirect_apply && !redirect_apply_d1 && !fetch_stall
&& room && predict_trusted`. This is a real, reproducible defect, independent of the sixth bug and
of self-modifying code — waveform inspection of `formal/imemcheck`'s own counterexample is the
red-then-green proof: before the fix, `imemcheck` failed at `imemcheck.sv:88` (the low-half
retire-value assertion) with `shadow_addr = 0xeac90`, `mem_wstrb = 0` throughout, `predicted_active`
having latched a guess whose own `predicted_target` (`0xeac90`) reused a `pair_base` computed one
cycle before a real, unrelated redirect had actually settled fetch there. After the fix, that exact
trace closes; `imemcheck` finds a *different* counterexample instead (below), which is the proof the
fix changed the behaviour it targeted rather than merely relabeling the same failure.

**The text-write invalidation the sixth bug called for is also built now** (`rtl/littlecpu.v`
computes `mem_text_write` from the accessor's own `mem_addr`/`mem_wstrb` against `LS_TEXT_WORDS`,
the same range test `rtl/decoder.v`'s own `ls_supported` already makes for a different reason;
`rtl/fetchctrl.v` takes it as a new `text_write` input and clears `predicted_active` on it,
ahead of `predict_commit`'s own set arm so a same-cycle collision favours invalidation). Clearing
`predicted_active` is sufficient on its own — exactly as the fifth bug's fix already established —
because it makes the eventual resolution look unpredicted, which the existing redirect path already
flushes correctly; no second flush mechanism was added. This is coarser than address-exact (any
write anywhere in the 8 KB text window clears whatever guess is outstanding, not only a write to the
guessed target's own address), which costs nothing measurable on Dhrystone or CoreMark since neither
benchmark ever writes its own text, and avoids adding an address-range comparison to the fetch
loop's own critical path — `predicted_active`'s clear is a register-enable input, not a term in
`fetch_pc`'s combinational next-state logic, so it does not extend the fetch loop's own cone.

**Both fixes are inert with `predict_found` held at `1'b0`**: `predict_commit` can never fire
without a real, currently-eligible candidate (`predict_trusted`), so the redirect-window widening
changes nothing when nothing is ever eligible; `mem_text_write` only clears a register that is
never set. `make -C formal imemcheck` was re-run a third time, jal-only prediction OFF, and passed
exactly as A3 left it — no behavioural change to the shipped configuration. `make lint` and
`make elaborate-strict` are clean with both fixes present.

**With jal-only prediction live again and the eighth bug fixed, `imemcheck` still fails, at a
different assertion (`imemcheck.sv:90`, the high-half check) and a different `shadow_addr`
(`0x30010`).** Reading that trace the same way found no write either (`mem_wstrb = 0` throughout
again) and, this time, no active guess at the failing retire (`predicted_active` had already
cleared several cycles earlier, after an unrelated real redirect, and never rose again for the rest
of the trace) — so this ninth counterexample is not obviously the eighth bug's own class, and not
obviously prediction's own bookkeeping either, at least not by inspection of the signals this
session captured. **Not root-caused.** Continuing to fix one `imemcheck` counterexample at a time
and finding another is the same pattern the owner's brief warned this exact ticket has produced
twice already; this session stops here rather than repeat it a third time, and records what it
found rather than guess further. `predict_found` ships `1'b0`, unchanged from A3's own shipped
configuration — the net change this session makes is two structural fixes, both inert, plus a
narrower, better-characterized description of what is still open: at least one more defect in the
guess mechanism's interaction with an ordinary (non-predicted) redirect, reachable by `imemcheck`
within its own derived depth with no store involved.

**Items 2 and 3 of the follow-up ticket were not reached.** Both need `predict_found` live to
reproduce (CoreMark's own corruption, and the branch/candidate-B livelock), and this session could
not certify any live configuration against the one check that already twice found a real
counterexample in one. Investigating either against a mechanism this session cannot ship would be
guessing at a moving target; both stay exactly as A3 left them, open, for whoever next lands a
configuration that clears `imemcheck` outright.

### Consequences (this update)

- **Eighth bug fixed, inert in the shipped tree**: `predict_commit`'s own redirect-window exclusion
  now matches `flush`'s two cycles. A future session enabling jal-only prediction inherits one fewer
  bug to rediscover.
- **The sixth bug's own proposed fix (a text-write snoop) is built**, also inert in the shipped
  tree, and is not by itself sufficient to clear `imemcheck` with prediction live — the ninth
  counterexample below has no write in it, so a working invalidation path does not imply the guess
  mechanism is otherwise sound.
- **A ninth, distinct, undiagnosed `imemcheck` counterexample is open**: no store, no active guess
  at the failing retire, a different assertion and a different address than either the sixth bug or
  the eighth. Whoever picks this up next should read the counterexample's own waveform (`sby`'s
  `engine_0/trace.vcd`) rather than the summary line alone — both bugs this session found were
  invisible in the summary and only became clear from `pair_base`/`fetch_odd`/`redirect_apply*`
  read cycle by cycle.
- **`predict_found` stays `1'b0`.** Dhrystone and CoreMark figures are unchanged from A3's own
  shipped configuration (1001 cycles/Dhrystone, 0.568 DMIPS/MHz; CoreMark `PASS` at 1.712
  CoreMark/MHz) because nothing observable changed for that configuration.
- **Items 2 (CoreMark's own corruption) and 3 (the branch/candidate-B livelock) are untouched** —
  reproducing either needs a live configuration this session could not certify safe to run to
  completion against the one check built to catch exactly this class of defect.

## Update: the oracle is sound, five more defects, and the guess ships from every lane

**The question the escalation asked first — is `formal/imemcheck.sv` still describing the
interface the design has? — has a definite answer: yes.** Its assume is the ROM's own contract,
restated: `imem_data`/`imem_data2` answer the word-aligned address presented on `imem_addr_next`
one cycle earlier (`past_imem_addr_next`), skipped on the one cycle `fetch_stall` says the port was
stolen, which is exactly the cycle `rtl/fetchctrl.v`'s own `req_valid` skips. Nothing in it names
decode's pc, the queue or the guess; a predicted `fetch_pc` is just another address on that port.
Its assertion is gated on `!shadow_stored`, so it never compares a retire against a watched
halfword that has been written — which is why the sixth-bug story ("a text write with no
invalidation path") could never have been what it was catching, and why every counterexample it
produced had `mem_wstrb` at zero. The check is a coherence oracle over an unwritten ROM: whatever
word fetch was handed for an address, decode must retire at that address. Under a predictor that is
precisely the property that matters. So the ninth counterexample was a real defect, and reading it
cycle by cycle (`imemcheck/engine_0/trace.vcd`, reproduced in twelve seconds at
`shadow_addr = 0x30010`, step 12, `imemcheck.sv:90`) found it in one pass:

```
t2  fetch_pc=0x000008  req_valid  predict_commit      guess: src 0x4 -> 0xffffa
t3  fetch_pc=0x0ffffa  decode issues the jal at 0x0 unpredicted: redirect to 0x3000a
t4  redirect_apply     fetch_pc=0x100002 (the guess's successor)   predicted_active cleared
t5  redirect_apply_d1  fetch_pc=0x03000a  the abandoned pair ROM[0x100002] is on imem_data
                       with a jal in its second word: predict_trusted=1, predict_commit=0,
                       and fetch_pc <= predict_tgt = 0x130012 instead of 0x30012
t6  ROM[0x3000a] pushed;  t7 ROM[0x130012] pushed as if it were 0x30012's pair
t11 the retire at 0x3000e straddles into 0x30010 and reports 0x130010's upper half
```

**Ninth bug: the fetch address took the guess off `predict_trusted` while the record took it off
`predict_commit`.** The eighth bug's fix widened `predict_commit` to exclude `redirect_apply_d1`
but left `fetch_pc`'s own mux reading `predict_trusted`, so a candidate in the pair a redirect was
discarding still steered fetch, with no record that it had. One gate now drives both: `fetch_pc`,
`stolen_pc`, `waiting` and the record all key on `predict_commit`. Regression: `test/asm/predstale.S`
— a taken branch whose abandoned pairs each end in a `jal` to a block that stores a failure code
through an absolute address (the words execute at a pc they were not linked for, so nothing in
them may be pc-relative); `FAIL 2` with the mux back on `predict_trusted`, `PASS` fixed.

**Tenth bug: a steal on the commit cycle retried the abandoned successor.** `stolen_pc` records the
address a stolen ROM read must re-present; on a commit cycle that address is the successor the
guess abandons, so a text load or store landing on that cycle sent fetch back to it, and the
successor's pair was queued behind the candidate with the record still pointing at the target.
This is what CoreMark's `TRAP TO ZERO` at retire 2481 was: it reads its own `.rodata` out of ROM,
and a load from text steals the fetch port exactly as a store does. Fixed: the commit arm writes
`stolen_pc <= predict_tgt` too. Regression: `test/asm/predsteal.S` — seventeen sixteen-word blocks
that slide a text load (from deep inside the window, so it issues with no region wait and its bus
cycle can meet a pair's arrival) across a `jal` at the twelfth word; the abandoned word is a store
of `TESTNUM` to `tohost`. Two of the seventeen blocks meet the commit cycle exactly; `FAIL 8` with
the arm reverted, `PASS` fixed. CoreMark passes with its self-check and the 2K validation run.

**Eleventh bug: a second candidate overwrote an outstanding record, and this is the branch
livelock.** `predict_commit` never read `predicted_active`, so a candidate in the target's own pair
— which arrives before decode reaches the guessing branch — replaced `predicted_src_pc`. The first
branch then resolved as unpredicted: taken, it redirected and merely wasted the guess; **not
taken, it fell through into the target pair's words as if they were its own successor**, which for
a loop's exit branch is a loop that re-enters its body from the fall-through and never exits —
5.45 M retires of forward progress with `mispredict = 0`, because no record ever matched. Fixed:
`predict_commit` requires `!predicted_active`; one guess in flight is now a gate, not a hope.
Regression: `test/asm/predtwice.S` — a backward `bne` held on a load hazard so the target pair's
`jal` arrives first, with `entry` laid out so a `jal` executed at the wrong pc lands sixteen bytes
past `poison`, on the failure store; `FAIL 2` with the gate dropped, `PASS` fixed.

**Twelfth bug: the same-pair exclusion compared eight-byte blocks, and pairs are four-aligned.**
After any redirect to a `4 mod 8` target every pair base is `4 mod 8`, so `target[31:3] ==
pair_base[31:3]` neither excluded a target in the candidate's own word nor kept one in the pair's
first word. A target inside the candidate's own word is the real hazard: `pop` fires when the pc
leaves a word, so a guess there queues the word a second time and the loop's exit reads the copy.
A target in the pair's first word is fine — `pop` moves to the target pair, which begins with that
word. Fixed: exclude a target inside a word the candidate occupies, read off the immediate.
Regression: `test/asm/predword.S` — `c.addi; c.bnez` in one word behind a `4 mod 8` entry, two
passes; `FAIL 2` (the counter reads −1) with the block compare back, `PASS` fixed.

**Thirteenth: the text-write clear is removed, not fixed.** Clearing `predicted_active` while the
guessed detour is still queued is the eleventh bug by another route: a cleared record makes a
not-taken resolution fall into the target's words. It was built for the sixth bug, whose
counterexample the eighth-bug pass had already shown contains no store. The architectural
guarantee is `fence.i`, which serializes, redirects and so flushes the queue and clears the record
— the same path a sequential prefetch always relied on, since a store to a word already queued was
stale before any predictor existed. `test/asm/selfmod.S` still passes; `text_write` and
`mem_text_write` are gone from `rtl/fetchctrl.v`, `rtl/littlecpu.v` and `formal/pcloop.sv`.

With those five in, `imemcheck` passed at depth 15 with both candidate shapes live, and the suite,
Dhrystone and CoreMark all passed — at **1005 cycles/Dhrystone**, four cycles *worse* than the guess
off: 18,057 guesses resolved against roughly 157,000 redirects, a third of them wrong. The
predictor as designed could not pay for Stage A. The rest of this update is what it took to make
it pay, each step measured on the same `DHRY_RUNS=2000` run:

| configuration | cycles/Dhrystone | DMIPS/MHz | `kill` | guesses | mispredicts |
|---|---|---|---|---|---|
| guess off (A2/A3 as shipped) | 1001 | 0.568 | 488,703 | 0 | 0 |
| A and B as designed, five fixes | 1005 | 0.572 | 470,643 | 18,057 | 6,018 |
| + compressed at lane 2, quadrant 01 only | 897 | 0.634 | 278,268 | 74,182 | 2,019 |
| + every lane, half push | 880 | 0.646 | 229,790 | 90,506 | 2,061 |
| + straddling 32-bit, double pop (ships) | **820** | **0.694** | 109,240 | 134,731 | 4,081 |
| `main` (ADR-0190) | ~788 | 0.722 | — | — | — |

- **Quadrant 01 only.** Candidate B read `funct3` alone, and `c.sw`/`c.swsp` share the branch
  codes: a store at lane 3 with bit 12 set was a "backward branch", guessed, and always wrong. That
  was two thirds of the mispredicts.
- **Lane 2.** A compressed jump or branch at lane 2 leaves lane 3 stranded, and that is harmless:
  `pop` leaves the word once the guess is taken. The ticket's own restriction to candidates ending
  flush with the pair was one word too strict, and this step alone is 108 cycles/Dhrystone.
- **Every lane, half push.** A candidate in the pair's first word needs the target's pair queued
  right behind that word, so `rtl/fetchqueue.v` takes `req_half` and queues the low word alone
  (`test/fetchqueue_tb.v` vectors it). The first taken candidate in program order wins; one
  immediate mux and one adder serve every position. The price is a bubble when decode was waiting
  on that very pair — `q_valid` needs two words, so the lone word waits for the target pair —
  which is the `fetch` column rising 21,476 → 35,679; the gain is the redirect it replaces.
  Regression: `test/asm/predhalf.S` (`FAIL 1` with `req_half` tied low).
- **Straddling 32-bit branches, double pop.** After the step above, 42,327 of Dhrystone's 48,412
  remaining 32-bit branch redirects were at `2 mod 4` addresses — a hot loop's back-edge sitting
  across two words. Lanes 1-2 are one word's worth of decode; lane 3 into the next pair's lane 0
  keeps lane 3 in `prev_lane3` and forms the candidate when the next pair arrives, with a half
  push of that pair. Either way the instruction leaves both its words behind, which is
  `rtl/fetcher.v`'s new `pop2`, restated in `formal/pcloop.sv`'s Property 2. Regression:
  `test/asm/predstraddle.S` (`FAIL 3` with `pop2` tied low).

**What stays unpredicted, and why.** `jalr`, `c.jr` and `c.jalr` — a return needs a stack, and
Dhrystone's 20,057 of them are now the largest remaining class; forward branches, which BTFN
guesses not taken and which pay the redirect they always paid when taken (6,045 compressed on
Dhrystone, 2,036 of them backward: the mispredicts and the refusals); traps, `mret` and `fence.i`;
a second candidate while one record is outstanding (4,004 refusals on Dhrystone) and a candidate
in a pair a redirect is discarding (8,209); and a target inside a word the candidate occupies.
Remaining `kill` on Dhrystone is 109,240 cycles, about 36,000 redirects.

**Costs, from the same run.** A correctly guessed branch or `jal` in the second word costs zero
cycles — decode issues the target's first instruction the cycle after the guessing one, as a
plain instruction would. A correctly guessed first-word candidate costs zero when decode is behind
fetch and up to two when it was waiting on that pair. A mispredict costs the three cycles a
redirect always costs (ROM latency plus the two-cycle `flush` window), never more for a
second-word guess; a first-word one that was also waited on can reach five. On Dhrystone 4,081
mispredicts against 134,731 guesses is 3.0%; on CoreMark 6.9%.

**Against the owner's +3% ceiling over `main`: 820 against 788 is +4.1% of cycles, missed.** The
whole remaining gap to `main` is Stage A's own fetch queue on redirects the static guess cannot
reach — returns above all — and a return-address guess is the next lever, not another static form.
CoreMark: **1.906 CoreMark/MHz** at the every-lane step (52,494,453 cycles, 2,385,101 guesses),
against 1.712 with the guess off and 2.155 on `main`; the straddling step's figure is in the PR.

**Verification, on the shipping configuration.** `imemcheck` at depth 15 (its cover and forced-red
stalled-bus probe unchanged); `components_pcloop` by k-induction, with `rtl/fetchctrl.v`'s `FORMAL`
block now asserting that the cycle after a commit both `fetch_pc` and `stolen_pc` hold the target
with the record active — the ninth and tenth bugs' tripwire; `make -C formal remeasure-fg` and
`make -C formal all`; the suite at 81 programs (the six regressions added to `test/OBSERVED_FLOOR`,
each shown red under its own mutation and green under every other's); `make test`, `make lint`,
`make elaborate-strict`, `make cosim-suite`, `make mutation-check`, `make dual-smoke`, `make fit`,
`make ecp5-timing`. The Zkt walk is unchanged: every new register is in `rtl/fetchctrl.v`, outside
the netlist it traces, and `predict_resolved` reaches `predict_commit` only through
`predicted_active`, a register. `test/cxxrtl.cc`'s `STALLS` line gains `guess=`, the count of
resolved guesses, beside `mispredict=`.
