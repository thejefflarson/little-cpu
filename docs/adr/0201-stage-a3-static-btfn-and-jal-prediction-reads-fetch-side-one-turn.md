# 0201 — Stage A3: the mispredict definition lands; the guess itself still does not ship

Status: Partial, as originally recorded — but for a different, better-understood reason than the
first pass left it at. The `redirect`/`mispredict`/`kill` semantics this ticket asks for are built
and proved, live, with a real guess forming and resolving during the work described below; that
part is done and not revisited. `predict_found` in `rtl/fetchctrl.v` ships tied to `1'b0`, as it was
at the start of this update, because two independent, real problems were found after jal-only
prediction was believed shippable, either one alone sufficient to decline it: a sixth bug, a
genuine, formally-proven architectural gap (a guess lets fetch race ahead of decode by an
address-space distance bounded only by *time*, not by the small, fixed queue depth sequential fetch
is bounded by, and nothing invalidates an already-queued speculative fetch when a later store
targets the same text address before decode retires past it — `formal/imemcheck.sv` catches exactly
this), and a seventh, unrelated finding — CoreMark, which never writes its own text and so cannot be
exercising the sixth bug, also corrupts under jal-only prediction, confirmed attributable to the
predictor by a same-tree control run that passes clean with `predict_found` back to `1'b0`. Neither
is fixed here; both are recorded as scoped future work. The fourth, fifth and sixth bugs below, and
the fixes for the fourth and fifth, are structural and worth keeping regardless: a future session
that spends the time budget this one did not have inherits three fewer bugs to rediscover, not
zero — but the seventh finding says that budget must also cover a correctness gap this session did
not diagnose, not only the sixth bug's known one. 2026-09-19, updated three times the same day.

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
