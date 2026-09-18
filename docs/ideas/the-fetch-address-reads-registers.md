# The fetch address reads registers, and the register file gets its own stage

**Status:** planned · not started · 2026-09-18. The cycle figures below were re-taken under the
pinned `riscv-none-elf-gcc` 15.2.0-1 (ADR-0190); the ECP5 clock figures and both opponents' products
predate that pin and are flagged where they appear, because the weekly cross-core stamp has not
re-taken them yet. The area and Fmax figures for the proposed design are **estimates**, labelled as
such, and nothing here has been built. Where this brief and a later ADR disagree, the ADR wins.

Make littlecpu competitive with VexRiscv and Hazard3 in `soc/compare/` by restructuring the pipeline
so the fetch address reads registers only and the register file is read in its own stage. The
owner's framing, and the reason this is a restructure rather than another tuning pass: **a fetch
stage is standard for a reason, and the novel merged single-cycle fetch-and-decode is what costs the
cycles.** So fetch becomes its own stage, and "no wrong-path state" — a commitment this project
invented rather than inherited — is amended alongside the two commitments that hold the same loop
closed.

## What the gap actually is, per part

One harness, `soc/compare/`, RV32IM, same part, memories, program and seeds.

| | littlecpu | VexRiscv | Hazard3 |
|---|---|---|---|
| Dhrystone cycles (ratio to littlecpu) | 313,627 (1.00) | 262,827 (0.838) | 252,026 (0.804) |
| CoreMark cycles | 446,995 (1.00) | 426,430 (0.954) | 666,552 (1.491) |
| up5k worst-of-12 MHz | 12.40 | 21.92 | 14.30 |
| ECP5 worst-of-12 MHz *(pre-pin)* | 32.01 | 52.91 | 48.88 (flagged: read 33.26 in an earlier session) |

On the up5k all three cores quantise to the same 12 MHz step, so **the up5k gap is cycles alone** —
24% behind Hazard3 on Dhrystone. On ECP5 the gap is **mostly clock**, 1.65× to VexRiscv. "The gap is
mostly clock" is true of one part and false of the other, and planning one goal for both parts is how
these numbers became confusing.

**The gcc pin widened the gap it did not cause.** Pinning the compiler moved littlecpu's Dhrystone
cycles 290,825 → 313,627 while Hazard3 moved 252,825 → 252,026 and VexRiscv 254,026 → 262,827: the
same C, compiled honestly, costs this core more than it costs them. Nothing about the design changed.
It does mean every target below is harder than it was when this plan was first drawn.

## Where littlecpu's cycles go

Dhrystone, CPI 1.585 (ADR-0175): issue 63.1%, hazard 21.1% (hzA 11.1%, a producer executing this
cycle; hzB 0.4%, load-use; hzC 9.5%, a ready result forwarding cannot reach because the reader sits in
the fetch loop), operand-fetch 15.8% (the guessed register pair missed). Redirects are 16.93% of
issues and cost zero cycles today.

**The operand column plus hzA and hzC — 36% of all cycles — exist only because decode reads register
VALUES in the same cycle it reads the instruction word.** That is the same fact that makes the fetch
loop long. One restructure removes both, which is the whole argument for doing this as a restructure.

## Why single-term fixes cannot work here

ADR-0076 measured that no single input to `next_pc` is worth more than 5% of the period and that all
of them together are worth 21%. ADR-0078 then built the one-term change — a fetch register with a
kill — and collected a third of that ceiling (−4.5%), because `stall` and the guessed register pair
still chained the fetch address to this cycle's decode. Every later candidate (ADR-0083's forwarding,
ADR-0097's candidate 4, ADR-0129's same-cycle region test) was priced against a 3–5% margin over a
12 MHz step and declined.

Single-term edits cannot spend a plateau, and on the up5k a partial clock gain buys nothing at all,
because the next step down is 6 MHz. Only a restructure spends the whole 21%, and only a restructure
turns the up5k's unspendable clock into spendable margin for cycle work.

## Assumptions challenged

1. **"No wrong-path state is something we made up" — validated, but half the story.** It is a means,
   and CLAUDE.md says as much. But ADR-0078 and ADR-0188 showed that amending it *alone* collects
   −4.5% and costs 3–11% of cycles: the fetch address still reads `stall` and the fetched word's
   decode. Three commitments hold the loop closed together — no wrong-path state (1), traps committed
   in decode (2, whose region test, misalignment and `jalr` target all want register values in the
   fetch cycle), and the guessed register pair (6, whose miss is `operand_stall`, which feeds `stall`,
   which feeds the fetch address). **Amend all three or none.**
2. **"Fetch-and-discard meets fast, simple, readable" — validated for the decoupled form, false for
   the coupled form.** ADR-0188's register-only fetch moved the critical path off the ROM address
   decode and read 37.3–41.0 MHz on ECP5 against a 33.0–35.4 base, at four seeds. The coupled forms
   (K, K+BTFN) did not move it.
3. **"The gap is mostly clock" — shaky.** True on ECP5, false on up5k.
4. **"CPI can be kept close to today's while the clock rises" — false as posed, and the real outcome
   is better.** The restructure removes 36% of Dhrystone's cycles and adds roughly 23 redirect cycles
   per 100 issues. Net CPI goes *down*, estimated 1.585 → ~1.28, and that is what closes the up5k gap.
5. **"littlecpu carries more ISA, so it must be slower" — false for the clock.** C, A, Zkt and the CSR
   set cost area and decode depth, not the fetch loop; VexRiscv also has C in this harness. What costs
   the clock is one architectural choice: register values in the fetch cycle.
6. **"The up5k has room" — shaky, and the largest real risk.** The placed SoC is ~4,900–4,950
   `ICESTORM_LC` of 5,280. ADR-0188's 8-word queue did not fit, at 5,861.

## Targets

| part | measure | today | target | why |
|---|---|---|---|---|
| up5k @ 12 MHz | Dhrystone cycles | 313,627 | **≤ 257,000** | within 2% of the leader (Hazard3, 252,026) |
| up5k @ 12 MHz | CoreMark cycles | 446,995 | **≤ 426,000** | keep the one lead we have, ahead of VexRiscv's 426,430 |
| up5k | worst-of-16 Fmax | 12.40 (3–5% margin) | **≥ 13.8** (15% margin) | so the next cycle-buying change is not declined on 0.5% of margin |
| ECP5 | worst-of-12 Fmax | 32.01 *(pre-pin)* | **≥ 45** | Hazard3-level clock, so the product is a real comparison |
| ECP5 | Dhrystone DMIPS, worst placement | ~23.1 | **≥ 40** | follows from ≥45 MHz and the predicted CPI |
| ECP5 | CoreMark, worst placement | ~71.6 | **≈ 105** at 45 MHz | the product of the two targets above, not a third target |

**The cycle targets got materially harder under the pin.** Reaching ≤257,000 from 313,627 is −18.1%,
where the same target was −11.3% against the pre-pin baseline. The predicted CPI gain is −19%. The
pin therefore consumed nearly the entire planned margin, and this plan now has to land close to its
own estimate rather than comfortably inside it. That is a reason to measure early, not a reason to
soften the target: the target is what "competitive" means.

The ECP5 rows are stated against clock figures that predate the pin, and both opponents' ECP5 products
likewise. They are re-based when the weekly `compare-product-schedule` stamp re-takes both factors in
one session. The final ECP5 row is written as a product of the two targets above rather than as an
independent goal, because a product is a measurement only when both factors came from one tree and one
toolchain.

**24 MHz on the up5k is dropped as a goal.** 41.67 ns needs about 12 LUT levels on every path at
icetime's ~3.3 ns per level. ADR-0078's ceiling with every redirect term and `stall` off the fetch
address and decode untouched was 61.27 ns — 16.3 MHz — and the path underneath it was
`por_done → csrs.mstatus_mie` at 16 levels, a CSR-file path no fetch work touches. Reaching 24 is a
second campaign over the CSR file and the SoC glue, with unknown yield. This restructure is estimated
to land the up5k at 15–19 MHz worst-of-16: unspendable as clock, entirely spendable as margin.

Also not in this round: no dynamic predictor, no radix-4 divider, no ISA change, and nothing in
`nano/`.

## Approaches considered

**A. Decoupled fetch only** — fix and shrink ADR-0188's `prototype-decoupled.patch`, keeping today's
merged stage that reads register values through the guess. ECP5 ~37–41 worst-of-12 (measured at four
seeds, functionally broken); up5k ~13–14, since the decode head still sets the period. CPI +3–4%.
Area +150–250 LC (estimate). On its own it **loses on the up5k** — cycles alone, −3% — and wins about
+10% of ECP5 product. It forecloses nothing: it is the first half of C.

**B. VexRiscv-shaped 5-stage** — F → D (regfile read) → X → M (branch resolves here) → W, full bypass.
Highest ECP5 clock of the three (estimate 48–55) because X carries no branch verdict to the fetch pc;
up5k ~18–21. But the mispredict penalty is 3 cycles, so redirects cost ~35 per 100 issues on
Dhrystone and CPI lands ~1.40 — roughly VexRiscv parity, not a lead. Area +350–500 LC, the biggest
rewrite, and the highest up5k area risk.

**C. Textbook 5-stage with the branch resolved in X (recommended).** F (register-only fetch address,
4-word buffer, static BTFN and `jal` guessed in D) → D (decode the buffered word, present the regfile
pair from the registered word, scoreboard) → X (operands arrive, forwarding, ALU, branch, `jalr`,
effective address, region test, all traps, interrupt) → M (bus data phase) → W (unpack, regfile
write). Mispredict 2 cycles, predicted-correct 1, `jalr` 2. Estimated ECP5 42–48 worst-of-12, up5k
15–19 worst-of-16, CPI 1.585 → ~1.28. Area +100 to +400 LC.

**C beats A** because A alone is CPI-negative on the part that can only spend cycles. **C beats B**
because B pays a third mispredict cycle for a clock gain the up5k cannot spend and ECP5 does not need;
C's X path — forward mux → 33-bit subtract → verdict → `fetch_pc` mux — is about 7 LUT levels plus one
carry chain, roughly 34 ns on up5k and ~12 ns on ECP5, which is not the critical path on either part.
And C is the pipeline every reader already knows: it deletes more concepts than it adds.

Hazard3's 3-stage shape was considered and rejected — its fetch address phase lives in X, so its loop
is ours, and it clocks only 15% above littlecpu on the up5k despite half the area. A 16-bit fetch
datapath was rejected: fetch needs two neighbouring 32-bit words per cycle for C, and ADR-0135 already
showed a 16-bit window is four SPRAMs.

## The pipeline, concretely

**Fetch.** `fetch_pc` is a register updated from registers and from two verdicts that are themselves
registered before they reach the ROM: `redirect` from X (branch, `jalr`, trap, `mret`, interrupt
target), `predict` from D (BTFN or `jal` target, `pc + imm`, from the buffered word's opcode and
immediate sign — no register read), else `+8` when the buffer has room, else hold. `imem_addr_next` is
`fetch_pc`, unconditionally. `rtl/imemory.v` is unchanged. A 4-word FIFO of the shape in ADR-0188's
`rtl/fetchqueue.v` pushes a pair and pops a word when D's pc crosses a word boundary; depth is 4, not
8, because ROM latency is 1 and one in-flight request plus a skid window is all a stall needs. The
buffer carries `imem_fault` with each word.

Two fixes the prototype needs: a **discard bit** — a flush must drop the response of the request
already in flight, and since ROM latency is fixed at 1 this is one bit rather than a counter; this is
the class of the prototype's open defect. And `fetch_stall` (a text load or store stealing the port)
becomes a fetch-side **retry** — `fetch_pc` holds, the response is not pushed — never a decode-side
stall.

**Kill.** A flush from X zeroes the buffer and D's valid; the word in D never issues. Nothing past X is
ever un-committed: no register write, CSR write, store, retire or `minstret` tick happens before X.
**X is the commit point.**

**D.** Decodes the buffer head, a register. Presents `rs1`/`rs2` from the word's raw fields, with
`rtl/regsel.v` instantiated **once**, on a registered word — no guess, no `operand_stall`. The
scoreboard stalls D one cycle only when X holds a load, AMO, `lr.w` or `sc.w` whose `rd` matches.
Serialization holds D until X, M and W are empty — same rule, same three slots. The D/X register
carries pc, immediate, rd, the register numbers, the `is_*` flags, the predicted target (so X needs no
second target adder) and whether a guess was taken.

**Regfile.** `rtl/regfile.v` is unchanged: synchronous EBR pair, write-first into the read register,
write-through bypass on the held pair. The presented pair is D's, except on an X hold (the divider),
when it is X's — a mux selected by a registered `divider_stall`. The invariant that replaces
ADR-0064's: **whenever X is valid, the regfile's held pair equals X's `rs1`/`rs2`**, asserted in the
`FORMAL` block and proven by k-induction. The write-through bypass then is simply the W forwarding
source.

**X.** Operands are `fwd_from_XM ? executor_out.rd_data : reg_rsN`, one 2:1 mux per operand whose
select is precomputed in D and registered. Every reader in X uses the forwarded operand: the ALU, the
branch compare (sharing `alu_sub`), the `jalr` target and the effective address (one shared `rs1 + imm`
adder, since loads/stores and `jalr` never coexist), the region test on that sum, misalignment,
`csr_arg`, `atomic_addr`. hzC's whole population moves off the fetch loop. All traps are detected and
committed here, a trap being a redirect on the path the branches use, with `mepc`/`mcause`/`mtval`
from X, and the interrupt taken on a cycle X would otherwise commit. The region test reads the X sum
directly, so `region_stall` and its deferred answer are deleted; if that test turns out to set X's
period on ECP5, the fallback is ADR-0129's one-sided deep-block fast arm restated in X.

**M and W.** Unchanged in substance. The bus transaction launches from the X/M register on the cycle X
commits it, preserving ADR-0099's `launch_taken` semantics — a re-presented request must not reach a
device twice. X/M carries store data.

**C extension.** The buffer's two outputs and `rtl/fetcher.v`'s 16-bit windowing are unchanged; a
32-bit instruction straddling two fetched words is the existing word-boundary pop. Branch targets at
2-byte alignment need no extra cycle, where Hazard3 pays one.

**Stall broadcast.** Reasons shrink from eight to six: divider (hold), atomic write cycle, load-use
scoreboard, serialization, buffer-empty (replacing the stolen window), ungranted bus. `operand_stall`
and `region_stall` are deleted. `kill` becomes a new **non-stall bubble** category — exactly the
accounting ADR-0078 said this would need — so `make cycles` gains `kill` and `predict` columns and its
identity becomes issues + stalls + kills = cycles.

## up5k area budget

Estimates against ~4,950 placed SoC LC, about 330 free. Sign is cost.

| | LC |
|---|---|
| 4-word buffer: 128 flops, count/head/tail, two 32-bit output muxes | +180 to +220 |
| D/X register regrowth, less `mem_addr`'s 32 and 64 operand-value flops it no longer needs | +20 to +40 |
| X forwarding, 2 × 32-bit 2:1 (today's executor-only mux moves rather than doubles) | +10 to +30 |
| D prediction adder, BTFN and `jal` recognition | +45 |
| Deleted: second `regsel`, the read muxes, `prev_rs1/2`, `operand_stall` | −60 to −100 |
| Deleted: decode's `cmp_sub` (shares `alu_sub`) and the `pc + pc_inc` adder | −65 |
| Deleted: the region deferred answer and deep-block tests | −40 |
| `next_pc`'s five-arm 32-bit priority chain becomes a three-arm `fetch_pc` mux | −30 |
| **Net** | **+100 to +250** (worst plausible +400) |

Reserve: `rtl/spiflash.v` is wired to no board pin (ADR-0135) and is the first thing to drop from the
up5k SoC if the fit misses — roughly 80–100 LC. `FIT_MAX_LC` is re-derived by its own span method
after Stage B; `SOC_EXPECT_EBR` is unchanged, since the buffer is flops.

## Key decisions

1. **Branches resolve in X** — not D, which has no register values, and not M, which would cost a
   third mispredict cycle for a clock gain the up5k cannot spend.
2. **Static prediction only: BTFN and `jal`, read from the buffered word.** No table, no register
   read. ADR-0188 measured BTFN cutting Dhrystone's mispredicts from 16.93% to 6.26% of issues. A
   `jalr` return-address guess waits until its share is measured. **"Mispredict" means
   `resolved_target != guessed_target`**, never an instruction class — that was ADR-0188's recorded
   bug.
3. **The fetch buffer is a 4-word FIFO with a one-bit discard** — depth follows from ROM latency 1,
   and the discard bit is the fix for the prototype's open defect.
4. **All traps commit in X, and X is the single commit point.** Replaces commitment 2. Nothing in M or
   W can fault; the platform still answers a refusal with the address, read in X.
5. **Forwarding runs from X/M into every X reader, plus the regfile's existing W bypass, and load-use
   is the only RAW stall.** Replaces commitment 4 and ADR-0083's exception list.
6. **The regfile is unchanged and presents D's pair, or X's on an X hold.** Replaces commitment 6; the
   guess, `operand_stall` and ADR-0064's coupling argument are deleted.
7. **Wrong-path state is confined to fetch and D, and one registered `kill` clears it.** Replaces
   commitment 1.
8. **Two merges, each shippable and measured at 12–16 paired seeds on both parts.** Stage A costs
   about 3% of up5k cycles and buys roughly 10–15% of ECP5 product plus up5k margin; it lands with a
   dated ADR naming Stage B as the recovery, because the alternative — one three-month branch — is how
   ADR-0047's equivalence run diverged.
9. **Depth derivation moves with the stage count.** F and G are re-measured by
   `make -C formal remeasure-fg` at each stage before any check runs, never edited.

## Replacement wording for the commitments

- **(1) Wrong-path state stops at issue.** A word may be fetched, buffered, guessed at and decoded
  before it is known to be wanted, and one registered `kill` from X discards it; nothing X has
  committed — a register write, a CSR write, a store, a retire, a `minstret` tick — is ever revoked.
  The fetch address is a register whose update reads registers only; the word arriving this cycle
  never reaches it. Enforced by `formal/pcloop.sv`, `rtl/decoder.v`'s `FORMAL` block (`kill ⇒
  !issuing`) and the generated `pc_fwd`/`pc_bwd` checks.
- **(2) All traps are detected and committed in X.** A trap is a redirect on the path the branches
  use; the platform's refusal still arrives with the address, read in X; nothing in M or W faults.
  Enforced by `components_traps` over `formal/traps.sv`, in two hops: X registers the target, and the
  next issuing pc is that target.
- **(4) Hazards forward, and one stalls.** X reads every operand through one mux fed by X/M's result
  and the regfile's write-through bypass; the scoreboard holds D one cycle only when X's `rd` is a
  load's, an AMO's, `lr.w`'s or `sc.w`'s still-unpacked result. RVFI reports the forwarded value.
- **(6) The regfile read is synchronous and answers the pair D presented for the instruction now in
  X.** On an X hold it is X's pair that is presented, so the answer is always X's operands; there is
  no guess and no operand-fetch cycle. Standing probe: delete the write-through bypass and `reg_ch0`
  must go SAT.
- **(8) Stalls are one broadcast over six reasons and two mechanisms, and a kill is not a stall.** The
  divider holds; the other five bubble; `kill` is charged before issue in `make cycles`, and a cycle
  that issues nothing must be a stall or a kill.
- The **two loops** paragraph is replaced by: **the period is set by D or X, never by fetch.** D is
  ROM/buffer output → window → decode → scoreboard → stall fan-out. X is forward mux →
  `alu_sub`/address adder → verdict → `fetch_pc` mux. Every ceiling on the old loop (ADR-0076, 0078,
  0083, 0087, 0091, 0092, 0097, 0100, 0113, 0129) is retired with a pointer here.

## Risks and unknowns

**up5k fit**, the largest: +100 to +400 LC against about 330 free. Settled by `make fit` and the SoC
placement at Stage A and again at Stage B, with the reserve named above. If it is still over, the
depth-2 skid — no output mux, shifting work toward D — is the fallback at roughly −100 LC.

**Formal.** `pcloop`'s `pc == $past(next_pc)` and `past_pc + 4 == pc` are false by construction and
split into three properties: `fetch_pc` advances by 8 or takes a registered target; the buffer's word
and pc stay consistent; a killed word never issues. The architectural pc chain is what riscv-formal's
`pc_fwd`/`pc_bwd` already grade at retire. ADR-0078 found that this induction closes *more* easily,
one state bit replacing a transcribed redirect list. `formal/traps.sv` builds decoder and CSRs without
the core and must gain the X stage, since traps commit there — the largest single piece of formal
work. **F and G grow**: first retire +2 for buffer fill and the extra stage, worst gap +2 for a
mispredict, so F+2G goes 18 → ~24 and the generated depths rise with it against the formal job's
20-minute wall. Measure with `remeasure-fg` at Stage A; if the wall is hit, shard the check set. The
buffer adds no core input, so `INTERRUPT_TIE_OFF` and `MULTIHART_TIE_OFF` are unchanged.

**RVFI and the monitor.** `rvfi_pc_wdata` must be X's resolved next pc, never the guess, and a killed
word must never reach `rvfi_valid`. `test/cosim.cc` reads `regs_a` and is the grader that cannot be
fooled by a consistently wrong forward.

**Zkt.** The predictor reads instruction bits only and forwarding stalls nothing; `region_stall`, the
one register-reading stall, is gone, which strengthens the claim. `test/zkt_isolation_test.py`'s
`STALL_TARGETS` and both `-zkt-probe.py` red directions must be **re-derived, not edited to pass**.

**Readability.** ADR-0078 charged its candidate four new concepts. This adds three — buffer, kill,
predict — and deletes seven: the pair guess and its second `regsel`, `operand_stall`, the held-pair
coupling, the region deferred answer, the forwarding eligibility exception list, two odd members of
the eight-reason stall taxonomy, and the two-loops rule. A reader who knows the textbook pipeline
reads it in one sitting; today's reader needs twelve ADRs.

**Dual hart.** `rtl/littledual.v` drives `bus_wait`; the buffer's retry on `fetch_stall` and the
bubble on `bus_wait` keep the arbiter contract, and `make dual-smoke` is the grader.

**Open until built:** the load-use share; whether D or X sets ECP5's period after Stage B; and whether
the CSR-file path that set ADR-0078's ceiling still exists on this tree, since it sets the up5k
ceiling afterwards.

## Sequence

**The trace model is cut.** An earlier draft opened with a Spike-style replay model that would predict
the cycle gain before any RTL existed, with a kill gate if the modelled Dhrystone gain came in under
8%. The owner cut it: a fetch stage is standard practice, and the merged fetch-and-decode is the
identified cost, so the direction does not need a model's permission. What that costs is worth stating
plainly — **the first real evidence now arrives at the end of Stage A instead of three days in**, and
Stage B's original kill criterion, "modelled and measured cycles differ by more than 5%", loses the
half that made it meaningful. It is replaced below by an absolute measured target.

**Stage A — register-only fetch on today's decode** (~1–2 weeks). Start from the spike branch's
`soc/fetch_ahead/prototype-decoupled.patch`; depth 4; the discard bit; BTFN and `jal` guessed from the
buffer head; a `kill` bucket in `test/cxxrtl.cc` and `test/stall_report.py`; `fetchqueue_tb.v` with
forced-red vectors in `probe-gates`; `decoder_tb.v` retimed; `pcloop` and `traps` rewired;
`remeasure-fg`; 16 paired up5k seeds and 12 ECP5 seeds; `make fit`.
*Expected:* suite 75/75, co-sim matching its baseline, ECP5 ≥ 37 worst-of-12, up5k ≥ 12.5
worst-of-16, Dhrystone cycles +3%, fit ≤ 5,100 placed.
*Kill:* the SoC does not place under 5,280 at depth 4 (→ depth-2 skid, then the reserve); ECP5
worst-of-12 under 35, meaning no structural gain and the whole direction is wrong; any co-sim
divergence not already in the baseline.

**Stage B — the D/X split** (~3–5 weeks). The regfile presented from the buffered word; operands,
forwarding, compare, address, region, traps and interrupt in X; `operand_stall` and `region_stall`
deleted; the accessor launching from X/M; `traps.sv` gaining X; `decoder_tb.v` and `exec_tb.v` split;
commitments 1, 2, 4, 6 and 8 amended in CLAUDE.md with the ADR that measures them; `FIT_MAX_LC`
re-derived; the `netlist-digest` sweep owed by rule.
*Expected:* Dhrystone ≤ 257,000 cycles and CoreMark ≤ 426,000; up5k worst-of-16 ≥ 13.8; ECP5 ≥ 42;
fit under 5,280 with ≥ 150 LC of slack.
*Kill:* Dhrystone above 270,000 after Stage B — that is −14% against a −19% estimate, and missing by
that much means the cycle model behind this plan is wrong and the remaining work should be re-planned
rather than continued; up5k under 12.0 at any of 16 seeds, which is a requirement and not a floor; the
`reg_ch0` liveness probe ceasing to fire.

**Stage C — re-stamp the comparison.** `make compare-product` at 12 seeds on both parts, both
toolchain halves in one session; refresh the CLAUDE.md figures; retire the fetch-loop dead-end list
into one pointer. Then measure the `jalr` and return share and decide the return-address guess on its
own numbers.

## Deferred

Dynamic prediction (BHT/BTB) — area the up5k does not have; measure the static residual first. A
`jalr` return-address guess — cheap, one 32-bit register, but decide it on a measured share rather
than on folklore. 24 MHz on the up5k — a separate CSR-file and SoC-glue depth campaign, after Stage
B's sweep names the ceiling path. M-stage branch resolution — only if X sets ECP5's period after
Stage B. And, unaffected: the radix-4 divider (ADR-0038), 16 KB of up5k text, and `nano/`.
