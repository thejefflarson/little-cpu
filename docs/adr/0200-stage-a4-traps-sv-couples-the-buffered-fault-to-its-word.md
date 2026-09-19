# 0200 — Stage A4: `formal/traps.sv` couples the buffered fault to its word, and the
decoder bench needed no further retiming

Status: Accepted. 2026-09-18.

## What this is

The fourth of the Stage A merges `docs/ideas/the-fetch-address-reads-registers.md` describes.
ADR-0196 wired `formal/traps.sv` for the decoupled fetch at the minimum the mechanical rename
needed — `q0`/`q1` took the free inputs `imem_data`/`imem_data2` already had, `buffer_empty`
joined as a new free input, `hard_stall` and `ls_answer_valid`'s hold condition were fixed to
read it — and said so explicitly: "minimum wiring." This ticket closes that out: it rewires
`imem_fault`'s own standing to match the interface `rtl/fetchctrl.v`/`rtl/fetchqueue.v` actually
present, re-derives both forced-red probes against the current tree, and re-confirms F, G, the
generated check depths and the memcheck depths rather than assuming ADR-0196's numbers still
hold after the depth-2 skid was attempted and abandoned on this branch.

## The gap: a fault bit free of the word it belongs to

`rtl/littlecpu.v` wires `decoder`'s `imem_fault` port to `queue_q0_fault`, one of
`rtl/fetchqueue.v`'s own outputs, not to anything `rtl/imemory.v` answers this cycle. Inside
`rtl/fetchqueue.v`, a push writes `mem[tail]`/`mem[tail+1]` and `fault_mem[tail]`/
`fault_mem[tail+1]` from the same `imem_data`/`imem_data2`/`imem_fault` sample in the same
cycle, and a read presents `q0`/`q0_fault` out of the same `head` index. A word and its own
fault bit are always written together and read together, at the same array slot, by
construction — there is no path for one to reach decode without the other.

`formal/traps.sv`'s own freeze-consistency assumption already modeled half of what that buys:
"a stalled cycle re-presents the same word," `assume(fetcher_out == prev_fetcher_out)` when the
previous cycle did not issue. `imem_fault` was left out of it — still a fully free input, the
same standing it always had, with nothing tying it to `fetcher_out`'s own held value. That is an
over-approximation (a strict superset of what real hardware can present), not an unsound one, so
it never let a real defect hide; `components_traps` proved under it before this change and
proves under the tighter version after. But it did not describe the interface Stage A actually
built, which is what this ticket asks for. Fixed: `imem_fault` now joins the same freeze
assumption, registered as `prev_held_imem_fault` and asserted equal to it whenever the previous
cycle did not issue, with a comment at both the port and the assumption site saying why —
`rtl/fetchqueue.v` answers a fault bit out of the same slot its word comes from, so the two can
no more diverge across a held cycle than `fetcher_out`'s own fields can.

## Commitment 2, checked against the buffered fault

Commitment 2's rule is that a refusal counts as committed in decode only when it arrives with
the address, in the cycle decode reads the word. A buffered fault's address and its own fault
bit are no longer computed in the same cycle they are read — the fault bit was decided whenever
`rtl/imemory.v` answered the request that filled this queue slot, cycles before decode gets to
it. The rule still holds, on two facts neither this ticket nor ADR-0196 had to add:

1. **The queue never separates a word from its own fault bit.** `rtl/fetchqueue.v`'s push writes
   both from one sample in one cycle at one index, and its pop reads both from that same index.
   There is no index or cycle skew between them by construction — no formal property is needed
   to state this, since nothing in the module can express reading one without the other.
2. **Decode never separates a word from its own address.** `rtl/decoder.v`'s mtval mux reports
   `in.pc` unconditionally on an `imem_fault` trap, never a different register's value or a
   stale copy — checked here, in the mtval arm (`assert(csr_rdata == prev_tval)`, with
   `prev_tval` built from `past_pc` for this cause), and probed red by `traps-tval-probe.py`'s
   `wrong-addr`/`wrong-word` cases. `formal/pcloop.sv`'s own word/pc consistency property
   (ADR-0196) is the complementary half, over in `rtl/fetchctrl.v`'s and `rtl/fetcher.v`'s
   territory: it is what says the pc decode reports for the word at the queue's head is that
   word's own fetch address, not a stale one left over from before a redirect.

Composed, a buffered fault still "arrives with the address" under commitment 2's original test —
the address and the fault travel together through the queue, and decode reports exactly the
address it was handed, never a different one. The mechanism moved from "one ROM read, same
cycle" to "one array read, same slot," and the guarantee survives the move.

## `test/decoder_tb.v`: already retimed, one line over budget

ADR-0196 already renamed every `fetch_stall` vector to `buffer_empty` — the OR-identity check,
the region-wait task, and every stall/hold/interrupt vector that drives it — and no
`fetch_stall` reference remains anywhere in the file. `decoder_tb.v` drives `imem_fault` and
`in` combinationally, one cycle at a time: it is a statement about what `rtl/decoder.v` does
with whatever it is handed a given cycle, never about whether the queue kept a word and its
fault bit paired across cycles — that half is `formal/traps.sv`'s and `rtl/fetchqueue.v`'s job,
not a unit bench driving the decoder alone. Read start to finish, its region vectors (the
`region_access` task, the deep-RAM/deep-text fast-path vectors, the two-block boundary vectors,
the misaligned-out-of-region vectors) already reflect the buffered gate — `ls_answer_valid`'s
`bus_wait || buffer_empty` hold condition — so no vector needed retiming for this ticket. No
functional change landed here.

## The depth machinery's own probe was stale, not just `checks.cfg`'s numbers

`test/probe_gates.sh`'s `formal/check-memcheck-depth.py` group builds a synthetic `checks.cfg`
with its own hardcoded `#derive F 6`/`#derive G 6` lines and a `fixture_anchor` guard that reads
the real `formal/checks.cfg` to confirm that text still appears there verbatim — the same
freshness discipline `docs/comment-budget.md`'s neighbours use elsewhere in this file. ADR-0196's
F 6 → 8 / G 6 → 8 move left that anchor pointing at text that no longer exists, so `make
probe-gates` stopped dead at `formal/check-memcheck-depth.py`'s group with `fixture anchor stale:
'#derive F 6 ...' is no longer in .../formal/checks.cfg` — never reaching a single probe after
it, `test/probes_header_test.py` included. This is squarely "the depth machinery" this ticket
owns, not A1's or A2's RTL, and is fixed here: both anchors, the synthetic `checks.cfg`'s own
`#derive` lines, and every depth number the group's probes assert against are moved from F=6/G=6
to F=8/G=8 (the floor was F+G+2=14 → 18, F+2=8 → 10), with three probes that reused a depth of 14
purely as "some depth past the floor" (a cover-depth mismatch, a missing cover sibling, a
cover `.sby` with no depth line) bumped to 18 so they still clear the new floor and reach the
logic they exist to test; the remaining `mcd_fixture 14` reuses (a missing `.sby`, a missing
depth line, an invalid `<retires>` argument, a doubled `depth` key, a depth line outside
`[options]`, a `checks.cfg` missing `#derive G`) all fail before `check-memcheck-depth.py` ever
reaches its floor comparison, traced through the script's own control flow rather than assumed,
so they needed no change.

`formal/remeasure-fg.py`'s own probe group carried the same class of drift, found running
`make probe-gates` after the fix above got past it: `remeasure-fg.py` sweeps `BELOW`/`ABOVE=3,3`
cycles either side of the *declared* F, so its first swept row moved from `F-3=3` to `F-3=5`, and
a fixture asserting the exact literal string `"not the 3 this row swept"` stopped matching what
the script now prints (`"not the 5 this row swept"`); the fix is that one literal. A second
fixture narrows `BELOW`/`ABOVE` to `2,1` and stubs `sby` to PASS only once the swept
`RISCV_FORMAL_CHECK_CYCLE` reaches a flip point three cycles past the declared F, to prove a
narrowed window cannot bracket it — the stub's threshold was hardcoded to `8` (three past the old
F=6) and is now `11` (three past F=8), or the new window (`F-2=6` to `F+1=9`) would have bracketed
the old threshold by coincidence and reported a normal flip point instead of the "too narrow"
refusal the probe exists to demonstrate.

## Comment density, two files over budget on this branch and nine that stay that way

`test/comment_density_test.py`, a `make test` prerequisite, failed on eleven files measured at
the start of this session, two of them this ticket's own: `formal/traps.sv` was already over its
5% budget before this ticket touched it (24/409 lines, 5.9%, from ADR-0196's own additions), and
this ticket's `imem_fault` fix would have pushed it further; `test/decoder_tb.v` sat right on the
line ADR-0196 left it at (62/1237, past 5.0% once read to full precision). Both are fixed here —
restatement trimmed, a few explanations folded from two lines to one, no comment's content lost
— and both now measure clean. The other nine files over budget
(`Makefile`, `formal/components.sby`, `formal/imemcheck.sv`, `formal/pcloop.sv`,
`rtl/decoder.v`, `rtl/fetchctrl.v`, `rtl/fetcher.v`, `rtl/fetchqueue.v`, `rtl/littlecpu.v`) are
Stage A1's and Stage A2's own additions, several of them files this ticket is explicitly told to
stay out of; they are not touched here and stay red under `comment-density-test`, which means
`make test` does not reach `test/run_tests.sh` on this branch at all — it stops at
`comment-density-test`, a prerequisite ahead of the suite in `test`'s own dependency list. The
suite, the unit benches, lint and `elaborate-strict` were each run directly instead (below), and
`make probe-gates` was run standalone, since it is an earlier `test` prerequisite than
`comment-density-test` and reaches its own end regardless of that file's standing.

## Both probes re-derived

`rtl/decoder.v` is unchanged by this ticket, and the depth-2 skid's abandonment (the commit
immediately ahead of this branch) touched no line either probe mutates:
`traps-region-probe.py`'s `no-trap` and `wrong-cause` cases still replace the same `ls_fault`
assign and the same cause-swap arms, and `traps-tval-probe.py`'s `wrong-addr`/`wrong-word` cases
still replace the same two arms of the mtval mux. Re-run fresh against the edited `traps.sv`:

```
no-trap:     FAIL, assertions failed at [437]   (assert(trap_entry))
wrong-cause: FAIL, assertions failed at [388]   (assert(csr_rdata == prev_cause))
control:     PASS, assertions failed at none
wrong-addr:  FAIL, assertions failed at [396]   (assert(csr_rdata == prev_tval))
wrong-word:  FAIL, assertions failed at [396]
```

Each mutation still fails at its own named arm and nowhere else, so neither probe was edited to
pass — both demonstrate red before `components_traps` is read as evidence of anything.

## F, G and every generated depth reconfirmed, not assumed

`formal/traps.sv` is not one of `formal/Makefile`'s `RTL_SOURCES`, so a formal-harness-only edit
cannot move F or G by construction; `make -C formal remeasure-fg` was still run fresh, on this
tree, rather than trusting that fact by inspection alone: **F = 8, G = 8**, reproducing
ADR-0196's own figures exactly (flip points at check cycle 9 for `hang` and gap 8 for `liveness`
at both trigger depths). `make -C formal checks` (`genchecks-audit.py`): 86 checks generated, all
at or above the F=8/G=8 floor, `EXPECTED_CHECKS` an exact match both ways.
`check-memcheck-depth.py`: `imemcheck` 15 ≥ F+2 = 10, `dmemcheck` 20 ≥ F+G+2 = 18, both
unaffected (`formal/traps.sv` is not part of either memcheck's build either). `components_traps`
passes by k-induction on the edited harness (basecase and induction both `pass`, ~30-50s locally).

## The BMC wall

CI shards the generated check set four ways (`formal-checks-shard`, `CHECK_SHARD=<i>/4
JOBS=4`), 20-minute wall each. Run locally at the same `JOBS=4` shard width (not the full
`nproc`-wide `make check`, which is not what CI's per-shard timing measures), each check's
result read off its own `status` file's mtime rather than trusted to log flush timing:

| shard | checks | wall time | % of the 20-minute wall |
|---|---|---|---|
| 1/4 | 22 | 14m01s | 70% |
| 2/4 | 22 | 12m58s | 65% |
| 3/4 | 22 | 13m58s | 70% |
| 4/4 | 21 | **24m07s** | **121%, over the wall** |

Shard 4 was measured twice — 33m57s under heavy contention from concurrent local work (this
session's own suite runs and `probe-gates` re-runs sharing the same 10 cores), 24m07s with that
contention cleared — and even the cleaner figure is the one to trust least literally (a laptop is
not a CI pod, ADR's own "different instruments" rule) but the one most worth flagging: shard 4's
own membership (round-robin by sorted name, `NR % 4 == 0`) lands `insn_div_ch0`, `insn_rem_ch0`
and `pc_fwd_ch0` together, and `pc_fwd` is one of the checks ADR-0196 already names as sitting at
the deepest tier next to `hang`/`liveness`. Shards 1-3 are not interchangeable with shard 4's own
number by construction. **This is the wall the brief predicted** ("F+2G goes 18 → ~24 ... against
the formal job's 20-minute wall") arriving, not yet from every shard but from the one the round
robin happened to load heaviest — shard rather than shorten a depth is still the right call, but
the four-way split itself may not be even enough to keep every shard under the wall much longer;
a rebalanced (not merely re-numbered) shard split, or a wider shard count, is the next stage's to
consider if F or G grows again.

## Verification, run in full

- `components_traps` (k-induction): PASS, both probes demonstrated red first (above).
- `make -C formal remeasure-fg`: F=8, G=8, reproducing ADR-0196.
- `make -C formal checks`: 86 checks, all at/above floor; `EXPECTED_CHECKS` exact match.
- `check-memcheck-depth.py`: `imemcheck` 15≥10, `dmemcheck` 20≥18.
- `formal/COMPLETE_EXCLUSIONS`: unchanged — this stage adds no instruction.
- `make -C formal all`: **fully green** — `complete`, `complete_cover`, all 86 generated checks
  (`check-baseline.sh`: "86 checks: 86 pass, 0 fail", `EXPECTED_FAIL`/`EXPECTED_CHECKS` exact
  matches), `dmemcheck`/`imemcheck` and both their covers, and all six `components_*`
  k-inductions (`decoder`, `executor`, `accessor`, `pcloop`, `traps`, `busarbiter`) — the full
  hand-written proof set this ticket does not directly touch, confirmed unbroken.
- The suite (`STALL_REPORT=1 ./test/run_tests.sh ./sim test/asm test/EXPECTED_FAIL
  test/OBSERVED_FLOOR`, run directly against the already-built `./sim`): **75/75**, matching
  `test/EXPECTED_FAIL` and `test/OBSERVED_FLOOR`'s shape exactly — unaffected, as expected, since
  no RTL changed.
- `make test-units`: all 14 unit benches pass, `fetchqueue_tb.v` included, unmodified.
- `make lint`: clean, both passes (RVFI macros off and on).
- `make elaborate-strict`: clean, no warnings beyond the standing `Deep recursion` allowlist
  entry.
- `make probe-gates`, run standalone: **green**, "86 graded comparisons" reaching
  `test/probes_header_test.py`'s group at the end with none red, once both stale fixtures above
  were re-derived and this ADR's own README row landed (`test/adr_numbering_test.sh`'s group reads
  the working tree, not a merged one, so an ADR with no index row is exactly the orphaned-row case
  it is written to catch — not a bug in the checker). One remaining red is not this ticket's:
  `test/mutation_coverage_test.sh`'s control fails naming `rtl/fetchctrl.v` with no
  `test/MUTATION_COVERAGE` ruling — Stage A1's file, gaining no ruling when it landed, left for
  whoever owns it.
- `make test` overall: does not complete — stops at `comment-density-test`, red on nine files
  this ticket does not own (see above), a pre-existing condition on this branch this ticket
  neither introduced nor is positioned to fix.

## Consequences

- `components_traps` proves a strictly more faithful model of the buffered fetch than ADR-0196's
  minimum wiring did, with no change to what it was already proving about traps, CSR state or
  `mtval` — the tightened assumption is an over-approximation narrowed toward reality, not a new
  claim.
- Commitment 2 is confirmed to survive the buffered fetch, on the composition of a fact intrinsic
  to `rtl/fetchqueue.v`'s array (word and fault share an index) and a fact already proved about
  `rtl/decoder.v` and `formal/pcloop.sv` (decode never reports a stale address). No new formal
  property was needed to state this — it falls out of what Stage A1 already proved plus how the
  queue is built — but it is recorded here since the brief asked the question explicitly.
- A stage that moves F or G owes a `test/probe_gates.sh` audit of `formal/check-memcheck-depth.py`
  and `formal/remeasure-fg.py`'s own probe groups, not just `formal/checks.cfg`'s declared numbers
  and the generated depths — both groups build synthetic fixtures with the old F/G baked into
  literal strings and a stub's flip-cycle threshold, and neither is exercised by anything on
  `make test`'s path except `make probe-gates` itself, which is exactly what let this drift stand
  since ADR-0196.
- **DECISION NEEDED, flagged for the architect**: `make test` cannot complete on this branch
  independent of this ticket — `comment-density-test` fails on nine files across Stage A1 and
  Stage A2's own work, none of them this ticket's to fix under its stacking instructions. Whoever
  lands the stage that owns each file (A1's `rtl/fetcher.v`/`rtl/fetchqueue.v`/`rtl/littlecpu.v`
  and `formal/pcloop.sv`/`formal/imemcheck.sv`/`formal/components.sby`, A2's
  `rtl/fetchctrl.v`/`rtl/decoder.v`, and whoever last touched `Makefile`) owes a trim before the
  stack can present one green `make test` run. `rtl/fetchctrl.v` owes `test/MUTATION_COVERAGE` a
  ruling too, caught the same way.
- **The BMC wall is arriving, unevenly**: shard 4/4 measured over CI's 20-minute wall locally
  (above); Stage A2's own kill path is the next thing to grow F or G, and it should re-measure the
  four-way shard split's wall time, not just F+2G, before it lands — a rebalanced split (by
  measured cost, not sorted name) or a wider shard count is the more likely fix than a fifth
  measurement showing the same shape.
