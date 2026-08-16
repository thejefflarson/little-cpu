# Architecture Decision Records

Load-bearing decisions for Little CPU. Each ADR records the context, the decision, why it was made,
and what it costs. Reversing one is fine — write a new ADR that supersedes it.

| # | Decision | Status |
|---|---|---|
| [0001](0001-finish-the-staged-rewrite.md) | Finish the staged rewrite rather than resurrect the serialized core | Accepted |
| [0002](0002-isa-target-rv32imc-zicsr.md) | ISA target is RV32IMC_Zicsr_Zifencei, machine mode only | Accepted · Zifencei claimed by 0048 |
| [0003](0003-dual-word-combinational-fetch-window.md) | Dual-word combinational fetch window for compressed instructions | Accepted |
| [0004](0004-stall-only-hazard-interlock.md) | Stall-only hazard interlock with a combinational-read regfile | Accepted |
| [0005](0005-traps-and-csrs-commit-in-decode.md) | Traps and CSR accesses are detected and committed in decode | Accepted |
| [0006](0006-port-the-wave-0-formal-harness.md) | Port the wave-0 formal harness; RVFI via per-stage shadow payloads | Accepted |
| [0007](0007-cxxrtl-is-the-primary-simulator.md) | cxxrtl is the primary simulator; iverilog is the microscope | Accepted |
| [0008](0008-test-memory-map-and-tohost-protocol.md) | Test memory map and the `tohost` protocol | Accepted |
| [0009](0009-stall-protocol-semantics.md) | Stall protocol — upstream freezes, downstream drains | Accepted |
| [0010](0010-muldiv-verification-under-altops.md) | A randomized differential bench is the primary mul/div guarantee | Accepted |
| [0011](0011-misalignment-detection-stays-in-accessor-until-m3.md) | Misalignment detection stays in the accessor until M3 | Accepted |
| [0012](0012-divider-is-unsigned-signs-are-a-wrapper.md) | The divider is unsigned; sign handling is a wrapper around it | Accepted |
| [0013](0013-the-riscv-formal-pin-is-an-enforced-control.md) | The riscv-formal pin is an enforced control, not a comment | Accepted |
| [0014](0014-expected-fail-is-the-m1-regression-baseline.md) | `test/EXPECTED_FAIL` is the M1 regression baseline | Accepted |
| [0015](0015-accessor-load-response-stall.md) | The accessor's load-response turnaround is the third stall source | Accepted |
| [0016](0016-no-unreviewed-dependabot-automerge.md) | No unreviewed auto-merge for the actions dependabot updates | **Superseded by 0018** |
| [0017](0017-component-proof-assumptions-must-name-their-model.md) | A component-proof `assume` must name the structural fact it models | Accepted |
| [0018](0018-dependabot-automerge-gated-on-required-checks.md) | Dependabot auto-merge is re-armed, gated on required status checks | Accepted |
| [0019](0019-the-monitor-sanitizer-is-the-place-to-fix-generated-monitor-defects.md) | The monitor sanitizer is where generated-monitor defects get fixed | Accepted |
| [0020](0020-the-rvfi-non-perturbation-guarantee-is-argued-not-proven.md) | The RVFI non-perturbation guarantee is argued, not proven | Accepted · consequences rewritten by 0047 |
| [0021](0021-the-formal-ladder-runs-the-compressed-checks.md) | The formal ladder runs the compressed checks, and it found a real bug | Accepted |
| [0022](0022-the-formal-nightly-reports-against-an-explicit-baseline.md) | The formal nightly reports against an explicit baseline, not `\|\| true` | Accepted · corrected by 0037 |
| [0023](0023-the-first-ladder-run-does-not-reach-m2.md) | The first ladder run does not reach M2 — three named holes | Accepted |
| [0024](0024-the-ladders-default-bmc-engine-is-btormc.md) | The ladder's default BMC engine is `btor btormc`, not `smtbmc yices` | Accepted |
| [0025](0025-formal-ladder-depths-are-derived-not-inherited.md) | The ladder's depths are derived from the pipeline, not inherited | Accepted · numbers superseded by 0046 |
| [0026](0026-stalls-are-four-reasons-over-two-mechanisms.md) | Stalls are four reasons over two mechanisms | Accepted |
| [0027](0027-minstret-counts-non-trapping-issues.md) | `minstret` counts non-trapping issues; serialization buys exactness | Accepted |
| [0028](0028-the-rvfi-convention-for-a-trapping-retire.md) | The RVFI convention for a trapping retire | Accepted · corrected by 0037 |
| [0029](0029-mtvec-resets-to-zero-and-a-pre-handler-trap-is-loud.md) | `mtvec` resets to zero, and a pre-handler trap is made loud | Accepted |
| [0030](0030-trap-cause-priority-and-why-the-causes-are-disjoint.md) | Trap cause priority, and why the causes are disjoint | Accepted |
| [0031](0031-the-vendored-genchecks-copy-tracks-the-pin.md) | The vendored `genchecks` copy tracks the pin, and only `basedir` differs | Accepted |
| [0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md) | Sail co-simulation is worth building, and stays opt-in | Accepted · its "stays opt-in" decision amended by 0095, whose precondition it named |
| [0033](0033-what-the-green-ladder-does-not-cover.md) | What a green ladder does not cover — three assurance gaps, named | Accepted · decision 4 closed by 0095 |
| [0034](0034-what-the-csr-ladder-checks-cannot-see.md) | What the CSR ladder checks cannot see, and the decisions the CSR file forced | Accepted · corrected by 0037 |
| [0035](0035-the-baseline-pins-the-failure-mode.md) | `test/EXPECTED_FAIL` pins the failure mode, not just the file name | Accepted |
| [0036](0036-three-gate-hardening-decisions-ratified-at-integration.md) | Three gate-hardening decisions ratified at integration, and a correction to ADR-0031 | Accepted |
| [0037](0037-an-empty-baseline-is-not-m2.md) | An empty formal baseline is not M2, and the milestone criterion said it was | Accepted · amended by 0045, 0047; its six terms ruled on by 0079 |
| [0038](0038-area-is-measured-in-logic-cells-and-two-levers-are-rejected.md) | Area is measured in logic cells, Fmax is declared at 12 MHz, and two area levers are rejected | Accepted · decision 2 amended by 0066, which makes 12 MHz a requirement |
| [0039](0039-co-simulation-runs-the-whole-suite-against-a-baseline.md) | Co-simulation runs the whole suite against a baseline, and `tohost` becomes a doubleword | Accepted |
| [0040](0040-the-ladder-refuses-a-negedge-regfile-and-make-check-was-re-grading.md) | The ladder refuses a negedge regfile rather than mis-modelling one, and `make check` had been re-grading the previous run | Accepted |
| [0041](0041-integration-decisions-from-the-fit-cosim-and-negedge-wave.md) | Integration decisions from the fit / co-simulation / negedge wave | Accepted |
| [0042](0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md) | The regfile read is synchronous and costs a cycle, and the ladder is told that instruction memory is memory | Accepted |
| [0043](0043-the-reference-model-is-configured-as-this-core.md) | The reference model is configured as *this* core, and what is left over is exempted by name | Accepted |
| [0045](0045-two-m2-terms-close-by-amendment-and-one-was-already-met.md) | Two M2 terms close by amendment, one was already met, and the measurements that force it | Accepted |
| [0046](0046-the-ladder-depths-are-re-derived-and-the-derivation-is-measured.md) | The ladder's depths are re-derived for the five-reason pipeline, and the derivation is measured | Accepted · supersedes 0025 |
| [0044](0044-what-the-memory-system-has-to-be.md) | What the memory system has to be, and why today's placeholders cannot be it | Accepted |
| [0047](0047-non-perturbation-is-proved-structurally-and-equiv-sh-is-retired.md) | The RVFI non-perturbation guarantee is proved structurally, and `equiv.sh` is retired | Accepted |
| [0048](0048-what-an-independent-read-of-the-no-oracle-rtl-found.md) | What an independent read of the no-oracle RTL found: `c.ebreak`'s cause, a counter carry, and two missing CSRs | Accepted |
| [0049](0049-every-formal-assume-names-its-scope-and-its-discharge.md) | Every formal `assume` names its scope as well as its discharge | Accepted · supplements 0017 |
| [0050](0050-the-nightly-is-deleted-and-its-checks-move-to-the-gate.md) | The formal nightly is deleted and its checks move to the PR gate | Accepted · supersedes 0022, rewrites 0037 term 6 |
| [0051](0051-the-multiply-proof-is-decomposed-not-mitered.md) | The multiply proof is decomposed, not mitered; the signed divide path gets its first assertions | Accepted · closes 0049 F1/F5 |
| [0052](0052-m2-term-6-is-verified-and-the-fit-ratchet-gets-a-job.md) | M2 term 6 is verified against the gate's own run, and the fit ratchet gets a job | Accepted · closes 0037 term 6 as rewritten by 0050 |
| [0053](0053-every-graded-comparison-carries-a-probe-of-its-red-direction.md) | Every graded comparison carries an executable probe of its own red direction | Accepted · extends 0035, 0033 |
| [0054](0054-the-memory-system-and-the-first-real-timing-number.md) | The memory system, and the first real timing number | Accepted · the design 0044 called for; answers 0038 decision 2 |
| [0055](0055-the-iverilog-leg-is-graded-in-ci-without-the-multi-program-runner.md) | The iverilog leg is graded in CI, without the multi-program runner | Accepted · closes part of the `make waves` bullet in CLAUDE.md's "what does not work" |
| [0056](0056-the-makefile-embedded-ratchets-get-probes-and-soc-timing-gets-a-job.md) | The Makefile-embedded ratchets get probes, and `soc-timing` gets a job | Accepted · extends 0053, follows 0052 |
| [0057](0057-what-writable-text-costs-in-ladder-depth-and-in-nanoseconds.md) | What writable text costs, in ladder depth and in nanoseconds | Accepted · re-runs 0046's derivation and 0054's timing; corrects 0054's `SOC_MIN_MHZ` |
| [0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md) | The fetch loop is not two levels too deep | Accepted · a measured null; corrects how 0054's logic-level count reads; its decode-head recommendation is withdrawn by 0076 |
| [0059](0059-text-is-writable-and-the-arbiter-lives-in-the-memory.md) | Text is writable, and the arbiter lives in the memory | Accepted · builds 0057's step 2; answers its condition 3 |
| [0060](0060-the-steal-reaches-decode-as-the-sixth-stall-reason.md) | The steal reaches decode, as the sixth stall reason | Accepted · builds 0057's step 3 and lands its condition 1; amends 0026, 0009 |
| [0061](0061-fence-i-has-to-serialize.md) | `fence.i` has to serialize | Accepted · amends 0002; lands with 0060 |
| [0062](0062-twelve-megahertz-is-reachable-and-the-bypass-select-is-the-cost.md) | Twelve megahertz is reachable, and the write-through bypass select is the cost | Accepted · corrects 0058's decode-head attribution and its second-path cap |
| [0063](0063-the-suite-runs-programs-that-use-writable-text.md) | The suite runs programs that use writable text | Accepted · builds 0057's step 5 less its `.data` half; discharges 0061 |
| [0064](0064-the-write-through-bypass-is-addressed-from-the-held-pair.md) | The write-through bypass is addressed from the held pair, and invariant 6 now leans on invariant 9 | Accepted · implements 0062; moves `SOC_MIN_MHZ` |
| [0065](0065-the-ladder-speaks-the-writable-text-bus.md) | The ladder speaks the writable-text bus, and `imemcheck` stops at the first store | Accepted · builds 0057's step 4; pays off 0060 decision 3 |
| [0066](0066-twelve-megahertz-is-a-requirement.md) | Twelve megahertz is a requirement | Accepted · amends 0038 decision 2; answers the question 0064 left open |
| [0067](0067-the-bus-never-refuses-and-the-eleven-are-ruled.md) | The bus never refuses a transaction, and the eleven declined checks are ruled | Accepted · pays off 0044's closing obligation; settles invariant 2 |
| [0068](0068-a-one-hot-marking-is-spent-against-an-assertion.md) | A one-hot marking is spent against an assertion, not against inspection | Accepted · supplements 0030, which declines one of the two statements it was asked to mark |
| [0069](0069-a-dispatched-run-does-not-reach-the-merge-gate.md) | A dispatched run does not reach the merge gate | Accepted · amends 0013; the pin bump proposes by issue, and a PAT is rejected on public-repo grounds |
| [0070](0070-the-suites-cycles-are-charged-to-a-named-stall-reason.md) | The suite's cycles are charged to a named stall reason | Accepted · gives 0042's accepted +18.0% an invoice; measures 0026 and 0060's six reasons |
| [0071](0071-the-trap-commit-path-is-proved-and-its-priority-encoder-is-still-vacuous.md) | The trap commit path is proved by k-induction, and its priority encoder is still vacuous | Accepted · supplements 0011, 0028 and 0030; adds a fourth `components.sby` task |
| [0074](0074-the-operand-fetch-cycle-is-removable-and-costs-more-than-it-saves.md) | The operand-fetch cycle is removable, and it costs more clock than it saves | Accepted · declines a change to 0042; measured with 0070, graded by 0066; sharpens 0064's coupling |
| [0075](0075-downloaded-tools-live-outside-the-checkout.md) | Downloaded tools live outside the checkout, so a worktree can run co-simulation | Accepted · moves the install location named by 0032, 0033 and 0043 |
| [0076](0076-the-decode-head-is-a-plateau-not-a-lever.md) | The decode head is a plateau, not a lever | Accepted · withdraws 0058's recommendation; blocks 0074's reopening condition; graded by 0066 |
| [0078](0078-a-one-deep-kill-is-cheap-and-buys-a-clock-the-board-cannot-use.md) | A one-deep kill is cheap, and it buys a clock the board cannot use | Accepted · prices the no-wrong-path-state commitment; re-attributes 0076's 21%, graded by 0066 |
| [0079](0079-all-six-m2-terms-are-re-measured-and-m2-is-declared.md) | All six M2 terms are re-measured against merged main, and M2 is declared | Accepted · rules on 0037's conjunction as amended by 0045, 0046, 0047, 0050, 0051 and 0052; narrows 0027 |
| [0080](0080-twenty-four-megahertz-is-not-reachable-on-the-up5k.md) | Twenty-four megahertz is not reachable on the up5k, and the part is the constraint | Accepted · collects 0074, 0076 and 0078 into one verdict; measures the same RTL at 31 MHz on hx8k; narrows 0078's "nothing in between" |
| [0081](0081-the-data-image-lives-in-rom-and-crt0-copies-it.md) | The `.data` image lives in ROM and a `crt0` copies it, and the 8 KB budget is measured | Accepted · closes the copy-stub half 0054 deferred; keeps 0044's SPI-flash boot deferred; widens 0035's suite contract |
| [0082](0082-the-machine-timer-interrupt-is-taken-at-a-decode-boundary.md) | The machine timer interrupt is taken at a decode boundary | Accepted · amends `CLAUDE.md`'s "no interrupts"; restricts the generated checks per 0010 and mechanises it per 0014; defers co-simulation per 0032 |
| [0083](0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md) | The forwarding network is priced, and declined on the margin | Accepted · prices 0004's stall-only interlock; replaces 0042's 44/52 as the evidence `CLAUDE.md` cites; graded against 0066 by 0076's ceiling method; its 29.3% qualified by 0084 |
| [0084](0084-dhrystone-is-the-comparable-number-and-the-raw-share-does-not-transfer.md) | Dhrystone is the number this core can be quoted by, and the suite's RAW share does not transfer | Accepted · runs on 0081's `.data` image; re-reads 0070's accounting on compiled code and qualifies 0083's 29.3%; enlarges 0074's prize without reopening it |
| [0085](0085-the-memory-map-has-one-source-and-every-restatement-is-checked.md) | The memory map has one source, and every restatement is checked | Accepted · closes the defect class 0084 and 0081 each found once; makes 0008's map a shared default rather than two copies; leaves 0054's ROM ceiling and 0044's boot path where they are |
| [0086](0086-both-cores-in-one-harness-and-the-gap-is-the-fetch-loop.md) | Both cores in one harness, and the gap is the fetch loop | Accepted · pays the comparison 0080 said was owed and corrects two of its statements about the artefact; points at the fetch loop 0076 and 0078 already measured, reopening neither |
| [0087](0087-the-instruction-memory-does-not-come-out-of-the-fetch-loop.md) | The instruction memory half comes out of the fetch loop, and depth does not pay for it | Accepted · answers the two questions 0086 left and declines both; ranks below 0076's ceiling and 0083's forwarding; rules the discarded fetch buffer forbidden as written under 0001's commitment |
| [0088](0088-the-win-is-in-what-the-expression-cannot-say.md) | yosys already does everything derivable from the expression; the win is in what the expression cannot say | Accepted · an area result, not a speed one; retires the congestion hypothesis 0086 flagged as unresolvable on this part, and leaves 0087's ranking of the Fmax levers standing |
| [0089](0089-the-operand-fetch-guess-ships-and-one-workload-feels-it.md) | The operand-fetch guess ships, and only one workload feels it | Accepted · reverses 0074's decline of its cheaper variant on a re-measured margin; restates 0064's correctness argument without changing the bypass; leaves 0066's requirement and its grading exactly as they are; 0084's warning about the suite's share is the finding |
| [0090](0090-the-executor-carries-half-the-registers-it-had.md) | The executor carries half the registers it had, one negator and an unsigned multiplier | Accepted · applies 0088's rule to three edits that each clear the churn band alone; widens the divider's recorded magnitude restriction rather than dropping it, and corrects a FORMAL assumption that contradicted `CLAUDE.md`'s own stall ruling |
| [0091](0091-the-stall-costs-the-same-at-a-memory-pin.md) | The stall costs the same at a memory pin, and the arm it would replace is worth nothing | Accepted · declines the ROM read enable as a null in both directions, and retires the −3.8% ceiling it was filed on: re-taken over eight seeds on each of two bases, that term buys nothing. Prices a lever 0087 left unranked |
| [0092](0092-the-writeback-slot-costs-more-than-the-bypass-it-replaces.md) | The writeback slot costs more than the bypass it replaces, and the two loops do not pay together | Accepted · declines the fourth scoreboard slot at 19.5% of suite cycles for period inside the churn band; measures the pairing claim 0091 shares and finds the pair no better than either half; the evidence for the second forwarding point 0064 and 0089 describe |
| [0093](0093-the-compressed-successor-is-decoded-and-the-compiled-workload-is-what-moves.md) | The compressed successor is decoded, and the compiled workload is what moves | Accepted · reverses 0074's decline of its first variant on a ten-placement re-measurement: 16.3% of Dhrystone's cycles, 0.640 DMIPS/MHz, and 12.0 MHz cleared at ten placements of ten with the median a null. Collects the win 0089 named and could not take; puts the register-number mapping in one module read twice |
| [0094](0094-the-compressed-decode-is-already-mapped-and-the-block-is-closed.md) | The compressed decode and the immediate generator are already mapped, and the block is closed | Accepted · 0088's question asked of the last block that had never had it, and answered with a null. Ceilings: the immediate mux is 101 LUTs whole, the compressed decode 253, and deleting the fetch window's upper-half mask *costs* 13. The one group built reads −50 LUTs on `fit` and −1 on the SoC — the ±50 churn band caught at synthesis, where neither count has a placement in it · amended by 0097, which measures the four it declined without building |
| [0095](0095-co-simulation-is-required-and-its-fetch-is-verified.md) | Co-simulation is a required check, and the precondition for that was met two changes ago | Accepted · amends 0032's opt-in decision and closes 0033's decision 4: the fetch is digest-verified before extraction, so the leg that reads the real register file gates on every PR. The tarball is cached, not the unpacked tree. Both failure paths demonstrated in CI |
| [0096](0096-the-csr-file-the-soc-glue-and-the-register-file-are-already-mapped.md) | The CSR file, the SoC glue and the register file are already mapped, and all three blocks are closed | Accepted · 0088's question asked of the last three blocks, answered with a null and six ceilings. `csrs.v` is 727 LUTs whole and 340 of that is the mandated counters; its WARL mux and masks are worth **1**; the regfile's bypass is 31 and all its fabric 133; the SoC's read-back combine is at its floor and an XOR in its place *costs* 19. The one edit that looked free — a counter tick on the adder's carry-in — misses 12 MHz at six placements of six, because yosys had already spent that mux on 128 clock enables |
| [0097](0097-the-decode-stack-pays-only-in-the-fetch-loop-and-that-is-where-it-cannot.md) | The decode stack pays only in the fetch loop, and that is where it cannot | Accepted · amends 0094. Seven candidates stacked read −117 SoC LUTs, and 103 of them are the shared `next_pc` adder 0094 declined without building — which misses 12 MHz at six placements of six. The other six are −28 and a second null, so 0094's block stays closed on two measurements. New decode vectors, each demonstrated red |
| [0098](0098-dhrystone-on-both-cores-and-their-published-rate-reproduces.md) | Dhrystone runs on both cores in one harness, and their published rate reproduces | Accepted · amends 0086's table with its missing factor. 0.679 DMIPS/MHz here against 0.557 there, so this core needs 18.0% fewer cycles and the throughput gap is 1.21× (22.10 against 26.84 DMIPS). Their published 0.52 comes out 7.1% low, where the same project's 92 MHz missed by 1.9×. Dhrystone needs 26 of an hx8k's 32 block RAMs before either core's own, so the cycles are simulated at a larger map than the clock was placed at, and every distortion is listed |
| [0099](0099-the-memory-transaction-launches-from-the-execute-slot.md) | The memory transaction launches from the execute slot, and the load turnaround stops existing | Accepted · amends design commitments 8 and 5 and re-derives 0046's F and G (G 6 → 5). The bus is driven from `decoder_out` during the executor's cycle, so a load costs what an add costs: −11.8% of Dhrystone's cycles, 0.640 → 0.727 DMIPS/MHz, −106 `fit` cells, and the worst of six placements moves from 0.83% over 12 MHz to 5.9% — re-measured on the tree that merges, −74 cells in the `fit` job and 4.8%. A held launch is gated by a new `components_accessor` proof and a transaction count, both demonstrated red |
| [0100](0100-the-early-register-write-is-built-and-costs-the-clock.md) | The early register write is built, buys 6.5% of Dhrystone, and costs the clock | Accepted (declined) · third entry beside 0083 and 0092, all three declined on the clock. A committed result taking the idle write port a cycle early is −9.1% of suite cycles and −6.5% of Dhrystone's, and +9.4% of median period — under 12 MHz at four placements of six. The ceiling survives; the RTL does not |
| [0101](0101-the-successor-pair-is-learned-and-deferred-for-margin.md) | The successor pair is learned from history, holds 12 MHz, and is deferred for margin | Accepted (deferred) · extends 0089 and 0093 to the case a fetch window cannot reach. 256 tagged entries in one EBR: −8.0% of Dhrystone's cycles against **+0.5% of the suite's**, +77 cells, one more EBR, and +5.0% of median period — which leaves 3.0% over the worst of six placements, inside the churn band, where the queued fetch-loop work needs 5.9%. Not a clock failure and not a correctness one: a margin trade, re-take it on the tree of the day |
| [0102](0102-sails-reservation-survives-a-trap-and-the-misa-bit-is-the-divergence.md) | Sail's reservation survives a trap, and `misa` — not the reservation — is what would have gone red | Accepted · amends 0043's configuration with the atomics knobs. Measured under Sail alone, no core: the model clears the reservation on neither trap entry nor `mret`, which is this core's settled policy exactly, so there is nothing to mirror and no baseline entry. `A` is the `misa.A` bit and NOT an umbrella — `Zaamo`/`Zalrsc` execute the instructions on their own, and `A: true` diverges `csr.S` at `0x40001105` against the core's `0x40001104`, so it flips with `rtl/csrs.v` and not before. Probe committed as `make sail-reservation-probe`, with its red direction demonstrated |
| [0103](0103-the-performance-monitor-reads-zero-and-the-model-was-the-wrong-instrument.md) | The performance monitor reads zero, and the reference model was the wrong instrument | Accepted · corrects finding 3 of 0048, amends 0043's extension audit. The 87 `mhpmcounter`/`mhpmevent` addresses all raised illegal instruction where the spec asks for the counters and permits read-only zero; the gap survived because it was closed against Sail's behaviour rather than the spec text, and Sail trapped only because this repo's own config said the counters do not exist. No new state: the read mux's default arm already answered zero, so the change is address decode on `implemented`. Co-sim is closed by configuring the model (`Zihpm: true`), not by a baseline entry |
| [0104](0104-the-fetch-bus-refuses-and-the-data-bus-cannot-afford-to.md) | The fetch bus refuses, and the data bus cannot afford to | Accepted · replaces 0067 decision 1 IN PART and re-reads its decisions 2-4. The privileged spec retired `vacant` as a region category and strongly recommends a precise access fault; this SoC did none of the three things it allows. `rtl/imemory.v` already computed the range test and now exports it, so an out-of-region fetch is cause 1 instead of the cause 2 a word of zeroes decoded to — committed in decode with the word it arrives with, which is why commitment 2 survives. The load/store half is built, measured and declined: worst of four seeds 10.57 MHz against 12.0, 23 to 27 logic levels in the fetch loop, four extra LUT levels charged to decode. Five `#omit` rulings re-taken by name, `fault_ch0` generated at a depth derived from F and G, and Sail's one executable region split — it had been reproducing the core's own wrong cause |
| [0105](0105-the-fencei-wait-is-unobservable-in-the-suite.md) | The `fence.i` wait is unobservable in the suite, and the memory's port arbitration is why | Accepted · records what 0099 cost the behavioural leg and narrows what a green `.S` run says about commitment 5, without amending it. Deleting `instr_fencei` from `serialize` fails `selfmod.S` on `b20c33c` and passes the whole suite on `3d183fe`: the write edge moved a cycle earlier, and the one slot a `fence.i` could still have been covering is one the write's own fetch-port steal already holds back. `test/decoder_tb.v` is the grader for that term; `selfmod.S` is red only when both mechanisms are deleted, and its note now says so |
| [0106](0106-the-a-extension-is-built-and-the-board-still-closes.md) | The A extension is built, and the board still closes | Accepted · builds the single-hart A brief and amends commitment 8's reason list with a sixth. All eleven `.w` encodings decode; one 33-bit adder/subtractor serves the add and all four min/max compares; the reservation is refused outside the region `rtl/memory.v` answers, which is where 0104's declined decode-time test is paid for instead. The atomic write cycle **bubbles** where the divider holds, because the executor has already taken the instruction — a hold is two transactions and two retires. G re-measured 5 → 6, so the thinnest depths clear `F + 2G` by one again. `fit` 3464 → 3935 and `FIT_MAX_LC` 3625 → 4000; the go/no-go sweep clears 12 MHz at every seed, and two spellings of one design sit 3.3% apart at their worst placements |
| [0107](0107-the-depth-derivation-is-machine-checked-and-f-and-g-are-a-command.md) | The depth derivation is machine-checked, and F and G are a command | Accepted · mechanises 0046's rule. F and G are declared to `formal/genchecks-audit.py` in `#`-prefixed lines genchecks' parser drops, and every check's depth is graded against its family's floor — `F+2G` for a one-retire check, `start+G`/`trig+G` for a two-retire one — read back off the `.sby` just generated. A stale G fails generation and names the entries that need more depth, demonstrated. `make -C formal remeasure-fg` sweeps `hang` and `liveness` and reproduces F = 6, G = 6 with both flip points red-then-green. 0106's AMO exclusion and its cover goal re-run rather than rebuilt; no depth, no RTL and no generated byte moves |
| [0108](0108-the-a-extension-is-claimed-and-one-bit-is-the-whole-claim.md) | The A extension is claimed, and one bit is the whole claim | Accepted · closes 0106's decision 5 and amends 0002's ISA target to RV32IMAC_Zicsr_Zifencei. `misa` goes `0x4000_1104` → `0x4000_1105` and Sail's `A` key flips with it, the transition demonstrated in both directions rather than the end state confirmed: `csr.S` diverges at architectural change 17, then agrees. A `csrw misa, x0` case and a `misa-drops-the-a-bit` mutation pair the claim with the one asm case and two bench reads that are its entire coverage. Records this platform's PMAs in the spec's vocabulary, the reservation policy's survival across traps and `mret`, and the `.aq`/`.rl` argument with its four invalidators. Says plainly that A is claimed while causes 5 and 7 are unimplemented. `fit` +53 cells, shown to be re-mapping churn by two one-bit probes spanning 68; eight seeds a side move the median +2.1%, a null, with 16 placements of 16 over 12 MHz. No ratchet moves |
| [0109](0109-an-atomic-outside-the-ram-faults-and-the-declined-measurement-did-not-transfer.md) | An atomic outside the RAM faults, and the declined measurement did not transfer | Accepted · closes the half of 0104's decision 2 that 0106's decision 3 left open, and closes 0108's decision 4 — the caveat `misa.A` was claimed under. `rtl/memory.v` answers its range test about a **second** address, the one an atomic's rs1 names, and decode raises cause 5 for `lr.w` and 7 for the ten that write, alignment outranking the region; the machine timer gets the `AMONone` 0108's PMA table marked wrong. `soc/depth/path_stages.py`: **zero extra logic levels against the general case's four**, the fetch loop a level shallower at both placements taken, and what moved instead is routing at 89% occupancy. Two spellings of one design swept 11.82 and 12.24 MHz at their worst of eight seeds and only the one-term one meets the requirement; re-swept against merged main on the pinned suite it is **12.39 against the base's 12.12**, so **two toolchains disagree about the sign** and only the verdict travels. `amoregion.S`'s co-simulation entry comes out — `RsrvNone` and `AMONone` are what this platform is now — and `fault_ch0` becomes the oracle for both causes, having been one for neither. `fit` job 3934 → 3966, `FIT_MAX_LC` untouched at 4000 with 34 cells of headroom, which is inside the churn band |

0001–0007 came from the design brief
([`docs/ideas/finish-the-rewrite.md`](../ideas/finish-the-rewrite.md)). 0008–0011 came out of
sprint planning, when the panel ran the toolchain and found the brief's assumptions needed
resolving before work could start. 0012–0014 came out of integrating the first build sprint, when
review of the build-repair and datapath-fix changes turned up decisions neither had recorded, and the test
suite that fails in its entirety. 0010 supplements 0006; 0011 scopes 0005; 0012 supplements 0010;
0013 implements 0006's pinning clause; 0014 supplements 0007 and 0008. 0015-0017 came out of
integrating the second build sprint: 0015 extends 0009 with a stall source 0009 did not know
existed, 0016 constrains the CI gate, and 0017 records what the newly-passing decoder
component proof does and does not establish. 0018-0020 came out of integrating the third build
sprint: 0018 supersedes 0016 now that required checks exist, 0019 makes the monitor sanitizer the
place a generated-oracle defect gets repaired, and 0020 records that ADR-0006's non-perturbation
guarantee is argued rather than proven. 0021-0023 came out of integrating the riscv-formal ladder
port — the first time any formal check ran against the pipelined core. 0021 keeps the compressed
checks in the ladder and records the C.JR/C.JALR decode defect they found; 0022 replaces the
nightly's blanket failure-swallowing with an ADR-0014-style baseline; 0023 states what the run
does and does not establish, and why M2 is not reached. 0024 closes one of 0023's three named
holes by switching the ladder's default BMC engine. 0031 re-vendors the `genchecks` copy from the
pin — retiring the local mechanism 0024 built to reach that engine, while leaving 0024's
measurement intact — and records why none of the ten `fault`/`bus_*` checks it unlocks apply here.
0032 came out of a time-boxed spike against the Sail RISC-V model and resolves the "Spike or Sail
co-simulation" item that used to sit in the deferred list below. 0033 came out of integrating those
three together: it audits the machinery M2 is *measured* by rather than the core it measures, and
records what a green ladder and a matching baseline do not, on their own, establish. 0034 came out of
integrating the CSR file: it records the two decode-side decisions ADR-0005's field list left open,
corrects ADR-0027 on the counter `h` halves, measures what the `csrw_*` checks cannot see, and fixes
a `CLAUDE.md` engineering rule that named the wrong file. 0035 amends 0014 and applies 0033's lens
to the *simulation* gate: `test/run_tests.sh` had three ways to report success without having tested
anything, so the failure set now records name **and** status and every build step is checked. 0036 was
recorded integrating those three gate changes together: it ratifies putting `hang` on the ladder
(`liveness` does not subsume it — it *assumes* a retire at its trigger cycle and is vacuous on a core
that never retires), corrects 0031 on how the `csrc_*` family is actually reached, and closes the
`case_default` question by measuring that yosys already errors on the latch. It also records a
gap it did not close: `formal/EXPECTED_FAIL` still matches on names only, so a red check that flips
from `FAIL` to `ERROR` keeps the ladder green — 0035's lesson, unapplied to the formal side. 0039
finishes the first half of 0032's integration list: the whole `.S` suite now runs under
co-simulation behind `make cosim-suite`, graded by 0014's set equality in 0035's name-and-status
format, and `tohost` becomes the doubleword the HTIF protocol it borrows always specified — an
amendment to 0008, and the one change here that touches what both existing sim legs read. 0040 is
0038's question asked of the verification harness rather than of the area: it measures whether the
riscv-formal ladder can model the negedge regfile 0038 recommends, finds that it refuses to rather
than mis-modelling it, rejects `clk2fflogic` as the remedy on measured vacuity, and — applying 0033's
lens once more — fixes a `make -C formal check` that could not re-run the ladder and had been
re-grading the previous run's verdict. 0041 came out of integrating those three together: it
measures what 0039's `tohost` change actually did to the architectural register trace (382 values,
zero events), and records the co-simulation nightly 0039 deliberately omitted as owed work with its
preconditions, so it stops depending on anyone's memory. 0043 takes 0039's two baselined
divergences to zero without touching the core: the reference model was a `--config-override` on
Sail's *default* RV32 machine, so it had atomics, bit-manipulation, float, supervisor mode, user
mode and vectors that nobody chose, and `csrr misa` was the one register a test happened to read.
It is now a complete `--config` describing 0002's RV32IMC_Zicsr, and the leftover — an
implementation-defined value with no knob — is exempted by name, one register at a time, or moved
to a bench with no reference model in it when the program *branches* on it.
0046 is 0025 asked again of a pipeline it no longer described. 0025's own addendum said the sweep
should be repeated once the CSR RTL existed and it never was, and `e4f5250` then added a fifth stall
reason without touching the depth table — so the one M2 term marked met rested on a derivation two
changes stale. It re-derives the numbers and changes how: the whole argument now rests on two
quantities the ladder measures about itself in seconds (`hang` for the worst-case first retire,
`liveness` for the worst-case retire gap) rather than on hand-tracing, which is the part that went
stale. No depth moves. It also gives the 70 `insn_*` checks the liveness probe they had never had,
and that probe exhibits the failure mode the whole file is about: swept below its floor, a check
passes on a core that is broken.

## Deferred decisions

These are deliberately *not* decided yet. Each requires a new ADR before being built, because each
trades away simplicity the current design depends on.

- ~~**Forwarding network**~~ — **priced by
  [ADR-0083](0083-the-forwarding-network-is-priced-and-declined-on-the-margin.md), and decided
  AGAINST.** The old entry called it a CPI-only optimisation, safe to add once the core was verified.
  It is verified now, and the cost is not CPI: both spellings were built, both run the suite and pass
  every proof, and both are declined on the period they cost against a 12 MHz requirement that does
  not slide. The ceiling is what settles it — deleting the scoreboard outright buys 0% of period, so
  nothing in this direction pays for the muxes.
- ~~**Radix-4 divider / early termination**~~ — **rejected outright by
  [ADR-0038](0038-area-is-measured-in-logic-cells-and-two-levers-are-rejected.md).** CPI-only and it
  *increases* area, which is the wrong direction on a part that does not currently place. The
  "55–70% utilisation" figure this entry used to cite was a yosys cell count and was wrong; the
  measured figure is 126%.
- ~~**negedge-BRAM regfile**~~ — **resolved by [ADR-0042](0042-the-regfile-read-is-synchronous-and-costs-a-cycle.md),
  and decided AGAINST.** The regfile did move to block RAM — 6971/132% to 4236/80%, which is the
  whole area problem in one lever — but with a **posedge** read plus a one-cycle operand-fetch stall,
  not a negedge strobe. All three reconciliations of invariant 6 with a synchronous EBR were built
  and measured, and area does not choose between them (86 cells across the three, 1.6% of the part).
  The negedge variant is 99 cells cheaper and costs **no** cycles; it was rejected because sby's
  `prep` fails closed on mixed clock polarity, so the generated ladder cannot run against it at all —
  every check, not one — and shipping it would mean a posedge substitution in `formal/wrapper.v` plus
  an equivalence proof whose k-induction does not close. Serialising the two read ports onto one
  array was built too: 44/52, three red checks, and it needs a second bypass level. See
  [ADR-0040](0040-the-ladder-refuses-a-negedge-regfile-and-make-check-was-re-grading.md) for the
  verification-side data the decision was made with.
- ~~**FPGA timing closure / nextpnr flow**~~ — **built by
  [ADR-0054](0054-the-memory-system-and-the-first-real-timing-number.md)**, ahead of its post-M4
  slot and with every M2 term re-run rather than assumed. The SoC places on up5k/sg48 and
  `make soc-timing` reports `icetime`'s critical path with its logic/routing split. ADR-0003's
  second ROM read did become interleaved banks — at **word** granularity, not the halfword split
  ADR-0044 named, because this core's fetch interface asks for two adjacent words and windows them
  itself. The bootloader is half-built: the `.data` copy stub landed with
  [ADR-0081](0081-the-data-image-lives-in-rom-and-crt0-copies-it.md), so the SoC runs a C program
  that reads its own globals. ADR-0044's SPI-flash path stays deferred, now against the measured
  ROM budget rather than a guess.
- ~~**Interrupts**~~ — **the machine timer is built by
  [ADR-0082](0082-the-machine-timer-interrupt-is-taken-at-a-decode-boundary.md).** The old entry's
  reason ("no interrupt sources exist") was circular: `rtl/timer.v` is the source, and it is four
  words on the data bus. `mie.MTIE` and `mip.MTIP` are real; `mip.MSIP`/`mip.MEIP` stay read-only
  zero because this platform has neither source. What is still deferred is a controller, more
  sources and a vectored `mtvec` — the same mechanism with more inputs.
- ~~**Spike or Sail co-simulation**~~ — **resolved by [ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md).**
  The old test ("revisit only if formal and simulation ever disagree") could only fire on a bug both
  legs can see; a spike measured what neither can.
  [ADR-0039](0039-co-simulation-runs-the-whole-suite-against-a-baseline.md) integrated it across the
  whole suite against a baseline, and
  [ADR-0095](0095-co-simulation-is-required-and-its-fetch-is-verified.md) makes it a required check
  on `main` once the digest-verified fetch ADR-0032 asked for existed. Still `make cosim-run` /
  `make cosim-suite` and never `make test`. The memory comparison remains future work; the nightly
  job does not, having been overtaken by the gate.
