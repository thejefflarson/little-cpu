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
| [0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md) | Sail co-simulation is worth building, and stays opt-in | Accepted |
| [0033](0033-what-the-green-ladder-does-not-cover.md) | What a green ladder does not cover — three assurance gaps, named | Accepted |
| [0034](0034-what-the-csr-ladder-checks-cannot-see.md) | What the CSR ladder checks cannot see, and the decisions the CSR file forced | Accepted · corrected by 0037 |
| [0035](0035-the-baseline-pins-the-failure-mode.md) | `test/EXPECTED_FAIL` pins the failure mode, not just the file name | Accepted |
| [0036](0036-three-gate-hardening-decisions-ratified-at-integration.md) | Three gate-hardening decisions ratified at integration, and a correction to ADR-0031 | Accepted |
| [0037](0037-an-empty-baseline-is-not-m2.md) | An empty formal baseline is not M2, and the milestone criterion said it was | Accepted · amended by 0045, 0047 |
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
| [0058](0058-the-fetch-loop-is-not-two-levels-too-deep.md) | The fetch loop is not two levels too deep | Accepted · a measured null; corrects how 0054's logic-level count reads |
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

- **Forwarding network** — CPI-only optimisation on top of ADR-0004's stall-only interlock. Safe to
  add post-verification; unsafe to add while the core is unverified.
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
  itself. What is still deferred is a bootloader: SPRAM cannot be initialised, so a real product
  needs a `.data` copy stub or ADR-0044's SPI-flash path.
- **Interrupts** (`mie`/`mip` real rather than read-only zero) — no interrupt sources exist.
- ~~**Spike or Sail co-simulation**~~ — **resolved by [ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md).**
  The old test ("revisit only if formal and simulation ever disagree") could only fire on a bug both
  legs can see; a spike measured what neither can. The harness exists and is deliberately opt-in —
  `make cosim-run` / `make cosim-suite`, never `make test`, never CI's required set.
  [ADR-0039](0039-co-simulation-runs-the-whole-suite-against-a-baseline.md) integrated it across the
  whole suite against a baseline; a nightly job and memory comparison remain future work.
