# Architecture Decision Records

Load-bearing decisions for Little CPU. Each ADR records the context, the decision, why it was made,
and what it costs. Reversing one is fine — write a new ADR that supersedes it.

| # | Decision | Status |
|---|---|---|
| [0001](0001-finish-the-staged-rewrite.md) | Finish the staged rewrite rather than resurrect the serialized core | Accepted |
| [0002](0002-isa-target-rv32imc-zicsr.md) | ISA target is RV32IMC_Zicsr, machine mode only | Accepted |
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
| [0020](0020-the-rvfi-non-perturbation-guarantee-is-argued-not-proven.md) | The RVFI non-perturbation guarantee is argued, not proven | Accepted |
| [0021](0021-the-formal-ladder-runs-the-compressed-checks.md) | The formal ladder runs the compressed checks, and it found a real bug | Accepted |
| [0022](0022-the-formal-nightly-reports-against-an-explicit-baseline.md) | The formal nightly reports against an explicit baseline, not `\|\| true` | Accepted · corrected by 0037 |
| [0023](0023-the-first-ladder-run-does-not-reach-m2.md) | The first ladder run does not reach M2 — three named holes | Accepted |
| [0024](0024-the-ladders-default-bmc-engine-is-btormc.md) | The ladder's default BMC engine is `btor btormc`, not `smtbmc yices` | Accepted |
| [0025](0025-formal-ladder-depths-are-derived-not-inherited.md) | The ladder's depths are derived from the pipeline, not inherited | Accepted |
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
| [0037](0037-an-empty-baseline-is-not-m2.md) | An empty formal baseline is not M2, and the milestone criterion said it was | Accepted |

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
from `FAIL` to `ERROR` keeps the ladder green — 0035's lesson, unapplied to the formal side.

## Deferred decisions

These are deliberately *not* decided yet. Each requires a new ADR before being built, because each
trades away simplicity the current design depends on.

- **Forwarding network** — CPI-only optimisation on top of ADR-0004's stall-only interlock. Safe to
  add post-verification; unsafe to add while the core is unverified.
- **Radix-4 divider / early termination** — CPI-only. Roughly doubles comparator and mux logic on a
  part already at 55–70% utilisation, and the payoff is largely absorbed by ADR-0007 (cxxrtl) and by
  the 65→32 iteration-count fix.
- **negedge-BRAM regfile** — escape hatch if ADR-0004's flip-flop regfile blocks timing closure.
- **FPGA timing closure / nextpnr flow** — including ADR-0003's second ROM read becoming interleaved
  16-bit banks. Post-M4.
- **Interrupts** (`mie`/`mip` real rather than read-only zero) — no interrupt sources exist.
- ~~**Spike or Sail co-simulation**~~ — **resolved by [ADR-0032](0032-sail-co-simulation-is-worth-building-and-stays-opt-in.md).**
  The old test ("revisit only if formal and simulation ever disagree") could only fire on a bug both
  legs can see; a spike measured what neither can. The harness exists and is deliberately opt-in —
  `make cosim-run`, never `make test`, never CI. Integrating it across the whole suite is
  still future work, scoped in that ADR's consequences.
