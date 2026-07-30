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
| [0022](0022-the-formal-nightly-reports-against-an-explicit-baseline.md) | The formal nightly reports against an explicit baseline, not `\|\| true` | Accepted |
| [0023](0023-the-first-ladder-run-does-not-reach-m2.md) | The first ladder run does not reach M2 — three named holes | Accepted |
| [0024](0024-the-ladders-default-bmc-engine-is-btormc.md) | The ladder's default BMC engine is `btor btormc`, not `smtbmc yices` | Accepted |
| [0025](0025-formal-ladder-depths-are-derived-not-inherited.md) | The ladder's depths are derived from the pipeline, not inherited | Accepted |

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
holes by switching the ladder's default BMC engine.

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
- **Spike or Sail co-simulation** — riscv-formal is the oracle. Revisit only if formal and
  simulation ever disagree.
