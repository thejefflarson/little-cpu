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

0001–0007 came from the design brief
([`docs/ideas/finish-the-rewrite.md`](../ideas/finish-the-rewrite.md)). 0008–0011 came out of
sprint planning, when the panel ran the toolchain and found the brief's assumptions needed
resolving before work could start. 0012–0014 came out of integrating the first build sprint, when
review of the build-repair and datapath-fix changes turned up decisions neither had recorded, and the test
suite that fails in its entirety. 0010 supplements 0006; 0011 scopes 0005; 0012 supplements 0010;
0013 implements 0006's pinning clause; 0014 supplements 0007 and 0008. 0015-0017 came out of
integrating the second build sprint: 0015 extends 0009 with a stall source 0009 did not know
existed, 0016 constrains the CI gate, and 0017 records what the newly-passing decoder
component proof does and does not establish.

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
