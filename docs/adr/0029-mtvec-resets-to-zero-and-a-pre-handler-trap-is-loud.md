# ADR-0029: `mtvec` resets to zero, and a pre-handler trap is made loud

**Status:** Accepted · 2026-07-31 · *Supplements ADR-0005*

## Context

ADR-0005 specifies `mtvec` as a read/write CSR in direct mode with a WARL 4-byte-aligned base. It
says nothing about its **reset value**, and the RISC-V spec leaves that implementation-defined.

That silence is a trap of its own here. `test/asm/sections.lds` places `.text` at `0x0`. So with
`mtvec` resetting to 0, a trap taken *before* a test installs its handler redirects to address 0 —
which is `_start`. The program silently restarts.

That failure mode is the worst kind: it looks like a livelock or a timeout, it produces no error,
and the actual cause (a fault several instructions earlier) is nowhere in the symptom.

## Decision

**Reset `mtvec` to 0** — the spec permits it, and any nonzero choice is an arbitrary constant a
reader would have to look up.

**And make the resulting condition loud rather than silent.** `test/testbench.v` and
`test/cxxrtl.cc` treat trap entry with `mtvec == 0` as a hard test failure, using the redefined
`trap` pulse from ADR-0028.

The two halves are one decision: 0 is the readable reset value *provided* the harness refuses to
let a jump to it pass as normal execution.

## Rationale

The alternative — resetting `mtvec` to some sentinel like `0xDEAD_0000` — makes the RTL carry a
magic number to compensate for a harness limitation, and still produces a confusing symptom (a
fetch from unmapped ROM) rather than a clear one.

Detecting it in the harness costs a few lines, produces a named failure, and keeps the RTL honest.
It also generalises: any future test that faults before installing a handler gets the same clear
message, without anyone remembering this ADR exists.

## Consequences

- `rtl/csrs.v` resets `mtvec` to 0 with a comment pointing here.
- Both sim legs gain a check that fires on trap-to-zero. This is a *harness* assertion, not an RTL
  one — a real program is entitled to place a handler at 0 if it wants to, and the check would need
  revisiting if `sections.lds` ever moved `.text`.
- Trap tests must install `mtvec` before faulting, which is the correct discipline anyway.
