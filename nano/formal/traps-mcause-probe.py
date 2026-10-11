#!/usr/bin/env python3
"""Forces nano/formal/traps.sby's interrupt-cause arm to fail, and requires the
shipping core to pass first.

Usage: traps-mcause-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. traps.sv asserts that the first retirement after an interrupt entry
sees mcause equal to one of the two interrupt causes this core takes, the timer's
0x8000_0007 or the external 0x8000_000B. That is an arm of a proof that passes, and
an arm in that position is worth nothing until it has been shown to fail -- and, for
an assertion about an interrupt a free input has to raise through two CSR writes, to
be reachable inside the proof's depth at all, which only a failing mutant can show.

One core is built, a line of nano/nano.v away from the shipping one: the timer's
cause is 0x8000_0003, a code this core never takes. The proof must go FAIL. The
shipping core is built once more as the required control, the same reason
traps-region-probe.py's own control exists.

NOT HERMETIC -- it runs sby twice, at nano/formal/traps.sby's own depth. So it is a
prerequisite of `make -C nano/formal components_traps` rather than of `make test`.
"""

import probe_common

MUTATIONS = {
    "timer-cause": (
        "CAUSE_MACHINE_TIMER       = 32'h8000_0007;",
        "CAUSE_MACHINE_TIMER       = 32'h8000_0003;",
    ),
}

if __name__ == "__main__":
    probe_common.main(
        __doc__,
        MUTATIONS,
        "mcause-probe",
        "the interrupt-cause arm",
        "The interrupt-cause mutant fails, and the shipping core passes.",
    )
