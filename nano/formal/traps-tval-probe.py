#!/usr/bin/env python3
"""Forces nano/formal/traps.sby's mtval arm to fail, and requires it to fail as that
arm rather than as anything else.

Usage: traps-tval-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. mtval is the one thing a trap saves that no self-reporting oracle
in this tree looks at: riscv-formal ships no spec model for SYSTEM at the pin, so
the generated checks never read it, and the two sim legs see only what a program
chose to load it into. traps.sv's arm is therefore the only statement that a trap
reports the right thing about the right access -- and an arm nobody has watched
fail is worth nothing, which is what `make probe-gates` demands of every other
graded comparison here.

Two cores are built, each one line of nano/nano.v from the shipping one:

  wrong-addr   the load-region-fault arm reports rs1 where it must report the
               effective address. The two differ by the instruction's immediate
               and by nothing else, which is exactly the defect a suite whose only
               out-of-window load carried a zero offset could not see.
  wrong-value  the load-misaligned arm reports zero where it must report the
               address that was actually misaligned.

Both mutations must go FAIL at the mtval comparison. The shipping core is built
once more as the required control, the same reason traps-region-probe.py's own
control exists: a probe that never shows the shipping core passing proves nothing
about a mutant failing for the right reason.

NOT HERMETIC -- it runs sby three times, at nano/formal/traps.sby's own depth.
So it is a prerequisite of `make -C nano/formal components_traps` rather than of
`make test`.
"""

import probe_common

MUTATIONS = {
    "wrong-addr": (
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = load_store_address;
""",
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
      trap_tval_value  = `RF_RS1;
""",
    ),
    "wrong-value": (
        """    end else if (load_misaligned) begin
      trap_cause_value = CAUSE_LOAD_MISALIGNED;
      trap_tval_value  = load_store_address;
""",
        """    end else if (load_misaligned) begin
      trap_cause_value = CAUSE_LOAD_MISALIGNED;
      trap_tval_value  = 32'b0;
""",
    ),
}

if __name__ == "__main__":
    probe_common.main(
        __doc__,
        MUTATIONS,
        "tval-probe",
        "the mtval arm",
        "Both mtval mutants fail, and the shipping core passes.",
    )
