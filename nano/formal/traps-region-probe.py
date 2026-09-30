#!/usr/bin/env python3
"""Forces nano/formal/traps.sby's load/store region arms to fail, and requires the
shipping core to pass first.

Usage: traps-region-probe.py [--repo DIR] [--workdir DIR] [--sby SBY]

WHY THIS EXISTS. traps.sv states two things about a plain load or store whose
effective address lands outside the RAM window: that nano.v must trap, and that the
cause must be 5 for a load and 7 for a store. Both are arms of a proof that passes,
and an arm in that position is worth nothing until it has been shown to fail --
which is what `make probe-gates` demands of every other graded comparison in this
tree and what this file does for the two that need a solver.

Two cores are built, each a few lines of nano/nano.v away from the shipping one:

  no-trap      ls_in_range is forced true, so an aligned load or store outside the
               RAM window never faults. traps.sv's own independent oracle still
               expects one (it recomputes the window from RAM_BASE/RAM_WORDS, never
               from nano.v's own signals), so `assert(rvfi_trap)` under
               `expected_trap` must go FAIL.
  wrong-cause  swaps the two causes -- 7 for a load and 5 for a store -- and the
               proof must go FAIL at the mcause comparison. A core that faults the
               right access with the wrong cause is what that arm exists to catch.

The unmutated core is not built to prove anything new here: it is what
`components_traps` proves, and this file is a prerequisite of that target. It is
still built once, as the required control -- a probe that never shows the shipping
core passing proves nothing about a mutant failing for the right reason.

NOT HERMETIC -- it runs sby three times, at nano/formal/traps.sby's own depth.
So it is a prerequisite of `make -C nano/formal components_traps` rather than of
`make test`.
"""

import probe_common

MUTATIONS = {
    "no-trap": (
        """  assign ls_in_range = load_store_address >= RAM_BASE &&
    load_store_address < RAM_BASE + RAM_WORDS * 4;
""",
        """  assign ls_in_range = 1'b1;
""",
    ),
    "wrong-cause": (
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
    end else if (store_region_fault) begin
      trap_cause_value = CAUSE_STORE_ACCESS_FAULT;
    end else begin
""",
        """    end else if (load_region_fault) begin
      trap_cause_value = CAUSE_STORE_ACCESS_FAULT;
    end else if (store_region_fault) begin
      trap_cause_value = CAUSE_LOAD_ACCESS_FAULT;
    end else begin
""",
    ),
}

if __name__ == "__main__":
    probe_common.main(
        __doc__,
        MUTATIONS,
        "region-probe",
        "this arm",
        "Both load/store region arms fail for their own reason, and the shipping core passes.",
    )
