# nano/bench/QSPI_CONTROL

The zero-wait control for `make nano-qspi-timing`, and the only place its cycle counts are
stated. Each non-comment line is `<benchmark> <runs> <cycles>`: Dhrystone at 200 runs and CoreMark
at 5 iterations, the counts `nano/bench/run_qspi_timing.sh` sweeps at, measured on
`nano/tb/nano_memory.v` (zero-wait, flat) by `make nano-dhrystone` and `make nano-coremark`.

**What the control is for.** Every row of the QSPI sweep is a ratio against a core that waits
for nothing, so a sweep taken after nano's instruction timing or the compiler moved would
publish rows against a baseline the table never saw. `nano/bench/qspi_control_check.sh` grades
the fresh zero-wait run against this file and fails on any difference, a run-count disagreement,
a missing or doubled line, or a log with no `BENCH` line. `make nano-qspi-control-test` runs the
real control (both benchmarks, the shipping `nano-sim`) on `make test`'s path, so a cycle count
that moves is caught by the change that moved it and not by the next sweep; its forced-red
direction is `make probe-gates`.

**Re-taking it.** When a change is meant to move nano's timing or the pinned compiler moves,
run `make nano-dhrystone` and `make nano-coremark`, read the two `BENCH` cycle counts, edit the
two lines, and re-take ADR-0186's table on the same tree in the same commit: the table's rows
are only comparable with a control from the tree they were measured on. Never edit a line to
silence the check without that sweep. The ADR amendment records the commit, the tree and the
toolchain each value was measured on; this file does not, because a file that is hand-edited
should not carry a stamp nothing grades.
