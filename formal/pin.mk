# The riscv-formal SHA this repo is pinned to. Read by both the root
# Makefile (test/monitor.v, monitor-check) and formal/Makefile (the
# riscv-formal clone target) so there is exactly one place to bump it.
#
# Bumping this requires regenerating test/monitor.v (`make monitor-check`
# will fail until you do) and rerunning the formal ladder. See ADR-0006.
RISCV_FORMAL_SHA := c992aa61fdfe0846c5ed90324c596202a1c69b76
