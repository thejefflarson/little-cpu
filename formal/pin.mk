# The riscv-formal SHA this repo is pinned to. Included by both the root
# Makefile and formal/Makefile so there is exactly one place to bump it, and
# exactly one recipe that materialises the clone.
#
# Bumping this requires regenerating test/monitor.v (`make monitor-check`
# will fail until you do) and rerunning the formal ladder. See ADR-0006.
#
# `override` so the pin cannot be defeated with `make RISCV_FORMAL_SHA=...`
# from a script or CI job: this is a supply-chain control, not a knob. The
# repo executes Python straight out of this clone (checks/rvfi_macros.py,
# monitor/generate.py) and commits its stdout as tracked, compiled source.
override RISCV_FORMAL_SHA := c992aa61fdfe0846c5ed90324c596202a1c69b76
override RISCV_FORMAL_URL := https://github.com/YosysHQ/riscv-formal.git

ifeq ($(shell printf '%s' '$(RISCV_FORMAL_SHA)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error RISCV_FORMAL_SHA must be a full 40-hex commit id, not a branch or tag: '$(RISCV_FORMAL_SHA)')
endif

# Resolved relative to this file, so the same path works whether make was
# invoked from the repo root or from formal/.
override RISCV_FORMAL_DIR := \
  $(patsubst ./%,%,$(dir $(lastword $(MAKEFILE_LIST)))riscv-formal)

# Fail closed: if a clone is already on disk at anything other than the pin
# (a pre-existing unpinned clone, or one left behind after the pin was
# bumped), stop rather than silently building against it. Make will not
# re-run the clone recipe for a directory that already exists, so without
# this check the pin reads as a control while providing no protection.
# Skipped for `clean`, which is how you get out of this state.
ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifneq ($(wildcard $(RISCV_FORMAL_DIR)/.git),)
RISCV_FORMAL_HEAD := $(shell git -C $(RISCV_FORMAL_DIR) rev-parse HEAD 2>/dev/null)
ifneq ($(RISCV_FORMAL_HEAD),$(RISCV_FORMAL_SHA))
$(error $(RISCV_FORMAL_DIR) is at '$(RISCV_FORMAL_HEAD)', not the pin \
  '$(RISCV_FORMAL_SHA)'. Re-pin it with: git -C $(RISCV_FORMAL_DIR) fetch origin \
  && git -C $(RISCV_FORMAL_DIR) checkout --detach $(RISCV_FORMAL_SHA))
endif
endif
endif

# Clone atomically into $@.tmp and rename only once HEAD is verifiably at the
# pin. A failed checkout therefore leaves no directory behind, so make retries
# the whole thing next run instead of treating a half-built unpinned clone as
# a satisfied target.
$(RISCV_FORMAL_DIR):
	rm -rf $@.tmp
	git clone --no-checkout $(RISCV_FORMAL_URL) $@.tmp
	git -C $@.tmp checkout --detach $(RISCV_FORMAL_SHA)
	@test "$$(git -C $@.tmp rev-parse HEAD)" = "$(RISCV_FORMAL_SHA)" \
	  || { echo "riscv-formal HEAD is not the pin $(RISCV_FORMAL_SHA)"; exit 1; }
	mv $@.tmp $@

# Everything under the clone comes into existence with the clone itself, so
# recipes can depend on a specific script (and track its mtime) rather than on
# the directory, whose mtime changes on any write anywhere inside it.
$(RISCV_FORMAL_DIR)/%: | $(RISCV_FORMAL_DIR) ;
