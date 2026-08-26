# The Hazard3 SHA this repo is pinned to, the same way formal/pin.mk pins
# riscv-formal. Included by the root Makefile so there is exactly one place
# to bump it and exactly one recipe that materialises the clone.
#
# Hazard3 (github.com/Wren6991/Hazard3) is Apache-2.0, checked at this SHA
# before vendoring: LICENSE at the repo root, no third-party notices that
# narrow it.
#
# `override` so the pin cannot be defeated with `make HAZARD3_SHA=...` from a
# script or CI job: this is a supply-chain control, not a knob, the same
# reasoning formal/pin.mk states for riscv-formal. Nothing here executes code
# out of the clone -- soc/compare/bench_hazard3.v reads its Verilog sources
# straight out of it, read_verilog and iverilog both do, so a re-pin changes
# only which RTL text those tools compile.
override HAZARD3_SHA := 8af992930f71a69b0e06c38734c1094f41a05ca0
override HAZARD3_URL := https://github.com/Wren6991/Hazard3.git

ifeq ($(shell printf '%s' '$(HAZARD3_SHA)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error HAZARD3_SHA must be a full 40-hex commit id, not a branch or tag: '$(HAZARD3_SHA)')
endif

# Resolved relative to this file, so the same path works whether make was
# invoked from the repo root or from soc/compare/.
override HAZARD3_DIR := \
  $(patsubst ./%,%,$(dir $(lastword $(MAKEFILE_LIST)))hazard3)

# Fail closed: if a clone is already on disk at anything other than the pin,
# stop rather than silently building against it -- the same guard
# formal/pin.mk carries for riscv-formal, for the same reason. Skipped for
# `clean`, which is how you get out of this state.
ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifneq ($(wildcard $(HAZARD3_DIR)/.git),)
HAZARD3_HEAD := $(shell git -C $(HAZARD3_DIR) rev-parse HEAD 2>/dev/null)
ifneq ($(HAZARD3_HEAD),$(HAZARD3_SHA))
$(error $(HAZARD3_DIR) is at '$(HAZARD3_HEAD)', not the pin '$(HAZARD3_SHA)'. \
  Re-pin it with: git -C $(HAZARD3_DIR) fetch origin \
  && git -C $(HAZARD3_DIR) checkout --detach $(HAZARD3_SHA))
endif
endif
endif

# Clone atomically into $@.tmp and rename only once HEAD is verifiably at the
# pin, the same recipe shape as formal/pin.mk's.
$(HAZARD3_DIR):
	rm -rf $@.tmp
	git clone --no-checkout $(HAZARD3_URL) $@.tmp
	git -C $@.tmp checkout --detach $(HAZARD3_SHA)
	@test "$$(git -C $@.tmp rev-parse HEAD)" = "$(HAZARD3_SHA)" \
	  || { echo "Hazard3 HEAD is not the pin $(HAZARD3_SHA)"; exit 1; }
	mv $@.tmp $@

$(HAZARD3_DIR)/%: | $(HAZARD3_DIR) ;
