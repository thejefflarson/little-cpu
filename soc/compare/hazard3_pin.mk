# The Hazard3 SHA this repo is pinned to, the same way formal/pin.mk pins riscv-formal.
override HAZARD3_SHA := 8af992930f71a69b0e06c38734c1094f41a05ca0
override HAZARD3_URL := https://github.com/Wren6991/Hazard3.git

ifeq ($(shell printf '%s' '$(HAZARD3_SHA)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error HAZARD3_SHA must be a full 40-hex commit id, not a branch or tag: '$(HAZARD3_SHA)')
endif

override HAZARD3_DIR := \
  $(patsubst ./%,%,$(dir $(lastword $(MAKEFILE_LIST)))hazard3)

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

$(HAZARD3_DIR):
	rm -rf $@.tmp
	git clone --no-checkout $(HAZARD3_URL) $@.tmp
	git -C $@.tmp checkout --detach $(HAZARD3_SHA)
	@test "$$(git -C $@.tmp rev-parse HEAD)" = "$(HAZARD3_SHA)" \
	  || { echo "Hazard3 HEAD is not the pin $(HAZARD3_SHA)"; exit 1; }
	mv $@.tmp $@

$(HAZARD3_DIR)/%: | $(HAZARD3_DIR) ;
