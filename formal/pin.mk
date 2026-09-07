# The riscv-formal SHA this repo is pinned to.
override RISCV_FORMAL_SHA := c992aa61fdfe0846c5ed90324c596202a1c69b76
override RISCV_FORMAL_URL := https://github.com/YosysHQ/riscv-formal.git

ifeq ($(shell printf '%s' '$(RISCV_FORMAL_SHA)' | grep -cE '^[0-9a-f]{40}$$'),0)
$(error RISCV_FORMAL_SHA must be a full 40-hex commit id, not a branch or tag: '$(RISCV_FORMAL_SHA)')
endif

override RISCV_FORMAL_DIR := \
  $(patsubst ./%,%,$(dir $(lastword $(MAKEFILE_LIST)))riscv-formal)

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

$(RISCV_FORMAL_DIR):
	rm -rf $@.tmp
	git clone --no-checkout $(RISCV_FORMAL_URL) $@.tmp
	git -C $@.tmp checkout --detach $(RISCV_FORMAL_SHA)
	@test "$$(git -C $@.tmp rev-parse HEAD)" = "$(RISCV_FORMAL_SHA)" \
	  || { echo "riscv-formal HEAD is not the pin $(RISCV_FORMAL_SHA)"; exit 1; }
	mv $@.tmp $@

$(RISCV_FORMAL_DIR)/%: | $(RISCV_FORMAL_DIR) ;
