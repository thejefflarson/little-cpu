include ../../formal/pin.mk

.DELETE_ON_ERROR:

JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
ifeq ($(shell printf '%s' '$(JOBS)' | grep -cE '^[1-9][0-9]{0,3}$$'),0)
override JOBS := 4
endif

FORCE:
.PHONY: FORCE

# checks.cfg is generated with absolute paths spliced in: @basedir@ in the shared
# genchecks-local.py always names formal/riscv-formal, never this directory.
checks.cfg: checks.cfg.in
	sed 's#@@NANO_FORMAL_DIR@@#$(abspath .)#g' $< > $@

.PHONY: checks
checks: ../../formal/genchecks-local.py checks.cfg | $(RISCV_FORMAL_DIR)
	rm -rf checks
	python3 $<

.PHONY: check
check: checks $(RISCV_FORMAL_DIR)
	-$(MAKE) -C checks -j$(JOBS) -k
	$(MAKE) check-baseline

.PHONY: check-baseline
check-baseline: ../../formal/check-baseline.sh EXPECTED_FAIL EXPECTED_CHECKS
	../../formal/check-baseline.sh checks EXPECTED_FAIL EXPECTED_CHECKS

%: %.sby FORCE
	rm -rf $@
	sby -f $<

dmemcheck: dmemcheck.sby dmemcheck.sv ../nano.v | $(RISCV_FORMAL_DIR)
imemcheck: imemcheck.sby imemcheck.sv ../nano.v | $(RISCV_FORMAL_DIR)
complete:  complete.sby  complete.sv  ../nano.v | $(RISCV_FORMAL_DIR)

.PHONY: remeasure-fg
remeasure-fg: remeasure-fg.py ../../formal/depth_rules.py checks.cfg ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

clean:
	rm -rf checks complete dmemcheck imemcheck fg-probe fg-probe.cfg checks.cfg

.PHONY: all clean
all: check dmemcheck imemcheck complete
