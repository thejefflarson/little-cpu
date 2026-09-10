include ../../formal/pin.mk

.DELETE_ON_ERROR:

JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
ifeq ($(shell printf '%s' '$(JOBS)' | grep -cE '^[1-9][0-9]{0,3}$$'),0)
override JOBS := 4
endif

FORCE:
.PHONY: FORCE

.PHONY: checks
checks: ../../formal/genchecks-audit.py ../../formal/genchecks-local.py checks.cfg \
        EXPECTED_CHECKS | $(RISCV_FORMAL_DIR)
	rm -rf checks
	python3 $< .

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
complete_cover: complete_cover.sby complete.sv ../nano.v complete-cover-probe | $(RISCV_FORMAL_DIR)

.PHONY: complete-cover-probe
complete-cover-probe: complete-cover-probe.py complete.sv complete_cover.sby ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

.PHONY: remeasure-fg
remeasure-fg: remeasure-fg.py ../../formal/depth_rules.py checks.cfg ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

clean:
	rm -rf checks complete complete_cover dmemcheck imemcheck fg-probe fg-probe.cfg
	rm -rf complete-cover-probe

.PHONY: all clean
all: check dmemcheck imemcheck complete complete_cover
