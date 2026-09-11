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

# The same slice-and-stop as formal/'s, and the same script: two sets, one layout.
.PHONY: check-shard
check-shard: checks $(RISCV_FORMAL_DIR)
	../../formal/check-shard.sh checks '$(CHECK_SHARD)' $(JOBS)

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

ill_e:       ill_e.sby       ill_e.sv ill-e-probe
ill_e_cover: ill_e_cover.sby ill_e.sv ill-e-probe

.PHONY: ill-e-probe
ill-e-probe: ill-e-probe.py ill_e.sv ill_e.sby ill_e_cover.sby
	python3 $<

.PHONY: check-rvfi-insn-check
check-rvfi-insn-check: $(RISCV_FORMAL_DIR)/checks/rvfi_insn_check.sv rvfi_insn_check.sv \
                       check-rvfi-insn-check.py | $(RISCV_FORMAL_DIR)
	python3 check-rvfi-insn-check.py $(RISCV_FORMAL_DIR)/checks/rvfi_insn_check.sv rvfi_insn_check.sv

.PHONY: remeasure-fg
remeasure-fg: ../../formal/remeasure-fg.py ../../formal/depth_rules.py checks.cfg ../nano.v \
              | $(RISCV_FORMAL_DIR)
	python3 $< .

clean:
	rm -rf checks complete complete_cover dmemcheck imemcheck fg-probe fg-probe.cfg
	rm -rf complete-cover-probe ill_e ill_e_cover ill-e-probe

.PHONY: all clean
all: check dmemcheck imemcheck complete complete_cover ill_e ill_e_cover check-rvfi-insn-check
