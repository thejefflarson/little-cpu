include ../../formal/pin.mk
include ../../mk/toolchain.mk

.DELETE_ON_ERROR:

JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
ifeq ($(shell printf '%s' '$(JOBS)' | grep -cE '^[1-9][0-9]{0,3}$$'),0)
override JOBS := 4
endif

FORCE:
.PHONY: FORCE

.PHONY: checks
checks: ../../formal/genchecks-audit.py ../../formal/genchecks-local.py checks.cfg \
        EXPECTED_CHECKS rvfi_insn_check.sv | $(RISCV_FORMAL_DIR)
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

# FORCE, not .PHONY: .PHONY takes `%` literally, and listing the expansions disables this rule.
check-solver-%: FORCE
	@../../mk/check-solvers.sh $*

.PHONY: memcheck-depth
memcheck-depth: ../../formal/check-memcheck-depth.py ../../formal/depth_rules.py checks.cfg \
                dmemcheck.sby imemcheck.sby dmemcheck_latch.sby imemcheck_latch.sby
	python3 ../../formal/check-memcheck-depth.py . dmemcheck.sby 2
	python3 ../../formal/check-memcheck-depth.py . imemcheck.sby 1
	python3 ../../formal/check-memcheck-depth.py . dmemcheck_latch.sby 2 --clk2fflogic
	python3 ../../formal/check-memcheck-depth.py . imemcheck_latch.sby 1 --clk2fflogic

dmemcheck: dmemcheck.sby dmemcheck.sv ../nano.v memcheck-depth | $(RISCV_FORMAL_DIR)
imemcheck: imemcheck.sby imemcheck.sv ../nano.v memcheck-depth | $(RISCV_FORMAL_DIR)
dmemcheck_latch: dmemcheck_latch.sby dmemcheck.sv ../nano.v memcheck-depth | $(RISCV_FORMAL_DIR)
imemcheck_latch: imemcheck_latch.sby imemcheck.sv ../nano.v memcheck-depth | $(RISCV_FORMAL_DIR)
complete:  complete.sby  complete.sv  ../nano.v complete-exclusions | $(RISCV_FORMAL_DIR)
complete_cover: complete_cover.sby complete.sv ../nano.v complete-cover-probe \
                complete-exclusions | $(RISCV_FORMAL_DIR)

.PHONY: complete-exclusions
complete-exclusions: ../../formal/check-complete-exclusions.py complete.sv \
                     COMPLETE_EXCLUSIONS | $(RISCV_FORMAL_DIR)
	python3 $< complete.sv COMPLETE_EXCLUSIONS $(RISCV_FORMAL_DIR)
ill_e: ill_e.sby ill_e.sv ../nano.v ill-e-probe | $(RISCV_FORMAL_DIR)
ill_e_cover: ill_e_cover.sby ill_e.sv ../nano.v | $(RISCV_FORMAL_DIR)
ill_e_latch: ill_e_latch.sby ill_e.sv ../nano.v ill-e-latch-probe | $(RISCV_FORMAL_DIR)
ill_e_latch_cover: ill_e_latch_cover.sby ill_e.sv ../nano.v | $(RISCV_FORMAL_DIR)

.PHONY: components_traps
components_traps: traps.sby traps.sv ../nano.v traps-region-probe traps-tval-probe \
                  | $(RISCV_FORMAL_DIR)
	rm -rf traps
	sby -f traps.sby

.PHONY: components_traps_latch
components_traps_latch: traps_latch.sby traps.sv ../nano.v traps-region-latch-probe \
                        traps-tval-latch-probe | $(RISCV_FORMAL_DIR)
	rm -rf traps_latch
	sby -f traps_latch.sby

.PHONY: traps-region-probe
traps-region-probe: traps-region-probe.py probe_common.py traps.sv ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

.PHONY: traps-tval-probe
traps-tval-probe: traps-tval-probe.py probe_common.py traps.sv ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

.PHONY: traps-region-latch-probe
traps-region-latch-probe: traps-region-probe.py probe_common.py traps.sv ../nano.v \
                          | $(RISCV_FORMAL_DIR)
	python3 $< --sby-file traps_latch.sby --workdir region-probe-latch

.PHONY: traps-tval-latch-probe
traps-tval-latch-probe: traps-tval-probe.py probe_common.py traps.sv ../nano.v \
                        | $(RISCV_FORMAL_DIR)
	python3 $< --sby-file traps_latch.sby --workdir tval-probe-latch

dmemcheck_cover: dmemcheck_cover.sby dmemcheck.sv ../nano.v dmemcheck-cover-probe | $(RISCV_FORMAL_DIR)
imemcheck_cover: imemcheck_cover.sby imemcheck.sv ../nano.v imemcheck-cover-probe | $(RISCV_FORMAL_DIR)
dmemcheck_latch_cover: dmemcheck_latch_cover.sby dmemcheck.sv ../nano.v \
                       dmemcheck-latch-cover-probe | $(RISCV_FORMAL_DIR)
imemcheck_latch_cover: imemcheck_latch_cover.sby imemcheck.sv ../nano.v \
                       imemcheck-latch-cover-probe | $(RISCV_FORMAL_DIR)

.PHONY: complete-cover-probe
complete-cover-probe: complete-cover-probe.py ../../formal/depth_rules.py complete.sv \
                      complete_cover.sby complete.sby ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

.PHONY: dmemcheck-cover-probe imemcheck-cover-probe
dmemcheck-cover-probe: ../../formal/memcheck-cover-probe.py dmemcheck.sv dmemcheck_cover.sby \
                       ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $< --harness nano/formal --check dmemcheck --workdir memcheck-cover-probe/dmemcheck
imemcheck-cover-probe: ../../formal/memcheck-cover-probe.py imemcheck.sv imemcheck_cover.sby \
                       ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $< --harness nano/formal --check imemcheck --workdir memcheck-cover-probe/imemcheck

.PHONY: dmemcheck-latch-cover-probe imemcheck-latch-cover-probe
dmemcheck-latch-cover-probe: ../../formal/memcheck-cover-probe.py dmemcheck.sv \
                             dmemcheck_latch_cover.sby ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $< --harness nano/formal --check dmemcheck_latch \
	  --workdir memcheck-cover-probe/dmemcheck_latch
imemcheck-latch-cover-probe: ../../formal/memcheck-cover-probe.py imemcheck.sv \
                             imemcheck_latch_cover.sby ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $< --harness nano/formal --check imemcheck_latch \
	  --workdir memcheck-cover-probe/imemcheck_latch

.PHONY: ill-e-probe
ill-e-probe: ill-e-probe.py ill_e.sv ill_e.sby ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $<

.PHONY: ill-e-latch-probe
ill-e-latch-probe: ill-e-probe.py ill_e.sv ill_e_latch.sby ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $< --sby-file ill_e_latch.sby --workdir ill-e-latch-probe

.PHONY: qspi-probe
qspi-probe: qspi-probe.py qspi.sby ../qspi.v
	python3 $<

.PHONY: components_qspi
components_qspi: qspi.sby ../qspi.v qspi-probe
	rm -rf qspi
	sby -f qspi.sby

.PHONY: check-rvfi-insn-check
check-rvfi-insn-check: $(RISCV_FORMAL_DIR)/checks/rvfi_insn_check.sv rvfi_insn_check.sv \
                       check-rvfi-insn-check.py | $(RISCV_FORMAL_DIR)
	python3 check-rvfi-insn-check.py $(RISCV_FORMAL_DIR)/checks/rvfi_insn_check.sv rvfi_insn_check.sv

.PHONY: remeasure-fg
remeasure-fg: ../../formal/remeasure-fg.py ../../formal/depth_rules.py checks.cfg check-solver-btorsim \
              ../nano.v | $(RISCV_FORMAL_DIR)
	python3 $< .

clean:
	rm -rf checks complete complete_cover dmemcheck imemcheck fg-probe fg-probe.cfg
	rm -rf complete-cover-probe ill_e ill_e_cover ill-e-probe
	rm -rf dmemcheck_cover imemcheck_cover memcheck-cover-probe
	rm -rf qspi qspi-probe traps region-probe tval-probe
	rm -rf dmemcheck_latch imemcheck_latch dmemcheck_latch_cover imemcheck_latch_cover
	rm -rf ill_e_latch ill_e_latch_cover ill-e-latch-probe
	rm -rf traps_latch region-probe-latch tval-probe-latch

.PHONY: all clean
all: check dmemcheck imemcheck dmemcheck_cover imemcheck_cover complete complete_cover \
     ill_e ill_e_cover check-rvfi-insn-check components_qspi components_traps

# NANO_LATCH_RF's own set. `check` and `remeasure-fg` stay flop-only: both derive a
# generated .sby's BMC depth from the same value as RISCV_FORMAL_CHECK_CYCLE, which a
# pinned genchecks-local.py forbids decoupling for clk2fflogic; every check below is
# hand-written, with no such macro.
.PHONY: all-latch
all-latch: dmemcheck_latch imemcheck_latch dmemcheck_latch_cover imemcheck_latch_cover \
           ill_e_latch ill_e_latch_cover components_traps_latch
