include pin.mk

.DELETE_ON_ERROR:

# JOBS is spliced into a shell command line, so anything that is not a plain decimal --
# an error message, an empty string -- falls back rather than being passed through.
JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
ifeq ($(shell printf '%s' '$(JOBS)' | grep -cE '^[1-9][0-9]{0,3}$$'),0)
override JOBS := 4
endif

all: complete complete_cover complete-exclusions interrupt-tie-off \
     multihart-tie-off check \
     dmemcheck imemcheck \
     components_decoder components_executor components_accessor \
     components_pcloop components_traps components_busarbiter

# Every sby target here names a directory sby creates, and a directory's mtime bumps
# whenever an entry is written inside it -- so after one run the workdir is newer than
# every prerequisite and make calls the target up to date forever.
FORCE:
.PHONY: FORCE

%: %.sby FORCE
	rm -rf $@
	sby -f $<

check: checks interrupt-tie-off multihart-tie-off $(RISCV_FORMAL_DIR)
	-$(MAKE) -C $< -j$(JOBS) -k
	$(MAKE) check-baseline

.PHONY: check-baseline
check-baseline: check-baseline.sh EXPECTED_FAIL EXPECTED_CHECKS
	./check-baseline.sh checks EXPECTED_FAIL EXPECTED_CHECKS

.PHONY: checks
checks: genchecks-audit.py genchecks-local.py checks.cfg EXPECTED_CHECKS | $(RISCV_FORMAL_DIR)
	rm -rf $@
	python3 $<

.PHONY: genchecks-check
genchecks-check: $(RISCV_FORMAL_DIR)/checks/genchecks.py genchecks-local.py | $(RISCV_FORMAL_DIR)
	python3 check-genchecks.py $^

RTL_SOURCES := ../rtl/structs.v ../rtl/fetcher.v ../rtl/regfile.v \
               ../rtl/csrs.v ../rtl/decoder.v ../rtl/regsel.v ../rtl/pairtable.v ../rtl/executor.v \
               ../rtl/accessor.v ../rtl/writeback.v ../rtl/littlecpu.v

.PHONY: remeasure-fg
remeasure-fg: remeasure-fg.py depth_rules.py genchecks-local.py checks.cfg $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
	python3 $<

dmemcheck: dmemcheck.sby dmemcheck.sv arbiter.v $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
imemcheck: imemcheck.sby imemcheck.sv arbiter.v $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
cover:     cover.sby     cover.sv     arbiter.v $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

pcloop_cover: pcloop_cover.sby pcloop.sv ../rtl/fetcher.v ../rtl/decoder.v ../rtl/regsel.v \
              ../rtl/structs.v

busarbiter_cover: busarbiter_cover.sby busarbiter.sv ../rtl/busarbiter.v

complete:  complete.sby  complete.sv  arbiter.v complete-exclusions abc-engine $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

.PHONY: abc-engine
abc-engine: check-abc-engine.sh
	@./$<

complete_cover: complete_cover.sby complete.sv arbiter.v complete-exclusions $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

.PHONY: complete-exclusions
complete-exclusions: check-complete-exclusions.py complete.sv COMPLETE_EXCLUSIONS | $(RISCV_FORMAL_DIR)
	python3 $< complete.sv COMPLETE_EXCLUSIONS $(RISCV_FORMAL_DIR)

.PHONY: interrupt-tie-off
interrupt-tie-off: check-interrupt-tie-off.py INTERRUPT_TIE_OFF | $(RISCV_FORMAL_DIR)
	python3 $< . INTERRUPT_TIE_OFF $(RISCV_FORMAL_DIR)

.PHONY: multihart-tie-off
multihart-tie-off: check-multihart-tie-off.py MULTIHART_TIE_OFF ../rtl/littlecpu.v \
                   ../test/port_connect_test.py
	python3 $< . MULTIHART_TIE_OFF ..

.PHONY: nonperturbation
nonperturbation: check-nonperturbation.py
	python3 $<

.PHONY: components_decoder components_executor components_accessor components_pcloop components_traps \
        components_busarbiter
components_decoder: components.sby decoder-zkt-probe ../rtl/decoder.v ../rtl/regsel.v \
                     ../rtl/structs.v
	rm -rf $@
	sby -f $< decoder

.PHONY: decoder-zkt-probe
decoder-zkt-probe: decoder-zkt-probe.py ../rtl/decoder.v components.sby
	python3 $<
components_executor: components.sby executor-zkt-probe ../rtl/executor.v ../rtl/structs.v
	rm -rf $@
	sby -f $< executor

.PHONY: executor-zkt-probe
executor-zkt-probe: executor-zkt-probe.py ../rtl/executor.v components.sby
	python3 $<
components_accessor: components.sby ../rtl/accessor.v ../rtl/structs.v
	rm -rf $@
	sby -f $< accessor
components_pcloop: components.sby pcloop_cover ../rtl/fetcher.v ../rtl/decoder.v ../rtl/regsel.v \
                   ../rtl/structs.v pcloop.sv
	rm -rf $@
	sby -f $< pcloop
components_traps: components.sby traps-region-probe traps-tval-probe ../rtl/fetcher.v \
                  ../rtl/decoder.v ../rtl/regsel.v ../rtl/csrs.v ../rtl/structs.v traps.sv
	rm -rf $@
	sby -f $< traps

.PHONY: traps-region-probe
traps-region-probe: traps-region-probe.py traps.sv ../rtl/decoder.v
	python3 $<

.PHONY: traps-tval-probe
traps-tval-probe: traps-tval-probe.py traps.sv ../rtl/decoder.v
	python3 $<

components_busarbiter: components.sby busarbiter_cover busarbiter-probe \
                       ../rtl/busarbiter.v busarbiter.sv
	rm -rf $@
	sby -f $< busarbiter

.PHONY: busarbiter-probe
busarbiter-probe: busarbiter-probe.py busarbiter.sv ../rtl/busarbiter.v
	python3 $<

clean:
	rm -rf checks complete complete_cover cover dmemcheck imemcheck components components_*
	rm -rf pcloop_cover busarbiter_cover fg-probe fg-probe.cfg
	rm -rf region-probe tval-probe busarbiter-probe executor-zkt-probe
	rm -rf $(RISCV_FORMAL_DIR) $(RISCV_FORMAL_DIR).tmp

.PHONY: all clean
