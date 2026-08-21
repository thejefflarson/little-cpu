include pin.mk

.DELETE_ON_ERROR:

# JOBS is spliced into a shell command line, so anything that is not a plain
# decimal -- an error message, an empty string -- falls back rather than being
# passed through.
JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
ifeq ($(shell printf '%s' '$(JOBS)' | grep -cE '^[1-9][0-9]{0,3}$$'),0)
override JOBS := 4
endif

all: complete complete_cover complete-exclusions interrupt-tie-off \
     multihart-tie-off check \
     dmemcheck imemcheck \
     components_decoder components_executor components_accessor \
     components_pcloop components_traps components_busarbiter

# Every sby target here names a directory sby creates, and a directory's mtime
# bumps whenever an entry is written inside it -- so after one run the workdir
# is newer than every prerequisite and make calls the target up to date
# forever. FORCE and the `rm -rf` are what stop that, and the same applies to
# any sby target added later.
#
# FORCE rather than .PHONY: a .PHONY target is excluded from implicit rule
# search, so `.PHONY: imemcheck` would delete the only rule that builds it and
# `make imemcheck` would answer "Nothing to be done" and exit 0. The explicit
# targets further down are not pattern-matched and can use .PHONY.
FORCE:
.PHONY: FORCE

%: %.sby FORCE
	rm -rf $@
	sby -f $<

# `-k` so one failing check does not stop the rest from being scheduled. The
# leading `-` lets check-baseline below be the verdict: every generated .sby
# carries `expect pass,fail`, so sby exits 0 whether a check passed or failed,
# and anything that really failed to run reaches check-baseline as a missing
# status file.
check: checks interrupt-tie-off multihart-tie-off $(RISCV_FORMAL_DIR)
	-$(MAKE) -C $< -j$(JOBS) -k
	$(MAKE) check-baseline

# Split out so a finished run can be re-graded in a second without re-running
# it, and so a local `make check` is gated by the same script CI grades with.
.PHONY: check-baseline
check-baseline: check-baseline.sh EXPECTED_FAIL EXPECTED_CHECKS
	./check-baseline.sh checks EXPECTED_FAIL EXPECTED_CHECKS

# genchecks-audit.py runs genchecks-local.py and also compares the generated
# set against EXPECTED_CHECKS, and the set genchecks silently declined against
# checks.cfg's `#omit` lines, both ways round, in about a second.
#
# PHONY and the `rm -rf` are both load-bearing; reverting either brings back a
# verdict nobody computed. `checks` is a directory, and a directory's mtime
# bumps on every entry sby writes into it, so a plain file target reads as up
# to date forever. genchecks-local.py also emits a makefile hardcoding `sby`
# with no `-f`, which then refuses to overwrite the old workdir. Measured: a
# second run printed "Directory already exists" for every check, wrote no
# status file, and check-baseline still reported them all passing.
.PHONY: checks
checks: genchecks-audit.py genchecks-local.py checks.cfg EXPECTED_CHECKS | $(RISCV_FORMAL_DIR)
	rm -rf $@
	python3 $<

# genchecks-local.py is vendored from the pin and may differ from it only by
# this repo's header and `basedir`. Nothing enforced that before this target.
.PHONY: genchecks-check
genchecks-check: $(RISCV_FORMAL_DIR)/checks/genchecks.py genchecks-local.py | $(RISCV_FORMAL_DIR)
	python3 check-genchecks.py $^

# Documentation of what each whole-core task compiles, not a gate -- the FORCE
# rule re-runs the target whatever this list says. What it buys is a loud "No
# rule to make target" if an rtl/ file is renamed out from under a check.
RTL_SOURCES := ../rtl/structs.v ../rtl/fetcher.v ../rtl/regfile.v \
               ../rtl/csrs.v ../rtl/decoder.v ../rtl/regsel.v ../rtl/executor.v \
               ../rtl/accessor.v ../rtl/writeback.v ../rtl/littlecpu.v

# The two figures every [depth] entry is derived from, swept out of `hang` and
# `liveness` and graded against the `#derive` lines that declare them. Six sby
# runs, a few seconds each, so it is cheap to reach for -- but it is not on CI
# and adds no ratchet: what CI enforces is that the depths clear the DECLARED
# figures, which `checks` does in a second. This is what says the declaration is
# still true of the design, and it is the target to run before landing anything
# that adds a stall reason or lengthens a stage.
.PHONY: remeasure-fg
remeasure-fg: remeasure-fg.py depth_rules.py genchecks-local.py checks.cfg $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
	python3 $<

dmemcheck: dmemcheck.sby dmemcheck.sv arbiter.v $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
imemcheck: imemcheck.sby imemcheck.sv arbiter.v $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
cover:     cover.sby     cover.sv     arbiter.v $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

# The anti-vacuity control for components_pcloop, over the same pcloop.sv. No
# riscv-formal clone: it instantiates two rtl/ modules and nothing else, the way
# the components tasks do, so it runs in the same CI job they do.
pcloop_cover: pcloop_cover.sby pcloop.sv ../rtl/fetcher.v ../rtl/decoder.v ../rtl/regsel.v \
              ../rtl/structs.v

# The anti-vacuity control for components_busarbiter, over the same
# busarbiter.sv. Every assertion in that harness is guarded, and an arbiter that
# grants nobody satisfies the lot; these six goals are what say otherwise.
busarbiter_cover: busarbiter_cover.sby busarbiter.sv ../rtl/busarbiter.v

# complete-exclusions is a prerequisite rather than a companion target: the
# exclusion set is what makes this check passable at all, and checking it costs
# milliseconds and fails before sby starts.
#
# abc-engine is a prerequisite for the second half of that reason only. It grades
# nothing about the design; it turns one toolchain failure into a sentence.
complete:  complete.sby  complete.sv  arbiter.v complete-exclusions abc-engine $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

# This is the only target whose engine is `abc bmc3`, and where yosys and sby
# were installed separately they can disagree about the call sby's AIG build
# makes. That dies on a command syntax error in a pass nobody here wrote, which
# reads like a broken branch and has been diagnosed as local more than once. The
# script says what it is, and can only add a failure: it exits 0 unless this
# yosys really refuses the call this sby is going to make.
.PHONY: abc-engine
abc-engine: check-abc-engine.sh
	@./$<

# The anti-vacuity control for the line above, over the same complete.sv: every
# opcode class the exclusion set does not name must be reachable as a real
# non-trapping retire.
complete_cover: complete_cover.sby complete.sv arbiter.v complete-exclusions $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

# Set equality both ways between complete.sv's declared exclusions and
# COMPLETE_EXCLUSIONS, plus a re-derivation from the pinned clone that each
# excluded mnemonic really has no spec model.
.PHONY: complete-exclusions
complete-exclusions: check-complete-exclusions.py complete.sv COMPLETE_EXCLUSIONS | $(RISCV_FORMAL_DIR)
	python3 $< complete.sv COMPLETE_EXCLUSIONS $(RISCV_FORMAL_DIR)

# A prerequisite of `check` rather than a companion target, the same way
# complete-exclusions is of `complete`: the tie-off is what makes the generated
# checks mean what their depths say, and checking it costs milliseconds and
# fails before sby starts.
.PHONY: interrupt-tie-off
interrupt-tie-off: check-interrupt-tie-off.py INTERRUPT_TIE_OFF | $(RISCV_FORMAL_DIR)
	python3 $< . INTERRUPT_TIE_OFF $(RISCV_FORMAL_DIR)

# The other half of the same boundary, and a prerequisite of `check` for the
# same reason: the shared-bus inputs are constants in every harness, so the
# generated checks describe one hart and the depths are derived on that machine.
# It needs no riscv-formal clone -- what it re-derives is littlecpu's own port
# list, through test/port_connect_test.py's parser rather than a second one.
.PHONY: multihart-tie-off
multihart-tie-off: check-multihart-tie-off.py MULTIHART_TIE_OFF ../rtl/littlecpu.v \
                   ../test/port_connect_test.py
	python3 $< . MULTIHART_TIE_OFF ..

# A structural comparison of two yosys builds rather than a solver query, so it
# is not covered by the `%: %.sby` rule above. It shows the RVFI instrumentation
# is unread by the core, which is strictly weaker than sequential equivalence
# and must not be quoted as if it were.
.PHONY: nonperturbation
nonperturbation: check-nonperturbation.py
	python3 $<

# fetcher and writeback are deliberately absent: their tasks carried zero
# assertions and were deleted rather than left to pass having asked nothing.
# The accessor is back because it acquired one -- the request block's gate is
# the only thing between a held store and a device written thirty-three times.
#
# These five pass a task name, so they need their own recipes and cannot reach
# the `%: %.sby` rule -- which is why they can say .PHONY where that rule
# cannot. Same two properties as it: always run, delete the workdir first.
.PHONY: components_decoder components_executor components_accessor components_pcloop components_traps \
        components_busarbiter
components_decoder: components.sby ../rtl/decoder.v ../rtl/regsel.v ../rtl/structs.v
	rm -rf $@
	sby -f $< decoder
components_executor: components.sby ../rtl/executor.v ../rtl/structs.v
	rm -rf $@
	sby -f $< executor
# One bus transaction per memory instruction. Decode holds `launch` for every
# cycle of a divide, and a device written twice for one store is a defect no
# value anywhere in the machine records.
components_accessor: components.sby ../rtl/accessor.v ../rtl/structs.v
	rm -rf $@
	sby -f $< accessor
# Discharges the standalone decoder task's `assume(in.pc == pc)`, which is
# where that proof's content went.
#
# pcloop_cover is a prerequisite rather than a companion target, the same way
# complete-exclusions is of `complete`: it is what says this proof's increment
# assertion is still reached, and a control that can be run separately from the
# thing it controls eventually is not run at all.
components_pcloop: components.sby pcloop_cover ../rtl/fetcher.v ../rtl/decoder.v ../rtl/regsel.v \
                   ../rtl/structs.v pcloop.sv
	rm -rf $@
	sby -f $< pcloop
# The only place mtvec, mepc, mcause and mstatus are real registers under a
# proof. The generated checks drop every value comparison once an instruction
# traps, keeping only the trap flag, and their pc checks accept whatever target
# the core reports -- so nothing else says a trap lands on mtvec, or that the
# state it saved is right.
#
# traps-region-probe is a prerequisite for the reason pcloop_cover is one of the
# proof it controls: this core raises no load or store access fault, so the arm
# that models one is satisfied by never being reached, and a control that can be
# run separately from the thing it controls eventually is not run at all.
components_traps: components.sby traps-region-probe ../rtl/fetcher.v ../rtl/decoder.v \
                  ../rtl/regsel.v ../rtl/csrs.v ../rtl/structs.v traps.sv
	rm -rf $@
	sby -f $< traps

.PHONY: traps-region-probe
traps-region-probe: traps-region-probe.py traps.sv ../rtl/decoder.v
	python3 $<

# The bus arbiter, which no core is wired to yet: two harts, free requests, free
# locks and two bits of state, so this proves the whole reachable state of it
# rather than a bounded walk through part. Starvation is why it exists, and it
# is the one property no program can be written to catch.
#
# Both prerequisites are prerequisites rather than companion targets, for the
# reason pcloop_cover is one: busarbiter_cover says the guarded assertions are
# still reached at all, busarbiter-probe says two of them still fail for their
# own reason, and a control that can be run apart from the thing it controls
# eventually is not run at all.
components_busarbiter: components.sby busarbiter_cover busarbiter-probe \
                       ../rtl/busarbiter.v busarbiter.sv
	rm -rf $@
	sby -f $< busarbiter

.PHONY: busarbiter-probe
busarbiter-probe: busarbiter-probe.py busarbiter.sv ../rtl/busarbiter.v
	python3 $<

# Written out rather than brace-expanded: `rm -rf {a,b,c}` is a bashism that
# /bin/sh, make's default shell, passes through literally and deletes nothing.
clean:
	rm -rf checks complete complete_cover cover dmemcheck imemcheck components components_*
	rm -rf pcloop_cover busarbiter_cover fg-probe fg-probe.cfg region-probe busarbiter-probe
	rm -rf $(RISCV_FORMAL_DIR) $(RISCV_FORMAL_DIR).tmp

.PHONY: all clean
