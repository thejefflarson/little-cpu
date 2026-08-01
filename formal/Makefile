include pin.mk

.DELETE_ON_ERROR:

# GNU nproc / macOS sysctl; neither is on every box, so fall back to 4. The
# result is spliced into a shell command line, so anything that is not a plain
# decimal (an error message on stderr-to-stdout, an empty string) falls back
# rather than being passed through.
JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
ifeq ($(shell printf '%s' '$(JOBS)' | grep -cE '^[1-9][0-9]{0,3}$$'),0)
override JOBS := 4
endif

all: complete check dmemcheck imemcheck components_decoder components_executor components_pcloop

# THE RULE FOR EVERY sby TARGET IN THIS FILE, INCLUDING ONES ADDED LATER:
# it re-runs unconditionally, and it throws its workdir away first.
#
# Every one of them names a DIRECTORY that sby creates, and make's up-to-date
# decision on a directory is worthless: a directory's mtime bumps whenever an
# entry is created inside it, so a workdir is newer than every prerequisite in
# the list the moment the previous run wrote its first file into it. That is
# ADR-0040's finding about `checks`, and the identical shape sat unfixed on its
# siblings for as long again -- with `imemcheck` and `dmemcheck` reachable
# through this rule and listing NO rtl/ file at all, so `make imemcheck` after
# a fetcher change printed "`imemcheck' is up to date" and exited 0. Both CI
# workflows are fresh checkouts and never saw it; the path a developer uses to
# convince themselves before pushing is the path that lied.
#
# `rm -rf $@` is belt to `sby -f`'s braces. `-f` already force-overwrites, so
# unlike `checks` -- whose generated makefile hardcodes an `sby` with no `-f`
# (ADR-0040) -- this recipe does not depend on the delete to re-run. It is here
# so that "the workdir is gone" is a property of THIS file rather than of a
# flag someone may later move, and so a run interrupted mid-write cannot leave
# half a workdir that the next run reads as its own output.
#
# FORCE, not `.PHONY`. A .PHONY target is EXCLUDED from implicit rule search,
# so `.PHONY: imemcheck` deletes the only rule that builds it and `make
# imemcheck` answers "Nothing to be done for `imemcheck'" and exits 0 --
# measured on GNU Make 3.81 and 4.x alike. That is the same failure this rule
# exists to remove, restated as a fix, so it must not be reached for here.
# Explicit targets (components_* below) are not pattern-matched and CAN use
# .PHONY; the split is deliberate and is why two mechanisms appear in one file.
FORCE:
.PHONY: FORCE

%: %.sby FORCE
	rm -rf $@
	sby -f $<

# `-k`: without it, GNU make stops scheduling new jobs at the first failing
# check, so a ladder with known failures (formal/EXPECTED_FAIL) never runs
# past its first parallel wave and the rest report as "no status" rather
# than PASS or FAIL (ADR-0022). `-k` is what makes "all 82 checks ran" true.
#
# The leading `-` on the sub-make is what lets check-baseline below be the
# target's verdict, and it is not laundering an error. Every generated .sby
# carries `expect pass,fail`, so `sby` exits 0 whether a check passed or
# failed and the sub-make's exit status never carried a verdict in the first
# place -- what it CAN do is abort before the compare step runs, which would
# leave the target exiting 0 with nothing having been checked. Anything the
# sub-make really failed to run shows up as a missing status file, which
# check-baseline counts as non-PASS.
check: checks $(RISCV_FORMAL_DIR)
	-$(MAKE) -C $< -j$(JOBS) -k
	$(MAKE) check-baseline

# Set equality against BOTH baselines, in both directions: EXPECTED_CHECKS
# (which checks exist) and EXPECTED_FAIL (which of them are red). Split out
# as its own target so a ladder that has already run can be re-graded in a
# second without re-running it, and so a local `make check` is gated by the
# same script the nightly grades with rather than by sby's suppressed exit
# code (ADR-0022, ADR-0033).
.PHONY: check-baseline
check-baseline: check-baseline.sh EXPECTED_FAIL EXPECTED_CHECKS
	./check-baseline.sh checks EXPECTED_FAIL EXPECTED_CHECKS

# genchecks-audit.py runs genchecks-local.py and additionally asserts the
# ladder's SHAPE before anything runs it: the generated set against
# EXPECTED_CHECKS, and the set genchecks DECLINED (no [depth] line, silently
# skipped) against checks.cfg's `#omit` declarations. Both are set
# equalities in both directions. It fails in about a second, which is the
# point -- check-baseline catches the same shrunken ladder, but only after
# the full run (ADR-0033).
#
# `checks` is PHONY and starts by deleting the directory, and both halves of
# that are load-bearing (ADR-0040). It used to be a plain file target whose
# recipe was just `python3 $<`, and in that form `make check` could not
# re-run the ladder at all -- it silently re-graded the PREVIOUS run's
# verdict:
#
#   1. `checks` is a DIRECTORY, and a directory's mtime bumps whenever an
#      entry is created inside it. `sby` creates checks/<name>/ for every
#      check it runs, so after one ladder run checks/ is newer than
#      checks.cfg, genchecks-local.py and EXPECTED_CHECKS all at once, and
#      make reports it up to date forever. Editing checks.cfg between two
#      runs therefore did NOT regenerate the ladder.
#   2. genchecks-local.py:72 is `sbycmd = "sby"` with no `-f`, and the
#      makefile it emits hardcodes that string. sby refuses to overwrite an
#      existing workdir, so every re-invocation aborted with "Directory
#      '<name>' already exists" and left the prior `status` file untouched.
#      `make -BC checks` -- which this target used to pass -- forces the
#      RULE, not sby, so it made the aborts louder without making anything
#      re-run.
#
# Measured on this repo at 317b86c, both runs from the same tree: run 1 took
# 310s and wrote 82 status files; run 2 took 22s, printed "Directory already
# exists" for all 82, and check-baseline still reported "82 checks: 82 pass".
# Every status file's mtime was byte-identical between the two. A ladder that
# reports a verdict it did not compute is docs/THREAT_MODEL.md's category 1,
# and it is a verdict about RTL that may since have changed underneath it.
#
# genchecks-local.py must not be patched to add `-f` (ADR-0031 constrains that
# vendored copy to differ from the pin by `basedir` only, and
# `make genchecks-check` enforces it), so the fix is here: throw the run
# directories away, regenerate, and let every status file be built by a real
# sby invocation. Regeneration costs about a second. `-B` is correspondingly
# gone from `check` above -- with no status file on disk every rule fires on
# its own, and keeping a flag that forces rules rather than reruns is what
# made this look handled for as long as it did.
#
# Re-grading a finished ladder without re-running it is still available, and
# is what `make check-baseline` is for.
.PHONY: checks
checks: genchecks-audit.py genchecks-local.py checks.cfg EXPECTED_CHECKS | $(RISCV_FORMAL_DIR)
	rm -rf $@
	python3 $<

# genchecks-local.py is vendored from the pin and may differ from it only by
# this repo's header and `basedir` (ADR-0031). That rule used to be enforced by
# a `cp` + `diff -u` recipe in the script's own header, run by hand or not at
# all -- the same gap `make monitor-check` already closes for test/monitor.v,
# the repo's other vendored generated artifact. Drift here is what produced the
# five-year-stale fork ADR-0031 exists to end.
.PHONY: genchecks-check
genchecks-check: $(RISCV_FORMAL_DIR)/checks/genchecks.py genchecks-local.py | $(RISCV_FORMAL_DIR)
	python3 check-genchecks.py $^

# The whole core, in the order every whole-core .sby reads it. These four
# targets each ran against an `.sby` and an `.sv` and NOTHING ELSE before this
# list existed, which is how `make imemcheck` -- the check CLAUDE.md names for
# ADR-0003's dual-word fetch window, and so the one most likely to be re-run by
# hand after touching rtl/fetcher.v -- could report a verdict about RTL it had
# never read.
#
# Read these lists as DOCUMENTATION of what each task compiles, not as a gate.
# The gate is the FORCE rule above: the target re-runs whatever this says. What
# they still buy is a loud `No rule to make target` if an rtl/ file is renamed
# out from under a check, instead of a confusing failure inside sby.
RTL_SOURCES := ../rtl/structs.v ../rtl/fetcher.v ../rtl/regfile.v \
               ../rtl/csrs.v ../rtl/decoder.v ../rtl/executor.v \
               ../rtl/accessor.v ../rtl/writeback.v ../rtl/littlecpu.v

dmemcheck: dmemcheck.sby dmemcheck.sv $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
imemcheck: imemcheck.sby imemcheck.sv $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)
cover:     cover.sby     cover.sv     $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

# `complete` is the depth-50 walk and by far the most expensive thing here, so
# unconditional re-running is the least welcome on it. It gets it anyway: a
# target asked for BY NAME must run, and the alternative -- a prerequisite list
# deciding -- is what this whole change removes, because the thing that list is
# compared against is a directory whose mtime is meaningless. Nothing pulls
# `complete` in implicitly except `all`.
complete:  complete.sby  complete.sv  $(RTL_SOURCES) | $(RISCV_FORMAL_DIR)

# ADR-0006 / ADR-0020 / ADR-0047: proves RVFI instrumentation doesn't change
# core behaviour. Not a .sby task -- it is a structural comparison of two yosys
# builds, not a solver query -- so it isn't covered by the generic `%: %.sby`
# rule above. Needs nothing but yosys and python3, and runs in about nine
# seconds, which is why it is a PR gate rather than a nightly.
#
# This REPLACES formal/equiv.sh, deleted in ADR-0047 along with its nightly
# step. That script asked equiv_make/equiv_simple/equiv_induct to prove
# sequential equivalence of the two builds and never converged at any budget:
# equiv_make's name-based matching left 459 of 495 $equiv cells unproven
# (essentially the whole datapath), because the two builds optimise to
# differently-named netlists. It sat in the nightly behind `continue-on-error`
# and `timeout 3600`, so "it's on the nightly" read as coverage for a check
# that had never once produced a verdict. A suppressed red is worse than no
# check.
#
# What replaced it is STRICTLY WEAKER than sequential equivalence and the
# script says so at length: it proves the instrumentation is UNREAD, not that
# two designs behave alike. That is the property ADR-0006 actually asked for.
.PHONY: nonperturbation
nonperturbation: check-nonperturbation.py
	python3 $<

# pipeline stages. fetcher/accessor/writeback are deliberately not here: their
# formal/components.sby tasks were deleted for carrying zero assertions
# (ADR-0006, ADR-0017) -- a "passing" proof with nothing asserted is not a
# result.
#
# These three carry their own recipes rather than reaching the `%: %.sby` rule
# (they pass a task name), so they are the half of the file that CAN say
# .PHONY -- implicit rule search is not involved. Same two properties as that
# rule: always run, delete the workdir first. Before this they named the
# directory sby creates and listed one rtl/ file each, so a warm tree made
# `make components_decoder` after a structs.v edit answer "up to date".
.PHONY: components_decoder components_executor components_pcloop
components_decoder: components.sby ../rtl/decoder.v ../rtl/structs.v
	rm -rf $@
	sby -f $< decoder
components_executor: components.sby ../rtl/executor.v ../rtl/structs.v
	rm -rf $@
	sby -f $< executor
# The composed fetcher+decoder task. It exists to DISCHARGE the standalone
# decoder task's `assume(in.pc == pc)` (ADR-0017), which is the one place that
# proof's content went, and ADR-0017 puts the pc loop in M2's scope. It had no
# target here and no CI step, and it had been FAILING since `e4f5250` -- its
# f_may_stall over-approximation predates ADR-0042's fifth stall reason, so the
# sequential-advance assertion covered a cycle on which the decoder legitimately
# holds the pc. Nothing noticed for the obvious reason (ADR-0046). It runs on CI
# now; a proof nobody runs is a prose-only guard.
components_pcloop: components.sby ../rtl/fetcher.v ../rtl/decoder.v \
                   ../rtl/structs.v pcloop.sv
	rm -rf $@
	sby -f $< pcloop

# The riscv-formal clone rule and its pin guard live in pin.mk, included above,
# so this and the root Makefile cannot drift apart on it.

# Written out rather than brace-expanded: `rm -rf {a,b,c}` is a bashism that
# /bin/sh (make's default shell) passes through literally, silently deleting
# nothing.
clean:
	rm -rf checks complete cover dmemcheck imemcheck components components_*
	rm -rf $(RISCV_FORMAL_DIR) $(RISCV_FORMAL_DIR).tmp

.PHONY: all clean
