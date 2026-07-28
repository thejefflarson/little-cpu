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

all: complete check dmemcheck imemcheck components_fetcher

%: %.sby
	sby -f $<

check: checks $(RISCV_FORMAL_DIR)
	$(MAKE) -BC $< -j$(JOBS)

checks: genchecks-local.py checks.cfg | $(RISCV_FORMAL_DIR)
	python3 $<

dmemcheck: dmemcheck.sby dmemcheck.sv | $(RISCV_FORMAL_DIR)
imemcheck: imemcheck.sby imemcheck.sv | $(RISCV_FORMAL_DIR)
complete: complete.sby complete.sv | $(RISCV_FORMAL_DIR)
cover: cover.sby cover.sv | $(RISCV_FORMAL_DIR)

# pipeline stages
components_fetcher: components.sby ../rtl/fetcher.v
	sby -f $< fetcher
components_decoder: components.sby ../rtl/decoder.v
	sby -f $< decoder
components_executor: components.sby ../rtl/executor.v
	sby -f $< executor
components_accessor: components.sby ../rtl/accessor.v
	sby -f $< accessor
components_writeback: components.sby ../rtl/writeback.v
	sby -f $< writeback

# The riscv-formal clone rule and its pin guard live in pin.mk, included above,
# so this and the root Makefile cannot drift apart on it.

# Written out rather than brace-expanded: `rm -rf {a,b,c}` is a bashism that
# /bin/sh (make's default shell) passes through literally, silently deleting
# nothing.
clean:
	rm -rf checks complete cover dmemcheck imemcheck components components_*
	rm -rf $(RISCV_FORMAL_DIR) $(RISCV_FORMAL_DIR).tmp

.PHONY: all clean
