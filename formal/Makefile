include pin.mk

# GNU nproc / macOS sysctl; both aren't on every box, so fall back to 4.
JOBS := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

all: complete check dmemcheck imemcheck components_fetcher

%: %.sby
	sby -f $<

check: checks riscv-formal
	make -BC $< -j$(JOBS)

checks: genchecks-local.py riscv-formal checks.cfg
	python3 $<

dmemcheck: dmemcheck.sby dmemcheck.sv riscv-formal
imemcheck: imemcheck.sby imemcheck.sv riscv-formal
complete: complete.sby complete.sv riscv-formal
cover: cover.sby cover.sv riscv-formal

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

riscv-formal:
	git clone https://github.com/YosysHQ/riscv-formal.git $@
	git -C $@ checkout $(RISCV_FORMAL_SHA)

clean:
	rm -rf {checks,complete,cover,riscv-formal,dmemcheck,imemcheck,components,components_*}

.PHONY: all clean
