all: complete check dmemcheck imemcheck fetcher

%: %.sby
	sby -f $<

check: checks riscv-formal
	make -BC $< -j$(nproc)

checks: genchecks.py riscv-formal checks.cfg
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
components_handshake: components.sby ../rtl/handshake.v
	sby -f $< handshake
components_skidbuffer: components.sby ../rtl/skidbuffer.v
	sby -f $< skidbuffer

riscv-formal:
	git clone https://github.com/YosysHQ/riscv-formal.git

clean:
	rm -rf {checks,complete,cover,riscv-formal,dmemcheck,imemcheck,components_*}

.PHONY: all clean
