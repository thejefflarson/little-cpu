all: complete check dmemcheck imemcheck fetcher decoder

%: %.sby
	sby -f $<

check: checks | riscv-formal
	make -BC $< -j$(nproc)

checks: genchecks.py riscv-formal checks.cfg
	python3 $<

dmemcheck: dmemcheck.sby | dmemcheck.sv riscv-formal
imemcheck: imemcheck.sby | imemcheck.sv riscv-formal
complete: complete.sby | complete.sv riscv-formal
cover: cover.sby | cover.sv riscv-formal
fetcher: fetcher.sby | ../rtl/fetcher.v
decoder: decoder.sby | ../rtl/decoder.v

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

clean:
	rm -rf {checks,complete,cover,riscv-formal,dmemcheck,imemcheck}

.PHONY: all clean
