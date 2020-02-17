all: complete check dmemcheck imemcheck

check: checks riscv-formal
	make -BC $< -j$(nproc)

checks: genchecks.py riscv-formal checks.cfg
	python3 $<

dmemcheck: dmemcheck.sby dmemcheck.sv riscv-formal
	sby -f $<

imemcheck: imemcheck.sby imemcheck.sv riscv-formal
	sby -f $<

complete: complete.sby riscv-formal
	sby -f $<

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

clean:
	rm -rf {checks,complete,riscv-formal,dmemcheck}

.PHONY: all clean
