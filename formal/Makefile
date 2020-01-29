check: checks
	make -C $< -j$(nproc)

checks: genchecks.py riscv-formal checks.cfg
	rm -rf $@
	python3 $<

complete: riscv-formal
	sby -f complete.sby

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

.PHONY: complete check
