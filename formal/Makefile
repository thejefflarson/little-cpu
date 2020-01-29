check: checks
	cd checks
	make
	cd ..

checks: genchecks.py riscv-formal checks.cfg
	python3 $<

complete: riscv-formal
	sby -f complete.sby

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

.PHONY: complete
