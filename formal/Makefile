check: checks complete dmemcheck imemcheck
	make -C $< -j$(nproc)
	find . -name FAIL | $(exit $(wc -l))

checks: genchecks.py riscv-formal checks.cfg
	python3 $<

dmemcheck: dmemcheck.sby dememcheck.sv
	sby -f $<

imemcheck: imemcheck.sby imemcheck.sv
	sby -f $<

complete: riscv-formal
	sby -f complete.sby
	python3 ./riscv-formal/cores/picorv32/disasm.py complete/engine_0/trace.vcd

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

clean:
	rm -rf {checks,complete,riscv-formal,dmemcheck}

.PHONY: complete check clean
