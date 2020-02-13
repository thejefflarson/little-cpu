check: checks
	make -C $< -j$(nproc)

checks: genchecks.py riscv-formal checks.cfg
	python3 $<

complete: riscv-formal
	sby -f complete.sby
	python3 ./riscv-formal/cores/picorv32/disasm.py complete/engine_0/trace.vcd

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

clean:
	rm -rf {checks,complete,riscv-formal}

.PHONY: complete check clean
