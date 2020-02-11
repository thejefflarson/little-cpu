check: checks
	make -C $< -j4

checks: genchecks.py riscv-formal checks.cfg
	python3 $<
	#find ./checks -name '*.sby' | xargs gsed -i 's/prep/synth/g'

complete: riscv-formal
	sby -f complete.sby
	python3 ./riscv-formal/cores/picorv32/disasm.py complete/engine_0/trace.vcd

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal

clean:
	rm -rf {checks,complete,riscv-formal}

.PHONY: complete check clean
