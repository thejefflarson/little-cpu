rvfi_macros.vh: formal/riscv-formal/checks/rvfi_macros.py
	python $^ > $@

waves.vcd: sim
	./sim

sim: test/cxxrtl.cc test/rtl.cc
	clang++ -g -std=c++14 -I $$(yosys-config --datdir)/include $< -o $@

test/rtl.cc: rtl/structs.v rtl/handshake.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v
	yosys -p 'read_verilog -sv $^; hierarchy -top testbench; write_cxxrtl $@'

test/monitor.v: formal/riscv-formal/monitor/generate.py
	python ./$^ -i rv32imc -c 1 -a -p monitor > $@

pll.v: timing
	icepll -m -f $@ -i 12 -o $(shell cat $^)

riscv.json:  rtl/structs.v rtl/handshake.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/skidbuffer.v rtl/writeback.v rtl/littlecpu.v rtl/littlesoc.v rtl/imemory.v rtl/memory.v
	yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlesoc -json $@'

riscv.asc: riscv.json riscv.pcf
	nextpnr-ice40 --up5k --json riscv.json --pcf riscv.pcf --asc riscv.asc --pcf-allow-unconstrained --opt-timing

timing: riscv.asc
	icetime -d up5k $^ | egrep -oi '\(\d+' | egrep -o '\d+' > $@

clean:
	rm -f riscv.json
	rm -f riscv.asc
	rm -f timing
	rm -f pll.v
	rm -f waves.vcd
	rm -f sim

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal formal/riscv-formal
