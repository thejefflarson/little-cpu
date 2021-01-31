test: testbench.vvp
	vvp -N $<

rvfi_macros.vh: formal/riscv-formal/checks/rvfi_macros.py
	python $^ > $@

testbench.vvp: rvfi_macros.vh testbench.v riscv.v monitor.v decoder.v alu.v
	iverilog -DRISCV_FORMAL -DRISCV_FORMAL_COMPRESSED -DRISCV_FORMAL_ALIGNED_MEM -DRISCV_FORMAL_NRET=1 -DRISCV_FORMAL_XLEN=32 -DRISCV_FORMAL_ILEN=32 -g2012 -o $@ $^
	chmod +x $@

pll.v: timing
	icepll -m -f $@ -i 12 -o $(shell cat $^)

riscv.json: riscv.v decoder.v alu.v
	yosys -p 'read_verilog -sv $^; synth_ice40 -top riscv -json $@'

riscv.asc: riscv.json riscv.pcf
	nextpnr-ice40 --lp8k --json riscv.json --pcf riscv.pcf --asc riscv.asc --pcf-allow-unconstrained --package ct256 --opt-timing

timing: riscv.asc
	icetime -d lp8k $^ | egrep -oi '\(\d+' | egrep -o '\d+' > $@

clean:
	rm -f riscv.json
	rm -f riscv.asc
	rm -f timing
	rm -f pll.v
	rm -f testbench.vvp

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal formal/riscv-formal
