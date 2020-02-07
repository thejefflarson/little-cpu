test: testbench.vvp
	vvp -N $<

testbench.vvp: testbench.v riscv.v monitor.v
	iverilog -DRISCV_FORMAL -g2012 -o $@ $^
	chmod +x $@

pll.v: timing
	icepll -m -f $@ -i 12 -o $(shell cat $^)

riscv.json: riscv.v
	yosys -p 'read_verilog -sv $^; synth_ice40 -top riscv -json $@'

riscv.asc: riscv.json riscv.pcf
	nextpnr-ice40 --up5k --json $< --pcf riscv.pcf --asc $@ --package sg48 --pcf-allow-unconstrained

timing: riscv.asc
	icetime -d up5k $^ | egrep -oi '\(\d+' | egrep -o '\d+' > $@

clean:
	rm -f riscv.json
	rm -f riscv.asc
	rm -f timing
	rm -f pll.v
	rm -f testbench.vvp

riscv-formal:
	git clone https://github.com/SymbioticEDA/riscv-formal
