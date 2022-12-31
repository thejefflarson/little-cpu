test: testbench.vvp
	vvp -N $<

rvfi_macros.vh: formal/riscv-formal/checks/rvfi_macros.py
	python $^ > $@

testbench.vvp: rtl/structs.v rtl/handshake.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/skidbuffer.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v test/monitor.v
	iverilog -DRISCV_FORMAL -DRISCV_FORMAL_COMPRESSED -DRISCV_FORMAL_ALIGNED_MEM -DRISCV_FORMAL_NRET=1 -DRISCV_FORMAL_XLEN=32 -DRISCV_FORMAL_ILEN=32 -g2012 -o $@ $^
	chmod +x $@

testbench: test/driver.cc rtl/structs.v rtl/handshake.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/skidbuffer.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v test/monitor.v
	verilator -top-module testbench -Wall -Wno-declfilename --trace --cc --exe --build -o $@ $^

# Not sure why this doesn't work :(
test/rtl.cc: rtl/structs.v rtl/handshake.v rtl/accessor.v rtl/decoder.v rtl/executor.v rtl/fetcher.v rtl/regfile.v rtl/skidbuffer.v rtl/writeback.v rtl/littlecpu.v rvfi_macros.vh test/testbench.v
	yosys -p 'read_verilog -sv $^; hierarchy -top testbench; write_cxxrtl $@'

pll.v: timing
	icepll -m -f $@ -i 12 -o $(shell cat $^)

riscv.json: rtl/littlecpu.v  rtl/fetcher.v rtl/rtl/decoder.v rtl/
	yosys -p 'read_verilog -sv $^; synth_ice40 -dsp -top littlecpu -json $@'

riscv.asc: riscv.json riscv.pcf
	nextpnr-ice40 --up5k --json riscv.json --pcf riscv.pcf --asc riscv.asc --pcf-allow-unconstrained --package ct256 --opt-timing

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
