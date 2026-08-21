`timescale 1 ns / 1 ps
`default_nettype none
// The whole chip: core, both memories, the machine timer, a transmit-only UART
// and five pins. `make soc-timing` measures this. `make fit` measures the core
// on its own with the memories outside it, so its cell count is a different
// design's and the two do not compare -- and rtl/uart.v is outside that top
// entirely, so it moves this design's cell count and not that one's.
//
// The ROM says when it has nothing at the address it is answering, and decode
// takes that as an instruction access fault. The data RAM says which addresses
// answer an ATOMIC, and decode faults the eleven encodings elsewhere -- an
// atomic's address is a register value with no adder in front of it, so that
// question fits in the cycle the pc is chosen. A plain load or store is still
// turned down by neither: out of range both read zero and drop the write, and
// the decode-side test that would fault them has to wait on the top of a 32-bit
// sum, which was measured at four logic levels in the fetch loop and declined.
//
// The ROM is initialised from the bitstream and the SPRAM cannot be, so a
// program's `.data` is not there at power-on and every `.S` program in test/asm
// has one. Running a real program needs a stub in `.text` that copies the data
// image out of ROM, or a boot path that reads it from external SPI flash.
module littlesoc (
  input  logic clk,
  input  logic btn_n,
  // Give yosys a design whose output nothing can see and it deletes the lot. An
  // earlier version of this module did exactly that and reported 4 logic cells.
  // These two only watch signals the core already drives, so they add no
  // registers to the memory map and change nothing about how it runs.
  output logic ledr_n,
  output logic ledg_n,
  // The one pin a program can say anything through. It goes to the FTDI bridge
  // on the board, so a host sees it as a serial port; soc/littlesoc.pcf carries
  // the assignment.
  output logic uart_tx
);
  // The FPGA comes out of configuration with no reset of its own, so this makes
  // one. Keep `reset` registered. The button is asynchronous and needs the two
  // flops anyway, but the register also keeps the pin out of the logic behind
  // it: `make soc-timing`'s first run found the longest path in the whole design
  // starting at `btn_n`.
  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic [1:0] btn_sync  = 2'b0;
  logic       reset     = 1'b1;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    btn_sync <= {btn_sync[0], btn_n};
    reset    <= !por_done || !btn_sync[1];
  end

  logic        trap;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [31:0] imem_mem_rdata, dmem_mem_rdata, timer_mem_rdata, uart_mem_rdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren, fetch_stall, irq_timer, imem_fault, mem_reservable;
  logic        atomic_supported, mem_lock, bus_request;
  logic [31:0] atomic_addr;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic [31:0] imem_data, imem_data2;

  // The text window's size, the one number the core's copy of the map cannot
  // take from a memory's own default: the harness simulates a larger ROM than
  // this part has. It is the same 2048 the `imemory` below is given, and
  // test/memmap_test.sh compares the two.
  littlecpu #(.LS_TEXT_WORDS(2048)) riscv (
    .clk(clk),
    .reset(reset),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault),
    .mem_reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    // One bus master, so nothing ever takes the bus away and nothing else
    // writes memory, and `mem_lock` has no arbiter to tell. The two inputs fold
    // away here; the output does not, and its wire is why this SoC's mapped
    // netlist is not the one it was before the surface landed.
    .bus_wait(1'b0),
    .snoop_write(1'b0),
    .snoop_addr(32'b0),
    .mem_lock(mem_lock),
    .bus_request(bus_request),
    .irq_timer(irq_timer),
    .trap(trap)
  );

  // `imem_addr` and `imem_addr2` are left unconnected on purpose. The ROM needs
  // its address a cycle early, so it takes `imem_addr_next` instead. The core
  // keeps both ports because the formal checks read them.
  //
  // 2048 words = 8 KB = 16 block RAMs of the part's 30. rtl/regfile.v takes 4.
  imemory #(
    .ROM_WORDS(2048),
    .INIT_EVEN("soc/rom_even.hex"),
    .INIT_ODD("soc/rom_odd.hex")
  ) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(imem_mem_rdata),
    .fetch_stall(fetch_stall),
    .imem_fault(imem_fault)
  );

  // Base and size are rtl/memory.v's own defaults -- 64 KB at 0x0001_0000, two
  // `SB_SPRAM256KA` of the part's four -- and test/testbench.v takes the same
  // ones rather than restating them, so the simulated machine and this one
  // cannot describe different memories. `make soc-timing`'s SPRAM census is the
  // second half of that: the count only comes out at 2 for this size.
  memory dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_mem_rdata),
    .reservable(mem_reservable),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported)
  );

  // The machine timer, at rtl/timer.v's default base, which is the first word
  // past the data RAM above. This is the only interrupt source on this
  // platform, so `mip.MSIP` and `mip.MEIP` stay read-only zero in rtl/csrs.v.
  timer mtimer (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(timer_mem_rdata),
    .mtip(irq_timer)
  );

  // The transmit-only UART, at rtl/uart.v's default base, which is the first
  // word past the timer's four. Its read-back is one bit wide, so the OR below
  // gains a fourth input on bit 0 and nothing anywhere else.
  uart tty (
    .clk(clk),
    .reset(reset),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(uart_mem_rdata),
    .tx(uart_tx)
  );

  // An OR instead of a mux, which is one less level of logic here. The four
  // ranges do not overlap. On a store the RAM holds its last read value and the
  // others give zero, so this can never mix two real values together.
  assign mem_rdata = imem_mem_rdata | dmem_mem_rdata | timer_mem_rdata | uart_mem_rdata;

  logic store_bit, trap_seen;
  always_ff @(posedge clk) begin
    if (reset) begin
      store_bit <= 1'b0;
      trap_seen <= 1'b0;
    end else begin
      if (|mem_wstrb) store_bit <= mem_wdata[0];
      if (trap)       trap_seen <= 1'b1;
    end
  end
  assign ledg_n = !store_bit;
  assign ledr_n = !trap_seen;
endmodule // littlesoc
