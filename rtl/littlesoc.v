`timescale 1 ns / 1 ps
`default_nettype none
// The SoC: the core, the two memories it talks to, a power-on reset, and two
// LEDs. It PLACES on an up5k in an sg48 package, which is what makes the first
// real timing measurement this project has ever had possible (ADR-0054).
//
// `make fit` does NOT synthesise this module and must not start: ADR-0038
// decision 1 measures `littlecpu` with its memories external, because that is
// the thing the core's own work changes. `make soc-timing` is this module's
// measurement, and the two numbers are separate on purpose -- the SoC's
// includes the ROM's depth mux, the RAM's range decode and the LED tap, none of
// which are the core.
//
// ---- the memory map --------------------------------------------------------
//
// ADR-0008's, unchanged, and Harvard: the instruction bus indexes ROM from 0
// and the data bus indexes RAM from RAM_BASE, and they are not two windows on
// one address space. Everything out of range on either bus reads as zero.
//
// ---- what this cannot do yet, stated rather than discovered -----------------
//
// The ROM is initialised from the bitstream; the RAM CANNOT BE. `SB_SPRAM256KA`
// has no INIT capability at all, so a program's `.data` is not there at
// power-on. Every `.S` program in test/asm has a `.data` section, so a
// bitstream built from one would run against zeroed data. Fixing that means a
// copy stub in `.text` (a crt0 that copies an image linked into ROM) or the
// SPI-flash boot path ADR-0044 describes; both are out of scope here and
// ADR-0054 records the gap rather than leaving it to be found on hardware.
module littlesoc (
  // 12 MHz on an iCEBreaker (ADR-0038 declares Fmax at 12 MHz as an INTENT;
  // `make soc-timing` measures what the design actually closes at, and
  // reconciling the two is a decision rather than a consequence).
  input  logic clk,
  // Active-low user button. Held down, it resets the core.
  input  logic btn_n,
  // SPIKE: the steal-mux select, brought out to a pad so it is genuinely free
  // and yosys cannot fold it away. See rtl/imemory.v's `steal` port for why the
  // select is isolated from the arbiter's own logic cone.
  input  logic steal_pin,
  // Active-low LEDs. They exist because a design with no observable output is
  // a design yosys is entitled to delete -- which is exactly what happened to
  // the previous version of this module, and it reported 4 logic cells and 0%
  // utilisation while doing it (ADR-0038's consequences). Both are passive taps
  // on signals the core already drives; neither adds an MMIO region, so the
  // memory map and the bus are unchanged (ADR-0044's `causal_io_ch0` /
  // `bus_dmem_io_*` questions stay exactly where they were).
  output logic ledr_n,
  output logic ledg_n
);
  // ---- power-on reset, and the button ---------------------------------------
  // The FPGA comes out of configuration with no reset pulse of its own, and the
  // core needs one: rtl/decoder.v only forces pc to 0 while `reset` is high.
  // 16 cycles, counted once and then never again -- the counter saturates.
  //
  // `reset` IS REGISTERED, and that is a timing decision as much as a
  // metastability one. `btn_n` arrives asynchronously, so the two-flop
  // synchroniser below is the ordinary correctness requirement -- but it also
  // keeps the pad out of the fabric's timing paths, and `make soc-timing`'s
  // first run measured a critical path that STARTED at the `btn_n` pad and ran
  // through reset's fanout into decode. 1.105 ns of pad delay plus four routing
  // hops, on the longest path in the design, bought by a wire.
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

  // ---- the core ------------------------------------------------------------
  logic        trap;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0]  mem_wstrb;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic [31:0] imem_data, imem_data2;
  // SPIKE: the arbiter. `fetch_stall` is the registered form of the same
  // predicate the steal mux selects on -- a data access that landed in the
  // text region took the ROM's read port last cycle.
  logic        mem_ren, fetch_stall, text_access, dmem_rdata_sel;
  logic [31:0] dmem_rdata, text_rdata;
  assign text_access = (mem_ren || |mem_wstrb) && mem_addr < 32'h0000_2000;
  always_ff @(posedge clk) begin
    fetch_stall    <= !reset && text_access;
    dmem_rdata_sel <= !reset && text_access;
  end
  assign mem_rdata = dmem_rdata_sel ? text_rdata : dmem_rdata;

  littlecpu riscv (
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
    .mem_rdata(mem_rdata),
    .mem_ren(mem_ren),
    .fetch_stall(fetch_stall),
    .trap(trap)
  );

  // ---- instruction ROM -----------------------------------------------------
  // ONE instance, not two. The previous version of this module instantiated
  // `imemory` TWICE to get ADR-0003's second fetch port, which doubles storage
  // to add a port and was most of the 43 KB overage ADR-0044 measured; the
  // banking inside rtl/imemory.v replaces it.
  //
  // `imem_addr`/`imem_addr2` are left unread here, deliberately: the ROM is
  // synchronous, so it is addressed off `imem_addr_next` a cycle early
  // (ADR-0054), and `imem_addr` is the same value one edge later. The two ports
  // stay on the core because that is the bus the riscv-formal ladder speaks.
  //
  // 2048 words = 8 KB = 16 EBRs of the part's 30, with rtl/regfile.v taking 4.
  // See ADR-0054 for what that ceiling holds and what it does not.
  imemory #(
    .ROM_WORDS(2048),
    .INIT_EVEN("soc/rom_even.hex"),
    .INIT_ODD("soc/rom_odd.hex")
  ) imem (
    .clk(clk),
    .imem_addr_next(imem_addr_next),
    .imem_data(imem_data),
    .imem_data2(imem_data2),
    .steal(steal_pin),
    .mem_addr(mem_addr),
    .mem_wstrb(text_access ? mem_wstrb : 4'b0),
    .mem_wdata(mem_wdata),
    .text_rdata(text_rdata)
  );

  // ---- data RAM ------------------------------------------------------------
  // 16384 words = 64 KB = two `SB_SPRAM256KA` of the part's four.
  memory #(.BASE(32'h0001_0000), .RAM_WORDS(16384)) dmem (
    .clk(clk),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dmem_rdata)
  );

  // ---- the two observable bits --------------------------------------------
  // A tap on the store bus and a sticky trap flag. Between them they keep every
  // stage of the core reachable from an output pin, which is what stops
  // synthesis from deleting it: a store's data comes from the register file,
  // which comes from writeback, which comes from loads and the executor, which
  // come from decode, which comes from the fetch window.
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
