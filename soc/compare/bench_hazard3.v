`timescale 1 ns / 1 ps
`default_nettype none
// Hazard3 in the same harness as soc/compare/bench_littlecpu.v and
// soc/compare/bench_vexriscv.v: same ROM depth, the same rtl/memory.v at the
// same base, the same three pads, the same program image, the same part and
// the same seeds.
//
// The core is not vendored into rtl/ or copied anywhere in this tree. It is
// read straight out of the SHA-pinned clone soc/compare/hazard3_pin.mk
// materialises, the way soc/compare/bench_vexriscv.v reads VexRiscv out of
// the riscv-formal clone, so nothing here can drift from the pin.
//
// **This is fpga_icebreaker.v's configuration, not the RP2350's.** Every
// parameter below is copied verbatim from that project's
// example_soc/fpga/fpga_icebreaker.v: EXTENSION_C=0 (no compressed
// instructions -- the ISA this core has to share with it is RV32IMA, not
// RV32IMAC), MULDIV_UNROLL=1 (one bit of division per cycle, and of
// multiplication too since MUL_FAST=0), no bitmanip, no branch predictor, no
// U-mode, no PMP, no counter CSRs, and the same 12 MHz `CLK_MHZ` this board
// targets. `docs/adr/` records the deviations this harness makes from
// fpga_icebreaker.v's own integration -- none of them touch this list.
//
// **DEBUG_SUPPORT is left at its default of 0.** fpga_icebreaker.v does not
// set it; its own example_soc.v hardcodes 1 for the JTAG debug module this
// harness does not build (no scan chain, no DTM, the same limitation
// soc/compare/bench_littlecpu.v and soc/compare/bench_vexriscv.v already
// have). RESET_VECTOR and MTVEC_INIT are this harness's own memory map (text
// at 0, matching the other two benches) rather than example_soc.v's
// bootloader offset of 0x40, which exists to leave room for a JTAG-loaded
// program this harness has no debugger to load.
//
// **CSR_M_MANDATORY and CSR_M_TRAP stay on** (example_soc.v's own values,
// not fpga_icebreaker.v's list): with them off the core cannot execute at
// all -- `misa` and the trap CSRs are the bare minimum any RISC-V CPU with
// CSRs needs. CSR_COUNTER stays off exactly as fpga_icebreaker.v sets it,
// so this core has no `mcycle`/`minstret` and cannot self-time a benchmark
// the way the other two cores in this harness can.
//
// ---- the bus ----------------------------------------------------------
//
// hazard3_cpu_1port arbitrates fetch and load/store down to ONE AHB5 master
// port, unlike VexRiscv's separate iBus/dBus in this harness's other bench.
// The adapter below answers a read at zero wait states -- a read's address
// phase in cycle N is the read rtl/memory.v (reused unmodified for the RAM
// half) registers into its data phase in cycle N+1. A WRITE is not zero
// wait states: AHB5 presents `hwdata` one cycle after the write's own
// address phase, so `hready` is held low for that one cycle while the write
// is serviced against the captured address -- see the fuller comment beside
// `wr_pending_q` below for why a single-ported memory needs this.
// AHB has no separate read/write bus, so the ROM and the RAM answer the same
// haddr, distinguished purely by which side of ROM_WORDS*4 it falls on.
// hwdata is not shifted to byte 0 for a narrow store -- hazard3_core.v
// replicates it across all four lanes (MEMOP_SB/MEMOP_SH), the same
// replication soc/compare/bench_vexriscv.v's own comment names for VexRiscv
// -- so the byte strobe alone, shifted by the low address bits, is what
// picks the right byte out of a lane that already holds it everywhere.
//
// This harness's memory map keeps text and data in disjoint ranges (ROM at
// 0, RAM at rtl/memory.v's own default BASE), so a RAM-range read response is
// necessarily a data load and never an instruction fetch, even though both
// travel the one AHB port -- that is what lets led1_n below single out loads
// the way the other two benches' own dedicated data buses do for free.
module bench_hazard3 #(
  parameter integer ROM_WORDS = 1024,
  parameter integer RAM_WORDS = 512,
  parameter INIT_ROM = "soc/compare/rom_flat.hex"
) (
  input  logic clk,
  output logic led0_n,
  output logic led1_n
);
  localparam int ROM_BITS = $clog2(ROM_WORDS);
  localparam bit [31:0] ROM_BYTES = ROM_WORDS * 4;

  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic       rst_n     = 1'b0;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    rst_n <= por_done;
  end

  logic [31:0] haddr, hwdata, hrdata;
  logic        hwrite;
  logic [1:0]  htrans;
  logic [2:0]  hsize, hburst;
  logic [3:0]  hprot;
  logic        hmastlock, hexcl;
  logic [7:0]  hmaster;
  logic        hready;

  logic pwrup_req, unblock_out;

  hazard3_cpu_1port #(
    .RESET_VECTOR         (32'h0000_0000),
    .MTVEC_INIT           (32'h0000_0000),
    .CSR_M_MANDATORY      (1),
    .CSR_M_TRAP           (1),
    .NUM_IRQS             (1),
    .EXTENSION_A          (1),
    .EXTENSION_C          (0),
    .EXTENSION_M          (1),
    .EXTENSION_ZBA        (0),
    .EXTENSION_ZBB        (0),
    .EXTENSION_ZBC        (0),
    .EXTENSION_ZBS        (0),
    .EXTENSION_ZBKB       (0),
    .EXTENSION_ZIFENCEI   (0),
    .EXTENSION_XH3BEXTM   (0),
    .EXTENSION_XH3PMPM    (0),
    .EXTENSION_XH3POWER   (0),
    .CSR_COUNTER          (0),
    .U_MODE               (0),
    .PMP_REGIONS          (0),
    .BREAKPOINT_TRIGGERS  (0),
    .IRQ_PRIORITY_BITS    (0),
    .REDUCED_BYPASS       (0),
    .MULDIV_UNROLL        (1),
    .MUL_FAST             (0),
    .MUL_FASTER           (0),
    .MULH_FAST            (0),
    .FAST_BRANCHCMP       (1),
    .BRANCH_PREDICTOR     (0)
  ) core (
    .clk           (clk),
    .clk_always_on (clk),
    .rst_n         (rst_n),

    .pwrup_req   (pwrup_req),
    .pwrup_ack   (pwrup_req),
    .clk_en      (),
    .unblock_out (unblock_out),
    .unblock_in  (unblock_out),

    .haddr     (haddr),
    .hwrite    (hwrite),
    .htrans    (htrans),
    .hsize     (hsize),
    .hburst    (hburst),
    .hprot     (hprot),
    .hmastlock (hmastlock),
    .hmaster   (hmaster),
    .hexcl     (hexcl),
    .hready    (hready),
    .hresp     (1'b0),
    .hexokay   (1'b1),
    .hwdata    (hwdata),
    .hrdata    (hrdata),

    .fence_i_vld (),
    .fence_d_vld (),
    .fence_rdy   (1'b1),

    .dbg_req_halt          (1'b0),
    .dbg_req_halt_on_reset (1'b0),
    .dbg_req_resume        (1'b0),
    .dbg_halted (),
    .dbg_running (),
    .dbg_data0_rdata (32'b0),
    .dbg_data0_wdata (),
    .dbg_data0_wen (),
    .dbg_instr_data     (32'b0),
    .dbg_instr_data_vld (1'b0),
    .dbg_instr_data_rdy (),
    .dbg_instr_caught_exception (),
    .dbg_instr_caught_ebreak    (),

    .dbg_sbus_addr  (32'b0),
    .dbg_sbus_write (1'b0),
    .dbg_sbus_size  (2'b0),
    .dbg_sbus_vld   (1'b0),
    .dbg_sbus_rdy   (),
    .dbg_sbus_err   (),
    .dbg_sbus_wdata (32'b0),
    .dbg_sbus_rdata (),

    .mhartid_val (32'b0),
    .eco_version (4'b0),

    .irq      (1'b0),
    .soft_irq (1'b0),
    .timer_irq(1'b0)
  );

  // hwdata is NOT valid in a write's address phase -- AHB5 presents it one
  // cycle later, in the data phase, overlapping the NEXT transfer's own
  // address phase. rtl/memory.v takes one address for both a read and a
  // write on the SAME cycle, so a write cannot be performed until its data
  // phase, and that cycle's one memory port cannot also serve a new
  // transfer's address phase. So this slave is NOT zero-wait-state: it holds
  // `hready` low for exactly one cycle after a write's address phase,
  // finishing the write with the now-valid `hwdata` against the CAPTURED
  // address, and only then accepts what the core presents next. A plain
  // synchronous SRAM slave with one port needs exactly this for a write
  // immediately followed by a read of the same word -- `sw`/`lw` back to
  // back, which soc/compare/bench.S does on every iteration -- and running
  // this at true zero wait states once produced silently wrong store data,
  // read back and folded into the program's own accumulator until it went X.
  logic        wr_pending_q;
  logic [31:0] wr_addr_q;
  logic [3:0]  wr_strb_q;
  logic        hready_int;
  assign hready_int = !wr_pending_q;
  assign hready = hready_int;

  logic [3:0] size_mask;
  // A continuous assign, not a `case` in an `always_comb`: iverilog will not
  // fully evaluate a constant part-select (`hsize[1:0]`) used as a case
  // expression inside a process, and this repo allowlists that "sorry" only
  // for rtl/writeback.v's struct reads.
  assign size_mask = hsize[1:0] == 2'b00 ? 4'b0001 :
                      hsize[1:0] == 2'b01 ? 4'b0011 : 4'b1111;
  // htrans[1] is the bit that separates {NONSEQ, SEQ} from {IDLE, BUSY} --
  // hazard3_cpu_1port.v's own `bus_hold_aph` reads it for the same purpose --
  // so a stale hwrite on an idle cycle cannot raise a strobe here.
  logic want_write;
  assign want_write = htrans[1] && hwrite;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_pending_q <= 1'b0;
    end else if (hready_int) begin
      // Accepting a new address phase this cycle (or an idle one): latch it
      // in case it turns out to be a write, which is serviced next cycle.
      wr_pending_q <= want_write;
      wr_addr_q    <= haddr;
      wr_strb_q    <= want_write ? (size_mask << haddr[1:0]) : 4'b0000;
    end else begin
      // The write latched last cycle is serviced THIS cycle, below; nothing
      // new was accepted, so there is nothing left pending after it.
      wr_pending_q <= 1'b0;
    end
  end

  // The one memory port this cycle: the captured write if one is pending,
  // otherwise whatever address is live on the bus (a read, or an address
  // phase that has not yet resolved into anything). ROM/RAM range is decided
  // off the SAME muxed address, since a write is never serviced except
  // against the RAM range in this harness's map.
  logic [31:0] mem_addr_mux;
  logic [3:0]  mem_wstrb_mux;
  assign mem_addr_mux  = wr_pending_q ? wr_addr_q : haddr;
  assign mem_wstrb_mux = wr_pending_q ? wr_strb_q : 4'b0000;

  // A transfer's address phase and a read's data phase are one cycle apart,
  // so which side of ROM_BYTES the address this cycle's memory port serves
  // falls on has to be latched for the cycle hrdata actually carries the
  // answer.
  logic is_rom_next, is_rom_q;
  assign is_rom_next = mem_addr_mux < ROM_BYTES;
  always_ff @(posedge clk) is_rom_q <= is_rom_next;

  logic [ROM_BITS-1:0] rom_index;
  logic [31:0]         rom_rdata;
  assign rom_index = mem_addr_mux[ROM_BITS+1:2];

  logic [31:0] rom[0:ROM_WORDS-1];
  generate if (INIT_ROM != "") begin : l_rom_init
    initial $readmemh(INIT_ROM, rom);
  end endgenerate

  // Unconditional every cycle, the same shape soc/compare/bench_vexriscv.v
  // uses for its own ROM: block RAM has no write port to conflict with here,
  // so there is no no-change rule to observe the way rtl/memory.v has one.
  always_ff @(posedge clk) rom_rdata <= rom[rom_index];

  logic [31:0] ram_rdata;
  memory #(.RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(mem_addr_mux),
    .mem_wdata(hwdata),
    .mem_wstrb(mem_wstrb_mux),
    .mem_rdata(ram_rdata)
  );

  assign hrdata = is_rom_q ? rom_rdata : ram_rdata;

  // A RAM-range read (not a ROM fetch, not a write) is a data load, and
  // rtl/memory.v answers it exactly one cycle later -- the same cycle this
  // becomes true.
  logic ram_read_q;
  always_ff @(posedge clk) ram_read_q <= !is_rom_next && !(|mem_wstrb_mux);

  logic store_bit, load_bit;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      store_bit <= 1'b0;
      load_bit  <= 1'b0;
    end else begin
      if (wr_pending_q) store_bit <= hwdata[0];
      if (ram_read_q)  load_bit  <= hrdata[0];
    end
  end
  assign led0_n = !store_bit;
  assign led1_n = !load_bit;
endmodule
