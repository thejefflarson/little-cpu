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
// CSRs needs -- and CSR_COUNTER, the one this ADR's ticket is actually
// about, stays off exactly as fpga_icebreaker.v sets it, which is why this
// core cannot self-time a benchmark.
//
// ---- the bus ----------------------------------------------------------
//
// hazard3_cpu_1port arbitrates fetch and load/store down to ONE AHB5 master
// port, unlike VexRiscv's separate iBus/dBus in this harness's other bench.
// The adapter below is a zero-wait-state AHB5 slave: hready is tied high, so
// the data phase for the transfer addressed in cycle N is the read this
// module registers in cycle N+1 -- the same latency rtl/memory.v already
// has, which is why that module is reused here unmodified for the RAM half.
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
    .hready    (1'b1),
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

  // A transfer's address phase and a read's data phase are one cycle apart at
  // hready=1, so which side of ROM_BYTES the CURRENT haddr falls on has to be
  // latched for the cycle hrdata actually carries that transfer's answer.
  logic is_rom_next, is_rom_q;
  assign is_rom_next = haddr < ROM_BYTES;
  always_ff @(posedge clk) is_rom_q <= is_rom_next;

  logic [ROM_BITS-1:0] rom_index;
  logic [31:0]         rom_rdata;
  assign rom_index = haddr[ROM_BITS+1:2];

  logic [31:0] rom[0:ROM_WORDS-1];
  generate if (INIT_ROM != "") begin : l_rom_init
    initial $readmemh(INIT_ROM, rom);
  end endgenerate

  // Unconditional every cycle, the same shape soc/compare/bench_vexriscv.v
  // uses for its own ROM: block RAM has no write port to conflict with here,
  // so there is no no-change rule to observe the way rtl/memory.v has one.
  always_ff @(posedge clk) rom_rdata <= rom[rom_index];

  // A byte or halfword store arrives on hwdata replicated across all four
  // lanes (hazard3_core.v's MEMOP_SB/MEMOP_SH), so the strobe alone -- not a
  // shifted copy of hwdata -- is what rtl/memory.v needs.
  // A continuous assign, not a `case` in an `always_comb`: iverilog will not
  // fully evaluate a constant part-select (`hsize[1:0]`) used as a case
  // expression inside a process, and this repo allowlists that "sorry" only
  // for rtl/writeback.v's struct reads.
  logic [3:0] size_mask, mem_wstrb;
  assign size_mask = hsize[1:0] == 2'b00 ? 4'b0001 :
                      hsize[1:0] == 2'b01 ? 4'b0011 : 4'b1111;
  // htrans[1] is the bit that separates {NONSEQ, SEQ} from {IDLE, BUSY} --
  // hazard3_cpu_1port.v's own `bus_hold_aph` reads it for the same purpose --
  // so a stale hwrite on an idle cycle cannot raise a strobe here.
  assign mem_wstrb = (htrans[1] && hwrite)
                    ? (size_mask << haddr[1:0]) : 4'b0000;

  logic [31:0] ram_rdata;
  memory #(.RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(haddr),
    .mem_wdata(hwdata),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(ram_rdata)
  );

  assign hrdata = is_rom_q ? rom_rdata : ram_rdata;

  // A RAM-range read (not a ROM fetch, not a write) is a data load, and
  // rtl/memory.v answers it exactly one cycle later -- the same cycle this
  // becomes true.
  logic ram_read_q;
  always_ff @(posedge clk) ram_read_q <= !is_rom_next && !(|mem_wstrb);

  logic store_bit, load_bit;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      store_bit <= 1'b0;
      load_bit  <= 1'b0;
    end else begin
      if (|mem_wstrb) store_bit <= hwdata[0];
      if (ram_read_q) load_bit  <= hrdata[0];
    end
  end
  assign led0_n = !store_bit;
  assign led1_n = !load_bit;
endmodule
