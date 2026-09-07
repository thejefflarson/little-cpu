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
// hazard3_cpu_2port gives fetch and load/store each their own AHB5 master
// port, matching the topology soc/compare/bench_littlecpu.v (dedicated
// imemory/memory) and soc/compare/bench_vexriscv.v (separate iBus/dBus)
// already have. This harness used to instantiate hazard3_cpu_1port instead,
// which arbitrates both onto ONE shared port -- the one core here forced
// through a single memory port while its two neighbours each got two. That
// forced sharing was what put a real AHB5 protocol cost (a write's data
// trails its own address by one cycle, so a single-ported RAM slave must
// hold the bus for it) onto every fetch that happened to follow a store,
// not only onto a genuinely conflicting load or store; measured, that cost
// 9.01% of Hazard3's own Dhrystone cycles and 1.98% of its CoreMark ones.
// Giving Hazard3 the same two-memory topology its two neighbours already
// have removes the forced sharing rather than trying to out-schedule it.
//
// The I-port fetches only, from its own `rom` array below. Nothing on this
// port ever writes, so `i_hready` is a tied-high constant with no logic
// behind it -- the same zero-wait-state promise
// soc/compare/bench_littlecpu.v's own dedicated imem port makes.
//
// The D-port carries every load and store, against `rtl/memory.v` (reused
// unmodified). Its write buffer is the pre-two-port mechanism, unchanged:
// AHB5 presents `d_hwdata` one cycle after a write's own address phase, so
// the address and strobe are captured here and drained the following cycle,
// with `d_hready` held low for exactly that one cycle if another D-port
// transaction lands on it -- see `wr_pending_q` below. What this buffer no
// longer has to decide is whether the transaction it might be holding back
// is a fetch: fetches are never on this port, so `d_hready` needs no
// dependence on `d_haddr` at all, and stays the bare flip-flop output it
// was before either of the other two cores in this harness had an AHB5
// write cost to arbitrate.
//
// The D-port has NO PATH TO ROM, the same choice soc/compare/bench_vexriscv.v
// already makes and CLAUDE.md already states as this harness's standing
// rule: a load from a ROM address reads back whatever rtl/memory.v's own
// out-of-range arm returns (zero). This is not a new gap opened for
// Hazard3 -- no program this harness runs needs the opposite. The harness
// pokes every program's initialised data and read-only data straight into
// the simulated RAM before the run rather than having the program copy it
// out of ROM at boot: soc/compare/dhry_start.S and
// soc/compare/coremark_start.S are test/crt0.S with that copy removed, and
// soc/compare/coremark.lds keeps CoreMark's own string tables in the poked
// RAM region for the identical reason. So the two-port split needs no
// second ROM array and no read-side ROM decode on the D-port.
// hwdata is not shifted to byte 0 for a narrow store -- hazard3_core.v
// replicates it across all four lanes (MEMOP_SB/MEMOP_SH), the same
// replication soc/compare/bench_vexriscv.v's own comment names for VexRiscv
// -- so the byte strobe alone, shifted by the low address bits, is what
// picks the right byte out of a lane that already holds it everywhere.
module bench_hazard3 #(
  parameter integer ROM_WORDS = 1024,
  parameter integer RAM_WORDS = 16384,
  parameter INIT_ROM = "soc/compare/rom_flat.hex"
) (
  input  logic clk,
  output logic led0_n,
  output logic led1_n
);
  localparam int ROM_BITS = $clog2(ROM_WORDS);

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

  logic [31:0] i_haddr, i_hwdata, i_hrdata;
  logic        i_hwrite;
  logic [1:0]  i_htrans;
  logic [2:0]  i_hsize, i_hburst;
  logic [3:0]  i_hprot;
  logic        i_hmastlock;
  logic [7:0]  i_hmaster;

  logic [31:0] d_haddr, d_hwdata, d_hrdata;
  logic        d_hwrite;
  logic [1:0]  d_htrans;
  logic [2:0]  d_hsize, d_hburst;
  logic [3:0]  d_hprot;
  logic        d_hmastlock, d_hexcl;
  logic [7:0]  d_hmaster;
  logic        d_hready;

  logic pwrup_req, unblock_out;

  hazard3_cpu_2port #(
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

    .i_haddr     (i_haddr),
    .i_hwrite    (i_hwrite),
    .i_htrans    (i_htrans),
    .i_hsize     (i_hsize),
    .i_hburst    (i_hburst),
    .i_hprot     (i_hprot),
    .i_hmastlock (i_hmastlock),
    .i_hmaster   (i_hmaster),
    .i_hready    (1'b1),
    .i_hresp     (1'b0),
    .i_hwdata    (i_hwdata),
    .i_hrdata    (i_hrdata),

    .d_haddr     (d_haddr),
    .d_hwrite    (d_hwrite),
    .d_htrans    (d_htrans),
    .d_hsize     (d_hsize),
    .d_hburst    (d_hburst),
    .d_hprot     (d_hprot),
    .d_hmastlock (d_hmastlock),
    .d_hmaster   (d_hmaster),
    .d_hexcl     (d_hexcl),
    .d_hready    (d_hready),
    .d_hresp     (1'b0),
    .d_hexokay   (1'b1),
    .d_hwdata    (d_hwdata),
    .d_hrdata    (d_hrdata),

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

  // I-port: fetch only, always ROM, answered exactly one cycle later --
  // `i_hready` above is a tied constant, so this array never has to be
  // asked to wait.
  logic [ROM_BITS-1:0] rom_index;
  logic [31:0]         rom_rdata;
  assign rom_index = i_haddr[ROM_BITS+1:2];

  logic [31:0] rom[0:ROM_WORDS-1];
  generate if (INIT_ROM != "") begin : l_rom_init
    initial $readmemh(INIT_ROM, rom);
  end endgenerate

  // Unconditional every cycle, the same shape soc/compare/bench_vexriscv.v
  // uses for its own ROM: block RAM has no write port to conflict with here,
  // so there is no no-change rule to observe the way rtl/memory.v has one.
  always_ff @(posedge clk) rom_rdata <= rom[rom_index];
  assign i_hrdata = rom_rdata;

  // D-port: every load and store, against rtl/memory.v.
  logic        wr_pending_q;
  logic [31:0] wr_addr_q;
  logic [3:0]  wr_strb_q;
  assign d_hready = !wr_pending_q;

  logic [3:0] size_mask;
  // A continuous assign, not a `case` in an `always_comb`: iverilog will not
  // fully evaluate a constant part-select (`d_hsize[1:0]`) used as a case
  // expression inside a process, and this repo allowlists that "sorry" only
  // for rtl/writeback.v's struct reads.
  assign size_mask = d_hsize[1:0] == 2'b00 ? 4'b0001 :
                      d_hsize[1:0] == 2'b01 ? 4'b0011 : 4'b1111;
  // d_htrans[1] is the bit that separates {NONSEQ, SEQ} from {IDLE, BUSY} --
  // hazard3_cpu_2port.v's own `bus_hold_aph` reads it for the same purpose --
  // so a stale d_hwrite on an idle cycle cannot raise a strobe here.
  logic want_write;
  assign want_write = d_htrans[1] && d_hwrite;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_pending_q <= 1'b0;
    end else if (d_hready) begin
      // Accepting a new address phase this cycle (or an idle one): latch it
      // in case it turns out to be a write, which is serviced next cycle.
      wr_pending_q <= want_write;
      wr_addr_q    <= d_haddr;
      wr_strb_q    <= want_write ? (size_mask << d_haddr[1:0]) : 4'b0000;
    end else begin
      // The write latched last cycle is serviced THIS cycle, below; nothing
      // new was accepted, so there is nothing left pending after it.
      wr_pending_q <= 1'b0;
    end
  end

  // The one RAM port this cycle: the captured write if one is pending,
  // otherwise whatever address is live on the D-port (a read, or an address
  // phase that has not yet resolved into anything).
  logic [31:0] dmem_addr_mux;
  logic [3:0]  dmem_wstrb_mux;
  assign dmem_addr_mux  = wr_pending_q ? wr_addr_q : d_haddr;
  assign dmem_wstrb_mux = wr_pending_q ? wr_strb_q : 4'b0000;

  memory #(.RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(dmem_addr_mux),
    .mem_wdata(d_hwdata),
    .mem_wstrb(dmem_wstrb_mux),
    .mem_rdata(d_hrdata)
  );

  // A D-port read (not a write, not a stalled address phase) is a data
  // load, and rtl/memory.v answers it exactly one cycle later -- the same
  // cycle this becomes true.
  logic ram_read_q;
  always_ff @(posedge clk) ram_read_q <= d_htrans[1] && !d_hwrite && d_hready;

  logic store_bit, load_bit;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      store_bit <= 1'b0;
      load_bit  <= 1'b0;
    end else begin
      if (wr_pending_q) store_bit <= d_hwdata[0];
      if (ram_read_q)  load_bit  <= d_hrdata[0];
    end
  end
  assign led0_n = !store_bit;
  assign led1_n = !load_bit;
endmodule
