`timescale 1 ns / 1 ps
`default_nettype none
// Hazard3 in the same harness as soc/compare/bench_littlecpu.v and
// soc/compare/bench_vexriscv.v: same ROM depth, the same rtl/memory.v at the same base,
// the same three pads, the same program image, the same part and the same seeds.
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

  // hwdata is NOT valid in a write's address phase -- AHB5 presents it one cycle later,
  // in the data phase, overlapping the NEXT transfer's own address phase.
  logic        wr_pending_q;
  logic [31:0] wr_addr_q;
  logic [3:0]  wr_strb_q;
  assign hready = !wr_pending_q;

  logic [3:0] size_mask;
  // A continuous assign, not a `case` in an `always_comb`: iverilog will not fully
  // evaluate a constant part-select (`hsize[1:0]`) used as a case expression inside a
  // process, and this repo allowlists that "sorry" only for rtl/writeback.v's struct
  // reads.
  assign size_mask = hsize[1:0] == 2'b00 ? 4'b0001 :
                      hsize[1:0] == 2'b01 ? 4'b0011 : 4'b1111;
  logic want_write;
  assign want_write = htrans[1] && hwrite;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_pending_q <= 1'b0;
    end else if (hready) begin
      wr_pending_q <= want_write;
      wr_addr_q    <= haddr;
      wr_strb_q    <= want_write ? (size_mask << haddr[1:0]) : 4'b0000;
    end else begin
      wr_pending_q <= 1'b0;
    end
  end

  logic [31:0] mem_addr_mux;
  logic [3:0]  mem_wstrb_mux;
  assign mem_addr_mux  = wr_pending_q ? wr_addr_q : haddr;
  assign mem_wstrb_mux = wr_pending_q ? wr_strb_q : 4'b0000;

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

  logic ram_read_q;
  always_ff @(posedge clk) ram_read_q <= !is_rom_next && !wr_pending_q;

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
