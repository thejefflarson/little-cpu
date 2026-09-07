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

  // I-port: fetch only, always ROM, answered exactly one cycle later -- `i_hready` above
  // is a tied constant, so this array never has to be asked to wait.
  logic [ROM_BITS-1:0] rom_index;
  logic [31:0]         rom_rdata;
  assign rom_index = i_haddr[ROM_BITS+1:2];

  logic [31:0] rom[0:ROM_WORDS-1];
  generate if (INIT_ROM != "") begin : l_rom_init
    initial $readmemh(INIT_ROM, rom);
  end endgenerate

  // Unconditional every cycle, the same shape soc/compare/bench_vexriscv.v uses for its
  // own ROM: block RAM has no write port to conflict with here, so there is no no-change
  // rule to observe the way rtl/memory.v has one.
  always_ff @(posedge clk) rom_rdata <= rom[rom_index];
  assign i_hrdata = rom_rdata;

  // D-port: every load and store, against rtl/memory.v.
  logic        wr_pending_q;
  logic [31:0] wr_addr_q;
  logic [3:0]  wr_strb_q;
  assign d_hready = !wr_pending_q;

  logic [3:0] size_mask;
  assign size_mask = d_hsize[1:0] == 2'b00 ? 4'b0001 :
                      d_hsize[1:0] == 2'b01 ? 4'b0011 : 4'b1111;
  logic want_write;
  assign want_write = d_htrans[1] && d_hwrite;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_pending_q <= 1'b0;
    end else if (d_hready) begin
      wr_pending_q <= want_write;
      wr_addr_q    <= d_haddr;
      wr_strb_q    <= want_write ? (size_mask << d_haddr[1:0]) : 4'b0000;
    end else begin
      wr_pending_q <= 1'b0;
    end
  end

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
