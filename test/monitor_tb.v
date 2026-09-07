// Unit bench for the generated RVFI monitor itself -- not for the core.
`timescale 1ns / 1ps

module monitor_tb;
  localparam [31:0] MTVEC = 32'h0000_0100;  // where the trapping vectors redirect
  localparam [31:0] RAM   = 32'h0001_0000;  // RAM base in the test memory map
  localparam [31:0] UNMAPPED = 32'h0004_0000;  // no memory on this platform answers

  reg clock = 0;
  reg reset = 1;

  reg  [0:0]  rvfi_valid = 0;
  reg  [63:0] rvfi_order = 0;
  reg  [31:0] rvfi_insn = 0;
  reg  [0:0]  rvfi_trap = 0;
  reg  [0:0]  rvfi_halt = 0;
  reg  [0:0]  rvfi_intr = 0;
  reg  [1:0]  rvfi_mode = 2'b11;
  reg  [4:0]  rvfi_rs1_addr = 0;
  reg  [4:0]  rvfi_rs2_addr = 0;
  reg  [31:0] rvfi_rs1_rdata = 0;
  reg  [31:0] rvfi_rs2_rdata = 0;
  reg  [4:0]  rvfi_rd_addr = 0;
  reg  [31:0] rvfi_rd_wdata = 0;
  reg  [31:0] rvfi_pc_rdata = 0;
  reg  [31:0] rvfi_pc_wdata = 0;
  reg  [31:0] rvfi_mem_addr = 0;
  reg  [3:0]  rvfi_mem_rmask = 0;
  reg  [3:0]  rvfi_mem_wmask = 0;
  reg  [31:0] rvfi_mem_rdata = 0;
  reg  [31:0] rvfi_mem_wdata = 0;
  reg  [0:0]  rvfi_mem_extamo = 0;
  reg  [0:0]  rvfi_mem_fault = 0;

  wire [15:0] errcode;

  monitor dut (
    .clock(clock),
    .reset(reset),
    .rvfi_valid(rvfi_valid),
    .rvfi_order(rvfi_order),
    .rvfi_insn(rvfi_insn),
    .rvfi_trap(rvfi_trap),
    .rvfi_halt(rvfi_halt),
    .rvfi_intr(rvfi_intr),
    .rvfi_mode(rvfi_mode),
    .rvfi_rs1_addr(rvfi_rs1_addr),
    .rvfi_rs2_addr(rvfi_rs2_addr),
    .rvfi_rs1_rdata(rvfi_rs1_rdata),
    .rvfi_rs2_rdata(rvfi_rs2_rdata),
    .rvfi_rd_addr(rvfi_rd_addr),
    .rvfi_rd_wdata(rvfi_rd_wdata),
    .rvfi_pc_rdata(rvfi_pc_rdata),
    .rvfi_pc_wdata(rvfi_pc_wdata),
    .rvfi_mem_addr(rvfi_mem_addr),
    .rvfi_mem_rmask(rvfi_mem_rmask),
    .rvfi_mem_wmask(rvfi_mem_wmask),
    .rvfi_mem_rdata(rvfi_mem_rdata),
    .rvfi_mem_wdata(rvfi_mem_wdata),
    .rvfi_mem_extamo(rvfi_mem_extamo),
    .rvfi_mem_fault(rvfi_mem_fault),
    .errcode(errcode)
  );

  always #5 clock = ~clock;

  // `errcode` is a one-cycle pulse two clocks behind the retire that caused it (the
  // per-channel code is registered, then the top-level mux registers it again), so latch
  // it rather than sample on a guessed cycle.
  reg [15:0] latched = 0;
  reg        clear_latched = 0;

  always @(posedge clock) begin
    if (clear_latched) latched <= 0;
    else if (errcode) latched <= errcode;
  end

  integer failures = 0;

  task drive_retire;
    input [31:0] insn;
    input        trap;
    input [4:0]  rs1_addr;
    input [4:0]  rs2_addr;
    input [31:0] rs1_rdata;
    input [31:0] rs2_rdata;
    input [4:0]  rd_addr;
    input [31:0] rd_wdata;
    input [31:0] pc_rdata;
    input [31:0] pc_wdata;
    input [31:0] mem_addr;
    input [3:0]  mem_rmask;
    input [3:0]  mem_wmask;
    input [31:0] mem_rdata;
    input [31:0] mem_wdata;
    input        mem_fault;
    begin
      @(negedge clock);
      rvfi_valid     <= 1;
      rvfi_insn      <= insn;
      rvfi_trap      <= trap;
      rvfi_rs1_addr  <= rs1_addr;
      rvfi_rs2_addr  <= rs2_addr;
      rvfi_rs1_rdata <= rs1_rdata;
      rvfi_rs2_rdata <= rs2_rdata;
      rvfi_rd_addr   <= rd_addr;
      rvfi_rd_wdata  <= rd_wdata;
      rvfi_pc_rdata  <= pc_rdata;
      rvfi_pc_wdata  <= pc_wdata;
      rvfi_mem_addr  <= mem_addr;
      rvfi_mem_rmask <= mem_rmask;
      rvfi_mem_wmask <= mem_wmask;
      rvfi_mem_rdata <= mem_rdata;
      rvfi_mem_wdata <= mem_wdata;
      rvfi_mem_fault <= mem_fault;
      @(negedge clock);
      rvfi_valid <= 0;
      rvfi_order <= rvfi_order + 1;  // the ROB requires a dense, in-order stream
    end
  endtask

  // Drains the two-cycle errcode pipeline, compares, and clears the latch.
  task expect_errcode;
    input [15:0] expected;
    input [511:0] label;  // 64 chars; $display("%0s") left-truncates past that
    begin
      repeat (4) @(negedge clock);
      if (latched !== expected) begin
        $display("FAIL: %0s: expected monitor errcode %0d, got %0d", label, expected, latched);
        failures = failures + 1;
      end else begin
        $display("ok: %0s (errcode %0d)", label, latched);
      end
      clear_latched <= 1;
      @(negedge clock);
      clear_latched <= 0;
      @(negedge clock);
    end
  endtask

  initial begin
    repeat (4) @(negedge clock);
    reset <= 0;
    @(negedge clock);

    // 1. add x3, x1, x2 with x1=5, x2=7 -- correct, non-trapping. Establishes the shadow
    // PC at 4 and shadow x3 = 12 for the vectors that follow. A correct non-trapping
    // retire must be accepted.
    drive_retire(32'h0020_81b3, 1'b0,
                 5'd1, 5'd2, 32'd5, 32'd7,
                 5'd3, 32'd12,
                 32'h0000_0000, 32'h0000_0004,
                 32'd0, 4'b0000, 4'b0000, 32'd0, 32'd0, 1'b0);
    expect_errcode(16'd0, "correct add retire");

    // 2. lw x5, 1(x1) with x1 = RAM base -- address 0x00010001, misaligned, so spec_trap
    // holds. A correct core reports a trapping retire this way.
    drive_retire(32'h0010_a283, 1'b1,
                 5'd1, 5'd0, RAM, 32'd0,
                 5'd0, 32'd0,
                 32'h0000_0004, MTVEC,
                 32'd0, 4'b0000, 4'b0000, 32'hdead_beef, 32'd0, 1'b0);
    expect_errcode(16'd0, "trapping misaligned lw retire");

    drive_retire(32'h0030_0313, 1'b0,
                 5'd0, 5'd0, 32'd0, 32'd0,
                 5'd6, 32'd3,
                 MTVEC, MTVEC + 4,
                 32'd0, 4'b0000, 4'b0000, 32'd0, 32'd0, 1'b0);
    expect_errcode(16'd0, "correct addi retire after the trap");

    $display("monitor_tb: the monitor error banner below is EXPECTED (positive control)");
    drive_retire(32'h0020_81b3, 1'b0,
                 5'd1, 5'd2, 32'd5, 32'd7,
                 5'd3, 32'd13,
                 MTVEC + 4, MTVEC + 8,
                 32'd0, 4'b0000, 4'b0000, 32'd0, 32'd0, 1'b0);
    expect_errcode(16'd105, "wrong rd_wdata is still caught");

    drive_retire(32'h0020_a0a3, 1'b1,
                 5'd1, 5'd2, RAM, 32'hcafe_f00d,
                 5'd0, 32'd0,
                 MTVEC + 8, MTVEC,
                 32'd0, 4'b0000, 4'b0000, 32'd0, 32'd0, 1'b0);
    expect_errcode(16'd0, "trapping misaligned sw retire");

    drive_retire(32'h0000_0073, 1'b1,
                 5'd0, 5'd0, 32'd0, 32'd0,
                 5'd0, 32'd0,
                 MTVEC + 12, MTVEC,
                 32'd0, 4'b0000, 4'b0000, 32'd0, 32'd0, 1'b0);
    expect_errcode(16'd0, "ecall retire (no spec model exists -- unchecked)");

    drive_retire(32'h0000_a283, 1'b1,
                 5'd1, 5'd0, UNMAPPED, 32'd0,
                 5'd0, 32'd0,
                 MTVEC, MTVEC,
                 32'd0, 4'b0000, 4'b0000, 32'd0, 32'd0, 1'b1);
    expect_errcode(16'd0, "a refused load is not graded against the spec model");

    $display("monitor_tb: the monitor error banner below is EXPECTED (positive control)");
    drive_retire(32'h0000_a283, 1'b0,
                 5'd1, 5'd0, UNMAPPED, 32'd0,
                 5'd5, 32'hdead_beef,
                 MTVEC, MTVEC + 16,
                 UNMAPPED, 4'b1111, 4'b0000, 32'hdead_beef, 32'd0, 1'b1);
    expect_errcode(16'd150, "a refused access reported without a trap is caught");

    $display("monitor_tb: the monitor error banner below is EXPECTED (positive control)");
    drive_retire(32'h0010_a283, 1'b0,
                 5'd1, 5'd0, RAM, 32'd0,
                 5'd5, 32'hdead_beef,
                 MTVEC + 16, MTVEC + 20,
                 RAM + 1, 4'b1111, 4'b0000, 32'hdead_beef, 32'd0, 1'b0);
    expect_errcode(16'd101, "failure to trap on a misaligned lw is still caught");

    if (failures != 0) begin
      $display("monitor_tb: %0d vector(s) failed", failures);
      $fatal(1);
    end
    $display("monitor_tb: all vectors passed");
    $finish;
  end
endmodule
