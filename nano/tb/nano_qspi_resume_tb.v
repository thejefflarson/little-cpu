`timescale 1ns/1ps
// The root-caused chained-resume nibble drift, reproduced against nano_qspi_ctrl and the
// pin-level flash model directly: two "fetch_hit0 && !queue_full" resumes back to back.
// QSPI_RESUME_TB_DELAY_CYCLES delays sio's return half that many clocks, for a pad-mux round trip.
`ifndef QSPI_RESUME_TB_DELAY_CYCLES
`define QSPI_RESUME_TB_DELAY_CYCLES 0
`endif
module nano_qspi_resume_tb;
  logic clk = 0;
  always #5 clk = ~clk;
  logic reset = 1;

  logic mem_valid, mem_instr, mem_ready;
  logic [31:0] mem_addr, mem_wdata, mem_rdata;
  logic [3:0] mem_wstrb;

  logic sck, flash_cs_n, psram_cs_n, spare_cs_n;
  logic [3:0] sio_c2d, sio_d2c, sio_d2c_delayed;
  logic sio_oe_ctrl;
  logic [3:0] flash_out, psram_out;
  logic flash_oe, psram_oe;

  assign sio_d2c = flash_oe ? flash_out : (psram_oe ? psram_out : 4'bz);

  generate
    if (`QSPI_RESUME_TB_DELAY_CYCLES == 0) begin : g_no_delay
      assign sio_d2c_delayed = sio_d2c;
    end else begin : g_delay
      logic [3:0] stages[0:`QSPI_RESUME_TB_DELAY_CYCLES-1];
      always_ff @(posedge clk) begin
        stages[0] <= sio_d2c;
        for (int i = 1; i < `QSPI_RESUME_TB_DELAY_CYCLES; i++) stages[i] <= stages[i-1];
      end
      assign sio_d2c_delayed = stages[`QSPI_RESUME_TB_DELAY_CYCLES-1];
    end
  endgenerate

  nano_qspi_ctrl #(.FLASH_DUMMY_SCK(4), .PSRAM_DUMMY_SCK(4)) dut (
    .clk(clk), .reset(reset),
    .mem_valid(mem_valid), .mem_instr(mem_instr), .mem_ready(mem_ready),
    .mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_wstrb(mem_wstrb), .mem_rdata(mem_rdata),
    .sck(sck), .flash_cs_n(flash_cs_n), .psram_cs_n(psram_cs_n), .spare_cs_n(spare_cs_n),
    .sio_out(sio_c2d), .sio_oe(sio_oe_ctrl), .sio_in(sio_d2c_delayed)
  );

  nano_qspi_flash_model #(.WORDS(1024), .DUMMY_SCK(4)) flash (
    .clk(clk), .reset(reset), .sck(sck), .cs_n(flash_cs_n),
    .sio_in(sio_c2d), .sio_out(flash_out), .sio_oe(flash_oe)
  );

  nano_qspi_psram_model #(.WORDS(1024), .DUMMY_SCK(4)) psram (
    .clk(clk), .reset(reset), .sck(sck), .cs_n(psram_cs_n),
    .sio_in(sio_c2d), .sio_out(psram_out), .sio_oe(psram_oe)
  );

  int errors = 0;

  // mem_valid stays high through mem_ready's cycle, matching nano.v's own held-request overlap.
  task automatic do_fetch(input [31:0] addr, output [31:0] rdata);
    integer c;
    mem_addr = addr; mem_instr = 1; mem_valid = 1; mem_wstrb = 0;
    c = 0;
    do begin
      @(posedge clk); #1;
      c = c + 1;
      if (c > 400) begin
        $display("TIMEOUT fetch %0h", addr);
        $display("FAIL");
        $finish;
      end
    end while (!mem_ready);
    rdata = mem_rdata;
    mem_valid = 0;
    @(posedge clk); #1;
  endtask

  logic [31:0] rd;
  initial begin
    mem_valid = 0; mem_instr = 0; mem_addr = 0; mem_wdata = 0; mem_wstrb = 0;

    // parcel0 is compressed; parcels 1/3/5 are uncompressed, each needing the next.
    flash.mem[0] = {16'h1003, 16'h0000};
    flash.mem[1] = {16'h1007, 16'h1004};
    flash.mem[2] = {16'h100b, 16'h1008};
    flash.mem[3] = {16'hffff, 16'h100c};

    @(posedge clk); #1; reset = 0;

    do_fetch(32'h0, rd);  // fresh access: streams and pauses with slot0=parcel0, slot1=parcel1
    if (rd[15:0] !== 16'h0000) begin
      $display("FAIL: fetch(0) expected parcel0=0000, got %h", rd[15:0]);
      errors++;
    end

    do_fetch(32'h2, rd);  // resume #1: fetch_hit0 && !queue_full, streams parcel2, pauses
    if (rd !== 32'h10041003) begin
      $display("FAIL: fetch(2) expected parcel1:2=10041003, got %h", rd);
      errors++;
    end

    do_fetch(32'h6, rd);  // resume #2, chained immediately after resume #1
    if (rd !== 32'h10081007) begin
      $display("FAIL: fetch(6) expected parcel3:4=10081007, got %h", rd);
      errors++;
    end

    do_fetch(32'ha, rd);
    if (rd !== 32'h100c100b) begin
      $display("FAIL: fetch(a) expected parcel5:6=100c100b, got %h", rd);
      errors++;
    end

    if (errors == 0) $display("PASS");
    else $display("FAIL: %0d mismatch(es)", errors);
    $finish;
  end
endmodule
