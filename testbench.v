`timescale 1 ns / 1 ps

module testbench;
  logic [31:0] memory[0:255];
  logic clk = 0;
  logic reset = 0;
  always #5 clk = ~clk;

  initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, testbench);
    repeat (1) @(posedge clk);
    reset <= 1;
    repeat (20) @(posedge clk);
    $finish;
  end

  logic        awvalid;
  logic        awready;
  logic [31:0] awaddress;
  logic [2:0]  awprot;
  logic        wvalid;
  logic        wready;
  logic [31:0] wdata;
  logic [3:0]  wstrb;
  logic        bvalid;
  logic        bready;
  logic [1:0]  bresp;
  logic        arvalid;
  logic        arready;
  logic [31:0] araddress;
  logic [2:0]  arprot;
  logic        rvalid;
  logic        rready;
  logic [31:0] rdata;
  logic [1:0]  rresp;
  logic        trap;
  logic [1:0]  trap_code;


  always @(posedge clk) begin
    arready <= 0;
    rvalid <= 0;

    if (arvalid && !arready && rready && !rvalid) begin
      arready <= 1;
      rvalid <= 1;
      if (araddress < 1024) begin
        rdata <= memory[araddress >> 2];
        rresp <= 2'b00;
      end else begin
        rresp <= 3'b11;
      end
    end
  end

  riscv uut (
    .clk(clk),
    .reset(reset),
    .awvalid(awvalid),
    .awready(awready),
    .awaddress(awaddress),
    .awprot(awprot),
    .wvalid(wvalid),
    .wready(wready),
    .wdata(wdata),
    .wstrb(wstrb),
    .bvalid(bvalid),
    .bready(bready),
    .bresp(bresp),
    .arvalid(arvalid),
    .arready(arready),
    .araddress(araddress),
    .arprot(arprot),
    .rvalid(rvalid),
    .rready(rready),
    .rdata(rdata),
    .rresp(rresp),
    .trap(trap),
    .trap_code(trap_code)
  );

  initial begin
    memory[0] = 32'h0000000f; // fence
    memory[1] = 32'h00000073; // ebreak
  end

  always_ff @(posedge clk) begin
    if (arvalid && arready && rvalid && rready) begin
      if (arprot == 3'b101) begin
        $display("fetch 0x%08x: 0x%08x", araddress, rdata);
      end else begin
        $display("read 0x%08x: 0x%08x", araddress, rdata);
      end
    end
  end
endmodule // testbench
