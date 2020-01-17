module riscv (
  input         clk, reset,
  // 32bit AXI4-lite

  output        awvalid, // we wrote the address
  input         awready, // address is ready for write
  output [31:0] awaddress, // address to write
  output [2:0]  awprot, // permissions

  output        wvalid, // we wrote the value
  input         wready, // value is ready
  output [31:0] wdata, // value to write
  output [3:0]  wrstrb, // what bytes we wrote, here this will always be b'1111, or all 32bits

  output        bvalid, // we received the response
  input         bready, // status is ready
  input         bresp, // status of our write request

  output        arvalid, // we put something in the read addreas
  input         arready, // they are reading thej value
  output [31:0] araddress, // address to read
  output [2:0]  arprot, // permissions

  input         rvalid, // Data is valid and can be read by us
  output        rready, // we are ready to read
  input  [31:0] rdata, // value to read
  input         rresp // status of our read request
);
  assign wrstrb = 4'b1;
  reg [31:0] regs[0:31];
  reg [31:0] pc;
  reg [31:0] instr;

  reg 	     trap; // ruroh
  reg 	     execute; // we can execute
  reg [31:0] next_pc;
  reg [31:0] next_instr;

  integer i;

  always @(posedge clk) begin
     if (!reset) begin
        for (i = 0; i < 32; i = i + 1) begin
          regs[i] <= 31'b0;
	  pc <= 31'b0;
	end
     end
  end

  // instruction fetcher
  always @(posedge clk) begin
    if (!reset) begin
      arvalid <= 0; // we haven't put anything in the address
      rready <= 1; // we are ready to read
    end else begin

    end
  end

  // opcode decoder
  always @(posedge clk) begin
    if(!reset) begin

    end
  end

  // instruction decoder
  always @(posedge clk) begin
    if(!reset) begin

    end
  end

  // exectution

endmodule
