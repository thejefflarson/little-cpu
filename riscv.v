module riscv (
  input         clk, resetn,
  // 32bit AXI4-lite

  output        awvalid, // we wrote the address
  input         awready, // address is ready for write
  output [31:0] awaddress, // address to write
  output [2:0]  awprot, // permissions

  output        wvalid, // we wrote the value
  input         wready, // value is ready
  output [31:0] wdata, // value to write
  output [3:0]  wrstrb, // what byte we wrote, here this will always be b'1111

  output        bvalid, // we received the response
  input         bready, // status is ready
  input         bresp, // status of our write request

  output        arvalid, // we read the address
  input         arready, // address is ready for reading
  input  [31:0] araddress, // address to read
  output [2:0]  arprot, // permissions

  output        rvalid, // we read the value
  input         rready, // value is ready
  input  [31:0] rdata, // value to read
  input         rresp // status of our read request
);
  reg [31:0] regs[0:31];
  assign regs[0] = 32b'0;

  always @(posedge clk) begin
     a <= ~clk;
  end
endmodule
