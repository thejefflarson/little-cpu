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
  input         arready, // they are reading the value
  output [31:0] araddress, // address to read
  output [2:0]  arprot, // permissions

  input         rvalid, // Data is valid and can be read by us
  output        rready, // we are ready to read
  input [31:0]  rdata, // value to read
  input [1:0]   rresp, // status of our read request

  // outputs
  output        trap,
  output [1:0]  trap_code
);
  reg [31:0] regs[0:31];
  reg [31:0] pc;
  reg [31:0] instr;

  localparam trap_mem = 2'b00;

  // we can execute, no memory operations pending
  reg        execute;
  // please laod an instruction
  reg        load_instr;
  reg [31:0] load_address;
  reg [31:0] load_data;
  reg [31:0] next_pc;
  reg [31:0] next_instr;

  integer i;

  // reset registers
  always @(posedge clk) begin
     if (!reset) begin
        for (i = 0; i < 32; i = i + 1) begin
          regs[i] <= 32'b1;
        end
     end
  end

  // reset memory module
  always @(posedge clk) begin
    if (!reset) begin
      arvalid <= 0; // we haven't put anything in the address
      rready <= 0; // we are ready to read
      execute <= 0;
      pc <= 32'b0;
      next_pc <= 32'b0;
      arprot <= inst_prot;
      load_instr <= 0;
      load_address <= 32'b0;
      load_data <= 32'b0;
    end
  end
  // privileged, insecure and instruction
  localparam inst_prot = 3'b101;
  // privileged, insecure and data
  localparam data_prot = 3'b000;
  // memory request
  always @(posedge clk) begin
    // we aren't executing and we haven't requested a read
    if (reset && !execute && !arvalid) begin
      if (load_instr) begin
        araddress <= next_pc;
        arprot <= inst_prot;
      end else begin
        araddress <= load_address;
        arprot <= data_prot;
      end
      // we're ready to read
      arvalid <= 1;
      rready <= 1;
    end
  end
  localparam rresp_ok = 2'b00;
  localparam rresp_xok = 2'b01;
  // memory fetch
  always @(posedge clk) begin
    // everyone is ready
    if (reset && rready && rvalid && arready && arvalid) begin
      if (rresp == rresp_ok || rresp == rresp_xok) begin
        execute <= 1;
        if (load_instr)
          next_instr <= rdata;
        else
          load_data <= rdata;
      end else begin
        trap <= 1;
        trap_code <= trap_mem;
      end
      rready <= 0;
      arvalid <= 0;
    end
  end


  // opcode decoder
  always @(posedge clk) begin
    if (!reset) begin

    end
  end

  // instruction decoder
  always @(posedge clk) begin
    if (!reset) begin

    end
  end

  // state_machine
  always @(posedge clk) begin
    if (!reset) begin
    end
  end

endmodule
