`default_nettype none
module memory #(
  // in four byte words
  parameter integer RAM = 16384,
  parameter integer INSTR = 16384,
  parameter string  ROM = "./rom.mem"
) (
  input  var logic        clk,
  input  var logic        mem_ready,
  output var logic        mem_valid,
  input  var logic [31:0] mem_addr,
  input  var logic [31:0] mem_wdata,
  input  var logic [3:0]  mem_wstrb,
  input  var logic        mem_instr,
  output var logic [31:0] mem_rdata
);
  logic [31:0] instr[0:INSTR-1];
  initial $readmemb(ROM, instr);

  logic [31:0] ram[0:RAM-1];
  always_ff @(posedge clk) begin
    if (!mem_instr) begin
      mem_valid <= 0;
      if (mem_ready && !mem_valid) begin
        if (mem_addr < 4*RAM) begin
          mem_rdata <= ram[mem_addr >> 2];
          if(mem_wstrb[0]) ram[mem_addr >> 2][7:0] <= mem_wdata[7:0];
          if(mem_wstrb[1]) ram[mem_addr >> 2][15:8] <= mem_wdata[15:8];
          if(mem_wstrb[2]) ram[mem_addr >> 2][23:16] <= mem_wdata[23:16];
          if(mem_wstrb[3]) ram[mem_addr >> 2][31:24] <= mem_wdata[31:24];
          mem_valid <= 1;
        end
      end
    end else begin // if (!mem_instr)
      mem_valid <= 0;
      if (mem_ready && !mem_valid) begin
        if(mem_addr < 4*INSTR) begin
          mem_rdata <= instr[mem_addr >> 2];
          mem_valid <= 1;
        end
      end
    end
  end
endmodule
