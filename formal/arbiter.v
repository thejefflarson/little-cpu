module imem_arbiter (
  input  logic        clock,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [3:0]  mem_wstrb,
  input  logic        mem_ren,
  output logic        fetch_stall,
  output logic        text_write
);
  localparam logic [31:0] TEXT_BYTES = 32'h0000_2000;

  logic text_range, text_access;
  assign text_range  = mem_addr < TEXT_BYTES;
  assign text_access = !reset && (mem_ren || |mem_wstrb) && text_range;
  assign text_write  = !reset && |mem_wstrb && text_range;

  initial fetch_stall = 1'b0;
  always @(posedge clock)
    fetch_stall <= text_access;
endmodule
