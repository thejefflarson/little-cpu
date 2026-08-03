// rtl/imemory.v's arbiter, transcribed. A data access inside the text range
// takes the ROM banks' read port for that edge, so the fetch window presented on
// the next cycle carries the data word instead of the instruction. The memory
// reports that as `fetch_stall`.
//
// Drive `fetch_stall` from this in every harness rather than leaving it a free
// input. An environment allowed to choose it can hold the core still forever,
// and `hang` and `liveness_ch0` are the two checks that measure exactly that.
//
// TEXT_BYTES is rtl/littlesoc.v's `ROM_WORDS = 2048`. Nothing here depends on
// the exact size: it decides how often a steal is reachable, not whether it is.
module imem_arbiter (
  input  logic        clock,
  input  logic        reset,
  input  logic [31:0] mem_addr,
  input  logic [3:0]  mem_wstrb,
  input  logic        mem_ren,
  output logic        fetch_stall,
  // High on the cycle a store lands in the text range. The array changes on that
  // edge, which is a second thing the fetch bus has to account for.
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
