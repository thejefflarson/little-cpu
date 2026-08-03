`define RISCV_FORMAL
`define RISCV_FORMAL_NRET 1
`define RISCV_FORMAL_XLEN 32
`define RISCV_FORMAL_ILEN 32
`define RISCV_FORMAL_ALIGNED_MEM
`include "rvfi_macros.vh"
`include "rvfi_channel.sv"
`include "rvfi_imem_check.sv"

module testbench (
  input clk
);
  logic reset = 1;
  logic trap;

  always_ff @(posedge clk)
    reset <= 0;

  `RVFI_WIRES

  logic [31:0] imem_addr;
  logic [15:0] imem_data;

  rvfi_imem_check checker_inst (
    .clock(clk),
    .reset(reset),
    .enable(1'b1),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    `RVFI_CONN
  );

  logic [31:0] uut_imem_addr;
  logic [31:0] uut_imem_data;
  // ADR-0003: the dual-word fetch window's second word/address. Checking
  // both ports against the same 16-bit-granularity checker model is what
  // makes this the ready-made consistency check for the window (ADR-0003's
  // own consequences section) -- if imem_addr2 ever disagreed with
  // imem_addr about what's at a given halfword, this catches it, which a
  // check that only looked at imem_addr could not.
  logic [31:0] uut_imem_addr2;
  logic [31:0] uut_imem_data2;
  // ADR-0054: the fetch address one cycle early, for a synchronous memory.
  // Deliberately unread: the four assumes above pin `imem_data`/`imem_data2`
  // against `uut_imem_addr`/`uut_imem_addr2` IN THE SAME CYCLE, which is the
  // combinational fetch bus this check was written for and is what it keeps
  // checking. A memory that answers this port a cycle early is outside its
  // contact -- see ADR-0054, which says so rather than leaving it implied.
  logic [31:0] uut_imem_addr_next;
  logic [31:0] mem_addr;
  logic [31:0] mem_wdata;
  logic [3:0]  mem_wstrb;
  logic        mem_ren;
  logic [31:0] mem_rdata;

  // FACT       no data access ever takes the fetch window (ADR-0059's steal).
  // DISCHARGED formal/wrapper.v, which transcribes rtl/imemory.v's arbiter and
  //            drives `fetch_stall` from it, so all 85 ladder checks run
  //            against a stealing memory.
  // SCOPE      this whole task. It is the same fiction the four assumes below
  //            already rest on -- a fetch bus that answers combinationally,
  //            from a memory this task does not model -- and a steal against an
  //            unmodelled memory would add stall cycles without adding
  //            coverage. The write path this task needs to observe a text store
  //            is a separate obligation and is not built yet.
  logic        fetch_stall = 1'b0;

  // FACT       the fetch bus answers from memory: whenever either of the two
  //            fetch ports covers the halfword `checker_inst` has pinned, the
  //            data the core sees there is that halfword.
  // DISCHARGED NOWHERE. rvfi_imem_check asserts that the CORE is consistent
  //            with the pinned halfword; nothing asserts that the environment
  //            is. The backing is structural and is the same one
  //            formal/wrapper.v:83 rests on -- rtl/imemory.v is a $readmemh
  //            ROM (ADR-0044) -- and is likewise believed, not proved.
  // SCOPE      the one rvfi_imem_check assertion this task contains, which is
  //            also everything it was written for. All four assumes constrain
  //            DUT INPUTS (uut_imem_data/uut_imem_data2), so they narrow the
  //            environment and can never excuse the core; the checker's own
  //            imem_addr/imem_data are rand_const, so there is no cycle
  //            history for the scope to leak across either.
  //
  // No mem_valid/mem_ready to gate on: rtl/fetcher.v drives imem_addr/
  // imem_addr2 = {pc[31:2],2'b00}/+4 and out.instr = the windowed pair
  // combinationally, unconditionally, every non-reset cycle (CLAUDE.md
  // invariant 1 -- fetch never stalls or waits). checker_inst's imem_addr/
  // imem_data are `rand_const_reg`s: fixed for the whole trace, not
  // resampled per cycle, so this comparison needs no cycle-history
  // bookkeeping either -- whenever either of the DUT's two fetch ports this
  // cycle targets the checker's fixed address, its data must agree, full
  // stop.
  always_comb begin
    if (!reset) begin
      if (uut_imem_addr == imem_addr)
        assume(uut_imem_data[15:0] == imem_data);
      if (uut_imem_addr + 2 == imem_addr)
        assume(uut_imem_data[31:16] == imem_data);
      if (uut_imem_addr2 == imem_addr)
        assume(uut_imem_data2[15:0] == imem_data);
      if (uut_imem_addr2 + 2 == imem_addr)
        assume(uut_imem_data2[31:16] == imem_data);
    end
  end

  littlecpu uut (
    .clk(clk),
    .reset(reset),
    .imem_addr(uut_imem_addr),
    .imem_data(uut_imem_data),
    .imem_addr2(uut_imem_addr2),
    .imem_data2(uut_imem_data2),
    .imem_addr_next(uut_imem_addr_next),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_wstrb(mem_wstrb),
    .mem_ren(mem_ren),
    .mem_rdata(mem_rdata),
    .fetch_stall(fetch_stall),
    .trap(trap),
    `RVFI_CONN
  );
endmodule
