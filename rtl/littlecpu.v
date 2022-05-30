`default_nettype none
module littlecpu(
  input  var logic clk,
  input  var logic reset,
  output var logic imem_addr,
  input  var logic imem_data,
  input  var logic mem_valid,
  output var logic mem_ready,
  output var logic [31:0] mem_addr,
  output var logic [31:0] mem_wdata,
  output var logic [3:0] mem_wstrb,
  input  var logic [31:0] mem_rdata,
  output var logic trap
         `ifdef RISCV_FORMAL
         ,
  output var logic rvfi_valid,
  output var logic [63:0] rvfi_order,
  output var logic [31:0] rvfi_insn,
  output var logic rvfi_trap,
  output var logic rvfi_halt,
  output var logic rvfi_intr,
  output var logic [ 1:0] rvfi_mode,
  output var logic [ 1:0] rvfi_ixl,
  output var logic [ 4:0] rvfi_rs1_addr,
  output var logic [ 4:0] rvfi_rs2_addr,
  output var logic [31:0] rvfi_rs1_rdata,
  output var logic [31:0] rvfi_rs2_rdata,
  output var logic [ 4:0] rvfi_rd_addr,
  output var logic [31:0] rvfi_rd_wdata,
  output var logic [31:0] rvfi_pc_rdata,
  output var logic [31:0] rvfi_pc_wdata,
  output var logic [31:0] rvfi_mem_addr,
  output var logic [ 3:0] rvfi_mem_rmask,
  output var logic [ 3:0] rvfi_mem_wmask,
  output var logic [31:0] rvfi_mem_rdata,
  output var logic [31:0] rvfi_mem_wdata,
  output var logic [63:0] rvfi_csr_mcycle_rmask,
  output var logic [63:0] rvfi_csr_mcycle_wmask,
  output var logic [63:0] rvfi_csr_mcycle_rdata,
  output var logic [63:0] rvfi_csr_mcycle_wdata,
  output var logic [63:0] rvfi_csr_minstret_rmask,
  output var logic [63:0] rvfi_csr_minstret_wmask,
  output var logic [63:0] rvfi_csr_minstret_rdata,
  output var logic [63:0] rvfi_csr_minstret_wdata
         `endif //  `ifdef RISCV_FORMAL
  );
  logic mem_instr;
  logic fetcher_valid, fetcher_mem_ready, accessor_mem_ready;
  // possible hazard here
  assign mem_ready = fetcher_mem_ready || accessor_mem_ready;
  logic [31:0] pc;
  fetcher_output fetcher_out;
  fetcher fetcher(
    .clk(clk),
    .reset(reset),
    // handshake
    .mem_valid(mem_valid),
    .fetcher_valid(fetcher_valid),
    .fetcher_ready(fetcher_decoder_ready),
    // inputs
    .pc(pc),
    .mem_rdata(mem_rdata),
    // outputs
    .out(fetcher_out),
    .mem_instr(mem_instr),
    .mem_ready(fetcher_mem_ready),
    .mem_addr(mem_addr)
  );

  logic fetcher_decoder_ready, fetcher_decoder_valid;
  fetcher_output  fetcher_decoder_out;
  skidbuffer #(.WIDTH($bits(fetcher_out))) fetcher_decoder(
    .clk(clk),
    .reset(reset),
    .input_ready(fetcher_decoder_ready),
    .input_valid(fetcher_valid),
    .input_data(fetcher_out),
    .output_ready(decoder_ready),
    .output_valid(fetcher_decoder_valid),
    .output_data(fetcher_decoder_out)
  );

  logic [31:0] reg_rs1, reg_rs2, wdata;
  logic [4:0]  waddr;
  logic        wen;
  regfile regfile(
    .clk(clk),
    // from the decoder
    .rs1(rs1),
    .rs2(rs2),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );

  logic decoder_ready, decoder_valid;
  decoder_output decoder_out;
  decoder decoder(
    .clk(clk),
    .reset(reset),
    // handshake
    .fetcher_valid(fetcher_decoder_valid),
    .decoder_ready(decoder_ready),
    .decoder_valid(decoder_valid),
    .executor_ready(executor_ready),
    // inputs
    .in(fetcher_decoder_out),
    // The decoder is largely synchronous so these are assigned a clock cycle early
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    // outputs
    // The whole trick! we update the program counter here to keep the pipeline filled
    .pc(pc),
     // rs1 and rs2 are not latched: used to get reg_rs1 and reg_rs2 from the reg file
    .rs1(rs1),
    .rs2(rs2),
    .out(decoder_out)
  );

  logic decoder_executor_ready, decoder_executor_valid;
  decoder_output decoder_executor_output;
    skidbuffer #(.WIDTH($bits(decoder_out))) decoder_executor(
    .clk(clk),
    .reset(reset),
    .input_ready(decoder_executor_ready),
    .input_valid(decoder_valid),
    .input_data(decoder_out),
    .output_ready(executor_ready),
    .output_valid(decoder_executor_valid),
    .output_data(decoder_executor_out)
  );

  logic executor_ready, executor_valid;
  executor_output executor_out;
  executor executor(
    .clk(clk),
    .reset(reset),
    // handshake
    .decoder_valid(decoder_executor_valid),
    .executor_ready(executor_ready),
    .executor_valid(executor_valid),
    .accessor_ready(accessor_ready),
    // inputs
    .in(decoder_executor_out),
    // outputs
    .out(executor_out)
  );

  logic executor_accessor_ready, executor_accessor_valid;
  executor_output executor_accessor_output;
  skidbuffer #(.WIDTH($bits(executor_out))) executor_accessor(
    .clk(clk),
    .reset(reset),
    .input_ready(executor_accessor_ready),
    .input_valid(executor_valid),
    .input_data(executor_out),
    .output_ready(accessor_ready),
    .output_valid(executor_accessor_valid),
    .output_data(executor_accessor_out)
  );

  logic accessor_ready, accessor_valid;
  accessor_output accessor_out;
  accessor accessor(
    .clk(clk),
    .reset(reset),
    // handshake
    .executor_valid(executor_valid),
    .accessor_ready(accessor_ready),
    .accessor_valid(accessor_valid),
    .writeback_ready(writeback_ready),
    // inputs
    .in(executor_accessor_out),
    // memory access
    .mem_instr(mem_instr),
    .mem_ready(accessor_mem_ready),
    .mem_addr(mem_addr),
    .mem_valid(mem_valid),
    .mem_wstrb(mem_wstrb),
    .mem_wdata(mem_wdata),
    .mem_rdata(mem_rdata),
    // forwards
    .out(accessor_out)
  );

  logic writeback_ready;
  writeback writeback(
    .clk(clk),
    .reset(reset),
    // handshake
    .writeback_ready(writeback_ready),
    .accessor_valid(accessor_valid),
    // inputs
    .in(accessor_out),
    // outputs
    .wen(wen),
    .waddr(waddr),
    .wdata(wdata)
  );
endmodule
