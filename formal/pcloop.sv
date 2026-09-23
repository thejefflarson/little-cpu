// The fetcher and the decoder, wired together the way rtl/littlecpu.v wires them.
`default_nettype none

module pcloop (
    input logic clk,
    input logic reset,
    input logic [31:0] imem_data,
    input logic [31:0] imem_data2,
    input logic [31:0] reg_rs1,
    input logic [31:0] reg_rs2,
    input executor_output executor_out,
    input logic divider_stall,
    input logic fetch_stall,
    // Free, like every other stall input here: a hart that has not been granted the
    // shared bus holds the pc, and the increment assertion has to skip that cycle the
    // same way it skips a stolen fetch window.
    input logic bus_wait,
    // Free, like everything else not instantiated here.
    input logic imem_fault,
    // Free for the same reason and with the same effect: an atomic the platform does not
    // answer redirects the pc, and `branch_jump` names that trap too.
    input logic atomic_supported,
    input logic accessor_out_valid,
    input logic [31:0] csr_rdata,
    input logic csr_implemented,
    input logic [31:0] mtvec,
    input logic [31:0] mepc,
    // Free, like everything else not instantiated here.
    input logic interrupt_pending
);
  logic [31:0] pc;
  logic [31:0] imem_addr, imem_addr2;
  logic [31:0] next_pc, imem_addr_next;
  // The address the decoder publishes for a platform to decode.
  logic [31:0] atomic_addr;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] rs1, rs2, read_rs1, read_rs2;
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc, trap_tval;

  fetcher fetcher (
    .clk(clk),
    .reset(reset),
    .pc(pc),
    .next_pc(next_pc),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .out(fetcher_out)
  );

  decoder decoder (
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stall),
    .fetch_stall(fetch_stall),
    .bus_wait(bus_wait),
    .bus_request(bus_request),
    .imem_fault(imem_fault),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .accessor_out_valid(accessor_out_valid),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec),
    .mepc(mepc),
    .interrupt_pending(interrupt_pending),
    .pc(pc),
    .next_pc(next_pc),
    .rs1(rs1),
    .rs2(rs2),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .trap_tval(trap_tval),
    .mret_entry(mret_entry),
    .out(decoder_out)
  );

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;

  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  logic [31:0] f_instr;
  assign f_instr = fetcher_out.instr;
  logic f_uncompressed;
  assign f_uncompressed = f_instr[1:0] == 2'b11;
  logic f_jump_branch;
  assign f_jump_branch =
      (f_uncompressed && f_instr[6:2] == 5'b11011) ||
      (f_uncompressed && f_instr[6:2] == 5'b11001 && f_instr[14:12] == 3'b000) ||
      (f_uncompressed && f_instr[6:2] == 5'b11000 &&
         f_instr[14:12] != 3'b010 && f_instr[14:12] != 3'b011) ||
      (f_instr[1:0] == 2'b01 && (f_instr[15:13] == 3'b101 || f_instr[15:13] == 3'b001 ||
                                 f_instr[15:13] == 3'b110 || f_instr[15:13] == 3'b111)) ||
      (f_instr[1:0] == 2'b10 && f_instr[15:13] == 3'b100 && f_instr[6:2] == 5'b0 &&
         f_instr[11:7] != 5'b0);

  logic f_live_rs1, f_live_rs2, f_may_stall;
  assign f_live_rs1 = rs1 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs1) ||
       (executor_out.valid && executor_out.rd == rs1));
  assign f_live_rs2 = rs2 != 0 &&
      ((decoder_out.valid && decoder_out.rd == rs2) ||
       (executor_out.valid && executor_out.rd == rs2));
  logic f_system;
  assign f_system = f_uncompressed && f_instr[6:2] == 5'b11100;

  logic [4:0] f_prev_rs1, f_prev_rs2;
  logic       f_read_taken, f_operand_fetch;
  always_ff @(posedge clk) begin
    if (reset) begin
      f_prev_rs1   <= 5'd0;
      f_prev_rs2   <= 5'd0;
      f_read_taken <= 1'b0;
    end else begin
      f_prev_rs1   <= read_rs1;
      f_prev_rs2   <= read_rs2;
      f_read_taken <= 1'b1;
    end
  end
  assign f_operand_fetch = !f_read_taken || f_prev_rs1 != rs1 || f_prev_rs2 != rs2;

  logic f_pair_moved;
  assign f_pair_moved = read_rs1 != f_prev_rs1 || read_rs2 != f_prev_rs2;

  logic f_fencei;
  assign f_fencei = f_uncompressed && f_instr[6:2] == 5'b00011;

  logic f_amo_wait;
  assign f_amo_wait = decoder_out.valid && decoder_out.is_amo;

  logic f_load_store;
  assign f_load_store =
      (f_uncompressed && (f_instr[6:2] == 5'b00000 || f_instr[6:2] == 5'b01000)) ||
      ((f_instr[1:0] == 2'b00 || f_instr[1:0] == 2'b10) &&
         (f_instr[15:13] == 3'b010 || f_instr[15:13] == 3'b110));

  assign f_may_stall = divider_stall || fetch_stall || bus_wait ||
      f_live_rs1 || f_live_rs2 || f_system || f_fencei || f_operand_fetch ||
      f_amo_wait || f_load_store;

  logic f_redirect;
  assign f_redirect = trap_entry || mret_entry;

  logic [31:0] past_pc, prev_mtvec, prev_mepc;
  logic prev_reset, prev_may_stall, prev_hard_stall, prev_jump_branch, prev_uncompressed;
  logic prev_trap_entry, prev_mret_entry, prev_fetch_stall, prev_pair_moved, prev_bus_wait;
  always_ff @(posedge clk) begin
    past_pc           <= pc;
    prev_reset        <= reset;
    prev_may_stall    <= f_may_stall;
    prev_hard_stall   <= divider_stall;
    prev_fetch_stall  <= fetch_stall;
    prev_bus_wait     <= bus_wait;
    prev_jump_branch  <= f_jump_branch || f_redirect;
    prev_uncompressed <= f_uncompressed;
    prev_trap_entry   <= trap_entry;
    prev_mret_entry   <= mret_entry;
    prev_mtvec        <= mtvec;
    prev_mepc         <= mepc;
    prev_pair_moved   <= f_pair_moved;
  end

  always_comb if (clocked && !reset) assert(fetcher_out.pc == pc);

  logic [31:0] past_imem_addr_next;
  always_ff @(posedge clk) past_imem_addr_next <= imem_addr_next;
  always_comb if (clocked) assert(imem_addr == past_imem_addr_next);

  logic f_increment_checked;
  assign f_increment_checked =
      clocked && !prev_reset && !prev_may_stall && !prev_jump_branch;

  always_ff @(posedge clk)
    if (f_increment_checked)
      assert(pc == past_pc + (prev_uncompressed ? 32'd4 : 32'd2));

  always_ff @(posedge clk)
    if (f_increment_checked) begin
      increment_reached: cover (1'b1);
      increment_reached_on_moved_pair: cover (prev_pair_moved);
    end

  always_ff @(posedge clk)
    if (clocked && prev_hard_stall && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && prev_fetch_stall && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && prev_bus_wait && !prev_reset) assert(pc == past_pc);

  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_trap_entry) assert(pc == prev_mtvec);
  always_ff @(posedge clk)
    if (clocked && !prev_reset && prev_mret_entry) assert(pc == prev_mepc);
 `endif
endmodule

`default_nettype wire
