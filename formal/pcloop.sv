// The fetcher, D and X, wired the way rtl/littlecpu.v wires them, fetch_pc register
// included (the split moved it into the integrator, since it spans F and X).
`default_nettype none

module pcloop (
    input logic clk,
    input logic reset,
    input logic [31:0] imem_data,
    input logic [31:0] imem_data2,
    input logic [31:0] reg_rs1,
    input logic [31:0] reg_rs2,
    input logic imem_stall,  // the ROM's stolen-read flag; the fetcher turns it into fetch_stall
    input logic bus_wait,  // free, like every stall input here: an ungranted hart holds fetch_pc
    input logic rom_fault,  // free, like everything else not instantiated here
    input logic atomic_supported,  // free; an unanswered atomic redirects fetch_pc too
    input logic accessor_out_valid,
    input logic [31:0] csr_rdata,
    input logic csr_implemented,
    input logic [31:0] mtvec,
    input logic [31:0] mepc,
    input logic interrupt_pending  // free, like everything else not instantiated here
);
  logic [31:0] fetch_pc, fetch_pc_next;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic        fetch_wait, fetch_fault, decoder_issuing, x_redirect;
  logic [31:0] decoder_predicted_pc;
  fetcher_output fetcher_out;
  dx_output dx_out;
  decoder_output decoder_out;
  executor_output executor_out;
  logic [4:0] read_rs1, read_rs2;
  logic        x_busy;
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc, trap_tval;
  logic [31:0] atomic_addr;
  logic [31:0] x_redirect_target;
  logic        bus_request;  // unread; an undeclared output net is an error under default_nettype none

  fetcher fetcher (
    .clk(clk),
    .reset(reset),
    .pc(fetch_pc),
    .next_pc(fetch_pc_next),
    .issuing(decoder_issuing),
    .redirect(x_redirect),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .imem_addr2(imem_addr2),
    .imem_data2(imem_data2),
    .imem_addr_next(imem_addr_next),
    .imem_stall(imem_stall),
    .imem_fault(rom_fault),
    .fetch_stall(fetch_wait),
    .fault(fetch_fault),
    .out(fetcher_out)
  );
  // fetch_pc ownership: the guess is D's, the override is X's.
  assign fetch_pc_next = x_redirect      ? x_redirect_target :
                         !decoder_issuing ? fetch_pc :
                                            decoder_predicted_pc;
  always_ff @(posedge clk) fetch_pc <= reset ? 32'b0 : fetch_pc_next;

  decoder decoder (
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .x_busy(x_busy),
    .executor_out(executor_out),
    .fetch_stall(fetch_wait),
    .bus_wait(bus_wait),
    .bus_request(bus_request),
    .imem_fault(fetch_fault),
    .accessor_out_valid(accessor_out_valid),
    .issuing(decoder_issuing),
    .predicted_pc(decoder_predicted_pc),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .interrupt_pending(interrupt_pending),
    .x_redirect(x_redirect),
    .out(dx_out)
  );

  executor executor (
    .clk(clk),
    .reset(reset),
    .in(dx_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .x_busy(x_busy),
    .atomic_addr(atomic_addr),
    .atomic_supported(atomic_supported),
    .csr_addr(csr_addr),
    .csr_ren(csr_ren),
    .csr_wen(csr_wen),
    .csr_wdata(csr_wdata),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .trap_tval(trap_tval),
    .mret_entry(mret_entry),
    .mtvec(mtvec),
    .mepc(mepc),
    .redirect(x_redirect),
    .redirect_target(x_redirect_target),
    .launch(decoder_out),
    .out(executor_out)
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
  logic [31:0] fetcher_out_pc;
  assign fetcher_out_pc = fetcher_out.pc;

  logic [31:0] past_fetch_pc, prev_mtvec, prev_mepc, prev_predicted_pc, prev_redirect_target;
  logic prev_reset, prev_issuing, prev_uncompressed, prev_x_redirect;
  logic prev_trap_entry, prev_mret_entry;
  always_ff @(posedge clk) begin
    past_fetch_pc        <= fetch_pc;
    prev_reset            <= reset;
    prev_issuing           <= decoder_issuing;
    prev_uncompressed      <= f_uncompressed;
    prev_x_redirect        <= x_redirect;
    prev_predicted_pc      <= decoder_predicted_pc;
    prev_redirect_target   <= x_redirect_target;
    prev_trap_entry        <= trap_entry;
    prev_mret_entry         <= mret_entry;
    prev_mtvec              <= mtvec;
    prev_mepc                <= mepc;
  end

  always_comb if (clocked && !reset) assert(fetcher_out_pc == fetch_pc);

  logic [31:0] past_imem_addr_next;
  always_ff @(posedge clk) past_imem_addr_next <= imem_addr_next;
  always_comb if (clocked) assert(imem_addr == past_imem_addr_next);

  // D's own guess, checked against the word it read: `predicted_pc` is `fetcher_pc + 2`
  // for a compressed word and `+4` for an uncompressed one.
  always_comb if (clocked && !reset)
    assert(decoder_predicted_pc == fetch_pc + (f_uncompressed ? 32'd4 : 32'd2));

  logic f_settled;
  assign f_settled = clocked && !prev_reset;

  always_ff @(posedge clk)
    if (f_settled && prev_issuing && !prev_x_redirect) begin
      assert(fetch_pc == prev_predicted_pc);
      increment_reached: cover (1'b1);
    end

  always_ff @(posedge clk)
    if (f_settled && !prev_issuing && !prev_x_redirect) assert(fetch_pc == past_fetch_pc);

  always_ff @(posedge clk)
    if (f_settled && prev_x_redirect) assert(fetch_pc == prev_redirect_target);

  always_ff @(posedge clk)
    if (f_settled && prev_trap_entry) assert(fetch_pc == prev_mtvec);
  always_ff @(posedge clk)
    if (f_settled && prev_mret_entry) assert(fetch_pc == prev_mepc);
 `endif
endmodule

`default_nettype wire
