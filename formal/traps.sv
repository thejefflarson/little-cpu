// The fetcher, the decoder and the CSR file, wired together the way rtl/littlecpu.v wires
// them, so that mtvec, mepc, mcause and mstatus are real registers rather than free
// inputs.
`default_nettype none

module traps #(
    // The data bus's map: the addresses at which some memory on it answers a plain load
    // or store.
    parameter integer      LS_TEXT_WORDS = 2048,
    parameter logic [31:0] LS_RAM_BASE   = 32'h0001_0000,
    parameter integer      LS_RAM_WORDS  = 16384,
    parameter logic [31:0] LS_TIMER_BASE = 32'h0002_0000,
    parameter logic [31:0] LS_UART_BASE  = 32'h0002_0020,
    parameter logic [31:0] LS_FLASH_BASE = 32'h0002_0028
) (
    input logic clk,
    input logic reset,
    input logic [31:0] imem_data,
    input logic [31:0] imem_data2,
    input logic [31:0] reg_rs1,
    input logic [31:0] reg_rs2,
    input executor_output executor_out,
    input logic divider_stall,
    input logic fetch_stall,
    // Free, like the other two: a hart waiting for the shared bus issues nothing, so no
    // trap is committed on that cycle either.
    input logic bus_wait,
    // Free, like everything else not instantiated here.
    input logic imem_fault,
    // The platform's answer about the address an atomic in decode would use.
    input logic atomic_supported,
    input logic accessor_out_valid,
    input logic pair_hit,
    input logic [4:0] pair_rs1,
    input logic [4:0] pair_rs2,
    // The platform's timer line, free every cycle.
    input logic irq_timer
);
  logic [31:0] pc, next_pc;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  // The address the decoder publishes for a platform to decode.
  logic [31:0] atomic_addr;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] read_rs1, read_rs2;
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata, csr_rdata;
  logic        csr_implemented;
  logic        trap_entry, mret_entry;
  // Unread here, and declared anyway: an output connected to an undeclared identifier is
  // an implicit net, which `default_nettype none` makes an error in iverilog and a
  // warning in yosys.
  logic        bus_request;
  logic [31:0] trap_cause, trap_epc, trap_tval;
  logic [31:0] mtvec_value, mepc_value;
  logic        interrupt_pending;

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

  decoder #(
    .LS_TEXT_WORDS(LS_TEXT_WORDS),
    .LS_RAM_BASE(LS_RAM_BASE),
    .LS_RAM_WORDS(LS_RAM_WORDS),
    .LS_TIMER_BASE(LS_TIMER_BASE),
    .LS_UART_BASE(LS_UART_BASE),
    .LS_FLASH_BASE(LS_FLASH_BASE)
  ) decoder (
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
    .pair_hit(pair_hit),
    .pair_rs1(pair_rs1),
    .pair_rs2(pair_rs2),
    .pair_wen(),
    .pair_write_pc(),
    .pair_write_rs1(),
    .pair_write_rs2(),
    .csr_rdata(csr_rdata),
    .csr_implemented(csr_implemented),
    .mtvec(mtvec_value),
    .mepc(mepc_value),
    .interrupt_pending(interrupt_pending),
    .pc(pc),
    .next_pc(next_pc),
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

  csrs csrs (
    .clk(clk),
    .reset(reset),
    .addr(csr_addr),
    .ren(csr_ren),
    .wen(csr_wen),
    .wdata(csr_wdata),
    .rdata(csr_rdata),
    .implemented(csr_implemented),
    .instret(instret),
    .trap_entry(trap_entry),
    .trap_cause(trap_cause),
    .trap_epc(trap_epc),
    .trap_tval(trap_tval),
    .mret_entry(mret_entry),
    .irq_timer(irq_timer),
    .mtvec_value(mtvec_value),
    .mepc_value(mepc_value),
    .interrupt_pending(interrupt_pending)
  );

 `ifdef FORMAL
  localparam logic [11:0] MSTATUS   = 12'h300;
  localparam logic [11:0] MIE       = 12'h304;
  localparam logic [11:0] MEPC      = 12'h341;
  localparam logic [11:0] MCAUSE    = 12'h342;
  localparam logic [11:0] MTVAL     = 12'h343;
  localparam logic [11:0] MIP       = 12'h344;
  localparam logic [11:0] MCYCLE    = 12'hB00;
  localparam logic [11:0] MINSTRET  = 12'hB02;
  localparam logic [11:0] MCYCLEH   = 12'hB80;
  localparam logic [11:0] MINSTRETH = 12'hB82;

  localparam logic [31:0] CAUSE_ILLEGAL     = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT  = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MIS    = 32'd4;
  localparam logic [31:0] CAUSE_LOAD_FAULT  = 32'd5;
  localparam logic [31:0] CAUSE_STORE_MIS   = 32'd6;
  localparam logic [31:0] CAUSE_STORE_FAULT = 32'd7;
  localparam logic [31:0] CAUSE_ECALL_M     = 32'd11;
  // Bit 31 says interrupt; 7 is the machine timer.
  localparam logic [31:0] CAUSE_TIMER_IRQ   = 32'h8000_0007;

  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;

  // Assumed: reset is high before the first clock edge and low forever after.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Build every guard from this module's own signals.
  logic [31:0] instr;
  assign instr = (fetcher_out.instr[1:0] == 2'b11) ? fetcher_out.instr
                                                   : {16'b0, fetcher_out.instr[15:0]};
  logic       uncompressed;
  logic [4:0] opcode;
  logic [2:0] funct3;
  assign uncompressed = instr[1:0] == 2'b11;
  assign opcode = instr[6:2];
  assign funct3 = instr[14:12];

  // `issuing` is not a port, but it is exactly this: the decoder counts a retired
  // instruction on every cycle it issues one that does not trap, and raises trap_entry on
  // every cycle it issues one that does.
  logic issuing;
  assign issuing = instret || trap_entry;

  logic hard_stall;
  assign hard_stall = divider_stall || fetch_stall || bus_wait;

  logic [31:0] prev_reg_rs1;
  fetcher_output prev_fetcher_out;
  logic        prev_issuing;
  always_ff @(posedge clk) begin
    prev_reg_rs1     <= reg_rs1;
    prev_fetcher_out <= fetcher_out;
    prev_issuing     <= issuing || reset;
  end
  always_comb if (clocked && !reset && !prev_issuing) begin
    assume(reg_rs1 == prev_reg_rs1);
    assume(fetcher_out == prev_fetcher_out);
  end

  logic [31:0] i_immediate, s_immediate;
  assign i_immediate = {{20{instr[31]}}, instr[31:20]};
  assign s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};

  logic [31:0] load_addr, store_addr;
  assign load_addr  = $signed(i_immediate) + $signed(reg_rs1);
  assign store_addr = $signed(s_immediate) + $signed(reg_rs1);

  logic is_load_op, is_store_op;
  assign is_load_op  = uncompressed && opcode == 5'b00000;
  assign is_store_op = uncompressed && opcode == 5'b01000;

  logic lw_misaligned, lh_misaligned, sw_misaligned, sh_misaligned;
  assign lw_misaligned = is_load_op && funct3 == 3'b010 && load_addr[1:0] != 2'b00;
  assign lh_misaligned = is_load_op && (funct3 == 3'b001 || funct3 == 3'b101) && load_addr[0];
  assign sw_misaligned = is_store_op && funct3 == 3'b010 && store_addr[1:0] != 2'b00;
  assign sh_misaligned = is_store_op && funct3 == 3'b001 && store_addr[0];

  localparam logic [31:0] LS_TEXT_TOP  = LS_TEXT_WORDS * 4;
  localparam logic [31:0] LS_RAM_TOP   = LS_RAM_BASE + LS_RAM_WORDS * 4;
  localparam logic [31:0] LS_TIMER_TOP = LS_TIMER_BASE + 32'd32;
  localparam logic [31:0] LS_UART_TOP  = LS_UART_BASE + 32'd8;
  localparam logic [31:0] LS_FLASH_TOP = LS_FLASH_BASE + 32'd8;

  logic [31:0] data_addr;
  logic        data_mapped;
  assign data_addr = is_store_op ? store_addr : load_addr;
  assign data_mapped = data_addr < LS_TEXT_TOP ||
                       (data_addr >= LS_RAM_BASE && data_addr < LS_RAM_TOP) ||
                       (data_addr >= LS_TIMER_BASE && data_addr < LS_TIMER_TOP) ||
                       (data_addr >= LS_UART_BASE && data_addr < LS_UART_TOP) ||
                       (data_addr >= LS_FLASH_BASE && data_addr < LS_FLASH_TOP);

  logic is_load, is_store;
  assign is_load  = is_load_op && (funct3 == 3'b000 || funct3 == 3'b001 ||
                                   funct3 == 3'b010 || funct3 == 3'b100 || funct3 == 3'b101);
  assign is_store = is_store_op && (funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b010);

  logic load_region_fault, store_region_fault;
  assign load_region_fault  = is_load  && !data_mapped && !lw_misaligned && !lh_misaligned;
  assign store_region_fault = is_store && !data_mapped && !sw_misaligned && !sh_misaligned;

  logic is_amo_op, is_lr, is_sc, is_amo, is_atomic;
  logic atomic_word_aligned, atomic_refused;
  assign is_amo_op = uncompressed && opcode == 5'b01011 && funct3 == 3'b010;
  assign is_lr = is_amo_op && instr[31:27] == 5'b00010 && instr[24:20] == 5'b0;
  assign is_sc = is_amo_op && instr[31:27] == 5'b00011;
  assign is_amo = is_amo_op && (instr[31:27] == 5'b00000 || instr[31:27] == 5'b00001 ||
                                instr[31:27] == 5'b00100 || instr[31:27] == 5'b01000 ||
                                instr[31:27] == 5'b01100 || instr[31:27] == 5'b10000 ||
                                instr[31:27] == 5'b10100 || instr[31:27] == 5'b11000 ||
                                instr[31:27] == 5'b11100);
  assign is_atomic = is_amo || is_lr || is_sc;
  assign atomic_word_aligned = reg_rs1[1:0] == 2'b00;
  assign atomic_refused = !atomic_supported && atomic_word_aligned;

  logic reserved_opcode, zero_halfword, is_illegal;
  assign reserved_opcode = uncompressed && opcode == 5'b11111;
  assign zero_halfword = instr == 32'h0000_0000;
  assign is_illegal = reserved_opcode || zero_halfword;

  logic is_ecall, is_ebreak;
  assign is_ecall  = instr == 32'h0000_0073;
  assign is_ebreak = instr == 32'h0010_0073 || instr == 32'h0000_9002;

  logic expected_trap;
  logic [31:0] expected_cause, expected_tval;
  assign expected_trap = is_illegal || is_ebreak || is_ecall ||
                         lw_misaligned || lh_misaligned || sw_misaligned || sh_misaligned ||
                         load_region_fault || store_region_fault ||
                         (is_atomic && atomic_refused);
  always_comb begin
    if (is_illegal) begin
      expected_cause = CAUSE_ILLEGAL;
      expected_tval  = instr;
    end else if (is_ebreak) begin
      expected_cause = CAUSE_BREAKPOINT;
      expected_tval  = 32'b0;
    end else if (is_ecall) begin
      expected_cause = CAUSE_ECALL_M;
      expected_tval  = 32'b0;
    end else if (lw_misaligned || lh_misaligned) begin
      expected_cause = CAUSE_LOAD_MIS;
      expected_tval  = load_addr;
    end else if (sw_misaligned || sh_misaligned) begin
      expected_cause = CAUSE_STORE_MIS;
      expected_tval  = store_addr;
    end else if (load_region_fault) begin
      expected_cause = CAUSE_LOAD_FAULT;
      expected_tval  = data_addr;
    end else if (store_region_fault) begin
      expected_cause = CAUSE_STORE_FAULT;
      expected_tval  = data_addr;
    end else if (is_lr) begin
      expected_cause = CAUSE_LOAD_FAULT;
      expected_tval  = reg_rs1;
    end else begin
      expected_cause = CAUSE_STORE_FAULT;
      expected_tval  = reg_rs1;
    end
  end

  logic cause_modelled;
  assign cause_modelled = expected_trap;

  logic must_not_trap;
  assign must_not_trap =
      (uncompressed && opcode == 5'b01100 && instr[31:25] == 7'b0 && funct3 == 3'b000) ||
      (is_load_op && funct3 == 3'b010 && load_addr[1:0] == 2'b00 && data_mapped) ||
      (is_store_op && funct3 == 3'b010 && store_addr[1:0] == 2'b00 && data_mapped) ||
      (is_atomic && atomic_supported && atomic_word_aligned);

  logic mstatus_addressed, mstatus_static;
  assign mstatus_addressed = csr_addr == MSTATUS;
  assign mstatus_static = !csr_wen && !trap_entry && !mret_entry;

  logic counter_ticking;
  assign counter_ticking = csr_addr == MCYCLE || csr_addr == MCYCLEH ||
      csr_addr == MIP ||
      (instret && (csr_addr == MINSTRET || csr_addr == MINSTRETH));

  logic csr_written_by_trap;
  assign csr_written_by_trap =
      (trap_entry && (csr_addr == MEPC || csr_addr == MCAUSE || csr_addr == MTVAL ||
                      mstatus_addressed)) ||
      (mret_entry && mstatus_addressed);

  logic [31:0] past_pc, prev_mtvec, prev_mepc, prev_rdata, prev_cause, prev_tval;
  logic [11:0] prev_csr_addr;
  logic prev_reset, prev_trap_entry, prev_mret_entry, prev_csr_wen;
  logic prev_cause_modelled, prev_counter_ticking, prev_written_by_trap;
  logic prev_mstatus_addressed, prev_mstatus_static;
  logic prev_interrupt_pending, prev_interrupt_entry, prev_imem_fault;
  logic [31:0] prev2_rdata;
  logic prev2_reset, prev2_mstatus_addressed, prev2_mstatus_static;
  always_ff @(posedge clk) begin
    past_pc                <= pc;
    prev_reset             <= reset;
    prev_mtvec             <= mtvec_value;
    prev_mepc              <= mepc_value;
    prev_rdata             <= csr_rdata;
    prev_csr_addr          <= csr_addr;
    prev_csr_wen           <= csr_wen;
    prev_trap_entry        <= trap_entry;
    prev_mret_entry        <= mret_entry;
    prev_cause             <= expected_cause;
    prev_tval              <= expected_tval;
    prev_cause_modelled    <= cause_modelled;
    prev_counter_ticking   <= counter_ticking;
    prev_written_by_trap   <= csr_written_by_trap;
    prev_mstatus_addressed <= mstatus_addressed;
    prev_mstatus_static    <= mstatus_static;
    prev_interrupt_pending <= interrupt_pending;
    prev_imem_fault        <= imem_fault;
    prev_interrupt_entry   <= trap_entry && interrupt_pending;

    prev2_rdata             <= prev_rdata;
    prev2_reset             <= prev_reset;
    prev2_mstatus_addressed <= prev_mstatus_addressed;
    prev2_mstatus_static    <= prev_mstatus_static;
  end

  logic addr_held;
  assign addr_held = csr_addr == prev_csr_addr;

  logic settled, settled2;
  assign settled = clocked && !prev_reset;
  assign settled2 = settled && !prev2_reset;

  always_comb if (clocked && hard_stall) assert(!issuing);
  always_comb if (clocked && !issuing) assert(!csr_wen && !csr_ren && !mret_entry);

  always_comb if (settled && !prev_csr_wen && !prev_trap_entry) begin
    assert(mtvec_value == prev_mtvec);
    assert(mepc_value == prev_mepc);
  end

  always_comb if (settled && addr_held && !prev_counter_ticking && !prev_csr_wen &&
                  !prev_written_by_trap)
    assert(csr_rdata == prev_rdata);

  always_comb if (settled && prev_trap_entry) assert(pc == prev_mtvec);
  always_comb if (settled && prev_mret_entry) assert(pc == prev_mepc);

  always_comb if (settled && prev_trap_entry) assert(mepc_value == {past_pc[31:1], 1'b0});

  always_comb if (settled && prev_trap_entry && !prev_interrupt_pending &&
                  !prev_imem_fault && prev_cause_modelled && csr_addr == MCAUSE)
    assert(csr_rdata == prev_cause);

  always_comb if (settled && prev_trap_entry && !prev_interrupt_pending &&
                  prev_imem_fault && csr_addr == MCAUSE)
    assert(csr_rdata == 32'd1);

  always_comb if (settled && prev_trap_entry && !prev_interrupt_pending &&
                  !prev_imem_fault && prev_cause_modelled && csr_addr == MTVAL)
    assert(csr_rdata == prev_tval);

  always_comb if (settled && prev_trap_entry && !prev_interrupt_pending &&
                  prev_imem_fault && csr_addr == MTVAL)
    assert(csr_rdata == past_pc);

  always_comb if (settled && prev_interrupt_entry && csr_addr == MTVAL)
    assert(csr_rdata == 32'b0);

  always_comb if (settled && prev_trap_entry && prev_mstatus_addressed && mstatus_addressed) begin
    assert(csr_rdata[3] == 1'b0);
    assert(csr_rdata[7] == prev_rdata[3]);
  end

  always_comb if (settled && prev_mret_entry && mstatus_addressed) begin
    assert(csr_rdata[7] == 1'b1);
    if (settled2 && prev2_mstatus_addressed && prev2_mstatus_static)
      assert(csr_rdata[3] == prev2_rdata[7]);
  end

  always_comb if (settled && prev_trap_entry) begin
    assert(decoder_out.rd == 5'b0);
    assert(!decoder_out.is_lb && !decoder_out.is_lbu && !decoder_out.is_lh &&
           !decoder_out.is_lhu && !decoder_out.is_lw);
    assert(!decoder_out.is_sb && !decoder_out.is_sh && !decoder_out.is_sw);
    assert(!decoder_out.is_amo);
    assert(!decoder_out.is_amoswap && !decoder_out.is_amoadd && !decoder_out.is_amoxor &&
           !decoder_out.is_amoand && !decoder_out.is_amoor && !decoder_out.is_amomin &&
           !decoder_out.is_amomax && !decoder_out.is_amominu && !decoder_out.is_amomaxu &&
           !decoder_out.is_lr && !decoder_out.is_sc);
  end

  always_comb if (clocked) assert(!(trap_entry && instret));
  always_comb if (settled && prev_trap_entry && addr_held &&
                  (csr_addr == MINSTRET || csr_addr == MINSTRETH))
    assert(csr_rdata == prev_rdata);

  always_comb if (clocked) assert(!(trap_entry && mret_entry));

  always_comb if (clocked) assert(!(trap_entry && (csr_wen || csr_ren)));

  always_comb if (clocked && issuing && expected_trap) assert(trap_entry);
  always_comb if (clocked && issuing && must_not_trap && !interrupt_pending && !imem_fault)
    assert(!trap_entry);

  always_comb if (clocked && !irq_timer) assert(!interrupt_pending);
  always_comb if (clocked && csr_addr == MIE && !csr_rdata[7]) assert(!interrupt_pending);
  always_comb if (clocked && mstatus_addressed && !csr_rdata[3]) assert(!interrupt_pending);

  always_comb if (clocked && csr_addr == MIP)
    assert(csr_rdata == {24'b0, irq_timer, 7'b0});

  always_comb if (clocked && interrupt_pending)
    assert(!instret && !csr_wen && !csr_ren && !mret_entry);
  always_comb if (settled && prev_interrupt_entry) assert(!decoder_out.valid);

  always_comb if (settled && prev_interrupt_entry)
    assert(mepc_value == {past_pc[31:1], 1'b0});

  always_comb if (settled && prev_interrupt_entry && csr_addr == MCAUSE)
    assert(csr_rdata == CAUSE_TIMER_IRQ);

  always_comb if (settled && prev_trap_entry) assert(!interrupt_pending);

  always_comb if (clocked) assert(mtvec_value[1:0] == 2'b00);
  always_comb if (clocked) assert(mepc_value[0] == 1'b0);
  always_comb if (clocked && mstatus_addressed) assert(csr_rdata[12:11] == 2'b11);
 `endif
endmodule

`default_nettype wire
