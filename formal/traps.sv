// The fetcher, D, X and the CSR file, wired together the way rtl/littlecpu.v wires them,
// so that mtvec, mepc, mcause and mstatus are real registers rather than free inputs.
`default_nettype none

module traps #(
    // The data bus's map: the addresses a plain load or store answers at.
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
    input logic imem_stall,  // the ROM's stolen-read flag, free; turned into fetch_stall
    input logic bus_wait,  // free; an ungranted hart issues nothing, so it commits no trap either
    input logic rom_fault,  // free, like everything else not instantiated here
    input logic atomic_supported,  // the platform's answer about an atomic's address
    input logic accessor_out_valid,
    input logic irq_timer  // the platform's timer line, free every cycle
);
  logic [31:0] fetch_pc, fetch_pc_next;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  logic        fetch_wait, fetch_fault, decoder_issuing, x_redirect;
  // The address X publishes for a platform to decode.
  logic [31:0] atomic_addr;
  fetcher_output fetcher_out;
  dx_output dx_out;
  decoder_output decoder_out;
  executor_output executor_out;
  logic [4:0] read_rs1, read_rs2;
  logic        x_busy;
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata, csr_rdata;
  logic        csr_implemented;
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc, trap_tval;
  logic [31:0] mtvec_value, mepc_value;
  logic        interrupt_pending;
  logic [31:0] decoder_predicted_pc;
  logic [31:0] x_redirect_target;
  // Unread here, and declared anyway: an output connected to an undeclared identifier is
  // an implicit net, which `default_nettype none` makes an error in iverilog and a
  // warning in yosys.
  logic        bus_request;

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
  // fetch_pc ownership lives in the integrator, not in either stage: the guess is D's
  // (`decoder_predicted_pc`) and the override is X's (`x_redirect`/`x_redirect_target`).
  assign fetch_pc_next = x_redirect       ? x_redirect_target :
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

  executor #(
    .LS_TEXT_WORDS(LS_TEXT_WORDS),
    .LS_RAM_BASE(LS_RAM_BASE),
    .LS_RAM_WORDS(LS_RAM_WORDS),
    .LS_TIMER_BASE(LS_TIMER_BASE),
    .LS_UART_BASE(LS_UART_BASE),
    .LS_FLASH_BASE(LS_FLASH_BASE)
  ) executor (
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
    .mtvec(mtvec_value),
    .mepc(mepc_value),
    .redirect(x_redirect),
    .redirect_target(x_redirect_target),
    .launch(decoder_out),
    .out(executor_out)
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
  // A split task defines TRAPS_SPLIT plus one TRAPS_CHECK_*; every other consumer gets all four.
 `ifndef TRAPS_SPLIT
  `define TRAPS_CHECK_PC
  `define TRAPS_CHECK_CAUSE
  `define TRAPS_CHECK_STATUS
  `define TRAPS_CHECK_QUIESCENCE
 `endif

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

  // Every guard below reads this module's own signals, never a hierarchical instance read.
  logic [31:0] instr;
  assign instr = (fetcher_out.instr[1:0] == 2'b11) ? fetcher_out.instr
                                                   : {16'b0, fetcher_out.instr[15:0]};
  logic       uncompressed;
  logic [4:0] opcode;
  logic [2:0] funct3;
  assign uncompressed = instr[1:0] == 2'b11;
  assign opcode = instr[6:2];
  assign funct3 = instr[14:12];

  logic issuing;
  assign issuing = decoder_issuing;  // already folds in every stall reason, x_busy included

  logic [31:0] dx_pc;  // the trapping instruction's own pc, stable through commit
  assign dx_pc = dx_out.pc;

  logic [31:0] prev_reg_rs1, prev_reg_rs2;
  fetcher_output prev_fetcher_out;
  logic        prev_issuing;
  logic [4:0]  dx_out_rs1, dx_out_rs2;
  assign dx_out_rs1 = dx_out.rs1;
  assign dx_out_rs2 = dx_out.rs2;
  logic [4:0]  prev_dx_rs1, prev_dx_rs2;
  always_ff @(posedge clk) begin
    prev_reg_rs1     <= reg_rs1;
    prev_reg_rs2     <= reg_rs2;
    prev_fetcher_out <= fetcher_out;
    prev_issuing     <= issuing || reset;
    prev_dx_rs1      <= dx_out_rs1;
    prev_dx_rs2      <= dx_out_rs2;
  end
  always_comb if (clocked && !reset && !prev_issuing) assume(fetcher_out == prev_fetcher_out);
  // reg_rs1 belongs to whichever register dx_out.rs1 (a captured field, not a guess)
  // names, and must not change while dx_out.rs1 itself has not, issue through commit.
  always_comb if (clocked && !reset && dx_out_rs1 == prev_dx_rs1) assume(reg_rs1 == prev_reg_rs1);
  always_comb if (clocked && !reset && dx_out_rs2 == prev_dx_rs2) assume(reg_rs2 == prev_reg_rs2);

 `ifdef PROBE_ENV_HINT
  // Only defined by the region/tval probes' own throwaway build, never by
  // components_traps' real proof: a fixed, known-unmapped address collapses the
  // 32-bit search over reg_rs1 that made those probes' basecase cost minutes a step.
  always_comb assume(reg_rs1 == 32'h0004_0000);
 `endif

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

  // Tracks c_expected_trap (dx_out-anchored, see below), not expected_trap.
  logic cause_modelled;
  assign cause_modelled = c_expected_trap;

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

  logic [31:0] past_fetch_pc, past_dx_pc, prev_mtvec, prev_mepc, prev_rdata, prev_cause, prev_tval;
  logic [11:0] prev_csr_addr;
  logic prev_reset, prev_trap_entry, prev_mret_entry, prev_csr_wen;
  logic prev_cause_modelled, prev_counter_ticking, prev_written_by_trap;
  logic prev_mstatus_addressed, prev_mstatus_static;
  logic prev_interrupt_pending, prev_interrupt_entry, prev_fetch_fault;
  logic [31:0] prev2_rdata, past2_dx_pc, prev2_cause, prev2_tval;
  logic prev2_reset, prev2_mstatus_addressed, prev2_mstatus_static;
  logic prev2_trap_entry, prev2_interrupt_pending, prev2_fetch_fault, prev2_cause_modelled;
  logic prev2_interrupt_entry;
  always_ff @(posedge clk) begin
    past_fetch_pc          <= fetch_pc;
    past_dx_pc              <= dx_pc;
    prev_reset             <= reset;
    prev_mtvec             <= mtvec_value;
    prev_mepc              <= mepc_value;
    prev_rdata              <= csr_rdata;
    prev_csr_addr          <= csr_addr;
    prev_csr_wen            <= csr_wen;
    prev_trap_entry        <= trap_entry;
    prev_mret_entry         <= mret_entry;
    prev_cause              <= c_expected_cause;
    prev_tval                <= c_expected_tval;
    prev_cause_modelled    <= cause_modelled;
    prev_counter_ticking   <= counter_ticking;
    prev_written_by_trap   <= csr_written_by_trap;
    prev_mstatus_addressed <= mstatus_addressed;
    prev_mstatus_static    <= mstatus_static;
    // Both read dx_out.is_interrupt, X's own captured decision, not CSRs' live interrupt_pending.
    prev_interrupt_pending <= dx_is_interrupt;
    prev_fetch_fault        <= dx_imem_fault;
    prev_interrupt_entry   <= trap_entry && dx_is_interrupt;

    prev2_rdata             <= prev_rdata;
    prev2_reset             <= prev_reset;
    prev2_mstatus_addressed <= prev_mstatus_addressed;
    prev2_mstatus_static    <= prev_mstatus_static;
    // A second tap: a CSR-read instruction reaches X a cycle behind trap_entry itself.
    past2_dx_pc              <= past_dx_pc;
    prev2_cause              <= prev_cause;
    prev2_tval                <= prev_tval;
    prev2_trap_entry        <= prev_trap_entry;
    prev2_interrupt_pending <= prev_interrupt_pending;
    prev2_fetch_fault        <= prev_fetch_fault;
    prev2_cause_modelled    <= prev_cause_modelled;
    prev2_interrupt_entry   <= prev_interrupt_entry;
  end

  logic addr_held;
  assign addr_held = csr_addr == prev_csr_addr;

  logic settled, settled2;
  assign settled = clocked && !prev_reset;
  assign settled2 = settled && !prev2_reset;

  // Below, X's trap_entry is checked against a model rebuilt fresh every cycle from `dx_out.instr`.
  logic        dx_valid, dx_is_interrupt, dx_imem_fault;
  logic [31:0] dx_instr;
  assign dx_valid = dx_out.valid;
  assign dx_is_interrupt = dx_out.is_interrupt;
  assign dx_imem_fault = dx_out.imem_fault;
  assign dx_instr = dx_out.instr;

  logic        c_uncompressed;
  logic [4:0]  c_opcode;
  logic [2:0]  c_funct3;
  assign c_uncompressed = dx_instr[1:0] == 2'b11;
  assign c_opcode = dx_instr[6:2];
  assign c_funct3 = dx_instr[14:12];

  logic [31:0] c_i_immediate, c_s_immediate;
  assign c_i_immediate = {{20{dx_instr[31]}}, dx_instr[31:20]};
  assign c_s_immediate = {{20{dx_instr[31]}}, dx_instr[31:25], dx_instr[11:7]};

  logic [31:0] c_load_addr, c_store_addr;
  assign c_load_addr  = $signed(c_i_immediate) + $signed(reg_rs1);
  assign c_store_addr = $signed(c_s_immediate) + $signed(reg_rs1);

  logic c_is_load_op, c_is_store_op;
  assign c_is_load_op  = c_uncompressed && c_opcode == 5'b00000;
  assign c_is_store_op = c_uncompressed && c_opcode == 5'b01000;

  logic c_lw_mis, c_lh_mis, c_sw_mis, c_sh_mis;
  assign c_lw_mis = c_is_load_op && c_funct3 == 3'b010 && c_load_addr[1:0] != 2'b00;
  assign c_lh_mis = c_is_load_op && (c_funct3 == 3'b001 || c_funct3 == 3'b101) && c_load_addr[0];
  assign c_sw_mis = c_is_store_op && c_funct3 == 3'b010 && c_store_addr[1:0] != 2'b00;
  assign c_sh_mis = c_is_store_op && c_funct3 == 3'b001 && c_store_addr[0];

  logic [31:0] c_data_addr;
  logic        c_data_mapped;
  assign c_data_addr = c_is_store_op ? c_store_addr : c_load_addr;
  assign c_data_mapped = c_data_addr < LS_TEXT_TOP ||
                         (c_data_addr >= LS_RAM_BASE && c_data_addr < LS_RAM_TOP) ||
                         (c_data_addr >= LS_TIMER_BASE && c_data_addr < LS_TIMER_TOP) ||
                         (c_data_addr >= LS_UART_BASE && c_data_addr < LS_UART_TOP) ||
                         (c_data_addr >= LS_FLASH_BASE && c_data_addr < LS_FLASH_TOP);

  logic c_is_load, c_is_store;
  assign c_is_load  = c_is_load_op && (c_funct3 == 3'b000 || c_funct3 == 3'b001 ||
                       c_funct3 == 3'b010 || c_funct3 == 3'b100 || c_funct3 == 3'b101);
  assign c_is_store = c_is_store_op && (c_funct3 == 3'b000 || c_funct3 == 3'b001 ||
                       c_funct3 == 3'b010);

  logic c_load_region_fault, c_store_region_fault;
  assign c_load_region_fault  = c_is_load  && !c_data_mapped && !c_lw_mis && !c_lh_mis;
  assign c_store_region_fault = c_is_store && !c_data_mapped && !c_sw_mis && !c_sh_mis;

  logic c_is_amo_op, c_is_lr, c_is_sc, c_is_amo, c_is_atomic;
  logic c_atomic_word_aligned, c_atomic_refused;
  assign c_is_amo_op = c_uncompressed && c_opcode == 5'b01011 && c_funct3 == 3'b010;
  assign c_is_lr = c_is_amo_op && dx_instr[31:27] == 5'b00010 && dx_instr[24:20] == 5'b0;
  assign c_is_sc = c_is_amo_op && dx_instr[31:27] == 5'b00011;
  assign c_is_amo = c_is_amo_op && (dx_instr[31:27] == 5'b00000 || dx_instr[31:27] == 5'b00001 ||
                     dx_instr[31:27] == 5'b00100 || dx_instr[31:27] == 5'b01000 ||
                     dx_instr[31:27] == 5'b01100 || dx_instr[31:27] == 5'b10000 ||
                     dx_instr[31:27] == 5'b10100 || dx_instr[31:27] == 5'b11000 ||
                     dx_instr[31:27] == 5'b11100);
  assign c_is_atomic = c_is_amo || c_is_lr || c_is_sc;
  assign c_atomic_word_aligned = reg_rs1[1:0] == 2'b00;
  assign c_atomic_refused = !atomic_supported && c_atomic_word_aligned;

  logic c_reserved_opcode, c_zero_halfword, c_is_illegal;
  assign c_reserved_opcode = c_uncompressed && c_opcode == 5'b11111;
  assign c_zero_halfword = dx_instr == 32'h0000_0000;
  assign c_is_illegal = c_reserved_opcode || c_zero_halfword;

  logic c_is_ecall, c_is_ebreak;
  assign c_is_ecall  = dx_instr == 32'h0000_0073;
  assign c_is_ebreak = dx_instr == 32'h0010_0073 || dx_instr == 32'h0000_9002;

  logic c_expected_trap, c_must_not_trap;
  assign c_expected_trap = c_is_illegal || c_is_ebreak || c_is_ecall ||
                           c_lw_mis || c_lh_mis || c_sw_mis || c_sh_mis ||
                           c_load_region_fault || c_store_region_fault ||
                           (c_is_atomic && c_atomic_refused);
  assign c_must_not_trap =
      (c_uncompressed && c_opcode == 5'b01100 && dx_instr[31:25] == 7'b0 &&
       c_funct3 == 3'b000) ||
      (c_is_load_op && c_funct3 == 3'b010 && c_load_addr[1:0] == 2'b00 && c_data_mapped) ||
      (c_is_store_op && c_funct3 == 3'b010 && c_store_addr[1:0] == 2'b00 && c_data_mapped) ||
      (c_is_atomic && atomic_supported && c_atomic_word_aligned);

  // Mirrors expected_cause/expected_tval's case statement against dx_instr, not `instr`.
  logic [31:0] c_expected_cause, c_expected_tval;
  always_comb begin
    if (c_is_illegal) begin
      c_expected_cause = CAUSE_ILLEGAL;
      c_expected_tval  = dx_instr;
    end else if (c_is_ebreak) begin
      c_expected_cause = CAUSE_BREAKPOINT;
      c_expected_tval  = 32'b0;
    end else if (c_is_ecall) begin
      c_expected_cause = CAUSE_ECALL_M;
      c_expected_tval  = 32'b0;
    end else if (c_lw_mis || c_lh_mis) begin
      c_expected_cause = CAUSE_LOAD_MIS;
      c_expected_tval  = c_load_addr;
    end else if (c_sw_mis || c_sh_mis) begin
      c_expected_cause = CAUSE_STORE_MIS;
      c_expected_tval  = c_store_addr;
    end else if (c_load_region_fault) begin
      c_expected_cause = CAUSE_LOAD_FAULT;
      c_expected_tval  = c_data_addr;
    end else if (c_store_region_fault) begin
      c_expected_cause = CAUSE_STORE_FAULT;
      c_expected_tval  = c_data_addr;
    end else if (c_is_lr) begin
      c_expected_cause = CAUSE_LOAD_FAULT;
      c_expected_tval  = reg_rs1;
    end else begin
      c_expected_cause = CAUSE_STORE_FAULT;
      c_expected_tval  = reg_rs1;
    end
  end

  // Named continuous assigns, not struct-field reads inside always_* below (ADR-0037).
  logic [30:0] past_fetch_pc_hi, past_dx_pc_hi;
  assign past_fetch_pc_hi = past_fetch_pc[31:1];
  assign past_dx_pc_hi = past_dx_pc[31:1];  // mepc saves dx_out.pc, not the advancing fetch_pc
  logic csr_rdata_bit3, csr_rdata_bit7, prev_rdata_bit3, prev2_rdata_bit7;
  logic [1:0] csr_rdata_hi;
  assign csr_rdata_bit3 = csr_rdata[3];
  assign csr_rdata_bit7 = csr_rdata[7];
  assign csr_rdata_hi = csr_rdata[12:11];
  assign prev_rdata_bit3 = prev_rdata[3];
  assign prev2_rdata_bit7 = prev2_rdata[7];
  logic [1:0] mtvec_lo;
  logic       mepc_bit0;
  assign mtvec_lo = mtvec_value[1:0];
  assign mepc_bit0 = mepc_value[0];
  logic        decoder_out_valid;
  logic [4:0]  decoder_out_rd;
  logic decoder_out_is_lb, decoder_out_is_lbu, decoder_out_is_lh, decoder_out_is_lhu,
    decoder_out_is_lw, decoder_out_is_sb, decoder_out_is_sh, decoder_out_is_sw,
    decoder_out_is_amo, decoder_out_is_amoswap, decoder_out_is_amoadd, decoder_out_is_amoxor,
    decoder_out_is_amoand, decoder_out_is_amoor, decoder_out_is_amomin, decoder_out_is_amomax,
    decoder_out_is_amominu, decoder_out_is_amomaxu, decoder_out_is_lr, decoder_out_is_sc;
  assign decoder_out_valid = decoder_out.valid;
  assign decoder_out_rd = decoder_out.rd;
  assign decoder_out_is_lb = decoder_out.is_lb;
  assign decoder_out_is_lbu = decoder_out.is_lbu;
  assign decoder_out_is_lh = decoder_out.is_lh;
  assign decoder_out_is_lhu = decoder_out.is_lhu;
  assign decoder_out_is_lw = decoder_out.is_lw;
  assign decoder_out_is_sb = decoder_out.is_sb;
  assign decoder_out_is_sh = decoder_out.is_sh;
  assign decoder_out_is_sw = decoder_out.is_sw;
  assign decoder_out_is_amo = decoder_out.is_amo;
  assign decoder_out_is_amoswap = decoder_out.is_amoswap;
  assign decoder_out_is_amoadd = decoder_out.is_amoadd;
  assign decoder_out_is_amoxor = decoder_out.is_amoxor;
  assign decoder_out_is_amoand = decoder_out.is_amoand;
  assign decoder_out_is_amoor = decoder_out.is_amoor;
  assign decoder_out_is_amomin = decoder_out.is_amomin;
  assign decoder_out_is_amomax = decoder_out.is_amomax;
  assign decoder_out_is_amominu = decoder_out.is_amominu;
  assign decoder_out_is_amomaxu = decoder_out.is_amomaxu;
  assign decoder_out_is_lr = decoder_out.is_lr;
  assign decoder_out_is_sc = decoder_out.is_sc;

 `ifdef TRAPS_CHECK_PC
  // Not "!issuing": D holds back for a whole serializing CSR/mret cycle in X. What's
  // invariant is that nothing commits from an empty slot.
  always_comb if (clocked && !dx_valid) assert(!csr_wen && !csr_ren && !mret_entry);

  always_comb if (settled && !prev_csr_wen && !prev_trap_entry) begin
    assert(mtvec_value == prev_mtvec);
    assert(mepc_value == prev_mepc);
  end

  always_comb if (settled && addr_held && !prev_counter_ticking && !prev_csr_wen &&
                  !prev_written_by_trap)
    assert(csr_rdata == prev_rdata);

  always_comb if (settled && prev_trap_entry) assert(fetch_pc == prev_mtvec);
  always_comb if (settled && prev_mret_entry) assert(fetch_pc == prev_mepc);

  always_comb if (settled && prev_trap_entry) assert(mepc_value == {past_dx_pc_hi, 1'b0});
 `endif

 `ifdef TRAPS_CHECK_CAUSE
  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  !prev2_fetch_fault && prev2_cause_modelled && csr_addr == MCAUSE)
    assert(csr_rdata == prev2_cause);

  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  prev2_fetch_fault && csr_addr == MCAUSE)
    assert(csr_rdata == 32'd1);

  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  !prev2_fetch_fault && prev2_cause_modelled && csr_addr == MTVAL)
    assert(csr_rdata == prev2_tval);

  // past2_dx_pc, not fetch_pc's own tap: fetch has moved past the word that faulted.
  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  prev2_fetch_fault && csr_addr == MTVAL)
    assert(csr_rdata == past2_dx_pc);

  always_comb if (settled2 && prev2_interrupt_entry && csr_addr == MTVAL)
    assert(csr_rdata == 32'b0);

  // Each guard below was unreachable under prev_; traps_cover.sby proves prev2_ isn't.
  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  !prev2_fetch_fault && prev2_cause_modelled && csr_addr == MCAUSE)
    mcause_normal_reached: cover(1'b1);
  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  prev2_fetch_fault && csr_addr == MCAUSE)
    mcause_fetch_fault_reached: cover(1'b1);
  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  !prev2_fetch_fault && prev2_cause_modelled && csr_addr == MTVAL)
    mtval_normal_reached: cover(1'b1);
  always_comb if (settled2 && prev2_trap_entry && !prev2_interrupt_pending &&
                  prev2_fetch_fault && csr_addr == MTVAL)
    mtval_fetch_fault_reached: cover(1'b1);
  always_comb if (settled2 && prev2_interrupt_entry && csr_addr == MTVAL)
    mtval_interrupt_reached: cover(1'b1);
  always_comb if (settled2 && prev2_interrupt_entry && csr_addr == MCAUSE)
    mcause_interrupt_reached: cover(1'b1);
 `endif

 `ifdef TRAPS_CHECK_STATUS
  always_comb if (settled && prev_trap_entry && prev_mstatus_addressed && mstatus_addressed) begin
    assert(csr_rdata_bit3 == 1'b0);
    assert(csr_rdata_bit7 == prev_rdata_bit3);
  end

  always_comb if (settled && prev_mret_entry && mstatus_addressed) begin
    assert(csr_rdata_bit7 == 1'b1);
    if (settled2 && prev2_mstatus_addressed && prev2_mstatus_static)
      assert(csr_rdata_bit3 == prev2_rdata_bit7);
  end
 `endif

 `ifdef TRAPS_CHECK_QUIESCENCE
  always_comb if (settled && prev_trap_entry) begin
    assert(decoder_out_rd == 5'b0);
    assert(!decoder_out_is_lb && !decoder_out_is_lbu && !decoder_out_is_lh &&
           !decoder_out_is_lhu && !decoder_out_is_lw);
    assert(!decoder_out_is_sb && !decoder_out_is_sh && !decoder_out_is_sw);
    assert(!decoder_out_is_amo);
    assert(!decoder_out_is_amoswap && !decoder_out_is_amoadd && !decoder_out_is_amoxor &&
           !decoder_out_is_amoand && !decoder_out_is_amoor && !decoder_out_is_amomin &&
           !decoder_out_is_amomax && !decoder_out_is_amominu && !decoder_out_is_amomaxu &&
           !decoder_out_is_lr && !decoder_out_is_sc);
  end

  always_comb if (clocked) assert(!(trap_entry && instret));
  always_comb if (settled && prev_trap_entry && addr_held &&
                  (csr_addr == MINSTRET || csr_addr == MINSTRETH))
    assert(csr_rdata == prev_rdata);
 `endif

 `ifdef TRAPS_CHECK_PC
  always_comb if (clocked) assert(!(trap_entry && mret_entry));

  always_comb if (clocked) assert(!(trap_entry && (csr_wen || csr_ren)));
 `endif

 `ifdef TRAPS_CHECK_QUIESCENCE
  // held_* tracks the instruction X holds in `in`, checked on its settling cycle.
  always_comb
    if (clocked && dx_valid && !x_busy && !dx_is_interrupt && !dx_imem_fault && c_expected_trap)
      assert(trap_entry);
  always_comb
    if (clocked && dx_valid && !x_busy && !dx_is_interrupt && !dx_imem_fault && c_must_not_trap)
      assert(!trap_entry);
 `endif

 `ifdef TRAPS_CHECK_STATUS
  always_comb if (clocked && !irq_timer) assert(!interrupt_pending);
  always_comb if (clocked && csr_addr == MIE && !csr_rdata_bit7) assert(!interrupt_pending);
  always_comb if (clocked && mstatus_addressed && !csr_rdata_bit3) assert(!interrupt_pending);

  always_comb if (clocked && csr_addr == MIP)
    assert(csr_rdata == {24'b0, irq_timer, 7'b0});
 `endif

 `ifdef TRAPS_CHECK_QUIESCENCE
  // Not "interrupt_pending -> nothing commits": X may still be settling a pre-pending
  // instruction. What's invariant -- an interrupt commits nothing of its own -- is
  // covered by trap_entry's own launch flags in executor.v and the instret check below.
  always_comb if (settled && prev_interrupt_entry) assert(!decoder_out_valid);

  always_comb if (settled && prev_interrupt_entry)
    assert(mepc_value == {past_dx_pc_hi, 1'b0});
 `endif

 `ifdef TRAPS_CHECK_CAUSE
  always_comb if (settled2 && prev2_interrupt_entry && csr_addr == MCAUSE)
    assert(csr_rdata == CAUSE_TIMER_IRQ);
 `endif

 `ifdef TRAPS_CHECK_STATUS
  always_comb if (settled && prev_trap_entry) assert(!interrupt_pending);
 `endif

 `ifdef TRAPS_CHECK_PC
  always_comb if (clocked) assert(mtvec_lo == 2'b00);
  always_comb if (clocked) assert(mepc_bit0 == 1'b0);
  always_comb if (clocked && mstatus_addressed) assert(csr_rdata_hi == 2'b11);
 `endif
 `endif
endmodule

`default_nettype wire
