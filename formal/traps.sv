// The fetcher, the decoder and the CSR file, wired together the way
// rtl/littlecpu.v wires them, so that mtvec, mepc, mcause and mstatus are real
// registers rather than free inputs. That is the whole reason this file exists.
// The generated riscv-formal checks stop comparing values the moment an
// instruction traps -- the instruction check keeps only the trap flag itself --
// and the two pc checks accept any target the core reports, mtvec of 0
// included. So nothing else in the tree says where a trap goes or what it
// writes.
//
// Keep reading the RTL without -formal for this task; formal/components.sby
// does. With -formal the decoder brings its own assume(in.pc == pc) along, and
// the fetcher instance below is what answers that question here.
//
// mcause, mstatus and the two counters are internal to rtl/csrs.v. They are read
// here the way software reads them: `csr_addr` is instr[31:20] every cycle,
// whatever the instruction, and `csr_rdata` answers it combinationally. The
// instruction word is a free input, so the solver can point that address
// anywhere it likes on any cycle -- including the cycle after a trap.
`default_nettype none

module traps (
    input logic clk,
    input logic reset,
    input logic [31:0] imem_data,
    input logic [31:0] imem_data2,
    input logic [31:0] reg_rs1,
    input logic [31:0] reg_rs2,
    input executor_output executor_out,
    input logic divider_stall,
    input logic fetch_stall,
    input logic accessor_out_valid,
    // The platform's timer line, free every cycle. rtl/csrs.v decides what to
    // do with it, so `interrupt_pending` below is a real signal of this design
    // rather than something the solver picks -- which is what lets the mie and
    // mstatus.MIE gates be asserted rather than assumed.
    input logic irq_timer
);
  logic [31:0] pc, next_pc;
  logic [31:0] imem_addr, imem_addr2, imem_addr_next;
  fetcher_output fetcher_out;
  decoder_output decoder_out;
  logic [4:0] read_rs1, read_rs2;
  logic [11:0] csr_addr;
  logic        csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata, csr_rdata;
  logic        csr_implemented;
  logic        trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc;
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

  decoder decoder (
    .clk(clk),
    .reset(reset),
    .in(fetcher_out),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stall),
    .fetch_stall(fetch_stall),
    .accessor_out_valid(accessor_out_valid),
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
  localparam logic [11:0] MIP       = 12'h344;
  localparam logic [11:0] MCYCLE    = 12'hB00;
  localparam logic [11:0] MINSTRET  = 12'hB02;
  localparam logic [11:0] MCYCLEH   = 12'hB80;
  localparam logic [11:0] MINSTRETH = 12'hB82;

  localparam logic [31:0] CAUSE_ILLEGAL    = 32'd2;
  localparam logic [31:0] CAUSE_BREAKPOINT = 32'd3;
  localparam logic [31:0] CAUSE_LOAD_MIS   = 32'd4;
  localparam logic [31:0] CAUSE_STORE_MIS  = 32'd6;
  localparam logic [31:0] CAUSE_ECALL_M    = 32'd11;
  // Bit 31 says interrupt; 7 is the machine timer.
  localparam logic [31:0] CAUSE_TIMER_IRQ  = 32'h8000_0007;

  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;

  // Assumed: reset is high before the first clock edge and low forever after.
  // Every harness in this tree drives it that way, and rtl/littlesoc.v holds it
  // over a power-on counter. Nothing proves it, so this is a convention.
  //
  // It carries more weight here than it does in formal/pcloop.sv. mtvec, mepc,
  // mcause and mstatus have no initial value in the RTL, so without a reset the
  // first step of the base case starts them anywhere, and the two WARL
  // assertions below would fail on a register nothing had written yet.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Build every guard from this module's own signals. Do not write
  // `decoder.trap_pending` or anything like it: yosys does not reach into the
  // instance, it declares a new undriven wire with that name and the solver
  // picks its value, so the proof passes having asked nothing.
  //
  // Mask the upper half exactly as the decoder does. A compressed instruction
  // sits in the low 16 bits and the high 16 are the next instruction in memory.
  logic [31:0] instr;
  assign instr = (fetcher_out.instr[1:0] == 2'b11) ? fetcher_out.instr
                                                   : {16'b0, fetcher_out.instr[15:0]};
  logic       uncompressed;
  logic [4:0] opcode;
  logic [2:0] funct3;
  assign uncompressed = instr[1:0] == 2'b11;
  assign opcode = instr[6:2];
  assign funct3 = instr[14:12];

  // `issuing` is not a port, but it is exactly this: the decoder counts a
  // retired instruction on every cycle it issues one that does not trap, and
  // raises trap_entry on every cycle it issues one that does. Nothing else
  // raises either.
  logic issuing;
  assign issuing = instret || trap_entry;

  // The three stall reasons that arrive as inputs. Each one on its own forces
  // the decoder to hold, so this is a sufficient condition for a stall and
  // never a necessary one -- the hazard, serialization and operand-fetch
  // reasons are decided inside the decoder and are not visible here.
  logic hard_stall;
  assign hard_stall = divider_stall || fetch_stall;

  // Which instructions must trap, and with which cause. Written from the ISA,
  // not transcribed from rtl/decoder.v: each encoding below is one this core
  // must either execute or fault on, and the cause is the one the privileged
  // spec names. A decoder that changed its mind about any of them disagrees
  // with this, which is the point of writing it out a second way.
  //
  // The list is deliberately small. Working out whether an arbitrary word is
  // legal is most of decode, and a copy of decode makes a poor oracle for
  // decode.
  logic [31:0] i_immediate, s_immediate;
  assign i_immediate = {{20{instr[31]}}, instr[31:20]};
  assign s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};

  // Each address is its own statement with both operands marked signed. A
  // signed sum written as an arm of a conditional takes its signedness from the
  // other arms, and that has silently produced unsigned arithmetic in this repo
  // twice.
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

  // Opcode 7'b1111111 is reserved for instructions longer than 32 bits, and an
  // all-zero halfword is the encoding the C extension defines as illegal. Both
  // are illegal in every conforming RV32 implementation, so neither depends on
  // what this core chose to decode.
  logic reserved_opcode, zero_halfword, is_illegal;
  assign reserved_opcode = uncompressed && opcode == 5'b11111;
  assign zero_halfword = instr == 32'h0000_0000;
  assign is_illegal = reserved_opcode || zero_halfword;

  logic is_ecall, is_ebreak;
  assign is_ecall  = instr == 32'h0000_0073;
  assign is_ebreak = instr == 32'h0010_0073 || instr == 32'h0000_9002;

  // The order is illegal, breakpoint, environment call, load misaligned, store
  // misaligned. No two of the terms above can hold at once -- a reserved opcode
  // is not a load -- so the chain states the order rather than resolving
  // anything. Make two causes overlap and this is what decides which one the
  // core is allowed to report.
  logic expected_trap;
  logic [31:0] expected_cause;
  assign expected_trap = is_illegal || is_ebreak || is_ecall ||
                         lw_misaligned || lh_misaligned || sw_misaligned || sh_misaligned;
  always_comb begin
    if (is_illegal) expected_cause = CAUSE_ILLEGAL;
    else if (is_ebreak) expected_cause = CAUSE_BREAKPOINT;
    else if (is_ecall) expected_cause = CAUSE_ECALL_M;
    else if (lw_misaligned || lh_misaligned) expected_cause = CAUSE_LOAD_MIS;
    else expected_cause = CAUSE_STORE_MIS;
  end

  // The other direction. Without these a core that trapped on everything would
  // satisfy most of this file.
  logic must_not_trap;
  assign must_not_trap =
      (uncompressed && opcode == 5'b01100 && instr[31:25] == 7'b0 && funct3 == 3'b000) ||
      (is_load_op && funct3 == 3'b010 && load_addr[1:0] == 2'b00) ||
      (is_store_op && funct3 == 3'b010 && store_addr[1:0] == 2'b00);

  // mstatus changes on three edges and no others: a write to it, a trap and an
  // mret. The write term is any write, not just one to this address -- coarser
  // than it needs to be, which only narrows the cycles the mret assertion below
  // fires on.
  logic mstatus_addressed, mstatus_static;
  assign mstatus_addressed = csr_addr == MSTATUS;
  assign mstatus_static = !csr_wen && !trap_entry && !mret_entry;

  // These change without any instruction asking, so a held address reading one
  // of them says nothing about side effects. mcycle counts every cycle;
  // minstret counts only the cycles an instruction retires, so it is excluded
  // only on those; mip is a live view of the platform's interrupt lines, which
  // are free inputs here and are asserted about separately below.
  logic counter_ticking;
  assign counter_ticking = csr_addr == MCYCLE || csr_addr == MCYCLEH ||
      csr_addr == MIP ||
      (instret && (csr_addr == MINSTRET || csr_addr == MINSTRETH));

  // Trap entry writes mepc, mcause and mstatus. mret writes mstatus. Every
  // other address must read back the same value it read last cycle.
  logic csr_written_by_trap;
  assign csr_written_by_trap =
      (trap_entry && (csr_addr == MEPC || csr_addr == MCAUSE || mstatus_addressed)) ||
      (mret_entry && mstatus_addressed);

  logic [31:0] past_pc, prev_mtvec, prev_mepc, prev_rdata, prev_cause;
  logic [11:0] prev_csr_addr;
  logic prev_reset, prev_trap_entry, prev_mret_entry, prev_csr_wen;
  logic prev_expected_trap, prev_counter_ticking, prev_written_by_trap;
  logic prev_mstatus_addressed, prev_mstatus_static;
  logic prev_interrupt_pending, prev_interrupt_entry;
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
    prev_expected_trap     <= expected_trap;
    prev_counter_ticking   <= counter_ticking;
    prev_written_by_trap   <= csr_written_by_trap;
    prev_mstatus_addressed <= mstatus_addressed;
    prev_mstatus_static    <= mstatus_static;
    prev_interrupt_pending <= interrupt_pending;
    prev_interrupt_entry   <= trap_entry && interrupt_pending;

    prev2_rdata             <= prev_rdata;
    prev2_reset             <= prev_reset;
    prev2_mstatus_addressed <= prev_mstatus_addressed;
    prev2_mstatus_static    <= prev_mstatus_static;
  end

  // The address held across an edge, so that two reads of csr_rdata are two
  // reads of the same register.
  logic addr_held;
  assign addr_held = csr_addr == prev_csr_addr;

  // Reset holds only before the first edge, so the CSR registers carry no
  // defined value until then and neither does any history register above.
  // Every assertion that compares two cycles waits for this.
  logic settled, settled2;
  assign settled = clocked && !prev_reset;
  assign settled2 = settled && !prev2_reset;

  // Nothing raises a CSR enable, a retire or a redirect on a cycle the decoder
  // did not issue an instruction. This is the interlock, and it is what a change
  // to the stall protocol would break without anything else noticing: a trap
  // committed on a stalled cycle is a trap taken twice.
  always_comb if (clocked && hard_stall) assert(!issuing);
  always_comb if (clocked && !issuing) assert(!csr_wen && !csr_ren && !mret_entry);

  // The other half of the same property, and stated on the write enables rather
  // than on the stall, so it covers the three stall reasons the decoder keeps to
  // itself as well. mtvec moves only through a write to it; mepc through a write
  // or a trap.
  always_comb if (settled && !prev_csr_wen && !prev_trap_entry) begin
    assert(mtvec_value == prev_mtvec);
    assert(mepc_value == prev_mepc);
  end

  // A CSR reads back what it read last cycle unless one of the three write
  // paths named it. This is one assertion over every register the read mux
  // answers for, so a fourth way to change CSR state has to come through a port
  // this file already watches.
  always_comb if (settled && addr_held && !prev_counter_ticking && !prev_csr_wen &&
                  !prev_written_by_trap)
    assert(csr_rdata == prev_rdata);

  // Where the trap goes. mtvec is the register the CSR file held when the
  // trapping instruction issued, not a free input.
  always_comb if (settled && prev_trap_entry) assert(pc == prev_mtvec);
  always_comb if (settled && prev_mret_entry) assert(pc == prev_mepc);

  // mepc is the faulting instruction's own address with bit 0 cleared, which is
  // what lets a handler read the instruction and resume past it. Bit 1 survives
  // because a compressed instruction can sit at a two-byte address.
  always_comb if (settled && prev_trap_entry) assert(mepc_value == {past_pc[31:1], 1'b0});

  // `!prev_interrupt_pending` because an interrupted instruction did not
  // execute, so whatever it would have faulted on is not what happened. Its own
  // cause is asserted below.
  always_comb if (settled && prev_trap_entry && !prev_interrupt_pending &&
                  prev_expected_trap && csr_addr == MCAUSE)
    assert(csr_rdata == prev_cause);

  // MIE moves into MPIE and interrupts go off. Both halves need the value
  // mstatus held before the trap, so this fires only when the trapping
  // instruction happened to address mstatus -- the instruction word is free, so
  // the solver can always arrange that.
  always_comb if (settled && prev_trap_entry && prev_mstatus_addressed && mstatus_addressed) begin
    assert(csr_rdata[3] == 1'b0);
    assert(csr_rdata[7] == prev_rdata[3]);
  end

  // mret pops the pair back. MPIE is observable on the cycle after, but MIE
  // needs the value from before the mret, and an mret's own instruction word
  // addresses 0x302 rather than mstatus. So this reaches one cycle further
  // back, and holds mstatus still across the cycle in between.
  always_comb if (settled && prev_mret_entry && mstatus_addressed) begin
    assert(csr_rdata[7] == 1'b1);
    if (settled2 && prev2_mstatus_addressed && prev2_mstatus_static)
      assert(csr_rdata[3] == prev2_rdata[7]);
  end

  // A trapping instruction issues and reaches the end of the pipeline having
  // done nothing. No register write, and no memory access for the accessor to
  // start.
  always_comb if (settled && prev_trap_entry) begin
    assert(decoder_out.rd == 5'b0);
    assert(!decoder_out.is_lb && !decoder_out.is_lbu && !decoder_out.is_lh &&
           !decoder_out.is_lhu && !decoder_out.is_lw);
    assert(!decoder_out.is_sb && !decoder_out.is_sh && !decoder_out.is_sw);
  end

  // minstret counts instructions that retired, and a trapping one did not.
  // Asserted twice: once on the count enable, and once on the register, because
  // a counter that ticked from somewhere else would satisfy the first.
  always_comb if (clocked) assert(!(trap_entry && instret));
  always_comb if (settled && prev_trap_entry && addr_held &&
                  (csr_addr == MINSTRET || csr_addr == MINSTRETH))
    assert(csr_rdata == prev_rdata);

  always_comb if (clocked) assert(!(trap_entry && mret_entry));

  // A trapping instruction commits no CSR access, whatever else it was. Today
  // this is belt to the address decode's braces: an access can only trap by
  // naming an address that is read-only or not implemented, and rtl/csrs.v
  // writes neither. Drop the gate and nothing about the architectural state
  // changes, so this assertion is the only thing that would say so.
  always_comb if (clocked) assert(!(trap_entry && (csr_wen || csr_ren)));

  // The encodings the ISA fixes: these fault, those do not, whatever else the
  // decoder decides about them. The second one carries `!interrupt_pending`
  // because an interrupt on the cycle a harmless instruction would have issued
  // is a redirect the instruction had no part in.
  always_comb if (clocked && issuing && expected_trap) assert(trap_entry);
  always_comb if (clocked && issuing && must_not_trap && !interrupt_pending)
    assert(!trap_entry);

  // ---- the machine timer interrupt ----------------------------------------
  //
  // riscv-formal ships no model of any of this at the pin -- its two pc checks
  // read `rvfi_intr` only to stop expecting pc continuity, and no check names
  // mie, mip, mstatus or an interrupt cause. So these assertions are the oracle,
  // not a second opinion.

  // Nothing arms without a source, an enable and the global enable, and each is
  // read the way software reads it rather than out of the CSR file's internals.
  always_comb if (clocked && !irq_timer) assert(!interrupt_pending);
  always_comb if (clocked && csr_addr == MIE && !csr_rdata[7]) assert(!interrupt_pending);
  always_comb if (clocked && mstatus_addressed && !csr_rdata[3]) assert(!interrupt_pending);

  // mip.MTIP is the platform line and nothing else; MSIP and MEIP read zero
  // because this platform has neither source.
  always_comb if (clocked && csr_addr == MIP)
    assert(csr_rdata == {24'b0, irq_timer, 7'b0});

  // The instruction the interrupt displaced committed nothing. It did not
  // retire, it wrote no CSR, it did not `mret`, and it never reached the
  // executor -- so there is no state a later cycle has to take back, which is
  // the whole reason an asynchronous event can be committed in decode.
  always_comb if (clocked && interrupt_pending)
    assert(!instret && !csr_wen && !csr_ren && !mret_entry);
  always_comb if (settled && prev_interrupt_entry) assert(!decoder_out.valid);

  // Its address is in mepc, so `mret` re-executes it. The general form is
  // asserted further up against `past_pc`; this says the interrupt is not an
  // exception to it.
  always_comb if (settled && prev_interrupt_entry)
    assert(mepc_value == {past_pc[31:1], 1'b0});

  always_comb if (settled && prev_interrupt_entry && csr_addr == MCAUSE)
    assert(csr_rdata == CAUSE_TIMER_IRQ);

  // What bounds the response. Entry clears MIE on the same edge it redirects,
  // so the cycle after cannot take a second interrupt and nothing re-arms until
  // an `mret` or an explicit write to mstatus. Together with the decoder's own
  // proof that an armed interrupt is taken on the first cycle that is not
  // stalled, that is the bound: one entry per arming, on the next issuing cycle.
  always_comb if (settled && prev_trap_entry) assert(!interrupt_pending);

  // WARL. mtvec is direct mode only, so its low two bits are a mode field that
  // must stay zero; mepc's bit 0 must stay zero because instructions are at
  // least two bytes apart; MPP is hardwired to machine mode because there is no
  // other mode.
  always_comb if (clocked) assert(mtvec_value[1:0] == 2'b00);
  always_comb if (clocked) assert(mepc_value[0] == 1'b0);
  always_comb if (clocked && mstatus_addressed) assert(csr_rdata[12:11] == 2'b11);
 `endif
endmodule

`default_nettype wire
