`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// Decode vectors, driven straight at rtl/decoder.v with no pipeline around it.
//
// The one timing rule: a combinational decode flag is readable the instant
// `in.instr` settles, but anything about issuing, publishing or committing has
// to be checked after the operand-fetch cycle is spent. Every vector below
// leaves `in.next_instr` at zero, so decode's guess at the next pair is x0/x0
// and misses; drive it and the vector after it issues a cycle earlier than the
// checks expect. The three vectors that do drive it put it back.
module decoder_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  fetcher_output in;
  logic [31:0] reg_rs1, reg_rs2;
  logic [31:0] pc;
  logic [31:0] next_pc;
  logic [4:0] rs1, rs2, read_rs1, read_rs2;
  decoder_output out;
  // Driven high by the vectors that need something in flight to serialize
  // against; the hazard scoreboard is test/regfile_tb.v's and hazard.S's.
  executor_output executor_out = '0;
  logic divider_stall = 1'b0;
  logic fetch_stall = 1'b0;
  // The platform has not granted this core the shared bus. Low for every vector
  // but the ones that drive it below: with one bus initiator it never rises, and
  // nothing single-hart can otherwise say which arm of the publish block it
  // takes.
  logic bus_wait = 1'b0;
  // The instruction memory had nothing at `pc`. Driven high only by the
  // instruction-access-fault vectors below; every other vector presents a word
  // some memory really answered.
  logic imem_fault = 1'b0;
  // The platform answers atomics at the address decode is publishing. Held high
  // for every vector but the region-fault ones below, the way a memory that
  // answers everywhere would drive it.
  logic atomic_supported = 1'b1;
  logic [31:0] atomic_addr;
  logic accessor_out_valid = 1'b0;
  // rtl/csrs.v is a sibling of the decoder, not part of it, so it is stubbed.
  logic [31:0] csr_rdata = 32'b0;
  logic csr_implemented = 1'b0;
  logic [11:0] csr_addr;
  logic csr_ren, csr_wen, instret;
  logic [31:0] csr_wdata;
  logic [31:0] mtvec = 32'h0000_0100;
  logic [31:0] mepc  = 32'h0000_0244;
  // rtl/csrs.v has already ANDed the source, mie and mstatus.MIE together, so
  // driving this directly is driving the whole interrupt decision.
  logic interrupt_pending = 1'b0;
  logic trap_entry, mret_entry;
  logic [31:0] trap_cause, trap_epc, trap_tval;

  decoder dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .reg_rs1(reg_rs1),
    .reg_rs2(reg_rs2),
    .executor_out(executor_out),
    .divider_stall(divider_stall),
    .fetch_stall(fetch_stall),
    .bus_wait(bus_wait),
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
    .out(out)
  );

  int errors = 0;

  task automatic check_hex(input string what, input logic [31:0] got, input logic [31:0] expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, expected);
        errors++;
      end
    end
  endtask

  task automatic check_bit(input string what, input logic got, input logic expected);
    begin
      if (got !== expected) begin
        $display("MISMATCH %s: got=%b expected=%b", what, got, expected);
        errors++;
      end
    end
  endtask

  // pc must have exactly one driver, and it must be next_pc. The memory latches
  // its address off next_pc one cycle before the fetch that reads it. Give pc a
  // second driver and the memory runs a cycle out of step with decode, so the
  // core executes whatever is at the wrong addresses. No riscv-formal check
  // reads that port.
  //
  // Checked on every edge rather than in one vector, because a change like that
  // shows up on some instructions and not on others.
  logic [31:0] prev_next_pc;
  logic        prev_next_pc_valid = 1'b0;
  always @(posedge clk) begin
    if (prev_next_pc_valid && pc !== prev_next_pc) begin
      $display("MISMATCH next_pc predicted %08x but pc became %08x", prev_next_pc, pc);
      errors++;
    end
    prev_next_pc <= next_pc;
    prev_next_pc_valid <= 1'b1;
  end

  // `stall` is exactly these nine terms ORed together. `make cycles` charges
  // every stalled cycle to the first of them that is true, so a term added to
  // `stall` and not there would leave the cycles it costs unexplained. This says
  // so in a gate that runs on every change, rather than the next time somebody
  // asks for the table.
  //
  // `interrupt_pending` is deliberately NOT one of them, and this is what says
  // so: an interrupt entry happens ON an issuing cycle and costs no stalled
  // cycle to explain. Put it in `stall` and the pc would hold instead of
  // vectoring to mtvec, and this check goes red.
  //
  // On both clock edges, not just the rising one. Every vector here presents its
  // instruction just after a rising edge, so the falling edge is where it is
  // settled and being decoded. Sampling only the rising edge misses that, and
  // the probe for this check -- a tenth term ORed into `stall` -- goes green.
  always @(clk) begin
    if (dut.stall !== (dut.divider_stall || dut.atomic_stall || dut.hazard_rs1 ||
                       dut.hazard_rs2 || dut.serialize || dut.operand_stall ||
                       dut.fetch_stall || dut.bus_wait || dut.region_stall)) begin
      $display("MISMATCH stall is not the OR of the eight named reasons: stall=%b divider=%b atomic=%b rs1=%b rs2=%b serialize=%b operand=%b fetch=%b bus=%b region=%b",
               dut.stall, dut.divider_stall, dut.atomic_stall, dut.hazard_rs1,
               dut.hazard_rs2, dut.serialize, dut.operand_stall, dut.fetch_stall,
               dut.bus_wait, dut.region_stall);
      errors++;
    end
  end

  task automatic operand_fetch_cycle();
    begin
      @(posedge clk);
      #1;
    end
  endtask

  // The addi x0, x0, 0 matters. operand_stall compares rs1 and rs2 against
  // whatever they were at the last clock edge, and most vectors here take no
  // edge at all. Without parking on x0 first, an earlier vector can leave the
  // same pair behind, no fetch cycle happens, and every check below lands a
  // cycle early.
  task automatic present_and_fetch(input logic [31:0] instr);
    begin
      in.instr = 32'h00000013;
      @(posedge clk);
      #1;
      in.instr = instr;
      #1;
      operand_fetch_cycle();
    end
  endtask

  // The load/store region answer is registered on the cycle the access waits and
  // read on the next, so a vector that checks the trap in the presenting cycle
  // reads the answer to the previous access. This presents the access, takes the
  // wait, and asserts both halves of it -- a core that stopped waiting and one
  // that answered from a stale flip-flop both go red here rather than passing.
  //
  // `present_and_fetch` first, for its own reason: it parks on a nop so the
  // previous vector's answer expires without that vector's access issuing, and
  // it spends the operand-fetch cycle the changed pair costs. Both of those are
  // cycles the region wait is NOT, which is what makes the check below the
  // wait's own.
  task automatic region_access(input logic [31:0] instr);
    begin
      present_and_fetch(instr);
      check_bit("the access waits for its region answer", dut.region_stall, 1'b1);
      check_bit("...which is a stall", dut.stall, 1'b1);
      @(posedge clk);
      #1;
      check_bit("...and the answer is there on the next cycle",
                dut.ls_answer_valid, 1'b1);
      check_bit("...with the wait over", dut.region_stall, 1'b0);
    end
  endtask

  initial begin
    reset = 1;
    in = '0;
    // rtl/fetcher.v drives this to exactly !reset, so the real decoder never
    // sees a non-reset cycle with it low.
    in.valid = 1'b1;
    reg_rs1 = 0;
    reg_rs2 = 0;
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    // xori x1, x2, -1  =>  imm=0xfff (sign -1), rs1=x2, funct3=100, rd=x1,
    // opcode=0010011 (I-type math-immediate).
    in.instr = 32'hfff14093;
    in.pc = 32'h0;
    #1;
    check_hex("xori math_arg", dut.math_arg, 32'hffffffff);
    check_bit("a newly presented instruction stalls for its operands",
              dut.operand_stall, 1'b1);
    // Two separate things to get wrong, both silent. Drop the stall and the
    // instruction issues with a register the file has not read yet. Drop the
    // bubble and it publishes during its own fetch cycle. Either way every
    // other vector here still passes.
    check_bit("...so it does not issue in that cycle", dut.issuing, 1'b0);
    operand_fetch_cycle();
    check_bit("...and the fetch cycle bubbled decoder_out", out.valid, 1'b0);
    check_bit("...and is eligible to issue on the very next cycle",
              dut.operand_stall, 1'b0);
    check_bit("...which it does", dut.issuing, 1'b1);
    @(posedge clk);
    #1;
    check_hex("xori out.rs2 (registered math_arg)", out.rs2, 32'hffffffff);

    // The register file is asked for the NEXT instruction's pair on a cycle
    // that issues, read flat out of the fetch window's successor word. Right,
    // and the instruction behind it issues with no operand-fetch cycle at all;
    // that is the whole of what the guess buys, and nothing else here shows it.
    in.pc = 32'h0000_0040;
    present_and_fetch(32'h00100093);   // addi x1, x0, 1
    in.next_instr = 32'h00110193;      // addi x3, x2, 1 -- reads x2
    #1;
    check_hex("an issuing cycle presents the successor's rs1",
              {27'b0, read_rs1}, 32'd2);
    check_hex("...while its own decoded rs1 is still x0", {27'b0, rs1}, 32'd0);
    @(posedge clk);
    #1;
    in.instr = 32'h00110193;
    in.next_instr = 32'b0;
    #1;
    check_bit("a right guess costs the successor no operand-fetch cycle",
              dut.operand_stall, 1'b0);
    check_bit("...so it issues in the cycle it is presented", dut.issuing, 1'b1);

    // The red direction, which is what every instruction did before the guess:
    // present a pair the successor does not read and it pays the cycle.
    in.pc = 32'h0000_0060;
    present_and_fetch(32'h00100093);
    in.next_instr = 32'h00000013;      // addi x0, x0, 0 -- guesses x0/x0
    @(posedge clk);
    #1;
    in.instr = 32'h00110193;
    #1;
    check_bit("a wrong guess costs the successor its operand-fetch cycle",
              dut.operand_stall, 1'b1);
    check_bit("...so it does not issue in that cycle", dut.issuing, 1'b0);
    @(posedge clk);
    #1;
    check_bit("...and the re-presented pair lets it issue on the next one",
              dut.issuing, 1'b1);
    in.next_instr = 32'b0;

    // A compressed successor goes through the same register-number mapping, so
    // it is guessed as accurately as an uncompressed one. The successor word
    // arrives raw -- its upper half is whatever follows it in memory -- and
    // rtl/regsel.v masks that off, which is what the 0xffff here tests: sliced
    // flat, the two fields read x31 and the vector below pays a cycle it should
    // not.
    in.pc = 32'h0000_0080;
    present_and_fetch(32'h00100093);   // addi x1, x0, 1
    in.next_instr = 32'hffff_918a;     // c.add x3, x2 -- reads x3 and x2
    #1;
    check_hex("a compressed successor's rs1 is decoded, not sliced",
              {27'b0, read_rs1}, 32'd3);
    check_hex("...and so is its rs2", {27'b0, read_rs2}, 32'd2);
    @(posedge clk);
    #1;
    in.instr = 32'h0000_918a;
    in.next_instr = 32'b0;
    #1;
    check_bit("...so it issues with no operand-fetch cycle of its own",
              dut.issuing, 1'b1);
    check_hex("...on the pair it really reads", {27'b0, rs1}, 32'd3);
    check_hex("...on both halves of it", {27'b0, rs2}, 32'd2);

    // A shift's second operand is its amount, zero-extended, at both widths.
    // The poison in reg_rs2 is what says the register operand is not what
    // reaches `out.rs2` for these.
    reg_rs2 = 32'ha5a5_a5a5;
    present_and_fetch(32'h00d11093);   // slli x1, x2, 13
    check_hex("slli hands the executor its amount", dut.math_arg, 32'd13);
    @(posedge clk);
    #1;
    check_hex("...and out.rs2 carries it", out.rs2, 32'd13);

    // srai is the form that says the amount comes from the register-number
    // field and not from the immediate: its funct7 sits directly above the
    // amount, so an immediate carrying it would arrive as 0x40d. The executor
    // reads five bits, so nothing else here would notice.
    present_and_fetch(32'h40d15093);   // srai x1, x2, 13
    check_hex("srai's amount arrives with nothing above it", dut.math_arg, 32'd13);

    present_and_fetch(32'h0000_8035);  // c.srli x8, 13
    check_hex("a compressed right shift's amount is zero-extended too",
              dut.math_arg, 32'd13);
    present_and_fetch(32'h0000_00b6);  // c.slli x1, 13
    check_hex("...and so is a compressed left shift's", dut.math_arg, 32'd13);
    reg_rs2 = 32'b0;

    // The compressed group that shares one quadrant and funct3 and is told
    // apart by the two fields above them, each decoded off a different width of
    // that prefix: c.andi off funct3, c.sub off funct6.
    in.instr = 32'h0000_987d;   // c.andi x8, -1
    #1;
    check_bit("c.andi decodes", dut.instr_candi, 1'b1);
    check_bit("...as an andi", dut.instr_andi, 1'b1);
    in.instr = 32'h0000_8c05;   // c.sub x8, x9
    #1;
    check_bit("c.sub decodes", dut.instr_csub, 1'b1);
    check_bit("...as a sub", dut.instr_sub, 1'b1);
    // instr[12] is the whole of what separates that group from the RV64 row
    // above it, and a compressed shift from its reserved shamt[5]. Widen either
    // test to funct3 and both encodings decode, with every vector above still
    // passing -- so these two are what say the narrower field is read.
    in.instr = 32'h0000_9c05;   // the same row with instr[12] set: c.subw
    #1;
    check_bit("the RV64 row above it is not a sub", dut.instr_csub, 1'b0);
    check_bit("...it is illegal here", dut.instr_valid, 1'b0);
    in.instr = 32'h0000_9035;   // c.srli with shamt[5] set -- reserved in RV32
    #1;
    check_bit("a compressed shift with shamt[5] set is not a shift",
              dut.instr_csrli, 1'b0);
    check_bit("...it is illegal too", dut.instr_valid, 1'b0);

    // c.lui's reserved encoding is the one whose immediate is zero. Both
    // directions, because a test that is always true is silent.
    in.instr = 32'h0000_6085;   // c.lui x1, 1
    #1;
    check_bit("c.lui with a non-zero immediate decodes", dut.instr_clui, 1'b1);
    in.instr = 32'h0000_6081;   // the same, immediate zero -- reserved
    #1;
    check_bit("...and the reserved zero-immediate form does not",
              dut.instr_clui, 1'b0);
    check_bit("...which makes it an illegal instruction", dut.instr_valid, 1'b0);

    // LUI reaches the executor as an add of its immediate and zero, the way a
    // CSR read does, so no flag of its own rides down for it. The poison in both
    // register operands is what says neither reaches `out`: LUI names no
    // register, and the bits a register number would be read from are its
    // immediate.
    reg_rs1 = 32'ha5a5_a5a5;
    reg_rs2 = 32'h5a5a_5a5a;
    in.pc = 32'h0000_0090;
    present_and_fetch(32'h123450b7);   // lui x1, 0x12345
    @(posedge clk);
    #1;
    check_hex("lui hands the executor its immediate as rs1", out.rs1, 32'h1234_5000);
    check_hex("...and a zero rs2, so an add produces it", out.rs2, 32'b0);
    check_bit("...which is what it issues as", out.is_add, 1'b1);
    check_hex("...writing the rd it names", {27'b0, out.rd}, 32'd1);
    present_and_fetch(32'h0000_6085);  // c.lui x1, 1
    @(posedge clk);
    #1;
    check_hex("c.lui takes the same route", out.rs1, 32'h0000_1000);
    check_hex("...with the same zero rs2", out.rs2, 32'b0);
    check_bit("...and issues as an add too", out.is_add, 1'b1);
    reg_rs1 = 32'b0;
    reg_rs2 = 32'b0;

    // Every SYSTEM form with funct3 zero is told apart by funct12 alone, so the
    // rs1 and rd fields have to be checked for zero or a neighbouring encoding
    // decodes as one of them. Both fields, both directions -- and both read as
    // raw encoding fields, which is the whole of what keeps the compressed
    // register-select decode out of the trap cause and so out of the fetch loop.
    in.instr = 32'h0000_0073;   // ecall
    #1;
    check_bit("ecall decodes", dut.instr_ecall, 1'b1);
    in.instr = 32'h0000_00f3;   // the same funct12, rd = x1
    #1;
    check_bit("...but not with a non-zero rd field", dut.instr_ecall, 1'b0);
    check_bit("...which makes it illegal instead", dut.instr_valid, 1'b0);
    in.instr = 32'h0000_8073;   // the same funct12, rs1 = x1
    #1;
    check_bit("...nor with a non-zero rs1 field", dut.instr_ecall, 1'b0);
    check_bit("...which is illegal as well", dut.instr_valid, 1'b0);

    // Both arms of the next-pc chain that add to the fetched pc. The always
    // block above already checks that pc follows next_pc; these check the value
    // it takes, which nothing here did.
    in.pc = 32'h0000_00a0;
    present_and_fetch(32'h0000_0013);  // addi x0, x0, 0
    check_hex("an uncompressed instruction steps four", next_pc, 32'h0000_00a4);
    present_and_fetch(32'h0000_0001);  // c.nop
    check_hex("a compressed one steps two", next_pc, 32'h0000_00a2);
    present_and_fetch(32'h008000ef);   // jal x1, 8
    check_hex("a jal adds its immediate", next_pc, 32'h0000_00a8);
    present_and_fetch(32'h00000463);   // beq x0, x0, 8
    check_hex("a taken branch adds its own", next_pc, 32'h0000_00a8);
    reg_rs1 = 32'd1;
    #1;
    check_hex("an untaken one steps four", next_pc, 32'h0000_00a4);
    reg_rs1 = 32'b0;

    // Read off the decode flag, not `out.is_ebreak`: a trapping issue
    // suppresses every execution flag, so the registered flag is 0 for all
    // three of these and the vector would pass vacuously.
    in.instr = 32'h00100073;   // ebreak
    #1;
    check_bit("ebreak sets instr_ebreak", dut.instr_ebreak, 1'b1);
    in.instr = 32'h30200073;   // mret
    #1;
    check_bit("mret does not set instr_ebreak", dut.instr_ebreak, 1'b0);
    check_bit("...it sets instr_mret", dut.instr_mret, 1'b1);
    in.instr = 32'h10500073;   // wfi
    #1;
    check_bit("wfi does not set instr_ebreak", dut.instr_ebreak, 1'b0);
    check_bit("...it sets instr_wfi", dut.instr_wfi, 1'b1);
    @(posedge clk);
    #1;

    csr_implemented = 1'b1;

    // csrrw a1, mscratch, a0  ->  0x340515f3
    in.instr = 32'h340515f3;
    reg_rs1 = 32'hdeadbeef;   // re-presented below through present_and_fetch
    csr_rdata = 32'h0000cafe;
    #1;
    check_hex("csrrw csr_addr", {20'b0, dut.csr_addr}, 32'h340);
    check_bit("csrrw is not an immediate form", dut.is_csr_imm, 1'b0);
    check_hex("csrrw operand is rs1", dut.csr_arg, 32'hdeadbeef);
    check_hex("csrrw wdata is the operand", dut.csr_wdata, 32'hdeadbeef);
    check_bit("csrrw writes", dut.csr_write_op, 1'b1);
    check_bit("csrrw with rd != x0 reads", dut.csr_read_op, 1'b1);
    check_bit("csrrw uses rs1", dut.uses_rs1, 1'b1);
    check_bit("an implemented CSR is a valid instruction", dut.instr_valid, 1'b1);

    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;   // x0 raises no RAW hazard of its own
    present_and_fetch(32'h340515f3);
    check_bit("a CSR instruction serializes while the pipe is busy",
              dut.serialize, 1'b1);
    check_bit("...and that is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_bit("...and no CSR write commits", dut.csr_wen, 1'b0);
    check_bit("...and it bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it issues now", dut.issuing, 1'b1);
    check_bit("...committing the write", dut.csr_wen, 1'b1);
    @(posedge clk);
    #1;
    check_hex("csr read rides the is_add pass-through", out.rs1, 32'h0000cafe);
    check_hex("...with rs2 zeroed", out.rs2, 32'b0);
    check_bit("...as an add", out.is_add, 1'b1);
    check_hex("...to the encoded rd", {27'b0, out.rd}, 32'd11);

    in.instr = 32'h340fe573;   // csrrsi a0, mscratch, 0x1f
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd31; // == the zimm bits, if they were read as rs1
    #1;
    check_bit("csrrsi is an immediate form", dut.is_csr_imm, 1'b1);
    check_hex("csrrsi operand is the zimm", dut.csr_arg, 32'h1f);
    check_hex("csrrsi wdata sets the zimm bits", dut.csr_wdata, 32'h0000caff);
    check_bit("csrrsi does not use rs1", dut.uses_rs1, 1'b0);
    check_bit("...so the zimm raises no interlock", dut.hazard_rs1, 1'b0);

    in.instr = 32'h340fa573;   // csrrs a0, mscratch, x31: the same five bits
    #1;
    check_bit("csrrs x31 uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and interlocks on it", dut.hazard_rs1, 1'b1);
    executor_out.valid = 1'b0;

    in.instr = 32'h30102573;   // csrr a0, misa == csrrs a0, misa, x0
    #1;
    check_bit("csrrs with rs1 == x0 suppresses the write", dut.csr_write_op, 1'b0);
    check_bit("...and still reads", dut.csr_read_op, 1'b1);

    in.instr = 32'h34051073;   // csrw mscratch, a0 == csrrw x0, mscratch, a0
    #1;
    check_bit("csrrw with rd == x0 suppresses the read", dut.csr_read_op, 1'b0);
    check_bit("...and still writes", dut.csr_write_op, 1'b1);

    csr_implemented = 1'b0;
    #1;
    check_bit("an unimplemented CSR is not a valid instruction", dut.instr_valid, 1'b0);
    check_bit("...so it is illegal", dut.instr_illegal, 1'b1);
    check_hex("...with cause 2", trap_cause, 32'd2);
    csr_implemented = 1'b1;

    // Anything the decoder does not recognise traps, so a legal encoding left
    // out of instr_valid faults.
    in.instr = 32'h0ff0000f;   // fence iorw, iorw
    #1;
    check_bit("fence is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h0000100f;   // fence.i
    #1;
    check_bit("fence.i is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h10500073;   // wfi
    #1;
    check_bit("wfi is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h30200073;   // mret
    #1;
    check_bit("mret is a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    check_bit("...and is not an ecall", dut.instr_ecall, 1'b0);

    in.instr = 32'h00000000;
    #1;
    check_bit("the all-zero word is illegal", dut.instr_illegal, 1'b1);
    check_hex("...cause 2", trap_cause, 32'd2);
    check_bit("...and traps", dut.trap_pending, 1'b1);

    // The memory had nothing at `pc`. The word it hands over is zero, which on
    // its own is an illegal instruction and cause 2; the fault is what makes it
    // cause 1, which is what the privileged spec asks for. Nothing else in the
    // tree distinguishes the two.
    imem_fault = 1'b1;
    #1;
    check_bit("a fetch the memory could not answer traps", dut.trap_pending, 1'b1);
    check_hex("...as an instruction access fault, not an illegal instruction",
              trap_cause, 32'd1);
    // Arm order is the whole mechanism, and this is what pins it: the zero word
    // really does decode as illegal, and the cause is 1 anyway. Swap the two
    // arms and nothing else in the tree says so.
    check_bit("...even though the zero word it was handed decodes as illegal",
              dut.instr_illegal, 1'b1);

    // The fault outranks every cause the word could have produced, because the
    // word is not an instruction. Presented with a real encoding it still wins:
    // the memory saying it has nothing is not something a fetched word can
    // argue with.
    in.instr = 32'h00000073;   // ecall
    #1;
    check_hex("the fault outranks anything the unfetched word decodes to",
              trap_cause, 32'd1);
    in.instr = 32'h00452583;   // lw a1, 4(a0), misaligned below
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("...including a misalignment computed from it", trap_cause, 32'd1);
    imem_fault = 1'b0;
    #1;
    check_hex("with the fault clear the word decides again", trap_cause, 32'd4);
    reg_rs1 = 32'b0;

    in.instr = 32'h00000000;
    #1;
    check_hex("...and a zero word is an illegal instruction once more",
              trap_cause, 32'd2);

    in.instr = 32'h00100073;
    #1;
    check_hex("ebreak is cause 3", trap_cause, 32'd3);
    in.instr = 32'h00000073;
    #1;
    check_hex("ecall is cause 11", trap_cause, 32'd11);

    // The EFFECTIVE address decides, not rs1 and not the immediate.
    in.instr = 32'h00452583;   // lw a1, 4(a0)
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("misaligned lw is cause 4", trap_cause, 32'd4);
    check_bit("...and traps", dut.trap_pending, 1'b1);
    reg_rs1 = 32'h0001_0000;
    #1;
    check_bit("an aligned lw does not trap", dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0001_0002;
    #1;
    check_bit("a 2-aligned lw still traps", dut.trap_pending, 1'b1);

    in.instr = 32'h00b51023;   // sh a1, 0(a0): only 2-byte alignment needed
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("misaligned sh is cause 6", trap_cause, 32'd6);
    reg_rs1 = 32'h0001_0002;
    #1;
    check_bit("a 2-aligned sh does not trap", dut.trap_pending, 1'b0);

    in.instr = 32'h00b50023;   // sb a1, 0(a0), at the most awkward address
    reg_rs1 = 32'h0001_0003;
    #1;
    check_bit("a byte store never traps", dut.trap_pending, 1'b0);
    in.instr = 32'h00050583;   // lb a1, 0(a0)
    #1;
    check_bit("a byte load never traps", dut.trap_pending, 1'b0);
    reg_rs1 = 32'b0;

    // The other illegal-CSR rule: read-only by address, addr[11:10] == 2'b11.
    in.instr = 32'hf1151073;   // csrw mvendorid, a0
    #1;
    check_bit("an implemented read-only CSR is still a valid encoding",
              dut.instr_valid, 1'b1);
    check_bit("...but writing it is illegal", dut.csr_readonly_write, 1'b1);
    check_hex("...cause 2", trap_cause, 32'd2);
    in.instr = 32'hf1102573;   // csrr a0, mvendorid
    #1;
    check_bit("reading a read-only CSR is not a write", dut.csr_readonly_write, 1'b0);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);

    // Run again with nothing in the pipeline. A CSR instruction waits either
    // way, legal or not, so otherwise the checks below pass because it stalled
    // rather than because it trapped.
    present_and_fetch(32'hf1151073);
    check_bit("...it issues once the pipe drains", dut.issuing, 1'b1);
    check_bit("a trapping issue does not count in minstret", instret, 1'b0);
    check_bit("...and commits no CSR write", csr_wen, 1'b0);
    check_bit("...and no CSR read", csr_ren, 1'b0);
    check_bit("...but it does commit a trap", trap_entry, 1'b1);

    //-----------------------------------------------------------------------
    // What the trap reports it happened to. The value is a platform statement
    // firmware cannot derive, so every cause gets a vector and the two that
    // report zero are asserted rather than left to the default arm -- a mux
    // that fell through to zero for a cause that should carry an address would
    // otherwise be indistinguishable from one that meant to.
    //-----------------------------------------------------------------------

    in.pc = 32'h0000_0240;
    imem_fault = 1'b1;
    in.instr = 32'h00000073;
    #1;
    check_hex("an instruction access fault reports the address it was refused at",
              trap_tval, 32'h0000_0240);
    imem_fault = 1'b0;

    in.instr = 32'hf1151073;   // csrw mvendorid, a0
    #1;
    check_hex("an illegal instruction reports the faulting word",
              trap_tval, 32'hf1151073);
    in.instr = 32'h00000000;
    #1;
    check_hex("...and the all-zero word reports itself", trap_tval, 32'h0);

    // A compressed illegal instruction is zero-extended, not handed its
    // neighbour: the upper half of the fetch window is the NEXT instruction in
    // memory, and reporting it would name a word that did not fault.
    in.instr = 32'hdead_0000;
    #1;
    check_hex("a compressed illegal instruction reports its 16 bits alone",
              trap_tval, 32'h0000_0000);

    in.instr = 32'h00100073;   // ebreak
    #1;
    check_hex("a breakpoint reports nothing -- mepc already has its address",
              trap_tval, 32'h0);
    in.instr = 32'h00000073;   // ecall
    #1;
    check_hex("...and so does an environment call", trap_tval, 32'h0);

    // The EFFECTIVE address, which is what a handler cannot cheaply recompute:
    // it would have to re-fetch the instruction, decode it and redo this add
    // out of its own saved context.
    in.instr = 32'h00452583;   // lw a1, 4(a0)
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("a misaligned load reports the address it computed",
              trap_tval, 32'h0001_0005);
    in.instr = 32'h00b51023;   // sh a1, 0(a0)
    #1;
    check_hex("...and a misaligned store the same", trap_tval, 32'h0001_0001);

    // An atomic's effective address is rs1 verbatim, and the sum is what is
    // reported. Those two are the same number by construction here; a mux
    // reading the sum for one and the register for the other would still agree.
    reg_rs1 = 32'h0004_0000;
    atomic_supported = 1'b0;
    in.instr = 32'h1006252f;   // lr.w
    #1;
    check_hex("a refused lr.w reports its address", trap_tval, 32'h0004_0000);
    in.instr = 32'h00b6252f;   // amoadd.w
    #1;
    check_hex("...and a refused AMO reports its address", trap_tval, 32'h0004_0000);
    atomic_supported = 1'b1;
    reg_rs1 = 32'b0;

    // An interrupt happened to nothing, and it outranks whatever the displaced
    // instruction would have faulted on -- so this is driven over an
    // instruction that reports an address of its own.
    in.instr = 32'h00452583;
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("the displaced instruction would have reported an address",
              trap_tval, 32'h0001_0005);
    interrupt_pending = 1'b1;
    #1;
    check_hex("...and the interrupt that outranks it reports nothing",
              trap_tval, 32'h0);
    interrupt_pending = 1'b0;
    reg_rs1 = 32'b0;

    // Nothing is trapping, so there is nothing to report.
    in.instr = 32'h00000013;   // nop
    #1;
    check_bit("a non-trapping instruction is not a trap", dut.trap_taken, 1'b0);
    check_hex("...and reports nothing", trap_tval, 32'h0);

    // Through the region wait, which a misaligned access spends like any other:
    // the wait is raised on where the base register points and not on what the
    // instruction will do about it, so an access that is going to trap on its
    // alignment waits first and traps a cycle later.
    reg_rs1 = 32'h0001_0001;
    in.pc = 32'h0000_0080;
    region_access(32'h00452583);   // the misaligned lw again
    check_hex("trap_epc is the FAULTING pc, not the next one", trap_epc, 32'h0000_0080);
    @(posedge clk);
    #1;
    check_hex("a trap redirects pc to mtvec", pc, 32'h0000_0100);
    check_bit("...the trapping instruction still retires", out.valid, 1'b1);
    check_hex("...writing no register", {27'b0, out.rd}, 32'b0);
    check_bit("...and issuing no load", out.is_lw, 1'b0);
    reg_rs1 = 32'b0;

    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;
    present_and_fetch(32'h30200073);
    check_bit("mret serializes while the pipe is busy", dut.serialize, 1'b1);
    check_bit("...so it does not commit yet", mret_entry, 1'b0);
    check_bit("...and it bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it commits now", mret_entry, 1'b1);
    check_bit("...and is not a trap", trap_entry, 1'b0);
    @(posedge clk);
    #1;
    check_hex("mret redirects pc to mepc", pc, 32'h0000_0244);

    // Issued once so decoder_out holds something a bubble would visibly destroy.
    present_and_fetch(32'h00100093);   // addi x1, x0, 1
    @(posedge clk);
    #1;
    check_bit("the instruction issued", out.valid, 1'b1);
    check_hex("...into decoder_out", {27'b0, out.rd}, 32'd1);

    fetch_stall = 1'b1;
    #1;
    check_bit("a stolen fetch window is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_hex("...and the pc holds, so the instruction is presented again",
              next_pc, pc);
    check_bit("...and no trap is committed out of the stolen window", trap_entry, 1'b0);

    // A divide holds decoder_out; a steal clears it. On a cycle with both,
    // holding has to win or the held instruction is lost. Only the order of the
    // arms in the publish block decides that, and swapping them is silent.
    divider_stall = 1'b1;
    #1;
    @(posedge clk);
    #1;
    check_bit("a steal coinciding with a divide holds decoder_out",
              out.valid, 1'b1);
    check_hex("...unchanged", {27'b0, out.rd}, 32'd1);
    divider_stall = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("a steal on its own bubbles decoder_out instead", out.valid, 1'b0);
    fetch_stall = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("...and the same instruction issues once the window is back",
              out.valid, 1'b1);
    check_hex("...as itself", {27'b0, out.rd}, 32'd1);

    // The window a steal delivers holds a data word, so the pair decoded from it
    // is not a pair any instruction reads. Presenting it would throw away the
    // successor guess made before the steal, and the instruction behind the
    // steal would pay an operand-fetch cycle it had already earned. The pair is
    // held across the steal instead, and `operand_stall` is the same compare
    // either way.
    in.pc = 32'h0000_0280;
    present_and_fetch(32'h00100093);   // addi x1, x0, 1
    in.next_instr = 32'h00110193;      // addi x3, x2, 1 -- reads x2
    #1;
    check_hex("the cycle before a steal presents the successor's rs1",
              {27'b0, read_rs1}, 32'd2);
    @(posedge clk);
    #1;
    // The steal arrives with a data word in the window. Both halves of it name
    // registers the guess did not, which is what a held pair has to survive.
    fetch_stall = 1'b1;
    in.instr = 32'hdead_beef;
    #1;
    check_hex("a stolen window presents the pair from before it, not one off a data word",
              {27'b0, read_rs1}, 32'd2);
    @(posedge clk);
    #1;
    fetch_stall = 1'b0;
    in.instr = 32'h00110193;           // the successor, arriving for real now
    #1;
    check_bit("...so the instruction behind the steal owes no operand-fetch cycle",
              dut.operand_stall, 1'b0);
    check_bit("...and issues in the cycle the window comes back", dut.issuing, 1'b1);
    in.next_instr = 32'b0;
    @(posedge clk);
    #1;

    // Text is writable, so a store just before a fence.i can change the words
    // being fetched right behind it. Waiting is what puts that store's write
    // ahead of the next fetch address.
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;
    present_and_fetch(32'h0000100f);
    check_bit("fence.i serializes while the pipe is busy", dut.serialize, 1'b1);
    check_bit("...and that is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_bit("...and it bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it issues now", dut.issuing, 1'b1);

    // A plain `fence` has nothing to wait for: one bus, one access in flight.
    executor_out.valid = 1'b1;
    in.instr = 32'h0ff0000f;
    #1;
    check_bit("a plain fence does not serialize", dut.serialize, 1'b0);
    executor_out.valid = 1'b0;

    //-----------------------------------------------------------------------
    // The machine timer interrupt. It is asynchronous and this core commits
    // every trap in decode, which only works because the interrupt is taken on
    // a cycle that would otherwise have ISSUED: the instruction it displaces
    // has not issued, so there is nothing downstream to take back.
    //-----------------------------------------------------------------------

    in.pc = 32'h0000_0300;
    present_and_fetch(32'h00100093);   // addi x1, x0, 1 -- a harmless victim
    check_bit("without an interrupt the instruction just issues", trap_entry, 1'b0);

    interrupt_pending = 1'b1;
    #1;
    check_bit("an armed interrupt is taken on an issuing cycle", trap_entry, 1'b1);
    check_hex("...with cause interrupt/7", trap_cause, 32'h8000_0007);
    check_hex("...and mepc pointing AT the instruction, not past it",
              trap_epc, 32'h0000_0300);
    check_hex("...vectoring to mtvec", next_pc, 32'h0000_0100);
    check_bit("...counting nothing in minstret", instret, 1'b0);
    check_bit("...committing no CSR write", csr_wen, 1'b0);
    check_bit("...and no CSR read", csr_ren, 1'b0);
    check_bit("...and it is not an mret", mret_entry, 1'b0);
    check_bit("...and it is not itself a stall", dut.stall, 1'b0);
    @(posedge clk);
    #1;
    check_hex("...so the next fetch is the handler", pc, 32'h0000_0100);
    check_bit("...and decoder_out is a bubble: the victim never issued",
              out.valid, 1'b0);
    interrupt_pending = 1'b0;

    // The other direction. Without this the vectors above pass on a decoder
    // that traps unconditionally.
    in.pc = 32'h0000_0400;
    present_and_fetch(32'h00100093);
    check_bit("a disarmed interrupt takes nothing", trap_entry, 1'b0);
    check_hex("...and leaves trap_cause at zero", trap_cause, 32'b0);
    @(posedge clk);
    #1;
    check_bit("...and the instruction issues normally", out.valid, 1'b1);

    // `stall` outranks the trap arm of the next_pc chain, so every reason the
    // core already has to wait holds the interrupt off too -- no new logic, and
    // no way for an interrupt to cut into a divide or a serialization.
    in.pc = 32'h0000_0500;
    present_and_fetch(32'h00100093);
    interrupt_pending = 1'b1;
    divider_stall = 1'b1;
    #1;
    check_bit("a divide holds the interrupt off", trap_entry, 1'b0);
    check_hex("...and the pc with it", next_pc, pc);
    divider_stall = 1'b0;
    fetch_stall = 1'b1;
    #1;
    check_bit("so does a stolen fetch window", trap_entry, 1'b0);
    fetch_stall = 1'b0;
    #1;
    check_bit("and the interrupt is taken the moment they clear", trap_entry, 1'b1);
    @(posedge clk);
    #1;
    interrupt_pending = 1'b0;

    // Serialization is the one that matters most: an `mret` interrupted
    // half-way would pop mstatus and then push it again. It cannot happen,
    // because `mret` is still waiting for the pipeline to empty and waiting is
    // a stall.
    in.pc = 32'h0000_0540;
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd0;
    present_and_fetch(32'h30200073);   // mret
    interrupt_pending = 1'b1;
    #1;
    check_bit("a serializing mret holds the interrupt off", trap_entry, 1'b0);
    check_bit("...and does not commit either", mret_entry, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("once the pipe is empty the interrupt is taken", trap_entry, 1'b1);
    check_bit("...and the mret it displaced still does not commit", mret_entry, 1'b0);
    check_hex("...so mepc is the mret's own address, and it runs again",
              trap_epc, 32'h0000_0540);
    @(posedge clk);
    #1;
    interrupt_pending = 1'b0;

    // An instruction that would fault AND an armed interrupt. The interrupt
    // wins because the instruction does not execute; it faults instead when it
    // re-executes after the handler returns.
    in.pc = 32'h0000_0600;
    reg_rs1 = 32'h0001_0001;
    region_access(32'h00452583);   // the misaligned lw, through its region wait
    check_hex("on its own it is a load-misaligned fault", trap_cause, 32'd4);
    interrupt_pending = 1'b1;
    #1;
    check_hex("an interrupt outranks the instruction's own fault",
              trap_cause, 32'h8000_0007);
    check_bit("...and it is still one trap entry, not two", trap_entry, 1'b1);
    check_hex("...at the same mepc either way", trap_epc, 32'h0000_0600);
    @(posedge clk);
    #1;
    check_bit("...publishing nothing, where the fault would have retired",
              out.valid, 1'b0);
    interrupt_pending = 1'b0;
    reg_rs1 = 32'b0;

    // ...and the same ordering for a fetch that never happened. The interrupt
    // still wins: the instruction did not issue, so the fetch that failed is
    // re-attempted after the handler returns and faults then.
    in.pc = 32'h0000_0640;
    present_and_fetch(32'h00000000);
    imem_fault = 1'b1;
    #1;
    check_hex("on its own an unfetchable pc is an instruction access fault",
              trap_cause, 32'd1);
    interrupt_pending = 1'b1;
    #1;
    check_hex("an interrupt outranks it too", trap_cause, 32'h8000_0007);
    interrupt_pending = 1'b0;
    #1;

    // A fetch fault is committed on the same override the jumps use, and the
    // instruction reaches the end of the pipeline having done nothing but
    // redirect. Everything that could act on the word the memory did not supply
    // has to be clear.
    check_bit("...and on its own it is one trap entry", trap_entry, 1'b1);
    check_hex("...vectoring to mtvec", next_pc, 32'h0000_0100);
    check_hex("...with mepc at the address that could not be fetched",
              trap_epc, 32'h0000_0640);
    check_bit("...counting nothing in minstret", instret, 1'b0);
    check_bit("...and it is not a stall", dut.stall, 1'b0);
    @(posedge clk);
    #1;
    check_bit("...it still retires", out.valid, 1'b1);
    check_hex("...writing no register", {27'b0, out.rd}, 32'b0);
    check_bit("...and starting no memory access",
              out.is_lw || out.is_lh || out.is_lhu || out.is_lb || out.is_lbu ||
              out.is_sw || out.is_sh || out.is_sb, 1'b0);
    imem_fault = 1'b0;

    //-----------------------------------------------------------------------
    // The A extension. Every encoding below is `.w`, rd = a0, rs1 = a2 and
    // rs2 = a1, so only funct5 and the ordering bits differ between them.
    //-----------------------------------------------------------------------

    in.instr = 32'h00b6252f;   // amoadd.w a0, a1, (a2)
    #1;
    check_bit("amoadd.w decodes", dut.instr_amoadd, 1'b1);
    check_bit("...as a valid instruction", dut.instr_valid, 1'b1);
    check_bit("...and does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h08b6252f;
    #1;
    check_bit("amoswap.w decodes", dut.instr_amoswap, 1'b1);
    in.instr = 32'h20b6252f;
    #1;
    check_bit("amoxor.w decodes", dut.instr_amoxor, 1'b1);
    in.instr = 32'h40b6252f;
    #1;
    check_bit("amoor.w decodes", dut.instr_amoor, 1'b1);
    in.instr = 32'h60b6252f;
    #1;
    check_bit("amoand.w decodes", dut.instr_amoand, 1'b1);
    in.instr = 32'h80b6252f;
    #1;
    check_bit("amomin.w decodes", dut.instr_amomin, 1'b1);
    in.instr = 32'ha0b6252f;
    #1;
    check_bit("amomax.w decodes", dut.instr_amomax, 1'b1);
    in.instr = 32'hc0b6252f;
    #1;
    check_bit("amominu.w decodes", dut.instr_amominu, 1'b1);
    in.instr = 32'he0b6252f;
    #1;
    check_bit("amomaxu.w decodes", dut.instr_amomaxu, 1'b1);
    in.instr = 32'h1006252f;
    #1;
    check_bit("lr.w decodes", dut.instr_lr, 1'b1);
    check_bit("...and is not an AMO", dut.instr_amo, 1'b0);
    in.instr = 32'h18b6252f;
    #1;
    check_bit("sc.w decodes", dut.instr_sc, 1'b1);
    check_bit("...and is not an AMO either", dut.instr_amo, 1'b0);

    // The reserved rows of the same opcode, both directions of each field.
    in.instr = 32'h28b6252f;   // funct5 = 00101, which names nothing
    #1;
    check_bit("an unassigned funct5 is not an atomic", dut.instr_atomic, 1'b0);
    check_bit("...and is illegal", dut.instr_valid, 1'b0);
    in.instr = 32'h00b6352f;   // amoadd.d -- funct3 = 011, RV64 only
    #1;
    check_bit("the doubleword width is not implemented here", dut.instr_atomic, 1'b0);
    check_bit("...so it is illegal", dut.instr_valid, 1'b0);
    // lr.w's rs2 field is an encoding constant. A non-zero one is a different
    // encoding, and reading it as a register would make the decoder wait on a
    // value the instruction does not read.
    in.instr = 32'h1056252f;
    #1;
    check_bit("lr.w with a non-zero rs2 field is not an lr.w", dut.instr_lr, 1'b0);
    check_bit("...it is illegal", dut.instr_valid, 1'b0);

    // `.aq` and `.rl` are decoded and ignored, so all four ordering suffixes of
    // one instruction are one instruction. Vectored rather than argued: the two
    // bits sit inside the field the immediate is read from, and a decode that
    // let them through would differ here and nowhere else.
    in.instr = 32'h02b6252f;   // amoadd.w.rl
    #1;
    check_bit("amoadd.w.rl is the same instruction", dut.instr_amoadd, 1'b1);
    in.instr = 32'h04b6252f;   // amoadd.w.aq
    #1;
    check_bit("...and so is amoadd.w.aq", dut.instr_amoadd, 1'b1);
    in.instr = 32'h06b6252f;   // amoadd.w.aqrl
    #1;
    check_bit("...and amoadd.w.aqrl", dut.instr_amoadd, 1'b1);
    check_hex("...and none of them puts anything in the immediate",
              dut.immediate, 32'b0);

    // The effective address is rs1 exactly. The bits an I-immediate would be
    // read from are funct5, `aq`, `rl` and rs2 here, and 0x06b is what they
    // would arrive as, so this is the vector that says the A arm of the
    // immediate mux is doing something.
    reg_rs1 = 32'h0001_0000;
    present_and_fetch(32'h06b6252f);
    check_hex("an atomic's effective address is rs1 and nothing else",
              dut.mem_addr_calc, 32'h0001_0000);
    @(posedge clk);
    #1;
    check_hex("...which is what reaches the accessor", out.mem_addr, 32'h0001_0000);
    reg_rs1 = 32'b0;

    // The operands each of the eleven really reads. Widen `uses_rs2` to lr.w
    // and the core waits on a register its encoding does not name; narrow it
    // from sc.w and the store goes out with a stale word.
    in.instr = 32'h00b6252f;
    #1;
    check_bit("an AMO uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and rs2", dut.uses_rs2, 1'b1);
    in.instr = 32'h18b6252f;
    #1;
    check_bit("sc.w uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and rs2, which is the word it stores", dut.uses_rs2, 1'b1);
    in.instr = 32'h1006252f;
    #1;
    check_bit("lr.w uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and NOT rs2: that field is an encoding constant",
              dut.uses_rs2, 1'b0);

    // All three misalignment causes. An atomic is word-wide and never split, so
    // anything but a word-aligned address faults -- as a store for the ten that
    // write and as a load for the one that does not.
    reg_rs1 = 32'h0001_0002;
    in.instr = 32'h00b6252f;
    #1;
    check_hex("a misaligned AMO is a store misalignment", trap_cause, 32'd6);
    in.instr = 32'h18b6252f;
    #1;
    check_hex("...and so is a misaligned sc.w", trap_cause, 32'd6);
    in.instr = 32'h1006252f;
    #1;
    check_hex("a misaligned lr.w is a LOAD misalignment", trap_cause, 32'd4);
    reg_rs1 = 32'h0001_0001;
    #1;
    check_hex("...at a byte offset too", trap_cause, 32'd4);
    reg_rs1 = 32'h0001_0000;
    #1;
    check_bit("an aligned lr.w does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h00b6252f;
    #1;
    check_bit("...nor does an aligned AMO", dut.trap_pending, 1'b0);
    reg_rs1 = 32'b0;

    // The two region causes. The platform decodes the address the decoder
    // publishes and hands back one bit, so this drives that bit rather than a
    // map: what is being checked here is what decode does with the answer.
    reg_rs1 = 32'h0004_0000;
    atomic_supported = 1'b0;
    in.instr = 32'h1006252f;   // lr.w
    #1;
    check_hex("the address the platform is asked about is rs1 verbatim",
              atomic_addr, 32'h0004_0000);
    check_bit("an lr.w the platform does not answer traps", dut.trap_pending, 1'b1);
    check_hex("...as a LOAD access fault", trap_cause, 32'd5);
    in.instr = 32'h00b6252f;   // amoadd.w
    #1;
    check_hex("an AMO there is a STORE/AMO access fault", trap_cause, 32'd7);
    in.instr = 32'h18b6252f;   // sc.w
    #1;
    check_hex("...and so is an sc.w", trap_cause, 32'd7);

    // A plain load and a plain store at the same address raise the same two
    // causes, off the map the decoder is elaborated with rather than off the
    // bit. This instance takes the parameter defaults: 8 KB of text at 0, 64 KB
    // of RAM at 0x0001_0000, the timer's reserved 32 bytes at 0x0002_0000, the
    // UART's eight above them and the SPI controller's eight above those, so
    // 0x0004_0000 is outside all five.
    //
    // THE ANSWER IS A CYCLE LATE, and `region_access` is what every vector from
    // here down goes through because of it. Check the trap in the cycle the
    // access is presented and what comes back is the answer to the PREVIOUS
    // access, which is how a core whose deferral had stopped working would pass
    // this file. So the task asserts the wait it takes as well as taking it.
    reg_rs1 = 32'h0004_0000;
    region_access(32'h00062583);   // lw a1, 0(a2)
    check_bit("a plain lw at the same address faults too", dut.trap_pending, 1'b1);
    check_hex("...as a LOAD access fault", trap_cause, 32'd5);
    region_access(32'h00b62023);   // sw a1, 0(a2)
    check_hex("...and a plain sw as a STORE access fault", trap_cause, 32'd7);
    region_access(32'h00060583);   // lb a1, 0(a2)
    check_hex("a byte load faults there as well", trap_cause, 32'd5);
    region_access(32'h00b60023);   // sb a1, 0(a2)
    check_hex("...and a byte store", trap_cause, 32'd7);

    // The whole sum is what the answer is about, which is what the cycle buys:
    // each of the four below sits in a 2 KB block the other side of a window's
    // edge from its own effective address, so a test that stopped at the base
    // register would get every one of them the wrong way round.
    reg_rs1 = 32'h0000_1FFC;       // the last word of the 8 KB text window
    region_access(32'h00462583);   // lw a1, 4(a2)
    check_bit("a load off the top of text leaves its block", dut.trap_pending, 1'b1);
    check_hex("...and faults as a load", trap_cause, 32'd5);
    reg_rs1 = 32'h0000_2000;       // the first word past it
    region_access(32'hFFC62583);   // lw a1, -4(a2)
    check_bit("...and a load back into text from just past it does not fault",
              dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0001_0000;       // the base of the RAM
    region_access(32'hFFC62583);
    check_bit("a negative offset off the bottom of RAM faults",
              dut.trap_pending, 1'b1);
    check_hex("...as a load", trap_cause, 32'd5);
    reg_rs1 = 32'h0001_0008;
    region_access(32'hFFC62583);
    check_bit("...and one that stays inside RAM does not", dut.trap_pending, 1'b0);

    // The timer and the UART are words, not blocks. Neither window is three
    // blocks wide, so no address in either can reach the fast path and every
    // access there is answered from the sum.
    reg_rs1 = 32'h0001_FFFC;
    region_access(32'h00462583);   // lw a1, 4(a2)
    check_bit("a load of mtime from the top of RAM is answered",
              dut.trap_pending, 1'b0);
    // The timer's window is the eight words the map reserves for one mtimecmp
    // per hart, so 16 bytes up is inside it even where only four are decoded.
    reg_rs1 = 32'h0002_0000;
    region_access(32'h01062583);   // lw a1, 16(a2)
    check_bit("...and one 16 bytes up is inside the timer's reserved window",
              dut.trap_pending, 1'b0);

    // 32 past the base is the UART, which the map answers.
    region_access(32'h02062583);   // lw a1, 32(a2)
    check_bit("...and one 32 bytes past the base is the UART",
              dut.trap_pending, 1'b0);

    // 40 past the base is the SPI controller, which the map answers too.
    region_access(32'h02862583);   // lw a1, 40(a2)
    check_bit("...and one 40 bytes past the base is the SPI controller",
              dut.trap_pending, 1'b0);

    // 48 past it is the first address in that page no device claims.
    region_access(32'h03062583);   // lw a1, 48(a2)
    check_bit("...and one 48 bytes past it is claimed by nothing",
              dut.trap_pending, 1'b1);
    check_hex("...faulting as a load", trap_cause, 32'd5);

    // The other direction, without which this file would pass on a core that
    // faulted every load and every store.
    reg_rs1 = 32'h0001_0000;
    region_access(32'h00062583);   // lw a1, 0(a2)
    check_bit("a load the map answers does not fault", dut.trap_pending, 1'b0);
    region_access(32'h00b62023);   // sw a1, 0(a2)
    check_bit("...nor does a store there", dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0000_0000;
    region_access(32'h00b62023);
    check_bit("...nor a store into text", dut.trap_pending, 1'b0);

    // THE FAST PATH, which is what the wait is being spent to buy. A base
    // register a whole block inside a window cannot leave it whatever the
    // immediate is, so decode issues with no region term in the cycle at all --
    // no wait, and no answer to read. Delete the settled test and every load in
    // a program costs a cycle; make it two-sided and it starts faulting
    // addresses a memory answers, which the pair after these is the check for.
    reg_rs1 = 32'h0001_1000;       // deep inside the 64 KB RAM
    present_and_fetch(32'h00062583);   // lw a1, 0(a2)
    check_bit("a load deep inside RAM waits for nothing", dut.region_stall, 1'b0);
    check_bit("...and is not a stall at all", dut.stall, 1'b0);
    check_bit("...and does not fault", dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0001_17FC;       // the top word of that block
    present_and_fetch(32'h00462583);   // lw a1, 4(a2) -- into the block above
    check_bit("...nor does one that leaves the block upwards",
              dut.region_stall, 1'b0);
    check_bit("...which still does not fault", dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0001_1000;
    present_and_fetch(32'h80062583);   // lw a1, -2048(a2) -- and downwards
    check_bit("...nor one that leaves it downwards", dut.region_stall, 1'b0);
    check_bit("...which still does not fault", dut.trap_pending, 1'b0);
    reg_rs1 = 32'h0000_0800;       // deep inside the 8 KB text window
    present_and_fetch(32'h00062583);
    check_bit("a load deep inside text waits for nothing too",
              dut.region_stall, 1'b0);
    check_bit("...and does not fault", dut.trap_pending, 1'b0);

    // A window's own first and last blocks are what the fast path may not
    // claim, and this pair says so. 0x0001_0400 is the RAM's first block, where
    // a negative offset really does leave the window; 0x0001_0800 is the second,
    // where none can. Widen the settled test by a block and the first of these
    // stops waiting and starts reading memory that is not there.
    reg_rs1 = 32'h0001_0400;
    present_and_fetch(32'h00062583);
    check_bit("the RAM's first block does not reach the fast path",
              dut.region_stall, 1'b1);
    reg_rs1 = 32'h0001_0800;
    present_and_fetch(32'h00062583);
    check_bit("...and the block above it does", dut.region_stall, 1'b0);

    // THE WAIT'S ARM, both ways round. It BUBBLES: nothing has issued, so a held
    // decoder_out would hand the executor the same instruction twice. A wait
    // coinciding with a divide HOLDS, for the divider's own reason -- what
    // decoder_out carries then is an instruction the executor has not taken.
    // Only the order of the arms in the publish block decides either, and
    // swapping them is silent everywhere else.
    //
    // `in.next_instr` is driven so the second load's pair is the one decode
    // guessed: without it that load pays an operand-fetch cycle, which bubbles
    // decoder_out for a reason that is not this one and leaves the hold below
    // comparing zero against zero.
    in.pc = 32'h0000_07C0;
    reg_rs1 = 32'h0001_0000;
    region_access(32'h00062303);       // lw x6, 0(a2) -- the RAM's base block
    in.next_instr = 32'h00462383;      // lw x7, 4(a2) -- the same pair
    @(posedge clk);
    #1;
    check_bit("a load whose answer has arrived issues", out.valid, 1'b1);
    check_hex("...into decoder_out", {27'b0, out.rd}, 32'd6);
    in.instr = 32'h00462383;
    in.next_instr = 32'b0;
    #1;
    check_bit("the load behind it waits for its own answer",
              dut.region_stall, 1'b1);
    check_bit("...with nothing else holding it", dut.stall_other, 1'b0);
    divider_stall = 1'b1;
    #1;
    @(posedge clk);
    #1;
    check_bit("a region wait coinciding with a divide holds decoder_out",
              out.valid, 1'b1);
    check_hex("...unchanged", {27'b0, out.rd}, 32'd6);
    divider_stall = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("the region wait on its own bubbles decoder_out instead",
              out.valid, 1'b0);

    reg_rs1 = 32'h0004_0000;

    // Misalignment outranks the region for a plain access too, the order the
    // atomic term states. Drop either alignment term from `ls_fault` and two
    // arms of the cause chain match at once.
    reg_rs1 = 32'h0004_0002;
    in.instr = 32'h00062583;   // lw a1, 0(a2)
    #1;
    check_hex("a misaligned lw out of region reports the misalignment",
              trap_cause, 32'd4);
    in.instr = 32'h00b62023;   // sw a1, 0(a2)
    #1;
    check_hex("...and a misaligned sw reports cause 6", trap_cause, 32'd6);
    reg_rs1 = 32'h0004_0000;

    // Misalignment outranks the region, which keeps the four data causes
    // disjoint and matches what the reference model reports. Drop the alignment
    // term from the region test and two arms of the cause chain match at once.
    reg_rs1 = 32'h0004_0002;
    in.instr = 32'h1006252f;
    #1;
    check_hex("a misaligned lr.w out of region reports the misalignment",
              trap_cause, 32'd4);
    in.instr = 32'h00b6252f;
    #1;
    check_hex("...and a misaligned AMO out of region reports cause 6",
              trap_cause, 32'd6);

    // ...and with the platform answering, the same encodings issue. Without
    // this the file would pass on a core that faulted every atomic.
    reg_rs1 = 32'h0001_0000;
    atomic_supported = 1'b1;
    in.instr = 32'h1006252f;
    #1;
    check_bit("an lr.w the platform answers does not trap", dut.trap_pending, 1'b0);
    in.instr = 32'h00b6252f;
    #1;
    check_bit("...nor does an AMO there", dut.trap_pending, 1'b0);

    // A refused atomic publishes none of its eleven flags, so no transaction
    // goes out and no reservation is taken. The misaligned case below asserts
    // the same thing for the other reason an atomic can trap.
    atomic_supported = 1'b0;
    reg_rs1 = 32'h0004_0000;
    present_and_fetch(32'h003120af);   // amoadd.w x1, x3, (x2)
    check_bit("a refused AMO commits a trap", trap_entry, 1'b1);
    @(posedge clk);
    #1;
    check_bit("...and retires with every atomic flag clear",
              out.is_amo || out.is_amoadd || out.is_lr || out.is_sc, 1'b0);
    check_hex("...and no rd", {27'b0, out.rd}, 32'd0);
    atomic_supported = 1'b1;
    reg_rs1 = 32'b0;

    // The scoreboard. An AMO's result arrives a cycle later than a load's and a
    // store-conditional writes a register at all, which no other store does --
    // so both are checked, at the one place decode can see it, for the gap the
    // scoreboard forbids: an in-flight rd invisible for a cycle between issue
    // and the regfile write-through.
    in.pc = 32'h0000_0700;
    present_and_fetch(32'h003120af);   // amoadd.w x1, x3, (x2)
    check_bit("an AMO issues", dut.issuing, 1'b1);
    @(posedge clk);
    #1;
    check_bit("...into decoder_out", out.valid, 1'b1);
    check_hex("...carrying its rd, where the scoreboard can see it",
              {27'b0, out.rd}, 32'd1);
    check_bit("...and its own flag", out.is_amoadd, 1'b1);
    // Published beside the nine rather than ORed back together by each reader.
    // Decode spends the write cycle off this bit and rtl/accessor.v routes the
    // read-modify-write off it, so the two would agree about an AMO and
    // disagree about which cycle the bus is busy.
    check_bit("...and the AMO bit the write cycle is spent on", out.is_amo, 1'b1);
    in.instr = 32'h00108093;           // addi x1, x1, 1 -- reads the AMO's rd
    #1;
    check_bit("...so the instruction behind it interlocks", dut.hazard_rs1, 1'b1);

    // The atomic wait itself, which is the sixth stall reason. It is raised on
    // the cycle after the AMO issues, because that is the cycle rtl/accessor.v
    // needs the bus for the write half.
    check_bit("an AMO in flight raises the atomic wait", dut.atomic_stall, 1'b1);
    check_bit("...and that is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_hex("...and the pc holds", next_pc, pc);
    @(posedge clk);
    #1;
    // BUBBLES rather than holds, which is the opposite of the divider's
    // ruling. The executor has already taken the AMO, so a held decoder_out
    // would put a second read on the bus beside the write and retire the
    // instruction twice. Only the arm order in the publish block decides this.
    check_bit("the atomic wait bubbles decoder_out rather than holding it",
              out.valid, 1'b0);
    check_bit("...and it is over after that one cycle", dut.atomic_stall, 1'b0);

    // A held AMO has not been taken yet, so the write cycle is not next and the
    // wait must not be raised. Without this the wait would fire on every cycle
    // of a divide and cost the pipeline an instruction each time.
    in.pc = 32'h0000_0740;
    present_and_fetch(32'h003120af);
    @(posedge clk);
    #1;
    divider_stall = 1'b1;
    #1;
    check_bit("a held AMO does not raise the atomic wait", dut.atomic_stall, 1'b0);
    repeat (3) begin
      @(posedge clk);
      #1;
      check_bit("...and decoder_out holds it, unchanged, for as long as the divide runs",
                out.valid, 1'b1);
      check_bit("...still raising no wait", dut.atomic_stall, 1'b0);
    end
    divider_stall = 1'b0;
    #1;
    check_bit("...the wait comes the moment the executor takes it",
              dut.atomic_stall, 1'b1);
    @(posedge clk);
    #1;
    @(posedge clk);
    #1;

    // A store-conditional is a store that writes a register. Neither lr.w nor
    // sc.w needs the extra cycle -- each is one bus transaction -- so neither
    // raises the wait.
    in.pc = 32'h0000_0780;
    present_and_fetch(32'h183120af);   // sc.w x1, x3, (x2)
    check_bit("sc.w issues", dut.issuing, 1'b1);
    @(posedge clk);
    #1;
    check_hex("...carrying an rd the scoreboard can see, unlike every other store",
              {27'b0, out.rd}, 32'd1);
    check_bit("...and its own flag", out.is_sc, 1'b1);
    check_bit("...but not the AMO bit: neither store-conditional nor lr.w is one",
              out.is_amo, 1'b0);
    check_bit("...and it raises no atomic wait: one transaction, one cycle",
              dut.atomic_stall, 1'b0);
    in.instr = 32'h00108093;
    #1;
    check_bit("...and the instruction behind it interlocks on that rd",
              dut.hazard_rs1, 1'b1);

    in.pc = 32'h0000_07c0;
    present_and_fetch(32'h100120af);   // lr.w x1, (x2)
    @(posedge clk);
    #1;
    check_hex("lr.w carries its rd too", {27'b0, out.rd}, 32'd1);
    check_bit("...and no AMO bit", out.is_amo, 1'b0);
    check_bit("...and raises no atomic wait either", dut.atomic_stall, 1'b0);

    // The seventh stall reason: the platform has given the shared bus to
    // somebody else. Nothing single-hart raises it, so this is the only place
    // in the tree that says which arm of the publish block it takes, and both
    // ways of getting that wrong are silent -- a hold hands the executor an
    // instruction it has already run, and a bubble on the divide's cycle throws
    // away one it has not.
    in.pc = 32'h0000_0840;
    present_and_fetch(32'h00100093);   // addi x1, x0, 1
    @(posedge clk);
    #1;
    check_bit("the instruction issued", out.valid, 1'b1);
    check_hex("...into decoder_out", {27'b0, out.rd}, 32'd1);

    bus_wait = 1'b1;
    #1;
    check_bit("an ungranted bus is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", dut.issuing, 1'b0);
    check_hex("...and the pc holds, so the instruction asks again next cycle",
              next_pc, pc);
    check_bit("...and no trap is committed on a cycle that issued nothing",
              trap_entry, 1'b0);

    // The divider holds; every other reason bubbles. On a cycle with both, the
    // hold has to win or the instruction the executor has not taken is lost.
    divider_stall = 1'b1;
    #1;
    @(posedge clk);
    #1;
    check_bit("a bus wait coinciding with a divide holds decoder_out",
              out.valid, 1'b1);
    check_hex("...unchanged", {27'b0, out.rd}, 32'd1);
    divider_stall = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("a bus wait on its own bubbles decoder_out instead", out.valid, 1'b0);
    bus_wait = 1'b0;
    #1;
    @(posedge clk);
    #1;
    check_bit("...and the same instruction issues once the bus is granted",
              out.valid, 1'b1);
    check_hex("...as itself", {27'b0, out.rd}, 32'd1);

    // An interrupt is taken on a cycle that would otherwise have issued, so it
    // waits out an ungranted bus the way it waits out everything else. Without
    // this the trap would be committed for an instruction that never issued and
    // then issued again.
    interrupt_pending = 1'b1;
    bus_wait = 1'b1;
    #1;
    check_bit("an interrupt waits for the grant", trap_entry, 1'b0);
    bus_wait = 1'b0;
    #1;
    check_bit("...and is taken on the cycle the grant arrives", trap_entry, 1'b1);
    interrupt_pending = 1'b0;
    @(posedge clk);
    #1;
    @(posedge clk);
    #1;

    // A trapping atomic publishes none of its flags, so nothing downstream
    // starts a transaction for an instruction that faulted in decode.
    reg_rs1 = 32'h0001_0002;
    in.pc = 32'h0000_0800;
    present_and_fetch(32'h003120af);
    check_bit("a misaligned AMO commits a trap", trap_entry, 1'b1);
    @(posedge clk);
    #1;
    check_bit("...and retires with every atomic flag clear",
              out.is_amo || out.is_amoadd || out.is_lr || out.is_sc, 1'b0);
    check_hex("...writing no register", {27'b0, out.rd}, 32'b0);
    reg_rs1 = 32'b0;

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: decode vectors (xori immediate, ebreak/mret/wfi, Zicsr, M3 traps, the A extension)");
      $finish;
    end
  end
endmodule
