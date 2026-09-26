`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"

// D vectors: decode, the hazard scoreboard, serialization and the stall broadcast.
// Branch resolution, the region test, CSR access and every trap but the timer interrupt
// moved to X and are test/executor_tb.v's job.
module decoder_tb;
  logic clk = 0;
  always #5 clk = ~clk;

  logic reset;
  fetcher_output in;
  logic x_busy = 1'b0;
  executor_output executor_out = '0;
  logic fetch_stall = 1'b0;
  logic bus_wait = 1'b0;
  logic bus_request;
  logic imem_fault = 1'b0;
  logic accessor_out_valid = 1'b0;
  logic issuing;
  logic [31:0] predicted_pc;
  logic [4:0] read_rs1, read_rs2;
  logic interrupt_pending = 1'b0;
  logic x_redirect = 1'b0;
  dx_output out;

  decoder dut (
    .clk(clk),
    .reset(reset),
    .in(in),
    .x_busy(x_busy),
    .executor_out(executor_out),
    .fetch_stall(fetch_stall),
    .bus_wait(bus_wait),
    .bus_request(bus_request),
    .imem_fault(imem_fault),
    .accessor_out_valid(accessor_out_valid),
    .issuing(issuing),
    .predicted_pc(predicted_pc),
    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .interrupt_pending(interrupt_pending),
    .x_redirect(x_redirect),
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

  // `stall` is the OR of these seven raw signals, graded against the other declared sites.
  always @(clk) begin
    if (dut.stall !== (dut.hazard_rs1 || dut.hazard_rs2 || dut.serialize ||
                       dut.fetch_stall || dut.atomic_stall || dut.x_busy || dut.bus_wait)) begin
      $display("MISMATCH stall is not the OR of the seven named signals: stall=%b rs1=%b rs2=%b serialize=%b fetch=%b atomic=%b x_busy=%b bus=%b",
               dut.stall, dut.hazard_rs1, dut.hazard_rs2, dut.serialize, dut.fetch_stall,
               dut.atomic_stall, dut.x_busy, dut.bus_wait);
      errors++;
    end
    if (dut.stall_own !== (dut.hazard_rs1 || dut.hazard_rs2 || dut.serialize ||
                           dut.fetch_stall || dut.atomic_stall || dut.x_busy)) begin
      $display("MISMATCH stall_own is not the OR of the six named signals that exclude bus_wait");
      errors++;
    end
  end

  task automatic present(input logic [31:0] instr);
    begin
      in.instr = instr;
      #1;
    end
  endtask

  task automatic settle_issue(input logic [31:0] instr);
    begin
      present(instr);
      @(posedge clk);
      #1;
    end
  endtask

  initial begin
    reset = 1;
    in = '0;
    in.valid = 1'b1;   // rtl/fetcher.v drives this to exactly !reset
    repeat (2) @(posedge clk);
    #1;
    reset = 0;

    present(32'hfff14093);   // xori x1, x2, -1 -- immediate is still D's own job
    check_hex("xori immediate", dut.immediate, 32'hffffffff);
    check_hex("...and its rs1 field", {27'b0, dut.rs1}, 32'd2);
    check_hex("...and its rd field", {27'b0, dut.rd}, 32'd1);

    present(32'h00d11093);   // slli x1, x2, 13 -- rs2 IS the shamt, regsel maps the field
    check_hex("slli's rs2 field carries its shift amount", {27'b0, dut.rs2}, 32'd13);
    present(32'h40d15093);   // srai x1, x2, 13 -- funct7 sits right above the amount
    check_hex("srai's rs2 field carries its amount, nothing above it", {27'b0, dut.rs2}, 32'd13);
    present(32'h0000_8035);  // c.srli x8, 13
    check_hex("a compressed right shift's amount is zero-extended too", {27'b0, dut.rs2}, 32'd13);
    present(32'h0000_00b6);  // c.slli x1, 13
    check_hex("...and so is a compressed left shift's", {27'b0, dut.rs2}, 32'd13);

    present(32'h123450b7);   // lui x1, 0x12345 -- reaches X as an add of imm and zero
    check_hex("lui's immediate is the U-type field, shifted into place",
              dut.immediate, 32'h1234_5000);
    present(32'h0000_6085);  // c.lui x1, 1
    check_hex("c.lui's immediate takes the same shape", dut.immediate, 32'h0000_1000);

    in.pc = 32'h0000_00a0;   // predicted_pc is D's own guess, never F's word-granular one
    present(32'h0000_0013);  // addi x0, x0, 0 -- uncompressed
    check_hex("an uncompressed word predicts pc+4", predicted_pc, 32'h0000_00a4);
    present(32'h0000_0001);  // c.nop -- compressed
    check_hex("a compressed word predicts pc+2", predicted_pc, 32'h0000_00a2);

    in.pc = 32'h0000_0040;   // unstalled: issues this cycle, lands whole in out next edge
    present(32'h00100093);   // addi x1, x0, 1
    check_bit("an unstalled instruction issues the cycle it is presented", issuing, 1'b1);
    @(posedge clk);
    #1;
    check_bit("...and lands in out.valid", out.valid, 1'b1);
    check_hex("...carrying its rd", {27'b0, out.rd}, 32'd1);
    check_hex("...and the pc it was fetched at", out.pc, 32'h0000_0040);
    check_bit("...as an add", out.is_add, 1'b1);

    present(32'h0000_987d);   // c.andi x8, -1 -- shares a quadrant/funct3 with c.sub
    check_bit("c.andi decodes", dut.instr_candi, 1'b1);
    check_bit("...as an andi", dut.instr_andi, 1'b1);
    present(32'h0000_8c05);   // c.sub x8, x9
    check_bit("c.sub decodes", dut.instr_csub, 1'b1);
    check_bit("...as a sub", dut.instr_sub, 1'b1);
    present(32'h0000_9c05);   // the same row with instr[12] set: c.subw
    check_bit("the RV64 row above it is not a sub", dut.instr_csub, 1'b0);
    present(32'h0000_9035);   // c.srli with shamt[5] set -- reserved in RV32
    check_bit("a compressed shift with shamt[5] set is not a shift", dut.instr_csrli, 1'b0);

    present(32'h0000_6085);   // c.lui x1, 1
    check_bit("c.lui with a non-zero immediate decodes", dut.instr_clui, 1'b1);
    present(32'h0000_6081);   // the same, immediate zero -- reserved
    check_bit("...and the reserved zero-immediate form does not", dut.instr_clui, 1'b0);

    // SYSTEM with funct3 zero is told apart by funct12 alone, so rs1/rd must read zero.
    present(32'h0000_0073);   // ecall
    check_bit("ecall decodes", dut.instr_ecall, 1'b1);
    present(32'h0000_00f3);   // the same funct12, rd = x1
    check_bit("...but not with a non-zero rd field", dut.instr_ecall, 1'b0);
    present(32'h0000_8073);   // the same funct12, rs1 = x1
    check_bit("...nor with a non-zero rs1 field", dut.instr_ecall, 1'b0);
    present(32'h00100073);    // ebreak
    check_bit("ebreak decodes", dut.instr_ebreak, 1'b1);
    present(32'h30200073);    // mret
    check_bit("mret decodes as mret, not ebreak", dut.instr_mret, 1'b1);
    check_bit("...", dut.instr_ebreak, 1'b0);
    present(32'h10500073);    // wfi
    check_bit("wfi decodes as wfi, not ebreak", dut.instr_wfi, 1'b1);
    check_bit("...", dut.instr_ebreak, 1'b0);

    present(32'h00b6252f);   // amoadd.w -- the first of the eleven A encodings
    check_bit("amoadd.w decodes", dut.instr_amoadd, 1'b1);
    present(32'h08b6252f);
    check_bit("amoswap.w decodes", dut.instr_amoswap, 1'b1);
    present(32'h20b6252f);
    check_bit("amoxor.w decodes", dut.instr_amoxor, 1'b1);
    present(32'h40b6252f);
    check_bit("amoor.w decodes", dut.instr_amoor, 1'b1);
    present(32'h60b6252f);
    check_bit("amoand.w decodes", dut.instr_amoand, 1'b1);
    present(32'h80b6252f);
    check_bit("amomin.w decodes", dut.instr_amomin, 1'b1);
    present(32'ha0b6252f);
    check_bit("amomax.w decodes", dut.instr_amomax, 1'b1);
    present(32'hc0b6252f);
    check_bit("amominu.w decodes", dut.instr_amominu, 1'b1);
    present(32'he0b6252f);
    check_bit("amomaxu.w decodes", dut.instr_amomaxu, 1'b1);
    present(32'h1006252f);
    check_bit("lr.w decodes", dut.instr_lr, 1'b1);
    check_bit("...and is not an AMO", dut.instr_amo, 1'b0);
    present(32'h18b6252f);
    check_bit("sc.w decodes", dut.instr_sc, 1'b1);
    check_bit("...and is not an AMO either", dut.instr_amo, 1'b0);

    present(32'h28b6252f);   // funct5 = 00101, which names nothing
    check_bit("an unassigned funct5 is not an atomic", dut.instr_atomic, 1'b0);
    present(32'h00b6352f);   // amoadd.d -- funct3 = 011, RV64 only
    check_bit("the doubleword width is not implemented here", dut.instr_atomic, 1'b0);
    present(32'h1056252f);   // lr.w with a non-zero rs2 field
    check_bit("lr.w with a non-zero rs2 field is not an lr.w", dut.instr_lr, 1'b0);

    present(32'h02b6252f);   // amoadd.w.rl
    check_bit("amoadd.w.rl is the same instruction", dut.instr_amoadd, 1'b1);
    present(32'h04b6252f);   // amoadd.w.aq
    check_bit("...and so is amoadd.w.aq", dut.instr_amoadd, 1'b1);
    present(32'h06b6252f);   // amoadd.w.aqrl
    check_bit("...and amoadd.w.aqrl", dut.instr_amoadd, 1'b1);
    check_hex("...and none of them puts anything in the immediate", dut.immediate, 32'b0);

    present(32'h00b6252f);
    check_bit("an AMO uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and rs2", dut.uses_rs2, 1'b1);
    present(32'h18b6252f);
    check_bit("sc.w uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and rs2, the word it stores", dut.uses_rs2, 1'b1);
    present(32'h1006252f);
    check_bit("lr.w uses rs1", dut.uses_rs1, 1'b1);
    check_bit("...and NOT rs2: that field is an encoding constant", dut.uses_rs2, 1'b0);

    present(32'h340515f3);   // csrrw a1, mscratch, a0 -- the write-data mux is X's job now
    check_hex("csrrw's address field", {20'b0, dut.instr[31:20]}, 32'h340);
    check_bit("csrrw is not an immediate form", dut.is_csr_imm, 1'b0);
    check_bit("csrrw uses rs1", dut.uses_rs1, 1'b1);
    present(32'h340fe573);   // csrrsi a0, mscratch, 0x1f
    check_bit("csrrsi is an immediate form", dut.is_csr_imm, 1'b1);
    check_bit("...so it does not use rs1", dut.uses_rs1, 1'b0);

    // Hazards: dx_match against a same-cycle-ready producer forwards from the X/M
    // register instead of stalling; a match on a producer that will not be ready next
    // cycle (a load, an AMO, `lr.w`, `sc.w`) still stalls, and so does a CSR access's own
    // rs1, which never reads the forwarded value. A match against `executor_out` (two
    // instructions back) needs no forwarding path at all -- the regfile's own
    // write-through bypass reaches it in time -- except when that producer's own result
    // is not yet unpacked (rd_ready low).
    in.pc = 32'h0000_00c0;
    settle_issue(32'h000100b3);   // add x1, x2, x0 -- reads x2, writes x1
    check_bit("a producer reaches out", out.valid, 1'b1);
    check_hex("...carrying the rd the next check depends on", {27'b0, out.rd}, 32'd1);
    check_bit("...and it will publish a ready result next cycle", dut.out_has_result, 1'b1);
    present(32'h00008233);        // add x4, x1, x0 -- reads x1, still in `out`
    check_bit("the instruction behind it interlocks on out.rd", dut.dx_match_rs1, 1'b1);
    check_bit("...but the producer is ready, so it forwards instead of stalling",
              dut.fwd_rs1, 1'b1);
    check_bit("...which is no hazard", dut.hazard_rs1, 1'b0);
    check_bit("...and no stall", dut.stall, 1'b0);
    check_bit("...so it issues", issuing, 1'b1);
    @(posedge clk);
    #1;
    check_bit("the forward select rode along into out", out.fwd_rs1, 1'b1);

    x_redirect = 1'b1;
    @(posedge clk);
    #1;
    x_redirect = 1'b0;
    check_bit("drained ahead of the load-use vector", out.valid, 1'b0);

    in.pc = 32'h0000_00d0;
    settle_issue(32'h0000a103);   // lw x2, 0(x1) -- a load, never ready next cycle
    check_bit("a load producer reaches out", out.valid, 1'b1);
    check_bit("...and out_has_result correctly excludes it", dut.out_has_result, 1'b0);
    present(32'h00010233);        // add x4, x2, x0 -- reads x2, the load's rd
    check_bit("dx_match against a load", dut.dx_match_rs1, 1'b1);
    check_bit("...raises no forward select", dut.fwd_rs1, 1'b0);
    check_bit("...so it is a genuine (load-use) hazard", dut.hazard_rs1, 1'b1);
    check_bit("...and it stalls", issuing, 1'b0);

    x_redirect = 1'b1;
    @(posedge clk);
    #1;
    x_redirect = 1'b0;
    check_bit("drained ahead of the ex_match vectors", out.valid, 1'b0);

    executor_out = '0;
    executor_out.valid = 1'b1;
    executor_out.rd = 5'd2;
    executor_out.rd_ready = 1'b1;
    in.pc = 32'h0000_00e4;
    present(32'h00008233);        // add x4, x1, x0 -- reads x1 (x0 + x1), and x1 != x2
    check_bit("a producer only in executor_out and a mismatched rs1 raises no hazard",
              dut.ex_match_rs1, 1'b0);
    in.instr = 32'h00010233;      // add x4, x2, x0 -- reads x2, matching executor_out.rd
    #1;
    check_bit("a match against executor_out.rd, ready", dut.ex_match_rs1, 1'b1);
    check_bit("...raises no hazard: the regfile's own bypass reaches it in time",
              dut.hazard_rs1, 1'b0);

    executor_out.rd_ready = 1'b0;   // the same match, but not yet unpacked (a pending load)
    #1;
    check_bit("a match against executor_out.rd, not yet unpacked, still stalls",
              dut.hazard_rs1, 1'b1);

    executor_out.rd = 5'd0;   // x0 is exempt on both sides
    executor_out.rd_ready = 1'b0;
    in.instr = 32'h00000233;      // add x4, x0, x0
    #1;
    check_bit("x0 raises no hazard even when it matches a producer's rd",
              dut.hazard_rs1, 1'b0);
    executor_out = '0;

    executor_out.valid = 1'b1;
    executor_out.rd = 5'd3;
    in.instr = 32'h00310063;      // beq x2, x3, 0 -- reads x3, producer not yet unpacked
    #1;
    check_bit("a branch's rs2 raises a hazard like any other read", dut.hazard_rs2, 1'b1);
    in.instr = 32'h00312023;      // sw x3, 0(x2) -- data operand is x3
    #1;
    check_bit("...and so does a store's data operand", dut.hazard_rs2, 1'b1);
    in.instr = 32'h00d10093;      // addi x1, x2, 13 -- rs2 field is a shamt/imm, not read
    #1;
    check_bit("a math-immediate's rs2 field is never a hazard", dut.hazard_rs2, 1'b0);
    executor_out = '0;

    // A CSR access's own rs1 feeds csr_arg, which reads reg_rs1 verbatim: dx_match
    // against a ready producer must still stall, never forward.
    x_redirect = 1'b1;
    @(posedge clk);
    #1;
    x_redirect = 1'b0;
    check_bit("drained ahead of the CSR forwarding-exclusion vector", out.valid, 1'b0);

    in.pc = 32'h0000_00f0;
    settle_issue(32'h000100b3);   // add x1, x2, x0 -- a ready producer, rd = x1
    present(32'h340095f3);        // csrrw a1, mscratch, x1 -- rs1 = x1, matches
    check_bit("a CSR access dx_matches its own producer", dut.dx_match_rs1, 1'b1);
    check_bit("...but never forwards", dut.fwd_rs1, 1'b0);
    check_bit("...so it still stalls", dut.hazard_rs1, 1'b1);

    // `out` still holds "add x1, x2, x0" from the hazard vectors above; drain it so the
    // serialize checks below start from a genuinely empty pipe, matching their own comment.
    x_redirect = 1'b1;
    @(posedge clk);
    #1;
    x_redirect = 1'b0;
    check_bit("out is drained ahead of the serialize block", out.valid, 1'b0);

    executor_out.valid = 1'b1;   // serialize waits for out/executor_out/accessor all empty
    present(32'h340515f3);   // csrrw a1, mscratch, a0
    check_bit("a CSR instruction serializes while the pipe is busy", dut.serialize, 1'b1);
    check_bit("...which is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", issuing, 1'b0);
    executor_out.valid = 1'b0;
    #1;
    check_bit("the drained pipe releases it", dut.pipe_drained, 1'b1);
    check_bit("...so it issues now", issuing, 1'b1);

    accessor_out_valid = 1'b1;
    present(32'h30200073);   // mret
    check_bit("mret serializes against a store still in the accessor too",
              dut.serialize, 1'b1);
    accessor_out_valid = 1'b0;
    #1;
    check_bit("...and issues once that drains as well", issuing, 1'b1);

    executor_out.valid = 1'b1;
    present(32'h0000100f);   // fence.i
    check_bit("fence.i serializes while the pipe is busy", dut.serialize, 1'b1);
    executor_out.valid = 1'b0;
    #1;
    check_bit("...and issues once drained", issuing, 1'b1);

    executor_out.valid = 1'b1;
    present(32'h0ff0000f);   // fence iorw, iorw -- a plain fence
    check_bit("a plain fence does not serialize", dut.serialize, 1'b0);
    executor_out.valid = 1'b0;

    in.pc = 32'h0000_0700;   // an AMO about to launch holds D for the one cycle X takes it
    settle_issue(32'h003120af);   // amoadd.w x1, x3, (x2)
    check_bit("...into out", out.valid, 1'b1);
    check_bit("...and its own class flag", out.is_amoadd, 1'b1);
    check_bit("an AMO in flight raises the atomic wait", dut.atomic_stall, 1'b1);
    check_bit("...which is a stall", dut.stall, 1'b1);
    check_bit("...so nothing issues", issuing, 1'b0);
    @(posedge clk);
    #1;
    check_bit("the atomic wait bubbles out rather than holding it", out.valid, 1'b0);
    check_bit("...and is over after that one cycle", dut.atomic_stall, 1'b0);

    in.pc = 32'h0000_0780;
    settle_issue(32'h183120af);   // sc.w x1, x3, (x2)
    check_bit("sc.w raises no atomic wait: one transaction, one cycle",
              dut.atomic_stall, 1'b0);
    in.pc = 32'h0000_07c0;
    settle_issue(32'h100120af);   // lr.w x1, (x2)
    check_bit("nor does lr.w", dut.atomic_stall, 1'b0);

    in.pc = 32'h0000_0840;   // x_busy holds out and its pair; every other reason bubbles
    settle_issue(32'h00100093);   // addi x1, x0, 1
    check_bit("the instruction issued", out.valid, 1'b1);
    check_hex("...into out", {27'b0, out.rd}, 32'd1);

    x_busy = 1'b1;
    in.instr = 32'hdead_beef;     // an unrelated word must not leak into `out`
    #1;
    check_hex("while x_busy holds, out presents ITS OWN pair, not the new word's",
              {27'b0, read_rs1}, {27'b0, out.rs1});
    check_hex("...on rs2 too", {27'b0, read_rs2}, {27'b0, out.rs2});
    @(posedge clk);
    #1;
    check_bit("x_busy holds out unchanged across the edge", out.valid, 1'b1);
    check_hex("...with the same rd", {27'b0, out.rd}, 32'd1);
    @(posedge clk);
    #1;
    check_bit("...for as long as x_busy stays asserted", out.valid, 1'b1);
    check_hex("...still the same rd", {27'b0, out.rd}, 32'd1);
    x_busy = 1'b0;
    in.instr = 32'h00008233;      // add x4, x1, x0 -- dx_matches out.rd once x_busy clears
    #1;
    check_bit("a dx_match against the just-released, ready out.rd forwards", dut.fwd_rs1, 1'b1);
    check_bit("...raising no hazard", dut.hazard_rs1, 1'b0);
    check_bit("...so it issues rather than stalling", issuing, 1'b1);

    in.pc = 32'h0000_0880;
    settle_issue(32'h00100093);   // addi x1, x0, 1
    fetch_stall = 1'b1;
    #1;
    check_bit("a stolen fetch window is a stall", dut.fetch_stall, 1'b1);
    check_bit("...not x_busy", dut.x_busy, 1'b0);
    @(posedge clk);
    #1;
    check_bit("...and it bubbles out, unlike x_busy", out.valid, 1'b0);
    fetch_stall = 1'b0;

    in.pc = 32'h0000_08c0;   // x_redirect discards a wrong-path word unconditionally
    settle_issue(32'h00100093);   // addi x1, x0, 1 -- a harmless word, otherwise issuable
    check_bit("an unrelated word would issue on its own", issuing, 1'b1);
    x_redirect = 1'b1;
    #1;
    check_bit("issuing still tracks !stall, not the kill", issuing, 1'b1);
    @(posedge clk);
    #1;
    check_bit("...so out is bubbled, not the wrong-path word", out.valid, 1'b0);
    x_redirect = 1'b0;

    in.pc = 32'h0000_0900;   // bus_request over-asks on purpose, never on a stalled cycle
    present(32'h00100093);   // addi x1, x0, 1 -- not a memory access
    check_bit("a non-memory instruction asks for nothing", bus_request, 1'b0);
    present(32'h00062583);   // lw a1, 0(a2)
    check_bit("a load asks for the bus", bus_request, 1'b1);
    present(32'h00b62023);   // sw a1, 0(a2)
    check_bit("...and so does a store", bus_request, 1'b1);
    present(32'h1006252f);   // lr.w
    check_bit("...and an atomic", bus_request, 1'b1);

    executor_out.valid = 1'b1;
    executor_out.rd = 5'd12;      // a2, the base register "lw a1, 0(a2)" reads
    in.instr = 32'h00062583;      // lw a1, 0(a2) -- blocked by a hazard on rs1
    #1;
    check_bit("a stalled cycle asks for nothing, even for a load", bus_request, 1'b0);
    executor_out = '0;

    in.pc = 32'h0000_0940;   // the bubble is pc only; the trap commit is X's job later
    present(32'h00100093);   // addi x1, x0, 1 -- a harmless victim
    interrupt_pending = 1'b1;
    #1;
    check_bit("an armed interrupt is taken instead of issuing the victim", issuing, 1'b1);
    @(posedge clk);
    #1;
    check_bit("the bubble reaches out", out.valid, 1'b1);
    check_bit("...marked as the interrupt", out.is_interrupt, 1'b1);
    check_hex("...at the pc that would have issued", out.pc, 32'h0000_0940);
    check_hex("...with no rd", {27'b0, out.rd}, 32'b0);
    check_hex("...and no instruction word", out.instr, 32'b0);
    interrupt_pending = 1'b0;

    reset = 1;   // zeroes out unconditionally
    #1;
    @(posedge clk);
    #1;
    check_bit("reset zeroes out.valid", out.valid, 1'b0);
    if (out !== '0) begin
      $display("MISMATCH reset does not zero the whole out struct");
      errors++;
    end
    reset = 0;

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: D vectors (decode, hazard, serialize, atomic wait, x_busy hold/bubble, x_redirect, bus_request, the interrupt bubble)");
      $finish;
    end
  end
endmodule
