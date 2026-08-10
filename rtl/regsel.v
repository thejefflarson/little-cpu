`timescale 1 ns / 1 ps
`default_nettype none
// The register numbers a fetched word names, for any 32-bit word. Two
// instances read this one mapping: one over the instruction being issued, one
// over the word after it, which decode asks the register file for a cycle
// early. Written out twice instead, the two copies could disagree, and then the
// early read would miss because the same bytes decoded differently rather than
// because the flow went somewhere else -- a lost cycle nothing in this tree
// grades.
module regsel (
  input  logic [31:0] word,
  output logic [4:0]  rs1,
  output logic [4:0]  rs2
);
  // The upper half of a compressed word belongs to the instruction after it, so
  // zero it the way decode zeroes its own. Every compressed encoding with no
  // register in an arm below falls to the default arms, which read the
  // uncompressed field positions; masked, those read x0, which is what such an
  // encoding means.
  logic [31:0] instr;
  assign instr = (word[1:0] == 2'b11) ? word : {16'b0, word[15:0]};

  // Named continuous assigns rather than part-selects inside the always_comb
  // blocks below, for the reason rtl/decoder.v states: iverilog cannot build a
  // precise sensitivity entry for a constant select there.
  logic [4:0] rd_field, rs1_field, rs2_field, c_rs2_field;
  logic [2:0] c_rd_rs1_prime, c_rs2_prime;
  assign rd_field       = instr[11:7];
  assign rs1_field      = instr[19:15];
  assign rs2_field      = instr[24:20];
  assign c_rs2_field    = instr[6:2];
  assign c_rd_rs1_prime = instr[9:7];
  assign c_rs2_prime    = instr[4:2];

  logic [1:0] quadrant, cfunct2, cmath_funct2;
  logic [2:0] cfunct3;
  logic [3:0] cfunct4;
  logic [5:0] cfunct6;
  assign quadrant     = instr[1:0];
  assign cfunct3      = instr[15:13];
  assign cfunct2      = instr[11:10];
  assign cmath_funct2 = instr[6:5];
  assign cfunct4      = instr[15:12];
  assign cfunct6      = instr[15:10];

  // Both reserved-immediate tests are transcribed from rtl/decoder.v's
  // immediates rather than reduced to the bit range they cover: a zero
  // immediate makes each of these a different instruction, which names a
  // different register.
  logic [31:0] caddi4spn_immediate, caddi16sp_immediate;
  assign caddi4spn_immediate = {22'b0, instr[10:7], instr[12:11], instr[5], instr[6], 2'b00};
  assign caddi16sp_immediate = {{22{instr[12]}}, instr[12], instr[4:3], instr[5], instr[2], instr[6], 4'b0};

  logic instr_clwsp, instr_cswsp, instr_caddi4spn, instr_clw, instr_csw, instr_cbeqz, instr_cbnez,
    instr_csrli, instr_csrai, instr_candi, instr_cand, instr_cor, instr_cxor, instr_csub,
    instr_cjr, instr_cjalr, instr_cslli, instr_cli, instr_cmv, instr_caddi, instr_caddi16sp,
    instr_cadd;
  assign instr_clwsp = quadrant == 2'b10 && cfunct3 == 3'b010 && instr[11:7] != 5'b0;
  assign instr_cswsp = quadrant == 2'b10 && cfunct3 == 3'b110;
  assign instr_caddi4spn = quadrant == 2'b00 && cfunct3 == 3'b000 && caddi4spn_immediate != 0;
  assign instr_clw = quadrant == 2'b00 && cfunct3 == 3'b010;
  assign instr_csw = quadrant == 2'b00 && cfunct3 == 3'b110;
  assign instr_cbeqz = quadrant == 2'b01 && cfunct3 == 3'b110;
  assign instr_cbnez = quadrant == 2'b01 && cfunct3 == 3'b111;
  assign instr_csrli = quadrant == 2'b01 && cfunct4 == 4'b1000 && cfunct2 == 2'b00;
  assign instr_csrai = quadrant == 2'b01 && cfunct4 == 4'b1000 && cfunct2 == 2'b01;
  assign instr_candi = quadrant == 2'b01 && cfunct3 == 3'b100 && cfunct2 == 2'b10;
  assign instr_csub = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b00;
  assign instr_cxor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b01;
  assign instr_cor = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b10;
  assign instr_cand = quadrant == 2'b01 && cfunct6 == 6'b100011 && cmath_funct2 == 2'b11;
  assign instr_cjr = quadrant == 2'b10 && cfunct3 == 3'b100 && instr[12] == 0 && instr[6:2] == 0 &&
    instr[11:7] != 0;
  assign instr_cjalr = quadrant == 2'b10 && cfunct3 == 3'b100 && instr[12] == 1 && instr[6:2] == 0 &&
    instr[11:7] != 0;
  assign instr_cslli = quadrant == 2'b10 && cfunct4 == 4'b0000;
  assign instr_cli = quadrant == 2'b01 && cfunct3 == 3'b010;
  assign instr_cmv = quadrant == 2'b10 && cfunct4 == 4'b1000 && instr[6:2] != 0;
  assign instr_cadd = quadrant == 2'b10 && cfunct4 == 4'b1001 && instr[6:2] != 0;
  assign instr_caddi = quadrant == 2'b01 && cfunct3 == 3'b000;
  assign instr_caddi16sp = quadrant == 2'b01 && cfunct3 == 3'b011 && instr[11:7] == 2 &&
    caddi16sp_immediate != 0;

  always_comb begin
    (* parallel_case, full_case *)
    case (1'b1)
      instr_clwsp || instr_cswsp || instr_caddi4spn: rs1 = 2;
      instr_clw || instr_csw || instr_cbeqz || instr_cbnez ||
        instr_csrai || instr_csrli || instr_candi || instr_cand ||
        instr_cor || instr_cxor || instr_csub: rs1 = {2'b01, c_rd_rs1_prime};
      instr_cjr || instr_cjalr || instr_cslli: rs1 = rd_field;
      instr_cli || instr_cmv: rs1 = 0;
      instr_caddi || instr_caddi16sp || instr_cadd: rs1 = rd_field;
      default: rs1 = rs1_field;
    endcase // case (1'b1)
  end

  always_comb begin
    (* parallel_case, full_case *)
    case(1'b1)
      instr_cswsp || instr_cslli || instr_csrai || instr_csrli || instr_cmv || instr_cadd: rs2 = c_rs2_field;
      instr_csw || instr_cand || instr_cor || instr_cxor || instr_csub: rs2 = {2'b01, c_rs2_prime};
      instr_cbeqz || instr_cbnez: rs2 = 0;
      default: rs2 = rs2_field;
    endcase
  end

 `ifdef FORMAL
  // Both case statements above are marked one-hot for synthesis, and each
  // assertion here is one of those arm lists. Without it an encoding change
  // that made two arms match at once would leave synthesis free to pick either,
  // and nothing would say so. Each list is transcribed, not shared with the
  // statement it describes, so adding an arm to one means adding it here too.
  always_comb assert($onehot0({
    instr_clwsp || instr_cswsp || instr_caddi4spn,
    instr_clw || instr_csw || instr_cbeqz || instr_cbnez ||
      instr_csrai || instr_csrli || instr_candi || instr_cand ||
      instr_cor || instr_cxor || instr_csub,
    instr_cjr || instr_cjalr || instr_cslli,
    instr_cli || instr_cmv,
    instr_caddi || instr_caddi16sp || instr_cadd}));
  always_comb assert($onehot0({
    instr_cswsp || instr_cslli || instr_csrai || instr_csrli || instr_cmv || instr_cadd,
    instr_csw || instr_cand || instr_cor || instr_cxor || instr_csub,
    instr_cbeqz || instr_cbnez}));
 `endif
endmodule
