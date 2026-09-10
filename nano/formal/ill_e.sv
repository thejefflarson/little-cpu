// RV32E's register-naming restriction, checked against the reference model below
// (`ill_e_top`) rather than `nano.v`, which has no 16-register limit or CSR layer yet.

module ill_e_top (
  input logic        clk,
  input logic [31:0] instr
);
  logic reset = 1;
  always_ff @(posedge clk)
    reset <= 0;

  wire       is32   = instr[1:0] == 2'b11;
  wire [6:0] opcode = instr[6:0];
  wire [4:0] rd     = instr[11:7];
  wire [4:0] rs1    = instr[19:15];
  wire [4:0] rs2    = instr[24:20];

  wire class_load   = is32 && opcode == 7'b0000011;
  wire class_opimm  = is32 && opcode == 7'b0010011;
  wire class_auipc  = is32 && opcode == 7'b0010111;
  wire class_store  = is32 && opcode == 7'b0100011;
  wire class_op     = is32 && opcode == 7'b0110011;
  wire class_lui    = is32 && opcode == 7'b0110111;
  wire class_branch = is32 && opcode == 7'b1100011;
  wire class_jalr   = is32 && opcode == 7'b1100111;
  wire class_jal    = is32 && opcode == 7'b1101111;

  wire uses_rd  = class_load || class_opimm || class_auipc || class_op ||
                  class_lui  || class_jalr  || class_jal;
  wire uses_rs1 = class_load || class_opimm || class_store || class_op ||
                  class_branch || class_jalr;
  wire uses_rs2 = class_store || class_op || class_branch;

  wire e_illegal = (uses_rd && rd[4]) || (uses_rs1 && rs1[4]) || (uses_rs2 && rs2[4]);

  logic       trap;
  logic [4:0] rd_addr;
  logic       mem_write;
  assign trap      = e_illegal;
  assign rd_addr   = e_illegal ? 5'd0 : rd;
  assign mem_write = e_illegal ? 1'b0 : class_store;

  always_comb begin
    if (!reset && e_illegal) begin
      assert (trap);
      assert (rd_addr == 5'd0);
      assert (!mem_write);
    end
  end

  wire live = !reset;
  cover property (live && e_illegal && class_load);
  cover property (live && e_illegal && class_opimm);
  cover property (live && e_illegal && class_auipc);
  cover property (live && e_illegal && class_store);
  cover property (live && e_illegal && class_op);
  cover property (live && e_illegal && class_lui);
  cover property (live && e_illegal && class_branch);
  cover property (live && e_illegal && class_jalr);
  cover property (live && e_illegal && class_jal);
endmodule
