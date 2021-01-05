module alu (
  input logic         clk,
  input logic [31:0]  rs1,
  input logic [31:0]  rs2,
  input logic [4:0]   shamt,
  input logic         is_add,
  input logic         is_sub,
  input logic         is_xor,
  input logic         is_or,
  input logic         is_and,
  input logic         is_sll,
  input logic         is_slt,
  input logic         is_sltu,
  input logic         is_srl,
  input logic         is_sra,
  input logic         valid,
  output logic        ready,
  output logic [31:0] out
);
  always_ff @(posedge clk) begin
    if (!valid) begin
      ready <= 0;
    end else begin
      (* parallel_case, full_case *)
      case(1'b1)
        is_add: out <= rs1 + rs2;
        is_sub: out <= rs1 - rs2;
        is_sll: out <= rs1 << shamt;
        is_slt: out <= {31'b0, $signed(rs1) < $signed(rs2)};
        is_sltu: out <= {31'b0, rs1 < rs2};
        is_xor: out <= rs1 ^ rs2;
        is_srl: out <= rs1 >> shamt;
        is_sra: out <= $signed(rs1) >>> shamt;
        is_or: out <= rs1 | rs2;
        is_and: out <= rs1 & rs2;
      endcase // case (1'b1)
      ready <= 1;
    end
  end
endmodule
