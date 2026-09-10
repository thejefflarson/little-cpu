`default_nettype none
// Variant 4 of 4 -- see README.md: PDPW16KD instantiated directly, bypassing
// yosys's memory-inference pass, with variant 1's inferred parameters.
module prim_raw (
  input  wire clk,
  input  wire rst_in,
  output wire do0
);
  wire rst_logic = rst_in ^ clk;

  PDPW16KD #(
    .DATA_WIDTH_W(36),
    .DATA_WIDTH_R(36),
    .REGMODE("NOREG"),
    .RESETMODE("SYNC"),
    .ASYNC_RESET_RELEASE("SYNC"),
    .CSDECODE_W("0b000"),
    .CSDECODE_R("0b000"),
    .GSR("DISABLED")
  ) ram (
    .DI35(1'b0), .DI34(1'b0), .DI33(1'b0), .DI32(1'b0), .DI31(1'b0), .DI30(1'b0),
    .DI29(1'b0), .DI28(1'b0), .DI27(1'b0), .DI26(1'b0), .DI25(1'b0), .DI24(1'b0),
    .DI23(1'b0), .DI22(1'b0), .DI21(1'b0), .DI20(1'b0), .DI19(1'b0), .DI18(1'b0),
    .DI17(1'b0), .DI16(1'b0), .DI15(1'b0), .DI14(1'b0), .DI13(1'b0), .DI12(1'b0),
    .DI11(1'b0), .DI10(1'b0), .DI9(1'b0),  .DI8(1'b0),  .DI7(1'b0),  .DI6(1'b0),
    .DI5(1'b0),  .DI4(1'b0),  .DI3(1'b0),  .DI2(1'b0),  .DI1(1'b0),  .DI0(1'b0),
    .ADW8(1'b0), .ADW7(1'b0), .ADW6(1'b0), .ADW5(1'b0), .ADW4(1'b0), .ADW3(1'b0),
    .ADW2(1'b0), .ADW1(1'b0), .ADW0(1'b0),
    .BE3(1'b0), .BE2(1'b0), .BE1(1'b0), .BE0(1'b0),
    .CEW(1'b0), .CLKW(clk), .CSW2(1'b0), .CSW1(1'b0), .CSW0(1'b0),
    .ADR13(1'b0), .ADR12(1'b0), .ADR11(1'b0), .ADR10(1'b0), .ADR9(1'b0),
    .ADR8(1'b0), .ADR7(1'b0), .ADR6(1'b0), .ADR5(1'b0), .ADR4(1'b0),
    .ADR3(1'b0), .ADR2(1'b0), .ADR1(1'b0), .ADR0(1'b0),
    .CER(1'b1), .OCER(1'b1), .CLKR(clk), .CSR2(1'b0), .CSR1(1'b0), .CSR0(1'b0),
    .RST(rst_logic),
    .DO35(), .DO34(), .DO33(), .DO32(), .DO31(), .DO30(), .DO29(), .DO28(),
    .DO27(), .DO26(), .DO25(), .DO24(), .DO23(), .DO22(), .DO21(), .DO20(),
    .DO19(), .DO18(), .DO17(), .DO16(), .DO15(), .DO14(), .DO13(), .DO12(),
    .DO11(), .DO10(), .DO9(), .DO8(), .DO7(), .DO6(), .DO5(), .DO4(), .DO3(),
    .DO2(), .DO1(), .DO0(do0)
  );
endmodule
