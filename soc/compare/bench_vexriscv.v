`timescale 1 ns / 1 ps
`default_nettype none
// VexRiscv in the same harness as soc/compare/bench_littlecpu.v: same ROM depth,
// the same rtl/memory.v at the same base and depth, the same three pads, the
// same program image, the same part and the same seeds.
//
// The core is not vendored. It is read straight out of the SHA-pinned
// riscv-formal clone at formal/riscv-formal/cores/VexRiscv/VexRiscv.v, which is
// the generated Verilog for that project's FormalSimple configuration, so no
// Scala toolchain is involved and nothing here can drift from the pin.
//
// **That configuration is RV32IC.** The compressed decoder is there and runs --
// soc/compare/dhry.lds' image is built for it -- but there is no M extension, no
// CSR file, no traps and no interrupt. Roughly half of why it is a quarter of
// this core's size. Any number taken from this file is a measurement of two
// different ISAs and must be quoted as one.
//
// It has NO DATA PATH TO THE ROM: the memory below is a read port for fetch and
// nothing else, so a load from a ROM address reads back zero. Keep any program
// run here from putting read-only data in ROM. soc/compare/bench.S has none,
// which is why this went unnoticed until a benchmark with string literals ran.
//
// Its `rvfi_*` outputs are deleted in synthesis rather than tied off here --
// `delete -port VexRiscv/rvfi_*`, the same technique formal/check-nonperturbation.py
// uses on this core. Left connected they present 556 SB_IO and no ice40 package
// can place them.
//
// ---- the bus ---------------------------------------------------------------
//
// Both sides are valid/ready with the command always accepted and the response
// one cycle later, which is what the memories on the other harness already do.
// `dBus_rsp_ready` is an INPUT and means "response valid" -- the name is the
// generator's.
//
// Store data arrives replicated across all four byte lanes and the core shifts
// load data itself (`writeBack_DBusSimplePlugin_rspShifted`), so the byte strobe
// below is the size mask shifted by the low address bits and the read side hands
// back the whole aligned word. Get that backwards and `sb` writes a word.
module bench_vexriscv #(
  parameter integer ROM_WORDS = 1024,
  parameter integer RAM_WORDS = 512,
  parameter INIT_ROM = "soc/compare/rom_flat.hex"
) (
  input  logic clk,
  output logic led0_n,
  output logic led1_n
);
  localparam int ROM_BITS = $clog2(ROM_WORDS);

  logic [3:0] por_count = 4'b0;
  logic       por_done  = 1'b0;
  logic       reset     = 1'b1;
  always_ff @(posedge clk) begin
    if (!por_done) begin
      por_count <= por_count + 4'd1;
      if (por_count == 4'hf) por_done <= 1'b1;
    end
    reset <= !por_done;
  end

  logic        ibus_cmd_valid, ibus_rsp_valid;
  logic [31:0] ibus_cmd_pc, ibus_rsp_inst;
  logic        dbus_cmd_valid, dbus_cmd_wr, dbus_rsp_valid;
  logic [31:0] dbus_cmd_address, dbus_cmd_data, dbus_rsp_data;
  logic [1:0]  dbus_cmd_size;

  VexRiscv riscv (
    .clk(clk),
    .reset(reset),
    .iBus_cmd_valid(ibus_cmd_valid),
    .iBus_cmd_ready(1'b1),
    .iBus_cmd_payload_pc(ibus_cmd_pc),
    .iBus_rsp_valid(ibus_rsp_valid),
    .iBus_rsp_payload_error(1'b0),
    .iBus_rsp_payload_inst(ibus_rsp_inst),
    .dBus_cmd_valid(dbus_cmd_valid),
    .dBus_cmd_ready(1'b1),
    .dBus_cmd_payload_wr(dbus_cmd_wr),
    .dBus_cmd_payload_address(dbus_cmd_address),
    .dBus_cmd_payload_data(dbus_cmd_data),
    .dBus_cmd_payload_size(dbus_cmd_size),
    .dBus_rsp_ready(dbus_rsp_valid),
    .dBus_rsp_error(1'b0),
    .dBus_rsp_data(dbus_rsp_data),
    .rvfi_valid(),
    .rvfi_order(),
    .rvfi_insn(),
    .rvfi_trap(),
    .rvfi_halt(),
    .rvfi_intr(),
    .rvfi_mode(),
    .rvfi_ixl(),
    .rvfi_rs1_addr(),
    .rvfi_rs1_rdata(),
    .rvfi_rs2_addr(),
    .rvfi_rs2_rdata(),
    .rvfi_rd_addr(),
    .rvfi_rd_wdata(),
    .rvfi_pc_rdata(),
    .rvfi_pc_wdata(),
    .rvfi_mem_addr(),
    .rvfi_mem_rmask(),
    .rvfi_mem_wmask(),
    .rvfi_mem_rdata(),
    .rvfi_mem_wdata()
  );

  // One word per cycle, where rtl/imemory.v serves two so a compressed
  // instruction can straddle a word boundary. Same depth, same block count:
  // 1024 words either way is 8 SB_RAM40_4K.
  //
  // The read is unconditional with the range test registered alongside it,
  // which is rtl/imemory.v's shape and the one yosys turns into a block RAM's
  // read port. Written as `inst <= in_range ? rom[i] : 0` it maps to logic.
  logic [31:0] rom[0:ROM_WORDS-1];
  generate if (INIT_ROM != "") begin : l_rom_init
    initial $readmemh(INIT_ROM, rom);
  end endgenerate

  logic [29:0]        ibus_word;
  logic [ROM_BITS-1:0] ibus_index;
  logic [31:0]        rom_data;
  logic               ibus_in_range;
  assign ibus_word  = ibus_cmd_pc[31:2];
  assign ibus_index = ibus_word[ROM_BITS-1:0];

  always_ff @(posedge clk) begin
    rom_data       <= rom[ibus_index];
    ibus_in_range  <= ibus_word < 30'(ROM_WORDS);
    ibus_rsp_valid <= !reset && ibus_cmd_valid;
  end
  // Out of range reads as zero, which is an illegal instruction, so a pc that
  // runs off the end cannot wrap round onto real code. Same rule as rtl/imemory.v.
  assign ibus_rsp_inst = ibus_in_range ? rom_data : 32'b0;

  logic [3:0] size_mask, mem_wstrb;
  always_comb begin
    case (dbus_cmd_size)
      2'b00:   size_mask = 4'b0001;
      2'b01:   size_mask = 4'b0011;
      default: size_mask = 4'b1111;
    endcase
  end
  assign mem_wstrb = (dbus_cmd_valid && dbus_cmd_wr)
                   ? (size_mask << dbus_cmd_address[1:0]) : 4'b0000;

  memory #(.RAM_WORDS(RAM_WORDS)) dmem (
    .clk(clk),
    .mem_addr(dbus_cmd_address),
    .mem_wdata(dbus_cmd_data),
    .mem_wstrb(mem_wstrb),
    .mem_rdata(dbus_rsp_data)
  );

  always_ff @(posedge clk) dbus_rsp_valid <= dbus_cmd_valid && !dbus_cmd_wr;

  // The same two bits of the data bus soc/compare/bench_littlecpu.v publishes.
  logic store_bit, load_bit;
  always_ff @(posedge clk) begin
    if (reset) begin
      store_bit <= 1'b0;
      load_bit  <= 1'b0;
    end else begin
      if (|mem_wstrb)   store_bit <= dbus_cmd_data[0];
      if (dbus_rsp_valid) load_bit <= dbus_rsp_data[0];
    end
  end
  assign led0_n = !store_bit;
  assign led1_n = !load_bit;
endmodule
