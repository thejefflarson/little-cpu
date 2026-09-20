`timescale 1 ns / 1 ps
// A pin-level behavioural model of the PSRAM half of nano_qspi_ctrl's Pmod: Fast Read
// (0Bh) and Page Program (02h), word-addressed to match nano_qspi_ctrl's fixed transfer.
// Single-clock, the same fix as nano_qspi_flash_model.v; no pause exposes it here.
module nano_qspi_psram_model #(
  parameter int WORDS = 20480,
  parameter int DUMMY_SCK = 4
) (
  input  logic       clk,
  input  logic       reset,
  input  logic       sck,
  input  logic       cs_n,
  input  logic [3:0] sio_in,
  output logic [3:0] sio_out,
  output logic       sio_oe
);
  logic [31:0] mem[0:WORDS-1];

  localparam logic [7:0] CMD_FAST_READ = 8'h0B;
  localparam logic [7:0] CMD_WRITE     = 8'h02;

  localparam int PH_CMD   = 0;
  localparam int PH_ADDR  = 1;
  localparam int PH_DUMMY = 2;
  localparam int PH_READ  = 3;
  localparam int PH_WRITE = 4;
  localparam int PH_IGNORE= 5;

  int          phase;
  int          nibbles_done;
  int          dummy_left;
  logic [7:0]  cmd_byte;
  logic [23:0] addr_byte;
  logic [31:0] read_data;
  logic [31:0] write_data;

  assign sio_out = read_data[31:28];
  assign sio_oe  = !cs_n && phase == PH_READ;

  always_ff @(posedge clk) begin
    if (reset) begin
      phase <= PH_IGNORE;
    end else if (cs_n) begin
      phase        <= PH_CMD;
      nibbles_done <= 0;
    end else begin
      // See nano_qspi_flash_model.v: cs_n and sck read here already lag one clk cycle.
      if (!sck) begin
        case (phase)
          PH_CMD: begin
            cmd_byte <= {cmd_byte[3:0], sio_in};
            if (nibbles_done == 1) begin
              logic [7:0] full_cmd;
              full_cmd = {cmd_byte[3:0], sio_in};
              if (full_cmd == CMD_FAST_READ || full_cmd == CMD_WRITE) begin
                phase <= PH_ADDR;
              end else begin
                phase <= PH_IGNORE;
              end
              nibbles_done <= 0;
            end else nibbles_done <= nibbles_done + 1;
          end
          PH_ADDR: begin
            addr_byte <= {addr_byte[19:0], sio_in};
            if (nibbles_done == 5) begin
              if (cmd_byte == CMD_WRITE) begin
                phase        <= PH_WRITE;
                nibbles_done <= 0;
              end else begin
                read_data   <= mem[({addr_byte[19:0], sio_in} >> 2)];
                dummy_left  <= DUMMY_SCK + 1;  // +1: the coming fall is this rise's own.
                phase       <= PH_DUMMY;
              end
            end else nibbles_done <= nibbles_done + 1;
          end
          PH_WRITE: begin
            write_data <= {write_data[27:0], sio_in};
            if (nibbles_done == 7) begin
              mem[addr_byte >> 2] <= {write_data[27:0], sio_in};
              nibbles_done <= 0;
            end else nibbles_done <= nibbles_done + 1;
          end
          default: ;
        endcase
      end else begin
        case (phase)
          PH_DUMMY: begin
            if (dummy_left == 1) phase <= PH_READ;
            else dummy_left <= dummy_left - 1;
          end
          PH_READ: read_data <= {read_data[27:0], 4'b0};
          default: ;
        endcase
      end
    end
  end
endmodule
