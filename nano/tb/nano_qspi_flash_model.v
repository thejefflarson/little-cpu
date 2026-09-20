`timescale 1 ns / 1 ps
// A pin-level behavioural model of the flash half of nano_qspi_ctrl's Pmod (sck/cs_n/sio),
// single-clock throughout: cxxrtl never re-evaluates a design-internal derived clock like
// sck after its first eval per commit, so `@(posedge sck)` never fires there (silently --
// iverilog runs it fine, which is what let this pass unnoticed). sck toggles every real
// clk cycle while running, so its CURRENT (pre-this-edge) value already predicts the
// transition about to become visible: `!sck` means the coming edge is a rise, `sck` means
// the coming edge is a fall, and reacting on that prediction -- not on a registered
// sck/sck_d comparison, which would read one clk cycle behind -- lands the reaction on the
// same edge the controller itself samples.
module nano_qspi_flash_model #(
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

  localparam logic [7:0] CMD_FAST_READ_QIO = 8'hEB;

  localparam int PH_CMD    = 0;
  localparam int PH_ADDR   = 1;
  localparam int PH_MODE   = 2;
  localparam int PH_DUMMY  = 3;
  localparam int PH_STREAM = 4;
  localparam int PH_IGNORE = 5;

  int          phase;
  int          nibbles_done;
  int          dummy_left;
  logic [7:0]  cmd_byte;
  logic [23:0] addr_byte;
  logic [7:0]  mode_byte;
  logic        cont_mode;
  logic [1:0]  nibble_idx;
  logic [30:0] parcel_addr;

  logic [15:0] current_parcel;
  assign current_parcel = parcel_addr[0] ? mem[parcel_addr[30:1]][31:16]
                                          : mem[parcel_addr[30:1]][15:0];
  assign sio_out = nibble_idx == 2'd0 ? current_parcel[15:12] :
                   nibble_idx == 2'd1 ? current_parcel[11:8]  :
                   nibble_idx == 2'd2 ? current_parcel[7:4]   :
                                        current_parcel[3:0];
  assign sio_oe = !cs_n && phase == PH_STREAM;

  always_ff @(posedge clk) begin
    if (reset) begin
      cont_mode <= 1'b0;
      phase     <= PH_IGNORE;
    end else if (cs_n) begin
      // Deasserted: the next assertion starts fresh, at a command byte unless latched.
      phase        <= cont_mode ? PH_ADDR : PH_CMD;
      nibbles_done <= 0;
    end else begin
      // Reading cs_n here, not a registered copy, already reads one clk cycle behind its
      // own visible change -- the same lag every register read gets -- so the cycle CS
      // first shows low still takes the branch above, and this one starts exactly when
      // real mid-session activity does, with no separate delay register needed.
      if (!sck) begin
        case (phase)
          PH_CMD: begin
            cmd_byte <= {cmd_byte[3:0], sio_in};
            if (nibbles_done == 1) begin
              phase <= ({cmd_byte[3:0], sio_in} == CMD_FAST_READ_QIO) ? PH_ADDR : PH_IGNORE;
              nibbles_done <= 0;
            end else nibbles_done <= nibbles_done + 1;
          end
          PH_ADDR: begin
            addr_byte <= {addr_byte[19:0], sio_in};
            if (nibbles_done == 5) begin
              phase        <= PH_MODE;
              nibbles_done <= 0;
            end else nibbles_done <= nibbles_done + 1;
          end
          PH_MODE: begin
            mode_byte <= {mode_byte[3:0], sio_in};
            if (nibbles_done == 1) begin
              cont_mode    <= mode_byte[3:2] == 2'b10;
              parcel_addr  <= addr_byte[23:1];
              // +1: the coming fall is this same rise's own mode byte, not a dummy one.
              dummy_left   <= DUMMY_SCK + 1;
              phase        <= PH_DUMMY;
              nibbles_done <= 0;
            end else nibbles_done <= nibbles_done + 1;
          end
          default: ;
        endcase
      end else begin
        case (phase)
          PH_DUMMY: begin
            if (dummy_left == 1) begin
              phase      <= PH_STREAM;
              nibble_idx <= 2'd0;
            end else dummy_left <= dummy_left - 1;
          end
          PH_STREAM: begin
            if (nibble_idx == 2'd3) begin
              nibble_idx  <= 2'd0;
              parcel_addr <= parcel_addr + 31'd1;
            end else nibble_idx <= nibble_idx + 2'd1;
          end
          default: ;
        endcase
      end
    end
  end
endmodule
