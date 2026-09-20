`default_nettype none
// The QSPI memory front end: bridges nano.v's picorv32-style valid/ready bus to flash
// (instruction fetch) and PSRAM (load/store) on one shared clock and 4-bit data bus, SCK
// at clk/2, mode 0. Worst-case latency is proved separately by nano/formal/qspi.sby.
module nano_qspi_ctrl #(
  parameter int FLASH_DUMMY_SCK = 4,
  parameter int PSRAM_DUMMY_SCK = 4,
  // Pad-mux registered input capture, 0-3 stages -- TinyQV's own answer to the same risk.
  parameter int IN_CAPTURE_STAGES = 1,
  // Proved, not conventional: no PSRAM transaction may hold psram_cs_n low longer than this.
  parameter int PSRAM_CS_LOW_LIMIT = 512
) (
  input  logic        clk,
  input  logic        reset,

  input  logic        mem_valid,
  input  logic        mem_instr,
  output logic        mem_ready,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,

  output logic       sck,
  output logic       flash_cs_n,
  output logic       psram_cs_n,
  output logic       spare_cs_n,
  output logic [3:0] sio_out,
  output logic       sio_oe,
  input  logic [3:0] sio_in
);
  initial begin
    if (FLASH_DUMMY_SCK < 1) $fatal(1, "FLASH_DUMMY_SCK must be at least 1");
    if (PSRAM_DUMMY_SCK < 1) $fatal(1, "PSRAM_DUMMY_SCK must be at least 1");
    if (IN_CAPTURE_STAGES < 0 || IN_CAPTURE_STAGES > 3)
      $fatal(1, "IN_CAPTURE_STAGES must be 0-3, TinyQV's own registered-capture range");
  end

  // Flash stays in Fast Read Quad I/O via the mode byte's continuation pattern (M[7:6]==2'b10).
  localparam logic [7:0] FLASH_CMD_FAST_READ_QIO = 8'hEB;
  localparam logic [7:0] FLASH_MODE_CONTINUE     = 8'hA0;
  localparam logic [7:0] FLASH_MODE_EXIT         = 8'hFF;
  localparam logic [7:0] PSRAM_CMD_FAST_READ     = 8'h0B;
  localparam logic [7:0] PSRAM_CMD_WRITE         = 8'h02;

  localparam logic [3:0] ST_RESET_JUNK  = 4'd0;
  localparam logic [3:0] ST_IDLE        = 4'd1;
  localparam logic [3:0] ST_FLASH_REOPEN= 4'd2;
  localparam logic [3:0] ST_FLASH_ADDR  = 4'd3;
  localparam logic [3:0] ST_FLASH_DUMMY = 4'd4;
  localparam logic [3:0] ST_FLASH_STREAM= 4'd5;
  localparam logic [3:0] ST_PSRAM_REOPEN= 4'd6;
  localparam logic [3:0] ST_PSRAM_ADDR  = 4'd7;
  localparam logic [3:0] ST_PSRAM_DUMMY = 4'd8;
  localparam logic [3:0] ST_PSRAM_READ  = 4'd9;
  localparam logic [3:0] ST_PSRAM_WRITE = 4'd10;

  logic [3:0] state;

  // sio_phase == 0 captures before SCK's rise; sio_phase == 1 launches and moves the counter.
  logic sio_phase;
  logic sck_run;
  assign sck = sck_run && sio_phase;

  logic [39:0] tx_shift;   // loaded MSB-first; only the top (nibbles_left*4) bits matter
  logic [31:0] rx_shift;
  logic [3:0]  nibbles_left;

  logic in_continuous_mode;

  logic        slot0_valid, slot1_valid;
  logic [15:0] slot0_data,  slot1_data;
  logic [30:0] slot0_addr,  slot1_addr;    // parcel address: mem_addr[31:1]
  logic [30:0] stream_next_addr;           // the parcel address the flash delivers next

  logic        psram_is_write;
  logic        psram_rmw_pending;
  logic [30:0] psram_addr_pending;
  logic [31:0] psram_wdata_pending;
  logic [3:0]  psram_wstrb_pending;
  logic [31:0] psram_byte_mask;
  assign psram_byte_mask = {{8{psram_wstrb_pending[3]}}, {8{psram_wstrb_pending[2]}},
                            {8{psram_wstrb_pending[1]}}, {8{psram_wstrb_pending[0]}}};

  logic [3:0] sio_captured;
  generate
    if (IN_CAPTURE_STAGES == 0) begin : g_capture_comb
      assign sio_captured = sio_in;
    end else begin : g_capture_reg
      logic [3:0] stages[0:IN_CAPTURE_STAGES-1];
      always_ff @(posedge clk) begin
        stages[0] <= sio_in;
        for (int i = 1; i < IN_CAPTURE_STAGES; i++) stages[i] <= stages[i-1];
      end
      assign sio_captured = stages[IN_CAPTURE_STAGES-1];
    end
  endgenerate

  logic [1:0] active_dev;
  localparam logic [1:0] DEV_NONE  = 2'b00;
  localparam logic [1:0] DEV_FLASH = 2'b01;
  localparam logic [1:0] DEV_PSRAM = 2'b10;

  assign flash_cs_n = (active_dev != DEV_FLASH);
  assign psram_cs_n = (active_dev != DEV_PSRAM);
  assign spare_cs_n = 1'b1;
  assign sio_out = tx_shift[39:36];

  logic [30:0] req_parcel_addr;
  assign req_parcel_addr = mem_addr[31:1];
  logic fetch_hit0, fetch_needs_second_parcel, fetch_hit_ready;
  assign fetch_hit0 = slot0_valid && slot0_addr == req_parcel_addr;
  assign fetch_needs_second_parcel = fetch_hit0 && slot0_data[1:0] == 2'b11;
  assign fetch_hit_ready = fetch_hit0 &&
      (!fetch_needs_second_parcel || (slot1_valid && slot1_addr == slot0_addr + 31'd1));

  // mem_rdata is registered: a hit's retire moves slot0/slot1 the same cycle mem_ready decides.

  logic queue_full;
  assign queue_full = slot0_valid && slot1_valid;

  logic retire_one, retire_two;
  assign retire_one = state == ST_IDLE && mem_valid && mem_instr && fetch_hit_ready &&
                       !fetch_needs_second_parcel;
  assign retire_two = state == ST_IDLE && mem_valid && mem_instr && fetch_hit_ready &&
                       fetch_needs_second_parcel;

  // Invariant 2's own counter: cycles psram_cs_n has read low, without a break.
  logic [9:0] psram_cs_low_count;
  always_ff @(posedge clk) begin
    if (reset || psram_cs_n) psram_cs_low_count <= '0;
    else if (psram_cs_low_count != '1) psram_cs_low_count <= psram_cs_low_count + 10'd1;
  end

  always_ff @(posedge clk) begin
    if (reset) begin
      state              <= ST_RESET_JUNK;
      sio_phase          <= 1'b0;
      sck_run            <= 1'b0;
      sio_oe             <= 1'b0;
      active_dev         <= DEV_NONE;
      in_continuous_mode <= 1'b0;
      slot0_valid        <= 1'b0;
      slot1_valid        <= 1'b0;
      mem_ready          <= 1'b0;
      mem_rdata          <= '0;
      nibbles_left       <= 4'd0;
      stream_next_addr   <= '0;
      tx_shift           <= '0;
      psram_rmw_pending  <= 1'b0;
    end else begin
      mem_ready <= 1'b0;

      if (retire_one) begin
        slot0_valid <= slot1_valid;
        slot0_data  <= slot1_data;
        slot0_addr  <= slot1_addr;
        slot1_valid <= 1'b0;
      end else if (retire_two) begin
        slot0_valid <= 1'b0;
        slot1_valid <= 1'b0;
      end

      if (sck_run) sio_phase <= !sio_phase;

      case (state)
        // A mode byte whose top bits aren't 2'b10 exits continuous read; 0xFF is a no-op cold.
        ST_RESET_JUNK: begin
          if (!sck_run) begin
            active_dev   <= DEV_FLASH;
            sck_run      <= 1'b1;
            sio_oe       <= 1'b1;
            sio_phase    <= 1'b0;
            tx_shift     <= {FLASH_MODE_EXIT, FLASH_MODE_EXIT, FLASH_MODE_EXIT, FLASH_MODE_EXIT,
                              8'b0};
            nibbles_left <= 4'd8;
          end else if (sio_phase) begin
            tx_shift <= {tx_shift[35:0], 4'b0};
            if (nibbles_left == 4'd1) begin
              sck_run    <= 1'b0;
              sio_oe     <= 1'b0;
              active_dev <= DEV_NONE;
              state      <= ST_IDLE;
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_IDLE: begin
          sck_run <= 1'b0;
          sio_oe  <= 1'b0;
          // mem_valid outlives mem_ready by one cycle; wait for the pulse to clear first.
          if (mem_ready) begin
          end else if (mem_valid && mem_instr) begin
            if (fetch_hit_ready) begin
              mem_ready <= 1'b1;
              mem_rdata <= {slot1_data, slot0_data};
            end else if (fetch_hit0 && !queue_full) begin
              // The first parcel is in hand and the second is owed: let the stream continue.
              active_dev <= DEV_FLASH;
              sck_run    <= 1'b1;
              sio_phase  <= 1'b0;
              nibbles_left <= 4'd4;
              state      <= ST_FLASH_STREAM;
            end else if (!slot0_valid && !slot1_valid && active_dev == DEV_FLASH &&
                         req_parcel_addr == stream_next_addr) begin
              sck_run      <= 1'b1;
              sio_phase    <= 1'b0;
              nibbles_left <= 4'd4;
              state        <= ST_FLASH_STREAM;
            end else begin
              slot0_valid      <= 1'b0;
              slot1_valid      <= 1'b0;
              stream_next_addr <= req_parcel_addr;
              state            <= ST_FLASH_REOPEN;
            end
          end else if (mem_valid && !mem_instr) begin
            // A partial mask has no wire equivalent: read the word first, merge locally.
            psram_is_write      <= mem_wstrb == 4'b1111;
            psram_rmw_pending   <= mem_wstrb != 4'b0000 && mem_wstrb != 4'b1111;
            psram_addr_pending  <= mem_addr[31:1];
            psram_wdata_pending <= mem_wdata;
            psram_wstrb_pending <= mem_wstrb;
            slot0_valid <= 1'b0;
            slot1_valid <= 1'b0;
            state <= ST_PSRAM_REOPEN;
          end
        end

        ST_FLASH_REOPEN: begin
          active_dev <= DEV_NONE;
          sck_run    <= 1'b0;
          if (in_continuous_mode) begin
            tx_shift     <= {stream_next_addr[22:0], 1'b0, FLASH_MODE_CONTINUE, 8'b0};
            nibbles_left <= 4'd8;
          end else begin
            tx_shift     <= {FLASH_CMD_FAST_READ_QIO, stream_next_addr[22:0], 1'b0,
                              FLASH_MODE_CONTINUE};
            nibbles_left <= 4'd10;
            in_continuous_mode <= 1'b1;
          end
          state <= ST_FLASH_ADDR;
        end

        ST_FLASH_ADDR: begin
          if (!sck_run) begin
            active_dev <= DEV_FLASH;
            sck_run    <= 1'b1;
            sio_oe     <= 1'b1;
            sio_phase  <= 1'b0;
          end else if (sio_phase) begin
            tx_shift <= {tx_shift[35:0], 4'b0};
            if (nibbles_left == 4'd1) begin
              sio_oe       <= 1'b0;
              nibbles_left <= FLASH_DUMMY_SCK[3:0];
              state        <= ST_FLASH_DUMMY;
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_FLASH_DUMMY: begin
          if (sio_phase) begin
            if (nibbles_left == 4'd1) begin
              nibbles_left <= 4'd4;
              state        <= ST_FLASH_STREAM;
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_FLASH_STREAM: begin
          if (!sio_phase) begin
            rx_shift[15:0] <= {rx_shift[11:0], sio_captured};
          end else begin
            if (nibbles_left == 4'd1) begin
              // A parcel arrived: push it, then keep streaming or pause -- CS stays asserted.
              stream_next_addr <= stream_next_addr + 31'd1;
              if (!slot0_valid) begin
                slot0_valid  <= 1'b1;
                slot0_data   <= rx_shift[15:0];
                slot0_addr   <= stream_next_addr;
                nibbles_left <= 4'd4;
              end else begin
                slot1_valid <= 1'b1;
                slot1_data  <= rx_shift[15:0];
                slot1_addr  <= stream_next_addr;
                sck_run     <= 1'b0;
                state       <= ST_IDLE;
              end
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_PSRAM_REOPEN: begin
          active_dev   <= DEV_NONE;
          sck_run      <= 1'b0;
          tx_shift     <= {psram_is_write ? PSRAM_CMD_WRITE : PSRAM_CMD_FAST_READ,
                            psram_addr_pending[22:0], 1'b0, 8'b0};
          nibbles_left <= 4'd8;
          state        <= ST_PSRAM_ADDR;
        end

        ST_PSRAM_ADDR: begin
          if (!sck_run) begin
            active_dev <= DEV_PSRAM;
            sck_run    <= 1'b1;
            sio_oe     <= 1'b1;
            sio_phase  <= 1'b0;
          end else if (sio_phase) begin
            tx_shift <= {tx_shift[35:0], 4'b0};
            if (nibbles_left == 4'd1) begin
              if (psram_is_write) begin
                tx_shift     <= {psram_wdata_pending, 8'b0};
                nibbles_left <= 4'd8;
                state        <= ST_PSRAM_WRITE;
              end else begin
                sio_oe       <= 1'b0;
                nibbles_left <= PSRAM_DUMMY_SCK[3:0];
                state        <= ST_PSRAM_DUMMY;
              end
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_PSRAM_DUMMY: begin
          if (sio_phase) begin
            if (nibbles_left == 4'd1) begin
              nibbles_left <= 4'd8;
              state        <= ST_PSRAM_READ;
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_PSRAM_READ: begin
          if (!sio_phase) begin
            rx_shift <= {rx_shift[27:0], sio_captured};
          end else begin
            if (nibbles_left == 4'd1) begin
              sck_run    <= 1'b0;
              active_dev <= DEV_NONE;
              if (psram_rmw_pending) begin
                // Merge the read word with the store's bytes, then issue a full-word write.
                psram_wdata_pending <= (psram_wdata_pending & psram_byte_mask) |
                                        (rx_shift & ~psram_byte_mask);
                psram_rmw_pending <= 1'b0;
                psram_is_write    <= 1'b1;
                state             <= ST_PSRAM_REOPEN;
              end else begin
                mem_ready <= 1'b1;
                mem_rdata <= rx_shift;
                state     <= ST_IDLE;
              end
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_PSRAM_WRITE: begin
          if (sio_phase) begin
            tx_shift <= {tx_shift[35:0], 4'b0};
            if (nibbles_left == 4'd1) begin
              sio_oe     <= 1'b0;
              sck_run    <= 1'b0;
              active_dev <= DEV_NONE;
              mem_ready  <= 1'b1;
              state      <= ST_IDLE;
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

`ifdef FORMAL
  logic clocked;
  initial clocked = 1'b0;
  always_ff @(posedge clk) clocked <= 1'b1;
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  // Invariant 1: no two of the three chip selects are ever low together.
  always_comb if (clocked) begin
    assert(!(flash_cs_n == 1'b0 && psram_cs_n == 1'b0));
    assert(!(flash_cs_n == 1'b0 && spare_cs_n == 1'b0));
    assert(!(psram_cs_n == 1'b0 && spare_cs_n == 1'b0));
  end

  // Invariant 2: psram_cs_n never reads low for PSRAM_CS_LOW_LIMIT clocks or more.
  always_comb if (clocked) assert(psram_cs_low_count < PSRAM_CS_LOW_LIMIT[9:0]);

  // Invariant 3: the prefetch buffer holds exactly the parcels just behind stream_next_addr,
  // stated as modular-subtraction equalities since parcel_addr is 31 bits and wraps.
  always_comb if (clocked && !slot0_valid) assert(!slot1_valid);
  always_comb if (clocked && slot0_valid && !slot1_valid)
    assert(stream_next_addr - slot0_addr == 31'd1);
  always_comb if (clocked && slot0_valid && slot1_valid) begin
    assert(slot1_addr == slot0_addr + 31'd1);
    assert(stream_next_addr - slot1_addr == 31'd1);
  end
`endif
endmodule

`default_nettype wire
