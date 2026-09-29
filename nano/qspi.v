`default_nettype none
// The QSPI memory front end: bridges nano.v's valid/ready bus to flash (fetch) and PSRAM (load/store) on one shared clock and 4-bit data bus, SCK at clk/2, mode 0; worst-case latency is proved by nano/formal/qspi.sby.
module nano_qspi_ctrl #(
  parameter int FLASH_DUMMY_SCK = 4,
  parameter int PSRAM_DUMMY_SCK = 4,
  // Proved, not conventional: no PSRAM transaction may hold psram_cs_n low longer than this.
  parameter int PSRAM_CS_LOW_LIMIT = 512,
  parameter int FLASH_WINDOW_BYTES = 32'h0100_0000
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
  end

  localparam int FLASH_TAG_BITS = 23;
  if (FLASH_WINDOW_BYTES > (32'd1 << (FLASH_TAG_BITS + 1))) begin : l_flash_window_fits_tag
    $fatal(1, "nano_qspi_ctrl: FLASH_WINDOW_BYTES exceeds what FLASH_TAG_BITS parcel tags can address");
  end
  localparam logic [FLASH_TAG_BITS-1:0] PARCEL_STEP  = FLASH_TAG_BITS'(1);
  localparam logic [FLASH_TAG_BITS-1:0] PARCEL_STEP2 = FLASH_TAG_BITS'(2);

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

  // A read nibble captures on SCK's falling edge, not the rising edge, for more round-trip margin.
  logic sio_phase;
  logic sck_run;
  assign sck = sck_run && sio_phase;

  logic [39:0] tx_shift;   // loaded MSB-first; only the top (nibbles_left*4) bits matter
  logic [31:0] rx_shift;
  logic [3:0]  nibbles_left;

  logic in_continuous_mode;

  logic [FLASH_TAG_BITS-1:0] stream_next_addr;    // the parcel address the flash delivers next
  logic        second_parcel_pending;             // the fetch under way needs the parcel after it
  // rx_shift[31:16] idles during a flash stream except to hold a two-parcel fetch's first parcel while the second shifts into rx_shift[15:0].
  logic        fetch_two_parcels;   // this completion delivered two parcels, not one

  logic        psram_write_phase;   // set once an RMW's read half has completed
  logic [31:0] psram_rmw_result;    // the merged word, valid only during the write-back phase
  logic        psram_partial_store;
  assign psram_partial_store = mem_wstrb != 4'b0000 && mem_wstrb != 4'b1111;
  logic        psram_write_now;
  assign psram_write_now = psram_write_phase || mem_wstrb == 4'b1111;
  logic [31:0] psram_byte_mask;
  assign psram_byte_mask = {{8{mem_wstrb[3]}}, {8{mem_wstrb[2]}},
                            {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}}};

  logic [1:0] active_dev;
  localparam logic [1:0] DEV_NONE  = 2'b00;
  localparam logic [1:0] DEV_FLASH = 2'b01;
  localparam logic [1:0] DEV_PSRAM = 2'b10;

  assign flash_cs_n = (active_dev != DEV_FLASH);
  assign psram_cs_n = (active_dev != DEV_PSRAM);
  assign spare_cs_n = 1'b1;
  assign sio_out = tx_shift[39:36];

  logic [FLASH_TAG_BITS-1:0] req_parcel_addr;
  assign req_parcel_addr = mem_addr[FLASH_TAG_BITS:1];

  // A completed parcel belongs to the request nano.v still holds stable, so it retires straight out of rx_shift; a two-parcel fetch swaps the halves into place.
  assign mem_rdata = fetch_two_parcels ? {rx_shift[15:0], rx_shift[31:16]} : rx_shift;

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
      second_parcel_pending <= 1'b0;
      fetch_two_parcels  <= 1'b0;
      mem_ready          <= 1'b0;
      nibbles_left       <= 4'd0;
      stream_next_addr   <= '0;
      tx_shift           <= '0;
      psram_write_phase  <= 1'b0;
    end else begin
      mem_ready <= 1'b0;

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
            if (active_dev == DEV_FLASH && req_parcel_addr == stream_next_addr) begin
              sck_run      <= 1'b1;
              sio_phase    <= 1'b0;
              nibbles_left <= 4'd4;
              state        <= ST_FLASH_STREAM;
            end else begin
              stream_next_addr <= req_parcel_addr;
              state            <= ST_FLASH_REOPEN;
            end
          end else if (mem_valid && !mem_instr) begin
            // A partial mask has no wire equivalent: read the word first, merge locally.
            psram_write_phase <= 1'b0;
            state <= ST_PSRAM_REOPEN;
          end
        end

        ST_FLASH_REOPEN: begin
          active_dev <= DEV_NONE;
          sck_run    <= 1'b0;
          if (in_continuous_mode) begin
            tx_shift     <= {stream_next_addr, 1'b0, FLASH_MODE_CONTINUE, 8'b0};
            nibbles_left <= 4'd8;
          end else begin
            tx_shift     <= {FLASH_CMD_FAST_READ_QIO, stream_next_addr, 1'b0,
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
          if (sio_phase) begin
            rx_shift[15:0] <= {rx_shift[11:0], sio_in};
            if (nibbles_left == 4'd1) begin
              // A parcel arrived: retire it into whichever request is still outstanding rather than caching it for a later cycle to find.
              stream_next_addr <= stream_next_addr + PARCEL_STEP;
              if (second_parcel_pending) begin
                mem_ready             <= 1'b1;
                fetch_two_parcels     <= 1'b1;
                second_parcel_pending <= 1'b0;
                sck_run               <= 1'b0;
                state                 <= ST_IDLE;
              end else if (sio_in[1:0] == 2'b11) begin
                // The other half of a four-byte instruction is owed: keep streaming.
                second_parcel_pending <= 1'b1;
                rx_shift[31:16]       <= {rx_shift[11:0], sio_in};
                nibbles_left          <= 4'd4;
              end else begin
                mem_ready         <= 1'b1;
                fetch_two_parcels <= 1'b0;
                sck_run           <= 1'b0;
                state             <= ST_IDLE;
              end
            end else begin
              nibbles_left <= nibbles_left - 4'd1;
            end
          end
        end

        ST_PSRAM_REOPEN: begin
          active_dev   <= DEV_NONE;
          sck_run      <= 1'b0;
          tx_shift     <= {psram_write_now ? PSRAM_CMD_WRITE : PSRAM_CMD_FAST_READ,
                            mem_addr[23:1], 1'b0, 8'b0};
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
              if (psram_write_now) begin
                tx_shift     <= {psram_write_phase ? psram_rmw_result : mem_wdata, 8'b0};
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
          if (sio_phase) begin
            rx_shift <= {rx_shift[27:0], sio_in};
            if (nibbles_left == 4'd1) begin
              sck_run    <= 1'b0;
              active_dev <= DEV_NONE;
              if (psram_partial_store) begin
                // Merge the read word with the store's bytes, then issue a full-word write.
                psram_rmw_result  <= (mem_wdata & psram_byte_mask) |
                                      ({rx_shift[27:0], sio_in} & ~psram_byte_mask);
                psram_write_phase <= 1'b1;
                state             <= ST_PSRAM_REOPEN;
              end else begin
                mem_ready         <= 1'b1;
                fetch_two_parcels <= 1'b0;
                state             <= ST_IDLE;
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

  // nano.v's own bus contract -- a request stays exactly as issued until mem_ready reads high -- is assumed here, since completion now reads mem_addr/mem_wdata/mem_wstrb/mem_instr live rather than from a registered copy.
  logic        mem_valid_q, mem_ready_q, mem_instr_q;
  logic [31:0] mem_addr_q, mem_wdata_q;
  logic [3:0]  mem_wstrb_q;
  always_ff @(posedge clk) begin
    mem_valid_q <= mem_valid;
    mem_ready_q <= mem_ready;
    mem_instr_q <= mem_instr;
    mem_addr_q  <= mem_addr;
    mem_wdata_q <= mem_wdata;
    mem_wstrb_q <= mem_wstrb;
  end
  always_comb if (clocked && mem_valid_q && !mem_ready_q) begin
    assume(mem_valid == 1'b1);
    assume(mem_instr == mem_instr_q);
    assume(mem_addr  == mem_addr_q);
    assume(mem_wdata == mem_wdata_q);
    assume(mem_wstrb == mem_wstrb_q);
  end

  // Invariant 1: no two of the three chip selects are ever low together.
  always_comb if (clocked) begin
    assert(!(flash_cs_n == 1'b0 && psram_cs_n == 1'b0));
    assert(!(flash_cs_n == 1'b0 && spare_cs_n == 1'b0));
    assert(!(psram_cs_n == 1'b0 && spare_cs_n == 1'b0));
  end

  // Invariant 2: psram_cs_n never reads low for PSRAM_CS_LOW_LIMIT clocks or more.
  always_comb if (clocked) assert(psram_cs_low_count < PSRAM_CS_LOW_LIMIT[9:0]);

  // Invariant 3: a completing fetch advances stream_next_addr by exactly the parcels it delivered, counted from the address nano.v still holds stable.
  always_comb if (clocked && mem_ready && mem_instr)
    assert(stream_next_addr - req_parcel_addr == (fetch_two_parcels ? PARCEL_STEP2 : PARCEL_STEP));
`endif
endmodule

`default_nettype wire
