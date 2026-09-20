`default_nettype none
// The QSPI memory front end: bridges nano.v's picorv32-style valid/ready bus to two
// physical QSPI devices sharing one clock and one 4-bit data bus -- flash for
// instruction fetch, PSRAM for load/store, never the other way (docs/ideas/
// nanocpu-a-verified-core-on-a-2x2-tile.md decision 4: "no code from PSRAM"). SCK runs
// at clk/2 (mode 0: idles low, launched on its falling edge, sampled on its rising
// edge), so one nibble crosses the wire every two core clocks -- a 16-bit parcel is 8
// core clocks, matching the PARCEL_CYCLES this design's own timing brief already
// assumed. mem_addr is always word-aligned for a load or store (nano.v computes it that
// way before issuing it), so a PSRAM access is a fixed 32-bit transfer with byte lanes
// resolved device-side from mem_wstrb; the controller carries no partial-word phase.
//
// riscv-formal's own genchecks treat mem_ready as a free input under a fairness
// assumption (nano/formal/checks.cfg's RISCV_FAIRNESS macro on hang/liveness): that is a
// claim that SOME memory eventually answers, not a claim about this one's actual bound.
// This module's own worst-case latency -- a flash redirect or a PSRAM access, each a
// fixed, finite cycle count -- is proved separately, on this module alone, by
// nano/formal/qspi.sby.
module nano_qspi_ctrl #(
  parameter int FLASH_DUMMY_SCK = 4,
  parameter int PSRAM_DUMMY_SCK = 4,
  // Pad-mux registered input capture (brief risk: "pad-mux latency at 64 MHz"), 0-3
  // stages deep -- TinyQV's own answer to the same unknown mux delay.
  parameter int IN_CAPTURE_STAGES = 1,
  // The invariant this controller proves about itself, not a convention: no PSRAM
  // transaction may hold psram_cs_n low longer than this many clocks.
  parameter int PSRAM_CS_LOW_LIMIT = 512
) (
  input  logic        clk,
  input  logic        reset,

  // nano.v's bus.
  input  logic        mem_valid,
  input  logic        mem_instr,
  output logic        mem_ready,
  input  logic [31:0] mem_addr,
  input  logic [31:0] mem_wdata,
  input  logic [3:0]  mem_wstrb,
  output logic [31:0] mem_rdata,

  // Physical QSPI pins, shared by every device on the bus.
  output logic       sck,
  output logic       flash_cs_n,
  output logic       psram_cs_n,
  // Never driven low: the third chip select the Pmod reserves and this design does not
  // use, carried so the mutual-exclusion invariant is stated over the same three signals
  // the brief names rather than assumed true of a signal that does not exist here.
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

  // Command bytes. Flash: Fast Read Quad I/O, entered once and held open by the mode
  // byte's continuation pattern (M[7:6] == 2'b10) on every later redirect. PSRAM: Fast
  // Read / Page Program, the pair the brief's timing model prices separately.
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

  // One SCK half-period per core clock. sio_phase == 0 is the half before SCK's rising
  // edge (where an incoming nibble is captured); sio_phase == 1 is the half before its
  // falling edge (where the next outgoing nibble is launched and the phase's nibble
  // counter moves).
  logic sio_phase;
  logic sck_run;
  assign sck = sck_run && sio_phase;

  logic [39:0] tx_shift;   // loaded MSB-first; only the top (nibbles_left*4) bits matter
  logic [31:0] rx_shift;
  logic [3:0]  nibbles_left;

  logic in_continuous_mode;

  // The two-slot prefetch queue this ticket's third invariant is about: each valid
  // slot's tag is real functional state (what the hit test below reads), not a shadow
  // copy kept only for the proof.
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

  // Exactly one of {flash, psram} may be the active device, and never the spare, by the
  // encoding of one register -- not by separate combinational guards that could drift
  // apart under a later edit.
  logic [1:0] active_dev;
  localparam logic [1:0] DEV_NONE  = 2'b00;
  localparam logic [1:0] DEV_FLASH = 2'b01;
  localparam logic [1:0] DEV_PSRAM = 2'b10;

  assign flash_cs_n = (active_dev != DEV_FLASH);
  assign psram_cs_n = (active_dev != DEV_PSRAM);
  assign spare_cs_n = 1'b1;
  assign sio_out = tx_shift[39:36];

  // A miss is any fetch request whose parcel address is not exactly the older valid
  // slot's tag: sequential-only, since v1 has no loop buffer to serve an older address
  // (the brief defers one, gated on the accounting nano/bench/run_qspi_timing.sh prints).
  logic [30:0] req_parcel_addr;
  assign req_parcel_addr = mem_addr[31:1];
  logic fetch_hit0, fetch_needs_second_parcel, fetch_hit_ready;
  assign fetch_hit0 = slot0_valid && slot0_addr == req_parcel_addr;
  assign fetch_needs_second_parcel = fetch_hit0 && slot0_data[1:0] == 2'b11;
  assign fetch_hit_ready = fetch_hit0 &&
      (!fetch_needs_second_parcel || (slot1_valid && slot1_addr == slot0_addr + 31'd1));

  // Registered, not combinational: a hit's retire (below) can move slot0/slot1 in the
  // very same cycle mem_ready is decided, so the answer is latched at decision time
  // rather than re-read off queue state that has already moved by the time mem_ready
  // reads high.

  logic queue_full;
  assign queue_full = slot0_valid && slot1_valid;

  // A hit retires from the front of the queue the same cycle mem_ready answers it,
  // regardless of what the state below is doing next.
  logic retire_one, retire_two;
  assign retire_one = state == ST_IDLE && mem_valid && mem_instr && fetch_hit_ready &&
                       !fetch_needs_second_parcel;
  assign retire_two = state == ST_IDLE && mem_valid && mem_instr && fetch_hit_ready &&
                       fetch_needs_second_parcel;

  // Invariant 2's own counter: cycles psram_cs_n has read low, without a break. Proved
  // never to reach PSRAM_CS_LOW_LIMIT in nano/formal/qspi.sby; the transactions this
  // controller ever issues are a fixed handful of cycles, far under it, so the margin is
  // wide on purpose -- the property is about a stuck state machine, not a tight budget.
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
        // Safe regardless of the flash's power-on state: a mode byte whose top two bits
        // are not 2'b10 exits continuous read on a part left in it by a warm reset, and
        // 0xFF is a reserved no-op to a part that was cold.
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
          // mem_ready is still asserted for the request just served: nano.v has not
          // reacted to it yet (mem_valid stays high for that one overlapping cycle),
          // and a hit's own retire already moved the queue, so re-reading the same
          // held address this cycle would see a spurious miss. Wait for the pulse to
          // clear before deciding anything new.
          if (mem_ready) begin
            // Nothing to decide this cycle; see above.
          end else if (mem_valid && mem_instr) begin
            if (fetch_hit_ready) begin
              mem_ready <= 1'b1;
              mem_rdata <= {slot1_data, slot0_data};
            end else if (fetch_hit0 && !queue_full) begin
              // The first parcel is already in hand and the second is still owed: no
              // redirect, just let the open stream keep delivering.
              active_dev <= DEV_FLASH;
              sck_run    <= 1'b1;
              sio_phase  <= 1'b0;
              nibbles_left <= 4'd4;
              state      <= ST_FLASH_STREAM;
            end else if (!slot0_valid && !slot1_valid && active_dev == DEV_FLASH &&
                         req_parcel_addr == stream_next_addr) begin
              // The queue emptied, but the flash's own position never moved and never
              // closed (nothing paused it here except an empty queue): resume without
              // paying a redirect.
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
            // A byte-addressable write has no wire-level equivalent on a device that
            // only takes a 32-bit-aligned command: a partial mask reads the word first
            // and merges it locally, so the transaction issued first is a read even
            // though this access is ultimately a store.
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
              // A parcel just arrived: push it, then decide whether room remains to
              // keep the stream open or this is where it pauses (CS stays asserted
              // either way -- pausing only stops the clock, per the ADR this reproduces).
              stream_next_addr <= stream_next_addr + 31'd1;
              if (!slot0_valid) begin
                // The queue was empty: this parcel fills the first slot, and the
                // second is still open, so the stream keeps running.
                slot0_valid  <= 1'b1;
                slot0_data   <= rx_shift[15:0];
                slot0_addr   <= stream_next_addr;
                nibbles_left <= 4'd4;
              end else begin
                // The queue had exactly one slot free: this parcel fills it, both
                // slots are now valid, and the clock pauses -- CS stays asserted, so
                // resuming later owes no redirect.
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
                // Merge: the read word everywhere wstrb is clear, the store's own
                // bytes everywhere it is set, then issue that merged word as a plain
                // full-word write.
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

  // None of the three invariants below assumes anything about how mem_valid/mem_addr
  // behave from one cycle to the next: each is a property of this module's own state
  // (the chip-select register, the cs-low counter, the prefetch queue's own bookkeeping)
  // that holds under an adversarial bus, not only a well-behaved one.

  // Invariant 1: no two of the three chip selects are ever low together. True by the
  // one-hot encoding of active_dev; asserted anyway so a later edit that stops deriving
  // both from one register is caught here rather than believed.
  always_comb if (clocked) begin
    assert(!(flash_cs_n == 1'b0 && psram_cs_n == 1'b0));
    assert(!(flash_cs_n == 1'b0 && spare_cs_n == 1'b0));
    assert(!(psram_cs_n == 1'b0 && spare_cs_n == 1'b0));
  end

  // Invariant 2: a PSRAM transaction never holds psram_cs_n low for PSRAM_CS_LOW_LIMIT
  // clocks or more -- proved of the counter, not assumed of the schedule that feeds it.
  always_comb if (clocked) assert(psram_cs_low_count < PSRAM_CS_LOW_LIMIT[9:0]);

  // Invariant 3: the prefetch buffer holds exactly the parcels at
  // [stream_next_addr - valid_count, stream_next_addr) -- contiguous, in order, and
  // always exactly behind the next parcel the flash has been asked for. Stated as
  // modular-subtraction equalities, not "<" comparisons: parcel_addr is 31 bits and
  // wraps, and a wraparound trace is exactly the case an ordering compare gets wrong.
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
