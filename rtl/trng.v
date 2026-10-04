`timescale 1 ns / 1 ps
`default_nettype none
// Turns a slow free-running oscillator into the 16-bit words `seed` returns; four health
// tests set the sticky `dead`, and no word is buffered until 1,024 samples pass. A beat that
// yields no corrected bit is caught by the 64-sample starvation count.
module trng (
  input  logic        clk,
  input  logic        reset,
  input  logic        raw,
  input  logic        pop,
  output logic [31:0] seed
);
  localparam logic [1:0] BIST = 2'b00, WAIT = 2'b01, ES16 = 2'b10, DEAD = 2'b11;
  localparam logic [4:0] RUN_LIMIT = 5'd31;
  // Dead when one value fills 410 of a 512-sample window: 0.5 bit per sample at 2^-20.
  localparam logic [8:0] APT_LAST = 9'd409;
  localparam logic [5:0] STARVE_LAST = 6'd63;
  localparam logic [2:0] FOLD_LAST = 3'd7;

  logic [2:0] sync;
  always_ff @(posedge clk) sync <= {sync[1:0], raw};
  logic edge_seen;
  assign edge_seen = sync[1] && !sync[2];

  logic [11:0] ticks;
  logic [12:0] ticks_next;
  assign ticks_next = {1'b0, ticks} + 13'd1;

  logic sample;
  assign sample = ticks[1] ^ ticks[0];

  logic        dead, warm;
  logic        run_last;
  logic [4:0]  run;
  logic [9:0]  samples;
  logic        apt_ref;
  logic [8:0]  apt_count;
  logic [5:0]  starve;
  logic        have, held;
  logic [2:0]  fold_count;
  logic        fold_acc;
  logic [16:0] shift;

  logic full, emit, word_bit;
  assign full = shift[16];
  assign emit = edge_seen && have && (held != sample);
  assign word_bit = fold_acc ^ held;

  logic word_done;
  assign word_done = emit && fold_count == FOLD_LAST;

  always_ff @(posedge clk) begin
    if (reset) begin
      ticks      <= 12'b0;
      dead       <= 1'b0;
      warm       <= 1'b0;
      run_last   <= 1'b0;
      run        <= 5'b0;
      samples    <= 10'b0;
      apt_ref    <= 1'b0;
      apt_count  <= 9'b0;
      starve     <= 6'b0;
      have       <= 1'b0;
      held       <= 1'b0;
      fold_count <= 3'b0;
      fold_acc   <= 1'b0;
      shift      <= 17'b1;
    end else begin
      ticks <= edge_seen ? 12'b0 : ticks_next[11:0];
      if (!edge_seen && ticks_next[12]) dead <= 1'b1;

      if (edge_seen) begin
        samples <= samples + 10'd1;
        if (&samples) warm <= 1'b1;
        if (samples[8:0] == 9'b0) begin
          apt_ref   <= sample;
          apt_count <= 9'd1;
        end else if (sample == apt_ref) begin
          apt_count <= apt_count + 9'd1;
          if (apt_count == APT_LAST) dead <= 1'b1;
        end
        starve <= emit ? 6'b0 : starve + 6'd1;
        if (!emit && starve == STARVE_LAST) dead <= 1'b1;
        have <= !have;
        held <= sample;
      end

      if (emit) begin
        fold_count <= fold_count + 3'd1;
        fold_acc   <= (fold_count == FOLD_LAST) ? 1'b0 : fold_acc ^ held;
        if (word_done && !full && warm) shift <= {shift[15:0], word_bit};
      end

      if (word_done) begin
        run_last <= word_bit;
        if (word_bit == run_last) begin
          run <= run + 5'd1;
          if (run == RUN_LIMIT) dead <= 1'b1;
        end else begin
          run <= 5'b0;
        end
      end

      if (pop && full) shift <= 17'b1;
    end
  end

  logic [1:0] opst;
  always_comb begin
    if (dead)      opst = DEAD;
    else if (full) opst = ES16;
    else if (!warm) opst = BIST;
    else           opst = WAIT;
  end

  assign seed = {opst, 14'b0, (full && !dead) ? shift[15:0] : 16'b0};
endmodule
