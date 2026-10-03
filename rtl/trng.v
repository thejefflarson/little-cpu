`timescale 1 ns / 1 ps
`default_nettype none
// Turns a slow free-running oscillator into the 16-bit words the `seed` CSR returns. The
// raw bit is the low interval bits between the oscillator's rising edges, in `clk` cycles.
module trng (
  input  logic        clk,
  input  logic        reset,
  input  logic        raw,
  // A committed read of `seed`; the word is consumed only when the status says ES16.
  input  logic        pop,
  output logic [31:0] seed
);
  localparam logic [1:0] BIST = 2'b00, WAIT = 2'b01, ES16 = 2'b10, DEAD = 2'b11;
  localparam logic [4:0] RUN_LIMIT = 5'd31;
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

  logic        dead, bist;
  logic        run_last;
  logic [4:0]  run;
  logic        have, held;
  logic [2:0]  fold_count;
  logic        fold_acc;
  logic [16:0] shift;

  logic full, emit, word_bit;
  assign full = shift[16];
  assign emit = edge_seen && have && (held != sample);
  assign word_bit = fold_acc ^ held;

  always_ff @(posedge clk) begin
    if (reset) begin
      ticks      <= 12'b0;
      dead       <= 1'b0;
      bist       <= 1'b1;
      run_last   <= 1'b0;
      run        <= 5'b0;
      have       <= 1'b0;
      held       <= 1'b0;
      fold_count <= 3'b0;
      fold_acc   <= 1'b0;
      shift      <= 17'b1;
    end else begin
      ticks <= edge_seen ? 12'b0 : ticks_next[11:0];
      if (!edge_seen && ticks_next[12]) dead <= 1'b1;

      if (edge_seen) begin
        run_last <= sample;
        if (sample == run_last) begin
          run <= run + 5'd1;
          if (run == RUN_LIMIT) dead <= 1'b1;
        end else begin
          run <= 5'b0;
        end
        have <= !have;
        held <= sample;
      end

      if (emit) begin
        fold_count <= fold_count + 3'd1;
        fold_acc   <= (fold_count == FOLD_LAST) ? 1'b0 : fold_acc ^ held;
        if (fold_count == FOLD_LAST && !full) shift <= {shift[15:0], word_bit};
      end

      if (full) bist <= 1'b0;
      if (pop && full) shift <= 17'b1;
    end
  end

  logic [1:0] opst;
  always_comb begin
    if (dead)      opst = DEAD;
    else if (full) opst = ES16;
    else if (bist) opst = BIST;
    else           opst = WAIT;
  end

  assign seed = {opst, 14'b0, (full && !dead) ? shift[15:0] : 16'b0};
endmodule
