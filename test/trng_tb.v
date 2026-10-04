`timescale 1 ns / 1 ps
`default_nettype none

// rtl/trng.v driven by streams standing in for the board's slow oscillator: jittered square
// waves that must reach ES16, and every source below that must end DEAD and never read ES16.
// The generated sources (5 to 9) pick each rising-edge interval so that the sample bit
// `ticks[1] ^ ticks[0]` takes the pattern named beside them: interval 5 samples 0 and 6
// samples 1.
module trng_tb;
  localparam logic [1:0] BIST = 2'b00, WAIT = 2'b01, ES16 = 2'b10, DEAD = 2'b11;
  localparam int N = 15;

  logic clk = 0;
  always #5 clk = ~clk;
  logic reset = 1'b1;

  // 0: jittered, healthy. 1: stuck low. 2: stuck high. 3: constant period, so every
  // interval sample is the same bit. 4: healthy, then stops. 5: samples alternate 0,1.
  // 6: samples 1,1,1,1,1,1,1,0 repeating. 7: ten 0s then ten 1s, a slow beat. 8: samples
  // 87% ones at random. 9: samples 62% ones at random, which must still reach ES16.
  // 10-12: 31, 32 and 33 folded ones between alternating bits. 13-14: 31 and 32 zeros from reset.
  logic [N-1:0]  raw = '0;
  logic [N-1:0]  pop = '0;
  logic [31:0]   seed [N];
  logic [15:0]   lfsr [N];
  logic [1:0]    wait_left [N];
  logic [3:0]    period_count = 4'd0;
  logic          stop_source4 = 1'b0;
  int            left [N];
  int            since [N];
  int            idx [N];

  for (genvar i = 0; i < N; i++) begin : g_dut
    trng dut (
      .clk(clk),
      .reset(reset),
      .raw(raw[i]),
      .pop(pop[i]),
      .seed(seed[i])
    );
  end

  initial begin
    for (int i = 0; i < N; i++) begin
      lfsr[i]      = 16'hACE1 + 16'(i);
      wait_left[i] = 2'b0;
      left[i]      = 0;
      since[i]     = 0;
      idx[i]       = 0;
    end
  end

  // Ones at least eight corrected bits apart put at most one in any fold window, so the folded
  // stream is the same whichever corrected bit the fold happens to start on.
  function automatic logic corrected_bit(input int which, input int j);
    int n;
    if (which >= 13) begin
      n = which - 13 + 31;
      corrected_bit = j >= 8 * n && (j - 8 * n) % 16 == 0;
    end else begin
      n = which - 10 + 31;
      if (j < 64) corrected_bit = j % 16 == 0;
      else if (j < 64 + 8 * n) corrected_bit = (j - 64) % 8 == 0;
      else corrected_bit = (j - 64 - 8 * (n - 1)) % 16 == 0;
    end
  endfunction

  function automatic logic sample_of(input int which, input int k, input logic [15:0] r);
    if (which >= 10)
      return k == 0 || (corrected_bit(which, (k - 1) / 2) == ((k + 1) % 2 == 0));
    case (which)
      5:       sample_of = k % 2 == 1;
      6:       sample_of = k % 8 != 7;
      7:       sample_of = k % 20 >= 10;
      8:       sample_of = r[3:0] < 4'd14;
      default: sample_of = r[2:0] < 3'd5;
    endcase
  endfunction

  always_ff @(posedge clk) begin
    for (int i = 5; i < N; i++) begin
      if (i >= 10 && reset) begin
        raw[i] <= 1'b0;
      end else if (left[i] == 0) begin
        left[i]  <= sample_of(i, idx[i], lfsr[i]) ? 5 : 4;
        since[i] <= 1;
        idx[i]   <= idx[i] + 1;
        raw[i]   <= 1'b1;
      end else begin
        left[i]  <= left[i] - 1;
        since[i] <= since[i] + 1;
        raw[i]   <= since[i] < 2;
      end
    end

    for (int i = 0; i < N; i++)
      lfsr[i] <= {lfsr[i][14:0], lfsr[i][15] ^ lfsr[i][13] ^ lfsr[i][12] ^ lfsr[i][10]};

    if (wait_left[0] == 2'b0) begin
      raw[0]       <= !raw[0];
      wait_left[0] <= lfsr[0][1:0];
    end else begin
      wait_left[0] <= wait_left[0] - 2'd1;
    end

    raw[1] <= 1'b0;
    raw[2] <= 1'b1;

    period_count <= period_count + 4'd1;
    raw[3] <= period_count[2];

    if (stop_source4) begin
      raw[4] <= 1'b0;
    end else if (wait_left[4] == 2'b0) begin
      raw[4]       <= !raw[4];
      wait_left[4] <= lfsr[4][1:0];
    end else begin
      wait_left[4] <= wait_left[4] - 2'd1;
    end
  end

  int errors = 0;
  int reds_forced = 0;

  function automatic logic agrees(input logic [31:0] got, input logic [31:0] want);
    agrees = (got === want);
  endfunction

  task automatic check(input string what, input logic [31:0] got, input logic [31:0] want);
    begin
      if (!agrees(got, want)) begin
        $display("MISMATCH %s: got=%08x expected=%08x", what, got, want);
        errors++;
      end
    end
  endtask

  // The comparison above must be able to fail, or every check in this bench is decoration.
  task automatic require_disagreement(input string what, input logic [31:0] got,
                                      input logic [31:0] wrong);
    begin
      reds_forced++;
      if (agrees(got, wrong)) begin
        $display("MISMATCH %s: the comparison AGREED with a wrong value", what);
        errors++;
      end
    end
  endtask

  logic [N-1:0] es16_seen = '0;
  logic [N-1:0] dead_seen = '0;
  logic [N-1:0] dead_then_other = '0;
  logic [N-1:0] noise_in_dead = '0;
  logic [1:0]   prev_status [N];
  logic [N-1:0] left_bist_into_es16 = '0;
  int           edges0 = 0;
  int           edges_at_warm = 0;
  logic         raw0_q = 1'b0;
  always_ff @(posedge clk) if (!reset) begin
    raw0_q <= raw[0];
    if (raw[0] && !raw0_q) edges0 <= edges0 + 1;
    if (prev_status[0] == BIST && seed[0][31:30] != BIST && edges_at_warm == 0)
      edges_at_warm <= edges0;
    for (int i = 0; i < N; i++) begin
      prev_status[i] <= seed[i][31:30];
      if (prev_status[i] == BIST && seed[i][31:30] == ES16) left_bist_into_es16[i] <= 1'b1;
      if (seed[i][31:30] == ES16) es16_seen[i] <= 1'b1;
      if (seed[i][31:30] == DEAD) dead_seen[i] <= 1'b1;
      if (dead_seen[i] && seed[i][31:30] != DEAD) dead_then_other[i] <= 1'b1;
      if (seed[i][31:30] == DEAD && seed[i][15:0] != 16'b0) noise_in_dead[i] <= 1'b1;
    end
  end

  task automatic wait_status(input int which, input logic [1:0] want, input int limit);
    int c;
    begin
      c = 0;
      while (seed[which][31:30] !== want && c < limit) begin
        @(posedge clk);
        #1;
        c++;
      end
    end
  endtask

  logic [15:0] first_word, second_word;

  initial begin
    repeat (4) @(posedge clk);
    #1;
    reset = 1'b0;

    check("after reset the first status is BIST", {30'b0, seed[0][31:30]}, {30'b0, BIST});
    check("...and carries no entropy", {16'b0, seed[0][15:0]}, 32'b0);
    pop[0] = 1'b1;
    repeat (3) @(posedge clk);
    #1;
    pop[0] = 1'b0;
    check("a pop with nothing ready changes nothing", {30'b0, seed[0][31:30]}, {30'b0, BIST});

    wait_status(0, ES16, 60000);
    check("a healthy source reaches ES16", {30'b0, seed[0][31:30]}, {30'b0, ES16});
    check("reserved and custom bits are zero", {14'b0, seed[0][29:16]}, 32'b0);
    first_word = seed[0][15:0];
    repeat (5) @(posedge clk);
    #1;
    check("a word is held until it is popped", {16'b0, seed[0][15:0]}, {16'b0, first_word});
    pop[0] = 1'b1;
    @(posedge clk);
    #1;
    pop[0] = 1'b0;
    check("a pop consumes the word: back to WAIT", {30'b0, seed[0][31:30]}, {30'b0, WAIT});
    check("...with the entropy field cleared", {16'b0, seed[0][15:0]}, 32'b0);
    wait_status(0, ES16, 60000);
    second_word = seed[0][15:0];
    check("the next word arrives", {30'b0, seed[0][31:30]}, {30'b0, ES16});
    if (second_word === first_word) begin
      $display("MISMATCH two consecutive words are identical: %04x", first_word);
      errors++;
    end

    wait_status(1, DEAD, 6000);
    check("a source stuck low reads DEAD", {30'b0, seed[1][31:30]}, {30'b0, DEAD});
    wait_status(2, DEAD, 6000);
    check("a source stuck high reads DEAD", {30'b0, seed[2][31:30]}, {30'b0, DEAD});
    wait_status(3, DEAD, 6000);
    check("a constant-period source reads DEAD", {30'b0, seed[3][31:30]}, {30'b0, DEAD});
    wait_status(5, DEAD, 200000);
    check("samples alternating 0,1 read DEAD", {30'b0, seed[5][31:30]}, {30'b0, DEAD});
    wait_status(6, DEAD, 200000);
    check("a short periodic interval pattern reads DEAD", {30'b0, seed[6][31:30]}, {30'b0, DEAD});
    wait_status(7, DEAD, 200000);
    check("a beat pattern reads DEAD", {30'b0, seed[7][31:30]}, {30'b0, DEAD});
    wait_status(8, DEAD, 200000);
    check("a source 87% biased reads DEAD", {30'b0, seed[8][31:30]}, {30'b0, DEAD});
    wait_status(9, ES16, 200000);
    check("a mildly biased source still reaches ES16", {30'b0, seed[9][31:30]}, {30'b0, ES16});

    wait_status(4, ES16, 60000);
    check("the fourth source starts healthy", {30'b0, seed[4][31:30]}, {30'b0, ES16});
    stop_source4 = 1'b1;
    wait_status(4, DEAD, 6000);
    check("a source that stops reads DEAD even with a word buffered",
          {30'b0, seed[4][31:30]}, {30'b0, DEAD});
    check("...and gives the word up", {16'b0, seed[4][15:0]}, 32'b0);

    check("31 identical folded bits do not read DEAD", {31'b0, dead_seen[10]}, 32'b0);
    check("exactly 32 identical folded bits read DEAD", {31'b0, dead_seen[11]}, 32'b1);
    check("33 identical folded bits read DEAD", {31'b0, dead_seen[12]}, 32'b1);
    check("31 folded zeros from reset do not read DEAD: reset counts no bit",
          {31'b0, dead_seen[13]}, 32'b0);
    check("32 folded zeros from reset read DEAD", {31'b0, dead_seen[14]}, 32'b1);

    repeat (100) @(posedge clk);
    #1;
    check("a stuck-low source never read as ES16", {31'b0, es16_seen[1]}, 32'b0);
    check("a stuck-high source never read as ES16", {31'b0, es16_seen[2]}, 32'b0);
    check("a constant-period source never read as ES16", {31'b0, es16_seen[3]}, 32'b0);
    check("a source that stops reads ES16 before it stops",
          {31'b0, es16_seen[4]}, 32'b1);
    check("an alternating source never read as ES16", {31'b0, es16_seen[5]}, 32'b0);
    check("a periodic source never read as ES16", {31'b0, es16_seen[6]}, 32'b0);
    check("a beat source never read as ES16", {31'b0, es16_seen[7]}, 32'b0);
    check("a biased source never read as ES16", {31'b0, es16_seen[8]}, 32'b0);
    check("DEAD is sticky for every dead source",
          32'(dead_then_other), 32'b0);
    check("DEAD never carries entropy", 32'(noise_in_dead), 32'b0);
    check("no source reads ES16 straight out of BIST: nothing is buffered in the start-up window",
          32'(left_bist_into_es16), 32'b0);
    if (edges_at_warm < 1024) begin
      $display("MISMATCH BIST ended after %0d raw edges, expected at least 1024", edges_at_warm);
      errors++;
    end

    pop[1] = 1'b1;
    repeat (3) @(posedge clk);
    #1;
    pop[1] = 1'b0;
    check("a pop does not revive a dead source", {30'b0, seed[1][31:30]}, {30'b0, DEAD});

    require_disagreement("the status comparison", {30'b0, seed[1][31:30]}, {30'b0, ES16});
    require_disagreement("the entropy comparison", {16'b0, second_word}, {16'b0, ~second_word});
    require_disagreement("the sticky comparison", {31'b0, es16_seen[0]}, 32'b0);
    check("the healthy source did read ES16 (the ES16 detector can fire)",
          {31'b0, es16_seen[0]}, 32'b1);

    if (reds_forced != 3) begin
      $display("MISMATCH the forced failures did not all run: %0d of 3", reds_forced);
      errors++;
    end

    if (errors != 0) begin
      $display("FAILED: %0d mismatches", errors);
      $fatal(1);
    end else begin
      $display("PASSED: trng (BIST over 1024 samples, ES16, destructive read, WAIT, DEAD on stuck, constant, alternating, periodic, beat and biased sources and one that stops; the repetition count trips on exactly 32)");
      $finish;
    end
  end
endmodule
