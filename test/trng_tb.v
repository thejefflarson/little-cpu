`timescale 1 ns / 1 ps
`default_nettype none

// rtl/trng.v driven by streams standing in for the board's slow oscillator: a jittered
// square wave that must reach ES16, and three sources that must never read as anything
// but DEAD.
module trng_tb;
  localparam logic [1:0] BIST = 2'b00, WAIT = 2'b01, ES16 = 2'b10, DEAD = 2'b11;
  localparam int N = 5;

  logic clk = 0;
  always #5 clk = ~clk;
  logic reset = 1'b1;

  // 0: jittered, healthy. 1: stuck low. 2: stuck high. 3: constant period, so every
  // interval sample is the same bit. 4: healthy, then stops.
  logic [N-1:0]  raw = '0;
  logic [N-1:0]  pop = '0;
  logic [31:0]   seed [N];
  logic [15:0]   lfsr [N];
  logic [1:0]    wait_left [N];
  logic [3:0]    period_count = 4'd0;
  logic          stop_source4 = 1'b0;

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
    end
  end

  always_ff @(posedge clk) begin
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
  always_ff @(posedge clk) if (!reset) begin
    for (int i = 0; i < N; i++) begin
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

    wait_status(0, ES16, 20000);
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
    wait_status(0, ES16, 20000);
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

    wait_status(4, ES16, 20000);
    check("the fourth source starts healthy", {30'b0, seed[4][31:30]}, {30'b0, ES16});
    stop_source4 = 1'b1;
    wait_status(4, DEAD, 6000);
    check("a source that stops reads DEAD even with a word buffered",
          {30'b0, seed[4][31:30]}, {30'b0, DEAD});
    check("...and gives the word up", {16'b0, seed[4][15:0]}, 32'b0);

    repeat (100) @(posedge clk);
    #1;
    check("a stuck-low source never read as ES16", {31'b0, es16_seen[1]}, 32'b0);
    check("a stuck-high source never read as ES16", {31'b0, es16_seen[2]}, 32'b0);
    check("a constant-period source never read as ES16", {31'b0, es16_seen[3]}, 32'b0);
    check("a source that stops reads ES16 before it stops",
          {31'b0, es16_seen[4]}, 32'b1);
    check("DEAD is sticky for every dead source",
          {27'b0, dead_then_other}, 32'b0);
    check("DEAD never carries entropy", {27'b0, noise_in_dead}, 32'b0);

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
      $display("PASSED: trng (BIST, ES16, destructive read, WAIT, DEAD on stuck low/high, constant interval and a source that stops)");
      $finish;
    end
  end
endmodule
