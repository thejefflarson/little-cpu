// The bus arbiter alone. `request` and `mem_lock` are free inputs and nothing else is
// instantiated, because the module reads no core state.
`default_nettype none

module busarbiter_check (
    input logic       clk,
    input logic       reset,
    input logic [1:0] request,
    input logic [1:0] mem_lock
);
  logic [1:0] grant;

  busarbiter arbiter (
      .clk(clk),
      .reset(reset),
      .request(request),
      .mem_lock(mem_lock),
      .grant(grant)
  );

`ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;

  // Assumed: reset is high before the first clock edge and low forever after.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);
  always_comb if (clocked) assume(!reset);

  logic [1:0] past_grant, past_request, past_mem_lock;
  logic past_reset;
  always_ff @(posedge clk) begin
    past_grant    <= grant;
    past_request  <= request;
    past_mem_lock <= mem_lock;
    past_reset    <= reset;
  end

  logic settled;
  assign settled = clocked && !past_reset;

  always_comb if (clocked) assume((mem_lock & past_mem_lock) == 2'b00);

  always_comb if (clocked) assert($onehot0(grant));

  for (genvar h = 0; h < 2; h++) begin : l_hart
    localparam int OTHER = 1 - h;
    localparam int TIE = h;
    localparam int BOUND = 2 + TIE;

    logic [3:0] waited;
    always_ff @(posedge clk) begin
      if (reset || !request[h] || grant[h]) waited <= 4'd0;
      else if (waited != 4'hf) waited <= waited + 4'd1;
    end

    always_comb
      if (settled && past_grant[h] && past_mem_lock[h]) assert(grant[h]);

    always_comb
      if (settled && past_grant[h] && !past_mem_lock[h] && past_request[OTHER])
        assert(!grant[h]);

    always_comb
      if (settled && grant[h])
        assert(past_request[h] || (past_grant[h] && past_mem_lock[h]));

    always_comb
      if (settled && waited >= 1 + TIE && !grant[h])
        assert(grant[OTHER] && past_mem_lock[OTHER]);

    always_comb if (settled && waited >= BOUND) assert(grant[h]);

    always_comb if (clocked) assert(waited <= BOUND);

    always_comb if (clocked) begin
      cover (grant[h]);
      cover (settled && grant[h] && past_grant[h] && past_mem_lock[h] &&
             past_request[OTHER]);
      cover (waited == BOUND);
    end
  end
`endif
endmodule
