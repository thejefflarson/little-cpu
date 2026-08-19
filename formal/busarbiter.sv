// The bus arbiter alone. `request` and `mem_lock` are free inputs and nothing
// else is instantiated, because the module reads no core state: everything it
// decides, it decides from two request bits, two lock bits and the two bits of
// grant it published last cycle. So this is a proof over the whole reachable
// state of the thing, not a bounded walk through part of it.
//
// Four properties, and the last of them is why the file exists. At most one
// hart drives the bus. A lock keeps the bus with the hart that raised it. A
// lock buys exactly one extra cycle and never a second. And a hart that keeps
// asking is granted within a stated number of cycles -- the starvation
// question, which no program can answer: a fixed-priority arbiter passes every
// torture test ever written for it, right up until the losing hart matters.
//
// The wait bound is a safety property with a constant rather than a liveness
// one, so it holds at every depth instead of eventually. The two invariants
// under it are what make it inductive, and each carries one step of the
// argument.
//
// The cover goals at the bottom are the anti-vacuity control, run by
// formal/busarbiter_cover.sby, which formal/Makefile makes a prerequisite of
// the proof. A round-robin proof over a machine where nobody is ever granted is
// green and worthless.
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
  // Every harness in this tree drives it that way. Nothing proves it, so this
  // is a convention, not a fact.
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

  // Every assertion comparing two cycles waits for two real ones. The
  // registered copies above hold whatever the solver picked for the pre-reset
  // cycle, and a grant "held" from before the design was reset is not a
  // counterexample about anything.
  logic settled;
  assign settled = clocked && !past_reset;

  // THE ENVIRONMENT ASSUMPTION. A lock lasts one cycle: no hart raises
  // `mem_lock` on two cycles running. That is what an AMO needs -- it reads on
  // the cycle it issues and writes on the cycle after -- and the wait bound
  // below is false without it, because a lock held forever holds the bus
  // forever and no arbiter can take it back.
  //
  // NOTHING IN THIS TREE DISCHARGES IT YET. The core publishes no `mem_lock` at
  // all today; the duration property that owes this assumption its proof lands
  // with that output, and until it does the wait bound is conditional on an
  // environment nobody has checked. An assumption nothing discharges is a hole,
  // not a proof, and this is the hole -- read a PASS on the wait bound as
  // "granted within the bound, provided the core really does lock for one
  // cycle".
  //
  // Bounding the lock in the arbiter instead -- a bit refusing a second
  // extension -- would delete the assumption and is deliberately not done. It
  // would cut an AMO's bus tenure short rather than report anything, turning a
  // fairness bug this proof catches into a torn atomic nothing in the machine
  // records.
  always_comb if (clocked) assume((mem_lock & past_mem_lock) == 2'b00);

  // At most one driver. The whole point of the module, and the one property
  // that needs no cycle of history to state.
  always_comb if (clocked) assert($onehot0(grant));

  // Both harts carry the same argument with one number different, so it is
  // written once. Hart 0 wins every tie it is not already holding: an idle bus
  // reads 2'b00 and 2'b00's tie goes to hart 0. Hart 1 can be made to lose that
  // one tie first, so every step of its chain arrives a cycle later and its
  // bound is a cycle longer. Both are bounded, which is the property; the
  // asymmetry is what reading the round-robin pointer off `grant` costs instead
  // of keeping a register for it.
  for (genvar h = 0; h < 2; h++) begin : l_hart
    localparam int OTHER = 1 - h;
    // Cycles of tie this hart can lose before the argument below starts.
    localparam int TIE = h;
    // THE WAIT BOUND, and it is this hart's own. One cycle to be handed the bus
    // it asked for, one more if the other hart holds a lock, and for hart 1 one
    // more still if the bus was idle when both asked.
    localparam int BOUND = 2 + TIE;

    // Cycles this hart has asked for the bus without getting it. Saturating,
    // not wrapping: a counter that wrapped would carry a starved hart back
    // inside the bound and the assertion would pass over the trace that breaks
    // it.
    logic [3:0] waited;
    always_ff @(posedge clk) begin
      if (reset || !request[h] || grant[h]) waited <= 4'd0;
      else if (waited != 4'hf) waited <= waited + 4'd1;
    end

    // A lock keeps the bus. This is the indivisibility half of an AMO: the
    // write cycle gets the same grant the read cycle had.
    always_comb
      if (settled && past_grant[h] && past_mem_lock[h]) assert(grant[h]);

    // And exactly one cycle of it. Without the lock the grant goes to a hart
    // that was asking, so a lock cannot be spun into a second cycle of the bus
    // and an AMO's tenure is exactly the two cycles it needs.
    always_comb
      if (settled && past_grant[h] && !past_mem_lock[h] && past_request[OTHER])
        assert(!grant[h]);

    // No grant to a hart that did not ask for one, except the cycle a lock
    // bought. The arbiter parks on nobody, so `grant` reads 2'b00 on an idle
    // bus and every set bit is answering a request.
    always_comb
      if (settled && grant[h])
        assert(past_request[h] || (past_grant[h] && past_mem_lock[h]));

    // The two steps that make the bound inductive. Each is a fact about a hart
    // that has been waiting, and each is one cycle of the argument:
    //
    //   waiting past its tie -- the other hart is holding a lock, because
    //                           nothing else could beat this hart's turn;
    //   waiting to the bound -- that lock has run out, so this hart is granted.
    //
    // Both are measured rather than believed: at `depth 2` the proof closes
    // with them and returns UNKNOWN without either. This task runs at sby's
    // default depth, which reaches the same verdict by unrolling instead -- so
    // what they buy is the argument written down, and a proof whose depth does
    // not have to grow with the chain. A third step, "waiting at all means the
    // other hart has the bus", was written and deleted: it is true and it is
    // free at either depth, so it was saying nothing the two below do not.
    always_comb
      if (settled && waited >= 1 + TIE && !grant[h])
        assert(grant[OTHER] && past_mem_lock[OTHER]);

    always_comb if (settled && waited >= BOUND) assert(grant[h]);

    always_comb if (clocked) assert(waited <= BOUND);

    // Only cover mode looks at these; formal/busarbiter_cover.sby is that run.
    //
    // The first says this hart is granted at all -- an arbiter that never
    // grants anybody satisfies every assertion above. The second says a lock
    // really does span two cycles WHILE THE OTHER HART IS ASKING, which is the
    // only class the indivisibility assertion exists for: a lock nobody was
    // competing for demonstrates nothing about indivisibility. The third says
    // the bound is reached, so the constant is this hart's wait rather than a
    // number comfortably above it -- widen the bound and this goes unreachable.
    //
    // Unlabelled on purpose: yosys names an assertion cell after its label
    // without the generate scope in front of it, so a label here would be the
    // same name twice and refuse to elaborate. sby reports each goal by scope
    // and line instead, which says which hart as well as which goal.
    always_comb if (clocked) begin
      cover (grant[h]);
      cover (settled && grant[h] && past_grant[h] && past_mem_lock[h] &&
             past_request[OTHER]);
      cover (waited == BOUND);
    end
  end
`endif
endmodule
