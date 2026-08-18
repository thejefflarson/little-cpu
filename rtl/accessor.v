`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    // The instruction the executor is working on this cycle, one stage ahead of
    // `in`. Its bus transaction goes out now, so a synchronous memory has the
    // answer ready on the cycle the same instruction arrives here -- which is
    // what makes a load cost exactly what an add costs.
    input  decoder_output launch,
    // The executor consumes `launch` this cycle. While a divide runs decode
    // holds an instruction nothing has taken, and presenting a held one again
    // would be a second bus transaction for one store. Two writes are one write
    // for RAM and are not one for a device.
    input  logic launch_taken,
    // The same instruction a cycle later, carrying the result the executor
    // computed for it. A load's result is not in it: that comes off the bus.
    input  executor_output in,
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // There is no fault output here on purpose. A misaligned load or store is
    // caught in decode, which clears its `is_l*`/`is_s*` flags, so an access
    // that gets this far cannot fail. Add one and an instruction could fail
    // after decode had already let the ones behind it through, and there is no
    // way to take those back.
    //
    // The instruction memory shares one read port between fetch and data, and
    // uses this to tell a real load from an idle bus. An idle bus shows address
    // 0, which is a text address, so a memory looking only at the address would
    // give up a fetch every idle cycle and the core would never get anywhere.
    output logic mem_ren,
    // The platform says whether the address it is answering is reservable main
    // memory. `lr.w` outside it sets no reservation, so every `sc.w` there
    // fails and stores nothing -- a spurious failure, which the spec permits
    // anywhere and which the eventual-success guarantee excuses because that
    // guarantee attaches only to a region whose reservability PMA says so.
    // Without it an `sc.w` to an address no memory answers would report success
    // for a store that went nowhere, and the spec has no room for that.
    input  logic mem_reservable,
    output accessor_output out
);
  // Pulled out here rather than read inside the always_* blocks below: iverilog
  // cannot work out a precise sensitivity list for a field read taken inside
  // one, and warns. Use a continuous assign for any field read added later.
  logic launch_is_lb;
  logic launch_is_lbu;
  logic launch_is_lh;
  logic launch_is_lhu;
  logic launch_is_lw;
  logic launch_is_sb;
  logic launch_is_sh;
  logic launch_is_sw;
  logic launch_is_amoswap;
  logic launch_is_amoadd;
  logic launch_is_amoxor;
  logic launch_is_amoand;
  logic launch_is_amoor;
  logic launch_is_amomin;
  logic launch_is_amomax;
  logic launch_is_amominu;
  logic launch_is_amomaxu;
  logic launch_is_lr;
  logic launch_is_sc;
  logic launch_is_amo;
  logic [31:0] launch_mem_addr;
  logic [31:0] launch_mem_data;
  logic launch_valid;
  logic [4:0] in_rd;
  logic [31:0] in_rd_data;
  logic in_valid;

  assign launch_is_lb = launch.is_lb;
  assign launch_is_lbu = launch.is_lbu;
  assign launch_is_lh = launch.is_lh;
  assign launch_is_lhu = launch.is_lhu;
  assign launch_is_lw = launch.is_lw;
  assign launch_is_sb = launch.is_sb;
  assign launch_is_sh = launch.is_sh;
  assign launch_is_sw = launch.is_sw;
  assign launch_is_amoswap = launch.is_amoswap;
  assign launch_is_amoadd = launch.is_amoadd;
  assign launch_is_amoxor = launch.is_amoxor;
  assign launch_is_amoand = launch.is_amoand;
  assign launch_is_amoor = launch.is_amoor;
  assign launch_is_amomin = launch.is_amomin;
  assign launch_is_amomax = launch.is_amomax;
  assign launch_is_amominu = launch.is_amominu;
  assign launch_is_amomaxu = launch.is_amomaxu;
  assign launch_is_lr = launch.is_lr;
  assign launch_is_sc = launch.is_sc;
  assign launch_is_amo = launch.is_amo;
  assign launch_mem_addr = launch.mem_addr;
  assign launch_mem_data = launch.rs2;
  assign launch_valid = launch.valid;
  assign in_rd = in.rd;
  assign in_rd_data = in.rd_data;
  assign in_valid = in.valid;

  // The one gate on the whole request block. Everything below is zero when it
  // is low, which is what makes the transaction fire exactly once per memory
  // instruction however many cycles decode holds that instruction for.
  logic requesting;
  assign requesting = !reset && launch_taken && launch_valid;

  logic addr16;
  assign addr16 = launch_mem_addr[1];
  logic [1:0] addr24;
  assign addr24 = launch_mem_addr[1:0];

  // One word address, and whether anything is reserved at it. A word is all the
  // set has to be: the spec's minimum reservation set is the aligned word the
  // instruction names, and a wider one only fails more store-conditionals.
  logic [29:0] rsrv_word;
  logic        rsrv_held;
  logic        rsrv_hit;
  assign rsrv_hit = rsrv_held && rsrv_word == launch_mem_addr[31:2];

  // A store-conditional that finds no reservation puts nothing on the bus at
  // all -- not a dropped write, no transaction. That is what makes its failure
  // free and what a device on the bus would otherwise see.
  logic sc_store;
  assign sc_store = launch_is_sc && rsrv_hit;

  logic is_load, is_store;
  assign is_load = launch_is_lw || launch_is_lh || launch_is_lhu || launch_is_lb ||
    launch_is_lbu || launch_is_lr || launch_is_amo;
  assign is_store = launch_is_sw || launch_is_sh || launch_is_sb || sc_store;
  assign mem_ren = requesting && is_load;

  always_ff @(posedge clk) begin
    if (reset) begin
      rsrv_held <= 1'b0;
      rsrv_word <= 30'b0;
    end else if (requesting) begin
      if (launch_is_lr) begin
        rsrv_held <= mem_reservable;
        rsrv_word <= launch_mem_addr[31:2];
      end else if (launch_is_sc ||
                   (rsrv_hit && (launch_is_sb || launch_is_sh || launch_is_sw ||
                                 launch_is_amo))) begin
        // Cleared by any store-conditional whether it stored or not, and by a
        // write of this hart's own to the reserved word. NOT cleared on a trap
        // or an `mret`: a timer handler that never touches the lock word cannot
        // then fail the store-conditional, which is how a constrained LR/SC
        // sequence keeps the eventual-success guarantee with interrupts
        // running. The reference model was measured to do the same.
        rsrv_held <= 1'b0;
      end
    end
  end

  // What the request needs back on the cycle its answer arrives. `in` carries
  // the same instruction then, but the address and the width were spent with
  // the request a cycle earlier and are not in it.
  logic       take_is_lb, take_is_lbu, take_is_lh, take_is_lhu, take_is_lw;
  logic [1:0] take_lane;
  logic       take_load;
  // The read half of an atomic joins the load path unchanged: the address is
  // word-aligned because decode faults it otherwise, so the lane shifter and
  // the extension both pass the word through.
  logic       take_is_lr, take_amo;
  logic       take_is_sc, take_sc_failed;
  assign take_load = take_is_lb || take_is_lbu || take_is_lh || take_is_lhu || take_is_lw ||
    take_is_lr || take_amo;

  // One shift down to the addressed byte, then one extension. Written out per
  // width and per offset the same thing is twelve 32-bit selects over four
  // distinct lanes. A halfword has `addr[0]` clear because a misaligned access
  // traps in decode, so this one shift serves both widths, and a word is the
  // lane at offset zero.
  //
  // Two continuous assigns rather than a four-way case: a constant part-select
  // read inside an `always_*` draws iverilog's `sorry:` note, and the two stages
  // are the byte shifter a LUT fabric builds from the case anyway.
  logic [31:0] load_lane1, load_lane, load_value;
  logic        load_byte, load_half, load_sign;
  assign load_lane1 = take_lane[0] ? {8'b0,  mem_rdata[31:8]}   : mem_rdata;
  assign load_lane  = take_lane[1] ? {16'b0, load_lane1[31:16]} : load_lane1;
  assign load_byte = take_is_lb || take_is_lbu;
  assign load_half = take_is_lh || take_is_lhu;
  assign load_sign = (take_is_lb && load_lane[7]) || (take_is_lh && load_lane[15]);
  assign load_value[7:0]   = load_lane[7:0];
  assign load_value[15:8]  = load_byte ? {8{load_sign}} : load_lane[15:8];
  assign load_value[31:16] = (load_byte || load_half) ? {16{load_sign}} : load_lane[31:16];

  logic [31:0] word_aligned_addr;
  logic [15:0] store_halfword;
  logic [7:0]  store_byte;
  assign word_aligned_addr = {launch_mem_addr[31:2], 2'b00};
  assign store_halfword    = launch_mem_data[15:0];
  assign store_byte        = launch_mem_data[7:0];

  // ---- the read-modify-write, on the cycle after the read ------------------
  //
  // The op, the address and rs2 are all spent with the read a cycle earlier, so
  // each is held here. The memory word is not: it is on `mem_rdata` for exactly
  // this cycle, which is why the arithmetic lives here and not in
  // rtl/executor.v. That module's merged subtractor works on rs1 and rs2 at
  // issue, and an AMO's operands are the memory word and rs2.
  logic [31:0] take_amo_addr, take_amo_arg;
  logic take_amo_swap, take_amo_add, take_amo_xor, take_amo_and, take_amo_or,
        take_amo_min, take_amo_max, take_amo_minu, take_amo_maxu;

  logic [31:0] amo_mem;
  assign amo_mem = mem_rdata;

  // One 33-bit adder/subtractor for the add and all four compares. Unsigned
  // less-than is the borrow out; signed less-than is the same fact except when
  // the sign bits disagree, and then the negative operand is the smaller. The
  // subtracting operand is inverted across all 33 bits, not just the low 32:
  // narrower and the borrow comes out of the wrong end. Nothing here is a
  // signed expression, so no arm can take its signedness from a neighbour.
  logic amo_compare, amo_signed;
  assign amo_compare = take_amo_min || take_amo_max || take_amo_minu || take_amo_maxu;
  assign amo_signed  = take_amo_min || take_amo_max;

  logic [32:0] amo_wide_mem, amo_wide_arg, amo_addend, amo_sum;
  assign amo_wide_mem = {1'b0, amo_mem};
  assign amo_wide_arg = {1'b0, take_amo_arg};
  assign amo_addend   = amo_compare ? ~amo_wide_arg : amo_wide_arg;
  assign amo_sum      = amo_wide_mem + amo_addend + {32'b0, amo_compare};

  logic amo_ltu, amo_lt, amo_less, amo_keep_mem;
  assign amo_ltu      = amo_sum[32];
  assign amo_lt       = (amo_mem[31] ^ take_amo_arg[31]) ? amo_mem[31] : amo_sum[32];
  assign amo_less     = amo_signed ? amo_lt : amo_ltu;
  assign amo_keep_mem = (take_amo_min || take_amo_minu) ? amo_less : !amo_less;

  // Named out here rather than sliced inside the mux below: a constant
  // part-select read inside an `always_*` draws iverilog's `sorry:` note.
  logic [31:0] amo_add_result;
  assign amo_add_result = amo_sum[31:0];

  // Eight of the nine functions are one bit function of a memory bit and an rs2
  // bit repeated 32 times, so the op picks a four-entry truth table once and
  // every bit indexes it with its own operand pair -- one LUT deep, where five
  // 32-bit arms into a one-hot mux are two. The index is {memory bit, rs2 bit}
  // and the entries run from index 3 down to 0. The add cannot join them: its
  // bits depend on the carry into them and not on that pair alone, so it stays a
  // mux over the adder's sum.
  logic [3:0] amo_fn;
  always_comb begin
    (* parallel_case *)
    case (1'b1)
      take_amo_swap: amo_fn = 4'b1010;
      take_amo_xor:  amo_fn = 4'b0110;
      take_amo_and:  amo_fn = 4'b1000;
      take_amo_or:   amo_fn = 4'b1110;
      amo_compare:   amo_fn = amo_keep_mem ? 4'b1100 : 4'b1010;
      default: amo_fn = 4'b0000;
    endcase
  end

  logic [31:0] amo_bitwise, amo_result;
  for (genvar i = 0; i < 32; i++) begin : l_amo_fn
    assign amo_bitwise[i] = amo_fn[{amo_mem[i], take_amo_arg[i]}];
  end
  assign amo_result = take_amo_add ? amo_add_result : amo_bitwise;

  // All three outputs are driven from this one block and none of them is
  // registered: the write data has to change on the same cycle the address and
  // the strobes do, or the memory writes the last cycle's data to this cycle's
  // address.
  always_comb begin
    mem_addr = 0;
    mem_wstrb = 0;
    mem_wdata = 0;
    // The write half of an AMO outranks the request block, and never competes
    // with it: decode spends the cycle after a taken AMO, so `requesting` is
    // low here. The assertions below state both halves of that.
    if (take_amo) begin
      mem_addr = take_amo_addr;
      mem_wstrb = 4'b1111;
      mem_wdata = amo_result;
    end else if (requesting) begin
      mem_wdata = launch_mem_data;
      (* parallel_case *)
      case (1'b1)
        is_load: begin
          mem_addr = word_aligned_addr;
        end

        is_store: begin
          (* parallel_case, full_case *)
          case (1'b1)
            // A successful store-conditional is a word store of rs2. Its
            // address is word-aligned already, because decode faults an
            // unaligned one.
            launch_is_sw || sc_store: begin
              mem_addr = launch_mem_addr;
              mem_wstrb = 4'b1111;
              mem_wdata = launch_mem_data;
            end

            launch_is_sh: begin
              mem_wstrb = addr16 ? 4'b1100 : 4'b0011;
              mem_wdata = {2{store_halfword}};
            end

            launch_is_sb: begin
              mem_wstrb = 4'b0001 << addr24;
              mem_wdata = {4{store_byte}};
            end
          endcase // case (1'b1)
          mem_addr = word_aligned_addr;
        end // case: is_store

      endcase // case (1'b1)
    end // if (requesting)
  end // always_comb

 `ifdef RISCV_FORMAL
  // The request as it really went out, held for the one cycle until the
  // instruction it belongs to reaches `out`. This is the only stage that ever
  // knows the real address, masks and write data, and now it knows them a cycle
  // before it reports them.
  logic [31:0] take_rvfi_mem_addr, take_rvfi_mem_wdata;
  logic [3:0]  take_rvfi_mem_wstrb;
 `endif

  always_ff @(posedge clk) begin
    if (reset) begin
      take_is_lb <= 0; take_is_lbu <= 0; take_is_lh <= 0; take_is_lhu <= 0; take_is_lw <= 0;
      take_lane <= 0;
      take_is_lr <= 0; take_is_sc <= 0; take_sc_failed <= 0; take_amo <= 0;
      take_amo_addr <= 0; take_amo_arg <= 0;
      take_amo_swap <= 0; take_amo_add <= 0; take_amo_xor <= 0; take_amo_and <= 0;
      take_amo_or <= 0; take_amo_min <= 0; take_amo_max <= 0; take_amo_minu <= 0;
      take_amo_maxu <= 0;
     `ifdef RISCV_FORMAL
      take_rvfi_mem_addr <= 0;
      take_rvfi_mem_wdata <= 0;
      take_rvfi_mem_wstrb <= 0;
     `endif
    end else begin
      take_is_lb <= requesting && launch_is_lb;
      take_is_lbu <= requesting && launch_is_lbu;
      take_is_lh <= requesting && launch_is_lh;
      take_is_lhu <= requesting && launch_is_lhu;
      take_is_lw <= requesting && launch_is_lw;
      take_lane <= addr24;
      take_is_lr <= requesting && launch_is_lr;
      // The result a store-conditional writes to rd is decided here, on the
      // cycle the reservation is still the one it was tested against: 0 for the
      // store that went out, 1 for the one that did not.
      take_is_sc <= requesting && launch_is_sc;
      take_sc_failed <= !sc_store;
      take_amo <= requesting && launch_is_amo;
      take_amo_addr <= word_aligned_addr;
      take_amo_arg <= launch_mem_data;
      take_amo_swap <= requesting && launch_is_amoswap;
      take_amo_add <= requesting && launch_is_amoadd;
      take_amo_xor <= requesting && launch_is_amoxor;
      take_amo_and <= requesting && launch_is_amoand;
      take_amo_or <= requesting && launch_is_amoor;
      take_amo_min <= requesting && launch_is_amomin;
      take_amo_max <= requesting && launch_is_amomax;
      take_amo_minu <= requesting && launch_is_amominu;
      take_amo_maxu <= requesting && launch_is_amomaxu;
     `ifdef RISCV_FORMAL
      take_rvfi_mem_addr <= mem_addr;
      take_rvfi_mem_wdata <= mem_wdata;
      take_rvfi_mem_wstrb <= mem_wstrb;
     `endif
    end
  end

  always_ff @(posedge clk) begin
    if (reset || !in_valid) begin
      out <= 0;
    end else begin
      // A load and everything else settle in the same cycle: the only
      // difference is where the result comes from.
      out.valid <= 1'b1;
      out.rd <= in_rd;
      // A store-conditional is the one instruction here whose result is neither
      // the bus answer nor the executor's: it is whether the store happened.
      out.rd_data <= take_is_sc ? {31'b0, take_sc_failed} :
                     take_load  ? load_value : in_rd_data;
     `ifdef RISCV_FORMAL
      out.rvfi <= in.rvfi;
      // Zero for every op that touched no memory, so a plain ALU op cannot
      // inherit a stale access from whatever last used this register. A load's
      // read mask is full whatever its width -- the monitor complains about a
      // bit that should be set and is not, never the other way round.
      //
      // An AMO reports one access assembled from two cycles: the word it read
      // is on the bus now, and the address and the word it is writing are on
      // the bus now too, so those three come from the live signals rather than
      // from the registered copy of the read. Report the registered pair for an
      // AMO and the address and write data belong to the read, which reports
      // the old word as if the core had written it back unchanged.
      out.rvfi_mem_addr <= take_amo ? mem_addr : take_rvfi_mem_addr;
      out.rvfi_mem_rmask <= take_load ? 4'b1111 : 4'b0;
      out.rvfi_mem_rdata <= take_load ? mem_rdata : 32'b0;
      out.rvfi_mem_wmask <= take_amo ? mem_wstrb : take_rvfi_mem_wstrb;
      out.rvfi_mem_wdata <= take_amo ? mem_wdata :
                            |take_rvfi_mem_wstrb ? take_rvfi_mem_wdata : 32'b0;
     `endif
    end
  end

 `ifdef FORMAL
  logic clocked;
  initial clocked = 0;
  always_ff @(posedge clk) clocked <= 1;
  // Assumed: reset is asserted before the first clock edge. True of every
  // harness in the tree and asserted by none. Without it the `take_*` registers
  // hold no defined value at step 0.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);

  // Assumed: decode emits at most one memory flag per instruction, which its
  // own `$onehot` assertion under `instr_valid` discharges. Without it the
  // solver picks a word that is a load and a store at once, which no encoding
  // produces.
  always_comb assume($onehot0({launch_is_lb, launch_is_lbu, launch_is_lh, launch_is_lhu,
                               launch_is_lw, launch_is_sb, launch_is_sh, launch_is_sw,
                               launch_is_amoswap, launch_is_amoadd, launch_is_amoxor,
                               launch_is_amoand, launch_is_amoor, launch_is_amomin,
                               launch_is_amomax, launch_is_amominu, launch_is_amomaxu,
                               launch_is_lr, launch_is_sc}));

  // Assumed: the AMO bit is exactly those nine functions, which is how
  // rtl/decoder.v publishes it and what its own FORMAL block asserts. It arrives
  // as a bit rather than being rebuilt here, so nothing in this module relates
  // the two, and the solver would otherwise launch a read-modify-write that is
  // none of the nine -- or one of the nine with no read behind it.
  always_comb assume(launch_is_amo == (launch_is_amoswap || launch_is_amoadd ||
    launch_is_amoxor || launch_is_amoand || launch_is_amoor || launch_is_amomin ||
    launch_is_amomax || launch_is_amominu || launch_is_amomaxu));

  // Otherwise the solver may start the trace part-way through an AMO, for free,
  // at step 0: reset clears these on the first edge and there is no edge before
  // step 0, so the result mux would see two functions selected at once and the
  // one-hot marking below would be violated by a state the pipeline cannot
  // reach. rtl/executor.v pins its divider state for the same reason.
  initial begin
    take_amo = 0; take_is_lr = 0; take_is_sc = 0;
    take_amo_swap = 0; take_amo_add = 0; take_amo_xor = 0; take_amo_and = 0;
    take_amo_or = 0; take_amo_min = 0; take_amo_max = 0; take_amo_minu = 0;
    take_amo_maxu = 0;
  end

  // Assumed: decode spends the cycle after a taken AMO, so nothing presents a
  // transaction on the cycle the read-modify-write goes out. rtl/decoder.v's
  // `atomic_stall` is what does it, and its own FORMAL block discharges this by
  // asserting that the cycle after the wait carries a bubble.
  // Without this the solver hands this module a launch the pipeline cannot
  // produce, and the request block's two arms would both want the bus.
  always_comb if (take_amo) assume(!requesting);

  // The exact arm list of the request block's outer `(* parallel_case *)`. A
  // marking is spent against an assertion, never against belief.
  always_comb assert($onehot0({is_load, is_store}));
  // ...and of the store arm's inner one, which a store-conditional now shares
  // with `sw`.
  always_comb assert($onehot0({launch_is_sw || sc_store, launch_is_sh, launch_is_sb}));
  // ...and of the read-modify-write's truth-table select, which is the first
  // five of these. The sixth is the add's own mux below the table, and it is in
  // the list because a function selecting the table and the sum at once would
  // take the sum with the table's entries chosen for something else.
  always_comb assert($onehot0({take_amo_swap, take_amo_xor, take_amo_and,
                               take_amo_or, amo_compare, take_amo_add}));

  logic transacting;
  assign transacting = mem_ren || |mem_wstrb;

  // The 33-bit adder/subtractor against the operators it replaced, the way
  // rtl/executor.v and rtl/decoder.v state their own merged subtractors. Each
  // reference is its own self-determined statement over signed nets rather than
  // an arm of a conditional: that is what a signed comparison loses its
  // signedness to. These are what say the borrow really is unsigned less-than
  // and the sign-bit correction really is signed less-than.
  logic signed [31:0] amo_ref_x, amo_ref_y;
  assign amo_ref_x = amo_mem;
  assign amo_ref_y = take_amo_arg;
  always_comb if (!amo_compare) assert(amo_sum[31:0] == amo_mem + take_amo_arg);
  always_comb if (amo_compare) assert(amo_ltu == (amo_mem < take_amo_arg));
  always_comb if (amo_compare) assert(amo_lt == (amo_ref_x < amo_ref_y));

  // The nine functions, each against the expression the spec names for it. The
  // four min/max lines are what say the shared comparator was read the right
  // way round, which is the half a swapped `keep` term would otherwise pass.
  always_comb if (take_amo_swap) assert(amo_result == take_amo_arg);
  always_comb if (take_amo_add)  assert(amo_result == amo_mem + take_amo_arg);
  always_comb if (take_amo_xor)  assert(amo_result == (amo_mem ^ take_amo_arg));
  always_comb if (take_amo_and)  assert(amo_result == (amo_mem & take_amo_arg));
  always_comb if (take_amo_or)   assert(amo_result == (amo_mem | take_amo_arg));
  always_comb if (take_amo_min)
    assert(amo_result == ((amo_ref_x < amo_ref_y) ? amo_mem : take_amo_arg));
  always_comb if (take_amo_max)
    assert(amo_result == ((amo_ref_x < amo_ref_y) ? take_amo_arg : amo_mem));
  always_comb if (take_amo_minu)
    assert(amo_result == ((amo_mem < take_amo_arg) ? amo_mem : take_amo_arg));
  always_comb if (take_amo_maxu)
    assert(amo_result == ((amo_mem < take_amo_arg) ? take_amo_arg : amo_mem));

  // A reservation is never held for an address the platform said is not
  // reservable, so a store-conditional there cannot report the success of a
  // store no memory would have made.
  logic prev_reservable;
  always_ff @(posedge clk) prev_reservable <= mem_reservable;
  always_comb if (clocked && take_is_lr && !prev_reservable) assert(!rsrv_held);

  // A failed store-conditional leaves the bus completely alone. This is the
  // half no register value can show: the result it writes is the same 1 either
  // way a wrong design might reach it.
  always_comb if (requesting && launch_is_sc && !rsrv_hit) assert(!transacting);

  // The issued-once guard, in the two halves that make it one. Decode holds
  // `launch` unchanged for every cycle of a divide and the executor takes it on
  // exactly one of them, so a bus driven only on a taken cycle is a bus driven
  // once per memory instruction. Two writes are one write for RAM and are not
  // one for a device, which is why this is a correctness statement and not a
  // tidiness one. Delete `launch_taken` from `requesting` above and both go red.
  // An AMO is the one instruction that drives the bus on a cycle it is not
  // being launched on, and it drives it exactly once more: `take_amo` is high
  // for the single cycle after the read.
  always_comb assert(transacting == (take_amo || (requesting && (is_load || is_store))));
  always_comb if (!launch_taken && !take_amo) assert(!transacting && mem_addr == 32'b0);

  // An instruction that touches no memory leaves the bus alone, so an idle
  // cycle cannot be mistaken for a load by a memory that arbitrates on address
  // alone -- address 0 is a text address.
  always_comb if (!is_load && !is_store && !take_amo)
    assert(!transacting && mem_addr == 32'b0);
 `endif
endmodule
