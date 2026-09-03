`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    // The executor's instruction this cycle, one stage ahead of `in`; its bus
    // transaction goes out now so a synchronous memory answers when it arrives.
    input  decoder_output launch,
    // The executor consumes `launch` this cycle. A held instruction presented
    // again would be a second transaction: one write for RAM, two for a device.
    input  logic launch_taken,
    // `launch` a cycle later; a load's result is not in it but on the bus.
    input  executor_output in,
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // Tells the instruction memory, which shares one read port between fetch
    // and data, a real load from an idle bus showing address 0, a text address.
    output logic mem_ren,
    // The platform says the address it is answering is reservable main memory;
    // `lr.w` elsewhere sets no reservation, so every `sc.w` there fails.
    input  logic mem_reservable,
    // Another bus initiator's write, and where; one initiator ties it low.
    input  logic        snoop_write,
    input  logic [31:0] snoop_addr,
    // High on the cycle an AMO's read goes out, promising an arbiter the
    // write-back cycle after it; never high on two cycles running.
    output logic        mem_lock,
    output accessor_output out
);
  // iverilog cannot derive a sensitivity list for a struct field read inside an
  // always_* block, so every field is read through a continuous assign here.
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

  logic requesting;
  assign requesting = !reset && launch_taken && launch_valid;

  logic addr16;
  assign addr16 = launch_mem_addr[1];
  logic [1:0] addr24;
  assign addr24 = launch_mem_addr[1:0];

  logic [29:0] rsrv_word;
  logic        rsrv_held;
  logic        rsrv_hit;
  assign rsrv_hit = rsrv_held && rsrv_word == launch_mem_addr[31:2];

  logic sc_store;
  assign sc_store = launch_is_sc && rsrv_hit;

  logic is_load, is_store;
  assign is_load = launch_is_lw || launch_is_lh || launch_is_lhu || launch_is_lb ||
    launch_is_lbu || launch_is_lr || launch_is_amo;
  assign is_store = launch_is_sw || launch_is_sh || launch_is_sb || sc_store;
  assign mem_ren = requesting && is_load;

  assign mem_lock = requesting && launch_is_amo;

  // Outranks a same-cycle `lr.w` below, so a reservation is never kept over a
  // foreign write to its word.
  logic snoop_clear;
  assign snoop_clear = snoop_write && rsrv_held && snoop_addr[31:2] == rsrv_word;

  always_ff @(posedge clk) begin
    if (reset) begin
      rsrv_held <= 1'b0;
      rsrv_word <= 30'b0;
    end else if (snoop_clear) begin
      rsrv_held <= 1'b0;
    end else if (requesting) begin
      if (launch_is_lr) begin
        rsrv_held <= mem_reservable;
        rsrv_word <= launch_mem_addr[31:2];
      end else if (launch_is_sc ||
                   (rsrv_hit && (launch_is_sb || launch_is_sh || launch_is_sw ||
                                 launch_is_amo))) begin
        // Not cleared on a trap or an `mret`, so a timer handler that leaves
        // the lock word alone cannot fail the store-conditional it interrupted.
        rsrv_held <= 1'b0;
      end
    end
  end

  // `take_*`: the request's own fields, held for the cycle its answer arrives.
  logic       take_is_lb, take_is_lbu, take_is_lh, take_is_lhu, take_is_lw;
  logic [1:0] take_lane;
  logic       take_load;
  logic       take_is_lr, take_amo;
  logic       take_is_sc, take_sc_failed;
  assign take_load = take_is_lb || take_is_lbu || take_is_lh || take_is_lhu || take_is_lw ||
    take_is_lr || take_amo;

  // One shift down to the addressed byte, then one extension. A misaligned
  // access traps in decode, so a halfword has `addr[0]` clear and an atomic's
  // word sits at lane zero; nothing here checks alignment.
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

  logic [31:0] take_amo_addr, take_amo_arg;
  logic take_amo_swap, take_amo_add, take_amo_xor, take_amo_and, take_amo_or,
        take_amo_min, take_amo_max, take_amo_minu, take_amo_maxu;

  logic [31:0] amo_mem;
  assign amo_mem = mem_rdata;

  // One 33-bit adder/subtractor for the add and all four compares: unsigned
  // less-than is the borrow out, and signed less-than is the same unless the
  // sign bits differ, when the negative operand is smaller. The subtracting
  // operand is inverted across all 33 bits, not just the low 32: with bit 32
  // left clear, `amo_sum[32]` is the carry out, `mem >= arg`, the borrow's
  // complement.
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

  logic [31:0] amo_add_result;
  assign amo_add_result = amo_sum[31:0];

  // A per-bit truth table indexed by {memory bit, rs2 bit}; the add depends on
  // the carry into each bit, so it stays a mux over the sum.
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

  always_comb begin
    mem_addr = 0;
    mem_wstrb = 0;
    mem_wdata = 0;
    // Decode bubbles the cycle after a taken AMO, so `take_amo` and
    // `requesting` are never high together.
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
      out.valid <= 1'b1;
      out.rd <= in_rd;
      out.rd_data <= take_is_sc ? {31'b0, take_sc_failed} :
                     take_load  ? load_value : in_rd_data;
     `ifdef RISCV_FORMAL
      out.rvfi <= in.rvfi;
      // A load's read mask is full whatever its width; the monitor accepts a
      // superset. An AMO's write is on the bus this cycle, so its address,
      // strobe and data are read live rather than from the held read.
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
  // Reset precedes the first clock edge, or the `take_*` registers are
  // undefined at step 0; every harness does it and none asserts it.
  initial assume(reset);
  always_comb if (!clocked) assume(reset);

  // Decode emits at most one memory flag per instruction; its own FORMAL block
  // asserts it.
  always_comb assume($onehot0({launch_is_lb, launch_is_lbu, launch_is_lh, launch_is_lhu,
                               launch_is_lw, launch_is_sb, launch_is_sh, launch_is_sw,
                               launch_is_amoswap, launch_is_amoadd, launch_is_amoxor,
                               launch_is_amoand, launch_is_amoor, launch_is_amomin,
                               launch_is_amomax, launch_is_amominu, launch_is_amomaxu,
                               launch_is_lr, launch_is_sc}));

  // The AMO bit arrives as a bit, not rebuilt here; decode asserts this too.
  always_comb assume(launch_is_amo == (launch_is_amoswap || launch_is_amoadd ||
    launch_is_amoxor || launch_is_amoand || launch_is_amoor || launch_is_amomin ||
    launch_is_amomax || launch_is_amominu || launch_is_amomaxu));

  // Without these the solver can start at step 0 inside an AMO, with two
  // functions selected at once, before reset's first edge clears them.
  initial begin
    take_amo = 0; take_is_lr = 0; take_is_sc = 0;
    take_amo_swap = 0; take_amo_add = 0; take_amo_xor = 0; take_amo_and = 0;
    take_amo_or = 0; take_amo_min = 0; take_amo_max = 0; take_amo_minu = 0;
    take_amo_maxu = 0;
  end

  // Discharged by rtl/decoder.v's own FORMAL block.
  always_comb if (take_amo) assume(!requesting);

  // The exact arm lists of the three `(* parallel_case *)` markings above; the
  // add joins the third because `amo_result` selects on it too.
  always_comb assert($onehot0({is_load, is_store}));
  always_comb assert($onehot0({launch_is_sw || sc_store, launch_is_sh, launch_is_sb}));
  always_comb assert($onehot0({take_amo_swap, take_amo_xor, take_amo_and,
                               take_amo_or, amo_compare, take_amo_add}));

  logic transacting;
  assign transacting = mem_ren || |mem_wstrb;

  // The merged adder/subtractor against the operators it replaces. Each signed
  // reference is a self-determined statement over signed nets, never an arm of
  // a conditional, which would evaluate it unsigned.
  logic signed [31:0] amo_ref_x, amo_ref_y;
  assign amo_ref_x = amo_mem;
  assign amo_ref_y = take_amo_arg;
  always_comb if (!amo_compare) assert(amo_sum[31:0] == amo_mem + take_amo_arg);
  always_comb if (amo_compare) assert(amo_ltu == (amo_mem < take_amo_arg));
  always_comb if (amo_compare) assert(amo_lt == (amo_ref_x < amo_ref_y));

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

  logic prev_reservable;
  always_ff @(posedge clk) prev_reservable <= mem_reservable;
  always_comb if (clocked && take_is_lr && !prev_reservable) assert(!rsrv_held);

  // `snoop_write` and `snoop_addr` are free inputs here, so this covers a real
  // second initiator rather than a port nobody drove.
  logic prev_snoop_clear;
  always_ff @(posedge clk) prev_snoop_clear <= snoop_clear;
  always_comb if (clocked && prev_snoop_clear) assert(!rsrv_held);

  logic past_mem_lock;
  always_ff @(posedge clk) past_mem_lock <= mem_lock;
  always_comb if (clocked) assert(take_amo == past_mem_lock);

  always_comb if (clocked) assert(!(mem_lock && past_mem_lock));

  always_comb if (requesting && launch_is_sc && !rsrv_hit) assert(!transacting);

  // The bus is driven once per memory instruction, on the cycle the executor
  // takes it, plus an AMO's write-back; delete `launch_taken` from
  // `requesting` and both go red.
  always_comb assert(transacting == (take_amo || (requesting && (is_load || is_store))));
  always_comb if (!launch_taken && !take_amo) assert(!transacting && mem_addr == 32'b0);

  always_comb if (!is_load && !is_store && !take_amo)
    assert(!transacting && mem_addr == 32'b0);
 `endif
endmodule
