`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
module accessor(
    input  logic clk,
    input  logic reset,
    // inputs
    input  executor_output in,
    // memory access
    output logic [31:0] mem_addr,
    output logic [3:0]  mem_wstrb,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    // No fault signal, and this stage has none to give: every trap is detected
    // and committed in decode (CLAUDE.md invariant 2). Misalignment used to be
    // detected HERE, which was post-decode and contradicted that invariant;
    // ADR-0011 scoped the move to M3 and it has happened, in rtl/decoder.v,
    // where the effective address is already computed. A trapping load or
    // store never reaches this stage with any `is_l*`/`is_s*` flag set, so it
    // issues no bus request at all.
    // ADR-0009-style stall broadcast (ADR-0009): the memory (test/testbench.v,
    // rtl/memory.v) registers mem_rdata one cycle after the address is
    // presented — a real, unavoidable round trip, not a choice — so a load
    // cannot be answered in the single cycle every other instruction takes
    // through this stage. High for exactly the cycle a load's *request*
    // fires; littlecpu.v folds this into the same global stall that already
    // freezes decode for a divide, and rtl/executor.v freezes (emits a
    // bubble) for that one cycle so nothing new arrives here while the
    // response is still in flight — otherwise the completing load and a
    // fresh instruction would collide over this stage's single output
    // register and the single-port memory bus.
    output logic stalled,
    // High for the cycle a load's request is on the bus. rtl/imemory.v needs
    // it to tell a real read from the idle bus, which presents address 0 --
    // inside the text range, so an unqualified address would steal a fetch
    // every idle cycle and the core would never run (ADR-0059). Same predicate
    // as `stalled` above plus the reset term the request block below applies.
    output logic mem_ren,
    // ADR-0004 hazard scoreboard (ADR-0004): a load's result is a live
    // producer from the moment its request is issued here until write-through
    // makes it visible, which is one cycle later than decoder_out/executor_out
    // alone can see (that's what `stalled` above is fixing). Decode also
    // checks this — a third stage, but only for this specific one-cycle gap,
    // not a general widening of the scoreboard.
    output logic       pending_valid,
    output logic [4:0] pending_rd,
    // outputs
    output accessor_output out
);
  // The incoming payload's fields, selected out once here. A struct field
  // read is a constant part-select, and iverilog cannot build a precise
  // sensitivity entry for one taken inside an always_* block -- it reports
  // `sorry: constant selects in always_* processes are not fully supported`
  // and conservatively makes the process sensitive to the whole struct.
  // That is safe (over-sensitivity re-evaluates redundantly; only
  // under-sensitivity can go stale) but CLAUDE.md says warnings are errors,
  // and a continuous assign has an exact sensitivity. rtl/executor.v keeps
  // the `in.` form deliberately -- see CLAUDE.md's documented exception.
  logic in_is_lb;
  logic in_is_lbu;
  logic in_is_lh;
  logic in_is_lhu;
  logic in_is_lw;
  logic in_is_sb;
  logic in_is_sh;
  logic in_is_sw;
  logic [31:0] in_mem_addr;
  logic [31:0] in_mem_data;
  logic [4:0] in_rd;
  logic [31:0] in_rd_data;
  logic in_valid;

  assign in_is_lb = in.is_lb;
  assign in_is_lbu = in.is_lbu;
  assign in_is_lh = in.is_lh;
  assign in_is_lhu = in.is_lhu;
  assign in_is_lw = in.is_lw;
  assign in_is_sb = in.is_sb;
  assign in_is_sh = in.is_sh;
  assign in_is_sw = in.is_sw;
  assign in_mem_addr = in.mem_addr;
  assign in_mem_data = in.mem_data;
  assign in_rd = in.rd;
  assign in_rd_data = in.rd_data;
  assign in_valid = in.valid;

  logic addr16;
  assign addr16 = in_mem_addr[1];
  logic [1:0] addr24;
  assign addr24 = in_mem_addr[1:0];
  logic is_load;
  assign is_load = in_is_lw || in_is_lh || in_is_lhu || in_is_lb || in_is_lbu;
  assign stalled = in_valid && is_load;
  assign mem_ren = !reset && in_valid && is_load;
  logic is_store;
  assign is_store = in_is_sw || in_is_sh || in_is_sb;

  // pending_valid/pending_rd are declared as ports above (the hazard
  // scoreboard needs them too); the rest of the pending-load state below is
  // internal-only. All of it is latched at the request cycle below and
  // consumed one cycle later once mem_rdata actually reflects this load —
  // see `stalled` above.
  logic       pending_is_lb, pending_is_lbu, pending_is_lh, pending_is_lhu, pending_is_lw;
  logic       pending_addr16;
  logic [1:0] pending_addr24;
 `ifdef RISCV_FORMAL
  // ADR-0006: a load's RVFI shadow payload (and the word-aligned
  // request address, needed for rvfi_mem_addr) has to survive the same
  // one-cycle request/response gap pending_rd does, for the same reason —
  // `in` is a guaranteed bubble on the response cycle (see pending_valid
  // below), so nothing of this instruction's is left to read off `in` then.
  rvfi_shadow  pending_rvfi;
  logic [31:0] pending_rvfi_mem_addr;
 `endif
  logic [31:0] write_request;

  // Hoisted out of the always_comb below for the same reason as decoder.v's
  // register-index fields: iverilog cannot build a precise sensitivity entry
  // for a constant part-select taken inside an always_* block, so it warns
  // and falls back to whole-signal sensitivity. Conservative and safe, but
  // noise -- and CLAUDE.md says warnings are errors. A continuous assign has
  // an exact sensitivity, so selecting out here silences it honestly.
  logic [31:0] word_aligned_addr;
  logic [15:0] store_halfword;
  logic [7:0]  store_byte;
  assign word_aligned_addr = {in_mem_addr[31:2], 2'b00};
  assign store_halfword    = in_mem_data[15:0];
  assign store_byte        = in_mem_data[7:0];
  // mem_wdata must be combinational, in lockstep with mem_addr/mem_wstrb
  // above: a registered mem_wdata (the original bug here) lags the address
  // and strobe by one cycle, so the memory model latches the *previous*
  // cycle's data at the *current* cycle's address — exactly the "address
  // recovers before data" skew traced while landing the cxxrtl runner in `9cd0c67` (see
  // test/EXPECTED_FAIL's history and ADR-0004).
  assign mem_wdata = write_request;
  // make the request. Defaults to no request every cycle — reset, a bubble,
  // and a real non-memory instruction (e.g. add) all fall out of the same
  // single default below rather than each re-stating "0, 0, 0" — and that's
  // also the direct fix for the divide-replay defect, where the accessor
  // used to keep re-issuing whatever request was last in `in` for every
  // cycle the executor sat busy in `divide`, because nothing here defaulted
  // back to zero in between.
  always_comb begin
    mem_addr = 0;
    mem_wstrb = 0;
    write_request = 0;
    if (!reset && in_valid) begin
      write_request = in_mem_data;
      // request is synchronous
      (* parallel_case *)
      case (1'b1)
        in_is_lw || in_is_lh || in_is_lhu || in_is_lb || in_is_lbu: begin
          mem_addr = word_aligned_addr;
        end

        in_is_sw || in_is_sh || in_is_sb: begin
          (* parallel_case, full_case *)
          case (1'b1)
            in_is_sw: begin
              mem_addr = in_mem_addr;
              mem_wstrb = 4'b1111;
              write_request = in_mem_data;
            end

            in_is_sh: begin
              // Offset to the right position
              mem_wstrb = addr16 ? 4'b1100 : 4'b0011;
              write_request = {2{store_halfword}};
            end

            in_is_sb: begin
              mem_wstrb = 4'b0001 << addr24;
              write_request = {4{store_byte}};
            end
          endcase // case (1'b1)
          mem_addr = word_aligned_addr;
        end // case: in_is_sw || in_is_sh || in_is_sb

        // default: not a load or a store this cycle (e.g. an add) — the
        // zeroed defaults above stand.
      endcase // case (1'b1)
    end // if (!reset && in_valid)
  end // always_comb

  always_ff @(posedge clk) begin
    // response is registered
    if (reset) begin
      out <= 0;
      pending_valid <= 1'b0;
      pending_rd <= 0;
      pending_is_lb <= 0; pending_is_lbu <= 0; pending_is_lh <= 0; pending_is_lhu <= 0; pending_is_lw <= 0;
      pending_addr16 <= 0;
      pending_addr24 <= 0;
     `ifdef RISCV_FORMAL
      pending_rvfi <= '0;
      pending_rvfi_mem_addr <= 0;
     `endif
    end else if (pending_valid) begin
      // The request fired last cycle (`stalled` was high then); mem_rdata
      // now reflects it. rtl/decoder.v froze upstream and rtl/executor.v
      // bubbled for that one cycle, so `in` here is guaranteed to be a
      // bubble right now — nothing else is competing for `out` this cycle.
      out.valid <= 1'b1;
      out.rd <= pending_rd;
     `ifdef RISCV_FORMAL
      // Full read mask regardless of load width (byte/half/word): the
      // monitor only flags a *missing* bit against what the spec expects,
      // never an extra one, so an over-approximation is safe -- ported
      // as-is from the serialized core's green run (ADR-0006 —
      // notes), not re-derived.
      out.rvfi <= pending_rvfi;
      out.rvfi_mem_addr <= pending_rvfi_mem_addr;
      out.rvfi_mem_rmask <= 4'b1111;
      out.rvfi_mem_wmask <= 4'b0;
      out.rvfi_mem_rdata <= mem_rdata;
      out.rvfi_mem_wdata <= 32'b0;
     `endif
      (* parallel_case, full_case *)
      case (1'b1)
        pending_is_lb: begin
          case (pending_addr24)
            2'b00: out.rd_data <= {{24{mem_rdata[7]}}, mem_rdata[7:0]};
            2'b01: out.rd_data <= {{24{mem_rdata[15]}}, mem_rdata[15:8]};
            2'b10: out.rd_data <= {{24{mem_rdata[23]}}, mem_rdata[23:16]};
            2'b11: out.rd_data <= {{24{mem_rdata[31]}}, mem_rdata[31:24]};
          endcase
        end

        pending_is_lbu: begin
          case (pending_addr24)
            2'b00: out.rd_data <= {24'b0, mem_rdata[7:0]};
            2'b01: out.rd_data <= {24'b0, mem_rdata[15:8]};
            2'b10: out.rd_data <= {24'b0, mem_rdata[23:16]};
            2'b11: out.rd_data <= {24'b0, mem_rdata[31:24]};
          endcase
        end

        pending_is_lh: begin
          case (pending_addr16)
            1'b0: out.rd_data <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};
            1'b1: out.rd_data <= {{16{mem_rdata[31]}}, mem_rdata[31:16]};
          endcase
        end

        pending_is_lhu: begin
          case (pending_addr16)
            1'b0: out.rd_data <= {16'b0, mem_rdata[15:0]};
            1'b1: out.rd_data <= {16'b0, mem_rdata[31:16]};
          endcase
        end

        pending_is_lw: out.rd_data <= mem_rdata;
      endcase
      pending_valid <= 1'b0;
    end else if (!in_valid) begin
      out <= 0;
    end else if (is_load) begin
      // Request just issued combinationally above; the response isn't back
      // until next cycle (see `stalled`). Emit a bubble now and remember
      // what's needed to decode mem_rdata once it arrives.
      out <= 0;
      pending_valid <= 1'b1;
      pending_rd <= in_rd;
      pending_is_lb <= in_is_lb;
      pending_is_lbu <= in_is_lbu;
      pending_is_lh <= in_is_lh;
      pending_is_lhu <= in_is_lhu;
      pending_is_lw <= in_is_lw;
      pending_addr16 <= addr16;
      pending_addr24 <= addr24;
     `ifdef RISCV_FORMAL
      // `mem_addr` here is this cycle's combinational request address (the
      // always_comb block above), already word-aligned -- the same value
      // driving the real bus this cycle. Captured now because `in` (and so
      // this address) is gone by the response cycle above.
      pending_rvfi <= in.rvfi;
      pending_rvfi_mem_addr <= mem_addr;
     `endif
    end else begin
      // Not a load: stores and every other op settle in the one cycle every
      // other pipeline stage takes.
      out.valid <= 1'b1;
      out.rd_data <= in_rd_data;
      out.rd <= in_rd;
     `ifdef RISCV_FORMAL
      out.rvfi <= in.rvfi;
      // `mem_addr`/`mem_wstrb`/`write_request` are this cycle's real bus
      // values from the always_comb block above -- exact, not an
      // approximation, so store bytes outside the byte-accurate wmask never
      // get compared against something this instruction didn't actually
      // write. Zeroed for every non-store op (including reads never reach
      // this branch -- loads take the is_load branch above), so a plain ALU
      // op never inherits a stale memory access from whatever last used
      // this register.
      out.rvfi_mem_addr <= is_store ? mem_addr : 32'b0;
      out.rvfi_mem_wmask <= is_store ? mem_wstrb : 4'b0;
      out.rvfi_mem_wdata <= is_store ? write_request : 32'b0;
      out.rvfi_mem_rmask <= 4'b0;
      out.rvfi_mem_rdata <= 32'b0;
     `endif
    end
  end

endmodule
