`timescale 1 ns / 1 ps
`default_nettype none
`include "structs.v"
// A sibling of the decoder, not a pipeline stage. Every access is read and
// committed in decode on the same edge the accessing instruction issues, so no
// CSR state exists downstream: nothing to forward and nothing to replay. CSR
// instructions serialize, and rtl/decoder.v is what holds them; this module has
// no idea a stall exists.
//
// The set below is a floor, not a closed list. Every register the privileged
// spec lists unconditionally for RV32 machine mode is here, because trapping on
// a mandatory register is non-conformant however minimal the set is. Read it as
// the named addresses plus the performance monitor's three ranges -- 87 of the
// implemented addresses are recognised by a range compare and appear in no
// list.
//
// This module never decides that an access is illegal. `implemented` feeds the
// decoder's `instr_valid` term and the read-only test is on the address, so both
// illegal-CSR rules are decided in rtl/decoder.v with every other trap cause.
//
// What mtval reports, per cause. Firmware cannot derive this -- the privileged
// spec leaves every entry to the platform -- so it is stated rather than
// implied, the way the machine timer's period is:
//
//   1 instruction access fault   the address the fetch was refused at
//   2 illegal instruction        the faulting instruction, a compressed one
//                                zero-extended into the upper half
//   4 load misaligned            the effective address
//   5 load access fault          the effective address
//   6 store misaligned           the effective address
//   7 store/AMO access fault     the effective address
//   3 breakpoint                 zero
//   11 ecall from M              zero
//   an interrupt                 zero
//
// The two exceptions that report zero do so because `mepc` already holds the
// only address either could name, and the spec asks for a nonzero mtval on
// neither: its rule is that IF a value is written it must be the faulting
// address, not that one must be. rtl/decoder.v builds the value; this file only
// latches it.
//
// A software write is honoured in full, with no legal-value mask. Every 32-bit
// pattern is a value some trap could have left here -- it holds addresses and
// instruction words -- so a mask would have nothing to reject, and a handler
// that nests traps has to be able to save and restore it.
module csrs #(
  // The value mhartid reads: this hart's unique id, chosen by the integrator
  // rather than by the core. The spec requires the ids to be unique and one of
  // them to be zero, so a single-hart machine takes the default and never says
  // anything, and a second core is the parameter's only caller.
  parameter logic [31:0] HART_ID = 32'd0
) (
  input  logic clk,
  input  logic reset,

  // `addr` is instr[31:20], presented every cycle; `rdata` and `implemented`
  // answer it the same cycle, because the decoder needs the read value in time
  // to register it into decoder_out at this edge. Zicsr's suppression rules are
  // resolved in the decoder, where the operand fields live, so `wen` here means
  // "commit this".
  input  logic [11:0] addr,
  input  logic        ren,
  input  logic        wen,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,
  output logic        implemented,

  // minstret counts non-trapping issues, so this is high for exactly the cycles
  // the decoder issues one.
  input  logic        instret,

  // `trap_entry` is high for exactly the cycle the decoder commits a trap and
  // `mret_entry` for the cycle it commits an `mret`. They cannot coincide with
  // each other (a trapping instruction does not also execute) nor with `wen`
  // (the decoder gates `csr_wen` on a non-trapping commit, and `mret` is not a
  // CSR access), so the three write paths below need no priority between them.
  input  logic        trap_entry,
  input  logic [31:0] trap_cause,
  input  logic [31:0] trap_epc,
  input  logic [31:0] trap_tval,
  input  logic        mret_entry,
  // The platform's machine-timer line, high while its mtime has reached
  // mtimecmp. It must arrive REGISTERED -- rtl/timer.v registers the comparison
  // -- because `interrupt_pending` below is one AND away from the decoder's
  // next_pc chain, and an unregistered comparator would put a 64-bit carry
  // chain on the fetch loop.
  input  logic        irq_timer,
  // Read back by rtl/decoder.v, combinationally, to redirect the PC.
  output logic [31:0] mtvec_value,
  output logic [31:0] mepc_value,
  // The whole interrupt decision: a source is asserting, software enabled that
  // source, and software has interrupts on. Every term is a flip-flop, so this
  // costs the decoder one gate. Trap entry clears MIE on the same edge it
  // redirects, which is what stops the next cycle taking the interrupt again.
  output logic        interrupt_pending
 `ifdef RISCV_FORMAL
  ,
  // Shadow payloads for exactly the CSRs formal/checks.cfg's `[csrs]` list
  // names. The decoder latches them into the issuing instruction's shadow, so
  // they ride to writeback on the ordinary valid-bit protocol.
  output rvfi_csr64 rvfi_mcycle,
  output rvfi_csr64 rvfi_minstret,
  output rvfi_csr32 rvfi_mscratch
  `ifdef RISCV_FORMAL_CSR_MCAUSE
  // mcause is NOT on that list -- rvfi_csrw_check.sv has no WARL model and
  // would fail a correct core -- but checks/rvfi_fault_check.sv reads this
  // report directly, and at the pinned SHA it does not even parse without the
  // macro. So the shadow exists exactly where that check does.
  , output rvfi_csr32 rvfi_mcause
  `endif
 `endif
);
  localparam logic [11:0] MSTATUS   = 12'h300;
  localparam logic [11:0] MISA      = 12'h301;
  // Both read-only zero, and zero is legal for each rather than a stub:
  // mstatush's only fields are SBE and MBE, where 0 means the little-endian
  // M-mode accesses this core does, and mconfigptr = 0 is the spec's own
  // encoding for "no configuration data structure exists".
  localparam logic [11:0] MSTATUSH  = 12'h310;
  localparam logic [11:0] MCONFIGPTR = 12'hF15;
  localparam logic [11:0] MIE       = 12'h304;
  localparam logic [11:0] MTVEC     = 12'h305;
  localparam logic [11:0] MSCRATCH  = 12'h340;
  localparam logic [11:0] MEPC      = 12'h341;
  localparam logic [11:0] MCAUSE    = 12'h342;
  localparam logic [11:0] MTVAL     = 12'h343;
  localparam logic [11:0] MIP       = 12'h344;
  localparam logic [11:0] MCYCLE    = 12'hB00;
  localparam logic [11:0] MINSTRET  = 12'hB02;
  localparam logic [11:0] MCYCLEH   = 12'hB80;
  localparam logic [11:0] MINSTRETH = 12'hB82;
  localparam logic [11:0] MVENDORID = 12'hF11;
  localparam logic [11:0] MARCHID   = 12'hF12;
  localparam logic [11:0] MIMPID    = 12'hF13;
  localparam logic [11:0] MHARTID   = 12'hF14;

  // RV32 I M A C, MXL = 1. Bit 0 is A: the only run-time claim that this core
  // implements the atomics, which a -march string does not make.
  localparam logic [31:0] MISA_VALUE = 32'h4000_1105;

  logic [63:0] mcycle, minstret;
  logic [31:0] mscratch, mtvec, mepc, mcause, mtval;
  // mstatus is three fields rather than a register: MPP is hardwired to M-mode
  // (WARL, and this core has no other mode) and every other bit is 0.
  logic        mstatus_mie, mstatus_mpie;
  // mie is one field for the same reason. MSIE and MEIE stay read-only zero:
  // there is no software-interrupt register and no external controller on this
  // platform, and WARL lets an absent source read zero. Adding either is adding
  // its hardware first.
  logic        mie_mtie;

  // Selected out here rather than inside the always_* blocks below: a constant
  // part-select taken inside one defeats iverilog's sensitivity analysis and
  // draws a `sorry:` note. Use a named continuous assign for any added later.
  logic [31:0] mcycle_lo, mcycle_hi, minstret_lo, minstret_hi;
  assign mcycle_lo   = mcycle[31:0];
  assign mcycle_hi   = mcycle[63:32];
  assign minstret_lo = minstret[31:0];
  assign minstret_hi = minstret[63:32];

  logic [31:0] mstatus_value;
  // MPP = 2'b11 at [12:11]; MPIE at [7]; MIE at [3]; everything else 0.
  assign mstatus_value = {19'b0, 2'b11, 3'b0, mstatus_mpie, 3'b0, mstatus_mie, 3'b0};

  // MTIE and MTIP both sit at bit 7. mip is read-only here: MTIP is the
  // platform's line and the only way software lowers it is by moving mtimecmp,
  // which is a store to rtl/timer.v rather than a CSR write.
  logic [31:0] mie_value, mip_value;
  assign mie_value = {24'b0, mie_mtie, 7'b0};
  assign mip_value = {24'b0, irq_timer, 7'b0};

  assign interrupt_pending = irq_timer && mie_mtie && mstatus_mie;

  // The 87 hardware performance monitor addresses: mhpmcounter3-31
  // (0xB03-0xB1F), mhpmcounter3h-31h (0xB83-0xB9F) and mhpmevent3-31
  // (0x323-0x33F). The privileged spec asks for all 29 counters and their
  // event selectors and expressly permits both to be read-only zero, which is
  // what these are -- no counter, no event logic, and the read mux's default
  // arm already answers zero, so only `implemented` learns about them.
  //
  // Each of the three is an aligned 32-address window carrying the counter
  // number in its low five bits, and the numbers start at 3 because 0-2 are the
  // machine counters and their reserved neighbours. One window per spec range,
  // priced against the cheaper spelling that folds the two counter windows into
  // one compare on the bit that separates them: that one is 39 placed cells
  // smaller -- inside `make fit`'s churn band -- and was declined because three
  // named windows read the way the spec's three ranges do, so do not re-derive
  // it as a saving.
  localparam logic [6:0] MHPMCOUNTER_WINDOW  = 7'h58; // 0xB00-0xB1F
  localparam logic [6:0] MHPMCOUNTERH_WINDOW = 7'h5C; // 0xB80-0xB9F
  localparam logic [6:0] MHPMEVENT_WINDOW    = 7'h19; // 0x320-0x33F
  logic hpm_number, hpm_counter, hpm_event, hpm_zero;
  assign hpm_number  = addr[4:0] > 5'd2;
  assign hpm_counter = addr[11:5] == MHPMCOUNTER_WINDOW ||
                       addr[11:5] == MHPMCOUNTERH_WINDOW;
  assign hpm_event   = addr[11:5] == MHPMEVENT_WINDOW;
  assign hpm_zero    = hpm_number && (hpm_counter || hpm_event);

  // As of the start of the issuing cycle, which is the right phase: decode
  // issues at most one instruction per cycle, so a trapping instruction and a
  // `csrw mtvec` are never the same edge, and the trap must vector through the
  // mtvec that was in place when it issued.
  assign mtvec_value = mtvec;
  assign mepc_value  = mepc;

  always_comb begin
    implemented = 1'b1;
    (* parallel_case *)
    case (addr)
      MSTATUS:   rdata = mstatus_value;
      MSTATUSH:  rdata = 32'b0;
      MISA:      rdata = MISA_VALUE;
      MIE:       rdata = mie_value;
      MTVEC:     rdata = mtvec;
      MSCRATCH:  rdata = mscratch;
      MEPC:      rdata = mepc;
      MCAUSE:    rdata = mcause;
      MTVAL:     rdata = mtval;
      MIP:       rdata = mip_value;
      MCYCLE:    rdata = mcycle_lo;
      MCYCLEH:   rdata = mcycle_hi;
      MINSTRET:  rdata = minstret_lo;
      MINSTRETH: rdata = minstret_hi;
      MHARTID:   rdata = HART_ID;
      MVENDORID, MARCHID, MIMPID, MCONFIGPTR: rdata = 32'b0;
      default: begin
        rdata = 32'b0;
        implemented = hpm_zero;
      end
    endcase
  end

  // `warl` is the value the addressed CSR holds after this write, with the
  // legal-value mask applied. It is defined for every address, including the
  // read-only ones, and that is what lets the RVFI report below say what landed
  // rather than echo back an operand the mask threw away.
  logic [31:0] wdata_mtvec, wdata_mepc, wdata_mstatus, wdata_mie;
  // Direct mode only: mtvec[1:0] is the mode field.
  assign wdata_mtvec   = {wdata[31:2], 2'b00};
  // Only bit 0 is forced. C makes 2-byte branch targets legal, so bit 1 is a
  // legal value here and must survive.
  assign wdata_mepc    = {wdata[31:1], 1'b0};
  assign wdata_mstatus = {19'b0, 2'b11, 3'b0, wdata[7], 3'b0, wdata[3], 3'b0};
  // MTIE only. The other bits name sources this platform does not have, and a
  // WARL field with no source behind it reads zero however it was written.
  assign wdata_mie     = {24'b0, wdata[7], 7'b0};

  logic [31:0] warl;
  always_comb begin
    (* parallel_case *)
    case (addr)
      MSTATUS:   warl = wdata_mstatus;
      MIE:       warl = wdata_mie;
      MTVEC:     warl = wdata_mtvec;
      MSCRATCH:  warl = wdata;
      MEPC:      warl = wdata_mepc;
      MCAUSE:    warl = wdata;
      MTVAL:     warl = wdata;
      MCYCLE, MCYCLEH, MINSTRET, MINSTRETH: warl = wdata;
      // Read-only, or not implemented at all: nothing lands, so the value
      // after the write is the value before it.
      default:   warl = rdata;
    endcase
  end
  logic warl_mie, warl_mpie, warl_mtie;
  assign warl_mie  = warl[3];
  assign warl_mpie = warl[7];
  assign warl_mtie = warl[7];

  // Trap entry writes mepc through the same mask an explicit `csrw mepc` goes
  // through. `trap_epc` is an instruction address, so bit 0 is already clear and
  // the mask changes nothing today; what it states is that bit 1 survives,
  // because clearing it too would resume two bytes early after a fault at
  // `pc % 4 == 2`. test/asm/trap.S faults at exactly that alignment.
  logic [31:0] trap_epc_warl;
  assign trap_epc_warl = {trap_epc[31:1], 1'b0};

  logic wr_mcycle, wr_mcycleh, wr_minstret, wr_minstreth, wr_mscratch;
  assign wr_mcycle    = wen && addr == MCYCLE;
  assign wr_mcycleh   = wen && addr == MCYCLEH;
  assign wr_minstret  = wen && addr == MINSTRET;
  assign wr_minstreth = wen && addr == MINSTRETH;
  assign wr_mscratch  = wen && addr == MSCRATCH;

  // "Any CSR write takes precedence over the automatic increment" is about the
  // 64-bit counter, not the half whose address the instruction names: mcycle and
  // mcycleh are two views of one register, so a write to either half suppresses
  // that cycle's increment of the whole thing. Hence a `_tick` term per counter
  // rather than the per-half overrides below doing it alone. Suppressing per
  // half differs only at the carry boundary, where the carry lands in the high
  // half while the write replaces the low one, so `csrw mcycle` would advance
  // mcycleh. test/csr_tb.v is the only thing that sees it.
  logic mcycle_tick, minstret_tick;
  assign mcycle_tick   = !wr_mcycle   && !wr_mcycleh;
  assign minstret_tick = !wr_minstret && !wr_minstreth && instret;

  logic [63:0] mcycle_plus, minstret_plus;
  assign mcycle_plus   = mcycle_tick   ? mcycle   + 64'd1 : mcycle;
  assign minstret_plus = minstret_tick ? minstret + 64'd1 : minstret;

  logic [31:0] mcycle_plus_lo, mcycle_plus_hi, minstret_plus_lo, minstret_plus_hi;
  assign mcycle_plus_lo   = mcycle_plus[31:0];
  assign mcycle_plus_hi   = mcycle_plus[63:32];
  assign minstret_plus_lo = minstret_plus[31:0];
  assign minstret_plus_hi = minstret_plus[63:32];

  logic [31:0] mcycle_next_lo, mcycle_next_hi, minstret_next_lo, minstret_next_hi;
  assign mcycle_next_lo   = wr_mcycle    ? warl : mcycle_plus_lo;
  assign mcycle_next_hi   = wr_mcycleh   ? warl : mcycle_plus_hi;
  assign minstret_next_lo = wr_minstret  ? warl : minstret_plus_lo;
  assign minstret_next_hi = wr_minstreth ? warl : minstret_plus_hi;

  always_ff @(posedge clk) begin
    if (reset) begin
      mcycle       <= 64'b0;
      minstret     <= 64'b0;
      mscratch     <= 32'b0;
      // The spec leaves mtvec's reset value implementation-defined. 0 puts it at
      // the base of .text, so both sim legs catch a trap taken before a handler
      // is installed rather than let it look like a silent program restart.
      mtvec        <= 32'b0;
      mepc         <= 32'b0;
      mcause       <= 32'b0;
      mtval        <= 32'b0;
      mstatus_mie  <= 1'b0;
      mstatus_mpie <= 1'b0;
      mie_mtie     <= 1'b0;
    end else begin
      mcycle   <= {mcycle_next_hi, mcycle_next_lo};
      minstret <= {minstret_next_hi, minstret_next_lo};
      if (wen) begin
        (* parallel_case *)
        case (addr)
          MSTATUS: begin
            mstatus_mie  <= warl_mie;
            mstatus_mpie <= warl_mpie;
          end
          MIE:      mie_mtie <= warl_mtie;
          MTVEC:    mtvec    <= warl;
          MSCRATCH: mscratch <= warl;
          MEPC:     mepc     <= warl;
          MCAUSE:   mcause   <= warl;
          MTVAL:    mtval    <= warl;
          // The counters are driven unconditionally above, with an explicit
          // write folded into *_next so it beats the increment.
          default: ;
        endcase
      end
      // The privileged-spec sequence: a trap saves the faulting PC and the
      // cause, pushes MIE into MPIE and disables interrupts; `mret` pops it back
      // and leaves MPIE set. Clearing MIE here is what bounds interrupt entry:
      // `interrupt_pending` goes low on this same edge, so the cycle after a
      // trap cannot take another one, and nothing re-arms until an `mret` or an
      // explicit write. `else if` rather than two independent `if`s because the
      // two are mutually exclusive by construction (see the port comments), so a
      // future cause that broke that fails at rtl/decoder.v's priority encoder
      // rather than silently double-writing mstatus.
      if (trap_entry) begin
        mepc         <= trap_epc_warl;
        mcause       <= trap_cause;
        mtval        <= trap_tval;
        mstatus_mpie <= mstatus_mie;
        mstatus_mie  <= 1'b0;
      end else if (mret_entry) begin
        mstatus_mie  <= mstatus_mpie;
        mstatus_mpie <= 1'b1;
      end
    end
  end

 `ifdef RISCV_FORMAL
  // Write-only with respect to the core: nothing below drives a signal any
  // non-`ifdef` logic reads. That is what `make -C formal nonperturbation` rests
  // on, so do not break it.
  //
  // The two counters are one 64-bit RVFI CSR each, with mcycleh/minstreth
  // addressing the upper half of the same report, which is why the masks are
  // built per half. rvfi_csrw_check asserts that a write through the low address
  // leaves the high half unchanged, so a mask claiming both fails on a correct
  // core.
  logic rd_mcycle, rd_mcycleh, rd_minstret, rd_minstreth, rd_mscratch;
  assign rd_mcycle    = ren && addr == MCYCLE;
  assign rd_mcycleh   = ren && addr == MCYCLEH;
  assign rd_minstret  = ren && addr == MINSTRET;
  assign rd_minstreth = ren && addr == MINSTRETH;
  assign rd_mscratch  = ren && addr == MSCRATCH;

  // What the instruction wrote, which is not what the register will hold: the
  // free-running increment is not this instruction's doing, and RVFI's wdata is
  // the instruction's write.
  logic [31:0] mcycle_next_lo_reported, mcycle_next_hi_reported;
  logic [31:0] minstret_next_lo_reported, minstret_next_hi_reported;
  assign mcycle_next_lo_reported   = wr_mcycle    ? warl : mcycle_lo;
  assign mcycle_next_hi_reported   = wr_mcycleh   ? warl : mcycle_hi;
  assign minstret_next_lo_reported = wr_minstret  ? warl : minstret_lo;
  assign minstret_next_hi_reported = wr_minstreth ? warl : minstret_hi;

  always_comb begin
    rvfi_mcycle.rmask = {{32{rd_mcycleh}}, {32{rd_mcycle}}};
    rvfi_mcycle.wmask = {{32{wr_mcycleh}}, {32{wr_mcycle}}};
    rvfi_mcycle.rdata = mcycle;
    rvfi_mcycle.wdata = {mcycle_next_hi_reported, mcycle_next_lo_reported};

    rvfi_minstret.rmask = {{32{rd_minstreth}}, {32{rd_minstret}}};
    rvfi_minstret.wmask = {{32{wr_minstreth}}, {32{wr_minstret}}};
    rvfi_minstret.rdata = minstret;
    rvfi_minstret.wdata = {minstret_next_hi_reported, minstret_next_lo_reported};

    rvfi_mscratch.rmask = {32{rd_mscratch}};
    rvfi_mscratch.wmask = {32{wr_mscratch}};
    rvfi_mscratch.rdata = mscratch;
    rvfi_mscratch.wdata = wr_mscratch ? warl : mscratch;
  end

  `ifdef RISCV_FORMAL_CSR_MCAUSE
  // Two writers, and trap entry is the one the fault check is about: it lands
  // on the same edge the trapping instruction issues, so it is that
  // instruction's write and the whole register is written. The `csrw` path is
  // reported the way mscratch's is.
  logic rd_mcause, wr_mcause;
  assign rd_mcause = ren && addr == MCAUSE;
  assign wr_mcause = wen && addr == MCAUSE;

  always_comb begin
    rvfi_mcause.rmask = {32{rd_mcause}};
    rvfi_mcause.wmask = {32{wr_mcause || trap_entry}};
    rvfi_mcause.rdata = mcause;
    rvfi_mcause.wdata = trap_entry ? trap_cause : (wr_mcause ? warl : mcause);
  end
  `endif
 `endif
endmodule
