module riscv (
  input  var        clk, reset,

  // 32bit AXI4-lite memory interface
  output var        awvalid, // we wrote the address
  input  var        awready, // address is ready for write
  output var [31:0] awaddress, // address to write
  output var [2:0]  awprot, // permissions

  output var        wvalid, // we wrote the value
  input  var        wready, // value is ready
  output var [31:0] wdata, // value to write
  output var [3:0]  wstrb, // what bytes we wrote

  output var        bvalid, // we received the response
  input  var        bready, // status is ready
  input  var [1:0]  bresp, // status of our write request

  output var        arvalid, // we put something in the read addreas
  input  var        arready, // they are reading the value
  output var [31:0] araddress, // address to read
  output var [2:0]  arprot, // permissions

  input  var        rvalid, // Data is valid and can be read by us
  output var        rready, // we are ready to read
  input  var [31:0] rdata, // value to read
  input  var [1:0]  rresp, // status of our read request

  // outputs
  output var        trap,
  output var [1:0]  trap_code

  // Formal
`ifdef RISCV_FORMAL
   ,
   `RVFI_OUTPUTS
`endif
);
  // did we get a memory fault
  logic mem_trap;
  // memory error code
  localparam trap_mem = 2'b00;
  // privileged, insecure and instruction protection flag
  localparam inst_prot = 3'b101;
  // privileged, insecure and data protection flag
  localparam data_prot = 3'b000;
  localparam rresp_ok = 2'b00;
  localparam rresp_xok = 2'b01;
  localparam bresp_ok = 2'b01;
  localparam bresp_xok = 2'b00;

  logic [31:0] load_data;
  // memory interface TODO this would be better as a module
  always_ff @(posedge clk) begin
    // reset memory
    if (!reset) begin
      arvalid <= 0;
      rready <= 0;
      execute <= 0;
      arprot <= inst_prot;
      load_data <= 32'b0;
    end

    // memory fetch
    // the core has requested a read but we haven't forwarded to memory
    if (reset && memory && load && !arvalid) begin
      if (load_instr) begin
        araddress <= next_pc;
        arprot <= inst_prot;
      end else begin
        araddress <= memory_address;
        arprot <= data_prot;
      end
      // we're ready to read
      arvalid <= 1;
      rready <= 1;
      execute <= 0;
    end

    // everyone is ready to finish the read and start executing
    if (reset && memory && !execute && rready && rvalid && arready && arvalid) begin
      if (rresp == rresp_ok || rresp == rresp_xok) begin
        execute <= 1;
        if (load_instr) begin
          next_instr <= rdata;
        end else begin
          load_data <= rdata;
        end
      end else begin
        mem_trap <= 1;
      end
      rready <= 0;
      arvalid <= 0;
    end

    // memory store
    if (reset && memory && store && !wvalid && !awvalid) begin
      awaddress <= memory_address;
      wdata <= store_data;
      if (store_instr) begin
        awprot <= inst_prot;
      end else begin
        awprot <= data_prot;
      end
      wvalid <= 1;
      awvalid <= 1;
      execute <= 0;
    end

    // We wrote something and are waiting for the response
    if (reset && memory && !execute && wvalid && awvalid) begin
      // We can check the status of our write request
      if (awready && wready && bready) begin
        awvalid <= 0;
        wvalid <= 0;
        if (bresp != bresp_ok || bresp != bresp_xok) begin
          mem_trap <= 1;
        end else begin
          execute <= 1;
        end
        bvalid <= 1; // we're all done
      end else begin
        bvalid <= 0;
      end
    end
  end

  // instruction decoder (figure 2.3)
  logic [6:0] opcode = instr[6:0];
  logic [4:0] rd = instr[11:7];
  logic [4:0] rs1 = instr[19:15];
  logic [4:0] rs2 = instr[24:20];
  // For shift immediates
  logic [4:0] shamt = rs2;

  logic [2:0] funct3 = instr[14:12];
  logic [6:0] funct7 = instr[31:25];
  logic math_flag = funct7 == 7'b0100000;

  // immediate decoder (figure 2.4)
  logic [31:0] i_immediate = {{20{instr[31]}}, instr[31:20]};
  logic [31:0] s_immediate = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  logic [31:0] b_immediate = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  logic [31:0] u_immediate = {instr[31], instr[30:20], instr[19:12], 12'b0};
  logic [31:0] j_immediate = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

  // Table 24.2 RV32I
  logic is_lui = opcode == 7'b0110111;
  logic is_auipc = opcode == 7'b0010111;
  logic is_jal = opcode == 7'b1101111;
  logic is_jalr = opcode == 7'b1100111;

  logic is_branch = opcode == 7'b1100011;
  logic is_beq = is_branch && funct3 == 3'b000;
  logic is_bne = is_branch && funct3 == 3'b001;
  logic is_blt = is_branch && funct3 == 3'b100;
  logic is_bge = is_branch && funct3 == 3'b101;
  logic is_bltu = is_branch && funct3 == 3'b110;
  logic is_bgeu = is_branch && funct3 == 3'b111;

  logic is_load = opcode == 7'b0000011;
  logic is_lb = is_load && funct3 == 3'b000;
  logic is_lh = is_load && funct3 == 3'b001;
  logic is_lw = is_load && funct3 == 3'b010;
  logic is_lbu = is_load && funct3 == 3'b100;
  logic is_lhu = is_load && funct3 == 3'b101;

  logic is_store = opcode == 7'b0100011;
  logic is_sb = is_store && funct3 == 3'b000;
  logic is_sh = is_store && funct3 == 3'b010;
  logic is_sw = is_store && funct3 == 3'b100;

  logic is_math_immediate = opcode == 7'b0010011;
  logic is_addi = is_math_immediate && funct3 == 3'b000;
  logic is_slti = is_math_immediate && funct3 == 3'b010;
  logic is_sltiu = is_math_immediate && funct3 == 3'b011;
  logic is_xori = is_math_immediate && funct3 == 3'b100;
  logic is_ori = is_math_immediate && funct3 == 3'b110;
  logic is_andi = is_math_immediate && funct3 == 3'b111;
  logic is_slli = is_math_immediate && funct3 == 3'b001;
  logic is_srli = is_math_immediate && !math_flag && funct3 == 3'b101;
  logic is_srai = is_math_immediate && math_flag && funct3 == 3'b101;

  logic is_math = opcode == 7'b0110011;
  logic is_add = is_math && !math_flag && funct3 == 3'b000;
  logic is_sub = is_math && math_flag && funct3 == 3'b000;
  logic is_sll = is_math && funct3 == 3'b001;
  logic is_slt = is_math && funct3 == 3'b010;
  logic is_sltu = is_math && funct3 == 3'b011;
  logic is_xor = is_math && funct3 == 3'b100;
  logic is_srl = is_math && !math_flag && funct3 == 3'b101;
  logic is_sra = is_math && math_flag && funct3 == 3'b101;
  logic is_or = is_math && funct3 == 3'b110;
  logic is_and = is_math && funct3 == 3'b111;

  logic is_fence = opcode == 7'b0001111;
  logic is_error = opcode == 7'b1110011;
  logic is_ecall = is_error && !instr[20];
  logic is_ebreak = is_error && instr[20];

  logic [31:0]immediate;
  always_comb begin
    case (1'b1)
      is_load || is_jalr: immediate = i_immediate;
      is_store: immediate = s_immediate;
      is_lui || is_auipc: immediate = u_immediate;
      is_jal: immediate = j_immediate;
      is_branch: immediate = b_immediate;
      default: immediate = 32'b0;
    endcase
  end

  // registers
  logic [31:0] regs[0:31];
  `define zero regs[0];
  `define ra regs[1];
  `define sp regs[2];
  `define gp regs[3];
  `define fp regs[8];
  logic [31:0] pc;
  logic [31:0] instr;
  // storage for the next program counter and instruction
  logic [31:0] next_pc;
  logic [31:0] next_instr;

  // memory requests
  // we can execute, no memory operations pending
  logic execute;
  // we are waiting on memory
  logic memory;
  // we would like to load
  logic load;
  // load an instruction, otherwise load memory
  logic load_instr;
  // storage for requested address
  // we would like to store
  logic store;
  // store instruction or memory
  logic store_instr;
  // storage for memory request
  logic [31:0] store_data;
  logic [31:0] memory_address;


  // state_machine
  logic [4:0] cpu_state;
  localparam fetch_instr = 5'b00001;
  localparam execute_instr = 5'b00010;
  localparam finish_load = 5'b00011;
  localparam finish_store = 5'b00100;
  localparam cpu_trap = 5'b00000;
  integer i;
  always_ff @(posedge clk) begin
    if (!reset) begin
      for (i = 0; i < 32; i = i + 1) begin
        regs[i] <= 32'b1;
      end
      next_pc <= 32'b0;
      load_instr <= 0;
      store_instr <= 0;
      memory_address <= 32'b0;
      store_data <= 32'b0;
      cpu_state <= fetch_instr;
      memory <= 1;
    end else begin
      case (cpu_state)
        fetch_instr: begin
          if (execute) begin
            pc <= next_pc;
            instr <= next_instr;
            cpu_state <= execute_instr;
          end else begin
            load <= 1;
            load_instr <= 1;
            next_pc <= pc + 4;
            memory <= 1;
          end
        end

        execute_instr: begin
          case (1'b1)
            is_lb || is_lh || is_lw || is_lbu || is_lhu: begin
              if (rd == 0) begin // can't load into x0
                cpu_state <= cpu_trap;
              end else begin
                load <= 1;
                memory_address <= $signed(immediate) + $signed(regs[rs1]);
                load_instr <= 0; // can we have data
                memory <= 1; // kick off a memory request
                cpu_state <= finish_load;
              end
            end

            is_sw || is_sb || is_sh: begin
              memory_address <= $signed(immediate) + $signed(regs[rs1]);
              store_data <= regs[rs2];
              case (1'b1)
                is_sw: wstrb <= 4'b1111;
                is_sh: wstrb <= 4'b0011;
                is_sb: wstrb <= 4'b0001;
              endcase
              store <= 1;
              memory <= 1; // kick off a memory request
              cpu_state <= finish_store;
            end
          endcase
        end

        finish_load: begin
          if (execute && memory) begin
            case (1'b1)
              is_lb: regs[rs2] <= {24'b0, load_data[7:0]};
              is_lbu: regs[rs2] <= {{24{load_data[7]}}, load_data[7:0]};
              is_lh: regs[rs2] <= {16'b0, load_data[15:0]};
              is_lhu: regs[rs2] <= {{16{load_data[7]}}, load_data[15:0]};
              is_lw: regs[rs2] <= load_data;
            endcase
            if (mem_trap) begin
              cpu_state <= cpu_trap;
              trap_code <= trap_mem;
            end else begin
              cpu_state <= fetch_instr;
            end
            memory <= 0;
          end
        end

        finish_store: begin
          if (execute && memory) begin
            if (mem_trap) begin
              cpu_state <= cpu_trap;
              trap_code <= trap_mem;
            end else begin
              cpu_state <= fetch_instr;
            end
            memory <= 0;
          end
        end

        cpu_trap: begin
          trap <= 1;
        end

        default: begin
          cpu_state <= cpu_trap;
        end
      endcase
    end
  end

`ifdef RISCV_FORMAL
  assign rvfi_valid = !reset && execute && !memory;
  assign rvfi_rs2_addr = rs2;
  assign rvfi_rs1_addr = rs1;
  assign rvfi_insn = opcode;
  assign rvfi_rd_addr = rd;
  assign rvfi_trap = trap;
  assign rvfi_halt = trap;
  assign rvfi_pc_rdata = pc;
  assign rvfi_mem_rdata = 0;
  assign rvfi_rs2_rdata = 0;
  assign rvfi_rs1_rdata = 0;
  assign rvfi_rd_wdata = 0;
  assign rvfi_pc_wdata = 0;
  assign rvfi_order = 0;
  assign rvfi_mode = 3;
  assign rvfi_ixl = 1;
  assign rvfi_mem_wmask = 0;
  assign rvfi_mem_wdata = 0;
  assign rvfi_mem_rmask = 0;
  assign rvfi_mem_addr = 0;
  assign rvfi_intr = 0;
`endif
endmodule
