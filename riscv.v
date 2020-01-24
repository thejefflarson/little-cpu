module riscv (
  input             clk, reset,

  // 32bit AXI4-lite memory interface
  output reg        awvalid, // we wrote the address
  input             awready, // address is ready for write
  output reg [31:0] awaddress, // address to write
  output reg [2:0]  awprot, // permissions

  output reg        wvalid, // we wrote the value
  input             wready, // value is ready
  output reg [31:0] wdata, // value to write
  output reg [3:0]  wstrb, // what bytes we wrote

  output reg        bvalid, // we received the response
  input             bready, // status is ready
  input      [1:0]  bresp, // status of our write request

  output reg        arvalid, // we put something in the read addreas
  input             arready, // they are reading the value
  output reg [31:0] araddress, // address to read
  output reg [2:0]  arprot, // permissions

  input             rvalid, // Data is valid and can be read by us
  output reg        rready, // we are ready to read
  input      [31:0] rdata, // value to read
  input      [1:0]  rresp, // status of our read request

  // outputs
  output reg        trap,
  output reg [1:0]  trap_code
);
  reg [31:0] regs[0:31];
  `define zero regs[0];
  `define ra regs[1];
  `define sp regs[2];
  `define gp regs[3];
  `define fp regs[8];

  reg [31:0] pc;
  reg [31:0] instr;
  // did we get a memory fault
  reg mem_trap;
  // memory error code
  localparam trap_mem = 2'b00;
  // we can execute, no memory operations pending
  reg execute;
  // load an instruction, otherwise load memory
  reg load_instr;
  // storage for requested memory and address
  reg [31:0] load_address;
  reg [31:0] load_data;

  // storage for the next program counter and instruction
  reg [31:0] next_pc;
  reg [31:0] next_instr;

  integer i;

  // reset registers
  always_ff @(posedge clk) begin
    if (!reset) begin
      for (i = 0; i < 32; i = i + 1) begin
        regs[i] <= 32'b1;
      end
    end
  end

  // reset memory
  always_ff @(posedge clk) begin
    if (!reset) begin
      arvalid <= 0; // we haven't put anything in the address
      rready <= 0; // we are ready to read
      execute <= 0;
      next_pc <= 32'b0;
      arprot <= inst_prot;
      load_instr <= 0;
      load_address <= 32'b0;
      load_data <= 32'b0;
    end
  end
  // privileged, insecure and instruction protection flag
  localparam inst_prot = 3'b101;
  // privileged, insecure and data protection flag
  localparam data_prot = 3'b000;
  // memory request
  always_ff @(posedge clk) begin
    // we aren't executing and we haven't requested a read
    if (reset && !execute && !arvalid) begin
      if (load_instr) begin
        araddress <= next_pc;
        arprot <= inst_prot;
      end else begin
        araddress <= load_address;
        arprot <= data_prot;
      end
      // we're ready to read
      arvalid <= 1;
      rready <= 1;
    end
  end

  localparam rresp_ok = 2'b00;
  localparam rresp_xok = 2'b01;
  // memory fetch
  always_ff @(posedge clk) begin
    // everyone is ready
    if (reset && rready && rvalid && arready && arvalid) begin
      if (rresp == rresp_ok || rresp == rresp_xok) begin
        execute <= 1;
        if (load_instr)
          next_instr <= rdata;
        else
          load_data <= rdata;
      end else begin
        mem_trap <= 1;
      end
      rready <= 0;
      arvalid <= 0;
    end
  end

  localparam bresp_ok = 2'b01;
  localparam bresp_xok = 2'b00;
  // memory store
  always_ff @(posedge clk) begin
    // We wrote something and are waiting for the response
    if (reset && !execute && wvalid && awvalid) begin
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
  logic math_flag = instr[31:25] == 7'b0100000;

  logic [2:0] funct3 = instr[14:12];
  logic [6:0] funct7 = instr[31:25];
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

  reg [4:0] cpu_state;
  localparam fetch_instr = 5'b00001;
  localparam execute_instr = 5'b00010;
  localparam finish_load = 5'b00011;
  localparam finish_store = 5'b00100;
  localparam cpu_trap = 5'b00000;

  // state_machine
  always_ff @(posedge clk) begin
    if (!reset) begin
      cpu_state <= fetch_instr;
    end else begin
      case (cpu_state)
        fetch_instr: begin
          if (execute) begin
            pc <= next_pc;
            instr <= next_instr;
            cpu_state <= execute_instr;
          end else begin
            load_instr <= 1;
            next_pc <= pc + 4;
          end
        end

        execute_instr: begin
          case (1'b1)
            is_lb || is_lh || is_lw || is_lbu || is_lhu: begin
              if (rd == 0) begin // can't load into x0
                cpu_state <= cpu_trap;
              end else begin
                load_address <= $signed(load_immediate) + $signed(regs[rs1]);
                load_instr <= 0; // can we have data
                cpu_state <= finish_load;
                execute <= 0; // kick off a memory request
              end
            end

            is_sw || is_sb || is_sh: begin
              awaddress <= $signed(store_immediate) + $signed(regs[rs1]);
	      wdata <= regs[rs2];
	      case (1'b1)
		is_sw: wstrb <= 4'b1111;
		is_sh: wstrb <= 4'b0011;
		is_sb: wstrb <= 4'b0001;
	      endcase
	      awprot <= data_prot;
              wvalid <= 1;
              awvalid <= 1;
              execute <= 0; // kick off a memory request
            end
          endcase
        end

        finish_load: begin
          if (execute) begin
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
	    end else
              cpu_state <= fetch_instr;
          end
        end

        finish_store: begin
          if (execute) begin
            if (mem_trap) begin
	      cpu_state <= cpu_trap;
	      trap_code <= trap_mem;
	    end else
              cpu_state <= fetch_instr;
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
endmodule
