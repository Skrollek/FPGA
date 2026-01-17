`include "decoder.sv"
`include "logic_file.sv"
`include "rng.sv"
`include "ro_mem.sv"
`include "memory.sv"
`include "alu.sv"
`include "gpu.sv"
`include "alu_op.svh"

module cpu (
    input  logic clk,
    input  logic tick_60hz,
    input  logic tick_next,
    input  logic [15:0] keys,
    output logic out
);
module cpu (
    input  logic        clk,
    input  logic        nrst,
    input  logic        tick_60hz,
    input  logic        tick_next,
    input  logic [15:0] keys,
    output logic        out
);

  // ------------------------------------------------------------
  // CPU state machine
  // ------------------------------------------------------------
  typedef enum logic [4:0] {
    STATE_NEXT           = 5'd0,
    STATE_FETCH_HI       = 5'd1,
    STATE_FETCH_LO       = 5'd2,
    STATE_DECODE         = 5'd3,
    STATE_LOAD_VX        = 5'd4,
    STATE_LOAD_VY        = 5'd5,
    STATE_LOAD_V0        = 5'd6,
    STATE_STORE_VX       = 5'd7,
    STATE_STORE_CARRY    = 5'd8,
    STATE_PUSH_HI        = 5'd9,
    STATE_PUSH_LO        = 5'd10,
    STATE_POP_HI         = 5'd11,
    STATE_POP_LO         = 5'd12,
    STATE_TRANSFER_LOAD  = 5'd13,
    STATE_TRANSFER_STORE = 5'd14,
    STATE_CLEAR          = 5'd15,
    STATE_BCD_1          = 5'd16,
    STATE_BCD_2          = 5'd17,
    STATE_BCD_3          = 5'd18,
    STATE_GPU            = 5'd19,
    STATE_STOP           = 5'd31
  } cpu_state_t;

  cpu_state_t state, state_n;

  // ------------------------------------------------------------
  // Memory interface
  // ------------------------------------------------------------
  logic        mem_wren;
  logic        mem_reen;
  logic [11:0] mem_write_addr;
  logic [11:0] mem_read_addr;
  logic [7:0]  mem_write_data;
  logic [7:0]  mem_read_data;
  logic        mem_read_ack;

  Chip8_memory mem0 (
    .clk        (clk),
    .wren       (mem_wren),
    .write_addr (mem_write_addr),
    .write_data (mem_write_data),
    .reen       (mem_reen),
    .read_addr  (mem_read_addr),
    .read_data  (mem_read_data),
    .read_ack   (mem_read_ack)
  );

  // ------------------------------------------------------------
  // Register file
  // ------------------------------------------------------------
  logic [3:0] reg_addr;
  logic [7:0] reg_wdata;
  logic [7:0] vx, vy;
  logic       reg_wren;
  logic       reg_wren_i;

  reg_file rf0 (
    .clk     (clk),
    .nrst    (nrst),
    .addr    (reg_addr),
    .a       (reg_wdata),
    .wren    (reg_wren),
    .wren_i  (reg_wren_i),
    .x       (vx),
    .y       (vy)
  );

  // ------------------------------------------------------------
  // Decoder
  // ------------------------------------------------------------
  logic [15:0] instr;
  logic [3:0]  op_main;
  logic [3:0]  op_sub;
  logic [3:0]  x;
  logic [3:0]  y;
  logic [11:0] nnn;
  logic [7:0]  nn;
  logic [3:0]  n;
  alu_op_t     alu_op;

  decoder dec0 (
    .op_code (instr),
    .op_main (op_main),
    .op_sub  (op_sub),
    .x       (x),
    .y       (y),
    .nnn     (nnn),
    .nn      (nn),
    .n       (n),
    .alu_op  (alu_op)
  );

  // ------------------------------------------------------------
  // Timers and core registers
  // ------------------------------------------------------------
  logic [11:0] pc;
  logic [11:0] I;
  logic [11:0] ret_pc;
  logic [3:0]  sp;
  logic [7:0]  dt, st;
  logic        carry;

  // ------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------
  logic [7:0] new_vx;
  logic [11:0] transfer_src_addr, transfer_dest_addr;
  logic [7:0]  transfer_counter;

  // GPU
  logic gpu_draw = 0;
  logic [11:0] gpu_addr;
  logic [3:0] gpu_lines;
  logic [5:0] gpu_x;
  logic [4:0] gpu_y;
  logic gpu_busy;
  logic gpu_collision;
  logic gpu_read;
  logic [11:0] gpu_read_idx;
  logic gpu_write;
  logic [11:0] gpu_write_idx;
  logic [7:0] gpu_write_byte;

  gpu gpu0( .clk(clk),
            .draw(gpu_draw),
            .addr(gpu_addr),
            .lines(gpu_lines),
            .x(gpu_x),
            .y(gpu_y),
            .busy(gpu_busy),
            .collision(gpu_collision),
            .mem_read(gpu_read),
            .mem_read_idx(gpu_read_idx),
            .mem_read_byte(mem_read_byte), // pass-through
            .mem_read_ack(mem_read_ack), // pass-through
            .mem_write(gpu_write),
            .mem_write_idx(gpu_write_idx),
            .mem_write_byte(gpu_write_byte));


always_comb begin
    mem_reen = 0;
    mem_wren = 0;
    mem_read_addr = 0;
    wren_to_r0 = 0;
    reg_addr = 0;

    case (state)
      STATE_NEXT, STATE_STOP: begin
        mem_reen = scr_read;
        mem_read_addr = {4'h1, scr_read_idx};
      end
      STATE_FETCH_HI: if (!mem_read_ack) begin
        mem_reen = 1;
        mem_read_addr = pc[11:0];
      end
      STATE_FETCH_LO: if (!mem_read_ack) begin
        mem_reen = 1;
        mem_read_addr = pc[11:0] + 1;
      end
      STATE_LOAD_VX: 
        reg_addr = x;
      STATE_LOAD_VY: 
        reg_addr = y;
      STATE_LOAD_V0:
        reg_addr = '0;
      STATE_POP_HI: if (!mem_read_ack) begin
        mem_reen = 1;
        mem_read_addr = 2 * sp;
      end
      STATE_POP_LO: if (!mem_read_ack) begin
        mem_reen = 1;
        mem_read_addr = 2 * sp + 1;
      end
      STATE_PUSH_HI: begin
        mem_wren = 1;
        mem_write_addr = 2 * sp - 2;
        mem_write_data = {4'b0, ret_pc[11:8]};
      end
      STATE_PUSH_LO: begin
        mem_wren = 1;
        mem_write_addr = 2 * sp - 1;
        mem_write_data = ret_pc[7:0];
      end
      STATE_TRANSFER_LOAD: begin
        if (!transfer_to_mem) begin
          // -------- memory → registers --------
          if (!mem_read_ack) begin
              mem_reen = 1;
              mem_read_addr = transfer_src_addr
                            + {4'b0, transfer_counter};
          end else begin
              reg_wren  = 1;
              reg_addr  = transfer_counter[3:0];
              reg_wdata = mem_read_data;
          end
        end
      // if transfer_to_mem == 1 → nothing to do here
      end

     STATE_TRANSFER_STORE: begin
      if (transfer_to_mem) begin
        // -------- registers → memory --------
        reg_addr = transfer_counter[3:0];
        mem_wren = 1;
        mem_write_addr = transfer_dest_addr
                       + {4'b0, transfer_counter};
        mem_write_data = vx;
      end
      // if transfer_to_mem == 0 → store already done in LOAD
      end

      STATE_STORE_VX: begin
        wren_to_r0 = 1;
        reg_addr   = x;
        new_reg_value = new_vx;
      end
      STATE_STORE_CARRY: begin
        wren_to_r0 = 1;
        reg_addr = 4'hF;
        new_reg_value = {7'b0, carry};
      end
      STATE_BCD_1: begin
        mem_wren = 1;
        mem_write_addr = addr;
        mem_write_data = {6'b0, bcd_1};
      end
      STATE_BCD_2: begin
        mem_wren = 1;
        mem_write_addr = addr + 1;
        mem_write_data = {4'b0, bcd_2};
      end
      STATE_BCD_3: begin
        mem_wren = 1;
        mem_write_addr = addr + 2;
        mem_write_data = {4'b0, bcd_3};
      end
      STATE_GPU: begin
        mem_reen = gpu_read;
        mem_read_addr = gpu_read_idx;
        mem_wren = gpu_write;
        mem_write_addr = gpu_write_idx;
        mem_write_data = gpu_write_byte;
      end
    endcase
  end


logic [31:0] rng_state;
rng rng(.clk(clk), .out(rng_state), .user_input(&keys));


reg [11:0] pc = 'h200;
reg [11:0] ret_pc;
reg [11:0] addr = 0;
reg [11:0] transfer_src_addr, transfer_dest_addr;
reg [7:0] transfer_counter;
reg [3:0] sp = 0;
reg [7:0] dt = 0;
reg [7:0] st = 0;

reg [7:0] vx, vy, new_vx;
reg carry;
wire needs_carry = op_main == 'h8 && (n == 'h4 || n == 'h5 || n == 'h6 || n == 'h7 || n == 'hE);
// Can go to the next instruction (for rate limiting by tick_next)
reg next = 1;



reg [15:0] instr;
wire [3:0] op_main;
logic [3:0]  sub_op;
wire [3:0] x;
wire [3:0] y;
wire [11:0] nnn;
wire [7:0] nn;
wire [3:0] n;
alu_op_t alu_op;
decoder d0(
  instr,
  op_main,
  op_sub,
  x,
  y,
  nnn,
  nn,
  n,
  alu_op
)



always_ff @(posedge clk) begin
  if (tick_60hz) begin
      if (dt != 0) dt <= dt - 1;
      if (st != 0) st <= st - 1;
  end
  if(tick_next) next <= 1;
  scr_read_ack <= 0;

    unique case (state)

        STATE_NEXT: begin
            if (scr_read && mem_read_ack) begin
                scr_read_ack <= 1;
                scr_read_byte <= mem_read_byte;
            end
            if (state == STATE_NEXT && !scr_busy && next) begin
                next <= 0;
                state <= STATE_FETCH_HI;
            end
        end

        STATE_FETCH_HI: begin
          if(mem_read_ack) begin
            instr[15:8] <= mem_read_byte;
            state       <= STATE_FETCH_LO;
          end
        end

        STATE_FETCH_LO: begin
            if(mem_read_ack) begin
                instr[7:0] <= mem_read_byte;
                if (op_main == 'hB)
                  state <= STATE_LOAD_V0;
            end else begin
                  state <= STATE_LOAD_VX;
            end
        end

        STATE_LOAD_VX: begin
            vx <= reg_value;
            state <= STATE_LOAD_VY;
        end

        STATE_LOAD_VY: begin
            vy <= reg_value;
            state <= STATE_DECODE;
        end

        STATE_LOAD_V0: begin
            // Load V0 (used by JP V0, nnn)
            vx <= reg_value;
            state <= STATE_DECODE;
        end

        STATE_POP_HI: begin
            // Pop return address high nibble from stack
            if (mem_read_ack) begin
              pc[11:8] <= mem_read_byte[3:0];
              state <= STATE_POP_LO;
            end
        end

        STATE_POP_LO: begin
            // Pop return address low byte from stack
            if (mem_read_ack) begin
              pc[7:0] <= mem_read_byte;
              state <= STATE_NEXT;
            end
        end

        STATE_PUSH_HI: begin
            // Push return address high nibble to stack
            state <= STATE_PUSH_LO;
        end

        STATE_PUSH_LO: begin
            // Push return address low byte to stack
            state <= STATE_NEXT;
        end


        STATE_TRANSFER_LOAD: begin
          if (!transfer_to_mem) begin
            if (mem_read_ack)
              state <= STATE_TRANSFER_STORE;
          end else begin
          // Fx55 skips read phase
          state <= STATE_TRANSFER_STORE;
          end
        end

        STATE_TRANSFER_STORE: begin
            if (transfer_counter == 0) begin
                state <= STATE_NEXT;
            end else begin
                transfer_counter <= transfer_counter - 1;
                state <= STATE_TRANSFER_LOAD;
            end
        end

        STATE_CLEAR: begin
            // Clear screen memory (CLS)
            if (transfer_counter == 0)
                state <= STATE_NEXT;
            else begin
                transfer_counter <= transfer_counter - 1;
            end
        end

        STATE_STORE_VX: begin
            state <= needs_carry ? STATE_STORE_CARRY : STATE_NEXT;
        end

        STATE_STORE_CARRY: begin
            // Store carry flag into VF
            state <= STATE_NEXT;    
        end

        STATE_BCD_1: begin
            // Store hundreds digit
            state <= STATE_BCD_2;
        end

        STATE_BCD_2: begin
            // Store tens digit
            state <= STATE_BCD_3;
        end

        STATE_BCD_3: begin
            // Store ones digit
            state <= STATE_NEXT;    
        end

        STATE_GPU: begin
            // Wait for GPU draw to complete
            begin
          gpu_draw <= 0;
          if (!gpu_draw && !gpu_busy) begin
            carry <= gpu_collision;
            state <= STATE_STORE_CARRY;
          end
        end
        end

        STATE_STOP: begin
            // CPU halted (EXIT instruction)
            if (scr_read && mem_read_ack) begin
                scr_read_ack <= 1;
                scr_read_byte <= mem_read_byte;
            end
            if (state == STATE_NEXT && !scr_busy && next) begin
                next <= 0;
                state <= STATE_FETCH_HI;
            end
        end
        STATE_DECODE: begin
    pc <= pc + 2;
    state <= STATE_NEXT;

    case (op_main)

        // ------------------------------------------------
        // 0x00E0 / 00EE / 00FD
        // ------------------------------------------------
        4'h0: begin
            case (nnn)
                12'h0E0: begin
                    transfer_dest_addr <= 12'h100;
                    transfer_counter   <= 8'hFF;
                    state <= STATE_CLEAR;
                end
                12'h0EE: begin
                    sp <= sp - 1;
                    state <= STATE_POP_HI;
                end
                12'h0FD: begin
                    state <= STATE_STOP;
                end
                default: ;
            endcase
        end

        // ------------------------------------------------
        // JP addr
        // ------------------------------------------------
        4'h1: pc <= nnn;

        // ------------------------------------------------
        // CALL addr
        // ------------------------------------------------
        4'h2: begin
            sp <= sp + 1;
            ret_pc <= pc + 2;
            pc <= nnn;
            state <= STATE_PUSH_HI;
        end

        // ------------------------------------------------
        // SE / SNE
        // ------------------------------------------------
        4'h3: if (vx == nn) pc <= pc + 4;
        4'h4: if (vx != nn) pc <= pc + 4;
        4'h5: if (vx == vy) pc <= pc + 4;
        4'h9: if (vx != vy) pc <= pc + 4;

        // ------------------------------------------------
        // LD Vx, imm
        // ------------------------------------------------
        4'h6: begin
            new_vx <= nn;
            state <= STATE_STORE_VX;
        end

        // ------------------------------------------------
        // ADD Vx, imm
        // ------------------------------------------------
        4'h7: begin
            new_vx <= vx + nn;
            state <= STATE_STORE_VX;
        end

        // ------------------------------------------------
        // ALU ops (8xy*)
        // ------------------------------------------------
        4'h8: begin
            new_vx <= alu_res;
            carry  <= alu_carry;
            state  <= STATE_STORE_VX;
        end

        // ------------------------------------------------
        // LD I, addr
        // ------------------------------------------------
        4'hA: I <= nnn;

        // ------------------------------------------------
        // JP V0, addr
        // ------------------------------------------------
        4'hB: pc <= nnn + vx;

        // ------------------------------------------------
        // RND
        // ------------------------------------------------
        4'hC: begin
            new_vx <= rng_state[15:8] & nn;
            state <= STATE_STORE_VX;
        end

        // ------------------------------------------------
        // DRW
        // ------------------------------------------------
        4'hD: begin
            gpu_draw  <= 1;
            gpu_addr  <= I;
            gpu_lines <= n;
            gpu_x     <= vx[5:0];
            gpu_y     <= vy[4:0];
            state     <= STATE_GPU;
        end

        // ------------------------------------------------
        // Keys
        // ------------------------------------------------
        4'hE: begin
            case (nn)
                8'h9E: if (keys[vx[3:0]]) pc <= pc + 4;
                8'hA1: if (!keys[vx[3:0]]) pc <= pc + 4;
                default: ;
            endcase
        end

        // ------------------------------------------------
        // Fx**
        // ------------------------------------------------
        4'hF: begin
            case (nn)
                8'h07: begin new_vx <= dt; state <= STATE_STORE_VX; end
                8'h15: dt <= vx;
                8'h18: st <= vx;
                8'h1E: I <= I + vx;
                8'h29: I <= 12'h30 + vx * 5;
                8'h33: state <= STATE_BCD_1;
                8'h55: begin
                  // LD [I], Vx  (registers → memory)
                  transfer_to_mem    <= 1'b1;
                  transfer_src_addr  <= 12'h020;   // register file base
                  transfer_dest_addr <= I;
                  transfer_counter   <= x;
                  state <= STATE_TRANSFER_LOAD;
                end
                8'h65: begin
                  // LD Vx, [I]  (memory → registers)
                  transfer_to_mem    <= 1'b0;
                  transfer_src_addr  <= I;
                  transfer_dest_addr <= 12'h020;   // register file base
                  transfer_counter   <= x;
                  state <= STATE_TRANSFER_LOAD;
                end

                default: ;
            endcase
        end

        default: ;
    endcase
end

    endcase
end

endmodule