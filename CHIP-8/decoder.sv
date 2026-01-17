`include "alu_op.svh"
module decoder (
    input  logic [15:0] op_code,
    output logic [3:0]  op_main,
    output logic [3:0]  op_sub,    
    output logic [3:0]  x,
    output logic [3:0]  y,
    output logic [11:0] nnn,
    output logic [7:0]  nn,
    output logic [3:0]  n,
    output logic [2:0]  alu_op
);
    assign op_main = op_code[15:12];
    assign x   = op_code[11:8];
    assign y   = op_code[7:4];
    assign nnn = op_code[11:0];
    assign nn  = op_code[7:0];
    assign n   = op_code[3:0];
    always_comb begin
    op_sub = 4'h0;
    is_alu = 1'b0;
    alu_op = '0;

    case (op_main)

        // 00E0, 00EE
        4'h0: begin
            op_sub = n;
        end

        // 8xy?
        4'h8: begin
            is_alu = 1'b1;
            op_sub = n;

            case (n)
                4'h0: alu_op = ALU_Y;  
                4'h1: alu_op = ALU_OR;
                4'h2: alu_op = ALU_AND;
                4'h3: alu_op = ALU_XOR;
                4'h4: alu_op = ALU_ADD;
                4'h5: alu_op = ALU_SUB;
                4'h6: alu_op = ALU_SHR;
                4'h7: alu_op = ALU_SUB; 
                4'hE: alu_op = ALU_SHL;
                default: ;
            endcase
        end

        // Ex9E, ExA1
        4'hE: begin
            op_sub = nn[3:0];
        end

        // Fx??
        4'hF: begin
            op_sub = nn[3:0];
        end

        default: ;
    endcase
end

endmodule