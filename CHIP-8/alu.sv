`include "alu_op.svh"
module alu(
    input  logic [7:0] x,
    input  logic [7:0] y,
    input  alu_op_t    op,
    output logic [7:0] res,
    output logic carry
);

    always_comb begin : 
        case (op)
            ALU_OR: begin
                carry = '0;
                res = x | y;
            end
            ALU_AND: begin
                carry = '0;
                res = x & y;
            end
            ALU_XOR: begin
                carry = '0;
                res = x ^ y;
            end 
            ALU_ADD: begin
                {carry,res} = x + y;
            end
            ALU_SUB: begin
                {carry,res} = x + y;
            end
            ALU_SHR: begin
                carry = '0;
                out = x >> y;
            end
            ALU_SHL: begin
                carry = '0;
                out = x << y;
            end
            ALU_EQL: begin
                carry = '0;
                out = x == y;
            end
            ALU_GRE: begin
                carry = '0;
                out = x > y;
            end
            ALU_INC: begin
                carry = '0;
                out = x + 1'b1;
            end
            default: begin
                carry = 'x;
                out = 'x;
            end 
        endcase
    end

endmodule