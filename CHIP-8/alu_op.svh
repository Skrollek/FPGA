typedef enum logic [3:0] {
    ALU_Y   = 4'h0,
    ALU_OR  = 4'h1,
    ALU_AND = 4'h2,
    ALU_XOR = 4'h3,
    ALU_ADD = 4'h4,
    ALU_SUB = 4'h5,
    ALU_SHR = 4'h6,
    ALU_SHL = 4'h7,
    ALU_EQL = 4'h8,
    ALU_GRE = 4'h9,
    ALU_INC = 4'hA
} alu_op_t;
