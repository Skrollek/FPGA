module rng #(
    parameter FIRST_STATE = 42;
) (
    input  logic clk,
    output logic [7:0] out,
    input  logic user_input,

);
    logic [31:0] state = FIRST_STATE;
    logic [31:0] next;

    always_comb begin 
        next = state;
        next = next ^ (next << 11);
        next = next ^ (next >> 22);
        next = next ^ (next << 1);
        if(user_input) begin
            next = next ^ (next << 11);
            next = next ^ (next >> 22);
            next = next ^ (next << 1);
        end
    end

    always_ff @(posedge clk) begin
        state <= next;
    end

    assign out = state[7:0];
endmodule