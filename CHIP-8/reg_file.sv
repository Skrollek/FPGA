module reg_file(
    input  logic clk,
    input  loigc nrst,
    input  logic [3:0] addr,
    input  logic [7:0] a,
    input  logic wren,
    input  logic wren_i,
    output logic [7:0] x,
    output logic [7:0] y,
);
    logic [7:0] r [15:0];
    logic [7:0] i;
    assign x = r[addr];
    always_ff @(posedge clk) begin 
        if(wren) r[addr] <= a;
    end
    assign y = i;
    always_ff @(posedge clk) begin 
        if(wren_i) i <= a;
    end


endmodule