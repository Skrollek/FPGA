module ro_mem (
    input  logic clk,
    input  logic addr,
    output logic [15:0] out
)

    logic [15:0] mem [0:511];
    always_ff @(posedge clk) begin
        out <= mem[addr];
    end
    
endmodule