module circuit(
    input clk,
    input j,
    input k,
    output q
);

  reg r, r_next;

  always @(*) begin
    case ({j, k})
      2'b00: r_next = r;      
      2'b01: r_next = 1'b0;   
      2'b10: r_next = 1'b1;   
      2'b11: r_next = ~r;     
    endcase
  end

  always @(posedge clk) begin
    r <= r_next;
  end

  assign q = r;
endmodule

module tb;
  reg clk, j, k;
  wire q;

  circuit u(clk, j, k, q);

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    j = 0; k = 0;
    #10 j = 0; k = 1;
    #10 j = 1; k = 0;
    #10 j = 1; k = 1;
    #10 j = 0; k = 0;
    #20 $finish;
  end
endmodule