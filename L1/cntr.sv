module licz(
  input clk,
  input rst,
  input dir,
  input zero,
  output reg [15:0] cntr
);
  reg [15:0] cntr_next;

  assign cntr_next = zero ? 16'b0 : (dir ? cntr - 16'b1 : cntr + 16'b1);

  always @(posedge clk or posedge rst) begin
    if (rst)
      cntr <= 16'b0;
    else
      cntr <= cntr_next;
  end

endmodule

module tb;
  reg clk, rst, dir, zero;
  wire [15:0] c;

  licz u(clk, rst, dir, zero, c);

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst = 1; dir = 0; zero = 0;
    #10 rst = 0;
    #20 dir = 0;
    #20 dir = 1;
    #20 zero = 1;
    #10 zero = 0;
    #20 $finish;
  end
endmodule