module fib(input clk, input nrst, output [31:0] f);
  reg [31:0] f1;
  reg [31:0] f2;
  
  always @(posedge clk or negedge nrst) begin
    if(!nrst)
      f1 <= 32'b0;
    else
      f1 <= f2;
  end

  always @(posedge clk or negedge nrst) begin
    if(!nrst)
      f2 <= 32'b1;
    else
      f2 <= f1 + f2;
  end

  assign f = f1;
endmodule



module fib_tb;
  logic clk;
  logic nrst;
  logic [31:0] f;

  fib dut (
    .clk(clk),
    .nrst(nrst),
    .f(f)
  );

  always #5 clk = ~clk; 

  reg [63:0] f_prev, f_curr, f_next;

  initial begin
    $display("Time\tFib(n)\t\tValue");
    clk = 0;
    nrst = 0;
    f_prev = 0;
    f_curr = 1;
    #12;
    nrst = 1;

    forever begin
      @(posedge clk);
      $display("%0t\t%0d\t\t%0d", $time, f_prev, f);
      f_next = f_prev + f_curr;
      if (f_prev != f) begin
        $display("Overflow detected at: %0d, circuit value: %0d", f_prev, f);
        $finish;
      end
      f_prev = f_curr;
      f_curr = f_next;
    end

    $finish;
  end

endmodule