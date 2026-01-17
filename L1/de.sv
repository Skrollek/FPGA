module encoder2 (
    input  wire [1:0] I,
    output wire       Y,
    output wire       valid
);
    assign Y = I[1];
    assign valid = I[1] | I[0];
endmodule

module encoder4 (
    input  wire [3:0] I,
    output wire [1:0] Y,
    output wire       valid
);
    wire y_low, y_high;
    wire valid_low, valid_high;

    encoder2 low  (.I(I[1:0]), .Y(y_low),  .valid(valid_low));
    encoder2 high (.I(I[3:2]), .Y(y_high), .valid(valid_high));

    assign valid = valid_high | valid_low;
    assign Y[1]  = valid_high;
    assign Y[0]  = valid_high ? y_high : y_low;
endmodule

module tb;
  reg  [3:0] I;
  wire [1:0] Y;
  wire v;

  encoder4 u(I, Y, v);

  initial begin
    I = 0;
    #5  I = 4'b0001;
    #5  I = 4'b0010;
    #5  I = 4'b0100;
    #5  I = 4'b1000;
    #5  I = 4'b0000;
    #5  $finish;
  end
endmodule