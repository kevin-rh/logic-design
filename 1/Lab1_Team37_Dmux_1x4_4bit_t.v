`timescale 1ns/1ps

module Dmux_1x4_4bit_t;
reg [3:0] in = 4'b0000;
reg [1:0] sel = 2'b00;

wire [3:0] a;
wire [3:0] b;
wire [3:0] c;
wire [3:0] d;

Dmux_1x4_4bit M1(
    .in(in), 
    .a(a), 
    .b(b), 
    .c(c), 
    .d(d), 
    .sel(sel)
);

initial begin
    repeat (2 ** 4) begin
        #1 in = in + 4'b1;
        sel = sel + 2'b1;
    end
    #1 $finish;
end

endmodule