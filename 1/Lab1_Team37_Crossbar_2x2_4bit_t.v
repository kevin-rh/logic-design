`timescale 1ns/1ps

module Crossbar_2x2_4bit_t;
reg [3:0] in1 = 4'b0000;
reg [3:0] in2 = 4'b0000;
reg control = 1'b0;

wire[3:0]out1;
wire[3:0]out2;

Crossbar_2x2_4bit m1(
    .in1(in1), 
    .in2(in2), 
    .control(control), 
    .out1(out1), 
    .out2(out2)
);

initial begin
	in1=4'b0100;
	in2=4'b0000;
	control=1'b1;
	#10
    repeat (2 ** 3) begin
        #1 
        in1 = in1 + 4'b1;
        in2 = in2 + 4'b1;
        control = control + 1'b1;
    end
    #1 $finish;
end

endmodule