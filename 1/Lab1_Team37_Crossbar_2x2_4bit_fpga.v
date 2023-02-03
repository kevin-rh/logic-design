`timescale 1ns/1ps

module Crossbar_2x2_4bit(in1, in2, control, out1, out2);
input [3:0] in1, in2;
input control;
output [7:0] out1, out2;

wire neg_control;
not neg(neg_control, control);

wire [3:0] wire11, wire12, wire21, wire22;

Dmux_1x2_4bit dmux1 (
	.in(in1), 
	.out1(wire11), 
	.out2(wire12), 
	.control(neg_control)
);
Dmux_1x2_4bit dmux2 (
	.in(in2), 
	.out1(wire21), 
	.out2(wire22), 
	.control(control)
);

Mux_1x2_4bit mux1 (
	.in1(wire11), 
	.in2(wire21), 
	.out(out1[3:0]), 
	.control(neg_control)
);
Mux_1x2_4bit mux2 (
	.in1(wire12), 
	.in2(wire22), 
	.out(out2[3:0]), 
	.control(control)
);

and final1 [3:0] (out1[7:4], out1[3:0], {4{1'b1}});
and final2 [3:0] (out2[7:4], out2[3:0], {4{1'b1}});

endmodule

module Dmux_1x2_4bit(in, out1, out2, control);
input [3:0] in;
input control;
output [3:0] out1, out2;

wire neg_control;
not neg(neg_control, control);

and and0 [3:0](out1, in, {4{neg_control}});
and and1 [3:0](out2, in, {4{control}});

endmodule

module Mux_1x2_4bit(in1, in2, out, control);
input [3:0] in1, in2;
input control;
output [3:0] out;

wire neg_control;
not neg(neg_control, control);

wire [3:0] out1,out2;

and and0 [3:0](out1, in1, {4{neg_control}});
and and1 [3:0](out2, in2, {4{control}});

or or1 [3:0] (out, out1, out2);

endmodule