`timescale 1ns/1ps

module Crossbar_4x4_4bit(in1, in2, in3, in4, out1, out2, out3, out4, control);
input [3:0] in1, in2, in3, in4;
input [4:0] control;
output [3:0] out1, out2, out3, out4;

wire [3:0] wire03 ,wire02, wire12, wire14, wire23, wire24;
Crossbar_2x2_4bit cb2_0(.in1(in1),
						.in2(in2),
						.control(control[0]),
						.out1(wire03),
						.out2(wire02));

Crossbar_2x2_4bit cb2_1(.in1(in3),
						.in2(in4),
						.control(control[1]),
						.out1(wire12),
						.out2(wire14));
						
Crossbar_2x2_4bit cb2_2(.in1(wire02),
						.in2(wire12),
						.control(control[2]),
						.out1(wire23),
						.out2(wire24));
						
Crossbar_2x2_4bit cb2_3(.in1(wire03),
						.in2(wire23),
						.control(control[3]),
						.out1(out1),
						.out2(out2));
						
Crossbar_2x2_4bit cb2_4(.in1(wire24),
						.in2(wire14),
						.control(control[4]),
						.out1(out3),
						.out2(out4));
endmodule

//`timescale 1ns/1ps

module Crossbar_2x2_4bit(in1, in2, control, out1, out2);
input [3:0] in1, in2;
input control;
output [3:0] out1, out2;

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
	.out(out1), 
	.control(neg_control)
);
Mux_1x2_4bit mux2 (
	.in1(wire12), 
	.in2(wire22), 
	.out(out2), 
	.control(control)
);

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