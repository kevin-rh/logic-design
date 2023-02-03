`timescale 1ns/1ps

module Dmux_1x4_4bit(in, a, b, c, d, sel);
input [3:0] in;
input [1:0] sel;
output [3:0] a, b, c, d;

wire [1:0] not_sel;
wire [3:0] yes;

not not1 [1:0] (not_sel,sel);

Assign_4bit assignA(not_sel[1],not_sel[0],in,a);
Assign_4bit assignB(not_sel[1],sel[0],in,b);
Assign_4bit assignC(sel[1],not_sel[0],in,c);
Assign_4bit assignD(sel[1],sel[0],in,d);

endmodule


module Assign_4bit(sel1,sel0,in,out);
input sel1,sel0;
input [3:0] in;
output [3:0] out;

wire yes;

and (yes,sel1,sel0);

and and1(out[0],in[0],yes);
and and2(out[1],in[1],yes);
and and3(out[2],in[2],yes);
and and4(out[3],in[3],yes);

endmodule