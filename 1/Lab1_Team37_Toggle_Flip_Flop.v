`timescale 1ns/1ps

module Toggle_Flip_Flop(clk, q, t, rst_n);
input clk;
input t;
input rst_n;
output q;

wire wire_xor, wire_xor1, wire_xor2, wire_and;

// NOR & OR == XOR
// t XOR q -> wire_xor
nand xor1(wire_xor1, t, q);
or xor2(wire_xor2, t, q);
and xor3(wire_xor,wire_xor1,wire_xor2);

// rst_n AND wire_xor -> wire_and
and and1(wire_and, rst_n, wire_xor);

// DFF 
D_Flip_Flop dff(
	.clk(clk),
	.d(wire_and),
	.q(q)
);

endmodule

module D_Flip_Flop(clk, d, q);
input clk;
input d;
output q;

wire negclk,w0;

not not1 (negclk,clk);
D_Latch Master(
	.e(negclk),
	.d(d),
	.q(w0)
);
D_Latch Slave(
	.e(clk),
	.d(w0),
	.q(q)
);

endmodule

module D_Latch(e, d, q);
input e;
input d;
output q;

wire w0, w1, negd, negq;

not not1(negd,d);

nand nand1 (w0, d, e);
nand nand2 (w1, negd, e);
nand nand3 (q, w0, negq);
nand nand4 (negq, w1, q);

endmodule