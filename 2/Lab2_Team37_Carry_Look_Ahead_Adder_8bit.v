`timescale 1ns/1ps

module Carry_Look_Ahead_Adder_8bit(a, b, c0, s, c8);
input [7:0] a, b;
input c0;
output [7:0] s;
output c8;

wire c4;
wire [7:0] p,g;

CLA_4bit cla1 (
	.a(a[3:0]), 
	.b(b[3:0]), 
	.c0(c0), 
	.s(s[3:0]), 
	.p(p[3:0]), //out
	.g(g[3:0])	//out
);

CLA_4bit cla2 (
	.a(a[7:4]), 
	.b(b[7:4]), 
	.c0(c4), 
	.s(s[7:4]), 
	.p(p[7:4]), //out
	.g(g[7:4])	//out
);

CLA_2bit_Gen c4c8 (
	.c0(c0),	//in
	.p(p[7:0]), //in
	.g(p[7:0]),	//in
	.c4(c4),	//out
	.c8(c8)		//out
);

endmodule


module CLA_4bit(a, b, c0, s, p, g);
input [3:0] a, b;
input c0;
output [3:0] s;

wire [3-1:1-1] c;
output [3:0] p,g;

Full_Adder_PG fa0 (
	.a  (a[0]),
	.b  (b[0]), 
	.cin( c0),
	.sum(s[0]),
	.p  (p[0]),
	.g  (g[0])
);
Full_Adder_PG fa1 (
	.a  (a[1]),
	.b  (b[1]), 
	.cin(c[1-1]),
	.sum(s[1]),
	.p  (p[1]),
	.g  (g[1])
);
Full_Adder_PG fa2 (
	.a  (a[2]),
	.b  (b[2]), 
	.cin(c[2-1]),
	.sum(s[2]),
	.p  (p[2]),
	.g  (g[2])
);
Full_Adder_PG fa3 (
	.a  (a[3]),
	.b  (b[3]), 
	.cin(c[3-1]),
	.sum(s[3]),
	.p  (p[3]),
	.g  (g[3])
);

Carry_Gen_4bit cg30 (
	.c_in(c0),
	.p(p[3:0]),
	.g(g[3:0]),
	.c_out(c[3-1:1-1])
);

endmodule


// Generate c1,c2,c3 instantly
module Carry_Gen_4bit(c_in,p,g,c_out);
input c_in;
input [3:0] p, g;
output [3-1:1-1] c_out;

wire w0,w10,w11,w12,w20,w21,w22,w23,w24;
//c_out [1]
AND oa00 (w0,p[0],c_in); //p0c0
OR oo00 (c_out[1-1], g[0], w0); //g0 + p0c0
//c_out [2] 
AND oa10 (w10,w0,p[1]); //p1p0c0
AND oa11 (w11,g[0],p[1]); //p1g0
OR oo10 (w12,w10,w11); //p1g0+p1p0c0
OR oo12 (c_out[2-1],g[1],w12);//g1+p1g0+p1p0c0
//c_out [3]
AND oa20 (w20,w10,p[2]); //p2p1p0c0
AND oa21 (w21,w11,p[2]); //p2p1g0
AND oa22 (w22,g[1],p[2]); //p2g1
OR oo20 (w23,w20,w21); // p2p1g0+p2p1p0c0
OR oo21 (w24,w23,w22); //p2g1+p2p1g0+p2p1p0c0
OR oo22 (c_out[3-1],g[2],w24);// g2+p2g1+p2p1g0+p2p1p0c0
endmodule

// Generate c4 c8 instantly
module CLA_2bit_Gen(c0,p,g,c4,c8);
input c0;
input [7:0] p, g;
output wire c4,c8;
CLA_Gen gen0 (c0, p[3:0], g[3:0], c4);
CLA_Gen gen1 (c4, p[7:4], g[7:4], c8);

endmodule

module CLA_Gen(c0,p,g,c4);
input c0;
input [3:0] p, g;
output c4;

wire w0,w10,w11,w20,w21,w22,w30,w31,w32,w33;
wire o0,o1,o2;

AND oa00 (w0,p[0],c0);//p0c0

AND oa10 (w10,w0,p[1]); //p1p0c0
AND oa11 (w11,g[0],p[1]);//p1g0

AND oa20 (w20,w10,p[2]); // p2p1p0c0
AND oa21 (w21,w11,p[2]); // p2p1g0
AND oa22 (w22,g[1],p[2]);// p2g1

AND oa30 (w30,w20,p[3]); // p3p2p1p0c0
AND oa31 (w31,w21,p[3]); // p3p2p1g0
AND oa32 (w32,w22,p[3]); // p3p2g1
AND oa33 (w33,g[2],p[3]);// p3g2

OR or0(o0,w30,w31); // o0 = p3p2p1g0 + p3p2p1p0c0
OR or1(o1,w32,w33); // o1 = p3p2g1 + p3g2
OR or2(o2,o1,o0); 	// o2 = o0 + o1 = p3g2 + p3p2g1 + p3p2p1g0 + p3p2p1p0c0 
OR or3(c4,o2,g[3]);	// c4 = g3 + p3g2 + p3p2g1 + p3p2p1g0 + p3p2p1p0c0 
endmodule

module Full_Adder (a, b, cin, sum, p, g);
    input a, b, cin;
    output sum, p, g;
	
	//Generate
	AND gi(g,a,b);
	//Propagate
	OR pi(p,a,b);
	//Sum
	wire w0;
	XOR si0(w0,a,b);
	XOR si1(sum,w0,cin);
	
endmodule

module XOR (out, a, b);
input a,b;
output out;
wire xor0,xor1,xor2;
nand n40(xor0,a,b);
nand n41(xor1,xor0,a);
nand n42(xor2,xor0,b);
nand n43(out,xor1,xor2);
endmodule

module AND (out, a, b);
input a,b;
output out;
wire and0;
nand n10(and0,a,b);
nand n11(out,and0,and0);
endmodule

module OR (out, a, b);
input a,b;
output out;
wire or0,or1;
nand n20(or0,a,a);
nand n21(or1,b,b);
nand n22(out,or0,or1);
endmodule

