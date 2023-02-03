
module Multiplier_4bit(a, b, p);
    input [3:0] a, b;
    output [7:0] p;

    wire [3:0] A0,A1,A2,A3;

	// Ai [3:0] = a[3:0] * bi
    AND_bitwise4x1 b0 ( 
		.a	(a[3:0]),
		.b	(b[0]),
		.out({A0[3:1],p[0]})
	);
	AND_bitwise4x1 b1 (
		.a	(a[3:0]),
		.b	(b[1]),
		.out(A1[3:0])
	);
	AND_bitwise4x1 b2 (
		.a	(a[3:0]),
		.b	(b[2]),
		.out(A2[3:0])
	);
	AND_bitwise4x1 b3 (
		.a	(a[3:0]),
		.b	(b[3]),
		.out(A3[3:0])
	);
    
	wire [3:0] B0, B1;
    //Full_Adder_4bit (a,b,cin,cout,sum);
    Full_Adder_4bit fa0(
        .a(A1[3:0]),
        .b({1'b0,A0[3:1]}),
        .cin(1'b0),
        .cout(B0[3]),
        .sum({B0[2:0],p[1]})
        );
    Full_Adder_4bit fa1(
        .a(A2[3:0]),
        .b(B0[3:0]),
        .cin(1'b0),
        .cout(B1[3]),
        .sum({B1[2:0],p[2]})
        );
    Full_Adder_4bit fa2(
        .a(A3[3:0]),
        .b(B1[3:0]),
        .cin(1'b0),
        .cout(p[7]),
        .sum(p[6:3])
        );

endmodule

module AND_bitwise4x1 (a,b,out);
    input [3:0] a;
    input b;
    output [3:0] out;

    AND a0 (out[0],a[0],b);
    AND a1 (out[1],a[1],b);
    AND a2 (out[2],a[2],b);
    AND a3 (out[3],a[3],b);
endmodule

module Full_Adder_4bit (a,b,cin,cout,sum);
    input [3:0]a,b;
    input cin;
    output cout;
    output [3:0] sum;
    wire wc1, wc2, wc3;

    Full_Adder fa0 (a[0], b[0], cin, wc1, sum[0]);
    Full_Adder fa1 (a[1], b[1], wc1, wc2, sum[1]);
    Full_Adder fa2 (a[2], b[2], wc2, wc3, sum[2]);
    Full_Adder fa3 (a[3], b[3], wc3, cout, sum[3]);

endmodule


module Full_Adder (a, b, cin, cout, sum);
input a, b, cin;
output cout, sum;

wire tsum,tcout1,tcout2;
Half_Adder ha1(.a(a),.b(b),.cout(tcout1),.sum(tsum));
Half_Adder ha2(.a(cin),.b(tsum),.cout(tcout2),.sum(sum));
Majority major(.a(a),.b(b),.c(cin),.out(cout));

endmodule


module Half_Adder(a, b, cout, sum);
input a, b;
output cout, sum;

//cout = a&b
NAND_Implement and_cout(a, b,3'b001,cout);
//sum = a^b
NAND_Implement xor_sum (a,b,3'b100,sum);

endmodule


module Majority(a, b, c, out);
input a, b, c;
output out;

wire a1,a2,a3,a4;
NAND_Implement n1(a,b,3'b001,a1);
NAND_Implement n2(b,c,3'b001,a2);
NAND_Implement n3(a,c,3'b001,a3);
NAND_Implement n4(a1,a2,3'b010,a4);
NAND_Implement n5(a3,a4,3'b010,out);

endmodule

//`timescale 1ns/1ps

module NAND_Implement (a, b, sel, out);
input a, b;
input [2:0] sel;
output out;

//negation of sel[2:0]
wire [2:0] nsel;
NOT no0(nsel[0],sel[0]);
NOT no1(nsel[1],sel[1]);
NOT no2(nsel[2],sel[2]);
wire o0,o1,o2,o3,o4,o5,o6;

//solve
NAND x0(o0,a,b);
AND x1(o1,a,b);
OR x2(o2,a,b);
NOR x3(o3,a,b);
XOR x4(o4,a,b);
XNOR x5(o5,a,b);
NOT x6(o6,a);

//output
wire out0,out1,out2,out3,out4,out5,out6,out7;
AND_4x1 f0 (out0,o0,nsel[2],nsel[1],nsel[0]);
AND_4x1 f1 (out1,o1,nsel[2],nsel[1],sel[0]);
AND_4x1 f2 (out2,o2,nsel[2],sel[1],nsel[0]);
AND_4x1 f3 (out3,o3,nsel[2],sel[1],sel[0]);
AND_4x1 f4 (out4,o4,sel[2],nsel[1],nsel[0]);
AND_4x1 f5 (out5,o5,sel[2],nsel[1],sel[0]);
AND_4x1 f6 (out6,o6,sel[2],sel[1],nsel[0]);
AND_4x1 f7 (out7,o6,sel[2],sel[1],sel[0]);

OR_8x1 finals (out,out0,out1,out2,out3,out4,out5,out6,out7);

endmodule

module AND_4x1 (out,s0,s1,s2,s3);
input s0,s1,s2,s3;
output out;
wire w0,w1;
AND l0(w0,s0,s1);
AND l1(w1,s2,s3);
AND l2(out,w1,w0);
endmodule

module OR_8x1 (out,s0,s1,s2,s3,s4,s5,s6,s7);
input s0,s1,s2,s3,s4,s5,s6,s7;
output out;
wire w0,w1,w2,w3,w4,w5;
OR or80(w0,s0,s1);
OR or81(w1,s2,s3);
OR or82(w2,s4,s5);
OR or83(w3,s6,s7);
OR or84(w4,w0,w1);
OR or85(w5,w2,w3);
OR or86(out,w4,w5);
endmodule


//0nand
module NAND (out, a, b);
input a,b;
output out;
nand n0(out,a,b);
endmodule
//1and
module AND (out, a, b);
input a,b;
output out;
wire and0;
nand n10(and0,a,b);
nand n11(out,and0,and0);
endmodule
//2or
module OR (out, a, b);
input a,b;
output out;
wire or0,or1;
nand n20(or0,a,a);
nand n21(or1,b,b);
nand n22(out,or0,or1);
endmodule
//3nor
module NOR (out, a, b);
input a,b;
output out;
wire nor0,nor1,nor2;
nand n30(nor0,a,a);
nand n31(nor1,b,b);
nand n32(nor2,nor0,nor1);
nand n33(out,nor2,nor2);
endmodule
//4xor
module XOR (out, a, b);
input a,b;
output out;
wire xor0,xor1,xor2;
nand n40(xor0,a,b);
nand n41(xor1,xor0,a);
nand n42(xor2,xor0,b);
nand n43(out,xor1,xor2);
endmodule
//5xnor
module XNOR (out, a, b);
input a,b;
output out;
wire xnor0,xnor1,xnor2,xnor3;
nand n50(xnor0,a,b);
nand n51(xnor1,xnor0,a);
nand n52(xnor2,xnor0,b);
nand n53(xnor3,xnor1,xnor2);
nand n54(out,xnor3,xnor3);
endmodule
//6 or 7not
module NOT (out, a);
input a;
output out;
nand n6(out,a,a);
endmodule
