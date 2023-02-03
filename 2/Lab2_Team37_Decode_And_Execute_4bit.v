`timescale 1ns/1ps
`include "Universal_Gate.v"

module Decode_And_Execute(rs, rt, sel, rd);
input [3:0] rs, rt;
input [2:0] sel;
output [3:0] rd;

wire [3:0] o0,o1,o2,o3,o4,o5,o6,o7;
// 000: ADD
ADD_4bit e0(
    .a(rs[3:0]),
    .b(rt[3:0]),
    .out(o0[3:0])
);

// 001: SUB
SUB_4bit e1(
    .a(rs[3:0]),
    .b(rt[3:0]),
    .out(o1[3:0])
);

// 010: BITWISE AND
AND_4bit e2(
    .a(rs[3:0]),
    .b(rt[3:0]),
    .out(o2[3:0])
);

// 011: BITWISE OR
OR_4bit e3(
    .a(rs[3:0]),
    .b(rt[3:0]),
    .out(o3[3:0])
);

// 100: RS CIR. LEFT SHIFT
Assign_4bits e4(
    .in({rs[2:0],rs[3]}),
    .out(o4[3:0])
);

// 101: RT ARI. RIGHT SHIFT
Assign_4bits e5(
    .in({rt[3],rt[3:1]})
    ,.out(o5[3:0])
);

wire cmp1,cmp2;
// 110: COMPARE EQ
EQ_4bit e60(
    .a(rs[3:0]),
    .b(rt[3:0]),
    .out(cmp1)
);
Assign_4bits e61(
    .in({3'b111,cmp1}),
    .out(o6[3:0])
);

// 111: COMPARE GT
GT_4bit e70(
    .a(rs[3:0]),
    .b(rt[3:0]),
    .out(cmp2)
);
Assign_4bits e71(
    .in({3'b101,cmp2}),
    .out(o7[3:0])
);

MUX_8x1_4bit mx (
    .a(o0[3:0]),
    .b(o1[3:0]),
    .c(o2[3:0]),
    .d(o3[3:0]),
    .e(o4[3:0]),
    .f(o5[3:0]),
    .g(o6[3:0]),
    .h(o7[3:0]),
    .sel(sel[2:0]),
    .out(rd[3:0])
); 	

endmodule


//MUX 8x1 from 9-MUXs 2x1
module MUX_8x1_4bit (a,b,c,d,e,f,g,h,sel,out);
input [3:0] a,b,c,d,e,f,g,h;
input [2:0] sel;
output [3:0] out; 
	
wire [3:0] w1,w2;
MUX_4x1_4bit m1 (a[3:0],b[3:0],c[3:0],d[3:0],sel[1:0],w1[3:0]);
MUX_4x1_4bit m2 (e[3:0],f[3:0],g[3:0],h[3:0],sel[1:0],w2[3:0]);
MUX_2x1_4bit m3 (w1[3:0],w2[3:0],sel[2],out[3:0]);
endmodule

module MUX_4x1_4bit (a,b,c,d,sel,out);
input [3:0] a,b,c,d;
input [1:0] sel;
output [3:0] out; 
	
wire [3:0] w1,w2;
MUX_2x1_4bit m1 (a[3:0],b[3:0],sel[0],w1[3:0]);
MUX_2x1_4bit m2 (c[3:0],d[3:0],sel[0],w2[3:0]);
MUX_2x1_4bit m3 (w1[3:0],w2[3:0],sel[1],out[3:0]);
endmodule

module MUX_2x1_4bit (a,b,sel,out);
input [3:0] a,b;
input sel;
output [3:0] out;

wire nsel;
NOT n1 (sel,nsel);

wire [3:0]o1,o2;
AND_4bit a1(a[3:0],{4{nsel}},o1[3:0]);
AND_4bit b1(b[3:0],{4{sel}},o2[3:0]);
OR_4bit or1 (o1[3:0],o2[3:0],out[3:0]);
endmodule

module AND_4bit (a,b,out);
input [3:0] a,b;
output [3:0] out;
AND a0 (a[0],b[0],out[0]);
AND a1 (a[1],b[1],out[1]);
AND a2 (a[2],b[2],out[2]);
AND a3 (a[3],b[3],out[3]);
endmodule

module OR_4bit (a,b,out);
input [3:0] a,b;
output [3:0] out;
OR o0 (a[0],b[0],out[0]);
OR o1 (a[1],b[1],out[1]);
OR o2 (a[2],b[2],out[2]);
OR o3 (a[3],b[3],out[3]);
endmodule
//endMUX 8x1 from 9-MUXs 2x1


// Ripple Carry Subtractor (4bit) A + (-B) -> OUT (4bits)
module SUB_4bit (a,b,out);
input [3:0]a,b;
output [3:0]out;

wire [3:0]nb;
NOT n0 (b[0],nb[0]);
NOT n1 (b[1],nb[1]);
NOT n2 (b[2],nb[2]);
NOT n3 (b[3],nb[3]);

// 2's complement [~(b)+1]
wire c0,c1,c2,c3;
Full_Adder fa0 (a[0],nb[0],1'b1,out[0],c0);
Full_Adder fa1 (a[1],nb[1],c0,out[1],c1);
Full_Adder fa2 (a[2],nb[2],c1,out[2],c2);
Full_Adder fa3 (a[3],nb[3],c2,out[3],c3);
endmodule

// Ripple Carry Adder (4bit) A + B -> OUT (4bits)
module ADD_4bit (a,b,out);
input [3:0]a,b;
output [3:0]out;

wire c0,c1,c2,c3;
Full_Adder fa0 (a[0],b[0],1'b0,out[0],c0);
Full_Adder fa1 (a[1],b[1],c0,out[1],c1);
Full_Adder fa2 (a[2],b[2],c1,out[2],c2);
Full_Adder fa3 (a[3],b[3],c2,out[3],c3);
endmodule

// FULL ADDER 1 bit
module Full_Adder(a,b,cin,sum,cout);
input a,b,cin;
output sum,cout;

//SUM
wire w1,w2,w3,w4,w5,w6,w7,w8,w9,w10;
Universal_Gate u1(a,b,w1);
Universal_Gate u2(b,a,w2);
NOT u3(w1,w3);
Universal_Gate u4(w3,w2,w4);
NOT u5(w4,w5);
Universal_Gate u6(w5,cin,w6);
Universal_Gate u7(cin,w5,w7);
NOT u8(w7,w8);
Universal_Gate u9(w8,w6,w9);
NOT u10(w9,sum);

//COUT
wire nb,nc;
NOT neg1(b,nb);
NOT neg2(cin,nc);
wire c1,c2,c3,c4,c5,c6;
Universal_Gate d1(a,nb,c1);
Universal_Gate d2(a,nc,c2);
Universal_Gate d3(b,nc,c3);
NOT d4(c2,c4);
Universal_Gate d5(c4,c1,c5);
Universal_Gate d6(c5,c3,c6);
NOT d7 (c6,cout);
endmodule

// AND 2x1 Gate
module AND (a,b,out);
input a,b;
input out;
wire nb;
NOT a1 (b,nb);
Universal_Gate a2(a,nb,out);
endmodule

// OR 2x1 Gate
module OR (a,b,out);
input a,b;
input out;
wire nb,nout;
NOT a1 (b,nb);
Universal_Gate a2(nb,a,nout);
NOT a3(nout,out);
endmodule

// Compare A==B
module EQ_4bit(a,b,out);
input [3:0]a,b;
output out;

wire q0,q1,q2,q3,q4,q5,q6;
XOR eq0(a[0],b[0],q0);
XOR eq1(a[1],b[1],q1);
XOR eq2(a[2],b[2],q2);
XOR eq3(a[3],b[3],q3);

OR eq4(q0,q1,q4);
OR eq5(q2,q3,q5);

OR eq6(q4,q5,q6);

NOT n1(q6,out);
endmodule

// Compare A>B
module GT_4bit(a,b,out);
input [3:0]a,b;
output out;

// qi = ai>bi
wire q0,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10;
Universal_Gate gt0(a[0],b[0],q0);
Universal_Gate gt1(a[1],b[1],q1);
Universal_Gate gt2(a[2],b[2],q2);
Universal_Gate gt3(a[3],b[3],q3);

wire nq4,nq5,nq6;
// q4 = a1==b1
XOR gt4(a[1],b[1],nq4);
NOT xor4(nq4,q4);
// q5 = a2==b2
XOR gt5(a[2],b[2],nq5);
NOT xor5(nq5,q5);
// q6 = a3==b3
XOR gt6(a[3],b[3],nq6);
NOT xor6(nq6,q6);

// q9 = a3==b3 & a2==b2 & a1==a1 & a0>b0
AND gt7(q6,q5,q7); //a3==b3 a2==b2
AND gt8(q7,q4,q8); //a1==b1
AND gt9(q8,q0,q9); //a0>b0

// q10 = a3==b3 & a2==b2 & a1>b1
AND gt10(q7,q1,q10);

// q11 = a3==b3 & a2>b2
AND gt11(q6,q2,q11);

//OR ALL with a3>b3
OR gt12(q3,q9,q12);
OR gt13(q10,q11,q13);
OR gt14(q12,q13,out);

endmodule

// Assign 4 bits
module Assign_4bits(in,out);
input [3:0] in;
output [3:0] out;
Assign_1bit w0(in[0],out[0]);
Assign_1bit w1(in[1],out[1]);
Assign_1bit w2(in[2],out[2]);
Assign_1bit w3(in[3],out[3]);
endmodule

// Assign 1 bit
module Assign_1bit(in,out);
input in;
output out;
Universal_Gate io (in,1'b0,out);
endmodule

// NOT Gate
module NOT(in,out);
input in;
output out;
Universal_Gate neg (1'b1,in,out);
endmodule

// XOR 2x1 Gate
module XOR(a,b,out);
input a,b;
output out;

wire o1,o2,o3;
Universal_Gate xr1 (a,b,o1);
Universal_Gate xr2 (b,a,o2);
NOT xr3 (o2,o3);
Universal_Gate xr4 (o3,o1,o4);
NOT xr5 (o4,out);
endmodule
