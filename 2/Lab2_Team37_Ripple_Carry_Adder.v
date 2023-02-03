`timescale 1ns/1ps

module Ripple_Carry_Adder(a, b, cin, cout, sum);
    input [7:0] a, b;
    input cin;
    output cout;
    output [7:0] sum;

    wire w0, w1, w2, w3, w4, w5, w6, w7;

    Full_Adder fa0(a[0],b[0],cin,w0,sum[0]);
    Full_Adder fa1(a[1],b[1],w0,w1,sum[1]);
    Full_Adder fa2(a[2],b[2],w1,w2,sum[2]);
    Full_Adder fa3(a[3],b[3],w2,w3,sum[3]);
    Full_Adder fa4(a[4],b[4],w3,w4,sum[4]);
    Full_Adder fa5(a[5],b[5],w4,w5,sum[5]);
    Full_Adder fa6(a[6],b[6],w5,w6,sum[6]);
    Full_Adder fa7(a[7],b[7],w6,cout,sum[7]);
    
endmodule

module Full_Adder (a, b, cin, cout, sum);
    input a, b, cin;
    output cout, sum;

    wire temp1, temp2, tempsum;

    Half_Adder ha1(a,b,temp1,tempsum);
    Half_Adder ha2(cin, tempsum, temp2, sum);
    Majority m1(a,b,cin, cout);
endmodule

module Half_Adder(a, b, cout, sum);
    input a, b;
    output cout, sum;

    XOR xor1(sum, a, b);
    AND and1(cout, a, b);
endmodule

module XOR(out, in1, in2);
    input in1, in2;
    output out;
    wire AB, ABA, ABB;
    nand nandAB(AB, in1, in2);
    nand nand1(ABA, AB, in1);
    nand nand2(ABB, AB, in2);
    nand xorAB(out, ABA, ABB);
endmodule

module Majority(a, b, c, out);
    input a, b, c;
    output out;

    wire andAB, andAC, andBC, orABAC;

    AND andab(andAB, a, b);
    AND andac(andAC, a, c);
    AND andbc(andBC, b, c);
    OR orabac(orABAC, andAB, andAC);

    OR oror1andbc(out, orABAC, andBC);

endmodule

module AND(out, in1, in2);
    input in1, in2;
    output out;
    wire nandAB;
    nand nand1(nandAB, in1, in2);
    nand nand2(out, nandAB, nandAB);
endmodule

module OR(out, in1, in2);
    input in1, in2;
    output out;
    wire A,B;
    nand nand1(A, in1, in1);
    nand nand2(B, in2, in2);
    nand orAB(out, A, B);
endmodule