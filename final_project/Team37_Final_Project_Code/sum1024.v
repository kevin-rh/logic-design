
// Module sum(n) to count how many 1's in (n) bits array.
module sum1024 (
    input [1024-1:0] in,
    output [10:0] out
);
    wire [9:0] outr, outl;
    sum512 l(in[1024-1:512], outl);
    sum512 r(in[512-1:0], outr);
    assign out = outr + outl;
endmodule

module sum512(
    input [512-1:0] in,
    output [9:0] out
);
    wire [8:0] outr, outl;
    sum256 l(in[512-1:256], outl);
    sum256 r(in[256-1:0], outr);
    assign out = outr + outl;
endmodule

module sum256(
    input [256-1:0] in,
    output [8:0] out
);
    wire [7:0] outr, outl;
    sum128 l(in[256-1:128], outl);
    sum128 r(in[128-1:0], outr);
    assign out = outr + outl;
endmodule

module sum128(
    input [128-1:0] in,
    output [7:0] out
);
    wire [6:0] outr, outl;
    sum64 l(in[128-1:64], outl);
    sum64 r(in[64-1:0], outr);
    assign out = outr + outl;
endmodule

module sum64(
    input [64-1:0] in,
    output [6:0] out
);
    wire [5:0] outr, outl;
    sum32 l(in[64-1:32], outl);
    sum32 r(in[32-1:0], outr);
    assign out = outr + outl;
endmodule

module sum32(
    input [32-1:0] in,
    output [5:0] out
);
    wire [4:0] outr, outl;
    sum16 l(in[32-1:16], outl);
    sum16 r(in[16-1:0], outr);
    assign out = outr + outl;
endmodule

module sum16(
    input [16-1:0] in,
    output [4:0] out
);
    wire [3:0] outr, outl;
    sum8 l(in[16-1:8], outl);
    sum8 r(in[8-1:0], outr);
    assign out = outr + outl;
endmodule

module sum8(
    input [8-1:0] in,
    output [3:0] out
);
    wire [2:0] outr, outl;
    sum4 l(in[8-1:4], outl);
    sum4 r(in[4-1:0], outr);
    assign out = outr + outl;
endmodule

module sum4(
    input [4-1:0] in,
    output [2:0] out
);
    wire [1:0] outr, outl;
    sum2 l(in[4-1:2], outl);
    sum2 r(in[2-1:0], outr);
    assign out = outr + outl;
endmodule

module sum2(
    input [2-1:0] in,
    output [1:0] out
);
    assign out = in[1] + in[0];
endmodule