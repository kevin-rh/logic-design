`timescale 1ns/1ps

module Content_Addressable_Memory(clk, wen, ren, din, addr, dout, hit);
    input clk;
    input wen, ren;
    input [7:0] din;
    input [3:0] addr;
    output reg [3:0] dout;
    output reg hit;
    
    reg [3:0] next_dout;
    reg next_hit;
    
    parameter n = 16, m=8;
    reg [m-1:0] mem [n-1:0];
    reg [m-1:0] next_mem [n-1:0];

    wire [n-1:0] compare_val;   // compare value's output
    wire [3:0] chosen_address;  // priority encoder's output
    wire exist;
    
    always @(posedge clk) begin
        dout <= next_dout;
        hit <= next_hit;
        mem[0] <= next_mem[0];
        mem[1] <= next_mem[1];
        mem[2] <= next_mem[2];
        mem[3] <= next_mem[3];
        mem[4] <= next_mem[4];
        mem[5] <= next_mem[5];
        mem[6] <= next_mem[6];
        mem[7] <= next_mem[7];
        mem[8] <= next_mem[8];
        mem[9] <= next_mem[9];
        mem[10] <= next_mem[10];
        mem[11] <= next_mem[11];
        mem[12] <= next_mem[12];
        mem[13] <= next_mem[13];
        mem[14] <= next_mem[14];
        mem[15] <= next_mem[15];
    end
    
    always @(*) begin
        next_dout = 4'b0;
        next_hit = 1'b0;
        next_mem[0] = mem[0];
        next_mem[1] = mem[1];
        next_mem[2] = mem[2];
        next_mem[3] = mem[3];
        next_mem[4] = mem[4];
        next_mem[5] = mem[5];
        next_mem[6] = mem[6];
        next_mem[7] = mem[7];
        next_mem[8] = mem[8];
        next_mem[9] = mem[9];
        next_mem[10] = mem[10];
        next_mem[11] = mem[11];
        next_mem[12] = mem[12];
        next_mem[13] = mem[13];
        next_mem[14] = mem[14];
        next_mem[15] = mem[15];

        if(ren) begin
            if(exist) begin
                next_dout = chosen_address;
                next_hit = 1'b1;
            end else begin
                next_dout = 4'b0;
                next_hit = 1'b0;
            end
        end else begin
            if(wen) begin
                next_mem[addr] = din;
                next_dout = 4'b0;
                next_hit = 1'b0;
            end else begin
                next_dout = 4'b0;
                next_hit = 1'b0;
            end
        end
    end

    Comparator_Array comp_0 (
        .out(compare_val[0]), 
        .a(mem[0]), 
        .b(din)
    );
    Comparator_Array comp_1 (
        .out(compare_val[1]), 
        .a(mem[1]), 
        .b(din)
    );
    Comparator_Array comp_2 (
        .out(compare_val[2]), 
        .a(mem[2]), 
        .b(din)
    );
    Comparator_Array comp_3 (
        .out(compare_val[3]), 
        .a(mem[3]), 
        .b(din)
    );
    Comparator_Array comp_4 (
        .out(compare_val[4]), 
        .a(mem[4]), 
        .b(din)
    );
    Comparator_Array comp_5 (
        .out(compare_val[5]), 
        .a(mem[5]), 
        .b(din)
    );
    Comparator_Array comp_6 (
        .out(compare_val[6]), 
        .a(mem[6]), 
        .b(din)
    );
    Comparator_Array comp_7 (
        .out(compare_val[7]), 
        .a(mem[7]), 
        .b(din)
    );
    Comparator_Array comp_8 (
        .out(compare_val[8]), 
        .a(mem[8]), 
        .b(din)
    );
    Comparator_Array comp_9 (
        .out(compare_val[9]), 
        .a(mem[9]), 
        .b(din)
    );
    Comparator_Array comp_10 (
        .out(compare_val[10]), 
        .a(mem[10]), 
        .b(din)
    );
    Comparator_Array comp_11 (
        .out(compare_val[11]), 
        .a(mem[11]), 
        .b(din)
    );
    Comparator_Array comp_12 (
        .out(compare_val[12]), 
        .a(mem[12]), 
        .b(din)
    );
    Comparator_Array comp_13 (
        .out(compare_val[13]), 
        .a(mem[13]), 
        .b(din)
    );
    Comparator_Array comp_14 (
        .out(compare_val[14]), 
        .a(mem[14]), 
        .b(din)
    );
    Comparator_Array comp_15 (
        .out(compare_val[15]), 
        .a(mem[15]), 
        .b(din)
    );

    Priority_Encoder prior_addr_out(
        .out(chosen_address),
        .valid(exist), 
        .in(compare_val)
    );

endmodule

module Comparator_Array(out, a, b);
    input [7:0] a, b;
    output out;
    assign out = &(a~^b);
endmodule

module Priority_Encoder(out, valid, in);
    parameter n = 16;
    input [n-1:0] in;
    output reg [4-1:0] out;
    output valid;
    
    assign valid = |in;
    
    always@ (*) begin
        if(in[15])begin out=4'b1111;
        end else if(in[14])begin out=4'b1110;
        end else if(in[13])begin out=4'b1101;
        end else if(in[12])begin out=4'b1100;
        end else if(in[11])begin out=4'b1011;
        end else if(in[10])begin out=4'b1010;
        end else if(in[9])begin out=4'b1001;
        end else if(in[8])begin out=4'b1000;
        end else if(in[7])begin out=4'b0111;
        end else if(in[6])begin out=4'b0110;
        end else if(in[5])begin out=4'b0101;
        end else if(in[4])begin out=4'b0100;
        end else if(in[3])begin out=4'b0011;
        end else if(in[2])begin out=4'b0010;
        end else if(in[1])begin out=4'b0001;
        end else if(in[0])begin out=4'b0000;
        end else begin out=4'b0000;
        end
    end
endmodule
