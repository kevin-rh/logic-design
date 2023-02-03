`timescale 1ns/1ps

module Sliding_Window_Sequence_Detector (clk, rst_n, in, dec);
input clk, rst_n;
input in;
output dec;

parameter
s0 = 3'd0,
s1 = 3'd1,
s2 = 3'd2,
s3 = 3'd3,
s4 = 3'd4,
s5 = 3'd5,
s6 = 3'd6,
s7 = 3'd7;

reg [3:0] state;
reg [3:0] next_state;

always@(posedge clk)begin
    if(rst_n === 1'b0)begin
        state <= s0;
    end
    else begin
        state <= next_state;
    end
end

assign dec = ((state===s7)&&(in===1'b1));

always @(*)begin
    case(state)
    s0:begin
        if (in===1'b1) next_state = s1;
        else next_state = s0;
    end
    s1:begin
        if (in===1'b1) next_state = s2;
        else next_state = s0;
    end
    s2:begin
        if (in===1'b0) next_state = s3;
        else next_state = s2;
    end
    s3:begin
        if (in===1'b0) next_state = s4;
        else next_state = s1;
    end
    s4:begin
        if (in===1'b1) next_state = s5;
        else next_state = s0;
    end
    s5:begin
        if (in===1'b0) next_state = s6;
        else next_state = s1;
    end
    s6:begin
        if (in===1'b0) next_state = s7;
        else next_state = s5;
    end
    s7:begin
        if (in===1'b1) next_state = s1;
        else next_state = s0; 
    end
    default: begin
        next_state = s0;
    end
    endcase
end

endmodule 