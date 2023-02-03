`timescale 1ns/1ps

module Mealy_Sequence_Detector (clk, rst_n, in, dec);
    input clk, rst_n;
    input in;
    output reg dec;
    
    parameter 
    S0 = 4'd0,
    S1 = 4'd1,
    S2 = 4'd2,
    S3 = 4'd3,
    S4 = 4'd4,
    S5 = 4'd5,
    S6 = 4'd6,
    S7 = 4'd7,
    SXX = 4'd9,
    SX = 4'd10;
    
    reg [3:0] state;
    reg [3:0] next_state;
    
    always@(posedge clk) begin
        if(rst_n==1'b0) state <= S0;
        else state <= next_state;
    end
    
    always@(*)begin
        case(state)
        S0: begin
            next_state = ((in==1'b0)? S1:S4);
            dec = 1'b0;
        end
        S1: begin
            next_state = ((in==1'b0)? SXX:S2);
            dec = 1'b0;
        end
        S2: begin
            next_state = ((in==1'b0)? SX:S3);
            dec = 1'b0;
        end
        S3: begin
            next_state = ((in==1'b0)? S0:S0);
            dec = ((in==1'b0)? 1'b0:1'b1);
        end
        S4: begin
            next_state = ((in==1'b0)? S5:S6);
            dec = 1'b0;
        end
        S5: begin
            next_state = ((in==1'b0)? S3:SX);
            dec = 1'b0;
        end
        S6: begin
            next_state = ((in==1'b0)? SX:S7);
            dec = 1'b0;
        end
        S7: begin
            next_state = ((in==1'b0)? S0:S0);
            dec = ((in==1'b0)? 1'b1:1'b0);
        end
        SXX: begin
            next_state = SX;
            dec = 1'b0;
        end
        SX: begin
            next_state = S0;
            dec = 1'b0;
        end
        default: begin
            next_state = S0;
            dec = 1'b0;
        end
        endcase
    end

endmodule
