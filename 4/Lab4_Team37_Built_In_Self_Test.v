`timescale 1ns/1ps

module Built_In_Self_Test(clk, rst_n, scan_en, scan_in, scan_out);
    input clk;
    input rst_n;
    input scan_en;
    output scan_in; 
    output scan_out;
    
    Many_To_One_LFSR MtO_LFSR (
        .clk(clk), 
        .rst_n(rst_n), 
        .out(scan_in)
    );
    
    Scan_Chain_Design SCD_8bits (
        .clk(clk), 
        .rst_n(rst_n), 
        .scan_in(scan_in), 
        .scan_en(scan_en), 
        .scan_out(scan_out)
    );

endmodule


module Scan_Chain_Design(clk, rst_n, scan_in, scan_en, scan_out);
    input clk;
    input rst_n;
    input scan_in;
    input scan_en;
    output scan_out;
    
    wire [7:0] p;
    wire [3:0] a, b;
    
    wire [7:0] SDFF_out;
    
    assign scan_out = SDFF_out[7];
    
    Multiplier_4bits multiplier_4bits (
        .p(p),
        .b({SDFF_out[4],
            SDFF_out[5],
            SDFF_out[6],
            SDFF_out[7]}),
        .a({SDFF_out[0],
            SDFF_out[1],
            SDFF_out[2],
            SDFF_out[3]})
    );
    
    SDFF scan_DFF [7:0] (
        .clk({8{clk}}), 
        .rst_n({8{rst_n}}), 
        .scan_en({8{scan_en}}), 
        .scan_in({SDFF_out[6], 
                  SDFF_out[5], 
                  SDFF_out[4], 
                  SDFF_out[3], 
                  SDFF_out[2], 
                  SDFF_out[1], 
                  SDFF_out[0], 
                  scan_in}), 
        .data({p[0],
               p[1],
               p[2],
               p[3],
               p[4],
               p[5],
               p[6],
               p[7]}), 
        .scan_out({SDFF_out[7],
                   SDFF_out[6], 
                   SDFF_out[5], 
                   SDFF_out[4], 
                   SDFF_out[3], 
                   SDFF_out[2], 
                   SDFF_out[1], 
                   SDFF_out[0]})
    );
    
endmodule


module Multiplier_4bits (p, a, b);
    input [3:0] a, b;
    output [7:0] p;
    
    assign p = a * b;
endmodule


module SDFF (clk, rst_n, scan_en, scan_in, data, scan_out);
    input clk;
    input rst_n;
    input scan_in;
    input scan_en;
    input data;
    output reg scan_out;
    
    reg next_out;
    
    always @(posedge clk) begin
        if(rst_n) begin
            scan_out <= next_out;
        end else begin
            scan_out <= 1'b0;
        end
    end
    
    always @(*) begin
        if(scan_en) begin
            next_out = scan_in;
        end else begin
            next_out = data;
        end
    end
    
endmodule


module Many_To_One_LFSR(clk, rst_n, out);
    input clk;
    input rst_n;
    output out;
    
    reg [7:0] DFF;
    wire XOR;
    
    assign XOR = ((DFF [1] ^ DFF[2]) ^ (DFF[3] ^ DFF[7]));
    assign out = DFF[7];
    
    always @(posedge clk) begin
        if(rst_n == 1'b0) begin
            DFF <= 8'b10111101;
        end else begin
            DFF[7:1] <= DFF[6:0];
            DFF[0] 	 <= XOR;
        end
    end
    
endmodule