`timescale 1ns/1ps
module Ping_Pong_Counter_t;
reg clk=1'b0, rst_n=1'b1;
reg enable=1'b0;
wire direction;
wire [3:0] out;

reg error;
parameter cycle =2;

always #(cycle/2) clk=~clk;
Ping_Pong_Counter ping_ping_cnt(
    .clk(clk), 
    .rst_n(rst_n), 
    .enable(enable), 
    .direction(direction), 
    .out(out)
);
initial begin
    error=1'b0;
    rst_n=1'b0;
    enable=1'b0;
    #(4*cycle);
    
    rst_n=1'b1;
    enable=1'b1;
    #(4*cycle);
    repeat (2**4) begin
        #cycle;
    end
    
    enable=1'b0;
    #(4*cycle);
    repeat (2**2) begin
        #cycle;
    end
    
    enable=1'b1;
    #(4*cycle);
    repeat (2**4) begin
        #cycle;
    end
    
    rst_n=1'b0;
    #(4*cycle);
    rst_n=1'b1;
    #(4*cycle);
    repeat (2**4) begin
        #cycle;
    end
end
endmodule