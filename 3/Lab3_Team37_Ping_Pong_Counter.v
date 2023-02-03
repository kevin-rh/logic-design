`timescale 1ns/1ps

module Ping_Pong_Counter (clk, rst_n, enable, direction, out);
input clk, rst_n;
input enable;
output direction;
output [3:0] out;

parameter N=4;
reg [N-1:0] pos;
reg dir;

assign out = pos;
assign direction = dir;

always @(posedge clk) begin
    if (rst_n==1'b0) begin
        pos<=4'b0000;
        dir<=1'b1;
    end
    else if(enable==1'b1) begin
        if (pos==4'b1111) begin
            pos<=4'b1110;
            dir<=1'b0;
        end
        else if (pos==4'b0000) begin
            pos<=4'b0001;
            dir<=1'b1;
        end
        else begin
            pos=pos+((dir==1'b1)?(1'b1):(-1'b1));
        end
    end
	else begin
		pos<=pos;
		dir<=dir;
	end
end

endmodule