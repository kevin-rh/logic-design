`timescale 1ns/1ps

module FIFO_8(clk, rst_n, wen, ren, din, dout, error);
	input clk;
	input rst_n;
	input wen, ren;
	input [7:0] din;
	output reg [7:0] dout;
	output reg error;

	parameter N=8;

	reg [N-1:0] memory [N-1:0];
	reg [2:0] wp,rp;
	reg empty;

	always @(posedge clk)begin
		if(rst_n==1'b0) begin
			wp<=3'b000;
			rp<=3'b000;
			empty<=1'b0;
		end
		else begin
			if ((wp==rp && empty==1'b1 && wen)||(empty==1'b0 && ren)) begin
				error<=1'b1;
			end
			else begin
				error<=1'b0;
				if(ren==1'b1) begin // read
					dout<=memory[rp];
					rp<=((rp==3'b111)?(3'b000):(rp + 1));
					// when it's empty
					if((wp==3'b000&&rp==3'b111)||((rp+3'b001)==wp))begin
						empty<=1'b0;
					end
				end
				else if(wen==1'b1) begin // write
					memory[wp]<=din;
					wp<=((wp==3'b111)?(3'b000):(wp + 1));
					// it's not empty anymore
					empty<=1'b1;
					
				end
			end
		end
	end

endmodule
